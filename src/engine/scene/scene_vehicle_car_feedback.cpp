// Contact feedback, turbo, water response, and the per-tick force pipeline.

#include "engine/scene/scene_vehicle_car_internal.h"
using namespace SceneVehicleCarDynamics;

void CSceneVehicleCar::UpdateBodyContactSnapshot() {
  CSceneVehicleCar::SVehicleCarState *frame = &frameHistory.physicsCurrent;
  if (contacts.bodyContactCount == 0) {
    frame->hasBodyContact = false;
    frame->bodyContactVerticalAngle = 0.0f;
    frame->bodyContactHorizontalAngle = 0.0f;
    return;
  }

  float normalLenSq =
      SceneVehicleMath::LengthSquared(contacts.bodyContactNormalSum);
  if (normalLenSq > VectorEpsilonSquared) {
    contacts.bodyContactNormalSum = SceneVehicleMath::Scale(
        contacts.bodyContactNormalSum, 1.0f / CIsqrt(normalLenSq));
  }
  contacts.bodyContactNormalSum =
      SceneVehicleMath::Scale(contacts.bodyContactNormalSum, -1.0f);

  float invCount =
      1.0f / Binary32::FromUnsignedInteger(contacts.bodyContactCount);
  contacts.bodyContactPointSum =
      SceneVehicleMath::Scale(contacts.bodyContactPointSum, invCount);

  GmVec3 horizontal = {contacts.bodyContactNormalSum.x,
                       contacts.bodyContactNormalSum.y, 0.0f};
  float horizontalLenSq =
      horizontal.x * horizontal.x + horizontal.y * horizontal.y;
  if (horizontalLenSq > VectorEpsilonSquared) {
    horizontal =
        SceneVehicleMath::Scale(horizontal, 1.0f / CIsqrt(horizontalLenSq));
  }
  frame->bodyContactHorizontalAngle =
      std::fabs(CIatan2(std::fabs(horizontal.x), -horizontal.y)) /
      SceneVehicleMath::Pi;

  GmVec3 vertical = {0.0f, contacts.bodyContactNormalSum.y,
                     contacts.bodyContactNormalSum.z};
  float verticalLenSq = vertical.y * vertical.y + vertical.z * vertical.z;
  if (verticalLenSq > VectorEpsilonSquared) {
    vertical = SceneVehicleMath::Scale(vertical, 1.0f / CIsqrt(verticalLenSq));
  }
  frame->bodyContactVerticalAngle =
      std::fabs(CIatan2(std::fabs(vertical.z), -vertical.y)) /
      SceneVehicleMath::Pi;
  frame->bodyContactZPositive = 0.0f < vertical.z;
  frame->hasBodyContact = true;
}

void CSceneVehicleCar::ResetContactAccumulators() {
  contacts.wheelContactCount = 0;
  contacts.bodyContactCount = 0;
  contacts.bodyContactPointSum = SceneVehicleMath::Zero();
  contacts.bodyContactNormalSum = SceneVehicleMath::Zero();
}

void CSceneVehicleCar::UpdateWheelSnapshot(
    CSceneVehicleCar::SSimulationWheel &wheel) {
  wheel.previousPhysicsState = wheel.currentPhysicsState;

  CSceneVehicleCar::SSimulationWheel::SState *snapshot =
      &wheel.currentPhysicsState;
  snapshot->damperAbsorb = wheel.realTimeState.damperAbsorb;
  snapshot->wheelSpinAngle = wheel.realTimeState.wheelSpinAngle;
  snapshot->currentVisualSteerAngle =
      wheel.realTimeState.currentVisualSteerAngle;
  snapshot->contactMaterial = wheel.realTimeState.contactMaterial;
  snapshot->contactPresent = wheel.realTimeState.contactPresent;
  snapshot->slipping = wheel.realTimeState.slipping;

  snapshot->localSurfacePoint = wheel.surfaceHandler.CurrentPoint();
  snapshot->localSurfacePoint.y -= wheel.rollingRadius;
  snapshot->worldSurfacePoint = TransformPoint(
      frameHistory.physicsCurrent.corpusIso, snapshot->localSurfacePoint);
  snapshot->visualRotation = wheel.realTimeState.visualRotation;
  snapshot->rejectedNormalContact = wheel.realTimeState.rejectedNormalContact;
  snapshot->rejectedNormalContactPoint =
      wheel.realTimeState.rejectedNormalContactPoint;
}

void CSceneVehicleCar::UpdateMaterialFeedback() {
  u32 remappedMaterialIndex =
      MaterialRemapAt(contacts.lastWheelContactMaterial);
  CSceneVehicleMaterial *material =
      MaterialContainer()->MaterialAt(remappedMaterialIndex);

  float absForwardSpeed = std::fabs(frameHistory.physicsCurrent.forwardSpeed);
  float depth = 0.0f;
  if (airControl.refreshMemory) {
    float scaledDepth = material->fakeContactSpeedScale * absForwardSpeed;
    depth = material->fakeContactDepthMax;
    if (!(depth < scaledDepth)) {
      depth = scaledDepth;
    }
  }

  float intensityTarget = material->feedbackScale * depth;
  float cappedIntensity = 0.15f;
  if (intensityTarget < cappedIntensity) {
    cappedIntensity = intensityTarget;
  }

  float speedTarget = absForwardSpeed / material->feedbackSpeedDivisor;
  float feedbackSpeed = SmoothValue(
      frameHistory.physicsCurrent.materialFeedbackSpeed, speedTarget, 0.5f);
  frameHistory.physicsCurrent.materialFeedbackSpeed = feedbackSpeed;

  float feedbackIntensity =
      SmoothValue(frameHistory.physicsCurrent.materialFeedbackIntensity,
                  cappedIntensity, 0.05f);
  frameHistory.physicsCurrent.materialFeedbackIntensity = feedbackIntensity;

  if (contacts.bodyImpactState >= CSceneVehicleCarImpactState_High ||
      (contacts.frontWheelImpactState >= CSceneVehicleCarImpactState_High &&
       contacts.rearWheelImpactState != CSceneVehicleCarImpactState_None)) {
    frameHistory.physicsCurrent.materialFeedbackSpeed = 2.0f;
    if (2.0f <= feedbackSpeed) {
      frameHistory.physicsCurrent.materialFeedbackSpeed = feedbackSpeed;
    }
    frameHistory.physicsCurrent.materialFeedbackIntensity = 1.0f;
    if (1.0f <= feedbackIntensity) {
      frameHistory.physicsCurrent.materialFeedbackIntensity = feedbackIntensity;
    }
    return;
  }

  if (contacts.bodyImpactState == CSceneVehicleCarImpactState_None &&
      (contacts.frontWheelImpactState == CSceneVehicleCarImpactState_None ||
       contacts.rearWheelImpactState == CSceneVehicleCarImpactState_None)) {
    return;
  }

  frameHistory.physicsCurrent.materialFeedbackSpeed = 4.0f;
  if (4.0f <= feedbackSpeed) {
    frameHistory.physicsCurrent.materialFeedbackSpeed = feedbackSpeed;
  }
  frameHistory.physicsCurrent.materialFeedbackIntensity = 0.66f;
  if (0.66f <= feedbackIntensity) {
    frameHistory.physicsCurrent.materialFeedbackIntensity = feedbackIntensity;
  }
}

void CSceneVehicleCar::AfterContacts() {
  frameHistory.physicsPrevious.Set(frameHistory.physicsCurrent);

  u32 wheelCount = WheelGetCount();
  for (u32 wheelIndex = 0; wheelIndex < wheelCount; wheelIndex++) {
    CSceneVehicleCar::SSimulationWheel *wheel = &WheelAt(wheelIndex);
    wheel->previousPhysicsState = wheel->currentPhysicsState;
  }

  GmVec3 linearSpeed;
  HmsItem()->GetLinearSpeed(linearSpeed);

  CSceneVehicleCar::SVehicleCarState *frame = &frameHistory.physicsCurrent;
  frame->forwardSpeed = linearSpeed.z;
  frame->sideSpeed = linearSpeed.x;
  frame->engineInputMemory = engine.engineInputMemory;
  frame->turboActive = turbo.type != ETurboType_Inactive;
  frame->turboProgressRatio = turbo.progressRatio;

  CHmsCorpus *corpus = HmsItem()->CorpusAt(0u);
  frame->corpusIso = corpus->LocalLocation();
  frame->vehicleEvent0Value =
      VehicleEventValue(CSceneVehicle::EVehicleEvent_None);
  frame->waterSplashEventCounter =
      VehicleEventValue(CSceneVehicle::EVehicleEvent_WaterSplash);
  frame->localLinearSpeed = TransformDirection(frame->corpusIso, linearSpeed);

  frame->wheelSpeedOverrideActive = gearedDrive.wheelSpeedOverrideActive;
  frame->airControlRefreshMemory = airControl.refreshMemory;
  frame->surfaceFeedbackAccumulator = feedback.surfaceAccumulator;
  frame->steeringControl = controls.steeringControl;
  frame->lowSpeedGateB = controls.lowSpeedGateB;
  frame->lowSpeedGateA = controls.lowSpeedGateA;
  frame->engineControlState = gearedDrive.engineState;
  frame->shiftDirection = gearedDrive.shiftDirection;
  frame->feedbackSideSpringValue = feedback.sideSpring.value;
  frame->feedbackForwardSpringValue = feedback.forwardSpring.value;
  frame->feedbackRamp1 = feedback.ramp1;
  frame->feedbackRamp0 = feedback.ramp0;

  UpdateBodyContactSnapshot();
  frame->hasWheelContact = contacts.wheelContactCount != 0;
  frame->noGroundFrictionGuard = controls.noGroundFrictionGuard;
  ResetContactAccumulators();

  wheelCount = WheelGetCount();
  for (u32 wheelIndex = 0; wheelIndex < wheelCount; wheelIndex++) {
    CSceneVehicleCar::SSimulationWheel *wheel = &WheelAt(wheelIndex);
    UpdateWheelSnapshot(*wheel);
  }

  UpdateMaterialFeedback();
}

void CSceneVehicleCar::ComputeVehicleGroundMaterialVals(
    CSceneVehicleMaterial::SBlendableVals &out, int &hasMaterial) {
  hasMaterial = 0;
  out.x = 0.0f;
  out.y = 0.0f;
  out.z = 0.0f;
  out.w = 0.0f;

  u32 contactCount = 0;
  u32 wheelCount = WheelGetCount();
  for (u32 wheelIndex = 0; wheelIndex < wheelCount; wheelIndex++) {
    CSceneVehicleCar::SSimulationWheel *wheel = &WheelAt(wheelIndex);
    if (!wheel->realTimeState.contactPresent) {
      continue;
    }

    CSceneVehicleCar::SSimulationWheel *firstWheel = &WheelAt(0);
    u32 remappedMaterialIndex =
        MaterialRemapAt(firstWheel->realTimeState.contactMaterial);
    CSceneVehicleMaterial *material =
        MaterialContainer()->MaterialAt(remappedMaterialIndex);

    contactCount++;
    out.x += material->blendableVals.x;
    out.y += material->blendableVals.y;
    out.z += material->blendableVals.z;
    out.w += material->blendableVals.w;
    hasMaterial = 1;
  }

  if (contactCount != 0) {
    float invCount = 1.0f / static_cast<float>(contactCount);
    out.x *= invCount;
    out.y *= invCount;
    out.z *= invCount;
    out.w *= invCount;
  }
}

void CSceneVehicleCar::GetSlopeAdherence(const GmVec3 &normal, float &outFirst,
                                         float &outSecond) {
  float lenSq = normal.y * normal.y + normal.x * normal.x + normal.z * normal.z;
  if (!(1.0e-10f < lenSq)) {
    return;
  }

  CSceneVehicleCarTuning *tuning = Tunings()->ActiveTuning();
  float len = CIsqrt(lenSq);
  float slope = Binary32::FromDouble(static_cast<double>(normal.y) /
                                     static_cast<double>(len));
  slope = std::fabs(slope);

  outFirst = slope;
  outFirst =
      SlopeAdherenceBlend(slope, tuning->bodyAirResponse.slopeAdherence1Min,
                          tuning->bodyAirResponse.slopeAdherence1Max);

  outSecond = slope;
  outSecond =
      SlopeAdherenceBlend(slope, tuning->bodyAirResponse.slopeAdherence2Min,
                          tuning->bodyAirResponse.slopeAdherence2Max);
}

void CSceneVehicleCar::ApplyFrictionForces(const GmVec3 &speed) {
  CSceneVehicleCarTuning *tuning = Tunings()->ActiveTuning();

  if ((tuning->handlingModel == CSceneVehicleCarHandlingModel_SlipResponse ||
       tuning->handlingModel == CSceneVehicleCarHandlingModel_GearedDrive) &&
      controls.noGroundFrictionGuard && IsGroundContact() == 0) {
    return;
  }

  float gateSpeed = !engine.useLowSpeedGateB ? controls.lowSpeedGateA
                                             : controls.lowSpeedGateB;
  if (gateSpeed < 1.0e-5f || controls.forcedLowSpeedFriction != 0) {
    GmVec3 force = {
        (speed.x),
        (speed.y),
        (speed.z),
    };
    float lenSq = SceneVehicleMath::LengthSquaredYXZ(force);
    if (1.0e-10f < lenSq) {
      float speedLen = CIsqrt(lenSq);
      float invLen = SceneVehicleMath::Reciprocal(speedLen);
      force.x = invLen * force.x;
      force.y = force.y * invLen;
      force.z = invLen * force.z;

      float frictionScale = -tuning->lowSpeedFrictionMagnitude;
      force.x = frictionScale * force.x;
      force.y = force.y * frictionScale;
      force.z = frictionScale * force.z;

      if (controls.forcedLowSpeedFriction == 0) {
        float dampingScale = -tuning->lowSpeedLinearDamping;
        GmVec3 damping = {
            (speed.x * dampingScale),
            (speed.y * dampingScale),
            (dampingScale * speed.z),
        };
        force.x = damping.x + force.x;
        force.y = damping.y + force.y;
        force.z = damping.z + force.z;
      }
      AddVehicleCentralForce(force);
    }
  }

  if (static_cast<int32_t>(tuning->handlingModel) <
      CSceneVehicleCarHandlingModel_SlipResponse) {
    if (contacts.lateralSlowDownContactActive != 0) {
      float lateralSlowdownScale =
          -tuning->GetLateralContactSlowDownFromSpeed(speed.z);
      GmVec3 force = {
          (speed.x * lateralSlowdownScale),
          (speed.y * lateralSlowdownScale),
          (lateralSlowdownScale * speed.z),
      };
      AddVehicleCentralForce(force);
    }
    return;
  }

  u32 tick = CMwCmdBufferCore::Current()->Timer().GetTickTime();
  if (contacts.lateralSlowDownContactActive != 0) {
    contacts.lateralSlowDownLastTick = tick;
  }

  if (contacts.lateralSlowDownLastTick <= tick) {
    u32 elapsed = tick - contacts.lateralSlowDownLastTick;
    if (elapsed < tuning->slipResponse.lateralSlowDownTickWindow) {
      float speedSq = SceneVehicleMath::LengthSquaredYXZ(speed);
      float speedLen = CIsqrt(speedSq);
      if (1.0e-5f < speedLen) {
        // reciprocal for the radiusSteering/5 lateral slowdown branch.
        float invLen = SceneVehicleMath::Reciprocal(speedLen);
        GmVec3 unit = {
            (speed.x * invLen),
            (speed.y * invLen),
            (invLen * speed.z),
        };
        float slipHandlingLateralSlowdownScale =
            (-tuning->M5GetLateralContactSlowDownFromSpeed(speedLen));
        GmVec3 force = {
            (slipHandlingLateralSlowdownScale * unit.x),
            (unit.y * slipHandlingLateralSlowdownScale),
            (slipHandlingLateralSlowdownScale * unit.z),
        };
        AddVehicleCentralForce(force);
      }
    }
  }
}

void CSceneVehicleCar::ComputeAirControl(const GmVec3 &angularSpeed,
                                         unsigned long tick,
                                         int isGroundContact, int resetMemory) {
  CSceneVehicleCarTuning *tuning = Tunings()->ActiveTuning();
  if ((tuning->handlingModel == CSceneVehicleCarHandlingModel_SlipResponse ||
       tuning->handlingModel == CSceneVehicleCarHandlingModel_GearedDrive) &&
      controls.noGroundFrictionGuard) {
    return;
  }

  GmVec3 torque = {
      (-angularSpeed.x),
      (-angularSpeed.y),
      (-angularSpeed.z),
  };

  const bool resetAirControlMemory = resetMemory != 0;
  if (resetAirControlMemory) {
    airControl.memoryTick = tick;
  }
  if (resetAirControlMemory || airControl.refreshMemory) {
    airControl.memoryAngular.x = angularSpeed.x;
    airControl.memoryAngular.y = angularSpeed.y;
    airControl.memoryAngular.z = angularSpeed.z;
  } else if (static_cast<u32>(tick - airControl.memoryTick) <
             tuning->bodyAirResponse.airControlMemoryTickWindow) {
    GmVec3 target = {angularSpeed.x, angularSpeed.y, angularSpeed.z};
    bool strong = false;

    if ((1.0e-5f < controls.steeringControl &&
         airControl.memoryAngular.y < 0.0f) ||
        (controls.steeringControl < -1.0e-5f &&
         0.0f < airControl.memoryAngular.y)) {
      float absY = std::fabs(angularSpeed.y);
      if (tuning->bodyAirResponse.airControlYSwitchThreshold < absY) {
        strong = true;
        airControl.memoryAngular.y = angularSpeed.y;
      }
    } else {
      if (!(1.0e-5f < controls.steeringControl) ||
          !(0.0f < airControl.memoryAngular.y)) {
        if (controls.steeringControl < -1.0e-5f &&
            airControl.memoryAngular.y < 0.0f) {
          strong = true;
        }
      } else {
        strong = true;
      }
      airControl.memoryAngular.y = angularSpeed.y;
    }

    target.y = airControl.memoryAngular.y;

    if (tuning->handlingModel == CSceneVehicleCarHandlingModel_SlipResponse ||
        tuning->handlingModel == CSceneVehicleCarHandlingModel_GearedDrive) {
      float targetX;
      if (1.0e-5f < controls.lowSpeedGateB &&
          0.0f < airControl.memoryAngular.x) {
        targetX = 0.0f;
      } else {
        targetX = angularSpeed.x;
      }
      airControl.memoryAngular.x = targetX;
      target.x = airControl.memoryAngular.x;
    }

    if (strong) {
      torque.x = torque.x * 3.0f;
      torque.y = torque.y * 3.0f;
      torque.z = 3.0f * torque.z;
    }

    if (isGroundContact == 0) {
      unsigned long keyIndex = 0ul;
      float zScale = std::fabs(angularSpeed.z);
      tuning->bodyAirResponse.airControlZScaleCurve.Value().GetValue(
          zScale, zScale, keyIndex);
      torque.z = torque.z * zScale;
    }

    SetVehicleAngularSpeed(target);
  }

  if (isGroundContact == 0) {
    float torqueXYLenSq = torque.x * torque.x + torque.y * torque.y;
    float torqueLenSq = torque.z * torque.z + torqueXYLenSq;
    float torqueLen = CIsqrt(torqueLenSq);
    if (torqueLen >= 1.0e-5f) {
      float invLen = 1.0f / torqueLen;
      GmVec3 unitTorque = {
          (invLen * torque.x),
          (invLen * torque.y),
          (invLen * torque.z),
      };
      float quadraticTerm =
          tuning->bodyAirResponse.airTorqueQuadraticCoef * torqueLen;
      quadraticTerm *= torqueLen;
      float linearTerm =
          torqueLen * tuning->bodyAirResponse.airTorqueLinearCoef;
      float torqueMagnitude = quadraticTerm + linearTerm;
      GmVec3 airTorque = {
          (torqueMagnitude * unitTorque.x),
          (unitTorque.y * torqueMagnitude),
          (torqueMagnitude * unitTorque.z),
      };
      AddVehicleTorque(airTorque);
    }
  }
}

void CSceneVehicleCar::ApplyWaterSplashImpulse(const CSceneVehicleCarTuning &tuning,
                                               const GmIso4 &iso,
                                               const GmVec3 &localSpeed,
                                               float curveInput) {
  unsigned long keyIndex = 0ul;
  float verticalScale = curveInput;
  float horizontalScale = curveInput;

  tuning.water.splashVerticalImpulseCurve.Value().GetValue(
      curveInput, verticalScale, keyIndex);
  tuning.water.splashHorizontalImpulseCurve.Value().GetValue(
      curveInput, horizontalScale, keyIndex);

  GmVec3 impulse = {
      -horizontalScale * localSpeed.x,
      -verticalScale * localSpeed.y,
      -horizontalScale * localSpeed.z,
  };
  impulse.MultTranspose(iso.RotationMatrix());
  WaterSplash(localSpeed);
  AddVehicleImpulse(impulse);
}

int CSceneVehicleCar::ApplyWaterForces(const GmVec3 &forceToSubtract) {
  const CHmsZone *zone = HmsItem()->GetZone(0ul);
  const CSceneVehicleWaterZone *waterZone = zone->WaterZone();
  if (waterZone == nullptr || !waterZone->IsEnabled()) {
    return 0;
  }

  CHmsCorpus *corpus = HmsItem()->CorpusAt(0u);
  const GmIso4 *iso = corpus->GetLocation();

  GmBoxAligned worldBox;
  worldBox.SetMult(WaterState().boxLocal, *iso);
  GmVec2 sample = {worldBox.center.x, worldBox.center.z};
  float halfY = std::fabs(worldBox.halfExtents.y);
  float lowerY = worldBox.center.y - halfY;
  float upperY = worldBox.center.y + halfY;

  if (!waterZone->AcceptsRegion(sample, lowerY, upperY)) {
    return 0;
  }

  float depth = waterZone->SurfaceHeight() - lowerY;
  if (!(depth > 0.5f)) {
    return 0;
  }

  GmVec3 linearSpeedWorld;
  HmsItem()->GetLinearSpeed(linearSpeedWorld);
  GmVec3 localSpeed;
  localSpeed.SetMult(linearSpeedWorld, iso->RotationMatrix());

  CSceneVehicleCarTuning *tuning = Tunings()->ActiveTuning();
  float horizontalSpeedSq =
      localSpeed.x * localSpeed.x + localSpeed.z * localSpeed.z;
  float splashCurveInput = 0.0f;

  if (!airControl.refreshMemory && depth < 0.9f &&
      (waterZone->SurfaceHeight() - upperY) < 0.0f && localSpeed.y < -1.0e-5f) {
    float horizontalThreshold = tuning->water.splashHorizontalSpeedThreshold;
    if (horizontalSpeedSq > horizontalThreshold * horizontalThreshold) {
      float horizontalSpeed = CIsqrt(horizontalSpeedSq);
      splashCurveInput = -horizontalSpeed / localSpeed.y;
      if (splashCurveInput != splashCurveInput) {
        return 0;
      }
      if (!(splashCurveInput < 0.0f)) {
        ApplyWaterSplashImpulse(*tuning, *iso, localSpeed, splashCurveInput);
        return 0;
      }
    } else {
      float totalThreshold = tuning->water.splashTotalSpeedThreshold;
      if (SceneVehicleMath::LengthSquared(linearSpeedWorld) >
          totalThreshold * totalThreshold) {
        splashCurveInput = 0.0f;
        ApplyWaterSplashImpulse(*tuning, *iso, localSpeed, splashCurveInput);
        return 0;
      }
    }
  }

  GmVec3 waterDrag = {0.0f, 0.0f, 0.0f};
  float speedLen = CIsqrt(SceneVehicleMath::LengthSquared(linearSpeedWorld));
  if (1.0e-5f < speedLen) {
    float dragScale = -tuning->GetWaterFrictionFromSpeed(speedLen);
    waterDrag = SceneVehicleMath::Scale(linearSpeedWorld, dragScale);
  }

  GmVec3 angularSpeed;
  HmsItem()->GetAngularSpeed(angularSpeed);
  float angularLinearScale = -tuning->water.angularLinearDamping;
  GmVec3 torque = SceneVehicleMath::Scale(angularSpeed, angularLinearScale);
  float angularSpeedLen = CIsqrt(SceneVehicleMath::LengthSquared(angularSpeed));
  torque = SceneVehicleMath::Add(
      torque,
      SceneVehicleMath::Scale(
          angularSpeed, -angularSpeedLen * tuning->water.angularSpeedDamping));

  GmVec3 buoyancy = {0.0f, -tuning->water.buoyancyForce, 0.0f};
  buoyancy.MultTranspose(iso->RotationMatrix());
  GmVec3 centralForce = SceneVehicleMath::Subtract(
      SceneVehicleMath::Add(buoyancy, waterDrag), forceToSubtract);

  AddVehicleCentralForce(centralForce);
  AddVehicleTorque(torque);
  return 1;
}

void CSceneVehicleCar::SetZeroDynamics() {
  GmVec3 zero = SceneVehicleMath::Zero();
  HmsItem()->SetLinearSpeed(zero);
  HmsItem()->SetAngularSpeed(zero);
  HmsItem()->SetForce(zero);
  HmsItem()->SetTorque(zero);
}

void CSceneVehicleCar::UpdateDynaParamsForGroundContact(
    CSceneVehicleCarTuning *tuning, int isGroundContact) {
  CPlugPhysicalObject *physical = &HmsItem()->Solid()->Physical();
  float feedbackScale = isGroundContact != 0
                            ? tuning->bodyAirResponse.groundedSolidFeedback1
                            : tuning->bodyAirResponse.airborneSolidFeedback1;
  physical->Parameters().vehicleContactFeedbackScale = feedbackScale;

  float fluidFriction = isGroundContact != 0
                            ? 0.0f
                            : tuning->bodyAirResponse.airborneSolidFeedback0;
  physical->Parameters().linearFluidFriction = fluidFriction;
  HmsItem()->RefreshCorpusesFromSolid();
}

void CSceneVehicleCar::ClampLinearSpeed(GmVec3 &linearSpeed) {
  float capSq = linearSpeedCap * linearSpeedCap;
  float speedXY = linearSpeed.y * linearSpeed.y + linearSpeed.x * linearSpeed.x;
  float speedSq = linearSpeed.z * linearSpeed.z + speedXY;
  if (capSq < speedSq && capSq > VectorEpsilonSquared) {
    double capWide = static_cast<double>(linearSpeedCap);
    float speedLen = CIsqrt(speedSq);
    float linearSpeedCapScale = capWide / speedLen;
    linearSpeed.x = linearSpeedCapScale * linearSpeed.x;
    linearSpeed.y = linearSpeed.y * linearSpeedCapScale;
    linearSpeed.z = linearSpeedCapScale * linearSpeed.z;
    SetVehicleLinearSpeed(linearSpeed);
  }
}

float CSceneVehicleCar::ComputeVisualSteerYaw(CSceneVehicleCarTuning *tuning,
                                              const GmVec3 &linearSpeed) {
  float absSpeedZ = std::fabs(linearSpeed.z);
  float denom = absSpeedZ * tuning->visual.wheelSpeedScale +
                tuning->visual.wheelSpeedBase;
  float asinValue = 0.0f;
  if (!(denom < ScalarEpsilon)) {
    float asinInput = 1.0f / denom;
    asinValue = GmFunc::AsinSafe(asinInput);
  }
  float visualSteerYaw = (-controls.currentSteering) * asinValue;
  return visualSteerYaw;
}

void CSceneVehicleCar::ComputeSelectedHandlingForces(
    CSceneVehicleCarTuning *tuning, float dt, const GmVec3 &currentForce,
    float slopeAdherenceA, float slopeAdherenceB, const GmVec3 &linearSpeed,
    const GmVec3 &angularSpeed, float visualSteerYaw, int hasGroundMaterial,
    CSceneVehicleMaterial::SBlendableVals &materialVals, int &outSlipFlag,
    float &surfaceFeedback) {
  switch (tuning->handlingModel) {
  case CSceneVehicleCarHandlingModel_RadiusSteering:
    ComputeForcesModel4(dt, currentForce, slopeAdherenceA, slopeAdherenceB,
                        linearSpeed, angularSpeed, visualSteerYaw,
                        hasGroundMaterial, &materialVals, outSlipFlag,
                        surfaceFeedback);
    break;
  case CSceneVehicleCarHandlingModel_SlipResponse:
    ComputeForcesModel5(dt, currentForce, slopeAdherenceA, slopeAdherenceB,
                        linearSpeed, angularSpeed, visualSteerYaw,
                        hasGroundMaterial, &materialVals, outSlipFlag,
                        surfaceFeedback);
    break;
  case CSceneVehicleCarHandlingModel_GearedDrive:
    ComputeForcesModel6(dt, currentForce, slopeAdherenceA, slopeAdherenceB,
                        linearSpeed, angularSpeed, visualSteerYaw,
                        hasGroundMaterial, &materialVals, outSlipFlag,
                        surfaceFeedback);
    break;
  default:
    ComputeForcesModel3(dt, currentForce, slopeAdherenceA, slopeAdherenceB,
                        linearSpeed, angularSpeed, visualSteerYaw,
                        hasGroundMaterial, &materialVals, outSlipFlag,
                        surfaceFeedback);
    break;
  }
}

int CSceneVehicleCar::ScanWheelSideSpeedKillContacts(
    int &hasSideSpeedKillContact) {
  int hasAnyContact = 0;
  hasSideSpeedKillContact = 0;

  u32 wheelCount = WheelGetCount();
  for (u32 wheelIndex = 0; wheelIndex < wheelCount; wheelIndex++) {
    CSceneVehicleCar::SSimulationWheel *wheel = &WheelAt(wheelIndex);
    if (wheel->realTimeState.contactPresent) {
      hasAnyContact = 1;
      if (wheel->killsLateralSpeedOnContact) {
        hasSideSpeedKillContact = 1;
      }
    }
  }

  return hasAnyContact;
}

void CSceneVehicleCar::UpdateLowSpeedFeedback(CSceneVehicleCarTuning *tuning,
                                              int hasAnyContact) {
  if (hasAnyContact != 0) {
    float gateTerm = -controls.lowSpeedGateB * engine.lowSpeedFeedbackGateScale;
    float frictionTerm = tuning->lowSpeedFrictionMagnitude *
                         engine.lowSpeedFeedbackFrictionScale;
    engine.lowSpeedFeedbackForce = gateTerm - frictionTerm;
  }
}

void CSceneVehicleCar::KillSideSpeedForTaggedContact(
    CSceneVehicleCarTuning *tuning, int hasSideSpeedKillContact,
    GmVec3 &linearSpeed) {
  if (hasSideSpeedKillContact != 0 &&
      tuning->gearedDrive.lateralForceScale ==
          tuning->gearedDrive.lateralForceScale &&
      0.0f > tuning->gearedDrive.lateralForceScale) {
    linearSpeed.x = 0.0f;
    SetVehicleLinearSpeed(linearSpeed);
  }
}

void CSceneVehicleCar::ApplySpecialContactResponse(
    CSceneVehicleCarTuning *tuning, const GmVec3 &currentForce, u32 tick,
    int isGroundContact) {
  if (!(controls.specialContactResponseGate > ScalarEpsilon)) {
    return;
  }

  switch (controls.specialContactResponseMode) {
  case CSceneVehicleCarSpecialContactMode_ImpulseFromForce:
    if (isGroundContact != 0 &&
        contacts.specialContactImpulseCooldownUntil < tick) {
      GmVec3 impulse = {-currentForce.x, -currentForce.y, -currentForce.z};
      float lenSq = SceneVehicleMath::LengthSquared(impulse);
      if (lenSq > VectorEpsilonSquared) {
        impulse = SceneVehicleMath::Scale(impulse, 1.0f / CIsqrt(lenSq));
      }
      impulse = SceneVehicleMath::Scale(
          impulse, tuning->contactResponse.specialContactImpulseMagnitude);
      AddVehicleImpulse(impulse);
      contacts.specialContactImpulseCooldownUntil = tick + 100;
    }
    break;
  case CSceneVehicleCarSpecialContactMode_SolidFeedback: {
    const float feedbackScale =
        tuning->contactResponse.specialSolidFeedbackValue;
    HmsItem()->Solid()->Physical().Parameters().vehicleContactFeedbackScale =
        feedbackScale;
    HmsItem()->RefreshCorpusesFromSolid();
  } break;
  case CSceneVehicleCarSpecialContactMode_Turbo:
    turbo.type = ETurboType_Direct;
    turbo.impulseScale = tuning->turbo.impulseScaleA;
    break;
  default:
    break;
  }
}

static CSceneVehicleCarImpactState
impact_severity(float bucket, float lowThreshold, float highThreshold,
                CSceneVehicleCarImpactState current) {
  if (!(lowThreshold < bucket)) {
    return current;
  }
  if (highThreshold < bucket) {
    return current < CSceneVehicleCarImpactState_High
               ? CSceneVehicleCarImpactState_High
               : current;
  }
  return current == CSceneVehicleCarImpactState_None
             ? CSceneVehicleCarImpactState_Low
             : current;
}

void CSceneVehicleCar::UpdateImpactStates(CSceneVehicleCarTuning *tuning) {
  contacts.frontWheelImpactState =
      impact_severity(contacts.frontWheelImpactBucket,
                      tuning->contactResponse.wheelImpactFeedbackLowThreshold,
                      tuning->contactResponse.wheelImpactFeedbackHighThreshold,
                      contacts.frontWheelImpactState);
  contacts.rearWheelImpactState =
      impact_severity(contacts.rearWheelImpactBucket,
                      tuning->contactResponse.wheelImpactFeedbackLowThreshold,
                      tuning->contactResponse.wheelImpactFeedbackHighThreshold,
                      contacts.rearWheelImpactState);
  contacts.bodyImpactState =
      impact_severity(contacts.bodyImpactBucket,
                      tuning->contactResponse.bodyImpactFeedbackLowThreshold,
                      tuning->contactResponse.bodyImpactFeedbackHighThreshold,
                      contacts.bodyImpactState);

  if (contacts.peakRearWheelImpactState < contacts.rearWheelImpactState) {
    contacts.peakRearWheelImpactState = contacts.rearWheelImpactState;
    contacts.peakWheelImpactMaterial = contacts.lastWheelContactMaterial;
  }
  if (contacts.peakFrontWheelImpactState < contacts.frontWheelImpactState) {
    contacts.peakFrontWheelImpactState = contacts.frontWheelImpactState;
    contacts.peakWheelImpactMaterial = contacts.lastWheelContactMaterial;
  }
  if (contacts.peakBodyImpactState < contacts.bodyImpactState) {
    contacts.peakBodyImpactState = contacts.bodyImpactState;
    contacts.peakBodyImpactMaterial = contacts.lastBodyContactMaterial;
  }
}

void CSceneVehicleCar::ProcessTurboContacts(CSceneVehicleCarTuning *tuning,
                                            u32 tick) {
  GmVec3 peerAxis;
  unsigned long peerCorpusId = 0u;

  if (IsGroundContactId(TurboDurationAContactId, peerAxis, peerCorpusId) != 0) {
    EnableTurbo(tick, tuning->turbo.durationA,
                tuning->turbo.impulseScaleA * peerAxis.z, ETurboType_Direct,
                peerCorpusId);
  }
  if (IsGroundContactId(TurboDurationBContactId, peerAxis, peerCorpusId) != 0) {
    EnableTurbo(tick, tuning->turbo.durationB,
                tuning->turbo.impulseScaleB * peerAxis.z, ETurboType_Direct,
                peerCorpusId);
  }
  if (IsGroundContactId(TurboRouletteContactId, peerAxis, peerCorpusId) != 0) {
    EnableTurbo(tick, tuning->turbo.durationA,
                tuning->turbo.impulseScaleA * peerAxis.z, ETurboType_Roulette,
                peerCorpusId);
  }
  if (IsGroundContactId(ForcedLowSpeedFrictionContactId, peerAxis,
                        peerCorpusId) != 0) {
    controls.forcedLowSpeedFriction = 1;
  }
}

void CSceneVehicleCar::UpdateTurbo(unsigned long tick) {
  const u32 tickValue = static_cast<u32>(tick);
  if (turbo.type != ETurboType_Inactive) {
    if (tickValue > turbo.endTick) {
      turbo.type = ETurboType_Inactive;
    }
    if (turbo.type != ETurboType_Inactive) {
      float elapsed = UnsignedTickDelta(tickValue, turbo.startTick);
      float duration = UnsignedTickDelta(turbo.endTick, turbo.startTick);
      turbo.progressRatio = elapsed / duration;
      return;
    }
  }

  turbo.progressRatio = 0.0f;
}

void CSceneVehicleCar::EnableTurbo(unsigned long tick, unsigned long duration,
                                   float impulseScale, ETurboType nextTurboType,
                                   unsigned long sourceCorpusIdValue) {
  const CHmsCorpusId sourceCorpusId =
      CHmsCorpusId::FromValue(static_cast<u32>(sourceCorpusIdValue));
  const u32 tickValue = static_cast<u32>(tick);
  const u32 durationValue = static_cast<u32>(duration);

  if (turbo.type != nextTurboType) {
    turbo.startTick = tickValue;
    if (turboSoundSource.has_value()) {
      turboSoundSource->get().Play();
    }
    turbo.sourceCorpusId = CHmsCorpusId{};
  }

  if (nextTurboType == ETurboType_Direct) {
    turbo.impulseScale = impulseScale;
  } else if (nextTurboType == ETurboType_Roulette &&
             turbo.sourceCorpusId != sourceCorpusId) {
    float phase = CSceneVehicleCar::GetRouletteValue01(
        tickValue - turbo.rouletteTickOrigin,
        CSceneVehicleCar::s_TurboRoulettePeriodMs);
    turbo.type2Phase = phase;
    turbo.impulseScale =
        CSceneVehicleCar::GetRouletteBoostFactorFromValue01(phase) *
        impulseScale;
    turbo.sourceCorpusId = sourceCorpusId;
  }

  turbo.endTick = tickValue + durationValue;
  turbo.type = nextTurboType;
}

void CSceneVehicleCar::UpdateFeedbackSpringAxis(GmSpring<float> &spring,
                                                float dt, float savedForceAxis,
                                                float savedImpulseAxis,
                                                int invertDrive) {
  // signed drive, then clamp drive, value, and velocity independently.
  spring.Integrate(dt);

  float unclampedSpringDrive = invertDrive != 0
                                   ? (-savedForceAxis * dt - savedImpulseAxis)
                                   : (savedForceAxis * dt + savedImpulseAxis);
  float springDrive =
      ClampSymmetric(unclampedSpringDrive, feedback.springDriveLimit);
  float springDelta =
      (springDrive / feedback.springDriveLimit) * feedback.springVelocityLimit;

  spring.value = ClampSymmetric(spring.value, feedback.springValueLimit);
  spring.velocity = ClampSymmetric((spring.velocity + springDelta),
                                   feedback.springVelocityLimit);
}

void CSceneVehicleCar::UpdateFeedbackTail(CSceneVehicleCarTuning *tuning,
                                          float dt, const GmVec3 &linearSpeed,
                                          const GmVec3 &savedForce,
                                          const GmVec3 &savedImpulse,
                                          float surfaceFeedback) {
  HmsItem()->GetForce(gearedDrive.scaledCurrentForce);
  float forceScale = 1.0f / tuning->feedback.forceDivisor;
  gearedDrive.scaledCurrentForce.x =
      forceScale * gearedDrive.scaledCurrentForce.x;
  gearedDrive.scaledCurrentForce.y =
      forceScale * gearedDrive.scaledCurrentForce.y;
  gearedDrive.scaledCurrentForce.z =
      forceScale * gearedDrive.scaledCurrentForce.z;

  float surfaceCurveValue =
      tuning->feedback.surfaceCurve.Value().GetValue(surfaceFeedback, nullptr);
  float feedbackRate = surfaceCurveValue + tuning->feedback.surfaceBaseRate;
  float unclampedSurfaceFeedback =
      feedbackRate * dt + feedback.surfaceAccumulator;
  feedback.surfaceAccumulator = unclampedSurfaceFeedback;
  feedback.surfaceAccumulator = ClampZeroOne(unclampedSurfaceFeedback);

  UpdateFeedbackSpringAxis(feedback.sideSpring, dt, savedForce.x,
                           savedImpulse.x, 0);
  UpdateFeedbackSpringAxis(feedback.forwardSpring, dt, savedForce.z,
                           savedImpulse.z, 1);

  float contactRampDirection =
      IsAllWheelGroundContactId(FeedbackRampContactId) != 0 ? 1.0f : -1.0f;
  float curveInput =
      std::fabs(linearSpeed.z * SceneVehicleMath::SpeedKilometersPerHourScale);
  float ramp0 = FeedbackRamp0Curve().GetValue(curveInput, nullptr);
  feedback.ramp0 =
      ClampZeroOne(feedback.ramp0 + ramp0 * dt * contactRampDirection);

  float ramp1 = FeedbackRamp1Curve().GetValue(curveInput, nullptr);
  feedback.ramp1 =
      ClampZeroOne(feedback.ramp1 + ramp1 * dt * contactRampDirection);
}

void CSceneVehicleCar::ClearWheelContactScratch() {
  u32 wheelCount = WheelGetCount();
  for (u32 wheelIndex = 0; wheelIndex < wheelCount; wheelIndex++) {
    CSceneVehicleCar::SSimulationWheel *wheel = &WheelAt(wheelIndex);
    wheel->realTimeState.contactPresent = false;
    wheel->realTimeState.contactNormalSampleCount = 0;
    wheel->realTimeState.latestContactPoint = SceneVehicleMath::Zero();
    wheel->realTimeState.accumulatedContactNormal = SceneVehicleMath::Zero();
    wheel->ClearContactCarryStamp();
    wheel->realTimeState.rejectedNormalContact = false;
  }
}

void CSceneVehicleCar::ResetPerTickContactFeedback() {
  contacts.frontWheelImpactBucket = 0.0f;
  contacts.bodyContactPresent = false;
  contacts.rearWheelImpactBucket = 0.0f;
  airControl.refreshMemory = false;
  contacts.bodyImpactBucket = 0.0f;
  contacts.lateralSlowDownContactActive = false;
}

void CSceneVehicleCar::SaveAndClearAccumulatedFeedback(GmVec3 &savedForce,
                                                       GmVec3 &savedImpulse) {
  // Feedback consumes the completed force snapshot after the accumulators
  // clear.
  savedImpulse.x = forceAccumulators.impulse.x;
  savedImpulse.y = forceAccumulators.impulse.y;
  savedImpulse.z = forceAccumulators.impulse.z;
  savedForce.x = forceAccumulators.force.x;
  savedForce.y = forceAccumulators.force.y;
  savedForce.z = forceAccumulators.force.z;

  forceAccumulators.force.z = 0.0f;
  forceAccumulators.force.y = 0.0f;
  forceAccumulators.force.x = 0.0f;
  forceAccumulators.impulse.z = 0.0f;
  forceAccumulators.impulse.y = 0.0f;
  forceAccumulators.impulse.x = 0.0f;
}

void CSceneVehicleCar::ComputeForces(float dt) {

  GmVec3 savedForce;
  GmVec3 savedImpulse;
  SaveAndClearAccumulatedFeedback(savedForce, savedImpulse);

  if (integration.speedBlocked || integration.speedBlockedSecondary ||
      WaterState().boxLocal.halfExtents.x < 0.0f) {
    SetZeroDynamics();
    return;
  }

  CreateFakeContacts();
  IntegrateVehicle(dt);

  u32 tick = CMwCmdBufferCore::Current()->Timer().GetTickTime();
  int isGroundContact = IsGroundContact();
  CSceneVehicleCarTuning *tuning = Tunings()->ActiveTuning();
  UpdateDynaParamsForGroundContact(tuning, isGroundContact);

  if (!integration.integrateWheels) {
    return;
  }

  GmVec3 linearSpeed;
  GmVec3 angularSpeed;
  GmVec3 currentForce;
  CSceneVehicleMaterial::SBlendableVals materialVals = {1.0f, 1.0f, 1.0f, 1.0f};
  int hasGroundMaterial = 0;
  float slopeAdherenceA = 1.0f;
  float slopeAdherenceB = 1.0f;
  float visualSteerYaw = 0.0f;
  int modelSlipFlag = 0;
  int hasSideSpeedKillContact = 0;
  int hasAnyContact = 0;
  HmsItem()->GetLinearSpeed(linearSpeed);

  float surfaceFeedback = 0.0f;
  if (integration.zeroHorizontalSpeed) {
    linearSpeed.x = 0.0f;
    linearSpeed.z = 0.0f;
    HmsItem()->SetLinearSpeed(linearSpeed);
  } else {
    HmsItem()->GetAngularSpeed(angularSpeed);
    HmsItem()->GetForce(currentForce);

    engine.lowSpeedFeedbackForce = 0.0f;
    ApplyFrictionForces(linearSpeed);
    ClampLinearSpeed(linearSpeed);

    ComputeVehicleGroundMaterialVals(materialVals, hasGroundMaterial);

    GetSlopeAdherence(currentForce, slopeAdherenceA, slopeAdherenceB);

    visualSteerYaw = ComputeVisualSteerYaw(tuning, linearSpeed);
    gearedDrive.localSpeed = linearSpeed;

    ComputeSelectedHandlingForces(tuning, dt, currentForce, slopeAdherenceA,
                                  slopeAdherenceB, linearSpeed, angularSpeed,
                                  visualSteerYaw, hasGroundMaterial,
                                  materialVals, modelSlipFlag, surfaceFeedback);

    hasAnyContact = ScanWheelSideSpeedKillContacts(hasSideSpeedKillContact);
    UpdateLowSpeedFeedback(tuning, hasAnyContact);
    KillSideSpeedForTaggedContact(tuning, hasSideSpeedKillContact, linearSpeed);

    ComputeAirControl(angularSpeed, tick, isGroundContact,
                      hasSideSpeedKillContact);

    ApplySpecialContactResponse(tuning, currentForce, tick, isGroundContact);
    UpdateImpactStates(tuning);
    lastComputeForcesTick = tick;
    ProcessTurboContacts(tuning, tick);
    UpdateTurbo(tick);

    OtherVehicleForces();
  }

  UpdateFeedbackTail(tuning, dt, linearSpeed, savedForce, savedImpulse,
                     surfaceFeedback);
  ClearWheelContactScratch();
  ResetPerTickContactFeedback();
}
