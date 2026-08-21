// Wheel suspension, contact absorption, and impulse application.

#include "engine/scene/scene_vehicle_car_internal.h"
using namespace SceneVehicleCarDynamics;

void CSceneVehicleCar::OtherVehicleForces() {}

float CSceneVehicleCar::GetRouletteValue01(unsigned long tickDelta,
                                           unsigned long period) {
  u32 remainder = tickDelta % period;
  float phase = (Binary32::FromUnsignedInteger(remainder) /
                 Binary32::FromUnsignedInteger(period));
  if (phase < TurboRouletteZeroPhaseEnd) {
    return 0.0f;
  }
  if (phase < TurboRouletteHalfPhaseEnd) {
    return 0.5f;
  }
  return 1.0f;
}

float CSceneVehicleCar::GetRouletteBoostFactorFromValue01(float value) {
  return Binary32::FromDouble(static_cast<double>(value) +
                              TurboRouletteBoostBase);
}

float CSceneVehicleCar::GetRouletteCurrentBoostFactor(void) const {
  return GetRouletteBoostFactorFromValue01(turbo.type2Phase);
}

void CSceneVehicleCar::OtherUpdateAsync(void) {}

void CSceneVehicleCar::OtherVehiclePhysics(void) {}

void SDynaMath::ComputeImpulse(float mass, const GmMat3 &inertia,
                               float restitution, const GmVec3 &speed,
                               const GmVec3 &normal, const GmVec3 &lever,
                               GmVec3 &out) {
  GmVec3 angularUnit = {
      SceneVehicleMath::SubtractProducts(normal.z, lever.y, normal.y, lever.z),
      SceneVehicleMath::SubtractProducts(normal.x, lever.z, lever.x, normal.z),
      SceneVehicleMath::SubtractProducts(normal.y, lever.x, normal.x, lever.y),
  };
  angularUnit.Mult(inertia);

  GmVec3 angularUnitStored = angularUnit;
  GmVec3 angularSpeedAtPoint = {
      SceneVehicleMath::SubtractProducts(lever.z, angularUnitStored.y,
                                         angularUnitStored.z, lever.y),
      SceneVehicleMath::SubtractProducts(lever.x, angularUnitStored.z, lever.z,
                                         angularUnitStored.x),
      SceneVehicleMath::SubtractProducts(angularUnitStored.x, lever.y, lever.x,
                                         angularUnitStored.y),
  };

  float speedAlongNormal = SceneVehicleMath::DotYXZ(speed, normal);
  float numerator = (restitution - 1.0f) * speedAlongNormal;

  float angularMassTerm = SceneVehicleMath::DotXYZ(normal, angularSpeedAtPoint);
  float effectiveMassDenom = 1.0f / mass + angularMassTerm;

  float absDenom = std::fabs(effectiveMassDenom);
  if (absDenom < ScalarEpsilon) {
    out.z = 0.0f;
    out.y = 0.0f;
    out.x = 0.0f;
  }

  float impulseScale = numerator / effectiveMassDenom;
  out.x = normal.x * impulseScale;
  out.y = normal.y * impulseScale;
  out.z = impulseScale * normal.z;
}

void CSceneVehicleCar::ComputeAndApplyContactImpulse(float restitution,
                                                     const GmVec3 &speed,
                                                     const GmVec3 &normal,
                                                     const GmVec3 &point) {
  CPlugSolid *solid = HmsItem()->Solid();
  GmVec3 lever = SolidLeverToPoint(*solid, point);
  GmVec3 impulse;
  SDynaMath::ComputeImpulse(solid->Physical().Parameters().mass,
                            solid->Physical().Parameters().impulseInertia,
                            restitution, speed, normal, lever, impulse);
  AddVehicleImpulse(impulse, point);
}

void CSceneVehicleCar::AddVehicleForce(const GmVec3 &force,
                                       const GmVec3 &point) {
  HmsItem()->AddForce(force, point);
  forceAccumulators.force.x = forceAccumulators.force.x + force.x;
  forceAccumulators.force.y = force.y + forceAccumulators.force.y;
  forceAccumulators.force.z = force.z + forceAccumulators.force.z;
}

void CSceneVehicleCar::AddVehicleCentralForce(const GmVec3 &force) {
  HmsItem()->AddForce(force);
  forceAccumulators.force.x = forceAccumulators.force.x + force.x;
  forceAccumulators.force.y = force.y + forceAccumulators.force.y;
  forceAccumulators.force.z = force.z + forceAccumulators.force.z;
}

void CSceneVehicleCar::AddVehicleTorque(const GmVec3 &torque) {
  HmsItem()->AddTorque(torque);
}

void CSceneVehicleCar::SetVehicleLinearSpeed(const GmVec3 &speed) {
  HmsItem()->SetLinearSpeed(speed);
}

void CSceneVehicleCar::SetVehicleAngularSpeed(const GmVec3 &speed) {
  HmsItem()->SetAngularSpeed(speed);
}

void CSceneVehicleCar::AccumulateCentralImpulse(const GmVec3 &impulse) {
  forceAccumulators.impulse.x = forceAccumulators.impulse.x + impulse.x;
  forceAccumulators.impulse.y = impulse.y + forceAccumulators.impulse.y;
  forceAccumulators.impulse.z = impulse.z + forceAccumulators.impulse.z;
}

void CSceneVehicleCar::AccumulatePointImpulse(const GmVec3 &impulseLocal) {
  forceAccumulators.impulse.x = impulseLocal.x + forceAccumulators.impulse.x;
  forceAccumulators.impulse.y = impulseLocal.y + forceAccumulators.impulse.y;
  forceAccumulators.impulse.z = impulseLocal.z + forceAccumulators.impulse.z;
}

GmVec3 CSceneVehicleCar::BuildLinearSpeedAfterPointImpulse(
    CHmsDyna::CHmsStateDyna &state, const GmVec3 &worldImpulse, float invMass) {
  float deltaX = worldImpulse.x * invMass;
  float deltaY = worldImpulse.y * invMass;
  float deltaZ = worldImpulse.z * invMass;
  GmVec3 candidateLinear = {
      (state.linearSpeed.x + deltaX),
      (state.linearSpeed.y + deltaY),
      (state.linearSpeed.z + deltaZ),
  };
  return candidateLinear;
}

int CSceneVehicleCar::RejectPointImpulseLinearSpeedGrowth(
    const CHmsDyna::CHmsStateDyna &state, const GmVec3 &candidateLinear,
    const CSceneVehicleCarTuning &tuning) {
  float preImpulseLinearSpeedSq = (state.linearSpeed.y * state.linearSpeed.y +
                                   state.linearSpeed.x * state.linearSpeed.x +
                                   state.linearSpeed.z * state.linearSpeed.z);
  float candidateLinearSpeedSq = (candidateLinear.y * candidateLinear.y +
                                  candidateLinear.x * candidateLinear.x +
                                  candidateLinear.z * candidateLinear.z);
  float linearSpeedGrowthSq = candidateLinearSpeedSq - preImpulseLinearSpeedSq;
  return preImpulseLinearSpeedSq < candidateLinearSpeedSq &&
         tuning.contactResponse.pointImpulseLinearSpeedGrowthLimitSq <
             linearSpeedGrowthSq;
}

void CSceneVehicleCar::ClampAngularSpeedAfterPointImpulse(
    CHmsDyna::CHmsStateDyna &state, const CSceneVehicleCarTuning &tuning) {
  float maxAngularSpeed = tuning.contactResponse.pointImpulseAngularSpeedMax;
  float currentAngularSpeedSq = (state.angularSpeed.y * state.angularSpeed.y +
                                 state.angularSpeed.x * state.angularSpeed.x +
                                 state.angularSpeed.z * state.angularSpeed.z);
  if (currentAngularSpeedSq > maxAngularSpeed * maxAngularSpeed) {
    const double maxAngularSpeedDouble = static_cast<double>(maxAngularSpeed);
    float currentAngularSpeedLen = CIsqrt(currentAngularSpeedSq);
    const float angularClampScale = Binary32::FromDouble(
        maxAngularSpeedDouble / static_cast<double>(currentAngularSpeedLen));
    state.angularSpeed.x = state.angularSpeed.x * angularClampScale;
    state.angularSpeed.y = state.angularSpeed.y * angularClampScale;
    state.angularSpeed.z = state.angularSpeed.z * angularClampScale;
  }
}

void CSceneVehicleCar::AddVehicleImpulse(const GmVec3 &impulse) {
  HmsItem()->AddImpulse(impulse);
  AccumulateCentralImpulse(impulse);
}

void CSceneVehicleCar::AddVehicleImpulse(const GmVec3 &impulseLocal,
                                         const GmVec3 &pointLocal) {
  CHmsCorpus *corpus = HmsItem()->CorpusAt(0u);
  CHmsDyna *dyna = corpus->Dynamics();
  if (dyna == nullptr) {
    return;
  }

  CHmsDyna::CHmsStateDyna *state = &dyna->CurrentState();
  CHmsDynaParams *params = &dyna->Parameters();
  CSceneVehicleCarTuning *tuning = Tunings()->ActiveTuning();

  GmVec3 worldImpulse;
  worldImpulse.SetMult(impulseLocal, state->rotation);
  // Transform the contact point and center of mass through the body's pose.
  GmVec3 worldPoint;
  GmIso4 stateIso;
  stateIso.Set(state->rotation, state->position);
  worldPoint.SetMult(pointLocal, stateIso);

  GmVec3 worldCom;
  worldCom.SetMult(params->localCenterOfMass, stateIso);

  float invMass = 1.0f / params->mass;
  GmVec3 candidateLinear =
      this->BuildLinearSpeedAfterPointImpulse(*state, worldImpulse, invMass);
  if (this->RejectPointImpulseLinearSpeedGrowth(*state, candidateLinear,
                                                *tuning)) {
    candidateLinear = SceneVehicleMath::Zero();
  }
  state->linearSpeed = candidateLinear;

  GmVec3 worldComToPoint;
  this->BuildWorldComToImpulsePoint(worldComToPoint, worldPoint, worldCom);
  GmVec3 pointImpulseAngularDelta;
  this->BuildPointImpulseTorqueSeed(pointImpulseAngularDelta, worldComToPoint,
                                    worldImpulse);
  pointImpulseAngularDelta.Mult(state->inverseInertiaWorld);

  this->ScaleAndAddPointImpulseAngularDelta(*state, pointImpulseAngularDelta,
                                            *tuning);

  this->ClampAngularSpeedAfterPointImpulse(*state, *tuning);

  AccumulatePointImpulse(impulseLocal);
}

unsigned long
CSceneVehicleCar::GetWheelFromSurfaceTree(const CPlugTree *surfaceTree) {
  u32 wheelCount = WheelGetCount();
  for (u32 wheelIndex = 0; wheelIndex < wheelCount; wheelIndex++) {
    CSceneVehicleCar::SSimulationWheel *wheel = &WheelAt(wheelIndex);
    if (wheel->surfaceHandler.Tree() == surfaceTree) {
      return wheelIndex;
    }
  }
  return UINT32_MAX;
}

int CSceneVehicleCar::IsGroundContact() {
  u32 wheelCount = WheelGetCount();
  for (u32 wheelIndex = 0; wheelIndex < wheelCount; wheelIndex++) {
    CSceneVehicleCar::SSimulationWheel *wheel = &WheelAt(wheelIndex);
    if (wheel->realTimeState.contactPresent) {
      return 1;
    }
  }
  return 0;
}

float CSceneVehicleCar::GetMaxSpeed(void) {
  const CSceneVehicleCarTuning *tuning = Tunings()->ActiveTuning();
  const float rounded =
      (static_cast<double>(tuning->engineSpeedNorm) *
       static_cast<double>(SceneVehicleMath::SpeedKilometersPerHourScale));
  return rounded;
}

unsigned long CSceneVehicleCar::WheelGetCount(void) const {
  return static_cast<unsigned long>(wheels.size());
}

int CSceneVehicleCar::WheelIsSliding(unsigned long wheelIndex) const {
  const CSceneVehicleCar::SSimulationWheel *wheel = &WheelAt(wheelIndex);
  return wheel->asyncState.slipping && wheel->asyncState.contactPresent;
}

unsigned short
CSceneVehicleCar::WheelGetContactMaterial(unsigned long wheelIndex) const {
  const CSceneVehicleCar::SSimulationWheel *wheel = &WheelAt(wheelIndex);
  if (!wheel->asyncState.contactPresent) {
    return 0xffff;
  }
  return wheel->asyncState.contactMaterial;
}

const GmVec3 &CSceneVehicleCar::WheelGetAsyncGroundContactPos(
    unsigned long wheelIndex) const {
  const CSceneVehicleCar::SSimulationWheel *wheel = &WheelAt(wheelIndex);
  return wheel->asyncState.localSurfacePoint;
}

int CSceneVehicleCar::IsAllWheelGroundContactId(uint8_t materialId) {
  u32 missingContactCount = 0;
  u32 wheelCount = WheelGetCount();
  for (u32 wheelIndex = 0; wheelIndex < wheelCount; wheelIndex++) {
    CSceneVehicleCar::SSimulationWheel *wheel = &WheelAt(wheelIndex);
    if (!wheel->realTimeState.contactPresent) {
      missingContactCount++;
      continue;
    }

    if (wheel->realTimeState.contactMaterial !=
        static_cast<EPlugSurfaceMaterialId>(materialId)) {
      return 0;
    }
  }

  return missingContactCount != wheelCount;
}

int CSceneVehicleCar::IsGroundContactId(uint8_t materialId, GmVec3 &outPeerAxis,
                                        unsigned long &outPeerCorpusId) {
  u32 wheelCount = WheelGetCount();
  for (u32 wheelIndex = 0; wheelIndex < wheelCount; wheelIndex++) {
    CSceneVehicleCar::SSimulationWheel *wheel = &WheelAt(wheelIndex);
    if (wheel->realTimeState.contactPresent &&
        wheel->realTimeState.contactMaterial ==
            static_cast<EPlugSurfaceMaterialId>(materialId)) {
      outPeerAxis = wheel->realTimeState.peerZAxisInCarLocal;
      outPeerCorpusId = wheel->realTimeState.peerCorpusId.Value();
      return 1;
    }
  }

  return 0;
}

void CSceneVehicleCar::WheelAddForceToVehicle(
    CSceneVehicleCar::SSimulationWheel &wheel,
    const GmVec3 &ignoredInputForce) {
  (void)ignoredInputForce;

  CSceneVehicleCarTuning *tuning = Tunings()->ActiveTuning();
  if (!wheel.realTimeState.contactPresent) {
    return;
  }

  GmVec3 force = {0.0f, 0.0f, 0.0f};

  if (tuning->wheelForceMode == CSceneVehicleCarWheelForceMode_DirectSpring) {
    // Static suspension force is evaluated in the declared operation order.
    force.y = SceneVehicleMath::DirectSpringForce(
        tuning->suspension.wheelRestDamperAbsorb,
        wheel.realTimeState.damperAbsorb, tuning->suspension.wheelSpringCoef,
        tuning->suspension.wheelStaticSpringScale);
    AddVehicleForce(force, wheel.forceApplicationPoint);
    return;
  }

  if (tuning->wheelForceMode == CSceneVehicleCarWheelForceMode_FollowAbsorb ||
      tuning->wheelForceMode ==
          CSceneVehicleCarWheelForceMode_FollowAbsorbWithImpulse) {
    force.y = SceneVehicleMath::FollowAbsorbForce(
        tuning->suspension.wheelRestDamperAbsorb,
        wheel.realTimeState.damperAbsorb, tuning->suspension.wheelSpringCoef,
        tuning->suspension.wheelDamperCoef, wheel.realTimeState.damperVelocity);
    AddVehicleForce(force, wheel.forceApplicationPoint);
  }
}

static float BodyContactImpulse(const CSceneVehicleCarTuning &tuning,
                                EPlugSurfaceMaterialId material) {
  return material == EPlugSurfaceMaterialId_Metal
             ? tuning.contactResponse.bodyContactImpulseMetal
             : tuning.contactResponse.bodyContactImpulseOther;
}

static float BodyContactTangentLimit(const CSceneVehicleCarTuning &tuning,
                                     EPlugSurfaceMaterialId material) {
  return material == EPlugSurfaceMaterialId_Metal
             ? tuning.contactResponse.bodyContactTangentLimitMetal
             : tuning.contactResponse.bodyContactTangentLimitOther;
}

static float WheelContactImpulse(const CSceneVehicleCarTuning &tuning,
                                 EPlugSurfaceMaterialId material) {
  return material == EPlugSurfaceMaterialId_Metal
             ? tuning.contactResponse.wheelContactImpulseMetal
             : tuning.contactResponse.wheelContactImpulseOther;
}

static void ApplyFollowAbsorbImpulse(CSceneVehicleCar &car,
                                     CSceneVehicleCar::SSimulationWheel &wheel,
                                     CHmsPhysicalContact &contact,
                                     const CSceneVehicleCarTuning &tuning,
                                     bool replacementClampBlocked) {
  const float restitution = -WheelContactImpulse(tuning, contact.peerMaterial);
  GmVec3 speedAtContact = {
      contact.localRelativeSpeed.x,
      contact.localRelativeSpeed.y,
      contact.localRelativeSpeed.z,
  };
  float normalSpeed =
      SceneVehicleMath::DotYXZ(contact.localImpulseNormal, speedAtContact);
  if (!(normalSpeed < 0.0f)) {
    return;
  }

  GmVec3 normalComponent = {
      contact.localImpulseNormal.x * normalSpeed,
      contact.localImpulseNormal.y * normalSpeed,
      normalSpeed * contact.localImpulseNormal.z,
  };
  float normalComponentY = SceneVehicleMath::VerticalComponent(normalComponent);
  if (replacementClampBlocked != 0 || !(normalComponentY < 0.0f)) {
    car.ComputeAndApplyContactImpulse(restitution, speedAtContact,
                                      contact.localImpulseNormal,
                                      contact.localContactPoint);
    return;
  }

  float normalComponentXZ = SceneVehicleMath::ZeroScaled(normalComponentY);
  GmVec3 verticalNormalComponent = {
      normalComponentXZ,
      SceneVehicleMath::VerticalComponent(normalComponent),
      normalComponentXZ,
  };
  GmVec3 speedWithoutVerticalNormal = {
      contact.localRelativeSpeed.x,
      contact.localRelativeSpeed.y,
      contact.localRelativeSpeed.z,
  };
  speedWithoutVerticalNormal.x =
      speedWithoutVerticalNormal.x - verticalNormalComponent.x;
  speedWithoutVerticalNormal.y =
      speedWithoutVerticalNormal.y - normalComponentY;
  speedWithoutVerticalNormal.z =
      speedWithoutVerticalNormal.z - verticalNormalComponent.z;
  if (SceneVehicleMath::DotYXZ(contact.localImpulseNormal,
                               speedWithoutVerticalNormal) < 0.0f) {
    const GmVec3 currentSurfacePoint = wheel.surfaceHandler.CurrentPoint();
    car.ComputeAndApplyContactImpulse(restitution, speedWithoutVerticalNormal,
                                      contact.localImpulseNormal,
                                      currentSurfacePoint);
  }
}

static bool
ApplyFollowAbsorbReplacement(CSceneVehicleCar::SSimulationWheel &wheel,
                             CHmsPhysicalContact &contact,
                             const CSceneVehicleCarTuning &tuning) {
  float replacementY = SceneVehicleMath::VerticalComponent(contact.replacement);
  bool replacementClampBlocked = false;
  if (replacementY > 0.0f) {
    float replacementCandidate = replacementY;
    float damperAbsorbMax = tuning.suspension.damperModulationMaxAbsorb;
    if (damperAbsorbMax >= -ScalarEpsilon) {
      float clampedReplacementY =
          wheel.realTimeState.damperAbsorb - damperAbsorbMax;
      if (clampedReplacementY <= replacementY) {
        replacementCandidate = clampedReplacementY;
        replacementClampBlocked = true;
      }
    }

    float replacementToSubtract = replacementCandidate;
    float previousMax = wheel.realTimeState.maxReplacementY;
    float replacementMax = replacementToSubtract;
    if (!(replacementToSubtract > previousMax)) {
      replacementMax = previousMax;
    }
    wheel.realTimeState.maxReplacementY = replacementMax;

    float replacementXZ = SceneVehicleMath::ZeroScaled(replacementToSubtract);
    GmVec3 verticalReplacement = {
        replacementXZ,
        replacementToSubtract,
        replacementXZ,
    };
    contact.replacement.x = contact.replacement.x - verticalReplacement.x;
    contact.replacement.y = contact.replacement.y - verticalReplacement.y;
    contact.replacement.z = contact.replacement.z - verticalReplacement.z;
  } else {
    replacementClampBlocked = true;
  }
  return replacementClampBlocked;
}

void CSceneVehicleCar::WheelAbsorbContact(
    CSceneVehicleCar::SSimulationWheel &wheel, CHmsPhysicalContact &contact) {
  float maxNormalX = CIsinQuarterPi();
  wheel.realTimeState.contactPresent =
      std::fabs(contact.localImpulseNormal.x) < maxNormalX;
  if (!wheel.realTimeState.contactPresent) {
    contacts.lateralSlowDownContactActive = 1;
    wheel.realTimeState.rejectedNormalContactPoint = contact.localContactPoint;
    wheel.realTimeState.rejectedNormalContact = true;
  }

  if (wheel.realTimeState.contactPresent) {
    wheel.realTimeState.contactNormalSampleCount++;
    SceneVehicleMath::Accumulate(wheel.realTimeState.accumulatedContactNormal,
                                 contact.localImpulseNormal);
    wheel.realTimeState.contactMaterial = contact.peerMaterial;
  }

  contact.accepted = 0;

  if (contact.peer != nullptr) {
    const GmIso4 *peerIso = contact.peer->GetLocation();
    peerIso->GetDir(wheel.realTimeState.peerZAxisInCarLocal);
    wheel.realTimeState.peerCorpusId = contact.peer->Id();

    CHmsCorpus *selfCorpus = HmsItem()->CorpusAt(0u);
    const GmIso4 *selfIso = selfCorpus->GetLocation();
    wheel.realTimeState.peerZAxisInCarLocal.MultTranspose(
        selfIso->RotationMatrix());
  }

  wheel.realTimeState.latestContactPoint = contact.localContactPoint;

  CSceneVehicleCarTuning *tuning = Tunings()->ActiveTuning();
  if (tuning->wheelForceMode !=
      CSceneVehicleCarWheelForceMode_FollowAbsorbWithImpulse) {
    return;
  }

  const bool replacementClampBlocked =
      ApplyFollowAbsorbReplacement(wheel, contact, *tuning);

  if (wheel.realTimeState.contactPresent) {
    ApplyFollowAbsorbImpulse(*this, wheel, contact, *tuning,
                             replacementClampBlocked);
    return;
  }

  const float restitution = -BodyContactImpulse(*tuning, contact.peerMaterial);
  if (SceneVehicleMath::DotYXZ(contact.localImpulseNormal,
                               contact.localRelativeSpeed) < 0.0f) {
    GmVec3 impulsePoint = {
        (contact.localContactPoint.x),
        (HmsItem()->Solid()->Physical().Parameters().localCenterOfMass.y),
        (contact.localContactPoint.z),
    };
    ComputeAndApplyContactImpulse(restitution, contact.localRelativeSpeed,
                                  contact.localImpulseNormal, impulsePoint);
  }
}

static void AbsorbBodyContactWithImpulse(CSceneVehicleCar &car,
                                         CHmsPhysicalContact &contact,
                                         const CSceneVehicleCarTuning &tuning) {
  // Keep the Y/X/Z accumulation order explicit: it is part of the numerical
  // rule.
  float speedAlongNormal =
      contact.localImpulseNormal.y * contact.localRelativeSpeed.y +
      contact.localImpulseNormal.x * contact.localRelativeSpeed.x +
      contact.localImpulseNormal.z * contact.localRelativeSpeed.z;
  if (speedAlongNormal < 0.0f) {
    float restitution = -BodyContactImpulse(tuning, contact.peerMaterial);
    float tangentLimit = BodyContactTangentLimit(tuning, contact.peerMaterial);

    GmVec3 normalSpeed = {
        contact.localImpulseNormal.x * speedAlongNormal,
        contact.localImpulseNormal.y * speedAlongNormal,
        speedAlongNormal * contact.localImpulseNormal.z,
    };
    GmVec3 tangentSpeed = {
        contact.localRelativeSpeed.x - normalSpeed.x,
        contact.localRelativeSpeed.y - normalSpeed.y,
        contact.localRelativeSpeed.z - normalSpeed.z,
    };
    float normalSpeedLen =
        CIsqrt(SceneVehicleMath::LengthSquaredYXZ(normalSpeed));
    float tangentLen = CIsqrt(SceneVehicleMath::LengthSquaredYXZ(tangentSpeed));
    float tangentMax = normalSpeedLen * tangentLimit;
    if (tangentLen > tangentMax) {
      float tangentClampScale = tangentMax / tangentLen;
      tangentSpeed.x = tangentClampScale * tangentSpeed.x;
      tangentSpeed.y = tangentSpeed.y * tangentClampScale;
      tangentSpeed.z = tangentClampScale * tangentSpeed.z;
    }

    GmVec3 impulseNormal = {
        -(tangentSpeed.x + normalSpeed.x),
        -(tangentSpeed.y + normalSpeed.y),
        -(tangentSpeed.z + normalSpeed.z),
    };
    float impulseNormalLen =
        CIsqrt(SceneVehicleMath::LengthSquaredYXZ(impulseNormal));
    if (ScalarEpsilon < impulseNormalLen) {
      float invLen = 1.0f / impulseNormalLen;
      impulseNormal.x = invLen * impulseNormal.x;
      impulseNormal.y = impulseNormal.y * invLen;
      impulseNormal.z = invLen * impulseNormal.z;
      car.ComputeAndApplyContactImpulse(restitution, contact.localRelativeSpeed,
                                        impulseNormal,
                                        contact.localContactPoint);
    }
  }

  contact.accepted = 0;
}

void CSceneVehicleCar::AbsorbContact(CHmsPhysicalContact &contact) {
  if (contact.peerMaterial == EPlugSurfaceMaterialId_Water ||
      contact.peerMaterial == EPlugSurfaceMaterialId_GolfBall) {
    contact.accepted = 0;
    contact.replacement = SceneVehicleMath::Zero();
    return;
  }

  airControl.refreshMemory = 1;
  u32 wheelIndex = GetWheelFromSurfaceTree(contact.ContactTree());
  float impact = std::fabs(SceneVehicleMath::DotYXZ(
      contact.localImpulseNormal, contact.localRelativeSpeed));
  if (wheelIndex == UINT32_MAX || !(contact.localImpulseNormal.y > 0.2f)) {
    contacts.bodyImpactBucket = contacts.bodyImpactBucket + impact;
  } else {
    CSceneVehicleCar::SSimulationWheel *wheel = &WheelAt(wheelIndex);
    if (!IsFrontVehicleWheel(wheel->axle)) {
      contacts.rearWheelImpactBucket =
          (contacts.rearWheelImpactBucket + impact);
    } else {
      contacts.frontWheelImpactBucket =
          (impact + contacts.frontWheelImpactBucket);
    }
  }

  if (wheelIndex != UINT32_MAX) {
    contacts.lastWheelContactMaterial = contact.peerMaterial;
    CSceneVehicleCar::SSimulationWheel *wheel = &WheelAt(wheelIndex);
    WheelAbsorbContact(*wheel, contact);
    contacts.wheelContactCount++;
    return;
  }

  CSceneVehicleCarTuning *tuning = Tunings()->ActiveTuning();
  if (contact.localImpulseNormal.y < -0.75f &&
      tuning->handlingModel == CSceneVehicleCarHandlingModel_GearedDrive) {
    float replacementAlongNormal = SceneVehicleMath::DotYXZ(
        contact.localImpulseNormal, contact.replacement);
    contact.replacement = SceneVehicleMath::Scale(contact.localImpulseNormal,
                                                  replacementAlongNormal);
  }

  SceneVehicleMath::Accumulate(contacts.bodyContactPointSum,
                               contact.localContactPoint);
  SceneVehicleMath::Accumulate(contacts.bodyContactNormalSum,
                               contact.localImpulseNormal);
  contacts.bodyContactCount++;
  contacts.bodyContactPresent = 1;
  contacts.lastBodyContactMaterial = contact.peerMaterial;

  if (tuning->wheelForceMode ==
      CSceneVehicleCarWheelForceMode_FollowAbsorbWithImpulse) {
    AbsorbBodyContactWithImpulse(*this, contact, *tuning);
  }
}

static u32 FakeContactTextureIndex(float coordinate, float period, int dim) {
  float wrapped = CIfmod(coordinate, period);
  float absWrapped = std::fabs(wrapped);
  return static_cast<u32>((absWrapped / period) *
                          Binary32::FromUnsignedInteger(static_cast<u32>(dim)));
}

static float FakeContactSpeedFromPixel(uint8_t pixel, float forwardSpeed,
                                       float speedScale) {
  const float pixelRatio = (Binary32::FromUnsignedInteger(pixel)) / 255.0f;
  const float pixelSpeed = pixelRatio * forwardSpeed;
  return pixelSpeed * speedScale;
}

void CSceneVehicleCar::CreateFakeContacts() {
  GmVec3 linearSpeed;
  HmsItem()->GetLinearSpeed(linearSpeed);

  u32 wheelCount = WheelGetCount();
  for (u32 wheelIndex = 0; wheelIndex < wheelCount; wheelIndex++) {
    CSceneVehicleCar::SSimulationWheel *wheel = &WheelAt(wheelIndex);
    if (!wheel->realTimeState.contactPresent) {
      continue;
    }

    u32 remappedMaterialIndex =
        MaterialRemapAt(wheel->realTimeState.contactMaterial);
    CSceneVehicleMaterial *material =
        MaterialContainer()->MaterialAt(remappedMaterialIndex);
    CPlugBitmap *bitmap = material->fakeContactBitmap;
    if (bitmap == nullptr) {
      return;
    }

    CPlugFileImg *image = bitmap->Image();
    if (image->IsInSystemMemory() == 0 && bitmap->ReGenerate() == 0) {
      return;
    }
    image = bitmap->Image();

    CHmsCorpus *corpus = HmsItem()->CorpusAt(0u);
    const GmIso4 *iso = corpus->GetLocation();
    GmVec3 worldSamplePoint;
    worldSamplePoint.SetMult(wheel->surfaceHandler.RestPoint(), *iso);

    u32 pixelX = FakeContactTextureIndex(
        worldSamplePoint.x, material->fakeContactPeriodX, image->Width());
    u32 pixelY = FakeContactTextureIndex(
        worldSamplePoint.z, material->fakeContactPeriodZ, image->Height());
    const uint8_t pixel = *image->GetPixel(pixelX, pixelY);
    if (pixel == 0) {
      continue;
    }

    float fakeSpeed = FakeContactSpeedFromPixel(
        pixel, linearSpeed.z, material->fakeContactSpeedScale);
    if (material->fakeContactDepthMax < fakeSpeed) {
      fakeSpeed = material->fakeContactDepthMax;
    }

    CHmsPhysicalContact fakeContact;
    fakeContact.self = nullptr;
    fakeContact.contactTree = nullptr;
    fakeContact.ownMaterial = EPlugSurfaceMaterialId_Concrete;
    fakeContact.localImpulseNormal = GmVec3{0.0f, 1.0f, 0.0f};
    fakeContact.localContactPoint = wheel->surfaceHandler.RestPoint();
    fakeContact.localRelativeSpeed = GmVec3{0.0f, -fakeSpeed, 0.0f};
    fakeContact.replacement = SceneVehicleMath::Zero();
    fakeContact.peer = nullptr;
    fakeContact.peerCollisionTree = nullptr;
    fakeContact.peerMaterial = wheel->realTimeState.contactMaterial;
    WheelAbsorbContact(*wheel, fakeContact);
  }
}

void CSceneVehicleCar::RefreshCollisionTree() {
  CPlugTree *tree = HmsItem()->Solid()->CollisionTree();
  // A bound refresh replays the same box transforms and unions in the same
  // order; it declines whenever the tree is not the shape it was compiled for,
  // and then the recursive walk below runs as it always did.
  if (collisionBoundsRefresh != nullptr &&
      collisionBoundsRefresh->TryRefreshCollisionBounds(tree)) {
    return;
  }
  tree->UpdateBoundingBox(0);
}

void CSceneVehicle::SSurfaceHandler::Init(const CPlugSolid &solid,
                                          const SSurfaceId &surfaceId) {
  CSceneVehicle::SSurfaceHandler *handler = this;
  CPlugTree *tree = solid.GetPlugFromId(surfaceId.Id());
  handler->BindTree(tree);
  if (tree != nullptr) {
    handler->SetRestPose(tree->LocalIso());
  } else {
    GmIso4 identity;
    identity.SetIdentity();
    handler->SetRestPose(identity);
  }
  handler->ResetCurrentPose();
}

CSceneVehicle::SSurfaceHandler::SSurfaceHandler(void) {
  tree = nullptr;
  restIso.SetIdentity();
  currentIso.SetIdentity();
}

int CSceneVehicleCar::SSimulationWheel::GetSurfaceTreeRollingRadius(
    float &outRadius) const {
  CPlugTree *tree = surfaceHandler.Tree();
  CPlugSurface *surface = tree != nullptr ? tree->Surface() : nullptr;
  if (surface == nullptr || surface->Geometry() == nullptr) {
    return 0;
  }

  const GmSurf *gmSurf = surface->Geometry();
  if (gmSurf == nullptr) {
    return 0;
  }

  return gmSurf->GetRollingRadius(outRadius);
}

void CSceneVehicleCar::VehicleInitFromSolid(void) {
  if (!vehicleStruct.has_value()) {
    return;
  }
  CSceneVehicleStruct &definition = vehicleStruct->get();

  CPlugSolid *solid = HmsItem()->Solid();
  const std::vector<CSceneVehicleStruct::SSimulationWheel> &sourceWheels =
      definition.SimulationWheels();
  SetWheelCount(static_cast<u32>(sourceWheels.size()));

  for (u32 wheelIndex = 0; wheelIndex < WheelGetCount(); wheelIndex++) {
    CSceneVehicleCar::SSimulationWheel *wheel = &WheelAt(wheelIndex);
    const CSceneVehicleStruct::SSimulationWheel *source =
        &sourceWheels[wheelIndex];

    wheel->killsLateralSpeedOnContact = source->killsLateralSpeedOnContact;
    wheel->axle = source->axle;
    wheel->surfaceHandler.Init(*solid, source->surfaceId);

    CPlugTree *tree = wheel->surfaceHandler.Tree();
    if (tree != nullptr) {
      float rollingRadius = 0.0f;
      if (wheel->GetSurfaceTreeRollingRadius(rollingRadius)) {
        wheel->rollingRadius = rollingRadius;
      }

      GmIso4 thisToRoot;
      tree->GetThisToRootTransfo(thisToRoot, 1, nullptr);
      wheel->forceApplicationPoint = thisToRoot.Translation();
    } else {
      wheel->forceApplicationPoint.x = 0.0f;
      wheel->forceApplicationPoint.y = 0.0f;
      wheel->forceApplicationPoint.z = 0.0f;
    }
  }
}

void CSceneVehicleCar::SSimulationWheel::ApplySingleMaterialRefFromTuning(
    const CSceneVehicleCarTuning &tuning) {
  CPlugSurface *surface = surfaceHandler.Tree()->Surface();
  const EPlugSurfaceMaterialId material = tuning.contactResponse.singleMaterial;
  if (material < EPlugSurfaceMaterialId_Count) {
    surface->SetMaterialAt(0u, PlugSurfaceMaterials().Default(material));
  }
}

void CSceneVehicleCar::UpdateParamsFromTuning() {
  CPlugSolid *solid = HmsItem()->Solid();
  if (solid == nullptr) {
    return;
  }

  const u32 wheelCount = WheelGetCount();
  if (wheelCount == 0) {
    return;
  }

  CSceneVehicleCarTuning *tuning = Tunings()->ActiveTuning();
  GmVec3 minPoint = {0.0f, 0.0f, 0.0f};
  GmVec3 maxPoint = {0.0f, 0.0f, 0.0f};
  float bottomYSum = 0.0f;

  for (u32 wheelIndex = 0; wheelIndex < wheelCount; wheelIndex++) {
    CSceneVehicleCar::SSimulationWheel *wheel = &WheelAt(wheelIndex);
    CPlugTree *tree = wheel->surfaceHandler.Tree();
    CPlugSurface *surface = tree != nullptr ? tree->Surface() : nullptr;
    if (surface != nullptr && surface->MaterialCount() == 1u) {
      wheel->ApplySingleMaterialRefFromTuning(*tuning);
    }

    const GmVec3 point = wheel->surfaceHandler.RestPoint();
    const float bottomY = point.y - wheel->rollingRadius;
    if (wheelIndex == 0) {
      minPoint = point;
      maxPoint = point;
      bottomYSum = bottomY;
    } else {
      if (point.x < minPoint.x) {
        minPoint.x = point.x;
      }
      if (point.y < minPoint.y) {
        minPoint.y = point.y;
      }
      if (point.z < minPoint.z) {
        minPoint.z = point.z;
      }
      if (maxPoint.x < point.x) {
        maxPoint.x = point.x;
      }
      if (maxPoint.y < point.y) {
        maxPoint.y = point.y;
      }
      if (maxPoint.z < point.z) {
        maxPoint.z = point.z;
      }
      bottomYSum = bottomYSum + bottomY;
    }
  }

  GmBoxAligned wheelBounds;
  wheelBounds.SetMinMax(minPoint, maxPoint);
  gearedDrive.wheelLongitudinalSpan =
      (wheelBounds.halfExtents.z + wheelBounds.halfExtents.z);

  GmVec3 localCenterOfMass = wheelBounds.center;
  const float centerZOffset =
      (tuning->bodyAirResponse.solidCenterZHalfExtentScale *
       wheelBounds.halfExtents.z);
  localCenterOfMass.z = wheelBounds.center.z + centerZOffset;
  const float inverseWheelCount =
      1.0f / Binary32::FromUnsignedInteger(wheelCount);
  const float averageBottomY = inverseWheelCount * bottomYSum;
  localCenterOfMass.y =
      averageBottomY + tuning->bodyAirResponse.solidCenterYOffset;

  CPlugPhysicalObject *physical = &solid->Physical();
  physical->Parameters().mass = tuning->bodyAirResponse.solidPhysicalMass;
  physical->Parameters().vehicleContactFeedbackScale =
      tuning->bodyAirResponse.groundedSolidFeedback1;
  physical->Parameters().linearFluidFriction = 0.0f;
  physical->Parameters().physicalResponseCoefA = 0.0f;
  physical->Parameters().physicalResponseCoefB =
      tuning->bodyAirResponse.solidPhysicalResponseCoefB;
  physical->SetComPos(localCenterOfMass);
  physical->SetInertiaMatrixBox(tuning->bodyAirResponse.solidInertiaMass,
                                tuning->bodyAirResponse.solidInertiaBoxSize);

  gearedDrive.activeSteerSlowDownScale = tuning->steering.slowDownScale;
  engine.engineInputMax = tuning->gearedDrive.input.engineInputMaximum;
}

void CSceneVehicle::SSurfaceHandler::UpdateSurface(void) {
  if (tree != nullptr) {
    tree->SetLocation(currentIso);
  }
}

void CSceneVehicle::SSurfaceHandler::Reset(void) { currentIso = restIso; }

void CSceneVehicleCar::SSimulationWheel::CopyRestSurfaceIso() {
  surfaceHandler.ResetCurrentPose();
}

void CSceneVehicleCar::SSimulationWheel::OffsetCurrentSurfaceY(float yOffset) {
  surfaceHandler.OffsetCurrentY(yOffset);
}

void CSceneVehicleCar::WheelUpdateSpeedFromVehicleSpeed(
    CSceneVehicleCar::SSimulationWheel &wheel, float vehicleForwardSpeed,
    float dt) {
  if (wheel.realTimeState.contactPresent) {
    if (gearedDrive.wheelSpeedOverrideActive != 0 &&
        gearedDrive.wheelDriveSpeedInhibited == 0) {
      CSceneVehicleCarTuning *tuning = Tunings()->ActiveTuning();
      wheel.realTimeState.wheelAngularSpeed =
          tuning->gearedDrive.burnout.wheelAngularSpeedOverride;
      return;
    }

    wheel.realTimeState.wheelAngularSpeed =
        vehicleForwardSpeed / wheel.rollingRadius;
    return;
  }

  float targetAngularSpeed = 0.0f;
  float angularAcceleration = 0.0f;
  if (controls.lowSpeedGateB > ScalarEpsilon) {
    targetAngularSpeed = 1.0f - controls.lowSpeedGateB;
    if (targetAngularSpeed <= 0.0f) {
      targetAngularSpeed = 0.0f;
    } else if (targetAngularSpeed >= 1.0f) {
      targetAngularSpeed = 1.0f;
    }
    angularAcceleration = WheelAngularAccelNegative;
  } else if (controls.lowSpeedGateA > ScalarEpsilon &&
             gearedDrive.wheelDriveSpeedInhibited == 0 &&
             controls.forcedLowSpeedFriction == 0) {
    targetAngularSpeed =
        Binary32::FromDouble(static_cast<double>(controls.lowSpeedGateA) *
                             WheelLowSpeedGateASpeedScale);
    angularAcceleration = WheelAngularAccelPositive;
  } else {
    wheel.realTimeState.wheelAngularSpeed = Binary32::FromDouble(
        static_cast<double>(wheel.realTimeState.wheelAngularSpeed) *
        WheelNoContactDamping);
  }

  float absAngularAcceleration = std::fabs(angularAcceleration);
  if (!(absAngularAcceleration < ScalarEpsilon)) {
    float nextAngularSpeed =
        (angularAcceleration * dt + wheel.realTimeState.wheelAngularSpeed);
    wheel.realTimeState.wheelAngularSpeed = nextAngularSpeed;

    if (angularAcceleration > 0.0f && nextAngularSpeed > targetAngularSpeed) {
      wheel.realTimeState.wheelAngularSpeed = targetAngularSpeed;
    } else if (angularAcceleration < 0.0f &&
               nextAngularSpeed < targetAngularSpeed) {
      wheel.realTimeState.wheelAngularSpeed = targetAngularSpeed;
    }
  }
}

void CSceneVehicleCar::SSimulationWheel::SRealTimeState::Integrate(float dt) {
  CSceneVehicleCar::SSimulationWheel::SRealTimeState *wheel = this;

  float spinInput = wheel->wheelAngularSpeed * dt + wheel->wheelSpinAngle;
  wheel->wheelSpinAngle =
      GmFunc::Mod(spinInput, 0.0f, SceneVehicleMath::WheelSpinAnglePeriod);

  float normalLenSq =
      SceneVehicleMath::LengthSquaredYXZ(wheel->accumulatedContactNormal);
  if (normalLenSq > VectorEpsilonSquared) {
    float len = CIsqrt(normalLenSq);
    float invLen = 1.0f / len;
    wheel->accumulatedContactNormal.x =
        wheel->accumulatedContactNormal.x * invLen;
    wheel->accumulatedContactNormal.y =
        wheel->accumulatedContactNormal.y * invLen;
    wheel->accumulatedContactNormal.z =
        invLen * wheel->accumulatedContactNormal.z;

    GmVec3 directionOfView = {
        0.0f,
        -wheel->accumulatedContactNormal.z,
        wheel->accumulatedContactNormal.y,
    };
    wheel->contactFrame.SetUpVandDOV(wheel->accumulatedContactNormal,
                                     directionOfView);
  }

  float current = wheel->currentVisualSteerAngle;
  float target = wheel->targetVisualSteerAngle;
  if (target > current) {
    float next = current + dt;
    wheel->currentVisualSteerAngle = next;
    if (next > target) {
      wheel->currentVisualSteerAngle = target;
    }
  } else {
    float next = current - dt;
    wheel->currentVisualSteerAngle = next;
    if (target > next) {
      wheel->currentVisualSteerAngle = target;
    }
  }
}

void CSceneVehicleCar::WheelReset(CSceneVehicleCar::SSimulationWheel &wheel) {
  CSceneVehicleCarTuning *tuning = Tunings()->ActiveTuning();

  wheel.realTimeState.maxReplacementY = 0.0f;
  wheel.realTimeState.damperVelocity = 0.0f;
  wheel.realTimeState.damperAbsorb = tuning->suspension.wheelRestDamperAbsorb;

  const float negativeAbsorb = -wheel.realTimeState.damperAbsorb;
  wheel.surfaceHandler.Reset();
  wheel.surfaceHandler.OffsetCurrentY(negativeAbsorb);
  wheel.surfaceHandler.UpdateSurface();

  wheel.realTimeState.wheelAngularSpeed = 0.0f;
  wheel.realTimeState.wheelSpinAngle = 0.0f;
  wheel.realTimeState.contactPresent = false;
  wheel.realTimeState.contactFrame.SetIdentity();
  wheel.realTimeState.accumulatedContactNormal.x = 0.0f;
  wheel.realTimeState.accumulatedContactNormal.y = 0.0f;
  wheel.realTimeState.accumulatedContactNormal.z = 0.0f;
  wheel.realTimeState.currentVisualSteerAngle = 0.0f;
  wheel.realTimeState.targetVisualSteerAngle = 0.0f;
  wheel.realTimeState.slipping = false;
  wheel.realTimeState.contactNormalSampleCount = 0;
  wheel.realTimeState.rejectedNormalContact = false;
  wheel.realTimeState.rejectedNormalContactPoint.x = 0.0f;
  wheel.realTimeState.rejectedNormalContactPoint.y = 0.0f;
  wheel.realTimeState.rejectedNormalContactPoint.z = 0.0f;
  wheel.previousAsyncState.Reset();
  wheel.asyncState.Reset();
  wheel.previousPhysicsState.Reset();
  wheel.currentPhysicsState.Reset();
}

void CSceneVehicleCar::WheelIntegrate(CSceneVehicleCar::SSimulationWheel &wheel,
                                      float dt) {
  CSceneVehicleCarTuning *tuning = Tunings()->ActiveTuning();

  switch (tuning->wheelForceMode) {
  case CSceneVehicleCarWheelForceMode_DirectSpring: {
    wheel.realTimeState.damperAbsorb =
        wheel.realTimeState.damperAbsorb - wheel.realTimeState.maxReplacementY;
    wheel.realTimeState.maxReplacementY = 0.0f;

    wheel.CopyRestSurfaceIso();

    float acceleration =
        (tuning->suspension.wheelRestDamperAbsorb -
         wheel.realTimeState.damperAbsorb) *
            tuning->suspension.wheelSpringCoef -
        tuning->suspension.wheelDamperCoef * wheel.realTimeState.damperVelocity;
    wheel.realTimeState.damperVelocity =
        acceleration * dt + wheel.realTimeState.damperVelocity;
    wheel.realTimeState.damperAbsorb = dt * wheel.realTimeState.damperVelocity +
                                       wheel.realTimeState.damperAbsorb;

    wheel.OffsetCurrentSurfaceY(-wheel.realTimeState.damperAbsorb);
    break;
  }
  case CSceneVehicleCarWheelForceMode_FollowAbsorb:
  case CSceneVehicleCarWheelForceMode_FollowAbsorbWithImpulse: {
    float baseAbsorb =
        wheel.realTimeState.damperAbsorb - wheel.realTimeState.maxReplacementY;

    wheel.CopyRestSurfaceIso();
    // Follow the rest position before deriving the next damper velocity.
    float targetAbsorb = SceneVehicleMath::FollowAbsorbTarget(
        tuning->suspension.wheelRestDamperAbsorb, baseAbsorb, dt,
        tuning->suspension.wheelAbsorbFollowCoef);

    wheel.realTimeState.damperVelocity =
        (targetAbsorb - wheel.realTimeState.damperAbsorb) / dt;
    wheel.realTimeState.damperAbsorb = targetAbsorb;
    wheel.realTimeState.maxReplacementY = 0.0f;

    wheel.OffsetCurrentSurfaceY(-targetAbsorb);
    break;
  }
  default:
    break;
  }

  wheel.surfaceHandler.UpdateSurface();
  if (wheelSurfaceObserver != nullptr) {
    wheelSurfaceObserver->OnWheelSurfaceUpdated(*this, wheel);
  }
}
