// Collision impulse response and tangent-speed limiting.

#include "engine/physics/collision/hms_collision.h"
#include "engine/physics/collision/hms_collision_types.h"
#include "engine/physics/dynamics/hms_corpus.h"
#include "engine/physics/dynamics/hms_dyna.h"
#include "engine/physics/dynamics/hms_item.h"
#include "engine/physics/world/hms_zone.h"
#include "engine/scene/plug_solid.h"
#include "engine/physics/geometry/geometry_helpers.h"
#include "engine/core/binary32_math.h"
#include "engine/scene/scene_vehicle_math.h"
namespace {

constexpr float kImpulseSpeedEpsilon = 1.0e-5f;

}

u32 CHmsItem::DynamicTypeForSolveImpulse(void) const {
    return static_cast<u32>(properties.dynamicType);
}

int CHmsCorpus::UsesLinearImpulseOnlyForSolveImpulse(void) const {
    return item->GetProperties().forcePointDynamicCollisionResponse;
}

GmIso4 CHmsCorpus::LocationForSolveImpulse(void) const {
    if (dyna == 0) {
        return localIso;
    }
    GmIso4 location;
    location.Set(
            dyna->CurrentState().rotation,
            dyna->CurrentState().position);
    return location;
}

GmVec3 CHmsCorpus::CollisionPointVelocityForSolveImpulse(const GmVec3 &point) const {
    GmVec3 speed = GmVec3::Zero();
    if (dyna != 0) {
        dyna->GetSpeed(point, speed);
    }
    return speed;
}

static void SplitEqualPriorityReplacement(const SHmsPhysicalCollision *collision,
                                             float massA,
                                             float massB,
                                             GmVec3 *replacementA,
                                             GmVec3 *replacementB) {
    // replacementA = separation * (-massB / (massB + massA)) and
    // replacementB = separation * ( massA / (massB + massA)).
    const float invSum = (
            1.0f / (massB + massA));
    const float scaleA = (-massB * invSum);
    const float scaleB = (massA * invSum);
    replacementA->x = (collision->separation.x * scaleA);
    replacementA->y = (scaleA * collision->separation.y);
    replacementA->z = (scaleA * collision->separation.z);
    replacementB->x = (collision->separation.x * scaleB);
    replacementB->y = (scaleB * collision->separation.y);
    replacementB->z = (scaleB * collision->separation.z);
}

void CHmsCorpus::RunContactHandlerAndApplyReplacementForSolveImpulse(
        CHmsPhysicalContact *contact,
        GmVec3 *worldReplacement,
        GmVec3 *relativeSpeed,
        int sideAContact) {
    if (contact == 0) {
        return;
    }

    const GmIso4 location = LocationForSolveImpulse();
    const GmMat3 rotation = location.RotationMatrix();
    contact->accepted = 1;
    contact->replacement = *worldReplacement;
    contact->replacement.MultTranspose(rotation);

    GmVec3 localSpeed = sideAContact ? relativeSpeed->Negated() : *relativeSpeed;
    localSpeed.MultTranspose(rotation);
    contact->localRelativeSpeed = localSpeed;

    if (item != nullptr) {
        item->RunAbsorbContactCallback(contact);
    }

    if (sideAContact) {
        *worldReplacement =
            contact->replacement.LocalToWorldMat3SideAForSolveImpulse(
                    rotation);
    } else {
        *worldReplacement =
            contact->replacement.LocalToWorldMat3ForSolveImpulse(rotation);
    }

    contact->localRelativeSpeed.Mult(rotation);
    if (sideAContact) {
        *relativeSpeed = contact->localRelativeSpeed.Negated();
    }
}

static GmVec3 ClampTangentSpeedForSolveImpulse(GmVec3 speed,
                                  GmVec3 normal,
                                  float materialFrictionProduct) {
    // Cap tangent speed using the normal component and material friction.
    const float normalSpeed = (
        normal.z * speed.z + normal.x * speed.x + normal.y * speed.y);
    GmVec3 normalComponent = {
        (normal.x * normalSpeed),
        (normal.y * normalSpeed),
        (normalSpeed * normal.z),
    };
    GmVec3 tangent = {
        (speed.x - normalComponent.x),
        (speed.y - normalComponent.y),
        (speed.z - normalComponent.z),
    };

    const float normalLenSq =
            SceneVehicleMath::LengthSquaredYXZ(normalComponent);
    const float normalLen = (CIsqrt(normalLenSq));
    const float tangentLenSq =
            SceneVehicleMath::LengthSquaredYXZ(tangent);
    const float tangentLen = (CIsqrt(tangentLenSq));
    const float tangentLimit = (normalLen * materialFrictionProduct);
    if (tangentLen > tangentLimit) {
        const float tangentClampScale = (tangentLimit / tangentLen);
        tangent.x = (tangentClampScale * tangent.x);
        tangent.y = (tangent.y * tangentClampScale);
        tangent.z = (tangentClampScale * tangent.z);
    }

    GmVec3 out = {
        (tangent.x + normalComponent.x),
        (tangent.y + normalComponent.y),
        (tangent.z + normalComponent.z),
    };
    return out;
}

void SHmsPhysicalCollision::NegateContactGeometryForBodyB() {
    Neg();
}

float CHmsDyna::AngularEffectiveMassTermForSolveImpulse(
        const SHmsPhysicalCollision *collision,
        GmVec3 unitImpulse,
        int sideB) {
    GmVec3 center = WorldCenterOfMass(currentState);
    GmVec3 contactLeverArm = collision->contactPoint - center;
    GmVec3 angular = contactLeverArm.Cross(unitImpulse);
    angular.SetMult(angular, currentState.inverseInertiaWorld);
    GmVec3 angularAtPoint = angular.Cross(contactLeverArm);
    const float angularDot = sideB
            ? SceneVehicleMath::DotXYZ(angularAtPoint, unitImpulse)
            : SceneVehicleMath::DotYXZ(angularAtPoint, unitImpulse);
    return (angularDot + InverseMass());
}

void CHmsCorpus::ApplyImpulseForSolveImpulse(
        CHmsDyna *targetDyna,
        SHmsPhysicalCollision *collision,
        GmVec3 speedAtPoint,
        float restitution,
        float materialFrictionProduct,
        int sideB) {
    if (targetDyna == 0) {
        return;
    }

    GmVec3 normal = collision->impulseNormal;
    GmVec3 adjustedSpeed =
            ClampTangentSpeedForSolveImpulse(speedAtPoint, normal, materialFrictionProduct);
    GmVec3 negAdjustedSpeed = {
        (-adjustedSpeed.x),
        (-adjustedSpeed.y),
        (-adjustedSpeed.z),
    };
    float speedLenSq =
            SceneVehicleMath::LengthSquaredYXZ(negAdjustedSpeed);
    float speed = (CIsqrt(speedLenSq));

    if (!(speed > kImpulseSpeedEpsilon)) {
        return;
    }

    float invSpeed = (1.0f / speed);
    GmVec3 impulseDir = {
        (invSpeed * negAdjustedSpeed.x),
        (negAdjustedSpeed.y * invSpeed),
        (invSpeed * negAdjustedSpeed.z),
    };
    const int usesLinearImpulseOnly = UsesLinearImpulseOnlyForSolveImpulse();
    float denominator;
    if (targetDyna->UsesFullAngularDynamics() && !usesLinearImpulseOnly) {
        denominator = targetDyna->AngularEffectiveMassTermForSolveImpulse(
                collision, impulseDir, sideB);
    } else {
        denominator = targetDyna->InverseMass();
    }

    float restitutionScale = (speed * (restitution + 1.0f));
    float impulseMagnitude = (restitutionScale / denominator);
    GmVec3 impulse = {
        (impulseDir.x * impulseMagnitude),
        (impulseDir.y * impulseMagnitude),
        (impulseMagnitude * impulseDir.z),
    };

    if (usesLinearImpulseOnly) {
        targetDyna->AddImpulse(impulse);
    } else {
        targetDyna->AddImpulse(impulse, collision->contactPoint);
    }
}

void CHmsZoneDynamic::SolveImpulse(
        const SHmsPhysicalCollision &sourceCollision,
        CHmsPhysicalContact *contactA,
        CHmsPhysicalContact *contactB) {
    (void)this;
    SHmsPhysicalCollision collisionState = sourceCollision;
    SHmsPhysicalCollision *collision = &collisionState;

    CHmsCorpus *a = collision->CorpusA();
    CHmsCorpus *b = collision->CorpusB();
    CHmsDyna *dynaA = a->Dynamics();
    CHmsDyna *dynaB = b->Dynamics();

    CPlugSurfaceMaterialData matA =
            CPlugSurfaceMaterialData::ForMaterial(collision->materialA);
    CPlugSurfaceMaterialData matB =
            CPlugSurfaceMaterialData::ForMaterial(collision->materialB);
    float restitution = matA.GetRestitutionCoefWith(matB);

    GmVec3 replacementA = GmVec3::Zero();
    GmVec3 replacementB = GmVec3::Zero();
    u32 dynamicTypeA = a->Item()->DynamicTypeForSolveImpulse();
    u32 dynamicTypeB = b->Item()->DynamicTypeForSolveImpulse();
    if (dynamicTypeB < dynamicTypeA) {
        replacementA = collision->separation.Negated();
    } else if (dynamicTypeA < dynamicTypeB) {
        replacementB = collision->separation;
    } else {
        float massA = (a->Item()->Solid()->Physical().Parameters().mass);
        float massB = (b->Item()->Solid()->Physical().Parameters().mass);
        SplitEqualPriorityReplacement(
                collision,
                massA,
                massB,
                &replacementA,
                &replacementB);
    }

    GmVec3 speedA = a->CollisionPointVelocityForSolveImpulse(collision->contactPoint);
    GmVec3 speedB = b->CollisionPointVelocityForSolveImpulse(collision->contactPoint);
    GmVec3 relativeSpeed = speedB - speedA;

    a->RunContactHandlerAndApplyReplacementForSolveImpulse(
            contactA, &replacementA, &relativeSpeed, 1);
    if (dynaA != 0) {
        dynaA->AddReplacement(replacementA);
    }

    b->RunContactHandlerAndApplyReplacementForSolveImpulse(
            contactB, &replacementB, &relativeSpeed, 0);
    if (dynaB != 0) {
        dynaB->AddReplacement(replacementB);
    }

    if ((contactA != 0 && contactA->accepted == 0) ||
        (contactB != 0 && contactB->accepted == 0)) {
        return;
    }

    const float materialFrictionProduct =
            (matA.frictionCoef * matB.frictionCoef);

    a->ApplyImpulseForSolveImpulse(
            dynaA, collision, speedA, restitution, materialFrictionProduct, 0);

    collision->NegateContactGeometryForBodyB();
    b->ApplyImpulseForSolveImpulse(
            dynaB, collision, speedB, restitution, materialFrictionProduct, 1);
}
