#include "simulation/backends/optimized_cpu/optimized_cpu_vehicle_forces.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <new>
#include <type_traits>
#include <typeinfo>

#include "engine/core/binary32_math.h"
#include "engine/core/func_keys_real.h"
#include "engine/core/mw_cmd_buffer_core.h"
#include "engine/physics/dynamics/hms_corpus.h"
#include "engine/physics/dynamics/hms_force_field.h"
#include "engine/physics/geometry/physics_tolerances.h"
#include "engine/physics/world/hms_zone.h"
#include "engine/scene/scene_vehicle_car_internal.h"
#include "simulation/backends/optimized_cpu/optimized_cpu_compiled_tuning_curve.h"

#if defined(__GNUC__) || defined(__clang__)
#define FV_E019_ALWAYS_INLINE inline __attribute__((always_inline))
#define FV_E019_HOT_NOINLINE __attribute__((hot, noinline))
#else
#define FV_E019_ALWAYS_INLINE inline
#define FV_E019_HOT_NOINLINE
#endif

using forevervalidator::simulation::OptimizedCpuBinary32FromDoubleX86Sse2;
using forevervalidator::simulation::OptimizedCpuBinary32MathPath;
using forevervalidator::simulation::OptimizedCpuBinary32SqrtX86Sse2;
using forevervalidator::simulation::OptimizedCpuCompiledTuningCurve;
using namespace SceneVehicleCarDynamics;

namespace {

constexpr float VehicleCurveKeyEpsilon = 1.0e-5f;

template<bool NativeBinary32>
FV_E019_ALWAYS_INLINE float VehicleFromDouble(double value) noexcept {
    if constexpr (NativeBinary32) {
        return OptimizedCpuBinary32FromDoubleX86Sse2(value);
    }
    return Binary32::FromDouble(value);
}

template<bool NativeBinary32>
FV_E019_ALWAYS_INLINE float VehicleSqrt(float value) noexcept {
    if constexpr (NativeBinary32) {
        return OptimizedCpuBinary32SqrtX86Sse2(value);
    }
    return CIsqrt(value);
}

struct VehicleReducedAngle {
    double value;
    unsigned quadrant;
};

FV_E019_ALWAYS_INLINE VehicleReducedAngle VehicleReduceSmallAngle(
        float input) noexcept {
    constexpr double TwoOverPi =
            0.636619772367581343075535053490057448;
    constexpr double HalfPiHigh = 1.570796326734125614166;
    constexpr double HalfPiLow = 6.07710050650619224932e-11;
    const double value = static_cast<double>(input);
    const double scaled = value * TwoOverPi;
    const std::int64_t nearest = scaled >= 0.0
            ? static_cast<std::int64_t>(scaled + 0.5)
            : static_cast<std::int64_t>(scaled - 0.5);
    const double count = static_cast<double>(nearest);
    return {
            (value - count * HalfPiHigh) - count * HalfPiLow,
            static_cast<unsigned>(nearest) & 3u,
    };
}

FV_E019_ALWAYS_INLINE double VehicleSinPolynomial(double value) noexcept {
    const double square = value * value;
    double coefficient = -1.0 / 121645100408832000.0;
    coefficient = 1.0 / 355687428096000.0 + square * coefficient;
    coefficient = -1.0 / 1307674368000.0 + square * coefficient;
    coefficient = 1.0 / 6227020800.0 + square * coefficient;
    coefficient = -1.0 / 39916800.0 + square * coefficient;
    coefficient = 1.0 / 362880.0 + square * coefficient;
    coefficient = -1.0 / 5040.0 + square * coefficient;
    coefficient = 1.0 / 120.0 + square * coefficient;
    coefficient = -1.0 / 6.0 + square * coefficient;
    return value + value * square * coefficient;
}

FV_E019_ALWAYS_INLINE double VehicleCosPolynomial(double value) noexcept {
    const double square = value * value;
    double coefficient = -1.0 / 6402373705728000.0;
    coefficient = 1.0 / 20922789888000.0 + square * coefficient;
    coefficient = -1.0 / 87178291200.0 + square * coefficient;
    coefficient = 1.0 / 479001600.0 + square * coefficient;
    coefficient = -1.0 / 3628800.0 + square * coefficient;
    coefficient = 1.0 / 40320.0 + square * coefficient;
    coefficient = -1.0 / 720.0 + square * coefficient;
    coefficient = 1.0 / 24.0 + square * coefficient;
    coefficient = -0.5 + square * coefficient;
    return 1.0 + square * coefficient;
}

template<bool NativeBinary32>
FV_E019_ALWAYS_INLINE float VehicleSin(float input) noexcept {
    if constexpr (!NativeBinary32) {
        return CIsin(input);
    }
    if (!std::isfinite(input) || std::fabs(input) > 1000000.0f) {
        return CIsin(input);
    }
    const VehicleReducedAngle reduced = VehicleReduceSmallAngle(input);
    double result;
    switch (reduced.quadrant) {
    case 0u:
        result = VehicleSinPolynomial(reduced.value);
        break;
    case 1u:
        result = VehicleCosPolynomial(reduced.value);
        break;
    case 2u:
        result = -VehicleSinPolynomial(reduced.value);
        break;
    default:
        result = -VehicleCosPolynomial(reduced.value);
        break;
    }
    return VehicleFromDouble<true>(result);
}

template<bool NativeBinary32>
FV_E019_ALWAYS_INLINE float VehicleCos(float input) noexcept {
    if constexpr (!NativeBinary32) {
        return CIcos(input);
    }
    if (!std::isfinite(input) || std::fabs(input) > 1000000.0f) {
        return CIcos(input);
    }
    const VehicleReducedAngle reduced = VehicleReduceSmallAngle(input);
    double result;
    switch (reduced.quadrant) {
    case 0u:
        result = VehicleCosPolynomial(reduced.value);
        break;
    case 1u:
        result = -VehicleSinPolynomial(reduced.value);
        break;
    case 2u:
        result = -VehicleCosPolynomial(reduced.value);
        break;
    default:
        result = VehicleSinPolynomial(reduced.value);
        break;
    }
    return VehicleFromDouble<true>(result);
}

FV_E019_ALWAYS_INLINE double VehicleAtanUnit(double value) noexcept {
    constexpr double QuarterPi =
            0.785398163397448309615660845819875721;
    const bool aroundOne = value > 0.4142135623730950488;
    const double reduced = aroundOne
            ? (value - 1.0) / (value + 1.0)
            : value;
    const double square = reduced * reduced;
    double power = reduced;
    double result = reduced;
    for (unsigned termIndex = 1u; termIndex <= 24u; ++termIndex) {
        power *= -square;
        result += power / static_cast<double>(termIndex * 2u + 1u);
    }
    return aroundOne ? QuarterPi + result : result;
}

FV_E019_ALWAYS_INLINE double VehicleAtan2(double y, double x) noexcept {
    constexpr double Pi =
            3.14159265358979323846264338327950288;
    constexpr double HalfPi =
            1.57079632679489661923132169163975144;
    const bool yNegative = std::signbit(y);
    const bool xNegative = std::signbit(x);
    const double absY = std::fabs(y);
    const double absX = std::fabs(x);
    if (absY == 0.0) {
        if (xNegative) {
            return yNegative ? -Pi : Pi;
        }
        return y;
    }
    if (absX == 0.0) {
        return yNegative ? -HalfPi : HalfPi;
    }
    double angle = absY <= absX
            ? VehicleAtanUnit(absY / absX)
            : HalfPi - VehicleAtanUnit(absX / absY);
    if (xNegative) {
        angle = Pi - angle;
    }
    return yNegative ? -angle : angle;
}

template<bool NativeBinary32>
FV_E019_ALWAYS_INLINE float VehicleAsin(float value) noexcept {
    if constexpr (!NativeBinary32) {
        return CIasin(value);
    }
    if (std::isnan(value) || value < -1.0f || value > 1.0f) {
        return CIasin(value);
    }
    const float positiveFactor = 1.0f + value;
    const float negativeFactor = 1.0f - value;
    const float radical = VehicleSqrt<true>(
            positiveFactor * negativeFactor);
    return VehicleFromDouble<true>(
            VehicleAtan2(
                    static_cast<double>(value),
                    static_cast<double>(radical)));
}

FV_E019_ALWAYS_INLINE float VehicleLengthSquared(const GmVec3 &value) noexcept {
    const float xy = value.x * value.x + value.y * value.y;
    return xy + value.z * value.z;
}

template<bool NativeBinary32>
FV_E019_ALWAYS_INLINE GmVec3 VehicleNormalizeOr(
        const GmVec3 &value,
        const GmVec3 &shortVectorResult,
        float minimumLengthSquared) noexcept {
    const float lengthSquared = VehicleLengthSquared(value);
    if (!(lengthSquared > minimumLengthSquared)) {
        return shortVectorResult;
    }
    const float scale = 1.0f / VehicleSqrt<NativeBinary32>(lengthSquared);
    return {
            value.x * scale,
            value.y * scale,
            value.z * scale,
    };
}

}  // namespace

namespace forevervalidator::simulation {

struct OptimizedCpuCompiledModel6Tuning {
    const CSceneVehicleCarTuning *source = nullptr;
    OptimizedCpuCompiledTuningCurve maxSideFriction;
    OptimizedCpuCompiledTuningCurve damperModulation;
    OptimizedCpuCompiledTuningCurve steerDriveTorque;
    OptimizedCpuCompiledTuningCurve rolloverLateralFromSpeedRatio;
    OptimizedCpuCompiledTuningCurve slippingAcceleration;
    OptimizedCpuCompiledTuningCurve acceleration;
    OptimizedCpuCompiledTuningCurve rearGearAcceleration;
    OptimizedCpuCompiledTuningCurve steerSlowdown;
    OptimizedCpuCompiledTuningCurve burnoutRollover;

    bool TryBuild(const CSceneVehicleCarTuning &tuning) noexcept {
        if (!tuning.maxSideFrictionFromSpeedCurve.IsBound() ||
            !tuning.suspension.damperAbsorbModulationCurve.IsBound() ||
            !tuning.steering.driveTorqueFromSpeedCurve.IsBound() ||
            !tuning.gearedDrive.burnout.
                    rolloverLateralFromSpeedRatioCurve.IsBound() ||
            !tuning.slipResponse.slippingAccelFromSpeedCurve.IsBound() ||
            !tuning.slipResponse.accelFromSpeedCurve.IsBound() ||
            !tuning.gearedDrive.transmission.
                    rearGearAccelFromSpeedCurve.IsBound() ||
            !tuning.steering.slowDownFromSpeedCurve.IsBound() ||
            !tuning.gearedDrive.burnout.rolloverFromSpeedCurve.IsBound()) {
            return false;
        }
        OptimizedCpuCompiledModel6Tuning rebuilt;
        rebuilt.source = &tuning;
        if (!rebuilt.maxSideFriction.TryBuild(
                    tuning.maxSideFrictionFromSpeedCurve.Value()) ||
            !rebuilt.damperModulation.TryBuild(
                    tuning.suspension.damperAbsorbModulationCurve.Value()) ||
            !rebuilt.steerDriveTorque.TryBuild(
                    tuning.steering.driveTorqueFromSpeedCurve.Value()) ||
            !rebuilt.rolloverLateralFromSpeedRatio.TryBuild(
                    tuning.gearedDrive.burnout.
                            rolloverLateralFromSpeedRatioCurve.Value()) ||
            !rebuilt.slippingAcceleration.TryBuild(
                    tuning.slipResponse.slippingAccelFromSpeedCurve.Value()) ||
            !rebuilt.acceleration.TryBuild(
                    tuning.slipResponse.accelFromSpeedCurve.Value()) ||
            !rebuilt.rearGearAcceleration.TryBuild(
                    tuning.gearedDrive.transmission.
                            rearGearAccelFromSpeedCurve.Value()) ||
            !rebuilt.steerSlowdown.TryBuild(
                    tuning.steering.slowDownFromSpeedCurve.Value()) ||
            !rebuilt.burnoutRollover.TryBuild(
                    tuning.gearedDrive.burnout.rolloverFromSpeedCurve.Value())) {
            return false;
        }
        *this = std::move(rebuilt);
        return true;
    }

    bool IsFor(const CSceneVehicleCarTuning &tuning) const noexcept {
        return source == &tuning &&
               maxSideFriction.IsFor(
                       tuning.maxSideFrictionFromSpeedCurve.Value()) &&
               damperModulation.IsFor(
                       tuning.suspension.damperAbsorbModulationCurve.Value()) &&
               steerDriveTorque.IsFor(
                       tuning.steering.driveTorqueFromSpeedCurve.Value()) &&
               rolloverLateralFromSpeedRatio.IsFor(
                       tuning.gearedDrive.burnout.
                               rolloverLateralFromSpeedRatioCurve.Value()) &&
               slippingAcceleration.IsFor(
                       tuning.slipResponse.slippingAccelFromSpeedCurve.Value()) &&
               acceleration.IsStorageFor(
                       tuning.slipResponse.accelFromSpeedCurve.Value()) &&
               rearGearAcceleration.IsFor(
                       tuning.gearedDrive.transmission.
                               rearGearAccelFromSpeedCurve.Value()) &&
               steerSlowdown.IsFor(
                       tuning.steering.slowDownFromSpeedCurve.Value()) &&
               burnoutRollover.IsFor(
                       tuning.gearedDrive.burnout.rolloverFromSpeedCurve.Value());
    }
};

}  // namespace forevervalidator::simulation

using forevervalidator::simulation::OptimizedCpuCompiledModel6Tuning;

struct OptimizedCpuVehicleForceAccess {
    static CSceneVehicleCarTuning *ActiveTuning(
            CSceneVehicleCar &car) noexcept {
        return car.ActiveTuningOrNull();
    }

    static CSceneVehicleCarWheelSurfaceObserver *WheelSurfaceObserver(
            CSceneVehicleCar &car) noexcept {
        return car.wheelSurfaceObserver;
    }

    static bool HasRequiredModel3Configuration(
            const CSceneVehicleCarTuning &tuning) noexcept {
        return tuning.handlingModel == CSceneVehicleCarHandlingModel_Lateral &&
               tuning.maxSideFrictionFromSpeedCurve.IsBound() &&
               tuning.rolloverLateralFromSpeedCurve.IsBound() &&
               tuning.rolloverLateralCoefFromAngleCurve.IsBound() &&
               tuning.slipResponse.accelFromSpeedCurve.IsBound() &&
               tuning.steering.driveTorqueFromSpeedCurve.IsBound() &&
               tuning.steering.slowDownFromSpeedCurve.IsBound();
    }

    static bool HasRequiredModel6Configuration(
            const CSceneVehicleCar &car,
            const CSceneVehicleCarTuning &tuning) noexcept {
        return tuning.handlingModel ==
                       CSceneVehicleCarHandlingModel_GearedDrive &&
               car.wheels.size() == 4u &&
               tuning.maxSideFrictionFromSpeedCurve.IsBound() &&
               tuning.suspension.damperAbsorbModulationCurve.IsBound() &&
               tuning.steering.driveTorqueFromSpeedCurve.IsBound() &&
               tuning.gearedDrive.burnout.
                       rolloverLateralFromSpeedRatioCurve.IsBound() &&
               tuning.slipResponse.slippingAccelFromSpeedCurve.IsBound() &&
               tuning.slipResponse.accelFromSpeedCurve.IsBound() &&
               tuning.gearedDrive.transmission.
                       rearGearAccelFromSpeedCurve.IsBound() &&
               tuning.steering.slowDownFromSpeedCurve.IsBound() &&
               tuning.gearedDrive.burnout.rolloverFromSpeedCurve.IsBound();
    }

    static bool HasStableEligibility(
            CSceneVehicleCar &car,
            CHmsItem *item,
            CSceneVehicleCarTuning *tuning,
            OptimizedCpuBinary32MathPath mathPath) noexcept {
        if (mathPath != OptimizedCpuBinary32MathPath::X86Sse2 || item == nullptr ||
            tuning == nullptr || typeid(car) != typeid(CSceneVehicleCar) ||
            typeid(*tuning) != typeid(CSceneVehicleCarTuning) ||
            car.HmsItem() != item) {
            return false;
        }
        return HasRequiredModel3Configuration(*tuning) ||
               HasRequiredModel6Configuration(car, *tuning);
    }

    static bool CanUseModel6CommonPath(
            CSceneVehicleCar &car,
            CHmsDyna &dyna) noexcept {
        if (car.gearedDrive.burnoutPhase ==
            CSceneVehicleCarBurnoutPhase_CircularDrift) {
            return false;
        }
        if (car.controls.lowSpeedGateA > LowSpeedGateThreshold &&
            car.controls.lowSpeedGateB > LowSpeedGateThreshold) {
            return false;
        }
        GmVec3 linearSpeed;
        dyna.GetLinearSpeed(linearSpeed);
        return std::isfinite(linearSpeed.x) &&
               std::isfinite(linearSpeed.y) &&
               std::isfinite(linearSpeed.z);
    }

    static FV_E019_ALWAYS_INLINE GmVec3 LocalDirectionToWorld(
            const CHmsDyna &dyna,
            const GmVec3 &local) noexcept {
        const GmMat3 &rotation = dyna.CurrentState().rotation;
        return {
                rotation.Element(GmAxis::X, GmAxis::Y) * local.y +
                        rotation.Element(GmAxis::X, GmAxis::X) * local.x +
                        rotation.Element(GmAxis::X, GmAxis::Z) * local.z,
                rotation.Element(GmAxis::Y, GmAxis::X) * local.x +
                        rotation.Element(GmAxis::Y, GmAxis::Y) * local.y +
                        rotation.Element(GmAxis::Y, GmAxis::Z) * local.z,
                rotation.Element(GmAxis::Z, GmAxis::X) * local.x +
                        rotation.Element(GmAxis::Z, GmAxis::Y) * local.y +
                        rotation.Element(GmAxis::Z, GmAxis::Z) * local.z,
        };
    }

    static FV_E019_ALWAYS_INLINE void AddVehicleCentralForce(
            CSceneVehicleCar &car,
            CHmsDyna &dyna,
            const GmVec3 &localForce) noexcept {
        const GmVec3 force = LocalDirectionToWorld(dyna, localForce);
        CHmsDyna::CHmsStateDyna &state = dyna.CurrentState();
        state.force.x = state.force.x + force.x;
        state.force.y = force.y + state.force.y;
        state.force.z = force.z + state.force.z;
        car.forceAccumulators.force.x =
                car.forceAccumulators.force.x + localForce.x;
        car.forceAccumulators.force.y =
                localForce.y + car.forceAccumulators.force.y;
        car.forceAccumulators.force.z =
                localForce.z + car.forceAccumulators.force.z;
    }

    static FV_E019_ALWAYS_INLINE void AddVehicleTorque(
            CHmsDyna &dyna,
            const GmVec3 &localTorque) noexcept {
        const GmVec3 torque = LocalDirectionToWorld(dyna, localTorque);
        CHmsDyna::CHmsStateDyna &state = dyna.CurrentState();
        state.torque.x = state.torque.x + torque.x;
        state.torque.y = torque.y + state.torque.y;
        state.torque.z = torque.z + state.torque.z;
    }

    template<bool NativeBinary32>
    static FV_E019_ALWAYS_INLINE float LowerKeyBound(float key) noexcept {
        return VehicleFromDouble<NativeBinary32>(
                static_cast<double>(key) -
                static_cast<double>(VehicleCurveKeyEpsilon));
    }

    template<bool NativeBinary32>
    static FV_E019_ALWAYS_INLINE float UpperKeyBound(float key) noexcept {
        return VehicleFromDouble<NativeBinary32>(
                static_cast<double>(key) +
                static_cast<double>(VehicleCurveKeyEpsilon));
    }

    template<bool NativeBinary32>
    static FV_E019_ALWAYS_INLINE bool IsWithinKeyBounds(
            float x,
            float lower,
            float upper) noexcept {
        const float lowerBound = LowerKeyBound<NativeBinary32>(lower);
        const float upperBound = UpperKeyBound<NativeBinary32>(upper);
        return !std::isnan(x) && !std::isnan(lowerBound) &&
               !std::isnan(upperBound) && x >= lowerBound && x <= upperBound;
    }

    template<bool NativeBinary32>
    static FV_E019_ALWAYS_INLINE void GetBoundingIndices(
            const CFuncKeysReal &curve,
            float x,
            unsigned long &keyIndex,
            unsigned long &nextKeyIndex) noexcept {
        const unsigned long count =
                static_cast<unsigned long>(curve.keyPositions.size());
        if (count == 0u) {
            keyIndex = InvalidEngineIndex;
            nextKeyIndex = InvalidEngineIndex;
            return;
        }
        if (count == 1u ||
            x < LowerKeyBound<NativeBinary32>(curve.keyPositions.front())) {
            keyIndex = 0u;
            nextKeyIndex = 0u;
            return;
        }
        if (x > UpperKeyBound<NativeBinary32>(curve.keyPositions.back())) {
            keyIndex = count - 1u;
            nextKeyIndex = count - 1u;
            return;
        }

        unsigned long current = keyIndex < count ? keyIndex : 0ul;
        for (unsigned long scanned = 0ul; scanned <= count; ++scanned) {
            const unsigned long next =
                    current + 1ul < count ? current + 1ul : 0ul;
            if (IsWithinKeyBounds<NativeBinary32>(
                        x,
                        curve.keyPositions[current],
                        curve.keyPositions[next])) {
                keyIndex = current;
                nextKeyIndex = next;
                return;
            }
            current = next;
        }
        keyIndex = current;
        nextKeyIndex = current + 1u < count ? current + 1u : 0u;
    }

    template<bool NativeBinary32>
    static FV_E019_ALWAYS_INLINE bool ComputeBlendCoefficient(
            const CFuncKeysReal &curve,
            float x,
            unsigned long &keyIndex,
            unsigned long &nextKeyIndex,
            float &blendCoefficient) noexcept {
        GetBoundingIndices<NativeBinary32>(
                curve, x, keyIndex, nextKeyIndex);
        if (keyIndex == InvalidEngineIndex) {
            return false;
        }
        if (keyIndex == nextKeyIndex) {
            blendCoefficient = 0.0f;
            return true;
        }

        const float x0 = curve.keyPositions[keyIndex];
        const float span = curve.keyPositions[nextKeyIndex] - x0;
        blendCoefficient = std::fabs(span) >= VehicleCurveKeyEpsilon
                ? (x - x0) / span
                : 0.0f;
        return true;
    }

    template<bool NativeBinary32>
    static FV_E019_ALWAYS_INLINE float EvaluateCurve(
            const CFuncKeysReal &curve,
            float x) noexcept {
        unsigned long keyIndex = 0ul;
        unsigned long nextKeyIndex = keyIndex + 1ul;
        float blendCoefficient = 0.0f;
        if (!ComputeBlendCoefficient<NativeBinary32>(
                    curve,
                    x,
                    keyIndex,
                    nextKeyIndex,
                    blendCoefficient)) {
            return 0.0f;
        }

        if (curve.interpolationMode == CFuncKeysReal::Constant) {
            return curve.values[keyIndex];
        }
        const float value0 = curve.values[keyIndex];
        const float value1 = curve.values[nextKeyIndex];
        return (1.0f - blendCoefficient) * value0 +
               blendCoefficient * value1;
    }

    template<bool NativeBinary32>
    static FV_E019_ALWAYS_INLINE float EvaluateSpeedCurve(
            const CSceneVehicleTuningCurve &curve,
            float speed) noexcept {
        const float kilometersPerHour = VehicleFromDouble<NativeBinary32>(
                static_cast<double>(speed) * static_cast<double>(3.6f));
        return EvaluateCurve<NativeBinary32>(curve.Value(), kilometersPerHour);
    }

    template<bool NativeBinary32>
    static FV_E019_ALWAYS_INLINE float EvaluateLinearSpeedCurve(
            const CSceneVehicleTuningCurve &curve,
            float speed) noexcept {
        CFuncKeysReal &values = curve.Value();
        values.interpolationMode = CFuncKeysReal::Constant;
        return EvaluateSpeedCurve<NativeBinary32>(curve, speed);
    }

    template<bool NativeBinary32>
    static FV_E019_ALWAYS_INLINE void IntegrateWheelState(
            CSceneVehicleCar::SSimulationWheel::SRealTimeState &wheel,
            float dt) {
        if constexpr (!NativeBinary32) {
            wheel.Integrate(dt);
            return;
        }

        const float spinInput =
                wheel.wheelAngularSpeed * dt + wheel.wheelSpinAngle;
        wheel.wheelSpinAngle = GmFunc::Mod(
                spinInput,
                0.0f,
                SceneVehicleMath::WheelSpinAnglePeriod);

        const float normalLenSq =
                (wheel.accumulatedContactNormal.y *
                         wheel.accumulatedContactNormal.y +
                 wheel.accumulatedContactNormal.x *
                         wheel.accumulatedContactNormal.x) +
                wheel.accumulatedContactNormal.z *
                        wheel.accumulatedContactNormal.z;
        if (normalLenSq > VectorEpsilonSquared) {
            const float len = VehicleSqrt<true>(normalLenSq);
            const float invLen = 1.0f / len;
            wheel.accumulatedContactNormal.x =
                    wheel.accumulatedContactNormal.x * invLen;
            wheel.accumulatedContactNormal.y =
                    wheel.accumulatedContactNormal.y * invLen;
            wheel.accumulatedContactNormal.z =
                    invLen * wheel.accumulatedContactNormal.z;

            const GmVec3 directionOfView = {
                    0.0f,
                    -wheel.accumulatedContactNormal.z,
                    wheel.accumulatedContactNormal.y,
            };
            const GmVec3 sideSeed = GmMath::Cross(
                    wheel.accumulatedContactNormal,
                    directionOfView);
            const GmVec3 side = VehicleNormalizeOr<true>(
                    sideSeed,
                    sideSeed,
                    PhysicsTolerance::SurfaceDirectionLengthSquared);
            const GmVec3 normalizedUp = VehicleNormalizeOr<true>(
                    wheel.accumulatedContactNormal,
                    wheel.accumulatedContactNormal,
                    PhysicsTolerance::SurfaceDirectionLengthSquared);
            wheel.contactFrame.basisX = side;
            wheel.contactFrame.basisY = normalizedUp;
            wheel.contactFrame.basisZ = GmMath::Cross(side, normalizedUp);
        }

        const float current = wheel.currentVisualSteerAngle;
        const float target = wheel.targetVisualSteerAngle;
        if (target > current) {
            const float next = current + dt;
            wheel.currentVisualSteerAngle = next;
            if (next > target) {
                wheel.currentVisualSteerAngle = target;
            }
        } else {
            const float next = current - dt;
            wheel.currentVisualSteerAngle = next;
            if (target > next) {
                wheel.currentVisualSteerAngle = target;
            }
        }
    }

    template<bool NativeBinary32>
    static FV_E019_ALWAYS_INLINE void UpdateWheelSpeed(
            CSceneVehicleCar &car,
            CSceneVehicleCar::SSimulationWheel &wheel,
            float vehicleForwardSpeed,
            float dt) {
        if constexpr (!NativeBinary32) {
            car.WheelUpdateSpeedFromVehicleSpeed(
                    wheel, vehicleForwardSpeed, dt);
            return;
        }

        if (wheel.realTimeState.contactPresent) {
            if (car.gearedDrive.wheelSpeedOverrideActive != 0 &&
                car.gearedDrive.wheelDriveSpeedInhibited == 0) {
                CSceneVehicleCarTuning *tuning =
                        car.Tunings()->ActiveTuning();
                wheel.realTimeState.wheelAngularSpeed =
                        tuning->gearedDrive.burnout.
                                wheelAngularSpeedOverride;
                return;
            }
            wheel.realTimeState.wheelAngularSpeed =
                    vehicleForwardSpeed / wheel.rollingRadius;
            return;
        }

        float targetAngularSpeed = 0.0f;
        float angularAcceleration = 0.0f;
        if (car.controls.lowSpeedGateB > ScalarEpsilon) {
            targetAngularSpeed = 1.0f - car.controls.lowSpeedGateB;
            if (targetAngularSpeed <= 0.0f) {
                targetAngularSpeed = 0.0f;
            } else if (targetAngularSpeed >= 1.0f) {
                targetAngularSpeed = 1.0f;
            }
            angularAcceleration = WheelAngularAccelNegative;
        } else if (car.controls.lowSpeedGateA > ScalarEpsilon &&
                   car.gearedDrive.wheelDriveSpeedInhibited == 0 &&
                   car.controls.forcedLowSpeedFriction == 0) {
            targetAngularSpeed = VehicleFromDouble<true>(
                    static_cast<double>(car.controls.lowSpeedGateA) *
                    WheelLowSpeedGateASpeedScale);
            angularAcceleration = WheelAngularAccelPositive;
        } else {
            wheel.realTimeState.wheelAngularSpeed =
                    VehicleFromDouble<true>(
                            static_cast<double>(
                                    wheel.realTimeState.wheelAngularSpeed) *
                            WheelNoContactDamping);
        }

        const float absAngularAcceleration =
                std::fabs(angularAcceleration);
        if (!(absAngularAcceleration < ScalarEpsilon)) {
            const float nextAngularSpeed =
                    angularAcceleration * dt +
                    wheel.realTimeState.wheelAngularSpeed;
            wheel.realTimeState.wheelAngularSpeed = nextAngularSpeed;
            if (angularAcceleration > 0.0f &&
                nextAngularSpeed > targetAngularSpeed) {
                wheel.realTimeState.wheelAngularSpeed = targetAngularSpeed;
            } else if (angularAcceleration < 0.0f &&
                       nextAngularSpeed < targetAngularSpeed) {
                wheel.realTimeState.wheelAngularSpeed = targetAngularSpeed;
            }
        }
    }

    struct WheelVisualSteerTrigCache {
        float sine = 0.0f;
        float cosine = 1.0f;
        bool ready = false;
    };

    template<bool NativeBinary32>
    static FV_E019_ALWAYS_INLINE void UpdateWheelVisualState(
            CSceneVehicleCar &car,
            CSceneVehicleCar::SSimulationWheel &wheel,
            CSceneVehicleCarTuning *tuning,
            float vehicleForwardSpeed,
            float dt,
            float visualSpeedDenominator,
            WheelVisualSteerTrigCache &steerTrigCache) {
        if constexpr (!NativeBinary32) {
            car.UpdateWheelVisualState(
                    wheel,
                    tuning,
                    vehicleForwardSpeed,
                    dt,
                    visualSpeedDenominator);
            return;
        }

        wheel.realTimeState.visualRotation.Set(
                wheel.surfaceHandler.RestRotation());
        float wheelVisualSteerAngle = 0.0f;
        if (IsFrontVehicleWheel(wheel.axle)) {
            float yaw = 0.0f;
            if (!(visualSpeedDenominator < 1.0e-5f)) {
                yaw = -car.controls.currentSteering /
                      visualSpeedDenominator;
            }
            if constexpr (NativeBinary32) {
                if (!steerTrigCache.ready) {
                    steerTrigCache.sine = VehicleSin<true>(yaw);
                    steerTrigCache.cosine = VehicleCos<true>(yaw);
                    steerTrigCache.ready = true;
                }
                const float sine = steerTrigCache.sine;
                const float cosine = steerTrigCache.cosine;
                const GmVec3 oldX =
                        wheel.realTimeState.visualRotation.Row(GmAxis::X);
                const GmVec3 oldZ =
                        wheel.realTimeState.visualRotation.Row(GmAxis::Z);
                wheel.realTimeState.visualRotation.SetRow(
                        GmAxis::X,
                        GmMath::Add(
                                GmMath::Scale(oldX, cosine),
                                GmMath::Scale(oldZ, sine)));
                wheel.realTimeState.visualRotation.SetRow(
                        GmAxis::Z,
                        GmMath::Add(
                                GmMath::Scale(oldX, -sine),
                                GmMath::Scale(oldZ, cosine)));
            } else {
                wheel.realTimeState.visualRotation.RotateY(yaw);
            }

            float maxSteerDegrees = WheelVisualDefaultMaxSteerDegrees;
            if (tuning->visual.wheelSteerAngleFromSpeedCurve.IsBound()) {
                const float curveInput =
                        std::fabs(vehicleForwardSpeed) *
                        SceneVehicleMath::SpeedKilometersPerHourScale;
                maxSteerDegrees = EvaluateCurve<true>(
                        tuning->visual.wheelSteerAngleFromSpeedCurve.Value(),
                        curveInput);
            }
            const float maxSteerRadians =
                    (maxSteerDegrees * SceneVehicleMath::Pi) /
                    DegreesToRadiansDivisor;
            wheelVisualSteerAngle =
                    -car.controls.currentSteering * maxSteerRadians;
        }

        wheel.realTimeState.targetVisualSteerAngle =
                wheelVisualSteerAngle;
        UpdateWheelSpeed<true>(car, wheel, vehicleForwardSpeed, dt);
        IntegrateWheelState<true>(wheel.realTimeState, dt);
    }

    template<bool NativeBinary32>
    static FV_E019_ALWAYS_INLINE void IntegrateVehicle(
            CSceneVehicleCar &car,
            float dt,
            GmVec3 &linearSpeed,
            forevervalidator::simulation::
                    OptimizedCpuVehicleCollisionBoundsPlan
                            *collisionBoundsPlan) {
        if constexpr (!NativeBinary32) {
            car.IntegrateVehicle(dt);
            car.HmsItem()->GetLinearSpeed(linearSpeed);
            return;
        }

        car.HmsItem()->GetLinearSpeed(linearSpeed);
        const float vehicleForwardSpeed = linearSpeed.z;
        CSceneVehicleCarTuning *tuning =
                car.Tunings()->ActiveTuning();

        if (car.integration.updateWheelVisuals) {
            const float visualSpeedDenominator =
                    std::fabs(vehicleForwardSpeed) *
                            tuning->visual.wheelSpeedScale +
                    tuning->visual.wheelSpeedBase;
            WheelVisualSteerTrigCache steerTrigCache;
            const u32 wheelCount = car.WheelGetCount();
            for (u32 wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex) {
                CSceneVehicleCar::SSimulationWheel &wheel =
                        car.WheelAt(wheelIndex);
                UpdateWheelVisualState<true>(
                        car,
                        wheel,
                        tuning,
                        vehicleForwardSpeed,
                        dt,
                        visualSpeedDenominator,
                        steerTrigCache);
            }
        }

        if (car.integration.integrateWheels) {
            const u32 wheelCount = car.WheelGetCount();
            for (u32 wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex) {
                car.WheelIntegrate(car.WheelAt(wheelIndex), dt);
            }
        }

        if (car.integration.integrateEngine) {
            if (car.controls.forcedLowSpeedFriction == 0) {
                const float input = !car.engine.useLowSpeedGateB
                        ? car.controls.lowSpeedGateA
                        : car.controls.lowSpeedGateB;
                car.EngineIntegrate(input, dt);
            } else {
                car.engine.engineInputMemory = 0.0f;
            }
        }

        car.UpdateCurrentSteering(tuning, dt);
        if (collisionBoundsPlan == nullptr) {
            car.RefreshCollisionTree();
        } else {
            collisionBoundsPlan->RefreshRuntimeCertified();
        }
    }

    template<bool NativeBinary32>
    static FV_E019_ALWAYS_INLINE void GetSlopeAdherence(
            CSceneVehicleCar &car,
            const GmVec3 &normal,
            float &outFirst,
            float &outSecond) {
        if constexpr (!NativeBinary32) {
            car.GetSlopeAdherence(normal, outFirst, outSecond);
            return;
        }

        const float lenSq =
                (normal.y * normal.y + normal.x * normal.x) +
                normal.z * normal.z;
        if (!(1.0e-10f < lenSq)) {
            return;
        }
        CSceneVehicleCarTuning *tuning =
                car.Tunings()->ActiveTuning();
        const float len = VehicleSqrt<true>(lenSq);
        float slope = VehicleFromDouble<true>(
                static_cast<double>(normal.y) /
                static_cast<double>(len));
        slope = std::fabs(slope);

        outFirst = slope;
        const auto slopeAdherenceBlend = [](float value,
                                            float minimum,
                                            float maximum) {
            if (!(minimum <= value)) {
                return 0.0f;
            }
            if (!(maximum >= value)) {
                return 1.0f;
            }
            const float angle =
                    ((value - minimum) / (maximum - minimum)) *
                    static_cast<double>(SceneVehicleMath::Pi) * 0.5;
            return 1.0f - VehicleCos<true>(angle);
        };
        outFirst = slopeAdherenceBlend(
                slope,
                tuning->bodyAirResponse.slopeAdherence1Min,
                tuning->bodyAirResponse.slopeAdherence1Max);
        outSecond = slope;
        outSecond = slopeAdherenceBlend(
                slope,
                tuning->bodyAirResponse.slopeAdherence2Min,
                tuning->bodyAirResponse.slopeAdherence2Max);
    }

    template<bool NativeBinary32>
    static FV_E019_ALWAYS_INLINE float ComputeVisualSteerYaw(
            CSceneVehicleCar &car,
            CSceneVehicleCarTuning *tuning,
            const GmVec3 &linearSpeed) {
        if constexpr (!NativeBinary32) {
            return car.ComputeVisualSteerYaw(tuning, linearSpeed);
        }
        const float absSpeedZ = std::fabs(linearSpeed.z);
        const float denominator =
                absSpeedZ * tuning->visual.wheelSpeedScale +
                tuning->visual.wheelSpeedBase;
        float asinValue = 0.0f;
        if (!(denominator < ScalarEpsilon)) {
            constexpr float SafeTrigInteriorLimit = 1.0f - 1.0e-6f;
            const float asinInput = 1.0f / denominator;
            if (asinInput < -SafeTrigInteriorLimit ||
                SafeTrigInteriorLimit < asinInput) {
                asinValue = GmFunc::AsinSafe(asinInput);
            } else {
                asinValue = VehicleAsin<true>(asinInput);
            }
        }
        return -car.controls.currentSteering * asinValue;
    }

    template<bool NativeBinary32>
    static FV_E019_ALWAYS_INLINE void UpdateFeedbackTail(
            CSceneVehicleCar &car,
            CSceneVehicleCarTuning *tuning,
            float dt,
            const GmVec3 &linearSpeed,
            const GmVec3 &savedForce,
            const GmVec3 &savedImpulse,
            float surfaceFeedback) {
        if constexpr (!NativeBinary32) {
            car.UpdateFeedbackTail(
                    tuning,
                    dt,
                    linearSpeed,
                    savedForce,
                    savedImpulse,
                    surfaceFeedback);
            return;
        }

        car.HmsItem()->GetForce(car.gearedDrive.scaledCurrentForce);
        const float forceScale = 1.0f / tuning->feedback.forceDivisor;
        car.gearedDrive.scaledCurrentForce.x =
                forceScale * car.gearedDrive.scaledCurrentForce.x;
        car.gearedDrive.scaledCurrentForce.y =
                forceScale * car.gearedDrive.scaledCurrentForce.y;
        car.gearedDrive.scaledCurrentForce.z =
                forceScale * car.gearedDrive.scaledCurrentForce.z;

        const float surfaceCurveValue = EvaluateCurve<true>(
                tuning->feedback.surfaceCurve.Value(), surfaceFeedback);
        const float feedbackRate =
                surfaceCurveValue + tuning->feedback.surfaceBaseRate;
        const float unclampedSurfaceFeedback =
                feedbackRate * dt + car.feedback.surfaceAccumulator;
        car.feedback.surfaceAccumulator = unclampedSurfaceFeedback;
        car.feedback.surfaceAccumulator =
                ClampZeroOne(unclampedSurfaceFeedback);

        car.UpdateFeedbackSpringAxis(
                car.feedback.sideSpring,
                dt,
                savedForce.x,
                savedImpulse.x,
                0);
        car.UpdateFeedbackSpringAxis(
                car.feedback.forwardSpring,
                dt,
                savedForce.z,
                savedImpulse.z,
                1);

        const float contactRampDirection =
                car.IsAllWheelGroundContactId(FeedbackRampContactId) != 0
                ? 1.0f
                : -1.0f;
        const float curveInput = std::fabs(
                linearSpeed.z *
                SceneVehicleMath::SpeedKilometersPerHourScale);
        const float ramp0 = EvaluateCurve<true>(
                car.FeedbackRamp0Curve(), curveInput);
        car.feedback.ramp0 = ClampZeroOne(
                car.feedback.ramp0 +
                ramp0 * dt * contactRampDirection);
        const float ramp1 = EvaluateCurve<true>(
                car.FeedbackRamp1Curve(), curveInput);
        car.feedback.ramp1 = ClampZeroOne(
                car.feedback.ramp1 +
                ramp1 * dt * contactRampDirection);
    }

    template<bool NativeBinary32>
    static FV_E019_ALWAYS_INLINE float MaxSideFrictionFromSpeed(
            const CSceneVehicleCarTuning &tuning,
            float speed) noexcept {
        return EvaluateSpeedCurve<NativeBinary32>(
                tuning.maxSideFrictionFromSpeedCurve, speed);
    }

    template<bool NativeBinary32>
    static FV_E019_ALWAYS_INLINE float AccelFromSpeed(
            const CSceneVehicleCarTuning &tuning,
            float speed) noexcept {
        return EvaluateLinearSpeedCurve<NativeBinary32>(
                tuning.slipResponse.accelFromSpeedCurve, speed);
    }

    template<bool NativeBinary32>
    static FV_E019_ALWAYS_INLINE float RolloverLateralFromSpeed(
            const CSceneVehicleCarTuning &tuning,
            float speed) noexcept {
        return EvaluateSpeedCurve<NativeBinary32>(
                tuning.rolloverLateralFromSpeedCurve, speed);
    }

    template<bool NativeBinary32>
    static FV_E019_ALWAYS_INLINE float RolloverLateralCoefficientFromAngle(
            const CSceneVehicleCarTuning &tuning,
            float angle) noexcept {
        return EvaluateCurve<NativeBinary32>(
                tuning.rolloverLateralCoefFromAngleCurve.Value(), angle);
    }

    template<bool NativeBinary32>
    static FV_E019_ALWAYS_INLINE float SteerDriveTorqueFromSpeed(
            const CSceneVehicleCarTuning &tuning,
            float speed) noexcept {
        return EvaluateSpeedCurve<NativeBinary32>(
                tuning.steering.driveTorqueFromSpeedCurve, speed);
    }

    template<bool NativeBinary32>
    static FV_E019_ALWAYS_INLINE float SteerSlowDownFromSpeed(
            const CSceneVehicleCarTuning &tuning,
            float speed) noexcept {
        return EvaluateLinearSpeedCurve<NativeBinary32>(
                tuning.steering.slowDownFromSpeedCurve, speed);
    }

    template<bool NativeBinary32>
    static FV_E019_ALWAYS_INLINE void ApplyContactForces(
            CSceneVehicleCar &car,
            CHmsDyna &dyna,
            const CSceneVehicleCar::LegacyForceRequest &request,
            const CSceneVehicleCarTuning &tuning) {
        const u32 wheelCount = static_cast<u32>(car.wheels.size());
        for (u32 wheelIndex = 0; wheelIndex < wheelCount; wheelIndex++) {
            CSceneVehicleCar::SSimulationWheel *wheel =
                    &car.wheels[wheelIndex];
            car.WheelAddForceToVehicle(*wheel, request.currentForce);

            CSceneVehicleMaterial *material = car.GetWheelMaterial(*wheel);
            if (!wheel->realTimeState.contactPresent ||
                !(tuning.gearedDrive.lateralForceScale > 0.0f) ||
                tuning.handlingModel != CSceneVehicleCarHandlingModel_Lateral) {
                continue;
            }

            float slipGrip = wheel->realTimeState.slipping
                    ? tuning.gearedDrive.slippingSideFrictionScale
                    : 1.0f;
            float maxSide = material->blendableVals.w *
                            request.slopeAdherenceA *
                            MaxSideFrictionFromSpeed<NativeBinary32>(
                                    tuning, request.linearSpeed.z) *
                            slipGrip;
            GmVec3 lateral = {
                    wheel->realTimeState.accumulatedContactNormal.y,
                    -wheel->realTimeState.accumulatedContactNormal.x,
                    0.0f,
            };
            lateral = VehicleNormalizeOr<NativeBinary32>(
                    lateral,
                    GmVec3{1.0f, 0.0f, 0.0f},
                    VectorEpsilonSquared);
            if (IsFrontVehicleWheel(wheel->axle)) {
                float visualSteerYawCos =
                        VehicleCos<NativeBinary32>(request.visualSteerYaw);
                float negVisualSteerYawSin =
                        -VehicleSin<NativeBinary32>(request.visualSteerYaw);
                lateral = GmVec3{
                        visualSteerYawCos * lateral.x,
                        visualSteerYawCos * lateral.y,
                        negVisualSteerYawSin +
                                visualSteerYawCos * lateral.z,
                };
            }

            float sideForce = -tuning.gearedDrive.lateralForceScale * 0.5f *
                              SceneVehicleMath::Dot(
                                      request.linearSpeed, lateral);
            float sideForceAbs = std::fabs(sideForce);
            if (!(maxSide < sideForceAbs)) {
                wheel->realTimeState.slipping = false;
            } else {
                float capped = SignNonNegative(sideForce) * maxSide;
                sideForce =
                        (1.0f -
                         tuning.gearedDrive.sideFrictionSlipBlend) *
                                capped +
                        tuning.gearedDrive.sideFrictionSlipBlend * sideForce;
                wheel->realTimeState.slipping = true;
            }
            if (wheel->realTimeState.slipping) {
                request.outSlipFlag = 1;
            }

            GmVec3 lateralForce =
                    SceneVehicleMath::Scale(lateral, sideForce);
            AddVehicleCentralForce(car, dyna, lateralForce);

            float rollover =
                    -RolloverLateralFromSpeed<NativeBinary32>(
                            tuning, request.linearSpeed.z) *
                    request.slopeAdherenceA *
                    RolloverLateralCoefficientFromAngle<NativeBinary32>(
                            tuning, std::fabs(lateral.y));
            GmVec3 rolloverTorque = {
                    lateralForce.z * rollover,
                    0.0f,
                    -rollover * lateralForce.x,
            };
            AddVehicleTorque(dyna, rolloverTorque);
        }
    }

    template<bool NativeBinary32>
    static FV_E019_ALWAYS_INLINE void ApplySteeringTorques(
            CSceneVehicleCar &car,
            CHmsDyna &dyna,
            const CSceneVehicleCar::LegacyForceRequest &request,
            const CSceneVehicleCarTuning &tuning,
            float speedMagnitude) {
        const u32 wheelCount = static_cast<u32>(car.wheels.size());
        for (u32 wheelIndex = 0; wheelIndex < wheelCount; wheelIndex++) {
            CSceneVehicleCar::SSimulationWheel *wheel =
                    &car.wheels[wheelIndex];
            float halfTrack =
                    (IsFrontVehicleWheel(wheel->axle)
                                     ? car.gearedDrive.wheelLongitudinalSpan
                                     : -car.gearedDrive.wheelLongitudinalSpan) *
                    0.5f;
            float steerRamp = 1.0f;
            if (!(tuning.steering.assistFullSpeed < speedMagnitude)) {
                steerRamp = VehicleSin<NativeBinary32>(static_cast<float>(
                        (speedMagnitude /
                         tuning.steering.assistFullSpeed) *
                        SceneVehicleMath::HalfPi));
            }
            float maxSide =
                    MaxSideFrictionFromSpeed<NativeBinary32>(
                            tuning, request.linearSpeed.z) *
                    request.materialVals.w;
            float wheelSideSpeed = request.linearSpeed.x +
                                   request.angularSpeed.y * halfTrack;
            float sideForce = -tuning.gearedDrive.lateralForceScale * 0.5f *
                              wheelSideSpeed;
            if (maxSide < std::fabs(sideForce)) {
                float blended =
                        (1.0f -
                         tuning.gearedDrive.driveSideFrictionSlipBlend) *
                                maxSide +
                        tuning.gearedDrive.driveSideFrictionSlipBlend *
                                std::fabs(sideForce);
                sideForce = SignNonNegative(sideForce) * blended;
            }

            float sideTorque =
                    tuning.gearedDrive.sideForceToDriveTorqueScale * sideForce;
            if (IsFrontVehicleWheel(wheel->axle)) {
                float reverseSign =
                        !car.engine.useLowSpeedGateB ? 1.0f : -1.0f;
                float slipScale = wheel->realTimeState.slipping
                        ? tuning.gearedDrive.slippingSteerTorqueScale
                        : 1.0f;
                float steerTorque =
                        SteerDriveTorqueFromSpeed<NativeBinary32>(
                                tuning, request.linearSpeed.z);
                const float steerAssist =
                        reverseSign * steerRamp *
                        car.controls.currentSteering * steerTorque * slipScale;
                sideTorque = sideTorque - steerAssist;
            }
            AddVehicleTorque(
                    dyna, {0.0f, sideTorque * halfTrack, 0.0f});
        }
    }

    template<bool NativeBinary32>
    static FV_E019_ALWAYS_INLINE void ApplyDriveForces(
            CSceneVehicleCar &car,
            CHmsDyna &dyna,
            const CSceneVehicleCar::LegacyForceRequest &request,
            const CSceneVehicleCarTuning &tuning) {
        float accelBase =
                AccelFromSpeed<NativeBinary32>(tuning, request.linearSpeed.z);
        float sideLimit =
                MaxSideFrictionFromSpeed<NativeBinary32>(
                        tuning, request.linearSpeed.z) *
                request.materialVals.w;
        float sideSlowdownInput = std::fabs(
                tuning.gearedDrive.lateralForceScale * 0.5f *
                request.linearSpeed.x);
        if (sideLimit < sideSlowdownInput) {
            sideSlowdownInput = sideLimit;
        }
        float driveScale =
                request.materialVals.y * car.controls.lowSpeedGateA +
                (car.engine.useLowSpeedGateB ? -1.0f : 0.0f) *
                        request.materialVals.y * car.controls.lowSpeedGateB +
                (car.turbo.type != CSceneVehicleCar::ETurboType_Inactive
                         ? car.turbo.impulseScale
                         : 0.0f);
        float driveForce =
                (accelBase -
                 tuning.steering.slowDownScale * sideSlowdownInput *
                         std::fabs(car.controls.currentSteering) *
                         SteerSlowDownFromSpeed<NativeBinary32>(
                                 tuning, request.linearSpeed.z)) *
                driveScale;
        if (car.controls.forcedLowSpeedFriction != 0) {
            driveForce =
                    car.turbo.type == CSceneVehicleCar::ETurboType_Inactive
                            ? 0.0f
                            : accelBase * car.turbo.impulseScale;
        }

        float opposingLongitudinalForce = 0.0f;
        if (request.linearSpeed.z > 0.0f) {
            opposingLongitudinalForce =
                    (tuning.gearedDrive.forwardAccelBase +
                     tuning.gearedDrive.forwardAccelSpeedCoef *
                             request.linearSpeed.z) *
                    car.controls.lowSpeedGateB;
            float cap =
                    request.materialVals.z *
                    (request.outSlipFlag == 0
                             ? tuning.gearedDrive.forwardAccelCap
                             : tuning.gearedDrive
                                       .forwardAccelCapWhenSlipping);
            if (cap < opposingLongitudinalForce) {
                opposingLongitudinalForce = cap;
                car.MarkAllWheelsSlipping();
            }
        }
        if (request.linearSpeed.z < 0.0f &&
            car.controls.forcedLowSpeedFriction != 0) {
            opposingLongitudinalForce =
                    (tuning.gearedDrive.forwardAccelBase -
                     tuning.gearedDrive.forwardAccelSpeedCoef *
                             request.linearSpeed.z) *
                    car.controls.lowSpeedGateA;
            float cap =
                    request.materialVals.z *
                    (request.outSlipFlag == 0
                             ? tuning.gearedDrive.forwardAccelCap
                             : tuning.gearedDrive
                                       .forwardAccelCapWhenSlipping);
            if (cap < opposingLongitudinalForce) {
                opposingLongitudinalForce = cap;
                car.MarkAllWheelsSlipping();
            }
            opposingLongitudinalForce = -opposingLongitudinalForce;
        }
        if (car.controls.forcedLowSpeedFriction != 0 &&
            std::fabs(request.linearSpeed.z) < 1.0f) {
            opposingLongitudinalForce *= std::fabs(request.linearSpeed.z);
        }

        request.outSurfaceFeedback = opposingLongitudinalForce;
        float netLongitudinal = driveForce - opposingLongitudinalForce;
        if (tuning.engineSpeedNorm * request.materialVals.x <
            request.linearSpeed.z) {
            netLongitudinal = -tuning.gearedDrive.speedLimitForce;
        }
        if (request.linearSpeed.z <
            -(tuning.gearedDrive.transmission.reverseSpeedNorm *
              request.materialVals.x)) {
            netLongitudinal = tuning.gearedDrive.speedLimitForce;
        }

        float longitudinalForceZ =
                netLongitudinal * request.slopeAdherenceB;
        AddVehicleCentralForce(
                car, dyna, {0.0f, 0.0f, longitudinalForceZ});
        AddVehicleTorque(dyna, {
                -longitudinalForceZ *
                        tuning.slipResponse.longitudinalTorqueScale,
                0.0f,
                0.0f,
        });
        AddVehicleCentralForce(car, dyna, {
                0.0f,
                0.0f,
                (-tuning.gearedDrive.forceZScale * request.currentForce.z) /
                        tuning.bodyAirResponse.groundedSolidFeedback1,
        });
    }

    static FV_E019_ALWAYS_INLINE float Model6DamperModulation(
            const CSceneVehicleCarTuning &tuning,
            const OptimizedCpuCompiledModel6Tuning &compiled,
            float damperAbsorb) noexcept {
        float normalized = 0.0f;
        if (tuning.suspension.damperModulationMinAbsorb !=
            tuning.suspension.damperModulationMaxAbsorb) {
            normalized =
                    (damperAbsorb -
                     tuning.suspension.damperModulationMinAbsorb) /
                    (tuning.suspension.damperModulationMaxAbsorb -
                     tuning.suspension.damperModulationMinAbsorb);
        }
        return compiled.damperModulation.Evaluate(normalized);
    }

    static FV_E019_ALWAYS_INLINE void ApplyModel6DirtSlideCompiled(
            CSceneVehicleCar &car,
            const CSceneVehicleCar::LegacyForceRequest &request,
            const CSceneVehicleCarTuning &tuning,
            const CSceneVehicleCar::Model6ForceState &state) {
        if (!state.dirtSlideSurface ||
            request.linearSpeed.z <= DirtSlideSpeedGate) {
            return;
        }
        if (car.controls.lowSpeedGateB > LowSpeedGateThreshold) {
            car.AddVehicleCentralForce({
                    -DirtSlideSlowdownScale * request.linearSpeed.x,
                    -DirtSlideSlowdownScale * request.linearSpeed.y,
                    -DirtSlideSlowdownScale * request.linearSpeed.z,
            });
        }
        if (car.controls.forcedLowSpeedFriction != 0 ||
            car.controls.lowSpeedGateA <= LowSpeedGateThreshold ||
            !car.CanApplyDirtSlideForces()) {
            return;
        }

        const GmVec3 unitSpeed = SceneVehicleMath::NormalizeOr(
                request.linearSpeed,
                GmVec3{0.0f, 0.0f, 1.0f},
                VectorEpsilonSquared);
        const CSceneVehicleCar::DirtSlideForces slideForces =
                car.BuildDirtSlideForces(
                        &tuning, request.linearSpeed, unitSpeed);
        const u32 slideWheelCount = car.WheelGetCount();
        for (u32 slideWheelIndex = 0u;
             slideWheelIndex < slideWheelCount;
             ++slideWheelIndex) {
            CSceneVehicleCar::SSimulationWheel &slideWheel =
                    car.WheelAt(slideWheelIndex);
            if (!slideWheel.realTimeState.slipping) {
                continue;
            }
            if (slideWheelIndex <= 1u) {
                car.AddVehicleForce(
                        slideForces.front,
                        slideWheel.forceApplicationPoint);
            }
            if (slideWheelIndex == 2u || slideWheelIndex == 3u) {
                car.AddVehicleForce(
                        slideForces.rear,
                        slideWheel.forceApplicationPoint);
            }
        }
    }

    template<bool NativeBinary32>
    static FV_E019_ALWAYS_INLINE void ApplyModel6ContactForcesCompiled(
            CSceneVehicleCar &car,
            CHmsDyna &dyna,
            const CSceneVehicleCar::LegacyForceRequest &request,
            const CSceneVehicleCarTuning &tuning,
            const OptimizedCpuCompiledModel6Tuning &compiled,
            float maxStaticSideForceCurve,
            float burnoutRollover,
            CSceneVehicleCar::Model6ForceState &state) {
        const GmVec3 bodyCenter = car.HmsItem()
                ->Solid()->Physical().Parameters().localCenterOfMass;
        state.wheelCount = static_cast<u32>(car.wheels.size());

        const float visualSteerYawCos =
                VehicleCos<NativeBinary32>(request.visualSteerYaw);
        const float negVisualSteerYawSin =
                -VehicleSin<NativeBinary32>(request.visualSteerYaw);
        GmVec3 feedbackTorqueForce = SceneVehicleMath::Scale(
                car.gearedDrive.scaledCurrentForce,
                -tuning.feedback.forceDivisor);
        const float feedbackTorqueForceLen = VehicleSqrt<NativeBinary32>(
                SceneVehicleMath::LengthSquared(feedbackTorqueForce));
        if (feedbackTorqueForceLen <
            tuning.gearedDrive.currentForceTorqueMin) {
            feedbackTorqueForce = SceneVehicleMath::Zero();
        }
        const float burnoutSideScale =
                car.BurnoutSideForceFade(&tuning, state.tick);

        for (u32 wheelIndex = 0u;
             wheelIndex < state.wheelCount;
             ++wheelIndex) {
            CSceneVehicleCar::SSimulationWheel &wheel =
                    car.wheels[wheelIndex];
            car.WheelAddForceToVehicle(wheel, request.currentForce);
            if (!wheel.realTimeState.contactPresent ||
                !(tuning.gearedDrive.lateralForceScale >= 0.0f)) {
                continue;
            }

            CSceneVehicleMaterial *material = car.GetWheelMaterial(wheel);
            const GmVec3 contactLever = SceneVehicleMath::Subtract(
                    wheel.realTimeState.latestContactPoint, bodyCenter);
            GmVec3 contactSideAxis = {
                    wheel.realTimeState.accumulatedContactNormal.y,
                    -wheel.realTimeState.accumulatedContactNormal.x,
                    0.0f,
            };
            contactSideAxis = VehicleNormalizeOr<NativeBinary32>(
                    contactSideAxis,
                    GmVec3{1.0f, 0.0f, 0.0f},
                    VectorEpsilonSquared);
            if (IsFrontVehicleWheel(wheel.axle)) {
                contactSideAxis = {
                        visualSteerYawCos * contactSideAxis.x,
                        visualSteerYawCos * contactSideAxis.y,
                        negVisualSteerYawSin +
                                visualSteerYawCos * contactSideAxis.z,
                };
            }

            GmVec3 feedbackTorque = SceneVehicleMath::Cross(
                    contactLever, feedbackTorqueForce);
            feedbackTorque = SceneVehicleMath::Scale(
                    feedbackTorque, -1.0f);
            feedbackTorque.x *= tuning.gearedDrive.currentTorqueXScale;
            feedbackTorque.y = 0.0f;
            feedbackTorque.z *= tuning.gearedDrive.currentTorqueZScale;
            AddVehicleTorque(dyna, feedbackTorque);

            if (car.gearedDrive.burnoutPhase ==
                CSceneVehicleCarBurnoutPhase_TimedSpin) {
                AddVehicleTorque(
                        dyna, {burnoutRollover, 0.0f, 0.0f});
            }

            const float damperMod = Model6DamperModulation(
                    tuning,
                    compiled,
                    wheel.realTimeState.damperAbsorb);
            const bool slipBefore = wheel.realTimeState.slipping;
            const float slippingSideGrip = slipBefore
                    ? tuning.gearedDrive.slippingSideFrictionScale
                    : 1.0f;
            const float lowSpeedBSlippingGrip =
                    (wheel.realTimeState.slipping &&
                     car.controls.lowSpeedGateB > LowSpeedGateThreshold)
                    ? tuning.gearedDrive.lowSpeedBSlippingGripScale
                    : 1.0f;
            const float maxStaticSideForce =
                    material->blendableVals.w * request.slopeAdherenceA *
                    maxStaticSideForceCurve * slippingSideGrip *
                    lowSpeedBSlippingGrip * damperMod;
            const float contactSideSpeed = SceneVehicleMath::Dot(
                    request.linearSpeed, contactSideAxis);
            float requestedSideForce =
                    -tuning.gearedDrive.lateralForceScale * 0.5f *
                    contactSideSpeed * burnoutSideScale;
            const float requestedSideForceAbs =
                    std::fabs(requestedSideForce);
            if (!(maxStaticSideForce < requestedSideForceAbs)) {
                wheel.realTimeState.slipping = false;
            } else {
                const float staticSideForceCap =
                        SignNonNegative(requestedSideForce) *
                        maxStaticSideForce;
                requestedSideForce =
                        (1.0f -
                         tuning.gearedDrive.sideFrictionSlipBlend) *
                                staticSideForceCap +
                        tuning.gearedDrive.sideFrictionSlipBlend *
                                requestedSideForce;
                wheel.realTimeState.slipping = true;
                request.outSlipFlag = 1;
            }

            const GmVec3 contactSideForce = SceneVehicleMath::Scale(
                    contactSideAxis, requestedSideForce);
            // Between the two, exactly as ApplyModel6ContactWheel orders it.
            ApplyModel6DirtSlideCompiled(car, request, tuning, state);
            AddVehicleCentralForce(car, dyna, contactSideForce);
        }
    }

    template<bool NativeBinary32>
    static FV_E019_ALWAYS_INLINE float Model6SteerAssistRamp(
            const CSceneVehicleCarTuning &tuning,
            const GmVec3 &linearSpeed) noexcept {
        const float speed = VehicleSqrt<NativeBinary32>(
                linearSpeed.x * linearSpeed.x +
                linearSpeed.y * linearSpeed.y +
                linearSpeed.z * linearSpeed.z);
        if (speed < 0.7f) {
            return 0.0f;
        }
        if (speed > tuning.steering.assistFullSpeed) {
            return 1.0f;
        }
        const float angle =
                (speed / tuning.steering.assistFullSpeed) *
                SceneVehicleMath::HalfPi;
        return VehicleSin<NativeBinary32>(angle);
    }

    static FV_E019_ALWAYS_INLINE float Model6DriveForceCompiled(
            CSceneVehicleCar &car,
            const CSceneVehicleCar::LegacyForceRequest &request,
            const CSceneVehicleCarTuning &tuning,
            const OptimizedCpuCompiledModel6Tuning &compiled,
            float speedKilometersPerHour,
            u32 tick,
            int waterActive,
            float slipAccelMix) {
        const float slippingAccelCurve =
                tuning.slipResponse.slippingAccelScale *
                compiled.slippingAcceleration.Evaluate(
                        speedKilometersPerHour);
        tuning.slipResponse.accelFromSpeedCurve.Value().SetInterpolation(
                CFuncKeysReal::Constant);
        const float gearAccelCurve = car.engine.useLowSpeedGateB
                ? compiled.rearGearAcceleration.Evaluate(
                          speedKilometersPerHour)
                : compiled.acceleration.EvaluateConstant(
                          speedKilometersPerHour);
        const float blendedAccelCurve =
                car.gearedDrive.engineState ==
                        CSceneVehicleCarEngineControlState_GearShift
                ? 0.0f
                : (1.0f - slipAccelMix) * slippingAccelCurve +
                          gearAccelCurve * slipAccelMix;
        const float turboDriveOverride =
                car.turbo.type != CSceneVehicleCar::ETurboType_Inactive
                ? gearAccelCurve * car.turbo.impulseScale
                : 0.0f;
        const float rearGearMaterialSign =
                car.engine.useLowSpeedGateB ? -1.0f : 0.0f;
        const float steeringSlowdownForce =
                tuning.steering.slowDownScale *
                std::fabs(car.controls.currentSteering) *
                compiled.steerSlowdown.Evaluate(
                        speedKilometersPerHour) *
                (car.engine.useLowSpeedGateB ? -1.0f : 1.0f);
        float driveForce =
                car.BurnoutExitAcceleration(&tuning, tick) +
                car.BurnoutDriveFade(&tuning, tick) *
                        (turboDriveOverride +
                         (car.controls.lowSpeedGateA *
                                  request.materialVals.y +
                          rearGearMaterialSign * request.materialVals.y *
                                  car.controls.lowSpeedGateB) *
                                 blendedAccelCurve) -
                steeringSlowdownForce;
        if (waterActive != 0) {
            driveForce *= 0.5f;
        }
        if (car.controls.forcedLowSpeedFriction != 0) {
            driveForce =
                    car.turbo.type != CSceneVehicleCar::ETurboType_Inactive
                    ? gearAccelCurve * car.turbo.impulseScale
                    : 0.0f;
        }
        return driveForce;
    }

    template<bool NativeBinary32>
    static FV_E019_ALWAYS_INLINE void ApplyModel6GroundForcesCompiled(
            CSceneVehicleCar &car,
            CHmsDyna &dyna,
            const CSceneVehicleCar::LegacyForceRequest &request,
            const CSceneVehicleCarTuning &tuning,
            const OptimizedCpuCompiledModel6Tuning &compiled,
            float speedKilometersPerHour,
            float maxStaticSideForceCurve,
            CSceneVehicleCar::Model6ForceState &state) {
        if (car.controls.forcedLowSpeedFriction == 0 &&
            car.controls.lowSpeedGateB > LowSpeedGateThreshold &&
            car.controls.lowSpeedGateA < LowSpeedGateThreshold &&
            car.gearedDrive.burnoutPhase ==
                    CSceneVehicleCarBurnoutPhase_TimedSpin) {
            car.gearedDrive.burnoutExitStartTick = state.tick;
            car.gearedDrive.burnoutPhase =
                    CSceneVehicleCarBurnoutPhase_ExitFade;
        }

        car.TryEnterForwardBurnout(
                &tuning,
                request.linearSpeed,
                request.visualSteerYaw,
                state.frameY,
                state.tick,
                request.hasGroundMaterial);
        car.UpdateGearDirection(request.linearSpeed);

        const float rolloverInput =
                (request.linearSpeed.x * request.linearSpeed.x) /
                (std::fabs(request.linearSpeed.z) + 1.0f);
        AddVehicleTorque(dyna, {
                0.0f,
                0.0f,
                -SignNonNegative(request.linearSpeed.x) *
                        compiled.rolloverLateralFromSpeedRatio.Evaluate(
                                OptimizedCpuCompiledTuningCurve::
                                        ConvertSpeedToKmh(rolloverInput)),
        });

        const float steerAssistRamp = Model6SteerAssistRamp<NativeBinary32>(
                tuning, request.linearSpeed);
        const float maxStaticSideForce =
                maxStaticSideForceCurve *
                request.materialVals.w;
        const float steerTorque =
                compiled.steerDriveTorque.Evaluate(
                        speedKilometersPerHour);
        for (u32 wheelIndex = 0u;
             wheelIndex < state.wheelCount;
             ++wheelIndex) {
            CSceneVehicleCar::SSimulationWheel &wheel =
                    car.wheels[wheelIndex];
            const bool front = IsFrontVehicleWheel(wheel.axle);
            const float signedHalfTrack =
                    (front ? car.gearedDrive.wheelLongitudinalSpan
                           : -car.gearedDrive.wheelLongitudinalSpan) *
                    0.5f;
            const float wheelSideSpeed = request.linearSpeed.x +
                    request.angularSpeed.y * signedHalfTrack;
            float requestedSideForce =
                    -tuning.gearedDrive.lateralForceScale * 0.5f *
                    wheelSideSpeed;
            CSceneVehicleCar::GearedWheelSideForceResult wheelForce;
            if (std::fabs(requestedSideForce) > maxStaticSideForce) {
                const float clippedSideForceMagnitude =
                        (1.0f -
                         tuning.gearedDrive.driveSideFrictionSlipBlend) *
                                maxStaticSideForce +
                        tuning.gearedDrive.driveSideFrictionSlipBlend *
                                std::fabs(requestedSideForce);
                wheelForce.sideLimit = maxStaticSideForce;
                wheelForce.sideRequested =
                        std::fabs(requestedSideForce);
                requestedSideForce =
                        SignNonNegative(requestedSideForce) *
                        clippedSideForceMagnitude;
                wheelForce.slipped = true;
            }

            float driveSideTorque =
                    tuning.gearedDrive.sideForceToDriveTorqueScale *
                    requestedSideForce;
            float steerAssistTorque = 0.0f;
            if (front) {
                steerAssistTorque =
                        steerAssistRamp * car.controls.currentSteering *
                        steerTorque;
                if (car.engine.useLowSpeedGateB) {
                    steerAssistTorque = -steerAssistTorque;
                }
                if (wheel.realTimeState.slipping) {
                    steerAssistTorque *=
                            tuning.gearedDrive.slippingSteerTorqueScale;
                }
                driveSideTorque -= steerAssistTorque;
            }
            AddVehicleTorque(
                    dyna,
                    {0.0f,
                     driveSideTorque * signedHalfTrack,
                     0.0f});
            wheelForce.force = requestedSideForce;
            state.sideForceLimitTotal += wheelForce.sideLimit;
            state.requestedSideForceTotal += wheelForce.sideRequested;
            state.slipSeen |= wheelForce.slipped;
        }

        car.UpdateSlipMemory(state.tick, state.slipSeen);
        const float slipAccelMix = car.ComputeSlipAccelerationBlend(
                &tuning,
                state.tick,
                state.sideForceLimitTotal,
                state.requestedSideForceTotal);
        const float driveForce = Model6DriveForceCompiled(
                car,
                request,
                tuning,
                compiled,
                speedKilometersPerHour,
                state.tick,
                state.waterActive,
                slipAccelMix);
        const CSceneVehicleCar::OpposingLongitudinalResult opposing =
                car.ComputeOpposingLongitudinalForce(
                        &tuning,
                        request.materialVals,
                        request.linearSpeed,
                        driveForce,
                        state.frameY,
                        state.tick,
                        request.outSlipFlag);
        const float opposingLongitudinalForce = opposing.force;
        state.slipSeen |= opposing.slipped;
        request.outSurfaceFeedback = opposingLongitudinalForce;

        float netLongitudinal =
                driveForce - SignNonNegative(request.linearSpeed.z) *
                                     opposingLongitudinalForce;
        if (request.linearSpeed.z >
            tuning.engineSpeedNorm * request.materialVals.x) {
            netLongitudinal = !(netLongitudinal < 0.0f)
                    ? -tuning.gearedDrive.speedLimitForce
                    : netLongitudinal -
                              tuning.gearedDrive.speedLimitForce;
        }
        if (request.linearSpeed.z <
            -tuning.gearedDrive.transmission.reverseSpeedNorm *
                    request.materialVals.x) {
            netLongitudinal = !(netLongitudinal > 0.0f)
                    ? tuning.gearedDrive.speedLimitForce
                    : netLongitudinal +
                              tuning.gearedDrive.speedLimitForce;
        }

        AddVehicleCentralForce(
                car,
                dyna,
                {0.0f,
                 0.0f,
                 netLongitudinal * request.slopeAdherenceB});
        AddVehicleCentralForce(car, dyna, {
                0.0f,
                0.0f,
                (-tuning.gearedDrive.forceZScale *
                 request.currentForce.z) /
                        tuning.bodyAirResponse.groundedSolidFeedback1,
        });
        car.slipMemory.active = state.slipSeen != 0;
        car.gearedDrive.localSpeed = request.linearSpeed;
    }

    template<bool NativeBinary32>
    static FV_E019_HOT_NOINLINE void ComputeModel6Compiled(
            CSceneVehicleCar &car,
            CHmsDyna &dyna,
            float dt,
            const GmVec3 &currentForce,
            float slopeAdherenceA,
            float slopeAdherenceB,
            const GmVec3 &linearSpeed,
            const GmVec3 &angularSpeed,
            float visualSteerYaw,
            int hasGroundMaterial,
            CSceneVehicleMaterial::SBlendableVals &materialVals,
            int &outSlipFlag,
            float &outSurfaceFeedback,
            const CSceneVehicleCarTuning &tuning,
            const OptimizedCpuCompiledModel6Tuning &compiled) {
        CSceneVehicleCar::LegacyForceRequest request{
                dt,
                currentForce,
                slopeAdherenceA,
                slopeAdherenceB,
                linearSpeed,
                angularSpeed,
                visualSteerYaw,
                hasGroundMaterial != 0,
                materialVals,
                outSlipFlag,
                outSurfaceFeedback,
        };
        car.CaptureBurnoutReferenceFrame();

        CSceneVehicleCar::Model6ForceState state;
        state.frameY = car.gearedDrive.frameIso.rotation.basisY.y;
        state.waterActive = car.ApplyWaterForces(currentForce);
        car.controls.noGroundFrictionGuard = state.waterActive != 0;
        // The reference derives this here, before the burnout check and the
        // contact pass, and the dirt-slide forces it gates are what kept this
        // whole specialization off any track with a slippery surface on it.
        state.dirtSlideSurface =
                car.AllWheelsContactMaterial(DirtSlideMaterial) != 0;
        state.tick = CMwCmdBufferCore::Current()->Timer().GetTickTime();
        state.slipSeen = car.AdvanceBurnoutPhases(&tuning, state.tick);
        const float speedKilometersPerHour =
                OptimizedCpuCompiledTuningCurve::ConvertSpeedToKmh(
                        request.linearSpeed.z);
        const float maxStaticSideForceCurve =
                compiled.maxSideFriction.Evaluate(speedKilometersPerHour);
        const float burnoutRollover =
                compiled.burnoutRollover.Evaluate(speedKilometersPerHour);
        ApplyModel6ContactForcesCompiled<NativeBinary32>(
                car,
                dyna,
                request,
                tuning,
                compiled,
                maxStaticSideForceCurve,
                burnoutRollover,
                state);
        if (!request.hasGroundMaterial) {
            if (car.gearedDrive.burnoutPhase ==
                CSceneVehicleCarBurnoutPhase_TimedSpin) {
                car.gearedDrive.burnoutExitStartTick = state.tick;
                car.gearedDrive.burnoutPhase =
                        CSceneVehicleCarBurnoutPhase_ExitFade;
            }
            car.slipMemory.active = state.slipSeen != 0;
            car.gearedDrive.localSpeed = linearSpeed;
            return;
        }
        ApplyModel6GroundForcesCompiled<NativeBinary32>(
                car,
                dyna,
                request,
                tuning,
                compiled,
                speedKilometersPerHour,
                maxStaticSideForceCurve,
                state);
    }

    template<bool NativeBinary32>
    static FV_E019_ALWAYS_INLINE void ComputeModel3(
            CSceneVehicleCar &car,
            CHmsDyna &dyna,
            float dt,
            const GmVec3 &currentForce,
            float slopeAdherenceA,
            float slopeAdherenceB,
            const GmVec3 &linearSpeed,
            const GmVec3 &angularSpeed,
            float visualSteerYaw,
            int hasGroundMaterial,
            CSceneVehicleMaterial::SBlendableVals &materialVals,
            int &outSlipFlag,
            float &outSurfaceFeedback,
            const CSceneVehicleCarTuning &tuning) {
        CSceneVehicleCar::LegacyForceRequest request{
                dt,
                currentForce,
                slopeAdherenceA,
                slopeAdherenceB,
                linearSpeed,
                angularSpeed,
                visualSteerYaw,
                hasGroundMaterial != 0,
                materialVals,
                outSlipFlag,
                outSurfaceFeedback,
        };
        ApplyContactForces<NativeBinary32>(car, dyna, request, tuning);
        if (!request.hasGroundMaterial ||
            tuning.handlingModel != CSceneVehicleCarHandlingModel_Lateral) {
            return;
        }

        float speedMagnitude =
                VehicleSqrt<NativeBinary32>(VehicleLengthSquared(linearSpeed));
        if (car.controls.forcedLowSpeedFriction == 0) {
            if (!(speedMagnitude < car.reverseGearSpeedThreshold)) {
                if (car.controls.lowSpeedGateA > LowSpeedGateThreshold) {
                    car.engine.useLowSpeedGateB = false;
                }
            } else if (!(LowSpeedGateThreshold <
                         car.controls.lowSpeedGateB)) {
                car.engine.useLowSpeedGateB = false;
            } else {
                car.engine.useLowSpeedGateB = true;
            }
        }
        ApplySteeringTorques<NativeBinary32>(
                car, dyna, request, tuning, speedMagnitude);
        ApplyDriveForces<NativeBinary32>(car, dyna, request, tuning);
    }

    template<bool NativeBinary32>
    static FV_E019_HOT_NOINLINE void ComputeForces(
            CSceneVehicleCar &car,
            CHmsDyna &dyna,
            float dt,
            const OptimizedCpuCompiledModel6Tuning *compiledModel6,
            bool reuseIntegratedLinearSpeed,
            forevervalidator::simulation::
                    OptimizedCpuVehicleCollisionBoundsPlan
                            *collisionBoundsPlan) {
        GmVec3 savedForce;
        GmVec3 savedImpulse;
        car.SaveAndClearAccumulatedFeedback(savedForce, savedImpulse);

        if (car.integration.speedBlocked ||
            car.integration.speedBlockedSecondary ||
            car.WaterState().boxLocal.halfExtents.x < 0.0f) {
            car.SetZeroDynamics();
            return;
        }

        car.CreateFakeContacts();
        GmVec3 linearSpeed;
        IntegrateVehicle<NativeBinary32>(
                car, dt, linearSpeed, collisionBoundsPlan);

        u32 tick = CMwCmdBufferCore::Current()->Timer().GetTickTime();
        int isGroundContact = car.IsGroundContact();
        CSceneVehicleCarTuning *tuning = car.Tunings()->ActiveTuning();
        car.UpdateDynaParamsForGroundContact(tuning, isGroundContact);

        if (!car.integration.integrateWheels) {
            return;
        }

        GmVec3 angularSpeed;
        GmVec3 currentForce;
        CSceneVehicleMaterial::SBlendableVals materialVals = {
                1.0f, 1.0f, 1.0f, 1.0f};
        int hasGroundMaterial = 0;
        float slopeAdherenceA = 1.0f;
        float slopeAdherenceB = 1.0f;
        float visualSteerYaw = 0.0f;
        int modelSlipFlag = 0;
        int hasSideSpeedKillContact = 0;
        int hasAnyContact = 0;
        // Wheel integration itself does not change item speed. Reuse its
        // authoritative pre-integration read only when the bound observer also
        // certifies that its wheel-update callback preserves dynamics.
        if (!reuseIntegratedLinearSpeed) {
            car.HmsItem()->GetLinearSpeed(linearSpeed);
        }

        float surfaceFeedback = 0.0f;
        if (car.integration.zeroHorizontalSpeed) {
            linearSpeed.x = 0.0f;
            linearSpeed.z = 0.0f;
            car.HmsItem()->SetLinearSpeed(linearSpeed);
        } else {
            car.HmsItem()->GetAngularSpeed(angularSpeed);
            car.HmsItem()->GetForce(currentForce);

            car.engine.lowSpeedFeedbackForce = 0.0f;
            car.ApplyFrictionForces(linearSpeed);
            car.ClampLinearSpeed(linearSpeed);

            car.ComputeVehicleGroundMaterialVals(
                    materialVals, hasGroundMaterial);
            GetSlopeAdherence<NativeBinary32>(
                    car,
                    currentForce,
                    slopeAdherenceA,
                    slopeAdherenceB);

            visualSteerYaw = ComputeVisualSteerYaw<NativeBinary32>(
                    car, tuning, linearSpeed);
            car.gearedDrive.localSpeed = linearSpeed;

            if (tuning->handlingModel ==
                        CSceneVehicleCarHandlingModel_GearedDrive &&
                compiledModel6 != nullptr) {
                ComputeModel6Compiled<NativeBinary32>(
                        car,
                        dyna,
                        dt,
                        currentForce,
                        slopeAdherenceA,
                        slopeAdherenceB,
                        linearSpeed,
                        angularSpeed,
                        visualSteerYaw,
                        hasGroundMaterial,
                        materialVals,
                        modelSlipFlag,
                        surfaceFeedback,
                        *tuning,
                        *compiledModel6);
            } else {
                ComputeModel3<NativeBinary32>(
                        car,
                        dyna,
                        dt,
                        currentForce,
                        slopeAdherenceA,
                        slopeAdherenceB,
                        linearSpeed,
                        angularSpeed,
                        visualSteerYaw,
                        hasGroundMaterial,
                        materialVals,
                        modelSlipFlag,
                        surfaceFeedback,
                        *tuning);
            }

            hasAnyContact = car.ScanWheelSideSpeedKillContacts(
                    hasSideSpeedKillContact);
            car.UpdateLowSpeedFeedback(tuning, hasAnyContact);
            car.KillSideSpeedForTaggedContact(
                    tuning, hasSideSpeedKillContact, linearSpeed);

            car.ComputeAirControl(
                    angularSpeed,
                    tick,
                    isGroundContact,
                    hasSideSpeedKillContact);
            car.ApplySpecialContactResponse(
                    tuning, currentForce, tick, isGroundContact);
            car.UpdateImpactStates(tuning);
            car.lastComputeForcesTick = tick;
            car.ProcessTurboContacts(tuning, tick);
            car.UpdateTurbo(tick);

            car.OtherVehicleForces();
        }

        UpdateFeedbackTail<NativeBinary32>(
                car,
                tuning,
                dt,
                linearSpeed,
                savedForce,
                savedImpulse,
                surfaceFeedback);
        car.ClearWheelContactScratch();
        car.ResetPerTickContactFeedback();
    }
};

namespace forevervalidator::simulation {

OptimizedCpuVehicleForceContext::OptimizedCpuVehicleForceContext(void) =
        default;

OptimizedCpuVehicleForceContext::~OptimizedCpuVehicleForceContext(void) =
        default;

void OptimizedCpuVehicleForceContext::BeginTick(
        CSceneVehicleCar &car,
        OptimizedCpuBinary32MathPath mathPath,
        CHmsItem::CCallback *enabledComputeForcesCallback) noexcept {
    collisionBoundsPlan_.InvalidateDirectLaneSnapshot();
    tickEligible_ = false;
    if (mathPath != OptimizedCpuBinary32MathPath::X86Sse2 ||
        enabledComputeForcesCallback == nullptr ||
        car.ArePhysicsUpdatesEnabled() == 0) {
        return;
    }

    CHmsItem *item = car.HmsItem();
    if (item == nullptr ||
        item->CallbackGet(CHmsItem::ECallback_ComputeForces) !=
                enabledComputeForcesCallback) {
        return;
    }

    CSceneVehicleCarTuning *tuning =
            OptimizedCpuVehicleForceAccess::ActiveTuning(car);
    CPlugTree *collisionTree = item->Solid() != nullptr
            ? item->Solid()->CollisionTree()
            : nullptr;
    CSceneVehicleCarWheelSurfaceObserver *wheelSurfaceObserver =
            OptimizedCpuVehicleForceAccess::WheelSurfaceObserver(car);
    const bool identityChanged =
            car_ != &car || item_ != item || tuning_ != tuning ||
            collisionTree_ != collisionTree;
    if (identityChanged) {
        car_ = &car;
        item_ = item;
        tuning_ = tuning;
        collisionTree_ = collisionTree;
        wheelSurfaceObserver_ = wheelSurfaceObserver;
        wheelSurfaceObserverPreservesDynamics_ =
                car.WheelSurfaceObserverPreservesDynamics();
        canonicalCallback_ = enabledComputeForcesCallback;
        compiledModel6_.reset();
        collisionBoundsPlan_.Clear();
        collisionBoundsPlanAttempted_ = false;
        stableEligible_ = false;
    } else if (canonicalCallback_ != enabledComputeForcesCallback) {
        return;
    } else if (wheelSurfaceObserver_ != wheelSurfaceObserver) {
        wheelSurfaceObserver_ = wheelSurfaceObserver;
        wheelSurfaceObserverPreservesDynamics_ =
                car.WheelSurfaceObserverPreservesDynamics();
    }

    if (!stableEligible_) {
        stableEligible_ =
                OptimizedCpuVehicleForceAccess::HasStableEligibility(
                        car, item, tuning, mathPath);
    }
    if (stableEligible_ && !collisionBoundsPlanAttempted_) {
        collisionBoundsPlanAttempted_ = true;
        if (collisionTree_ != nullptr) {
            (void)collisionBoundsPlan_.TryBuild(*collisionTree_);
        }
    }
    if (stableEligible_ && tuning->handlingModel ==
            CSceneVehicleCarHandlingModel_GearedDrive) {
        if (!OptimizedCpuVehicleForceAccess::
                    HasRequiredModel6Configuration(car, *tuning)) {
            compiledModel6_.reset();
            stableEligible_ = false;
            return;
        }
        if (compiledModel6_ != nullptr &&
            !compiledModel6_->IsFor(*tuning)) {
            compiledModel6_.reset();
        }
        if (compiledModel6_ == nullptr) {
            std::unique_ptr<OptimizedCpuCompiledModel6Tuning> compiled;
            try {
                compiled =
                        std::make_unique<OptimizedCpuCompiledModel6Tuning>();
            } catch (const std::bad_alloc &) {
                return;
            }
            if (!compiled->TryBuild(*tuning)) {
                return;
            }
            compiledModel6_ = std::move(compiled);
        }
    }
    tickEligible_ = stableEligible_ && item->SceneMobilOwner() == &car &&
            (OptimizedCpuVehicleForceAccess::
                     HasRequiredModel3Configuration(*tuning) ||
             (OptimizedCpuVehicleForceAccess::
                      HasRequiredModel6Configuration(car, *tuning) &&
              compiledModel6_ != nullptr));
}

void OptimizedCpuVehicleForceContext::Reset(void) noexcept {
    car_ = nullptr;
    item_ = nullptr;
    tuning_ = nullptr;
    collisionTree_ = nullptr;
    wheelSurfaceObserver_ = nullptr;
    canonicalCallback_ = nullptr;
    compiledModel6_.reset();
    collisionBoundsPlan_.Clear();
    collisionBoundsPlanAttempted_ = false;
    wheelSurfaceObserverPreservesDynamics_ = true;
    stableEligible_ = false;
    tickEligible_ = false;
}

bool OptimizedCpuVehicleForceContext::WouldUseSpecializationFor(
        const CHmsItem *item) const noexcept {
    return tickEligible_ && car_ != nullptr && item_ != nullptr &&
           tuning_ != nullptr && item == item_ && car_->HmsItem() == item_ &&
           item_->SceneMobilOwner() == car_ &&
           OptimizedCpuVehicleForceAccess::WheelSurfaceObserver(*car_) ==
                   wheelSurfaceObserver_ &&
           car_->ArePhysicsUpdatesEnabled() != 0 &&
           OptimizedCpuVehicleForceAccess::ActiveTuning(*car_) ==
                   tuning_ &&
           (tuning_->handlingModel == CSceneVehicleCarHandlingModel_Lateral ||
            (tuning_->handlingModel ==
                     CSceneVehicleCarHandlingModel_GearedDrive &&
             compiledModel6_ != nullptr && compiledModel6_->IsFor(*tuning_))) &&
           item_->CallbackGet(CHmsItem::ECallback_ComputeForces) ==
                   canonicalCallback_;
}

bool OptimizedCpuVehicleForceContext::TryRefreshCollisionBounds(
        CPlugTree *root) noexcept {
    // TryRefresh, not RefreshRuntimeCertified: the specialization earns the
    // unchecked variant by having certified the item and the callback first,
    // and none of that has been established on this path. The check is a
    // handful of pointer comparisons and one box compare per child, which is
    // still far less than the recursive traversal it replaces.
    return root != nullptr && collisionBoundsPlan_.IsFor(root) &&
           collisionBoundsPlan_.TryRefresh();
}

bool OptimizedCpuVehicleForceContext::TryComputeOwnerForces(
        CHmsCorpus *corpus,
        float dt) {
    if (corpus == nullptr ||
        !WouldUseSpecializationFor(corpus->Item()) ||
        item_->CorpusCount() != 1u ||
        item_->CorpusAt(0u) != corpus ||
        corpus->Dynamics() == nullptr) {
        return false;
    }
    if (tuning_->handlingModel ==
                CSceneVehicleCarHandlingModel_GearedDrive &&
        !OptimizedCpuVehicleForceAccess::CanUseModel6CommonPath(
                *car_, *corpus->Dynamics())) {
        return false;
    }
    OptimizedCpuVehicleForceAccess::ComputeForces<true>(
            *car_,
            *corpus->Dynamics(),
            dt,
            tuning_->handlingModel ==
                    CSceneVehicleCarHandlingModel_GearedDrive
                    ? compiledModel6_.get()
                    : nullptr,
            wheelSurfaceObserverPreservesDynamics_,
            collisionBoundsPlan_.IsFor(collisionTree_)
                    ? &collisionBoundsPlan_
                    : nullptr);
    return true;
}

float OptimizedCpuEvaluateVehicleCurveForDifferential(
        CFuncKeysReal &curve,
        float input,
        bool convertSpeedToKmh,
        bool forceConstantInterpolation,
        OptimizedCpuBinary32MathPath mathPath) noexcept {
    const auto evaluate = [&curve, input](auto nativeTag) noexcept {
        constexpr bool NativeBinary32 = decltype(nativeTag)::value;
        return OptimizedCpuVehicleForceAccess::
                EvaluateCurve<NativeBinary32>(curve, input);
    };
    const auto evaluateSpeed = [&curve, input](auto nativeTag) noexcept {
        constexpr bool NativeBinary32 = decltype(nativeTag)::value;
        const float converted = VehicleFromDouble<NativeBinary32>(
                static_cast<double>(input) * static_cast<double>(3.6f));
        return OptimizedCpuVehicleForceAccess::
                EvaluateCurve<NativeBinary32>(curve, converted);
    };

    if (forceConstantInterpolation) {
        curve.SetInterpolation(CFuncKeysReal::Constant);
    }
    if (mathPath == OptimizedCpuBinary32MathPath::X86Sse2) {
        return convertSpeedToKmh
                ? evaluateSpeed(std::true_type{})
                : evaluate(std::true_type{});
    }
    return convertSpeedToKmh
            ? evaluateSpeed(std::false_type{})
            : evaluate(std::false_type{});
}

}  // namespace forevervalidator::simulation

void CHmsZoneDynamic::ComputeCorpusForcesOptimizedCpuVehicle(
        CHmsCorpus *corpus,
        float dt,
        forevervalidator::simulation::
                OptimizedCpuVehicleForceContext &context) {
    CHmsDyna *dyna = corpus->Dynamics();
    if (dyna == 0) {
        return;
    }

    CHmsDynaParams *dynaParams = &dyna->Parameters();

    dyna->ValidateDynamicState();

    if (corpus->Item()->GetProperties().kinematicOnly) {
        GmVec3 zero = GmVec3::ZeroForComputeCorpusForces();
        dyna->SetForce(zero);
        dyna->SetTorque(zero);
        return;
    }

    GmVec3 accumulatedForce = GmVec3::ZeroForComputeCorpusForces();

    for (CHmsForceField *field : ForceFields()) {
        GmVec3 fieldValue;
        if (field->GetValue(dyna->CurrentState().position, fieldValue)) {
            accumulatedForce.AddScaledForComputeCorpusForces(
                    fieldValue,
                    dynaParams->forceScale * dynaParams->mass);
        }
    }

    GmVec3 linearSpeed;
    dyna->GetLinearSpeed(linearSpeed);
    accumulatedForce.AddScaledForComputeCorpusForces(
            linearSpeed,
            -linearDampingCoef_ * dynaParams->linearDampingScale);
    dyna->SetForce(accumulatedForce);

    if (dyna->DynamicType() ==
        CHmsDyna::EDynamicType_FullAngularDynamics) {
        GmVec3 angularSpeed;
        dyna->GetAngularSpeed(angularSpeed);
        GmVec3 dampingTorque = angularSpeed;
        dampingTorque.ScaleInPlaceForComputeCorpusForces(
                -angularDampingCoef_ * dynaParams->angularDampingScale);
        dyna->SetTorque(dampingTorque);
    }

    if (!context.TryComputeOwnerForces(corpus, dt)) {
        // Bound only for the duration of the call that can use it, so the
        // context never outlives its own binding on the car, and only when the
        // context is the one for this corpus -- otherwise the plan it holds
        // describes a different car's tree and would decline anyway.
        CSceneVehicleCar *car =
                corpus != nullptr &&
                                context.WouldUseSpecializationFor(corpus->Item())
                        ? context.SpecializedCar()
                        : nullptr;
        if (car != nullptr) {
            car->BindCollisionBoundsRefresh(context);
            corpus->ComputeOwnerForces(dt);
            car->ClearCollisionBoundsRefresh();
        } else {
            corpus->ComputeOwnerForces(dt);
        }
    }
}

#undef FV_E019_ALWAYS_INLINE
#undef FV_E019_HOT_NOINLINE
