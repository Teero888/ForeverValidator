#pragma once

#include <memory>

#include "engine/physics/dynamics/hms_item.h"
#include "simulation/backends/optimized_cpu/optimized_cpu_binary32_math.h"
#include "simulation/backends/optimized_cpu/optimized_cpu_vehicle_collision_bounds_plan.h"
#include "engine/scene/scene_vehicle_car.h"

class CFuncKeysReal;
struct CPlugTree;
class CSceneVehicleCar;
class CSceneVehicleCarTuning;
class CSceneVehicleCarWheelSurfaceObserver;
struct CHmsCorpus;

namespace forevervalidator::simulation {

struct OptimizedCpuCompiledModel6Tuning;

// Per-runtime routing state for exact native vehicle-force specializations.
// It never replaces the item's callback: a mismatch therefore falls through to
// the authoritative callback before any vehicle-owned state is changed.
class OptimizedCpuVehicleForceContext final
    : public CSceneVehicleCarCollisionBoundsRefresh {
public:
    OptimizedCpuVehicleForceContext(void);
    ~OptimizedCpuVehicleForceContext(void);

    OptimizedCpuVehicleForceContext(
            const OptimizedCpuVehicleForceContext &) = delete;
    OptimizedCpuVehicleForceContext &operator=(
            const OptimizedCpuVehicleForceContext &) = delete;

    void BeginTick(
            CSceneVehicleCar &car,
            OptimizedCpuBinary32MathPath mathPath,
            CHmsItem::CCallback *enabledComputeForcesCallback) noexcept;
    void Reset(void) noexcept;

    bool IsTickEligible(void) const noexcept { return tickEligible_; }
    bool WouldUseSpecializationFor(
            const CHmsItem *item) const noexcept;
    bool TryComputeOwnerForces(CHmsCorpus *corpus, float dt);
    // The reference force path refreshes the car's collision tree partway
    // through its work, recursively. The compiled bounds plan replays the same
    // transforms and unions exactly and is built whenever this context is
    // eligible at all, so it is offered to the reference path too rather than
    // being reachable only from the specialization that built it -- the
    // specialization still declines on a circular-drift burnout, on gas and
    // brake together, and for any tuning it has no compiled model for.
    bool TryRefreshCollisionBounds(CPlugTree *root) noexcept override;
    // The car this context is specialized for, or null when it has none.
    CSceneVehicleCar *SpecializedCar(void) const noexcept { return car_; }

private:
    CSceneVehicleCar *car_ = nullptr;
    CHmsItem *item_ = nullptr;
    CSceneVehicleCarTuning *tuning_ = nullptr;
    CPlugTree *collisionTree_ = nullptr;
    CSceneVehicleCarWheelSurfaceObserver *wheelSurfaceObserver_ = nullptr;
    CHmsItem::CCallback *canonicalCallback_ = nullptr;
    std::unique_ptr<OptimizedCpuCompiledModel6Tuning> compiledModel6_;
    OptimizedCpuVehicleCollisionBoundsPlan collisionBoundsPlan_;
    bool collisionBoundsPlanAttempted_ = false;
    bool wheelSurfaceObserverPreservesDynamics_ = true;
    bool stableEligible_ = false;
    bool tickEligible_ = false;
};

// Focused differential entry point for the exact native vehicle-curve
// evaluator used by the specialized force paths.
float OptimizedCpuEvaluateVehicleCurveForDifferential(
        CFuncKeysReal &curve,
        float input,
        bool convertSpeedToKmh,
        bool forceConstantInterpolation,
        OptimizedCpuBinary32MathPath mathPath) noexcept;

}  // namespace forevervalidator::simulation
