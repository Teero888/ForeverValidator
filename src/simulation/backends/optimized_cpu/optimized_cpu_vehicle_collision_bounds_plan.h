#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "engine/core/gm_types.h"

struct CPlugSurface;
struct CPlugSurfaceGeom;
struct CPlugTree;

namespace forevervalidator::simulation {

// Compiles the common Stadium vehicle collision-tree shape: one undecorated
// root and a fixed ordered list of direct surface leaves. Refresh replays the
// authoritative scalar box transforms and ordered unions exactly, but avoids
// recursive virtual traversal and repeated decoration discovery.
class OptimizedCpuVehicleCollisionBoundsPlan final {
public:
    static constexpr std::size_t MaxChildCount = 16u;

    // The eight-leaf vehicle path consumes these same values immediately
    // after bounds refresh. Keep the exact refreshed bits in one aligned SoA
    // so packet-lane collection does not gather them back out of eight tree
    // objects. The snapshot is invalidated at every tick boundary and before
    // every refresh attempt.
    struct alignas(32) DirectLaneSnapshot {
        std::array<CPlugTree *, 8u> trees{};
        std::array<float, 8u> centerX{};
        std::array<float, 8u> centerY{};
        std::array<float, 8u> centerZ{};
        std::array<float, 8u> halfX{};
        std::array<float, 8u> halfY{};
        std::array<float, 8u> halfZ{};
        std::array<float, 8u> locationXx{};
        std::array<float, 8u> locationXy{};
        std::array<float, 8u> locationXz{};
        std::array<float, 8u> locationYx{};
        std::array<float, 8u> locationYy{};
        std::array<float, 8u> locationYz{};
        std::array<float, 8u> locationZx{};
        std::array<float, 8u> locationZy{};
        std::array<float, 8u> locationZz{};
        std::array<float, 8u> locationTx{};
        std::array<float, 8u> locationTy{};
        std::array<float, 8u> locationTz{};
    };

    bool TryBuild(CPlugTree &root) noexcept;
    bool TryRefresh(void) const noexcept;
    void RefreshRuntimeCertified(void) const noexcept;
    void Clear(void) noexcept;

    bool IsAvailable(void) const noexcept {
        return root_ != nullptr && childCount_ >= 2u;
    }

    std::size_t ChildCount(void) const noexcept {
        return childCount_;
    }

    bool IsFor(const CPlugTree *root) const noexcept {
        return IsAvailable() && root_ == root;
    }

    const DirectLaneSnapshot *DirectLaneSnapshotFor(
            const CPlugTree *root) const noexcept {
        return directLaneSnapshotValid_ && root_ == root &&
                       childCount_ == 8u
                ? &directLaneSnapshot_
                : nullptr;
    }

    void InvalidateDirectLaneSnapshot(void) const noexcept {
        directLaneSnapshotValid_ = false;
    }

private:
    void RefreshUnchecked(void) const noexcept;
#if (defined(__i386__) || defined(__x86_64__)) && \
        (defined(__GNUC__) || defined(__clang__))
    void RefreshEightChildrenAvx2(void) const noexcept
            __attribute__((target("avx2")));
#endif

    struct Child {
        CPlugTree *tree = nullptr;
        CPlugSurface *surface = nullptr;
        CPlugSurfaceGeom *geometry = nullptr;
        GmBoxAligned geometryBounds{};
    };

    struct alignas(32) EightChildGeometryBounds {
        std::array<float, 8u> centerX{};
        std::array<float, 8u> centerY{};
        std::array<float, 8u> centerZ{};
        std::array<float, 8u> halfX{};
        std::array<float, 8u> halfY{};
        std::array<float, 8u> halfZ{};
    };

    CPlugTree *root_ = nullptr;
    std::array<Child, MaxChildCount> children_{};
    EightChildGeometryBounds eightChildGeometryBounds_{};
    const float *eightChildLocalIsoGatherBase_ = nullptr;
    alignas(32) std::array<std::int32_t, 8u>
            eightChildLocalIsoGatherOffsets_{};
    mutable DirectLaneSnapshot directLaneSnapshot_{};
    std::size_t childCount_ = 0u;
    bool eightChildAvx2Available_ = false;
    mutable bool directLaneSnapshotValid_ = false;
};

}  // namespace forevervalidator::simulation
