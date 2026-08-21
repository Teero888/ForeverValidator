#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "engine/physics/collision/hms_collision_manager.h"
#include "simulation/backends/optimized_cpu/optimized_cpu_ellipsoid_mesh_packet.h"
#include "simulation/backends/optimized_cpu/optimized_cpu_static_bvh.h"
#include "simulation/backends/optimized_cpu/optimized_cpu_static_scene_fingerprint.h"

struct CPlugSurface;
struct CPlugTree;

class OptimizedCpuMovingEllipsoidPacketPlan {
public:
    static constexpr std::size_t MaxNodeCount = 32u;
    static constexpr std::size_t MaxLaneCount = 8u;
    static constexpr std::size_t MaxOperationCount =
            MaxNodeCount + MaxLaneCount;
    static constexpr std::uint8_t NoParent = 0xffu;

    struct Node {
        const CPlugTree *tree = nullptr;
        std::uint8_t parentNodeIndex = NoParent;
        bool usesLocalTransform = false;
    };

    struct Lane {
        CPlugTree *tree = nullptr;
        CPlugSurface *surface = nullptr;
        std::uint8_t nodeIndex = 0u;
        u32 temporalSlotOrdinal = 0u;
    };

    enum class OperationKind : std::uint8_t {
        ComposeNode,
        EmitLane,
    };

    struct Operation {
        OperationKind kind = OperationKind::ComposeNode;
        std::uint8_t index = 0u;
    };

    bool TryBuild(const CPlugTree &root) noexcept;
    void Clear(void) noexcept;
    bool IsFor(const CPlugTree &root) const noexcept;

    const Node *NodeData(void) const noexcept {
        return nodes_.data();
    }

    const Lane *LaneData(void) const noexcept {
        return lanes_.data();
    }

    const Operation *OperationData(void) const noexcept {
        return operations_.data();
    }

    std::size_t NodeCount(void) const noexcept {
        return nodeCount_;
    }

    std::size_t LaneCount(void) const noexcept {
        return laneCount_;
    }

    std::size_t OperationCount(void) const noexcept {
        return operationCount_;
    }

    bool IsDirectLaneStar(void) const noexcept {
        return directLaneStar_;
    }

    bool DirectLanesUseLocalTransforms(void) const noexcept {
        return directLanesUseLocalTransforms_;
    }

private:
    bool TryAppendTree(const CPlugTree &tree,
                       std::uint8_t parentNodeIndex,
                       u32 *nextTemporalSlotOrdinal) noexcept;
    bool HasDirectLaneStarTopology(void) const noexcept;

    const CPlugTree *sourceRoot_ = nullptr;
    std::array<Node, MaxNodeCount> nodes_{};
    std::array<Lane, MaxLaneCount> lanes_{};
    std::array<Operation, MaxOperationCount> operations_{};
    std::size_t nodeCount_ = 0u;
    std::size_t laneCount_ = 0u;
    std::size_t operationCount_ = 0u;
    bool directLaneStar_ = false;
    bool directLanesUseLocalTransforms_ = false;
};

class OptimizedCpuStaticSurfaceTransformGroup {
public:
    struct TemporalCandidateSpan {
        const u32 *data = nullptr;
        std::size_t size = 0u;
    };

    const CHmsCollisionManagerSColOctreeCell *RecordData(void) const noexcept {
        return sourceRecords_;
    }

    const GmIso4 *InverseData(void) const noexcept {
        return inverses_.data();
    }

    const GmIso4 &InverseAt(u32 staticTreeIndex) const noexcept {
        return inverses_[staticTreeIndex];
    }

    const OptimizedCpuStaticMeshTriangleSidecar *const
            *TriangleSidecarData(void) const noexcept {
        return triangleSidecars_.data();
    }

    const OptimizedCpuStaticMeshTriangleSidecar *TriangleSidecarAt(
            u32 staticTreeIndex) const noexcept {
        return triangleSidecars_[staticTreeIndex];
    }

    const OptimizedCpuCertifiedStaticMeshPacket *CertifiedMeshPacketAt(
            u32 staticTreeIndex) const noexcept {
        const OptimizedCpuCertifiedStaticMeshPacket &packet =
                certifiedMeshPackets_[staticTreeIndex];
        return packet.IsAvailable() ? &packet : nullptr;
    }

    bool TemporalCandidateSpanFor(
            const CPlugTree &movingTree,
            u32 temporalSlotOrdinal,
            const GmBoxAligned &movingBounds,
            TemporalCandidateSpan *result) const noexcept;
    bool WholeTreeBoundsOverlapAnySurface(
            const GmBoxAligned &movingBounds) const noexcept;
    bool BroadPhaseArithmeticIsBoundedFor(
            const CPlugTree &movingTree,
            const GmIso4 &movingIso) const noexcept;
    bool ShouldRefreshWholePassPrediction(
            const CPlugTree &movingTree) const noexcept;
    void ObserveWholePassResult(
            const CPlugTree &movingTree,
            bool empty) const noexcept;
    void ClearTemporalCandidates(void) const noexcept;

private:
    friend class OptimizedCpuStaticSurfaceTransformCache;

    struct TemporalCandidateEntry {
        const CPlugTree *movingTree = nullptr;
        GmBoxAligned validityBounds{};
        std::vector<u32> candidateRecordIndices;
    };

    TemporalCandidateEntry *TemporalCandidateFallbackFor(
            const CPlugTree &movingTree,
            bool *entryTagMatches) const noexcept;

    const CHmsCollisionManagerSGroup *sourceGroup_ = nullptr;
    const CHmsCollisionManagerSColOctreeCell *sourceRecords_ = nullptr;
    std::size_t sourceRecordCount_ = 0u;
    std::vector<GmIso4> inverses_;
    std::vector<const OptimizedCpuStaticMeshTriangleSidecar *>
            triangleSidecars_;
    std::vector<OptimizedCpuCertifiedStaticMeshPacket>
            certifiedMeshPackets_;
    OptimizedCpuStaticBvh surfaceBvh_;
    bool staticBroadPhaseArithmeticIsBounded_ = false;
    mutable std::array<const CPlugTree *, 8u> boundedMovingTrees_{};
    mutable std::size_t boundedMovingTreeCount_ = 0u;
    struct WholePassPredictionEntry {
        const CPlugTree *movingTree = nullptr;
        std::uint8_t passesUntilRefresh = 0u;
        bool predictedEmpty = true;
    };
    mutable std::array<WholePassPredictionEntry, 8u>
            wholePassPredictions_{};
    mutable std::array<TemporalCandidateEntry, 64u>
            ordinalTemporalCandidates_{};
    mutable std::vector<TemporalCandidateEntry> temporalCandidates_;
};

class OptimizedCpuStaticSurfaceTransformCache {
public:
    OptimizedCpuStaticSurfaceTransformCache(void) = default;
    OptimizedCpuStaticSurfaceTransformCache(
            OptimizedCpuStaticSurfaceTransformCache &&) = default;
    OptimizedCpuStaticSurfaceTransformCache &operator=(
            OptimizedCpuStaticSurfaceTransformCache &&) = default;
    ~OptimizedCpuStaticSurfaceTransformCache(void);

    OptimizedCpuStaticSurfaceTransformCache(
            const OptimizedCpuStaticSurfaceTransformCache &) = delete;
    OptimizedCpuStaticSurfaceTransformCache &operator=(
            const OptimizedCpuStaticSurfaceTransformCache &) = delete;

    bool TryRebuild(
            const CHmsCollisionManagerSZone &zone) noexcept;
    void Clear(void) noexcept;
    void ClearTemporalCandidates(void) noexcept;
    bool CertifyForAdvance(
            const CHmsCollisionManagerSZone &zone,
            const CPlugTree *movingTree = nullptr) noexcept;
    bool CertifyForRuntimeAdvance(
            const CHmsCollisionManagerSZone &zone,
            const CPlugTree *movingTree) noexcept;
    void ClearRuntimeTemporalCandidates(
            const CHmsCollisionManagerSZone &zone,
            const CPlugTree *movingTree) noexcept;
    bool IsFor(const CHmsCollisionManagerSZone &zone) const noexcept;
    bool IsCertifiedFor(
            const CHmsCollisionManagerSZone &zone) const noexcept;
    const OptimizedCpuStaticSurfaceTransformGroup *GroupFor(
            const CHmsCollisionManagerSGroup &group) const noexcept;
    std::optional<OptimizedCpuStaticSceneFingerprint>
            CaptureSourceFingerprintForTesting(void) const noexcept;
    const OptimizedCpuMovingEllipsoidPacketPlan *MovingEllipsoidPacketPlanFor(
            const CPlugTree &movingTree) const noexcept;

private:
    const CHmsCollisionManagerSZone *sourceZone_ = nullptr;
    const CHmsCollisionManagerSZone *certifiedZone_ = nullptr;
    const CPlugTree *certifiedMovingTree_ = nullptr;
    std::array<OptimizedCpuStaticSurfaceTransformGroup,
               CHmsCollisionManager_GroupCount> groups_{};
    std::vector<std::unique_ptr<OptimizedCpuStaticMeshTriangleSidecar>>
            triangleSidecars_;
    std::vector<const GmSurfMesh *> unavailableTriangleSidecarMeshes_;
    OptimizedCpuMovingEllipsoidPacketPlan movingEllipsoidPacketPlan_;
};

// Focused differential hook for the fixed-width direct-star collector. This
// invokes the same production lane path and exposes only its transformed
// bounds, allowing exact arithmetic and floating-status comparison against
// eight scalar GmBoxAligned::SetMult calls.
bool OptimizedCpuCollectDirectLaneStarBoundsForDifferential(
        const GmIso4 &movingIso,
        const OptimizedCpuMovingEllipsoidPacketPlan &movingPlan,
        bool boundsArithmeticIsBounded,
        std::array<GmBoxAligned,
                   OptimizedCpuMovingEllipsoidPacketPlan::MaxLaneCount>
                *bounds) noexcept;
