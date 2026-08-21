#include "simulation/backends/optimized_cpu/optimized_cpu_static_surface_transform_cache.h"

#include <cmath>
#include <cstring>
#include <new>
#include <typeinfo>
#include <utility>

#include "engine/physics/geometry/plug_surface.h"
#include "engine/rendering/plug_tree.h"
#include "simulation/backends/optimized_cpu/optimized_cpu_static_bounds_overlap.h"

namespace {

bool IsFiniteTransform(const GmIso4 &transform) noexcept {
    const float *values = reinterpret_cast<const float *>(&transform);
    for (std::size_t index = 0u;
         index < sizeof(transform) / sizeof(float);
         ++index) {
        if (!std::isfinite(values[index])) {
            return false;
        }
    }
    return true;
}

const OptimizedCpuStaticMeshTriangleSidecar *FindTriangleSidecar(
        const std::vector<std::unique_ptr<
                OptimizedCpuStaticMeshTriangleSidecar>> &sidecars,
        const GmSurfMesh &mesh) {
    for (const auto &sidecar : sidecars) {
        if (sidecar->IsFor(mesh)) {
            return sidecar.get();
        }
    }
    return nullptr;
}

bool ContainsMesh(const std::vector<const GmSurfMesh *> &meshes,
                  const GmSurfMesh &mesh) {
    for (const GmSurfMesh *candidate : meshes) {
        if (candidate == &mesh) {
            return true;
        }
    }
    return false;
}

bool IsBoundedBroadPhaseFloat(float value) noexcept {
    std::uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    const std::uint32_t magnitude = bits & 0x7fffffffu;
    if (magnitude == 0u) {
        return true;
    }
    const std::uint32_t exponent = magnitude >> 23u;
    return exponent >= 67u && exponent <= 187u;
}

bool IsBoundedBroadPhaseVector(const GmVec3 &value) noexcept {
    return IsBoundedBroadPhaseFloat(value.x) &&
           IsBoundedBroadPhaseFloat(value.y) &&
           IsBoundedBroadPhaseFloat(value.z);
}

bool IsBoundedBroadPhaseIso(const GmIso4 &value) noexcept {
    return IsBoundedBroadPhaseVector(value.rotation.basisX) &&
           IsBoundedBroadPhaseVector(value.rotation.basisY) &&
           IsBoundedBroadPhaseVector(value.rotation.basisZ) &&
           IsBoundedBroadPhaseVector(value.translation);
}

bool IsBoundedMovingTree(const CPlugTree &tree) noexcept {
    if (!IsBoundedBroadPhaseVector(tree.Box().center) ||
        !IsBoundedBroadPhaseVector(tree.Box().halfExtents) ||
        (tree.HasLocalTransform() &&
         !IsBoundedBroadPhaseIso(tree.LocalIso()))) {
        return false;
    }
    for (u32 childIndex = 0u; childIndex < tree.GetChildCount(); ++childIndex) {
        if (!IsBoundedMovingTree(*tree.GetChild(childIndex))) {
            return false;
        }
    }
    return true;
}

}  // namespace

bool OptimizedCpuMovingEllipsoidPacketPlan::TryBuild(
        const CPlugTree &root) noexcept {
    OptimizedCpuMovingEllipsoidPacketPlan candidate;
    candidate.sourceRoot_ = &root;
    u32 nextTemporalSlotOrdinal = 0u;
    if (!candidate.TryAppendTree(
                root, NoParent, &nextTemporalSlotOrdinal) ||
        candidate.laneCount_ < 2u) {
        Clear();
        return false;
    }
    candidate.directLaneStar_ = candidate.HasDirectLaneStarTopology();
    candidate.directLanesUseLocalTransforms_ = candidate.directLaneStar_;
    for (std::size_t laneIndex = 0u;
         candidate.directLanesUseLocalTransforms_ &&
                 laneIndex < candidate.laneCount_;
         ++laneIndex) {
        candidate.directLanesUseLocalTransforms_ =
                candidate.nodes_[laneIndex + 1u].usesLocalTransform;
    }
    *this = candidate;
    return true;
}

bool OptimizedCpuMovingEllipsoidPacketPlan::
HasDirectLaneStarTopology(void) const noexcept {
    if (sourceRoot_ == nullptr || laneCount_ < 2u ||
        nodeCount_ != laneCount_ + 1u ||
        operationCount_ != laneCount_ * 2u + 1u ||
        nodes_[0u].tree != sourceRoot_ ||
        nodes_[0u].parentNodeIndex != NoParent ||
        operations_[0u].kind != OperationKind::ComposeNode ||
        operations_[0u].index != 0u) {
        return false;
    }
    for (std::size_t laneIndex = 0u;
         laneIndex < laneCount_;
         ++laneIndex) {
        const std::uint8_t nodeIndex = static_cast<std::uint8_t>(
                laneIndex + 1u);
        const Node &node = nodes_[nodeIndex];
        const Lane &lane = lanes_[laneIndex];
        const Operation &compose = operations_[laneIndex * 2u + 1u];
        const Operation &emit = operations_[laneIndex * 2u + 2u];
        if (node.parentNodeIndex != 0u ||
            lane.nodeIndex != nodeIndex ||
            lane.tree != node.tree ||
            compose.kind != OperationKind::ComposeNode ||
            compose.index != nodeIndex ||
            emit.kind != OperationKind::EmitLane ||
            emit.index != laneIndex) {
            return false;
        }
    }
    return true;
}

bool OptimizedCpuMovingEllipsoidPacketPlan::TryAppendTree(
        const CPlugTree &tree,
        std::uint8_t parentNodeIndex,
        u32 *nextTemporalSlotOrdinal) noexcept {
    const u32 temporalSlotOrdinal = (*nextTemporalSlotOrdinal)++;
    if (!tree.HasWorldBox()) {
        return true;
    }
    if (nodeCount_ >= nodes_.size() ||
        operationCount_ >= operations_.size()) {
        return false;
    }

    const std::uint8_t nodeIndex =
            static_cast<std::uint8_t>(nodeCount_++);
    nodes_[nodeIndex] = {
        &tree,
        parentNodeIndex,
        tree.HasLocalTransform() != 0,
    };
    operations_[operationCount_++] = {
        OperationKind::ComposeNode,
        nodeIndex,
    };

    const u32 childCount = tree.GetChildCount();
    for (u32 childIndex = 0u; childIndex < childCount; ++childIndex) {
        CPlugTree *child = tree.GetChild(childIndex);
        if (child == nullptr ||
            !TryAppendTree(
                    *child,
                    nodeIndex,
                    nextTemporalSlotOrdinal)) {
            return false;
        }
    }

    CPlugSurface *surface = tree.Surface();
    if (surface == nullptr) {
        return true;
    }
    const GmSurf *geometry = surface->Geometry();
    if (geometry == nullptr ||
        typeid(*geometry) != typeid(GmSurfEllipsoid) ||
        !surface->UsesSphereContactBuffer() ||
        laneCount_ >= lanes_.size() ||
        operationCount_ >= operations_.size()) {
        return false;
    }
    const GmVec3 radii =
            static_cast<const GmSurfEllipsoid &>(*geometry).radii;
    if (!(0.0f < radii.x && 0.0f < radii.y && 0.0f < radii.z) ||
        !std::isfinite(radii.x) || !std::isfinite(radii.y) ||
        !std::isfinite(radii.z)) {
        return false;
    }

    const std::uint8_t laneIndex =
            static_cast<std::uint8_t>(laneCount_++);
    lanes_[laneIndex] = {
        const_cast<CPlugTree *>(&tree),
        surface,
        nodeIndex,
        temporalSlotOrdinal,
    };
    operations_[operationCount_++] = {
        OperationKind::EmitLane,
        laneIndex,
    };
    return true;
}

void OptimizedCpuMovingEllipsoidPacketPlan::Clear(void) noexcept {
    sourceRoot_ = nullptr;
    nodeCount_ = 0u;
    laneCount_ = 0u;
    operationCount_ = 0u;
    directLaneStar_ = false;
    directLanesUseLocalTransforms_ = false;
}

bool OptimizedCpuMovingEllipsoidPacketPlan::IsFor(
        const CPlugTree &root) const noexcept {
    return sourceRoot_ == &root && laneCount_ >= 2u;
}

bool OptimizedCpuStaticSurfaceTransformGroup::
WholeTreeBoundsOverlapAnySurface(
        const GmBoxAligned &movingBounds) const noexcept {
    if (sourceRecords_ == nullptr || sourceRecordCount_ == 0u) {
        return true;
    }

    if (surfaceBvh_.IsAvailable()) {
        return surfaceBvh_.OverlapsAny(movingBounds);
    }

    u32 sourceIndex = 0u;
    while (sourceIndex < sourceRecordCount_) {
        const CHmsCollisionManagerSColOctreeCell &record =
                sourceRecords_[sourceIndex];
        if (!forevervalidator::simulation::OptimizedCpuStaticBoundsOverlap(
                    movingBounds, record.Bounds())) {
            sourceIndex += record.SubtreeEntryCount();
            continue;
        }
        if (record.ContainsSurface()) {
            return true;
        }
        ++sourceIndex;
    }
    return false;
}

bool OptimizedCpuStaticSurfaceTransformGroup::
BroadPhaseArithmeticIsBoundedFor(
        const CPlugTree &movingTree,
        const GmIso4 &movingIso) const noexcept {
    if (!staticBroadPhaseArithmeticIsBounded_ ||
        !IsBoundedBroadPhaseIso(movingIso)) {
        return false;
    }
    for (std::size_t index = 0u;
         index < boundedMovingTreeCount_;
         ++index) {
        if (boundedMovingTrees_[index] == &movingTree) {
            return true;
        }
    }
    if (!IsBoundedMovingTree(movingTree) ||
        boundedMovingTreeCount_ == boundedMovingTrees_.size()) {
        return false;
    }
    boundedMovingTrees_[boundedMovingTreeCount_++] = &movingTree;
    return true;
}

bool OptimizedCpuStaticSurfaceTransformGroup::
ShouldRefreshWholePassPrediction(
        const CPlugTree &movingTree) const noexcept {
    WholePassPredictionEntry *freeEntry = nullptr;
    for (WholePassPredictionEntry &entry : wholePassPredictions_) {
        if (entry.movingTree == &movingTree) {
            if (entry.predictedEmpty || entry.passesUntilRefresh == 0u) {
                return true;
            }
            --entry.passesUntilRefresh;
            return false;
        }
        if (entry.movingTree == nullptr && freeEntry == nullptr) {
            freeEntry = &entry;
        }
    }
    if (freeEntry == nullptr) {
        return false;
    }
    freeEntry->movingTree = &movingTree;
    return true;
}

void OptimizedCpuStaticSurfaceTransformGroup::ObserveWholePassResult(
        const CPlugTree &movingTree,
        bool empty) const noexcept {
    for (WholePassPredictionEntry &entry : wholePassPredictions_) {
        if (entry.movingTree != &movingTree) {
            continue;
        }
        entry.predictedEmpty = empty;
        entry.passesUntilRefresh = empty ? 0u : 7u;
        return;
    }
}

bool OptimizedCpuStaticSurfaceTransformGroup::TemporalCandidateSpanFor(
        const CPlugTree &movingTree,
        u32 temporalSlotOrdinal,
        const GmBoxAligned &movingBounds,
        TemporalCandidateSpan *result) const noexcept {
    if (result == nullptr || sourceRecords_ == nullptr ||
        sourceRecordCount_ == 0u) {
        return false;
    }

    auto contains = [](const GmBoxAligned &outer,
                       const GmBoxAligned &inner) {
        return inner.center.x - inner.halfExtents.x >=
                       outer.center.x - outer.halfExtents.x &&
               inner.center.x + inner.halfExtents.x <=
                       outer.center.x + outer.halfExtents.x &&
               inner.center.y - inner.halfExtents.y >=
                       outer.center.y - outer.halfExtents.y &&
               inner.center.y + inner.halfExtents.y <=
                       outer.center.y + outer.halfExtents.y &&
               inner.center.z - inner.halfExtents.z >=
                       outer.center.z - outer.halfExtents.z &&
               inner.center.z + inner.halfExtents.z <=
                       outer.center.z + outer.halfExtents.z;
    };

    TemporalCandidateEntry *entry = nullptr;
    bool entryTagMatches = false;
    if (temporalSlotOrdinal < ordinalTemporalCandidates_.size()) {
        entry = &ordinalTemporalCandidates_[temporalSlotOrdinal];
        if (entry->movingTree == &movingTree) {
            entryTagMatches = true;
        } else if (entry->movingTree == nullptr) {
            entry->movingTree = &movingTree;
        } else {
            entry = nullptr;
        }
    }
    if (entry == nullptr) {
        entry = TemporalCandidateFallbackFor(
                movingTree, &entryTagMatches);
        if (entry == nullptr) {
            return false;
        }
    }
    if (entryTagMatches &&
        contains(entry->validityBounds, movingBounds)) {
        result->size = entry->candidateRecordIndices.size();
        result->data = result->size == 0u
                ? nullptr
                : entry->candidateRecordIndices.data();
        return true;
    }

    try {
        constexpr float FatMargin = 8.0f;
        entry->validityBounds = movingBounds;
        entry->validityBounds.halfExtents.x += FatMargin;
        entry->validityBounds.halfExtents.y += FatMargin;
        entry->validityBounds.halfExtents.z += FatMargin;
        entry->candidateRecordIndices.clear();

        OptimizedCpuStaticUniformGrid::CandidateSpan acceleratedSpan;
        if (surfaceBvh_.CandidateSpanFor(
                    entry->validityBounds, &acceleratedSpan)) {
            if (acceleratedSpan.size != 0u) {
                entry->candidateRecordIndices.assign(
                        acceleratedSpan.data,
                        acceleratedSpan.data + acceleratedSpan.size);
            }
        } else {
            u32 sourceIndex = 0u;
            while (sourceIndex < sourceRecordCount_) {
                const CHmsCollisionManagerSColOctreeCell &record =
                        sourceRecords_[sourceIndex];
                if (!forevervalidator::simulation::
                            OptimizedCpuStaticBoundsOverlap(
                                    entry->validityBounds,
                                    record.Bounds())) {
                    sourceIndex += record.SubtreeEntryCount();
                    continue;
                }
                if (record.ContainsSurface()) {
                    entry->candidateRecordIndices.push_back(sourceIndex);
                }
                ++sourceIndex;
            }
        }
        result->size = entry->candidateRecordIndices.size();
        result->data = result->size == 0u
                ? nullptr
                : entry->candidateRecordIndices.data();
        return true;
    } catch (const std::bad_alloc &) {
        if (entry != nullptr) {
            entry->movingTree = nullptr;
            entry->candidateRecordIndices.clear();
        }
        return false;
    }
}

#if defined(_MSC_VER)
#define FOREVERVALIDATOR_COLD_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define FOREVERVALIDATOR_COLD_NOINLINE __attribute__((cold, noinline))
#else
#define FOREVERVALIDATOR_COLD_NOINLINE
#endif

FOREVERVALIDATOR_COLD_NOINLINE
OptimizedCpuStaticSurfaceTransformGroup::TemporalCandidateEntry *
OptimizedCpuStaticSurfaceTransformGroup::TemporalCandidateFallbackFor(
        const CPlugTree &movingTree,
        bool *entryTagMatches) const noexcept {
    for (TemporalCandidateEntry &candidate : temporalCandidates_) {
        if (candidate.movingTree == &movingTree) {
            *entryTagMatches = true;
            return &candidate;
        }
    }
    try {
        temporalCandidates_.push_back({});
        TemporalCandidateEntry *entry = &temporalCandidates_.back();
        entry->movingTree = &movingTree;
        *entryTagMatches = false;
        return entry;
    } catch (const std::bad_alloc &) {
        return nullptr;
    }
}

#undef FOREVERVALIDATOR_COLD_NOINLINE

void OptimizedCpuStaticSurfaceTransformGroup::ClearTemporalCandidates(
        void) const noexcept {
    for (TemporalCandidateEntry &entry : ordinalTemporalCandidates_) {
        entry = TemporalCandidateEntry{};
    }
    temporalCandidates_.clear();
}

OptimizedCpuStaticSurfaceTransformCache::
~OptimizedCpuStaticSurfaceTransformCache(void) {
    ClearTemporalCandidates();
}

bool OptimizedCpuStaticSurfaceTransformCache::TryRebuild(
        const CHmsCollisionManagerSZone &zone) noexcept {
    try {
        OptimizedCpuStaticSurfaceTransformCache rebuilt;
        rebuilt.sourceZone_ = &zone;
        for (u32 groupIndex = 0u;
             groupIndex < CHmsCollisionManager_GroupCount;
             ++groupIndex) {
            const CHmsCollisionManagerSGroup *group =
                    zone.GroupAtOrNull(groupIndex);
            OptimizedCpuStaticSurfaceTransformGroup &cachedGroup =
                    rebuilt.groups_[groupIndex];
            cachedGroup.sourceGroup_ = group;
            if (group == nullptr) {
                continue;
            }
            const auto &records = group->staticTrees.Entries();
            cachedGroup.sourceRecords_ = records.data();
            cachedGroup.sourceRecordCount_ = records.size();
            cachedGroup.staticBroadPhaseArithmeticIsBounded_ = true;
            for (const CHmsCollisionManagerSColOctreeCell &record : records) {
                const GmBoxAligned &bounds = record.Bounds();
                if (!IsBoundedBroadPhaseVector(bounds.center) ||
                    !IsBoundedBroadPhaseVector(bounds.halfExtents)) {
                    cachedGroup.staticBroadPhaseArithmeticIsBounded_ = false;
                    break;
                }
            }
            cachedGroup.inverses_.resize(records.size());
            cachedGroup.triangleSidecars_.resize(records.size(), nullptr);
            cachedGroup.certifiedMeshPackets_.resize(records.size());
            std::vector<OptimizedCpuStaticBvh::Entry> surfaceEntries;
            surfaceEntries.reserve(records.size());
            for (std::size_t recordIndex = 0u;
                 recordIndex < records.size();
                 ++recordIndex) {
                if (!records[recordIndex].ContainsSurface()) {
                    continue;
                }
                surfaceEntries.push_back({
                    static_cast<u32>(recordIndex),
                    records[recordIndex].Bounds(),
                });
                const CHmsCollisionManagerSColOctreeCell::StaticSurface
                        &surface = records[recordIndex].SurfaceData();
                cachedGroup.inverses_[recordIndex].SetInverse(
                        surface.location);

                const GmSurf *geometry = surface.surface == nullptr
                        ? nullptr
                        : surface.surface->Geometry();
                if (geometry == nullptr ||
                    typeid(*geometry) != typeid(GmSurfMesh)) {
                    continue;
                }
                const GmSurfMesh &mesh =
                        static_cast<const GmSurfMesh &>(*geometry);
                if (ContainsMesh(
                            rebuilt.unavailableTriangleSidecarMeshes_,
                            mesh)) {
                    continue;
                }
                const OptimizedCpuStaticMeshTriangleSidecar *sidecar =
                        FindTriangleSidecar(
                                rebuilt.triangleSidecars_, mesh);
                if (sidecar == nullptr) {
                    auto candidate = std::make_unique<
                            OptimizedCpuStaticMeshTriangleSidecar>();
                    if (!candidate->TryBuild(mesh)) {
                        rebuilt.unavailableTriangleSidecarMeshes_.push_back(
                                &mesh);
                        continue;
                    }
                    sidecar = candidate.get();
                    rebuilt.triangleSidecars_.push_back(
                            std::move(candidate));
                }
                cachedGroup.triangleSidecars_[recordIndex] = sidecar;
                OptimizedCpuStaticMeshTriangleHierarchyView hierarchy;
                if (IsFiniteTransform(
                            cachedGroup.inverses_[recordIndex]) &&
                    sidecar->TriangleHierarchyView(&hierarchy)) {
                    cachedGroup.certifiedMeshPackets_[recordIndex] = {
                        &mesh,
                        surface.location,
                        cachedGroup.inverses_[recordIndex],
                        sidecar,
                        hierarchy,
                    };
                }
            }
            cachedGroup.surfaceBvh_.TryBuild(
                    surfaceEntries, records.size());
        }
        *this = std::move(rebuilt);
        return true;
    } catch (const std::bad_alloc &) {
        Clear();
        return false;
    }
}

void OptimizedCpuStaticSurfaceTransformCache::Clear(void) noexcept {
    sourceZone_ = nullptr;
    certifiedZone_ = nullptr;
    certifiedMovingTree_ = nullptr;
    movingEllipsoidPacketPlan_.Clear();
    for (OptimizedCpuStaticSurfaceTransformGroup &group : groups_) {
        group.ClearTemporalCandidates();
        group.sourceGroup_ = nullptr;
        group.sourceRecords_ = nullptr;
        group.sourceRecordCount_ = 0u;
        group.staticBroadPhaseArithmeticIsBounded_ = false;
        group.boundedMovingTrees_.fill(nullptr);
        group.boundedMovingTreeCount_ = 0u;
        group.wholePassPredictions_.fill({});
        group.inverses_.clear();
        group.triangleSidecars_.clear();
        group.certifiedMeshPackets_.clear();
        group.surfaceBvh_.Clear();
    }
    triangleSidecars_.clear();
    unavailableTriangleSidecarMeshes_.clear();
}

bool OptimizedCpuStaticSurfaceTransformCache::CertifyForAdvance(
        const CHmsCollisionManagerSZone &zone,
        const CPlugTree *movingTree) noexcept {
    certifiedZone_ = nullptr;
    certifiedMovingTree_ = nullptr;
    movingEllipsoidPacketPlan_.Clear();
    if (sourceZone_ != &zone) {
        return false;
    }

    for (u32 groupIndex = 0u;
         groupIndex < CHmsCollisionManager_GroupCount;
         ++groupIndex) {
        const CHmsCollisionManagerSGroup *group =
                zone.GroupAtOrNull(groupIndex);
        const OptimizedCpuStaticSurfaceTransformGroup &cachedGroup =
                groups_[groupIndex];
        if (cachedGroup.sourceGroup_ != group) {
            return false;
        }
        if (group == nullptr) {
            continue;
        }

        const auto &records = group->staticTrees.Entries();
        if (cachedGroup.sourceRecords_ != records.data() ||
            cachedGroup.sourceRecordCount_ != records.size() ||
            cachedGroup.inverses_.size() != records.size() ||
            cachedGroup.triangleSidecars_.size() != records.size() ||
            cachedGroup.certifiedMeshPackets_.size() != records.size()) {
            return false;
        }
        for (std::size_t recordIndex = 0u;
             recordIndex < records.size();
             ++recordIndex) {
            const CHmsCollisionManagerSColOctreeCell &record =
                    records[recordIndex];
            // Static tree flags remain immutable during the synchronous
            // advance, so cached record loops can omit this dependent load.
            if (record.ContainsSurface() &&
                (record.SurfaceData().tree == nullptr ||
                 !record.SurfaceData().tree->AllowsSurfaceCollision())) {
                return false;
            }
            const OptimizedCpuStaticMeshTriangleSidecar *sidecar =
                    cachedGroup.triangleSidecars_[recordIndex];
            if (sidecar == nullptr) {
                continue;
            }
            if (!record.ContainsSurface()) {
                return false;
            }
            const CHmsCollisionManagerSColOctreeCell::StaticSurface
                    &surface = record.SurfaceData();
            const GmSurf *geometry = surface.surface == nullptr
                    ? nullptr
                    : surface.surface->Geometry();
            if (geometry == nullptr ||
                typeid(*geometry) != typeid(GmSurfMesh) ||
                !sidecar->IsFor(
                        static_cast<const GmSurfMesh &>(*geometry))) {
                return false;
            }
            const OptimizedCpuCertifiedStaticMeshPacket &packet =
                    cachedGroup.certifiedMeshPackets_[recordIndex];
            if (!packet.IsAvailable()) {
                continue;
            }
            OptimizedCpuStaticMeshTriangleHierarchyView hierarchy;
            if (packet.sourceMesh != geometry ||
                packet.triangles != sidecar ||
                std::memcmp(&packet.meshIso,
                            &surface.location,
                            sizeof(GmIso4)) != 0 ||
                std::memcmp(&packet.meshInverse,
                            &cachedGroup.inverses_[recordIndex],
                            sizeof(GmIso4)) != 0 ||
                !sidecar->TriangleHierarchyView(&hierarchy) ||
                packet.hierarchy.cells != hierarchy.cells ||
                packet.hierarchy.depths != hierarchy.depths ||
                packet.hierarchy.packetCells != hierarchy.packetCells ||
                packet.hierarchy.packetGroups != hierarchy.packetGroups ||
                packet.hierarchy.count != hierarchy.count ||
                packet.hierarchy.packetGroupCount !=
                        hierarchy.packetGroupCount ||
                packet.hierarchy.maximumTraversalDepth !=
                        hierarchy.maximumTraversalDepth) {
                return false;
            }
        }
    }

    if (movingTree != nullptr) {
        (void)movingEllipsoidPacketPlan_.TryBuild(*movingTree);
    }
    certifiedZone_ = &zone;
    certifiedMovingTree_ = movingTree;
    return true;
}

bool OptimizedCpuStaticSurfaceTransformCache::CertifyForRuntimeAdvance(
        const CHmsCollisionManagerSZone &zone,
        const CPlugTree *movingTree) noexcept {
    // Replay runtime construction owns the static scene and the vehicle
    // collision-tree topology for the lifetime of this cache. Dynamic clone
    // restore changes transforms and physics state in place, but not either
    // identity. Keep the full CertifyForAdvance path available to callers
    // that can mutate scene data directly.
    if (certifiedZone_ == &zone &&
        certifiedMovingTree_ == movingTree) {
        return true;
    }
    return CertifyForAdvance(zone, movingTree);
}

void OptimizedCpuStaticSurfaceTransformCache::
ClearRuntimeTemporalCandidates(
        const CHmsCollisionManagerSZone &zone,
        const CPlugTree *movingTree) noexcept {
    if (certifiedZone_ != &zone ||
        certifiedMovingTree_ != movingTree) {
        ClearTemporalCandidates();
        return;
    }
    for (OptimizedCpuStaticSurfaceTransformGroup &group : groups_) {
        group.ClearTemporalCandidates();
    }
}

void OptimizedCpuStaticSurfaceTransformCache::ClearTemporalCandidates(
        void) noexcept {
    certifiedZone_ = nullptr;
    certifiedMovingTree_ = nullptr;
    movingEllipsoidPacketPlan_.Clear();
    for (OptimizedCpuStaticSurfaceTransformGroup &group : groups_) {
        group.ClearTemporalCandidates();
    }
}

const OptimizedCpuMovingEllipsoidPacketPlan *
OptimizedCpuStaticSurfaceTransformCache::MovingEllipsoidPacketPlanFor(
        const CPlugTree &movingTree) const noexcept {
    return movingEllipsoidPacketPlan_.IsFor(movingTree)
            ? &movingEllipsoidPacketPlan_
            : nullptr;
}

bool OptimizedCpuStaticSurfaceTransformCache::IsFor(
        const CHmsCollisionManagerSZone &zone) const noexcept {
    return sourceZone_ == &zone;
}

bool OptimizedCpuStaticSurfaceTransformCache::IsCertifiedFor(
        const CHmsCollisionManagerSZone &zone) const noexcept {
    return certifiedZone_ == &zone;
}

const OptimizedCpuStaticSurfaceTransformGroup *
OptimizedCpuStaticSurfaceTransformCache::GroupFor(
        const CHmsCollisionManagerSGroup &group) const noexcept {
    for (const OptimizedCpuStaticSurfaceTransformGroup &candidate : groups_) {
        if (candidate.sourceGroup_ != &group) {
            continue;
        }
        // Static-tree records are immutable after scene construction. Check
        // the vector backing and count once per group pass; any rebuild uses
        // the established uncached OptimizedCpu path for the whole pass.
        const auto &records = group.staticTrees.Entries();
        if (candidate.sourceRecords_ != records.data() ||
            candidate.sourceRecordCount_ != records.size() ||
            candidate.inverses_.size() != records.size() ||
            candidate.triangleSidecars_.size() != records.size()) {
            return nullptr;
        }
        return &candidate;
    }
    return nullptr;
}
