#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <vector>

#include "engine/physics/geometry/gm_surface.h"
#include "simulation/backends/optimized_cpu/optimized_cpu_static_bvh.h"
#include "simulation/backends/optimized_cpu/optimized_cpu_static_uniform_grid.h"

struct OptimizedCpuStaticMeshTriangleSidecarTestAccess;

struct OptimizedCpuStaticMeshTriangleData {
    std::array<GmVec3, 3> vertices{};
    std::array<GmVec3, 3> edgeDirections{};
    std::array<GmVec3, 3> edgeNormals{};
    GmVec3 normal{};
    GmVec3 geometricNormal{};
    GmLocalMaterialIndex material;
};

// Packet collision needs only these fields. Keeping them dense avoids walking
// the 136-byte scalar-query record and fetching its material from the far end.
struct OptimizedCpuStaticMeshPacketTriangleData {
    std::array<GmVec3, 3> vertices{};
    GmLocalMaterialIndex material;
};

struct OptimizedCpuStaticMeshDirectTrianglePosting {
    GmBoxAligned bounds{};
    u32 triangleIndex = 0u;
};

// Compiled only after the source hierarchy has passed topology validation.
// Keep every field consumed by the packet traversal in one 36-byte stream,
// with control metadata first so reject paths fetch it before cell bounds.
struct OptimizedCpuStaticMeshPacketCell {
    u32 subtreeEntryCount = 0u;
    u32 triangleIndex = 0u;
    std::uint8_t depth = 0u;
    std::uint8_t containsTriangle = 0u;
    // One-based index into the optional sibling-group stream. Zero preserves
    // the ordinary per-cell traversal.
    std::uint16_t packetGroupOrdinal = 0u;
    GmBoxAligned bounds{};

    bool ContainsTriangle(void) const noexcept {
        return containsTriangle != 0u;
    }
};

// A conservative certificate for up to eight consecutive sibling
// roots. It stores the minimum and maximum child centers and the maximum child
// half extent independently per axis. The packet traversal can prove that all
// active query boxes reject every member without changing child ordering.
struct OptimizedCpuStaticMeshPacketGroup {
    GmVec4 minimumCenter{};
    GmVec4 maximumCenter{};
    GmVec3 maximumHalfExtents{};
    u32 subtreeEntryCount = 0u;
};

struct OptimizedCpuStaticMeshTriangleHierarchyView {
    const GmMeshOctreeCell *cells = nullptr;
    const std::uint8_t *depths = nullptr;
    const OptimizedCpuStaticMeshPacketCell *packetCells = nullptr;
    const OptimizedCpuStaticMeshPacketGroup *packetGroups = nullptr;
    std::size_t count = 0u;
    std::size_t packetGroupCount = 0u;
    std::size_t maximumTraversalDepth = 0u;
};

bool MeasureOptimizedCpuStaticMeshTraversalDepth(
        const GmMeshOctreeCell *cells,
        std::size_t count,
        std::size_t *maximumDepth) noexcept;

static_assert(std::is_standard_layout_v<
              OptimizedCpuStaticMeshDirectTrianglePosting>);
static_assert(std::is_standard_layout_v<
              OptimizedCpuStaticMeshPacketTriangleData>);
static_assert(offsetof(OptimizedCpuStaticMeshPacketTriangleData, material) ==
              sizeof(GmVec3) * 3u);
static_assert(sizeof(OptimizedCpuStaticMeshPacketTriangleData) == 40u);
static_assert(offsetof(OptimizedCpuStaticMeshDirectTrianglePosting,
                       triangleIndex) == sizeof(GmBoxAligned));
static_assert(sizeof(OptimizedCpuStaticMeshDirectTrianglePosting) ==
              sizeof(GmBoxAligned) + sizeof(u32));
static_assert(alignof(OptimizedCpuStaticMeshDirectTrianglePosting) ==
              alignof(GmBoxAligned));
static_assert(sizeof(OptimizedCpuStaticMeshPacketCell) == 36u);
static_assert(alignof(OptimizedCpuStaticMeshPacketCell) ==
              alignof(GmBoxAligned));
static_assert(offsetof(OptimizedCpuStaticMeshPacketCell,
                       subtreeEntryCount) == 0u);
static_assert(offsetof(OptimizedCpuStaticMeshPacketCell, triangleIndex) == 4u);
static_assert(offsetof(OptimizedCpuStaticMeshPacketCell, depth) == 8u);
static_assert(offsetof(OptimizedCpuStaticMeshPacketCell, containsTriangle) ==
              9u);
static_assert(offsetof(OptimizedCpuStaticMeshPacketCell, packetGroupOrdinal) ==
              10u);
static_assert(offsetof(OptimizedCpuStaticMeshPacketCell, bounds) == 12u);
static_assert(sizeof(OptimizedCpuStaticMeshPacketGroup) == 48u);
static_assert(alignof(OptimizedCpuStaticMeshPacketGroup) ==
              alignof(GmVec3));

class OptimizedCpuStaticMeshTriangleSidecar {
public:
    bool TryBuild(const GmSurfMesh &mesh) noexcept;
    void Clear(void) noexcept;
    bool IsFor(const GmSurfMesh &mesh) const noexcept;

    const OptimizedCpuStaticMeshTriangleData &TriangleAt(
            u32 triangleIndex) const noexcept {
        return triangles_[triangleIndex];
    }

    const OptimizedCpuStaticMeshPacketTriangleData &PacketTriangleAt(
            u32 triangleIndex) const noexcept {
        return packetTriangles_[triangleIndex];
    }

    bool DirectCandidateTriangleSpan(
            const GmBoxAligned &query,
            OptimizedCpuStaticUniformGrid::CandidateSpan *result) const
            noexcept {
        return (triangleGrid_.IsAvailable() &&
                triangleGrid_.DirectCandidateSpanForCertifiedQuery(
                        query, result)) ||
               triangleBvh_.CandidateSpanFor(query, result);
    }

    bool TriangleHierarchyView(
            OptimizedCpuStaticMeshTriangleHierarchyView *result) const
            noexcept {
        if (result == nullptr || sourceCells_ == nullptr ||
            sourceCellCount_ == 0u ||
            traversalDepths_.size() != sourceCellCount_ ||
            packetCells_.size() != sourceCellCount_) {
            return false;
        }
        result->cells = sourceCells_;
        result->depths = traversalDepths_.data();
        result->packetCells = packetCells_.data();
        result->packetGroups = packetGroups_.empty()
                ? nullptr
                : packetGroups_.data();
        result->count = sourceCellCount_;
        result->packetGroupCount = packetGroups_.size();
        result->maximumTraversalDepth = maximumTraversalDepth_;
        return true;
    }

    const OptimizedCpuStaticMeshDirectTrianglePosting &DirectTriangleAt(
            u32 postingIndex) const noexcept {
        return directTrianglePostings_[postingIndex];
    }

    std::size_t DirectTriangleCount(void) const noexcept {
        return directTrianglePostings_.size();
    }

    std::size_t MaximumTraversalDepth(void) const noexcept {
        return maximumTraversalDepth_;
    }

private:
    friend struct OptimizedCpuStaticMeshTriangleSidecarTestAccess;

    const GmSurfMesh *sourceMesh_ = nullptr;
    const GmVec3 *sourceVertices_ = nullptr;
    const GmSurfMeshTriangle *sourceTriangles_ = nullptr;
    const GmMeshOctreeCell *sourceCells_ = nullptr;
    std::size_t sourceVertexCount_ = 0u;
    std::size_t sourceTriangleCount_ = 0u;
    std::size_t sourceCellCount_ = 0u;
    std::size_t maximumTraversalDepth_ = 0u;
    std::vector<std::uint8_t> traversalDepths_;
    std::vector<OptimizedCpuStaticMeshPacketCell> packetCells_;
    std::vector<OptimizedCpuStaticMeshPacketGroup> packetGroups_;
    std::vector<OptimizedCpuStaticMeshPacketTriangleData> packetTriangles_;
    std::vector<OptimizedCpuStaticMeshTriangleData> triangles_;
    std::vector<OptimizedCpuStaticMeshDirectTrianglePosting>
            directTrianglePostings_;
    OptimizedCpuStaticUniformGrid triangleGrid_;
    OptimizedCpuStaticBvh triangleBvh_;
};
