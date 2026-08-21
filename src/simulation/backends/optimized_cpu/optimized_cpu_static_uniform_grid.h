#pragma once

#include <cstddef>
#include <vector>

#include "engine/core/gm_types.h"

class OptimizedCpuStaticUniformGrid {
public:
    struct Entry {
        u32 sourceIndex = 0u;
        GmBoxAligned bounds{};
    };

    struct CandidateSpan {
        const u32 *data = nullptr;
        std::size_t size = 0u;
    };

    bool TryBuild(const std::vector<Entry> &entries,
                  std::size_t sourceCount,
                  u32 maximumCellCount) noexcept;
    void Clear(void) noexcept;

    // A successful span is one immutable posting list in original flattened
    // tree order. False selects the authoritative octree traversal.
    bool DirectCandidateSpan(const GmBoxAligned &query,
                             CandidateSpan *result) const noexcept;

    // The caller has already certified a finite query with nonnegative
    // extents, an available grid, and a non-null result. This is the
    // packet-collision hot path; the ordinary entry point above remains
    // defensive for general callers.
    bool DirectCandidateSpanForCertifiedQuery(
            const GmBoxAligned &query,
            CandidateSpan *result) const noexcept;

    bool IsAvailable(void) const noexcept {
        return !looseLevels_.empty();
    }

private:
    struct LooseLevel {
        double halfCellSize = 0.0;
        double inverseCellSize = 0.0;
        u32 dimensionX = 0u;
        u32 dimensionY = 0u;
        u32 dimensionZ = 0u;
        std::vector<u32> offsets;
        std::vector<u32> entries;
    };

    double originX_ = 0.0;
    double originY_ = 0.0;
    double originZ_ = 0.0;
    int baseHalfCellExponent_ = 0;
    std::vector<LooseLevel> looseLevels_;
};
