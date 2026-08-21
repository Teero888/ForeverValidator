#include "simulation/backends/optimized_cpu/optimized_cpu_vehicle_collision_bounds_plan.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <typeinfo>

#include "engine/rendering/plug_tree.h"
#include "simulation/backends/optimized_cpu/optimized_cpu_ellipsoid_mesh_packet.h"

#if (defined(__i386__) || defined(__x86_64__)) && \
        (defined(__GNUC__) || defined(__clang__))
#include <immintrin.h>
#endif

namespace forevervalidator::simulation {

namespace {

inline float OrderedRowProduct(float x,
                               float xValue,
                               float y,
                               float yValue,
                               float z,
                               float zValue) noexcept {
    const float xy = x * xValue + y * yValue;
    return xy + z * zValue;
}

inline void TransformCertifiedBox(GmBoxAligned &output,
                                  const GmBoxAligned &source,
                                  const GmIso4 &transform) noexcept {
    const GmMat3 &rotation = transform.rotation;
    const GmVec3 &sourceCenter = source.center;
    const GmVec3 &sourceHalf = source.halfExtents;

    const float centerX = OrderedRowProduct(
            rotation.basisX.x, sourceCenter.x,
            rotation.basisY.x, sourceCenter.y,
            rotation.basisZ.x, sourceCenter.z);
    const float centerY = OrderedRowProduct(
            rotation.basisX.y, sourceCenter.x,
            rotation.basisY.y, sourceCenter.y,
            rotation.basisZ.y, sourceCenter.z);
    const float centerZ = OrderedRowProduct(
            rotation.basisX.z, sourceCenter.x,
            rotation.basisY.z, sourceCenter.y,
            rotation.basisZ.z, sourceCenter.z);
    output.center = {
        centerX + transform.translation.x,
        centerY + transform.translation.y,
        centerZ + transform.translation.z,
    };

    output.halfExtents = {
        OrderedRowProduct(
                std::fabs(rotation.basisX.x), sourceHalf.x,
                std::fabs(rotation.basisY.x), sourceHalf.y,
                std::fabs(rotation.basisZ.x), sourceHalf.z),
        OrderedRowProduct(
                std::fabs(rotation.basisX.y), sourceHalf.x,
                std::fabs(rotation.basisY.y), sourceHalf.y,
                std::fabs(rotation.basisZ.y), sourceHalf.z),
        OrderedRowProduct(
                std::fabs(rotation.basisX.z), sourceHalf.x,
                std::fabs(rotation.basisY.z), sourceHalf.y,
                std::fabs(rotation.basisZ.z), sourceHalf.z),
    };
}

#if (defined(__i386__) || defined(__x86_64__)) && \
        (defined(__GNUC__) || defined(__clang__))
__attribute__((target("avx2"), always_inline))
inline __m256 OrderedVectorProduct(__m256 x,
                                   __m256 xValue,
                                   __m256 y,
                                   __m256 yValue,
                                   __m256 z,
                                   __m256 zValue) noexcept {
    const __m256 xy = _mm256_add_ps(
            _mm256_mul_ps(x, xValue),
            _mm256_mul_ps(y, yValue));
    return _mm256_add_ps(xy, _mm256_mul_ps(z, zValue));
}

__attribute__((target("avx2"), always_inline))
inline __m256 GatherFloatLanes(const float *base,
                               __m256i byteOffsets,
                               std::size_t byteOffset) noexcept {
    const auto *componentBase = reinterpret_cast<const float *>(
            reinterpret_cast<const unsigned char *>(base) + byteOffset);
    return _mm256_i32gather_ps(componentBase, byteOffsets, 1);
}
#endif

// The plan only accepts geometry boxes that are valid for tree refresh.
// GmBoxAligned::SetMult applies absolute rotation elements to non-negative
// half-extents, so a finite transformed child remains valid. Preserve the
// authoritative ordered union arithmetic while avoiding the generic invalid-
// box guards and out-of-line call on every certified child.
inline void AddCertifiedBox(GmBoxAligned &merged,
                            const GmBoxAligned &other) noexcept {
    if (other.halfExtents.x < 0.0f) {
        // Defensive parity for a corrupted runtime transform. The certified
        // vehicle path never takes this branch.
        merged.AddValidPlugTreeBox(other);
        return;
    }

    GmVec3 minPoint = {
        merged.center.x - merged.halfExtents.x,
        merged.center.y - merged.halfExtents.y,
        merged.center.z - merged.halfExtents.z,
    };
    GmVec3 maxPoint = {
        merged.center.x + merged.halfExtents.x,
        merged.center.y + merged.halfExtents.y,
        merged.center.z + merged.halfExtents.z,
    };
    float otherX = other.center.x - other.halfExtents.x;
    float otherY = other.center.y - other.halfExtents.y;
    float otherZ = other.center.z - other.halfExtents.z;
    if (minPoint.x > otherX) {
        minPoint.x = otherX;
    }
    if (minPoint.y > otherY) {
        minPoint.y = otherY;
    }
    if (minPoint.z > otherZ) {
        minPoint.z = otherZ;
    }

    otherX = other.center.x + other.halfExtents.x;
    otherY = other.center.y + other.halfExtents.y;
    otherZ = other.center.z + other.halfExtents.z;
    if (maxPoint.x < otherX) {
        maxPoint.x = otherX;
    }
    if (maxPoint.y < otherY) {
        maxPoint.y = otherY;
    }
    if (maxPoint.z < otherZ) {
        maxPoint.z = otherZ;
    }
    merged.SetMinMax(minPoint, maxPoint);
}

}  // namespace

bool OptimizedCpuVehicleCollisionBoundsPlan::TryBuild(
        CPlugTree &root) noexcept {
    OptimizedCpuVehicleCollisionBoundsPlan candidate;
    const std::size_t childCount = root.GetChildCount();
    if (typeid(root) != typeid(CPlugTree) ||
        root.Visual() != nullptr || root.Surface() != nullptr ||
        childCount < 2u || childCount > MaxChildCount) {
        Clear();
        return false;
    }

    candidate.root_ = &root;
    candidate.childCount_ = childCount;
    for (std::size_t childIndex = 0u;
         childIndex < childCount;
         ++childIndex) {
        CPlugTree *child = root.GetChild(childIndex);
        CPlugSurface *surface = child != nullptr ? child->Surface() : nullptr;
        CPlugSurfaceGeom *geometry =
                surface != nullptr ? surface->GeometryNode() : nullptr;
        if (child == nullptr || typeid(*child) != typeid(CPlugTree) ||
            child->GetChildCount() != 0u || child->Visual() != nullptr ||
            surface == nullptr || geometry == nullptr ||
            !geometry->Bounds().IsValidForPlugTreeRefresh()) {
            Clear();
            return false;
        }
        candidate.children_[childIndex] = {
            child,
            surface,
            geometry,
            geometry->Bounds(),
        };
    }
    if (childCount == 8u) {
        std::array<std::uintptr_t, 8u> localIsoAddresses{};
        std::uintptr_t minimumLocalIsoAddress =
                std::numeric_limits<std::uintptr_t>::max();
        for (std::size_t childIndex = 0u;
             childIndex < childCount;
             ++childIndex) {
            const GmBoxAligned &bounds =
                    candidate.children_[childIndex].geometryBounds;
            candidate.directLaneSnapshot_.trees[childIndex] =
                    candidate.children_[childIndex].tree;
            candidate.eightChildGeometryBounds_.centerX[childIndex] =
                    bounds.center.x;
            candidate.eightChildGeometryBounds_.centerY[childIndex] =
                    bounds.center.y;
            candidate.eightChildGeometryBounds_.centerZ[childIndex] =
                    bounds.center.z;
            candidate.eightChildGeometryBounds_.halfX[childIndex] =
                    bounds.halfExtents.x;
            candidate.eightChildGeometryBounds_.halfY[childIndex] =
                    bounds.halfExtents.y;
            candidate.eightChildGeometryBounds_.halfZ[childIndex] =
                    bounds.halfExtents.z;
            const std::uintptr_t localIsoAddress =
                    reinterpret_cast<std::uintptr_t>(
                            &candidate.children_[childIndex].tree->LocalIso());
            localIsoAddresses[childIndex] = localIsoAddress;
            if (localIsoAddress < minimumLocalIsoAddress) {
                minimumLocalIsoAddress = localIsoAddress;
            }
        }
        bool localIsoOffsetsFit = true;
        for (std::size_t childIndex = 0u;
             childIndex < childCount;
             ++childIndex) {
            const std::uintptr_t offset =
                    localIsoAddresses[childIndex] - minimumLocalIsoAddress;
            if (offset > static_cast<std::uintptr_t>(
                                 std::numeric_limits<std::int32_t>::max())) {
                localIsoOffsetsFit = false;
                break;
            }
            candidate.eightChildLocalIsoGatherOffsets_[childIndex] =
                    static_cast<std::int32_t>(offset);
        }
        if (localIsoOffsetsFit) {
            candidate.eightChildLocalIsoGatherBase_ =
                    reinterpret_cast<const float *>(
                            minimumLocalIsoAddress);
        }
        candidate.eightChildAvx2Available_ =
                localIsoOffsetsFit &&
                OptimizedCpuEllipsoidMeshPacketAvailable();
    }
    *this = candidate;
    return true;
}

bool OptimizedCpuVehicleCollisionBoundsPlan::TryRefresh(void) const noexcept {
    directLaneSnapshotValid_ = false;
    if (!IsAvailable() || root_->GetChildCount() != childCount_ ||
        root_->Visual() != nullptr || root_->Surface() != nullptr) {
        return false;
    }

    // Prove every source and operation target before changing any box. If a
    // caller mutates topology or geometry, the authoritative recursive path
    // can therefore run without needing to repair a partial transition.
    for (std::size_t childIndex = 0u;
         childIndex < childCount_;
         ++childIndex) {
        const Child &child = children_[childIndex];
        if (root_->GetChild(childIndex) != child.tree ||
            child.tree->GetChildCount() != 0u ||
            child.tree->Visual() != nullptr ||
            child.tree->Surface() != child.surface ||
            child.surface->GeometryNode() != child.geometry ||
            std::memcmp(&child.geometry->Bounds(),
                        &child.geometryBounds,
                        sizeof(GmBoxAligned)) != 0) {
            return false;
        }
    }

    RefreshUnchecked();
    return true;
}

void OptimizedCpuVehicleCollisionBoundsPlan::
RefreshRuntimeCertified(void) const noexcept {
    RefreshUnchecked();
}

void OptimizedCpuVehicleCollisionBoundsPlan::RefreshUnchecked(
        void) const noexcept {
    directLaneSnapshotValid_ = false;
#if (defined(__i386__) || defined(__x86_64__)) && \
        (defined(__GNUC__) || defined(__clang__))
    if (childCount_ == 8u && eightChildAvx2Available_) {
        bool allChildrenUseLocalTransforms = true;
        for (std::size_t childIndex = 0u;
             childIndex < childCount_;
             ++childIndex) {
            allChildrenUseLocalTransforms =
                    allChildrenUseLocalTransforms &&
                    children_[childIndex].tree->HasLocalTransform();
        }
        if (allChildrenUseLocalTransforms) {
            RefreshEightChildrenAvx2();
            return;
        }
    }
#endif
    GmBoxAligned merged;
    for (std::size_t childIndex = 0u;
         childIndex < childCount_;
         ++childIndex) {
        const Child &child = children_[childIndex];
        GmBoxAligned childBounds;
        if (child.tree->HasLocalTransform()) {
            TransformCertifiedBox(
                    childBounds, child.geometryBounds, child.tree->LocalIso());
        } else {
            childBounds = child.geometryBounds;
        }
        child.tree->SetTreeBounds(childBounds);
        if (childIndex == 0u) {
            merged = childBounds;
        } else {
            AddCertifiedBox(merged, childBounds);
        }
    }

    GmBoxAligned rootBounds;
    if (root_->HasLocalTransform()) {
        TransformCertifiedBox(rootBounds, merged, root_->LocalIso());
    } else {
        rootBounds = merged;
    }
    root_->SetTreeBounds(rootBounds);
}

#if (defined(__i386__) || defined(__x86_64__)) && \
        (defined(__GNUC__) || defined(__clang__))
__attribute__((target("avx2")))
void OptimizedCpuVehicleCollisionBoundsPlan::RefreshEightChildrenAvx2(
        void) const noexcept {
    DirectLaneSnapshot &snapshot = directLaneSnapshot_;
    const __m256i localIsoOffsets = _mm256_load_si256(
            reinterpret_cast<const __m256i *>(
                    eightChildLocalIsoGatherOffsets_.data()));
    constexpr std::size_t RotationOffset = offsetof(GmIso4, rotation);
    constexpr std::size_t TranslationOffset =
            offsetof(GmIso4, translation);
    constexpr std::size_t BasisXOffset = offsetof(GmMat3, basisX);
    constexpr std::size_t BasisYOffset = offsetof(GmMat3, basisY);
    constexpr std::size_t BasisZOffset = offsetof(GmMat3, basisZ);
    constexpr std::size_t XOffset = offsetof(GmVec3, x);
    constexpr std::size_t YOffset = offsetof(GmVec3, y);
    constexpr std::size_t ZOffset = offsetof(GmVec3, z);
    const __m256 rotationXx = GatherFloatLanes(
            eightChildLocalIsoGatherBase_, localIsoOffsets,
            RotationOffset + BasisXOffset + XOffset);
    const __m256 rotationXy = GatherFloatLanes(
            eightChildLocalIsoGatherBase_, localIsoOffsets,
            RotationOffset + BasisXOffset + YOffset);
    const __m256 rotationXz = GatherFloatLanes(
            eightChildLocalIsoGatherBase_, localIsoOffsets,
            RotationOffset + BasisXOffset + ZOffset);
    const __m256 rotationYx = GatherFloatLanes(
            eightChildLocalIsoGatherBase_, localIsoOffsets,
            RotationOffset + BasisYOffset + XOffset);
    const __m256 rotationYy = GatherFloatLanes(
            eightChildLocalIsoGatherBase_, localIsoOffsets,
            RotationOffset + BasisYOffset + YOffset);
    const __m256 rotationYz = GatherFloatLanes(
            eightChildLocalIsoGatherBase_, localIsoOffsets,
            RotationOffset + BasisYOffset + ZOffset);
    const __m256 rotationZx = GatherFloatLanes(
            eightChildLocalIsoGatherBase_, localIsoOffsets,
            RotationOffset + BasisZOffset + XOffset);
    const __m256 rotationZy = GatherFloatLanes(
            eightChildLocalIsoGatherBase_, localIsoOffsets,
            RotationOffset + BasisZOffset + YOffset);
    const __m256 rotationZz = GatherFloatLanes(
            eightChildLocalIsoGatherBase_, localIsoOffsets,
            RotationOffset + BasisZOffset + ZOffset);
    const __m256 translationX = GatherFloatLanes(
            eightChildLocalIsoGatherBase_, localIsoOffsets,
            TranslationOffset + XOffset);
    const __m256 translationY = GatherFloatLanes(
            eightChildLocalIsoGatherBase_, localIsoOffsets,
            TranslationOffset + YOffset);
    const __m256 translationZ = GatherFloatLanes(
            eightChildLocalIsoGatherBase_, localIsoOffsets,
            TranslationOffset + ZOffset);
    _mm256_store_ps(snapshot.locationXx.data(), rotationXx);
    _mm256_store_ps(snapshot.locationXy.data(), rotationXy);
    _mm256_store_ps(snapshot.locationXz.data(), rotationXz);
    _mm256_store_ps(snapshot.locationYx.data(), rotationYx);
    _mm256_store_ps(snapshot.locationYy.data(), rotationYy);
    _mm256_store_ps(snapshot.locationYz.data(), rotationYz);
    _mm256_store_ps(snapshot.locationZx.data(), rotationZx);
    _mm256_store_ps(snapshot.locationZy.data(), rotationZy);
    _mm256_store_ps(snapshot.locationZz.data(), rotationZz);
    _mm256_store_ps(snapshot.locationTx.data(), translationX);
    _mm256_store_ps(snapshot.locationTy.data(), translationY);
    _mm256_store_ps(snapshot.locationTz.data(), translationZ);

    const __m256 sourceCenterX =
            _mm256_load_ps(eightChildGeometryBounds_.centerX.data());
    const __m256 sourceCenterY =
            _mm256_load_ps(eightChildGeometryBounds_.centerY.data());
    const __m256 sourceCenterZ =
            _mm256_load_ps(eightChildGeometryBounds_.centerZ.data());
    const __m256 sourceHalfX =
            _mm256_load_ps(eightChildGeometryBounds_.halfX.data());
    const __m256 sourceHalfY =
            _mm256_load_ps(eightChildGeometryBounds_.halfY.data());
    const __m256 sourceHalfZ =
            _mm256_load_ps(eightChildGeometryBounds_.halfZ.data());

    _mm256_store_ps(
            snapshot.centerX.data(),
            _mm256_add_ps(
                    OrderedVectorProduct(
                            rotationXx, sourceCenterX,
                            rotationYx, sourceCenterY,
                            rotationZx, sourceCenterZ),
                    translationX));
    _mm256_store_ps(
            snapshot.centerY.data(),
            _mm256_add_ps(
                    OrderedVectorProduct(
                            rotationXy, sourceCenterX,
                            rotationYy, sourceCenterY,
                            rotationZy, sourceCenterZ),
                    translationY));
    _mm256_store_ps(
            snapshot.centerZ.data(),
            _mm256_add_ps(
                    OrderedVectorProduct(
                            rotationXz, sourceCenterX,
                            rotationYz, sourceCenterY,
                            rotationZz, sourceCenterZ),
                    translationZ));

    const __m256 absoluteMask = _mm256_castsi256_ps(
            _mm256_set1_epi32(0x7fffffffu));
    _mm256_store_ps(
            snapshot.halfX.data(),
            OrderedVectorProduct(
                    _mm256_and_ps(rotationXx, absoluteMask), sourceHalfX,
                    _mm256_and_ps(rotationYx, absoluteMask), sourceHalfY,
                    _mm256_and_ps(rotationZx, absoluteMask), sourceHalfZ));
    _mm256_store_ps(
            snapshot.halfY.data(),
            OrderedVectorProduct(
                    _mm256_and_ps(rotationXy, absoluteMask), sourceHalfX,
                    _mm256_and_ps(rotationYy, absoluteMask), sourceHalfY,
                    _mm256_and_ps(rotationZy, absoluteMask), sourceHalfZ));
    _mm256_store_ps(
            snapshot.halfZ.data(),
            OrderedVectorProduct(
                    _mm256_and_ps(rotationXz, absoluteMask), sourceHalfX,
                    _mm256_and_ps(rotationYz, absoluteMask), sourceHalfY,
                    _mm256_and_ps(rotationZz, absoluteMask), sourceHalfZ));

    __m128 mergedCenter = _mm_setzero_ps();
    __m128 mergedHalfExtents = _mm_setzero_ps();
    const __m128 halfScale = _mm_set1_ps(0.5f);
    for (std::size_t childIndex = 0u;
         childIndex < 8u;
         ++childIndex) {
        const float centerX = snapshot.centerX[childIndex];
        const float centerY = snapshot.centerY[childIndex];
        const float centerZ = snapshot.centerZ[childIndex];
        const float halfX = snapshot.halfX[childIndex];
        const float halfY = snapshot.halfY[childIndex];
        const float halfZ = snapshot.halfZ[childIndex];
        const GmBoxAligned childBounds = {
            {centerX, centerY, centerZ},
            {halfX, halfY, halfZ},
        };
        children_[childIndex].tree->SetTreeBounds(childBounds);
        const __m128 childCenter =
                _mm_setr_ps(centerX, centerY, centerZ, 0.0f);
        const __m128 childHalfExtents =
                _mm_setr_ps(halfX, halfY, halfZ, 0.0f);
        if (childIndex == 0u) {
            mergedCenter = childCenter;
            mergedHalfExtents = childHalfExtents;
        } else {
            __m128 minimum =
                    _mm_sub_ps(mergedCenter, mergedHalfExtents);
            __m128 maximum =
                    _mm_add_ps(mergedCenter, mergedHalfExtents);
            const __m128 childMinimum =
                    _mm_sub_ps(childCenter, childHalfExtents);
            const __m128 childMaximum =
                    _mm_add_ps(childCenter, childHalfExtents);
            minimum = _mm_blendv_ps(
                    minimum,
                    childMinimum,
                    _mm_cmpgt_ps(minimum, childMinimum));
            maximum = _mm_blendv_ps(
                    maximum,
                    childMaximum,
                    _mm_cmplt_ps(maximum, childMaximum));
            mergedCenter = _mm_mul_ps(
                    _mm_add_ps(minimum, maximum), halfScale);
            mergedHalfExtents = _mm_mul_ps(
                    _mm_sub_ps(maximum, minimum), halfScale);
        }
    }

    alignas(16) std::array<float, 4u> mergedCenterValues{};
    alignas(16) std::array<float, 4u> mergedHalfExtentValues{};
    _mm_store_ps(mergedCenterValues.data(), mergedCenter);
    _mm_store_ps(mergedHalfExtentValues.data(), mergedHalfExtents);
    const GmBoxAligned merged = {
        {mergedCenterValues[0u],
         mergedCenterValues[1u],
         mergedCenterValues[2u]},
        {mergedHalfExtentValues[0u],
         mergedHalfExtentValues[1u],
         mergedHalfExtentValues[2u]},
    };

    GmBoxAligned rootBounds;
    if (root_->HasLocalTransform()) {
        TransformCertifiedBox(rootBounds, merged, root_->LocalIso());
    } else {
        rootBounds = merged;
    }
    root_->SetTreeBounds(rootBounds);
    directLaneSnapshotValid_ = true;
}
#endif

void OptimizedCpuVehicleCollisionBoundsPlan::Clear(void) noexcept {
    root_ = nullptr;
    childCount_ = 0u;
    eightChildLocalIsoGatherBase_ = nullptr;
    eightChildAvx2Available_ = false;
    directLaneSnapshotValid_ = false;
}

}  // namespace forevervalidator::simulation
