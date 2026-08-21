#include "simulation/backends/optimized_cpu/optimized_cpu_ellipsoid_mesh_packet.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <typeinfo>

#include "engine/physics/collision/gm_collision_buffer.h"
#include "engine/physics/geometry/gm_surface.h"
#include "engine/physics/geometry/physics_tolerances.h"
#include "simulation/backends/optimized_cpu/optimized_cpu_static_mesh_triangle_sidecar.h"

#if (defined(__i386__) || defined(__x86_64__)) && \
        (defined(__GNUC__) || defined(__clang__))
#define FV_E031_HAS_X86_PACKET 1
#include <immintrin.h>
#if defined(_WIN32)
#include <intrin.h>
#endif
#else
#define FV_E031_HAS_X86_PACKET 0
#endif

namespace {

constexpr std::size_t PacketWidth =
        OptimizedCpuPreparedEllipsoidMeshPacket::Width;
// Archived TMNF Stadium meshes in the broad replay corpus peak at depth 20.
// Deeper valid meshes remain exact through the scalar fallback because the
// certified depth is rejected before any packet collision is emitted.
constexpr std::size_t PacketTraversalCapacity = 24u;

bool IsFiniteTransform(const GmIso4 &transform) noexcept {
    const float values[] = {
        transform.rotation.basisX.x,
        transform.rotation.basisX.y,
        transform.rotation.basisX.z,
        transform.rotation.basisY.x,
        transform.rotation.basisY.y,
        transform.rotation.basisY.z,
        transform.rotation.basisZ.x,
        transform.rotation.basisZ.y,
        transform.rotation.basisZ.z,
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    };
    for (float value : values) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    return true;
}

bool BuildPreparedPacket(
        const OptimizedCpuEllipsoidMeshPacketLane *lanes,
        std::size_t laneCount,
        std::uint32_t requestedMask,
        OptimizedCpuPreparedEllipsoidMeshPacket *prepared) noexcept {
    if (lanes == nullptr || prepared == nullptr || laneCount < 2u ||
        laneCount > PacketWidth) {
        return false;
    }
    const std::uint32_t laneMask =
            (1u << static_cast<unsigned int>(laneCount)) - 1u;
    requestedMask &= laneMask;
    OptimizedCpuPreparedEllipsoidMeshPacket candidate{};
    candidate.laneCount = laneCount;
    candidate.preparedMask = requestedMask;
    if (requestedMask == 0u) {
        *prepared = candidate;
        return true;
    }

    for (std::size_t lane = 0u; lane < laneCount; ++lane) {
        if ((requestedMask & (1u << lane)) == 0u) {
            continue;
        }
        const LocatedGmSurf *located = lanes[lane].ellipsoid;
        if (located == nullptr || located->surf == nullptr ||
            located->iso == nullptr ||
            lanes[lane].collisionBuffer == nullptr ||
            typeid(*located->surf) != typeid(GmSurfEllipsoid)) {
            return false;
        }
        const GmSurfEllipsoid &ellipsoid =
                static_cast<const GmSurfEllipsoid &>(*located->surf);
        const GmVec3 radii = ellipsoid.radii;
        if (!(0.0f < radii.x && 0.0f < radii.y && 0.0f < radii.z) ||
            !std::isfinite(radii.x) || !std::isfinite(radii.y) ||
            !std::isfinite(radii.z)) {
            return false;
        }
        const GmIso4 ellipsoidWorld = located->enabled == 0
                ? GmIso4{
                      {{1.0f, 0.0f, 0.0f},
                       {0.0f, 1.0f, 0.0f},
                       {0.0f, 0.0f, 1.0f}},
                      {0.0f, 0.0f, 0.0f}}
                : *located->iso;
        candidate.worldXx.values[lane] =
                ellipsoidWorld.rotation.basisX.x;
        candidate.worldXy.values[lane] =
                ellipsoidWorld.rotation.basisY.x;
        candidate.worldXz.values[lane] =
                ellipsoidWorld.rotation.basisZ.x;
        candidate.worldYx.values[lane] =
                ellipsoidWorld.rotation.basisX.y;
        candidate.worldYy.values[lane] =
                ellipsoidWorld.rotation.basisY.y;
        candidate.worldYz.values[lane] =
                ellipsoidWorld.rotation.basisZ.y;
        candidate.worldZx.values[lane] =
                ellipsoidWorld.rotation.basisX.z;
        candidate.worldZy.values[lane] =
                ellipsoidWorld.rotation.basisY.z;
        candidate.worldZz.values[lane] =
                ellipsoidWorld.rotation.basisZ.z;
        candidate.worldTx.values[lane] = ellipsoidWorld.translation.x;
        candidate.worldTy.values[lane] = ellipsoidWorld.translation.y;
        candidate.worldTz.values[lane] = ellipsoidWorld.translation.z;
        candidate.radiiX.values[lane] = radii.x;
        candidate.radiiY.values[lane] = radii.y;
        candidate.radiiZ.values[lane] = radii.z;
        candidate.inverseRadiiX.values[lane] = 1.0f / radii.x;
        candidate.inverseRadiiY.values[lane] = 1.0f / radii.y;
        candidate.inverseRadiiZ.values[lane] = 1.0f / radii.z;
        candidate.materials[lane] = ellipsoid.material;
        candidate.buffers[lane] = lanes[lane].collisionBuffer;
    }
    *prepared = candidate;
    return true;
}

#if FV_E031_HAS_X86_PACKET

#if defined(__GNUC__) || defined(__clang__)
#define FV_E031_AVX2 __attribute__((target("avx2")))
#define FV_E031_HOT_AVX2 __attribute__((target("avx2"), aligned(32)))
#define FV_E031_INLINE inline __attribute__((always_inline, target("avx2")))
#else
#define FV_E031_AVX2
#define FV_E031_HOT_AVX2
#define FV_E031_INLINE inline
#endif

struct Vec3x8 {
    __m256 x;
    __m256 y;
    __m256 z;
};

struct Mat3x8 {
    __m256 xx, xy, xz;
    __m256 yx, yy, yz;
    __m256 zx, zy, zz;
};

struct Iso3x8 {
    Mat3x8 rotation;
    Vec3x8 translation;
};

struct Boxx8 {
    Vec3x8 center;
    Vec3x8 halfExtents;
};

FV_E031_INLINE __m256 Load(const std::array<float, PacketWidth> &values) {
    return _mm256_load_ps(values.data());
}

FV_E031_INLINE __m256 MaskForBits(std::uint32_t bits) {
    const __m256i bitValues = _mm256_setr_epi32(
            0x01, 0x02, 0x04, 0x08,
            0x10, 0x20, 0x40, 0x80);
    const __m256i selected = _mm256_and_si256(
            _mm256_set1_epi32(static_cast<int>(bits)), bitValues);
    return _mm256_castsi256_ps(
            _mm256_cmpeq_epi32(selected, bitValues));
}

FV_E031_INLINE std::uint32_t Bits(__m256 mask) {
    return static_cast<std::uint32_t>(_mm256_movemask_ps(mask));
}

FV_E031_INLINE __m256 And(__m256 left, __m256 right) {
    return _mm256_and_ps(left, right);
}

FV_E031_INLINE __m256 AndNot(__m256 left, __m256 right) {
    return _mm256_andnot_ps(left, right);
}

FV_E031_INLINE __m256 Or(__m256 left, __m256 right) {
    return _mm256_or_ps(left, right);
}

FV_E031_INLINE __m256 Select(__m256 mask, __m256 yes, __m256 no) {
    return _mm256_blendv_ps(no, yes, mask);
}

FV_E031_INLINE __m256 Abs(__m256 value) {
    return _mm256_andnot_ps(_mm256_set1_ps(-0.0f), value);
}

FV_E031_INLINE __m256 Dot(const Vec3x8 &left, const Vec3x8 &right) {
    const __m256 xy = _mm256_add_ps(
            _mm256_mul_ps(left.x, right.x),
            _mm256_mul_ps(left.y, right.y));
    return _mm256_add_ps(xy, _mm256_mul_ps(left.z, right.z));
}

FV_E031_INLINE Vec3x8 Add(const Vec3x8 &left, const Vec3x8 &right) {
    return {
        _mm256_add_ps(left.x, right.x),
        _mm256_add_ps(left.y, right.y),
        _mm256_add_ps(left.z, right.z),
    };
}

FV_E031_INLINE Vec3x8 Subtract(const Vec3x8 &left,
                               const Vec3x8 &right) {
    return {
        _mm256_sub_ps(left.x, right.x),
        _mm256_sub_ps(left.y, right.y),
        _mm256_sub_ps(left.z, right.z),
    };
}

FV_E031_INLINE Vec3x8 Scale(const Vec3x8 &value, __m256 scale) {
    return {
        _mm256_mul_ps(value.x, scale),
        _mm256_mul_ps(value.y, scale),
        _mm256_mul_ps(value.z, scale),
    };
}

FV_E031_INLINE Vec3x8 Negate(const Vec3x8 &value) {
    const __m256 sign = _mm256_set1_ps(-0.0f);
    return {
        _mm256_xor_ps(value.x, sign),
        _mm256_xor_ps(value.y, sign),
        _mm256_xor_ps(value.z, sign),
    };
}

FV_E031_INLINE Vec3x8 Cross(const Vec3x8 &left, const Vec3x8 &right) {
    return {
        _mm256_sub_ps(_mm256_mul_ps(left.y, right.z),
                      _mm256_mul_ps(left.z, right.y)),
        _mm256_sub_ps(_mm256_mul_ps(left.z, right.x),
                      _mm256_mul_ps(left.x, right.z)),
        _mm256_sub_ps(_mm256_mul_ps(left.x, right.y),
                      _mm256_mul_ps(left.y, right.x)),
    };
}

FV_E031_INLINE Vec3x8 Broadcast(const GmVec3 &value) {
    return {
        _mm256_set1_ps(value.x),
        _mm256_set1_ps(value.y),
        _mm256_set1_ps(value.z),
    };
}

FV_E031_INLINE Vec3x8 ZeroVector(void) {
    const __m256 zero = _mm256_setzero_ps();
    return {zero, zero, zero};
}

FV_E031_INLINE Vec3x8 TransformDirection(const Mat3x8 &matrix,
                                         const Vec3x8 &value) {
    return {
        _mm256_add_ps(
                _mm256_add_ps(_mm256_mul_ps(matrix.xx, value.x),
                              _mm256_mul_ps(matrix.xy, value.y)),
                _mm256_mul_ps(matrix.xz, value.z)),
        _mm256_add_ps(
                _mm256_add_ps(_mm256_mul_ps(matrix.yx, value.x),
                              _mm256_mul_ps(matrix.yy, value.y)),
                _mm256_mul_ps(matrix.yz, value.z)),
        _mm256_add_ps(
                _mm256_add_ps(_mm256_mul_ps(matrix.zx, value.x),
                              _mm256_mul_ps(matrix.zy, value.y)),
                _mm256_mul_ps(matrix.zz, value.z)),
    };
}

FV_E031_INLINE Vec3x8 TransformPoint(const Iso3x8 &transform,
                                     const Vec3x8 &value) {
    return Add(TransformDirection(transform.rotation, value),
               transform.translation);
}

// A component-wise edge bound avoids computing the full squared length in the
// overwhelmingly common clearly-inside case.  Since |edge| <= sqrt(3) *
// maxAbs(edge), 7/1024 is conservatively above sqrt(3)/256: every lane
// accepted by this test was also accepted by the old 1/256-length margin.
constexpr float EdgeInsideComponentMargin = 7.0f / 1024.0f;

FV_E031_INLINE Vec3x8 Normalize(const Vec3x8 &value,
                                __m256 activeMask) {
    const __m256 lengthSquared = Dot(value, value);
    const __m256 normalizeMask = And(
            activeMask,
            _mm256_cmp_ps(
                    _mm256_set1_ps(
                            PhysicsTolerance::SurfaceDirectionLengthSquared),
                    lengthSquared,
                    _CMP_LT_OQ));
    const __m256 safeLengthSquared = Select(
            normalizeMask, lengthSquared, _mm256_set1_ps(1.0f));
    const __m256 inverseLength = _mm256_div_ps(
            _mm256_set1_ps(1.0f), _mm256_sqrt_ps(safeLengthSquared));
    const Vec3x8 normalized = Scale(value, inverseLength);
    return {
        Select(normalizeMask, normalized.x, value.x),
        Select(normalizeMask, normalized.y, value.y),
        Select(normalizeMask, normalized.z, value.z),
    };
}

FV_E031_INLINE Iso3x8 LoadIso(
        const OptimizedCpuPreparedEllipsoidMeshPacket &prepared) {
    return {
        {
            Load(prepared.worldXx.values),
            Load(prepared.worldXy.values),
            Load(prepared.worldXz.values),
            Load(prepared.worldYx.values),
            Load(prepared.worldYy.values),
            Load(prepared.worldYz.values),
            Load(prepared.worldZx.values),
            Load(prepared.worldZy.values),
            Load(prepared.worldZz.values),
        },
        {
            Load(prepared.worldTx.values),
            Load(prepared.worldTy.values),
            Load(prepared.worldTz.values),
        },
    };
}

FV_E031_INLINE void Store(__m256 value,
                          std::array<float, PacketWidth> *output) {
    _mm256_store_ps(output->data(), value);
}

FV_E031_INLINE void Store(const Vec3x8 &value,
                          std::array<float, PacketWidth> *x,
                          std::array<float, PacketWidth> *y,
                          std::array<float, PacketWidth> *z) {
    Store(value.x, x);
    Store(value.y, y);
    Store(value.z, z);
}

FV_E031_INLINE Iso3x8 BroadcastIso(const GmIso4 &transform) {
    return {
        {
            _mm256_set1_ps(transform.rotation.basisX.x),
            _mm256_set1_ps(transform.rotation.basisY.x),
            _mm256_set1_ps(transform.rotation.basisZ.x),
            _mm256_set1_ps(transform.rotation.basisX.y),
            _mm256_set1_ps(transform.rotation.basisY.y),
            _mm256_set1_ps(transform.rotation.basisZ.y),
            _mm256_set1_ps(transform.rotation.basisX.z),
            _mm256_set1_ps(transform.rotation.basisY.z),
            _mm256_set1_ps(transform.rotation.basisZ.z),
        },
        {
            _mm256_set1_ps(transform.translation.x),
            _mm256_set1_ps(transform.translation.y),
            _mm256_set1_ps(transform.translation.z),
        },
    };
}

FV_E031_INLINE Iso3x8 Compose(const Iso3x8 &first,
                              const Iso3x8 &second) {
    const Vec3x8 basisX = TransformDirection(
            second.rotation,
            {first.rotation.xx, first.rotation.yx, first.rotation.zx});
    const Vec3x8 basisY = TransformDirection(
            second.rotation,
            {first.rotation.xy, first.rotation.yy, first.rotation.zy});
    const Vec3x8 basisZ = TransformDirection(
            second.rotation,
            {first.rotation.xz, first.rotation.yz, first.rotation.zz});
    return {
        {
            basisX.x, basisY.x, basisZ.x,
            basisX.y, basisY.y, basisZ.y,
            basisX.z, basisY.z, basisZ.z,
        },
        TransformPoint(second, first.translation),
    };
}

FV_E031_INLINE Iso3x8 Inverse(const Iso3x8 &transform) {
    const Mat3x8 inverseRotation = {
        transform.rotation.xx,
        transform.rotation.yx,
        transform.rotation.zx,
        transform.rotation.xy,
        transform.rotation.yy,
        transform.rotation.zy,
        transform.rotation.xz,
        transform.rotation.yz,
        transform.rotation.zz,
    };
    return {
        inverseRotation,
        TransformDirection(inverseRotation, Negate(transform.translation)),
    };
}

FV_E031_INLINE Iso3x8 DiagonalTransform(const Vec3x8 &scale) {
    const __m256 zero = _mm256_setzero_ps();
    return {
        {
            scale.x, zero, zero,
            zero, scale.y, zero,
            zero, zero, scale.z,
        },
        {zero, zero, zero},
    };
}

FV_E031_INLINE Iso3x8 ScaleRows(const Iso3x8 &transform,
                                const Vec3x8 &rowScale) {
    Iso3x8 result = transform;
    result.rotation.xx = _mm256_mul_ps(rowScale.x, transform.rotation.xx);
    result.rotation.xy = _mm256_mul_ps(transform.rotation.xy, rowScale.x);
    result.rotation.xz = _mm256_mul_ps(rowScale.x, transform.rotation.xz);
    result.translation.x =
            _mm256_mul_ps(transform.translation.x, rowScale.x);

    result.rotation.yx = _mm256_mul_ps(rowScale.y, transform.rotation.yx);
    result.rotation.yy = _mm256_mul_ps(transform.rotation.yy, rowScale.y);
    result.rotation.yz = _mm256_mul_ps(rowScale.y, transform.rotation.yz);
    result.translation.y =
            _mm256_mul_ps(transform.translation.y, rowScale.y);

    result.rotation.zx = _mm256_mul_ps(rowScale.z, transform.rotation.zx);
    result.rotation.zy = _mm256_mul_ps(transform.rotation.zy, rowScale.z);
    result.rotation.zz = _mm256_mul_ps(rowScale.z, transform.rotation.zz);
    result.translation.z =
            _mm256_mul_ps(transform.translation.z, rowScale.z);
    return result;
}

FV_E031_INLINE Boxx8 TransformEllipsoidBox(
        const Iso3x8 &transform,
        const Vec3x8 &radii) {
    const Mat3x8 absoluteRotation = {
        Abs(transform.rotation.xx),
        Abs(transform.rotation.xy),
        Abs(transform.rotation.xz),
        Abs(transform.rotation.yx),
        Abs(transform.rotation.yy),
        Abs(transform.rotation.yz),
        Abs(transform.rotation.zx),
        Abs(transform.rotation.zy),
        Abs(transform.rotation.zz),
    };
    return {
        TransformPoint(transform, ZeroVector()),
        TransformDirection(absoluteRotation, radii),
    };
}

FV_E031_INLINE __m256 BoundsMask(const Boxx8 &queries,
                                 const GmBoxAligned &candidate,
                                 __m256 activeMask) {
    const __m256 rejectZ = _mm256_cmp_ps(
            _mm256_add_ps(_mm256_set1_ps(candidate.halfExtents.z),
                          queries.halfExtents.z),
            Abs(_mm256_sub_ps(_mm256_set1_ps(candidate.center.z),
                              queries.center.z)),
            _CMP_LT_OQ);
    const __m256 rejectY = _mm256_cmp_ps(
            _mm256_add_ps(_mm256_set1_ps(candidate.halfExtents.y),
                          queries.halfExtents.y),
            Abs(_mm256_sub_ps(_mm256_set1_ps(candidate.center.y),
                              queries.center.y)),
            _CMP_LT_OQ);
    const __m256 rejectX = _mm256_cmp_ps(
            _mm256_add_ps(_mm256_set1_ps(candidate.halfExtents.x),
                          queries.halfExtents.x),
            Abs(_mm256_sub_ps(_mm256_set1_ps(candidate.center.x),
                              queries.center.x)),
            _CMP_LT_OQ);
    return AndNot(Or(Or(rejectZ, rejectY), rejectX), activeMask);
}

struct PacketGroupQueryBounds {
    __m128 minimumCenter;
    __m128 maximumCenter;
    __m256 maximumHalfExtents;
    GmBoxAligned directCandidateBounds;
};

FV_E031_INLINE float RoundPositiveExtentOutward(double extent);

FV_E031_INLINE __m256i InvalidPacketGroupFloatLanes(
        __m256 values,
        bool rejectNegative) {
    const __m256i bits = _mm256_castps_si256(values);
    const __m256i magnitude = _mm256_and_si256(
            bits, _mm256_set1_epi32(0x7fffffffu));
    const __m256i exponent = _mm256_srli_epi32(magnitude, 23u);
    const __m256i zero = _mm256_cmpeq_epi32(
            magnitude, _mm256_setzero_si256());
    const __m256i belowMinimum = _mm256_cmpgt_epi32(
            _mm256_set1_epi32(67), exponent);
    const __m256i aboveMaximum = _mm256_cmpgt_epi32(
            exponent, _mm256_set1_epi32(187));
    __m256i invalid = _mm256_andnot_si256(
            zero, _mm256_or_si256(belowMinimum, aboveMaximum));
    if (rejectNegative) {
        invalid = _mm256_or_si256(
                invalid, _mm256_srai_epi32(bits, 31u));
    }
    return invalid;
}

FV_E031_INLINE float HorizontalMinimum(__m256 values) {
    values = _mm256_min_ps(
            values, _mm256_permute2f128_ps(values, values, 0x01));
    values = _mm256_min_ps(
            values,
            _mm256_permute_ps(values, _MM_SHUFFLE(1, 0, 3, 2)));
    values = _mm256_min_ps(
            values,
            _mm256_permute_ps(values, _MM_SHUFFLE(2, 3, 0, 1)));
    return _mm_cvtss_f32(_mm256_castps256_ps128(values));
}

FV_E031_INLINE float HorizontalMaximum(__m256 values) {
    values = _mm256_max_ps(
            values, _mm256_permute2f128_ps(values, values, 0x01));
    values = _mm256_max_ps(
            values,
            _mm256_permute_ps(values, _MM_SHUFFLE(1, 0, 3, 2)));
    values = _mm256_max_ps(
            values,
            _mm256_permute_ps(values, _MM_SHUFFLE(2, 3, 0, 1)));
    return _mm_cvtss_f32(_mm256_castps256_ps128(values));
}

FV_E031_AVX2 bool BuildPacketGroupQueryBounds(
        const Boxx8 &queries,
        std::uint32_t activeMask,
        PacketGroupQueryBounds *result) noexcept {
    if (result == nullptr || activeMask == 0u) {
        return false;
    }
    const __m256 laneMask = MaskForBits(activeMask);
    __m256i invalid = InvalidPacketGroupFloatLanes(
            queries.center.x, false);
    invalid = _mm256_or_si256(
            invalid,
            InvalidPacketGroupFloatLanes(queries.center.y, false));
    invalid = _mm256_or_si256(
            invalid,
            InvalidPacketGroupFloatLanes(queries.center.z, false));
    invalid = _mm256_or_si256(
            invalid,
            InvalidPacketGroupFloatLanes(queries.halfExtents.x, true));
    invalid = _mm256_or_si256(
            invalid,
            InvalidPacketGroupFloatLanes(queries.halfExtents.y, true));
    invalid = _mm256_or_si256(
            invalid,
            InvalidPacketGroupFloatLanes(queries.halfExtents.z, true));
    invalid = _mm256_and_si256(invalid, _mm256_castps_si256(laneMask));
    if (_mm256_movemask_ps(_mm256_castsi256_ps(invalid)) != 0) {
        return false;
    }

    const __m256 positiveInfinity =
            _mm256_set1_ps(std::numeric_limits<float>::infinity());
    const __m256 negativeInfinity =
            _mm256_set1_ps(-std::numeric_limits<float>::infinity());
    const __m256 zero = _mm256_setzero_ps();
    result->minimumCenter = _mm_set_ps(
            0.0f,
            HorizontalMinimum(_mm256_blendv_ps(
                    positiveInfinity, queries.center.z, laneMask)),
            HorizontalMinimum(_mm256_blendv_ps(
                    positiveInfinity, queries.center.y, laneMask)),
            HorizontalMinimum(_mm256_blendv_ps(
                    positiveInfinity, queries.center.x, laneMask)));
    result->maximumCenter = _mm_set_ps(
            0.0f,
            HorizontalMaximum(_mm256_blendv_ps(
                    negativeInfinity, queries.center.z, laneMask)),
            HorizontalMaximum(_mm256_blendv_ps(
                    negativeInfinity, queries.center.y, laneMask)),
            HorizontalMaximum(_mm256_blendv_ps(
                    negativeInfinity, queries.center.x, laneMask)));
    const __m128 maximumHalfExtents = _mm_set_ps(
            0.0f,
            HorizontalMaximum(_mm256_blendv_ps(
                    zero, queries.halfExtents.z, laneMask)),
            HorizontalMaximum(_mm256_blendv_ps(
                    zero, queries.halfExtents.y, laneMask)),
            HorizontalMaximum(_mm256_blendv_ps(
                    zero, queries.halfExtents.x, laneMask)));
    result->maximumHalfExtents = _mm256_insertf128_ps(
            _mm256_castps128_ps256(maximumHalfExtents),
            maximumHalfExtents,
            1);
    alignas(16) std::array<float, 4u> minimumCenter;
    alignas(16) std::array<float, 4u> maximumCenter;
    alignas(16) std::array<float, 4u> maximumHalfExtentValues;
    std::array<float, 3u> directCenter;
    std::array<float, 3u> directHalfExtents;
    _mm_store_ps(minimumCenter.data(), result->minimumCenter);
    _mm_store_ps(maximumCenter.data(), result->maximumCenter);
    _mm_store_ps(maximumHalfExtentValues.data(), maximumHalfExtents);
    for (std::size_t axis = 0u; axis < 3u; ++axis) {
        const double lower =
                static_cast<double>(minimumCenter[axis]) -
                maximumHalfExtentValues[axis];
        const double upper =
                static_cast<double>(maximumCenter[axis]) +
                maximumHalfExtentValues[axis];
        const double exactCenter = (lower + upper) * 0.5;
        directCenter[axis] = static_cast<float>(exactCenter);
        const double extent = std::max(
                static_cast<double>(directCenter[axis]) - lower,
                upper - static_cast<double>(directCenter[axis]));
        directHalfExtents[axis] = RoundPositiveExtentOutward(extent);
    }
    result->directCandidateBounds = {
        {directCenter[0u], directCenter[1u], directCenter[2u]},
        {directHalfExtents[0u],
         directHalfExtents[1u],
         directHalfExtents[2u]},
    };
    return true;
}

FV_E031_INLINE bool PacketGroupRejectsAll(
        const PacketGroupQueryBounds &queries,
        const OptimizedCpuStaticMeshPacketGroup &group) {
    const __m256 groupCenters =
            _mm256_loadu_ps(&group.minimumCenter.x);
    const __m256 packedQueryCenters = _mm256_loadu_ps(
            reinterpret_cast<const float *>(&queries.minimumCenter));
    const __m256 queryCenters = _mm256_permute2f128_ps(
            packedQueryCenters, packedQueryCenters, 0x01);
    __m256 centerDistances = _mm256_sub_ps(
            groupCenters, queryCenters);
    // The low half is groupMin-queryMax. Negate the high half so that it is
    // queryMin-groupMax, then test both separation directions together.
    centerDistances = _mm256_xor_ps(
            centerDistances,
            _mm256_castsi256_ps(_mm256_set_epi32(
                    static_cast<int>(0x80000000u),
                    static_cast<int>(0x80000000u),
                    static_cast<int>(0x80000000u),
                    static_cast<int>(0x80000000u),
                    0,
                    0,
                    0,
                    0)));
    __m256 groupMaximumHalfExtents = _mm256_broadcast_ps(
            reinterpret_cast<const __m128 *>(
                    &group.maximumHalfExtents.x));
    groupMaximumHalfExtents = _mm256_blend_ps(
            groupMaximumHalfExtents, _mm256_setzero_ps(), 0x88);
    const __m256 reach = _mm256_add_ps(
            groupMaximumHalfExtents, queries.maximumHalfExtents);
    return (_mm256_movemask_ps(_mm256_cmp_ps(
                    reach, centerDistances, _CMP_LT_OQ)) &
            0x77) != 0;
}

FV_E031_INLINE float RoundPositiveExtentOutward(double extent) {
    float rounded = static_cast<float>(extent);
    if (static_cast<double>(rounded) < extent) {
        std::uint32_t bits;
        std::memcpy(&bits, &rounded, sizeof(bits));
        ++bits;
        std::memcpy(&rounded, &bits, sizeof(rounded));
    }
    return rounded;
}

FV_E031_INLINE GmBoxAligned PacketDirectCandidateBounds(
        const PacketGroupQueryBounds &queries) {
    return queries.directCandidateBounds;
}

struct PacketExecution {
    const OptimizedCpuPreparedEllipsoidMeshPacket &setup;
    Iso3x8 meshToUnit;
    Iso3x8 meshToEllipsoid;
    Vec3x8 radii;
    Vec3x8 inverseRadii;
    const GmIso4 *meshWorldSource = nullptr;
    Boxx8 meshBounds;
    __m256 packetMask;
    Iso3x8 contactToWorld{};
    Iso3x8 normalToWorld{};
    bool outputTransformsReady = false;
    std::uint32_t hitMask = 0u;

    FV_E031_INLINE void EnsureOutputTransforms(void) {
        if (outputTransformsReady) {
            return;
        }
        const Iso3x8 ellipsoidToMesh = Inverse(meshToEllipsoid);
        const Iso3x8 meshWorld = BroadcastIso(*meshWorldSource);
        contactToWorld = Compose(
                Compose(DiagonalTransform(radii), ellipsoidToMesh),
                meshWorld);
        normalToWorld = Compose(
                Compose(DiagonalTransform(inverseRadii), ellipsoidToMesh),
                meshWorld);
        outputTransformsReady = true;
    }

    FV_E031_INLINE void Emit(
            __m256 mask,
            const Vec3x8 &normalUnit,
            const Vec3x8 &separationUnit,
            const Vec3x8 &contactUnit,
            const Vec3x8 &extraUnit,
            GmLocalMaterialIndex triangleMaterial,
            bool primary) {
        const std::uint32_t bits = Bits(mask);
        if (bits == 0u) {
            return;
        }
        EnsureOutputTransforms();

        const Vec3x8 contactWorld =
                TransformPoint(contactToWorld, contactUnit);
        Vec3x8 normalWorld =
                TransformDirection(normalToWorld.rotation, normalUnit);
        normalWorld = Normalize(normalWorld, mask);
        const Vec3x8 separationWorld =
                TransformDirection(contactToWorld.rotation, separationUnit);

        alignas(32) std::array<float, PacketWidth> nx{}, ny{}, nz{};
        alignas(32) std::array<float, PacketWidth> sx{}, sy{}, sz{};
        alignas(32) std::array<float, PacketWidth> cx{}, cy{}, cz{};
        alignas(32) std::array<float, PacketWidth> ex{}, ey{}, ez{};
        Store(normalWorld, &nx, &ny, &nz);
        Store(separationWorld, &sx, &sy, &sz);
        Store(contactWorld, &cx, &cy, &cz);
        Store(extraUnit, &ex, &ey, &ez);

        for (std::size_t lane = 0u; lane < PacketWidth; ++lane) {
            if ((bits & (1u << lane)) == 0u) {
                continue;
            }
            GmCollision &collision = setup.buffers[lane]->AddCollision();
            collision.impulseNormal = {nx[lane], ny[lane], nz[lane]};
            collision.separation = {sx[lane], sy[lane], sz[lane]};
            collision.contactPoint = {cx[lane], cy[lane], cz[lane]};
            collision.localMaterialA = setup.materials[lane];
            collision.localMaterialB = triangleMaterial;
            collision.sphereMergePrimary = primary;
            collision.extraNegated = {ex[lane], ey[lane], ez[lane]};
        }
        hitMask |= bits;
    }

    FV_E031_INLINE void EmitFeature(
            __m256 terminalMask,
            const Vec3x8 &featurePoint,
            __m256 minimumDistanceSquared,
            bool requireRadiusContainment,
            const Vec3x8 &triangleNormal,
            GmLocalMaterialIndex triangleMaterial) {
        if (Bits(terminalMask) == 0u) {
            return;
        }
        const Vec3x8 featureToCenter =
                Subtract(ZeroVector(), featurePoint);
        const __m256 distanceSquared =
                Dot(featureToCenter, featureToCenter);
        __m256 invalid = _mm256_cmp_ps(
                minimumDistanceSquared, distanceSquared, _CMP_NLT_UQ);
        if (requireRadiusContainment) {
            invalid = Or(
                    invalid,
                    _mm256_cmp_ps(
                            _mm256_set1_ps(1.0f),
                            distanceSquared,
                            _CMP_LT_OQ));
        }
        const __m256 valid = AndNot(invalid, terminalMask);
        if (Bits(valid) == 0u) {
            return;
        }

        const __m256 safeDistanceSquared =
                Select(valid, distanceSquared, _mm256_set1_ps(1.0f));
        const __m256 distance = _mm256_sqrt_ps(safeDistanceSquared);
        const __m256 inverseDistance =
                _mm256_div_ps(_mm256_set1_ps(1.0f), distance);
        const Vec3x8 normal = Scale(featureToCenter, inverseDistance);
        const __m256 penetrationScale = _mm256_mul_ps(
                _mm256_sub_ps(distance, _mm256_set1_ps(1.0f)),
                inverseDistance);
        const Vec3x8 penetration =
                Scale(featureToCenter, penetrationScale);
        const __m256 separationAlongNormal =
                Dot(penetration, triangleNormal);
        const Vec3x8 separation =
                Scale(triangleNormal, separationAlongNormal);
        Emit(valid,
             normal,
             separation,
             featurePoint,
             triangleNormal,
             triangleMaterial,
             false);
    }

    FV_E031_INLINE void EmitEndpoint(
            __m256 terminalMask,
            const Vec3x8 &featurePoint,
            __m256 minimumDistance,
            const Vec3x8 &triangleNormal,
            GmLocalMaterialIndex triangleMaterial) {
        if (Bits(terminalMask) == 0u) {
            return;
        }
        const Vec3x8 featureToCenter =
                Subtract(ZeroVector(), featurePoint);
        const __m256 distanceSquared =
                Dot(featureToCenter, featureToCenter);
        const __m256 safeDistanceSquared =
                Select(terminalMask, distanceSquared, _mm256_set1_ps(1.0f));
        const __m256 distance = _mm256_sqrt_ps(safeDistanceSquared);
        const __m256 invalid = Or(
                _mm256_cmp_ps(
                        _mm256_set1_ps(1.0f), distance, _CMP_LT_OQ),
                _mm256_cmp_ps(
                        minimumDistance, distance, _CMP_NLT_UQ));
        const __m256 valid = AndNot(invalid, terminalMask);
        if (Bits(valid) == 0u) {
            return;
        }

        const __m256 safeDistance =
                Select(valid, distance, _mm256_set1_ps(1.0f));
        const __m256 endpointDistance = _mm256_sqrt_ps(safeDistance);
        const __m256 inverseEndpointDistance =
                _mm256_div_ps(_mm256_set1_ps(1.0f), endpointDistance);
        const Vec3x8 normal =
                Scale(featureToCenter, inverseEndpointDistance);
        const __m256 penetrationScale = _mm256_mul_ps(
                _mm256_sub_ps(endpointDistance, _mm256_set1_ps(1.0f)),
                inverseEndpointDistance);
        const Vec3x8 penetration =
                Scale(featureToCenter, penetrationScale);
        const __m256 separationAlongNormal =
                Dot(penetration, triangleNormal);
        const Vec3x8 separation =
                Scale(triangleNormal, separationAlongNormal);
        Emit(valid,
             normal,
             separation,
             featurePoint,
             triangleNormal,
             triangleMaterial,
             false);
    }

    FV_E031_INLINE void CollideTriangle(
            const OptimizedCpuStaticMeshPacketTriangleData &triangle,
            __m256 candidateMask) {
        if (Bits(candidateMask) == 0u) {
            return;
        }

        const Vec3x8 vertex0 = TransformPoint(
                meshToUnit, Broadcast(triangle.vertices[0u]));
        const Vec3x8 vertex1 = TransformPoint(
                meshToUnit, Broadcast(triangle.vertices[1u]));
        const Vec3x8 vertex2 = TransformPoint(
                meshToUnit, Broadcast(triangle.vertices[2u]));
        const Vec3x8 edge01 = Subtract(vertex1, vertex0);
        const Vec3x8 edge02 = Subtract(vertex2, vertex0);

        Vec3x8 triangleNormal = {
            _mm256_sub_ps(_mm256_mul_ps(edge02.z, edge01.y),
                          _mm256_mul_ps(edge02.y, edge01.z)),
            _mm256_sub_ps(_mm256_mul_ps(edge01.z, edge02.x),
                          _mm256_mul_ps(edge02.z, edge01.x)),
            _mm256_sub_ps(_mm256_mul_ps(edge01.x, edge02.y),
                          _mm256_mul_ps(edge02.x, edge01.y)),
        };
        const __m256 normalLengthSquared = _mm256_add_ps(
                _mm256_add_ps(
                        _mm256_mul_ps(triangleNormal.y, triangleNormal.y),
                        _mm256_mul_ps(triangleNormal.x, triangleNormal.x)),
                _mm256_mul_ps(triangleNormal.z, triangleNormal.z));
        const __m256 usableNormal = And(
                candidateMask,
                _mm256_cmp_ps(
                        _mm256_set1_ps(
                                PhysicsTolerance::SurfaceDirectionLengthSquared),
                        normalLengthSquared,
                        _CMP_LT_OQ));
        if (Bits(usableNormal) == 0u) {
            return;
        }
        const __m256 safeNormalLengthSquared = Select(
                usableNormal, normalLengthSquared, _mm256_set1_ps(1.0f));
        const __m256 inverseNormalLength = _mm256_div_ps(
                _mm256_set1_ps(1.0f),
                _mm256_sqrt_ps(safeNormalLengthSquared));
        triangleNormal = Scale(triangleNormal, inverseNormalLength);

        const __m256 planeDistance = Dot(
                Subtract(ZeroVector(), vertex0), triangleNormal);
        const __m256 planeReject = Or(
                _mm256_cmp_ps(_mm256_set1_ps(1.0f),
                              planeDistance,
                              _CMP_LT_OQ),
                _mm256_cmp_ps(planeDistance,
                              _mm256_setzero_ps(),
                              _CMP_LT_OQ));
        __m256 remaining = AndNot(planeReject, usableNormal);
        if (Bits(remaining) == 0u) {
            return;
        }

        const __m256 safeRadicand = Select(
                remaining,
                _mm256_sub_ps(
                        _mm256_set1_ps(1.0f),
                        _mm256_mul_ps(planeDistance, planeDistance)),
                _mm256_setzero_ps());
        const __m256 edgeReach = _mm256_sqrt_ps(safeRadicand);
        const Vec3x8 projectedPoint = Add(
                ZeroVector(),
                Scale(
                        triangleNormal,
                        _mm256_xor_ps(
                                planeDistance,
                                _mm256_set1_ps(-0.0f))));

        const Vec3x8 vertices[3u] = {vertex0, vertex1, vertex2};
        for (std::size_t edgeIndex = 0u; edgeIndex < 3u; ++edgeIndex) {
            if (Bits(remaining) == 0u) {
                break;
            }
            const std::size_t nextIndex = edgeIndex == 2u ? 0u : edgeIndex + 1u;
            const Vec3x8 edgeStart = vertices[edgeIndex];
            const Vec3x8 edgeEnd = vertices[nextIndex];
            const Vec3x8 rawEdge = Subtract(edgeEnd, edgeStart);

            // The cheap half of this test decides it almost every time.
            //
            // An edge whose signed distance is negative for every lane still in
            // play leaves the iteration a no-op: nothing can be rejected,
            // because edgeReach is a square root and so never below zero, and
            // nothing is outside, because that wants a positive distance. Only
            // the sign is needed to know that, and the sign does not depend on
            // normalizing the edge -- Normalize only ever scales by a positive
            // factor, or leaves the vector alone. So the sign of the same
            // product built from the raw edge answers it, without the square
            // root and the division that normalizing costs.
            //
            // The comparison is against the edge length rather than zero, so
            // that the margin means the same thing on a triangle of any size:
            // the distance the normalized form would produce is this one over
            // the edge length, and requiring it to clear 1/256 of a unit-sphere
            // radius puts it four orders of magnitude clear of anything
            // rounding can do to the sign. A lane nearer the boundary than that
            // simply falls through to the exact path below, which is where
            // roughly one iteration in five hundred ends up.
            {
                const Vec3x8 rawNormal = Cross(rawEdge, triangleNormal);
                const __m256 rawDistance = Dot(
                        Subtract(projectedPoint, edgeStart), rawNormal);
                const __m256 maximumEdgeComponent = _mm256_max_ps(
                        Abs(rawEdge.x),
                        _mm256_max_ps(Abs(rawEdge.y), Abs(rawEdge.z)));
                const __m256 clearlyInsideThreshold = _mm256_mul_ps(
                        maximumEdgeComponent,
                        _mm256_set1_ps(-EdgeInsideComponentMargin));
                const __m256 clearlyInside = _mm256_cmp_ps(
                        rawDistance, clearlyInsideThreshold, _CMP_LT_OQ);
                if (Bits(AndNot(clearlyInside, remaining)) == 0u) {
                    continue;
                }
            }

            const Vec3x8 edgeDirection = Normalize(rawEdge, remaining);
            const Vec3x8 edgeNormal =
                    Cross(edgeDirection, triangleNormal);
            const __m256 edgeDistance = Dot(
                    Subtract(projectedPoint, edgeStart), edgeNormal);

            const __m256 rejected = And(
                    remaining,
                    _mm256_cmp_ps(edgeReach, edgeDistance, _CMP_LT_OQ));
            remaining = AndNot(rejected, remaining);
            const __m256 outside = And(
                    remaining,
                    _mm256_cmp_ps(_mm256_setzero_ps(),
                                  edgeDistance,
                                  _CMP_LT_OQ));
            if (Bits(outside) == 0u) {
                continue;
            }

            const __m256 alongFromStart = Dot(
                    Subtract(projectedPoint, edgeStart), edgeDirection);
            const __m256 startFeature = And(
                    outside,
                    _mm256_cmp_ps(alongFromStart,
                                  _mm256_setzero_ps(),
                                  _CMP_LT_OQ));
            EmitFeature(
                    startFeature,
                    edgeStart,
                    _mm256_set1_ps(
                            PhysicsTolerance::SurfaceDirectionLengthSquared),
                    true,
                    triangleNormal,
                    triangle.material);

            const __m256 afterStart = AndNot(startFeature, outside);
            const __m256 alongFromEnd = Dot(
                    Subtract(projectedPoint, edgeEnd), edgeDirection);
            const __m256 edgeFeature = And(
                    afterStart,
                    _mm256_cmp_ps(_mm256_setzero_ps(),
                                  alongFromEnd,
                                  _CMP_NLT_UQ));
            const Vec3x8 featurePoint = Add(
                    projectedPoint,
                    Scale(
                            edgeNormal,
                            _mm256_xor_ps(
                                    edgeDistance,
                                    _mm256_set1_ps(-0.0f))));
            EmitFeature(
                    edgeFeature,
                    featurePoint,
                    _mm256_set1_ps(PhysicsTolerance::CollisionDistance),
                    false,
                    triangleNormal,
                    triangle.material);

            const __m256 endpoint = AndNot(edgeFeature, afterStart);
            EmitEndpoint(
                    endpoint,
                    edgeEnd,
                    _mm256_set1_ps(
                            PhysicsTolerance::SurfaceDirectionLengthSquared),
                    triangleNormal,
                    triangle.material);
            remaining = AndNot(outside, remaining);
        }

        const __m256 face = And(
                remaining,
                _mm256_cmp_ps(_mm256_setzero_ps(),
                              planeDistance,
                              _CMP_LT_OQ));
        Emit(face,
             triangleNormal,
             Scale(
                     triangleNormal,
                     _mm256_sub_ps(
                             planeDistance, _mm256_set1_ps(1.0f))),
             projectedPoint,
             triangleNormal,
             triangle.material,
             true);
    }
};

FV_E031_HOT_AVX2 bool RunPacketAvx2(
        const OptimizedCpuPreparedEllipsoidMeshPacket &setup,
        std::uint32_t activeMask,
        const GmIso4 &meshIso,
        const GmIso4 &meshInverse,
        const OptimizedCpuStaticMeshTriangleSidecar &triangles,
        const OptimizedCpuStaticMeshTriangleHierarchyView &hierarchy,
        std::uint32_t *hitMask) noexcept {
    if (hierarchy.packetCells == nullptr ||
        hierarchy.count == 0u ||
        hierarchy.maximumTraversalDepth > PacketTraversalCapacity) {
        return false;
    }
    const Iso3x8 ellipsoidWorld = LoadIso(setup);
    const Vec3x8 radii = {
        Load(setup.radiiX.values),
        Load(setup.radiiY.values),
        Load(setup.radiiZ.values),
    };
    const Vec3x8 inverseRadii = {
        Load(setup.inverseRadiiX.values),
        Load(setup.inverseRadiiY.values),
        Load(setup.inverseRadiiZ.values),
    };
    const Iso3x8 ellipsoidToMesh =
            Compose(ellipsoidWorld, BroadcastIso(meshInverse));
    const Boxx8 meshBounds =
            TransformEllipsoidBox(ellipsoidToMesh, radii);
    const Iso3x8 meshToEllipsoid = Inverse(ellipsoidToMesh);
    PacketGroupQueryBounds packetGroupQueries;
    constexpr unsigned int MxcsrControlMask = 0xffc0u;
    constexpr unsigned int DeterministicMxcsrControl = 0x1f80u;
    const unsigned int mxcsr = _mm_getcsr();
    // The conservative group arithmetic can add only FE_INEXACT for certified
    // operands. Run it only when that sticky status is already set and the
    // default rounding/denormal controls are active; otherwise preserve the
    // authoritative per-cell path without executing extra floating work.
    const bool conservativePacketQueriesReady =
            (mxcsr & MxcsrControlMask) == DeterministicMxcsrControl &&
            (mxcsr & _MM_EXCEPT_INEXACT) != 0u &&
            BuildPacketGroupQueryBounds(
                    meshBounds, activeMask, &packetGroupQueries);
    const bool usePacketGroups =
            hierarchy.packetGroups != nullptr &&
            hierarchy.packetGroupCount != 0u &&
            conservativePacketQueriesReady;
    const std::uint32_t packetGroupOrdinalMask = usePacketGroups
            ? 0xffff0000u
            : 0u;
    PacketExecution execution{
        setup,
        ScaleRows(meshToEllipsoid, inverseRadii),
        meshToEllipsoid,
        radii,
        inverseRadii,
        &meshIso,
        meshBounds,
        MaskForBits(activeMask),
        {},
        {},
        false,
        0u,
    };

    // The sidecar's direct candidates are a conservative posting list in the
    // same flattened DFS triangle order as the authoritative hierarchy. Query
    // it once with an outward-rounded envelope for all active lanes, then keep
    // the exact packet leaf-bounds test below. This removes internal hierarchy
    // traffic without changing triangle order or collision arithmetic.
    OptimizedCpuStaticUniformGrid::CandidateSpan directCandidates;
    if (conservativePacketQueriesReady &&
        triangles.DirectCandidateTriangleSpan(
                PacketDirectCandidateBounds(packetGroupQueries),
                &directCandidates) &&
        directCandidates.size <= 64u) {
        for (std::size_t candidateIndex = 0u;
             candidateIndex < directCandidates.size;
             ++candidateIndex) {
            const OptimizedCpuStaticMeshDirectTrianglePosting &posting =
                    triangles.DirectTriangleAt(
                            directCandidates.data[candidateIndex]);
            const __m256 laneMask = BoundsMask(
                    execution.meshBounds,
                    posting.bounds,
                    execution.packetMask);
            if (Bits(laneMask) == 0u) {
                continue;
            }
            execution.CollideTriangle(
                    triangles.PacketTriangleAt(posting.triangleIndex),
                    laneMask);
        }
        *hitMask = execution.hitMask;
        return true;
    }

    struct alignas(32) TraversalMask {
        __m256 value;
    };
    // The sidecar certifies each cell's DFS depth. Branch masks are written at
    // their certified depth before any descendant reads them, eliminating the
    // runtime subtree-end stack and pop loop.
    std::array<TraversalMask, PacketTraversalCapacity> traversalMasks;

    // The sidecar's topology certificate guarantees that every subtree skip
    // remains within this contiguous descriptor range.
    const OptimizedCpuStaticMeshPacketCell *cell = hierarchy.packetCells;
    const OptimizedCpuStaticMeshPacketCell *const cellEnd =
            cell + hierarchy.count;
    while (cell != cellEnd) {
        std::uint32_t cellMetadata;
        std::memcpy(&cellMetadata, &cell->depth, sizeof(cellMetadata));
        const std::size_t traversalDepth = cellMetadata & 0xffu;
        const std::uint32_t packetGroupOrdinalBits =
                cellMetadata & packetGroupOrdinalMask;
        if (packetGroupOrdinalBits != 0u) {
            const std::size_t groupIndex =
                    static_cast<std::size_t>(
                            (packetGroupOrdinalBits >> 16u) - 1u);
            const OptimizedCpuStaticMeshPacketGroup &group =
                    hierarchy.packetGroups[groupIndex];
            if (PacketGroupRejectsAll(packetGroupQueries, group)) {
                cell += group.subtreeEntryCount;
                continue;
            }
        }
        const __m256 parentMask = traversalDepth == 0u
                ? execution.packetMask
                : traversalMasks[traversalDepth - 1u].value;
        __m256 laneMask = BoundsMask(
                execution.meshBounds,
                cell->bounds,
                parentMask);
        std::uint32_t laneBits = Bits(laneMask);
        if (laneBits == 0u) {
            cell += cell->subtreeEntryCount;
            continue;
        }
        if ((cellMetadata & 0x100u) == 0u) {
            if (traversalDepth == traversalMasks.size()) {
                return false;
            }
            traversalMasks[traversalDepth].value = laneMask;
            ++cell;
            continue;
        }

        const u32 triangleIndex = cell->triangleIndex;
        ++cell;
        const OptimizedCpuStaticMeshPacketTriangleData &triangle =
                triangles.PacketTriangleAt(triangleIndex);
        // BoundsMask and every parent mask use canonical all-zero/all-one
        // lanes, so converting through movemask and rebuilding the vector is
        // redundant. Preserve the exact mask produced by the bounds test.
        execution.CollideTriangle(triangle, laneMask);
    }
    *hitMask = execution.hitMask;
    return true;
}

#undef FV_E031_AVX2
#undef FV_E031_HOT_AVX2
#undef FV_E031_INLINE

#endif

}  // namespace

bool OptimizedCpuEllipsoidMeshPacketAvailable(void) noexcept {
#if FV_E031_HAS_X86_PACKET
    static const bool available = []() noexcept {
#if defined(_WIN32)
        int registers[4]{};
        __cpuidex(registers, 0, 0);
        if (registers[0] < 7) {
            return false;
        }

        __cpuidex(registers, 1, 0);
        constexpr int OsXsave = 1 << 27;
        constexpr int Avx = 1 << 28;
        if ((registers[2] & (OsXsave | Avx)) != (OsXsave | Avx)) {
            return false;
        }
        constexpr unsigned long long XmmAndYmmState = 0x6ull;
        if ((_xgetbv(0) & XmmAndYmmState) != XmmAndYmmState) {
            return false;
        }

        __cpuidex(registers, 7, 0);
        constexpr int Avx2 = 1 << 5;
        return (registers[1] & Avx2) != 0;
#else
        __builtin_cpu_init();
        return __builtin_cpu_supports("avx2") != 0;
#endif
    }();
    return available;
#else
    return false;
#endif
}

bool PrepareOptimizedCpuEllipsoidMeshPacket(
        const OptimizedCpuEllipsoidMeshPacketLane *lanes,
        std::size_t laneCount,
        std::uint32_t preparedMask,
        OptimizedCpuPreparedEllipsoidMeshPacket *prepared) noexcept {
    if (!OptimizedCpuEllipsoidMeshPacketAvailable()) {
        return false;
    }
    return BuildPreparedPacket(
            lanes, laneCount, preparedMask, prepared);
}

bool GmCollision_PreparedEllipsoidPacket_Mesh_InlineMathOptimizedCpuNativeBinary32WithStaticCache(
        const OptimizedCpuPreparedEllipsoidMeshPacket &prepared,
        std::uint32_t activeMask,
        const LocatedGmSurf &mesh,
        const GmIso4 &meshInverse,
        const OptimizedCpuStaticMeshTriangleSidecar &triangles,
        std::uint32_t *hitMask) noexcept {
    if (hitMask == nullptr || prepared.laneCount < 2u ||
        prepared.laneCount > PacketWidth) {
        return false;
    }
    *hitMask = 0u;
    const std::uint32_t laneMask =
            (1u << static_cast<unsigned int>(prepared.laneCount)) - 1u;
    activeMask &= laneMask;
    if ((activeMask & ~prepared.preparedMask) != 0u ||
        mesh.surf == nullptr || mesh.iso == nullptr ||
        typeid(*mesh.surf) != typeid(GmSurfMesh) ||
        !IsFiniteTransform(meshInverse) ||
        !triangles.IsFor(static_cast<const GmSurfMesh &>(*mesh.surf))) {
        return false;
    }
    if (activeMask == 0u) {
        return true;
    }
#if FV_E031_HAS_X86_PACKET
    OptimizedCpuStaticMeshTriangleHierarchyView hierarchy;
    if (!triangles.TriangleHierarchyView(&hierarchy)) {
        return false;
    }
    return RunPacketAvx2(
            prepared,
            activeMask,
            *mesh.iso,
            meshInverse,
            triangles,
            hierarchy,
            hitMask);
#else
    return false;
#endif
}

bool GmCollision_PreparedEllipsoidPacket_Mesh_InlineMathOptimizedCpuNativeBinary32WithCertifiedStaticMesh(
        const OptimizedCpuPreparedEllipsoidMeshPacket &prepared,
        std::uint32_t activeMask,
        const OptimizedCpuCertifiedStaticMeshPacket &mesh,
        std::uint32_t *hitMask) noexcept {
    if (hitMask == nullptr || prepared.laneCount < 2u ||
        prepared.laneCount > PacketWidth) {
        return false;
    }
    *hitMask = 0u;
    const std::uint32_t laneMask =
            (1u << static_cast<unsigned int>(prepared.laneCount)) - 1u;
    activeMask &= laneMask;
    if ((activeMask & ~prepared.preparedMask) != 0u ||
        !mesh.IsAvailable()) {
        return false;
    }
    if (activeMask == 0u) {
        return true;
    }
#if FV_E031_HAS_X86_PACKET
    return RunPacketAvx2(
            prepared,
            activeMask,
            mesh.meshIso,
            mesh.meshInverse,
            *mesh.triangles,
            mesh.hierarchy,
            hitMask);
#else
    return false;
#endif
}

bool GmCollision_EllipsoidPacket_Mesh_InlineMathOptimizedCpuNativeBinary32WithStaticCache(
        const OptimizedCpuEllipsoidMeshPacketLane *lanes,
        std::size_t laneCount,
        std::uint32_t activeMask,
        const LocatedGmSurf &mesh,
        const GmIso4 &meshInverse,
        const OptimizedCpuStaticMeshTriangleSidecar &triangles,
        std::uint32_t *hitMask) noexcept {
    if (hitMask == nullptr || !OptimizedCpuEllipsoidMeshPacketAvailable()) {
        return false;
    }
    *hitMask = 0u;
    if (mesh.surf == nullptr || typeid(*mesh.surf) != typeid(GmSurfMesh) ||
        !triangles.IsFor(static_cast<const GmSurfMesh &>(*mesh.surf))) {
        return false;
    }
    if (mesh.iso == nullptr || !IsFiniteTransform(meshInverse)) {
        return false;
    }
    OptimizedCpuPreparedEllipsoidMeshPacket prepared;
    if (!BuildPreparedPacket(
                lanes, laneCount, activeMask, &prepared)) {
        return false;
    }
    return GmCollision_PreparedEllipsoidPacket_Mesh_InlineMathOptimizedCpuNativeBinary32WithStaticCache(
            prepared,
            activeMask,
            mesh,
            meshInverse,
            triangles,
            hitMask);
}

#undef FV_E031_HAS_X86_PACKET
