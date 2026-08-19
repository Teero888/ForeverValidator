#include <cmath>
#include <cstring>
#include <limits>

#if defined(__x86_64__) || defined(_M_X64)
#include <xmmintrin.h>
#define TMNF_BINARY32_HAS_NATIVE_SQRT 1
#else
#define TMNF_BINARY32_HAS_NATIVE_SQRT 0
#endif

#include "engine/core/binary32_math.h"
namespace {

thread_local bool nativeSqrtEnabled = false;

constexpr double kPi = 3.14159265358979323846264338327950288;
constexpr double kHalfPi = 1.57079632679489661923132169163975144;
constexpr double kQuarterPi = 0.785398163397448309615660845819875721;
constexpr double kTwoOverPi = 0.636619772367581343075535053490057448;
constexpr double kHalfPiHigh = 1.570796326734125614166;
constexpr double kHalfPiLow = 6.07710050650619224932e-11;
constexpr double kLog2E = 1.44269504088896340735992468100189214;
constexpr double kLn2High = 0.693147180559945286227;
constexpr double kLn2Low = 2.31904681384629955842e-17;

bool IsFiniteFloat(float value) { return std::isfinite(value); }

bool IsNaNFloat(float value) { return std::isnan(value); }

float QuietNaN(void) { return std::numeric_limits<float>::quiet_NaN(); }

double PowerOfTwo(int exponent) {
    if (exponent < -1022) {
        return 0.0;
    }
    if (exponent > 1023) {
        return std::numeric_limits<double>::infinity();
    }
    return std::ldexp(1.0, exponent);
}

double AtanUnit(double value) {
    bool aroundOne = value > 0.4142135623730950488;
    double reduced = aroundOne ? (value - 1.0) / (value + 1.0) : value;
    const double square = reduced * reduced;
    double power = reduced;
    double result = reduced;
    for (unsigned termIndex = 1u; termIndex <= 24u; termIndex++) {
        power *= -square;
        result += power / static_cast<double>(termIndex * 2u + 1u);
    }
    return aroundOne ? kQuarterPi + result : result;
}

double Atan2(double y, double x) {
    const bool yNegative = std::signbit(y);
    const bool xNegative = std::signbit(x);
    const double absY = (std::fabs((y)));
    const double absX = (std::fabs((x)));

    if (absY == 0.0) {
        if (xNegative) {
            return yNegative ? -kPi : kPi;
        }
        return y;
    }
    if (absX == 0.0) {
        return yNegative ? -kHalfPi : kHalfPi;
    }

    double angle = absY <= absX ? AtanUnit(absY / absX) : kHalfPi - AtanUnit(absX / absY);
    if (xNegative) {
        angle = kPi - angle;
    }
    return yNegative ? -angle : angle;
}

double SinPolynomial(double value) {
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

double CosPolynomial(double value) {
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

struct ReducedAngle {
    double value;
    unsigned quadrant;
};

struct FixedProduct {
    uint64_t limbs[4];
};

FixedProduct MultiplyTwoOverPi(uint32_t significand) {
    // First 192 fractional bits of 2/pi, stored least-significant limb first.
    constexpr uint64_t kTwoOverPiFixed192[3] = {
            0xdb6295993c439041ull,
            0xfc2757d1f534ddc0ull,
            0xa2f9836e4e441529ull,
    };
    FixedProduct result = {};
    uint64_t carry = 0u;
    for (unsigned limbIndex = 0u; limbIndex < 3u; limbIndex++) {
        const uint64_t limb = kTwoOverPiFixed192[limbIndex];
        const uint64_t lowProduct =
                static_cast<uint64_t>(static_cast<uint32_t>(limb)) * significand + carry;
        const uint64_t highProduct =
                static_cast<uint64_t>(static_cast<uint32_t>(limb >> 32u)) * significand +
                (lowProduct >> 32u);
        result.limbs[limbIndex] = (lowProduct & 0xffffffffu) | (highProduct << 32u);
        carry = highProduct >> 32u;
    }
    result.limbs[3] = carry;
    return result;
}

bool FixedBit(const FixedProduct &value, unsigned bitIndex) {
    return bitIndex < 256u && ((value.limbs[bitIndex / 64u] >> (bitIndex % 64u)) & 1u) != 0u;
}

bool AnyFixedBitsBelow(const FixedProduct &value, unsigned bitCount) {
    const unsigned fullLimbs = bitCount / 64u;
    for (unsigned limbIndex = 0u; limbIndex < fullLimbs && limbIndex < 4u; limbIndex++) {
        if (value.limbs[limbIndex] != 0u) {
            return true;
        }
    }
    const unsigned remaining = bitCount % 64u;
    return fullLimbs < 4u && remaining != 0u &&
           (value.limbs[fullLimbs] & ((uint64_t{1} << remaining) - 1u)) != 0u;
}

ReducedAngle ReduceLargeAngle(float input) {
    int binaryExponent = 0;
    const float mantissa = std::frexp(std::fabs(input), &binaryExponent);
    const int exponent = binaryExponent - 1;
    const uint32_t significand =
            static_cast<uint32_t>(std::ldexp(static_cast<double>(mantissa), 24));
    const FixedProduct product = MultiplyTwoOverPi(significand);
    const unsigned binaryPoint = static_cast<unsigned>(215 - exponent);

    unsigned nearest = (FixedBit(product, binaryPoint) ? 1u : 0u) |
                       (FixedBit(product, binaryPoint + 1u) ? 2u : 0u);
    const bool halfwayBit = FixedBit(product, binaryPoint - 1u);
    const bool belowHalfway = AnyFixedBitsBelow(product, binaryPoint - 1u);
    const bool roundUp = halfwayBit && (belowHalfway || (nearest & 1u) != 0u);
    if (roundUp) {
        nearest = (nearest + 1u) & 3u;
    }

    double fraction = 0.0;
    double weight = 0.5;
    for (unsigned offset = 1u; offset <= 64u; offset++) {
        if (FixedBit(product, binaryPoint - offset)) {
            fraction += weight;
        }
        weight *= 0.5;
    }
    if (roundUp) {
        fraction -= 1.0;
    }

    if (std::signbit(input)) {
        fraction = -fraction;
        nearest = (0u - nearest) & 3u;
    }
    return {fraction * kHalfPi, nearest};
}

ReducedAngle ReduceAngle(float input) {
    if (std::fabs(input) > 1000000.0f) {
        return ReduceLargeAngle(input);
    }
    const double value = static_cast<double>(input);
    const double scaled = value * kTwoOverPi;
    const int64_t nearest =
            scaled >= 0.0 ? static_cast<int64_t>(scaled + 0.5) : static_cast<int64_t>(scaled - 0.5);
    const double count = static_cast<double>(nearest);
    return {
            (value - count * kHalfPiHigh) - count * kHalfPiLow,
            static_cast<unsigned>(nearest) & 3u,
    };
}

[[maybe_unused]] uint64_t RoundPositiveToNearestEven(double value) {
    const uint64_t whole = static_cast<uint64_t>(value);
    const double fraction = value - static_cast<double>(whole);
    return whole + (fraction > 0.5 || (fraction == 0.5 && (whole & 1u) != 0u));
}

} // namespace

namespace Binary32 {

NativeSqrtScope::NativeSqrtScope(void) noexcept
        : previous_(nativeSqrtEnabled) {
    nativeSqrtEnabled = true;
}

NativeSqrtScope::~NativeSqrtScope(void) {
    nativeSqrtEnabled = previous_;
}

float FromDouble(double value) {
    if (std::isnan(value))
        return std::copysign(QuietNaN(), value);
    return static_cast<float>(value);
}

float FromUnsignedInteger(uint32_t value) { return FromDouble(static_cast<double>(value)); }

uint32_t TruncateToUint32Modulo(float value) {
    if (!std::isfinite(value) || std::fabs(value) >= 18446744073709551616.0) {
        return 0u;
    }
    const double truncated = std::trunc(static_cast<double>(value));
    const double wrappedMagnitude = std::fmod(std::fabs(truncated), 4294967296.0);
    const uint32_t magnitude = static_cast<uint32_t>(wrappedMagnitude);
    return std::signbit(value) ? 0u - magnitude : magnitude;
}

bool HaveSameEncoding(float left, float right) {
    uint32_t leftEncoding = 0u;
    uint32_t rightEncoding = 0u;
    std::memcpy(&leftEncoding, &left, sizeof(leftEncoding));
    std::memcpy(&rightEncoding, &right, sizeof(rightEncoding));
    return leftEncoding == rightEncoding;
}

} // namespace Binary32

float CIsqrt(float value) {
#if TMNF_BINARY32_HAS_NATIVE_SQRT
    if (nativeSqrtEnabled) {
        uint32_t bits = 0u;
        std::memcpy(&bits, &value, sizeof(bits));
        constexpr uint32_t PositiveInfinityBits = 0x7f800000u;
        constexpr uint32_t NegativeZeroBits = 0x80000000u;
        if (bits <= PositiveInfinityBits || bits == NegativeZeroBits) {
            return _mm_cvtss_f32(_mm_sqrt_ss(_mm_set_ss(value)));
        }
    }
#endif
    return Binary32::FromDouble(std::sqrt(static_cast<double>(value)));
}

float CIacos(float value) {
    if (IsNaNFloat(value) || value < -1.0f || value > 1.0f) {
        return QuietNaN();
    }
    const float positiveFactor = 1.0f + value;
    const float negativeFactor = 1.0f - value;
    const float radical = CIsqrt(positiveFactor * negativeFactor);
    return CIatan2(radical, value);
}

float CIasin(float value) {
    if (IsNaNFloat(value) || value < -1.0f || value > 1.0f) {
        return QuietNaN();
    }
    const float positiveFactor = 1.0f + value;
    const float negativeFactor = 1.0f - value;
    const float radical = CIsqrt(positiveFactor * negativeFactor);
    return CIatan2(value, radical);
}

float CIatan2(float y, float x) {
    if (IsNaNFloat(x) || IsNaNFloat(y)) {
        return QuietNaN();
    }
    return Binary32::FromDouble(Atan2(static_cast<double>(y), static_cast<double>(x)));
}

float CItan(float value) {
    if (!IsFiniteFloat(value)) {
        return QuietNaN();
    }
    const ReducedAngle reduced = ReduceAngle(value);
    const double sine = SinPolynomial(reduced.value);
    const double cosine = CosPolynomial(reduced.value);
    const double result = (reduced.quadrant & 1u) == 0u ? sine / cosine : -cosine / sine;
    return Binary32::FromDouble(result);
}

float CIexp(float value) {
    if (IsNaNFloat(value)) {
        return value;
    }
    if (value > 89.0f) {
        return std::numeric_limits<float>::infinity();
    }
    if (value < -110.0f) {
        return 0.0f;
    }

    const double input = static_cast<double>(value);
    const double scaled = input * kLog2E;
    const int exponent =
            scaled >= 0.0 ? static_cast<int>(scaled + 0.5) : static_cast<int>(scaled - 0.5);
    const double reduced = (input - static_cast<double>(exponent) * kLn2High) -
                           static_cast<double>(exponent) * kLn2Low;

    double tail = 1.0 / 355687428096000.0;
    tail = 1.0 / 20922789888000.0 + reduced * tail;
    tail = 1.0 / 1307674368000.0 + reduced * tail;
    tail = 1.0 / 87178291200.0 + reduced * tail;
    tail = 1.0 / 6227020800.0 + reduced * tail;
    tail = 1.0 / 479001600.0 + reduced * tail;
    tail = 1.0 / 39916800.0 + reduced * tail;
    tail = 1.0 / 3628800.0 + reduced * tail;
    tail = 1.0 / 362880.0 + reduced * tail;
    tail = 1.0 / 40320.0 + reduced * tail;
    tail = 1.0 / 5040.0 + reduced * tail;
    tail = 1.0 / 720.0 + reduced * tail;
    tail = 1.0 / 120.0 + reduced * tail;
    tail = 1.0 / 24.0 + reduced * tail;
    tail = 1.0 / 6.0 + reduced * tail;
    tail = 0.5 + reduced * tail;
    const double polynomial = 1.0 + reduced * (1.0 + reduced * tail);
    return Binary32::FromDouble(polynomial * PowerOfTwo(exponent));
}

float CIfmod(float value, float modulus) { return std::fmod(value, modulus); }

float CIsin(float value) {
    if (!IsFiniteFloat(value)) {
        return QuietNaN();
    }
    const ReducedAngle reduced = ReduceAngle(value);
    double result;
    switch (reduced.quadrant) {
    case 0u:
        result = SinPolynomial(reduced.value);
        break;
    case 1u:
        result = CosPolynomial(reduced.value);
        break;
    case 2u:
        result = -SinPolynomial(reduced.value);
        break;
    default:
        result = -CosPolynomial(reduced.value);
        break;
    }
    return Binary32::FromDouble(result);
}

float CIcos(float value) {
    if (!IsFiniteFloat(value)) {
        return QuietNaN();
    }
    const ReducedAngle reduced = ReduceAngle(value);
    double result;
    switch (reduced.quadrant) {
    case 0u:
        result = CosPolynomial(reduced.value);
        break;
    case 1u:
        result = -SinPolynomial(reduced.value);
        break;
    case 2u:
        result = -CosPolynomial(reduced.value);
        break;
    default:
        result = SinPolynomial(reduced.value);
        break;
    }
    return Binary32::FromDouble(result);
}

#undef TMNF_BINARY32_HAS_NATIVE_SQRT
