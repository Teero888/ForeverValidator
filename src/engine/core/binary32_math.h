#ifndef TMNF_BINARY32_MATH_H
#define TMNF_BINARY32_MATH_H

#include <cfenv>
#include <cstring>
#include <stdint.h>

#if defined(__i386__) || defined(__x86_64__) || defined(_M_IX86) || \
        defined(_M_X64)
#include <xmmintrin.h>
#endif

namespace Binary32 {

class NativeSqrtScope final {
public:
    // Callers must already have established the deterministic round-to-nearest
    // binary32 environment. The scope is nestable and affects only CIsqrt.
    NativeSqrtScope(void) noexcept;
    ~NativeSqrtScope(void);

    NativeSqrtScope(const NativeSqrtScope &) = delete;
    NativeSqrtScope &operator=(const NativeSqrtScope &) = delete;

private:
    bool previous_ = false;
};

// Explicit round-to-nearest-even conversion for algorithms evaluated in
// double precision before producing a simulation value.
float FromDouble(double value);
float FromUnsignedInteger(uint32_t value);
uint32_t TruncateToUint32Modulo(float value);
bool HaveSameEncoding(float left, float right);

} // namespace Binary32

float CIsqrt(float value);
float CIacos(float value);
float CIasin(float value);
float CIcos(float value);
float CIsin(float value);

inline float CIsinQuarterPi(void) noexcept {
    constexpr uint32_t QuarterPiSineEncoding = 0x3f3504f3u;
    float value;
    std::memcpy(&value, &QuarterPiSineEncoding, sizeof(value));
#if defined(__i386__) || defined(__x86_64__) || defined(_M_IX86) || \
        defined(_M_X64)
#if defined(__GNUC__) || defined(__clang__)
    float accumulator = 1.0f;
    const float halfUlpAtOne = 0x1p-24f;
    // The exact CIsin(QuarterPi) path always raises only FE_INEXACT. A benign
    // halfway scalar addition reproduces that sticky-status effect without the
    // serializing MXCSR read/conditional write; its rounded result is ignored.
    __asm__ volatile(
            "addss %1, %0"
            : "+x"(accumulator)
            : "x"(halfUlpAtOne));
#else
    std::feraiseexcept(FE_INEXACT);
#endif
#else
    std::feraiseexcept(FE_INEXACT);
#endif
    return value;
}
float CIatan2(float y, float x);
float CItan(float value);
float CIexp(float value);
float CIfmod(float value, float modulus);

#endif // TMNF_BINARY32_MATH_H
