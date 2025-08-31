/**
 * @file FastMath.h
 * @brief Fast approximations for common math operations
 * @author Andrés Guerrero
 * @date 29-08-2025
 *
 * Provides optimized approximations for performance-critical paths
 * where exact precision is not required. Includes SIMD optimizations.
 */

#pragma once

#include "MathConstants.h"
#include "MathFunctions.h"
#include "MathTypes.h"

#ifdef __SSE__
#include <xmmintrin.h>  // SSE
#endif
#ifdef __SSE2__
#include <emmintrin.h>  // SSE2
#endif
#ifdef __SSE3__
#include <pmmintrin.h>  // SSE3
#endif
#ifdef __SSE4_1__
#include <smmintrin.h>  // SSE4.1
#endif
#ifdef __AVX__
#include <immintrin.h>  // AVX y superiores
#endif

namespace engine::math::fast {
    // ============================================================================
    // Fast Inverse Square Root (Quake III Algorithm)
    // ============================================================================

    /**
     * @brief Fast inverse square root approximation
     * Error: ~0.175% maximum relative error
     * Speed: ~4x faster than 1.0f/sqrt(x)
     */
    [[nodiscard]] inline Float fastInvSqrt(Float number) noexcept {
        const Float x2 = number * 0.5f;
        constexpr Float threehalfs = 1.5f;

        union {
            Float f;
            std::uint32_t i;
        } conv = {.f = number};

        conv.i = 0x5f3759df - (conv.i >> 1); // Magic constant
        conv.f = conv.f * (threehalfs - (x2 * conv.f * conv.f)); // 1st Newton-Raphson iteration
        // conv.f = conv.f * (threehalfs - (x2 * conv.f * conv.f)); // 2nd iteration for more precision
        return conv.f;
    }

    /**
     * @brief Fast square root using inverse square root
     */
    [[nodiscard]] inline Float fastSqrt(Float x) noexcept {
        return x * fastInvSqrt(x);
    }

    /**
     * @brief Fast vector normalization using fast inverse square root
     */
    [[nodiscard]] inline Vec3 fastNormalize(const Vec3& v) noexcept {
        const Float lengthSq = glm::dot(v, v);
        if (lengthSq < EPSILON_SQUARED) return Vec3(0);
        return v * fastInvSqrt(lengthSq);
    }

    // ============================================================================
    // Fast Trigonometric Approximations
    // ============================================================================

    /**
     * @brief Fast sine approximation using Taylor series
     * Valid for x in [-PI, PI]
     * Error: < 0.001
     */
    [[nodiscard]] inline Float fastSin(Float x) noexcept {
        // Wrap to [-PI, PI]
        if (x < -PI<Float>) x += TWO_PI<Float>;
        else if (x > PI<Float>) x -= TWO_PI<Float>;

        // Taylor series: sin(x) ≈ x - x³/6 + x⁵/120
        const Float x2 = x * x;
        const Float x3 = x2 * x;
        const Float x5 = x3 * x2;

        return x - (x3 * 0.16666667f) + (x5 * 0.00833333f);
    }

    /**
     * @brief Fast cosine approximation using Taylor series
     * Valid for x in [-PI, PI]
     * Error: < 0.001
     */
    [[nodiscard]] inline Float fastCos(Float x) noexcept {
        // Wrap to [-PI, PI]
        if (x < -PI<Float>) x += TWO_PI<Float>;
        else if (x > PI<Float>) x -= TWO_PI<Float>;

        // Taylor series: cos(x) ≈ 1 - x²/2 + x⁴/24
        const Float x2 = x * x;
        const Float x4 = x2 * x2;

        return 1.0f - (x2 * 0.5f) + (x4 * 0.04166667f);
    }

    /**
     * @brief Fast tangent approximation
     */
    [[nodiscard]] inline Float fastTan(const Float x) noexcept {
        return fastSin(x) / fastCos(x);
    }

    /**
     * @brief Fast arc tangent approximation
     * Error: < 0.005 radians
     */
    [[nodiscard]] inline Float fastAtan(const Float x) noexcept {
        const Float x2 = x * x;
        constexpr Float a = 0.0776509570923569f;
        constexpr Float b = -0.287434475393028f;
        constexpr Float c = (PI<Float> / 4.0f) - a - b;

        return ((a * x2 + b) * x2 + c) * x;
    }

    /**
     * @brief Fast atan2 approximation
     */
    [[nodiscard]] inline Float fastAtan2(const Float y, const Float x) noexcept {
        if (x == 0.0f) {
            if (y > 0.0f) return HALF_PI<Float>;
            if (y < 0.0f) return -HALF_PI<Float>;
            return 0.0f;
        }

        const Float atan = fastAtan(y / x);
        if (x < 0.0f) {
            if (y >= 0.0f) return atan + PI<Float>;
            else return atan - PI<Float>;
        }
        return atan;
    }

    // ============================================================================
    // Fast Exponential and Logarithm
    // ============================================================================

    /**
     * @brief Fast exponential approximation
     * Based on IEEE 754 floating-point representation
     */
    [[nodiscard]] inline Float fastExp(Float x) noexcept {
        // Clamp to prevent overflow
        x = clamp(x, -87.0f, 88.0f);

        // exp(x) = 2^(x * log2(e))
        x = x * LOG2E<Float>;

        // Split into integer and fractional parts
        const Float xi = std::floor(x);
        const Float xf = x - xi;

        // Approximate 2^xf using polynomial
        const Float y = 1.0f + xf * (0.6931472f + xf * (0.2402265f + xf * 0.0554906f));

        // Combine with 2^xi using bit manipulation
        union {
            Float f;
            Int i;
        } conv;

        conv.i = static_cast<Int>((xi + 127.0f) * (1 << 23));
        return y * conv.f;
    }

    /**
     * @brief Fast natural logarithm approximation
     */
    [[nodiscard]] inline Float fastLog(const Float x) noexcept {
        if (x <= 0) return -INFINITY_VALUE<Float>;

        union {
            Float f;
            Int i;
        } conv = {.f = x};

        const Float exp = static_cast<Float>((conv.i >> 23) - 127);
        conv.i = (conv.i & 0x007FFFFF) | 0x3F800000;

        const Float y = conv.f;
        const Float y2 = y * y;

        // TODO: Revisar si esta bien esta formula
        // Polynomial approximation for log(1+x)
        const Float result = -1.49278f + y * (2.11263f + y2 * (-0.729104f + y * 0.10969f));

        return result + exp * LN2<Float>;
    }

    /**
     * @brief Fast power approximation
     */
    [[nodiscard]] inline Float fastPow(const Float base, const Float exponent) noexcept {
        return fastExp(exponent * fastLog(base));
    }

    // ============================================================================
    // SIMD Vector Operations
    // ============================================================================

#ifdef __SSE4_1__
    /**
     * @brief SIMD dot product for Vec3
     */
    [[nodiscard]] inline Float simdDot3(const Vec3& a, const Vec3& b) noexcept {
        __m128 va = _mm_set_ps(0.0f, a.z, a.y, a.x);
        __m128 vb = _mm_set_ps(0.0f, b.z, b.y, b.x);
        __m128 dp = _mm_dp_ps(va, vb, 0x71);
        return _mm_cvtss_f32(dp);
    }

    /**
     * @brief SIMD cross product for Vec3
     */
    [[nodiscard]] inline Vec3 simdCross(const Vec3& a, const Vec3& b) noexcept {
        __m128 va = _mm_set_ps(0.0f, a.z, a.y, a.x);
        __m128 vb = _mm_set_ps(0.0f, b.z, b.y, b.x);

        __m128 tmp0 = _mm_shuffle_ps(va, va, _MM_SHUFFLE(3, 0, 2, 1));
        __m128 tmp1 = _mm_shuffle_ps(vb, vb, _MM_SHUFFLE(3, 1, 0, 2));
        __m128 tmp2 = _mm_shuffle_ps(va, va, _MM_SHUFFLE(3, 1, 0, 2));
        __m128 tmp3 = _mm_shuffle_ps(vb, vb, _MM_SHUFFLE(3, 0, 2, 1));

        __m128 result = _mm_sub_ps(_mm_mul_ps(tmp0, tmp1), _mm_mul_ps(tmp2, tmp3));

        alignas(16) Float res[4];
        _mm_store_ps(res, result);
        return Vec3(res[0], res[1], res[2]);
    }

    /**
     * @brief SIMD batch normalize (4 vectors at once)
     */
    inline void simdNormalizeBatch(Vec3* vectors, std::size_t count) noexcept {
        for (std::size_t i = 0; i < count; i += 4) {
            // Process 4 vectors at once
            __m128 x = _mm_set_ps(vectors[i + 3].x, vectors[i + 2].x, vectors[i + 1].x, vectors[i].x);
            __m128 y = _mm_set_ps(vectors[i + 3].y, vectors[i + 2].y, vectors[i + 1].y, vectors[i].y);
            __m128 z = _mm_set_ps(vectors[i + 3].z, vectors[i + 2].z, vectors[i + 1].z, vectors[i].z);

            // Calculate length squared
            __m128 lenSq = _mm_add_ps(_mm_add_ps(_mm_mul_ps(x, x), _mm_mul_ps(y, y)), _mm_mul_ps(z, z));

            // Inverse square root
            __m128 invLen = _mm_rsqrt_ps(lenSq);

            // Normalize
            x = _mm_mul_ps(x, invLen);
            y = _mm_mul_ps(y, invLen);
            z = _mm_mul_ps(z, invLen);

            // Store results
            alignas(16) Float xRes[4], yRes[4], zRes[4];
            _mm_store_ps(xRes, x);
            _mm_store_ps(yRes, y);
            _mm_store_ps(zRes, z);

            for (std::size_t j = 0; j < 4 && (i + j) < count; ++j) {
                vectors[i + j] = Vec3(xRes[j], yRes[j], zRes[j]);
            }
        }
    }
#endif

    // ============================================================================
    // Approximation Helpers
    // ============================================================================

    /**
     * @brief Fast absolute value using bit manipulation
     */
    [[nodiscard]] inline Float fastAbs(const Float x) noexcept {
        union {
            Float f;
            Int i;
        } conv = {.f = x};
        conv.i &= 0x7FFFFFFF;
        return conv.f;
    }

    /**
     * @brief Fast floor using truncation
     */
    [[nodiscard]] inline Int fastFloor(const Float x) noexcept {
        const Int i = static_cast<Int>(x);
        return (x < 0 && x != i) ? i - 1 : i;
    }

    /**
     * @brief Fast ceiling using truncation
     */
    [[nodiscard]] inline Int fastCeil(const Float x) noexcept {
        const Int i = static_cast<Int>(x);
        return (x > 0 && x != i) ? i + 1 : i;
    }

    /**
     * @brief Fast modulo for positive integers
     * Only works when divisor is power of 2
     */
    [[nodiscard]] inline constexpr Int fastMod(const Int value,
                                               const Int divisor) noexcept {
        return value & (divisor - 1);
    }

    /**
     * @brief Fast lerp without clamping
     */
    template <typename T>
    [[nodiscard]] inline constexpr T fastLerp(T a, T b, Float t) noexcept {
        return a + (b - a) * t;
    }

    /**
     * @brief Fast distance approximation (Manhattan distance)
     */
    [[nodiscard]] inline Float fastDistance2D(const Vec2& a, const Vec2& b) noexcept {
        const Float dx = fastAbs(b.x - a.x);
        const Float dy = fastAbs(b.y - a.y);
        return dx + dy;
    }

    /**
     * @brief Fast distance approximation (octagonal)
     * More accurate than Manhattan, faster than Euclidean
     */
    [[nodiscard]] inline Float fastDistanceOctagonal(const Vec2& a, const Vec2& b) noexcept {
        const Float dx = fastAbs(b.x - a.x);
        const Float dy = fastAbs(b.y - a.y);
        return 0.41f * (dx + dy) + 0.941246f * max(dx, dy);
    }
} // namespace engine::math::fast
