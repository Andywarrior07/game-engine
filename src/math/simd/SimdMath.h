/**
 * @file SimdMath.h
 * @brief Optimized SIMD batch math operations
 *
 * Provides vectorized implementations of common math operations
 * for processing multiple values simultaneously across multiple platforms:
 * - Apple Silicon (ARM64/NEON)
 * - x86/x64 (SSE2/SSE4.1/AVX/AVX2/AVX-512)
 * - Scalar fallback for other architectures
 */

#pragma once

#include "SimdTypes.h"
#include "../core/MathTypes.h"
#include "../core/MathConstants.h"
#include "../geometry/Primitives.h"

#include <cmath>
#include <algorithm>

namespace engine::math::simd {

    // ============================================================================
    // Basic SIMD Math Functions
    // ============================================================================

    /**
     * @brief SIMD square root
     */
    inline Float4 sqrt(const Float4& a) noexcept {
#if defined(SIMD_NEON)
        return Float4(vsqrtq_f32(a.v));
#elif defined(SIMD_SSE2) || defined(SIMD_AVX) || defined(SIMD_AVX2) || defined(SIMD_AVX512)
        return Float4(_mm_sqrt_ps(a.v));
#else
        return Float4(std::sqrt(a[0]), std::sqrt(a[1]), std::sqrt(a[2]), std::sqrt(a[3]));
#endif
    }

    /**
     * @brief SIMD reciprocal square root (fast approximation)
     */
    inline Float4 rsqrt(const Float4& a) noexcept {
#if defined(SIMD_NEON)
        // NEON reciprocal square root estimate + Newton-Raphson refinement
        float32x4_t estimate = vrsqrteq_f32(a.v);
        // One Newton-Raphson iteration: estimate * (1.5 - 0.5 * x * estimate^2)
        estimate = vmulq_f32(estimate, vrsqrtsq_f32(vmulq_f32(a.v, estimate), estimate));
        return Float4(estimate);
#elif defined(SIMD_SSE2) || defined(SIMD_AVX) || defined(SIMD_AVX2) || defined(SIMD_AVX512)
        return Float4(_mm_rsqrt_ps(a.v));
#else
        return Float4(1.0f / std::sqrt(a[0]), 1.0f / std::sqrt(a[1]),
                     1.0f / std::sqrt(a[2]), 1.0f / std::sqrt(a[3]));
#endif
    }

    /**
     * @brief SIMD reciprocal (1/x)
     */
    inline Float4 reciprocal(const Float4& a) noexcept {
#if defined(SIMD_NEON)
        // NEON reciprocal estimate + Newton-Raphson refinement
        float32x4_t estimate = vrecpeq_f32(a.v);
        estimate = vmulq_f32(vrecpsq_f32(a.v, estimate), estimate);
        return Float4(estimate);
#elif defined(SIMD_SSE2) || defined(SIMD_AVX) || defined(SIMD_AVX2) || defined(SIMD_AVX512)
        return Float4(_mm_rcp_ps(a.v));
#else
        return Float4(1.0f / a[0], 1.0f / a[1], 1.0f / a[2], 1.0f / a[3]);
#endif
    }

    /**
     * @brief SIMD min
     */
    inline Float4 min(const Float4& a, const Float4& b) noexcept {
#if defined(SIMD_NEON)
        return Float4(vminq_f32(a.v, b.v));
#elif defined(SIMD_SSE2) || defined(SIMD_AVX) || defined(SIMD_AVX2) || defined(SIMD_AVX512)
        return Float4(_mm_min_ps(a.v, b.v));
#else
        return Float4(std::min(a[0], b[0]), std::min(a[1], b[1]),
                     std::min(a[2], b[2]), std::min(a[3], b[3]));
#endif
    }

    /**
     * @brief SIMD max
     */
    inline Float4 max(const Float4& a, const Float4& b) noexcept {
#if defined(SIMD_NEON)
        return Float4(vmaxq_f32(a.v, b.v));
#elif defined(SIMD_SSE2) || defined(SIMD_AVX) || defined(SIMD_AVX2) || defined(SIMD_AVX512)
        return Float4(_mm_max_ps(a.v, b.v));
#else
        return Float4(std::max(a[0], b[0]), std::max(a[1], b[1]),
                     std::max(a[2], b[2]), std::max(a[3], b[3]));
#endif
    }

    /**
     * @brief SIMD absolute value
     */
    inline Float4 abs(const Float4& a) noexcept {
#if defined(SIMD_NEON)
        return Float4(vabsq_f32(a.v));
#elif defined(SIMD_SSE2) || defined(SIMD_AVX) || defined(SIMD_AVX2) || defined(SIMD_AVX512)
        const __m128 signMask = _mm_set1_ps(-0.0f);
        return Float4(_mm_andnot_ps(signMask, a.v));
#else
        return Float4(std::abs(a[0]), std::abs(a[1]), std::abs(a[2]), std::abs(a[3]));
#endif
    }

    /**
     * @brief SIMD floor
     */
    inline Float4 floor(const Float4& a) noexcept {
#if defined(SIMD_NEON)
        #if defined(__aarch64__)
            return Float4(vrndmq_f32(a.v)); // Round towards minus infinity
        #else
            // ARMv7 fallback
            Float4 result;
            for (int i = 0; i < 4; ++i) {
                result.data[i] = std::floor(a[i]);
            }
            return result;
        #endif
#elif defined(SIMD_SSE41)
        return Float4(_mm_floor_ps(a.v));
#elif defined(SIMD_SSE2) || defined(SIMD_AVX) || defined(SIMD_AVX2) || defined(SIMD_AVX512)
        // SSE2 fallback implementation
        Float4 result;
        for (int i = 0; i < 4; ++i) {
            result.data[i] = std::floor(a[i]);
        }
        return result;
#else
        return Float4(std::floor(a[0]), std::floor(a[1]), std::floor(a[2]), std::floor(a[3]));
#endif
    }

    /**
     * @brief SIMD ceil
     */
    inline Float4 ceil(const Float4& a) noexcept {
#if defined(SIMD_NEON)
        #if defined(__aarch64__)
            return Float4(vrndpq_f32(a.v)); // Round towards plus infinity
        #else
            Float4 result;
            for (int i = 0; i < 4; ++i) {
                result.data[i] = std::ceil(a[i]);
            }
            return result;
        #endif
#elif defined(SIMD_SSE41)
        return Float4(_mm_ceil_ps(a.v));
#elif defined(SIMD_SSE2) || defined(SIMD_AVX) || defined(SIMD_AVX2) || defined(SIMD_AVX512)
        Float4 result;
        for (int i = 0; i < 4; ++i) {
            result.data[i] = std::ceil(a[i]);
        }
        return result;
#else
        return Float4(std::ceil(a[0]), std::ceil(a[1]), std::ceil(a[2]), std::ceil(a[3]));
#endif
    }

    /**
     * @brief SIMD dot product
     */
    inline Float dot(const Float4& a, const Float4& b) noexcept {
#if defined(SIMD_NEON)
        float32x4_t mul = vmulq_f32(a.v, b.v);
        float32x2_t sum = vpadd_f32(vget_low_f32(mul), vget_high_f32(mul));
        sum = vpadd_f32(sum, sum);
        return vget_lane_f32(sum, 0);
#elif defined(SIMD_SSE41)
        return _mm_cvtss_f32(_mm_dp_ps(a.v, b.v, 0xFF));
#elif defined(SIMD_SSE2) || defined(SIMD_AVX) || defined(SIMD_AVX2) || defined(SIMD_AVX512)
        Float4 mul = a * b;
        Float4 shuf = Float4(_mm_shuffle_ps(mul.v, mul.v, _MM_SHUFFLE(2, 3, 0, 1)));
        Float4 sums = mul + shuf;
        shuf = Float4(_mm_movehl_ps(shuf.v, sums.v));
        sums = sums + shuf;
        return _mm_cvtss_f32(sums.v);
#else
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
#endif
    }

    /**
     * @brief SIMD cross product (3D vectors, w component ignored)
     */
    inline Float4 cross(const Float4& a, const Float4& b) noexcept {
#if defined(SIMD_NEON)
        // a = [ax, ay, az, aw], b = [bx, by, bz, bw]
        // result = [ay*bz - az*by, az*bx - ax*bz, ax*by - ay*bx, 0]

        // Shuffle: a_yzx = [ay, az, ax, aw], b_zxy = [bz, bx, by, bw]
        float32x4_t a_yzx = vextq_f32(a.v, a.v, 1); // This gets [ay, az, aw, ax] - need custom shuffle
        float32x4_t b_zxy = vextq_f32(b.v, b.v, 2); // This gets [az, aw, bx, by] - need custom shuffle

        // Manual implementation for NEON
        Float4 result;
        result.data[0] = a.data[1] * b.data[2] - a.data[2] * b.data[1]; // ay*bz - az*by
        result.data[1] = a.data[2] * b.data[0] - a.data[0] * b.data[2]; // az*bx - ax*bz
        result.data[2] = a.data[0] * b.data[1] - a.data[1] * b.data[0]; // ax*by - ay*bx
        result.data[3] = 0.0f;
        return result;
#elif defined(SIMD_SSE2) || defined(SIMD_AVX) || defined(SIMD_AVX2) || defined(SIMD_AVX512)
        __m128 tmp0 = _mm_shuffle_ps(a.v, a.v, _MM_SHUFFLE(3, 0, 2, 1)); // [ay, az, ax, aw]
        __m128 tmp1 = _mm_shuffle_ps(b.v, b.v, _MM_SHUFFLE(3, 1, 0, 2)); // [bz, bx, by, bw]
        __m128 tmp2 = _mm_shuffle_ps(a.v, a.v, _MM_SHUFFLE(3, 1, 0, 2)); // [az, ax, ay, aw]
        __m128 tmp3 = _mm_shuffle_ps(b.v, b.v, _MM_SHUFFLE(3, 0, 2, 1)); // [by, bz, bx, bw]

        return Float4(_mm_sub_ps(_mm_mul_ps(tmp0, tmp1), _mm_mul_ps(tmp2, tmp3)));
#else
        return Float4(
            a[1] * b[2] - a[2] * b[1], // ay*bz - az*by
            a[2] * b[0] - a[0] * b[2], // az*bx - ax*bz
            a[0] * b[1] - a[1] * b[0], // ax*by - ay*bx
            0.0f
        );
#endif
    }

    /**
     * @brief SIMD normalize (3D vector, w component ignored)
     */
    inline Float4 normalize(const Float4& a) noexcept {
#if defined(SIMD_NEON)
        // Calculate dot product for length squared (only xyz components)
        Float4 xyz = Float4(a.data[0], a.data[1], a.data[2], 0.0f);
        Float lengthSq = dot(xyz, xyz);
        Float4 lengthSqVec(lengthSq);
        Float4 invLength = rsqrt(lengthSqVec);
        return Float4(a.data[0] * invLength[0], a.data[1] * invLength[0],
                     a.data[2] * invLength[0], a.data[3]);
#elif defined(SIMD_SSE41)
        Float4 lengthSq = Float4(_mm_dp_ps(a.v, a.v, 0x7F)); // Mask for xyz only
        return a * rsqrt(lengthSq);
#elif defined(SIMD_SSE2) || defined(SIMD_AVX) || defined(SIMD_AVX2) || defined(SIMD_AVX512)
        Float4 xyz = Float4(a.data[0], a.data[1], a.data[2], 0.0f);
        Float lengthSq = dot(xyz, xyz);
        Float4 lengthSqVec(lengthSq);
        return a * rsqrt(lengthSqVec);
#else
        Float lengthSq = a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
        Float invLength = 1.0f / std::sqrt(lengthSq);
        return Float4(a[0] * invLength, a[1] * invLength, a[2] * invLength, a[3]);
#endif
    }

    // ============================================================================
    // Batch Operations
    // ============================================================================

    /**
     * @brief Batch normalize vectors (Vec3 array)
     */
    inline void batchNormalize(Vec3* vectors, std::size_t count) noexcept {
        std::size_t i = 0;

        // Process 4 vectors at a time using SIMD
        for (; i + 3 < count; i += 4) {
            // Load 4 Vec3s into separate Float4s
            Float4 x(vectors[i].x, vectors[i+1].x, vectors[i+2].x, vectors[i+3].x);
            Float4 y(vectors[i].y, vectors[i+1].y, vectors[i+2].y, vectors[i+3].y);
            Float4 z(vectors[i].z, vectors[i+1].z, vectors[i+2].z, vectors[i+3].z);

            // Calculate length squared
            Float4 lenSq = x * x + y * y + z * z;

            // Calculate inverse length
            Float4 invLen = rsqrt(lenSq);

            // Normalize
            x = x * invLen;
            y = y * invLen;
            z = z * invLen;

            // Store back
            for (std::size_t j = 0; j < 4; ++j) {
                vectors[i + j].x = x[j];
                vectors[i + j].y = y[j];
                vectors[i + j].z = z[j];
            }
        }

        // Handle remaining vectors
        for (; i < count; ++i) {
            Float lenSq = vectors[i].x * vectors[i].x +
                         vectors[i].y * vectors[i].y +
                         vectors[i].z * vectors[i].z;
            Float invLen = 1.0f / std::sqrt(lenSq);
            vectors[i].x *= invLen;
            vectors[i].y *= invLen;
            vectors[i].z *= invLen;
        }
    }

    /**
     * @brief Batch dot product
     */
    inline void batchDot(const Vec3* a, const Vec3* b, Float* results, std::size_t count) noexcept {
        std::size_t i = 0;

        // Process 4 dot products at a time
        for (; i + 3 < count; i += 4) {
            Float4 ax(a[i].x, a[i+1].x, a[i+2].x, a[i+3].x);
            Float4 ay(a[i].y, a[i+1].y, a[i+2].y, a[i+3].y);
            Float4 az(a[i].z, a[i+1].z, a[i+2].z, a[i+3].z);

            Float4 bx(b[i].x, b[i+1].x, b[i+2].x, b[i+3].x);
            Float4 by(b[i].y, b[i+1].y, b[i+2].y, b[i+3].y);
            Float4 bz(b[i].z, b[i+1].z, b[i+2].z, b[i+3].z);

            Float4 dots = ax * bx + ay * by + az * bz;
            dots.store(&results[i]);
        }

        // Handle remaining
        for (; i < count; ++i) {
            results[i] = a[i].x * b[i].x + a[i].y * b[i].y + a[i].z * b[i].z;
        }
    }

    /**
     * @brief Batch matrix multiply (4x4 matrices)
     */
    inline void batchMatrixMultiply(const Mat4* a, const Mat4* b, Mat4* results, std::size_t count) noexcept {
        for (std::size_t i = 0; i < count; ++i) {
            Matrix4x4 matA(a[i]);
            Matrix4x4 matB(b[i]);
            Matrix4x4 result = matA * matB;
            results[i] = result.toMat4();
        }
    }

    /**
     * @brief Batch linear interpolation
     */
    inline void batchLerp(const Float* a, const Float* b, Float* results, Float t, std::size_t count) noexcept {
        const Float4 tVec(t);
        const Float4 oneMinusT(1.0f - t);

        std::size_t i = 0;

        // Process 4 values at a time
        for (; i + 3 < count; i += 4) {
            Float4 aVec = Float4::load(&a[i]);
            Float4 bVec = Float4::load(&b[i]);
            Float4 result = aVec * oneMinusT + bVec * tVec;
            result.store(&results[i]);
        }

        // Handle remaining
        for (; i < count; ++i) {
            results[i] = a[i] * (1.0f - t) + b[i] * t;
        }
    }

    /**
     * @brief Batch sine approximation using Taylor series
     */
    inline void batchSin(const Float* input, Float* output, std::size_t count) noexcept {
        // Taylor series coefficients for sin(x)
        const Float4 c1(1.0f);
        const Float4 c2(-0.166666667f);    // -1/6
        const Float4 c3(0.008333333f);     //  1/120
        const Float4 c4(-0.000198413f);    // -1/5040

        std::size_t i = 0;

        for (; i + 3 < count; i += 4) {
            Float4 x = Float4::load(&input[i]);

            // Range reduction to [-PI, PI]
            const Float4 invTwoPi(1.0f / (2.0f * PI<Float>));
            Float4 cycles = floor(x * invTwoPi + Float4(0.5f));
            x = x - cycles * Float4(2.0f * PI<Float>);

            // Taylor series: sin(x) ≈ x - x³/6 + x⁵/120 - x⁷/5040
            Float4 x2 = x * x;
            Float4 x3 = x2 * x;
            Float4 x5 = x3 * x2;
            Float4 x7 = x5 * x2;

            Float4 result = x * c1 + x3 * c2 + x5 * c3 + x7 * c4;
            result.store(&output[i]);
        }

        // Handle remaining
        for (; i < count; ++i) {
            Float x = input[i];

            // Range reduction
            x = std::fmod(x + PI<Float>, 2.0f * PI<Float>) - PI<Float>;

            // Taylor series
            Float x2 = x * x;
            Float x3 = x2 * x;
            Float x5 = x3 * x2;
            Float x7 = x5 * x2;

            output[i] = x - x3/6.0f + x5/120.0f - x7/5040.0f;
        }
    }

    /**
     * @brief Batch AABB vs frustum culling
     */
    inline void batchFrustumCull(const AABB* boxes, const Frustum& frustum, bool* visible, std::size_t count) noexcept {
        for (std::size_t i = 0; i < count; ++i) {
            const AABB& box = boxes[i];
            visible[i] = true;

            // Test against each frustum plane
            for (const auto& plane : frustum.planes) {
                // Find the positive vertex (farthest from plane)
                Vec3 positive = box.min;
                if (plane.normal.x >= 0) positive.x = box.max.x;
                if (plane.normal.y >= 0) positive.y = box.max.y;
                if (plane.normal.z >= 0) positive.z = box.max.z;

                // If positive vertex is behind plane, box is completely outside
                if (plane.getSignedDistance(positive) < 0) {
                    visible[i] = false;
                    break;
                }
            }
        }
    }

    // ============================================================================
    // AVX-specific optimized functions (8-wide processing)
    // ============================================================================

#if defined(SIMD_AVX) || defined(SIMD_AVX2) || defined(SIMD_AVX512)
    /**
     * @brief AVX batch normalize (processes 8 vectors at once)
     */
    inline void batchNormalizeAVX(Vec3* vectors, std::size_t count) noexcept {
        std::size_t i = 0;

        // Process 8 vectors at a time with AVX
        for (; i + 7 < count; i += 8) {
            // Load 8 Vec3s into Float8s
            Float8 x, y, z;
            for (std::size_t j = 0; j < 8; ++j) {
                x.data[j] = vectors[i + j].x;
                y.data[j] = vectors[i + j].y;
                z.data[j] = vectors[i + j].z;
            }

            // Calculate length squared
            Float8 lenSq = x * x + y * y + z * z;

            // Calculate inverse length using AVX rsqrt
            Float8 invLen = Float8(_mm256_rsqrt_ps(lenSq.v));

            // Normalize
            x = x * invLen;
            y = y * invLen;
            z = z * invLen;

            // Store back
            for (std::size_t j = 0; j < 8; ++j) {
                vectors[i + j].x = x.data[j];
                vectors[i + j].y = y.data[j];
                vectors[i + j].z = z.data[j];
            }
        }

        // Fall back to regular batch normalize for remaining vectors
        if (i < count) {
            batchNormalize(&vectors[i], count - i);
        }
    }

    /**
     * @brief AVX batch lerp (8-wide)
     */
    inline void batchLerpAVX(const Float* a, const Float* b, Float* results, Float t, std::size_t count) noexcept {
        const Float8 tVec(t);
        const Float8 oneMinusT(1.0f - t);

        std::size_t i = 0;

        // Process 8 values at a time
        for (; i + 7 < count; i += 8) {
            Float8 aVec = Float8::load(&a[i]);
            Float8 bVec = Float8::load(&b[i]);
            Float8 result = aVec * oneMinusT + bVec * tVec;
            result.store(&results[i]);
        }

        // Handle remaining with regular lerp
        if (i < count) {
            batchLerp(&a[i], &b[i], &results[i], t, count - i);
        }
    }
#endif

    // ============================================================================
    // Utility Functions
    // ============================================================================

    /**
     * @brief Check if all elements in Float4 are approximately equal to zero
     */
    inline bool isNearlyZero(const Float4& v, Float epsilon = EPSILON) noexcept {
        Float4 absV = abs(v);
        Float4 epsilonVec(epsilon);
        Float4 cmp = absV < epsilonVec;

        // Check if all components are less than epsilon
#if defined(SIMD_NEON)
        uint32x4_t mask = vreinterpretq_u32_f32(cmp.v);
        uint32x2_t tmp = vand_u32(vget_low_u32(mask), vget_high_u32(mask));
        return vget_lane_u32(vand_u32(tmp, vext_u32(tmp, tmp, 1)), 0) == 0xFFFFFFFF;
#elif defined(SIMD_SSE2) || defined(SIMD_AVX) || defined(SIMD_AVX2) || defined(SIMD_AVX512)
        int mask = _mm_movemask_ps(cmp.v);
        return mask == 0xF;
#else
        return absV[0] < epsilon && absV[1] < epsilon && absV[2] < epsilon && absV[3] < epsilon;
#endif
    }

    /**
     * @brief Horizontal sum of Float4
     */
    inline Float horizontalSum(const Float4& v) noexcept {
#if defined(SIMD_NEON)
        float32x2_t sum = vpadd_f32(vget_low_f32(v.v), vget_high_f32(v.v));
        sum = vpadd_f32(sum, sum);
        return vget_lane_f32(sum, 0);
#elif defined(SIMD_SSE3)
        __m128 shuf = _mm_movehdup_ps(v.v);        // [y, y, w, w]
        __m128 sums = _mm_add_ps(v.v, shuf);       // [x+y, y+y, z+w, w+w]
        shuf = _mm_movehl_ps(shuf, sums);          // [z+w, w+w, w, w]
        sums = _mm_add_ss(sums, shuf);             // [x+y+z+w, ?, ?, ?]
        return _mm_cvtss_f32(sums);
#elif defined(SIMD_SSE2) || defined(SIMD_AVX) || defined(SIMD_AVX2) || defined(SIMD_AVX512)
        __m128 shuf = _mm_shuffle_ps(v.v, v.v, _MM_SHUFFLE(2, 3, 0, 1));
        __m128 sums = _mm_add_ps(v.v, shuf);
        shuf = _mm_movehl_ps(shuf, sums);
        sums = _mm_add_ss(sums, shuf);
        return _mm_cvtss_f32(sums);
#else
        return v[0] + v[1] + v[2] + v[3];
#endif
    }

    /**
     * @brief Clamp Float4 values between min and max
     */
    inline Float4 clamp(const Float4& value, const Float4& minVal, const Float4& maxVal) noexcept {
        return min(max(value, minVal), maxVal);
    }

    /**
     * @brief Saturate Float4 values to [0, 1] range
     */
    inline Float4 saturate(const Float4& value) noexcept {
        return clamp(value, Float4(0.0f), Float4(1.0f));
    }

} // namespace engine::math::simd