/**
 * @file SimdTypes_portable.h
 * @brief Portable SIMD wrapper types (x86 SSE/AVX, ARM NEON, scalar fallback)
 *
 * This header provides small vector types and helpers that compile on:
 *  - Apple Silicon (ARM64/NEON)
 *  - x86/x64 (SSE2/SSE4.1/AVX/AVX2/AVX-512 when available)
 *  - Windows, Linux, macOS
 *
 * It assumes your engine defines in "../core/MathTypes.h":
 *   using Float = float; using Int = std::int32_t; and types Vec4, Mat4
 *
 * Design notes:
 *  - We detect the target ISA at compile time and include the correct
 *    intrinsics headers. No x86 intrinsics are referenced when building
 *    for ARM, and vice versa.
 *  - Float4 exists on all targets (NEON or SSE).
 *  - Float8 is provided only when AVX is available on x86; on other targets
 *    just do not compile paths that depend on Float8, or wrap them in #ifdefs.
 *  - Int4 exists on all targets.
 *  - There is a scalar fallback when no SIMD is available.
 */

#pragma once

#include "../core/MathTypes.h"

#include <cstdint>

// ============================
// ISA detection & includes
// ============================
#if defined(__ARM_NEON) || defined(__aarch64__)
  #define SIMD_NEON 1
  #define SIMD_WIDTH 4
  #include <arm_neon.h>
#elif defined(__AVX512F__)
  #define SIMD_AVX512 1
  #define SIMD_WIDTH 16
  #include <immintrin.h>
#elif defined(__AVX2__)
  #define SIMD_AVX2 1
  #define SIMD_WIDTH 8
  #include <immintrin.h>
#elif defined(__AVX__)
  #define SIMD_AVX 1
  #define SIMD_WIDTH 8
  #include <immintrin.h>
#elif defined(__SSE2__)
  #define SIMD_SSE2 1
  #define SIMD_WIDTH 4
  #include <emmintrin.h>
  #include <xmmintrin.h>
  #if defined(__SSE4_1__)
    #define SIMD_SSE41 1
    #include <smmintrin.h>
  #endif
#else
  #define SIMD_SCALAR 1
  #define SIMD_WIDTH 1
#endif

namespace engine::math::simd {

// ================================================
// Common helpers
// ================================================

#if defined(SIMD_NEON)

// --------------------------------
// Float4 (NEON)
// --------------------------------
class alignas(16) Float4 {
public:
    union {
        float32x4_t v;
        Float data[4];
        struct { Float x, y, z, w; };
    };

    // Constructors
    Float4() noexcept { v = vdupq_n_f32(0.0f); }
    explicit Float4(Float s) noexcept { v = vdupq_n_f32(s); }
    Float4(Float _x, Float _y, Float _z, Float _w) noexcept {
        float tmp[4] = {_x, _y, _z, _w};
        v = vld1q_f32(tmp);
    }
    explicit Float4(float32x4_t vec) noexcept { v = vec; }

    explicit Float4(const Vec4& vec) noexcept {
        float tmp[4] = {vec.x, vec.y, vec.z, vec.w};
        v = vld1q_f32(tmp);
    }

    // Load/Store
    static Float4 load(const Float* p) noexcept { return Float4(vld1q_f32(p)); }
    static Float4 loadAligned(const Float* p) noexcept { return Float4(vld1q_f32(p)); }
    void store(Float* p) const noexcept { vst1q_f32(p, v); }
    void storeAligned(Float* p) const noexcept { vst1q_f32(p, v); }

    // Arithmetic
    Float4 operator+(const Float4& o) const noexcept { return Float4(vaddq_f32(v, o.v)); }
    Float4 operator-(const Float4& o) const noexcept { return Float4(vsubq_f32(v, o.v)); }
    Float4 operator*(const Float4& o) const noexcept { return Float4(vmulq_f32(v, o.v)); }
    Float4 operator/(const Float4& o) const noexcept {
    #if defined(__aarch64__)
        return Float4(vdivq_f32(v, o.v));
    #else
        // Reciprocal approximation + 2 NR steps (ARMv7)
        float32x4_t recip = vrecpeq_f32(o.v);
        recip = vmulq_f32(vrecpsq_f32(o.v, recip), recip);
        recip = vmulq_f32(vrecpsq_f32(o.v, recip), recip);
        return Float4(vmulq_f32(v, recip));
    #endif
    }

    Float4& operator+=(const Float4& o) noexcept { v = vaddq_f32(v, o.v); return *this; }
    Float4& operator-=(const Float4& o) noexcept { v = vsubq_f32(v, o.v); return *this; }
    Float4& operator*=(const Float4& o) noexcept { v = vmulq_f32(v, o.v); return *this; }
    Float4& operator/=(const Float4& o) noexcept {
        *this = (*this) / o; return *this;
    }

    // Comparison (return mask-as-floats like SSE behavior)
    Float4 operator<(const Float4& o) const noexcept { return Float4(vreinterpretq_f32_u32(vcltq_f32(v, o.v))); }
    Float4 operator<=(const Float4& o) const noexcept { return Float4(vreinterpretq_f32_u32(vcleq_f32(v, o.v))); }
    Float4 operator>(const Float4& o) const noexcept { return Float4(vreinterpretq_f32_u32(vcgtq_f32(v, o.v))); }
    Float4 operator>=(const Float4& o) const noexcept { return Float4(vreinterpretq_f32_u32(vcgeq_f32(v, o.v))); }
    Float4 operator==(const Float4& o) const noexcept { return Float4(vreinterpretq_f32_u32(vceqq_f32(v, o.v))); }

    // Conversion
    [[nodiscard]] Vec4 toVec4() const noexcept {
        alignas(16) Float t[4]; storeAligned(t); return {t[0], t[1], t[2], t[3]};
    }

    // Element access (via union)
    Float operator[](int i) const noexcept { return data[i]; }
    Float& operator[](int i) noexcept { return data[i]; }
};

// --------------------------------
// Int4 (NEON)
// --------------------------------
class alignas(16) Int4 {
public:
    union {
        int32x4_t v;
        Int data[4];
        struct { Int x, y, z, w; };
    };

    Int4() noexcept { v = vdupq_n_s32(0); }
    explicit Int4(Int s) noexcept { v = vdupq_n_s32(s); }
    Int4(Int _x, Int _y, Int _z, Int _w) noexcept {
        int32_t tmp[4] = {_x, _y, _z, _w}; v = vld1q_s32(tmp);
    }
    explicit Int4(int32x4_t vec) noexcept { v = vec; }

    // Arithmetic
    Int4 operator+(const Int4& o) const noexcept { return Int4(vaddq_s32(v, o.v)); }
    Int4 operator-(const Int4& o) const noexcept { return Int4(vsubq_s32(v, o.v)); }
    Int4 operator*(const Int4& o) const noexcept { return Int4(vmulq_s32(v, o.v)); }

    // Bitwise
    Int4 operator&(const Int4& o) const noexcept { return Int4(vandq_s32(v, o.v)); }
    Int4 operator|(const Int4& o) const noexcept { return Int4(vorrq_s32(v, o.v)); }
    Int4 operator^(const Int4& o) const noexcept { return Int4(veorq_s32(v, o.v)); }

    // Shifts (variable counts)
    Int4 operator<<(int c) const noexcept { return Int4(vshlq_s32(v, vdupq_n_s32(c))); }
    Int4 operator>>(int c) const noexcept {
        // Logical right shift using unsigned lane type
        uint32x4_t u = vreinterpretq_u32_s32(v);
        // Negative shift for right shift with vshl
        auto r = vshlq_u32(u, vdupq_n_s32(-c));
        return Int4(vreinterpretq_s32_u32(r));
    }
};

#elif defined(SIMD_SSE2) || defined(SIMD_AVX) || defined(SIMD_AVX2) || defined(SIMD_AVX512)

// --------------------------------
// Float4 (SSE)
// --------------------------------
class alignas(16) Float4 {
public:
    union {
        __m128 v;
        Float data[4];
        struct { Float x, y, z, w; };
    };

    Float4() noexcept : v(_mm_setzero_ps()) {}
    explicit Float4(Float s) noexcept : v(_mm_set1_ps(s)) {}
    Float4(Float _x, Float _y, Float _z, Float _w) noexcept : v(_mm_set_ps(_w, _z, _y, _x)) {}
    explicit Float4(__m128 vec) noexcept : v(vec) {}
    explicit Float4(const Vec4& vec) noexcept : v(_mm_set_ps(vec.w, vec.z, vec.y, vec.x)) {}

    static Float4 load(const Float* p) noexcept { return Float4(_mm_loadu_ps(p)); }
    static Float4 loadAligned(const Float* p) noexcept { return Float4(_mm_load_ps(p)); }
    void store(Float* p) const noexcept { _mm_storeu_ps(p, v); }
    void storeAligned(Float* p) const noexcept { _mm_store_ps(p, v); }

    Float4 operator+(const Float4& o) const noexcept { return Float4(_mm_add_ps(v, o.v)); }
    Float4 operator-(const Float4& o) const noexcept { return Float4(_mm_sub_ps(v, o.v)); }
    Float4 operator*(const Float4& o) const noexcept { return Float4(_mm_mul_ps(v, o.v)); }
    Float4 operator/(const Float4& o) const noexcept { return Float4(_mm_div_ps(v, o.v)); }

    Float4& operator+=(const Float4& o) noexcept { v = _mm_add_ps(v, o.v); return *this; }
    Float4& operator-=(const Float4& o) noexcept { v = _mm_sub_ps(v, o.v); return *this; }
    Float4& operator*=(const Float4& o) noexcept { v = _mm_mul_ps(v, o.v); return *this; }
    Float4& operator/=(const Float4& o) noexcept { v = _mm_div_ps(v, o.v); return *this; }

    Float4 operator<(const Float4& o) const noexcept { return Float4(_mm_cmplt_ps(v, o.v)); }
    Float4 operator<=(const Float4& o) const noexcept { return Float4(_mm_cmple_ps(v, o.v)); }
    Float4 operator>(const Float4& o) const noexcept { return Float4(_mm_cmpgt_ps(v, o.v)); }
    Float4 operator>=(const Float4& o) const noexcept { return Float4(_mm_cmpge_ps(v, o.v)); }
    Float4 operator==(const Float4& o) const noexcept { return Float4(_mm_cmpeq_ps(v, o.v)); }

    [[nodiscard]] Vec4 toVec4() const noexcept {
        alignas(16) Float t[4]; storeAligned(t); return {t[0], t[1], t[2], t[3]};
    }

    Float operator[](int i) const noexcept { return data[i]; }
    Float& operator[](int i) noexcept { return data[i]; }
};

// --------------------------------
// Float8 (AVX) — only when AVX is enabled
// --------------------------------
#if defined(SIMD_AVX) || defined(SIMD_AVX2) || defined(SIMD_AVX512)
class alignas(32) Float8 {
public:
    union { __m256 v; Float data[8]; };

    Float8() noexcept : v(_mm256_setzero_ps()) {}
    explicit Float8(Float s) noexcept : v(_mm256_set1_ps(s)) {}
    explicit Float8(__m256 vec) noexcept : v(vec) {}
    Float8(Float a, Float b, Float c, Float d, Float e, Float f, Float g, Float h) noexcept
      : v(_mm256_set_ps(h,g,f,e,d,c,b,a)) {}

    static Float8 load(const Float* p) noexcept { return Float8(_mm256_loadu_ps(p)); }
    static Float8 loadAligned(const Float* p) noexcept { return Float8(_mm256_load_ps(p)); }
    void store(Float* p) const noexcept { _mm256_storeu_ps(p, v); }
    void storeAligned(Float* p) const noexcept { _mm256_store_ps(p, v); }

    Float8 operator+(const Float8& o) const noexcept { return Float8(_mm256_add_ps(v, o.v)); }
    Float8 operator-(const Float8& o) const noexcept { return Float8(_mm256_sub_ps(v, o.v)); }
    Float8 operator*(const Float8& o) const noexcept { return Float8(_mm256_mul_ps(v, o.v)); }
    Float8 operator/(const Float8& o) const noexcept { return Float8(_mm256_div_ps(v, o.v)); }

    // FMA if available
    Float8 fma(const Float8& b, const Float8& c) const noexcept {
    #if defined(__FMA__)
        return Float8(_mm256_fmadd_ps(v, b.v, c.v));
    #else
        return (*this) * b + c;
    #endif
    }

    // Horizontal sum
    Float sum() const noexcept {
        __m128 low = _mm256_castps256_ps128(v);
        __m128 high = _mm256_extractf128_ps(v, 1);
        __m128 s = _mm_add_ps(low, high);
        s = _mm_hadd_ps(s, s);
        s = _mm_hadd_ps(s, s);
        return _mm_cvtss_f32(s);
    }
};
#endif // AVX

// --------------------------------
// Int4 (SSE2)
// --------------------------------
class alignas(16) Int4 {
public:
    union {
        __m128i v; Int data[4]; struct { Int x, y, z, w; };
    };

    Int4() noexcept : v(_mm_setzero_si128()) {}
    explicit Int4(Int s) noexcept : v(_mm_set1_epi32(s)) {}
    Int4(Int _x, Int _y, Int _z, Int _w) noexcept : v(_mm_set_epi32(_w, _z, _y, _x)) {}
    explicit Int4(__m128i vec) noexcept : v(vec) {}

    Int4 operator+(const Int4& o) const noexcept { return Int4(_mm_add_epi32(v, o.v)); }
    Int4 operator-(const Int4& o) const noexcept { return Int4(_mm_sub_epi32(v, o.v)); }

    Int4 operator*(const Int4& o) const noexcept {
    #if defined(SIMD_SSE41)
        return Int4(_mm_mullo_epi32(v, o.v));
    #else
        // SSE2 fallback multiply 32-bit: do two 64-bit muls and pack
        __m128i a13 = _mm_shuffle_epi32(v, _MM_SHUFFLE(2,3,0,1));
        __m128i b13 = _mm_shuffle_epi32(o.v, _MM_SHUFFLE(2,3,0,1));
        __m128i prod02 = _mm_mul_epu32(v, o.v);
        __m128i prod13 = _mm_mul_epu32(a13, b13);
        __m128i res = _mm_unpacklo_epi32(_mm_shuffle_epi32(prod02, _MM_SHUFFLE(0,0,2,0)),
                                         _mm_shuffle_epi32(prod13, _MM_SHUFFLE(0,0,2,0)));
        return Int4(res);
    #endif
    }

    Int4 operator&(const Int4& o) const noexcept { return Int4(_mm_and_si128(v, o.v)); }
    Int4 operator|(const Int4& o) const noexcept { return Int4(_mm_or_si128(v, o.v)); }
    Int4 operator^(const Int4& o) const noexcept { return Int4(_mm_xor_si128(v, o.v)); }

    Int4 operator<<(int c) const noexcept { return Int4(_mm_slli_epi32(v, c)); }
    Int4 operator>>(int c) const noexcept { return Int4(_mm_srli_epi32(v, c)); }
};

#elif defined(SIMD_SCALAR)

// --------------------------------
// Scalar fallback
// --------------------------------
class Float4 {
public:
    Float data[4]{};
    Float4() noexcept = default;
    explicit Float4(Float s) noexcept { data[0]=data[1]=data[2]=data[3]=s; }
    Float4(Float x, Float y, Float z, Float w) noexcept { data[0]=x; data[1]=y; data[2]=z; data[3]=w; }
    explicit Float4(const Vec4& v) noexcept { data[0]=v.x; data[1]=v.y; data[2]=v.z; data[3]=v.w; }

    static Float4 load(const Float* p) noexcept { return Float4(p[0],p[1],p[2],p[3]); }
    static Float4 loadAligned(const Float* p) noexcept { return load(p); }
    void store(Float* p) const noexcept { p[0]=data[0]; p[1]=data[1]; p[2]=data[2]; p[3]=data[3]; }
    void storeAligned(Float* p) const noexcept { store(p); }

    Float4 operator+(const Float4& o) const noexcept { return {data[0]+o.data[0], data[1]+o.data[1], data[2]+o.data[2], data[3]+o.data[3]}; }
    Float4 operator-(const Float4& o) const noexcept { return {data[0]-o.data[0], data[1]-o.data[1], data[2]-o.data[2], data[3]-o.data[3]}; }
    Float4 operator*(const Float4& o) const noexcept { return {data[0]*o.data[0], data[1]*o.data[1], data[2]*o.data[2], data[3]*o.data[3]}; }
    Float4 operator/(const Float4& o) const noexcept { return {data[0]/o.data[0], data[1]/o.data[1], data[2]/o.data[2], data[3]/o.data[3]}; }

    Float& operator[](int i) noexcept { return data[i]; }
    Float operator[](int i) const noexcept { return data[i]; }

    [[nodiscard]] Vec4 toVec4() const noexcept { return {data[0],data[1],data[2],data[3]}; }
};

class Int4 {
public:
    Int data[4]{};
    Int4() noexcept = default;
    explicit Int4(Int s) noexcept { data[0]=data[1]=data[2]=data[3]=s; }
    Int4(Int x, Int y, Int z, Int w) noexcept { data[0]=x; data[1]=y; data[2]=z; data[3]=w; }

    Int4 operator+(const Int4& o) const noexcept { return {data[0]+o.data[0], data[1]+o.data[1], data[2]+o.data[2], data[3]+o.data[3]}; }
    Int4 operator-(const Int4& o) const noexcept { return {data[0]-o.data[0], data[1]-o.data[1], data[2]-o.data[2], data[3]-o.data[3]}; }
    Int4 operator*(const Int4& o) const noexcept { return {data[0]*o.data[0], data[1]*o.data[1], data[2]*o.data[2], data[3]*o.data[3]}; }

    Int4 operator&(const Int4& o) const noexcept { return {data[0]&o.data[0], data[1]&o.data[1], data[2]&o.data[2], data[3]&o.data[3]}; }
    Int4 operator|(const Int4& o) const noexcept { return {data[0]|o.data[0], data[1]|o.data[1], data[2]|o.data[2], data[3]|o.data[3]}; }
    Int4 operator^(const Int4& o) const noexcept { return {data[0]^o.data[0], data[1]^o.data[1], data[2]^o.data[2], data[3]^o.data[3]}; }

    Int4 operator<<(int c) const noexcept { return {data[0]<<c, data[1]<<c, data[2]<<c, data[3]<<c}; }
    Int4 operator>>(int c) const noexcept { return {static_cast<unsigned int>(data[0])>>c, static_cast<unsigned int>(data[1])>>c, static_cast<unsigned int>(data[2])>>c, static_cast<unsigned int>(data[3])>>c}; }
};

#endif // target blocks

// ================================================
// Matrix4x4 — implemented using Float4 ops
// ================================================
class alignas(64) Matrix4x4 {
public:
    Float4 rows[4];

    Matrix4x4() noexcept {
        rows[0] = Float4(1,0,0,0);
        rows[1] = Float4(0,1,0,0);
        rows[2] = Float4(0,0,1,0);
        rows[3] = Float4(0,0,0,1);
    }

    explicit Matrix4x4(const Mat4& m) noexcept {
        rows[0] = Float4(m[0][0], m[1][0], m[2][0], m[3][0]);
        rows[1] = Float4(m[0][1], m[1][1], m[2][1], m[3][1]);
        rows[2] = Float4(m[0][2], m[1][2], m[2][2], m[3][2]);
        rows[3] = Float4(m[0][3], m[1][3], m[2][3], m[3][3]);
    }

    Matrix4x4 operator*(const Matrix4x4& o) const noexcept {
        Matrix4x4 r;
        for (int i = 0; i < 4; ++i) {
            const Float4 row = rows[i];
        #if defined(SIMD_NEON)
            Float4 x(vdupq_laneq_f32(row.v, 0));
            Float4 y(vdupq_laneq_f32(row.v, 1));
            Float4 z(vdupq_laneq_f32(row.v, 2));
            Float4 w(vdupq_laneq_f32(row.v, 3));
        #elif defined(SIMD_SSE2) || defined(SIMD_AVX) || defined(SIMD_AVX2) || defined(SIMD_AVX512)
            Float4 x(_mm_shuffle_ps(row.v, row.v, _MM_SHUFFLE(0,0,0,0)));
            Float4 y(_mm_shuffle_ps(row.v, row.v, _MM_SHUFFLE(1,1,1,1)));
            Float4 z(_mm_shuffle_ps(row.v, row.v, _MM_SHUFFLE(2,2,2,2)));
            Float4 w(_mm_shuffle_ps(row.v, row.v, _MM_SHUFFLE(3,3,3,3)));
        #else
            Float4 x(row[0], row[0], row[0], row[0]);
            Float4 y(row[1], row[1], row[1], row[1]);
            Float4 z(row[2], row[2], row[2], row[2]);
            Float4 w(row[3], row[3], row[3], row[3]);
        #endif
            r.rows[i] = x * o.rows[0] + y * o.rows[1] + z * o.rows[2] + w * o.rows[3];
        }
        return r;
    }

    [[nodiscard]] Float4 transform(const Float4& v) const noexcept {
    #if defined(SIMD_NEON)
        Float4 x(vdupq_laneq_f32(v.v, 0));
        Float4 y(vdupq_laneq_f32(v.v, 1));
        Float4 z(vdupq_laneq_f32(v.v, 2));
        Float4 w(vdupq_laneq_f32(v.v, 3));
    #elif defined(SIMD_SSE2) || defined(SIMD_AVX) || defined(SIMD_AVX2) || defined(SIMD_AVX512)
        Float4 x(_mm_shuffle_ps(v.v, v.v, _MM_SHUFFLE(0,0,0,0)));
        Float4 y(_mm_shuffle_ps(v.v, v.v, _MM_SHUFFLE(1,1,1,1)));
        Float4 z(_mm_shuffle_ps(v.v, v.v, _MM_SHUFFLE(2,2,2,2)));
        Float4 w(_mm_shuffle_ps(v.v, v.v, _MM_SHUFFLE(3,3,3,3)));
    #else
        Float4 x(v[0], v[0], v[0], v[0]);
        Float4 y(v[1], v[1], v[1], v[1]);
        Float4 z(v[2], v[2], v[2], v[2]);
        Float4 w(v[3], v[3], v[3], v[3]);
    #endif
        return x * rows[0] + y * rows[1] + z * rows[2] + w * rows[3];
    }

    [[nodiscard]] Mat4 toMat4() const noexcept {
        Mat4 out{};
        for (int i = 0; i < 4; ++i) {
            alignas(16) Float t[4];
            rows[i].storeAligned(t);
            for (int j = 0; j < 4; ++j) out[j][i] = t[j];
        }
        return out;
    }
};

} // namespace engine::math::simd



// /**
//  * @file SimdTypes.h
//  * @brief SIMD vector wrapper types for optimized math operations
//  *
//  * Provides portable SIMD types that automatically use the best
//  * available instruction set (SSE, AVX, AVX2, AVX-512).
//  */
//
// #pragma once
//
// #include "../core/MathTypes.h"
//
// #include <immintrin.h>
// // #ifdef __SSE__
// // #include <xmmintrin.h>  // SSE
// // #endif
// // #ifdef __SSE2__
// // #include <emmintrin.h>  // SSE2
// // #endif
// // #ifdef __SSE3__
// // #include <pmmintrin.h>  // SSE3
// // #endif
// // #ifdef __SSE4_1__
// // #include <smmintrin.h>  // SSE4.1
// // #endif
// // #ifdef __AVX__
// // #include <immintrin.h>  // AVX y superiores
// // #endif
//
// namespace engine::math::simd {
//     // ============================================================================
//     // SIMD Detection and Configuration
//     // ============================================================================
//
// #ifdef __AVX512F__
// #define SIMD_AVX512 1
// #define SIMD_WIDTH 16
// #elif defined(__AVX2__)
// #define SIMD_AVX2 1
// #define SIMD_WIDTH 8
// #elif defined(__AVX__)
// #define SIMD_AVX 1
// #define SIMD_WIDTH 8
// #elif defined(__SSE4_1__)
// #define SIMD_SSE4 1
// #define SIMD_WIDTH 4
// #else
// #define SIMD_SCALAR 1
// #define SIMD_WIDTH 1
// #endif
//
//     // ============================================================================
//     // Float4 - 4-wide float vector
//     // ============================================================================
//
//     class alignas(16) Float4 {
//     public:
//         union {
//             __m128 v;
//             Float data[4];
//
//             // TODO: Ver si esta bien la inicializacion de este struct
//             struct {
//                 Float x{}, y{}, z{}, w{};
//             };
//         };
//
//         // Constructors
//         Float4() noexcept : v(_mm_setzero_ps()) {
//         }
//
//         explicit Float4(const Float scalar) noexcept : v(_mm_set1_ps(scalar)) {
//         }
//
//         Float4(const Float x, const Float y, const Float z, const Float w) noexcept
//             : v(_mm_set_ps(w, z, y, x)) {
//         }
//
//         explicit Float4(__m128 vec) noexcept : v(vec) {
//         }
//
//         explicit Float4(const Vec4& vec) noexcept
//             : v(_mm_set_ps(vec.w, vec.z, vec.y, vec.x)) {
//         }
//
//         // Load/Store
//         static Float4 load(const Float* ptr) noexcept {
//             return Float4(_mm_loadu_ps(ptr));
//         }
//
//         static Float4 loadAligned(const Float* ptr) noexcept {
//             return Float4(_mm_load_ps(ptr));
//         }
//
//         void store(Float* ptr) const noexcept {
//             _mm_storeu_ps(ptr, v);
//         }
//
//         void storeAligned(Float* ptr) const noexcept {
//             _mm_store_ps(ptr, v);
//         }
//
//         // Arithmetic operators
//         Float4 operator+(const Float4& other) const noexcept {
//             return Float4(_mm_add_ps(v, other.v));
//         }
//
//         Float4 operator-(const Float4& other) const noexcept {
//             return Float4(_mm_sub_ps(v, other.v));
//         }
//
//         Float4 operator*(const Float4& other) const noexcept {
//             return Float4(_mm_mul_ps(v, other.v));
//         }
//
//         Float4 operator/(const Float4& other) const noexcept {
//             return Float4(_mm_div_ps(v, other.v));
//         }
//
//         Float4& operator+=(const Float4& other) noexcept {
//             v = _mm_add_ps(v, other.v);
//             return *this;
//         }
//
//         Float4& operator-=(const Float4& other) noexcept {
//             v = _mm_sub_ps(v, other.v);
//             return *this;
//         }
//
//         Float4& operator*=(const Float4& other) noexcept {
//             v = _mm_mul_ps(v, other.v);
//             return *this;
//         }
//
//         Float4& operator/=(const Float4& other) noexcept {
//             v = _mm_div_ps(v, other.v);
//             return *this;
//         }
//
//         // Comparison
//         Float4 operator<(const Float4& other) const noexcept {
//             return Float4(_mm_cmplt_ps(v, other.v));
//         }
//
//         Float4 operator<=(const Float4& other) const noexcept {
//             return Float4(_mm_cmple_ps(v, other.v));
//         }
//
//         Float4 operator>(const Float4& other) const noexcept {
//             return Float4(_mm_cmpgt_ps(v, other.v));
//         }
//
//         Float4 operator>=(const Float4& other) const noexcept {
//             return Float4(_mm_cmpge_ps(v, other.v));
//         }
//
//         Float4 operator==(const Float4& other) const noexcept {
//             return Float4(_mm_cmpeq_ps(v, other.v));
//         }
//
//         // Conversion
//         [[nodiscard]] Vec4 toVec4() const noexcept {
//             alignas(16) Float temp[4];
//             storeAligned(temp);
//             return {temp[0], temp[1], temp[2], temp[3]};
//         }
//
//         // Element access
//         Float operator[](const int index) const noexcept {
//             return data[index];
//         }
//
//         Float& operator[](const int index) noexcept {
//             return data[index];
//         }
//     };
//
//     // ============================================================================
//     // Float8 - 8-wide float vector (AVX)
//     // ============================================================================
//
// #ifdef SIMD_AVX
//     class alignas(32) Float8 {
//     public:
//         union {
//             __m256 v;
//             Float data[8];
//         };
//
//         // Constructors
//         Float8() noexcept : v(_mm256_setzero_ps()) {
//         }
//
//         explicit Float8(Float scalar) noexcept : v(_mm256_set1_ps(scalar)) {
//         }
//
//         explicit Float8(__m256 vec) noexcept : v(vec) {
//         }
//
//         Float8(Float a, Float b, Float c, Float d, Float e, Float f, Float g, Float h) noexcept
//             : v(_mm256_set_ps(h, g, f, e, d, c, b, a)) {
//         }
//
//         // Load/Store
//         static Float8 load(const Float* ptr) noexcept {
//             return Float8(_mm256_loadu_ps(ptr));
//         }
//
//         static Float8 loadAligned(const Float* ptr) noexcept {
//             return Float8(_mm256_load_ps(ptr));
//         }
//
//         void store(Float* ptr) const noexcept {
//             _mm256_storeu_ps(ptr, v);
//         }
//
//         void storeAligned(Float* ptr) const noexcept {
//             _mm256_store_ps(ptr, v);
//         }
//
//         // Arithmetic
//         Float8 operator+(const Float8& other) const noexcept {
//             return Float8(_mm256_add_ps(v, other.v));
//         }
//
//         Float8 operator-(const Float8& other) const noexcept {
//             return Float8(_mm256_sub_ps(v, other.v));
//         }
//
//         Float8 operator*(const Float8& other) const noexcept {
//             return Float8(_mm256_mul_ps(v, other.v));
//         }
//
//         Float8 operator/(const Float8& other) const noexcept {
//             return Float8(_mm256_div_ps(v, other.v));
//         }
//
//         // FMA operations
//         Float8 fma(const Float8& b, const Float8& c) const noexcept {
//
//
// #ifdef __FMA__
//     return Float8 (_mm256_fmadd_ps(v, b.v, c.v));
// #else
//     return (*this) * b+ c;
// #endif
//     }
//
//     // Horizontal operations
//     Float sum() const noexcept {
//         __m128 low = _mm256_castps256_ps128(v);
//         __m128 high = _mm256_extractf128_ps(v, 1);
//         __m128 sum128 = _mm_add_ps(low, high);
//         sum128 = _mm_hadd_ps(sum128, sum128);
//         sum128 = _mm_hadd_ps(sum128, sum128);
//         return _mm_cvtss_f32(sum128);
//     }
//     };
// #endif
//
//     // ============================================================================
//     // Int4 - 4-wide integer vector
//     // ============================================================================
//
//     class alignas(16) Int4 {
//     public:
//         union {
//             __m128i v;
//             Int data[4];
//
//             // TODO: Revisar la inicializacion de esto
//             struct {
//                 Int x{}, y{}, z{}, w{};
//             };
//         };
//
//         Int4() noexcept : v(_mm_setzero_si128()) {
//         }
//
//         explicit Int4(const Int scalar) noexcept : v(_mm_set1_epi32(scalar)) {
//         }
//
//         Int4(const Int x, const Int y, const Int z, const Int w) noexcept
//             : v(_mm_set_epi32(w, z, y, x)) {
//         }
//
//         explicit Int4(__m128i vec) noexcept : v(vec) {
//         }
//
//         // Arithmetic
//         Int4 operator+(const Int4& other) const noexcept {
//             return Int4(_mm_add_epi32(v, other.v));
//         }
//
//         Int4 operator-(const Int4& other) const noexcept {
//             return Int4(_mm_sub_epi32(v, other.v));
//         }
//
//         Int4 operator*(const Int4& other) const noexcept {
//             return Int4(_mm_mullo_epi32(v, other.v));
//         }
//
//         // Bitwise operations
//         Int4 operator&(const Int4& other) const noexcept {
//             return Int4(_mm_and_si128(v, other.v));
//         }
//
//         Int4 operator|(const Int4& other) const noexcept {
//             return Int4(_mm_or_si128(v, other.v));
//         }
//
//         Int4 operator^(const Int4& other) const noexcept {
//             return Int4(_mm_xor_si128(v, other.v));
//         }
//
//         Int4 operator<<(const int count) const noexcept {
//             return Int4(_mm_slli_epi32(v, count));
//         }
//
//         Int4 operator>>(const int count) const noexcept {
//             return Int4(_mm_srli_epi32(v, count));
//         }
//     };
//
//     // ============================================================================
//     // Matrix4x4 - SIMD optimized 4x4 matrix
//     // ============================================================================
//
//     class alignas(64) Matrix4x4 {
//     public:
//         Float4 rows[4];
//
//         Matrix4x4() noexcept {
//             rows[0] = Float4(1, 0, 0, 0);
//             rows[1] = Float4(0, 1, 0, 0);
//             rows[2] = Float4(0, 0, 1, 0);
//             rows[3] = Float4(0, 0, 0, 1);
//         }
//
//         explicit Matrix4x4(const Mat4& mat) noexcept {
//             rows[0] = Float4(mat[0][0], mat[1][0], mat[2][0], mat[3][0]);
//             rows[1] = Float4(mat[0][1], mat[1][1], mat[2][1], mat[3][1]);
//             rows[2] = Float4(mat[0][2], mat[1][2], mat[2][2], mat[3][2]);
//             rows[3] = Float4(mat[0][3], mat[1][3], mat[2][3], mat[3][3]);
//         }
//
//         // Matrix multiplication using SIMD
//         Matrix4x4 operator*(const Matrix4x4& other) const noexcept {
//             Matrix4x4 result;
//
//             for (int i = 0; i < 4; ++i) {
//                 Float4 row = rows[i];
//
//                 const auto x = Float4(_mm_shuffle_ps(row.v, row.v, _MM_SHUFFLE(0,0,0,0)));
//                 const auto y = Float4(_mm_shuffle_ps(row.v, row.v, _MM_SHUFFLE(1,1,1,1)));
//                 const auto z = Float4(_mm_shuffle_ps(row.v, row.v, _MM_SHUFFLE(2,2,2,2)));
//                 const auto w = Float4(_mm_shuffle_ps(row.v, row.v, _MM_SHUFFLE(3,3,3,3)));
//
//                 result.rows[i] = x * other.rows[0] + y * other.rows[1] +
//                     z * other.rows[2] + w * other.rows[3];
//             }
//
//             return result;
//         }
//
//         // Transform vector
//         [[nodiscard]] Float4 transform(const Float4& vec) const noexcept {
//             const auto x = Float4(_mm_shuffle_ps(vec.v, vec.v, _MM_SHUFFLE(0,0,0,0)));
//             const auto y = Float4(_mm_shuffle_ps(vec.v, vec.v, _MM_SHUFFLE(1,1,1,1)));
//             const auto z = Float4(_mm_shuffle_ps(vec.v, vec.v, _MM_SHUFFLE(2,2,2,2)));
//             const auto w = Float4(_mm_shuffle_ps(vec.v, vec.v, _MM_SHUFFLE(3,3,3,3)));
//
//             return x * rows[0] + y * rows[1] + z * rows[2] + w * rows[3];
//         }
//
//         [[nodiscard]] Mat4 toMat4() const noexcept {
//             Mat4 result;
//
//             for (int i = 0; i < 4; ++i) {
//                 alignas(16) Float temp[4];
//                 rows[i].storeAligned(temp);
//
//                 for (int j = 0; j < 4; ++j) {
//                     result[j][i] = temp[j];
//                 }
//             }
//
//             return result;
//         }
//     };
// } // namespace engine::math::simd
