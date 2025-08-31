/**
 * @file MathFunctions.h
 * @brief Core mathematical functions and utilities
 * @author Andrés Guerrero
 * @date 29-08-2025
 *
 * Provides common mathematical operations, comparisons, interpolation,
 * and utility functions optimized for game development.
 */

#pragma once

#include "MathTypes.h"
#include "MathConstants.h"

#include <glm/gtc/epsilon.hpp>
#include <glm/gtx/norm.hpp>

namespace engine::math {
    // ============================================================================
    // Comparison Functions with Epsilon
    // ============================================================================

    /**
     * @brief Check if two floating-point values are approximately equal
     * @param a First value
     * @param b Second value
     * @param epsilon Tolerance for comparison
     */
    template<typename T>
    [[nodiscard]] inline constexpr bool isNearlyEqual(T a, T b,
                                                      T epsilon = EPSILON) noexcept {
        static_assert(std::is_arithmetic_v<T>, "T must be an arithmetic type");
        return std::abs(a - b) <= epsilon;
    }


    /**
     * @brief Check if a floating-point value is nearly zero
     */
    template <typename T>
    [[nodiscard]] inline constexpr bool isNearlyZero(T value,
                                                     T epsilon = EPSILON) noexcept {
        return std::abs(value) <= epsilon;
    }

    /**
     * @brief Check if two vectors are approximately equal
     */
    template<typename VecType>
    [[nodiscard]] inline auto isNearlyEqual(const VecType& a, const VecType& b,
                                           Float epsilon = EPSILON) noexcept
        -> decltype(glm::all(glm::epsilonEqual(a, b, epsilon))) {
        return glm::all(glm::epsilonEqual(a, b, epsilon));
    }


    /**
     * @brief Check if a vector is nearly zero
     */
    template <typename VecType>
    [[nodiscard]] inline bool isNearlyZero(const VecType& v,
                                           const Float epsilon = EPSILON) noexcept {
        return glm::length2(v) <= epsilon * epsilon;
    }

    /**
     * @brief Check if two quaternions are approximately equal
     */
    [[nodiscard]] inline bool isNearlyEqual(const Quat& a, const Quat& b,
                                            const Float epsilon = EPSILON) noexcept {
        return glm::all(glm::epsilonEqual(a, b, epsilon));
    }

    // ============================================================================
    // Clamping and Saturation
    // ============================================================================

    /**
     * @brief Clamp a value between min and max
     */
    template<typename T>
    [[nodiscard]] inline constexpr std::enable_if_t<std::is_arithmetic_v<T>, T>
    clamp(T value, T min, T max) noexcept {
        return value < min ? min : (value > max ? max : value);
    }


    /**
     * @brief Clamp a value to [0, 1] range
     */
    template<typename T>
    [[nodiscard]] inline constexpr std::enable_if_t<std::is_arithmetic_v<T>, T>
    saturate(T value) noexcept {
        return value < T(0) ? T(0) : (value > T(1) ? T(1) : value);
    }


    /**
     * @brief Clamp vector components between min and max
     */
    template<typename VecType>
    [[nodiscard]] inline auto clamp(const VecType& v, const VecType& min, const VecType& max) noexcept
        -> std::enable_if_t<!std::is_arithmetic_v<VecType>, decltype(glm::clamp(v, min, max))> {
        return glm::clamp(v, min, max);
    }

    /**
     * @brief Clamp vector components to [0, 1] range
     */
    template<typename VecType>
    [[nodiscard]] inline auto saturate(const VecType& v) noexcept
        -> std::enable_if_t<!std::is_arithmetic_v<VecType>, decltype(glm::clamp(v, VecType(0), VecType(1)))> {
        return glm::clamp(v, VecType(0), VecType(1));
    }


    // ============================================================================
    // Interpolation Functions
    // ============================================================================

    /**
     * @brief Linear interpolation between two values
     * @param a Start value (t=0)
     * @param b End value (t=1)
     * @param t Interpolation factor [0, 1]
     */
    template<typename T>
    [[nodiscard]] constexpr std::enable_if_t<std::is_arithmetic_v<T>, T>
    lerp(T a, T b, Float t) noexcept {
        return a + t * (b - a);
    }


    /**
     * @brief Linear interpolation for vectors
     */
    template<typename VecType>
    [[nodiscard]] auto lerp(const VecType& a, const VecType& b, Float t) noexcept
        -> std::enable_if_t<!std::is_arithmetic_v<VecType>, decltype(glm::mix(a, b, t))> {
        return glm::mix(a, b, t);
    }


    /**
     * @brief Inverse linear interpolation - get t value from lerp result
     * Returns the t value such that lerp(a, b, t) = value
     */
    template <typename T>
    [[nodiscard]] inline constexpr Float inverseLerp(T a, T b, T value) noexcept {
        if (isNearlyEqual(a, b)) return 0.0f;
        return static_cast<Float>((value - a) / (b - a));
    }

    /**
     * @brief Remap a value from one range to another
     */
    template <typename T>
    [[nodiscard]] inline constexpr T remap(T value, T inMin, T inMax,
                                           T outMin, T outMax) noexcept {
        Float t = inverseLerp(inMin, inMax, value);
        return lerp(outMin, outMax, t);
    }

    /**
     * @brief Smooth step interpolation (cubic smoothing)
     */
    template <typename T>
    [[nodiscard]] inline constexpr T smoothstep(T edge0, T edge1, T x) noexcept {
        T t = saturate((x - edge0) / (edge1 - edge0));
        return t * t * (T(3) - T(2) * t);
    }

    /**
     * @brief Smoother step interpolation (quintic smoothing)
     */
    template <typename T>
    [[nodiscard]] inline constexpr T smootherstep(T edge0, T edge1, T x) noexcept {
        T t = saturate((x - edge0) / (edge1 - edge0));
        return t * t * t * (t * (t * T(6) - T(15)) + T(10));
    }

    /**
     * @brief Cubic Hermite interpolation
     */
    template <typename T>
    [[nodiscard]] inline T hermite(T p0, T m0, T p1, T m1, const Float t) noexcept {
        const Float t2 = t * t;
        const Float t3 = t2 * t;
        Float h1 = 2 * t3 - 3 * t2 + 1;
        Float h2 = -2 * t3 + 3 * t2;
        Float h3 = t3 - 2 * t2 + t;
        Float h4 = t3 - t2;
        return h1 * p0 + h2 * p1 + h3 * m0 + h4 * m1;
    }

    /**
     * @brief Spherical linear interpolation for quaternions
     */
    [[nodiscard]] inline Quat slerp(const Quat& a, const Quat& b, const Float t) noexcept {
        return glm::slerp(a, b, t);
    }

    // ============================================================================
    // Min/Max Functions
    // ============================================================================

    template <typename T>
    [[nodiscard]] inline constexpr T min(T a, T b) noexcept {
        return (a < b) ? a : b;
    }

    template <typename T>
    [[nodiscard]] inline constexpr T max(T a, T b) noexcept {
        return (a > b) ? a : b;
    }

    template <typename T>
    [[nodiscard]] inline constexpr T min3(T a, T b, T c) noexcept {
        return min(min(a, b), c);
    }

    template <typename T>
    [[nodiscard]] inline constexpr T max3(T a, T b, T c) noexcept {
        return max(max(a, b), c);
    }

    // ============================================================================
    // Sign and Step Functions
    // ============================================================================

    /**
     * @brief Get the sign of a value (-1, 0, or 1)
     */
    template <typename T>
    [[nodiscard]] inline constexpr T sign(T value) noexcept {
        return (T(0) < value) - (value < T(0));
    }

    /**
     * @brief Step function: returns 0 if x < edge, else 1
     */
    template <typename T>
    [[nodiscard]] inline constexpr T step(T edge, T x) noexcept {
        return x < edge ? T(0) : T(1);
    }

    // ============================================================================
    // Power and Root Functions
    // ============================================================================

    /**
     * @brief Safe square root (returns 0 for negative values)
     */
    template <typename T>
    [[nodiscard]] inline T safeSqrt(T value) noexcept {
        return value <= T(0) ? T(0) : std::sqrt(value);
    }

    /**
     * @brief Square a value
     */
    template <typename T>
    [[nodiscard]] inline constexpr T square(T value) noexcept {
        return value * value;
    }

    /**
     * @brief Cube a value
     */
    template <typename T>
    [[nodiscard]] inline constexpr T cube(T value) noexcept {
        return value * value * value;
    }

    /**
     * @brief Check if a value is a power of two
     */
    [[nodiscard]] inline constexpr bool isPowerOfTwo(const Int value) noexcept {
        return value && !(value & (value - 1));
    }

    /**
     * @brief Round up to the next power of two
     */
    [[nodiscard]] inline constexpr Int nextPowerOfTwo(Int value) noexcept {
        value--;
        value |= value >> 1;
        value |= value >> 2;
        value |= value >> 4;
        value |= value >> 8;
        value |= value >> 16;
        value++;
        return value;
    }

    // ============================================================================
    // Rounding Functions
    // ============================================================================

    /**
     * @brief Round to nearest integer
     */
    template <typename T>
    [[nodiscard]] inline T round(T value) noexcept {
        return std::round(value);
    }

    /**
     * @brief Round down to integer
     */
    template <typename T>
    [[nodiscard]] inline T floor(T value) noexcept {
        return std::floor(value);
    }

    /**
     * @brief Round up to integer
     */
    template <typename T>
    [[nodiscard]] inline T ceil(T value) noexcept {
        return std::ceil(value);
    }

    /**
     * @brief Round to specified number of decimal places
     */
    template <typename T>
    [[nodiscard]] inline T roundToDecimal(T value, int decimalPlaces) noexcept {
        T multiplier = std::pow(T(10), decimalPlaces);
        return std::round(value * multiplier) / multiplier;
    }

    /**
     * @brief Snap value to grid
     */
    template <typename T>
    [[nodiscard]] inline T snapToGrid(T value, T gridSize) noexcept {
        return std::round(value / gridSize) * gridSize;
    }

    // ============================================================================
    // Modulo and Wrapping
    // ============================================================================

    /**
     * @brief Floating-point modulo
     */
    template <typename T>
    [[nodiscard]] inline T fmod(T x, T y) noexcept {
        return std::fmod(x, y);
    }

    /**
     * @brief Wrap value to range [0, max)
     */
    template <typename T>
    [[nodiscard]] inline T wrap(T value, T max) noexcept {
        T mod = fmod(value, max);
        return mod < 0 ? mod + max : mod;
    }

    /**
     * @brief Wrap value to range [min, max)
     */
    template <typename T>
    [[nodiscard]] inline T wrapRange(T value, T min, T max) noexcept {
        T range = max - min;
        return min + wrap(value - min, range);
    }

    /**
     * @brief Wrap angle to [-PI, PI]
     */
    [[nodiscard]] inline Float wrapAngle(Float angle) noexcept {
        angle = fmod(angle + PI<Float>, TWO_PI<Float>);
        if (angle < 0) angle += TWO_PI<Float>;
        return angle - PI<Float>;
    }

    /**
     * @brief Wrap angle to [0, 2*PI]
     */
    [[nodiscard]] inline Float wrapAngle360(Float angle) noexcept {
        angle = fmod(angle, TWO_PI<Float>);
        if (angle < 0) angle += TWO_PI<Float>;
        return angle;
    }

    // ============================================================================
    // Vector Operations
    // ============================================================================

    /**
     * @brief Safe normalize - returns zero vector if input is too small
     */
    template <typename VecType>
    [[nodiscard]] inline VecType safeNormalize(const VecType& v,
                                               const VecType& fallback = VecType(0)) noexcept {
        const Float lengthSq = glm::length2(v);
        if (lengthSq < EPSILON_SQUARED) {
            return fallback;
        }
        return v / std::sqrt(lengthSq);
    }

    /**
     * @brief Project vector a onto vector b
     */
    template <typename VecType>
    [[nodiscard]] inline VecType project(const VecType& a, const VecType& b) noexcept {
        const Float dot = glm::dot(a, b);
        const Float lengthSq = glm::length2(b);
        if (lengthSq < EPSILON_SQUARED) return VecType(0);
        return b * (dot / lengthSq);
    }

    /**
     * @brief Reflect vector around normal
     */
    template <typename VecType>
    [[nodiscard]] inline VecType reflect(const VecType& incident,
                                         const VecType& normal) noexcept {
        return glm::reflect(incident, normal);
    }

    /**
     * @brief Refract vector through surface
     */
    template <typename VecType>
    [[nodiscard]] inline VecType refract(const VecType& incident,
                                         const VecType& normal,
                                         Float ior) noexcept {
        return glm::refract(incident, normal, ior);
    }

    /**
     * @brief Get perpendicular vector in 2D (rotated 90 degrees counter-clockwise)
     */
    [[nodiscard]] inline Vec2 perpendicular(const Vec2& v) noexcept {
        return Vec2(-v.y, v.x);
    }

    /**
     * @brief Get any perpendicular vector in 3D
     */
    [[nodiscard]] inline Vec3 perpendicular(const Vec3& v) noexcept {
        // Use different axis based on smallest component to avoid near-parallel vectors
        const Vec3 absV = glm::abs(v);

        if (absV.x <= absV.y && absV.x <= absV.z) {
            return Vec3(0, -v.z, v.y);
        }

        if (absV.y <= absV.x && absV.y <= absV.z) {
            return Vec3(-v.z, 0, v.x);
        }

        return Vec3(-v.y, v.x, 0);
    }

    /**
     * @brief Build orthonormal basis from forward vector
     */
    inline void buildOrthonormalBasis(const Vec3& forward, Vec3& right, Vec3& up) noexcept {
        const Vec3 f = glm::normalize(forward);
        right = glm::normalize(perpendicular(f));
        up = glm::cross(f, right);
    }

    // ============================================================================
    // Angle Functions
    // ============================================================================

    /**
     * @brief Calculate angle between two vectors in radians
     */
    template <typename VecType>
    [[nodiscard]] inline Float angleBetween(const VecType& a, const VecType& b) noexcept {
        Float dot = glm::dot(glm::normalize(a), glm::normalize(b));
        dot = clamp(dot, -1.0f, 1.0f); // Prevent NaN from acos
        return std::acos(dot);
    }

    /**
     * @brief Calculate signed angle between two 2D vectors
     */
    [[nodiscard]] inline Float signedAngle(const Vec2& from, const Vec2& to) noexcept {
        const Float angle = angleBetween(from, to);
        const Float sign = (from.x * to.y - from.y * to.x) >= 0 ? 1.0f : -1.0f;
        return angle * sign;
    }

    /**
     * @brief Calculate shortest angular distance between two angles
     */
    [[nodiscard]] inline Float angularDistance(const Float from, const Float to) noexcept {
        const Float diff = wrapAngle(to - from);
        return std::abs(diff);
    }

    /**
     * @brief Lerp between angles (handles wrapping)
     */
    [[nodiscard]] inline Float lerpAngle(const Float from, const Float to, const Float t) noexcept {
        const Float diff = wrapAngle(to - from);
        return from + diff * t;
    }
} // namespace engine::math
