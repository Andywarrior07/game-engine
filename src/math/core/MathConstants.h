/**
 * @file MathConstants.h
 * @brief Mathematical constants and thresholds for the game engine
 * @author Andrés Guerrero
 * @date 29-08-2025
 *
 * Provides compile-time constants for common mathematical values,
 * epsilon values for floating-point comparisons, and physics constants.
 */

#pragma once

#include "MathTypes.h"
#include <limits>

namespace engine::math {

    // ============================================================================
    // Mathematical Constants
    // ============================================================================

    // Basic mathematical constants
    template<typename T = Float>
    inline constexpr T PI = T(3.14159265358979323846);

    template<typename T = Float>
    inline constexpr T TWO_PI = T(6.28318530717958647692);

    template<typename T = Float>
    inline constexpr T HALF_PI = T(1.57079632679489661923);

    template<typename T = Float>
    inline constexpr T QUARTER_PI = T(0.78539816339744830961);

    template<typename T = Float>
    inline constexpr T INV_PI = T(0.31830988618379067153);

    template<typename T = Float>
    inline constexpr T INV_TWO_PI = T(0.15915494309189533576);

    // Euler's number and related
    template<typename T = Float>
    inline constexpr T E = T(2.71828182845904523536);

    template<typename T = Float>
    inline constexpr T LOG2E = T(1.44269504088896340735);

    template<typename T = Float>
    inline constexpr T LOG10E = T(0.43429448190325182765);

    template<typename T = Float>
    inline constexpr T LN2 = T(0.69314718055994530941);

    template<typename T = Float>
    inline constexpr T LN10 = T(2.30258509299404568401);

    // Square roots of common values
    template<typename T = Float>
    inline constexpr T SQRT2 = T(1.41421356237309504880);

    template<typename T = Float>
    inline constexpr T SQRT3 = T(1.73205080756887729352);

    template<typename T = Float>
    inline constexpr T INV_SQRT2 = T(0.70710678118654752440);

    template<typename T = Float>
    inline constexpr T INV_SQRT3 = T(0.57735026918962576450);

    // Golden ratio
    template<typename T = Float>
    inline constexpr T GOLDEN_RATIO = T(1.61803398874989484820);

    // ============================================================================
    // Angle Conversion Constants
    // ============================================================================

    template<typename T = Float>
    inline constexpr T DEG_TO_RAD = PI<T> / T(180);

    template<typename T = Float>
    inline constexpr T RAD_TO_DEG = T(180) / PI<T>;

    // Conversion functions using the constants
    template<typename T>
    [[nodiscard]] constexpr T toRadians(T degrees) noexcept {
        return degrees * DEG_TO_RAD<T>;
    }

    template<typename T>
    [[nodiscard]] constexpr T toDegrees(T radians) noexcept {
        return radians * RAD_TO_DEG<T>;
    }

    // ============================================================================
    // Epsilon Values for Comparisons
    // ============================================================================

    // Standard epsilon for float comparisons
    inline constexpr Float EPSILON = 1e-6f;
    inline constexpr Float EPSILON_SQUARED = EPSILON * EPSILON;

    // Tighter epsilon for high-precision comparisons
    inline constexpr Float EPSILON_TIGHT = 1e-9f;

    // Looser epsilon for less critical comparisons
    inline constexpr Float EPSILON_LOOSE = 1e-4f;

    // Double precision epsilons
    inline constexpr Double EPSILON_DOUBLE = 1e-12;
    inline constexpr Double EPSILON_DOUBLE_TIGHT = 1e-15;
    inline constexpr Double EPSILON_DOUBLE_LOOSE = 1e-9;

    // Machine epsilon (smallest representable difference)
    template<typename T = Float>
    inline constexpr T MACHINE_EPSILON = std::numeric_limits<T>::epsilon();

    // ============================================================================
    // Physics Constants
    // ============================================================================

    namespace physics {
        // Gravity constant (m/s²)
        inline constexpr Float GRAVITY_EARTH = 9.80665f;
        inline constexpr Float GRAVITY_MOON = 1.62f;
        inline constexpr Float GRAVITY_MARS = 3.71f;

        // Universal gravitational constant (m³/kg·s²)
        inline constexpr Double G = 6.67430e-11;

        // Speed of light in vacuum (m/s)
        inline constexpr Double C = 299792458.0;

        // Common physics timesteps
        inline constexpr Float FIXED_TIMESTEP_60HZ = 1.0f / 60.0f;
        inline constexpr Float FIXED_TIMESTEP_120HZ = 1.0f / 120.0f;
        inline constexpr Float FIXED_TIMESTEP_30HZ = 1.0f / 30.0f;
    }

    // ============================================================================
    // Limits and Thresholds
    // ============================================================================

    // Infinity representations
    template<typename T = Float>
    inline constexpr T INFINITY_VALUE = std::numeric_limits<T>::infinity();

    template<typename T = Float>
    inline constexpr T NEG_INFINITY = -std::numeric_limits<T>::infinity();

    // Maximum and minimum values
    template<typename T = Float>
    inline constexpr T MAX_VALUE = std::numeric_limits<T>::max();

    template<typename T = Float>
    inline constexpr T MIN_VALUE = std::numeric_limits<T>::lowest();

    template<typename T = Float>
    inline constexpr T MIN_POSITIVE = std::numeric_limits<T>::min();

    // Common thresholds
    inline constexpr Float SMALL_NUMBER = 1e-8f;
    inline constexpr Float BIG_NUMBER = 3.4e+38f;

    // Normalization thresholds
    inline constexpr Float NORMALIZE_TOLERANCE = 0.00001f;
    inline constexpr Float THRESH_VECTOR_NORMALIZED = 0.01f;

    // Quaternion thresholds
    inline constexpr Float THRESH_QUAT_NORMALIZED = 0.01f;
    inline constexpr Float SINGULARITY_THRESHOLD = 0.499f; // For gimbal lock detection

    // ============================================================================
    // Common Fractions and Ratios
    // ============================================================================

    template<typename T = Float>
    inline constexpr T ONE_THIRD = T(1) / T(3);

    template<typename T = Float>
    inline constexpr T TWO_THIRDS = T(2) / T(3);

    template<typename T = Float>
    inline constexpr T ONE_SIXTH = T(1) / T(6);

    template<typename T = Float>
    inline constexpr T ONE_255TH = T(1) / T(255);  // For color conversions

    // ============================================================================
    // Matrix and Transform Constants
    // ============================================================================

    // Identity matrices (using GLM's built-in identity)
    inline const auto IDENTITY_2X2 = Mat2(1.0f);
    inline const auto IDENTITY_3X3 = Mat3(1.0f);
    inline const auto IDENTITY_4X4 = Mat4(1.0f);

    // Zero matrices
    inline const auto ZERO_2X2 = Mat2(0.0f);
    inline const auto ZERO_3X3 = Mat3(0.0f);
    inline const auto ZERO_4X4 = Mat4(0.0f);

    // Common vectors
    inline const auto VEC2_ZERO = Vec2(0.0f, 0.0f);
    inline const auto VEC2_ONE = Vec2(1.0f, 1.0f);
    inline const auto VEC2_HALF = Vec2(0.5f, 0.5f);
    inline const auto VEC2_UP = Vec2(0.0f, 1.0f);
    inline const auto VEC2_DOWN = Vec2(0.0f, -1.0f);
    inline const auto VEC2_LEFT = Vec2(-1.0f, 0.0f);
    inline const auto VEC2_RIGHT = Vec2(1.0f, 0.0f);

    inline const auto VEC3_ZERO = Vec3(0.0f, 0.0f, 0.0f);
    inline const auto VEC3_ONE = Vec3(1.0f, 1.0f, 1.0f);
    inline const auto VEC3_HALF = Vec3(0.5f, 0.5f, 0.5f);
    inline const auto VEC3_FORWARD = Vec3(0.0f, 0.0f, -1.0f);  // -Z in OpenGL
    inline const auto VEC3_BACK = Vec3(0.0f, 0.0f, 1.0f);
    inline const auto VEC3_UP = Vec3(0.0f, 1.0f, 0.0f);
    inline const auto VEC3_DOWN = Vec3(0.0f, -1.0f, 0.0f);
    inline const auto VEC3_LEFT = Vec3(-1.0f, 0.0f, 0.0f);
    inline const auto VEC3_RIGHT = Vec3(1.0f, 0.0f, 0.0f);

    inline const auto VEC4_ZERO = Vec4(0.0f, 0.0f, 0.0f, 0.0f);
    inline const auto VEC4_ONE = Vec4(1.0f, 1.0f, 1.0f, 1.0f);

    // Identity quaternion (no rotation)
    inline const auto QUAT_IDENTITY = Quat(1.0f, 0.0f, 0.0f, 0.0f);

    // ============================================================================
    // Array Sizes and Limits
    // ============================================================================

    // Maximum supported dimensions for various systems
    inline constexpr std::size_t MAX_BONES_PER_VERTEX = 4;
    inline constexpr std::size_t MAX_BONES_PER_SKELETON = 256;
    inline constexpr std::size_t MAX_TEXTURE_UNITS = 32;
    inline constexpr std::size_t MAX_VERTEX_ATTRIBUTES = 16;

    // Octree/spatial partitioning limits
    inline constexpr std::size_t MAX_OCTREE_DEPTH = 8;
    inline constexpr std::size_t MIN_OCTREE_NODE_SIZE = 1;
    inline constexpr std::size_t MAX_OBJECTS_PER_NODE = 8;

} // namespace engine::math