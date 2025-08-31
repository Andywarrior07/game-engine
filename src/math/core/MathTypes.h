/**
 * @file MathTypes.h
 * @brief Core mathematical type definitions and aliases for the game engine
 * @author Andrés Guerrero
 * @date 27-08-2025
 */

#pragma once

// GLM configuration - set before including GLM headers
#define GLM_FORCE_RADIANS // Use radians for all angle calculations
#define GLM_FORCE_DEPTH_ZERO_TO_ONE // Vulkan-compatible depth range
#define GLM_FORCE_LEFT_HANDED // Left-handed coordinate system (optional, change if needed)
#define GLM_FORCE_INTRINSICS // Enable SIMD intrinsics
#define GLM_ENABLE_EXPERIMENTAL // Enable experimental features

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/dual_quaternion.hpp>

#include <type_traits>

namespace engine::math {
    /**
     * This file provides the foundational mathematical types used throughout the engine.
     * All types are based on GLM for consistency and performance, with additional
     * custom types for engine-specific requirements.
     */
    // ============================================================================
    // Base Type Aliases from GLM
    // ============================================================================

    // Scalar types
    using Float = float;
    using Double = double;
    using Int = std::int32_t;
    using UInt = std::uint32_t;

    // 2D Vectors
    using Vec2 = glm::vec2; // float precision
    using Vec2f = glm::vec2; // explicit float
    using Vec2d = glm::dvec2; // double precision
    using Vec2i = glm::ivec2; // integer
    using Vec2u = glm::uvec2; // unsigned integer
    using Vec2b = glm::bvec2; // boolean

    // 3D Vectors
    using Vec3 = glm::vec3; // float precision (most common)
    using Vec3f = glm::vec3; // explicit float
    using Vec3d = glm::dvec3; // double precision
    using Vec3i = glm::ivec3; // integer
    using Vec3u = glm::uvec3; // unsigned integer
    using Vec3b = glm::bvec3; // boolean

    // 4D Vectors
    using Vec4 = glm::vec4; // float precision
    using Vec4f = glm::vec4; // explicit float
    using Vec4d = glm::dvec4; // double precision
    using Vec4i = glm::ivec4; // integer
    using Vec4u = glm::uvec4; // unsigned integer
    using Vec4b = glm::bvec4; // boolean

    // Matrices
    using Mat2 = glm::mat2; // 2x2 float matrix
    using Mat3 = glm::mat3; // 3x3 float matrix
    using Mat4 = glm::mat4; // 4x4 float matrix
    using Mat2x3 = glm::mat2x3; // 2x3 matrix
    using Mat2x4 = glm::mat2x4; // 2x4 matrix
    using Mat3x2 = glm::mat3x2; // 3x2 matrix
    using Mat3x4 = glm::mat3x4; // 3x4 matrix
    using Mat4x2 = glm::mat4x2; // 4x2 matrix
    using Mat4x3 = glm::mat4x3; // 4x3 matrix

    // Double precision matrices (for physics calculations)
    using Mat2d = glm::dmat2;
    using Mat3d = glm::dmat3;
    using Mat4d = glm::dmat4;

    // Quaternions
    using Quat = glm::quat; // float quaternion
    using Quatf = glm::quat; // explicit float
    using Quatd = glm::dquat; // double precision quaternion

    // Dual quaternions for skeletal animation
    using DualQuat = glm::dualquat;
    using DualQuatf = glm::fdualquat;
    using DualQuatd = glm::ddualquat;

    // ============================================================================
    // Engine-Specific Types
    // ============================================================================

    /**
     * @brief Color representation in RGBA format
     * Values are normalized [0,1] for each channel
     */
    struct Color {
        Float r, g, b, a;

        constexpr Color() noexcept : r(0), g(0), b(0), a(1) {
        }

        constexpr Color(const Float r, const Float g, const Float b, const Float a = 1.0f) noexcept
            : r(r), g(g), b(b), a(a) {
        }

        explicit constexpr Color(const Vec4& v) noexcept
            : r(v.x), g(v.y), b(v.z), a(v.w) {
        }

        explicit constexpr Color(const Vec3& v, const Float a = 1.0f) noexcept
            : r(v.x), g(v.y), b(v.z), a(a) {
        }

        // Conversion operators
        explicit operator Vec4() const noexcept { return Vec4(r, g, b, a); }
        explicit operator Vec3() const noexcept { return Vec3(r, g, b); }

        // Common colors as static members
        static const Color White;
        static const Color Black;
        static const Color Red;
        static const Color Green;
        static const Color Blue;
        static const Color Yellow;
        static const Color Magenta;
        static const Color Cyan;
        static const Color Transparent;
    };

    // Static color definitions
    constexpr Color Color::White{1.0f, 1.0f, 1.0f, 1.0f};
    constexpr Color Color::Black{0.0f, 0.0f, 0.0f, 1.0f};
    constexpr Color Color::Red{1.0f, 0.0f, 0.0f, 1.0f};
    constexpr Color Color::Green{0.0f, 1.0f, 0.0f, 1.0f};
    constexpr Color Color::Blue{0.0f, 0.0f, 1.0f, 1.0f};
    constexpr Color Color::Yellow{1.0f, 1.0f, 0.0f, 1.0f};
    constexpr Color Color::Magenta{1.0f, 0.0f, 1.0f, 1.0f};
    constexpr Color Color::Cyan{0.0f, 1.0f, 1.0f, 1.0f};
    constexpr Color Color::Transparent{0.0f, 0.0f, 0.0f, 0.0f};

    /**
     * @brief Integer color representation (0-255 per channel)
     * Used for texture data and UI colors
     */
    struct Color32 {
        std::uint8_t r, g, b, a;

        constexpr Color32() noexcept : r(0), g(0), b(0), a(255) {
        }

        constexpr Color32(const std::uint8_t r, const std::uint8_t g, const std::uint8_t b,
                          const std::uint8_t a = 255) noexcept
            : r(r), g(g), b(b), a(a) {
        }

        // Conversion from normalized color
        explicit Color32(const Color& c) noexcept
            : r(static_cast<std::uint8_t>(c.r * 255.0f))
              , g(static_cast<std::uint8_t>(c.g * 255.0f))
              , b(static_cast<std::uint8_t>(c.b * 255.0f))
              , a(static_cast<std::uint8_t>(c.a * 255.0f)) {
        }

        // Convert to normalized color
        [[nodiscard]] Color toColor() const noexcept {
            return Color(
                static_cast<Float>(r) / 255.0f,
                static_cast<Float>(g) / 255.0f,
                static_cast<Float>(b) / 255.0f,
                static_cast<Float>(a) / 255.0f
            );
        }

        // Pack into single 32-bit integer (RGBA)
        [[nodiscard]] UInt pack() const noexcept {
            return (static_cast<UInt>(r) << 24) |
                (static_cast<UInt>(g) << 16) |
                (static_cast<UInt>(b) << 8) |
                static_cast<UInt>(a);
        }
    };

    /**
     * @brief Euler angles representation
     * All angles are in radians
     */
    struct EulerAngles {
        Float pitch; // Rotation around X axis
        Float yaw; // Rotation around Y axis
        Float roll; // Rotation around Z axis

        constexpr EulerAngles() noexcept : pitch(0), yaw(0), roll(0) {
        }

        constexpr EulerAngles(const Float pitch, const Float yaw, const Float roll) noexcept
            : pitch(pitch), yaw(yaw), roll(roll) {
        }

        explicit EulerAngles(const Vec3& v) noexcept
            : pitch(v.x), yaw(v.y), roll(v.z) {
        }

        explicit operator Vec3() const noexcept { return Vec3(pitch, yaw, roll); }

        // Convert to quaternion (assuming ZYX rotation order)
        [[nodiscard]] Quat toQuaternion() const noexcept {
            return glm::quat(Vec3(pitch, yaw, roll));
        }

        // Create from quaternion
        static EulerAngles fromQuaternion(const Quat& q) noexcept {
            const Vec3 euler = glm::eulerAngles(q);
            return EulerAngles(euler.x, euler.y, euler.z);
        }
    };

    /**
     * @brief Axis-angle representation
     */
    struct AxisAngle {
        Vec3 axis;
        Float angle; // In radians

        AxisAngle() noexcept : axis(0, 1, 0), angle(0) {
        }

        AxisAngle(const Vec3& axis, Float angle) noexcept
            : axis(glm::normalize(axis)), angle(angle) {
        }

        // Convert to quaternion
        [[nodiscard]] Quat toQuaternion() const noexcept {
            return glm::angleAxis(angle, axis);
        }

        // Create from quaternion
        static AxisAngle fromQuaternion(const Quat& q) noexcept {
            const Float angle = glm::angle(q);
            const Vec3 axis = glm::axis(q);

            return AxisAngle(axis, angle);
        }
    };

    // ============================================================================
    // Type Traits and Compile-Time Checks
    // ============================================================================

    // Check if type is a vector type
    template <typename T>
    struct is_vector : std::false_type {
    };

    template <>
    struct is_vector<Vec2> : std::true_type {
    };

    template <>
    struct is_vector<Vec3> : std::true_type {
    };

    template <>
    struct is_vector<Vec4> : std::true_type {
    };

    template <>
    struct is_vector<Vec2d> : std::true_type {
    };

    template <>
    struct is_vector<Vec3d> : std::true_type {
    };

    template <>
    struct is_vector<Vec4d> : std::true_type {
    };

    template <typename T>
    inline constexpr bool is_vector_v = is_vector<T>::value;

    // Check if type is a matrix type
    template <typename T>
    struct is_matrix : std::false_type {
    };

    template <>
    struct is_matrix<Mat2> : std::true_type {
    };

    template <>
    struct is_matrix<Mat3> : std::true_type {
    };

    template <>
    struct is_matrix<Mat4> : std::true_type {
    };

    template <>
    struct is_matrix<Mat2d> : std::true_type {
    };

    template <>
    struct is_matrix<Mat3d> : std::true_type {
    };

    template <>
    struct is_matrix<Mat4d> : std::true_type {
    };

    template <typename T>
    inline constexpr bool is_matrix_v = is_matrix<T>::value;

    // Get vector/matrix dimension
    template <typename T>
    struct dimension {
        static constexpr std::size_t value = 0;
    };

    template <>
    struct dimension<Vec2> {
        static constexpr std::size_t value = 2;
    };

    template <>
    struct dimension<Vec3> {
        static constexpr std::size_t value = 3;
    };

    template <>
    struct dimension<Vec4> {
        static constexpr std::size_t value = 4;
    };

    template <>
    struct dimension<Mat2> {
        static constexpr std::size_t value = 2;
    };

    template <>
    struct dimension<Mat3> {
        static constexpr std::size_t value = 3;
    };

    template <>
    struct dimension<Mat4> {
        static constexpr std::size_t value = 4;
    };

    template <typename T>
    inline constexpr std::size_t dimension_v = dimension<T>::value;
} // namespace engine::math
