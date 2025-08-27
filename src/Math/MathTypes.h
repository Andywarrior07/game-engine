//
// Created by Andres Guerrero on 09-08-25.
//

#pragma once

// GLM - OpenGL Mathematics Library
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/hash.hpp>

// Standard library
#include <cmath>
#include <algorithm>
#include <array>
#include <functional>
#include <random>

namespace engine::math {

    // ========================================================================
    // TYPE ALIASES - Professional GLM-based math types
    // ========================================================================

    // === VECTOR TYPES ===
    using Vector2 = glm::vec2;                              // 2D vector (float precision)
    using Vector3 = glm::vec3;                              // 3D vector (float precision)
    using Vector4 = glm::vec4;                              // 4D vector (float precision)

    using Vector2i = glm::ivec2;                            // 2D integer vector
    using Vector3i = glm::ivec3;                            // 3D integer vector
    using Vector4i = glm::ivec4;                            // 4D integer vector

    using Vector2u = glm::uvec2;                            // 2D unsigned integer vector
    using Vector3u = glm::uvec3;                            // 3D unsigned integer vector
    using Vector4u = glm::uvec4;                            // 4D unsigned integer vector

    using Vector2d = glm::dvec2;                            // 2D double precision vector
    using Vector3d = glm::dvec3;                            // 3D double precision vector
    using Vector4d = glm::dvec4;                            // 4D double precision vector

    // === MATRIX TYPES ===
    using Matrix2 = glm::mat2;                              // 2x2 matrix
    using Matrix3 = glm::mat3;                              // 3x3 matrix
    using Matrix4 = glm::mat4;                              // 4x4 matrix (most common for 3D)

    using Matrix2x3 = glm::mat2x3;                          // 2x3 matrix
    using Matrix2x4 = glm::mat2x4;                          // 2x4 matrix
    using Matrix3x2 = glm::mat3x2;                          // 3x2 matrix
    using Matrix3x4 = glm::mat3x4;                          // 3x4 matrix
    using Matrix4x2 = glm::mat4x2;                          // 4x2 matrix
    using Matrix4x3 = glm::mat4x3;                          // 4x3 matrix

    // === QUATERNION TYPES ===
    using Quaternion = glm::quat;                           // Quaternion for rotations
    using Quaterniond = glm::dquat;                         // Double precision quaternion

    // === COMMON CONSTANTS ===
    namespace constants {
        constexpr float PI = glm::pi<float>();              // π
        constexpr float TWO_PI = 2.0f * PI;                 // 2π
        constexpr float HALF_PI = PI * 0.5f;                // π/2
        constexpr float QUARTER_PI = PI * 0.25f;            // π/4
        constexpr float INV_PI = 1.0f / PI;                 // 1/π
        constexpr float DEG_TO_RAD = PI / 180.0f;           // Degrees to radians
        constexpr float RAD_TO_DEG = 180.0f / PI;           // Radians to degrees
        constexpr float EPSILON = 1e-6f;                    // Small epsilon for float comparisons
        constexpr float SQRT_2 = 1.41421356237f;            // √2
        constexpr float SQRT_3 = 1.73205080757f;            // √3
        constexpr float E = 2.71828182846f;                 // Euler's number

        // Common vectors
        constexpr Vector2 VEC2_ZERO{0.0f, 0.0f};
        constexpr Vector2 VEC2_ONE{1.0f, 1.0f};
        constexpr Vector2 VEC2_UNIT_X{1.0f, 0.0f};
        constexpr Vector2 VEC2_UNIT_Y{0.0f, 1.0f};

        constexpr Vector3 VEC3_ZERO{0.0f, 0.0f, 0.0f};
        constexpr Vector3 VEC3_ONE{1.0f, 1.0f, 1.0f};
        constexpr Vector3 VEC3_UNIT_X{1.0f, 0.0f, 0.0f};       // Right
        constexpr Vector3 VEC3_UNIT_Y{0.0f, 1.0f, 0.0f};       // Up
        constexpr Vector3 VEC3_UNIT_Z{0.0f, 0.0f, 1.0f};       // Forward (OpenGL style)
        constexpr Vector3 VEC3_FORWARD{0.0f, 0.0f, -1.0f};     // Forward direction (looking into -Z)
        constexpr Vector3 VEC3_RIGHT{1.0f, 0.0f, 0.0f};        // Right direction
        constexpr Vector3 VEC3_UP{0.0f, 1.0f, 0.0f};           // Up direction
        constexpr Vector3 VEC3_MIN{-FLT_MAX, -FLT_MAX, -FLT_MAX};  // Minimum possible vector values
        constexpr Vector3 VEC3_MAX{FLT_MAX, FLT_MAX, FLT_MAX};      // Maximum possible vector values

        constexpr Matrix4 MAT4_IDENTITY{1.0f};                  // 4x4 Identity matrix
        constexpr Matrix3 MAT3_IDENTITY{1.0f};                  // 3x3 Identity matrix

        constexpr Quaternion QUAT_IDENTITY{1.0f, 0.0f, 0.0f, 0.0f}; // Identity quaternion
    }

    // ========================================================================
    // UTILITY FUNCTIONS - Extended math operations
    // ========================================================================

    /**
     * @brief Convert degrees to radians
     * @param degrees Angle in degrees
     * @return Angle in radians
     */
    inline float toRadians(float degrees) {
        return degrees * constants::DEG_TO_RAD;
    }

    /**
     * @brief Convert radians to degrees
     * @param radians Angle in radians
     * @return Angle in degrees
     */
    inline float toDegrees(float radians) {
        return radians * constants::RAD_TO_DEG;
    }

    /**
     * @brief Clamp value between min and max
     * @param value Value to clamp
     * @param minVal Minimum value
     * @param maxVal Maximum value
     * @return Clamped value
     */
    template<typename T>
    inline T clamp(const T& value, const T& minVal, const T& maxVal) {
        return glm::clamp(value, minVal, maxVal);
    }

    /**
     * @brief Linear interpolation between two values
     * @param a Start value
     * @param b End value
     * @param t Interpolation factor (0.0 to 1.0)
     * @return Interpolated value
     */
    template<typename T>
    inline T lerp(const T& a, const T& b, float t) {
        return glm::mix(a, b, t);
    }

    /**
     * @brief Smooth step interpolation (Hermite)
     * @param a Start value
     * @param b End value
     * @param t Interpolation factor (0.0 to 1.0)
     * @return Smoothly interpolated value
     */
    template<typename T>
    inline T smoothStep(const T& a, const T& b, float t) {
        t = clamp(t, 0.0f, 1.0f);
        t = t * t * (3.0f - 2.0f * t);
        return lerp(a, b, t);
    }

    /**
     * @brief Smoother step interpolation (improved Hermite)
     * @param a Start value
     * @param b End value
     * @param t Interpolation factor (0.0 to 1.0)
     * @return Very smoothly interpolated value
     */
    template<typename T>
    inline T smootherStep(const T& a, const T& b, float t) {
        t = clamp(t, 0.0f, 1.0f);
        t = t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
        return lerp(a, b, t);
    }

    /**
     * @brief Safe normalize vector (returns zero vector if length is too small)
     * @param vector Vector to normalize
     * @return Normalized vector or zero vector
     */
    inline Vector2 safeNormalize(const Vector2& vector) {
        float len = glm::length(vector);
        return (len > constants::EPSILON) ? vector / len : constants::VEC2_ZERO;
    }

    inline Vector3 safeNormalize(const Vector3& vector) {
        float len = glm::length(vector);
        return (len > constants::EPSILON) ? vector / len : constants::VEC3_ZERO;
    }

    /**
     * @brief Check if two floating point values are approximately equal
     * @param a First value
     * @param b Second value
     * @param epsilon Tolerance (default: EPSILON)
     * @return true if approximately equal
     */
    inline bool isEqual(float a, float b, float epsilon = constants::EPSILON) {
        return std::abs(a - b) < epsilon;
    }

    /**
     * @brief Check if two vectors are approximately equal
     * @param a First vector
     * @param b Second vector
     * @param epsilon Tolerance (default: EPSILON)
     * @return true if approximately equal
     */
    inline bool isEqual(const Vector2& a, const Vector2& b, float epsilon = constants::EPSILON) {
        return glm::length(a - b) < epsilon;
    }

    inline bool isEqual(const Vector3& a, const Vector3& b, float epsilon = constants::EPSILON) {
        return glm::length(a - b) < epsilon;
    }

    /**
     * @brief Check if a floating point value is approximately zero
     * @param value Value to check
     * @param epsilon Tolerance (default: EPSILON)
     * @return true if approximately zero
     */
    inline bool isZero(float value, float epsilon = constants::EPSILON) {
        return std::abs(value) < epsilon;
    }

    /**
     * @brief Check if a vector is approximately zero
     * @param vector Vector to check
     * @param epsilon Tolerance (default: EPSILON)
     * @return true if approximately zero
     */
    inline bool isZero(const Vector2& vector, float epsilon = constants::EPSILON) {
        return glm::length(vector) < epsilon;
    }

    inline bool isZero(const Vector3& vector, float epsilon = constants::EPSILON) {
        return glm::length(vector) < epsilon;
    }

    // ========================================================================
    // SPECIALIZED STRUCTS - Game-specific math types
    // ========================================================================

    /**
     * @brief Rectangle structure for 2D areas and UI
     */
    struct Rectangle {
        float x = 0.0f;
        float y = 0.0f;
        float width = 0.0f;
        float height = 0.0f;

        // Constructors
        Rectangle() = default;
        Rectangle(float x_, float y_, float w, float h) : x(x_), y(y_), width(w), height(h) {}
        Rectangle(const Vector2& position, const Vector2& size) : x(position.x), y(position.y), width(size.x), height(size.y) {}

        // Utility methods
        Vector2 getPosition() const { return Vector2(x, y); }
        Vector2 getSize() const { return Vector2(width, height); }
        Vector2 getCenter() const { return Vector2(x + width * 0.5f, y + height * 0.5f); }

        float getLeft() const { return x; }
        float getRight() const { return x + width; }
        float getTop() const { return y; }
        float getBottom() const { return y + height; }

        void setPosition(const Vector2& position) { x = position.x; y = position.y; }
        void setSize(const Vector2& size) { width = size.x; height = size.y; }
        void setCenter(const Vector2& center) { x = center.x - width * 0.5f; y = center.y - height * 0.5f; }

        bool contains(const Vector2& point) const {
            return point.x >= x && point.x <= x + width && point.y >= y && point.y <= y + height;
        }

        bool intersects(const Rectangle& other) const {
            return !(other.x > x + width || other.x + other.width < x ||
                    other.y > y + height || other.y + other.height < y);
        }

        Rectangle intersection(const Rectangle& other) const {
            float left = std::max(x, other.x);
            float right = std::min(x + width, other.x + other.width);
            float top = std::max(y, other.y);
            float bottom = std::min(y + height, other.y + other.height);

            if (left < right && top < bottom) {
                return Rectangle(left, top, right - left, bottom - top);
            }
            return Rectangle(); // Empty rectangle
        }
    };

    /**
     * @brief Circle structure for 2D collision detection
     */
    struct Circle {
        Vector2 center{0.0f, 0.0f};
        float radius = 0.0f;

        // Constructors
        Circle() = default;
        Circle(const Vector2& c, float r) : center(c), radius(r) {}
        Circle(float x, float y, float r) : center(x, y), radius(r) {}

        // Utility methods
        bool contains(const Vector2& point) const {
            return glm::length(point - center) <= radius;
        }

        bool intersects(const Circle& other) const {
            float distance = glm::length(center - other.center);
            return distance <= (radius + other.radius);
        }

        bool intersects(const Rectangle& rect) const {
            // Find closest point on rectangle to circle center
            Vector2 closest(
                clamp(center.x, rect.x, rect.x + rect.width),
                clamp(center.y, rect.y, rect.y + rect.height)
            );
            return glm::length(center - closest) <= radius;
        }

        float area() const { return constants::PI * radius * radius; }
        float circumference() const { return constants::TWO_PI * radius; }
    };

    /**
     * @brief Transform structure for 2D objects (Animation compatibility)
     */
    struct Transform2D {
        Vector2 position{0.0f, 0.0f};                       // Position in 2D space
        Vector2 scale{1.0f, 1.0f};                          // Scale factors
        float rotation = 0.0f;                              // Rotation in radians
        Vector2 origin{0.5f, 0.5f};                         // Origin point (0,0 = top-left, 0.5,0.5 = center)
        int layer = 0;                                      // Rendering layer
        float alpha = 1.0f;                                 // Alpha transparency (0.0 to 1.0)
        bool visible = true;                                // Visibility flag

        // Constructors
        Transform2D() = default;
        Transform2D(const Vector2& pos) : position(pos) {}
        Transform2D(const Vector2& pos, const Vector2& scl, float rot = 0.0f)
            : position(pos), scale(scl), rotation(rot) {}

        // Utility methods
        Matrix4 getTransformMatrix() const {
            // Convertir Vector2 a Vector3 agregando z=0
            Vector3 pos3D(position.x, position.y, 0.0f);
            Vector3 scale3D(scale.x, scale.y, 1.0f);

            // Usar el eje Z para rotación 2D
            Vector3 rotationAxis(0.0f, 0.0f, 1.0f);

            Matrix4 translation = glm::translate(Matrix4(1.0f), pos3D);
            Matrix4 rotationMat = glm::rotate(Matrix4(1.0f), rotation, rotationAxis);
            Matrix4 scaleMat = glm::scale(Matrix4(1.0f), scale3D);

            return translation * rotationMat * scaleMat;
        }


        Vector2 transformPoint(const Vector2& point) const {
            Matrix3 matrix = getTransformMatrix();
            Vector3 result = matrix * Vector3(point.x, point.y, 1.0f);
            return Vector2(result.x, result.y);
        }

        Rectangle getBounds(const Vector2& size) const {
            // Calculate bounding box of transformed rectangle
            std::array<Vector2, 4> corners = {
                transformPoint(Vector2(-origin.x * size.x, -origin.y * size.y)),
                transformPoint(Vector2((1.0f - origin.x) * size.x, -origin.y * size.y)),
                transformPoint(Vector2((1.0f - origin.x) * size.x, (1.0f - origin.y) * size.y)),
                transformPoint(Vector2(-origin.x * size.x, (1.0f - origin.y) * size.y))
            };

            float minX = corners[0].x, maxX = corners[0].x;
            float minY = corners[0].y, maxY = corners[0].y;

            for (const auto& corner : corners) {
                minX = std::min(minX, corner.x);
                maxX = std::max(maxX, corner.x);
                minY = std::min(minY, corner.y);
                maxY = std::max(maxY, corner.y);
            }

            return Rectangle(minX, minY, maxX - minX, maxY - minY);
        }
    };

    /**
     * @brief Transform structure for 3D objects (Camera compatibility)
     */
    struct Transform3D {
        Vector3 position{0.0f, 0.0f, 0.0f};                 // Position in 3D space
        Quaternion rotation{1.0f, 0.0f, 0.0f, 0.0f};        // Rotation as quaternion
        Vector3 scale{1.0f, 1.0f, 1.0f};                    // Scale factors

        // Constructors
        Transform3D() = default;
        Transform3D(const Vector3& pos) : position(pos) {}
        Transform3D(const Vector3& pos, const Quaternion& rot, const Vector3& scl = Vector3(1.0f))
            : position(pos), rotation(rot), scale(scl) {}

        // Euler angle convenience methods
        void setRotationEuler(float yaw, float pitch, float roll) {
            rotation = glm::quat(Vector3(pitch, yaw, roll));
        }

        Vector3 getRotationEuler() const {
            return glm::eulerAngles(rotation);
        }

        // Direction vectors
        Vector3 getForward() const {
            return rotation * constants::VEC3_FORWARD;
        }

        Vector3 getRight() const {
            return rotation * constants::VEC3_RIGHT;
        }

        Vector3 getUp() const {
            return rotation * constants::VEC3_UP;
        }

        // Transform matrix
        Matrix4 getTransformMatrix() const {
            Matrix4 translation = glm::translate(Matrix4(1.0f), position);
            Matrix4 rotationMat = glm::mat4_cast(rotation);
            Matrix4 scaleMat = glm::scale(Matrix4(1.0f), scale);

            return translation * rotationMat * scaleMat;
        }

        Vector3 transformPoint(const Vector3& point) const {
            Matrix4 matrix = getTransformMatrix();
            Vector4 result = matrix * Vector4(point.x, point.y, point.z, 1.0f);
            return Vector3(result.x, result.y, result.z);
        }

        Vector3 transformDirection(const Vector3& direction) const {
            return rotation * direction;
        }
    };

    /**
     * @brief Bounds structure for 3D collision detection and culling
     */
    struct Bounds3D {
        Vector3 min{-1000.0f, -1000.0f, -1000.0f};          // Minimum bounds
        Vector3 max{1000.0f, 1000.0f, 1000.0f};             // Maximum bounds
        bool enabled = false;                               // Whether bounds are active
        float padding = 0.0f;                               // Extra padding around bounds

        // Constructors
        Bounds3D() = default;
        Bounds3D(const Vector3& minBounds, const Vector3& maxBounds, float pad = 0.0f)
            : min(minBounds), max(maxBounds), enabled(true), padding(pad) {}

        // Utility methods
        Vector3 getCenter() const { return (min + max) * 0.5f; }
        Vector3 getSize() const { return max - min; }
        Vector3 getExtents() const { return (max - min) * 0.5f; }

        Vector3 clamp(const Vector3& position) const {
            if (!enabled) return position;
            return glm::clamp(position, min + padding, max - padding);
        }

        bool contains(const Vector3& position) const {
            if (!enabled) return true;
            return position.x >= min.x + padding && position.x <= max.x - padding &&
                   position.y >= min.y + padding && position.y <= max.y - padding &&
                   position.z >= min.z + padding && position.z <= max.z - padding;
        }

        bool intersects(const Bounds3D& other) const {
            return !(other.min.x > max.x || other.max.x < min.x ||
                    other.min.y > max.y || other.max.y < min.y ||
                    other.min.z > max.z || other.max.z < min.z);
        }
    };

    /**
     * @brief Viewport structure for rendering and coordinate transformations
     */
    struct Viewport {
        int x = 0;                                          // X position of viewport
        int y = 0;                                          // Y position of viewport
        int width = 1280;                                   // Width of viewport
        int height = 720;                                   // Height of viewport
        float pixelRatio = 1.0f;                           // Pixel density ratio for high-DPI displays

        // Constructors
        Viewport() = default;
        Viewport(int x_, int y_, int w, int h, float ratio = 1.0f)
            : x(x_), y(y_), width(w), height(h), pixelRatio(ratio) {}

        // Utility methods
        float getAspectRatio() const { return static_cast<float>(width) / static_cast<float>(height); }
        Vector2 getCenter() const { return Vector2(x + width * 0.5f, y + height * 0.5f); }
        Vector2 getSize() const { return Vector2(static_cast<float>(width), static_cast<float>(height)); }

        bool contains(const Vector2& point) const {
            return point.x >= x && point.x < x + width && point.y >= y && point.y < y + height;
        }

        Vector2 normalizePoint(const Vector2& point) const {
            return Vector2(
                (point.x - x) / (width * 0.5f) - 1.0f,
                (point.y - y) / (height * 0.5f) - 1.0f
            );
        }

        Vector2 worldToScreen(const Vector2& worldPoint) const {
            return Vector2(
                worldPoint.x + width * 0.5f + x,
                worldPoint.y + height * 0.5f + y
            );
        }

        Vector2 screenToWorld(const Vector2& screenPoint) const {
            return Vector2(
                screenPoint.x - width * 0.5f - x,
                screenPoint.y - height * 0.5f - y
            );
        }
    };

    // ========================================================================
    // EASING FUNCTIONS - For smooth animations and transitions
    // ========================================================================

    namespace easing {
        // Linear
        inline float linear(float t) { return t; }

        // Quadratic
        inline float easeInQuad(float t) { return t * t; }
        inline float easeOutQuad(float t) { return 1.0f - (1.0f - t) * (1.0f - t); }
        inline float easeInOutQuad(float t) {
            return t < 0.5f ? 2.0f * t * t : 1.0f - 2.0f * (1.0f - t) * (1.0f - t);
        }

        // Cubic
        inline float easeInCubic(float t) { return t * t * t; }
        inline float easeOutCubic(float t) { return 1.0f - (1.0f - t) * (1.0f - t) * (1.0f - t); }
        inline float easeInOutCubic(float t) {
            return t < 0.5f ? 4.0f * t * t * t : 1.0f - 4.0f * (1.0f - t) * (1.0f - t) * (1.0f - t);
        }

        // Back
        inline float easeInBack(float t) {
            const float c1 = 1.70158f;
            const float c3 = c1 + 1.0f;
            return c3 * t * t * t - c1 * t * t;
        }

        inline float easeOutBack(float t) {
            const float c1 = 1.70158f;
            const float c3 = c1 + 1.0f;
            return 1.0f + c3 * std::pow(t - 1.0f, 3.0f) + c1 * std::pow(t - 1.0f, 2.0f);
        }

        // Elastic
        inline float easeInElastic(float t) {
            if (t == 0.0f) return 0.0f;
            if (t == 1.0f) return 1.0f;
            const float c4 = constants::TWO_PI / 3.0f;
            return -std::pow(2.0f, 10.0f * (t - 1.0f)) * std::sin((t - 1.0f) * c4);
        }

        inline float easeOutElastic(float t) {
            if (t == 0.0f) return 0.0f;
            if (t == 1.0f) return 1.0f;
            const float c4 = constants::TWO_PI / 3.0f;
            return std::pow(2.0f, -10.0f * t) * std::sin(t * c4) + 1.0f;
        }

        // Bounce
        inline float easeOutBounce(float t) {
            const float n1 = 7.5625f;
            const float d1 = 2.75f;

            if (t < 1.0f / d1) {
                return n1 * t * t;
            } else if (t < 2.0f / d1) {
                return n1 * (t -= 1.5f / d1) * t + 0.75f;
            } else if (t < 2.5f / d1) {
                return n1 * (t -= 2.25f / d1) * t + 0.9375f;
            } else {
                return n1 * (t -= 2.625f / d1) * t + 0.984375f;
            }
        }
    }

    // ========================================================================
    // NOISE FUNCTIONS - For procedural generation and effects
    // ========================================================================

    namespace noise {
        /**
         * @brief Simple random number generator
         * @param seed Seed value
         * @return Random value between 0.0 and 1.0
         */
        inline float random(int seed) {
            seed = (seed << 13) ^ seed;
            return (1.0f - ((seed * (seed * seed * 15731 + 789221) + 1376312589) & 0x7fffffff) / 1073741824.0f) * 0.5f + 0.5f;
        }

        /**
         * @brief 2D random function
         * @param p 2D position
         * @return Random value between 0.0 and 1.0
         */
        inline float random(const Vector2& p) {
            return random(static_cast<int>(p.x * 12.9898f + p.y * 78.233f) * 43758);
        }

        /**
         * @brief Simple 1D noise function
         * @param x Input value
         * @return Noise value between -1.0 and 1.0
         */
        inline float noise1D(float x) {
            int i = static_cast<int>(std::floor(x));
            float f = x - i;
            float u = f * f * (3.0f - 2.0f * f); // Smoothstep

            float a = random(i);
            float b = random(i + 1);

            return lerp(a, b, u) * 2.0f - 1.0f;
        }

        /**
         * @brief Simple 2D noise function
         * @param p 2D position
         * @return Noise value between -1.0 and 1.0
         */
        inline float noise2D(const Vector2& p) {
            Vector2 i = glm::floor(p);
            Vector2 f = p - i;
            Vector2 u = f * f * (Vector2(3.0f) - 2.0f * f);

            float a = random(i);
            float b = random(i + Vector2(1.0f, 0.0f));
            float c = random(i + Vector2(0.0f, 1.0f));
            float d = random(i + Vector2(1.0f, 1.0f));

            float x1 = lerp(a, b, u.x);
            float x2 = lerp(c, d, u.x);

            return lerp(x1, x2, u.y) * 2.0f - 1.0f;
        }
    }

    // ========================================================================
    // COLOR UTILITIES - For color manipulation and conversion
    // ========================================================================

    namespace color {
        using Color3 = Vector3;                             // RGB color (0.0 to 1.0)
        using Color4 = Vector4;                             // RGBA color (0.0 to 1.0)
        using Color3u8 = Vector3u;                          // RGB color (0 to 255)
        using Color4u8 = Vector4u;                          // RGBA color (0 to 255)

        // Common colors
        const Color3 WHITE{1.0f, 1.0f, 1.0f};
        const Color3 BLACK{0.0f, 0.0f, 0.0f};
        const Color3 RED{1.0f, 0.0f, 0.0f};
        const Color3 GREEN{0.0f, 1.0f, 0.0f};
        const Color3 BLUE{0.0f, 0.0f, 1.0f};
        const Color3 YELLOW{1.0f, 1.0f, 0.0f};
        const Color3 MAGENTA{1.0f, 0.0f, 1.0f};
        const Color3 CYAN{0.0f, 1.0f, 1.0f};
        const Color3 GRAY{0.5f, 0.5f, 0.5f};

        const Color4 WHITE_ALPHA{1.0f, 1.0f, 1.0f, 1.0f};
        const Color4 BLACK_ALPHA{0.0f, 0.0f, 0.0f, 1.0f};
        const Color4 TRANSPARENT{0.0f, 0.0f, 0.0f, 0.0f};

        /**
         * @brief Convert RGB to HSV color space
         * @param rgb RGB color (0.0 to 1.0)
         * @return HSV color (H: 0-360, S: 0-1, V: 0-1)
         */
        inline Vector3 rgbToHsv(const Color3& rgb) {
            float max = std::max({rgb.r, rgb.g, rgb.b});
            float min = std::min({rgb.r, rgb.g, rgb.b});
            float delta = max - min;

            Vector3 hsv;

            // Value
            hsv.z = max;

            // Saturation
            hsv.y = (max != 0.0f) ? (delta / max) : 0.0f;

            // Hue
            if (delta == 0.0f) {
                hsv.x = 0.0f;
            } else if (max == rgb.r) {
                hsv.x = 60.0f * (std::fmod((rgb.g - rgb.b) / delta, 6.0f));
            } else if (max == rgb.g) {
                hsv.x = 60.0f * ((rgb.b - rgb.r) / delta + 2.0f);
            } else {
                hsv.x = 60.0f * ((rgb.r - rgb.g) / delta + 4.0f);
            }

            if (hsv.x < 0.0f) hsv.x += 360.0f;

            return hsv;
        }

        /**
         * @brief Convert HSV to RGB color space
         * @param hsv HSV color (H: 0-360, S: 0-1, V: 0-1)
         * @return RGB color (0.0 to 1.0)
         */
        inline Color3 hsvToRgb(const Vector3& hsv) {
            float c = hsv.z * hsv.y;
            float x = c * (1.0f - std::abs(std::fmod(hsv.x / 60.0f, 2.0f) - 1.0f));
            float m = hsv.z - c;

            Color3 rgb;

            if (hsv.x >= 0.0f && hsv.x < 60.0f) {
                rgb = Color3(c, x, 0.0f);
            } else if (hsv.x >= 60.0f && hsv.x < 120.0f) {
                rgb = Color3(x, c, 0.0f);
            } else if (hsv.x >= 120.0f && hsv.x < 180.0f) {
                rgb = Color3(0.0f, c, x);
            } else if (hsv.x >= 180.0f && hsv.x < 240.0f) {
                rgb = Color3(0.0f, x, c);
            } else if (hsv.x >= 240.0f && hsv.x < 300.0f) {
                rgb = Color3(x, 0.0f, c);
            } else {
                rgb = Color3(c, 0.0f, x);
            }

            return rgb + Vector3(m);
        }

        /**
         * @brief Convert color from 0-255 range to 0-1 range
         * @param color Color with values 0-255
         * @return Color with values 0-1
         */
        inline Color3 u8ToFloat(const Color3u8& color) {
            return Color3(color.r / 255.0f, color.g / 255.0f, color.b / 255.0f);
        }

        inline Color4 u8ToFloat(const Color4u8& color) {
            return Color4(color.r / 255.0f, color.g / 255.0f, color.b / 255.0f, color.a / 255.0f);
        }

        /**
         * @brief Convert color from 0-1 range to 0-255 range
         * @param color Color with values 0-1
         * @return Color with values 0-255
         */
        inline Color3u8 floatToU8(const Color3& color) {
            return Color3u8(
                static_cast<uint32_t>(clamp(color.r, 0.0f, 1.0f) * 255.0f),
                static_cast<uint32_t>(clamp(color.g, 0.0f, 1.0f) * 255.0f),
                static_cast<uint32_t>(clamp(color.b, 0.0f, 1.0f) * 255.0f)
            );
        }

        inline Color4u8 floatToU8(const Color4& color) {
            return Color4u8(
                static_cast<uint32_t>(clamp(color.r, 0.0f, 1.0f) * 255.0f),
                static_cast<uint32_t>(clamp(color.g, 0.0f, 1.0f) * 255.0f),
                static_cast<uint32_t>(clamp(color.b, 0.0f, 1.0f) * 255.0f),
                static_cast<uint32_t>(clamp(color.a, 0.0f, 1.0f) * 255.0f)
            );
        }

        /**
         * @brief Blend two colors using alpha blending
         * @param source Source color (foreground)
         * @param destination Destination color (background)
         * @return Blended color
         */
        inline Color4 alphaBlend(const Color4& source, const Color4& destination) {
            float alpha = source.a + destination.a * (1.0f - source.a);
            if (alpha == 0.0f) return TRANSPARENT;

            Color4 result;
            result.r = (source.r * source.a + destination.r * destination.a * (1.0f - source.a)) / alpha;
            result.g = (source.g * source.a + destination.g * destination.a * (1.0f - source.a)) / alpha;
            result.b = (source.b * source.a + destination.b * destination.a * (1.0f - source.a)) / alpha;
            result.a = alpha;

            return result;
        }
    }

    // ========================================================================
    // ADVANCED MATH UTILITIES - For specialized calculations
    // ========================================================================

    /**
     * @brief Calculate signed distance from point to line segment
     * @param point Point to test
     * @param lineStart Start of line segment
     * @param lineEnd End of line segment
     * @return Signed distance (negative if on left side)
     */
    inline float distanceToLine(const Vector2& point, const Vector2& lineStart, const Vector2& lineEnd) {
        Vector2 lineVec = lineEnd - lineStart;
        Vector2 pointVec = point - lineStart;

        float lineLength = glm::length(lineVec);
        if (lineLength < constants::EPSILON) {
            return glm::length(pointVec);
        }

        Vector2 lineDir = lineVec / lineLength;
        float projLength = glm::dot(pointVec, lineDir);

        if (projLength < 0.0f) {
            return glm::length(pointVec);
        } else if (projLength > lineLength) {
            return glm::length(point - lineEnd);
        } else {
            Vector2 projection = lineStart + lineDir * projLength;
            return glm::length(point - projection);
        }
    }

    /**
     * @brief Check if point is inside triangle
     * @param point Point to test
     * @param a First triangle vertex
     * @param b Second triangle vertex
     * @param c Third triangle vertex
     * @return true if point is inside triangle
     */
    inline bool pointInTriangle(const Vector2& point, const Vector2& a, const Vector2& b, const Vector2& c) {
        Vector2 v0 = c - a;
        Vector2 v1 = b - a;
        Vector2 v2 = point - a;

        float dot00 = glm::dot(v0, v0);
        float dot01 = glm::dot(v0, v1);
        float dot02 = glm::dot(v0, v2);
        float dot11 = glm::dot(v1, v1);
        float dot12 = glm::dot(v1, v2);

        float invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
        float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

        return (u >= 0.0f) && (v >= 0.0f) && (u + v <= 1.0f);
    }

    /**
     * @brief Calculate barycentric coordinates of point in triangle
     * @param point Point to calculate coordinates for
     * @param a First triangle vertex
     * @param b Second triangle vertex
     * @param c Third triangle vertex
     * @return Barycentric coordinates (u, v, w) where point = u*a + v*b + w*c
     */
    inline Vector3 barycentricCoordinates(const Vector2& point, const Vector2& a, const Vector2& b, const Vector2& c) {
        Vector2 v0 = b - a;
        Vector2 v1 = c - a;
        Vector2 v2 = point - a;

        float d00 = glm::dot(v0, v0);
        float d01 = glm::dot(v0, v1);
        float d11 = glm::dot(v1, v1);
        float d20 = glm::dot(v2, v0);
        float d21 = glm::dot(v2, v1);

        float denom = d00 * d11 - d01 * d01;
        if (std::abs(denom) < constants::EPSILON) {
            return Vector3(1.0f, 0.0f, 0.0f); // Degenerate triangle
        }

        float v = (d11 * d20 - d01 * d21) / denom;
        float w = (d00 * d21 - d01 * d20) / denom;
        float u = 1.0f - v - w;

        return Vector3(u, v, w);
    }

    /**
     * @brief Calculate area of triangle
     * @param a First vertex
     * @param b Second vertex
     * @param c Third vertex
     * @return Triangle area
     */
    inline float triangleArea(const Vector2& a, const Vector2& b, const Vector2& c) {
        return std::abs((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)) * 0.5f;
    }

    /**
     * @brief Create a look-at matrix (right-handed coordinate system)
     * @param eye Camera position
     * @param center Target position
     * @param up Up vector
     * @return Look-at matrix
     */
    inline Matrix4 lookAt(const Vector3& eye, const Vector3& center, const Vector3& up) {
        return glm::lookAt(eye, center, up);
    }

    /**
     * @brief Create a perspective projection matrix
     * @param fov Field of view in radians
     * @param aspect Aspect ratio (width/height)
     * @param near Near clipping plane
     * @param far Far clipping plane
     * @return Perspective projection matrix
     */
    inline Matrix4 perspective(float fov, float aspect, float near, float far) {
        return glm::perspective(fov, aspect, near, far);
    }

    /**
     * @brief Create an orthographic projection matrix
     * @param left Left clipping plane
     * @param right Right clipping plane
     * @param bottom Bottom clipping plane
     * @param top Top clipping plane
     * @param near Near clipping plane
     * @param far Far clipping plane
     * @return Orthographic projection matrix
     */
    inline Matrix4 ortho(float left, float right, float bottom, float top, float near, float far) {
        return glm::ortho(left, right, bottom, top, near, far);
    }

    /**
     * @brief Create a 2D orthographic projection matrix
     * @param left Left clipping plane
     * @param right Right clipping plane
     * @param bottom Bottom clipping plane
     * @param top Top clipping plane
     * @return 2D orthographic projection matrix
     */
    inline Matrix4 ortho2D(float left, float right, float bottom, float top) {
        return glm::ortho(left, right, bottom, top);
    }

    // ========================================================================
    // RANDOM NUMBER GENERATION - Improved random utilities
    // ========================================================================

    namespace random {
        /**
         * @brief Thread-local random number generator
         */
        class Generator {
        private:
            inline thread_local static std::mt19937 generator_;
            inline thread_local static bool initialized_;

        public:
            static void seed(uint32_t seedValue) {
                generator_.seed(seedValue);
                initialized_ = true;
            }

            static void seedWithTime() {
                auto now = std::chrono::high_resolution_clock::now();
                auto duration = now.time_since_epoch();
                auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
                seed(static_cast<uint32_t>(nanoseconds));
            }

            static std::mt19937& get() {
                if (!initialized_) {
                    seedWithTime();
                }
                return generator_;
            }
        };

        /**
         * @brief Generate random float between 0.0 and 1.0
         */
        inline float unit() {
            static thread_local std::uniform_real_distribution<float> dist(0.0f, 1.0f);
            return dist(Generator::get());
        }

        /**
         * @brief Generate random float between min and max
         */
        inline float range(float min, float max) {
            return min + unit() * (max - min);
        }

        /**
         * @brief Generate random integer between min and max (inclusive)
         */
        inline int range(int min, int max) {
            std::uniform_int_distribution<int> dist(min, max);
            return dist(Generator::get());
        }

        /**
         * @brief Generate random Vector2 with components between 0.0 and 1.0
         */
        inline Vector2 unitVector2() {
            return Vector2(unit(), unit());
        }

        /**
         * @brief Generate random Vector3 with components between 0.0 and 1.0
         */
        inline Vector3 unitVector3() {
            return Vector3(unit(), unit(), unit());
        }

        /**
         * @brief Generate random Vector2 within given ranges
         */
        inline Vector2 rangeVector2(const Vector2& min, const Vector2& max) {
            return Vector2(range(min.x, max.x), range(min.y, max.y));
        }

        /**
         * @brief Generate random Vector3 within given ranges
         */
        inline Vector3 rangeVector3(const Vector3& min, const Vector3& max) {
            return Vector3(range(min.x, max.x), range(min.y, max.y), range(min.z, max.z));
        }

        /**
         * @brief Generate random point on unit circle
         */
        inline Vector2 onUnitCircle() {
            float angle = range(0.0f, constants::TWO_PI);
            return Vector2(std::cos(angle), std::sin(angle));
        }

        /**
         * @brief Generate random point inside unit circle
         */
        inline Vector2 insideUnitCircle() {
            Vector2 point;
            do {
                point = Vector2(range(-1.0f, 1.0f), range(-1.0f, 1.0f));
            } while (glm::length(point) > 1.0f);
            return point;
        }

        /**
         * @brief Generate random point on unit sphere
         */
        inline Vector3 onUnitSphere() {
            float z = range(-1.0f, 1.0f);
            float a = range(0.0f, constants::TWO_PI);
            float r = std::sqrt(1.0f - z * z);
            return Vector3(r * std::cos(a), r * std::sin(a), z);
        }

        /**
         * @brief Generate random point inside unit sphere
         */
        inline Vector3 insideUnitSphere() {
            Vector3 point;
            do {
                point = Vector3(range(-1.0f, 1.0f), range(-1.0f, 1.0f), range(-1.0f, 1.0f));
            } while (glm::length(point) > 1.0f);
            return point;
        }
    }

} // namespace engine::math

// ========================================================================
// HASH SPECIALIZATIONS - For using math types in hash containers
// ========================================================================

namespace std {
    template<>
    struct hash<engine::math::Vector2> {
        size_t operator()(const engine::math::Vector2& v) const {
            return hash<glm::vec2>()(v);
        }
    };

    template<>
    struct hash<engine::math::Vector3> {
        size_t operator()(const engine::math::Vector3& v) const {
            return hash<glm::vec3>()(v);
        }
    };

    template<>
    struct hash<engine::math::Vector4> {
        size_t operator()(const engine::math::Vector4& v) const {
            return hash<glm::vec4>()(v);
        }
    };

    template<>
    struct hash<engine::math::Vector2i> {
        size_t operator()(const engine::math::Vector2i& v) const {
            return hash<glm::ivec2>()(v);
        }
    };

    template<>
    struct hash<engine::math::Vector3i> {
        size_t operator()(const engine::math::Vector3i& v) const {
            return hash<glm::ivec3>()(v);
        }
    };
}