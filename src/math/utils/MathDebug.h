/**
 * @file MathDebug.h
 * @brief Debug utilities and validation for math operations
 * @author Andrés Guerrero
 * @date 29-08-2025
 *
 * Provides debugging tools, validation functions, and visualization
 * helpers for mathematical operations in debug builds.
 */

#pragma once

#include "../core/MathTypes.h"
#include "../core/MathConstants.h"
#include "../core/MathFunctions.h"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <cassert>

namespace engine::math::debug {
    // ============================================================================
    // Validation Functions
    // ============================================================================

    /**
     * @brief Check if value is finite (not NaN or Inf)
     */
    template <typename T>
    [[nodiscard]] inline bool isFinite(T value) noexcept {
        return std::isfinite(value);
    }

    /**
     * @brief Check if vector is finite
     */
    template <typename VecType>
    [[nodiscard]] inline bool isFiniteVector(const VecType& v) noexcept {
        for (int i = 0; i < dimension_v<VecType>; ++i) {
            if (!std::isfinite(v[i])) return false;
        }
        return true;
    }

    /**
     * @brief Check if matrix is finite
     */
    [[nodiscard]] inline bool isFiniteMatrix(const Mat4& m) noexcept {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                if (!std::isfinite(m[i][j])) return false;
            }
        }
        return true;
    }

    /**
     * @brief Check if quaternion is normalized
     */
    [[nodiscard]] inline bool isNormalized(const Quat& q, const Float tolerance = 0.01f) noexcept {
        const Float lengthSq = glm::length2(q);
        return std::abs(lengthSq - 1.0f) < tolerance;
    }

    /**
     * @brief Check if vector is normalized
     */
    template <typename VecType>
    [[nodiscard]] inline bool isNormalized(const VecType& v, const Float tolerance = 0.01f) noexcept {
        const Float lengthSq = glm::length2(v);
        return std::abs(lengthSq - 1.0f) < tolerance;
    }

    /**
     * @brief Check if matrix is orthogonal
     */
    [[nodiscard]] inline bool isOrthogonal(const Mat3& m, const Float tolerance = 0.01f) noexcept {
        Mat3 shouldBeIdentity = glm::transpose(m) * m;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                if (const Float expected = (i == j) ? 1.0f : 0.0f; std::abs(shouldBeIdentity[i][j] - expected) > tolerance) {
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * @brief Check if transformation matrix is valid
     */
    [[nodiscard]] inline bool isValidTransform(const Mat4& m) noexcept {
        // Check if finite
        if (!isFiniteMatrix(m)) return false;

        // Check if last row is [0,0,0,1]
        if (!isNearlyEqual(m[0][3], 0.0f) ||
            !isNearlyEqual(m[1][3], 0.0f) ||
            !isNearlyEqual(m[2][3], 0.0f) ||
            !isNearlyEqual(m[3][3], 1.0f)) {
            return false;
        }

        // Check if determinant is reasonable
        if (const Float det = glm::determinant(m); std::abs(det) < EPSILON || std::abs(det) > 1000.0f) {
            return false;
        }

        return true;
    }

    // ============================================================================
    // Debug Assertions
    // ============================================================================

#ifdef _DEBUG
#define MATH_ASSERT(condition, message) \
            do { \
                if (!(condition)) { \
                    std::cerr << "Math Assertion Failed: " << message << "\n" \
                             << "  File: " << __FILE__ << "\n" \
                             << "  Line: " << __LINE__ << "\n"; \
                    assert(false); \
                } \
            } while(0)

#define MATH_ASSERT_FINITE(value) \
            MATH_ASSERT(engine::math::debug::isFinite(value), \
                       "Value is not finite (NaN or Inf)")

#define MATH_ASSERT_NORMALIZED(value) \
            MATH_ASSERT(engine::math::debug::isNormalized(value), \
                       "Value is not normalized")

#define MATH_ASSERT_VALID_TRANSFORM(matrix) \
            MATH_ASSERT(engine::math::debug::isValidTransform(matrix), \
                       "Invalid transformation matrix")
#else
#define MATH_ASSERT(condition, message) ((void)0)
#define MATH_ASSERT_FINITE(value) ((void)0)
#define MATH_ASSERT_NORMALIZED(value) ((void)0)
#define MATH_ASSERT_VALID_TRANSFORM(matrix) ((void)0)
#endif

    // ============================================================================
    // Debug Output
    // ============================================================================

    /**
     * @brief Format vector for debug output
     */
    template <typename VecType>
    [[nodiscard]] std::string toString(const VecType& v, const int precision = 3) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(precision);
        ss << "(";
        for (std::size_t i = 0; i < dimension_v<VecType>; ++i) {
            if (i > 0) ss << ", ";
            ss << v[i];
        }
        ss << ")";
        return ss.str();
    }

    /**
     * @brief Format matrix for debug output
     */
    [[nodiscard]] inline std::string toString(const Mat4& m, const int precision = 3) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(precision);
        ss << "[\n";
        for (int i = 0; i < 4; ++i) {
            ss << "  [";
            for (int j = 0; j < 4; ++j) {
                if (j > 0) ss << ", ";
                ss << std::setw(8) << m[j][i];
            }
            ss << "]\n";
        }
        ss << "]";
        return ss.str();
    }

    /**
     * @brief Format quaternion for debug output
     */
    [[nodiscard]] inline std::string toString(const Quat& q, const int precision = 3) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(precision);
        ss << "Quat(w=" << q.w << ", x=" << q.x << ", y=" << q.y << ", z=" << q.z << ")";
        return ss.str();
    }

    /**
     * @brief Format transform for debug output
     */
    [[nodiscard]] inline std::string toString(const Transform& t, const int precision = 3) {
        std::stringstream ss;
        ss << "Transform {\n";
        ss << "  Position: " << toString(t.getPosition(), precision) << "\n";
        ss << "  Rotation: " << toString(t.getRotation(), precision) << "\n";
        ss << "  Scale: " << toString(t.getScale(), precision) << "\n";
        ss << "}";
        return ss.str();
    }

    // ============================================================================
    // Debug Visualization
    // ============================================================================

    /**
     * @brief Generate debug lines for AABB
     */
    struct DebugLine {
        Vec3 start, end;
        Color color;
    };

    [[nodiscard]] inline std::vector<DebugLine> getAABBDebugLines(
        const AABB& aabb, const Color& color = Color::Green) {
        std::vector<DebugLine> lines;
        const auto corners = aabb.getCorners();

        // Bottom face
        lines.push_back({corners[0], corners[1], color});
        lines.push_back({corners[1], corners[3], color});
        lines.push_back({corners[3], corners[2], color});
        lines.push_back({corners[2], corners[0], color});

        // Top face
        lines.push_back({corners[4], corners[5], color});
        lines.push_back({corners[5], corners[7], color});
        lines.push_back({corners[7], corners[6], color});
        lines.push_back({corners[6], corners[4], color});

        // Vertical edges
        lines.push_back({corners[0], corners[4], color});
        lines.push_back({corners[1], corners[5], color});
        lines.push_back({corners[2], corners[6], color});
        lines.push_back({corners[3], corners[7], color});

        return lines;
    }

    // TODO: Revisar esto
    /**
     * @brief Generate debug lines for frustum
     */
    [[nodiscard]] inline std::vector<DebugLine> getFrustumDebugLines(
        const Frustum& frustum, const Color& color = Color::Yellow) {
        // This would require extracting frustum corners from planes
        // Simplified version
        std::vector<DebugLine> lines;
        // Implementation would calculate corner points from plane intersections
        return lines;
    }

    /**
     * @brief Generate debug sphere approximation
     */
    [[nodiscard]] inline std::vector<Vec3> getSphereDebugPoints(
        const Sphere& sphere, const int segments = 16) {
        std::vector<Vec3> points;

        // Generate circles
        for (int i = 0; i <= segments; ++i) {
            const Float angle = (static_cast<Float>(i) / static_cast<Float>(segments)) * TWO_PI<Float>;
            const Float cos_a = std::cos(angle);
            const Float sin_a = std::sin(angle);

            // XY circle
            points.push_back(sphere.center + Vec3(cos_a, sin_a, 0) * sphere.radius);
            // XZ circle
            points.push_back(sphere.center + Vec3(cos_a, 0, sin_a) * sphere.radius);
            // YZ circle
            points.push_back(sphere.center + Vec3(0, cos_a, sin_a) * sphere.radius);
        }

        return points;
    }

    // ============================================================================
    // Performance Profiling
    // ============================================================================

    /**
     * @brief Simple timer for math operations
     */
    class MathProfiler {
    public:
        void startTimer(const std::string& name) {
#ifdef _DEBUG
            timers_[name] = std::chrono::high_resolution_clock::now();
#endif
        }

        void endTimer(const std::string& name) {
#ifdef _DEBUG
            auto it = timers_.find(name);
            if (it != timers_.end()) {
                auto end = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - it->second);

                results_[name].totalTime += duration.count();
                results_[name].callCount++;
                timers_.erase(it);
            }
#endif
        }

        void printResults() const {
#ifdef _DEBUG
            std::cout << "\n=== Math Profiler Results ===\n";
            for (const auto& [name, data] : results_) {
                Float avgTime = data.totalTime / Float(data.callCount);
                std::cout << name << ":\n";
                std::cout << "  Calls: " << data.callCount << "\n";
                std::cout << "  Total: " << data.totalTime << " us\n";
                std::cout << "  Average: " << avgTime << " us\n";
            }
#endif
        }

    private:
#ifdef _DEBUG
        struct ProfileData {
            std::int64_t totalTime = 0;
            std::int32_t callCount = 0;
        };

        std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point> timers_;
        std::unordered_map<std::string, ProfileData> results_;
#endif
    };

    // Global profiler instance
    inline MathProfiler& getProfiler() {
        static MathProfiler profiler;
        return profiler;
    }

    // Profiling macros
#ifdef _DEBUG
#define MATH_PROFILE_START(name) \
            engine::math::debug::getProfiler().startTimer(name)
#define MATH_PROFILE_END(name) \
            engine::math::debug::getProfiler().endTimer(name)
#else
#define MATH_PROFILE_START(name) ((void)0)
#define MATH_PROFILE_END(name) ((void)0)
#endif
} // namespace engine::math::debug
