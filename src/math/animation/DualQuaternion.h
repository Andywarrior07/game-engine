/**
 * @file DualQuaternion.h
 * @brief Dual quaternion implementation for skeletal animation
 * @author Andrés Guerrero
 * @date 29-08-2025
 *
 * Provides dual quaternion mathematics for smooth skeletal animation
 * blending without artifacts common in linear blend skinning.
 */

#pragma once

#include <array>

#include "../core/MathTypes.h"
#include "../core/MathConstants.h"
#include "../core/MathFunctions.h"
#include "../transform/Transform.h"

namespace engine::math {
    /**
     * @brief Dual quaternion for rigid transformations
     *
     * Represents rotation and translation in a single mathematical object.
     * More efficient and artifact-free compared to matrix blending.
     *
     * Structure: q = q_r + ε*q_d where ε² = 0
     * q_r = real part (rotation quaternion)
     * q_d = dual part (encodes translation)
     */
    class DualQuaternion {
    public:
        Quat real; // Rotation quaternion
        Quat dual; // Dual part (encodes translation)

        // ============================================================================
        // Constructors
        // ============================================================================

        /**
         * @brief Default constructor - identity transformation
         */
        DualQuaternion() noexcept;

        /**
         * @brief Construct from real and dual parts
         */
        DualQuaternion(const Quat& r, const Quat& d) noexcept;

        /**
         * @brief Construct from rotation quaternion and translation
         */
        DualQuaternion(const Quat& rotation, const Vec3& translation) noexcept;

        /**
         * @brief Construct from transformation matrix
         */
        explicit DualQuaternion(const Mat4& matrix) noexcept;

        /**
         * @brief Construct from Transform
         */
        explicit DualQuaternion(const Transform& transform) noexcept;

        // ============================================================================
        // Operations
        // ============================================================================

        /**
         * @brief Multiplication of dual quaternions (composition of transformations)
         */
        [[nodiscard]] DualQuaternion operator*(const DualQuaternion& other) const noexcept {
            return DualQuaternion(
                real * other.real,
                real * other.dual + dual * other.real
            );
        }

        /**
         * @brief Addition (for blending)
         */
        [[nodiscard]] DualQuaternion operator+(const DualQuaternion& other) const noexcept {
            return DualQuaternion(real + other.real, dual + other.dual);
        }

        /**
         * @brief Scalar multiplication (for weighted blending)
         */
        [[nodiscard]] DualQuaternion operator*(const Float scalar) const noexcept {
            return DualQuaternion(real * scalar, dual * scalar);
        }

        /**
         * @brief Conjugate of dual quaternion
         */
        [[nodiscard]] DualQuaternion conjugate() const noexcept {
            return DualQuaternion(glm::conjugate(real), glm::conjugate(dual));
        }

        /**
         * @brief Normalize dual quaternion
         */
        [[nodiscard]] DualQuaternion normalized() const noexcept;

        void normalize() noexcept;

        // ============================================================================
        // Extraction Methods
        // ============================================================================

        /**
         * @brief Extract rotation quaternion
         */
        [[nodiscard]] Quat getRotation() const noexcept;

        /**
         * @brief Extract translation vector
         */
        [[nodiscard]] Vec3 getTranslation() const noexcept;

        /**
         * @brief Convert to transformation matrix
         */
        [[nodiscard]] Mat4 toMatrix() const noexcept;

        /**
         * @brief Convert to Transform
         */
        [[nodiscard]] Transform toTransform() const noexcept {
            return Transform(getTranslation(), getRotation(), Vec3(1));
        }

        // ============================================================================
        // Transformation Methods
        // ============================================================================

        /**
         * @brief Transform a point
         */
        [[nodiscard]] Vec3 transformPoint(const Vec3& point) const noexcept;

        /**
         * @brief Transform a vector (rotation only)
         */
        [[nodiscard]] Vec3 transformVector(const Vec3& vector) const noexcept;

        /**
         * @brief Inverse transformation
         */
        [[nodiscard]] DualQuaternion inverse() const noexcept;

        // ============================================================================
        // Interpolation Methods
        // ============================================================================

        /**
         * @brief Screw linear interpolation (ScLERP)
         * Interpolates rotation and translation along a screw motion
         */
        [[nodiscard]] static DualQuaternion sclerp(const DualQuaternion& a,
                                                   const DualQuaternion& b,
                                                   const Float t) noexcept;

        /**
         * @brief Dual quaternion linear blend (DLB)
         * Faster than ScLERP but less accurate for large rotations
         */
        [[nodiscard]] static DualQuaternion dlb(const std::vector<DualQuaternion>& dqs,
                                                const std::vector<Float>& weights) noexcept;

        /**
         * @brief Optimized dual quaternion blend for skinning
         * Handles up to 4 bone influences efficiently
         */
        [[nodiscard]] static DualQuaternion skinningBlend(
            const std::array<DualQuaternion, 4>& bones,
            const Vec4& weights) noexcept;

    private:
        /**
         * @brief Dual quaternion power (for ScLERP)
         */
        [[nodiscard]] static DualQuaternion power(const DualQuaternion& dq, Float t) noexcept;
    };

    // Convenience operator
    [[nodiscard]] inline DualQuaternion operator*(const Float scalar, const DualQuaternion& dq) noexcept {
        return dq * scalar;
    }
} // namespace engine::math
