/**
* @file DualQuaternion.h
 * @brief Dual quaternion implementation for skeletal animation
 * @author Andrés Guerrero
 * @date 29-08-2025
 *
 * Provides dual quaternion mathematics for smooth skeletal animation
 * blending without artifacts common in linear blend skinning.
 */

#include "DualQuaternion.h"

namespace engine::math {
    DualQuaternion::DualQuaternion() noexcept
        : real(1, 0, 0, 0), dual(0, 0, 0, 0) {
    }

    DualQuaternion::DualQuaternion(const Quat& r, const Quat& d) noexcept
        : real(r), dual(d) {
    }

    DualQuaternion::DualQuaternion(const Quat& rotation, const Vec3& translation) noexcept {
        real = rotation;
        // Dual part: 0.5 * t * r where t is translation as pure quaternion
        const Quat t(0, translation.x, translation.y, translation.z);
        dual = (t * real) * 0.5f;
    }

    DualQuaternion::DualQuaternion(const Mat4& matrix) noexcept {
        // Extract rotation
        const auto rotMatrix = Mat3(matrix);
        real = glm::normalize(glm::quat_cast(rotMatrix));

        // Extract translation
        const auto translation = Vec3(matrix[3]);
        const Quat t(0, translation.x, translation.y, translation.z);
        dual = (t * real) * 0.5f;
    }

    DualQuaternion::DualQuaternion(const Transform& transform) noexcept {
        real = transform.getRotation();
        const Vec3 translation = transform.getPosition();
        const Quat t(0, translation.x, translation.y, translation.z);
        dual = (t * real) * 0.5f;
    }

    DualQuaternion DualQuaternion::normalized() const noexcept {
        const Float magnitude = glm::length(real);
        if (magnitude < EPSILON) return DualQuaternion();

        const Float invMag = 1.0f / magnitude;
        return DualQuaternion(real * invMag, dual * invMag);
    }

    void DualQuaternion::normalize() noexcept {
        const Float magnitude = glm::length(real);
        if (magnitude < EPSILON) return;

        const Float invMag = 1.0f / magnitude;
        real *= invMag;
        dual *= invMag;
    }

    Quat DualQuaternion::getRotation() const noexcept {
        return glm::normalize(real);
    }

    Vec3 DualQuaternion::getTranslation() const noexcept {
        // t = 2 * q_d * conjugate(q_r)
        const Quat t = (dual * glm::conjugate(real)) * 2.0f;
        return Vec3(t.x, t.y, t.z);
    }

    Mat4 DualQuaternion::toMatrix() const noexcept {
        // Ensure normalized
        const DualQuaternion normalized = this->normalized();

        // Rotation matrix from real part
        Mat4 matrix = glm::mat4_cast(normalized.real);

        // Translation from dual part
        const Vec3 translation = normalized.getTranslation();
        matrix[3] = Vec4(translation, 1.0f);

        return matrix;
    }

    Vec3 DualQuaternion::transformPoint(const Vec3& point) const noexcept {
        // Convert point to dual quaternion
        const DualQuaternion p(Quat(1, 0, 0, 0), point);

        // Transform: p' = q * p * conjugate(q)
        const DualQuaternion result = (*this) * p * conjugate();

        return result.getTranslation();
    }

    Vec3 DualQuaternion::transformVector(const Vec3& vector) const noexcept {
        return glm::rotate(real, vector);
    }

    DualQuaternion DualQuaternion::inverse() const noexcept {
        const Quat realConj = glm::conjugate(real);
        const Quat dualConj = glm::conjugate(dual);

        // For unit dual quaternions: inverse = conjugate
        return DualQuaternion(realConj, -dualConj);
    }

    DualQuaternion DualQuaternion::sclerp(const DualQuaternion& a,
                                          const DualQuaternion& b,
                                          const Float t) noexcept {
        // Ensure quaternions are on the same hemisphere
        DualQuaternion bAdj = b;
        if (glm::dot(a.real, b.real) < 0) {
            bAdj.real = -b.real;
            bAdj.dual = -b.dual;
        }

        // Compute difference
        const DualQuaternion diff = bAdj * a.inverse();

        // Power for interpolation
        const DualQuaternion powered = power(diff, t);

        // Apply to start
        return powered * a;
    }

    DualQuaternion DualQuaternion::dlb(const std::vector<DualQuaternion>& dqs,
                                       const std::vector<Float>& weights) noexcept {
        if (dqs.empty()) return DualQuaternion();

        DualQuaternion result;
        result.real = Quat(0, 0, 0, 0);
        result.dual = Quat(0, 0, 0, 0);

        // Ensure all quaternions are in the same hemisphere
        const DualQuaternion reference = dqs[0];

        for (std::size_t i = 0; i < dqs.size(); ++i) {
            DualQuaternion dq = dqs[i];
            Float weight = weights[i];

            // Flip if needed
            if (glm::dot(reference.real, dq.real) < 0) {
                dq.real = -dq.real;
                dq.dual = -dq.dual;
            }

            result.real += dq.real * weight;
            result.dual += dq.dual * weight;
        }

        result.normalize();
        return result;
    }

    DualQuaternion DualQuaternion::skinningBlend(
            const std::array<DualQuaternion, 4>& bones,
            const Vec4& weights) noexcept {
        DualQuaternion result;
        result.real = Quat(0, 0, 0, 0);
        result.dual = Quat(0, 0, 0, 0);

        // Reference for hemisphere check
        DualQuaternion reference = bones[0];

        // Unrolled loop for performance
        if (weights.x > 0) {
            DualQuaternion dq = bones[0];
            result.real += dq.real * weights.x;
            result.dual += dq.dual * weights.x;
        }

        if (weights.y > 0) {
            DualQuaternion dq = bones[1];
            if (glm::dot(reference.real, dq.real) < 0) {
                dq.real = -dq.real;
                dq.dual = -dq.dual;
            }
            result.real += dq.real * weights.y;
            result.dual += dq.dual * weights.y;
        }

        if (weights.z > 0) {
            DualQuaternion dq = bones[2];
            if (glm::dot(reference.real, dq.real) < 0) {
                dq.real = -dq.real;
                dq.dual = -dq.dual;
            }
            result.real += dq.real * weights.z;
            result.dual += dq.dual * weights.z;
        }

        if (weights.w > 0) {
            DualQuaternion dq = bones[3];
            if (glm::dot(reference.real, dq.real) < 0) {
                dq.real = -dq.real;
                dq.dual = -dq.dual;
            }
            result.real += dq.real * weights.w;
            result.dual += dq.dual * weights.w;
        }

        result.normalize();
        return result;
    }

    DualQuaternion DualQuaternion::power(const DualQuaternion& dq, const Float t) noexcept {
        // Extract screw parameters
        const Float angle = 2.0f * std::acos(saturate(dq.real.w));
        Vec3 axis(dq.real.x, dq.real.y, dq.real.z);

        if (glm::length2(axis) < EPSILON_SQUARED) {
            // Pure translation
            return DualQuaternion(Quat(1, 0, 0, 0), dq.getTranslation() * t);
        }

        axis = glm::normalize(axis);
        const Float pitch = -2.0f * glm::dot(Quat(dq.dual), Quat(glm::conjugate(dq.real)));

        // Interpolated values
        const Float newAngle = angle * t;
        const Float newPitch = pitch * t;

        // Reconstruct dual quaternion
        const Float halfAngle = newAngle * 0.5f;
        const Float sinHalf = std::sin(halfAngle);
        const Float cosHalf = std::cos(halfAngle);

        const Quat newReal(cosHalf, axis.x * sinHalf, axis.y * sinHalf, axis.z * sinHalf);

        // Dual part from screw motion
        const Float halfPitch = newPitch * 0.5f;
        const Quat newDual = Quat(-halfPitch * sinHalf,
                                  axis.x * halfPitch * cosHalf,
                                  axis.y * halfPitch * cosHalf,
                                  axis.z * halfPitch * cosHalf) * 0.5f;

        return DualQuaternion(newReal, newDual);
    }
} // namespace engine::math
