/**
 * @file Transform.cpp
 * @brief Transform component for game objects
 * @author Andrés Guerrero
 * @date 27-08-2025
 *
 * Provides a complete transformation system with position, rotation, and scale.
 * Supports both local and world transformations with parent-child hierarchies.
 */

#include "Transform.h"

namespace engine::math {
    Transform::Transform() noexcept
        : position_(0, 0, 0)
          , rotation_(1, 0, 0, 0)
          , scale_(1, 1, 1)
          , localMatrix_(), worldMatrix_(), isDirty_(true)
          , isWorldDirty_(true)
          , parent_(nullptr) {
    }

    Transform::Transform(const Vec3& position,
                         const Quat& rotation,
                         const Vec3& scale) noexcept
        : position_(position)
          , rotation_(rotation)
          , scale_(scale)
          , localMatrix_(), worldMatrix_(), isDirty_(true)
          , isWorldDirty_(true)
          , parent_(nullptr) {
    }

    Vec3 Transform::getEulerAngles() const noexcept {
        return glm::eulerAngles(rotation_);
    }

    void Transform::setRotation(const Quat& rotation) noexcept {
        rotation_ = glm::normalize(rotation);
        setDirty();
    }

    void Transform::setRotation(const Float x, const Float y, const Float z, const Float w) noexcept {
        rotation_ = glm::normalize(Quat(w, x, y, z));
        setDirty();
    }

    void Transform::translateWorld(const Vec3& offset) noexcept {
        if (parent_) {
            // Convert world offset to local space
            const Mat4 parentWorldInv = glm::inverse(parent_->getWorldMatrix());
            const auto localOffset = Vec3(parentWorldInv * Vec4(offset, 0.0f));
            position_ += localOffset;
        }
        else {
            position_ += offset;
        }
        setDirty();
    }

    void Transform::rotate(const Quat& rotation) noexcept {
        rotation_ = glm::normalize(rotation * rotation_);
        setDirty();
    }

    void Transform::rotateAround(const Vec3& axis, const Float angle) noexcept {
        rotate(glm::angleAxis(angle, glm::normalize(axis)));
    }

    void Transform::lookAt(const Vec3& target, const Vec3& up) noexcept {
        const Vec3 forward = glm::normalize(target - position_);
        const Vec3 right = glm::normalize(glm::cross(forward, up));
        const Vec3 actualUp = glm::cross(right, forward);

        Mat3 rotMatrix;
        rotMatrix[0] = right;
        rotMatrix[1] = actualUp;
        // TODO: Ver que en vulkan tambien se use asi, si no usar algo tipo:
#ifdef VULKAN_COORDINATE_SYSTEM
        rotMatrix[2] = forward; // Vulkan usa +Z forward
#else
        rotMatrix[2] = -forward; // OpenGL usa -Z forward
#endif

        rotation_ = glm::normalize(glm::quat_cast(rotMatrix));
        setDirty();
    }

    Vec3 Transform::getWorldPosition() const noexcept {
        if (parent_) {
            updateWorldMatrix();
            return {worldMatrix_[3]};
        }

        return position_;
    }

    Quat Transform::getWorldRotation() const noexcept {
        if (parent_) {
            return parent_->getWorldRotation() * rotation_;
        }
        return rotation_;
    }

    Vec3 Transform::getWorldScale() const noexcept {
        if (parent_) {
            return parent_->getWorldScale() * scale_;
        }
        return scale_;
    }

    void Transform::setWorldPosition(const Vec3& worldPos) noexcept {
        if (parent_) {
            const Mat4 parentWorldInv = glm::inverse(parent_->getWorldMatrix());
            const auto localPos = Vec3(parentWorldInv * Vec4(worldPos, 1.0f));
            setPosition(localPos);
        }
        else {
            setPosition(worldPos);
        }
    }

    const Mat4& Transform::getLocalMatrix() const noexcept {
        if (isDirty_) {
            updateLocalMatrix();
        }
        return localMatrix_;
    }

    const Mat4& Transform::getWorldMatrix() const noexcept {
        if (isWorldDirty_) {
            updateWorldMatrix();
        }
        return worldMatrix_;
    }

    Mat4 Transform::getWorldMatrixInverse() const noexcept {
        return glm::inverse(getWorldMatrix());
    }

    void Transform::setParent(Transform* parent) noexcept {
        if (parent_ == parent) return;

        // Remove from old parent
        if (parent_) {
            if (const auto it = std::ranges::find(parent_->children_, this); it != parent_->children_.end()) {
                parent_->children_.erase(it);
            }
        }

        // Add to new parent
        parent_ = parent;
        if (parent_) {
            parent_->children_.push_back(this);
        }

        setWorldDirty();
    }

    void Transform::clearChildren() noexcept {
        for (auto* child : children_) {
            child->parent_ = nullptr;
            child->setWorldDirty();
        }
        children_.clear();
    }

    void Transform::lerpTo(const Transform& target, const Float t) noexcept {
        position_ = glm::mix(position_, target.position_, t);
        rotation_ = glm::slerp(rotation_, target.rotation_, t);
        scale_ = glm::mix(scale_, target.scale_, t);
        setDirty();
    }

    Transform Transform::lerp(const Transform& a, const Transform& b, const Float t) noexcept {
        Transform result;
        result.position_ = glm::mix(a.position_, b.position_, t);
        result.rotation_ = glm::slerp(a.rotation_, b.rotation_, t);
        result.scale_ = glm::mix(a.scale_, b.scale_, t);
        return result;
    }

    void Transform::reset() noexcept {
        position_ = Vec3(0);
        rotation_ = Quat(1, 0, 0, 0);
        scale_ = Vec3(1);
        setDirty();
    }

    bool Transform::decompose(const Mat4& matrix, Vec3& position, Quat& rotation, Vec3& scale) noexcept {
        // Extract position
        position = Vec3(matrix[3]);

        // Extract scale
        scale.x = glm::length(Vec3(matrix[0]));
        scale.y = glm::length(Vec3(matrix[1]));
        scale.z = glm::length(Vec3(matrix[2]));

        // Check for negative scale (reflection)
        if (glm::determinant(matrix) < 0) {
            scale.x *= -1;
        }

        // Remove scale from matrix to get rotation
        Mat3 rotMatrix;
        rotMatrix[0] = Vec3(matrix[0]) / scale.x;
        rotMatrix[1] = Vec3(matrix[1]) / scale.y;
        rotMatrix[2] = Vec3(matrix[2]) / scale.z;

        rotation = glm::normalize(glm::quat_cast(rotMatrix));

        return true;
    }

    void Transform::setWorldDirty() const noexcept {
        isWorldDirty_ = true;
        for (const auto* child : children_) {
            child->setWorldDirty();
        }
    }

    void Transform::updateLocalMatrix() const noexcept {
        const Mat4 T = glm::translate(Mat4(1.0f), position_);
        const Mat4 R = glm::mat4_cast(rotation_);
        const Mat4 S = glm::scale(Mat4(1.0f), scale_);
        localMatrix_ = T * R * S;
        isDirty_ = false;
    }

    void Transform::updateWorldMatrix() const noexcept {
        if (isDirty_) {
            updateLocalMatrix();
        }

        if (parent_) {
            worldMatrix_ = parent_->getWorldMatrix() * localMatrix_;
        }
        else {
            worldMatrix_ = localMatrix_;
        }
        isWorldDirty_ = false;
    }
} // namespace engine::math
