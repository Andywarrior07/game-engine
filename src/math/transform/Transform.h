/**
 * @file Transform.h
 * @brief Transform component for game objects
 * @author Andrés Guerrero
 * @date 27-08-2025
 *
 * Provides a complete transformation system with position, rotation, and scale.
 * Supports both local and world transformations with parent-child hierarchies.
 */


/*
 * TODO: Este motor aplica transformaciones en el orden TRS (escala local primero, luego rotación, luego traslación en espacio local)
 * Revisar como sería para Vulkan para no tener problemas de calculos. Ej: T * R * S
 * TODO: no gestionas ownership → cuidado con dangling pointers si alguien destruye un Transform padre sin limpiar hijos.
 * Transform* parent_ y std::vector<Transform*> children_.
 */

#pragma once

#include "../core/MathTypes.h"
#include "../core/MathConstants.h"
#include "../core/MathFunctions.h"

#include <vector>

namespace engine::math {
    /**
     * @brief Transform component representing position, rotation, and scale
     *
     * This class manages both local and world transformations, supporting
     * parent-child hierarchies for scene graphs. Transformations are cached
     * and only recalculated when dirty.
     */
    class Transform {
    public:
        // ============================================================================
        // Constructors and Initialization
        // ============================================================================

        // TODO: Preguntar por la inicializacion variables
        Transform() noexcept;

        // TODO: Preguntar por la inicializacion variables
        explicit Transform(const Vec3& position,
                           const Quat& rotation = QUAT_IDENTITY,
                           const Vec3& scale = VEC3_ONE) noexcept;

        // ============================================================================
        // Local Transform Getters
        // ============================================================================

        [[nodiscard]] const Vec3& getPosition() const noexcept { return position_; }
        [[nodiscard]] const Quat& getRotation() const noexcept { return rotation_; }
        [[nodiscard]] const Vec3& getScale() const noexcept { return scale_; }

        [[nodiscard]] Vec3 getEulerAngles() const noexcept;

        // ============================================================================
        // Local Transform Setters
        // ============================================================================

        void setPosition(const Vec3& position) noexcept {
            position_ = position;
            setDirty();
        }

        void setPosition(const Float x, const Float y, const Float z) noexcept {
            position_ = Vec3(x, y, z);
            setDirty();
        }

        void setRotation(const Quat& rotation) noexcept;

        void setRotation(Float x, Float y, Float z, Float w) noexcept;

        void setEulerAngles(const Vec3& eulerAngles) noexcept {
            rotation_ = Quat(eulerAngles);
            setDirty();
        }

        void setEulerAngles(const Float pitch, const Float yaw, const Float roll) noexcept {
            rotation_ = Quat(Vec3(pitch, yaw, roll));
            setDirty();
        }

        void setScale(const Vec3& scale) noexcept {
            scale_ = scale;
            setDirty();
        }

        void setScale(const Float scale) noexcept {
            scale_ = Vec3(scale);
            setDirty();
        }

        void setScale(const Float x, const Float y, const Float z) noexcept {
            scale_ = Vec3(x, y, z);
            setDirty();
        }

        // ============================================================================
        // Transform Operations
        // ============================================================================

        /**
         * @brief Translate by given offset in local space
         */
        void translate(const Vec3& offset) noexcept {
            position_ += offset;
            setDirty();
        }

        /**
         * @brief Translate by given offset in world space
         */
        void translateWorld(const Vec3& offset) noexcept;

        /**
         * @brief Rotate by given quaternion in local space
         */
        void rotate(const Quat& rotation) noexcept;

        /**
         * @brief Rotate by euler angles in local space
         */
        void rotate(const Float pitch, const Float yaw, const Float roll) noexcept {
            rotate(Quat(Vec3(pitch, yaw, roll)));
        }

        /**
         * @brief Rotate around axis by angle in local space
         */
        void rotateAround(const Vec3& axis, Float angle) noexcept;

        /**
         * @brief Scale by given factors
         */
        void scaleBy(const Vec3& factors) noexcept {
            scale_ *= factors;
            setDirty();
        }

        void scaleBy(const Float factor) noexcept {
            scale_ *= factor;
            setDirty();
        }

        // ============================================================================
        // Direction Vectors
        // ============================================================================

        [[nodiscard]] Vec3 getForward() const noexcept {
            return rotation_ * VEC3_FORWARD;
        }

        [[nodiscard]] Vec3 getRight() const noexcept {
            return rotation_ * VEC3_RIGHT;
        }

        [[nodiscard]] Vec3 getUp() const noexcept {
            return rotation_ * VEC3_UP;
        }

        [[nodiscard]] Vec3 getWorldForward() const noexcept {
            return getWorldRotation() * VEC3_FORWARD;
        }

        [[nodiscard]] Vec3 getWorldRight() const noexcept {
            return getWorldRotation() * VEC3_RIGHT;
        }

        [[nodiscard]] Vec3 getWorldUp() const noexcept {
            return getWorldRotation() * VEC3_UP;
        }

        // ============================================================================
        // Look At Functions
        // ============================================================================

        /**
         * @brief Orient transform to look at target position
         */
        void lookAt(const Vec3& target, const Vec3& up = VEC3_UP) noexcept;

        /**
         * @brief Orient transform to look in a direction
         */
        void lookInDirection(const Vec3& direction, const Vec3& up = VEC3_UP) noexcept {
            lookAt(position_ + direction, up);
        }

        // ============================================================================
        // World Transform
        // ============================================================================

        [[nodiscard]] Vec3 getWorldPosition() const noexcept;

        [[nodiscard]] Quat getWorldRotation() const noexcept;

        [[nodiscard]] Vec3 getWorldScale() const noexcept;

        void setWorldPosition(const Vec3& worldPos) noexcept;

        // ============================================================================
        // Matrix Generation
        // ============================================================================

        /**
         * @brief Get local transformation matrix
         */
        [[nodiscard]] const Mat4& getLocalMatrix() const noexcept;

        /**
         * @brief Get world transformation matrix
         */
        [[nodiscard]] const Mat4& getWorldMatrix() const noexcept;

        /**
         * @brief Get inverse world matrix
         */
        [[nodiscard]] Mat4 getWorldMatrixInverse() const noexcept;

        // ============================================================================
        // Hierarchy Management
        // ============================================================================

        void setParent(Transform* parent) noexcept;

        [[nodiscard]] Transform* getParent() const noexcept {
            return parent_;
        }

        [[nodiscard]] const std::vector<Transform*>& getChildren() const noexcept {
            return children_;
        }

        [[nodiscard]] std::size_t getChildCount() const noexcept {
            return children_.size();
        }

        [[nodiscard]] Transform* getChild(const std::size_t index) const noexcept {
            return index < children_.size() ? children_[index] : nullptr;
        }

        /**
         * @brief Remove all children
         */
        void clearChildren() noexcept;

        // ============================================================================
        // Space Conversions
        // ============================================================================

        /**
         * @brief Transform point from local to world space
         */
        [[nodiscard]] Vec3 transformPoint(const Vec3& localPoint) const noexcept {
            return Vec3(getWorldMatrix() * Vec4(localPoint, 1.0f));
        }

        /**
         * @brief Transform direction from local to world space
         */
        [[nodiscard]] Vec3 transformDirection(const Vec3& localDirection) const noexcept {
            return Vec3(getWorldMatrix() * Vec4(localDirection, 0.0f));
        }

        /**
         * @brief Transform point from world to local space
         */
        [[nodiscard]] Vec3 inverseTransformPoint(const Vec3& worldPoint) const noexcept {
            return Vec3(getWorldMatrixInverse() * Vec4(worldPoint, 1.0f));
        }

        /**
         * @brief Transform direction from world to local space
         */
        [[nodiscard]] Vec3 inverseTransformDirection(const Vec3& worldDirection) const noexcept {
            return Vec3(getWorldMatrixInverse() * Vec4(worldDirection, 0.0f));
        }

        // ============================================================================
        // Interpolation
        // ============================================================================

        /**
         * @brief Linearly interpolate to target transform
         */
        void lerpTo(const Transform& target, Float t) noexcept;

        /**
         * @brief Create interpolated transform between two transforms
         */
        [[nodiscard]] static Transform lerp(const Transform& a, const Transform& b, Float t) noexcept;

        // ============================================================================
        // Utility Functions
        // ============================================================================

        /**
         * @brief Reset transform to identity
         */
        void reset() noexcept;

        /**
         * @brief Check if transform has uniform scale
         */
        [[nodiscard]] bool hasUniformScale() const noexcept {
            return math::isNearlyEqual<float>(scale_.x, scale_.y) &&
                math::isNearlyEqual<float>(scale_.y, scale_.z);
        }

        /**
         * @brief Decompose a matrix into transform components
         */
        static bool decompose(const Mat4& matrix, Vec3& position, Quat& rotation, Vec3& scale) noexcept;

        /**
         * @brief Set transform from matrix
         */
        void setFromMatrix(const Mat4& matrix) noexcept {
            decompose(matrix, position_, rotation_, scale_);
            setDirty();
        }

    private:
        // Local transform components
        Vec3 position_;
        Quat rotation_;
        Vec3 scale_;

        // Cached matrices
        mutable Mat4 localMatrix_;
        mutable Mat4 worldMatrix_;
        mutable bool isDirty_;
        mutable bool isWorldDirty_;

        // Hierarchy
        Transform* parent_;
        std::vector<Transform*> children_;

        /**
         * @brief Mark matrices as dirty
         */
        void setDirty() const noexcept {
            isDirty_ = true;
            setWorldDirty();
        }

        /**
         * @brief Mark world matrix as dirty (propagates to children)
         */
        void setWorldDirty() const noexcept;

        /**
         * @brief Update cached local matrix
         */
        void updateLocalMatrix() const noexcept;

        /**
         * @brief Update cached world matrix
         */
        void updateWorldMatrix() const noexcept;
    };
} // namespace engine::math
