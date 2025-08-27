/**
 * @file CameraBounds.h
 * @brief Camera boundary constraints and spatial limits
 * @author Andrés Guerrero
 * @date 23-08-2025
 */

#pragma once

#include "CameraTypes.h"

namespace engine::camera {
    /**
     * @brief Camera boundary constraints for movement and position limits
     *
     * This class defines spatial boundaries that can be applied to cameras
     * to restrict their movement within a specific area. Useful for keeping
     * cameras within level bounds or specific zones.
     */
    class CameraBounds : public math::Bounds3D {
    public:
        /**
         * @brief Default constructor - creates unbounded camera bounds
         */
        //! Preguntar por VEC3_MIN Y MAX
        // Arreglar: CameraBounds intersection(const CameraBounds& other) const {
        CameraBounds();

        /**
         * @brief Constructor with specific bounds
         * @param min Minimum boundary point
         * @param max Maximum boundary point
         */
        CameraBounds(const Vector3& min, const Vector3& max);

        /**
         * @brief Constructor for 2D bounds (z is set to default range)
         * @param min2D Minimum 2D boundary point
         * @param max2D Maximum 2D boundary point
         */
        CameraBounds(const Vector2& min2D, const Vector2& max2D);

        // ========================================================================
        // ACCESSORS
        // ========================================================================

        /**
         * @brief Get minimum boundary point
         *  @return Minimum bounds vector
         */
        [[nodiscard]] const Vector3& getMin() const noexcept { return min_; }

        /**
         *  @brief Get maximum boundary point
         *  @return Maximum bounds vector
         */
        [[nodiscard]] const Vector3& getMax() const noexcept { return max_; }

        /**
         * @brief Check if bounds are enabled
         * @return true if bounds are active
         */
        [[nodiscard]] bool isEnabled() const noexcept { return enabled_; }

        /**
         * @brief Get center of bounds
         * @return Center point of the bounding box
         */
        [[nodiscard]] Vector3 getCenter() const {
            return (min_ + max_) * 0.5f;
        }

        /**
         * @brief Get size of bounds
         * @return Size vector (width, height, depth)
         */
        [[nodiscard]] Vector3 getSize() const {
            return max_ - min_;
        }

        /**
         * @brief Get bounds volume
         * @return Volume of the bounding box
         */
        [[nodiscard]] float getVolume() const noexcept {
            const Vector3 size = getSize();
            return size.x * size.y * size.z;
        }

        // ========================================================================
        // MUTATORS
        // ========================================================================

        /**
         * @brief Set minimum boundary point
         * @param min New minimum bounds
         */
        void setMin(const Vector3& min);

        /**
         * @brief Set maximum boundary point
         * @param max New maximum bounds
         */
        void setMax(const Vector3& max);

        /**
         * @brief Set both min and max bounds
         * @param min New minimum bounds
         * @param max New maximum bounds
         */
        void setBounds(const Vector3& min, const Vector3& max);

        /**
         * @brief Enable or disable bounds checking
         * @param enable Whether to enable bounds
         */
        void setEnabled(const bool enable) { enabled_ = enable; }

        /**
         * @brief Expand bounds by a uniform amount
         * @param amount Amount to expand in all directions
         */
        void expand(const float amount) {
            min_ -= Vector3(amount, amount, amount);
            max_ += Vector3(amount, amount, amount);
        }

        /**
         * @brief Expand bounds by a vector amount
         * @param expansion Amount to expand per axis
         */
        void expand(const Vector3& expansion) {
            min_ -= expansion;
            max_ += expansion;
        }

        // ========================================================================
        // OPERATIONS
        // ========================================================================

        /**
         * @brief Clamp a position to stay within bounds
         * @param position Position to clamp
         * @return Clamped position
         */
        [[nodiscard]] Vector3 clamp(const Vector3& position) const {
            if (!enabled_) return position;

            return {
                math::clamp(position.x, min_.x, max_.x),
                math::clamp(position.y, min_.y, max_.y),
                math::clamp(position.z, min_.z, max_.z)
            };
        }

        /**
         * @brief Clamp a 2D position to stay within bounds
         * @param position 2D position to clamp
         * @return Clamped 2D position
         */
        [[nodiscard]] Vector2 clamp(const Vector2& position) const {
            if (!enabled_) return position;

            return {
                math::clamp(position.x, min_.x, max_.x),
                math::clamp(position.y, min_.y, max_.y)
            };
        }

        /**
         * @brief Check if a 2D point is inside the bounds
         * @param point 2D Point to test
         * @return true if point is within bounds
         */
        [[nodiscard]] bool contains(const Vector3& point) const;

        /**
         * @brief Check if bounds intersect with another bounds
         * @param other Other bound to test
         * @return true if bounds overlap
         */
        [[nodiscard]] bool intersects(const CameraBounds& other) const;

        /**
         * @brief Merge with another bounds
         * @param other Bounds to merge with
        */
        void merge(const CameraBounds& other);

        /**
         * @brief Calculate intersection with another bounds
         * @param other Bounds to intersect with
         * @return Intersection bounds (disabled if no intersection)
        */
        CameraBounds intersection(const CameraBounds& other) const;

        /**
         * @brief Reset to unbounded state
         */
        void reset() {
            min_ = math::constants::VEC3_MIN;
            max_ = math::constants::VEC3_MAX;
            enabled_ = false;
        }

    private:
        Vector3 min_; ///< Minimum boundary point
        Vector3 max_; ///< Maximum boundary point
        bool enabled_; ///< Whether bound checking is enabled

        /**
         * @brief Internal validation and correction
         */
        void validateBounds();
    };
} // namespace engine::camera
