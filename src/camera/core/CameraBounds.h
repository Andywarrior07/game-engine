/**
 * @file CameraBounds.h
 * @brief Camera boundary constraints and spatial limits
 * @author Andrés Guerrero
 * @date 23-08-2025
 */

#pragma once

#include "CameraTypes.h"

#include "../../math/geometry/Primitives.h"
#include "../../math/core/MathFunctions.h"

namespace engine::camera {
    /**
     * @brief Camera boundary constraints for movement and position limits
     *
     * This class defines spatial boundaries that can be applied to cameras
     * to restrict their movement within a specific area. Useful for keeping
     * cameras within level bounds or specific zones.
     *
     * Built on top of the engine's AABB primitive for consistency with
     * the rest of the math system.
     */
    class CameraBounds {
    public:
        // ========================================================================
        // CONSTRUCTORS
        // ========================================================================

        /**
         * @brief Default constructor - creates unbounded camera bounds
         */
        CameraBounds() noexcept;

        /**
         * @brief Constructor with specific bounds
         * @param min Minimum boundary point
         * @param max Maximum boundary point
         * @param padding Extra padding around bounds
         */
        CameraBounds(const Vec3& min, const Vec3& max, float padding = 0.0f) noexcept;

        /**
         * @brief Constructor for 2D bounds (z is set to default range)
         * @param min2D Minimum 2D boundary point
         * @param max2D Maximum 2D boundary point
         * @param padding Extra padding around bounds
         */
        CameraBounds(const Vec2& min2D, const Vec2& max2D, float padding = 0.0f) noexcept;

        /**
         * @brief Constructor from existing AABB
         * @param aabb Axis-aligned bounding box
         * @param padding Extra padding around bounds
         */
        explicit CameraBounds(const math::AABB& aabb, float padding = 0.0f) noexcept;

        // ========================================================================
        // ACCESSORS
        // ========================================================================

        /**
         * @brief Get minimum boundary point
         * @return Minimum bounds vector
         */
        [[nodiscard]] const Vec3& getMin() const noexcept {
            return aabb_.min;
        }

        /**
         * @brief Get maximum boundary point
         * @return Maximum bounds vector
         */
        [[nodiscard]] const Vec3& getMax() const noexcept {
            return aabb_.max;
        }

        /**
         * @brief Get the underlying AABB
         * @return Reference to the internal AABB
         */
        [[nodiscard]] const math::AABB& getAABB() const noexcept {
            return aabb_;
        }

        /**
         * @brief Check if bounds are enabled
         * @return true if bounds are active
         */
        [[nodiscard]] bool isEnabled() const noexcept {
            return enabled_;
        }

        /**
         * @brief Get padding amount
         * @return Current padding value
         */
        [[nodiscard]] float getPadding() const noexcept {
            return padding_;
        }

        /**
         * @brief Get center of bounds
         * @return Center point of the bounding box
         */
        [[nodiscard]] Vec3 getCenter() const noexcept {
            return aabb_.getCenter();
        }

        /**
         * @brief Get size of bounds
         * @return Size vector (width, height, depth)
         */
        [[nodiscard]] Vec3 getSize() const noexcept {
            return aabb_.getSize();
        }

        /**
         * @brief Get half-extents of bounds (size / 2)
         * @return Half-extents vector
         */
        [[nodiscard]] Vec3 getExtents() const noexcept {
            return aabb_.getHalfExtents();
        }

        /**
         * @brief Get bounds volume
         * @return Volume of the bounding box
         */
        [[nodiscard]] float getVolume() const noexcept {
            return aabb_.getVolume();
        }

        /**
         * @brief Get surface area of bounds
         * @return Surface area of the bounding box
         */
        [[nodiscard]] float getSurfaceArea() const noexcept {
            return aabb_.getSurfaceArea();
        }

        /**
         * @brief Check if bounds are valid
         * @return true if min <= max for all axes
         */
        [[nodiscard]] bool isValid() const noexcept {
            return aabb_.isValid();
        }

        // ========================================================================
        // MUTATORS
        // ========================================================================

        /**
         * @brief Set minimum boundary point
         * @param min New minimum bounds
         */
        void setMin(const Vec3& min) {
            aabb_.min = min;
            validateBounds();
        }

        /**
         * @brief Set maximum boundary point
         * @param max New maximum bounds
         */
        void setMax(const Vec3& max) {
            aabb_.max = max;
            validateBounds();
        }

        /**
         * @brief Set both min and max bounds
         * @param min New minimum bounds
         * @param max New maximum bounds
         */
        void setBounds(const Vec3& min, const Vec3& max) {
            aabb_.min = min;
            aabb_.max = max;
            validateBounds();
        }

        /**
         * @brief Set bounds from existing AABB
         * @param aabb New AABB bounds
         */
        void setBounds(const math::AABB& aabb) {
            aabb_ = aabb;
            validateBounds();
        }

        /**
         * @brief Enable or disable bounds checking
         * @param enable Whether to enable bounds
         */
        void setEnabled(const bool enable) noexcept {
            enabled_ = enable;
        }

        /**
         * @brief Set padding amount
         * @param padding New padding value
         */
        void setPadding(const float padding) noexcept {
            padding_ = math::max(0.0f, padding);
        }

        /**
         * @brief Expand bounds by a uniform amount
         * @param amount Amount to expand in all directions
         */
        void expand(const float amount) {
            aabb_.expand(amount);
        }

        /**
         * @brief Expand bounds by a vector amount
         * @param expansion Amount to expand per axis
         */
        void expand(const Vec3& expansion) {
            aabb_.min -= expansion;
            aabb_.max += expansion;
        }

        // ========================================================================
        // OPERATIONS
        // ========================================================================

        /**
         * @brief Clamp a position to stay within bounds (including padding)
         * @param position Position to clamp
         * @return Clamped position
         */
        [[nodiscard]] Vec3 clamp(const Vec3& position) const noexcept;

        /**
         * @brief Clamp a 2D position to stay within bounds (including padding)
         * @param position 2D position to clamp
         * @return Clamped 2D position
         */
        [[nodiscard]] Vec2 clamp(const Vec2& position) const noexcept;

        /**
         * @brief Check if a point is inside the bounds (including padding)
         * @param point Point to test
         * @return true if point is within bounds
         */
        [[nodiscard]] bool contains(const Vec3& point) const noexcept;

        /**
         * @brief Check if a 2D point is inside the bounds
         * @param point 2D Point to test
         * @return true if point is within bounds
         */
        [[nodiscard]] bool contains(const Vec2& point) const noexcept {
            return contains(Vec3(point.x, point.y, 0.0f));
        }

        /**
         * @brief Check if bounds intersect with another bounds
         * @param other Other bounds to test
         * @return true if bounds overlap
         */
        [[nodiscard]] bool intersects(const CameraBounds& other) const noexcept {
            if (!enabled_ || !other.enabled_) return false;
            return aabb_.intersects(other.aabb_);
        }

        /**
         * @brief Check if bounds intersect with an AABB
         * @param aabb AABB to test intersection with
         * @return true if bounds overlap
         */
        [[nodiscard]] bool intersects(const math::AABB& aabb) const noexcept {
            if (!enabled_) return false;
            return aabb_.intersects(aabb);
        }

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
        [[nodiscard]] CameraBounds intersection(const CameraBounds& other) const;

        /**
         * @brief Expand bounds to include a point
         * @param point Point to include
         */
        void expandToInclude(const Vec3& point);

        /**
         * @brief Expand bounds to include another bounds
         * @param other Bounds to include
         */
        void expandToInclude(const CameraBounds& other);

        /**
         * @brief Get closest point on bounds to given point
         * @param point Input point
         * @return Closest point on bounds surface
         */
        [[nodiscard]] Vec3 getClosestPoint(const Vec3& point) const noexcept {
            if (!enabled_) return point;
            return aabb_.getClosestPoint(point);
        }

        /**
         * @brief Reset to unbounded state
         */
        void reset() noexcept;

        /**
         * @brief Transform bounds by matrix
         * @param transform Transformation matrix
         * @return Transformed bounds
         */
        [[nodiscard]] CameraBounds transform(const Mat4& transform) const;

        // ========================================================================
        // UTILITY METHODS
        // ========================================================================

        /**
         * @brief Get effective bounds (with padding applied)
         * @return AABB representing the effective bounds
         */
        [[nodiscard]] math::AABB getEffectiveBounds() const noexcept;

        /**
         * @brief Create bounds from center and size
         * @param center Center point
         * @param size Size in each dimension
         * @param padding Optional padding
         * @return CameraBounds instance
         */
        static CameraBounds fromCenterSize(const Vec3& center, const Vec3& size,
                                           float padding = 0.0f);

    private:
        math::AABB aabb_; ///< Underlying axis-aligned bounding box
        bool enabled_; ///< Whether bounds checking is enabled
        float padding_; ///< Extra padding around bounds

        /**
         * @brief Internal validation and correction
         * Ensures min <= max for all components
         */
        void validateBounds();
    };

    // ========================================================================
    // FREE FUNCTIONS
    // ========================================================================

    /**
     * @brief Create camera bounds from an AABB
     * @param aabb Source AABB
     * @param padding Optional padding
     * @return CameraBounds instance
     */
    inline CameraBounds makeCameraBounds(const math::AABB& aabb, const float padding = 0.0f) {
        return CameraBounds(aabb, padding);
    }

    /**
     * @brief Create 2D camera bounds
     * @param minX Minimum X coordinate
     * @param minY Minimum Y coordinate
     * @param maxX Maximum X coordinate
     * @param maxY Maximum Y coordinate
     * @param padding Optional padding
     * @return CameraBounds instance
     */
    inline CameraBounds makeCameraBounds2D(const float minX, const float minY, const float maxX, const float maxY,
                                           const float padding = 0.0f) {
        return CameraBounds(Vec2(minX, minY), Vec2(maxX, maxY), padding);
    }
} // namespace engine::camera
