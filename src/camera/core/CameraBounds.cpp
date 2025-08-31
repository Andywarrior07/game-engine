//
// Created by Andres Guerrero on 23-08-25.
//

#include "CameraBounds.h"

namespace engine::camera {
    CameraBounds::CameraBounds() noexcept
        : enabled_(false), padding_(0.0f) {
    }

    CameraBounds::CameraBounds(const Vec3& min, const Vec3& max, const float padding) noexcept
        : aabb_(min, max), enabled_(true), padding_(padding) {
        validateBounds();
    }

    CameraBounds::CameraBounds(const Vec2& min2D, const Vec2& max2D, const float padding) noexcept
        : aabb_(Vec3(min2D.x, min2D.y, -1000.0f),
                Vec3(max2D.x, max2D.y, 1000.0f)),
          enabled_(true), padding_(padding) {
        validateBounds();
    }

    CameraBounds::CameraBounds(const math::AABB& aabb, const float padding) noexcept
        : aabb_(aabb), enabled_(true), padding_(padding) {
        validateBounds();
    }

    Vec3 CameraBounds::clamp(const Vec3& position) const noexcept {
        if (!enabled_) return position;

        // Apply padding to create effective bounds
        const Vec3 paddingVec(padding_);
        const Vec3 effectiveMin = aabb_.min + paddingVec;
        const Vec3 effectiveMax = aabb_.max - paddingVec;

        return math::clamp(position, effectiveMin, effectiveMax);
    }

    Vec2 CameraBounds::clamp(const Vec2& position) const noexcept {
        if (!enabled_) return position;

        const Vec3 clampedPos = clamp(Vec3(position.x, position.y, 0.0f));
        return Vec2(clampedPos.x, clampedPos.y);
    }

    bool CameraBounds::contains(const Vec3& point) const noexcept {
        if (!enabled_) return true;

        // Apply padding to create effective bounds
        const Vec3 paddingVec(padding_);
        const Vec3 effectiveMin = aabb_.min + paddingVec;
        const Vec3 effectiveMax = aabb_.max - paddingVec;

        return point.x >= effectiveMin.x && point.x <= effectiveMax.x &&
            point.y >= effectiveMin.y && point.y <= effectiveMax.y &&
            point.z >= effectiveMin.z && point.z <= effectiveMax.z;
    }

    void CameraBounds::merge(const CameraBounds& other) {
        if (!other.enabled_) return;

        if (!enabled_) {
            *this = other;
            return;
        }

        aabb_.expandToInclude(other.aabb_);
        // Take maximum padding
        padding_ = math::max(padding_, other.padding_);
    }

    CameraBounds CameraBounds::intersection(const CameraBounds& other) const {
        if (!enabled_ || !other.enabled_ || !intersects(other)) {
            return CameraBounds(); // Return disabled bounds
        }

        // Calculate intersection using component-wise min/max operations
        // For intersection: min = max(min1, min2), max = min(max1, max2)
        const Vec3 intersectionMin{
            math::max(aabb_.min.x, other.aabb_.min.x),
            math::max(aabb_.min.y, other.aabb_.min.y),
            math::max(aabb_.min.z, other.aabb_.min.z)
        };

        const Vec3 intersectionMax{
            math::min(aabb_.max.x, other.aabb_.max.x),
            math::min(aabb_.max.y, other.aabb_.max.y),
            math::min(aabb_.max.z, other.aabb_.max.z)
        };

        // Verify that we have a valid intersection
        // (this should already be guaranteed by the intersects() check above)
        if (intersectionMin.x > intersectionMax.x ||
            intersectionMin.y > intersectionMax.y ||
            intersectionMin.z > intersectionMax.z) {
            return CameraBounds(); // Return disabled bounds
        }

        // Use minimum padding from both bounds
        const float newPadding = math::min(padding_, other.padding_);
        return CameraBounds(intersectionMin, intersectionMax, newPadding);
    }

    void CameraBounds::expandToInclude(const Vec3& point) {
        if (!enabled_) {
            // Initialize bounds with the point
            aabb_ = math::AABB(point, point);
            enabled_ = true;
            return;
        }
        aabb_.expandToInclude(point);
    }

    void CameraBounds::expandToInclude(const CameraBounds& other) {
        if (!other.enabled_) return;

        if (!enabled_) {
            *this = other;
            return;
        }

        aabb_.expandToInclude(other.aabb_);
        padding_ = math::max(padding_, other.padding_);
    }

    void CameraBounds::reset() noexcept {
        aabb_ = math::AABB(); // Default constructor creates invalid AABB
        enabled_ = false;
        padding_ = 0.0f;
    }

    CameraBounds CameraBounds::transform(const Mat4& transform) const {
        if (!enabled_) return CameraBounds();

        const math::AABB transformedAABB = aabb_.transform(transform);
        return CameraBounds(transformedAABB, padding_);
    }

    math::AABB CameraBounds::getEffectiveBounds() const noexcept {
        if (!enabled_) return math::AABB();

        const Vec3 paddingVec(padding_);
        return math::AABB(aabb_.min + paddingVec, aabb_.max - paddingVec);
    }

    CameraBounds CameraBounds::fromCenterSize(const Vec3& center, const Vec3& size,
                                              const float padding) {
        const Vec3 halfSize = size * 0.5f;
        return CameraBounds(center - halfSize, center + halfSize, padding);
    }

    void CameraBounds::validateBounds() {
        // Swap components if min > max
        if (aabb_.min.x > aabb_.max.x) std::swap(aabb_.min.x, aabb_.max.x);
        if (aabb_.min.y > aabb_.max.y) std::swap(aabb_.min.y, aabb_.max.y);
        if (aabb_.min.z > aabb_.max.z) std::swap(aabb_.min.z, aabb_.max.z);

        // Ensure padding is non-negative
        padding_ = math::max(0.0f, padding_);
    }
} // namespace engine::camera
