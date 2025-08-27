//
// Created by Andres Guerrero on 23-08-25.
//

#include "CameraBounds.h"

namespace engine::camera {
    CameraBounds::CameraBounds()
            : min_(math::constants::VEC3_MIN)
              , max_(math::constants::VEC3_MAX)
              , enabled_(false) {
    }

    CameraBounds::CameraBounds(const Vector3& min, const Vector3& max)
            : min_(min)
              , max_(max)
              , enabled_(true) {
        validateBounds();
    }

    CameraBounds::CameraBounds(const Vector2& min2D, const Vector2& max2D)
            : min_(min2D.x, min2D.y, -1000.0f)
              , max_(max2D.x, max2D.y, 1000.0f)
              , enabled_(true) {
        validateBounds();
    }

    void CameraBounds::setMin(const Vector3& min) {
        min_ = min;
        validateBounds();
    }

    void CameraBounds::setMax(const Vector3& max) {
        max_ = max;
        validateBounds();
    }

    void CameraBounds::setBounds(const Vector3& min, const Vector3& max) {
        min_ = min;
        max_ = max;
        validateBounds();
    }

    bool CameraBounds::contains(const Vector3& point) const {
        if (!enabled_) return true;

        return point.x >= min_.x && point.x <= max_.x &&
            point.y >= min_.y && point.y <= max_.y;
    }

    bool CameraBounds::intersects(const CameraBounds& other) const {
        if (!enabled_ || !other.enabled_) return false;

        return min_.x <= other.max_.x && max_.x >= other.min_.x &&
            min_.y <= other.max_.y && max_.y >= other.min_.y &&
            min_.z <= other.max_.z && max_.z >= other.min_.z;
    }

    void CameraBounds::merge(const CameraBounds& other) {
        if (!other.enabled_) return;

        if (!enabled_) {
            *this = other;
            return;
        }

        min_.x = std::min(min_.x, other.min_.x);
        min_.y = std::min(min_.y, other.min_.y);
        min_.z = std::min(min_.z, other.min_.z);

        max_.x = std::max(max_.x, other.max_.x);
        max_.y = std::max(max_.y, other.max_.y);
        max_.z = std::max(max_.z, other.max_.z);
    }

    CameraBounds CameraBounds::intersection(const CameraBounds& other) const {
        if (!enabled_ || !other.enabled_ || !intersects(other)) {
            return CameraBounds(); // Return disabled bounds
        }

        return {
                    {
                        std::max(min_.x, other.min_.x),
                        std::max(min_.y, other.min_.y),
                        std::max(min_.z, other.min_.z)
                    },
                    {
                        std::min(max_.x, other.max_.x),
                        std::min(max_.y, other.max_.y),
                        std::min(max_.z, other.max_.z)
                    }
        };
    }

    void CameraBounds::validateBounds() {
        // Ensure min is actually minimum
        if (min_.x > max_.x) std::swap(min_.x, max_.x);
        if (min_.y > max_.y) std::swap(min_.y, max_.y);
        if (min_.z > max_.z) std::swap(min_.z, max_.z);
    }
} // namespace engine::camera