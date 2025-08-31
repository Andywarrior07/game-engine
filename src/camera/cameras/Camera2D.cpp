//
// Created by Andres Guerrero on 24-08-25.
//

#include "Camera2D.h"

#include <sstream>

namespace engine::camera {
    Camera2D::Camera2D(const CameraID id, std::string name)
        : BaseCamera(id, std::move(name), CameraType::CAMERA_2D) {
    }

    void Camera2D::update(const float deltaTime) {
        if (!enabled_) return;

        updateMovement(deltaTime);

        updateZoom(deltaTime);

        updateRotation(deltaTime);

        applyZoomConstraints();

        const Vec3 pos3D(position_.x, position_.y, 0.0f);
        const Vec3 constrainedPos3D = bounds_.clamp(pos3D);

        position_ = Vec2(constrainedPos3D.x, constrainedPos3D.y);
    }

    void Camera2D::reset() {
        position_ = math::VEC2_ZERO;
        targetPosition_ = math::VEC2_ZERO;
        offset_ = math::VEC2_ZERO;
        zoom_ = 1.0f;
        targetZoom_ = 1.0f;
        rotation_ = 0.0f;
        targetRotation_ = 0.0f;
        velocity_ = math::VEC2_ZERO;
    }

    std::string Camera2D::getDebugInfo() const {
        std::ostringstream oss;

        oss << BaseCamera::getDebugInfo();
        oss << " 2D[Pos=(" << position_.x << "," << position_.y << ")"
            << ", Zoom=" << zoom_ << ", Rotation=" << rotation_
            << ", Target=(" << targetPosition_.x << "," << targetPosition_.y << ")"
            << ", FollowSpeed=" << followSpeed_ << "]";

        return oss.str();
    }

    bool Camera2D::validate() const {
        if (!BaseCamera::validate()) return false;

        // Validate zoom values
        if (zoom_ <= 0.0f || targetZoom_ <= 0.0f) return false;
        if (minZoom_ <= 0.0f || maxZoom_ <= minZoom_) return false;
        if (followSpeed_ < 0.0f) return false;

        return true;
    }

    void Camera2D::setPosition(const Vec2& position) {
        const Vec3 pos3D(position.x, position.y, 0.0f);
        const Vec3 constrainedPos3D = bounds_.clamp(pos3D);

        if (const Vec2 constrainedPos(constrainedPos3D.x, constrainedPos3D.y); constrainedPos != position_) {
            position_ = constrainedPos;
            targetPosition_ = constrainedPos;
            triggerCallback(getPosition(), math::VEC3_ZERO);
        }
    }

    void Camera2D::setZoom(const float zoom) {
        targetZoom_ = std::clamp(zoom, minZoom_, maxZoom_);
    }

    // ========================================================================
    // TARGET FOLLOWING
    // ========================================================================

    void Camera2D::setTarget(const Vec2& target) {
        const Vec3 target3D(target.x, target.y, 0.0f);
        const Vec3 constrainedTarget3D = bounds_.clamp(target3D);

        targetPosition_ = Vec2(constrainedTarget3D.x, constrainedTarget3D.y);
    }

    // ========================================================================
    // ZOOM CONFIGURATION
    // ========================================================================

    void Camera2D::setZoomLimits(const float minZoom, const float maxZoom) {
        if (minZoom > 0.0f && maxZoom > minZoom) {
            minZoom_ = minZoom;
            maxZoom_ = maxZoom;

            applyZoomConstraints();
        }
    }

    // ========================================================================
    // COORDINATE TRANSFORMATIONS
    // ========================================================================

    Vec2 Camera2D::worldToScreen(const Vec2& worldPos, const Viewport& viewport) const {
        // Apply camera transformation
        Vec2 relativePos = worldPos - position_;

        // Apply zoom
        relativePos *= zoom_;

        // Apply rotation if needed
        if (std::abs(rotation_) > 1e-3f) {
            const float radians = rotation_ * math::DEG_TO_RAD<math::Float>;
            const float cosR = std::cos(radians);
            const float sinR = std::sin(radians);

            const Vec2 rotated(
                relativePos.x * cosR - relativePos.y * sinR,
                relativePos.x * sinR + relativePos.y * cosR
            );
            relativePos = rotated;
        }

        // Apply offset and convert to screen coordinates
        Vec2 screenPos = relativePos + offset_;
        screenPos.x += viewport.getWidth() * 0.5f;
        screenPos.y += viewport.getHeight() * 0.5f;

        return screenPos;
    }

    Vec2 Camera2D::screenToWorld(const Vec2& screenPos, const Viewport& viewport) const {
        // Convert from screen coordinates to relative coordinates
        Vec2 relativePos = screenPos;
        relativePos.x -= viewport.getWidth() * 0.5f;
        relativePos.y -= viewport.getHeight() * 0.5f;
        relativePos -= offset_;

        // Apply inverse rotation
        if (std::abs(rotation_) > 1e-3f) {
            const float radians = -rotation_ * math::DEG_TO_RAD<math::Float>; // Negative for inverse
            const float cosR = std::cos(radians);
            const float sinR = std::sin(radians);

            Vec2 rotated(
                relativePos.x * cosR - relativePos.y * sinR,
                relativePos.x * sinR + relativePos.y * cosR
            );
            relativePos = rotated;
        }

        // Apply inverse zoom
        relativePos /= zoom_;

        // Add camera position
        return relativePos + position_;
    }

    CameraBounds Camera2D::getViewBounds(const Viewport& viewport) const {
        // Calculate half viewport size in world coordinates
        const float halfWidth = (viewport.getWidth() * 0.5f) / zoom_;
        const float halfHeight = (viewport.getHeight() * 0.5f) / zoom_;

        const Vec3 min3D(position_.x - halfWidth, position_.y - halfHeight, -1.0f);
        const Vec3 max3D(position_.x + halfWidth, position_.y + halfHeight, 1.0f);

        return {min3D, max3D};
    }

    bool Camera2D::isVisible(const Vec2& worldPos, const Viewport& viewport) const {
        const CameraBounds viewBounds = getViewBounds(viewport);

        return viewBounds.contains(Vec3(worldPos.x, worldPos.y, 0.0f));
    }

    // ========================================================================
    // PRIVATE METHODS
    // ========================================================================

    void Camera2D::updateMovement(float deltaTime) {
        const Vec2 oldPosition = position_;

        switch (mode_) {
        case CameraMode::FOLLOW_TARGET:
        case CameraMode::SIDE_SCROLLER: {
            // Smooth movement towards target
            const Vec2 direction = targetPosition_ - position_;

            if (const float distance = glm::length(direction); distance > 1e-3f) {
                // Don't overshoot the target
                if (const Vec2 movement = direction * followSpeed_ * deltaTime; glm::length(movement) > distance) {
                    // CAMBIO: usar glm::length
                    position_ = targetPosition_;
                }
                else {
                    position_ += movement;
                }
            }
        }
        break;

        case CameraMode::TOP_DOWN:
            // For top-down, position updates are usually direct
            position_ = applySmoothing(position_, targetPosition_, deltaTime);
            break;

        case CameraMode::STATIC:
        default:
            // No automatic movement for static cameras
            break;
        }

        // Trigger callback if position changed
        if (position_ != oldPosition) {
            triggerCallback(getPosition(), math::VEC3_ZERO);
        }
    }

    void Camera2D::updateZoom(const float deltaTime) {
        if (std::abs(zoom_ - targetZoom_) > 1e-3f) {
            zoom_ = applySmoothing(zoom_, targetZoom_, deltaTime);
        }
    }

    void Camera2D::updateRotation(const float deltaTime) {
        if (std::abs(rotation_ - targetRotation_) > 1e-3f) {
            rotation_ = applySmoothing(rotation_, targetRotation_, deltaTime);
        }
    }

    void Camera2D::applyZoomConstraints() {
        zoom_ = math::clamp(zoom_, minZoom_, maxZoom_);
        targetZoom_ = math::clamp(targetZoom_, minZoom_, maxZoom_);
    }
} // namespace engine::camera
