//
// Created by Andres Guerrero on 25-08-25.
//

#include "Camera3D.h"

#include "../../math/MathSystem.h"

#include <sstream>

namespace engine::camera {
    Camera3D::Camera3D(const CameraID id, std::string name, const bool perspective)
        : BaseCamera(id, std::move(name),
                     perspective ? CameraType::CAMERA_3D_PERSPECTIVE : CameraType::CAMERA_3D_ORTHOGRAPHIC)
          , target_(math::VEC3_ZERO)
          , up_(math::VEC3_UP)
          , isPerspective_(perspective)
          , followTarget_(math::VEC3_ZERO) {
        updateFromEuler();
    }

    void Camera3D::setPosition(const Vec3& position) {
        const Vec3 constrainedPos = bounds_.clamp(position);

        if (constrainedPos == position_) {
            return;
        }

        position_ = constrainedPos;
        triggerCallback(position_, target_);
    }

    void Camera3D::update(const float deltaTime) {
        if (!enabled_) return;

        updateMovement(deltaTime);

        applyPitchConstraints();

        position_ = bounds_.clamp(position_);
    }

    void Camera3D::reset() {
        position_ = Vec3(0.0f, 0.0f, 5.0f);
        target_ = math::VEC3_ZERO;
        up_ = math::VEC3_UP;
        yaw_ = -90.0f;
        pitch_ = 0.0f;
        roll_ = 0.0f;
        followTarget_ = math::VEC3_ZERO;
        orbitalAngle_ = 0.0f;

        updateFromEuler();
    }

    std::string Camera3D::getDebugInfo() const {
        std::ostringstream oss;
        oss << BaseCamera::getDebugInfo();
        oss << " 3D[Pos=(" << position_.x << "," << position_.y << "," << position_.z << ")"
            << ", Target=(" << target_.x << "," << target_.y << "," << target_.z << ")"
            << ", FOV=" << fov_ << ", Yaw=" << yaw_ << ", Pitch=" << pitch_
            << ", Projection=" << (isPerspective_ ? "Perspective" : "Orthographic") << "]";
        return oss.str();
    }

    bool Camera3D::validate() const {
        if (!BaseCamera::validate()) return false;

        if (fov_ <= 0.0f || fov_ >= 180.0f) return false;

        if (nearPlane_ <= 0.0f || farPlane_ <= nearPlane_) return false;

        if (minPitch_ < -90.0f || maxPitch_ > 90.0f || minPitch_ > maxPitch_) return false;

        if (orthographicSize_ <= 0.0f) return false;

        if (followDistance_ < 0.0f || followSpeed_ < 0.0f) return false;

        return true;
    }

    void Camera3D::setTarget(const Vec3& target) {
        if (target == target_) {
            return;
        }

        target_ = target;
        triggerCallback(position_, target_);
    }

    void Camera3D::setFOV(const float fov) {
        fov_ = math::clamp(fov, 10.0f, 170.0f); // Reasonable FOV range
    }

    void Camera3D::setClippingPlanes(const float nearPlane, const float farPlane) {
        if (nearPlane <= 0.0f || farPlane <= nearPlane) {
            return;
        }

        nearPlane_ = nearPlane;
        farPlane_ = farPlane;
    }

    void Camera3D::setYaw(const float yaw) {
        yaw_ = yaw;

        updateFromEuler();
    }

    void Camera3D::setPitch(const float pitch) {
        pitch_ = math::clamp(pitch, minPitch_, maxPitch_);

        updateFromEuler();
    }

    void Camera3D::setRoll(const float roll) {
        roll_ = roll;

        updateFromEuler();
    }

    void Camera3D::setPitchLimits(const float minPitch, const float maxPitch) {
        minPitch_ = math::clamp(minPitch, -89.9f, 89.9f);
        maxPitch_ = math::clamp(maxPitch, minPitch_, 89.9f);

        applyPitchConstraints();
    }

    void Camera3D::setFollowTarget(const Vec3& target) {
        followTarget_ = target;
    }

    Vec2 Camera3D::worldToScreen(const Vec3& worldPos, const Viewport& viewport) const {
        const Vec3 relative = worldPos - position_;
        const Vec3 forward = getForward();
        const Vec3 right = getRight();
        const Vec3 up = getUp();

        const float x = glm::dot(relative, right);
        const float y = glm::dot(relative, up);

        // Behind camera check
        if (const float z = glm::dot(relative, forward); z <= nearPlane_) {
            return Vec2(-10000.0f, -10000.0f);
        }

        Vec2 screenPos;

        if (isPerspective_) {
            // Perspective projection
            const float fovRad = fov_ * math::DEG_TO_RAD<math::Float>;
            const float tanHalfFov = std::tan(fovRad / 0.5f);

            screenPos.x = (x / y) / tanHalfFov;
            screenPos.y = (x / y) / tanHalfFov;

            // Adjust for aspect ratio
            screenPos.x /= viewport.getAspectRatio();
        }
        else {
            // Orthographic projection
            screenPos.x = x / orthographicSize_;
            screenPos.y = y / orthographicSize_;

            // Adjust for aspect ratio
            screenPos.x /= viewport.getAspectRatio();
        }

        // Convert from normalized coordinates (-1 to 1) to screen coordinates
        screenPos.x = (screenPos.x + 1.0f) * viewport.getWidth() * 0.5f + viewport.getX();
        screenPos.y = (1.0f - screenPos.y) * viewport.getHeight() * 0.5f + viewport.getY();
        // Flip Y for screen coordinates

        return screenPos;
    }

    std::pair<Vec3, Vec3> Camera3D::screenToWorldRay(const Vec2& screenPos, const Viewport& viewport) const {
        // Convert screen coordinates to normalized coordinates (-1 to 1)
        const Vec2 normalized(
            (screenPos.x - viewport.getX()) / (viewport.getWidth() * 0.5f) - 1.0f,
            1.0f - (screenPos.y - viewport.getY()) / (viewport.getHeight() * 0.5f)
        );

        Vec3 rayDirection;

        if (isPerspective_) {
            // Perspective ray calculation
            const float fovRad = fov_ * math::DEG_TO_RAD<math::Float>;
            const float tanHalfFov = std::tan(fovRad * 0.5f);

            const Vec3 right = getRight();
            const Vec3 up = getUp();
            const Vec3 forward = getForward();

            // Calculate ray direction in world space
            rayDirection = forward +
                right * (normalized.x * tanHalfFov * viewport.getAspectRatio()) +
                up * (normalized.y * tanHalfFov);

            rayDirection = glm::normalize(rayDirection); // CAMBIO: usar glm::normalize
        }
        else {
            // Orthographic ray calculation (parallel rays)
            rayDirection = getForward();
        }

        return std::make_pair(position_, rayDirection);
    }

    Mat4 Camera3D::calculateViewMatrix() const {
        return math::MathSystem::lookAt(position_, target_, up_);
    }

    Vec3 Camera3D::screenToWorld(const Vec2& screenPos, const Viewport& viewport, const float depth) const {
        auto [rayOrigin, rayDirection] = screenToWorldRay(screenPos, viewport);

        if (isPerspective_) {
            // For perspective, depth is distance along ray
            return rayOrigin + rayDirection * depth;
        }

        // For orthographic, calculate world position at depth
        const Vec2 normalized(
            (screenPos.x - viewport.getX()) / (viewport.getWidth() * 0.5f) - 1.0f,
            1.0f - (screenPos.y - viewport.getY()) / (viewport.getHeight() * 0.5f)
        );

        const Vec3 right = getRight();
        const Vec3 up = getUp();
        const Vec3 forward = getForward();

        const Vec3 worldPos = position_ +
            right * (normalized.x * orthographicSize_ * viewport.getAspectRatio()) +
            up * (normalized.y * orthographicSize_) +
            forward * depth;

        return worldPos;
    }

    Mat4 Camera3D::calculateProjectionMatrix(const Viewport& viewport) const {
        Mat4 projMatrix;

        if (isPerspective_) {
            const float aspect = viewport.getAspectRatio();
            const float fovRad = fov_ * math::DEG_TO_RAD<math::Float>;
            projMatrix = math::MathSystem::perspective(fovRad, aspect, nearPlane_, farPlane_);
        }
        else {
            const float aspect = viewport.getAspectRatio();
            const float right = orthographicSize_ * aspect;
            const float left = -right;
            const float top = orthographicSize_;
            const float bottom = -top;
            projMatrix = math::MathSystem::ortho(left, right, bottom, top, nearPlane_, farPlane_);
        }

        return projMatrix;
    }

    Mat4 Camera3D::getViewProjectionMatrix(const Viewport& viewport) const {
        const Mat4 viewMatrix = calculateViewMatrix();
        const Mat4 projectionMatrix = calculateProjectionMatrix(viewport);

        return projectionMatrix * viewMatrix;
    }

    void Camera3D::updateMovement(const float deltaTime) {
        const Vec3 oldPosition = position_;
        const Vec3 oldTarget = target_;

        switch (mode_) {
        case CameraMode::FREE_LOOK:
            updateFreeLook(deltaTime);
            break;

        case CameraMode::FOLLOW_TARGET:
            updateFollowTarget(deltaTime);
            break;

        case CameraMode::ORBITAL:
            updateOrbital(deltaTime);
            break;

        case CameraMode::STATIC:
        default:
            // No automatic movement for static cameras
            break;
        }

        // Trigger callback if position or target changed
        if (position_ != oldPosition || target_ != oldTarget) {
            triggerCallback(position_, target_);
        }
    }

    void Camera3D::updateFreeLook(float deltaTime) {
        // For free look, the target is calculated from position + forward direction
        // The position is typically updated by external input (WASD movement)
        updateFromEuler(); // Update target based on current Euler angles
    }

    void Camera3D::updateFollowTarget(const float deltaTime) {
        // Calculate desired position behind and above target
        Vec3 desiredPosition = followTarget_;

        // Calculate offset based on current yaw and pitch
        const float yawRad = yaw_ * math::DEG_TO_RAD<math::Float>;    // CAMBIO: usar constante de MathTypes
        const float pitchRad = pitch_ * math::DEG_TO_RAD<math::Float>; // CAMBIO: usar constante de MathTypes

        const Vec3 offset(
            std::cos(pitchRad) * std::sin(yawRad) * followDistance_,
            std::sin(pitchRad) * followDistance_ + followHeight_,
            std::cos(pitchRad) * std::cos(yawRad) * followDistance_
        );

        desiredPosition += offset;

        // Smooth movement to desired position
        position_ = applySmoothing(position_, desiredPosition, deltaTime);

        // Always look at the follow target
        target_ = followTarget_;
    }

    void Camera3D::updateOrbital(const float deltaTime) {
        // Update orbital angle
        orbitalAngle_ += orbitalSpeed_ * deltaTime;

        // Keep angle in valid range
        while (orbitalAngle_ > math::TWO_PI<math::Float>) { // CAMBIO: usar constante de MathTypes
            orbitalAngle_ -= math::TWO_PI<math::Float>;
        }
        while (orbitalAngle_ < 0.0f) {
            orbitalAngle_ += math::TWO_PI<math::Float>;
        }

        // Calculate position around target
        const Vec3 offset(
            std::cos(orbitalAngle_) * orbitalRadius_,
            followHeight_,
            std::sin(orbitalAngle_) * orbitalRadius_
        );

        position_ = followTarget_ + offset;
        target_ = followTarget_;  // Always look at the center
    }

    void Camera3D::updateFromEuler() {
        const float yawRad = yaw_ * math::DEG_TO_RAD<math::Float>;    // CAMBIO: usar constante de MathTypes
        const float pitchRad = pitch_ * math::DEG_TO_RAD<math::Float>; // CAMBIO: usar constante de MathTypes

        const Vec3 direction(
            std::cos(pitchRad) * std::cos(yawRad),
            std::sin(pitchRad),
            std::cos(pitchRad) * std::sin(yawRad)
        );

        target_ = position_ + direction;
    }

    void Camera3D::applyPitchConstraints() {
        pitch_ = math::clamp(pitch_, minPitch_, maxPitch_);
    }
} // namespace engine::camera
