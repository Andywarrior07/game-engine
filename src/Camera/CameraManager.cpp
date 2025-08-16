//
// Created by Andres Guerrero on 09-08-25.
//

#include "CameraManager.h"
#include <iostream>         // For logging and debug output
#include <algorithm>        // For std::clamp, std::sort, etc.
#include <cassert>          // For debug assertions
#include <cmath>            // For mathematical operations
#include <sstream>          // For string stream operations
#include <random>           // For shake effect random numbers


namespace engine::camera {
    // ========================================================================
    // BASE CAMERA IMPLEMENTATION
    // ========================================================================

    /**
     * @brief Constructor for base camera
     * CHANGE: Initialize common camera properties
     */
    BaseCamera::BaseCamera(CameraID id, std::string name, CameraType type)
        : id_(id) // Store unique identifier
          , name_(std::move(name)) // Move name to avoid copy
          , type_(type) // Camera type
          , mode_(CameraMode::STATIC) // Default to static mode
          , active_(false) // Not active by default
          , enabled_(true) // Enabled by default
          , smoothingSpeed_(5.0f) // Default smoothing speed
    {
        // Constructor body empty - all initialization in member list
    }

    /**
     * @brief Apply camera bounds to position
     * CHANGE: Safe bounds application with validation
  */
    Vector3 BaseCamera::applyBounds(const Vector3& position) const {
        return bounds_.clamp(position);
    }

    /**
     * @brief Trigger position change callback
     * CHANGE: Safe callback execution with null checking
     */
    void BaseCamera::triggerCallback(const Vector3& position, const Vector3& target) const {
        if (callback_) {
            callback_(id_, position, target);
        }
    }

    /**
     * @brief Apply smoothing to scalar value
     * CHANGE: Frame-rate independent smoothing
     */
    float BaseCamera::applySmoothing(float current, float target, float deltaTime) const {
        if (smoothingSpeed_ <= 0.0f) return target;

        float lerpFactor = 1.0f - std::exp(-smoothingSpeed_ * deltaTime);
        return current + (target - current) * lerpFactor;
    }

    /**
     * @brief Apply smoothing to 2D vector
     * CHANGE: Vector smoothing with frame-rate independence
     */
    Vector2 BaseCamera::applySmoothing(const Vector2& current, const Vector2& target, float deltaTime) const {
        if (smoothingSpeed_ <= 0.0f) return target;

        float lerpFactor = 1.0f - std::exp(-smoothingSpeed_ * deltaTime);
        return current + (target - current) * lerpFactor;
    }

    /**
     * @brief Apply smoothing to 3D vector
     * CHANGE: 3D vector smoothing
     */
    Vector3 BaseCamera::applySmoothing(const Vector3& current, const Vector3& target, float deltaTime) const {
        if (smoothingSpeed_ <= 0.0f) return target;

        float lerpFactor = 1.0f - std::exp(-smoothingSpeed_ * deltaTime);
        return current + (target - current) * lerpFactor;
    }

    /**
     * @brief Get debug information for base camera
     * CHANGE: Comprehensive debug output
     */
    std::string BaseCamera::getDebugInfo() const {
        std::ostringstream oss;
        oss << "Camera[ID=" << id_ << ", Name=" << name_ << ", Type=";

        switch (type_) {
        case CameraType::CAMERA_2D: oss << "2D";
            break;
        case CameraType::CAMERA_3D_PERSPECTIVE: oss << "3D_Perspective";
            break;
        case CameraType::CAMERA_3D_ORTHOGRAPHIC: oss << "3D_Orthographic";
            break;
        }

        oss << ", Mode=";
        switch (mode_) {
        case CameraMode::STATIC: oss << "Static";
            break;
        case CameraMode::FREE_LOOK: oss << "FreeLook";
            break;
        case CameraMode::FOLLOW_TARGET: oss << "FollowTarget";
            break;
        case CameraMode::ORBITAL: oss << "Orbital";
            break;
        case CameraMode::SIDE_SCROLLER: oss << "SideScroller";
            break;
        case CameraMode::TOP_DOWN: oss << "TopDown";
            break;
        case CameraMode::ISOMETRIC: oss << "Isometric";
            break;
        case CameraMode::CINEMATIC: oss << "Cinematic";
            break;
        }

        oss << ", Active=" << (active_ ? "Yes" : "No")
            << ", Enabled=" << (enabled_ ? "Yes" : "No")
            << ", Smoothing=" << smoothingSpeed_ << "]";

        return oss.str();
    }

    /**
     * @brief Validate base camera state
     * CHANGE: Basic validation for all cameras
     */
    bool BaseCamera::validate() const {
        // Check ID validity
        if (id_ == INVALID_CAMERA_ID) {
            return false;
        }

        // Check smoothing speed
        if (smoothingSpeed_ < 0.0f) {
            return false;
        }

        // Validate bounds if enabled
        if (bounds_.enabled) {
            // CAMBIO: Usar GLM vector comparisons
            if (bounds_.min.x > bounds_.max.x ||
                bounds_.min.y > bounds_.max.y ||
                bounds_.min.z > bounds_.max.z) {
                return false;
            }
        }

        return true;
    }

    // ========================================================================
    // CAMERA2D IMPLEMENTATION
    // ========================================================================

    /**
     * @brief Constructor for 2D camera
     * CHANGE: Initialize 2D-specific properties
     */
    Camera2D::Camera2D(CameraID id, std::string name)
        : BaseCamera(id, std::move(name), CameraType::CAMERA_2D) // Call base constructor
          , position_(math::constants::VEC2_ZERO) // CAMBIO: usar constantes de MathTypes
          , targetPosition_(math::constants::VEC2_ZERO) // CAMBIO: usar constantes de MathTypes
          , offset_(math::constants::VEC2_ZERO) // CAMBIO: usar constantes de MathTypes
          , zoom_(1.0f) // Normal zoom
          , targetZoom_(1.0f) // Target zoom same as current
          , rotation_(0.0f) // No rotation
          , targetRotation_(0.0f) // Target rotation
          , followSpeed_(5.0f) // Default follow speed
          , velocity_(math::constants::VEC2_ZERO) // CAMBIO: usar constantes de MathTypes
          , minZoom_(0.1f) // Minimum zoom
          , maxZoom_(10.0f) // Maximum zoom
    {
        // Constructor body empty - all initialization in member list
    }

    /**
     * @brief Set 2D camera position
     * CHANGE: Apply bounds and trigger callbacks
     */
    void Camera2D::setPosition(const Vector2& position) {
        // CAMBIO: Convertir a Vector3 para usar bounds_.clamp(), luego de vuelta a Vector2
        Vector3 pos3D(position.x, position.y, 0.0f);
        Vector3 constrainedPos3D = bounds_.clamp(pos3D);
        Vector2 constrainedPos(constrainedPos3D.x, constrainedPos3D.y);

        if (constrainedPos != position_) {
            position_ = constrainedPos;
            targetPosition_ = constrainedPos; // Update target too
            triggerCallback(getPosition(), math::constants::VEC3_ZERO);
        }
    }

    /**
     * @brief Set 2D camera zoom
     * CHANGE: Apply zoom constraints and smooth zoom
     */
    void Camera2D::setZoom(float zoom) {
        targetZoom_ = std::clamp(zoom, minZoom_, maxZoom_);
    }

    /**
     * @brief Set target for following modes
     * CHANGE: Update target for smooth following
     */
    void Camera2D::setTarget(const Vector2& target) {
        const Vector3 target3D(target.x, target.y, 0.0f);
        const Vector3 constrainedTarget3D = bounds_.clamp(target3D);
        targetPosition_ = Vector2(constrainedTarget3D.x, constrainedTarget3D.y);
    }

    /**
     * @brief Set zoom limits
     * CHANGE: Validate and apply zoom constraints
     */
    void Camera2D::setZoomLimits(float minZoom, float maxZoom) {
        if (minZoom > 0.0f && maxZoom > minZoom) {
            minZoom_ = minZoom;
            maxZoom_ = maxZoom;

            // Apply constraints to current zoom
            applyZoomConstraints();
        }
    }

    /**
     * @brief Convert world position to screen coordinates
     * CHANGE: Full 2D transformation pipeline
     */
    Vector2 Camera2D::worldToScreen(const Vector2& worldPos, const Viewport& viewport) const {
        // Apply camera transformation
        Vector2 relativePos = worldPos - position_;

        // Apply zoom
        relativePos *= zoom_;

        // Apply rotation if needed
        if (std::abs(rotation_) > 1e-3f) {
            float radians = rotation_ * math::constants::DEG_TO_RAD; // CAMBIO: usar constante de MathTypes
            float cosR = std::cos(radians);
            float sinR = std::sin(radians);

            Vector2 rotated(
                relativePos.x * cosR - relativePos.y * sinR,
                relativePos.x * sinR + relativePos.y * cosR
            );
            relativePos = rotated;
        }

        // Apply offset and convert to screen coordinates
        Vector2 screenPos = relativePos + offset_;
        screenPos.x += viewport.width * 0.5f;
        screenPos.y += viewport.height * 0.5f;

        return screenPos;
    }

    /**
     * @brief Convert screen coordinates to world position
     * CHANGE: Inverse 2D transformation pipeline
     */
    Vector2 Camera2D::screenToWorld(const Vector2& screenPos, const Viewport& viewport) const {
        // Convert from screen coordinates to relative coordinates
        Vector2 relativePos = screenPos;
        relativePos.x -= viewport.width * 0.5f;
        relativePos.y -= viewport.height * 0.5f;
        relativePos -= offset_;

        // Apply inverse rotation
        if (std::abs(rotation_) > 1e-3f) {
            float radians = -rotation_ * math::constants::DEG_TO_RAD; // Negative for inverse
            float cosR = std::cos(radians);
            float sinR = std::sin(radians);

            Vector2 rotated(
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

    /**
     * @brief Get camera view bounds in world space
     * CHANGE: Calculate visible area bounds
     */
    CameraBounds Camera2D::getViewBounds(const Viewport& viewport) const {
        // Calculate half viewport size in world coordinates
        float halfWidth = (viewport.width * 0.5f) / zoom_;
        float halfHeight = (viewport.height * 0.5f) / zoom_;

        const Vector3 min3D(position_.x - halfWidth, position_.y - halfHeight, -1.0f);
        const Vector3 max3D(position_.x + halfWidth, position_.y + halfHeight, 1.0f);

        return CameraBounds(min3D, max3D);
    }

    /**
     * @brief Update 2D camera state
     * CHANGE: Comprehensive 2D camera update
     */
    void Camera2D::update(float deltaTime) {
        if (!enabled_) return;

        // Update movement based on mode
        updateMovement(deltaTime);

        // Update zoom smoothing
        updateZoom(deltaTime);

        // Update rotation smoothing
        updateRotation(deltaTime);

        // Apply all constraints
        applyZoomConstraints();

        // Apply bounds to final position
        Vector3 pos3D(position_.x, position_.y, 0.0f);
        Vector3 constrainedPos3D = bounds_.clamp(pos3D);
        position_ = Vector2(constrainedPos3D.x, constrainedPos3D.y);
    }

    /**
     * @brief Reset 2D camera to default state
     * CHANGE: Reset all 2D camera properties
     */
    void Camera2D::reset() {
        position_ = math::constants::VEC2_ZERO;
        targetPosition_ = math::constants::VEC2_ZERO;
        offset_ = math::constants::VEC2_ZERO;
        zoom_ = 1.0f;
        targetZoom_ = 1.0f;
        rotation_ = 0.0f;
        targetRotation_ = 0.0f;
        velocity_ = math::constants::VEC2_ZERO;
    }

    /**
     * @brief Get 2D camera debug information
     * CHANGE: Detailed 2D camera debug output
     */
    std::string Camera2D::getDebugInfo() const {
        std::ostringstream oss;
        oss << BaseCamera::getDebugInfo();
        oss << " 2D[Pos=(" << position_.x << "," << position_.y << ")"
            << ", Zoom=" << zoom_ << ", Rotation=" << rotation_
            << ", Target=(" << targetPosition_.x << "," << targetPosition_.y << ")"
            << ", FollowSpeed=" << followSpeed_ << "]";
        return oss.str();
    }

    /**
     * @brief Validate 2D camera state
     * CHANGE: 2D-specific validation
     */
    bool Camera2D::validate() const {
        if (!BaseCamera::validate()) return false;

        // Validate zoom values
        if (zoom_ <= 0.0f || targetZoom_ <= 0.0f) return false;
        if (minZoom_ <= 0.0f || maxZoom_ <= minZoom_) return false;
        if (followSpeed_ < 0.0f) return false;

        return true;
    }

    /**
     * @brief Update 2D camera movement based on mode
     * CHANGE: Mode-specific movement implementation
     */
    void Camera2D::updateMovement(float deltaTime) {
        Vector2 oldPosition = position_;

        switch (mode_) {
        case CameraMode::FOLLOW_TARGET:
        case CameraMode::SIDE_SCROLLER: {
            // Smooth movement towards target
            Vector2 direction = targetPosition_ - position_;
            float distance = glm::length(direction); // CAMBIO: usar glm::length

            if (distance > 1e-3f) {
                Vector2 movement = direction * followSpeed_ * deltaTime;

                // Don't overshoot the target
                if (glm::length(movement) > distance) {
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
            triggerCallback(getPosition(), math::constants::VEC3_ZERO);
        }
    }

    /**
     * @brief Update zoom smoothing
     * CHANGE: Smooth zoom transitions
     */
    void Camera2D::updateZoom(float deltaTime) {
        if (std::abs(zoom_ - targetZoom_) > 1e-3f) {
            zoom_ = applySmoothing(zoom_, targetZoom_, deltaTime);
        }
    }

    /**
     * @brief Update rotation smoothing
     * CHANGE: Smooth rotation transitions
     */
    void Camera2D::updateRotation(float deltaTime) {
        if (std::abs(rotation_ - targetRotation_) > 1e-3f) {
            rotation_ = applySmoothing(rotation_, targetRotation_, deltaTime);
        }
    }

    /**
     * @brief Apply zoom constraints
     * CHANGE: Ensure zoom stays within valid range
     */
    void Camera2D::applyZoomConstraints() {
        zoom_ = math::clamp(zoom_, minZoom_, maxZoom_);
        targetZoom_ = math::clamp(targetZoom_, minZoom_, maxZoom_);
    }

    // ========================================================================
    // CAMERA3D IMPLEMENTATION
    // ========================================================================

    /**
     * @brief Constructor for 3D camera
     * CHANGE: Initialize 3D-specific properties with sensible defaults
     */
    Camera3D::Camera3D(CameraID id, std::string name, bool perspective)
        : BaseCamera(id, std::move(name),
                     perspective ? CameraType::CAMERA_3D_PERSPECTIVE : CameraType::CAMERA_3D_ORTHOGRAPHIC)
          , position_(0.0f, 0.0f, 5.0f) // Start 5 units back from origin
          , target_(math::constants::VEC3_ZERO) // CAMBIO: usar constante de MathTypes
          , up_(math::constants::VEC3_UP) // CAMBIO: usar constante de MathTypes
          , fov_(60.0f) // Common 60-degree FOV
          , nearPlane_(0.1f) // Close near plane
          , farPlane_(1000.0f) // Distant far plane
          , isPerspective_(perspective) // Store projection type
          , orthographicSize_(10.0f) // Default ortho size
          , yaw_(-90.0f) // Face forward (-Z)
          , pitch_(0.0f) // Level horizon
          , roll_(0.0f) // No roll
          , minPitch_(-89.0f) // Prevent gimbal lock
          , maxPitch_(89.0f) // Prevent gimbal lock
          , followTarget_(math::constants::VEC3_ZERO) // CAMBIO: usar constante de MathTypes
          , followDistance_(5.0f) // Default follow distance
          , followHeight_(2.0f) // Default height offset
          , followSpeed_(5.0f) // Default follow speed
          , orbitalRadius_(5.0f) // Default orbital radius
          , orbitalAngle_(0.0f) // Start at 0 degrees
          , orbitalSpeed_(1.0f) // Default orbital speed
    {
        // Update target based on initial Euler angles
        updateFromEuler();
    }

    /**
     * @brief Set 3D camera position
     * CHANGE: Apply bounds and trigger callbacks
     */
    void Camera3D::setPosition(const Vector3& position) {
        Vector3 constrainedPos = bounds_.clamp(position);

        if (constrainedPos != position_) {
            position_ = constrainedPos;
            triggerCallback(position_, target_);
        }
    }

    /**
     * @brief Set camera target (look-at point)
     * CHANGE: Update target and trigger callbacks
     */
    void Camera3D::setTarget(const Vector3& target) {
        if (target != target_) {
            target_ = target;
            triggerCallback(position_, target_);
        }
    }

    /**
     * @brief Set field of view
     * CHANGE: Clamp FOV to reasonable range
     */
    void Camera3D::setFOV(float fov) {
        fov_ = math::clamp(fov, 10.0f, 170.0f); // Reasonable FOV range
    }

    /**
     * @brief Set near and far clipping planes
     * CHANGE: Validate clipping plane relationship
     */
    void Camera3D::setClippingPlanes(const float nearPlane, const float farPlane) {
        if (nearPlane > 0.0f && farPlane > nearPlane) {
            nearPlane_ = nearPlane;
            farPlane_ = farPlane;
        }
    }

    /**
     * @brief Set pitch with constraints
     * CHANGE: Apply pitch limits to prevent gimbal lock
     */
    void Camera3D::setPitch(float pitch) {
        pitch_ = math::clamp(pitch, minPitch_, maxPitch_);
        updateFromEuler();
    }

    /**
     * @brief Set pitch constraints
     * CHANGE: Update pitch limits and apply to current pitch
     */
    void Camera3D::setPitchLimits(float minPitch, float maxPitch) {
        minPitch_ = math::clamp(minPitch, -89.9f, 89.9f);
        maxPitch_ = math::clamp(maxPitch, minPitch_, 89.9f);
        applyPitchConstraints();
    }

    /**
     * @brief Set follow target
     * CHANGE: Update follow target for third-person cameras
     */
    void Camera3D::setFollowTarget(const Vector3& target) {
        followTarget_ = target;
    }

    /**
     * @brief Convert world position to screen coordinates
     * CHANGE: Full 3D projection pipeline with proper matrix math
     */
    Vector2 Camera3D::worldToScreen(const Vector3& worldPos, const Viewport& viewport) const {
        // This is a simplified projection - in a real engine you'd use proper matrices
        Vector3 relative = worldPos - position_;
        Vector3 forward = getForward();
        Vector3 right = getRight();
        Vector3 up = getUp();

        // Transform to camera space
        float x = glm::dot(relative, right);   // CAMBIO: usar glm::dot
        float y = glm::dot(relative, up);      // CAMBIO: usar glm::dot
        float z = glm::dot(relative, forward); // CAMBIO: usar glm::dot

        // Behind camera check
        if (z <= nearPlane_) {
            return Vector2(-10000.0f, -10000.0f);  // Behind camera
        }

        Vector2 screenPos;

        if (isPerspective_) {
            // Perspective projection
            float fovRad = fov_ * math::constants::DEG_TO_RAD; // CAMBIO: usar constante de MathTypes
            float tanHalfFov = std::tan(fovRad * 0.5f);

            screenPos.x = (x / z) / tanHalfFov;
            screenPos.y = (y / z) / tanHalfFov;

            // Adjust for aspect ratio
            screenPos.x /= viewport.getAspectRatio();
        } else {
            // Orthographic projection
            screenPos.x = x / orthographicSize_;
            screenPos.y = y / orthographicSize_;

            // Adjust for aspect ratio
            screenPos.x /= viewport.getAspectRatio();
        }

        // Convert from normalized coordinates (-1 to 1) to screen coordinates
        screenPos.x = (screenPos.x + 1.0f) * viewport.width * 0.5f + viewport.x;
        screenPos.y = (1.0f - screenPos.y) * viewport.height * 0.5f + viewport.y;  // Flip Y for screen coordinates

        return screenPos;
    }

    /**
     * @brief Convert screen coordinates to world ray
     * CHANGE: Generate ray for 3D picking and interaction
     */
    std::pair<Vector3, Vector3> Camera3D::screenToWorldRay(const Vector2& screenPos, const Viewport& viewport) const {
        // Convert screen coordinates to normalized coordinates (-1 to 1)
        Vector2 normalized(
            (screenPos.x - viewport.x) / (viewport.width * 0.5f) - 1.0f,
            1.0f - (screenPos.y - viewport.y) / (viewport.height * 0.5f)
        );

        Vector3 rayDirection;

        if (isPerspective_) {
            // Perspective ray calculation
            float fovRad = fov_ * math::constants::DEG_TO_RAD; // CAMBIO: usar constante de MathTypes
            float tanHalfFov = std::tan(fovRad * 0.5f);

            Vector3 right = getRight();
            Vector3 up = getUp();
            Vector3 forward = getForward();

            // Calculate ray direction in world space
            rayDirection = forward +
                          right * (normalized.x * tanHalfFov * viewport.getAspectRatio()) +
                          up * (normalized.y * tanHalfFov);

            rayDirection = glm::normalize(rayDirection); // CAMBIO: usar glm::normalize
        } else {
            // Orthographic ray calculation (parallel rays)
            rayDirection = getForward();
        }

        return std::make_pair(position_, rayDirection);
    }

    /**
     * @brief Convert screen coordinates to world position at given depth
     * CHANGE: Project screen point to world space at specific depth
     */
    Vector3 Camera3D::screenToWorld(const Vector2& screenPos, const Viewport& viewport, float depth) const {
        auto [rayOrigin, rayDirection] = screenToWorldRay(screenPos, viewport);

        if (isPerspective_) {
            // For perspective, depth is distance along ray
            return rayOrigin + rayDirection * depth;
        }
        else {
            // For orthographic, calculate world position at depth
            Vector2 normalized(
                (screenPos.x - viewport.x) / (viewport.width * 0.5f) - 1.0f,
                1.0f - (screenPos.y - viewport.y) / (viewport.height * 0.5f)
            );

            Vector3 right = getRight();
            Vector3 up = getUp();
            Vector3 forward = getForward();

            Vector3 worldPos = position_ +
                right * (normalized.x * orthographicSize_ * viewport.getAspectRatio()) +
                up * (normalized.y * orthographicSize_) +
                forward * depth;

            return worldPos;
        }
    }

    /**
     * @brief Update 3D camera state
     * CHANGE: Comprehensive 3D camera update with all movement modes
     */
    void Camera3D::update(float deltaTime) {
        if (!enabled_) return;

        // Update movement based on mode
        updateMovement(deltaTime);

        // Apply constraints
        applyPitchConstraints();

        // Apply bounds to final position
        position_ = bounds_.clamp(position_);
    }

    /**
     * @brief Reset 3D camera to default state
     * CHANGE: Reset all 3D camera properties to sensible defaults
     */
    void Camera3D::reset() {
        position_ = Vector3(0.0f, 0.0f, 5.0f);
        target_ = math::constants::VEC3_ZERO;
        up_ = math::constants::VEC3_UP;
        yaw_ = -90.0f;
        pitch_ = 0.0f;
        roll_ = 0.0f;
        followTarget_ = math::constants::VEC3_ZERO;
        orbitalAngle_ = 0.0f;
        updateFromEuler();
    }

    /**
     * @brief Get 3D camera debug information
     * CHANGE: Detailed 3D camera debug output
     */
    std::string Camera3D::getDebugInfo() const {
        std::ostringstream oss;
        oss << BaseCamera::getDebugInfo();
        oss << " 3D[Pos=(" << position_.x << "," << position_.y << "," << position_.z << ")"
            << ", Target=(" << target_.x << "," << target_.y << "," << target_.z << ")"
            << ", FOV=" << fov_ << ", Yaw=" << yaw_ << ", Pitch=" << pitch_
            << ", Projection=" << (isPerspective_ ? "Perspective" : "Orthographic") << "]";
        return oss.str();
    }

    /**
     * @brief Validate 3D camera state
     * CHANGE: 3D-specific validation with proper bounds checking
     */
    bool Camera3D::validate() const {
        if (!BaseCamera::validate()) return false;

        // Validate FOV
        if (fov_ <= 0.0f || fov_ >= 180.0f) return false;

        // Validate clipping planes
        if (nearPlane_ <= 0.0f || farPlane_ <= nearPlane_) return false;

        // Validate pitch constraints
        if (minPitch_ < -90.0f || maxPitch_ > 90.0f || minPitch_ > maxPitch_) return false;

        // Validate orthographic size
        if (orthographicSize_ <= 0.0f) return false;

        // Validate follow parameters
        if (followDistance_ < 0.0f || followSpeed_ < 0.0f) return false;

        return true;
    }

    /**
     * @brief Update 3D camera movement based on mode
     * CHANGE: Comprehensive 3D movement system with all camera modes
     */
    void Camera3D::updateMovement(float deltaTime) {
        Vector3 oldPosition = position_;
        Vector3 oldTarget = target_;

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

    /**
     * @brief Update FPS-style free look camera
     * CHANGE: FPS camera movement with Euler angle control
     */
    void Camera3D::updateFreeLook(float deltaTime) {
        // For free look, the target is calculated from position + forward direction
        // The position is typically updated by external input (WASD movement)
        updateFromEuler(); // Update target based on current Euler angles
    }

    /**
     * @brief Update target following camera
     * CHANGE: Third-person camera with smooth following and offset
     */
    void Camera3D::updateFollowTarget(float deltaTime) {
        // Calculate desired position behind and above target
        Vector3 desiredPosition = followTarget_;

        // Calculate offset based on current yaw and pitch
        float yawRad = yaw_ * math::constants::DEG_TO_RAD;    // CAMBIO: usar constante de MathTypes
        float pitchRad = pitch_ * math::constants::DEG_TO_RAD; // CAMBIO: usar constante de MathTypes

        Vector3 offset(
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

    /**
     * @brief Update orbital camera movement
     * CHANGE: Orbital camera that rotates around a central point
     */
    void Camera3D::updateOrbital(float deltaTime) {
        // Update orbital angle
        orbitalAngle_ += orbitalSpeed_ * deltaTime;

        // Keep angle in valid range
        while (orbitalAngle_ > math::constants::TWO_PI) { // CAMBIO: usar constante de MathTypes
            orbitalAngle_ -= math::constants::TWO_PI;
        }
        while (orbitalAngle_ < 0.0f) {
            orbitalAngle_ += math::constants::TWO_PI;
        }

        // Calculate position around target
        Vector3 offset(
            std::cos(orbitalAngle_) * orbitalRadius_,
            followHeight_,
            std::sin(orbitalAngle_) * orbitalRadius_
        );

        position_ = followTarget_ + offset;
        target_ = followTarget_;  // Always look at the center
    }

    /**
     * @brief Update target from Euler angles
     * CHANGE: Convert yaw/pitch/roll to target direction
     */
    void Camera3D::updateFromEuler() {
        float yawRad = yaw_ * math::constants::DEG_TO_RAD;    // CAMBIO: usar constante de MathTypes
        float pitchRad = pitch_ * math::constants::DEG_TO_RAD; // CAMBIO: usar constante de MathTypes

        Vector3 direction(
            std::cos(pitchRad) * std::cos(yawRad),
            std::sin(pitchRad),
            std::cos(pitchRad) * std::sin(yawRad)
        );

        target_ = position_ + direction;
    }

    /**
     * @brief Apply pitch constraints to prevent gimbal lock
     * CHANGE: Clamp pitch to safe range
     */
    void Camera3D::applyPitchConstraints() {
        pitch_ = math::clamp(pitch_, minPitch_, maxPitch_);
    }

    /**
     * @brief Calculate view matrix for rendering integration
     * CHANGE: Standard look-at matrix calculation
     */
    std::array<float, 16> Camera3D::calculateViewMatrix() const {
        // CAMBIO: Usar math::lookAt que internamente usa glm::lookAt
        math::Matrix4 viewMatrix = math::lookAt(position_, target_, up_);

        // Convert to array
        std::array<float, 16> result;
        memcpy(result.data(), glm::value_ptr(viewMatrix), sizeof(float) * 16);
        return result;
    }

    /**
     * @brief Calculate projection matrix for rendering
     * CHANGE: Perspective or orthographic projection matrix
     */
    std::array<float, 16> Camera3D::calculateProjectionMatrix(const Viewport& viewport) const {
        math::Matrix4 projMatrix;

        if (isPerspective_) {
            // CAMBIO: Usar math::perspective que internamente usa glm::perspective
            float aspect = viewport.getAspectRatio();
            float fovRad = fov_ * math::constants::DEG_TO_RAD;
            projMatrix = math::perspective(fovRad, aspect, nearPlane_, farPlane_);
        } else {
            // CAMBIO: Usar math::ortho que internamente usa glm::ortho
            float aspect = viewport.getAspectRatio();
            float right = orthographicSize_ * aspect;
            float left = -right;
            float top = orthographicSize_;
            float bottom = -top;
            projMatrix = math::ortho(left, right, bottom, top, nearPlane_, farPlane_);
        }

        // Convert to array
        std::array<float, 16> result;
        memcpy(result.data(), glm::value_ptr(projMatrix), sizeof(float) * 16);
        return result;
    }

    // ========================================================================
    // CAMERA MANAGER IMPLEMENTATION
    // ========================================================================

    /**
     * @brief Constructor with configuration
     * CHANGE: Initialize camera manager with proper defaults
     */
    CameraManager::CameraManager(const CameraManagerConfig& config)
        : config_(config) // Store configuration
          , initialized_(false) // Start uninitialized
          , viewport_(0, 0, 1280, 720) // Default viewport
          , nextCameraId_(1) // Start camera IDs at 1
          , nextTransitionId_(1) // Start transition IDs at 1
          , activeCameraId_(INVALID_CAMERA_ID) // No active camera initially
          , lastUpdateTime_(std::chrono::steady_clock::now()) // Initialize timing
    {
        // Reserve space for expected number of cameras and transitions
        cameras_.reserve(config_.maxCameras);
        transitions_.reserve(config_.maxTransitions);
    }

    /**
     * @brief Destructor - cleanup all cameras and resources
     * CHANGE: Ensure proper cleanup order
     */
    CameraManager::~CameraManager() {
        shutdown();
    }

    /**
     * @brief Initialize camera manager
     * CHANGE: Setup camera system with validation
     */
    bool CameraManager::initialize() {
        if (initialized_) {
            return true; // Already initialized
        }

        std::lock_guard<std::mutex> lock(cameraMutex_);

        logCameraEvent("Initializing CameraManager...");

        // Validate configuration
        if (config_.maxCameras == 0 || config_.maxTransitions == 0) {
            logCameraEvent("Invalid configuration: maxCameras and maxTransitions must be > 0");
            return false;
        }

        // Initialize timing
        lastUpdateTime_ = std::chrono::steady_clock::now();

        initialized_ = true;
        logCameraEvent("CameraManager initialized successfully");

        return true;
    }

    /**
     * @brief Shutdown and cleanup all resources
     * CHANGE: Thread-safe cleanup with proper resource management
     */
    void CameraManager::shutdown() {
        if (!initialized_) {
            return; // Already shutdown
        }

        std::lock_guard<std::mutex> lock(cameraMutex_);

        logCameraEvent("Shutting down CameraManager...");

        // Clear all cameras and associated data
        cameras_.clear();
        cameraNameMap_.clear();
        transitions_.clear();
        shakeStates_.clear();

        // Reset IDs
        nextCameraId_ = 1;
        nextTransitionId_ = 1;
        activeCameraId_ = INVALID_CAMERA_ID;

        initialized_ = false;
        logCameraEvent("CameraManager shutdown complete");
    }

    /**
     * @brief Update all cameras and transitions
     * CHANGE: Comprehensive update with performance tracking
     */
    void CameraManager::update(float deltaTime) {
        if (!initialized_) {
            return;
        }

        std::lock_guard<std::mutex> lock(cameraMutex_);

        auto startTime = std::chrono::steady_clock::now();

        // Update all cameras
        for (auto& [cameraId, camera] : cameras_) {
            if (camera && camera->isEnabled()) {
                camera->update(deltaTime);
                camerasUpdated_.fetch_add(1, std::memory_order_relaxed);
            }
        }

        // Update transitions
        updateTransitions(deltaTime);

        // Update shake effects
        updateShakeEffects(deltaTime);

        // Cleanup completed transitions and inactive shakes
        cleanupCompletedTransitions();
        cleanupInactiveShakes();

        lastUpdateTime_ = startTime;
    }

    /**
     * @brief Create a new 2D camera
     * CHANGE: Thread-safe camera creation with validation
     */
    CameraID CameraManager::createCamera2D(const std::string& name) {
        if (!initialized_) {
            logCameraEvent("Cannot create camera: CameraManager not initialized");
            return INVALID_CAMERA_ID;
        }

        std::lock_guard<std::mutex> lock(cameraMutex_);

        // Check camera limit
        if (cameras_.size() >= config_.maxCameras) {
            logCameraEvent("Cannot create camera: Maximum camera limit reached");
            return INVALID_CAMERA_ID;
        }

        CameraID cameraId = nextCameraId_++;
        std::string cameraName = name.empty() ? ("Camera2D_" + std::to_string(cameraId)) : name;

        // Check for name conflicts
        if (cameraNameMap_.find(cameraName) != cameraNameMap_.end()) {
            cameraName += "_" + std::to_string(cameraId); // Make name unique
        }

        // Create camera
        auto camera = std::make_unique<Camera2D>(cameraId, cameraName);
        camera->setSmoothingSpeed(config_.defaultSmoothingSpeed);

        // Store camera
        cameras_[cameraId] = std::move(camera);
        cameraNameMap_[cameraName] = cameraId;

        logCameraEvent("Created 2D camera: " + cameraName + " (ID: " + std::to_string(cameraId) + ")");

        return cameraId;
    }

    /**
     * @brief Create a new 3D camera
     * CHANGE: Thread-safe 3D camera creation with configuration
     */
    CameraID CameraManager::createCamera3D(const std::string& name, bool perspective) {
        if (!initialized_) {
            logCameraEvent("Cannot create camera: CameraManager not initialized");
            return INVALID_CAMERA_ID;
        }

        std::lock_guard<std::mutex> lock(cameraMutex_);

        // Check camera limit
        if (cameras_.size() >= config_.maxCameras) {
            logCameraEvent("Cannot create camera: Maximum camera limit reached");
            return INVALID_CAMERA_ID;
        }

        CameraID cameraId = nextCameraId_++;
        std::string cameraName = name.empty() ? ("Camera3D_" + std::to_string(cameraId)) : name;

        // Check for name conflicts
        if (cameraNameMap_.find(cameraName) != cameraNameMap_.end()) {
            cameraName += "_" + std::to_string(cameraId); // Make name unique
        }

        // Create camera
        auto camera = std::make_unique<Camera3D>(cameraId, cameraName, perspective);
        camera->setSmoothingSpeed(config_.defaultSmoothingSpeed);

        // Store camera
        cameras_[cameraId] = std::move(camera);
        cameraNameMap_[cameraName] = cameraId;

        logCameraEvent("Created 3D camera: " + cameraName + " (ID: " + std::to_string(cameraId) +
            ", Perspective: " + (perspective ? "Yes" : "No") + ")");

        return cameraId;
    }

    /**
     * @brief Remove a camera and all its associated data
     * CHANGE: Safe camera removal with dependency cleanup
     */
    bool CameraManager::removeCamera(CameraID cameraId) {
        if (!isValidCameraId(cameraId)) {
            return false;
        }

        std::lock_guard<std::mutex> lock(cameraMutex_);

        auto it = cameras_.find(cameraId);
        if (it == cameras_.end()) {
            return false;
        }

        std::string cameraName = it->second->getName();

        // Remove from name map
        cameraNameMap_.erase(cameraName);

        // Stop all transitions for this camera
        stopAllTransitions(cameraId);

        // Stop shake effects for this camera
        stopCameraShake(cameraId);

        // If this was the active camera, clear active camera
        if (activeCameraId_ == cameraId) {
            activeCameraId_ = INVALID_CAMERA_ID;
        }

        // Remove camera
        cameras_.erase(it);

        logCameraEvent("Removed camera: " + cameraName + " (ID: " + std::to_string(cameraId) + ")");

        return true;
    }

    /**
     * @brief Get camera by ID
     * CHANGE: Thread-safe camera access with validation
     */
    BaseCamera* CameraManager::getCamera(CameraID cameraId) {
        if (!isValidCameraId(cameraId)) {
            return nullptr;
        }

        std::lock_guard<std::mutex> lock(cameraMutex_);

        auto it = cameras_.find(cameraId);
        return (it != cameras_.end()) ? it->second.get() : nullptr;
    }

    /**
     * @brief Get camera by ID (const version)
     * CHANGE: Const-correct camera access
     */
    const BaseCamera* CameraManager::getCamera(CameraID cameraId) const {
        if (!isValidCameraId(cameraId)) {
            return nullptr;
        }

        std::lock_guard<std::mutex> lock(cameraMutex_);

        auto it = cameras_.find(cameraId);
        return (it != cameras_.end()) ? it->second.get() : nullptr;
    }

    /**
     * @brief Get camera by name
     * CHANGE: Name-based camera lookup
     */
    CameraID CameraManager::getCameraByName(const std::string& name) const {
        std::lock_guard<std::mutex> lock(cameraMutex_);

        auto it = cameraNameMap_.find(name);
        return (it != cameraNameMap_.end()) ? it->second : INVALID_CAMERA_ID;
    }

    /**
     * @brief Get 2D camera with type checking
     * CHANGE: Type-safe 2D camera access
     */
    Camera2D* CameraManager::getCamera2D(CameraID cameraId) {
        BaseCamera* camera = getCamera(cameraId);
        if (camera && camera->getType() == CameraType::CAMERA_2D) {
            return static_cast<Camera2D*>(camera);
        }
        return nullptr;
    }

    /**
     * @brief Get 3D camera with type checking
     * CHANGE: Type-safe 3D camera access
     */
    Camera3D* CameraManager::getCamera3D(CameraID cameraId) {
        BaseCamera* camera = getCamera(cameraId);
        if (camera && (camera->getType() == CameraType::CAMERA_3D_PERSPECTIVE ||
            camera->getType() == CameraType::CAMERA_3D_ORTHOGRAPHIC)) {
            return static_cast<Camera3D*>(camera);
        }
        return nullptr;
    }

    /**
     * @brief Set the active camera
     * CHANGE: Thread-safe active camera management
     */
    bool CameraManager::setActiveCamera(CameraID cameraId) {
        if (cameraId != INVALID_CAMERA_ID && !isValidCameraId(cameraId)) {
            return false;
        }

        std::lock_guard<std::mutex> lock(cameraMutex_);

        // Deactivate current active camera
        if (activeCameraId_ != INVALID_CAMERA_ID) {
            auto it = cameras_.find(activeCameraId_);
            if (it != cameras_.end()) {
                it->second->active_ = false;
            }
        }

        // Activate new camera
        activeCameraId_ = cameraId;

        if (cameraId != INVALID_CAMERA_ID) {
            auto it = cameras_.find(cameraId);
            if (it != cameras_.end()) {
                it->second->active_ = true;
                logCameraEvent(
                    "Set active camera: " + it->second->getName() + " (ID: " + std::to_string(cameraId) + ")");
            }
        }
        else {
            logCameraEvent("Cleared active camera");
        }

        return true;
    }

    /**
     * @brief Set active camera by name
     * CHANGE: Name-based active camera setting
     */
    bool CameraManager::setActiveCamera(const std::string& name) {
        CameraID cameraId = getCameraByName(name);
        return setActiveCamera(cameraId);
    }

    /**
     * @brief Get the active camera pointer
     * CHANGE: Safe active camera access
     */
    BaseCamera* CameraManager::getActiveCamera() {
        return getCamera(activeCameraId_);
    }

    /**
     * @brief Get the active camera pointer (const version)
     * CHANGE: Const-correct active camera access
     */
    const BaseCamera* CameraManager::getActiveCamera() const {
        return getCamera(activeCameraId_);
    }

    /**
     * @brief Start a smooth transition to a new camera position
     * CHANGE: Position transition with comprehensive configuration
     */
    TransitionID CameraManager::transitionToPosition(CameraID cameraId, const Vector3& targetPosition,
                                                     const TransitionConfig& config) {
        if (!isValidCameraId(cameraId) || !config_.enableTransitions) {
            return INVALID_TRANSITION_ID;
        }

        std::lock_guard<std::mutex> lock(cameraMutex_);

        // Check transition limit
        if (transitions_.size() >= config_.maxTransitions) {
            logCameraEvent("Cannot create transition: Maximum transition limit reached");
            return INVALID_TRANSITION_ID;
        }

        BaseCamera* camera = cameras_[cameraId].get();
        if (!camera) {
            return INVALID_TRANSITION_ID;
        }

        TransitionID transitionId = nextTransitionId_++;

        CameraTransition& transition = transitions_[transitionId];
        transition.id = transitionId;
        transition.cameraId = cameraId;
        transition.type = config.type;
        transition.duration = config.duration;
        transition.currentTime = 0.0f;
        transition.active = true;

        // Position transition
        transition.hasPosition = true;
        transition.startPosition = camera->getPosition();
        transition.targetPosition = config.relative ? transition.startPosition + targetPosition : targetPosition;

        // Other properties
        transition.hasTarget = false;
        transition.hasZoom = false;
        transition.callback = config.onComplete;

        logCameraEvent("Started position transition for camera " + camera->getName() +
            " (Transition ID: " + std::to_string(transitionId) +
            ", Duration: " + std::to_string(config.duration) + "s)");

        return transitionId;
    }

    /**
     * @brief Start a smooth transition to look at a target
     * CHANGE: Target transition for 3D cameras
     */
    TransitionID CameraManager::transitionToTarget(CameraID cameraId, const Vector3& targetLookAt,
                                                   const TransitionConfig& config) {
        if (!isValidCameraId(cameraId) || !config_.enableTransitions) {
            return INVALID_TRANSITION_ID;
        }

        std::lock_guard<std::mutex> lock(cameraMutex_);

        // Check transition limit
        if (transitions_.size() >= config_.maxTransitions) {
            logCameraEvent("Cannot create transition: Maximum transition limit reached");
            return INVALID_TRANSITION_ID;
        }

        Camera3D* camera = getCamera3D(cameraId);
        if (!camera) {
            logCameraEvent("Target transition only supported for 3D cameras");
            return INVALID_TRANSITION_ID;
        }

        TransitionID transitionId = nextTransitionId_++;

        CameraTransition& transition = transitions_[transitionId];
        transition.id = transitionId;
        transition.cameraId = cameraId;
        transition.type = config.type;
        transition.duration = config.duration;
        transition.currentTime = 0.0f;
        transition.active = true;

        // Target transition
        transition.hasTarget = true;
        transition.startTarget = camera->getTarget();
        transition.targetLookAt = config.relative ? transition.startTarget + targetLookAt : targetLookAt;

        // Other properties
        transition.hasPosition = false;
        transition.hasZoom = false;
        transition.callback = config.onComplete;

        logCameraEvent("Started target transition for camera " + camera->getName() +
            " (Transition ID: " + std::to_string(transitionId) +
            ", Duration: " + std::to_string(config.duration) + "s)");

        return transitionId;
    }

    /**
     * @brief Start a transition between two cameras
     * CHANGE: Complete camera transition with position and target interpolation
     */
    TransitionID CameraManager::transitionBetweenCameras(CameraID fromCameraId, CameraID toCameraId,
                                                         const TransitionConfig& config) {
        if (!isValidCameraId(fromCameraId) || !isValidCameraId(toCameraId) || !config_.enableTransitions) {
            return INVALID_TRANSITION_ID;
        }

        std::lock_guard<std::mutex> lock(cameraMutex_);

        // Check transition limit
        if (transitions_.size() >= config_.maxTransitions) {
            logCameraEvent("Cannot create transition: Maximum transition limit reached");
            return INVALID_TRANSITION_ID;
        }

        BaseCamera* fromCamera = cameras_[fromCameraId].get();
        BaseCamera* toCamera = cameras_[toCameraId].get();

        if (!fromCamera || !toCamera) {
            return INVALID_TRANSITION_ID;
        }

        TransitionID transitionId = nextTransitionId_++;

        CameraTransition& transition = transitions_[transitionId];
        transition.id = transitionId;
        transition.cameraId = fromCameraId; // Transition the from camera
        transition.type = config.type;
        transition.duration = config.duration;
        transition.currentTime = 0.0f;
        transition.active = true;

        // Position transition
        transition.hasPosition = true;
        transition.startPosition = fromCamera->getPosition();
        transition.targetPosition = toCamera->getPosition();

        // Target transition (for 3D cameras)
        Camera3D* fromCamera3D = dynamic_cast<Camera3D*>(fromCamera);
        Camera3D* toCamera3D = dynamic_cast<Camera3D*>(toCamera);

        if (fromCamera3D && toCamera3D) {
            transition.hasTarget = true;
            transition.startTarget = fromCamera3D->getTarget();
            transition.targetLookAt = toCamera3D->getTarget();
        }
        else {
            transition.hasTarget = false;
        }

        // Zoom transition (for 2D cameras)
        Camera2D* fromCamera2D = dynamic_cast<Camera2D*>(fromCamera);
        Camera2D* toCamera2D = dynamic_cast<Camera2D*>(toCamera);

        if (fromCamera2D && toCamera2D) {
            transition.hasZoom = true;
            transition.startZoom = fromCamera2D->getZoom();
            transition.targetZoom = toCamera2D->getZoom();
        }
        else {
            transition.hasZoom = false;
        }

        transition.callback = config.onComplete;

        logCameraEvent("Started camera transition from " + fromCamera->getName() +
            " to " + toCamera->getName() +
            " (Transition ID: " + std::to_string(transitionId) +
            ", Duration: " + std::to_string(config.duration) + "s)");

        return transitionId;
    }

    /**
     * @brief Stop a specific transition
     * CHANGE: Safe transition termination
     */
    bool CameraManager::stopTransition(TransitionID transitionId) {
        if (transitionId == INVALID_TRANSITION_ID) {
            return false;
        }

        std::lock_guard<std::mutex> lock(cameraMutex_);

        auto it = transitions_.find(transitionId);
        if (it != transitions_.end()) {
            it->second.active = false;
            logCameraEvent("Stopped transition ID: " + std::to_string(transitionId));
            return true;
        }

        return false;
    }

    /**
     * @brief Stop all transitions for a camera
     * CHANGE: Camera-specific transition cleanup
     */
    int CameraManager::stopAllTransitions(CameraID cameraId) {
        if (!isValidCameraId(cameraId)) {
            return 0;
        }

        std::lock_guard<std::mutex> lock(cameraMutex_);

        int stoppedCount = 0;
        for (auto& [transitionId, transition] : transitions_) {
            if (transition.cameraId == cameraId && transition.active) {
                transition.active = false;
                ++stoppedCount;
            }
        }

        if (stoppedCount > 0) {
            logCameraEvent(
                "Stopped " + std::to_string(stoppedCount) + " transitions for camera ID: " + std::to_string(cameraId));
        }

        return stoppedCount;
    }

    /**
     * @brief Check if a transition is active
     * CHANGE: Thread-safe transition status check
     */
    bool CameraManager::isTransitionActive(TransitionID transitionId) const {
        if (transitionId == INVALID_TRANSITION_ID) {
            return false;
        }

        std::lock_guard<std::mutex> lock(cameraMutex_);

        auto it = transitions_.find(transitionId);
        return (it != transitions_.end()) && it->second.active;
    }

    /**
     * @brief Start camera shake effect
     * CHANGE: Comprehensive shake system with pattern support
     */
    bool CameraManager::startCameraShake(CameraID cameraId, const ShakeConfig& config) {
        if (!isValidCameraId(cameraId) || !config_.enableShake) {
            return false;
        }

        std::lock_guard<std::mutex> lock(cameraMutex_);

        BaseCamera* camera = cameras_[cameraId].get();
        if (!camera) {
            return false;
        }

        CameraShake& shake = shakeStates_[cameraId];
        shake.cameraId = cameraId;
        shake.config = config;
        shake.currentTime = 0.0f;
        shake.active = true;
        shake.basePosition = camera->getPosition();
        shake.currentOffset = math::constants::VEC3_ZERO; // CAMBIO: usar constante de MathTypes

        logCameraEvent("Started shake effect for camera " + camera->getName() +
                      " (Intensity: " + std::to_string(config.intensity) +
                      ", Duration: " + std::to_string(config.duration) + "s)");

        return true;
    }

    /**
     * @brief Stop camera shake for specific camera
     * CHANGE: Safe shake termination
     */
    bool CameraManager::stopCameraShake(CameraID cameraId) {
        if (!isValidCameraId(cameraId)) {
            return false;
        }

        std::lock_guard<std::mutex> lock(cameraMutex_);

        auto it = shakeStates_.find(cameraId);
        if (it != shakeStates_.end()) {
            it->second.active = false;
            logCameraEvent("Stopped shake effect for camera ID: " + std::to_string(cameraId));
            return true;
        }

        return false;
    }

    /**
     * @brief Update shake intensity for active shake
     * CHANGE: Runtime shake intensity modification
     */
    bool CameraManager::updateShakeIntensity(CameraID cameraId, float intensity) {
        if (!isValidCameraId(cameraId)) {
            return false;
        }

        std::lock_guard<std::mutex> lock(cameraMutex_);

        auto it = shakeStates_.find(cameraId);
        if (it != shakeStates_.end() && it->second.active) {
            it->second.config.intensity = std::max(0.0f, intensity);
            return true;
        }

        return false;
    }

    /**
     * @brief Check if camera is currently shaking
     * CHANGE: Thread-safe shake status check
     */
    bool CameraManager::isCameraShaking(CameraID cameraId) const {
        if (!isValidCameraId(cameraId)) {
            return false;
        }

        std::lock_guard<std::mutex> lock(cameraMutex_);

        auto it = shakeStates_.find(cameraId);
        return (it != shakeStates_.end()) && it->second.active;
    }

    /**
     * @brief Process mouse look input for active camera
     * CHANGE: Mouse input processing with sensitivity and constraints
     */
    void CameraManager::processMouseLook(float deltaX, float deltaY, float deltaTime) {
        BaseCamera* activeCamera = getActiveCamera();
        if (!activeCamera) {
            return;
        }

        Camera3D* camera3D = dynamic_cast<Camera3D*>(activeCamera);
        if (camera3D && (camera3D->getMode() == CameraMode::FREE_LOOK ||
            camera3D->getMode() == CameraMode::FOLLOW_TARGET)) {
            // Apply sensitivity
            float yawDelta = deltaX * config_.mouseSensitivity * deltaTime;
            float pitchDelta = deltaY * config_.mouseSensitivity * deltaTime;

            // Update camera angles
            float newYaw = camera3D->getYaw() + yawDelta;
            float newPitch = camera3D->getPitch() - pitchDelta; // Invert Y for natural feel

            camera3D->setYaw(newYaw);
            camera3D->setPitch(newPitch);
        }
    }

    /**
     * @brief Process zoom input for active camera
     * CHANGE: Zoom processing for both 2D and 3D cameras
     */
    void CameraManager::processZoom(float zoomDelta) {
        BaseCamera* activeCamera = getActiveCamera();
        if (!activeCamera) {
            return;
        }

        float scaledDelta = zoomDelta * config_.scrollSensitivity;

        if (Camera2D* camera2D = dynamic_cast<Camera2D*>(activeCamera)) {
            // 2D camera zoom
            float currentZoom = camera2D->getZoom();
            float newZoom = currentZoom * (1.0f + scaledDelta * 0.1f); // 10% per scroll unit
            camera2D->setZoom(newZoom);
        }
        else if (Camera3D* camera3D = dynamic_cast<Camera3D*>(activeCamera)) {
            // 3D camera zoom (adjust FOV or distance)
            if (camera3D->getMode() == CameraMode::FOLLOW_TARGET || camera3D->getMode() == CameraMode::ORBITAL) {
                // Adjust follow distance
                float currentDistance = camera3D->getFollowDistance();
                float newDistance = currentDistance * (1.0f + scaledDelta * 0.1f);
                camera3D->setFollowDistance(std::max(0.5f, newDistance));
            }
            else {
                // Adjust FOV
                float currentFOV = camera3D->getFOV();
                float newFOV = currentFOV - scaledDelta * 5.0f; // 5 degrees per scroll unit
                camera3D->setFOV(newFOV);
            }
        }
    }

    /**
     * @brief Process movement input for active camera
     * CHANGE: Camera movement with mode-specific behavior
     */
    void CameraManager::processMovement(float forward, float right, float up, float deltaTime) {
        BaseCamera* activeCamera = getActiveCamera();
        if (!activeCamera) {
            return;
        }

        if (Camera3D* camera3D = dynamic_cast<Camera3D*>(activeCamera)) {
            if (camera3D->getMode() == CameraMode::FREE_LOOK) {
                // FPS-style movement
                Vector3 forwardDir = camera3D->getForward();
                Vector3 rightDir = camera3D->getRight();
                Vector3 upDir = camera3D->getUp();

                // Calculate movement vector
                Vector3 movement = forwardDir * forward + rightDir * right + upDir * up;
                movement *= 10.0f * deltaTime; // Movement speed

                // Apply movement
                Vector3 newPosition = camera3D->getPosition() + movement;
                camera3D->setPosition(newPosition);
            }
        }
        else if (Camera2D* camera2D = dynamic_cast<Camera2D*>(activeCamera)) {
            // 2D camera movement
            Vector2 movement(right, -forward); // Invert forward for screen coordinates
            movement *= 100.0f * deltaTime; // Movement speed

            Vector2 newPosition = camera2D->getPosition2D() + movement;
            camera2D->setPosition(newPosition);
        }
    }

    /**
     * @brief Convert world position to screen coordinates using active camera
     * CHANGE: Active camera coordinate transformation
     */
    Vector2 CameraManager::worldToScreen(const Vector3& worldPos) const {
        const BaseCamera* activeCamera = getActiveCamera();
        if (!activeCamera) {
            return math::constants::VEC2_ZERO; // CAMBIO: usar constante de MathTypes
        }

        if (const Camera2D* camera2D = dynamic_cast<const Camera2D*>(activeCamera)) {
            return camera2D->worldToScreen(Vector2(worldPos.x, worldPos.y), viewport_);
        } else if (const Camera3D* camera3D = dynamic_cast<const Camera3D*>(activeCamera)) {
            return camera3D->worldToScreen(worldPos, viewport_);
        }

        return math::constants::VEC2_ZERO; // CAMBIO: usar constante de MathTypes
    }

    /**
     * @brief Convert screen coordinates to world position using active camera
     * CHANGE: Screen to world transformation with depth support
     */
    Vector3 CameraManager::screenToWorld(const Vector2& screenPos, float depth) const {
        const BaseCamera* activeCamera = getActiveCamera();
        if (!activeCamera) {
            return math::constants::VEC3_ZERO;
        }

        if (const Camera2D* camera2D = dynamic_cast<const Camera2D*>(activeCamera)) {
            Vector2 worldPos2D = camera2D->screenToWorld(screenPos, viewport_);
            return Vector3(worldPos2D.x, worldPos2D.y, depth);
        }
        else if (const Camera3D* camera3D = dynamic_cast<const Camera3D*>(activeCamera)) {
            return camera3D->screenToWorld(screenPos, viewport_, depth);
        }

        return math::constants::VEC3_ZERO;
    }

    /**
     * @brief Get view frustum bounds for active camera
     * CHANGE: Active camera bounds calculation
     */
    CameraBounds CameraManager::getViewBounds() const {
        const BaseCamera* activeCamera = getActiveCamera();
        if (!activeCamera) {
            return CameraBounds{};
        }

        if (const Camera2D* camera2D = dynamic_cast<const Camera2D*>(activeCamera)) {
            return camera2D->getViewBounds(viewport_);
        }
        else if (const Camera3D* camera3D = dynamic_cast<const Camera3D*>(activeCamera)) {
            // For 3D cameras, create bounds from projection
            Vector3 position = camera3D->getPosition();
            float distance = 10.0f; // Default distance for bounds calculation

            return CameraBounds(position - Vector3(distance, distance, distance),
                                position + Vector3(distance, distance, distance));
        }

        return CameraBounds{};
    }

    /**
     * @brief Setup FPS-style camera
     * CHANGE: Preset FPS camera configuration
     */
    bool CameraManager::setupFPSCamera(CameraID cameraId, const Vector3& position, float sensitivity) {
        Camera3D* camera = getCamera3D(cameraId);
        if (!camera) {
            return false;
        }

        camera->setPosition(position);
        camera->setMode(CameraMode::FREE_LOOK);
        camera->setPerspective(true);
        camera->setFOV(90.0f); // Typical FPS FOV

        // Set reasonable constraints
        camera->setPitchLimits(-85.0f, 85.0f);

        config_.mouseSensitivity = sensitivity;

        logCameraEvent("Setup FPS camera: " + camera->getName());
        return true;
    }

    /**
     * @brief Setup third-person camera
     * CHANGE: Preset third-person camera configuration
     */
    bool CameraManager::setupThirdPersonCamera(CameraID cameraId, const Vector3& target, float distance, float height) {
        Camera3D* camera = getCamera3D(cameraId);
        if (!camera) {
            return false;
        }

        camera->setMode(CameraMode::FOLLOW_TARGET);
        camera->setFollowTarget(target);
        camera->setFollowDistance(distance);
        camera->setFollowHeight(height);
        camera->setPerspective(true);
        camera->setFOV(60.0f); // Typical third-person FOV

        logCameraEvent("Setup third-person camera: " + camera->getName());
        return true;
    }

    /**
     * @brief Setup top-down 2D camera
     * CHANGE: Preset top-down camera configuration
     */
    bool CameraManager::setupTopDownCamera(CameraID cameraId, const Vector2& position, float zoom) {
        Camera2D* camera = getCamera2D(cameraId);
        if (!camera) {
            return false;
        }

        camera->setPosition(position);
        camera->setZoom(zoom);
        camera->setMode(CameraMode::TOP_DOWN);

        logCameraEvent("Setup top-down camera: " + camera->getName());
        return true;
    }

    /**
     * @brief Setup side-scrolling 2D camera
     * CHANGE: Preset side-scroller camera configuration
     */
    bool CameraManager::setupSideScrollerCamera(CameraID cameraId, const Vector2& position, float followSpeed) {
        Camera2D* camera = getCamera2D(cameraId);
        if (!camera) {
            return false;
        }

        camera->setPosition(position);
        camera->setMode(CameraMode::SIDE_SCROLLER);
        camera->setFollowSpeed(followSpeed);

        logCameraEvent("Setup side-scroller camera: " + camera->getName());
        return true;
    }

    /**
     * @brief Setup isometric 3D camera
     * CHANGE: Preset isometric camera configuration
     */
    bool CameraManager::setupIsometricCamera(CameraID cameraId, const Vector3& position, const Vector3& target) {
        Camera3D* camera = getCamera3D(cameraId);
        if (!camera) {
            return false;
        }

        camera->setPosition(position);
        camera->setTarget(target);
        camera->setMode(CameraMode::ISOMETRIC);
        camera->setPerspective(false); // Use orthographic for isometric
        camera->setOrthographicSize(20.0f); // Reasonable isometric size

        logCameraEvent("Setup isometric camera: " + camera->getName());
        return true;
    }

    /**
     * @brief Get number of cameras
     * CHANGE: Thread-safe camera count
     */
    std::size_t CameraManager::getCameraCount() const {
        std::lock_guard<std::mutex> lock(cameraMutex_);
        return cameras_.size();
    }

    /**
     * @brief Get number of active transitions
     * CHANGE: Active transition count
     */
    std::size_t CameraManager::getActiveTransitionCount() const {
        std::lock_guard<std::mutex> lock(cameraMutex_);

        std::size_t count = 0;
        for (const auto& [transitionId, transition] : transitions_) {
            if (transition.active) {
                ++count;
            }
        }
        return count;
    }

    /**
     * @brief Get memory usage statistics
     * CHANGE: Approximate memory usage calculation
     */
    std::size_t CameraManager::getMemoryUsage() const {
        std::lock_guard<std::mutex> lock(cameraMutex_);

        std::size_t usage = sizeof(CameraManager);
        usage += cameras_.size() * (sizeof(std::pair<CameraID, std::unique_ptr<BaseCamera>>) + sizeof(BaseCamera));
        usage += cameraNameMap_.size() * (sizeof(std::pair<std::string, CameraID>) + 20); // Estimate string size
        usage += transitions_.size() * sizeof(CameraTransition);
        usage += shakeStates_.size() * sizeof(CameraShake);

        return usage;
    }

    /**
     * @brief Get list of all camera names
     * CHANGE: Return camera names for UI
     */
    std::vector<std::string> CameraManager::getCameraNames() const {
        std::lock_guard<std::mutex> lock(cameraMutex_);

        std::vector<std::string> names;
        names.reserve(cameras_.size());

        for (const auto& [cameraId, camera] : cameras_) {
            names.push_back(camera->getName());
        }

        return names;
    }

    /**
     * @brief Clear all cameras and reset manager
     * CHANGE: Complete system reset
     */
    void CameraManager::clearAllCameras() {
        std::lock_guard<std::mutex> lock(cameraMutex_);

        cameras_.clear();
        cameraNameMap_.clear();
        transitions_.clear();
        shakeStates_.clear();

        nextCameraId_ = 1;
        nextTransitionId_ = 1;
        activeCameraId_ = INVALID_CAMERA_ID;

        logCameraEvent("Cleared all cameras");
    }

    /**
     * @brief Get debug information about camera system
     * CHANGE: Comprehensive debug output
     */
    std::string CameraManager::getDebugInfo() const {
        std::ostringstream oss;
        std::lock_guard<std::mutex> lock(cameraMutex_);

        oss << "=== CameraManager Debug Info ===\n";
        oss << "Initialized: " << (initialized_ ? "Yes" : "No") << "\n";
        oss << "Cameras: " << cameras_.size() << "/" << config_.maxCameras << "\n";
        oss << "Active Camera ID: " << activeCameraId_ << "\n";
        oss << "Active Transitions: " << getActiveTransitionCount() << "/" << config_.maxTransitions << "\n";
        oss << "Shake Effects: " << shakeStates_.size() << "\n";
        oss << "Viewport: " << viewport_.width << "x" << viewport_.height << "\n\n";

        // List all cameras
        oss << "=== Cameras ===\n";
        for (const auto& [cameraId, camera] : cameras_) {
            oss << camera->getDebugInfo() << "\n";
        }

        // List active transitions
        if (!transitions_.empty()) {
            oss << "\n=== Active Transitions ===\n";
            for (const auto& [transitionId, transition] : transitions_) {
                if (transition.active) {
                    oss << "Transition " << transitionId << ": Camera " << transition.cameraId
                        << " (" << transition.currentTime << "/" << transition.duration << "s)\n";
                }
            }
        }

        // List active shakes
        if (!shakeStates_.empty()) {
            oss << "\n=== Active Shakes ===\n";
            for (const auto& [cameraId, shake] : shakeStates_) {
                if (shake.active) {
                    oss << "Shake: Camera " << cameraId
                        << " (Intensity: " << shake.config.intensity << ")\n";
                }
            }
        }

        return oss.str();
    }

    /**
     * @brief Get performance statistics
     * CHANGE: Performance monitoring data
     */
    std::string CameraManager::getPerformanceStats() const {
        std::ostringstream oss;

        oss << "=== CameraManager Performance Stats ===\n";
        oss << "Cameras Updated: " << camerasUpdated_.load(std::memory_order_relaxed) << "\n";
        oss << "Transitions Processed: " << transitionsProcessed_.load(std::memory_order_relaxed) << "\n";
        oss << "Memory Usage: " << getMemoryUsage() << " bytes\n";

        auto now = std::chrono::steady_clock::now();
        auto timeSinceUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastUpdateTime_);
        oss << "Time Since Last Update: " << timeSinceUpdate.count() << "ms\n";

        return oss.str();
    }

    /**
     * @brief Validate all cameras and their states
     * CHANGE: System-wide validation
     */
    bool CameraManager::validateAllCameras() const {
        std::lock_guard<std::mutex> lock(cameraMutex_);

        for (const auto& [cameraId, camera] : cameras_) {
            if (!camera || !camera->validate()) {
                return false;
            }
        }

        return true;
    }

    // ========================================================================
    // PRIVATE IMPLEMENTATION METHODS
    // ========================================================================

    /**
     * @brief Validate camera ID
     * CHANGE: Thread-safe ID validation
     */
    bool CameraManager::isValidCameraId(CameraID cameraId) const {
        return cameraId != INVALID_CAMERA_ID && cameras_.find(cameraId) != cameras_.end();
    }

    /**
     * @brief Process all active transitions
     * CHANGE: Comprehensive transition processing
     */
    void CameraManager::updateTransitions(float deltaTime) {
        for (auto& [transitionId, transition] : transitions_) {
            if (transition.active) {
                bool completed = processTransition(transition, deltaTime);
                if (completed) {
                    transitionsProcessed_.fetch_add(1, std::memory_order_relaxed);
                }
            }
        }
    }

    /**
     * @brief Process all camera shake effects
     * CHANGE: Comprehensive shake processing
     */
    void CameraManager::updateShakeEffects(float deltaTime) {
        for (auto& [cameraId, shake] : shakeStates_) {
            if (shake.active) {
                processShake(shake, deltaTime);
            }
        }
    }

    /**
     * @brief Apply transition interpolation
     * CHANGE: Full transition processing with easing
     */
    bool CameraManager::processTransition(CameraTransition& transition, float deltaTime) {
        if (!transition.active) {
            return false;
        }

        // Update transition time
        transition.currentTime += deltaTime;

        // Check if transition completed
        if (transition.currentTime >= transition.duration) {
            transition.currentTime = transition.duration;
            transition.active = false;
        }

        // Calculate interpolation factor
        float t = transition.currentTime / transition.duration;
        float easedT = calculateTransitionEasing(transition.type, t);

        // Get target camera
        auto cameraIt = cameras_.find(transition.cameraId);
        if (cameraIt == cameras_.end()) {
            return true; // Camera no longer exists, mark transition as completed
        }

        BaseCamera* camera = cameraIt->second.get();

        // Apply position transition
        if (transition.hasPosition) {
            Vector3 newPosition = math::lerp(transition.startPosition, transition.targetPosition, easedT);
            camera->setPosition(newPosition);
        }

        // Apply target transition (3D cameras only)
        if (transition.hasTarget) {
            Camera3D* camera3D = dynamic_cast<Camera3D*>(camera);
            if (camera3D) {
                Vector3 newTarget = math::lerp(transition.startTarget, transition.targetLookAt, easedT);
                camera3D->setTarget(newTarget);
            }
        }

        // Apply zoom transition (2D cameras only)
        if (transition.hasZoom) {
            Camera2D* camera2D = dynamic_cast<Camera2D*>(camera);
            if (camera2D) {
                float newZoom = transition.startZoom + (transition.targetZoom - transition.startZoom) * easedT;
                camera2D->setZoom(newZoom);
            }
        }

        // Check if transition completed and execute callback
        if (!transition.active) {
            if (transition.callback) {
                transition.callback(transition.cameraId, transition.id, true);
            }
            return true;
        }

        return false;
    }

    /**
     * @brief Apply shake effect to camera
     * CHANGE: Comprehensive shake processing with patterns
     */
    bool CameraManager::processShake(CameraShake& shake, float deltaTime) {
        if (!shake.active) {
            return false;
        }

        // Update shake time
        shake.currentTime += deltaTime;

        // Check if shake completed (duration < 0 means infinite)
        if (shake.config.duration > 0.0f && shake.currentTime >= shake.config.duration) {
            shake.active = false;
            return true;
        }

        // Calculate intensity (with fade out if enabled)
        float intensity = shake.config.intensity;
        if (shake.config.fadeOut && shake.config.duration > 0.0f) {
            float fadeT = shake.currentTime / shake.config.duration;
            intensity *= (1.0f - fadeT); // Linear fade out
        }

        // Generate shake offset
        Vector3 shakeOffset = generateShakeOffset(shake.config.pattern, intensity,
                                                  shake.currentTime, shake.config.frequency);

        // Apply axis scaling
        shakeOffset.x *= shake.config.axes.x;
        shakeOffset.y *= shake.config.axes.y;
        shakeOffset.z *= shake.config.axes.z;

        shake.currentOffset = shakeOffset;

        // Apply shake to camera
        auto cameraIt = cameras_.find(shake.cameraId);
        if (cameraIt != cameras_.end()) {
            BaseCamera* camera = cameraIt->second.get();
            Vector3 shakenPosition = shake.basePosition + shakeOffset;
            camera->setPosition(shakenPosition);
        }

        return false;
    }

    /**
     * @brief Calculate transition interpolation value
     * CHANGE: Multiple easing functions for smooth transitions
     */
    float CameraManager::calculateTransitionEasing(TransitionType type, float t) const {
        t = std::clamp(t, 0.0f, 1.0f);

        switch (type) {
        case TransitionType::LINEAR:
            return t;

        case TransitionType::EASE_IN:
            return t * t;

        case TransitionType::EASE_OUT:
            return 1.0f - (1.0f - t) * (1.0f - t);

        case TransitionType::EASE_IN_OUT:
            if (t < 0.5f) {
                return 2.0f * t * t;
            }
            else {
                return 1.0f - 2.0f * (1.0f - t) * (1.0f - t);
            }

        case TransitionType::BOUNCE: {
            const float c1 = 1.70158f;
            const float c3 = c1 + 1.0f;
            return c3 * t * t * t - c1 * t * t;
        }

        case TransitionType::ELASTIC: {
            if (t == 0.0f) return 0.0f;
            if (t == 1.0f) return 1.0f;

            const float c4 = (2.0f * M_PI) / 3.0f;
            return -std::pow(2.0f, 10.0f * (t - 1.0f)) * std::sin((t - 1.0f) * c4);
        }

        default:
            return t; // Fallback to linear
        }
    }

    /**
     * @brief Generate shake offset based on pattern
     * CHANGE: Multiple shake patterns for different effects
     */
    Vector3 CameraManager::generateShakeOffset(ShakePattern pattern, float intensity, float time,
                                               float frequency) const {
        static thread_local std::mt19937 generator(std::random_device{}());
        static thread_local std::uniform_real_distribution<float> distribution(-1.0f, 1.0f);

        Vector3 offset;

        switch (pattern) {
        case ShakePattern::RANDOM: {
            // Random shake in all directions
            offset.x = distribution(generator) * intensity;
            offset.y = distribution(generator) * intensity;
            offset.z = distribution(generator) * intensity * 0.5f; // Less Z movement
        }
        break;

        case ShakePattern::HORIZONTAL: {
            // Only horizontal shake
            offset.x = distribution(generator) * intensity;
            offset.y = 0.0f;
            offset.z = 0.0f;
        }
        break;

        case ShakePattern::VERTICAL: {
            // Only vertical shake
            offset.x = 0.0f;
            offset.y = distribution(generator) * intensity;
            offset.z = 0.0f;
        }
        break;

        case ShakePattern::CIRCULAR: {
            // Circular shake pattern
            float angle = time * frequency * 2.0f * M_PI;
            offset.x = std::cos(angle) * intensity;
            offset.y = std::sin(angle) * intensity;
            offset.z = 0.0f;
        }
        break;

        case ShakePattern::EXPLOSION: {
            // Explosion-like shake with decreasing intensity
            float explosionT = std::max(0.0f, 1.0f - time * 2.0f); // Quick decay
            float explosionIntensity = intensity * explosionT * explosionT;

            offset.x = distribution(generator) * explosionIntensity;
            offset.y = distribution(generator) * explosionIntensity;
            offset.z = distribution(generator) * explosionIntensity * 0.3f;
        }
        break;

        case ShakePattern::EARTHQUAKE: {
            // Low frequency earthquake shake
            float lowFreq = frequency * 0.3f;
            float noise1 = std::sin(time * lowFreq * 2.0f * M_PI) * intensity;
            float noise2 = std::sin(time * lowFreq * 2.0f * M_PI * 1.7f) * intensity * 0.7f;

            offset.x = noise1 + distribution(generator) * intensity * 0.2f;
            offset.y = noise2 + distribution(generator) * intensity * 0.1f;
            offset.z = distribution(generator) * intensity * 0.1f;
        }
        break;

        default:
            offset = math::constants::VEC3_ZERO;
            break;
        }

        return offset;
    }

    /**
     * @brief Log camera event for debugging
     * CHANGE: Conditional logging based on configuration
     */
    void CameraManager::logCameraEvent(const std::string& message) const {
        if (config_.enableCameraLogging) {
            std::cout << "[CameraManager] " << message << std::endl;
        }
    }

    /**
     * @brief Cleanup completed transitions
     * CHANGE: Remove inactive transitions to prevent memory leaks
     */
    void CameraManager::cleanupCompletedTransitions() {
        auto it = transitions_.begin();
        while (it != transitions_.end()) {
            if (!it->second.active) {
                it = transitions_.erase(it);
            }
            else {
                ++it;
            }
        }
    }

    /**
     * @brief Cleanup inactive shake effects
     * CHANGE: Remove inactive shakes to prevent memory leaks
     */
    void CameraManager::cleanupInactiveShakes() {
        auto it = shakeStates_.begin();
        while (it != shakeStates_.end()) {
            if (!it->second.active) {
                // Restore original position before removing shake
                auto cameraIt = cameras_.find(it->first);
                if (cameraIt != cameras_.end()) {
                    BaseCamera* camera = cameraIt->second.get();
                    camera->setPosition(it->second.basePosition);
                }

                it = shakeStates_.erase(it);
            }
            else {
                // Update base position for active shakes
                auto cameraIt = cameras_.find(it->first);
                if (cameraIt != cameras_.end()) {
                    BaseCamera* camera = cameraIt->second.get();
                    // Base position is current position minus shake offset
                    it->second.basePosition = camera->getPosition() - it->second.currentOffset;
                }
                ++it;
            }
        }
    }
} // namespace engine::camera
