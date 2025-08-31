/**
* @file ThirdPersonCamera.cpp
 * @brief Third-person camera preset
 * @author Andrés Guerrero
 * @date 26-08-2025
 */

#include "ThirdPersonCamera.h"

namespace engine::camera {
    CameraID ThirdPersonCamera::create(CameraManager& manager,
                              const ThirdPersonCameraConfig& config = {},
                              const std::string& name = "ThirdPersonCamera") {
        // Create 3D perspective camera
        const CameraID cameraId = manager.createCamera3D(name, true);
        if (cameraId == INVALID_CAMERA_ID) {
            return INVALID_CAMERA_ID;
        }

        Camera3D* camera = manager.getCamera3D(cameraId);
        if (!camera) {
            return INVALID_CAMERA_ID;
        }

        // Configure camera
        setup(camera, config);

        // Configure input if available
        // if (auto* input = manager.getInputHandler()) {
        //     setupInput(input, config);
        // }

        return cameraId;
    }

    void ThirdPersonCamera::setup(Camera3D* camera, const ThirdPersonCameraConfig& config = {}) {
        if (!camera) return;

        // Set camera mode
        camera->setMode(CameraMode::FOLLOW_TARGET);
        camera->setPerspective(true);

        // Configure following
        camera->setFollowTarget(config.targetPosition + config.targetOffset);
        camera->setFollowDistance(config.distance);
        camera->setFollowHeight(config.height);
        camera->setFollowSpeed(config.followSpeed);

        // Configure projection
        camera->setFOV(config.fov);
        camera->setClippingPlanes(0.1f, 1000.0f);

        // Set initial position
        updateCameraPosition(camera, config);
    }

    void ThirdPersonCamera::updateFollow(Camera3D* camera,
                                 const Vec3& targetPosition,
                                 const Vec3& targetForward,
                                 const ThirdPersonCameraConfig& config,
                                 const float deltaTime) {
        if (!camera) return;

        // Update follow target
        const Vec3 followPoint = targetPosition + config.targetOffset;
        camera->setFollowTarget(followPoint);

        // Auto-rotate behind target if enabled
        if (config.autoRotate) {
            autoRotateBehindTarget(camera, targetForward, config, deltaTime);
        }

        // Update camera position
        updateCameraPosition(camera, config);
    }

    Vec3 ThirdPersonCamera::handleCollision(const Camera3D* camera,
                                       const Vec3& targetPosition,
                                       const ThirdPersonCameraConfig& config) {
        // Revisar esto: Pointer may be null
        if (!camera || !config.enableCollision) {
            return camera->getPosition();
        }

        const Vec3 cameraPos = camera->getPosition();
        const Vec3 direction = glm::normalize(cameraPos - targetPosition);

        // Simple sphere cast from target to camera
        // In a real implementation, this would interface with physics system
        const float adjustedDistance = config.distance;

        // TODO: Perform actual collision detection here
        // For now, just return the current position

        return targetPosition + direction * adjustedDistance +
            Vec3(0, config.height, 0);
    }

    void ThirdPersonCamera::adjustDistance(Camera3D* camera,
                                   const float zoomDelta,
                                   const ThirdPersonCameraConfig& config) {
        if (!camera) return;

        const float currentDistance = camera->getFollowDistance();
        float newDistance = currentDistance * (1.0f - zoomDelta * 0.1f);
        newDistance = math::clamp(newDistance, config.minDistance, config.maxDistance);
        camera->setFollowDistance(newDistance);
    }

    void ThirdPersonCamera::adjustHeight(Camera3D* camera,
                                 const float heightDelta,
                                 const ThirdPersonCameraConfig& config) {
        if (!camera) return;

        const float currentHeight = camera->getFollowHeight();
        float newHeight = currentHeight + heightDelta;
        newHeight = math::clamp(newHeight, config.minHeight, config.maxHeight);
        camera->setFollowHeight(newHeight);
    }

    void ThirdPersonCamera::updateCameraPosition(Camera3D* camera, const ThirdPersonCameraConfig& config) {
        const Vec3 target = camera->getFollowTarget();
        const float yaw = camera->getYaw() * math::DEG_TO_RAD<math::Float>;

        // Calculate position offset
        const Vec3 offset(
            std::sin(yaw) * config.distance,
            config.height,
            std::cos(yaw) * config.distance
        );

        camera->setPosition(target + offset);
        camera->setTarget(target);
    }

    void ThirdPersonCamera::autoRotateBehindTarget(Camera3D* camera,
                                           const Vec3& targetForward,
                                           const ThirdPersonCameraConfig& config,
                                           const float deltaTime) {
        // Calculate target yaw from forward direction
        const float targetYaw = std::atan2(targetForward.x, targetForward.z) * math::RAD_TO_DEG<math::Float>;
        const float currentYaw = camera->getYaw();

        // Smooth rotation
        float yawDiff = targetYaw - currentYaw;

        // Normalize angle difference
        while (yawDiff > 180.0f) yawDiff -= 360.0f;
        while (yawDiff < -180.0f) yawDiff += 360.0f;

        const float newYaw = currentYaw + yawDiff * config.autoRotateSpeed * deltaTime;
        camera->setYaw(newYaw);
    }
} // namespace engine::camera
