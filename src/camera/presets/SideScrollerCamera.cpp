/**
* @file SideScrollerCamera.h
 * @brief Side-scrolling camera preset for platformers
 * @author Andrés Guerrero
 * @date 26-08-2025
 */

#include "SideScrollerCamera.h"

namespace engine::camera {
    CameraID SideScrollerCamera::create(CameraManager& manager,
                                        const SideScrollerCameraConfig& config = {},
                                        const std::string& name = "SideScrollerCamera") {
        // Create 2D camera
        CameraID cameraId = manager.createCamera2D(name);
        if (cameraId == INVALID_CAMERA_ID) {
            return INVALID_CAMERA_ID;
        }

        Camera2D* camera = manager.getCamera2D(cameraId);
        if (!camera) {
            return INVALID_CAMERA_ID;
        }

        // Configure camera
        setup(camera, config);

        return cameraId;
    }

    void SideScrollerCamera::setup(Camera2D* camera, const SideScrollerCameraConfig& config = {}) {
        if (!camera) return;

        // Set camera mode and properties
        camera->setMode(CameraMode::SIDE_SCROLLER);
        camera->setPosition(config.position);
        camera->setZoom(config.zoom);
        camera->setFollowSpeed(config.followSpeed);
        camera->setSmoothingSpeed(1.0f / config.smoothTime);

        // Set vertical bounds if locked
        if (config.lockVertical) {
            CameraBounds bounds;
            bounds.setBounds(
                Vec3(-10000.0f, config.minY, -1.0f),
                Vec3(10000.0f, config.maxY, 1.0f)
            );
            camera->setBounds(bounds);
        }
    }

    void SideScrollerCamera::followPlayer(Camera2D* camera,
                                          const Vec2& playerPosition,
                                          const Vec2& playerVelocity,
                                          const SideScrollerCameraConfig& config,
                                          const float deltaTime) {
        if (!camera) return;

        Vec2 targetPosition = playerPosition;

        // Add offsets
        targetPosition.x += config.horizontalOffset;
        targetPosition.y += config.verticalOffset;

        // Add look-ahead based on velocity
        if (std::abs(playerVelocity.x) > 0.1f) {
            const float lookAhead = math::clamp(playerVelocity.x * 0.1f,
                                                -config.lookAheadDistance,
                                                config.lookAheadDistance);
            targetPosition.x += lookAhead;
        }

        // Handle vertical following with deadzone
        const Vec2 currentPos = camera->getPosition2D();
        const float verticalDiff = targetPosition.y - currentPos.y;

        if (config.lockVertical) {
            // Keep vertical position locked
            targetPosition.y = currentPos.y;
        }
        else if (std::abs(verticalDiff) > config.verticalDeadzone) {
            // Follow vertically only when outside deadzone
            const float verticalTarget = targetPosition.y -
                (verticalDiff > 0 ? config.verticalDeadzone : -config.verticalDeadzone);
            targetPosition.y = currentPos.y +
                (verticalTarget - currentPos.y) * config.verticalFollowSpeed * deltaTime;
        }
        else {
            // Stay within deadzone
            targetPosition.y = currentPos.y;
        }

        // Smooth horizontal following
        targetPosition.x = currentPos.x +
            (targetPosition.x - currentPos.x) * config.followSpeed * deltaTime;

        camera->setPosition(targetPosition);
    }

    void SideScrollerCamera::updateDynamicZoom(Camera2D* camera,
                                               const float playerSpeed,
                                               const float maxSpeed,
                                               const SideScrollerCameraConfig& config,
                                               const float deltaTime) {
        if (!camera || !config.enableDynamicZoom) return;

        // Calculate target zoom based on speed
        const float speedRatio = math::clamp(playerSpeed / maxSpeed, 0.0f, 1.0f);
        const float targetZoom = math::lerp(config.maxDynamicZoom, config.minDynamicZoom, speedRatio);

        // Smooth zoom transition
        const float currentZoom = camera->getZoom();
        const float newZoom = currentZoom + (targetZoom - currentZoom) * 5.0f * deltaTime;
        camera->setZoom(newZoom);
    }

    void SideScrollerCamera::handlePlatformBounds(Camera2D* camera,
                                                  const CameraBounds& platformBounds,
                                                  const float transitionSpeed,
                                                  const float deltaTime) {
        if (!camera) return;

        // Get current position
        const Vec2 currentPos = camera->getPosition2D();
        const Vec3 min = platformBounds.getMin();
        const Vec3 max = platformBounds.getMax();

        // Clamp to platform bounds
        const Vec2 clampedPos(
            math::clamp(currentPos.x, min.x, max.x),
            math::clamp(currentPos.y, min.y, max.y)
        );

        // Smooth transition to clamped position
        const Vec2 newPos = currentPos + (clampedPos - currentPos) * transitionSpeed * deltaTime;
        camera->setPosition(newPos);
    }

    std::vector<Vec2> SideScrollerCamera::calculateParallaxOffsets(const Camera2D* camera,
                                                                      const float* layerDepths,
                                                                      const int layerCount) {
        if (!camera) return {};

        std::vector<Vec2> offsets;
        offsets.reserve(layerCount);

        const Vec2 cameraPos = camera->getPosition2D();

        for (int i = 0; i < layerCount; ++i) {
            const float depth = layerDepths[i];
            // Parallax effect: farther layers move slower
            Vec2 layerOffset = cameraPos * (1.0f - depth);
            offsets.push_back(layerOffset);
        }

        return offsets;
    }

    TransitionID SideScrollerCamera::transitionToRoom(CameraManager& manager,
                                                      const CameraID cameraId,
                                                      const Vec2& newRoomCenter,
                                                      const Vec2& newRoomSize,
                                                      const float transitionDuration = 1.0f) {
        if (const Camera2D* camera = manager.getCamera2D(cameraId); !camera) return INVALID_TRANSITION_ID;

        // Calculate camera position for room
        const Vec3 targetPos(newRoomCenter.x, newRoomCenter.y, 0.0f);

        // Create transition
        TransitionConfig config;
        config.duration = transitionDuration;
        config.type = TransitionType::EASE_IN_OUT;

        return manager.transitionToPosition(cameraId, targetPos, config);
    }

    void SideScrollerCamera::applyImpactShake(CameraManager& manager,
                                              const CameraID cameraId,
                                              const float intensity = 2.0f,
                                              const float duration = 0.3f) {
        ShakeConfig shakeConfig = ShakeConfig::impact(intensity);
        shakeConfig.duration = duration;
        shakeConfig.axes = Vec3(1.0f, 0.5f, 0.0f); // More horizontal shake
        manager.startCameraShake(cameraId, shakeConfig);
    }
} // namespace engine::camera
