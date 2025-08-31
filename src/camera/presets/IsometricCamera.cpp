/**
* @file IsometricCamera.h
 * @brief Isometric camera preset for strategy and RPG games
 * @author Andrés Guerrero
 * @date 26-08-2025
 */

#include "IsometricCamera.h"

namespace engine::camera {
    CameraID IsometricCamera::create(CameraManager& manager,
                                     const IsometricCameraConfig& config = {},
                                     const std::string& name = "IsometricCamera") {
        // Create 3D orthographic camera
        const CameraID cameraId = manager.createCamera3D(name, false); // false for orthographic
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

    void IsometricCamera::setup(Camera3D* camera, const IsometricCameraConfig& config = {}) {
        if (!camera) return;

        // Set camera mode
        camera->setMode(CameraMode::ISOMETRIC);
        camera->setPerspective(false); // Use orthographic projection
        camera->setOrthographicSize(config.orthographicSize);

        // Calculate isometric position
        const Vec3 position = calculateIsometricPosition(config.target,
                                                            config.angle,
                                                            config.rotation,
                                                            20.0f); // Fixed distance
        camera->setPosition(position);
        camera->setTarget(config.target);

        // Set up vector for proper isometric view
        camera->setUp(Vec3(0, 1, 0));

        // Set smoothing
        camera->setSmoothingSpeed(config.smoothing);

        // Set bounds if provided
        if (config.worldBounds.isEnabled()) {
            camera->setBounds(config.worldBounds);
        }
    }

    void IsometricCamera::pan(Camera3D* camera,
                              const Vec2& direction,
                              const IsometricCameraConfig& config,
                              const float deltaTime) {
        if (!camera) return;

        // Convert screen direction to world space
        // Account for isometric rotation
        const float angleRad = config.rotation * math::DEG_TO_RAD<math::Float>;

        const Vec3 worldDir(
            direction.x * std::cos(angleRad) - direction.y * std::sin(angleRad),
            0.0f,
            direction.x * std::sin(angleRad) + direction.y * std::cos(angleRad)
        );

        // Apply movement
        const Vec3 currentPos = camera->getPosition();
        const Vec3 currentTarget = camera->getTarget();

        Vec3 movement = worldDir * config.panSpeed * deltaTime;

        // Apply grid snap if enabled
        if (config.enableGridSnap) {
            movement = snapToGrid(movement, config.gridSize);
        }

        camera->setPosition(currentPos + movement);
        camera->setTarget(currentTarget + movement);
    }

    void IsometricCamera::zoom(Camera3D* camera,
                               const float zoomDelta,
                               const IsometricCameraConfig& config) {
        if (!camera) return;

        const float currentSize = camera->getOrthographicSize();
        float newSize = currentSize * (1.0f - zoomDelta * config.zoomSpeed * 0.1f);

        newSize = math::clamp(newSize, config.minSize, config.maxSize);
        camera->setOrthographicSize(newSize);
    }

    void IsometricCamera::rotate(Camera3D* camera,
                                 const float angleDelta,
                                 const IsometricCameraConfig& config) {
        if (!camera || config.lockRotation) return;

        // Update rotation angle
        const float newRotation = config.rotation + angleDelta;

        // Recalculate camera position
        const Vec3 target = camera->getTarget();
        const float distance = glm::length(camera->getPosition() - target);

        const Vec3 newPosition = calculateIsometricPosition(target,
                                                               config.angle,
                                                               newRotation,
                                                               distance);
        camera->setPosition(newPosition);
    }

    void IsometricCamera::focusOn(Camera3D* camera,
                                  const Vec3& worldPosition,
                                  const bool instant = false) {
        if (!camera) return;

        const Vec3 currentPos = camera->getPosition();
        const Vec3 currentTarget = camera->getTarget();
        const Vec3 offset = currentPos - currentTarget;

        if (instant) {
            camera->setTarget(worldPosition);
            camera->setPosition(worldPosition + offset);
        }
        else {
            // Smooth transition handled by camera smoothing
            camera->setTarget(worldPosition);
            camera->setPosition(worldPosition + offset);
        }
    }

    void IsometricCamera::handleEdgePan(Camera3D* camera,
                                        const Vec2& mousePos,
                                        const Vec2& screenSize,
                                        const IsometricCameraConfig& config,
                                        const float deltaTime) {
        if (!camera || !config.enableEdgePan) return;

        Vec2 panDirection(0.0f, 0.0f);

        // Check screen edges
        if (mousePos.x < config.edgePanZone) {
            panDirection.x = -1.0f;
        }
        else if (mousePos.x > screenSize.x - config.edgePanZone) {
            panDirection.x = 1.0f;
        }

        if (mousePos.y < config.edgePanZone) {
            panDirection.y = 1.0f; // Up in screen space
        }
        else if (mousePos.y > screenSize.y - config.edgePanZone) {
            panDirection.y = -1.0f; // Down in screen space
        }

        // Apply edge panning
        if (panDirection.x != 0.0f || panDirection.y != 0.0f) {
            panDirection = glm::normalize(panDirection);
            pan(camera, panDirection * config.edgePanSpeed, config, deltaTime);
        }
    }

    Vec3 IsometricCamera::screenToWorld(const Camera3D* camera,
                                           const Vec2& screenPos,
                                           const Viewport& viewport,
                                           const float groundHeight = 0.0f) {
        if (!camera) return Vec3(0, 0, 0);

        // Get ray from camera
        auto [rayOrigin, rayDirection] = camera->screenToWorldRay(screenPos, viewport);

        // Intersect with ground plane (Y = groundHeight)
        if (std::abs(rayDirection.y) < 0.001f) {
            // Ray is parallel to ground
            return Vec3(0, groundHeight, 0);
        }

        const float t = (groundHeight - rayOrigin.y) / rayDirection.y;
        if (t < 0) {
            // Intersection behind camera
            return Vec3(0, groundHeight, 0);
        }

        return rayOrigin + rayDirection * t;
    }

    Vec2 IsometricCamera::worldToGrid(const Vec3& worldPos, float gridSize) {
        return Vec2(
            std::floor(worldPos.x / gridSize),
            std::floor(worldPos.z / gridSize)
        );
    }

    Vec3 IsometricCamera::gridToWorld(const Vec2& gridPos, const float gridSize, const float height = 0.0f) {
        return Vec3(
            gridPos.x * gridSize + gridSize * 0.5f,
            height,
            gridPos.y * gridSize + gridSize * 0.5f
        );
    }

    Vec3 IsometricCamera::calculateIsometricPosition(const Vec3& target,
                                                        const float angle,
                                                        const float rotation,
                                                        const float distance) {
        const float angleRad = angle * math::DEG_TO_RAD<math::Float>;
        const float rotationRad = rotation * math::DEG_TO_RAD<math::Float>;

        // Calculate position offset
        const Vec3 offset(
            std::sin(rotationRad) * std::cos(angleRad) * distance,
            std::sin(angleRad) * distance,
            std::cos(rotationRad) * std::cos(angleRad) * distance
        );

        return target + offset;
    }

    Vec3 IsometricCamera::snapToGrid(const Vec3& movement, const float gridSize) {
        return Vec3(
            std::round(movement.x / gridSize) * gridSize,
            movement.y,
            std::round(movement.z / gridSize) * gridSize
        );
    }
} // namespace engine::camera
