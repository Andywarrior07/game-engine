/**
 * @file TopDownCamera.cpp
 * @brief Top-down camera preset for 2D and isometric games
 * @author Andrés Guerrero
 * @date 26-08-2025
 */

#include "TopDownCamera.h"

namespace engine::camera {
    CameraID TopDownCamera::create(CameraManager& manager,
                                   const TopDownCameraConfig& config = {},
                                   const std::string& name = "TopDownCamera") {
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

        // Configure input if available
        // if (auto* input = manager.getInputHandler()) {
        //     setupInput(input, config);
        // }

        return cameraId;
    }


    void TopDownCamera::setup(Camera2D* camera, const TopDownCameraConfig& config = {}) {
        if (!camera) return;

        // Set camera mode and properties
        camera->setMode(CameraMode::TOP_DOWN);
        camera->setPosition(config.position);
        camera->setZoom(config.zoom);
        camera->setZoomLimits(config.minZoom, config.maxZoom);
        camera->setFollowSpeed(config.followSpeed);
        camera->setSmoothingSpeed(config.smoothing);

        // Set world bounds if enabled
        if (config.enableBounds) {
            camera->setBounds(config.worldBounds);
        }
    }

    void TopDownCamera::followWithDeadzone(Camera2D* camera,
                                           const Vec2& targetPosition,
                                           const TopDownCameraConfig& config,
                                           const float deltaTime) {
        if (!camera) return;

        const Vec2 cameraPos = camera->getPosition2D();
        const Vec2 diff = targetPosition - cameraPos;

        // Check if target is outside deadzone
        const bool outsideX = std::abs(diff.x) > config.deadzone.x;
        const bool outsideY = std::abs(diff.y) > config.deadzone.y;

        Vec2 newPosition = cameraPos;

        // Move camera to keep target within deadzone
        if (outsideX) {
            const float targetX = targetPosition.x -
                (diff.x > 0 ? config.deadzone.x : -config.deadzone.x);
            newPosition.x += (targetX - cameraPos.x) * config.followSpeed * deltaTime;
        }

        if (outsideY) {
            const float targetY = targetPosition.y -
                (diff.y > 0 ? config.deadzone.y : -config.deadzone.y);
            newPosition.y += (targetY - cameraPos.y) * config.followSpeed * deltaTime;
        }

        camera->setPosition(newPosition);
    }

    void TopDownCamera::handleEdgeScrolling(Camera2D* camera,
                                            const Vec2& mousePos,
                                            const Vec2& screenSize,
                                            const TopDownCameraConfig& config,
                                            const float deltaTime) {
        if (!camera || !config.enableEdgeScrolling) return;

        Vec2 scrollVelocity(0.0f, 0.0f);

        // Check edges
        if (mousePos.x < config.edgeScrollZone) {
            scrollVelocity.x = -config.edgeScrollSpeed;
        }
        else if (mousePos.x > screenSize.x - config.edgeScrollZone) {
            scrollVelocity.x = config.edgeScrollSpeed;
        }

        if (mousePos.y < config.edgeScrollZone) {
            scrollVelocity.y = -config.edgeScrollSpeed;
        }
        else if (mousePos.y > screenSize.y - config.edgeScrollZone) {
            scrollVelocity.y = config.edgeScrollSpeed;
        }

        // Apply scrolling
        if (scrollVelocity.x != 0.0f || scrollVelocity.y != 0.0f) {
            const Vec2 currentPos = camera->getPosition2D();
            const Vec2 newPos = currentPos + scrollVelocity * deltaTime / camera->getZoom();
            camera->setPosition(newPos);
        }
    }

    void TopDownCamera::handleDragPan(Camera2D* camera,
                                      const Vec2& mouseDelta,
                                      const bool isDragging,
                                      const TopDownCameraConfig& config) {
        if (!camera || !config.enableDragPan || !isDragging) return;

        // Convert mouse delta to world space
        Vec2 worldDelta = mouseDelta * config.dragSensitivity / camera->getZoom();

        // Invert Y for intuitive dragging
        worldDelta.y = -worldDelta.y;

        // Apply panning
        Vec2 currentPos = camera->getPosition2D();
        camera->setPosition(currentPos - worldDelta);
    }

    void TopDownCamera::focusOnArea(Camera2D* camera,
                                    const CameraBounds& bounds,
                                    const Viewport& viewport,
                                    const float padding = 0.1f) {
        if (!camera) return;

        // Calculate center of bounds
        const Vec3 center3D = bounds.getCenter();
        const Vec2 center(center3D.x, center3D.y);

        // Calculate required zoom to fit bounds
        const Vec3 size3D = bounds.getSize();
        const Vec2 size(size3D.x, size3D.y);

        const float zoomX = viewport.getWidth() / (size.x * (1.0f + padding));
        const float zoomY = viewport.getHeight() / (size.y * (1.0f + padding));
        const float zoom = std::min(zoomX, zoomY);

        // Apply camera settings
        camera->setPosition(center);
        camera->setZoom(zoom);
    }

    void TopDownCamera::smoothTransition(Camera2D* camera,
                                         const Vec2& targetPosition,
                                         const float targetZoom,
                                         const float speed,
                                         const float deltaTime) {
        if (!camera) return;

        // Smooth position transition
        const Vec2 currentPos = camera->getPosition2D();
        const Vec2 newPos = currentPos + (targetPosition - currentPos) * speed * deltaTime;
        camera->setPosition(newPos);

        // Smooth zoom transition
        const float currentZoom = camera->getZoom();
        const float newZoom = currentZoom + (targetZoom - currentZoom) * speed * deltaTime;
        camera->setZoom(newZoom);
    }
} // namespace engine::camera
