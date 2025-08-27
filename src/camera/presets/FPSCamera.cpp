/**
* @file FPSCamera.cpp
 * @brief First-person shooter camera preset
 * @author Andrés Guerrero
 * @date 26-08-2025
 */

#include "FPSCamera.h"

namespace engine::camera {
    CameraID FPSCamera::create(CameraManager& manager,
                               const FPSCameraConfig& config = {},
                               const std::string& name = "FPSCamera") {
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

    void FPSCamera::setup(Camera3D* camera, const FPSCameraConfig& config = {}) {
        if (!camera) return;

        // Set camera mode and position
        camera->setMode(CameraMode::FREE_LOOK);
        camera->setPosition(config.position);
        camera->setPerspective(true);

        // Configure projection
        camera->setFOV(config.fov);
        camera->setClippingPlanes(0.01f, 1000.0f); // Close near plane for weapon rendering

        // Set pitch limits to prevent over-rotation
        camera->setPitchLimits(-85.0f, 85.0f);

        // Initial look direction (forward)
        camera->setYaw(-90.0f);
        camera->setPitch(0.0f);
    }

    void FPSCamera::updateFeatures(Camera3D* camera,
                                   const FPSCameraConfig& config,
                                   const float deltaTime,
                                   const bool isMoving = false,
                                   const bool isSprinting = false) {
        if (!camera) return;

        // Head bob effect
        if (config.enableHeadBob && isMoving) {
            applyHeadBob(camera, config, deltaTime, isSprinting);
        }

        // FOV changes for sprint/ADS
        float targetFOV = config.fov;
        if (isSprinting) {
            targetFOV = config.fovSprint;
        }

        // Smooth FOV transition
        const float currentFOV = camera->getFOV();
        const float newFOV = currentFOV + (targetFOV - currentFOV) * deltaTime * 5.0f;
        camera->setFOV(newFOV);
    }

    void FPSCamera::applyHeadBob(Camera3D* camera,
                                 const FPSCameraConfig& config,
                                 const float deltaTime,
                                 const bool isSprinting) {
        headBobTime_ += deltaTime * config.headBobFrequency * (isSprinting ? 1.5f : 1.0f);

        Vector3 position = camera->getPosition();

        // Vertical bob (sine wave)
        const float bobY = std::sin(headBobTime_ * 2.0f) * config.headBobIntensity;

        // Horizontal bob (figure-8 pattern)
        const float bobX = std::sin(headBobTime_) * config.headBobIntensity * 0.5f;

        // Apply bob relative to camera orientation
        const Vector3 right = camera->getRight();
        const Vector3 up = camera->getUp();

        position += right * bobX + up * bobY;
        camera->setPosition(position);
    }
} // namespace engine::camera
