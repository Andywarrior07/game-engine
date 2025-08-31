/**
 * @file FPSCamera.h
 * @brief First-person shooter camera preset
 * @author Andrés Guerrero
 * @date 26-08-2025
 */

#pragma once

#include "../cameras/Camera3D.h"
#include "../manager/CameraManager.h"

namespace engine::camera {
    /**
     * @brief FPS camera preset configuration
     */
    struct FPSCameraConfig {
        Vec3 position{0.0f, 1.7f, 0.0f}; ///< Initial position (eye height)
        float mouseSensitivity = 2.0f; ///< Mouse look sensitivity
        float moveSpeed = 5.0f; ///< Movement speed (m/s)
        float sprintMultiplier = 2.0f; ///< Sprint speed multiplier
        float crouchHeight = 0.9f; ///< Crouch eye height
        float standHeight = 1.7f; ///< Standing eye height
        float fov = 90.0f; ///< Field of view
        float fovSprint = 100.0f; ///< FOV when sprinting
        float fovADS = 45.0f; ///< FOV when aiming down sights
        float headBobIntensity = 0.1f; ///< Head bob intensity
        float headBobFrequency = 2.0f; ///< Head bob frequency
        bool enableHeadBob = true; ///< Enable head bobbing
        bool enableLean = true; ///< Enable leaning
        float leanAngle = 15.0f; ///< Maximum lean angle
        float leanDistance = 0.3f; ///< Lean distance
    };

    /**
     * @brief First-person shooter camera preset
     *
     * Provides a pre-configured camera setup for FPS games with:
     * - Mouse look controls
     * - WASD movement
     * - Sprint, crouch, and lean mechanics
     * - Head bobbing
     * - Aim down sights (ADS) support
     */
    class FPSCamera {
    public:
        /**
         * @brief Create and configure an FPS camera
         * @param manager Camera manager reference
         * @param config FPS camera configuration
         * @param name Camera name
         * @return Camera ID of created FPS camera
         */
        static CameraID create(CameraManager& manager,
                               const FPSCameraConfig& config,
                               const std::string& name);

        /**
         * @brief Setup existing camera as FPS camera
         * @param camera Camera to configure
         * @param config FPS camera configuration
         */
        static void setup(Camera3D* camera, const FPSCameraConfig& config);

        // /**
        //  * @brief Setup input bindings for FPS camera
        //  * @param input Input handler
        //  * @param config FPS camera configuration
        //  */
        // static void setupInput(CameraInputHandler* input, const FPSCameraConfig& config = {}) {
        //     if (!input) return;
        //
        //     // Clear existing bindings
        //     input->clearBindings();
        //
        //     // Movement keys (WASD)
        //     input->bindKey('W', CameraInputAction::MOVE_FORWARD);
        //     input->bindKey('S', CameraInputAction::MOVE_BACKWARD);
        //     input->bindKey('A', CameraInputAction::MOVE_LEFT);
        //     input->bindKey('D', CameraInputAction::MOVE_RIGHT);
        //
        //     // Vertical movement
        //     input->bindKey(' ', CameraInputAction::MOVE_UP); // Space for jump
        //     input->bindKey('C', CameraInputAction::MOVE_DOWN); // C for crouch
        //
        //     // Sprint
        //     input->bindKey(0x10, CameraInputAction::SPEED_UP); // Shift for sprint
        //
        //     // Lean (Q/E)
        //     input->bindKey('Q', CameraInputAction::LOOK_LEFT);
        //     input->bindKey('E', CameraInputAction::LOOK_RIGHT);
        //
        //     // Configure input sensitivity
        //     auto inputConfig = input->getConfig();
        //     inputConfig.mouseSensitivity = config.mouseSensitivity;
        //     inputConfig.moveSpeed = config.moveSpeed;
        //     inputConfig.sprintMultiplier = config.sprintMultiplier;
        //     input->setConfig(inputConfig);
        // }

        /**
         * @brief Update FPS-specific features
         * @param camera Camera to update
         * @param config FPS configuration
         * @param deltaTime Time step
         * @param isMoving Whether player is moving
         * @param isSprinting Whether player is sprinting
         */
        static void updateFeatures(Camera3D* camera,
                                   const FPSCameraConfig& config,
                                   const float deltaTime,
                                   const bool isMoving,
                                   const bool isSprinting);

    private:
        static float headBobTime_; ///< Internal head bob timer

        /**
         * @brief Apply head bob effect
         * @param camera Camera to apply effect to
         * @param config FPS configuration
         * @param deltaTime Time step
         * @param isSprinting Whether player is sprinting
         */
        static void applyHeadBob(Camera3D* camera,
                                 const FPSCameraConfig& config,
                                 const float deltaTime,
                                 const bool isSprinting);
    };

    // Static member initialization
    inline float FPSCamera::headBobTime_ = 0.0f;
} // namespace engine::camera
