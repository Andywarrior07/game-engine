/**
 * @file ThirdPersonCamera.h
 * @brief Third-person camera preset
 * @author Andrés Guerrero
 * @date 26-08-2025
 */

#pragma once

#include "../cameras/Camera3D.h"
#include "../manager/CameraManager.h"

namespace engine::camera {
    /**
     * @brief Third-person camera configuration
     */
    struct ThirdPersonCameraConfig {
        Vector3 targetPosition{0.0f, 0.0f, 0.0f}; ///< Initial target position
        float distance = 5.0f; ///< Distance from target
        float minDistance = 1.0f; ///< Minimum zoom distance
        float maxDistance = 20.0f; ///< Maximum zoom distance
        float height = 2.0f; ///< Height offset from target
        float minHeight = 0.5f; ///< Minimum camera height
        float maxHeight = 10.0f; ///< Maximum camera height
        float followSpeed = 8.0f; ///< Camera follow speed
        float rotationSpeed = 2.0f; ///< Rotation speed
        float fov = 60.0f; ///< Field of view
        Vector3 targetOffset{0.0f, 1.0f, 0.0f}; ///< Offset from target center
        bool enableCollision = true; ///< Enable collision detection
        float collisionRadius = 0.5f; ///< Collision sphere radius
        bool autoRotate = false; ///< Auto-rotate behind target
        float autoRotateSpeed = 2.0f; ///< Auto-rotation speed
        bool lockOnTarget = false; ///< Lock camera to always face target
    };

    /**
     * @brief Third-person camera preset
     *
     * Provides a pre-configured camera setup for third-person games with:
     * - Orbital controls around character
     * - Zoom in/out functionality
     * - Collision detection
     * - Auto-rotation options
     * - Smooth following
     */
    class ThirdPersonCamera {
    public:
        /**
         * @brief Create and configure a third-person camera
         * @param manager Camera manager reference
         * @param config Third-person camera configuration
         * @param name Camera name
         * @return Camera ID of created camera
         */
        static CameraID create(CameraManager& manager,
                               const ThirdPersonCameraConfig& config,
                               const std::string& name);

        /**
         * @brief Setup existing camera as third-person camera
         * @param camera Camera to configure
         * @param config Third-person camera configuration
         */
        static void setup(Camera3D* camera, const ThirdPersonCameraConfig& config);

        // /**
        //  * @brief Setup input bindings for third-person camera
        //  * @param input Input handler
        //  * @param config Third-person camera configuration
        //  */
        // static void setupInput(CameraInputHandler* input, const ThirdPersonCameraConfig& config = {}) {
        //     if (!input) return;
        //
        //     // Mouse for camera rotation
        //     auto inputConfig = input->getConfig();
        //     inputConfig.mouseSensitivity = config.rotationSpeed;
        //     input->setConfig(inputConfig);
        //
        //     // Zoom controls
        //     input->bindKey('=', CameraInputAction::ZOOM_IN);   // Plus key
        //     input->bindKey('-', CameraInputAction::ZOOM_OUT);  // Minus key
        //
        //     // Camera height adjustment
        //     input->bindKey('R', CameraInputAction::MOVE_UP);   // Raise camera
        //     input->bindKey('F', CameraInputAction::MOVE_DOWN); // Lower camera
        //
        //     // Reset camera
        //     input->bindKey('T', CameraInputAction::RESET_CAMERA);
        // }

        /**
         * @brief Update camera to follow target
         * @param camera Camera to update
         * @param targetPosition Current target position
         * @param targetForward Target's forward direction
         * @param config Configuration
         * @param deltaTime Time step
         */
        static void updateFollow(Camera3D* camera,
                                 const Vector3& targetPosition,
                                 const Vector3& targetForward,
                                 const ThirdPersonCameraConfig& config,
                                 float deltaTime);

        /**
         * @brief Handle collision detection
         * @param camera Camera to check
         * @param targetPosition Target position
         * @param obstacles Collision geometry (simplified)
         * @param config Configuration
         * @return Adjusted camera position
         */
        static Vector3 handleCollision(const Camera3D* camera,
                                       const Vector3& targetPosition,
                                       const ThirdPersonCameraConfig& config);

        /**
         * @brief Adjust camera distance (zoom)
         * @param camera Camera to adjust
         * @param zoomDelta Zoom change (-1 to 1)
         * @param config Configuration
         */
        static void adjustDistance(Camera3D* camera,
                                   float zoomDelta,
                                   const ThirdPersonCameraConfig& config);

        /**
         * @brief Adjust camera height
         * @param camera Camera to adjust
         * @param heightDelta Height change
         * @param config Configuration
         */
        static void adjustHeight(Camera3D* camera,
                                 float heightDelta,
                                 const ThirdPersonCameraConfig& config);

    private:
        /**
         * @brief Update camera position based on configuration
         * @param camera Camera to update
         * @param config Configuration
         */
        static void updateCameraPosition(Camera3D* camera, const ThirdPersonCameraConfig& config);

        /**
         * @brief Auto-rotate camera behind target
         * @param camera Camera to rotate
         * @param targetForward Target's forward direction
         * @param config Configuration
         * @param deltaTime Time step
         */
        static void autoRotateBehindTarget(Camera3D* camera,
                                           const Vector3& targetForward,
                                           const ThirdPersonCameraConfig& config,
                                           float deltaTime);
    };
} // namespace engine::camera
