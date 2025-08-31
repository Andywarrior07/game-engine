/**
 * @file TopDownCamera.h
 * @brief Top-down camera preset for 2D and isometric games
 * @author Andrés Guerrero
 * @date 26-08-2025
 */

#pragma once

#include "../cameras/Camera2D.h"
#include "../manager/CameraManager.h"

namespace engine::camera {

    /**
     * @brief Top-down camera configuration
     */
    struct TopDownCameraConfig {
        Vec2 position{0.0f, 0.0f};           ///< Initial camera position
        float zoom = 1.0f;                      ///< Initial zoom level
        float minZoom = 0.5f;                   ///< Minimum zoom level
        float maxZoom = 3.0f;                   ///< Maximum zoom level
        float followSpeed = 8.0f;               ///< Target follow speed
        float smoothing = 5.0f;                 ///< Camera smoothing
        Vec2 deadzone{50.0f, 50.0f};        ///< Follow deadzone size
        bool enableBounds = false;              ///< Enable world bounds
        CameraBounds worldBounds;               ///< World boundary limits
        bool enableEdgeScrolling = true;        ///< Enable edge scrolling
        float edgeScrollSpeed = 500.0f;         ///< Edge scroll speed (pixels/sec)
        float edgeScrollZone = 20.0f;           ///< Edge scroll zone size (pixels)
        bool enableDragPan = true;              ///< Enable mouse drag panning
        float dragSensitivity = 1.0f;           ///< Drag pan sensitivity
    };

    /**
     * @brief Top-down camera preset
     *
     * Provides a pre-configured camera setup for top-down games with:
     * - Smooth target following
     * - Zoom controls
     * - Edge scrolling
     * - Drag panning
     * - Deadzone following
     */
    class TopDownCamera {
    public:
        /**
         * @brief Create and configure a top-down camera
         * @param manager Camera manager reference
         * @param config Top-down camera configuration
         * @param name Camera name
         * @return Camera ID of created camera
         */
        static CameraID create(CameraManager& manager,
                              const TopDownCameraConfig& config,
                              const std::string& name);

        /**
         * @brief Setup existing camera as top-down camera
         * @param camera Camera to configure
         * @param config Top-down camera configuration
         */
        static void setup(Camera2D* camera, const TopDownCameraConfig& config);

        // /**
        //  * @brief Setup input bindings for top-down camera
        //  * @param input Input handler
        //  * @param config Top-down camera configuration
        //  */
        // static void setupInput(CameraInputHandler* input, const TopDownCameraConfig& config = {}) {
        //     if (!input) return;
        //
        //     // Arrow keys for camera movement
        //     input->bindKey(0x26, CameraInputAction::MOVE_FORWARD);  // Up arrow
        //     input->bindKey(0x28, CameraInputAction::MOVE_BACKWARD); // Down arrow
        //     input->bindKey(0x25, CameraInputAction::MOVE_LEFT);     // Left arrow
        //     input->bindKey(0x27, CameraInputAction::MOVE_RIGHT);    // Right arrow
        //
        //     // WASD alternative
        //     input->bindKey('W', CameraInputAction::MOVE_FORWARD);
        //     input->bindKey('S', CameraInputAction::MOVE_BACKWARD);
        //     input->bindKey('A', CameraInputAction::MOVE_LEFT);
        //     input->bindKey('D', CameraInputAction::MOVE_RIGHT);
        //
        //     // Zoom controls
        //     input->bindKey('Q', CameraInputAction::ZOOM_IN);
        //     input->bindKey('E', CameraInputAction::ZOOM_OUT);
        //
        //     // Reset camera
        //     input->bindKey('R', CameraInputAction::RESET_CAMERA);
        //
        //     // Configure sensitivity
        //     auto inputConfig = input->getConfig();
        //     inputConfig.moveSpeed = config.edgeScrollSpeed;
        //     inputConfig.scrollSensitivity = 0.1f; // Zoom sensitivity
        //     input->setConfig(inputConfig);
        // }

        /**
         * @brief Update camera to follow target with deadzone
         * @param camera Camera to update
         * @param targetPosition Target position to follow
         * @param config Configuration
         * @param deltaTime Time step
         */
        static void followWithDeadzone(Camera2D* camera,
                                       const Vec2& targetPosition,
                                       const TopDownCameraConfig& config,
                                       float deltaTime);

        /**
         * @brief Handle edge scrolling
         * @param camera Camera to scroll
         * @param mousePos Mouse position in screen space
         * @param screenSize Screen dimensions
         * @param config Configuration
         * @param deltaTime Time step
         */
        static void handleEdgeScrolling(Camera2D* camera,
                                       const Vec2& mousePos,
                                       const Vec2& screenSize,
                                       const TopDownCameraConfig& config,
                                       float deltaTime);

        /**
         * @brief Handle drag panning
         * @param camera Camera to pan
         * @param mouseDelta Mouse movement delta
         * @param isDragging Whether mouse is being dragged
         * @param config Configuration
         */
        static void handleDragPan(Camera2D* camera,
                                 const Vec2& mouseDelta,
                                 bool isDragging,
                                 const TopDownCameraConfig& config);

        /**
         * @brief Focus camera on area
         * @param camera Camera to adjust
         * @param bounds Area to focus on
         * @param viewport Current viewport
         * @param padding Padding around area (0-1)
         */
        static void focusOnArea(Camera2D* camera,
                               const CameraBounds& bounds,
                               const Viewport& viewport,
                               float padding);

        /**
         * @brief Smoothly transition to target
         * @param camera Camera to transition
         * @param targetPosition Target position
         * @param targetZoom Target zoom
         * @param speed Transition speed
         * @param deltaTime Time step
         */
        static void smoothTransition(Camera2D* camera,
                                    const Vec2& targetPosition,
                                    float targetZoom,
                                    float speed,
                                    float deltaTime);
    };

} // namespace engine::camera