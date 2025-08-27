/**
 * @file IsometricCamera.h
 * @brief Isometric camera preset for strategy and RPG games
 * @author Andrés Guerrero
 * @date 26-08-2025
 */

#pragma once

#include "../cameras/Camera3D.h"
#include "../manager/CameraManager.h"

namespace engine::camera {
    /**
     * @brief Isometric camera configuration
     */
    struct IsometricCameraConfig {
        Vector3 position{10.0f, 10.0f, 10.0f}; ///< Camera position
        Vector3 target{0.0f, 0.0f, 0.0f}; ///< Look-at target
        float orthographicSize = 10.0f; ///< Orthographic projection size
        float minSize = 5.0f; ///< Minimum zoom size
        float maxSize = 50.0f; ///< Maximum zoom size
        float angle = 45.0f; ///< Isometric angle (degrees)
        float rotation = 45.0f; ///< Rotation around Y axis
        float panSpeed = 10.0f; ///< Pan movement speed
        float zoomSpeed = 2.0f; ///< Zoom speed
        float smoothing = 8.0f; ///< Camera smoothing
        bool enableGridSnap = false; ///< Snap to grid positions
        float gridSize = 1.0f; ///< Grid cell size
        bool enableEdgePan = true; ///< Enable edge panning
        float edgePanSpeed = 15.0f; ///< Edge pan speed
        float edgePanZone = 30.0f; ///< Edge pan zone size
        bool lockRotation = true; ///< Lock camera rotation
        CameraBounds worldBounds; ///< World boundary limits
    };

    /**
     * @brief Isometric camera preset
     *
     * Provides a pre-configured camera setup for isometric games with:
     * - Fixed isometric angle
     * - Orthographic projection
     * - Grid-based movement
     * - RTS-style controls
     * - Smooth panning and zooming
     */
    class IsometricCamera {
    public:
        /**
         * @brief Create and configure an isometric camera
         * @param manager Camera manager reference
         * @param config Isometric camera configuration
         * @param name Camera name
         * @return Camera ID of created camera
         */
        static CameraID create(CameraManager& manager,
                               const IsometricCameraConfig& config,
                               const std::string& name);

        /**
         * @brief Setup existing camera as isometric camera
         * @param camera Camera to configure
         * @param config Isometric camera configuration
         */
        static void setup(Camera3D* camera, const IsometricCameraConfig& config);

        // /**
        //  * @brief Setup input bindings for isometric camera
        //  * @param input Input handler
        //  * @param config Isometric camera configuration
        //  */
        // static void setupInput(CameraInputHandler* input, const IsometricCameraConfig& config = {}) {
        //     if (!input) return;
        //
        //     // WASD or Arrow keys for panning
        //     input->bindKey('W', CameraInputAction::MOVE_FORWARD);
        //     input->bindKey('S', CameraInputAction::MOVE_BACKWARD);
        //     input->bindKey('A', CameraInputAction::MOVE_LEFT);
        //     input->bindKey('D', CameraInputAction::MOVE_RIGHT);
        //
        //     // Q/E for rotation (if not locked)
        //     if (!config.lockRotation) {
        //         input->bindKey('Q', CameraInputAction::LOOK_LEFT);
        //         input->bindKey('E', CameraInputAction::LOOK_RIGHT);
        //     }
        //
        //     // Mouse wheel for zoom
        //     // Handled through processZoom
        //
        //     // Configure speeds
        //     auto inputConfig = input->getConfig();
        //     inputConfig.moveSpeed = config.panSpeed;
        //     inputConfig.scrollSensitivity = config.zoomSpeed * 0.1f;
        //     input->setConfig(inputConfig);
        // }

        /**
         * @brief Pan camera in isometric space
         * @param camera Camera to pan
         * @param direction Pan direction (screen space)
         * @param config Configuration
         * @param deltaTime Time step
         */
        static void pan(Camera3D* camera,
                        const Vector2& direction,
                        const IsometricCameraConfig& config,
                        float deltaTime);

        /**
         * @brief Zoom isometric camera
         * @param camera Camera to zoom
         * @param zoomDelta Zoom change (-1 to 1)
         * @param config Configuration
         */
        static void zoom(Camera3D* camera,
                         float zoomDelta,
                         const IsometricCameraConfig& config);

        /**
         * @brief Rotate isometric camera around target
         * @param camera Camera to rotate
         * @param angleDelta Rotation angle delta (degrees)
         * @param config Configuration
         */
        static void rotate(Camera3D* camera,
                           float angleDelta,
                           const IsometricCameraConfig& config);

        /**
         * @brief Focus camera on world position
         * @param camera Camera to focus
         * @param worldPosition Position to focus on
         * @param instant Whether to snap instantly or transition
         */
        static void focusOn(Camera3D* camera,
                            const Vector3& worldPosition,
                            bool instant);

        /**
         * @brief Handle edge panning
         * @param camera Camera to pan
         * @param mousePos Mouse position in screen space
         * @param screenSize Screen dimensions
         * @param config Configuration
         * @param deltaTime Time step
         */
        static void handleEdgePan(Camera3D* camera,
                                  const Vector2& mousePos,
                                  const Vector2& screenSize,
                                  const IsometricCameraConfig& config,
                                  float deltaTime);

        /**
         * @brief Convert screen position to world position
         * @param camera Isometric camera
         * @param screenPos Screen position
         * @param viewport Current viewport
         * @param groundHeight Y coordinate of ground plane
         * @return World position on ground plane
         */
        static Vector3 screenToWorld(const Camera3D* camera,
                                     const Vector2& screenPos,
                                     const Viewport& viewport,
                                     float groundHeight);

        /**
         * @brief Get grid cell from world position
         * @param worldPos World position
         * @param gridSize Size of grid cells
         * @return Grid coordinates
         */
        static Vector2 worldToGrid(const Vector3& worldPos, const float gridSize);

        /**
         * @brief Get world position from grid cell
         * @param gridPos Grid coordinates
         * @param gridSize Size of grid cells
         * @param height Y coordinate
         * @return World position at grid cell center
         */
        static Vector3 gridToWorld(const Vector2& gridPos, float gridSize, float height);

    private:
        /**
         * @brief Calculate isometric camera position
         * @param target Look-at target
         * @param angle Isometric angle (degrees)
         * @param rotation Rotation around Y axis (degrees)
         * @param distance Distance from target
         * @return Camera position
         */
        static Vector3 calculateIsometricPosition(const Vector3& target,
                                                  float angle,
                                                  float rotation,
                                                  float distance);

        /**
         * @brief Snap movement to grid
         * @param movement Movement vector
         * @param gridSize Grid cell size
         * @return Snapped movement
         */
        static Vector3 snapToGrid(const Vector3& movement, float gridSize);
    };
} // namespace engine::camera
