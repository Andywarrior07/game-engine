/**
 * @file SideScrollerCamera.h
 * @brief Side-scrolling camera preset for platformers
 * @author Andrés Guerrero
 * @date 26-08-2025
 */

#pragma once

#include "../cameras/Camera2D.h"
#include "../manager/CameraManager.h"

namespace engine::camera {

    /**
     * @brief Side-scroller camera configuration
     */
    struct SideScrollerCameraConfig {
        Vec2 position{0.0f, 0.0f};           ///< Initial camera position
        float zoom = 1.0f;                      ///< Zoom level
        float followSpeed = 10.0f;              ///< Horizontal follow speed
        float verticalFollowSpeed = 5.0f;       ///< Vertical follow speed
        float lookAheadDistance = 100.0f;       ///< Look-ahead distance
        float verticalDeadzone = 50.0f;         ///< Vertical deadzone height
        float horizontalOffset = 0.0f;          ///< Horizontal offset from target
        float verticalOffset = -30.0f;          ///< Vertical offset from target
        bool lockVertical = false;              ///< Lock vertical movement
        float minY = -1000.0f;                  ///< Minimum Y position
        float maxY = 1000.0f;                   ///< Maximum Y position
        bool enableParallax = true;             ///< Enable parallax scrolling
        float smoothTime = 0.3f;                ///< Smooth damp time
        bool enableScreenShake = true;          ///< Enable screen shake support
        bool enableDynamicZoom = false;         ///< Enable speed-based zoom
        float minDynamicZoom = 0.8f;           ///< Minimum dynamic zoom
        float maxDynamicZoom = 1.2f;           ///< Maximum dynamic zoom
    };

    /**
     * @brief Side-scrolling camera preset
     *
     * Provides a pre-configured camera setup for side-scrolling games with:
     * - Smooth horizontal following
     * - Vertical deadzone
     * - Look-ahead based on velocity
     * - Platform snapping
     * - Parallax support
     */
    class SideScrollerCamera {
    public:
        /**
         * @brief Create and configure a side-scroller camera
         * @param manager Camera manager reference
         * @param config Side-scroller camera configuration
         * @param name Camera name
         * @return Camera ID of created camera
         */
        static CameraID create(CameraManager& manager,
                              const SideScrollerCameraConfig& config,
                              const std::string& name);

        /**
         * @brief Setup existing camera as side-scroller camera
         * @param camera Camera to configure
         * @param config Side-scroller camera configuration
         */
        static void setup(Camera2D* camera, const SideScrollerCameraConfig& config);

        /**
         * @brief Update camera to follow player
         * @param camera Camera to update
         * @param playerPosition Player position
         * @param playerVelocity Player velocity
         * @param config Configuration
         * @param deltaTime Time step
         */
        static void followPlayer(Camera2D* camera,
                                const Vec2& playerPosition,
                                const Vec2& playerVelocity,
                                const SideScrollerCameraConfig& config,
                                float deltaTime);

        /**
         * @brief Update dynamic zoom based on player speed
         * @param camera Camera to update
         * @param playerSpeed Current player speed
         * @param maxSpeed Maximum player speed
         * @param config Configuration
         * @param deltaTime Time step
         */
        static void updateDynamicZoom(Camera2D* camera,
                                     float playerSpeed,
                                     float maxSpeed,
                                     const SideScrollerCameraConfig& config,
                                     float deltaTime);

        /**
         * @brief Handle camera bounds and platform transitions
         * @param camera Camera to update
         * @param platformBounds Current platform/room bounds
         * @param transitionSpeed Speed of bound transitions
         * @param deltaTime Time step
         */
        static void handlePlatformBounds(Camera2D* camera,
                                        const CameraBounds& platformBounds,
                                        float transitionSpeed,
                                        float deltaTime);

        /**
         * @brief Setup parallax layer offsets
         * @param camera Main camera
         * @param layerDepths Array of layer depths (0=far, 1=near)
         * @param layerCount Number of parallax layers
         * @return Calculated offsets for each layer
         */
        static std::vector<Vec2> calculateParallaxOffsets(const Camera2D* camera,
                                                            const float* layerDepths,
                                                            int layerCount);

        /**
         * @brief Handle room/section transitions
         * @param manager Camera manager
         * @param cameraId Camera to transition
         * @param newRoomCenter Center of new room
         * @param newRoomSize Size of new room
         * @param transitionDuration Transition duration
         * @return Transition ID for tracking
         */
        static TransitionID transitionToRoom(CameraManager& manager,
                                            CameraID cameraId,
                                            const Vec2& newRoomCenter,
                                            const Vec2& newRoomSize,
                                            float transitionDuration);

        /**
         * @brief Apply screen shake for impacts
         * @param manager Camera manager
         * @param cameraId Camera to shake
         * @param intensity Shake intensity
         * @param duration Shake duration
         */
        static void applyImpactShake(CameraManager& manager,
                                    CameraID cameraId,
                                    float intensity,
                                    float duration);
    };

} // namespace engine::camera