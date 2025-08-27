/**
* @file CameraManagerConfig.h
 * @brief Configuration parameters for CameraManager
 * @author Andrés Guerrero
 * @date 26-08-2025
 */

#pragma once

#include "../core/CameraTypes.h"

#include <chrono>

namespace engine::camera {
    /**
     * @brief Configuration parameters for CameraManager
     *
     * Contains all configurable parameters for the camera management system,
     * including limits, defaults, and feature toggles.
     */
    struct CameraManagerConfig {
        // ========================================================================
        // SYSTEM LIMITS
        // ========================================================================

        int maxCameras = 32; ///< Maximum number of cameras
        int maxTransitions = 16; ///< Maximum simultaneous transitions
        int maxShakeEffects = 8; ///< Maximum simultaneous shake effects

        // ========================================================================
        // FEATURE TOGGLES
        // ========================================================================

        bool enableCameraLogging = false; ///< Log camera events for debugging
        bool enableTransitions = true; ///< Enable smooth camera transitions
        bool enableShake = true; ///< Enable camera shake effects
        bool enableInput = true; ///< Enable input processing
        bool enableDebugVisualization = false; ///< Enable debug rendering
        bool enablePerformanceMetrics = false; ///< Track performance metrics
        bool threadSafe = true; ///< Enable thread-safe operations

        // ========================================================================
        // DEFAULT VALUES
        // ========================================================================

        float defaultSmoothingSpeed = defaults::SMOOTHING_SPEED; ///< Default camera smoothing
        float defaultFollowSpeed = defaults::FOLLOW_SPEED; ///< Default follow speed
        float mouseSensitivity = defaults::MOUSE_SENSITIVITY; ///< Default mouse sensitivity
        float scrollSensitivity = defaults::SCROLL_SENSITIVITY; ///< Default scroll sensitivity
        float defaultFOV = defaults::CAMERA_FOV; ///< Default field of view
        float defaultNearPlane = defaults::CAMERA_NEAR_PLANE; ///< Default near plane
        float defaultFarPlane = defaults::CAMERA_FAR_PLANE; ///< Default far plane

        // ========================================================================
        // UPDATE SETTINGS
        // ========================================================================

        std::chrono::milliseconds updateInterval{16}; ///< Update interval (60 FPS default)
        bool fixedTimeStep = false; ///< Use fixed time step
        float maxDeltaTime = 0.1f; ///< Maximum delta time cap

        // ========================================================================
        // MEMORY SETTINGS
        // ========================================================================

        bool preallocateCameras = false; ///< Preallocate camera memory
        bool preallocateTransitions = false; ///< Preallocate transition memory
        std::size_t cameraPoolSize = 8; ///< Initial camera pool size
        std::size_t transitionPoolSize = 4; ///< Initial transition pool size

        // ========================================================================
        // VALIDATION
        // ========================================================================

        /**
         * @brief Validate configuration
         * @return true if configuration is valid
         */
        bool validate() const {
            return maxCameras > 0 &&
                maxTransitions > 0 &&
                maxShakeEffects > 0 &&
                defaultSmoothingSpeed > 0.0f &&
                defaultFollowSpeed > 0.0f &&
                mouseSensitivity > 0.0f &&
                scrollSensitivity > 0.0f &&
                defaultFOV > 0.0f && defaultFOV < 180.0f &&
                defaultNearPlane > 0.0f &&
                defaultFarPlane > defaultNearPlane &&
                maxDeltaTime > 0.0f &&
                updateInterval.count() > 0;
        }

        // ========================================================================
        // PRESET CONFIGURATIONS
        // ========================================================================

        /**
         * @brief Create default configuration
         * @return Default configuration
         */
        static CameraManagerConfig defaultConfig() {
            return CameraManagerConfig{};
        }

        /**
         * @brief Create configuration for FPS games
         * @return FPS-optimized configuration
         */
        static CameraManagerConfig fpsConfig() {
            CameraManagerConfig config;
            config.maxCameras = 4;
            config.enableShake = true;
            config.mouseSensitivity = 2.0f;
            config.defaultFOV = 90.0f;
            config.defaultSmoothingSpeed = 10.0f;
            return config;
        }

        /**
         * @brief Create configuration for RTS games
         * @return RTS-optimized configuration
         */
        static CameraManagerConfig rtsConfig() {
            CameraManagerConfig config;
            config.maxCameras = 2;
            config.enableShake = false;
            config.scrollSensitivity = 2.0f;
            config.defaultSmoothingSpeed = 8.0f;
            return config;
        }

        /**
         * @brief Create configuration for platformer games
         * @return Platformer-optimized configuration
         */
        static CameraManagerConfig platformerConfig() {
            CameraManagerConfig config;
            config.maxCameras = 2;
            config.enableShake = true;
            config.defaultFollowSpeed = 8.0f;
            config.defaultSmoothingSpeed = 5.0f;
            return config;
        }

        /**
         * @brief Create configuration for cinematic cameras
         * @return Cinematic-optimized configuration
         */
        static CameraManagerConfig cinematicConfig() {
            CameraManagerConfig config;
            config.maxCameras = 16;
            config.maxTransitions = 32;
            config.enableTransitions = true;
            config.defaultSmoothingSpeed = 2.0f;
            config.defaultFOV = 45.0f;
            return config;
        }

        /**
         * @brief Create minimal configuration for testing
         * @return Minimal configuration
         */
        static CameraManagerConfig minimalConfig() {
            CameraManagerConfig config;
            config.maxCameras = 1;
            config.maxTransitions = 1;
            config.maxShakeEffects = 1;
            config.enableCameraLogging = true;
            config.enablePerformanceMetrics = false;
            config.threadSafe = false;
            return config;
        }
    };
} // namespace engine::camera
