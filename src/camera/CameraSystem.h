/**
 * @file CameraSystem.h
 * @brief Main header file for the Camera System
 * @author Andrés Guerrero
 * @date 23-08-2025
 *
 * This is the main include file for the camera system.
 * Include this file to get acces to all camera functionality
 */

#pragma once

// Core types and definitions
#include "core/CameraTypes.h"
#include "core/CameraBounds.h"

// Base and concrete camera implementations
#include "cameras/BaseCamera.h"
#include "cameras/Camera2D.h"
#include "cameras/Camera3D.h"

// Camera effects
#include "effects/CameraShake.h"
#include "effects/ShakeConfig.h"
#include "effects/ShakePattern.h"

// Transitions
#include "transitions/CameraTRansition.h"
#include "transitions/TransitionConfig.h"
#include "transitions/TransitionEasing.h"

// Input handling
// #include "../camera/input/CameraInputHandler.h"

// Camera management
#include "manager/CameraManager.h"
#include "manager/CameraManagerConfig.h"
#include "manager/CameraRegistry.h"

// Preset cameras
#include "presets/FPSCamera.h"
#include "presets/ThirdPersonCamera.h"
#include "presets/TopDownCamera.h"
#include "presets/SideScrollerCamera.h"
#include "presets/IsometricCamera.h"

// Utilities
#include "utils/CoordinateTransform.h"
#include "utils/CameraDebugger.h"

/**
 * @namespace engine::camera
 * @brief Complete camera system for 2D and 3D games
 *
 * The camera system provides:
 * - Multiple camera support with 2D and 3D implementations
 * - Smooth transitions and camera effects
 * - Input handling for camera controls
 * - Preset camera configurations for common game types
 * - Thread-safe camera management
 * - Coordinate transformation utilities
 */
namespace engine::camera {
    /**
     * @brief Camera system version information
     */
    constexpr const char* CAMERA_SYSTEM_VERSION = "1.0.0";

    /**
     * @brief Camera system build date
     */
    constexpr const char* CAMERA_SYSTEM_BUILD_DATE = __DATE__;
}
