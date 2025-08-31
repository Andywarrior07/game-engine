/**
 * @file CameraTypes.h
 * @brief Core type definitions, enums, and constants for the camera system
 * @author Andrés Guerrero
 * @date 23-08-2025
 */

#pragma once

#include "../../math/core/MathTypes.h"

#include <functional>

namespace engine::camera {
    // ========================================================================
    // TYPE ALIASES
    // ========================================================================

    /**
     * @brief Math type aliases from engine math system
     */
    using Vec2 = math::Vec2;
    using Vec3 = math::Vec3;
    using Vec4 = math::Vec4;
    using Mat4 = math::Mat4;

    /**
     * @brief Strong typing for camera identifiers
     * Prevents mixing with other ID types
     */
    using CameraID = std::uint32_t;

    /**
     * @brief Strong typing for transition identifiers
     */
    using TransitionID = std::uint32_t;

    // ========================================================================
    // CONSTANTS
    // ========================================================================

    /**
     * @brief Invalid ID constant for error checking
     */
    constexpr CameraID INVALID_CAMERA_ID = 0;

    /**
     * @brief Invalid transition ID constant
     */
    constexpr TransitionID INVALID_TRANSITION_ID = 0;

    /**
     * @brief Default camera configuration values
     */
    namespace defaults {
        constexpr float CAMERA_FOV = 60.0f; ///< Default field of view
        constexpr float CAMERA_NEAR_PLANE = 0.1f; ///< Default near clipping plane
        constexpr float CAMERA_FAR_PLANE = 1000.0f; ///< Default far clipping plane
        constexpr float CAMERA_ZOOM = 1.0f; ///< Default zoom level
        constexpr float SMOOTHING_SPEED = 5.0f; ///< Default smoothing speed
        constexpr float FOLLOW_SPEED = 5.0f; ///< Default follow speed
        constexpr float MOUSE_SENSITIVITY = 1.0f; ///< Default mouse sensitivity
        constexpr float SCROLL_SENSITIVITY = 1.0f; ///< Default scroll sensitivity
        constexpr float ORBITAL_RADIUS = 5.0f; ///< Default orbital radius
        constexpr float ORBITAL_SPEED = 1.0f; ///< Default orbital speed
        constexpr float MIN_PITCH = -89.0f; ///< Default minimum pitch angle
        constexpr float MAX_PITCH = 89.0f; ///< Default maximum pitch angle
        constexpr float MIN_ZOOM = 0.1f; ///< Default minimum zoom
        constexpr float MAX_ZOOM = 10.0f; ///< Default maximum zoom
    } // namespace defaults

    // ========================================================================
    // ENUMERATIONS
    // ========================================================================

    /**
     * @brief Camera types supported by the system
     */
    enum class CameraType : std::uint8_t {
        CAMERA_2D = 0, ///< 2D orthographic camera for sprites and UI
        CAMERA_3D_PERSPECTIVE = 1, ///< 3D perspective camera for 3D scenes
        CAMERA_3D_ORTHOGRAPHIC = 2 ///< 3D orthographic camera for isometric/technical
    };

    /**
     * @brief Camera movement modes for different game genres
     */
    enum class CameraMode : std::uint8_t {
        STATIC = 0, ///< Fixed position and rotation
        FREE_LOOK = 1, ///< FPS-style free look camera
        FOLLOW_TARGET = 2, ///< Third-person follow camera
        ORBITAL = 3, ///< Orbit around a target point
        SIDE_SCROLLER = 4, ///< 2D side-scrolling camera
        TOP_DOWN = 5, ///< 2D top-down camera
        ISOMETRIC = 6, ///< Isometric 3D camera
        CINEMATIC = 7 ///< Cinematic camera with predefined paths
    };

    /**
     * @brief Camera transition types for smooth movement
     */
    enum class TransitionType : std::uint8_t {
        LINEAR = 0, ///< Linear interpolation
        EASE_IN = 1, ///< Ease in (slow start)
        EASE_OUT = 2, ///< Ease out (slow end)
        EASE_IN_OUT = 3, ///< Ease in and out
        BOUNCE = 4, ///< Bounce effect at end
        ELASTIC = 5 ///< Elastic/spring effect
    };

    /**
     * @brief Camera projection types
     */
    enum class ProjectionType : std::uint8_t {
        PERSPECTIVE = 0, ///< Perspective projection (3D depth)
        ORTHOGRAPHIC = 1 ///< Orthographic projection (no depth distortion)
    };

    // ========================================================================
    // CALLBACK TYPES
    // ========================================================================

    /**
     * @brief Callback function type for camera position changes
     * @param cameraId ID of the camera that changed
     * @param position New camera position
     * @param target New camera target (for 3D cameras)
     */
    using CameraCallback = std::function<void(CameraID cameraId, const Vec3& position, const Vec3& target)>;

    /**
     * @brief Callback function type for transition completion
     * @param cameraId ID of the camera that was transitioned
     * @param transitionId ID of the completed transition
     * @param completed Whether the transition completed successfully
     */
    using TransitionCallback = std::function<void(CameraID cameraId, TransitionID transitionId, bool completed)>;

    /**
     * @brief Callback function type for shake effects
     * @param cameraId ID of the camera being shaken
     * @param intensity Current shake intensity
     * @param completed Whether the shake effect has completed
     */
    using ShakeCallback = std::function<void(CameraID cameraId, float intensity, bool completed)>;

    // ========================================================================
    // UTILITY FUNCTIONS
    // ========================================================================

    /**
     * @brief Convert camera type to string for debugging
     * @param type Camera type to convert
     * @return String representation of camera type
     */
    inline const char* cameraTypeToString(const CameraType type) {
        switch (type) {
        case CameraType::CAMERA_2D: return "2D";
        case CameraType::CAMERA_3D_PERSPECTIVE: return "3D_Perspective";
        case CameraType::CAMERA_3D_ORTHOGRAPHIC: return "3D_Orthographic";
        default: return "Unknown";
        }
    }

    /**
     * @brief Convert camera mode to string for debugging
     * @param mode Camera mode to convert
     * @return String representation of camera mode
     */
    inline const char* cameraModeToString(const CameraMode mode) {
        switch (mode) {
        case CameraMode::STATIC: return "Static";
        case CameraMode::FREE_LOOK: return "FreeLook";
        case CameraMode::FOLLOW_TARGET: return "FollowTarget";
        case CameraMode::ORBITAL: return "Orbital";
        case CameraMode::SIDE_SCROLLER: return "SideScroller";
        case CameraMode::TOP_DOWN: return "TopDown";
        case CameraMode::ISOMETRIC: return "Isometric";
        case CameraMode::CINEMATIC: return "Cinematic";
        default: return "Unknown";
        }
    }

    /**
     * @brief Convert transition type to string for debugging
     * @param type Transition type to convert
     * @return String representation of transition type
     */
    inline const char* transitionTypeToString(const TransitionType type) {
        switch (type) {
        case TransitionType::LINEAR: return "Linear";
        case TransitionType::EASE_IN: return "EaseIn";
        case TransitionType::EASE_OUT: return "EaseOut";
        case TransitionType::EASE_IN_OUT: return "EaseInOut";
        case TransitionType::BOUNCE: return "Bounce";
        case TransitionType::ELASTIC: return "Elastic";
        default: return "Unknown";
        }
    }
} // namespace engine::camera
