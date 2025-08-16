//
// Created by Andres Guerrero on 09-08-25.
//

#pragma once

#include <memory>                   // For smart pointers and memory management
#include <unordered_map>           // For fast hash-based lookups
#include <vector>                  // For dynamic arrays
#include <string>                  // For names and debug info
#include <chrono>                  // For timing and transitions
#include <functional>              // For std::function callbacks
#include <optional>                // For optional return values (C++17)
#include <atomic>                  // For thread-safe operations
#include <mutex>                   // For thread synchronization
#include <array>                   // For fixed-size arrays
#include "../Math/MathTypes.h"     // CAMBIO: Usar nuestro sistema de math unificado

namespace engine::camera {
    // CAMBIO: Usar tipos de GLM a través de nuestro MathTypes.h
    using Vector2 = math::Vector2;
    using Vector3 = math::Vector3;
    using CameraBounds = math::Bounds3D;  // CAMBIO: Usar Bounds3D de MathTypes.h
    using Viewport = math::Viewport;      // CAMBIO: Usar Viewport de MathTypes.h

    // Forward declarations
    class CameraManager;
    class Camera2D;
    class Camera3D;
    struct CameraTransition;

    // Strong typing for camera identifiers - prevents mixing with other ID types
    using CameraID = std::uint32_t;
    using TransitionID = std::uint32_t;

    // Invalid ID constants for error checking
    constexpr CameraID INVALID_CAMERA_ID = 0;
    constexpr TransitionID INVALID_TRANSITION_ID = 0;

    /**
     * @brief Camera types supported by the system
     */
    enum class CameraType : std::uint8_t {
        CAMERA_2D = 0,               // 2D orthographic camera for sprites and UI
        CAMERA_3D_PERSPECTIVE = 1,   // 3D perspective camera for 3D scenes
        CAMERA_3D_ORTHOGRAPHIC = 2   // 3D orthographic camera for isometric/UI
    };

    /**
     * @brief Camera movement modes for different game genres
     */
    enum class CameraMode : std::uint8_t {
        STATIC = 0,                  // Fixed position and rotation
        FREE_LOOK = 1,               // FPS-style free look camera
        FOLLOW_TARGET = 2,           // Third-person follow camera
        ORBITAL = 3,                 // Orbit around a target point
        SIDE_SCROLLER = 4,           // 2D side-scrolling camera
        TOP_DOWN = 5,                // 2D top-down camera
        ISOMETRIC = 6,               // Isometric 3D camera
        CINEMATIC = 7                // Cinematic camera with predefined paths
    };

    /**
     * @brief Camera transition types for smooth movement
     */
    enum class TransitionType : std::uint8_t {
        LINEAR = 0,                  // Linear interpolation
        EASE_IN = 1,                 // Ease in (slow start)
        EASE_OUT = 2,                // Ease out (slow end)
        EASE_IN_OUT = 3,             // Ease in and out
        BOUNCE = 4,                  // Bounce effect
        ELASTIC = 5                  // Elastic effect
    };

    /**
     * @brief Camera shake patterns for different effects
     */
    enum class ShakePattern : std::uint8_t {
        RANDOM = 0,                  // Random shake in all directions
        HORIZONTAL = 1,              // Only horizontal shake
        VERTICAL = 2,                // Only vertical shake
        CIRCULAR = 3,                // Circular shake pattern
        EXPLOSION = 4,               // Explosion-like shake (decreasing intensity)
        EARTHQUAKE = 5               // Earthquake-like shake (low frequency)
    };

    /**
     * @brief Camera shake configuration for screen effects
     */
    struct ShakeConfig {
        float intensity = 1.0f;                            // Shake intensity
        float frequency = 30.0f;                           // Shake frequency in Hz
        float duration = 1.0f;                             // Duration in seconds (-1 for infinite)
        ShakePattern pattern = ShakePattern::RANDOM;       // Shake pattern type
        Vector3 axes{1.0f, 1.0f, 0.5f};                   // Relative shake strength per axis
        bool fadeOut = true;                               // Whether to fade out over time

        ShakeConfig() = default;
        ShakeConfig(float intens, float dur, ShakePattern pat = ShakePattern::RANDOM)
            : intensity(intens), duration(dur), pattern(pat) {}
    };

    /**
     * @brief Callback function types for camera events
     */
    using CameraCallback = std::function<void(CameraID cameraId, const Vector3& position, const Vector3& target)>;
    using TransitionCallback = std::function<void(CameraID cameraId, TransitionID transitionId, bool completed)>;

    /**
     * @brief Camera transition configuration for smooth movement
     */
    struct TransitionConfig {
        float duration = 1.0f;                             // Transition duration in seconds
        TransitionType type = TransitionType::EASE_IN_OUT;  // Easing type
        bool relative = false;                              // Whether target is relative to current position
        TransitionCallback onComplete = nullptr;            // Callback when transition completes

        TransitionConfig() = default;
        TransitionConfig(float dur, TransitionType typ = TransitionType::EASE_IN_OUT)
            : duration(dur), type(typ) {}
    };

    /**
     * @brief Configuration parameters for CameraManager
     */
    struct CameraManagerConfig {
        bool enableCameraLogging = false;                  // Log camera events for debugging
        bool enableTransitions = true;                     // Enable smooth camera transitions
        bool enableShake = true;                           // Enable camera shake effects
        int maxCameras = 32;                               // Maximum number of cameras
        int maxTransitions = 16;                           // Maximum simultaneous transitions
        float defaultSmoothingSpeed = 5.0f;               // Default camera smoothing speed
        float mouseSensitivity = 1.0f;                     // Default mouse sensitivity
        float scrollSensitivity = 1.0f;                    // Default scroll wheel sensitivity
        std::chrono::milliseconds updateInterval{16};     // Update interval for cameras
    };

    /**
     * @brief Base camera class with common functionality
     */
    class BaseCamera {
    public:
        /**
         * @brief Constructor for base camera
         * @param id Unique identifier for this camera
         * @param name Human-readable name for debugging
         * @param type Type of camera (2D or 3D)
         */
        BaseCamera(CameraID id, std::string name, CameraType type);

        /**
         * @brief Virtual destructor for proper cleanup
         */
        virtual ~BaseCamera() = default;

        // Non-copyable but moveable
        BaseCamera(const BaseCamera&) = delete;
        BaseCamera& operator=(const BaseCamera&) = delete;
        BaseCamera(BaseCamera&&) = default;
        BaseCamera& operator=(BaseCamera&&) = default;

        // === Accessors ===

        CameraID getId() const noexcept { return id_; }
        const std::string& getName() const noexcept { return name_; }
        CameraType getType() const noexcept { return type_; }
        CameraMode getMode() const noexcept { return mode_; }
        bool isActive() const noexcept { return active_; }

        /**
         * @brief Get camera position (2D cameras return z=0)
         * @return Current camera position
         */
        virtual Vector3 getPosition() const = 0;

        /**
         * @brief Set camera position
         * @param position New camera position
         */
        virtual void setPosition(const Vector3& position) = 0;

        /**
         * @brief Update camera state
         * @param deltaTime Time elapsed since last update
         */
        virtual void update(float deltaTime) = 0;

        /**
         * @brief Reset camera to default state
         */
        virtual void reset() = 0;

        // === Common Properties ===

        /**
         * @brief Set camera movement mode
         * @param mode New movement mode
         */
        void setMode(CameraMode mode) { mode_ = mode; }

        /**
         * @brief Set camera bounds
         * @param bounds Movement bounds for this camera
         */
        void setBounds(const CameraBounds& bounds) { bounds_ = bounds; }

        /**
         * @brief Get camera bounds
         * @return Current movement bounds
         */
        const CameraBounds& getBounds() const noexcept { return bounds_; }

        /**
         * @brief Set smoothing speed for camera movement
         * @param speed Smoothing speed (higher = more responsive)
         */
        void setSmoothingSpeed(float speed) { smoothingSpeed_ = std::max(0.1f, speed); }

        /**
         * @brief Get smoothing speed
         * @return Current smoothing speed
         */
        float getSmoothingSpeed() const noexcept { return smoothingSpeed_; }

        /**
         * @brief Set camera callback for position changes
         * @param callback Function to call when camera moves
         */
        void setCallback(const CameraCallback& callback) { callback_ = callback; }

        /**
         * @brief Enable or disable this camera
         * @param enabled Whether camera should be enabled
         */
        void setEnabled(bool enabled) { enabled_ = enabled; }

        /**
         * @brief Check if camera is enabled
         * @return true if camera is enabled
         */
        bool isEnabled() const noexcept { return enabled_; }

        // === Debug and Utilities ===

        /**
         * @brief Get debug information string
         * @return String with camera debug info
         */
        virtual std::string getDebugInfo() const;

        /**
         * @brief Validate camera state
         * @return true if camera is in valid state
         */
        virtual bool validate() const;

    protected:
        // === Protected Members ===

        CameraID id_;                                       // Unique identifier
        std::string name_;                                  // Human-readable name
        CameraType type_;                                   // Camera type
        CameraMode mode_ = CameraMode::STATIC;              // Movement mode
        bool active_ = false;                               // Whether this is the active camera
        bool enabled_ = true;                               // Whether camera is enabled

        // Movement properties
        CameraBounds bounds_;                               // Movement constraints
        float smoothingSpeed_ = 5.0f;                       // Smoothing speed for movement

        // Callback
        CameraCallback callback_;                           // Position change callback

        // === Helper Methods ===

        /**
         * @brief Apply camera bounds to position
         * @param position Position to constrain
         * @return Constrained position
         */
        Vector3 applyBounds(const Vector3& position) const;

        /**
         * @brief Trigger callback if set
         * @param position Current position
         * @param target Current target
         */
        void triggerCallback(const Vector3& position, const Vector3& target) const;

        /**
         * @brief Apply smoothing to movement
         * @param current Current value
         * @param target Target value
         * @param deltaTime Time step
         * @return Smoothed value
         */
        float applySmoothing(float current, float target, float deltaTime) const;
        Vector2 applySmoothing(const Vector2& current, const Vector2& target, float deltaTime) const;
        Vector3 applySmoothing(const Vector3& current, const Vector3& target, float deltaTime) const;

        // Mark as friend so CameraManager can access protected members
        friend class CameraManager;
    };

    /**
     * @brief 2D Camera implementation for sprite-based games
     */
    class Camera2D : public BaseCamera {
    public:
        /**
         * @brief Constructor for 2D camera
         * @param id Unique identifier
         * @param name Human-readable name
         */
        Camera2D(CameraID id, std::string name);

        /**
         * @brief Destructor
         */
        ~Camera2D() override = default;

        // === Position and Transform ===

        Vector3 getPosition() const override { return Vector3(position_.x, position_.y, 0.0f); }
        void setPosition(const Vector3& position) override { setPosition(Vector2(position.x, position.y)); }

        /**
         * @brief Set 2D position
         * @param position New 2D position
         */
        void setPosition(const Vector2& position);

        /**
         * @brief Get 2D position
         * @return Current 2D position
         */
        const Vector2& getPosition2D() const noexcept { return position_; }

        /**
         * @brief Set camera zoom level
         * @param zoom New zoom level (1.0 = normal, 2.0 = 2x zoom)
         */
        void setZoom(float zoom);

        /**
         * @brief Get current zoom level
         * @return Current zoom level
         */
        float getZoom() const noexcept { return zoom_; }

        /**
         * @brief Set camera rotation in degrees
         * @param rotation New rotation angle
         */
        void setRotation(float rotation) { rotation_ = rotation; }

        /**
         * @brief Get camera rotation
         * @return Current rotation in degrees
         */
        float getRotation() const noexcept { return rotation_; }

        /**
         * @brief Set camera offset from center
         * @param offset Offset in screen pixels
         */
        void setOffset(const Vector2& offset) { offset_ = offset; }

        /**
         * @brief Get camera offset
         * @return Current offset
         */
        const Vector2& getOffset() const noexcept { return offset_; }

        // === Target Following ===

        /**
         * @brief Set target position for following modes
         * @param target Target position to follow
         */
        void setTarget(const Vector2& target);

        /**
         * @brief Get current target position
         * @return Current target position
         */
        const Vector2& getTarget() const noexcept { return targetPosition_; }

        /**
         * @brief Set follow speed for target following
         * @param speed Follow speed (higher = faster following)
         */
        void setFollowSpeed(float speed) { followSpeed_ = std::max(0.1f, speed); }

        /**
         * @brief Get follow speed
         * @return Current follow speed
         */
        float getFollowSpeed() const noexcept { return followSpeed_; }

        // === Zoom Configuration ===

        /**
         * @brief Set zoom constraints
         * @param minZoom Minimum allowed zoom
         * @param maxZoom Maximum allowed zoom
         */
        void setZoomLimits(float minZoom, float maxZoom);

        /**
         * @brief Get zoom limits
         * @return Pair of (min, max) zoom values
         */
        std::pair<float, float> getZoomLimits() const { return {minZoom_, maxZoom_}; }

        // === Coordinate Transformations ===

        /**
         * @brief Convert world position to screen coordinates
         * @param worldPos World position to convert
         * @param viewport Viewport for conversion
         * @return Screen coordinates
         */
        Vector2 worldToScreen(const Vector2& worldPos, const Viewport& viewport) const;

        /**
         * @brief Convert screen coordinates to world position
         * @param screenPos Screen position to convert
         * @param viewport Viewport for conversion
         * @return World coordinates
         */
        Vector2 screenToWorld(const Vector2& screenPos, const Viewport& viewport) const;

        /**
         * @brief Get camera view bounds in world space
         * @param viewport Viewport to calculate bounds for
         * @return Camera bounds that define visible area
         */
        CameraBounds getViewBounds(const Viewport& viewport) const;

        // === BaseCamera Implementation ===

        void update(float deltaTime) override;
        void reset() override;
        std::string getDebugInfo() const override;
        bool validate() const override;

    private:
        // === 2D Camera Properties ===

        Vector2 position_{0.0f, 0.0f};                     // Current camera position
        Vector2 targetPosition_{0.0f, 0.0f};               // Target position for following modes
        Vector2 offset_{0.0f, 0.0f};                       // Offset from center (for UI cameras)

        float zoom_ = 1.0f;                                 // Zoom level (1.0 = normal scale)
        float targetZoom_ = 1.0f;                           // Target zoom for smooth zooming
        float rotation_ = 0.0f;                             // Rotation in degrees
        float targetRotation_ = 0.0f;                       // Target rotation for smooth rotation

        // Movement properties
        float followSpeed_ = 5.0f;                          // Speed for following target
        Vector2 velocity_{0.0f, 0.0f};                     // Current movement velocity

        // Constraints
        float minZoom_ = 0.1f;                              // Minimum zoom level
        float maxZoom_ = 10.0f;                             // Maximum zoom level

        // === Update Methods ===

        /**
         * @brief Update camera based on current mode
         * @param deltaTime Time step
         */
        void updateMovement(float deltaTime);

        /**
         * @brief Update zoom smoothing
         * @param deltaTime Time step
         */
        void updateZoom(float deltaTime);

        /**
         * @brief Update rotation smoothing
         * @param deltaTime Time step
         */
        void updateRotation(float deltaTime);

        /**
         * @brief Apply zoom constraints
         */
        void applyZoomConstraints();
    };

    /**
     * @brief 3D Camera implementation for 3D scenes
     */
    class Camera3D : public BaseCamera {
    public:
        /**
         * @brief Constructor for 3D camera
         * @param id Unique identifier
         * @param name Human-readable name
         * @param perspective Whether to use perspective projection
         */
        Camera3D(CameraID id, std::string name, bool perspective = true);

        /**
         * @brief Destructor
         */
        ~Camera3D() override = default;

        // === Position and Transform ===

        Vector3 getPosition() const override { return position_; }
        void setPosition(const Vector3& position) override;

        /**
         * @brief Set camera target (look-at point)
         * @param target Point to look at
         */
        void setTarget(const Vector3& target);

        /**
         * @brief Get camera target
         * @return Current target position
         */
        const Vector3& getTarget() const noexcept { return target_; }

        /**
         * @brief Set camera up vector
         * @param up Up direction vector
         */
        void setUp(const Vector3& up) { up_ = glm::normalize(up); } // CAMBIO: usar glm::normalize

        /**
         * @brief Get camera up vector
         * @return Current up direction
         */
        const Vector3& getUp() const noexcept { return up_; }

        /**
         * @brief Get camera forward direction
         * @return Normalized forward vector
         */
        Vector3 getForward() const { return glm::normalize(target_ - position_); } // CAMBIO: usar glm::normalize

        /**
         * @brief Get camera right direction
         * @return Normalized right vector
         */
        Vector3 getRight() const { return glm::normalize(glm::cross(getForward(), up_)); } // CAMBIO: usar glm::cross y normalize

        // === Projection Properties ===

        /**
         * @brief Set field of view for perspective cameras
         * @param fov Field of view in degrees
         */
        void setFOV(float fov);

        /**
         * @brief Get field of view
         * @return Current FOV in degrees
         */
        float getFOV() const noexcept { return fov_; }

        /**
         * @brief Set near and far clipping planes
         * @param nearPlane Near clipping distance
         * @param farPlane Far clipping distance
         */
        void setClippingPlanes(float nearPlane, float farPlane);

        /**
         * @brief Get clipping planes
         * @return Pair of (near, far) clipping distances
         */
        std::pair<float, float> getClippingPlanes() const { return {nearPlane_, farPlane_}; }

        /**
         * @brief Set whether to use perspective projection
         * @param perspective true for perspective, false for orthographic
         */
        void setPerspective(bool perspective) { isPerspective_ = perspective; }

        /**
         * @brief Check if using perspective projection
         * @return true if perspective, false if orthographic
         */
        bool isPerspective() const noexcept { return isPerspective_; }

        /**
         * @brief Set orthographic size (for orthographic cameras)
         * @param size Orthographic view size
         */
        void setOrthographicSize(float size) { orthographicSize_ = std::max(0.1f, size); }

        /**
         * @brief Get orthographic size
         * @return Current orthographic size
         */
        float getOrthographicSize() const noexcept { return orthographicSize_; }

        // === FPS-Style Controls ===

        /**
         * @brief Set yaw angle for FPS-style cameras
         * @param yaw Yaw angle in degrees
         */
        void setYaw(float yaw) { yaw_ = yaw; updateFromEuler(); }

        /**
         * @brief Set pitch angle for FPS-style cameras
         * @param pitch Pitch angle in degrees
         */
        void setPitch(float pitch);

        /**
         * @brief Set roll angle
         * @param roll Roll angle in degrees
         */
        void setRoll(float roll) { roll_ = roll; updateFromEuler(); }

        /**
         * @brief Get current yaw angle
         * @return Yaw in degrees
         */
        float getYaw() const noexcept { return yaw_; }

        /**
         * @brief Get current pitch angle
         * @return Pitch in degrees
         */
        float getPitch() const noexcept { return pitch_; }

        /**
         * @brief Get current roll angle
         * @return Roll in degrees
         */
        float getRoll() const noexcept { return roll_; }

        /**
         * @brief Set pitch constraints for FPS cameras
         * @param minPitch Minimum pitch angle
         * @param maxPitch Maximum pitch angle
         */
        void setPitchLimits(float minPitch, float maxPitch);

        // === Target Following ===

        /**
         * @brief Set follow target for third-person modes
         * @param target Target position to follow
         */
        void setFollowTarget(const Vector3& target);

        /**
         * @brief Get follow target
         * @return Current follow target
         */
        const Vector3& getFollowTarget() const noexcept { return followTarget_; }

        /**
         * @brief Set distance from follow target
         * @param distance Distance to maintain from target
         */
        void setFollowDistance(float distance) { followDistance_ = std::max(0.1f, distance); }

        /**
         * @brief Get follow distance
         * @return Current follow distance
         */
        float getFollowDistance() const noexcept { return followDistance_; }

        /**
         * @brief Set follow height offset
         * @param height Height offset from target
         */
        void setFollowHeight(float height) { followHeight_ = height; }

        /**
         * @brief Get follow height
         * @return Current follow height offset
         */
        float getFollowHeight() const noexcept { return followHeight_; }

        // === Coordinate Transformations ===

        /**
         * @brief Convert world position to screen coordinates
         * @param worldPos World position to convert
         * @param viewport Viewport for conversion
         * @return Screen coordinates
         */
        Vector2 worldToScreen(const Vector3& worldPos, const Viewport& viewport) const;

        /**
         * @brief Convert screen coordinates to world ray
         * @param screenPos Screen position
         * @param viewport Viewport for conversion
         * @return Pair of (ray origin, ray direction)
         */
        std::pair<Vector3, Vector3> screenToWorldRay(const Vector2& screenPos, const Viewport& viewport) const;

        /**
         * @brief Convert screen coordinates to world position at given depth
         * @param screenPos Screen position
         * @param viewport Viewport for conversion
         * @param depth Depth in world units
         * @return World position at specified depth
         */
        Vector3 screenToWorld(const Vector2& screenPos, const Viewport& viewport, float depth) const;

        // === BaseCamera Implementation ===

        void update(float deltaTime) override;
        void reset() override;
        std::string getDebugInfo() const override;
        bool validate() const override;

    private:
        // === 3D Camera Properties ===

        Vector3 position_{0.0f, 0.0f, 5.0f};               // Current camera position
        Vector3 target_{0.0f, 0.0f, 0.0f};                 // Point camera is looking at
        Vector3 up_{0.0f, 1.0f, 0.0f};                     // Up direction vector

        // Projection properties
        float fov_ = 60.0f;                                 // Field of view in degrees
        float nearPlane_ = 0.1f;                            // Near clipping plane
        float farPlane_ = 1000.0f;                          // Far clipping plane
        bool isPerspective_ = true;                         // Perspective vs orthographic
        float orthographicSize_ = 10.0f;                    // Size for orthographic projection

        // FPS-style rotation (Euler angles)
        float yaw_ = -90.0f;                                // Horizontal rotation
        float pitch_ = 0.0f;                                // Vertical rotation
        float roll_ = 0.0f;                                 // Camera roll
        float minPitch_ = -89.0f;                           // Minimum pitch constraint
        float maxPitch_ = 89.0f;                            // Maximum pitch constraint

        // Following properties
        Vector3 followTarget_{0.0f, 0.0f, 0.0f};           // Target to follow
        float followDistance_ = 5.0f;                       // Distance from follow target
        float followHeight_ = 2.0f;                         // Height offset from target
        float followSpeed_ = 5.0f;                          // Speed for following movement

        // Orbital properties
        float orbitalRadius_ = 5.0f;                        // Radius for orbital movement
        float orbitalAngle_ = 0.0f;                         // Current orbital angle
        float orbitalSpeed_ = 1.0f;                         // Speed of orbital movement

        // === Update Methods ===

        /**
         * @brief Update camera based on current mode
         * @param deltaTime Time step
         */
        void updateMovement(float deltaTime);

        /**
         * @brief Update FPS-style free look camera
         * @param deltaTime Time step
         */
        void updateFreeLook(float deltaTime);

        /**
         * @brief Update target following camera
         * @param deltaTime Time step
         */
        void updateFollowTarget(float deltaTime);

        /**
         * @brief Update orbital camera movement
         * @param deltaTime Time step
         */
        void updateOrbital(float deltaTime);

        /**
         * @brief Update target from Euler angles (yaw, pitch, roll)
         */
        void updateFromEuler();

        /**
         * @brief Apply pitch constraints
         */
        void applyPitchConstraints();

        /**
         * @brief Calculate view matrix for this camera
         * @return 4x4 view matrix (for integration with rendering system)
         */
        std::array<float, 16> calculateViewMatrix() const;

        /**
         * @brief Calculate projection matrix for this camera
         * @param viewport Viewport to calculate projection for
         * @return 4x4 projection matrix
         */
        std::array<float, 16> calculateProjectionMatrix(const Viewport& viewport) const;
    };

    /**
     * @brief High-performance camera management system
     *
     * This class provides:
     * - Multiple camera support with handle-based access
     * - 2D and 3D camera implementations
     * - Smooth transitions between camera states
     * - Screen shake and camera effects
     * - Input integration for camera controls
     * - Coordinate transformation utilities
     * - Thread-safe operations for multithreaded games
     */
    class CameraManager {
    public:
        /**
         * @brief Constructor with configuration
         * @param config Configuration parameters for camera system
         */
        explicit CameraManager(const CameraManagerConfig& config = {});

        /**
         * @brief Destructor - cleanup all cameras and resources
         */
        ~CameraManager();

        // Non-copyable but moveable for flexibility
        CameraManager(const CameraManager&) = delete;
        CameraManager& operator=(const CameraManager&) = delete;
        CameraManager(CameraManager&&) = default;
        CameraManager& operator=(CameraManager&&) = default;

        /**
         * @brief Initialize camera manager
         * @return true if initialization was successful
         */
        bool initialize();

        /**
         * @brief Shutdown and cleanup all resources
         */
        void shutdown();

        /**
         * @brief Update all cameras and transitions
         * @param deltaTime Time elapsed since last update in seconds
         */
        void update(float deltaTime);

        // === Camera Creation and Management ===

        /**
         * @brief Create a new 2D camera
         * @param name Human-readable name for debugging
         * @return CameraID for the created camera, or INVALID_CAMERA_ID if failed
         */
        CameraID createCamera2D(const std::string& name = "");

        /**
         * @brief Create a new 3D camera
         * @param name Human-readable name for debugging
         * @param perspective Whether to use perspective projection
         * @return CameraID for the created camera, or INVALID_CAMERA_ID if failed
         */
        CameraID createCamera3D(const std::string& name = "", bool perspective = true);

        /**
         * @brief Remove a camera and all its associated data
         * @param cameraId Camera to remove
         * @return true if camera was successfully removed
         */
        bool removeCamera(CameraID cameraId);

        /**
         * @brief Get camera by ID
         * @param cameraId Camera identifier
         * @return Pointer to camera, or nullptr if not found
         */
        BaseCamera* getCamera(CameraID cameraId);

        /**
         * @brief Get camera by ID (const version)
         * @param cameraId Camera identifier
         * @return Const pointer to camera, or nullptr if not found
         */
        const BaseCamera* getCamera(CameraID cameraId) const;

        /**
         * @brief Get camera by name
         * @param name Camera name
         * @return CameraID if found, or INVALID_CAMERA_ID if not found
         */
        CameraID getCameraByName(const std::string& name) const;

        /**
         * @brief Get 2D camera (with type checking)
         * @param cameraId Camera identifier
         * @return Pointer to 2D camera, or nullptr if not found or wrong type
         */
        Camera2D* getCamera2D(CameraID cameraId);

        /**
         * @brief Get 3D camera (with type checking)
         * @param cameraId Camera identifier
         * @return Pointer to 3D camera, or nullptr if not found or wrong type
         */
        Camera3D* getCamera3D(CameraID cameraId);

        // === Active Camera Management ===

        /**
         * @brief Set the active camera
         * @param cameraId Camera to make active
         * @return true if camera was successfully set as active
         */
        bool setActiveCamera(CameraID cameraId);

        /**
         * @brief Set active camera by name
         * @param name Name of camera to make active
         * @return true if camera was found and set as active
         */
        bool setActiveCamera(const std::string& name);

        /**
         * @brief Get the currently active camera ID
         * @return Active camera ID, or INVALID_CAMERA_ID if none
         */
        CameraID getActiveCameraId() const noexcept { return activeCameraId_; }

        /**
         * @brief Get the active camera pointer
         * @return Pointer to active camera, or nullptr if none
         */
        BaseCamera* getActiveCamera();

        /**
         * @brief Get the active camera pointer (const version)
         * @return Const pointer to active camera, or nullptr if none
         */
        const BaseCamera* getActiveCamera() const;

        // === Camera Transitions ===

        /**
         * @brief Start a smooth transition to a new camera position
         * @param cameraId Camera to transition
         * @param targetPosition Target position to move to
         * @param config Transition configuration
         * @return TransitionID for tracking, or INVALID_TRANSITION_ID if failed
         */
        TransitionID transitionToPosition(CameraID cameraId, const Vector3& targetPosition, const TransitionConfig& config = {});

        /**
         * @brief Start a smooth transition to look at a target
         * @param cameraId Camera to transition (3D only)
         * @param targetLookAt Target point to look at
         * @param config Transition configuration
         * @return TransitionID for tracking, or INVALID_TRANSITION_ID if failed
         */
        TransitionID transitionToTarget(CameraID cameraId, const Vector3& targetLookAt, const TransitionConfig& config = {});

        /**
         * @brief Start a transition between two cameras
         * @param fromCameraId Source camera
         * @param toCameraId Target camera
         * @param config Transition configuration
         * @return TransitionID for tracking, or INVALID_TRANSITION_ID if failed
         */
        TransitionID transitionBetweenCameras(CameraID fromCameraId, CameraID toCameraId, const TransitionConfig& config = {});

        /**
         * @brief Stop a specific transition
         * @param transitionId Transition to stop
         * @return true if transition was stopped
         */
        bool stopTransition(TransitionID transitionId);

        /**
         * @brief Stop all transitions for a camera
         * @param cameraId Camera to stop transitions for
         * @return Number of transitions stopped
         */
        int stopAllTransitions(CameraID cameraId);

        /**
         * @brief Check if a transition is active
         * @param transitionId Transition to check
         * @return true if transition is currently running
         */
        bool isTransitionActive(TransitionID transitionId) const;

        // === Camera Effects ===

        /**
         * @brief Start camera shake effect
         * @param cameraId Camera to shake
         * @param config Shake configuration
         * @return true if shake was started successfully
         */
        bool startCameraShake(CameraID cameraId, const ShakeConfig& config);

        /**
         * @brief Stop camera shake for specific camera
         * @param cameraId Camera to stop shaking
         * @return true if shake was stopped
         */
        bool stopCameraShake(CameraID cameraId);

        /**
         * @brief Update shake intensity for active shake
         * @param cameraId Camera to update
         * @param intensity New shake intensity
         * @return true if shake was updated
         */
        bool updateShakeIntensity(CameraID cameraId, float intensity);

        /**
         * @brief Check if camera is currently shaking
         * @param cameraId Camera to check
         * @return true if camera has active shake effect
         */
        bool isCameraShaking(CameraID cameraId) const;

        // === Input Integration ===

        /**
         * @brief Process mouse look input for active camera
         * @param deltaX Horizontal mouse movement
         * @param deltaY Vertical mouse movement
         * @param deltaTime Time step for frame-rate independent movement
         */
        void processMouseLook(float deltaX, float deltaY, float deltaTime);

        /**
         * @brief Process zoom input for active camera
         * @param zoomDelta Zoom change amount (positive = zoom in)
         */
        void processZoom(float zoomDelta);

        /**
         * @brief Process movement input for active camera
         * @param forward Forward/backward movement
         * @param right Left/right movement
         * @param up Up/down movement
         * @param deltaTime Time step
         */
        void processMovement(float forward, float right, float up, float deltaTime);

        /**
         * @brief Set mouse sensitivity for camera controls
         * @param sensitivity Sensitivity multiplier
         */
        void setMouseSensitivity(float sensitivity) { config_.mouseSensitivity = std::max(0.1f, sensitivity); }

        /**
         * @brief Set scroll wheel sensitivity
         * @param sensitivity Sensitivity multiplier
         */
        void setScrollSensitivity(float sensitivity) { config_.scrollSensitivity = std::max(0.1f, sensitivity); }

        // === Viewport and Coordinate Transformations ===

        /**
         * @brief Set the current viewport
         * @param viewport Viewport information
         */
        void setViewport(const Viewport& viewport) { viewport_ = viewport; }

        /**
         * @brief Get the current viewport
         * @return Current viewport settings
         */
        const Viewport& getViewport() const noexcept { return viewport_; }

        /**
         * @brief Convert world position to screen coordinates using active camera
         * @param worldPos World position to convert
         * @return Screen coordinates, or Vector2::ZERO if no active camera
         */
        Vector2 worldToScreen(const Vector3& worldPos) const;

        /**
         * @brief Convert screen coordinates to world position using active camera
         * @param screenPos Screen position to convert
         * @param depth Depth for 3D cameras (ignored for 2D)
         * @return World position, or Vector3::ZERO if no active camera
         */
        Vector3 screenToWorld(const Vector2& screenPos, float depth = 0.0f) const;

        /**
         * @brief Get view frustum bounds for active camera
         * @return Camera bounds representing visible area
         */
        CameraBounds getViewBounds() const;

        // === Preset Camera Configurations ===

        /**
         * @brief Setup FPS-style camera
         * @param cameraId Camera to configure
         * @param position Initial position
         * @param sensitivity Mouse sensitivity
         * @return true if setup was successful
         */
        bool setupFPSCamera(CameraID cameraId, const Vector3& position = math::constants::VEC3_ZERO, float sensitivity = 1.0f);

        /**
         * @brief Setup third-person camera
         * @param cameraId Camera to configure
         * @param target Target to follow
         * @param distance Distance from target
         * @param height Height offset
         * @return true if setup was successful
         */
        bool setupThirdPersonCamera(CameraID cameraId, const Vector3& target = math::constants::VEC3_ZERO, float distance = 5.0f, float height = 2.0f);

        /**
         * @brief Setup top-down 2D camera
         * @param cameraId Camera to configure
         * @param position Initial position
         * @param zoom Initial zoom level
         * @return true if setup was successful
         */
        bool setupTopDownCamera(CameraID cameraId, const Vector2& position = math::constants::VEC2_ZERO, float zoom = 1.0f);

        /**
         * @brief Setup side-scrolling 2D camera
         * @param cameraId Camera to configure
         * @param position Initial position
         * @param followSpeed Speed for following targets
         * @return true if setup was successful
         */
        bool setupSideScrollerCamera(CameraID cameraId, const Vector2& position = math::constants::VEC2_ZERO, float followSpeed = 5.0f);

        /**
         * @brief Setup isometric 3D camera
         * @param cameraId Camera to configure
         * @param position Camera position
         * @param target Look-at target
         * @return true if setup was successful
         */
        bool setupIsometricCamera(CameraID cameraId, const Vector3& position, const Vector3& target = math::constants::VEC3_ZERO);

        // === Configuration and Statistics ===

        /**
         * @brief Get current configuration
         * @return Current configuration settings
         */
        const CameraManagerConfig& getConfig() const noexcept { return config_; }

        /**
         * @brief Update configuration
         * @param config New configuration settings
         */
        void updateConfig(const CameraManagerConfig& config) { config_ = config; }

        /**
         * @brief Get number of cameras
         * @return Total number of cameras
         */
        std::size_t getCameraCount() const;

        /**
         * @brief Get number of active transitions
         * @return Number of currently running transitions
         */
        std::size_t getActiveTransitionCount() const;

        /**
         * @brief Get memory usage statistics
         * @return Approximate memory usage in bytes
         */
        std::size_t getMemoryUsage() const;

        /**
         * @brief Get list of all camera names
         * @return Vector of camera name strings
         */
        std::vector<std::string> getCameraNames() const;

        /**
         * @brief Clear all cameras and reset manager
         */
        void clearAllCameras();

        // === Debug and Utilities ===

        /**
         * @brief Get debug information about camera system
         * @return String with detailed debug information
         */
        std::string getDebugInfo() const;

        /**
         * @brief Get performance statistics
         * @return String with performance metrics
         */
        std::string getPerformanceStats() const;

        /**
         * @brief Enable or disable camera logging
         * @param enable Whether to log camera events
         */
        void setCameraLogging(bool enable) { config_.enableCameraLogging = enable; }

        /**
         * @brief Validate all cameras and their states
         * @return true if all cameras are in valid states
         */
        bool validateAllCameras() const;

    private:
        // === Internal Structures ===

        /**
         * @brief Camera transition state tracking
         */
        struct CameraTransition {
            TransitionID id;                            // Unique transition identifier
            CameraID cameraId;                          // Camera being transitioned
            TransitionType type;                        // Type of transition
            float duration;                             // Total transition duration
            float currentTime;                          // Current transition time
            bool active;                                // Whether transition is active

            // Transition data (what's being transitioned)
            bool hasPosition;                           // Whether position is being transitioned
            bool hasTarget;                             // Whether target is being transitioned
            bool hasZoom;                               // Whether zoom is being transitioned

            Vector3 startPosition, targetPosition;     // Position transition data
            Vector3 startTarget, targetLookAt;         // Target transition data
            float startZoom, targetZoom;               // Zoom transition data

            TransitionCallback callback;               // Completion callback
        };

        /**
         * @brief Camera shake state tracking
         */
        struct CameraShake {
            CameraID cameraId;                          // Camera being shaken
            ShakeConfig config;                         // Shake configuration
            float currentTime;                          // Current shake time
            Vector3 currentOffset;                      // Current shake offset
            bool active;                                // Whether shake is active
            Vector3 basePosition;                       // Original position before shake
        };

        // === Member Variables ===

        // Configuration and state
        CameraManagerConfig config_;                   // Configuration parameters
        bool initialized_ = false;                     // Initialization state
        Viewport viewport_;                            // Current viewport settings

        // ID generation
        CameraID nextCameraId_ = 1;                    // Next camera ID to assign
        TransitionID nextTransitionId_ = 1;            // Next transition ID to assign

        // Camera storage
        std::unordered_map<CameraID, std::unique_ptr<BaseCamera>> cameras_;       // All cameras
        std::unordered_map<std::string, CameraID> cameraNameMap_;                 // Name to ID mapping
        CameraID activeCameraId_ = INVALID_CAMERA_ID;                            // Currently active camera

        // Effects and transitions
        std::unordered_map<TransitionID, CameraTransition> transitions_;         // Active transitions
        std::unordered_map<CameraID, CameraShake> shakeStates_;                  // Camera shake states

        // Threading and performance
        mutable std::mutex cameraMutex_;               // Mutex for thread-safe operations
        std::chrono::steady_clock::time_point lastUpdateTime_;                   // Last update timestamp

        // Statistics
        mutable std::atomic<std::uint64_t> camerasUpdated_{0};                   // Cameras updated counter
        mutable std::atomic<std::uint64_t> transitionsProcessed_{0};             // Transitions processed counter

        // === Internal Methods ===

        /**
         * @brief Validate camera ID
         * @param cameraId ID to validate
         * @return true if ID is valid and camera exists
         */
        bool isValidCameraId(CameraID cameraId) const;

        /**
         * @brief Process all active transitions
         * @param deltaTime Time step
         */
        void updateTransitions(float deltaTime);

        /**
         * @brief Process all camera shake effects
         * @param deltaTime Time step
         */
        void updateShakeEffects(float deltaTime);

        /**
         * @brief Apply transition interpolation
         * @param transition Transition to process
         * @param deltaTime Time step
         * @return true if transition completed
         */
        bool processTransition(CameraTransition& transition, float deltaTime);

        /**
         * @brief Apply shake effect to camera
         * @param shake Shake state to process
         * @param deltaTime Time step
         * @return true if shake completed
         */
        bool processShake(CameraShake& shake, float deltaTime);

        /**
         * @brief Calculate transition interpolation value
         * @param type Transition type
         * @param t Normalized time (0-1)
         * @return Interpolated value
         */
        float calculateTransitionEasing(TransitionType type, float t) const;

        /**
         * @brief Generate shake offset based on pattern
         * @param pattern Shake pattern
         * @param intensity Shake intensity
         * @param time Current time
         * @param frequency Shake frequency
         * @return Shake offset vector
         */
        Vector3 generateShakeOffset(ShakePattern pattern, float intensity, float time, float frequency) const;

        /**
         * @brief Log camera event for debugging
         * @param message Log message
         */
        void logCameraEvent(const std::string& message) const;

        /**
         * @brief Cleanup completed transitions
         */
        void cleanupCompletedTransitions();

        /**
         * @brief Cleanup inactive shake effects
         */
        void cleanupInactiveShakes();
    };

} // namespace engine::camera