/**
 * @file CameraManager.h
 * @brief Main camera management system
 * @author Andrés Guerrero
 * @date 26-08-2025
 */

#pragma once

#include "CameraManagerConfig.h"
#include "CameraRegistry.h"

#include "../core/CameraTypes.h"
#include "../cameras/Camera2D.h"
#include "../cameras/Camera3D.h"
#include "../effects/CameraShake.h"
#include "../transitions/CameraTransition.h"
// #include "../input/CameraInputHandler.h"
#include "../core/Viewport.h"

#include <memory>
#include <atomic>
#include <chrono>

namespace engine::camera {

    /**
     * @brief High-performance camera management system
     *
     * Central manager for all camera operations including:
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

        // Non-copyable but moveable
        CameraManager(const CameraManager&) = delete;
        CameraManager& operator=(const CameraManager&) = delete;
        CameraManager(CameraManager&&) = default;
        CameraManager& operator=(CameraManager&&) = default;

        // ========================================================================
        // LIFECYCLE
        // ========================================================================

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

        /**
         * @brief Check if manager is initialized
         * @return true if initialized
         */
        bool isInitialized() const noexcept { return initialized_; }

        // ========================================================================
        // CAMERA CREATION AND MANAGEMENT
        // ========================================================================

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

        // ========================================================================
        // ACTIVE CAMERA MANAGEMENT
        // ========================================================================

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
        const BaseCamera* getActiveCamera() const;

        // ========================================================================
        // CAMERA TRANSITIONS
        // ========================================================================

        /**
         * @brief Start a smooth transition to a new camera position
         * @param cameraId Camera to transition
         * @param targetPosition Target position to move to
         * @param config Transition configuration
         * @return TransitionID for tracking, or INVALID_TRANSITION_ID if failed
         */
        TransitionID transitionToPosition(CameraID cameraId, const Vec3& targetPosition,
                                         const TransitionConfig& config = {});

        /**
         * @brief Start a smooth transition to look at a target
         * @param cameraId Camera to transition (3D only)
         * @param targetLookAt Target point to look at
         * @param config Transition configuration
         * @return TransitionID for tracking, or INVALID_TRANSITION_ID if failed
         */
        TransitionID transitionToTarget(CameraID cameraId, const Vec3& targetLookAt,
                                       const TransitionConfig& config = {});

        /**
         * @brief Start a transition between two cameras
         * @param fromCameraId Source camera
         * @param toCameraId Target camera
         * @param config Transition configuration
         * @return TransitionID for tracking, or INVALID_TRANSITION_ID if failed
         */
        TransitionID transitionBetweenCameras(CameraID fromCameraId, CameraID toCameraId,
                                             const TransitionConfig& config = {});

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
        int stopAllTransitions(CameraID cameraId = INVALID_CAMERA_ID);

        /**
         * @brief Check if a transition is active
         * @param transitionId Transition to check
         * @return true if transition is currently running
         */
        bool isTransitionActive(TransitionID transitionId) const;

        // ========================================================================
        // CAMERA EFFECTS
        // ========================================================================

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
         * @brief Stop all shake effects
         * @return Number of shakes stopped
         */
        int stopAllShakes();

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

        // ========================================================================
        // INPUT HANDLING
        // ========================================================================

        // void setInputManager(InputManager* inputManager);

        /**
         * @brief Get input handler
         * @return Pointer to input handler
         */
        // CameraInputHandler* getInputHandler() { return inputHandler_.get(); }
        // const CameraInputHandler* getInputHandler() const { return inputHandler_.get(); }

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

        // ========================================================================
        // VIEWPORT AND TRANSFORMATIONS
        // ========================================================================

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
        Vec2 worldToScreen(const Vec3& worldPos) const;

        /**
         * @brief Convert screen coordinates to world position using active camera
         * @param screenPos Screen position to convert
         * @param depth Depth for 3D cameras (ignored for 2D)
         * @return World position, or Vector3::ZERO if no active camera
         */
        Vec3 screenToWorld(const Vec2& screenPos, float depth = 0.0f) const;

        /**
         * @brief Get view frustum bounds for active camera
         * @return Camera bounds representing visible area
         */
        math::AABB getViewBounds() const;

        // ========================================================================
        // CONFIGURATION AND STATISTICS
        // ========================================================================

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
         * @brief Get number of active shake effects
         * @return Number of currently active shakes
         */
        std::size_t getActiveShakeCount() const;

        /**
         * @brief Get memory usage statistics
         * @return Approximate memory usage in bytes
         */
        std::size_t getMemoryUsage() const;

        /**
         * @brief Get performance metrics
         * @return String with performance statistics
         */
        std::string getPerformanceStats() const;

        /**
         * @brief Clear all cameras and reset manager
         */
        void clearAllCameras();

        /**
         * @brief Enable or disable camera logging
         * @param enable Whether to log camera events
         */
        void setCameraLogging(const bool enable) { config_.enableCameraLogging = enable; }

        CameraRegistry& getRegistry() { return *registry_; }
        const CameraRegistry& getRegistry() const { return *registry_; }

    private:
        // Core components
        CameraManagerConfig config_;                          ///< Configuration parameters
        std::unique_ptr<CameraRegistry> registry_;            ///< Camera registry
        // InputManager* inputManager_ = nullptr;                ///< Input manager
        // std::unique_ptr<CameraInputHandler> inputHandler_;    ///< Input handler

        // State
        bool initialized_ = false;                            ///< Initialization state
        CameraID activeCameraId_ = INVALID_CAMERA_ID;        ///< Currently active camera
        Viewport viewport_;                        ///< Current viewport settings

        // ID generation
        std::atomic<CameraID> nextCameraId_{1};              ///< Next camera ID to assign
        std::atomic<TransitionID> nextTransitionId_{1};      ///< Next transition ID to assign

        // Effects and transitions
        std::unordered_map<TransitionID, std::unique_ptr<CameraTransition>> transitions_; ///< Active transitions
        std::unordered_map<CameraID, std::unique_ptr<CameraShake>> shakeStates_;         ///< Camera shake states

        // Threading and performance
        mutable std::mutex managerMutex_;                    ///< Mutex for thread-safe operations
        std::chrono::steady_clock::time_point lastUpdateTime_; ///< Last update timestamp

        // Statistics
        mutable std::atomic<std::uint64_t> camerasUpdated_{0};     ///< Cameras updated counter
        mutable std::atomic<std::uint64_t> transitionsProcessed_{0}; ///< Transitions processed counter

        // Internal methods
        void updateTransitions(float deltaTime);
        void updateShakeEffects(float deltaTime);
        void cleanupCompletedTransitions();
        void cleanupInactiveShakes();
        void logCameraEvent(const std::string& message) const;
    };

} // namespace engine::camera