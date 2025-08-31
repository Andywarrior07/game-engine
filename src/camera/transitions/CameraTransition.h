/**
 * @file CameraTransition.h
 * @brief Camera transition state and execution
 * @author Andrés Guerrero
 * @date 26-08-2025
 */

#pragma once

#include "TransitionConfig.h"

#include "../core/CameraTypes.h"

namespace engine::camera {
    /**
     * @brief Camera transition state tracking and execution
     *
     * Manages the state and execution of camera transitions,
     * including position, target, zoom, and rotation animations.
     */
    class CameraTransition {
    public:
        /**
         * @brief Default constructor
         */
        CameraTransition() = default;

        /**
         * @brief Constructor with ID and camera
         * @param id Transition ID
         * @param cameraId Target camera ID
         * @param config Transition configuration
         */
        CameraTransition(TransitionID id, CameraID cameraId, const TransitionConfig& config);

        // ========================================================================
        // ACCESSORS
        // ========================================================================

        /**
         * @brief Get transition ID
         * @return Unique transition identifier
         */
        [[nodiscard]] TransitionID getId() const noexcept { return id_; }

        /**
         * @brief Get camera ID
         * @return ID of camera being transitioned
         */
        [[nodiscard]] CameraID getCameraId() const noexcept { return cameraId_; }

        /**
         * @brief Get transition configuration
         * @return Current configuration
         */
        [[nodiscard]] const TransitionConfig& getConfig() const noexcept { return config_; }

        /**
         * @brief Check if transition is active
         * @return true if currently running
         */
        [[nodiscard]] bool isActive() const noexcept { return active_; }

        /**
         * @brief Get current time
         * @return Time elapsed since transition started
         */
        [[nodiscard]] float getCurrentTime() const noexcept { return currentTime_; }

        /**
         * @brief Get progress (0 to 1)
         * @return Normalized progress
         */
        [[nodiscard]] float getProgress() const {
            if (config_.duration <= 0.0f) return 1.0f;
            return std::min(currentTime_ / config_.duration, 1.0f);
        }

        /**
         * @brief Get eased progress
         * @return Progress with easing applied
         */
        [[nodiscard]] float getEasedProgress() const;

        /**
         * @brief Check if transition is complete
         * @return true if transition has finished
         */
        [[nodiscard]] bool isComplete() const {
            return !active_ || (delayRemaining_ <= 0.0f && currentTime_ >= config_.duration);
        }

        /**
         * @brief Check if still in delay phase
         * @return true if waiting for delay to complete
         */
        [[nodiscard]] bool isDelaying() const {
            return delayRemaining_ > 0.0f;
        }

        // ========================================================================
        // TRANSITION DATA
        // ========================================================================

        /**
         * @brief Setup position transition
         * @param start Starting position
         * @param target Target position
         */
        void setupPosition(const Vec3& start, const Vec3& target);

        /**
         * @brief Setup target transition (for 3D cameras)
         * @param start Starting look-at point
         * @param target Target look-at point
         */
        void setupTarget(const Vec3& start, const Vec3& target);

        /**
         * @brief Setup zoom transition (for 2D cameras)
         * @param start Starting zoom
         * @param target Target zoom
         */
        void setupZoom(float start, float target);

        /**
         * @brief Setup rotation transition
         * @param start Starting rotation
         * @param target Target rotation
         */
        void setupRotation(float start, float target);

        /**
         * @brief Setup FOV transition (for 3D cameras)
         * @param start Starting FOV
         * @param target Target FOV
         */
        void setupFOV(float start, float target);

        // ========================================================================
        // INTERPOLATION
        // ========================================================================

        /**
         * @brief Get interpolated position
         * @return Current position based on progress
         */
        Vec3 getInterpolatedPosition() const;

        /**
         * @brief Get interpolated target
         * @return Current target based on progress
         */
        Vec3 getInterpolatedTarget() const;

        /**
         * @brief Get interpolated zoom
         * @return Current zoom based on progress
         */
        float getInterpolatedZoom() const;

        /**
         * @brief Get interpolated rotation
         * @return Current rotation based on progress
         */
        float getInterpolatedRotation() const;

        /**
         * @brief Get interpolated FOV
         * @return Current FOV based on progress
         */
        float getInterpolatedFOV() const;

        // ========================================================================
        // CONTROL
        // ========================================================================

        /**
         * @brief Start the transition
         */
        void start();

        /**
         * @brief Stop the transition
         * @param triggerCallback Whether to trigger interrupt callback
         */
        void stop(bool triggerCallback = true);

        /**
         * @brief Update transition
         * @param deltaTime Time step
         * @return true if transition completed this frame
         */
        bool update(float deltaTime);

    private:
        // Identification
        TransitionID id_ = INVALID_TRANSITION_ID; ///< Unique transition identifier
        CameraID cameraId_ = INVALID_CAMERA_ID; ///< Camera being transitioned

        // Configuration
        TransitionConfig config_; ///< Transition configuration

        // State
        float currentTime_ = 0.0f; ///< Current transition time
        float delayRemaining_ = 0.0f; ///< Remaining delay time
        bool active_ = false; ///< Whether transition is active

        // Transition flags
        bool hasPosition_ = false; ///< Whether position is being transitioned
        bool hasTarget_ = false; ///< Whether target is being transitioned
        bool hasZoom_ = false; ///< Whether zoom is being transitioned
        bool hasRotation_ = false; ///< Whether rotation is being transitioned
        bool hasFOV_ = false; ///< Whether FOV is being transitioned

        // Position data
        Vec3 startPosition_{}; ///< Starting position
        Vec3 targetPosition_{}; ///< Target position

        // Target data (3D cameras)
        Vec3 startTarget_{}; ///< Starting look-at point
        Vec3 targetLookAt_{}; ///< Target look-at point

        // Zoom data (2D cameras)
        float startZoom_ = 1.0f; ///< Starting zoom
        float targetZoom_ = 1.0f; ///< Target zoom

        // Rotation data
        float startRotation_ = 0.0f; ///< Starting rotation
        float targetRotation_ = 0.0f; ///< Target rotation

        // FOV data (3D cameras)
        float startFOV_ = 60.0f; ///< Starting FOV
        float targetFOV_ = 60.0f; ///< Target FOV
    };
} // namespace engine::camera
