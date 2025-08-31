/**
 * @file CameraShake.h
 * @brief Camera shake effect implementation
 * @author Andrés Guerrero
 * @date 25-08-2025
 */
#pragma once

#include "ShakeConfig.h"

namespace engine::camera {
    /**
     * @brief Camera shake state tracking and processing
     *
     * Manage the state and execution of camera shake effects,
     * including pattern generation, fade in/out, and offset calculation.
     */
    class CameraShake {
    public:
        /**
         * @brief Default constructor
         */
        CameraShake() = default;

        /**
         * @brief Constructor with camera ID and configuration
         * @param cameraId ID of camera to shake
         * @param config Shake configuration
         */
        CameraShake(CameraID cameraId, const ShakeConfig& config);

        // ========================================================================
        // ACCESSORS
        // ========================================================================

        /**
         * @brief Get camera ID
         * @return ID of camera being shaken
         */
        [[nodiscard]] CameraID getCameraId() const noexcept { return cameraId_; }

        /**
         * @brief Get shake configuration
         * @return Current shake configuration
         */
        [[nodiscard]] const ShakeConfig& getConfig() const noexcept { return config_; }

        /**
         * @brief Check if shake is active
         * @return true if shake is currently active
         */
        [[nodiscard]] bool isActive() const noexcept { return active_; }

        /**
         * @brief Get current shake time
         * @return Time elapsed since shake started
         */
        [[nodiscard]] float getCurrentTime() const noexcept { return currentTime_; }

        /**
         * @brief Get current shake offset
         * @return Current position offset from shake
         */
        [[nodiscard]] const Vec3& getCurrentOffset() const noexcept { return currentOffset_; }

        /**
         * @brief Get base position (position before shake)
         * @return Original camera position
         */
        [[nodiscard]] const Vec3& getBasePosition() const noexcept { return basePosition_; }

        /**
         * @brief Get current intensity (with fade applied)
         * @return Current effective intensity
         */
        [[nodiscard]] float getCurrentIntensity() const;

        /**
         * @brief Get progress (0 to 1, or -1 if infinite)
         * @return Normalized progress or -1
         */
        float getProgress() const;

        // ========================================================================
        // MUTATORS
        // ========================================================================

        /**
         * @bbrief Set shake configuration
         * @param config New shake configuration
         */
        void setConfig(ShakeConfig config);

        /**
         * @brief Set base position
         * @param position New base position
         */
        void setBasePosition(const Vec3& position) {
            basePosition_ = position;
        }

        /**
         * @brief Stop the shake effect
         */
        void stop() {
            active_ = false;
        }

        /**
         * @brief Reset shake to beginning
         */
        void reset();

        // ========================================================================
        // UPDATE
        // ========================================================================

        /**
         * @brief Update shake effect
         * @param deltaTime Time step
         * @return true if shake completed this frame
         */
        bool update(float deltaTime);

        /**
         * @brief Get the shaken position
         * @return Base position plus current offset
         */
        Vec3 getShakenPosition() const {
            return basePosition_ + currentOffset_;
        }

    private:
        CameraID cameraId_ = INVALID_CAMERA_ID; ///< Camera being shaken
        ShakeConfig config_; ///< Shake configuration
        float currentTime_ = 0.0f; ///< Current shake time
        Vec3 currentOffset_; ///< Current shake offset
        bool active_ = false; ///< Whether shake is active
        Vec3 basePosition_; ///< Original position before shake

        /**
         * @brief Generate shake offset base on pattern
         * @param pattern Shake pattern
         * @param intensity Shake intensity
         * @param time Current time
         * @param frequency Shake frequency
         * @return Shake offset vector
         */
        static Vec3 generateOffset(ShakePattern pattern, float intensity,
                                      float time, float frequency);
    };
} // namespace engine::camera
