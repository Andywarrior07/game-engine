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
        [[nodiscard]] const Vector3& getCurrentOffset() const noexcept { return currentOffset_; }

        /**
         * @brief Get base position (position before shake)
         * @return Original camera position
         */
        [[nodiscard]] const Vector3& getBasePosition() const noexcept { return basePosition_; }

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
        void setBasePosition(const Vector3& position) {
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
        Vector3 getShakenPosition() const {
            return basePosition_ + currentOffset_;
        }

    private:
        CameraID cameraId_ = INVALID_CAMERA_ID; ///< Camera being shaken
        ShakeConfig config_; ///< Shake configuration
        float currentTime_ = 0.0f; ///< Current shake time
        Vector3 currentOffset_; ///< Current shake offset
        bool active_ = false; ///< Whether shake is active
        Vector3 basePosition_; ///< Original position before shake

        /**
         * @brief Generate shake offset base on pattern
         * @param pattern Shake pattern
         * @param intensity Shake intensity
         * @param time Current time
         * @param frequency Shake frequency
         * @return Shake offset vector
         */
        static Vector3 generateOffset(ShakePattern pattern, float intensity,
                                      float time, float frequency) {
            thread_local std::mt19937 generator(std::random_device{}());
            thread_local std::uniform_real_distribution<float> distribution(-1.0f, 1.0f);

            Vector3 offset;

            switch (pattern) {
                case ShakePattern::RANDOM: {
                    offset.x = distribution(generator) * intensity;
                    offset.y = distribution(generator) * intensity;
                    offset.z = distribution(generator) * intensity * 0.5f;
                    break;
                }

                case ShakePattern::HORIZONTAL: {
                    offset.x = distribution(generator) * intensity;
                    offset.y = 0.0f;
                    offset.z = 0.0f;
                    break;
                }

                case ShakePattern::VERTICAL: {
                    offset.x = 0.0f;
                    offset.y = distribution(generator) * intensity;
                    offset.z = 0.0f;
                    break;
                }

                case ShakePattern::CIRCULAR: {
                    float angle = time * frequency * 2.0f * M_PI;
                    offset.x = std::cos(angle) * intensity;
                    offset.y = std::sin(angle) * intensity;
                    offset.z = 0.0f;
                    break;
                }

                case ShakePattern::EXPLOSION: {
                    float explosionT = std::max(0.0f, 1.0f - time * 2.0f);
                    float explosionIntensity = intensity * explosionT * explosionT;
                    offset.x = distribution(generator) * explosionIntensity;
                    offset.y = distribution(generator) * explosionIntensity;
                    offset.z = distribution(generator) * explosionIntensity * 0.3f;
                    break;
                }

                case ShakePattern::EARTHQUAKE: {
                    float lowFreq = frequency * 0.3f;
                    float noise1 = std::sin(time * lowFreq * 2.0f * M_PI) * intensity;
                    float noise2 = std::sin(time * lowFreq * 2.0f * M_PI * 1.7f) * intensity * 0.7f;
                    offset.x = noise1 + distribution(generator) * intensity * 0.2f;
                    offset.y = noise2 + distribution(generator) * intensity * 0.1f;
                    offset.z = distribution(generator) * intensity * 0.1f;
                    break;
                }

                case ShakePattern::HANDHELD: {
                    // Low frequency, smooth movement
                    float angle1 = time * frequency;
                    float angle2 = time * frequency * 1.3f;
                    offset.x = std::sin(angle1) * intensity * 0.5f;
                    offset.y = std::sin(angle2) * intensity * 0.3f;
                    offset.z = std::sin(angle1 * 0.7f) * intensity * 0.1f;
                    break;
                }

                case ShakePattern::VIBRATION: {
                    // High frequency, low amplitude
                    offset.x = distribution(generator) * intensity * 0.3f;
                    offset.y = distribution(generator) * intensity * 0.3f;
                    offset.z = 0.0f;
                    break;
                }

                case ShakePattern::IMPACT: {
                    // Single direction with decay
                    float decay = std::max(0.0f, 1.0f - time * 5.0f);
                    offset = Vector3(intensity * decay, 0.0f, 0.0f);
                    break;
                }

                case ShakePattern::WAVE: {
                    float wave = std::sin(time * frequency * 2.0f * M_PI);
                    offset.x = wave * intensity;
                    offset.y = wave * intensity * 0.5f;
                    offset.z = 0.0f;
                    break;
                }

                default:
                    offset = math::constants::VEC3_ZERO;
                    break;
            }

            return offset;
        }
    };
} // namespace engine::camera
