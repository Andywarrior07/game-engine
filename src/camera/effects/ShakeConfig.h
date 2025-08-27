/**
 * @file ShakeConfig.h
 * @brief Camerashake configuration structure
 * @author Andrés Guerrero
 * @date 25-08-2025
 */
#pragma once

#include "ShakePattern.h"
#include "../core/CameraTypes.h"

namespace engine::camera {
    /**
     * @brief Camera shake configuration for screen effects
     *
     * Defines all parameters needed to create camera shake effects,
     * including intensity, duration, pattern, and axis control
     */
    struct ShakeConfig {
        float intensity = 1.0f; ///< Shake intensity/amplitude
        float frequency = 30.0f; ///< Shake frequency in Hz
        float duration = 1.0f; ///< Duration in seconds (-1 for infinite)
        ShakePattern pattern = ShakePattern::RANDOM; ///< Shake pattern type
        Vector3 axes{1.0f, 1.0f, 0.5f}; ///< Relative shake strength per axis
        bool fadeOut = true; ///< Whether to fade out over time
        bool fadeIn = false; ///< Whether to fade in at start
        float fadeInDuration = 0.1f; ///< Fade in duration in seconds
        float fadeOutDuration = 0.3f; ///< Fade out duration in seconds
        ShakeCallback onComplete = nullptr; ///< Callback when shake completes

        /**
         * @brief Default constructor
         */
        ShakeConfig() = default;

        /**
         * @brief Constructor with basic parameters
         * @param intens Shake intensity
         * @param dur Duration in seconds
         * @param pat Shake pattern
         */
        ShakeConfig(float intens, float dur, ShakePattern pat = ShakePattern::RANDOM)
            : intensity(intens), duration(dur), pattern(pat) {
        }

        /**
         * @brief Create a preset explosion shake
         * @param intensity Explosion intensity
         * @return Configured shake for explosion
         */
        static ShakeConfig explosion(const float intensity = 5.0f) {
            ShakeConfig config;
            config.intensity = intensity;
            config.duration = 0.5f;
            config.pattern = ShakePattern::EXPLOSION;
            config.frequency = 50.0f;
            config.fadeOut = true;
            config.fadeIn = false;
            return config;
        }

        /**
         * @brief Create a preset earthquake shake
         * @param intensity Earthquake intensity
         * @param duration Duration of earthquake
         * @return Configured shake for earthquake
         */
        static ShakeConfig earthquake(const float intensity = 2.0f, float duration = 3.0f) {
            ShakeConfig config;
            config.intensity = intensity;
            config.duration = duration;
            config.pattern = ShakePattern::EARTHQUAKE;
            config.frequency = 5.0f;
            config.fadeOut = true;
            config.fadeIn = true;
            config.fadeInDuration = 0.5f;
            config.fadeOutDuration = 1.0f;
            config.axes = Vector3(1.0f, 0.3f, 0.1f);
            return config;
        }

        /**
         * @brief Create a present handheld camera shake
         * @param intensity Handheld movement intensity
         * @return Configured shake for handheld effect
         */
        static ShakeConfig handheld(const float intensity = 0.3f) {
            ShakeConfig config;
            config.intensity = intensity;
            config.duration = -1.0f; // Infinite
            config.pattern = ShakePattern::HANDHELD;
            config.frequency = 2.0f;
            config.fadeOut = false;
            config.fadeIn = true;
            config.fadeInDuration = 1.0f;
            return config;
        }

        /**
         * @brief Create a preset impact shake
         * @param intensity Impact force
         * @param direction Direction of impact (normalized automatically)
         * @return Configured shake for impact
        */
        static ShakeConfig impact(const float intensity = 3.0f, const Vector3& direction = Vector3(1, 0, 0)) {
            ShakeConfig config;
            config.intensity = intensity;
            config.duration = 0.3f;
            config.pattern = ShakePattern::IMPACT;
            config.frequency = 40.0f;
            config.fadeOut = true;
            config.axes = glm::normalize(direction);
            return config;
        }

        /**
         * @brief Create a preset vibration shake
         * @param intensity Vibration intensity
         * @param duration Vibration duration
         * @return Configured shake for vibration
         */
        static ShakeConfig vibration(float intensity = 0.5f, float duration = 0.2f) {
            ShakeConfig config;
            config.intensity = intensity;
            config.duration = duration;
            config.pattern = ShakePattern::VIBRATION;
            config.frequency = 100.0f;
            config.fadeOut = false;
            return config;
        }

        /**
         * @brief Validate configuration parameters
         * @return true if configuration is valid
         */
        bool validate() const {
            return intensity >= 0.0f &&
                frequency > 0.0f &&
                (duration > 0.0f || duration == -1.0f) &&
                fadeInDuration >= 0.0f &&
                fadeOutDuration >= 0.0f;
        }
    };
} // namespace engine::camera
