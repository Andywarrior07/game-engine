/**
 * @file TransitionConfig.h
 * @brief Camera transition configuration
 * @author Andrés Guerrero
 * @date 26-08-2025
 */

#pragma once

#include "../core/CameraTypes.h"

namespace engine::camera {
    /**
     * @brief Camera transition configuration for smooth movement
     *
     * Defines parameters for camera transitions including duration,
     * easing type, and completion callbacks.
     */
    struct TransitionConfig {
        float duration = 1.0f;                              ///< Transition duration in seconds
        TransitionType type = TransitionType::EASE_IN_OUT;  ///< Easing type
        bool relative = false;                              ///< Whether target is relative to current position
        bool interruptible = true;                          ///< Whether transition can be interrupted
        float delay = 0.0f;                                 ///< Delay before starting transition
        TransitionCallback onComplete = nullptr;            ///< Callback when transition completes
        TransitionCallback onInterrupt = nullptr;           ///< Callback when transition is interrupted

        /**
         * @brief Default constructor
         */
        TransitionConfig() = default;

        /**
         * @brief Constructor with duration and type
         * @param dur Duration in seconds
         * @param typ Transition type
         */
        explicit TransitionConfig(const float dur, const TransitionType typ = TransitionType::EASE_IN_OUT)
            : duration(dur), type(typ) {}

        /**
         * @brief Create a linear transition config
         * @param duration Duration in seconds
         * @return Linear transition configuration
         */
        static TransitionConfig linear(const float duration = 1.0f) {
            return TransitionConfig(duration, TransitionType::LINEAR);
        }

        /**
         * @brief Create an ease-in transition config
         * @param duration Duration in seconds
         * @return Ease-in transition configuration
         */
        static TransitionConfig easeIn(const float duration = 1.0f) {
            return TransitionConfig(duration, TransitionType::EASE_IN);
        }

        /**
         * @brief Create an ease-out transition config
         * @param duration Duration in seconds
         * @return Ease-out transition configuration
         */
        static TransitionConfig easeOut(const float duration = 1.0f) {
            return TransitionConfig(duration, TransitionType::EASE_OUT);
        }

        /**
         * @brief Create an ease-in-out transition config
         * @param duration Duration in seconds
         * @return Ease-in-out transition configuration
         */
        static TransitionConfig easeInOut(const float duration = 1.0f) {
            return TransitionConfig(duration, TransitionType::EASE_IN_OUT);
        }

        /**
         * @brief Create a bounce transition config
         * @param duration Duration in seconds
         * @return Bounce transition configuration
         */
        static TransitionConfig bounce(const float duration = 1.0f) {
            return TransitionConfig(duration, TransitionType::BOUNCE);
        }

        /**
         * @brief Create an elastic transition config
         * @param duration Duration in seconds
         * @return Elastic transition configuration
         */
        static TransitionConfig elastic(const float duration = 1.0f) {
            return TransitionConfig(duration, TransitionType::ELASTIC);
        }

        /**
         * @brief Create an instant transition (no animation)
         * @return Instant transition configuration
         */
        static TransitionConfig instant() {
            return TransitionConfig(0.0f, TransitionType::LINEAR);
        }

        /**
         * @brief Validate configuration
         * @return true if configuration is valid
         */
        bool validate() const {
            return duration >= 0.0f && delay >= 0.0f;
        }

        /**
         * @brief Check if this is an instant transition
         * @return true if duration is zero
         */
        bool isInstant() const {
            return duration <= 0.0f;
        }

        /**
         * @brief Get total time including delay
         * @return Total transition time
         */
        float getTotalTime() const {
            return delay + duration;
        }
    };
} // namespace engine::camera
