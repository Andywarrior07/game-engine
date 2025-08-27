/**
 * @file TransitionEasing.h
 * @brief Easing functions for camera transitions
 * @author Andrés Guerrero
 * @date 26-08-2025
 */

#pragma once

#include "../core/CameraTypes.h"

namespace engine::camera {
    /**
     * @brief Collection of easing functions for smooth camera transitions
     *
     * Provides various mathematical easing functions to create
     * smooth and visually appealing camera movements.
     */
    class TransitionEasing {
    public:
        /**
         * @brief Calculate easing value based on transition type
         * @param type Type of transition
         * @param t Normalized time (0 to 1)
         * @return Eased value (0 to 1)
         */
        static float ease(const TransitionType type, float t);
    };
} // namespace engine::camera
