/**
* @file TransitionEasing.cpp
 * @brief Easing functions for camera transitions
 * @author Andrés Guerrero
 * @date 26-08-2025
 */

#include "TransitionEasing.h"

namespace engine::camera {
    float TransitionEasing::ease(const TransitionType type, float t) {
        t = std::clamp(t, 0.0f, 1.0f);

        switch (type) {
        case TransitionType::LINEAR:
            return math::easing::linear(t);
        case TransitionType::EASE_IN:
            return math::easing::easeInQuad(t);
        case TransitionType::EASE_OUT:
            return math::easing::easeOutQuad(t);
        case TransitionType::EASE_IN_OUT:
            return math::easing::easeInOutQuad(t);
        case TransitionType::BOUNCE:
            return math::easing::easeOutBounce(t);
        case TransitionType::ELASTIC:
            return math::easing::easeInElastic(t);
        default:
            return math::easing::linear(t);
        }
    }
} // namespace engine::camera
