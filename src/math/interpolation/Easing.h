/**
 * @file Easing.h
 * @brief Easing functions for smooth animation transitions
 * @author Andrés Guerrero
 * @date 27-08-2025
 *
 * Provides a comprehensive set of easing equations based on Robert Penner's
 * easing functions, commonly used in game animation and UI transitions.
 */

#pragma once

#include "../core/MathTypes.h"
#include "../core/MathConstants.h"
#include "../core/MathFunctions.h"

#include <functional>

namespace engine::math::easing {
    /**
     * @brief Easing function type
     * Takes a normalized time value [0,1] and returns an eased value
     */
    using EasingFunction = std::function<Float(Float)>;

    // ============================================================================
    // Linear (No easing)
    // ============================================================================

    [[nodiscard]] inline Float linear(const Float t) noexcept {
        return t;
    }

    // ============================================================================
    // Quadratic Easing (t^2)
    // ============================================================================

    [[nodiscard]] inline Float easeInQuad(const Float t) noexcept {
        return t * t;
    }

    [[nodiscard]] inline Float easeOutQuad(const Float t) noexcept {
        return t * (2.0f - t);
    }

    [[nodiscard]] inline Float easeInOutQuad(const Float t) noexcept {
        return t < 0.5f ? 2.0f * t * t : 1.0f - 2.0f * (1.0f - t) * (1.0f - t);
    }

    // ============================================================================
    // Cubic Easing (t^3)
    // ============================================================================

    [[nodiscard]] inline Float easeInCubic(const Float t) noexcept {
        return t * t * t;
    }

    [[nodiscard]] inline Float easeOutCubic(const Float t) noexcept {
        const Float t1 = t - 1.0f;
        return t1 * t1 * t1 + 1.0f;
    }

    [[nodiscard]] inline Float easeInOutCubic(const Float t) noexcept {
        return t < 0.5f ? 4.0f * t * t * t : 1.0f + 4.0f * (t - 1.0f) * (t - 1.0f) * (t - 1.0f);
    }

    // ============================================================================
    // Quartic Easing (t^4)
    // ============================================================================

    [[nodiscard]] inline Float easeInQuart(const Float t) noexcept {
        return t * t * t * t;
    }

    [[nodiscard]] inline Float easeOutQuart(const Float t) noexcept {
        const Float t1 = t - 1.0f;
        return 1.0f - t1 * t1 * t1 * t1;
    }

    [[nodiscard]] inline Float easeInOutQuart(const Float t) noexcept {
        if (t < 0.5f) {
            return 8.0f * t * t * t * t;
        }

        const Float t1 = t - 1.0f;
        return 1.0f - 8.0f * t1 * t1 * t1 * t1;
    }

    // ============================================================================
    // Quintic Easing (t^5)
    // ============================================================================

    [[nodiscard]] inline Float easeInQuint(const Float t) noexcept {
        return t * t * t * t * t;
    }

    [[nodiscard]] inline Float easeOutQuint(const Float t) noexcept {
        const Float t1 = t - 1.0f;

        return t1 * t1 * t1 * t1 * t1 + 1.0f;
    }

    [[nodiscard]] inline Float easeInOutQuint(const Float t) noexcept {
        if (t < 0.5f) {
            return 16.0f * t * t * t * t * t;
        }

        const Float t1 = t - 1.0f;
        return 16.0f * t1 * t1 * t1 * t1 * t1 + 1.0f;
    }

    // ============================================================================
    // Sine Easing
    // ============================================================================

    [[nodiscard]] inline Float easeInSine(const Float t) noexcept {
        return 1.0f - std::cos(t * HALF_PI<Float>);
    }

    [[nodiscard]] inline Float easeOutSine(const Float t) noexcept {
        return std::sin(t * HALF_PI<Float>);
    }

    [[nodiscard]] inline Float easeInOutSine(const Float t) noexcept {
        return 0.5f * (1.0f - std::cos(t * PI<Float>));
    }

    // ============================================================================
    // Exponential Easing
    // ============================================================================

    [[nodiscard]] inline Float easeInExpo(const Float t) noexcept {
        return t == 0.0f ? 0.0f : std::pow(2.0f, 10.0f * (t - 1.0f));
    }

    [[nodiscard]] inline Float easeOutExpo(const Float t) noexcept {
        return t == 1.0f ? 1.0f : 1.0f - std::pow(2.0f, -10.0f * t);
    }

    [[nodiscard]] inline Float easeInOutExpo(const Float t) noexcept {
        if (t == 0.0f) return 0.0f;
        if (t == 1.0f) return 1.0f;

        if (t < 0.5f) {
            return 0.5f * std::pow(2.0f, 20.0f * t - 10.0f);
        }

        return 1.0f - 0.5f * std::pow(2.0f, -20.0f * t + 10.0f);
    }

    // ============================================================================
    // Circular Easing
    // ============================================================================

    [[nodiscard]] inline Float easeInCirc(const Float t) noexcept {
        return 1.0f - std::sqrt(1.0f - t * t);
    }

    [[nodiscard]] inline Float easeOutCirc(const Float t) noexcept {
        const Float t1 = t - 1.0f;
        return std::sqrt(1.0f - t1 * t1);
    }

    [[nodiscard]] inline Float easeInOutCirc(const Float t) noexcept {
        if (t < 0.5f) {
            return 0.5f * (1.0f - std::sqrt(1.0f - 4.0f * t * t));
        }

        const Float t1 = 2.0f * t - 2.0f;
        return 0.5f * (std::sqrt(1.0f - t1 * t1) + 1.0f);
    }

    // ============================================================================
    // Elastic Easing (Spring effect)
    // ============================================================================

    [[nodiscard]] inline Float easeInElastic(const Float t) noexcept {
        if (t == 0.0f) return 0.0f;
        if (t == 1.0f) return 1.0f;

        constexpr Float p = 0.3f;
        constexpr Float a = 1.0f;
        constexpr Float s = p / 4.0f;

        const Float t1 = t - 1.0f;
        return -(a * std::pow(2.0f, 10.0f * t1) * std::sin((t1 - s) * TWO_PI<Float> / p));
    }

    [[nodiscard]] inline Float easeOutElastic(const Float t) noexcept {
        if (t == 0.0f) return 0.0f;
        if (t == 1.0f) return 1.0f;

        constexpr Float p = 0.3f;
        constexpr Float a = 1.0f;
        constexpr Float s = p / 4.0f;

        return a * std::pow(2.0f, -10.0f * t) * std::sin((t - s) * TWO_PI<Float> / p) + 1.0f;
    }

    [[nodiscard]] inline Float easeInOutElastic(const Float t) noexcept {
        if (t == 0.0f) return 0.0f;
        if (t == 1.0f) return 1.0f;

        constexpr Float p = 0.45f;
        constexpr Float a = 1.0f;
        constexpr Float s = p / 4.0f;

        if (t < 0.5f) {
            const Float t1 = 2.0f * t - 1.0f;
            return -0.5f * (a * std::pow(2.0f, 10.0f * t1) * std::sin((t1 - s) * TWO_PI<Float> / p));
        }

        const Float t1 = 2.0f * t - 1.0f;
        return a * std::pow(2.0f, -10.0f * t1) * std::sin((t1 - s) * TWO_PI<Float> / p) * 0.5f + 1.0f;
    }

    // ============================================================================
    // Back Easing (Overshoot)
    // ============================================================================

    [[nodiscard]] inline Float easeInBack(const Float t) noexcept {
        constexpr Float s = 1.70158f;
        return t * t * ((s + 1.0f) * t - s);
    }

    [[nodiscard]] inline Float easeOutBack(const Float t) noexcept {
        constexpr Float s = 1.70158f;
        const Float t1 = t - 1.0f;

        return t1 * t1 * ((s + 1.0f) * t1 + s) + 1.0f;
    }

    [[nodiscard]] inline Float easeInOutBack(const Float t) noexcept {
        constexpr Float s = 1.70158f * 1.525f;

        if (t < 0.5f) {
            const Float t1 = 2.0f * t;
            return 0.5f * (t1 * t1 * ((s + 1.0f) * t1 - s));
        }

        const Float t1 = 2.0f * t - 2.0f;
        return 0.5f * (t1 * t1 * ((s + 1.0f) * t1 + s) + 2.0f);
    }

    // ============================================================================
    // Bounce Easing
    // ============================================================================

    [[nodiscard]] inline Float easeOutBounce(const Float t) noexcept {
        if (t < 1.0f / 2.75f) {
            return 7.5625f * t * t;
        }

        if (t < 2.0f / 2.75f) {
            const Float t1 = t - 1.5f / 2.75f;
            return 7.5625f * t1 * t1 + 0.75f;
        }

        if (t < 2.5f / 2.75f) {
            const Float t1 = t - 2.25f / 2.75f;
            return 7.5625f * t1 * t1 + 0.9375f;
        }

        const Float t1 = t - 2.625f / 2.75f;
        return 7.5625f * t1 * t1 + 0.984375f;
    }

    [[nodiscard]] inline Float easeInBounce(const Float t) noexcept {
        return 1.0f - easeOutBounce(1.0f - t);
    }

    [[nodiscard]] inline Float easeInOutBounce(const Float t) noexcept {
        if (t < 0.5f) {
            return 0.5f * easeInBounce(2.0f * t);
        }

        return 0.5f * easeOutBounce(2.0f * t - 1.0f) + 0.5f;
    }

    // ============================================================================
    // Custom Parametric Easing
    // ============================================================================

    /**
     * @brief Power easing with custom exponent
     */
    [[nodiscard]] inline Float easePower(const Float t, const Float power) noexcept {
        return std::pow(t, power);
    }

    /**
     * @brief Elastic easing with custom amplitude and period
     */
    [[nodiscard]] inline Float easeElasticCustom(const Float t, const Float amplitude, const Float period) noexcept {
        if (t == 0.0f) return 0.0f;
        if (t == 1.0f) return 1.0f;

        const Float s = period / (TWO_PI<Float>) * std::asin(1.0f / amplitude);
        const Float t1 = t - 1.0f;

        return amplitude * std::pow(2.0f, -10.0f * t1) *
               std::sin((t1 - s) * TWO_PI<Float> / period) + 1.0f;
    }

    /**
     * @brief Back easing with custom overshoot amount
     */
    [[nodiscard]] inline Float easeBackCustom(const Float t, const Float overshoot) noexcept {
        const Float t1 = t - 1.0f;
        return t1 * t1 * ((overshoot + 1.0f) * t1 + overshoot) + 1.0f;
    }

    // ============================================================================
    // Easing Function Selector
    // ============================================================================

    enum class EaseType {
        LINEAR,
        QUAD_IN, QUAD_OUT, QUAD_IN_OUT,
        CUBIC_IN, CUBIC_OUT, CUBIC_IN_OUT,
        QUART_IN, QUART_OUT, QUART_IN_OUT,
        QUINT_IN, QUINT_OUT, QUINT_IN_OUT,
        SINE_IN, SINE_OUT, SINE_IN_OUT,
        EXPO_IN, EXPO_OUT, EXPO_IN_OUT,
        CIRC_IN, CIRC_OUT, CIRC_IN_OUT,
        ELASTIC_IN, ELASTIC_OUT, ELASTIC_IN_OUT,
        BACK_IN, BACK_OUT, BACK_IN_OUT,
        BOUNCE_IN, BOUNCE_OUT, BOUNCE_IN_OUT
    };

    /**
     * @brief Get easing function by type
     */
    [[nodiscard]] inline EasingFunction getEasingFunction(const EaseType type) noexcept {
        switch (type) {
        case EaseType::LINEAR: return linear;

        case EaseType::QUAD_IN: return easeInQuad;
        case EaseType::QUAD_OUT: return easeOutQuad;
        case EaseType::QUAD_IN_OUT: return easeInOutQuad;

        case EaseType::CUBIC_IN: return easeInCubic;
        case EaseType::CUBIC_OUT: return easeOutCubic;
        case EaseType::CUBIC_IN_OUT: return easeInOutCubic;

        case EaseType::QUART_IN: return easeInQuart;
        case EaseType::QUART_OUT: return easeOutQuart;
        case EaseType::QUART_IN_OUT: return easeInOutQuart;

        case EaseType::QUINT_IN: return easeInQuint;
        case EaseType::QUINT_OUT: return easeOutQuint;
        case EaseType::QUINT_IN_OUT: return easeInOutQuint;

        case EaseType::SINE_IN: return easeInSine;
        case EaseType::SINE_OUT: return easeOutSine;
        case EaseType::SINE_IN_OUT: return easeInOutSine;

        case EaseType::EXPO_IN: return easeInExpo;
        case EaseType::EXPO_OUT: return easeOutExpo;
        case EaseType::EXPO_IN_OUT: return easeInOutExpo;

        case EaseType::CIRC_IN: return easeInCirc;
        case EaseType::CIRC_OUT: return easeOutCirc;
        case EaseType::CIRC_IN_OUT: return easeInOutCirc;

        case EaseType::ELASTIC_IN: return easeInElastic;
        case EaseType::ELASTIC_OUT: return easeOutElastic;
        case EaseType::ELASTIC_IN_OUT: return easeInOutElastic;

        case EaseType::BACK_IN: return easeInBack;
        case EaseType::BACK_OUT: return easeOutBack;
        case EaseType::BACK_IN_OUT: return easeInOutBack;

        case EaseType::BOUNCE_IN: return easeInBounce;
        case EaseType::BOUNCE_OUT: return easeOutBounce;
        case EaseType::BOUNCE_IN_OUT: return easeInOutBounce;

        default: return linear;
        }
    }

    /**
     * @brief Apply easing to any interpolatable type
     */
    template <typename T>
    [[nodiscard]] inline T ease(const T& start, const T& end, const Float t, const EaseType type) noexcept {
        Float easedT = getEasingFunction(type)(saturate(t));
        return lerp(start, end, easedT);
    }

    /**
     * @brief Chain multiple easing functions
     */
    [[nodiscard]] inline Float chainEasing(const Float t, const std::vector<EasingFunction>& functions) noexcept {
        Float result = t;

        for (const auto& func : functions) {
            result = func(result);
        }

        return result;
    }
} // namespace engine::math::easing
