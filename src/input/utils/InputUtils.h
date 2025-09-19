/**
 * @file InputUtils.h
 * @brief Utility functions for input processing (uses Vec2 = glm::vec2)
 * @author Andrés Guerrero
 * @date 12-09-2025
 *
 * This is a drop-in variant of your InputUtils that uses `Vec2` (glm::vec2)
 * instead of the custom `Vector2` type. It preserves the original API and
 * behavior but delegates vector math to GLM (length/normalize/dot).
 */

#pragma once

#include "../core/InputTypes.h"

#include "../../math/core/MathTypes.h"
#include "../../math/core/MathFunctions.h"
#include "../../math/utils/Random.h"
#include "../../math/simd/SimdTypes.h"

#include <array>
#include <cmath>
#include <numeric>

namespace engine::input::utils {
    // If you prefer, you can uncomment the following fallback alias. If you
    // already have `using Vec2 = glm::vec2;` elsewhere, leave it commented.
    // namespace detail { using Vec2Alias = glm::vec2; }
    // using Vec2 = detail::Vec2Alias;

    // ==========================================================================
    // Deadzone Processing
    // ==========================================================================

    [[nodiscard]] inline math::Vec2 applyRadialDeadzone(const math::Vec2& stick,
                                                        const float innerDeadzone = 0.15f,
                                                        const float outerDeadzone = 0.95f) noexcept {
        const float magnitude = glm::length(stick);

        if (magnitude < innerDeadzone) {
            return math::Vec2(0.0f, 0.0f);
        }

        if (magnitude > outerDeadzone) {
            return glm::normalize(stick);
        }

        const float scaledMagnitude = (magnitude - innerDeadzone) / (outerDeadzone - innerDeadzone);
        return glm::normalize(stick) * scaledMagnitude;
    }

    [[nodiscard]] inline math::Vec2 applyAxialDeadzone(const math::Vec2& stick,
                                                       const float deadzoneX = 0.15f,
                                                       const float deadzoneY = 0.15f) noexcept {
        math::Vec2 result(0.0f, 0.0f);
        result.x = std::abs(stick.x) < deadzoneX ? 0.0f : stick.x;
        result.y = std::abs(stick.y) < deadzoneY ? 0.0f : stick.y;
        return result;
    }

    [[nodiscard]] inline math::Vec2 applyHybridDeadzone(const math::Vec2& stick,
                                                        const DeadzoneSettings& settings) noexcept {
        // First apply radial deadzone for circular feel
        const math::Vec2 radial = applyRadialDeadzone(stick, settings.innerDeadzone, settings.outerDeadzone);

        // Then apply slight axial correction for cardinal directions
        constexpr float AXIAL_BLEND = 0.1f;
        const math::Vec2 axial = applyAxialDeadzone(stick, settings.innerDeadzone, settings.innerDeadzone);

        return math::lerp(radial, axial, AXIAL_BLEND);
    }

    // ==========================================================================
    // Sensitivity and Response Curves
    // ==========================================================================

    [[nodiscard]] inline float applyResponseCurve(const float value,
                                                  const ResponseCurve curve = ResponseCurve::LINEAR,
                                                  const float strength = 1.0f) noexcept {
        const float absValue = std::abs(value);
        const float sign = value < 0 ? -1.0f : 1.0f;
        float result = absValue;

        switch (curve) {
        case ResponseCurve::LINEAR:
            break;

        case ResponseCurve::QUADRATIC:
            result = math::lerp(absValue, absValue * absValue, strength);
            break;

        case ResponseCurve::CUBIC:
            result = math::lerp(absValue, absValue * absValue * absValue, strength);
            break;

        case ResponseCurve::EXPONENTIAL:
            result = math::lerp(absValue, (std::exp(absValue) - 1.0f) / (std::exp(1.0f) - 1.0f), strength);
            break;

        case ResponseCurve::LOGARITHMIC:
            result = math::lerp(absValue, std::log(1.0f + absValue * 9.0f) / std::log(10.0f), strength);
            break;

        default:
            break;
        }

        return result * sign;
    }

    [[nodiscard]] inline math::Vec2 applyAcceleration(const math::Vec2& delta,
                                                      const float sensitivity = 1.0f,
                                                      const float acceleration = 1.5f) noexcept {
        const float speed = glm::length(delta);
        const float multiplier = sensitivity * std::pow(speed, acceleration - 1.0f);
        return delta * multiplier;
    }

    // ==========================================================================
    // Filtering and Smoothing
    // ==========================================================================

    template <std::size_t WindowSize = 5>
    class MovingAverageFilter {
    public:
        MovingAverageFilter() noexcept : index_(0), sum_(0), count_(0) {
            buffer_.fill(0);
        }

        [[nodiscard]] float filter(const float value) noexcept {
            sum_ -= buffer_[index_];
            buffer_[index_] = value;
            sum_ += value;

            index_ = (index_ + 1) % WindowSize;
            if (count_ < WindowSize) count_++;

            return sum_ / static_cast<float>(count_);
        }

        void reset() noexcept {
            buffer_.fill(0);
            index_ = 0;
            sum_ = 0;
            count_ = 0;
        }

    private:
        std::array<float, WindowSize> buffer_;
        std::size_t index_;
        float sum_;
        std::size_t count_;
    };

    class ExponentialFilter {
    public:
        explicit ExponentialFilter(const float alpha = 0.1f) noexcept
            : alpha_(math::clamp(alpha, 0.0f, 1.0f)), value_(0), initialized_(false) {
        }

        [[nodiscard]] float filter(const float input) noexcept {
            if (!initialized_) {
                value_ = input;
                initialized_ = true;
            }
            else {
                value_ = alpha_ * input + (1.0f - alpha_) * value_;
            }
            return value_;
        }

        void reset() noexcept {
            value_ = 0;
            initialized_ = false;
        }

        void setAlpha(const float alpha) noexcept {
            alpha_ = math::clamp(alpha, 0.0f, 1.0f);
        }

    private:
        float alpha_;
        float value_;
        bool initialized_;
    };

    class OneEuroFilter {
    public:
        explicit OneEuroFilter(const float minCutoff = 1.0f,
                               const float beta = 0.007f,
                               const float dCutoff = 1.0f) noexcept
            : minCutoff_(minCutoff), beta_(beta), dCutoff_(dCutoff),
              xFilter_(alpha(minCutoff)), dxFilter_(alpha(dCutoff)), lastValue_(0),
              lastTime_(0), initialized_(false) {
        }

        [[nodiscard]] float filter(const float value, const float dt) noexcept {
            if (!initialized_) {
                initialized_ = true;
                xFilter_ = ExponentialFilter(alpha(minCutoff_));
                dxFilter_ = ExponentialFilter(alpha(dCutoff_));
                lastValue_ = value;
                return value;
            }

            // Estimate derivative
            const float dx = (value - lastValue_) / dt;
            const float edx = dxFilter_.filter(dx);

            // Adapt cutoff frequency based on speed
            const float cutoff = minCutoff_ + beta_ * std::abs(edx);
            xFilter_.setAlpha(alpha(cutoff));

            lastValue_ = value;
            return xFilter_.filter(value);
        }

        void reset() noexcept {
            initialized_ = false;
            xFilter_.reset();
            dxFilter_.reset();
        }

    private:
        [[nodiscard]] static float alpha(const float cutoff) noexcept {
            const float tau = 1.0f / (2.0f * math::PI<float> * cutoff);
            return 1.0f / (1.0f + tau);
        }

        float minCutoff_;
        float beta_;
        float dCutoff_;
        ExponentialFilter xFilter_;
        ExponentialFilter dxFilter_;
        float lastValue_;
        // TODO: Ver porque no se usa
        float lastTime_;
        bool initialized_;
    };

    // ==========================================================================
    // SIMD-Accelerated Batch Processing
    // ==========================================================================

    inline void batchProcessAnalogInputs(float* inputs, const std::size_t count,
                                         const float deadzone = 0.15f,
                                         const float sensitivity = 1.0f) noexcept {
        using namespace engine::math::simd;

        const Float4 deadzoneVec(deadzone);
        const Float4 sensitivityVec(sensitivity);

        const std::size_t simdCount = count & ~3;
        for (std::size_t i = 0; i < simdCount; i += 4) {
            Float4 values = Float4::loadAligned(&inputs[i]);

            auto absValues = Float4(
                std::abs(values[0]),
                std::abs(values[1]),
                std::abs(values[2]),
                std::abs(values[3])
            );

            Float4 mask = absValues > deadzoneVec;
            values = values * mask;

            values = values * sensitivityVec;

            values = Float4(
                math::clamp(values[0], -1.0f, 1.0f),
                math::clamp(values[1], -1.0f, 1.0f),
                math::clamp(values[2], -1.0f, 1.0f),
                math::clamp(values[3], -1.0f, 1.0f)
            );

            values.storeAligned(&inputs[i]);
        }

        for (std::size_t i = simdCount; i < count; ++i) {
            if (std::abs(inputs[i]) < deadzone) {
                inputs[i] = 0.0f;
            }
            else {
                inputs[i] = math::clamp(inputs[i] * sensitivity, -1.0f, 1.0f);
            }
        }
    }

    // ==========================================================================
    // Input Validation and Sanitization
    // ==========================================================================

    [[nodiscard]] inline math::Vec2 clampToUnitCircle(const math::Vec2& stick) noexcept {
        if (const float magnitude = glm::length(stick); magnitude <= 1.0f) return stick;

        return glm::normalize(stick);
    }

    [[nodiscard]] inline float sanitizeTrigger(const float value) noexcept {
        return math::saturate(value);
    }

    [[nodiscard]] inline bool isValidInput(const float value,
                                           const float min = -1.0f,
                                           const float max = 1.0f) noexcept {
        return value >= min && value <= max && std::isfinite(value);
    }

    // ==========================================================================
    // Gesture Detection Helpers
    // ==========================================================================

    [[nodiscard]] inline math::Vec2 detectSwipeDirection(const math::Vec2& delta,
                                                         const float minDistance = 50.0f) noexcept {
        if (const float distance = glm::length(delta); distance < minDistance) {
            return math::Vec2(0.0f, 0.0f);
        }

        return glm::normalize(delta);
    }

    [[nodiscard]] inline bool isTap(const math::Vec2& delta,
                                    const float maxDistance = 10.0f) noexcept {
        return glm::dot(delta, delta) <= (maxDistance * maxDistance);
    }

    [[nodiscard]] inline float calculatePinchScale(const float oldDistance,
                                                   const float newDistance) noexcept {
        if (oldDistance < math::EPSILON) return 1.0f;
        return newDistance / oldDistance;
    }

    // ==========================================================================
    // Timing Utilities
    // ==========================================================================

    [[nodiscard]] inline bool isDoubleClick(const InputTimestamp& lastClickTime,
                                            const InputTimestamp& currentTime,
                                            const float threshold = 0.3f) noexcept {
        const auto duration = currentTime - lastClickTime;
        const float seconds = std::chrono::duration<float>(duration).count();
        return seconds <= threshold;
    }

    [[nodiscard]] inline bool isHeld(const InputTimestamp& startTime,
                                     const InputTimestamp& currentTime,
                                     const float holdThreshold = 0.5f) noexcept {
        const auto duration = currentTime - startTime;
        const float seconds = std::chrono::duration<float>(duration).count();
        return seconds >= holdThreshold;
    }

    // ==========================================================================
    // Input Mapping Helpers
    // ==========================================================================

    [[nodiscard]] inline bool axisToButton(const float axisValue,
                                           const float threshold = 0.5f,
                                           const bool positive = true) noexcept {
        if (positive) {
            return axisValue >= threshold;
        }
        else {
            return axisValue <= -threshold;
        }
    }

    [[nodiscard]] inline float buttonsToAxis(const bool negative,
                                             const bool positive) noexcept {
        float value = 0.0f;
        if (negative) value -= 1.0f;
        if (positive) value += 1.0f;
        return value;
    }

    [[nodiscard]] inline math::Vec2 buttonsToVector2D(const bool up, const bool down,
                                                      const bool left, const bool right) noexcept {
        math::Vec2 result(0.0f, 0.0f);
        result.x = buttonsToAxis(left, right);
        result.y = buttonsToAxis(down, up);

        if (glm::dot(result, result) > 1.0f) {
            result = glm::normalize(result);
        }

        return result;
    }

    // ==========================================================================
    // Debug and Visualization
    // ==========================================================================

    [[nodiscard]] inline float generateInputHeat(const float value,
                                                 const float decay = 2.0f,
                                                 const float dt = 0.016f) noexcept {
        static float heat = 0.0f;

        heat = math::saturate(heat + std::abs(value));

        heat *= std::exp(-decay * dt);

        return heat;
    }
} // namespace engine::input
