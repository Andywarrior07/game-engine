/**
 * @file TimeMath.h
 * @brief Optimized mathematical operations for time calculations
 * @details Provides high-performance time arithmetic, interpolation, and
 *          statistical functions optimized for game engine timing needs.
 *          Uses SIMD intrinsics where available and cache-friendly algorithms.
 *
 * @author Development Team
 * @date Created on 2024-01-20
 */

#pragma once

#include "../core/TimeTypes.h"
#include "TimeConverter.h"

#include <algorithm>
#include <numeric>
#include <span>
#include <vector>
#include <optional>

namespace engine::time {

    // =============================================================================
    // Interpolation Types
    // =============================================================================

    /**
     * @brief Interpolation methods for time-based animations
     */
    enum class InterpolationType : std::uint8_t {
        LINEAR, // Linear interpolation
        EASE_IN, // Quadratic ease in
        EASE_OUT, // Quadratic ease out
        EASE_IN_OUT, // Quadratic ease in/out
        CUBIC_IN, // Cubic ease in
        CUBIC_OUT, // Cubic ease out
        CUBIC_IN_OUT, // Cubic ease in/out
        SINE_IN, // Sinusoidal ease in
        SINE_OUT, // Sinusoidal ease out
        SINE_IN_OUT, // Sinusoidal ease in/out
        EXPONENTIAL_IN, // Exponential ease in
        EXPONENTIAL_OUT, // Exponential ease out
        ELASTIC, // Elastic bounce
        BACK, // Back overshoot
        BOUNCE // Bounce effect
    };

    // =============================================================================
    // TimeMath Class
    // =============================================================================

    /**
     * @brief High-performance time mathematics utilities
     * @details Static class providing optimized time calculations,
     *          interpolation, and statistical operations.
     */
    class TimeMath {
    public:
        // =============================================================================
        // Basic Arithmetic
        // =============================================================================

        /**
         * @brief Add multiple durations efficiently
         * @param durations Span of durations to sum
         * @return Sum of all durations
         * @details Uses parallel reduction for large arrays
         */
        [[nodiscard]] static Duration sum(std::span<const Duration> durations) noexcept;

        /**
         * @brief Calculate average of durations
         * @param durations Span of durations
         * @return Average duration or zero if empty
         */
        [[nodiscard]] static Duration average(std::span<const Duration> durations) noexcept;

        /**
         * @brief Calculate weighted average of durations
         * @param durations Span of durations
         * @param weights Corresponding weights
         * @return Weighted average
         */
        [[nodiscard]] static Duration weightedAverage(
                std::span<const Duration> durations,
                std::span<const double> weights
                ) noexcept;

        /**
         * @brief Scale duration by factor with overflow protection
         * @param duration Base duration
         * @param scale Scale factor
         * @return Scaled duration clamped to valid range
         */
        [[nodiscard]] static constexpr Duration scale(
                const Duration& duration,
                double scale
                ) noexcept {
            if (scale <= 0.0)
                return Duration::zero();
            if (scale == 1.0)
                return duration;

            const double scaled = toSeconds(duration) * scale;

            // Clamp to valid range
            if (scaled >= toSeconds(Duration::max())) {
                return Duration::max();
            }
            if (scaled <= toSeconds(Duration::min())) {
                return Duration::min();
            }

            return fromSeconds(scaled);
        }

        /**
         * @brief Modulo operation for durations
         * @param duration Dividend duration
         * @param divisor Divisor duration
         * @return Remainder after division
         */
        [[nodiscard]] static constexpr Duration modulo(
                const Duration& duration,
                const Duration& divisor
                ) noexcept {
            if (divisor == Duration::zero()) {
                return Duration::zero();
            }
            return duration % divisor;
        }

        // =============================================================================
        // Interpolation
        // =============================================================================

        /**
         * @brief Linear interpolation between durations
         * @param start Start duration
         * @param end End duration
         * @param t Interpolation factor [0, 1]
         * @return Interpolated duration
         */
        [[nodiscard]] static constexpr Duration lerp(
                const Duration& start,
                const Duration& end,
                double t
                ) noexcept {
            t = std::clamp(t, 0.0, 1.0);
            const double startSec = toSeconds(start);
            const double endSec = toSeconds(end);
            return fromSeconds(startSec + (endSec - startSec) * t);
        }

        /**
         * @brief Smooth interpolation with easing
         * @param start Start duration
         * @param end End duration
         * @param t Interpolation factor [0, 1]
         * @param type Easing type
         * @return Interpolated duration with easing
         */
        [[nodiscard]] static Duration interpolate(
                const Duration& start,
                const Duration& end,
                double t,
                InterpolationType type = InterpolationType::LINEAR
                ) noexcept;

        /**
         * @brief Calculate inverse lerp (get t from value)
         * @param start Start duration
         * @param end End duration
         * @param value Current value
         * @return Interpolation factor t
         */
        [[nodiscard]] static constexpr double inverseLerp(
                const Duration& start,
                const Duration& end,
                const Duration& value
                ) noexcept {
            const double range = toSeconds(end) - toSeconds(start);
            if (std::abs(range) < 1e-9)
                return 0.0;

            const double t = (toSeconds(value) - toSeconds(start)) / range;
            return std::clamp(t, 0.0, 1.0);
        }

        /**
         * @brief Remap duration from one range to another
         * @param value Value to remap
         * @param inStart Input range start
         * @param inEnd Input range end
         * @param outStart Output range start
         * @param outEnd Output range end
         * @return Remapped duration
         */
        [[nodiscard]] static Duration remap(
                const Duration& value,
                const Duration& inStart,
                const Duration& inEnd,
                const Duration& outStart,
                const Duration& outEnd
                ) noexcept;

        // =============================================================================
        // Statistical Operations
        // =============================================================================

        /**
         * @brief Calculate standard deviation
         * @param durations Sample durations
         * @return Standard deviation
         */
        [[nodiscard]] static Duration standardDeviation(
                std::span<const Duration> durations
                ) noexcept;

        /**
         * @brief Calculate median duration
         * @param durations Sample durations (will be modified)
         * @return Median value
         */
        [[nodiscard]] static Duration median(
                std::span<Duration> durations
                ) noexcept;

        /**
         * @brief Calculate percentile
         * @param durations Sample durations (will be modified)
         * @param percentile Percentile to calculate [0, 100]
         * @return Duration at given percentile
         */
        [[nodiscard]] static Duration percentile(
                std::span<Duration> durations,
                double percentile
                ) noexcept;

        /**
         * @brief Find minimum and maximum in single pass
         * @param durations Sample durations
         * @return Pair of {min, max} or nullopt if empty
         */
        [[nodiscard]] static std::optional<std::pair<Duration, Duration>> minMax(
                std::span<const Duration> durations
                ) noexcept;

        // =============================================================================
        // Smoothing and Filtering
        // =============================================================================

        /**
         * @brief Apply exponential smoothing
         * @param current Current value
         * @param target Target value
         * @param smoothing Smoothing factor [0, 1]
         * @return Smoothed value
         */
        [[nodiscard]] static constexpr Duration exponentialSmooth(
                const Duration& current,
                const Duration& target,
                double smoothing
                ) noexcept {
            smoothing = std::clamp(smoothing, 0.0, 1.0);
            return lerp(current, target, smoothing);
        }

        /**
         * @brief Apply moving average filter
         * @param values Values to filter
         * @param windowSize Moving average window
         * @return Filtered values
         */
        [[nodiscard]] static std::vector<Duration> movingAverage(
                std::span<const Duration> values,
                std::size_t windowSize
                ) noexcept;

        /**
         * @brief Apply low-pass filter
         * @param values Values to filter
         * @param cutoff Cutoff frequency (0-1)
         * @return Filtered values
         */
        [[nodiscard]] static std::vector<Duration> lowPassFilter(
                std::span<const Duration> values,
                double cutoff
                ) noexcept;

        // =============================================================================
        // Time Alignment
        // =============================================================================

        /**
         * @brief Align duration to nearest multiple
         * @param duration Duration to align
         * @param alignment Alignment interval
         * @return Aligned duration
         */
        [[nodiscard]] static constexpr Duration alignTo(
                const Duration& duration,
                const Duration& alignment
                ) noexcept {
            if (alignment == Duration::zero())
                return duration;

            const auto count = duration.count();
            const auto align = alignment.count();
            const auto aligned = ((count + align / 2) / align) * align;

            return Duration(aligned);
        }

        /**
         * @brief Quantize duration to fixed steps
         * @param duration Duration to quantize
         * @param stepSize Quantization step
         * @return Quantized duration
         */
        [[nodiscard]] static constexpr Duration quantize(
                const Duration& duration,
                const Duration& stepSize
                ) noexcept {
            if (stepSize == Duration::zero())
                return duration;

            const auto steps = duration / stepSize;
            return stepSize * steps;
        }

        /**
         * @brief Snap duration to grid
         * @param duration Duration to snap
         * @param gridSize Grid interval
         * @return Snapped duration
         */
        [[nodiscard]] static constexpr Duration snapToGrid(
                const Duration& duration,
                const Duration& gridSize
                ) noexcept {
            return alignTo(duration, gridSize);
        }

        // =============================================================================
        // Frame-based Operations
        // =============================================================================

        /**
         * @brief Calculate frame-perfect duration
         * @param targetDuration Target duration
         * @param fps Frame rate
         * @return Duration adjusted to frame boundaries
         */
        [[nodiscard]] static Duration frameAlign(
                const Duration& targetDuration,
                double fps
                ) noexcept;

        /**
         * @brief Calculate sub-frame interpolation factor
         * @param currentTime Current time
         * @param lastFrameTime Last frame time
         * @param frameTime Frame duration
         * @return Interpolation factor for sub-frame timing
         */
        [[nodiscard]] static double subFrameInterpolation(
                const TimeStamp& currentTime,
                const TimeStamp& lastFrameTime,
                const Duration& frameTime
                ) noexcept;

        // =============================================================================
        // Easing Functions
        // =============================================================================

        /**
         * @brief Apply easing function to normalized value
         * @param t Normalized time [0, 1]
         * @param type Easing type
         * @return Eased value [0, 1]
         */
        [[nodiscard]] static double ease(double t, InterpolationType type) noexcept;

        /**
         * @brief Quadratic ease in
         */
        [[nodiscard]] static constexpr double easeInQuad(double t) noexcept {
            return t * t;
        }

        /**
         * @brief Quadratic ease out
         */
        [[nodiscard]] static constexpr double easeOutQuad(double t) noexcept {
            return t * (2.0 - t);
        }

        /**
         * @brief Quadratic ease in/out
         */
        [[nodiscard]] static constexpr double easeInOutQuad(double t) noexcept {
            return t < 0.5 ? 2.0 * t * t : -1.0 + (4.0 - 2.0 * t) * t;
        }

        /**
         * @brief Cubic ease in
         */
        [[nodiscard]] static constexpr double easeInCubic(double t) noexcept {
            return t * t * t;
        }

        /**
         * @brief Cubic ease out
         */
        [[nodiscard]] static constexpr double easeOutCubic(double t) noexcept {
            const double t1 = t - 1.0;
            return t1 * t1 * t1 + 1.0;
        }

        /**
         * @brief Cubic ease in/out
         */
        [[nodiscard]] static constexpr double easeInOutCubic(double t) noexcept {
            return t < 0.5
                    ? 4.0 * t * t * t
                    : (t - 1.0) * (2.0 * t - 2.0) * (2.0 * t - 2.0) + 1.0;
        }

        // =============================================================================
        // Validation
        // =============================================================================

        /**
         * @brief Check if duration is within range
         * @param duration Duration to check
         * @param min Minimum duration
         * @param max Maximum duration
         * @return True if within range
         */
        [[nodiscard]] static constexpr bool isInRange(
                const Duration& duration,
                const Duration& min,
                const Duration& max
                ) noexcept {
            return duration >= min && duration <= max;
        }

        /**
         * @brief Check if durations are approximately equal
         * @param a First duration
         * @param b Second duration
         * @param epsilon Tolerance
         * @return True if approximately equal
         */
        [[nodiscard]] static constexpr bool areEqual(
                const Duration& a,
                const Duration& b,
                const Duration& epsilon = Duration(1)
                ) noexcept {
            const auto diff = (a > b) ? (a - b) : (b - a);
            return diff <= epsilon;
        }

    private:
        // Helper for easing functions
        static constexpr double PI = 3.14159265358979323846;
    };

} // namespace engine::time
