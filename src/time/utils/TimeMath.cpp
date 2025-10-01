/**
 * @file TimeMath.cpp
 * @brief Implementation of time mathematics utilities
 * @details Runtime implementations for complex time calculations
 *
 * @author Development Team
 * @date Created on 2024-01-20
 */

#include "TimeMath.h"

#include <algorithm>
#include <numeric>
#include <cmath>

namespace engine::time {

    // =============================================================================
    // Basic Arithmetic
    // =============================================================================

    Duration TimeMath::sum(std::span<const Duration> durations) noexcept {
        if (durations.empty()) {
            return Duration::zero();
        }

        // Use parallel reduction for large arrays
        // TODO: Lo remove por el error de par_unseq
        // if (durations.size() > 1000) {
        //     return std::reduce(
        //             std::execution::par_unseq,
        //             durations.begin(),
        //             durations.end(),
        //             Duration::zero()
        //             );
        // }

        // Sequential sum for smaller arrays
        return std::accumulate(
                durations.begin(),
                durations.end(),
                Duration::zero()
                );
    }

    Duration TimeMath::average(const std::span<const Duration> durations) noexcept {
        if (durations.empty()) {
            return Duration::zero();
        }

        const Duration total = sum(durations);
        const double avgSeconds = toSeconds(total) / static_cast<double>(durations.size());
        return fromSeconds(avgSeconds);
    }

    Duration TimeMath::weightedAverage(
            const std::span<const Duration> durations,
            const std::span<const double> weights
            ) noexcept {

        if (durations.empty() || weights.empty() ||
            durations.size() != weights.size()) {
            return Duration::zero();
        }

        double weightedSum = 0.0;
        double totalWeight = 0.0;

        for (std::size_t i = 0; i < durations.size(); ++i) {
            const double weight = std::max(0.0, weights[i]);
            weightedSum += toSeconds(durations[i]) * weight;
            totalWeight += weight;
        }

        if (totalWeight <= 0.0) {
            return Duration::zero();
        }

        return fromSeconds(weightedSum / totalWeight);
    }

    // =============================================================================
    // Interpolation
    // =============================================================================

    Duration TimeMath::interpolate(
            const Duration& start,
            const Duration& end,
            double t,
            const InterpolationType type
            ) noexcept {

        // Apply easing to t
        t = ease(t, type);

        // Linear interpolate with eased t
        return lerp(start, end, t);
    }

    Duration TimeMath::remap(
            const Duration& value,
            const Duration& inStart,
            const Duration& inEnd,
            const Duration& outStart,
            const Duration& outEnd
            ) noexcept {

        // Calculate normalized position in input range
        const double t = inverseLerp(inStart, inEnd, value);

        // Map to output range
        return lerp(outStart, outEnd, t);
    }

    // =============================================================================
    // Statistical Operations
    // =============================================================================

    Duration TimeMath::standardDeviation(const std::span<const Duration> durations) noexcept {
        if (durations.size() < 2) {
            return Duration::zero();
        }

        const Duration mean = average(durations);
        const double meanSeconds = toSeconds(mean);

        double sumSquaredDiff = 0.0;
        for (const auto& duration : durations) {
            const double diff = toSeconds(duration) - meanSeconds;
            sumSquaredDiff += diff * diff;
        }

        const double variance = sumSquaredDiff / static_cast<double>(durations.size() - 1);
        const double stdDev = std::sqrt(variance);

        return fromSeconds(stdDev);
    }

    Duration TimeMath::median(std::span<Duration> durations) noexcept {
        if (durations.empty()) {
            return Duration::zero();
        }

        const std::size_t size = durations.size();
        const std::size_t mid = size / 2;

        // Use nth_element for O(n) median finding
        std::ranges::nth_element(
                durations,
                durations.begin() + static_cast<std::ptrdiff_t>(mid)
                );

        if (size % 2 == 1) {
            return durations[mid];
        }

        // For even size, average two middle elements
        const Duration mid1 = durations[mid];
        std::nth_element(
                durations.begin(),
                durations.begin() + static_cast<std::ptrdiff_t>(mid - 1),
                durations.begin() + static_cast<std::ptrdiff_t>(mid)
                );
        const Duration mid2 = durations[mid - 1];

        return fromSeconds((toSeconds(mid1) + toSeconds(mid2)) / 2.0);
    }

    Duration TimeMath::percentile(
            std::span<Duration> durations,
            double percentile
            ) noexcept {

        if (durations.empty()) {
            return Duration::zero();
        }

        percentile = std::clamp(percentile, 0.0, 100.0);

        const std::size_t size = durations.size();
        const double index = (percentile / 100.0) * static_cast<double>(size - 1);
        const auto lower = static_cast<std::size_t>(std::floor(index));
        const auto upper = static_cast<std::size_t>(std::ceil(index));

        if (lower == upper) {
            std::ranges::nth_element(
                    durations,
                    durations.begin() + static_cast<std::ptrdiff_t>(lower)
                    );
            return durations[lower];
        }

        // Interpolate between two values
        std::ranges::nth_element(
                durations,
                durations.begin() + static_cast<std::ptrdiff_t>(upper)
                );
        const Duration upperValue = durations[upper];

        std::nth_element(
                durations.begin(),
                durations.begin() + static_cast<std::ptrdiff_t>(lower),
                durations.begin() + static_cast<std::ptrdiff_t>(upper)
                );
        const Duration lowerValue = durations[lower];

        const double fraction = index - static_cast<double>(lower);
        return lerp(lowerValue, upperValue, fraction);
    }

    std::optional<std::pair<Duration, Duration>> TimeMath::minMax(
            std::span<const Duration> durations
            ) noexcept {

        if (durations.empty()) {
            return std::nullopt;
        }

        // Parallel minmax for large arrays
        if (durations.size() > 1000) {
            const auto [minIt, maxIt] = std::minmax_element(
                    // std::execution::par_unseq,
                    durations.begin(),
                    durations.end()
                    );
            return std::make_pair(*minIt, *maxIt);
        }

        // Sequential for smaller arrays
        const auto [minIt, maxIt] = std::minmax_element(
                durations.begin(),
                durations.end()
                );
        return std::make_pair(*minIt, *maxIt);
    }

    // =============================================================================
    // Smoothing and Filtering
    // =============================================================================

    std::vector<Duration> TimeMath::movingAverage(
            const std::span<const Duration> values,
            std::size_t windowSize
            ) noexcept {

        if (values.empty() || windowSize == 0) {
            return {};
        }

        windowSize = std::min(windowSize, values.size());
        std::vector<Duration> result;
        result.reserve(values.size());

        // Calculate initial window sum
        double windowSum = 0.0;
        for (std::size_t i = 0; i < windowSize; ++i) {
            windowSum += toSeconds(values[i]);
        }

        // Process with sliding window
        for (std::size_t i = 0; i < values.size(); ++i) {
            if (i >= windowSize) {
                windowSum -= toSeconds(values[i - windowSize]);
                windowSum += toSeconds(values[i]);
            }

            const std::size_t currentWindowSize = std::min(i + 1, windowSize);
            result.push_back(fromSeconds(windowSum / static_cast<double>(currentWindowSize)));
        }

        return result;
    }

    std::vector<Duration> TimeMath::lowPassFilter(
            const std::span<const Duration> values,
            double cutoff
            ) noexcept {

        if (values.empty()) {
            return {};
        }

        cutoff = std::clamp(cutoff, 0.0, 1.0);
        std::vector<Duration> result;
        result.reserve(values.size());

        // First-order low-pass filter
        const double alpha = cutoff;
        Duration filtered = values[0];
        result.push_back(filtered);

        for (std::size_t i = 1; i < values.size(); ++i) {
            filtered = exponentialSmooth(filtered, values[i], alpha);
            result.push_back(filtered);
        }

        return result;
    }

    // =============================================================================
    // Frame-based Operations
    // =============================================================================

    Duration TimeMath::frameAlign(const Duration& targetDuration, const double fps) noexcept {
        if (fps <= 0.0) {
            return targetDuration;
        }

        const Duration frameDuration = fromSeconds(1.0 / fps);
        const auto frames = (targetDuration + frameDuration / 2) / frameDuration;
        return frameDuration * frames;
    }

    double TimeMath::subFrameInterpolation(
            const TimeStamp& currentTime,
            const TimeStamp& lastFrameTime,
            const Duration& frameTime
            ) noexcept {

        if (frameTime == Duration::zero()) {
            return 0.0;
        }

        const auto elapsed = std::chrono::duration_cast<Duration>(currentTime - lastFrameTime);
        return inverseLerp(Duration::zero(), frameTime, elapsed);
    }

    // =============================================================================
    // Easing Functions
    // =============================================================================

    double TimeMath::ease(double t, const InterpolationType type) noexcept {
        t = std::clamp(t, 0.0, 1.0);

        switch (type) {
            case InterpolationType::LINEAR:
                return t;

            case InterpolationType::EASE_IN:
                return easeInQuad(t);

            case InterpolationType::EASE_OUT:
                return easeOutQuad(t);

            case InterpolationType::EASE_IN_OUT:
                return easeInOutQuad(t);

            case InterpolationType::CUBIC_IN:
                return easeInCubic(t);

            case InterpolationType::CUBIC_OUT:
                return easeOutCubic(t);

            case InterpolationType::CUBIC_IN_OUT:
                return easeInOutCubic(t);

            case InterpolationType::SINE_IN:
                return 1.0 - std::cos((t * PI) / 2.0);

            case InterpolationType::SINE_OUT:
                return std::sin((t * PI) / 2.0);

            case InterpolationType::SINE_IN_OUT:
                return -(std::cos(PI * t) - 1.0) / 2.0;

            case InterpolationType::EXPONENTIAL_IN:
                return t == 0.0 ? 0.0 : std::pow(2.0, 10.0 * t - 10.0);

            case InterpolationType::EXPONENTIAL_OUT:
                return t == 1.0 ? 1.0 : 1.0 - std::pow(2.0, -10.0 * t);

            case InterpolationType::ELASTIC: {
                if (t == 0.0 || t == 1.0)
                    return t;
                constexpr double p = 0.3;
                constexpr double s = p / 4.0;
                const double postFix = std::pow(2.0, -10.0 * t);
                return postFix * std::sin((t - s) * (2.0 * PI) / p) + 1.0;
            }

            case InterpolationType::BACK: {
                constexpr double s = 1.70158;
                const double t1 = t - 1.0;
                return t1 * t1 * ((s + 1.0) * t1 + s) + 1.0;
            }

            case InterpolationType::BOUNCE: {
                double value = 1.0 - t;
                double result;

                if (value < 1.0 / 2.75) {
                    result = 7.5625 * value * value;
                } else if (value < 2.0 / 2.75) {
                    value -= 1.5 / 2.75;
                    result = 7.5625 * value * value + 0.75;
                } else if (value < 2.5 / 2.75) {
                    value -= 2.25 / 2.75;
                    result = 7.5625 * value * value + 0.9375;
                } else {
                    value -= 2.625 / 2.75;
                    result = 7.5625 * value * value + 0.984375;
                }

                return 1.0 - result;
            }

            default:
                return t;
        }
    }

} // namespace engine::time
