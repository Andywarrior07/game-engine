/**
 * @file TimeConverter.cpp
 * @brief Implementation of high-performance time unit conversion utilities
 * @details Provides optimized runtime implementations for time conversions
 *          with precision handling and performance-critical optimizations.
 *
 * @author Development Team
 * @date Created on 2024-01-20
 */

#include "TimeConverter.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <charconv>

namespace engine::time {

    // =============================================================================
    // Runtime Conversions Implementation
    // =============================================================================

    double TimeConverter::convertRuntime(
            const double value,
            const TimeUnit from,
            const TimeUnit to
            ) noexcept {
        // Fast path: same unit conversion
        if (from == to) [[likely]] {
            return value;
        }

        // Fast path: zero value (common in initialization)
        if (value == 0.0) [[likely]] {
            return 0.0;
        }

        // Fast path: special values (infinity, NaN)
        // These should propagate through conversions
        if (!std::isfinite(value)) [[unlikely]] {
            return value;
        }

        // Use pre-computed conversion cache for O(1) lookup
        // This avoids division in hot paths and improves cache locality
        const double ratio = conversionCache_.getRatio(from, to);

        // Perform conversion with optimal precision
        // Compiler will likely vectorize this in batch operations
        return value * ratio;
    }

    double TimeConverter::toUnitRuntime(
            const Duration& duration,
            const TimeUnit unit
            ) noexcept {
        // Convert duration to nanoseconds (our base unit)
        // Using duration_cast ensures proper rounding behavior
        const auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(duration);
        const auto nanosValue = static_cast<double>(nanos.count());

        // Fast path: if requesting nanoseconds, avoid division
        if (unit == TimeUnit::NANOSECONDS) [[unlikely]] {
            return nanosValue;
        }

        // Get conversion factor from lookup table
        // This is cache-friendly and avoids runtime calculation
        const double divisor = nanosPerUnit_[static_cast<std::size_t>(unit)];

        // Division by compile-time constant allows compiler optimization
        return nanosValue / divisor;
    }

    Duration TimeConverter::fromUnitRuntime(
            const double value,
            const TimeUnit unit
            ) noexcept {
        // Handle edge cases first
        if (!std::isfinite(value)) [[unlikely]] {
            // Return max duration for infinity, zero for NaN
            return std::isinf(value) && value > 0.0
                    ? Duration::max()
                    : Duration::zero();
        }

        // Fast path: zero value
        if (value == 0.0) [[likely]] {
            return Duration::zero();
        }

        // Convert to nanoseconds using lookup table
        const double nanos = value * nanosPerUnit_[static_cast<std::size_t>(unit)];

        // Clamp to valid range to prevent overflow
        // Duration is typically int64_t nanoseconds, so check bounds
        constexpr double maxNanos = static_cast<double>(
            std::chrono::nanoseconds::max().count()
        );
        constexpr double minNanos = static_cast<double>(
            std::chrono::nanoseconds::min().count()
        );

        // Clamp and convert with proper rounding
        const double clampedNanos = std::clamp(nanos, minNanos, maxNanos);
        const auto nanosInt = static_cast<std::int64_t>(std::round(clampedNanos));

        // Create duration efficiently without intermediate conversions
        return std::chrono::duration_cast<Duration>(
                std::chrono::nanoseconds(nanosInt)
                );
    }

    // =============================================================================
    // Precision Conversions Implementation
    // =============================================================================

    double TimeConverter::convertWithPrecisionCheck(
            const double value,
            const TimeUnit from,
            const TimeUnit to,
            bool& precision_loss
            ) noexcept {
        precision_loss = false;

        // Same unit means no precision loss
        if (from == to) {
            return value;
        }

        // Get conversion ratio
        const double ratio = conversionCache_.getRatio(from, to);
        const double converted = value * ratio;

        // Check for precision loss by doing round-trip conversion
        // If we lose significant digits, flag it
        const double roundTrip = converted / ratio;

        // Define precision threshold (0.01% relative error)
        // This catches meaningful precision loss while allowing for
        // acceptable floating-point rounding errors

        // Calculate relative error
        const double relativeError = std::abs(roundTrip - value) /
                std::max(std::abs(value), 1.0);

        // Flag precision loss if error exceeds threshold
        if (constexpr double PRECISION_THRESHOLD = 1e-4; relativeError > PRECISION_THRESHOLD) {
            precision_loss = true;
        }

        // Additional check: converting to larger units can lose precision
        // Example: 1.5 nanoseconds -> microseconds -> nanoseconds
        if (from < to) {
            // Converting to larger unit - check if we lose fractional part
            if (const double fractionalPart = std::fmod(value * ratio, 1.0); std::abs(fractionalPart) > 1e-10) {
                precision_loss = true;
            }
        }

        return converted;
    }

    TimeUnit TimeConverter::getOptimalUnit(const Duration& duration) noexcept {
        // Get absolute value in nanoseconds
        const auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(
                duration
                ).count();
        const double absNanos = std::abs(static_cast<double>(nanos));

        // Zero duration - return smallest unit
        if (absNanos == 0.0) {
            return TimeUnit::NANOSECONDS;
        }

        // Find the largest unit where the value is >= 1.0
        // This gives us the most readable representation
        // We iterate from largest to smallest for efficiency
        // (most common cases are seconds/milliseconds)

        // Check from weeks down to nanoseconds
        for (int i = static_cast<int>(TimeUnit::COUNT) - 1; i >= 0; --i) {
            const auto unit = static_cast<TimeUnit>(i);

            // If value is >= 1.0, this is our optimal unit
            // We want values in range [1.0, 999.99...] for readability
            if (const double value = absNanos / nanosPerUnit_[i]; value >= 1.0) {
                // Additional check: if value is too large (>= 1000),
                // try to use next larger unit if available
                if (value >= 1000.0 && i < static_cast<int>(TimeUnit::COUNT) - 1) {
                    // Check if next unit gives reasonable value
                    if (const double nextValue = absNanos / nanosPerUnit_[i + 1]; nextValue >= 1.0) {
                        return static_cast<TimeUnit>(i + 1);
                    }
                }
                return unit;
            }
        }

        // Fallback: use nanoseconds for very small values
        return TimeUnit::NANOSECONDS;
    }

    Duration TimeConverter::roundToUnit(
            const Duration& duration,
            const TimeUnit unit
            ) noexcept {
        // Convert to target unit
        const double unitValue = toUnitRuntime(duration, unit);

        // Round to nearest integer in that unit
        const double rounded = std::round(unitValue);

        // Convert back to Duration
        return fromUnitRuntime(rounded, unit);
    }

    // =============================================================================
    // Unit Information Implementation
    // =============================================================================

    std::optional<TimeUnit> TimeConverter::parseUnit(
            const std::string_view str
            ) noexcept {
        // Handle empty string
        if (str.empty()) {
            return std::nullopt;
        }

        // Create lowercase version for case-insensitive comparison
        // Note: This is a simple ASCII lowercase, sufficient for our unit strings
        std::string lowerStr;
        lowerStr.reserve(str.size());
        for (const char c : str) {
            lowerStr.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
        }
        const std::string_view lowerView = lowerStr;

        // Check abbreviations first (most common case)
        for (std::size_t i = 0; i < static_cast<std::size_t>(TimeUnit::COUNT); ++i) {
            // Create lowercase version of abbreviation for comparison
            std::string lowerAbbrev;
            lowerAbbrev.reserve(unitAbbreviations_[i].size());
            for (const char c : unitAbbreviations_[i]) {
                lowerAbbrev.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
            }

            if (lowerView == lowerAbbrev) {
                return static_cast<TimeUnit>(i);
            }
        }

        // Check full names
        for (std::size_t i = 0; i < static_cast<std::size_t>(TimeUnit::COUNT); ++i) {
            if (lowerView == unitNames_[i]) {
                return static_cast<TimeUnit>(i);
            }
        }

        // Check common aliases and variations
        // This makes the parser more user-friendly
        if (lowerView == "ns" || lowerView == "nano" || lowerView == "nanos") {
            return TimeUnit::NANOSECONDS;
        }
        if (lowerView == "us" || lowerView == "micro" || lowerView == "micros") {
            return TimeUnit::MICROSECONDS;
        }
        if (lowerView == "ms" || lowerView == "milli" || lowerView == "millis") {
            return TimeUnit::MILLISECONDS;
        }
        if (lowerView == "s" || lowerView == "sec" || lowerView == "secs" ||
            lowerView == "second") {
            return TimeUnit::SECONDS;
        }
        if (lowerView == "m" || lowerView == "min" || lowerView == "mins" ||
            lowerView == "minute") {
            return TimeUnit::MINUTES;
        }
        if (lowerView == "h" || lowerView == "hr" || lowerView == "hrs" ||
            lowerView == "hour") {
            return TimeUnit::HOURS;
        }
        if (lowerView == "d" || lowerView == "day") {
            return TimeUnit::DAYS;
        }
        if (lowerView == "w" || lowerView == "wk" || lowerView == "wks" ||
            lowerView == "week") {
            return TimeUnit::WEEKS;
        }

        // No match found
        return std::nullopt;
    }

    // =============================================================================
    // Validation Implementation
    // =============================================================================

    bool TimeConverter::wouldOverflow(
            const double value,
            const TimeUnit from,
            const TimeUnit to
            ) noexcept {
        // Special values never overflow
        if (!std::isfinite(value)) {
            return false;
        }

        // Zero never overflows
        if (value == 0.0) {
            return false;
        }

        // Get conversion ratio
        const double ratio = conversionCache_.getRatio(from, to);

        // Check if result would overflow double's range
        if (const double converted = value * ratio; !std::isfinite(converted)) {
            return true;
        }

        // For Duration conversion, check against nanosecond limits
        // Convert to nanoseconds to check actual storage limits
        const double nanosValue = value * nanosPerUnit_[static_cast<std::size_t>(from)];

        // Check against Duration's underlying type limits
        // Duration is typically based on int64_t nanoseconds
        constexpr auto maxNanos = static_cast<double>(
            std::numeric_limits<std::int64_t>::max()
        );
        constexpr auto minNanos = static_cast<double>(
            std::numeric_limits<std::int64_t>::min()
        );

        // Would overflow if outside valid range
        return nanosValue > maxNanos || nanosValue < minNanos;
    }

    bool TimeConverter::isValidForUnit(
            const Duration& duration,
            const TimeUnit unit
            ) noexcept {
        // Convert to nanoseconds to check full precision
        const auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(
                duration
                ).count();
        const auto nanosValue = static_cast<double>(nanos);

        // Get target unit's nanosecond scale
        const double unitScale = nanosPerUnit_[static_cast<std::size_t>(unit)];

        // Convert to target unit
        const double unitValue = nanosValue / unitScale;

        // Check if within reasonable range for the unit
        // We use double's finite range as the limit
        if (!std::isfinite(unitValue)) {
            return false;
        }

        // Check if precision is maintained
        // Convert back and compare
        const double roundTrip = unitValue * unitScale;
        const double difference = std::abs(roundTrip - nanosValue);

        // Allow for some floating-point error (1 nanosecond tolerance)
        // This is reasonable since that's our smallest unit
        return difference < 1.0;
    }

    // =============================================================================
    // Helper Functions for Common Conversions
    // =============================================================================
    // Note: These are defined in the header as constexpr templates
    // but we can add convenience overloads here if needed

} // namespace engine::time
