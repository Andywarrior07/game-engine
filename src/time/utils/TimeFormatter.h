/**
 * @file TimeFormatter.h
 * @brief Time formatting utilities for debugging and UI display
 * @details Provides human-readable formatting of time values with
 *          localization support and customizable precision.
 *          Header-only for optimal performance in debug builds.
 *
 * @author Development Team
 * @date Created on 2024-01-20
 */

#pragma once

#include "TimeConverter.h"

#include "../core/TimeTypes.h"

#include <ranges>
#include <string>
#include <sstream>
#include <iomanip>

namespace engine::time {

    // =============================================================================
    // Formatting Options
    // =============================================================================

    /**
     * @brief Time format styles
     */
    enum class FormatStyle : std::uint8_t {
        COMPACT, // "16.67ms"
        FULL, // "16 milliseconds"
        TECHNICAL, // "16667μs"
        HUMAN, // "16.7 ms"
        ISO8601, // "PT0.016667S"
        GAME, // "00:00:16"
        FRAME_TIME // "16.67ms (60 FPS)"
    };

    /**
     * @brief Format precision levels
     */
    enum class FormatPrecision : std::uint8_t {
        AUTO, // Automatically choose precision
        INTEGER, // No decimal places
        LOW, // 1 decimal place
        MEDIUM, // 2 decimal places
        HIGH, // 3 decimal places
        MAXIMUM // 6 decimal places
    };

    /**
     * @brief Format options structure
     */
    struct FormatOptions {
        FormatStyle style{FormatStyle::HUMAN};
        FormatPrecision precision{FormatPrecision::AUTO};
        TimeUnit preferredUnit{TimeUnit::MILLISECONDS};
        bool includeUnit{true};
        bool useOptimalUnit{true};
        bool showSign{false};
        char decimalSeparator{'.'};
        char thousandsSeparator{','};
        bool useThousandsSeparator{false};
    };

    // =============================================================================
    // TimeFormatter Class
    // =============================================================================

    /**
     * @brief High-performance time formatting utilities
     * @details Header-only implementation for zero-cost abstractions
     */
    class TimeFormatter {
    public:
        // =============================================================================
        // Quick Format Methods
        // =============================================================================

        /**
         * @brief Format duration with default options
         * @param duration Duration to format
         * @return Formatted string
         */
        [[nodiscard]] static std::string format(const Duration& duration) {
            return format(duration, FormatOptions{});
        }

        /**
         * @brief Format duration with a specific style
         * @param duration Duration to format
         * @param style Format style
         * @return Formatted string
         */
        [[nodiscard]] static std::string format(
                const Duration& duration,
                const FormatStyle style
                ) {
            FormatOptions options;
            options.style = style;
            return format(duration, options);
        }

        /**
         * @brief Format duration with full options
         * @param duration Duration to format
         * @param options Format options
         * @return Formatted string
         */
        [[nodiscard]] static std::string format(
                const Duration& duration,
                const FormatOptions& options
                ) {

            switch (options.style) {
                case FormatStyle::COMPACT:
                    return formatCompact(duration, options);
                case FormatStyle::FULL:
                    return formatFull(duration, options);
                case FormatStyle::TECHNICAL:
                    return formatTechnical(duration, options);
                case FormatStyle::HUMAN:
                    return formatHuman(duration, options);
                case FormatStyle::ISO8601:
                    return formatISO8601(duration);
                case FormatStyle::GAME:
                    return formatGameTime(duration);
                case FormatStyle::FRAME_TIME:
                    return formatFrameTime(duration);
                default:
                    return formatHuman(duration, options);
            }
        }

        /**
         * @brief Format timestamp as a date / time string
         * @param timestamp Time point to format
         * @param includeDate Include date portion
         * @param includeMicros Include microseconds
         * @return Formatted date/time string
         */
        [[nodiscard]] static std::string formatTimestamp(
                const TimeStamp& timestamp,
                const bool includeDate = true,
                const bool includeMicros = false
                ) {
            const auto now_steady = Clock::now();
            const auto now_system = std::chrono::system_clock::now();

            const auto time_diff = timestamp - now_steady;
            const auto time_diff_system = std::chrono::duration_cast<std::chrono::system_clock::duration>(time_diff);
            const auto system_time = now_system + time_diff_system;

            const auto time_t_val = std::chrono::system_clock::to_time_t(system_time);
            std::stringstream ss;

            // Format date/time
            std::tm tm_buf{};
#ifdef _WIN32
            localtime_s(&tm_buf, &time_t_val);
#else
            localtime_r(&time_t_val, &tm_buf);
#endif

            if (includeDate) {
                ss << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S");
            } else {
                ss << std::put_time(&tm_buf, "%H:%M:%S");
            }

            if (includeMicros) {
                const auto duration = timestamp.time_since_epoch();
                const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
                        duration % std::chrono::seconds(1)
                        ).count();
                ss << '.' << std::setfill('0') << std::setw(6) << micros;
            }

            return ss.str();
        }

        // =============================================================================
        // Specialized Formatters
        // =============================================================================

        /**
         * @brief Format as a compact string (e.g., "16.67 ms")
         */
        [[nodiscard]] static std::string formatCompact(
                const Duration& duration,
                const FormatOptions& options = {}
                ) {

            const TimeUnit unit = options.useOptimalUnit
                    ? TimeConverter::getOptimalUnit(duration)
                    : options.preferredUnit;

            const double value = TimeConverter::toUnitRuntime(duration, unit);
            const int precision = getPrecisionForValue(value, options.precision);

            std::stringstream ss;
            ss << std::fixed << std::setprecision(precision) << value;

            if (options.includeUnit) {
                ss << TimeConverter::getUnitAbbreviation(unit);
            }

            return ss.str();
        }

        /**
         * @brief Format with full unit names (e.g., "16 milliseconds")
         */
        [[nodiscard]] static std::string formatFull(
                const Duration& duration,
                const FormatOptions& options = {}
                ) {

            const TimeUnit unit = options.useOptimalUnit
                    ? TimeConverter::getOptimalUnit(duration)
                    : options.preferredUnit;

            const double value = TimeConverter::toUnitRuntime(duration, unit);
            const int precision = getPrecisionForValue(value, options.precision);

            std::stringstream ss;
            ss << std::fixed << std::setprecision(precision) << value;

            if (options.includeUnit) {
                ss << ' ' << TimeConverter::getUnitName(unit);
                if (std::abs(value - 1.0) > 0.001) {
                    // Plural form for non-one values
                    if (TimeConverter::getUnitName(unit).back() != 's') {
                        ss << 's';
                    }
                }
            }

            return ss.str();
        }

        /**
         * @brief Format for technical display (e.g., "16667μs")
         */
        [[nodiscard]] static std::string formatTechnical(
                const Duration& duration,
                const FormatOptions& options = {}
                ) {

            // Always use microseconds for technical display
            const double micros = TimeConverter::toUnit<TimeUnit::MICROSECONDS>(duration);

            std::stringstream ss;
            ss << std::fixed << std::setprecision(0) << micros;

            if (options.includeUnit) {
                ss << "μs";
            }

            return ss.str();
        }

        /**
         * @brief Format for human readability (e.g., "16.7 ms")
         */
        [[nodiscard]] static std::string formatHuman(
                const Duration& duration,
                const FormatOptions& options = {}
                ) {

            const TimeUnit unit = options.useOptimalUnit
                    ? TimeConverter::getOptimalUnit(duration)
                    : options.preferredUnit;

            const double value = TimeConverter::toUnitRuntime(duration, unit);
            const int precision = getPrecisionForValue(value, options.precision);

            std::stringstream ss;

            if (options.showSign && value >= 0) {
                ss << '+';
            }

            ss << std::fixed << std::setprecision(precision);

            if (options.useThousandsSeparator && std::abs(value) >= 1000) {
                ss << formatWithThousandsSeparator(value, options.thousandsSeparator);
            } else {
                ss << value;
            }

            if (options.includeUnit) {
                ss << ' ' << TimeConverter::getUnitAbbreviation(unit);
            }

            return ss.str();
        }

        /**
         * @brief Format as ISO 8601 duration (e.g., "PT0.016667S")
         */
        [[nodiscard]] static std::string formatISO8601(const Duration& duration) {
            const double totalSeconds = TimeConverter::toUnit<TimeUnit::SECONDS>(duration);
            const bool negative = totalSeconds < 0;
            const double absSeconds = std::abs(totalSeconds);

            const int days = static_cast<int>(absSeconds / 86400);
            const int hours = static_cast<int>(std::fmod(absSeconds, 86400) / 3600);
            const int minutes = static_cast<int>(std::fmod(absSeconds, 3600) / 60);
            const double seconds = std::fmod(absSeconds, 60);

            std::stringstream ss;
            if (negative)
                ss << '-';
            ss << 'P';

            if (days > 0)
                ss << days << 'D';

            if (hours > 0 || minutes > 0 || seconds > 0) {
                ss << 'T';
                if (hours > 0)
                    ss << hours << 'H';
                if (minutes > 0)
                    ss << minutes << 'M';
                if (seconds > 0 || (days == 0 && hours == 0 && minutes == 0)) {
                    ss << std::fixed << std::setprecision(6) << seconds << 'S';
                }
            }

            return ss.str();
        }

        /**
         * @brief Format as game time (e.g., "00:16:42")
         */
        [[nodiscard]] static std::string formatGameTime(const Duration& duration) {
            const double totalSeconds = std::abs(TimeConverter::toUnit<TimeUnit::SECONDS>(duration));

            const int hours = static_cast<int>(totalSeconds / 3600);
            const int minutes = static_cast<int>(std::fmod(totalSeconds, 3600) / 60);
            const int seconds = static_cast<int>(std::fmod(totalSeconds, 60));
            const int millis = static_cast<int>(std::fmod(totalSeconds * 1000, 1000));

            std::stringstream ss;

            if (duration < Duration::zero()) {
                ss << '-';
            }

            ss << std::setfill('0');

            if (hours > 0) {
                ss << std::setw(2) << hours << ':';
            }

            ss << std::setw(2) << minutes << ':'
                    << std::setw(2) << seconds;

            if (millis > 0) {
                ss << '.' << std::setw(3) << millis;
            }

            return ss.str();
        }

        /**
         * @brief Format as frame time with FPS (e.g., "16.67 ms (60 FPS)")
         */
        [[nodiscard]] static std::string formatFrameTime(const Duration& duration) {
            const double millis = TimeConverter::toUnit<TimeUnit::MILLISECONDS>(duration);
            const double fps = millis > 0 ? 1000.0 / millis : 0.0;

            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) << millis << "ms";

            if (fps > 0 && fps < 1000) {
                ss << " (" << std::setprecision(0) << fps << " FPS)";
            }

            return ss.str();
        }

        // =============================================================================
        // Relative Time Formatting
        // =============================================================================

        /**
         * @brief Format duration as relative time (e.g., "2 hours ago")
         */
        [[nodiscard]] static std::string formatRelative(const Duration& duration) {
            const double seconds = std::abs(TimeConverter::toUnit<TimeUnit::SECONDS>(duration));
            const bool past = duration < Duration::zero();

            std::string result;

            if (seconds < 1) {
                result = "now";
            } else if (seconds < 60) {
                const int sec = static_cast<int>(seconds);
                result = std::to_string(sec) + (sec == 1 ? " second" : " seconds");
            } else if (seconds < 3600) {
                const int min = static_cast<int>(seconds / 60);
                result = std::to_string(min) + (min == 1 ? " minute" : " minutes");
            } else if (seconds < 86400) {
                const int hours = static_cast<int>(seconds / 3600);
                result = std::to_string(hours) + (hours == 1 ? " hour" : " hours");
            } else if (seconds < 604800) {
                const int days = static_cast<int>(seconds / 86400);
                result = std::to_string(days) + (days == 1 ? " day" : " days");
            } else {
                const int weeks = static_cast<int>(seconds / 604800);
                result = std::to_string(weeks) + (weeks == 1 ? " week" : " weeks");
            }

            if (result != "now") {
                result += past ? " ago" : " from now";
            }

            return result;
        }

        /**
         * @brief Format duration range (e.g., "16.67 ms - 33.33 ms")
         */
        [[nodiscard]] static std::string formatRange(
                const Duration& min,
                const Duration& max,
                const FormatOptions& options = {}
                ) {

            return format(min, options) + " - " + format(max, options);
        }

    private:
        /**
         * @brief Determine precision based on value and options
         */
        [[nodiscard]] static int getPrecisionForValue(
                const double value,
                const FormatPrecision precision
                ) {

            switch (precision) {
                case FormatPrecision::AUTO:
                    if (std::abs(value) >= 100)
                        return 0;
                    if (std::abs(value) >= 10)
                        return 1;
                    if (std::abs(value) >= 1)
                        return 2;
                    return 3;

                case FormatPrecision::INTEGER:
                    return 0;
                case FormatPrecision::LOW:
                    return 1;
                case FormatPrecision::MEDIUM:
                    return 2;
                case FormatPrecision::HIGH:
                    return 3;
                case FormatPrecision::MAXIMUM:
                    return 6;
                default:
                    return 2;
            }
        }

        /**
         * @brief Format number with a thousand separator
         */
        [[nodiscard]] static std::string formatWithThousandsSeparator(
                const double value,
                const char separator
                ) {

            std::stringstream ss;
            ss << std::fixed << std::setprecision(0) << std::abs(value);
            std::string str = ss.str();

            std::string result;
            int count = 0;

            for (const char& it : std::ranges::reverse_view(str)) {
                if (count == 3) {
                    result += separator;
                    count = 0;
                }

                result += it;
                ++count;
            }

            if (value < 0) {
                result = '-' + result;
            }

            return result;
        }
    };

} // namespace engine::time
