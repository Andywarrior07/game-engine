/**
 * @file TimeConverter.h
 * @brief High-performance time unit conversion utilities
 * @details Provides compile-time and runtime conversion between various time units
 *          with zero-cost abstractions and precision preservation.
 *          Optimized for hot paths with extensive use of constexpr and templates.
 *
 * @author Development Team
 * @date Created on 2024-01-20
 */

#pragma once

#include "../core/TimeTypes.h"

#include <array>
#include <string_view>
#include <cmath>

namespace engine::time {

    // =============================================================================
    // Time Units Enumeration
    // =============================================================================

    /**
     * @brief Supported time units for conversion
     * @details Ordered from smallest to largest for optimization
     */
    enum class TimeUnit : std::uint8_t {
        NANOSECONDS = 0, MICROSECONDS = 1, MILLISECONDS = 2,
        SECONDS     = 3, MINUTES      = 4, HOURS        = 5,
        DAYS        = 6, WEEKS        = 7, COUNT        = 8 // Total number of units
    };

    // =============================================================================
    // Conversion Traits
    // =============================================================================

    /**
     * @brief Type traits for time unit conversions
     * @details Compile-time conversion ratios for type safety
     */
    template <TimeUnit Unit>
    struct TimeUnitTraits {
        static constexpr double nanosPerUnit = 1.0;
        static constexpr std::string_view abbreviation = "ns";
        static constexpr std::string_view fullName = "nanoseconds";
    };

    // Specializations for each unit
    template <>
    struct TimeUnitTraits<TimeUnit::NANOSECONDS> {
        static constexpr double nanosPerUnit = 1.0;
        static constexpr std::string_view abbreviation = "ns";
        static constexpr std::string_view fullName = "nanoseconds";
    };

    template <>
    struct TimeUnitTraits<TimeUnit::MICROSECONDS> {
        static constexpr double nanosPerUnit = 1'000.0;
        static constexpr std::string_view abbreviation = "μs";
        static constexpr std::string_view fullName = "microseconds";
    };

    template <>
    struct TimeUnitTraits<TimeUnit::MILLISECONDS> {
        static constexpr double nanosPerUnit = 1'000'000.0;
        static constexpr std::string_view abbreviation = "ms";
        static constexpr std::string_view fullName = "milliseconds";
    };

    template <>
    struct TimeUnitTraits<TimeUnit::SECONDS> {
        static constexpr double nanosPerUnit = 1'000'000'000.0;
        static constexpr std::string_view abbreviation = "s";
        static constexpr std::string_view fullName = "seconds";
    };

    template <>
    struct TimeUnitTraits<TimeUnit::MINUTES> {
        static constexpr double nanosPerUnit = 60'000'000'000.0;
        static constexpr std::string_view abbreviation = "min";
        static constexpr std::string_view fullName = "minutes";
    };

    template <>
    struct TimeUnitTraits<TimeUnit::HOURS> {
        static constexpr double nanosPerUnit = 3'600'000'000'000.0;
        static constexpr std::string_view abbreviation = "h";
        static constexpr std::string_view fullName = "hours";
    };

    template <>
    struct TimeUnitTraits<TimeUnit::DAYS> {
        static constexpr double nanosPerUnit = 86'400'000'000'000.0;
        static constexpr std::string_view abbreviation = "d";
        static constexpr std::string_view fullName = "days";
    };

    template <>
    struct TimeUnitTraits<TimeUnit::WEEKS> {
        static constexpr double nanosPerUnit = 604'800'000'000'000.0;
        static constexpr std::string_view abbreviation = "w";
        static constexpr std::string_view fullName = "weeks";
    };

    // =============================================================================
    // TimeConverter Class
    // =============================================================================

    /**
     * @brief High-performance time unit conversion utilities
     * @details Static class providing compile-time and runtime conversions
     *          with precision preservation and overflow detection.
     */
    class TimeConverter {
    public:
        // =============================================================================
        // Compile-time Conversions
        // =============================================================================

        /**
         * @brief Convert between time units at compile time
         * @tparam From Source time unit
         * @tparam To Target time unit
         * @param value Value to convert
         * @return Converted value
         * @details Zero-cost abstraction resolved at compile time
         */
        template <TimeUnit From, TimeUnit To>
        [[nodiscard]] static constexpr double convert(const double value) noexcept {
            if constexpr (From == To) {
                return value;
            } else {
                constexpr double ratio = TimeUnitTraits<From>::nanosPerUnit /
                        TimeUnitTraits<To>::nanosPerUnit;
                return value * ratio;
            }
        }

        /**
         * @brief Convert duration to specified unit
         * @tparam Unit Target time unit
         * @param duration Duration to convert
         * @return Value in target unit
         */
        template <TimeUnit Unit>
        [[nodiscard]] static constexpr double toUnit(const Duration& duration) noexcept {
            const double nanos = static_cast<double>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count()
            );
            return nanos / TimeUnitTraits<Unit>::nanosPerUnit;
        }

        /**
         * @brief Create duration from value in specified unit
         * @tparam Unit Source time unit
         * @param value Value in source unit
         * @return Duration object
         */
        template <TimeUnit Unit>
        [[nodiscard]] static constexpr Duration fromUnit(const double value) noexcept {
            const double nanos = value * TimeUnitTraits<Unit>::nanosPerUnit;
            return std::chrono::duration_cast<Duration>(
                    std::chrono::nanoseconds(static_cast<std::int64_t>(nanos))
                    );
        }

        // =============================================================================
        // Runtime Conversions
        // =============================================================================

        /**
         * @brief Convert between time units at runtime
         * @param value Value to convert
         * @param from Source unit
         * @param to Target unit
         * @return Converted value
         * @details Optimized with lookup table for performance
         */
        [[nodiscard]] static double convertRuntime(
                double value,
                TimeUnit from,
                TimeUnit to
                ) noexcept;

        /**
         * @brief Convert Duration to any unit at runtime
         * @param duration Duration to convert
         * @param unit Target unit
         * @return Value in target unit
         */
        [[nodiscard]] static double toUnitRuntime(
                const Duration& duration,
                TimeUnit unit
                ) noexcept;

        /**
         * @brief Create Duration from value in any unit
         * @param value Value in source unit
         * @param unit Source unit
         * @return Duration object
         */
        [[nodiscard]] static Duration fromUnitRuntime(
                double value,
                TimeUnit unit
                ) noexcept;

        // =============================================================================
        // Chrono Conversions
        // =============================================================================

        /**
         * @brief Convert std::chrono duration to engine Duration
         * @tparam Rep Representation type
         * @tparam Period Ratio type
         * @param chrono_duration Input chrono duration
         * @return Engine Duration
         */
        template <typename Rep, typename Period>
        [[nodiscard]] static constexpr Duration fromChrono(
                const std::chrono::duration<Rep, Period>& chrono_duration
                ) noexcept {
            return std::chrono::duration_cast<Duration>(chrono_duration);
        }

        /**
         * @brief Convert engine Duration to std::chrono duration
         * @tparam TargetDuration Target chrono duration type
         * @param duration Engine duration
         * @return Chrono duration
         */
        template <typename TargetDuration>
        [[nodiscard]] static constexpr TargetDuration toChrono(
                const Duration& duration
                ) noexcept {
            return std::chrono::duration_cast<TargetDuration>(duration);
        }

        // =============================================================================
        // Special Conversions
        // =============================================================================

        /**
         * @brief Convert frame count to duration at given FPS
         * @param frames Number of frames
         * @param fps Frames per second
         * @return Duration
         */
        [[nodiscard]] static constexpr Duration framesToDuration(
                const FrameNumber frames,
                const double fps
                ) noexcept {
            if (fps <= 0.0)
                return Duration::zero();
            const double seconds = static_cast<double>(frames) / fps;
            return fromSeconds(seconds);
        }

        /**
         * @brief Convert duration to frame count at given FPS
         * @param duration Time duration
         * @param fps Frames per second
         * @return Number of frames
         */
        [[nodiscard]] static constexpr FrameNumber durationToFrames(
                const Duration& duration,
                const double fps
                ) noexcept {
            if (fps <= 0.0)
                return 0;
            const double seconds = toSeconds(duration);
            return static_cast<FrameNumber>(std::round(seconds * fps));
        }

        /**
         * @brief Convert beats per minute to duration per beat
         * @param bpm Beats per minute
         * @return Duration per beat
         */
        [[nodiscard]] static constexpr Duration bpmToDuration(const double bpm) noexcept {
            if (bpm <= 0.0)
                return Duration::max();
            const double seconds_per_beat = 60.0 / bpm;
            return fromSeconds(seconds_per_beat);
        }

        /**
         * @brief Convert duration per beat to BPM
         * @param beat_duration Duration of one beat
         * @return Beats per minute
         */
        [[nodiscard]] static constexpr double durationToBpm(
                const Duration& beat_duration
                ) noexcept {
            const double seconds = toSeconds(beat_duration);
            if (seconds <= 0.0)
                return 0.0;
            return 60.0 / seconds;
        }

        // =============================================================================
        // Precision Conversions
        // =============================================================================

        /**
         * @brief Convert with precision loss detection
         * @param value Value to convert
         * @param from Source unit
         * @param to Target unit
         * @param precision_loss Output: true if precision lost
         * @return Converted value
         */
        [[nodiscard]] static double convertWithPrecisionCheck(
                double value,
                TimeUnit from,
                TimeUnit to,
                bool& precision_loss
                ) noexcept;

        /**
         * @brief Get optimal unit for displaying duration
         * @param duration Duration to analyze
         * @return Most readable unit for the duration
         * @details Selects unit that gives values between 1-999
         */
        [[nodiscard]] static TimeUnit getOptimalUnit(const Duration& duration) noexcept;

        /**
         * @brief Round duration to nearest multiple of unit
         * @param duration Duration to round
         * @param unit Unit to round to
         * @return Rounded duration
         */
        [[nodiscard]] static Duration roundToUnit(
                const Duration& duration,
                TimeUnit unit
                ) noexcept;

        // =============================================================================
        // Unit Information
        // =============================================================================

        /**
         * @brief Get unit abbreviation
         * @param unit Time unit
         * @return Abbreviation string (e.g., "ms", "s")
         */
        [[nodiscard]] static constexpr std::string_view getUnitAbbreviation(
                TimeUnit unit
                ) noexcept {
            return unitAbbreviations_[static_cast<std::size_t>(unit)];
        }

        /**
         * @brief Get unit full name
         * @param unit Time unit
         * @return Full name string (e.g., "milliseconds")
         */
        [[nodiscard]] static constexpr std::string_view getUnitName(
                TimeUnit unit
                ) noexcept {
            return unitNames_[static_cast<std::size_t>(unit)];
        }

        /**
         * @brief Parse time unit from string
         * @param str String representation
         * @return Time unit or nullopt if invalid
         */
        [[nodiscard]] static std::optional<TimeUnit> parseUnit(
                std::string_view str
                ) noexcept;

        // =============================================================================
        // Validation
        // =============================================================================

        /**
         * @brief Check if conversion would overflow
         * @param value Value to convert
         * @param from Source unit
         * @param to Target unit
         * @return True if conversion would overflow
         */
        [[nodiscard]] static bool wouldOverflow(
                double value,
                TimeUnit from,
                TimeUnit to
                ) noexcept;

        /**
         * @brief Check if duration is within valid range for unit
         * @param duration Duration to check
         * @param unit Target unit
         * @return True if duration fits in unit's range
         */
        [[nodiscard]] static bool isValidForUnit(
                const Duration& duration,
                TimeUnit unit
                ) noexcept;

    private:
        // Lookup tables for runtime performance
        static constexpr std::array<std::string_view, static_cast<std::size_t>(TimeUnit::COUNT)>
        unitAbbreviations_ = {
                        "ns",
                        "μs",
                        "ms",
                        "s",
                        "min",
                        "h",
                        "d",
                        "w"
                };

        static constexpr std::array<std::string_view, static_cast<std::size_t>(TimeUnit::COUNT)>
        unitNames_ = {
                        "nanoseconds",
                        "microseconds",
                        "milliseconds",
                        "seconds",
                        "minutes",
                        "hours",
                        "days",
                        "weeks"
                };

        static constexpr std::array<double, static_cast<std::size_t>(TimeUnit::COUNT)>
        nanosPerUnit_ = {
                        1.0,
                        // nanoseconds
                        1'000.0,
                        // microseconds
                        1'000'000.0,
                        // milliseconds
                        1'000'000'000.0,
                        // seconds
                        60'000'000'000.0,
                        // minutes
                        3'600'000'000'000.0,
                        // hours
                        86'400'000'000'000.0,
                        // days
                        604'800'000'000'000.0 // weeks
                };

        // Conversion ratio cache for runtime conversions
        struct ConversionCache {
            static constexpr std::size_t CACHE_SIZE =
                    static_cast<std::size_t>(TimeUnit::COUNT) *
                    static_cast<std::size_t>(TimeUnit::COUNT);

            std::array<double, CACHE_SIZE> ratios{};

            constexpr ConversionCache() noexcept {
                for (std::size_t from = 0; from < static_cast<std::size_t>(TimeUnit::COUNT); ++from) {
                    for (std::size_t to = 0; to < static_cast<std::size_t>(TimeUnit::COUNT); ++to) {
                        const std::size_t index = from * static_cast<std::size_t>(TimeUnit::COUNT) + to;
                        ratios[index] = nanosPerUnit_[from] / nanosPerUnit_[to];
                    }
                }
            }

            [[nodiscard]] constexpr double getRatio(TimeUnit from, TimeUnit to) const noexcept {
                const std::size_t index =
                        static_cast<std::size_t>(from) * static_cast<std::size_t>(TimeUnit::COUNT) +
                        static_cast<std::size_t>(to);
                return ratios[index];
            }
        };

        static inline ConversionCache conversionCache_{};
    };

} // namespace engine::time
