/**
 * @file TimeTypes.h
 * @brief Core type definitions and constants for the time management system
 * @details Provides platform-independent time representations, type-safe wrappers,
 *          and fundamental constants used throughout the time module.
 *          This is the foundational header with zero dependencies on other time system files.
 *
 * @author Andres Guerrero
 * @date Created on 2025-09-19
 */

#pragma once

#include <chrono>
#include <ratio>
#include <type_traits>
#include <limits>
#include <functional>

namespace engine::time {
    // =============================================================================
    // Core Time Type Aliases
    // =============================================================================

    /**
     * @brief High-resolution clock type used throughout the system
     * @details Uses steady_clock to ensure monotonic time progression.
     *          Immune to system clock adjustments and NTP synchronization.
     */
    using Clock = std::chrono::steady_clock;

    /**
     * @brief Primary timestamp type representing a point in time
     * @details Represents absolute time points for timeline tracking.
     *          Using steady_clock ensures monotonic behavior critical for game timing.
     */
    using TimeStamp = Clock::time_point;

    /**
     * @brief High-precision duration type for internal calculations
     * @details Uses microseconds for balance between precision and range.
     *          Provides ~292,000 year range with microsecond precision.
     */
    using Duration = std::chrono::duration<std::int64_t, std::micro>;

    /**
     * @brief Nanosecond duration for ultra-high precision measurements
     * @details Used for CPU profiling and micro-benchmarking where
     *          nanosecond precision is required. Limited to ~292 year range.
     */
    using NanoDuration = std::chrono::nanoseconds;

    /**
     * @brief Millisecond duration for user-facing time values
     * @details Common unit for game logic, animations, and UI.
     *          Provides intuitive values for designers and gameplay programmers.
     */
    using MilliDuration = std::chrono::milliseconds;

    /**
     * @brief Floating-point seconds for interpolation and scaling
     * @details Used for time scaling, interpolation factors, and shader uniforms.
     *          Double precision ensures accurate accumulation over long sessions.
     */
    using FloatSeconds = std::chrono::duration<double>;

    /**
     * @brief Frame counter type for frame-based timing
     * @details 64-bit to prevent overflow even at 144Hz for decades.
     *          Used for frame-perfect timing and replay systems.
     */
    using FrameNumber = std::uint64_t;

    /**
     * @brief Time scale factor type for timeline speed manipulation
     * @details Range [0.0, inf) where 1.0 is normal speed.
     *          0.0 represents pause, >1.0 is fast-forward, <1.0 is slow-motion.
     */
    using TimeScale = double;

    /**
     * @brief Unique identifier for timelines
     * @details Type-safe ID to prevent timeline confusion.
     *          Supports up to 256 concurrent timelines.
     */
    using TimelineID = std::uint8_t;

    /**
     * @brief Unique identifier for timers
     * @details 32-bit provides 4 billion unique timer IDs.
     *          IDs are recycled using a generation counter for safety.
     */
    using TimerID = std::uint32_t;

    /**
     * @brief Generation counter for timer handle validation
     * @details Prevents use-after-free by invalidating old handles.
     *          16-bit allows 65k timer recreations at the same ID.
     */
    using TimerGeneration = std::uint16_t;

    // =============================================================================
    // Constants and Limits
    // =============================================================================

    namespace constants {
        /**
         * @brief Target frame time for 60 FPS (16.666... ms)
         * @details Standard frame time for console games and vsync displays.
         *          Used as default timestep for fixed update loops.
         */
        constexpr auto TARGET_FRAME_TIME_60FPS = Duration(16667);

        /**
         * @brief Target frame time for 30 FPS (33.333... ms)
         * @details Common frame rate for mobile and lower-end hardware.
         *          Often used for complex scenes or battery optimization.
         */
        constexpr auto TARGET_FRAME_TIME_30FPS = Duration(33333);

        /**
         * @brief Target frame time for 120 FPS (8.333... ms)
         * @details High refresh rate for competitive gaming and VR.
         *          Requires optimized rendering and update loops.
         */
        constexpr auto TARGET_FRAME_TIME_120FPS = Duration(8333);

        /**
         * @brief Target frame time for 144 FPS (6.944... ms)
         * @details Premium gaming monitor refresh rate.
         *          Common target for esports and high-end PC gaming.
         */
        constexpr auto TARGET_FRAME_TIME_144FPS = Duration(6944);

        /**
         * @brief Maximum allowed delta time to prevent death spirals
         * @details Clamps frame time to 100ms (10 FPS minimum).
         *          Prevents physics explosions and gameplay issues during hitches.
         */
        constexpr auto MAX_DELTA_TIME = Duration(100000);

        /**
         * @brief Minimum delta time to prevent division by zero
         * @details 100 microseconds minimum ensures stable calculations.
         *          Prevents issues when timer resolution is exceeded.
         */
        constexpr auto MIN_DELTA_TIME = Duration(100);

        /**
         * @brief Default fixed timestep for physics (60Hz)
         * @details Industry standard for deterministic physics.
         *          Balances accuracy with performance across platforms.
         */
        constexpr Duration DEFAULT_FIXED_TIMESTEP = TARGET_FRAME_TIME_60FPS;

        /**
         * @brief Maximum fixed timestep iterations per frame
         * @details Prevents spiral of death in fixed timestep loops.
         *          Allows up to 5 physics updates per render frame.
         */
        constexpr std::uint8_t MAX_FIXED_ITERATIONS = 5;

        /**
         * @brief Default time scale for normal gameplay
         * @details 1.0 represents real-time progression.
         *          Modified for slow-motion and time manipulation effects.
         */
        constexpr TimeScale DEFAULT_TIME_SCALE = 1.0;

        /**
         * @brief Maximum time scale to prevent numerical issues
         * @details 10x speed maximum prevents precision loss.
         *          Higher values can cause animation and physics artifacts.
         */
        constexpr TimeScale MAX_TIME_SCALE = 10.0;

        /**
         * @brief Minimum time scale (excluding pause)
         * @details 0.01x (1% speed) for extreme slow-motion.
         *          Below this, consider using pause instead.
         */
        constexpr TimeScale MIN_TIME_SCALE = 0.01;

        /**
         * @brief Epsilon for time comparison operations
         * @details 1 microsecond tolerance for floating-point comparisons.
         *          Accounts for precision loss in time calculations.
         */
        constexpr auto TIME_EPSILON = Duration(1);

        /**
         * @brief Maximum number of concurrent timelines
         * @details Supports complex games with multiple time systems.
         *          Typically: real, game, UI, physics, audio + customs.
         */
        constexpr std::size_t MAX_TIMELINES = 16;

        /**
         * @brief Maximum number of active timers
         * @details Supports thousands of gameplay timers efficiently.
         *          Uses object pooling to minimize allocation overhead.
         */
        constexpr std::size_t MAX_TIMERS = 8192;

        /**
         * @brief Frame history buffer size for analytics
         * @details Stores 5 seconds of history at 60 FPS.
         *          Used for performance analysis and frame spike detection.
         */
        constexpr std::size_t FRAME_HISTORY_SIZE = 300;

        /**
         * @brief Invalid timeline ID sentinel value
         * @details Used to indicate uninitialized or invalid timelines.
         *          Checked in debug builds to catch usage errors.
         */
        constexpr TimelineID INVALID_TIMELINE_ID = std::numeric_limits<TimelineID>::max();

        /**
         * @brief Invalid timer ID sentinel value
         * @details Indicates uninitialized or destroyed timers.
         *          Combined with generation counter for safe handles.
         */
        constexpr TimerID INVALID_TIMER_ID = 0;
    } // namespace constants

    // =============================================================================
    // Time Concepts (C++20)
    // =============================================================================

    /**
     * @brief Concept for types that can represent time durations
     * @details Ensures type safety for duration-based operations.
     *          Accepts any std::chrono::duration specialization.
     */
    template <typename T>
    concept ChronoDuration = requires {
        typename T::rep;
        typename T::period;
        requires std::is_same_v<T, std::chrono::duration<typename T::rep, typename T::period>>;
    };

    /**
     * @brief Concept for types that can be used as time points
     * @details Validates clock-based time point types.
     *          Ensures compatibility with std::chrono time points.
     */
    template <typename T>
    concept ChronoTimePoint = requires {
        typename T::clock;
        typename T::duration;
        requires std::is_same_v<T, std::chrono::time_point<typename T::clock, typename T::duration>>;
    };

    /**
     * @brief Concept for numeric types suitable for time calculations
     * @details Restricts to arithmetic types for time math.
     *          Prevents accidental use of non-numeric types.
     */
    template <typename T>
    concept TimeArithmetic = std::is_arithmetic_v<T>;

    // =============================================================================
    // Type-Safe Time Handle
    // =============================================================================

    /**
     * @brief Type-safe handle for timer references
     * @details Combines ID and generation for safe timer access.
     *          Generation counter prevents use-after-free bugs.
     *          Zero-cost abstraction with validation in debug builds.
     */
    struct TimerHandle {
        TimerID id{constants::INVALID_TIMER_ID}; ///< Timer identifier
        TimerGeneration generation{0}; ///< Generation counter for validation

        /**
         * @brief Check if handle is valid (non-null)
         * @return True if handle references a potentially valid timer
         */
        [[nodiscard]] constexpr bool isValid() const noexcept {
            return id != constants::INVALID_TIMER_ID;
        }

        /**
         * @brief Reset handle to invalid state
         */
        constexpr void reset() noexcept {
            id = constants::INVALID_TIMER_ID;
            generation = 0;
        }

        /**
         * @brief Equality comparison for handle lookup
         */
        [[nodiscard]] constexpr bool operator==(const TimerHandle& other) const noexcept = default;

        /**
         * @brief Ordering comparison for container storage
         */
        [[nodiscard]] constexpr auto operator<=>(const TimerHandle& other) const noexcept = default;
    };

    // =============================================================================
    // Time Conversion Utilities
    // =============================================================================

    /**
     * @brief Convert duration to floating-point seconds
     * @tparam T Duration type (must satisfy ChronoDuration concept)
     * @param duration Input duration to convert
     * @return Duration in seconds as double
     * @details Preserves precision for sub-second durations.
     *          Used for shader uniforms and interpolation factors.
     */
    template <ChronoDuration T>
    [[nodiscard]] constexpr double toSeconds(const T& duration) noexcept {
        return std::chrono::duration<double>(duration).count();
    }

    /**
     * @brief Convert duration to floating-point milliseconds
     * @tparam T Duration type (must satisfy ChronoDuration concept)
     * @param duration Input duration to convert
     * @return Duration in milliseconds as double
     * @details Common unit for gameplay timers and animations.
     *          Provides intuitive values for designers.
     */
    template <ChronoDuration T>
    [[nodiscard]] constexpr double toMilliseconds(const T& duration) noexcept {
        return std::chrono::duration<double, std::milli>(duration).count();
    }

    /**
     * @brief Convert duration to integer microseconds
     * @tparam T Duration type (must satisfy ChronoDuration concept)
     * @param duration Input duration to convert
     * @return Duration in microseconds as int64_t
     * @details Preserves precision without floating-point errors.
     *          Used for internal time calculations.
     */
    template <ChronoDuration T>
    [[nodiscard]] constexpr std::int64_t toMicroseconds(const T& duration) noexcept {
        return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
    }

    /**
     * @brief Convert floating-point seconds to Duration
     * @param seconds Time in seconds
     * @return Duration representation
     * @details Converts designer-friendly values to internal format.
     *          Handles negative values for rewind operations.
     */
    [[nodiscard]] constexpr Duration fromSeconds(const double seconds) noexcept {
        return std::chrono::duration_cast<Duration>(FloatSeconds(seconds));
    }

    /**
     * @brief Convert floating-point milliseconds to Duration
     * @param milliseconds Time in milliseconds
     * @return Duration representation
     * @details Common conversion for UI and animation timers.
     *          Preserves sub-millisecond precision.
     */
    [[nodiscard]] constexpr Duration fromMilliseconds(const double milliseconds) noexcept {
        return std::chrono::duration_cast<Duration>(
            std::chrono::duration<double, std::milli>(milliseconds)
        );
    }

    /**
     * @brief Clamp duration to safe range
     * @tparam T Duration type
     * @param duration Input duration to clamp
     * @return Clamped duration within [MIN_DELTA_TIME, MAX_DELTA_TIME]
     * @details Prevents physics explosions and numerical issues.
     *          Essential for stable gameplay during frame drops.
     */
    template <ChronoDuration T>
    [[nodiscard]] constexpr T clampDeltaTime(const T& duration) noexcept {
        const auto clamped = std::chrono::duration_cast<Duration>(duration);
        if (clamped < constants::MIN_DELTA_TIME) {
            return std::chrono::duration_cast<T>(constants::MIN_DELTA_TIME);
        }
        if (clamped > constants::MAX_DELTA_TIME) {
            return std::chrono::duration_cast<T>(constants::MAX_DELTA_TIME);
        }
        return duration;
    }

    // =============================================================================
    // Callback Types
    // =============================================================================

    /**
     * @brief Timer callback function signature
     * @details Takes timer handle for self-reference and management.
     *          Can be used to reschedule or cancel from within callback.
     */
    using TimerCallback = std::function<void(TimerHandle)>;

    /**
     * @brief Timeline state change callback
     * @details Notifies when timeline is paused, resumed, or scaled.
     *          Used for UI updates and system synchronization.
     */
    using TimelineCallback = std::function<void(TimelineID, TimeScale)>;

    /**
     * @brief Frame update callback for custom systems
     * @details Receives frame number and delta time for updates.
     *          Used to integrate custom systems with time manager.
     */
    using FrameCallback = std::function<void(FrameNumber, Duration)>;

    /**
     * @brief Performance threshold breach callback
     * @details Triggered when frame time exceeds configured limits.
     *          Used for adaptive quality and performance monitoring.
     */
    using PerformanceCallback = std::function<void(Duration frameTime, Duration threshold)>;

    // =============================================================================
    // Hash Specializations
    // =============================================================================
} // namespace engine::time

// Provide std::hash specialization for TimerHandle
template <>
struct std::hash<engine::time::TimerHandle> {
    [[nodiscard]] std::size_t operator()(const engine::time::TimerHandle& handle) const noexcept {
        // Combine ID and generation for unique hash
        // Shift generation left by 32 bits and OR with ID
        return (static_cast<std::size_t>(handle.generation) << 32) |
            static_cast<std::size_t>(handle.id);
    }
}; // namespace std
