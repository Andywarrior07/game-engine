/**
 * @file TimeConfig.h
 * @brief Core time system configuration structures
 * @details Provides base configuration for time subsystems including default values,
 *          limits, validation rules, and platform-specific adjustments. This file
 *          defines the fundamental configuration building blocks used throughout
 *          the time management system.
 *
 * @author Andres Guerrero
 * @date Created on 2025-09-19
 */

#pragma once

#include "TimeTypes.h"
#include "TimelineType.h"
#include "TimestepMode.h"

#include "../profiling/ProfilingConfig.h"

#include <stdexcept>

namespace engine::time {
    // =============================================================================
    // Platform Detection
    // =============================================================================

    /**
     * @brief Target platform enumeration
     * @details Used for platform-specific optimizations and configurations.
     *          Detected at compile-time where possible.
     */
    enum class Platform : std::uint8_t {
        WINDOWS,
        MAC_OS,
        LINUX,
        ANDROID,
        IOS,
        PLAYSTATION,
        XBOX,
        NINTENDO_SWITCH,
        WEB,
        UNKNOWN
    };

    /**
     * @brief Get current platform at compile time
     */
    [[nodiscard]] constexpr Platform getCurrentPlatform() noexcept {
#ifdef _WIN32
        return Platform::WINDOWS;
#elif defined(__APPLE__)
#include <TargetConditionals.h>
#if TARGET_OS_IOS
        return Platform::IOS;
#else
        return Platform::MAC_OS;
#endif
#elif defined(__linux__)
#ifdef __ANDROID__
        return Platform::ANDROID;
#else
        return Platform::LINUX;
#endif
#elif defined(__EMSCRIPTEN__)
        return Platform::WEB;
#else
        return Platform::UNKNOWN;
#endif
    }

    // =============================================================================
    // Timer Configuration
    // =============================================================================

    /**
     * @brief Core timer system configuration
     * @details Settings for timer allocation, pooling, and performance.
     *          Affects memory usage and timer creation overhead.
     */
    struct TimerConfig {
        std::size_t maxTimers{constants::MAX_TIMERS}; ///< Maximum concurrent timers
        std::size_t initialPoolSize{1024}; ///< Initial timer pool size
        std::size_t poolGrowthFactor{2}; ///< Pool growth multiplier
        bool enableTimerPooling{true}; ///< Use object pooling
        bool enableTimerRecycling{true}; ///< Recycle timer IDs
        bool enablePriorityQueue{true}; ///< Use heap for scheduling
        bool enableBatchProcessing{true}; ///< Process timers in batches
        std::uint16_t batchSize{64}; ///< Timer batch size
        Duration minTimerDuration{Duration(1000)}; ///< Minimum timer duration (1ms)
        Duration timerResolution{Duration(100)}; ///< Timer precision (100μs)
        bool allowNegativeTimers{false}; ///< Allow countdown past zero

        /**
         * @brief Validate timer configuration
         * @throws std::invalid_argument if configuration is invalid
         */
        void validate() const {
            if (maxTimers == 0) {
                throw std::invalid_argument("maxTimers must be greater than 0");
            }
            if (initialPoolSize > maxTimers) {
                throw std::invalid_argument("initialPoolSize cannot exceed maxTimers");
            }
            if (poolGrowthFactor < 1) {
                throw std::invalid_argument("poolGrowthFactor must be at least 1");
            }
            if (batchSize == 0 || batchSize > maxTimers) {
                throw std::invalid_argument("Invalid batchSize");
            }
            if (timerResolution <= Duration::zero()) {
                throw std::invalid_argument("timerResolution must be positive");
            }
        }
    };

    // =============================================================================
    // Timeline Configuration Base
    // =============================================================================

    /**
     * @brief Base configuration for individual timelines
     * @details Common settings shared by all timeline types.
     *          Can be specialized per timeline type.
     */
    struct TimelineConfigBase {
        TimelineType type{TimelineType::GAME_TIME}; ///< Timeline type
        TimestepConfig timestepConfig; ///< Timestep strategy
        TimeScale initialScale{1.0}; ///< Initial time scale
        bool startPaused{false}; ///< Start in paused state
        bool autoStart{true}; ///< Auto-start on creation
        Duration updateInterval{Duration::zero()}; ///< Min time between updates
        std::uint8_t priority{128}; ///< Update priority (0-255)
        std::size_t maxEventQueue{256}; ///< Max queued events
        bool enableInterpolation{false}; ///< Enable interpolation
        bool enableExtrapolation{false}; ///< Enable extrapolation
        float maxExtrapolationFactor{0.25f}; ///< Max extrapolation amount

        /**
         * @brief Validate timeline configuration
         * @throws std::invalid_argument if configuration is invalid
         */
        void validate() const {
            if (initialScale < 0.0) {
                throw std::invalid_argument("initialScale cannot be negative");
            }
            if (initialScale > constants::MAX_TIME_SCALE) {
                throw std::invalid_argument("initialScale exceeds maximum");
            }
            if (updateInterval < Duration::zero()) {
                throw std::invalid_argument("updateInterval cannot be negative");
            }
            if (maxExtrapolationFactor < 0.0f || maxExtrapolationFactor > 1.0f) {
                throw std::invalid_argument("maxExtrapolationFactor must be in [0, 1]");
            }
            // Validate timestep config
            // TODO: Revisar esto
            timestepConfig.modeConfig.index(); // Ensure variant is valid
        }
    };

    // =============================================================================
    // Clock Configuration
    // =============================================================================

    /**
     * @brief Platform-specific clock configuration
     * @details Settings for time source and precision based on platform.
     *          Optimizes for accuracy vs performance trade-offs.
     */
    struct ClockConfig {
        bool useHighPrecisionClock{true}; ///< Use QPC/mach_time
        bool useMonotonicClock{true}; ///< Monotonic time source
        bool compensateForDrift{true}; ///< Compensate clock drift
        Duration clockResolution{Duration(100)}; ///< Expected resolution
        Duration driftThreshold{Duration(1000)}; ///< Drift detection threshold
        std::uint8_t driftSamples{10}; ///< Samples for drift calc
        bool syncWithSystemTime{false}; ///< Sync to wall clock
        Duration syncInterval{Duration(60000000)}; ///< Sync every minute

        /**
         * @brief Get platform-optimized clock configuration
         */
        [[nodiscard]] static ClockConfig getPlatformOptimized() {
            const Platform platform = getCurrentPlatform();

            switch (platform) {
            case Platform::WINDOWS:
                return {
                    .useHighPrecisionClock = true, // QueryPerformanceCounter
                    .useMonotonicClock = true,
                    .compensateForDrift = true,
                    .clockResolution = Duration(100), // ~100ns on modern Windows
                    .driftThreshold = Duration(1000),
                    .driftSamples = 10,
                    .syncWithSystemTime = false
                };

            case Platform::MAC_OS:
            case Platform::IOS:
                return {
                    .useHighPrecisionClock = true, // mach_absolute_time
                    .useMonotonicClock = true,
                    .compensateForDrift = false, // mach_time doesn't drift
                    .clockResolution = Duration(42), // ~42ns on Apple Silicon
                    .driftThreshold = Duration(100),
                    .driftSamples = 5,
                    .syncWithSystemTime = false
                };

            case Platform::LINUX:
                return {
                    .useHighPrecisionClock = true, // clock_gettime MONOTONIC
                    .useMonotonicClock = true,
                    .compensateForDrift = true,
                    .clockResolution = Duration(1), // 1ns resolution claimed
                    .driftThreshold = Duration(500),
                    .driftSamples = 8,
                    .syncWithSystemTime = false
                };

            default:
                return ClockConfig{}; // Default configuration
            }
        }

        /**
         * @brief Validate clock configuration
         */
        void validate() const {
            if (clockResolution <= Duration::zero()) {
                throw std::invalid_argument("clockResolution must be positive");
            }
            if (driftThreshold <= Duration::zero()) {
                throw std::invalid_argument("driftThreshold must be positive");
            }
            if (driftSamples == 0) {
                throw std::invalid_argument("driftSamples must be at least 1");
            }
        }
    };

    // =============================================================================
    // Memory Configuration
    // =============================================================================

    /**
     * @brief Memory settings for time system
     * @details Controls memory allocation and pooling strategies.
     *          Integrates with engine's memory management system.
     */
    struct TimeMemoryConfig {
        std::size_t totalBudget{16 * 1024 * 1024}; ///< Total memory budget (16MB)
        std::size_t timelineMemory{8 * 1024 * 1024}; ///< Timeline allocations (8MB)
        std::size_t timerMemory{4 * 1024 * 1024}; ///< Timer allocations (4MB)
        std::size_t profilingMemory{2 * 1024 * 1024}; ///< Profiling data (2MB)
        std::size_t scratchMemory{2 * 1024 * 1024}; ///< Temp allocations (2MB)
        bool useMemoryPools{true}; ///< Use memory pooling
        bool preallocateMemory{true}; ///< Preallocate all memory
        bool zeroMemoryOnAllocate{false}; ///< Zero new allocations
        std::size_t alignmentBoundary{64}; ///< Cache line alignment

        /**
         * @brief Validate memory configuration
         */
        void validate() const {
            const std::size_t totalUsed = timelineMemory + timerMemory +
                profilingMemory + scratchMemory;
            if (totalUsed > totalBudget) {
                throw std::invalid_argument("Memory allocations exceed total budget");
            }
            if (alignmentBoundary == 0 || (alignmentBoundary & (alignmentBoundary - 1)) != 0) {
                throw std::invalid_argument("alignmentBoundary must be power of 2");
            }
        }

        /**
         * @brief Get memory configuration for platform
         */
        [[nodiscard]] static TimeMemoryConfig getPlatformOptimized() {
            const Platform platform = getCurrentPlatform();

            // Mobile platforms get reduced memory budgets
            if (platform == Platform::ANDROID || platform == Platform::IOS) {
                return {
                    .totalBudget = 8 * 1024 * 1024, // 8MB total
                    .timelineMemory = 4 * 1024 * 1024, // 4MB
                    .timerMemory = 2 * 1024 * 1024, // 2MB
                    .profilingMemory = 1024 * 1024, // 1MB
                    .scratchMemory = 1024 * 1024, // 1MB
                    .useMemoryPools = true,
                    .preallocateMemory = false, // Allocate on demand
                    .zeroMemoryOnAllocate = false,
                    .alignmentBoundary = 32 // Smaller cache lines
                };
            }

            // Console platforms get fixed allocations
            if (platform == Platform::PLAYSTATION ||
                platform == Platform::XBOX ||
                platform == Platform::NINTENDO_SWITCH) {
                return {
                    .totalBudget = 32 * 1024 * 1024, // 32MB total
                    .timelineMemory = 16 * 1024 * 1024, // 16MB
                    .timerMemory = 8 * 1024 * 1024, // 8MB
                    .profilingMemory = 4 * 1024 * 1024, // 4MB
                    .scratchMemory = 4 * 1024 * 1024, // 4MB
                    .useMemoryPools = true,
                    .preallocateMemory = true, // Preallocate everything
                    .zeroMemoryOnAllocate = false,
                    .alignmentBoundary = 128 // Larger cache lines
                };
            }

            // Default PC configuration
            return TimeMemoryConfig{};
        }
    };

    // =============================================================================
    // Synchronization Configuration
    // =============================================================================

    /**
     * @brief Thread synchronization settings
     * @details Controls threading behavior and synchronization strategies.
     *          Critical for multi-threaded time system operation.
     */
    struct SyncConfig {
        bool enableMultithreading{true}; ///< Use multiple threads
        bool useLockFreeStructures{true}; ///< Prefer lock-free
        bool useAtomicOperations{true}; ///< Use atomics
        std::uint8_t maxConcurrentTimelines{8}; ///< Parallel timelines
        std::uint8_t timerProcessingThreads{2}; ///< Timer worker threads
        bool separateRenderThread{true}; ///< Dedicated render thread
        bool separatePhysicsThread{true}; ///< Dedicated physics thread
        Duration threadSleepGranularity{Duration(1000)}; ///< Thread sleep precision
        bool useSpinLocks{false}; ///< Spin vs mutex
        std::uint32_t spinIterations{1000}; ///< Spin lock iterations

        /**
         * @brief Validate synchronization configuration
         */
        void validate() const {
            if (maxConcurrentTimelines == 0) {
                throw std::invalid_argument("maxConcurrentTimelines must be at least 1");
            }
            if (useSpinLocks && spinIterations == 0) {
                throw std::invalid_argument("spinIterations must be positive when using spin locks");
            }
        }
    };

    // =============================================================================
    // Validation and Limits
    // =============================================================================

    /**
     * @brief System-wide limits and constraints
     * @details Hard limits to prevent resource exhaustion.
     *          These values should rarely need adjustment.
     */
    struct SystemLimits {
        static constexpr std::size_t MAX_TIMELINES = 16;
        static constexpr std::size_t MAX_TIMERS_PER_TIMELINE = 4096;
        static constexpr std::size_t MAX_CALLBACKS_PER_TIMER = 8;
        static constexpr std::size_t MAX_TIMELINE_DEPTH = 4; // Nested timeline limit
        static constexpr auto MIN_TIMER_RESOLUTION = Duration(1); // 1 microsecond
        static constexpr auto MAX_TIMER_DURATION = Duration(86400000000); // 24 hours
        static constexpr TimeScale MIN_TIME_SCALE = 0.0;
        static constexpr TimeScale MAX_TIME_SCALE = 100.0;
        static constexpr std::uint32_t MAX_FRAME_SKIP = 10;
        static constexpr std::uint32_t MAX_INTERPOLATION_SAMPLES = 4;
    };

    // =============================================================================
    // Core Configuration Bundle
    // =============================================================================

    /**
     * @brief Complete core configuration for time subsystems
     * @details Aggregates all core configurations with validation.
     *          This is the base configuration used by TimeManagerConfig.
     */
    struct CoreTimeConfig {
        ClockConfig clock; ///< Clock settings
        TimeMemoryConfig memory; ///< Memory settings
        TimerConfig timer; ///< Timer settings
        SyncConfig sync; ///< Synchronization
        ProfilingConfig profiling; ///< Profiling settings

        /**
         * @brief Validate entire configuration
         * @throws std::invalid_argument if any part is invalid
         */
        void validate() const {
            clock.validate();
            memory.validate();
            timer.validate();
            sync.validate();
            // ProfilingConfig doesn't need validation
        }

        /**
         * @brief Get optimized configuration for current platform
         */
        [[nodiscard]] static CoreTimeConfig getPlatformOptimized() {
            return {
                .clock = ClockConfig::getPlatformOptimized(),
                .memory = TimeMemoryConfig::getPlatformOptimized(),
                .timer = TimerConfig{}, // Use defaults
                .sync = SyncConfig{}, // Use defaults
                .profiling = ProfilingConfig::getCurrentBuildConfig()
            };
        }

        /**
         * @brief Get minimal configuration for testing
         */
        [[nodiscard]] static CoreTimeConfig getMinimal() {
            return {
                .clock = ClockConfig{
                    .useHighPrecisionClock = false,
                    .compensateForDrift = false
                },
                .memory = TimeMemoryConfig{
                    .totalBudget = 1024 * 1024, // 1MB
                    .timelineMemory = 512 * 1024,
                    .timerMemory = 256 * 1024,
                    .profilingMemory = 128 * 1024,
                    .scratchMemory = 128 * 1024,
                    .useMemoryPools = false,
                    .preallocateMemory = false
                },
                .timer = TimerConfig{
                    .maxTimers = 128,
                    .initialPoolSize = 32,
                    .enableTimerPooling = false,
                    .enableBatchProcessing = false
                },
                .sync = SyncConfig{
                    .enableMultithreading = false,
                    .useLockFreeStructures = false
                },
                .profiling = ProfilingConfig::getReleaseConfig()
            };
        }
    };
} // namespace engine::time
