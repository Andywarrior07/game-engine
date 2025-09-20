/**
 * @file ProfilingConfig.h
 * @brief Performance profiling and monitoring configuration
 * @details Defines performance monitoring settings, budget thresholds, analytics options,
 *          and debug/release mode behaviors. Provides compile-time and runtime
 *          configuration for comprehensive time system profiling.
 *
 * @author Andres Guerrero
 * @date Created on 2025-09-19
 */

#pragma once

#include "../core/TimeTypes.h"

#include <array>
#include <string_view>
#include <functional>

namespace engine::time {
    // =============================================================================
    // Profiling Levels
    // =============================================================================

    /**
     * @brief Profiling detail levels
     * @details Controls the granularity of performance data collection.
     *          Higher levels provide more detail but impact performance.
     */
    enum class ProfilingLevel : std::uint8_t {
        DISABLED = 0, ///< No profiling (production builds)
        MINIMAL = 1, ///< Basic frame times only
        STANDARD = 2, ///< Per-system timing
        DETAILED = 3, ///< Sub-system breakdown
        VERBOSE = 4, ///< Every timer and operation
        TRACE = 5 ///< Full execution trace (debug only)
    };

    // =============================================================================
    // Performance Metrics
    // =============================================================================

    /**
     * @brief Types of performance metrics to track
     * @details Bitflags for selective metric collection.
     *          Allows fine-grained control over what to measure.
     */
    enum class MetricType : std::uint32_t {
        NONE = 0, ///< No metrics
        FRAME_TIME = 1 << 0, ///< Total frame duration
        UPDATE_TIME = 1 << 1, ///< Logic update duration
        RENDER_TIME = 1 << 2, ///< Rendering duration
        PHYSICS_TIME = 1 << 3, ///< Physics simulation time
        ANIMATION_TIME = 1 << 4, ///< Animation update time
        AUDIO_TIME = 1 << 5, ///< Audio processing time
        NETWORK_TIME = 1 << 6, ///< Network update time
        SCRIPT_TIME = 1 << 7, ///< Script execution time
        TIMER_OVERHEAD = 1 << 8, ///< Timer system overhead
        MEMORY_USAGE = 1 << 9, ///< Memory allocations
        CACHE_MISSES = 1 << 10, ///< CPU cache performance
        GPU_TIME = 1 << 11, ///< GPU frame time
        VSYNC_MISSES = 1 << 12, ///< Missed vsyncs
        FRAME_DROPS = 1 << 13, ///< Dropped frames
        LATENCY = 1 << 14, ///< Input-to-display latency
        JITTER = 1 << 15, ///< Frame time variance
        PERCENTILES = 1 << 16, ///< 95th/99th percentiles
        TIMELINE_STATS = 1 << 17, ///< Per-timeline statistics
        INTERPOLATION = 1 << 18, ///< Interpolation accuracy
        PREDICTION_ERROR = 1 << 19, ///< Prediction accuracy

        ALL = 0xFFFFFFFF ///< Track everything
    };

    /**
     * @brief Bitwise operators for MetricType
     */
    [[nodiscard]] constexpr MetricType operator|(MetricType a, MetricType b) noexcept {
        return static_cast<MetricType>(
            static_cast<std::uint32_t>(a) | static_cast<std::uint32_t>(b)
        );
    }

    [[nodiscard]] constexpr MetricType operator&(MetricType a, MetricType b) noexcept {
        return static_cast<MetricType>(
            static_cast<std::uint32_t>(a) & static_cast<std::uint32_t>(b)
        );
    }

    [[nodiscard]] constexpr bool hasMetric(const MetricType metrics, const MetricType metric) noexcept {
        return (metrics & metric) == metric;
    }

    // =============================================================================
    // Performance Budgets
    // =============================================================================

    /**
     * @brief System-specific performance budget
     * @details Time allocation for each engine subsystem.
     *          Used to detect and report budget overruns.
     */
    struct SystemBudget {
        std::string_view name; ///< System identifier
        Duration targetTime; ///< Target execution time
        Duration warningThreshold; ///< Warning if exceeded
        Duration criticalThreshold; ///< Critical alert threshold
        float allocationPercent; ///< Percentage of frame budget
        bool enforceInRelease; ///< Enforce in release builds
    };

    /**
     * @brief Frame time budget configuration
     * @details Overall frame time targets and thresholds.
     *          Defines acceptable performance boundaries.
     */
    struct FrameBudget {
        Duration targetFrameTime{constants::TARGET_FRAME_TIME_60FPS}; ///< Target frame duration
        Duration acceptableFrameTime{Duration(20000)}; ///< 50fps threshold
        Duration warningFrameTime{Duration(33333)}; ///< 30fps warning
        Duration criticalFrameTime{Duration(50000)}; ///< 20fps critical
        std::uint8_t consecutiveDropsWarning{3}; ///< Drops before warning
        std::uint8_t consecutiveDropsCritical{10}; ///< Drops before critical
        float targetUtilization{0.9f}; ///< Target CPU usage (90%)
    };

    /**
     * @brief Default system budgets for 60fps target
     * @details Predefined time allocations for engine subsystems.
     *          Based on typical game workload distribution.
     */
    constexpr std::array<SystemBudget, 8> DEFAULT_SYSTEM_BUDGETS = {
        {
            {"Rendering", Duration(8000), Duration(10000), Duration(12000), 48.0f, true},
            {"Physics", Duration(3000), Duration(4000), Duration(5000), 18.0f, true},
            {"GameLogic", Duration(2000), Duration(3000), Duration(4000), 12.0f, false},
            {"Animation", Duration(1500), Duration(2000), Duration(2500), 9.0f, false},
            {"Audio", Duration(500), Duration(1000), Duration(1500), 3.0f, false},
            {"Networking", Duration(1000), Duration(1500), Duration(2000), 6.0f, true},
            {"UI", Duration(500), Duration(1000), Duration(1500), 3.0f, false},
            {"Scripting", Duration(167), Duration(500), Duration(1000), 1.0f, false}
        }
    };

    // =============================================================================
    // Analytics Configuration
    // =============================================================================

    /**
     * @brief Statistical analysis configuration
     * @details Controls what statistical data is computed.
     *          More analysis provides better insights but costs performance.
     */
    struct AnalyticsConfig {
        bool calculateMean{true}; ///< Average values
        bool calculateMedian{true}; ///< Median values
        bool calculateStdDev{true}; ///< Standard deviation
        bool calculatePercentiles{true}; ///< 95th/99th percentiles
        bool trackMinMax{true}; ///< Min/max values
        bool detectSpikes{true}; ///< Spike detection
        bool detectPatterns{false}; ///< Pattern recognition
        bool predictTrends{false}; ///< Trend prediction
        std::uint16_t historySize{300}; ///< Samples to keep
        std::uint8_t percentileLevels[3]{95, 99, 100}; ///< Percentile levels
        float spikeThresholdMultiplier{2.0f}; ///< Spike = mean * multiplier
        std::uint8_t spikeWindowSize{5}; ///< Spike detection window
    };

    // =============================================================================
    // Reporting Configuration
    // =============================================================================

    /**
     * @brief Performance report generation settings
     * @details Controls how and when performance reports are generated.
     *          Balances detail with performance impact.
     */
    struct ReportingConfig {
        bool enabled{true}; ///< Enable reporting
        bool autoReport{false}; ///< Automatic periodic reports
        Duration reportInterval{Duration(1000000)}; ///< Report every second
        bool reportOnBudgetExceed{true}; ///< Report budget violations
        bool reportOnFrameDrop{true}; ///< Report frame drops
        bool includeHistogram{false}; ///< Include timing histogram
        bool includeCallGraph{false}; ///< Include call graph
        bool csvExport{false}; ///< Export CSV data
        bool jsonExport{false}; ///< Export JSON data
        std::string_view reportPath{"profiling/"}; ///< Report output path
        std::uint32_t maxReportSize{1024 * 1024}; ///< Max report size (1MB)
    };

    // =============================================================================
    // Memory Profiling Configuration
    // =============================================================================

    /**
     * @brief Memory profiling settings
     * @details Controls memory usage tracking for time systems.
     *          Helps identify memory leaks and allocation patterns.
     */
    struct MemoryProfilingConfig {
        bool trackAllocations{true}; ///< Track all allocations
        bool trackDeallocations{true}; ///< Track deallocations
        bool trackPeakUsage{true}; ///< Track peak memory
        bool trackFragmentation{false}; ///< Analyze fragmentation
        bool trackCacheLines{false}; ///< Cache line analysis
        std::size_t allocationThreshold{1024}; ///< Min size to track
        std::uint32_t maxTrackedAllocations{10000}; ///< Max tracked allocs
    };

    // =============================================================================
    // Debug vs Release Configurations
    // =============================================================================

    /**
     * @brief Complete profiling configuration
     * @details Combines all profiling subsystem configurations.
     *          Different defaults for debug and release builds.
     */
    struct ProfilingConfig {
        ProfilingLevel level; ///< Overall profiling level
        MetricType enabledMetrics; ///< Which metrics to track
        FrameBudget frameBudget; ///< Frame time budgets
        std::array<SystemBudget, 8> systemBudgets; ///< Per-system budgets
        AnalyticsConfig analytics; ///< Statistical analysis
        ReportingConfig reporting; ///< Report generation
        MemoryProfilingConfig memory; ///< Memory tracking

        // Callbacks for performance events
        std::function<void(Duration)> onFrameExceedBudget;
        std::function<void(const SystemBudget&, Duration)> onSystemExceedBudget;
        std::function<void(std::uint32_t)> onFrameDrop;
        std::function<void(Duration)> onFrameSpike;

        /**
         * @brief Get debug configuration
         * @details Maximum profiling for development
         */
        // TODO: Revisar este warning
        [[nodiscard]] static ProfilingConfig getDebugConfig() {
            return {
                .level = ProfilingLevel::DETAILED,
                .enabledMetrics = MetricType::ALL,
                .frameBudget = FrameBudget{},
                .systemBudgets = DEFAULT_SYSTEM_BUDGETS,
                .analytics = AnalyticsConfig{
                    .detectPatterns = true,
                    .predictTrends = true,
                    .historySize = 600
                },
                .reporting = ReportingConfig{
                    .autoReport = true,
                    .includeHistogram = true,
                    .includeCallGraph = true
                },
                .memory = MemoryProfilingConfig{
                    .trackFragmentation = true,
                    .trackCacheLines = true
                }
            };
        }

        /**
         * @brief Get release configuration
         * @details Minimal profiling for production
         */
        // TODO: Revisar este warning
        [[nodiscard]] static ProfilingConfig getReleaseConfig() {
            return {
                .level = ProfilingLevel::MINIMAL,
                .enabledMetrics = MetricType::FRAME_TIME |
                MetricType::FRAME_DROPS |
                MetricType::VSYNC_MISSES,
                .frameBudget = FrameBudget{},
                .systemBudgets = DEFAULT_SYSTEM_BUDGETS,
                .analytics = AnalyticsConfig{
                    .calculatePercentiles = false,
                    .detectPatterns = false,
                    .predictTrends = false,
                    .historySize = 60
                },
                .reporting = ReportingConfig{
                    .enabled = false,
                    .autoReport = false
                },
                .memory = MemoryProfilingConfig{
                    .trackAllocations = false,
                    .trackDeallocations = false,
                    .trackFragmentation = false,
                    .trackCacheLines = false
                }
            };
        }

        /**
         * @brief Get profiling configuration for specific build
         */
        [[nodiscard]] static ProfilingConfig getCurrentBuildConfig() {
#ifdef _DEBUG
            return getDebugConfig();
#else
            return getReleaseConfig();
#endif
        }
    };

    // =============================================================================
    // Profiling Utilities
    // =============================================================================

    /**
     * @brief Check if profiling is enabled for metric
     */
    [[nodiscard]] inline bool isMetricEnabled(const ProfilingConfig& config, const MetricType metric) noexcept {
        return config.level != ProfilingLevel::DISABLED &&
            hasMetric(config.enabledMetrics, metric);
    }

    /**
     * @brief Get human-readable name for profiling level
     */
    [[nodiscard]] constexpr std::string_view getProfilingLevelName(const ProfilingLevel level) noexcept {
        switch (level) {
        case ProfilingLevel::DISABLED: return "Disabled";
        case ProfilingLevel::MINIMAL: return "Minimal";
        case ProfilingLevel::STANDARD: return "Standard";
        case ProfilingLevel::DETAILED: return "Detailed";
        case ProfilingLevel::VERBOSE: return "Verbose";
        case ProfilingLevel::TRACE: return "Trace";
        default: return "Unknown";
        }
    }

    /**
     * @brief Calculate total frame budget from system budgets
     */
    [[nodiscard]] inline Duration calculateTotalBudget(
        const std::array<SystemBudget, 8>& budgets) noexcept {
        Duration total{0};
        for (const auto& budget : budgets) {
            total += budget.targetTime;
        }
        return total;
    }
} // namespace engine::time
