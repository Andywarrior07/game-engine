/**
 * @file PerformanceTracker.h
 * @brief Comprehensive performance tracking and analysis system
 * @details Provides detailed performance metrics collection, analysis, and
 *          reporting for time-critical systems. Uses circular buffers for
 *          efficient memory usage and lock-free operations where possible.
 *
 * @author Development Team
 * @date Created on 2024-01-20
 */

#pragma once

#include "../core/TimeTypes.h"
#include "../profiling/TimeStats.h"
#include "../utils/ScopedTimer.h"

#include "../../memory/MemorySystem.h"

#include <memory>
#include <atomic>
#include <mutex>
#include <unordered_map>
#include <functional>

namespace engine::memory {
    class CircularBufferAllocator;
}

namespace engine::time {

    // =============================================================================
    // Performance Tracker Configuration
    // =============================================================================

    /**
     * @brief Configuration for performance tracking
     */
    struct PerformanceTrackerConfig {
        // History settings
        std::size_t frameHistorySize{300}; ///< Frames to track (5 sec @ 60fps)
        std::size_t sampleHistorySize{1000}; ///< General samples to keep
        std::size_t eventHistorySize{500}; ///< Event history size

        // Tracking settings
        bool trackFrameStats{true}; ///< Track frame timing
        bool trackSystemStats{true}; ///< Track per-system timing
        bool trackMemoryStats{true}; ///< Track memory usage
        bool trackEventLatency{true}; ///< Track event latencies

        // Analysis settings
        bool calculateStatistics{true}; ///< Compute statistics
        bool detectAnomalies{true}; ///< Detect performance anomalies
        bool predictTrends{false}; ///< Predict future performance

        // Thresholds
        Duration frameTimeWarning{Duration(20000)}; ///< 50fps warning
        Duration frameTimeCritical{Duration(33333)}; ///< 30fps critical
        float spikeThreshold{2.0f}; ///< Spike detection multiplier
        float memoryWarningPercent{80.0f}; ///< Memory warning at 80%

        // Reporting
        bool autoReport{false}; ///< Auto-generate reports
        Duration reportInterval{Duration(5000000)}; ///< Report every 5 seconds
        std::function<void(const std::string&)> reportCallback;
    };

    // =============================================================================
    // Performance Sample
    // =============================================================================

    /**
     * @brief Single performance measurement sample
     */
    struct PerformanceSample {
        std::string name; ///< Sample identifier
        TimeStamp timestamp; ///< When sampled
        Duration duration; ///< Measured duration
        std::size_t memoryUsed{0}; ///< Memory at sample time
        std::uint32_t threadId{0}; ///< Thread that took sample
        std::uint32_t category{0}; ///< Sample category/type

        /**
         * @brief Comparison for sorting
         */
        bool operator<(const PerformanceSample& other) const noexcept {
            return timestamp < other.timestamp;
        }
    };

    // =============================================================================
    // Performance Report
    // =============================================================================

    /**
     * @brief Comprehensive performance analysis report
     */
    struct PerformanceReport {
        // Frame statistics
        FrameStats frameStats;

        // System breakdown
        std::unordered_map<std::string, BasicTimeStats> systemStats;

        // Memory statistics
        struct MemoryStats {
            std::size_t currentUsage{0};
            std::size_t peakUsage{0};
            std::size_t totalAllocations{0};
            std::size_t totalDeallocations{0};
            float fragmentationPercent{0.0f};
        } memoryStats;

        // Performance metrics
        struct Metrics {
            double averageFPS{0.0};
            double percentile95FPS{0.0};
            double percentile99FPS{0.0};
            Duration averageFrameTime{};
            Duration worstFrameTime{};
            float cpuUtilization{0.0f};
            float gpuUtilization{0.0f};
        } metrics;

        // Issues found
        struct Issues {
            std::uint32_t frameDrops{0};
            std::uint32_t spikes{0};
            std::uint32_t budgetExceeded{0};
            std::vector<std::string> warnings;
            std::vector<std::string> criticals;
        } issues;

        // Time range
        TimeStamp startTime;
        TimeStamp endTime;
        Duration totalDuration;

        /**
         * @brief Generate formatted report string
         */
        [[nodiscard]] std::string toString() const;
    };

    // =============================================================================
    // PerformanceTracker Class
    // =============================================================================

    /**
     * @brief Main performance tracking and analysis system
     * @details Collects, analyzes, and reports on system performance metrics
     *          using efficient circular buffers and statistical analysis.
     */
    class PerformanceTracker {
    public:
        /**
         * @brief Constructor with configuration
         * @param config Tracker configuration
         */
        explicit PerformanceTracker(PerformanceTrackerConfig  config = {});

        /**
         * @brief Destructor
         */
        ~PerformanceTracker();

        // Delete copy operations
        PerformanceTracker(const PerformanceTracker&) = delete;
        PerformanceTracker& operator=(const PerformanceTracker&) = delete;

        // =============================================================================
        // Initialization
        // =============================================================================

        /**
         * @brief Initialize with memory manager
         * @param memoryManager Memory manager for allocations
         * @return True if initialization successful
         */
        bool initialize(memory::MemoryManager* memoryManager);

        /**
         * @brief Shutdown and cleanup
         */
        void shutdown();

        /**
         * @brief Check if initialized
         */
        [[nodiscard]] bool isInitialized() const noexcept {
            return initialized_;
        }

        // =============================================================================
        // Frame Tracking
        // =============================================================================

        /**
         * @brief Begin frame tracking
         * @param frameNumber Frame identifier
         */
        void beginFrame(FrameNumber frameNumber);

        /**
         * @brief End frame tracking
         * @param cpuTime CPU processing time
         * @param gpuTime GPU rendering time
         */
        void endFrame(
                Duration cpuTime = Duration::zero(),
                Duration gpuTime = Duration::zero()
                );

        /**
         * @brief Record frame drop
         * @param reason Optional reason for drop
         */
        void recordFrameDrop(const std::string& reason = "");

        /**
         * @brief Record VSync miss
         */
        void recordVSyncMiss();

        // =============================================================================
        // System Tracking
        // =============================================================================

        /**
         * @brief Begin system timing
         * @param systemName System identifier
         * @return Scoped timer for automatic timing
         */
        [[nodiscard]] ScopedTimer beginSystemTiming(const std::string& systemName);

        /**
         * @brief Record system timing manually
         * @param systemName System identifier
         * @param duration System execution time
         */
        void recordSystemTiming(const std::string& systemName, Duration duration);

        /**
         * @brief Record system budget exceeded
         * @param systemName System identifier
         * @param actual Actual time taken
         * @param budget Budget that was exceeded
         */
        void recordBudgetExceeded(
                const std::string& systemName,
                Duration actual,
                Duration budget
                );

        // =============================================================================
        // Event Tracking
        // =============================================================================

        /**
         * @brief Record event latency
         * @param eventName Event identifier
         * @param latency Event latency
         */
        void recordEventLatency(const std::string& eventName, Duration latency);

        /**
         * @brief Record custom sample
         * @param sample Performance sample
         */
        void recordSample(const PerformanceSample& sample) const;

        // =============================================================================
        // Memory Tracking
        // =============================================================================

        /**
         * @brief Update memory statistics
         * @param currentUsage Current memory usage
         * @param allocations Number of allocations
         * @param deallocations Number of deallocations
         */
        void updateMemoryStats(
                std::size_t currentUsage,
                std::size_t allocations = 0,
                std::size_t deallocations = 0
                );

        // =============================================================================
        // Analysis
        // =============================================================================

        /**
         * @brief Analyze performance and detect issues
         * @return Analysis report
         */
        [[nodiscard]] PerformanceReport analyze() const;

        /**
         * @brief Get current frame statistics
         */
        [[nodiscard]] FrameStats getCurrentFrameStats() const;

        /**
         * @brief Get system statistics
         * @param systemName System identifier
         * @return System stats or nullopt if not found
         */
        [[nodiscard]] std::optional<BasicTimeStats> getSystemStats(
                const std::string& systemName
                ) const;

        /**
         * @brief Calculate percentile
         * @param percentile Percentile to calculate (0-100)
         * @return Duration at percentile
         */
        [[nodiscard]] Duration calculatePercentile(float percentile) const;

        /**
         * @brief Detect performance anomalies
         * @return List of detected anomalies
         */
        [[nodiscard]] std::vector<std::string> detectAnomalies() const;

        // =============================================================================
        // Reporting
        // =============================================================================

        /**
         * @brief Generate performance report
         * @param detailed Include detailed breakdown
         * @return Report string
         */
        [[nodiscard]] std::string generateReport(bool detailed = true) const;

        /**
         * @brief Export data to CSV
         * @param filepath Output file path
         * @return True if export successful
         */
        bool exportToCSV(const std::string& filepath) const;

        /**
         * @brief Export data to JSON
         * @param filepath Output file path
         * @return True if export successful
         */
        bool exportToJSON(const std::string& filepath) const;

        // =============================================================================
        // Configuration
        // =============================================================================

        /**
         * @brief Update configuration
         * @param config New configuration
         */
        void updateConfig(const PerformanceTrackerConfig& config);

        /**
         * @brief Get configuration
         */
        [[nodiscard]] const PerformanceTrackerConfig& getConfig() const noexcept {
            return config_;
        }

        // =============================================================================
        // Utilities
        // =============================================================================

        /**
         * @brief Reset all tracking data
         */
        void reset();

        /**
         * @brief Set report callback
         * @param callback Callback for auto-reports
         */
        void setReportCallback(std::function<void(const std::string&)> callback) {
            config_.reportCallback = std::move(callback);
        }

    private:
        // Configuration
        PerformanceTrackerConfig config_;
        bool initialized_{false};

        // Memory management
        memory::MemoryManager* memoryManager_{nullptr};
        std::unique_ptr<memory::CircularBufferAllocator> frameHistoryAllocator_;
        std::unique_ptr<memory::CircularBufferAllocator> sampleHistoryAllocator_;

        // Frame tracking
        FrameNumber currentFrame_{0};
        TimeStamp frameStartTime_;
        FrameHistory<300> frameHistory_; // Default size, will be configured
        std::atomic<bool> inFrame_{false};

        // Statistics
        mutable std::mutex statsMutex_;
        FrameStats frameStats_;
        std::unordered_map<std::string, BasicTimeStats> systemStats_;
        std::unordered_map<std::string, BasicTimeStats> eventLatencies_;
        PerformanceCounters counters_;

        // Memory tracking
        std::atomic<std::size_t> currentMemoryUsage_{0};
        std::atomic<std::size_t> peakMemoryUsage_{0};
        std::atomic<std::size_t> totalAllocations_{0};
        std::atomic<std::size_t> totalDeallocations_{0};

        // Auto-reporting
        TimeStamp lastReportTime_;
        std::atomic<bool> reportScheduled_{false};

        // =============================================================================
        // Internal Methods
        // =============================================================================

        /**
         * @brief Check and trigger auto-report
         */
        void checkAutoReport();

        /**
         * @brief Update frame statistics
         */
        void updateFrameStats(Duration frameDuration, Duration cpuTime, Duration gpuTime);

        /**
         * @brief Detect frame spike
         */
        bool detectSpike(Duration frameDuration) const;

        /**
         * @brief Calculate statistics for samples
         */
        [[nodiscard]] BasicTimeStats calculateStats(
                const std::vector<PerformanceSample>& samples
                ) const;
    };

} // namespace engine::time
