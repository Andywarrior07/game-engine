/**
 * @file FrameProfiler.h
 * @brief Frame-level performance profiling and analysis
 * @details Provides comprehensive frame timing analysis including frame drops,
 *          VSync misses, GPU/CPU synchronization, and frame pacing. Optimized
 *          for minimal overhead with lock-free operations where possible.
 *
 * @author Development Team
 * @date Created on 2024-01-20
 */

#pragma once

#include "../core/TimeTypes.h"
#include "../profiling/TimeStats.h"
#include "../profiling/ProfilingConfig.h"
#include "../profiling/PerformanceTracker.h"
#include "../profiling/SystemProfiler.h"
#include "../utils/ScopedTimer.h"

#include "../../memory/manager/MemoryManager.h"
#include "../../memory/allocators/StackAllocator.h"

#include <atomic>
#include <mutex>
#include <memory>
#include <array>
#include <functional>
#include <optional>

namespace engine::time {

    // =============================================================================
    // Frame Profile Data
    // =============================================================================

    /**
     * @brief Detailed profile data for a single frame
     * @details Cache-friendly structure containing all timing data for one frame.
     *          Aligned for optimal memory access patterns.
     */
    struct alignas(64) FrameProfileData {
        FrameNumber frameNumber{0}; ///< Frame identifier
        TimeStamp startTime; ///< Frame start timestamp
        TimeStamp endTime; ///< Frame end timestamp

        // Core timings
        Duration totalDuration{Duration::zero()}; ///< Total frame time
        Duration cpuDuration{Duration::zero()}; ///< CPU processing time
        Duration gpuDuration{Duration::zero()}; ///< GPU rendering time
        Duration presentDuration{Duration::zero()}; ///< Present/swap time

        // Frame sections
        Duration updateDuration{Duration::zero()}; ///< Game update time
        Duration renderDuration{Duration::zero()}; ///< Render submission time
        Duration physicsDuration{Duration::zero()}; ///< Physics simulation time
        Duration audioDuration{Duration::zero()}; ///< Audio processing time

        // Synchronization
        Duration waitForGPU{Duration::zero()}; ///< GPU wait time
        Duration waitForVSync{Duration::zero()}; ///< VSync wait time

        // Flags
        bool droppedFrame{false}; ///< Frame was dropped
        bool missedVSync{false}; ///< Missed VSync deadline
        bool budgetExceeded{false}; ///< Exceeded frame budget

        // Memory snapshot
        std::size_t memoryUsed{0}; ///< Memory usage at frame end
        std::size_t allocations{0}; ///< Allocations this frame

        /**
         * @brief Check if frame met target time
         * @param targetTime Target frame duration
         */
        [[nodiscard]] bool metTarget(const Duration targetTime) const noexcept {
            return totalDuration <= targetTime;
        }

        /**
         * @brief Calculate CPU utilization percentage
         */
        [[nodiscard]] float getCPUUtilization() const noexcept {
            if (totalDuration == Duration::zero())
                return 0.0f;
            return (static_cast<float>(cpuDuration.count()) /
                static_cast<float>(totalDuration.count())) * 100.0f;
        }

        /**
         * @brief Calculate GPU utilization percentage
         */
        [[nodiscard]] float getGPUUtilization() const noexcept {
            if (totalDuration == Duration::zero())
                return 0.0f;
            return (static_cast<float>(gpuDuration.count()) /
                static_cast<float>(totalDuration.count())) * 100.0f;
        }
    };

    // =============================================================================
    // Frame Profiler Configuration
    // =============================================================================

    /**
     * @brief Configuration for frame profiler
     * @details Controls profiling behavior and analysis features.
     */
    struct FrameProfilerConfig {
        // Target settings
        Duration targetFrameTime{constants::TARGET_FRAME_TIME_60FPS}; ///< Target frame time
        float vsyncToleranceMs{0.5f}; ///< VSync tolerance in ms

        // History settings
        std::size_t historySize{300}; ///< Frame history (5s @ 60fps)
        std::size_t detailedHistorySize{60}; ///< Detailed history (1s @ 60fps)

        // Analysis features
        bool trackSections{true}; ///< Track frame sections
        bool detectSpikes{true}; ///< Detect frame spikes
        bool predictFrameDrops{true}; ///< Predict future drops
        bool trackMemory{true}; ///< Track memory usage

        // Thresholds
        float spikeThresholdMultiplier{2.0f}; ///< Spike detection threshold
        float warningThresholdPercent{80.0f}; ///< Warning at 80% budget
        float criticalThresholdPercent{95.0f}; ///< Critical at 95% budget
        std::uint32_t consecutiveDropsForAlert{3}; ///< Alert after N drops

        // Callbacks
        std::function<void(const FrameProfileData&)> onFrameDrop;
        std::function<void(const FrameProfileData&)> onVSyncMiss;
        std::function<void(float fps)> onFPSUpdate;
        std::function<void(const std::string&)> onPerformanceAlert;
    };

    // =============================================================================
    // Frame Section Tracker
    // =============================================================================

    /**
     * @brief Tracks individual frame sections for detailed profiling
     * @details RAII helper for automatic section timing.
     */
    class FrameSectionTracker {
    public:
        enum class Section : std::uint8_t {
            UPDATE, PHYSICS, RENDER,
            AUDIO, PRESENT, CUSTOM
        };

        FrameSectionTracker(class FrameProfiler* profiler, Section section) noexcept;
        ~FrameSectionTracker() noexcept;

        FrameSectionTracker(const FrameSectionTracker&) = delete;
        FrameSectionTracker& operator=(const FrameSectionTracker&) = delete;
        FrameSectionTracker(FrameSectionTracker&& other) noexcept;
        FrameSectionTracker& operator=(FrameSectionTracker&& other) noexcept;

    private:
        class FrameProfiler* profiler_;
        Section section_;
        TimeStamp startTime_;
        bool active_;
    };

    // =============================================================================
    // Frame Profiler Class
    // =============================================================================

    /**
     * @brief High-performance frame-level profiler
     * @details Provides comprehensive frame timing analysis with minimal overhead.
     *          Uses lock-free ring buffers and atomic operations for hot paths.
     *          Integrates with system profiler and performance tracker.
     */
    class FrameProfiler {
    public:
        /**
         * @brief Constructor with memory manager and configuration
         * @param memoryManager Engine memory manager
         * @param config Profiler configuration
         */
        FrameProfiler(
                memory::MemoryManager& memoryManager,
                const FrameProfilerConfig& config = {}
                );

        /**
         * @brief Destructor
         */
        ~FrameProfiler();

        // Delete copy operations
        FrameProfiler(const FrameProfiler&) = delete;
        FrameProfiler& operator=(const FrameProfiler&) = delete;

        // =============================================================================
        // Initialization
        // =============================================================================

        /**
         * @brief Initialize frame profiler
         * @return True if initialization successful
         */
        bool initialize();

        /**
         * @brief Shutdown profiler
         */
        void shutdown();

        /**
         * @brief Check if initialized
         */
        [[nodiscard]] bool isInitialized() const noexcept {
            return initialized_.load(std::memory_order_acquire);
        }

        // =============================================================================
        // Frame Management
        // =============================================================================

        /**
         * @brief Begin new frame
         * @param frameNumber Frame identifier
         */
        void beginFrame(FrameNumber frameNumber);

        /**
         * @brief End current frame
         * @param gpuTime Optional GPU time if available
         */
        void endFrame(Duration gpuTime = Duration::zero());

        /**
         * @brief Mark frame as dropped
         * @param reason Optional reason for drop
         */
        void markFrameDropped(const std::string& reason = "");

        /**
         * @brief Mark VSync miss
         */
        void markVSyncMiss();

        // =============================================================================
        // Section Tracking
        // =============================================================================

        /**
         * @brief Begin frame section
         * @param section Section type
         * @return Section tracker for RAII timing
         */
        [[nodiscard]] FrameSectionTracker beginSection(FrameSectionTracker::Section section);

        /**
         * @brief Record section timing manually
         * @param section Section type
         * @param duration Section duration
         */
        void recordSection(FrameSectionTracker::Section section, Duration duration);

        // =============================================================================
        // GPU/CPU Synchronization
        // =============================================================================

        /**
         * @brief Record GPU wait time
         * @param duration Wait duration
         */
        void recordGPUWait(Duration duration);

        /**
         * @brief Record VSync wait time
         * @param duration Wait duration
         */
        void recordVSyncWait(Duration duration);

        /**
         * @brief Record present/swap time
         * @param duration Present duration
         */
        void recordPresentTime(Duration duration);

        // =============================================================================
        // Analysis and Queries
        // =============================================================================

        /**
         * @brief Get current frame data
         * @return Current frame profile or nullopt if not in frame
         */
        [[nodiscard]] std::optional<FrameProfileData> getCurrentFrame() const;

        /**
         * @brief Get frame history
         * @param count Number of frames to retrieve (0 = all)
         * @return Vector of frame profiles
         */
        [[nodiscard]] std::vector<FrameProfileData> getFrameHistory(std::size_t count = 0) const;

        /**
         * @brief Calculate current FPS
         * @param windowSize Number of frames to average
         * @return Average FPS
         */
        [[nodiscard]] float calculateFPS(std::size_t windowSize = 60) const;

        /**
         * @brief Calculate frame time percentiles
         * @param percentile Percentile to calculate (0-100)
         * @return Frame time at percentile
         */
        [[nodiscard]] Duration calculatePercentile(float percentile) const;

        /**
         * @brief Detect frame spikes in recent history
         * @return Vector of frame numbers with spikes
         */
        [[nodiscard]] std::vector<FrameNumber> detectSpikes() const;

        /**
         * @brief Predict likelihood of frame drop
         * @return Probability of drop (0.0 - 1.0)
         */
        [[nodiscard]] float predictFrameDropProbability() const;

        // =============================================================================
        // Statistics
        // =============================================================================

        /**
         * @brief Get frame statistics
         * @return Current frame stats
         */
        [[nodiscard]] FrameStats getFrameStats() const;

        /**
         * @brief Get detailed performance metrics
         * @return Performance metrics structure
         */
        // TODO: Revisar esto
        // [[nodiscard]] PerformanceMetrics getMetrics() const;

        // =============================================================================
        // Reporting
        // =============================================================================

        /**
         * @brief Generate frame profiling report
         * @param detailed Include detailed breakdown
         * @return Report string
         */
        [[nodiscard]] std::string generateReport(bool detailed = true) const;

        /**
         * @brief Generate frame pacing analysis
         * @return Analysis string
         */
        [[nodiscard]] std::string analyzeFramePacing() const;

        // =============================================================================
        // Integration
        // =============================================================================

        /**
         * @brief Set system profiler for integration
         * @param profiler System profiler instance
         */
        void setSystemProfiler(SystemProfiler* profiler) {
            systemProfiler_ = profiler;
        }

        /**
         * @brief Set performance tracker for integration
         * @param tracker Performance tracker instance
         */
        void setPerformanceTracker(PerformanceTracker* tracker) {
            performanceTracker_ = tracker;
        }

        // =============================================================================
        // Configuration
        // =============================================================================

        /**
         * @brief Update configuration
         * @param config New configuration
         */
        void updateConfig(const FrameProfilerConfig& config);

        /**
         * @brief Get configuration
         */
        [[nodiscard]] const FrameProfilerConfig& getConfig() const noexcept {
            return config_;
        }

        /**
         * @brief Reset profiling data
         */
        void reset();

        /**
         * @brief Enable/disable profiling
         */
        void setEnabled(const bool enabled) {
            enabled_.store(enabled, std::memory_order_release);
        }

        /**
         * @brief Check if profiling is enabled
         */
        [[nodiscard]] bool isEnabled() const noexcept {
            return enabled_.load(std::memory_order_acquire);
        }

    private:
        friend class FrameSectionTracker;

        // Configuration
        FrameProfilerConfig config_;
        std::atomic<bool> initialized_{false};
        std::atomic<bool> enabled_{true};

        // Memory management
        memory::MemoryManager& memoryManager_;
        std::unique_ptr<memory::StackAllocator> frameAllocator_;

        // Current frame tracking
        std::atomic<bool> inFrame_{false};
        std::atomic<FrameNumber> currentFrameNumber_{0};
        FrameProfileData currentFrame_;
        TimeStamp frameStartTime_;
        mutable std::mutex currentFrameMutex_;

        // Frame history (lock-free ring buffer)
        struct FrameRingBuffer {
            static constexpr std::size_t MAX_SIZE = 512;
            std::array<FrameProfileData, MAX_SIZE> data;
            std::atomic<std::size_t> head{0};
            std::atomic<std::size_t> tail{0};
            std::atomic<std::size_t> size{0};

            void push(const FrameProfileData& frame);
            bool pop(FrameProfileData& frame);
            void clear();
            std::vector<FrameProfileData> getRecent(std::size_t count) const;
        };

        FrameRingBuffer frameHistory_;

        // Statistics
        mutable std::mutex statsMutex_;
        FrameStats frameStats_;
        std::atomic<std::uint32_t> consecutiveDrops_{0};
        std::atomic<std::uint32_t> totalDrops_{0};
        std::atomic<std::uint32_t> totalVSyncMisses_{0};

        // Spike detection
        Duration averageFrameTime_{Duration::zero()};
        Duration spikeThreshold_{Duration::zero()};

        // Integration
        SystemProfiler* systemProfiler_{nullptr};
        PerformanceTracker* performanceTracker_{nullptr};

        // =============================================================================
        // Internal Methods
        // =============================================================================

        /**
         * @brief End section timing
         */
        void endSection(FrameSectionTracker::Section section, Duration duration);

        /**
         * @brief Process completed frame
         */
        void processFrame(const FrameProfileData& frame);

        /**
         * @brief Update statistics
         */
        void updateStats(const FrameProfileData& frame);

        /**
         * @brief Check frame budget
         */
        void checkBudget(const FrameProfileData& frame) const;

        /**
         * @brief Trigger callbacks
         */
        void triggerCallbacks(const FrameProfileData& frame) const;

        /**
         * @brief Update spike detection threshold
         */
        void updateSpikeThreshold();
    };

} // namespace engine::time
