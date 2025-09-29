/**
 * @file TimeStats.h
 * @brief Statistics structures and performance metrics for time systems
 * @details Comprehensive data structures for frame timing statistics, performance
 *          counters, historical data tracking, and profiling metrics. Designed for
 *          zero-overhead collection in release builds with detailed analytics in debug.
 *
 * @author Andres Guerrero
 * @date Created on 2025-09-19
 */

#pragma once

#include "../core/TimeTypes.h"
#include "../core/TimelineType.h"

#include <atomic>
#include <array>
#include <vector>
#include <algorithm>
#include <numeric>
#include <mutex>
#include <unordered_map>

namespace engine::time {
    enum class TimelineType : std::uint8_t;
    // =============================================================================
    // Basic Statistics
    // =============================================================================

    /**
     * @brief Core statistical values for time measurements
     * @details Lock-free structure for high-frequency updates.
     *          Uses atomics to ensure thread-safe access without mutex overhead.
     */
    struct BasicTimeStats {
        std::atomic<Duration> min{Duration::max()}; ///< Minimum value
        std::atomic<Duration> max{Duration::min()}; ///< Maximum value
        std::atomic<Duration> sum{Duration::zero()}; ///< Sum for average
        std::atomic<Duration> lastValue{Duration::zero()}; ///< Most recent value
        std::atomic<std::uint64_t> sampleCount{0}; ///< Number of samples
        std::atomic<TimeStamp> firstSampleTime{TimeStamp{}}; ///< First sample timestamp
        std::atomic<TimeStamp> lastSampleTime{TimeStamp{}}; ///< Last sample timestamp

        BasicTimeStats() = default;

        // Copy
        BasicTimeStats(const BasicTimeStats& o) noexcept {
            min.store(o.min.load(std::memory_order_relaxed), std::memory_order_relaxed);
            max.store(o.max.load(std::memory_order_relaxed), std::memory_order_relaxed);
            sum.store(o.sum.load(std::memory_order_relaxed), std::memory_order_relaxed);
            lastValue.store(o.lastValue.load(std::memory_order_relaxed), std::memory_order_relaxed);
            sampleCount.store(o.sampleCount.load(std::memory_order_relaxed), std::memory_order_relaxed);
            firstSampleTime.store(o.firstSampleTime.load(std::memory_order_relaxed), std::memory_order_relaxed);
            lastSampleTime.store(o.lastSampleTime.load(std::memory_order_relaxed), std::memory_order_relaxed);
        }

        BasicTimeStats& operator=(const BasicTimeStats& o) noexcept {
            if (this == &o) return *this;
            min.store(o.min.load(std::memory_order_relaxed), std::memory_order_relaxed);
            max.store(o.max.load(std::memory_order_relaxed), std::memory_order_relaxed);
            sum.store(o.sum.load(std::memory_order_relaxed), std::memory_order_relaxed);
            lastValue.store(o.lastValue.load(std::memory_order_relaxed), std::memory_order_relaxed);
            sampleCount.store(o.sampleCount.load(std::memory_order_relaxed), std::memory_order_relaxed);
            firstSampleTime.store(o.firstSampleTime.load(std::memory_order_relaxed), std::memory_order_relaxed);
            lastSampleTime.store(o.lastSampleTime.load(std::memory_order_relaxed), std::memory_order_relaxed);
            return *this;
        }

        // Move — implementamos como copia de snapshot (seguro y sencillo)
        BasicTimeStats(BasicTimeStats&& o) noexcept { *this = o; }
        // BasicTimeStats& operator=(BasicTimeStats&& o) noexcept { return operator=(o); }

        /**
         * @brief Update statistics with new sample
         * @param value New duration value
         * @param timestamp Sample timestamp
         */
        void update(Duration value, const TimeStamp timestamp = Clock::now()) noexcept {
            // Update min/max using compare-exchange
            Duration currentMin = min.load(std::memory_order_relaxed);
            while (value < currentMin &&
                !min.compare_exchange_weak(currentMin, value,
                                           std::memory_order_release,
                                           std::memory_order_relaxed)) {
            }

            Duration currentMax = max.load(std::memory_order_relaxed);
            while (value > currentMax &&
                !max.compare_exchange_weak(currentMax, value,
                                           std::memory_order_release,
                                           std::memory_order_relaxed)) {
            }

            // Accumulate sum and count
            // sum.fetch_add(value, std::memory_order_relaxed);
            // TODO: Temporal, ya que std::atomic<Duration> da muchos problemas
            // Problema: Puede crear "thundering herd" - todos los hilos compiten por la misma ubicación de memoria.
            Duration currentSum = sum.load(std::memory_order_relaxed);
            Duration newSum;
            do {
                newSum = currentSum + value;
            }
            while (!sum.compare_exchange_weak(currentSum, newSum,
                                              //< Intenta actualizar el valor de sum (temas de thread safe)
                                              std::memory_order_release,
                                              std::memory_order_relaxed));

            // Update timestamps
            if (const auto count = sampleCount.fetch_add(1, std::memory_order_relaxed); count == 0) {
                firstSampleTime.store(timestamp, std::memory_order_relaxed);
            }

            lastSampleTime.store(timestamp, std::memory_order_relaxed);
            lastValue.store(value, std::memory_order_relaxed);
        }

        /**
         * @brief Calculate average duration
         * @return Average or zero if no samples
         */
        [[nodiscard]] Duration getAverage() const noexcept {
            const auto count = sampleCount.load(std::memory_order_acquire);
            if (count == 0) return Duration::zero();

            const auto total = sum.load(std::memory_order_acquire);
            return Duration(total.count() / count);
        }

        /**
         * @brief Reset all statistics
         */
        void reset() noexcept {
            min.store(Duration::max(), std::memory_order_relaxed);
            max.store(Duration::min(), std::memory_order_relaxed);
            sum.store(Duration::zero(), std::memory_order_relaxed);
            lastValue.store(Duration::zero(), std::memory_order_relaxed);
            sampleCount.store(0, std::memory_order_relaxed);
            firstSampleTime.store(TimeStamp{}, std::memory_order_relaxed);
            lastSampleTime.store(TimeStamp{}, std::memory_order_relaxed);
        }
    };

    // =============================================================================
    // Frame Timing Statistics
    // =============================================================================

    /**
     * @brief Comprehensive frame timing statistics
     * @details Tracks frame-level timing with multiple metrics for analysis.
     *          Optimized for cache-friendly access patterns in hot paths.
     */
    struct FrameStats {
        // Basic frame metrics
        BasicTimeStats frameDuration; ///< Total frame time
        BasicTimeStats cpuTime; ///< CPU processing time
        BasicTimeStats gpuTime; ///< GPU rendering time
        BasicTimeStats presentTime; ///< Present/swap time

        // Frame rate metrics
        std::atomic<float> instantFPS{0.0f}; ///< Current FPS
        std::atomic<float> averageFPS{0.0f}; ///< Average FPS
        std::atomic<float> percentile95FPS{0.0f}; ///< 95th percentile FPS
        std::atomic<float> percentile99FPS{0.0f}; ///< 99th percentile FPS

        // Frame consistency metrics
        std::atomic<Duration> frameTimeVariance{Duration::zero()}; ///< Variance
        std::atomic<Duration> frameTimeStdDev{Duration::zero()}; ///< Std deviation
        std::atomic<float> frameTimeCV{0.0f}; ///< Coefficient of variation

        // Frame drop tracking
        std::atomic<std::uint32_t> droppedFrames{0}; ///< Total dropped frames
        std::atomic<std::uint32_t> consecutiveDrops{0}; ///< Current drop streak
        std::atomic<std::uint32_t> maxConsecutiveDrops{0}; ///< Worst drop streak
        std::atomic<float> dropRate{0.0f}; ///< Drop percentage

        // VSync and tearing
        std::atomic<std::uint32_t> vsyncMisses{0}; ///< Missed vsyncs
        std::atomic<std::uint32_t> tearingEvents{0}; ///< Screen tears detected
        std::atomic<bool> vsyncEnabled{false}; ///< VSync state

        /**
         * @brief Update frame statistics
         * @param frameDur Total frame duration
         * @param cpuDur CPU processing duration
         * @param gpuDur GPU rendering duration
         */
        void updateFrame(const Duration frameDur, const Duration cpuDur = Duration::zero(),
                         const Duration gpuDur = Duration::zero()) noexcept {
            const auto now = Clock::now();

            // Update basic stats
            frameDuration.update(frameDur, now);

            if (cpuDur != Duration::zero()) {
                cpuTime.update(cpuDur, now);
            }

            if (gpuDur != Duration::zero()) {
                gpuTime.update(gpuDur, now);
            }

            // Calculate instant FPS
            if (frameDur.count() > 0) {
                instantFPS.store(1000000.0f / static_cast<float>(frameDur.count()), std::memory_order_relaxed);
            }

            // Update average FPS
            if (const auto avgFrame = frameDuration.getAverage(); avgFrame.count() > 0) {
                averageFPS.store(1000000.0f / static_cast<float>(avgFrame.count()), std::memory_order_relaxed);
            }
        }

        /**
         * @brief Record a dropped frame
         * @param targetFrameTime Expected frame duration
         */
        // TODO: Revisar esto
        void recordFrameDrop(Duration targetFrameTime) noexcept {
            droppedFrames.fetch_add(1, std::memory_order_relaxed);
            const auto drops = consecutiveDrops.fetch_add(1, std::memory_order_relaxed) + 1;

            // Update max consecutive drops
            auto maxDrops = maxConsecutiveDrops.load(std::memory_order_relaxed);
            while (drops > maxDrops &&
                !maxConsecutiveDrops.compare_exchange_weak(maxDrops, drops,
                                                           std::memory_order_release,
                                                           std::memory_order_relaxed)) {
            }
        }

        /**
         * @brief Reset consecutive drop counter
         */
        void resetDropStreak() noexcept {
            consecutiveDrops.store(0, std::memory_order_relaxed);
        }

        /**
 * @brief Reset all frame statistics to initial state
 * @details Resets all atomic counters and nested BasicTimeStats.
 *          Thread-safe operation that can be called from any thread.
 */
        void reset() noexcept {
            // Reset basic frame metrics
            frameDuration.reset();
            cpuTime.reset();
            gpuTime.reset();
            presentTime.reset();

            // Reset frame rate metrics
            instantFPS.store(0.0f, std::memory_order_relaxed);
            averageFPS.store(0.0f, std::memory_order_relaxed);
            percentile95FPS.store(0.0f, std::memory_order_relaxed);
            percentile99FPS.store(0.0f, std::memory_order_relaxed);

            // Reset consistency metrics
            frameTimeVariance.store(Duration::zero(), std::memory_order_relaxed);
            frameTimeStdDev.store(Duration::zero(), std::memory_order_relaxed);
            frameTimeCV.store(0.0f, std::memory_order_relaxed);

            // Reset frame drop tracking
            droppedFrames.store(0, std::memory_order_relaxed);
            consecutiveDrops.store(0, std::memory_order_relaxed);
            maxConsecutiveDrops.store(0, std::memory_order_relaxed);
            dropRate.store(0.0f, std::memory_order_relaxed);

            // Reset VSync metrics
            vsyncMisses.store(0, std::memory_order_relaxed);
            tearingEvents.store(0, std::memory_order_relaxed);
            vsyncEnabled.store(false, std::memory_order_relaxed);
        }
    };

    // =============================================================================
    // Timeline Statistics
    // =============================================================================

    /**
     * @brief Per-timeline performance statistics
     * @details Tracks timeline-specific metrics for optimization.
     *          Helps identify which timelines consume the most resources.
     */
    struct TimelineStats {
        TimelineID id{constants::INVALID_TIMELINE_ID}; ///< Timeline identifier
        TimelineType type{TimelineType::INVALID}; ///< Timeline type

        // Update metrics
        BasicTimeStats updateDuration; ///< Update cycle time
        std::atomic<std::uint64_t> updateCount{0}; ///< Total updates
        std::atomic<std::uint64_t> skippedUpdates{0}; ///< Skipped due to budget

        // Timer metrics
        std::atomic<std::uint32_t> activeTimers{0}; ///< Current active timers
        std::atomic<std::uint32_t> totalTimersCreated{0}; ///< Lifetime timer count
        std::atomic<std::uint32_t> timersFired{0}; ///< Timers executed
        BasicTimeStats timerProcessingTime; ///< Timer update overhead

        // State tracking
        std::atomic<bool> isPaused{false}; ///< Current pause state
        std::atomic<TimeScale> currentScale{1.0}; ///< Current time scale
        std::atomic<Duration> totalPausedTime{Duration::zero()}; ///< Total pause duration
        std::atomic<Duration> totalScaledTime{Duration::zero()}; ///< Time under scaling

        // Memory usage
        std::atomic<std::size_t> memoryUsed{0}; ///< Current memory usage
        std::atomic<std::size_t> peakMemoryUsed{0}; ///< Peak memory usage

        TimelineStats() = default;

        // Constructor de copia
        TimelineStats(const TimelineStats& other) noexcept
            : id(other.id)
            , type(other.type)
            , updateDuration(other.updateDuration)
            , updateCount(other.updateCount.load())
            , skippedUpdates(other.skippedUpdates.load())
            , activeTimers(other.activeTimers.load())
            , totalTimersCreated(other.totalTimersCreated.load())
            , timersFired(other.timersFired.load())
            , timerProcessingTime(other.timerProcessingTime)
            , isPaused(other.isPaused.load())
            , currentScale(other.currentScale.load())
            , totalPausedTime(other.totalPausedTime.load())
            , totalScaledTime(other.totalScaledTime.load())
            , memoryUsed(other.memoryUsed.load())
            , peakMemoryUsed(other.peakMemoryUsed.load())
        {}

        /**
         * @brief Update timeline statistics
         * @param duration Update duration
         * @param timersProcessed Number of timers processed
         */
        void updateTimeline(const Duration duration, const std::uint32_t timersProcessed = 0) noexcept {
            updateDuration.update(duration);
            updateCount.fetch_add(1, std::memory_order_relaxed);

            if (timersProcessed > 0) {
                timersFired.fetch_add(timersProcessed, std::memory_order_relaxed);
            }
        }

        TimelineStats& operator=(const TimelineStats& other) noexcept {
            if (this == &other) return *this;

            // Copiar valores no atómicos
            id = other.id;
            type = other.type;
            updateDuration = other.updateDuration;
            timerProcessingTime = other.timerProcessingTime;

            // Copiar valores atómicos uno por uno
            updateCount.store(other.updateCount.load());
            skippedUpdates.store(other.skippedUpdates.load());
            activeTimers.store(other.activeTimers.load());
            totalTimersCreated.store(other.totalTimersCreated.load());
            timersFired.store(other.timersFired.load());
            isPaused.store(other.isPaused.load());
            currentScale.store(other.currentScale.load());
            totalPausedTime.store(other.totalPausedTime.load());
            totalScaledTime.store(other.totalScaledTime.load());
            memoryUsed.store(other.memoryUsed.load());
            peakMemoryUsed.store(other.peakMemoryUsed.load());

            return *this;
        }
    };

    // =============================================================================
    // Historical Data Structures
    // =============================================================================

    /**
     * @brief Circular buffer for historical frame data
     * @details Fixed-size buffer for efficient frame history tracking.
     *          Uses ring buffer pattern to avoid allocations during runtime.
     */
    template <std::size_t Capacity>
    class FrameHistory {
    public:
        /**
         * @brief Single frame record
         */
        struct FrameRecord {
            FrameNumber frameNumber{0}; ///< Frame identifier
            TimeStamp timestamp; ///< Frame timestamp
            Duration frameDuration{}; ///< Total frame time
            Duration cpuDuration{}; ///< CPU time
            Duration gpuDuration{}; ///< GPU time
            float instantFPS{0.0f}; ///< Frame FPS
            bool wasDropped{false}; ///< Frame drop flag
            std::uint8_t qualityLevel{0}; ///< Quality setting
        };

        /**
         * @brief Add frame to history
         * @param record Frame data to store
         */
        void addFrame(const FrameRecord& record) noexcept {
            std::lock_guard lock(mutex_);

            frames_[writeIndex_] = record;
            writeIndex_ = (writeIndex_ + 1) % Capacity;

            if (size_ < Capacity) {
                ++size_;
            }
            else {
                readIndex_ = (readIndex_ + 1) % Capacity;
            }
        }

        /**
         * @brief Get frame at relative offset
         * @param offset Offset from newest (0 = newest)
         * @return Frame record or nullopt if out of range
         */
        [[nodiscard]] std::optional<FrameRecord> getFrame(const std::size_t offset) const noexcept {
            std::lock_guard lock(mutex_);

            if (offset >= size_) {
                return std::nullopt;
            }

            const std::size_t index = (writeIndex_ + Capacity - 1 - offset) % Capacity;
            return frames_[index];
        }

        /**
         * @brief Calculate statistics over window
         * @param windowSize Number of recent frames to analyze
         * @return Statistics for the window
         */
        [[nodiscard]] BasicTimeStats calculateWindowStats(const std::size_t windowSize) const noexcept {
            std::lock_guard lock(mutex_);

            BasicTimeStats stats;
            const std::size_t count = std::min(windowSize, size_);

            for (std::size_t i = 0; i < count; ++i) {
                const std::size_t index = (writeIndex_ + Capacity - 1 - i) % Capacity;
                stats.update(frames_[index].frameDuration, frames_[index].timestamp);
            }

            return stats;
        }

        /**
         * @brief Get percentile frame time
         * @param percentile Percentile to calculate (0-100)
         * @return Frame time at percentile
         */
        [[nodiscard]] Duration getPercentile(const float percentile) const noexcept {
            std::lock_guard lock(mutex_);

            if (size_ == 0) return Duration::zero();

            // Copy durations for sorting
            std::vector<Duration> durations;
            durations.reserve(size_);

            for (std::size_t i = 0; i < size_; ++i) {
                const std::size_t index = (readIndex_ + i) % Capacity;
                durations.push_back(frames_[index].frameDuration);
            }

            std::ranges::sort(durations);

            const auto percentileIndex =
                static_cast<std::size_t>(percentile * static_cast<float>(durations.size()) / 100.0f);

            return durations[std::min(percentileIndex, durations.size() - 1)];
        }

        /**
         * @brief Clear history
         */
        void clear() noexcept {
            std::lock_guard lock(mutex_);
            readIndex_ = 0;
            writeIndex_ = 0;
            size_ = 0;
        }

        /**
         * @brief Get current history size
         */
        [[nodiscard]] std::size_t size() const noexcept {
            std::lock_guard lock(mutex_);
            return size_;
        }

    private:
        mutable std::mutex mutex_; ///< Thread safety
        std::array<FrameRecord, Capacity> frames_{}; ///< Frame storage
        std::size_t readIndex_{0}; ///< Read position
        std::size_t writeIndex_{0}; ///< Write position
        std::size_t size_{0}; ///< Current size
    };

    // =============================================================================
    // Performance Counters
    // =============================================================================

    /**
     * @brief High-level performance counters
     * @details Aggregated metrics for system-wide performance monitoring.
     *          Designed for minimal overhead with maximum insight.
     */
    struct PerformanceCounters {
        // System-wide metrics
        std::atomic<std::uint64_t> totalFrames{0}; ///< Total frames rendered
        std::atomic<Duration> totalRuntime{Duration::zero()}; ///< Total runtime
        std::atomic<Duration> totalActiveTime{Duration::zero()}; ///< Non-paused time

        // Update cycle metrics
        std::atomic<std::uint64_t> totalUpdates{0}; ///< Total update cycles
        std::atomic<std::uint64_t> fixedUpdates{0}; ///< Fixed timestep updates
        std::atomic<std::uint64_t> variableUpdates{0}; ///< Variable updates
        std::atomic<std::uint64_t> interpolationUpdates{0}; ///< Interpolation passes

        // Timer metrics
        std::atomic<std::uint64_t> totalTimersCreated{0}; ///< Lifetime timer count
        std::atomic<std::uint64_t> totalTimersFired{0}; ///< Timers executed
        std::atomic<std::uint64_t> totalTimersRecycled{0}; ///< Recycled handles

        // Memory metrics
        std::atomic<std::size_t> currentMemoryUsage{0}; ///< Current memory
        std::atomic<std::size_t> peakMemoryUsage{0}; ///< Peak memory
        std::atomic<std::uint64_t> totalAllocations{0}; ///< Allocation count
        std::atomic<std::uint64_t> totalDeallocations{0}; ///< Deallocation count

        // Error and warning counts
        std::atomic<std::uint32_t> budgetExceeded{0}; ///< Budget violations
        std::atomic<std::uint32_t> spiralOfDeathEvents{0}; ///< Update spirals
        std::atomic<std::uint32_t> timelineStalls{0}; ///< Timeline stalls
        std::atomic<std::uint32_t> timerOverflows{0}; ///< Timer queue overflows

        /**
         * @brief Reset all counters
         */
        void reset() noexcept {
            totalFrames.store(0);
            totalRuntime.store(Duration::zero());
            totalActiveTime.store(Duration::zero());
            totalUpdates.store(0);
            fixedUpdates.store(0);
            variableUpdates.store(0);
            interpolationUpdates.store(0);
            totalTimersCreated.store(0);
            totalTimersFired.store(0);
            totalTimersRecycled.store(0);
            currentMemoryUsage.store(0);
            peakMemoryUsage.store(0);
            totalAllocations.store(0);
            totalDeallocations.store(0);
            budgetExceeded.store(0);
            spiralOfDeathEvents.store(0);
            timelineStalls.store(0);
            timerOverflows.store(0);
        }

        /**
         * @brief Update peak memory if current exceeds it
         * @param current Current memory usage
         */
        void updatePeakMemory(const std::size_t current) noexcept {
            std::size_t peak = peakMemoryUsage.load(std::memory_order_relaxed);
            while (current > peak &&
                !peakMemoryUsage.compare_exchange_weak(peak, current,
                                                       std::memory_order_release,
                                                       std::memory_order_relaxed)) {
            }
        }
    };

    // =============================================================================
    // Benchmark Data Holder
    // =============================================================================

    /**
     * @brief Benchmark results container
     * @details Stores results from performance benchmarks and tests.
     *          Used for regression testing and optimization validation.
     */
    struct BenchmarkData {
        struct BenchmarkRun {
            std::string name; ///< Benchmark name
            TimeStamp startTime; ///< Run start time
            TimeStamp endTime; ///< Run end time
            Duration totalDuration; ///< Total duration
            std::uint64_t iterations{0}; ///< Number of iterations
            Duration perIterationTime; ///< Time per iteration
            BasicTimeStats iterationStats; ///< Per-iteration stats
            std::unordered_map<std::string, double> customMetrics; ///< Custom data
        };

        std::vector<BenchmarkRun> runs; ///< All benchmark runs
        std::string platformInfo; ///< Platform description
        std::string buildConfiguration; ///< Build settings
        TimeStamp benchmarkDate; ///< When run

        /**
         * @brief Add benchmark run
         * @param run Benchmark results
         */
        void addRun(BenchmarkRun run) {
            runs.push_back(std::move(run));
        }

        /**
         * @brief Compare with baseline
         * @param baseline Previous benchmark data
         * @return Performance delta percentage
         */
        [[nodiscard]] float compareWithBaseline(const BenchmarkData& baseline) const {
            if (runs.empty() || baseline.runs.empty()) {
                return 0.0f;
            }

            // Simple comparison of average per-iteration times
            Duration avgCurrent{0};
            Duration avgBaseline{0};

            for (const auto& run : runs) {
                avgCurrent += run.perIterationTime;
            }
            for (const auto& run : baseline.runs) {
                avgBaseline += run.perIterationTime;
            }

            avgCurrent = Duration(avgCurrent.count() / runs.size());
            avgBaseline = Duration(avgBaseline.count() / baseline.runs.size());

            if (avgBaseline.count() == 0) return 0.0f;

            const float delta = static_cast<float>(avgCurrent.count() - avgBaseline.count()) /
                static_cast<float>(avgBaseline.count());

            return delta * 100.0f; // Percentage change
        }
    };
} // namespace engine::time
