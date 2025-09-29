/**
 * @file RealTimeline.h
 * @brief Always-advancing timeline for networking and profiling
 * @details Provides a timeline that never pauses or scales, maintaining perfect
 *          synchronization with wall clock time. Critical for networking, profiling,
 *          and any systems that require absolute time reference.
 *
 * @author Andres Guerrero
 * @date Created on 2025-09-19
 */

#pragma once

#include "../core/Timeline.h"
#include "../core/ITimeBackend.h"
#include "../profiling/TimeStats.h"

#include <queue>
#include <mutex>
#include <condition_variable>

namespace engine::time {
    /**
     * @brief Real-time timeline that always advances
     * @details This timeline is immune to game pause and time scaling, providing
     *          a reliable time source for systems that must continue regardless of
     *          game state. Used for networking, profiling, and frame rate measurement.
     *
     * Key characteristics:
     * - Never pauses (pause() has no effect)
     * - Never scales (always 1:1 with real time)
     * - Highest precision timing
     * - Thread-safe for multi-threaded access
     * - Minimal overhead for time queries
     *
     * Use cases:
     * - Network packet timestamps
     * - Performance profiling
     * - Frame rate calculation
     * - System monitoring
     * - Time synchronization
     */
    class RealTimeline final : public Timeline {
    public:
        /**
         * @brief Constructor with backend
         * @param backend Time backend for platform queries
         * @param name Timeline identifier
         */
        explicit RealTimeline(
                ITimeBackend& backend,
                const std::string& name = "RealTime"
                ) :
            Timeline(TimelineType::REAL_TIME, name)
            , backend_(backend)
            , frameCount_(0)
            , startTime_(TimeStamp{})
            , timerIdCounter_(1) {}

        /**
         * @brief Destructor
         */
        ~RealTimeline() override;

        // =============================================================================
        // Timeline Interface Implementation
        // =============================================================================

        /**
         * @brief Initialize real timeline
         * @param config Timeline configuration (most options ignored)
         * @return True if initialization successful
         */
        bool initialize(const TimelineConfigBase& config) override;

        /**
         * @brief Update real timeline
         * @param deltaTime Suggested delta (ignored, uses actual time)
         * @return Update result with actual time progression
         */
        TimelineUpdateResult update(Duration deltaTime) override;

        /**
         * @brief Pause has no effect on real timeline
         * @return Always returns false
         */
        bool pause() override {
            // Real timeline cannot be paused
            return false;
        }

        /**
         * @brief Resume has no effect (never paused)
         * @return Always returns true
         */
        bool resume() override {
            // Real timeline is never paused
            return true;
        }

        /**
         * @brief Set time scale has no effect
         * @param scale Ignored
         * @return Always returns false
         */
        bool setTimeScale(TimeScale scale) override {
            // Real timeline always runs at 1:1 scale
            return false;
        }

        /**
         * @brief Reset timeline (preserves real-time nature)
         */
        void reset() override;

        // =============================================================================
        // Timer Management
        // =============================================================================

        /**
         * @brief Create timer on real timeline
         * @param duration Timer duration
         * @param callback Timer callback
         * @param recurring True for repeating timer
         * @return Timer handle
         */
        SafeTimerHandle createTimer(
                Duration duration,
                TimerCallback callback,
                bool recurring
                ) override;

        /**
         * @brief Cancel timer
         * @param handle Timer handle
         * @return True if timer was cancelled
         */
        bool cancelTimer(const SafeTimerHandle& handle) override;

        /**
         * @brief Clear all timers
         */
        void clearAllTimers() override;

        /**
         * @brief Get active timer count
         */
        [[nodiscard]] std::size_t getActiveTimerCount() const override {
            std::lock_guard lock(timerMutex_);
            return timers_.size();
        }

        // =============================================================================
        // Configuration
        // =============================================================================

        [[nodiscard]] const TimelineConfigBase& getConfig() const override {
            return config_;
        }

        bool updateConfig(const TimelineConfigBase& config) override;

        [[nodiscard]] const TimestepConfig& getTimestepConfig() const override {
            return config_.timestepConfig;
        }

        bool setTimestepMode(const TimestepMode mode) override {
            // Real timeline always uses variable timestep
            return mode == TimestepMode::VARIABLE;
        }

        // =============================================================================
        // Real Timeline Specific Features
        // =============================================================================

        /**
         * @brief Get current frame rate
         * @return Instantaneous FPS
         */
        [[nodiscard]] float getFrameRate() const noexcept {
            return frameStats_.instantFPS.load(std::memory_order_acquire);
        }

        /**
         * @brief Get average frame rate
         * @return Average FPS over history
         */
        [[nodiscard]] float getAverageFrameRate() const noexcept {
            return frameStats_.averageFPS.load(std::memory_order_acquire);
        }

        /**
         * @brief Get frame statistics
         */
        [[nodiscard]] const FrameStats& getFrameStats() const noexcept {
            return frameStats_;
        }

        /**
         * @brief Get total frame count
         */
        [[nodiscard]] std::uint64_t getFrameCount() const noexcept {
            return frameCount_.load(std::memory_order_acquire);
        }

        /**
         * @brief Get time since timeline start
         */
        [[nodiscard]] Duration getTimeSinceStart() const noexcept {
            const TimeStamp now = backend_.now();
            return std::chrono::duration_cast<Duration>(now - startTime_);
        }

        /**
         * @brief Get high-precision timestamp
         * @return Current backend timestamp
         */
        [[nodiscard]] TimeStamp getPreciseTimestamp() const noexcept {
            return backend_.now();
        }

        /**
         * @brief Synchronize with external time source
         * @param externalTime External timestamp
         * @param latency Network/processing latency
         */
        void synchronizeWith(TimeStamp externalTime, Duration latency);

        /**
         * @brief Get synchronized time
         * @return Time adjusted for synchronization
         */
        [[nodiscard]] TimeStamp getSynchronizedTime() const noexcept {
            return backend_.now() + timeSyncOffset_;
        }

        /**
         * @brief Wait for specific time
         * @param targetTime Time to wait for
         * @param timeout Maximum wait duration
         * @return True if target time reached
         */
        bool waitForTime(Duration targetTime, Duration timeout) override;

    private:
        // =============================================================================
        // Private Types
        // =============================================================================

        struct RealTimer {
            TimerID id;
            TimeStamp fireTime;
            Duration duration;
            TimerCallback callback;
            bool recurring;
            bool cancelled{false};
            TimerGeneration generation;
        };

        struct TimerQueueEntry {
            TimeStamp fireTime;
            TimerID id{};

            bool operator>(const TimerQueueEntry& other) const {
                return fireTime > other.fireTime;
            }
        };

        // =============================================================================
        // Private Members
        // =============================================================================

        ITimeBackend& backend_; ///< Platform time source
        TimelineConfigBase config_; ///< Configuration

        std::atomic<std::uint64_t> frameCount_; ///< Total frames
        TimeStamp startTime_; ///< Timeline start time

        // Timer management
        mutable std::mutex timerMutex_; ///< Timer synchronization
        std::unordered_map<TimerID, RealTimer> timers_; ///< Active timers
        std::priority_queue<TimerQueueEntry,
                            std::vector<TimerQueueEntry>,
                            std::greater<>> timerQueue_; ///< Timer scheduling
        std::atomic<TimerID> timerIdCounter_; ///< Timer ID generator

        // Frame statistics
        FrameStats frameStats_; ///< Frame timing stats

        // Time synchronization
        Duration timeSyncOffset_{Duration::zero()}; ///< Sync offset
        TimeStamp lastSyncTime_; ///< Last sync timestamp

        // =============================================================================
        // Private Methods
        // =============================================================================

        /**
         * @brief Process expired timers
         * @param currentTime Current timestamp
         * @return Number of timers processed
         */
        std::uint32_t processTimers(TimeStamp currentTime);

        /**
         * @brief Update performance metrics
         * @param deltaTime Frame delta time
         */
        // TODO: revisar esto
        void updateMetrics(Duration deltaTime) {
            // Additional metric calculations can be added here
        }
    };
} // namespace engine::time
