/**
 * @file TimerSystem.h
 * @brief Timer system coordination and public API
 * @details Central registry for timer management providing the public API for
 *          timer creation, registration with timelines, batch operations, and
 *          cleanup. Serves as the integration point between timers and timelines.
 *
 * @author Andres Guerrero
 * @date Created on 2025-09-19
 */

#pragma once

#include <ranges>

#include "Timer.h"
#include "TimerPool.h"
#include "TimerQueue.h"
#include "../core/Timeline.h"
#include "../timers/TimerHandle.h"
#include <unordered_map>
#include <shared_mutex>
#include <unordered_set>

namespace engine::time {

    /**
     * @brief Central timer management system
     * @details Provides unified API for timer creation, management, and lifecycle.
     *          Coordinates between timer pools, queues, and timelines while ensuring
     *          efficient memory usage and proper cleanup.
     *
     * Key responsibilities:
     * - Timer creation and destruction
     * - Timeline registration and binding
     * - Handle validation and generation
     * - Batch timer operations
     * - Memory management via pools
     * - Statistics and monitoring
     *
     * Thread-safety:
     * - Thread-safe for all public operations
     * - Multiple readers, single writer for registry
     * - Lock-free handle validation
     */
    class TimerSystem {
    public:
        /**
         * @brief Registry configuration
         */
        struct Config {
            std::size_t maxTimers{constants::MAX_TIMERS}; ///< Maximum timers
            std::size_t poolInitialSize{256}; ///< Initial pool size
            std::size_t poolGrowthSize{128}; ///< Pool growth size
            bool enablePooling{true}; ///< Use object pooling
            bool enableBatching{true}; ///< Batch operations
            std::size_t batchSize{64}; ///< Batch size
            bool validateHandles{true}; ///< Validate handles
            bool trackStatistics{true}; ///< Track stats
        };

        explicit TimerSystem(memory::MemoryManager& memoryManager) :
            TimerSystem(memoryManager, Config{}) {}

        /**
         * @brief Constructor
         * @param memoryManager Engine memory manager
         * @param config Registry configuration
         */
        explicit TimerSystem(
                memory::MemoryManager& memoryManager,
                const Config& config
                ) :
            memoryManager_(memoryManager)
            , config_(config)
            , nextTimerId_(1) // Start at 1, 0 is invalid
            , isInitialized_(false) {}

        /**
         * @brief Destructor
         */
        ~TimerSystem();

        // Delete copy operations
        TimerSystem(const TimerSystem&) = delete;
        TimerSystem& operator=(const TimerSystem&) = delete;

        // =============================================================================
        // Initialization and Lifecycle
        // =============================================================================

        /**
         * @brief Initialize timer registry
         * @return True if initialization successful
         */
        bool initialize();

        /**
         * @brief Shutdown timer registry
         */
        void shutdown();

        // =============================================================================
        // Timer Creation and Management
        // =============================================================================

        /**
         * @brief Create a new timer
         * @param config Timer configuration
         * @param timelineId Timeline to bind to
         * @return Timer handle or invalid handle on failure
         */
        [[nodiscard]] SafeTimerHandle createTimer(
                const Timer::Config& config,
                TimelineID timelineId
                );

        /**
         * @brief Create one-shot timer
         * @param duration Timer duration
         * @param callback Timer callback
         * @param timelineId Timeline to bind to
         * @return Timer handle
         */
        [[nodiscard]] SafeTimerHandle createOneShotTimer(
                Duration duration,
                TimerCallback callback,
                TimelineID timelineId
                );

        /**
         * @brief Create recurring timer
         * @param interval Timer interval
         * @param callback Timer callback
         * @param timelineId Timeline to bind to
         * @return Timer handle
         */
        [[nodiscard]] SafeTimerHandle createRecurringTimer(
                Duration interval,
                TimerCallback callback,
                TimelineID timelineId
                );

        /**
         * @brief Cancel timer
         * @param handle Timer handle
         * @return True if timer was cancelled
         */
        bool cancelTimer(const SafeTimerHandle& handle);

        /**
         * @brief Pause timer
         * @param handle Timer handle
         * @return True if timer was paused
         */
        bool pauseTimer(const SafeTimerHandle& handle) const;

        /**
         * @brief Resume timer
         * @param handle Timer handle
         * @return True if timer was resumed
         */
        bool resumeTimer(const SafeTimerHandle& handle) const;

        /**
         * @brief Reset timer
         * @param handle Timer handle
         * @param restart If true, start immediately
         * @return True if timer was reset
         */
        bool resetTimer(const SafeTimerHandle& handle, bool restart = true) const;

        // =============================================================================
        // Batch Operations
        // =============================================================================

        /**
         * @brief Cancel all timers on a timeline
         * @param timelineId Timeline ID
         * @return Number of timers cancelled
         */
        std::size_t cancelTimelineTimers(TimelineID timelineId);

        /**
         * @brief Cancel all timers
         * @return Number of timers cancelled
         */
        std::size_t cancelAllTimers();

        /**
         * @brief Pause all timers on a timeline
         * @param timelineId Timeline ID
         * @return Number of timers paused
         */
        std::size_t pauseTimelineTimers(TimelineID timelineId);

        /**
         * @brief Resume all timers on a timeline
         * @param timelineId Timeline ID
         * @return Number of timers resumed
         */
        std::size_t resumeTimelineTimers(TimelineID timelineId);

        // =============================================================================
        // Handle Validation
        // =============================================================================

        /**
         * @brief Validate timer handle
         * @param handle Timer handle to validate
         * @return True if handle is valid and timer exists
         */
        [[nodiscard]] bool validateHandle(TimerHandle handle) const;

        // =============================================================================
        // Queries
        // =============================================================================

        /**
         * @brief Get timer count
         */
        [[nodiscard]] std::size_t getTimerCount() const;

        /**
         * @brief Get active timer count
         */
        [[nodiscard]] std::size_t getActiveTimerCount() const;

        /**
         * @brief Registry statistics
         */
        struct Stats {
            std::atomic<std::uint64_t> totalCreated{0}; ///< Total created
            std::atomic<std::uint64_t> totalCancelled{0}; ///< Total cancelled
            std::atomic<std::uint64_t> totalFired{0}; ///< Total fired
            std::atomic<std::size_t> currentActive{0}; ///< Currently active
            std::atomic<std::size_t> peakActive{0}; ///< Peak active count
        };

        /**
         * @brief Get registry statistics
         */
        [[nodiscard]] const Stats& getStats() const noexcept {
            return stats_;
        }

    private:
        friend class SafeTimerHandle;

        // Private types
        struct TimerRegistration {
            Timer* timer{};
            TimerHandle handle;
            TimelineID timelineId{};
            TimeStamp createdAt;
        };

        // Private members
        memory::MemoryManager& memoryManager_;
        Config config_;

        std::unique_ptr<TimerPool> timerPool_;
        std::unique_ptr<TimerHandlePool> handlePool_;

        mutable std::shared_mutex registryMutex_;
        std::unordered_map<TimerID, TimerRegistration> timers_;
        std::unordered_map<TimelineID, std::unordered_set<TimerID>> timersByTimeline_;

        std::atomic<TimerID> nextTimerId_;
        std::atomic<bool> isInitialized_;

        Stats stats_;

        // Private methods
        Timer* allocateTimer() const;

        void deallocateTimer(Timer* timer) const;

        Timer* getTimer(TimerHandle handle) const;

        void updatePeakActive();
    };

    // Implement SafeTimerHandle methods that require TimerSystem
    inline bool SafeTimerHandle::isAlive() const noexcept {
        return ownerSystem_ ? (ownerSystem_)->validateHandle(handle_) : false;
    }

    inline bool SafeTimerHandle::cancel() const noexcept {
        return ownerSystem_ ? (ownerSystem_)->cancelTimer(*this) : false;
    }

    inline bool SafeTimerHandle::pause() const noexcept {
        return ownerSystem_ ? (ownerSystem_)->pauseTimer(*this) : false;
    }

    inline bool SafeTimerHandle::resume() const noexcept {
        return ownerSystem_ ? (ownerSystem_)->resumeTimer(*this) : false;
    }

    inline bool SafeTimerHandle::reset(const bool restart) const noexcept {
        return ownerSystem_ ? ownerSystem_->resetTimer(*this, restart) : false;
    }

} // namespace engine::time
