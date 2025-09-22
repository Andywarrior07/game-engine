/**
 * @file Timeline.h
 * @brief Base timeline abstraction and interface
 * @details Pure virtual base class defining the contract for all timeline types.
 *          Provides common operations like pause, scale, and update while allowing
 *          specialized implementations for different timeline behaviors.
 *
 * @author Andres Guerrero
 * @date Created on 2025-09-19
 */

#pragma once

#include "TimeTypes.h"
#include "TimelineType.h"
#include "TimestepMode.h"
#include "TimeConfig.h"

#include "../timers/TimerHandle.h"

#include <memory>
#include <optional>

namespace engine::time {
    // Forward declarations
    class TimerSystem;
    class TimelineManager;

    // =============================================================================
    // Timeline State
    // =============================================================================

    /**
     * @brief Timeline execution state
     * @details Represents the current state of a timeline in its lifecycle.
     */
    enum class TimelineState : std::uint8_t {
        UNINITIALIZED, ///< Not yet initialized
        INITIALIZED, ///< Initialized but not started
        RUNNING, ///< Active and updating
        PAUSED, ///< Temporarily suspended
        STOPPED, ///< Stopped, can be restarted
        DESTROYED ///< Being destroyed
    };

    /**
     * @brief Timeline update result
     * @details Information returned after timeline update.
     */
    struct TimelineUpdateResult {
        Duration actualDeltaTime; ///< Actual time progressed
        Duration scaledDeltaTime; ///< After time scaling
        std::uint32_t fixedStepsExecuted{0}; ///< Fixed timesteps performed
        std::uint32_t timersProcessed{0}; ///< Timers that fired
        bool wasThrottled{false}; ///< Update was throttled
        bool budgetExceeded{false}; ///< Exceeded time budget
    };

    // =============================================================================
    // Timeline Base Class
    // =============================================================================

    /**
     * @brief Abstract base class for all timeline implementations
     * @details Defines the interface that all timeline types must implement.
     *          Provides virtual destructor and RAII compliance for proper cleanup.
     */
    class Timeline {
    public:
        /**
         * @brief Constructor with basic configuration
         * @param type Timeline type identifier
         * @param name Human-readable name
         */
        Timeline(const TimelineType type, const std::string& name) noexcept
            : type_(type)
              , name_(name)
              , id_(generateTimelineId())
              , state_(TimelineState::UNINITIALIZED)
              , isPaused_(false)
              , timeScale_(1.0)
              , currentTime_(Duration::zero())
              , totalElapsedTime_(Duration::zero())
              , totalPausedTime_(Duration::zero())
              , lastUpdateTime_(Clock::now()) {
        }

        /**
         * @brief Virtual destructor for proper cleanup
         * @details Ensures derived classes can clean up properly.
         *          Cancels all active timers before destruction.
         */
        virtual ~Timeline() {
            Timeline::destroy();
        }

        // Delete copy operations, allow move
        Timeline(const Timeline&) = delete;
        Timeline& operator=(const Timeline&) = delete;
        Timeline(Timeline&&) noexcept = default;
        Timeline& operator=(Timeline&&) noexcept = default;

        // =============================================================================
        // Initialization and Lifecycle
        // =============================================================================

        /**
         * @brief Initialize the timeline
         * @param config Timeline configuration
         * @return True if initialization successful
         */
        virtual bool initialize(const TimelineConfigBase& config) = 0;

        /**
         * @brief Destroy timeline and release resources
         * @details Called automatically in destructor.
         *          Can be called manually for explicit cleanup.
         */
        virtual void destroy() {
            if (state_ == TimelineState::DESTROYED) return;

            stop();
            clearAllTimers();
            state_ = TimelineState::DESTROYED;
        }

        /**
         * @brief Start the timeline
         * @return True if successfully started
         */
        virtual bool start() {
            if (state_ != TimelineState::INITIALIZED &&
                state_ != TimelineState::STOPPED) {
                return false;
            }

            state_ = TimelineState::RUNNING;
            isPaused_ = false;
            lastUpdateTime_ = Clock::now();
            onStart();
            return true;
        }

        /**
         * @brief Stop the timeline
         * @details Different from pause - resets timeline state
         */
        virtual void stop() {
            if (state_ == TimelineState::STOPPED ||
                state_ == TimelineState::DESTROYED) {
                return;
            }

            state_ = TimelineState::STOPPED;
            isPaused_ = false;
            onStop();
        }

        // =============================================================================
        // Core Timeline Operations
        // =============================================================================

        /**
         * @brief Update the timeline
         * @param deltaTime Time since last update
         * @return Update result information
         * @details Core update method that must be implemented by derived classes.
         *          Handles time progression, timer processing, and state updates.
         */
        virtual TimelineUpdateResult update(Duration deltaTime) = 0;

        /**
         * @brief Pause the timeline
         * @return True if timeline was paused
         */
        virtual bool pause() {
            if (!canPause() || isPaused_) {
                return false;
            }

            isPaused_ = true;
            pauseStartTime_ = Clock::now();
            onPause();
            return true;
        }

        /**
         * @brief Resume the timeline
         * @return True if timeline was resumed
         */
        virtual bool resume() {
            if (!isPaused_) {
                return false;
            }

            isPaused_ = false;
            if (pauseStartTime_) {
                const auto pauseDuration = Clock::now() - *pauseStartTime_;
                totalPausedTime_ += std::chrono::duration_cast<Duration>(pauseDuration);
                pauseStartTime_ = std::nullopt;
            }
            onResume();
            return true;
        }

        /**
         * @brief Set time scale
         * @param scale Time scale factor (1.0 = normal)
         * @return True if scale was set
         */
        virtual bool setTimeScale(const TimeScale scale) {
            if (!canScale() || scale < 0.0) {
                return false;
            }

            const TimeScale oldScale = timeScale_;
            timeScale_ = std::clamp(scale, constants::MIN_TIME_SCALE, constants::MAX_TIME_SCALE);

            if (oldScale != timeScale_) {
                onTimeScaleChanged(oldScale, timeScale_);
            }

            return true;
        }

        /**
         * @brief Reset timeline to initial state
         */
        virtual void reset() {
            currentTime_ = Duration::zero();
            totalElapsedTime_ = Duration::zero();
            totalPausedTime_ = Duration::zero();
            lastUpdateTime_ = Clock::now();
            clearAllTimers();
            onReset();
        }

        // =============================================================================
        // Timer Management
        // =============================================================================

        /**
         * @brief Create a timer on this timeline
         * @param duration Timer duration
         * @param callback Timer callback
         * @param recurring True for repeating timer
         * @return Timer handle
         */
        virtual SafeTimerHandle createTimer(
            Duration duration,
            TimerCallback callback,
            bool recurring = false) = 0;

        /**
         * @brief Cancel a timer
         * @param handle Timer handle to cancel
         * @return True if timer was cancelled
         */
        virtual bool cancelTimer(const SafeTimerHandle& handle) = 0;

        /**
         * @brief Clear all timers on this timeline
         */
        virtual void clearAllTimers() = 0;

        /**
         * @brief Get number of active timers
         * @return Active timer count
         */
        [[nodiscard]] virtual std::size_t getActiveTimerCount() const = 0;

        // =============================================================================
        // State Queries
        // =============================================================================

        /**
         * @brief Get timeline ID
         */
        [[nodiscard]] TimelineID getId() const noexcept { return id_; }

        /**
         * @brief Get timeline type
         */
        [[nodiscard]] TimelineType getType() const noexcept { return type_; }

        /**
         * @brief Get timeline name
         */
        [[nodiscard]] const std::string& getName() const noexcept { return name_; }

        /**
         * @brief Get current state
         */
        [[nodiscard]] TimelineState getState() const noexcept { return state_; }

        /**
         * @brief Check if timeline is running
         */
        [[nodiscard]] bool isRunning() const noexcept {
            return state_ == TimelineState::RUNNING && !isPaused_;
        }

        /**
         * @brief Check if timeline is paused
         */
        [[nodiscard]] bool isPaused() const noexcept { return isPaused_; }

        /**
         * @brief Get current time scale
         */
        [[nodiscard]] TimeScale getTimeScale() const noexcept { return timeScale_; }

        /**
         * @brief Get current timeline time
         */
        [[nodiscard]] Duration getCurrentTime() const noexcept { return currentTime_; }

        /**
         * @brief Get total elapsed time (excluding paused time)
         */
        [[nodiscard]] Duration getTotalElapsedTime() const noexcept { return totalElapsedTime_; }

        /**
         * @brief Get total paused time
         */
        [[nodiscard]] Duration getTotalPausedTime() const noexcept { return totalPausedTime_; }

        /**
         * @brief Get last update timestamp
         */
        [[nodiscard]] TimeStamp getLastUpdateTime() const noexcept { return lastUpdateTime_; }

        // =============================================================================
        // Capability Queries
        // =============================================================================

        /**
         * @brief Check if timeline can be paused
         */
        [[nodiscard]] bool canPause() const noexcept {
            return timelineHasCapability(type_, TimelineCapability::PAUSEABLE);
        }

        /**
         * @brief Check if timeline supports time scaling
         */
        [[nodiscard]] bool canScale() const noexcept {
            return timelineHasCapability(type_, TimelineCapability::SCALABLE);
        }

        /**
         * @brief Check if timeline can run backwards
         */
        [[nodiscard]] bool canReverse() const noexcept {
            return timelineHasCapability(type_, TimelineCapability::REVERSIBLE);
        }

        /**
         * @brief Check if timeline is deterministic
         */
        [[nodiscard]] bool isDeterministic() const noexcept {
            return timelineHasCapability(type_, TimelineCapability::DETERMINISTIC);
        }

        // =============================================================================
        // Configuration
        // =============================================================================

        /**
         * @brief Get timeline configuration
         */
        [[nodiscard]] virtual const TimelineConfigBase& getConfig() const = 0;

        /**
         * @brief Update timeline configuration
         * @param config New configuration
         * @return True if configuration was updated
         */
        virtual bool updateConfig(const TimelineConfigBase& config) = 0;

        /**
         * @brief Get timestep configuration
         */
        [[nodiscard]] virtual const TimestepConfig& getTimestepConfig() const = 0;

        /**
         * @brief Set timestep mode
         * @param mode New timestep mode
         * @return True if mode was changed
         */
        virtual bool setTimestepMode(TimestepMode mode) = 0;

        // =============================================================================
        // Synchronization and Dependencies
        // =============================================================================

        /**
         * @brief Synchronize with another timeline
         * @param other Timeline to sync with
         * @param offset Time offset to maintain
         * @return True if synchronization established
         */
        virtual bool synchronizeWith(Timeline* other, Duration offset = Duration::zero()) {
            // Default implementation - can be overridden
            return false;
        }

        /**
         * @brief Add dependency on another timeline
         * @param other Timeline this depends on
         * @return True if dependency added
         */
        virtual bool addDependency(Timeline* other) {
            // Default implementation - can be overridden
            return false;
        }

        /**
         * @brief Wait for timeline to reach specific time
         * @param targetTime Time to wait for
         * @param timeout Maximum wait duration
         * @return True if target time reached
         */
        virtual bool waitForTime(Duration targetTime, Duration timeout = Duration::max()) = 0;

    protected:
        // =============================================================================
        // Protected Virtual Hooks
        // =============================================================================

        /**
         * @brief Called when timeline starts
         */
        virtual void onStart() {
        }

        /**
         * @brief Called when timeline stops
         */
        virtual void onStop() {
        }

        /**
         * @brief Called when timeline pauses
         */
        virtual void onPause() {
        }

        /**
         * @brief Called when timeline resumes
         */
        virtual void onResume() {
        }

        /**
         * @brief Called when timeline resets
         */
        virtual void onReset() {
        }

        /**
         * @brief Called when time scale changes
         * @param oldScale Previous time scale
         * @param newScale New time scale
         */
        virtual void onTimeScaleChanged(TimeScale oldScale, TimeScale newScale) {
        }

        // =============================================================================
        // Protected Members
        // =============================================================================

        TimelineType type_; ///< Timeline type
        std::string name_; ///< Timeline name
        TimelineID id_; ///< Unique identifier
        TimelineState state_; ///< Current state

        std::atomic<bool> isPaused_; ///< Pause state
        std::atomic<TimeScale> timeScale_; ///< Time scaling factor

        Duration currentTime_; ///< Current timeline time
        Duration totalElapsedTime_; ///< Total elapsed time
        Duration totalPausedTime_; ///< Total paused duration

        TimeStamp lastUpdateTime_; ///< Last update timestamp
        std::optional<TimeStamp> pauseStartTime_; ///< When pause started

        /**
         * @brief Generate unique timeline ID
         */
        static TimelineID generateTimelineId() {
            static std::atomic<TimelineID> nextId{0};
            return nextId.fetch_add(1, std::memory_order_relaxed);
        }
    };

    // =============================================================================
    // Timeline Pointer Types
    // =============================================================================

    using TimelinePtr = std::shared_ptr<Timeline>;
    using TimelineWeakPtr = std::weak_ptr<Timeline>;
    using TimelineUniquePtr = std::unique_ptr<Timeline>;
} // namespace engine::time
