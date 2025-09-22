/**
 * @file Timer.h
 * @brief Individual timer implementation with callback functionality
 * @details Provides timer instances supporting one-shot, recurring, and conditional
 *          execution. Efficiently tracks timer state and manages callbacks while
 *          maintaining timeline binding for proper time synchronization.
 *
 * @author Andres Guerrero
 * @date Created on 2025-09-19
 */

#pragma once

#include "../core/TimeTypes.h"

#include <functional>
#include <atomic>
#include <optional>

namespace engine::time {
    // Forward declarations
    class Timeline;
    class TimerRegistry;

    /**
     * @brief Timer execution type
     */
    enum class TimerType : std::uint8_t {
        ONE_SHOT, ///< Executes once and stops
        RECURRING, ///< Repeats at fixed intervals
        CONDITIONAL, ///< Executes based on condition
        DELAYED, ///< One-shot with initial delay
        INTERVAL, ///< Recurring with different first delay
        COUNT
    };

    /**
     * @brief Timer execution state
     */
    enum class TimerState : std::uint8_t {
        INACTIVE, ///< Not initialized
        PENDING, ///< Waiting to fire
        EXECUTING, ///< Currently executing callback
        PAUSED, ///< Temporarily suspended
        COMPLETED, ///< Finished (one-shot)
        CANCELLED ///< Explicitly cancelled
    };

    /**
     * @brief Timer priority for execution order
     */
    enum class TimerPriority : std::uint8_t {
        CRITICAL = 0, ///< Execute first
        HIGH = 64, ///< High priority
        NORMAL = 128, ///< Default priority
        LOW = 192, ///< Low priority
        IDLE = 255 ///< Execute last
    };

    /**
     * @brief Individual timer with callback functionality
     * @details Core timer implementation supporting various execution modes.
     *          Designed for efficient state tracking and minimal memory overhead.
     *          Thread-safe for concurrent access from timeline and user code.
     *
     * Performance characteristics:
     * - O(1) state queries
     * - Minimal memory footprint (64-128 bytes)
     * - Lock-free state transitions where possible
     * - Cache-friendly layout
     */
    class Timer {
    public:
        /**
         * @brief Timer configuration
         */
        struct Config {
            Duration duration{Duration::zero()}; ///< Timer duration
            TimerCallback callback; ///< Execution callback
            TimerType type{TimerType::ONE_SHOT}; ///< Timer type
            TimerPriority priority{TimerPriority::NORMAL}; ///< Execution priority
            TimelineID timelineId{constants::INVALID_TIMELINE_ID}; ///< Bound timeline
            std::string name; ///< Debug name
            bool startImmediately{true}; ///< Auto-start
            bool ignoreTimeScale{false}; ///< Ignore timeline scaling
            std::function<bool()> condition; ///< For conditional timers
            Duration initialDelay{Duration::zero()}; ///< Initial delay
        };

        /**
         * @brief Default constructor
         */
        Timer() noexcept
            : id_(constants::INVALID_TIMER_ID)
              , generation_(0)
              , type_(TimerType::ONE_SHOT)
              , state_(TimerState::INACTIVE)
              , priority_(TimerPriority::NORMAL)
              , duration_(Duration::zero())
              , remaining_(Duration::zero())
              , elapsed_(Duration::zero())
              , initialDelay_(Duration::zero())
              , executionCount_(0)
              , maxExecutions_(0)
              , timelineId_(constants::INVALID_TIMELINE_ID)
              , ignoreTimeScale_(false) {
        }

        /**
         * @brief Construct timer with configuration
         * @param config Timer configuration
         */
        explicit Timer(const Config& config)
            : Timer() {
            initialize(config);
        }

        /**
         * @brief Destructor
         */
        ~Timer() {
            cancel();
        }

        // Delete copy, allow move
        Timer(const Timer&) = delete;
        Timer& operator=(const Timer&) = delete;
        Timer(Timer&& other) noexcept = default;
        Timer& operator=(Timer&& other) noexcept = default;

        // =============================================================================
        // Initialization and Configuration
        // =============================================================================

        /**
         * @brief Initialize timer with configuration
         * @param config Timer configuration
         * @return True if initialization successful
         */
        bool initialize(const Config& config) {
            if (state_ != TimerState::INACTIVE) {
                return false; // Already initialized
            }

            // Validate configuration
            if (config.duration <= Duration::zero() &&
                config.type != TimerType::CONDITIONAL) {
                return false; // Invalid duration
            }

            if (!config.callback) {
                return false; // No callback
            }

            // Set configuration
            type_ = config.type;
            priority_ = config.priority;
            duration_ = config.duration;
            remaining_ = config.duration;
            callback_ = config.callback;
            condition_ = config.condition;
            timelineId_ = config.timelineId;
            name_ = config.name;
            ignoreTimeScale_ = config.ignoreTimeScale;

            // Handle initial delay
            if (config.initialDelay > Duration::zero()) {
                remaining_ = config.initialDelay;
                initialDelay_ = config.initialDelay;
            }

            // Set max executions based on type
            switch (type_) {
            case TimerType::ONE_SHOT:
            case TimerType::DELAYED:
                maxExecutions_ = 1;
                break;

            case TimerType::RECURRING:
            case TimerType::INTERVAL:
            case TimerType::CONDITIONAL:
                maxExecutions_ = 0; // Unlimited
                break;

            default:
                break;
            }

            // Set initial state
            state_ = config.startImmediately ? TimerState::PENDING : TimerState::PAUSED;

            return true;
        }

        /**
         * @brief Reset timer to initial state
         * @param restart If true, start immediately
         */
        void reset(const bool restart = true) {
            elapsed_ = Duration::zero();
            remaining_ = initialDelay_ > Duration::zero() ? initialDelay_ : duration_;
            executionCount_ = 0;

            if (restart) {
                state_ = TimerState::PENDING;
            }
            else {
                state_ = TimerState::PAUSED;
            }
        }

        // =============================================================================
        // Timer Control
        // =============================================================================

        /**
         * @brief Start the timer
         * @return True if timer started
         */
        bool start() {
            if (state_ == TimerState::INACTIVE ||
                state_ == TimerState::CANCELLED ||
                state_ == TimerState::COMPLETED) {
                return false;
            }

            state_ = TimerState::PENDING;
            return true;
        }

        /**
         * @brief Pause the timer
         * @return True if timer paused
         */
        bool pause() {
            if (state_ != TimerState::PENDING) {
                return false;
            }

            state_ = TimerState::PAUSED;
            pauseTime_ = Clock::now();
            return true;
        }

        /**
         * @brief Resume the timer
         * @return True if timer resumed
         */
        bool resume() {
            if (state_ != TimerState::PAUSED) {
                return false;
            }

            state_ = TimerState::PENDING;

            if (pauseTime_) {
                // Adjust for pause duration
                // TODO: Revisar esto
                const auto pauseDuration = Clock::now() - *pauseTime_;
                // Store pause adjustment for timeline sync
                pauseTime_ = std::nullopt;
            }

            return true;
        }

        /**
         * @brief Cancel the timer
         */
        void cancel() {
            state_ = TimerState::CANCELLED;
            callback_ = nullptr;
            condition_ = nullptr;
        }

        /**
         * @brief Update timer state
         * @param deltaTime Time elapsed since last update
         * @return True if timer fired
         */
        bool update(const Duration deltaTime) {
            if (state_ != TimerState::PENDING) {
                return false;
            }

            // Load current values
            const Duration currentElapsed = elapsed_.load(std::memory_order_relaxed);
            const Duration currentRemaining = remaining_.load(std::memory_order_relaxed);

            // Calculate new values
            const Duration newElapsed = currentElapsed + deltaTime;
            const Duration newRemaining = currentRemaining - deltaTime;

            // Store new values
            elapsed_.store(newElapsed, std::memory_order_relaxed);
            remaining_.store(newRemaining, std::memory_order_relaxed);

            // Check condition for conditional timers
            if (type_ == TimerType::CONDITIONAL) {
                if (condition_ && condition_()) {
                    return fire();
                }
                return false;
            }

            // Check if timer should fire
            if (newRemaining <= Duration::zero()) {
                return fire();
            }

            return false;
        }

        /**
         * @brief Fire the timer
         * @return True if callback executed
         */
        bool fire() {
            if (state_ != TimerState::PENDING || !callback_) {
                return false;
            }

            state_ = TimerState::EXECUTING;

            // Execute callback
            callback_(TimerHandle{id_, generation_});

            ++executionCount_;

            // Handle post-execution state
            switch (type_) {
            case TimerType::ONE_SHOT:
            case TimerType::DELAYED:
                state_ = TimerState::COMPLETED;
                break;

            case TimerType::RECURRING:
                remaining_ = duration_;
                state_ = TimerState::PENDING;
                break;

            case TimerType::INTERVAL:
                // After first execution, use regular duration xd
                // TODO: Revisar esto
                if (executionCount_ == 1 && initialDelay_ > Duration::zero()) {
                    remaining_ = duration_;
                }
                else {
                    remaining_ = duration_;
                }
                state_ = TimerState::PENDING;
                break;

            case TimerType::CONDITIONAL:
                state_ = TimerState::PENDING;
                break;

            default:
                break;
            }

            // Check max executions
            if (maxExecutions_ > 0 && executionCount_ >= maxExecutions_) {
                state_ = TimerState::COMPLETED;
            }

            return true;
        }

        // =============================================================================
        // State Queries
        // =============================================================================

        /**
         * @brief Get timer ID
         */
        [[nodiscard]] TimerID getId() const noexcept {
            return id_;
        }

        /**
         * @brief Get timer generation
         */
        [[nodiscard]] TimerGeneration getGeneration() const noexcept {
            return generation_;
        }

        /**
         * @brief Get timer handle
         */
        [[nodiscard]] TimerHandle getHandle() const noexcept {
            return TimerHandle{id_, generation_};
        }

        /**
         * @brief Get timer type
         */
        [[nodiscard]] TimerType getType() const noexcept {
            return type_;
        }

        /**
         * @brief Get timer state
         */
        [[nodiscard]] TimerState getState() const noexcept {
            return state_.load(std::memory_order_acquire);
        }

        /**
         * @brief Get timer priority
         */
        [[nodiscard]] TimerPriority getPriority() const noexcept {
            return priority_;
        }

        /**
         * @brief Check if timer is active
         */
        [[nodiscard]] bool isActive() const noexcept {
            const auto currentState = state_.load(std::memory_order_acquire);
            return currentState == TimerState::PENDING ||
                currentState == TimerState::EXECUTING;
        }

        /**
         * @brief Check if timer is paused
         */
        [[nodiscard]] bool isPaused() const noexcept {
            return state_.load(std::memory_order_acquire) == TimerState::PAUSED;
        }

        /**
         * @brief Check if timer is completed
         */
        [[nodiscard]] bool isCompleted() const noexcept {
            return state_.load(std::memory_order_acquire) == TimerState::COMPLETED;
        }

        /**
         * @brief Get configured duration
         */
        [[nodiscard]] Duration getDuration() const noexcept {
            return duration_;
        }

        /**
         * @brief Get remaining time
         */
        [[nodiscard]] Duration getRemainingTime() const noexcept {
            return remaining_.load(std::memory_order_acquire);
        }

        /**
         * @brief Get elapsed time
         */
        [[nodiscard]] Duration getElapsedTime() const noexcept {
            return elapsed_.load(std::memory_order_acquire);
        }

        /**
         * @brief Get execution count
         */
        [[nodiscard]] std::uint32_t getExecutionCount() const noexcept {
            return executionCount_.load(std::memory_order_acquire);
        }

        /**
         * @brief Get progress ratio
         * @return Progress [0, 1] or 0 for conditional timers
         */
        [[nodiscard]] float getProgress() const noexcept {
            if (type_ == TimerType::CONDITIONAL) {
                return 0.0f;
            }

            if (duration_.count() == 0) {
                return 1.0f;
            }

            const float progress = 1.0f -
            (static_cast<float>(remaining_.load().count()) /
                static_cast<float>(duration_.count()));

            return std::clamp(progress, 0.0f, 1.0f);
        }

        /**
         * @brief Get bound timeline ID
         */
        [[nodiscard]] TimelineID getTimelineId() const noexcept {
            return timelineId_;
        }

        /**
         * @brief Get timer name
         */
        [[nodiscard]] const std::string& getName() const noexcept {
            return name_;
        }

        // =============================================================================
        // Configuration Updates
        // =============================================================================

        /**
         * @brief Set timer duration (for recurring timers)
         * @param duration New duration
         */
        void setDuration(const Duration duration) {
            if (type_ == TimerType::RECURRING ||
                type_ == TimerType::INTERVAL) {
                duration_ = duration;
            }
        }

        /**
         * @brief Set timer callback
         * @param callback New callback function
         */
        void setCallback(TimerCallback callback) {
            callback_ = std::move(callback);
        }

        /**
         * @brief Set condition (for conditional timers)
         * @param condition Condition function
         */
        void setCondition(std::function<bool()> condition) {
            if (type_ == TimerType::CONDITIONAL) {
                condition_ = std::move(condition);
            }
        }

        /**
         * @brief Set max executions
         * @param count Maximum execution count (0 = unlimited)
         */
        void setMaxExecutions(const std::uint32_t count) {
            maxExecutions_ = count;
        }

        /**
         * @brief Set priority
         * @param priority New priority
         */
        void setPriority(const TimerPriority priority) {
            priority_ = priority;
        }

    private:
        friend class TimerRegistry;

        // Core identification
        TimerID id_; ///< Timer identifier
        TimerGeneration generation_; ///< Handle generation

        // Timer configuration
        TimerType type_; ///< Timer type
        std::atomic<TimerState> state_; ///< Current state
        TimerPriority priority_; ///< Execution priority

        // Timing data
        Duration duration_; ///< Configured duration
        std::atomic<Duration> remaining_; ///< Time remaining
        std::atomic<Duration> elapsed_; ///< Time elapsed
        Duration initialDelay_; ///< Initial delay

        // Execution tracking
        std::atomic<std::uint32_t> executionCount_; ///< Times executed
        std::uint32_t maxExecutions_; ///< Max executions

        // Callbacks
        TimerCallback callback_; ///< Timer callback
        std::function<bool()> condition_; ///< Condition function

        // Timeline binding
        TimelineID timelineId_; ///< Bound timeline
        bool ignoreTimeScale_; ///< Ignore scaling

        // Debug info
        std::string name_; ///< Timer name

        // Pause tracking
        std::optional<TimeStamp> pauseTime_; ///< When paused
    };

    /**
     * @brief Comparison operator for timer priority queue
     */
    struct TimerCompare {
        bool operator()(const Timer* a, const Timer* b) const {
            // First compare by remaining time
            if (a->getRemainingTime() != b->getRemainingTime()) {
                return a->getRemainingTime() > b->getRemainingTime();
            }

            // Then by priority
            return static_cast<std::uint8_t>(a->getPriority()) >
                static_cast<std::uint8_t>(b->getPriority());
        }
    };
} // namespace engine::time
