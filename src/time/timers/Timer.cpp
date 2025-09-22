//
// Created by Andres Guerrero on 20-09-25.
//

#include "Timer.h"

namespace engine::time {
    Timer::Timer(const Config& config): Timer() {
        initialize(config);
    }

    Timer::~Timer() {
        cancel();
    }

    bool Timer::initialize(const Config& config) {
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

    bool Timer::update(const Duration deltaTime) {
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

    bool Timer::fire() {
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

    float Timer::getProgress() const noexcept {
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
} // namespace engine::time