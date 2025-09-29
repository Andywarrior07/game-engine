//
// Created by Andres Guerrero on 22-09-25.
//

#include "PhysicsTimeline.h"

namespace engine::time {
    PhysicsTimeline::~PhysicsTimeline() {
        Timeline::stop();
    }

    bool PhysicsTimeline::initialize(const TimelineConfigBase& config) {
        if (state_ != TimelineState::UNINITIALIZED) {
            return false;
        }

        config_ = config;

        // Force fixed timestep mode for physics
        config_.timestepConfig.mode = TimestepMode::FIXED;

        // Extract fixed timestep configuration
        if (const auto fixedConfig = config_.timestepConfig.getFixedConfig()) {
            fixedTimestep_ = fixedConfig->timestep;
            maxSubsteps_ = fixedConfig->maxIterationsPerFrame;
            maxAccumulator_ = fixedConfig->maxAccumulator;
            substeps_ = fixedConfig->substeps;
            catchUpAfterSpike_ = fixedConfig->catchUpAfterSpike;
        }

        // Set initial state
        timeScale_ = config_.initialScale;
        isPaused_ = config_.startPaused;

        // Initialize timing
        lastUpdateTime_ = backend_.now();
        lastFixedUpdateTime_ = lastUpdateTime_;

        // Pre-allocate storage
        timers_.reserve(config_.maxEventQueue);

        state_ = TimelineState::INITIALIZED;

        if (config_.autoStart) {
            return start();
        }

        return true;
    }

    TimelineUpdateResult PhysicsTimeline::update(const Duration deltaTime) {
        if (state_ != TimelineState::RUNNING || isPaused_) {
            return TimelineUpdateResult{};
        }

        // Apply time scaling to input delta
        const auto scaledDelta = Duration(
                static_cast<std::int64_t>(static_cast<double>(deltaTime.count()) * timeScale_.load())
                );

        // Add to accumulator
        accumulator_ += scaledDelta;

        // Clamp accumulator to prevent spiral of death
        if (accumulator_ > maxAccumulator_) {
            onSpiralOfDeath(accumulator_);
            accumulator_ = maxAccumulator_;
        }

        // Count fixed steps to execute
        std::uint32_t stepsExecuted = 0;
        Duration totalStepTime = Duration::zero();

        // Execute fixed timesteps
        while (accumulator_ >= fixedTimestep_ && stepsExecuted < maxSubsteps_) {
            inFixedUpdate_.store(true, std::memory_order_release);

            // Execute substeps if configured
            for (std::uint8_t substep = 0; substep < substeps_; ++substep) {
                const Duration substepTime = fixedTimestep_ / substeps_;
                executePhysicsStep(substepTime);
            }

            // Update timing
            currentTime_ += fixedTimestep_;
            totalElapsedTime_ += fixedTimestep_;
            accumulator_ -= fixedTimestep_;
            totalStepTime += fixedTimestep_;
            stepCount_++;
            stepsExecuted++;

            // Process physics timers
            processPhysicsTimers(currentTime_);

            inFixedUpdate_.store(false, std::memory_order_release);
        }

        // Calculate interpolation alpha for rendering
        interpolationAlpha_.store(
                static_cast<float>(accumulator_.count()) /
                static_cast<float>(fixedTimestep_.count()),
                std::memory_order_release
                );

        // Execute interpolation callback if set
        if (interpolationCallback_) {
            interpolationCallback_(interpolationAlpha_.load());
        }

        // Record state for history/rollback
        recordPhysicsState();

        // Update statistics
        updatePhysicsStats(deltaTime, stepsExecuted);

        return TimelineUpdateResult{
                        .actualDeltaTime = deltaTime,
                        .scaledDeltaTime = totalStepTime,
                        .fixedStepsExecuted = stepsExecuted,
                        .timersProcessed = static_cast<std::uint32_t>(processedTimers_),
                        .wasThrottled = accumulator_ >= maxAccumulator_,
                        .budgetExceeded = stepsExecuted >= maxSubsteps_
                };
    }

    void PhysicsTimeline::setFixedTimestep(const Duration timestep) {
        if (timestep <= Duration::zero()) {
            return;
        }

        fixedTimestep_ = timestep;
        config_.timestepConfig.modeConfig = FixedTimestepConfig{
                        .timestep = timestep,
                        .maxIterationsPerFrame = maxSubsteps_,
                        .maxAccumulator = maxAccumulator_,
                        .substeps = substeps_
                };
    }

    void PhysicsTimeline::forcePhysicsSteps(const std::uint32_t steps) {
        for (std::uint32_t i = 0; i < steps; ++i) {
            executePhysicsStep(fixedTimestep_);
            currentTime_ += fixedTimestep_;
            stepCount_++;
        }
    }

    void PhysicsTimeline::restorePhysicsState(const PhysicsState& state) {
        currentTime_ = state.simulationTime;
        accumulator_ = state.accumulatedTime;
        stepCount_ = state.stepCount;
        interpolationAlpha_.store(state.interpolationAlpha);

        // Clear future history
        while (!stateHistory_.empty() &&
            stateHistory_.back().stepCount > state.stepCount) {
            stateHistory_.pop_back();
        }
    }

    std::vector<PhysicsTimeline::PhysicsState> PhysicsTimeline::getStateHistory(const std::size_t steps) const {
        std::lock_guard lock(historyMutex_);

        std::vector<PhysicsState> result;
        const std::size_t available = std::min(steps, stateHistory_.size());

        result.reserve(available);
        for (std::size_t i = 0; i < available; ++i) {
            result.push_back(stateHistory_[stateHistory_.size() - 1 - i]);
        }

        return result;
    }

    SafeTimerHandle PhysicsTimeline::createTimer(
            const Duration duration,
            TimerCallback callback,
            const bool recurring
            ) {
        std::lock_guard lock(timerMutex_);

        const TimerID id = nextTimerId_++;
        const Duration fireTime = currentTime_ + duration;

        PhysicsTimer timer{
                        .id = id,
                        .fireTime = fireTime,
                        .duration = duration,
                        .callback = std::move(callback),
                        .recurring = recurring,
                        .generation = 1
                };

        const TimerHandle handle{id, timer.generation};
        timers_[id] = std::move(timer);

        return SafeTimerHandle{handle, nullptr};
    }

    bool PhysicsTimeline::updateConfig(const TimelineConfigBase& config) {
        config_ = config;

        // Extract physics-specific settings
        if (const auto fixedConfig = config.timestepConfig.getFixedConfig()) {
            fixedTimestep_ = fixedConfig->timestep;
            maxSubsteps_ = fixedConfig->maxIterationsPerFrame;
            maxAccumulator_ = fixedConfig->maxAccumulator;
            substeps_ = fixedConfig->substeps;
        }

        return true;
    }

    bool PhysicsTimeline::waitForTime(const Duration targetTime, const Duration timeout) {
        const auto startWait = backend_.now();
        const auto timeoutTime = startWait + timeout;

        while (currentTime_ < targetTime) {
            if (backend_.now() >= timeoutTime) {
                return false;
            }

            // Wait for physics update
            backend_.sleep(Duration(100)); // 100μs
        }

        return true;
    }

    void PhysicsTimeline::processPhysicsTimers(const Duration currentTime) {
        std::lock_guard lock(timerMutex_);
        processedTimers_ = 0;

        std::vector<TimerCallback> toExecute;
        std::vector<TimerID> toRemove;

        for (auto& [id, timer] : timers_) {
            if (currentTime >= timer.fireTime) {
                toExecute.push_back(timer.callback);

                if (timer.recurring) {
                    timer.fireTime = currentTime + timer.duration;
                } else {
                    toRemove.push_back(id);
                }

                processedTimers_++;
            }
        }

        for (TimerID id : toRemove) {
            timers_.erase(id);
        }

        // Execute callbacks outside lock
        for (const auto& callback : toExecute) {
            callback(TimerHandle{});
        }
    }

    void PhysicsTimeline::recordPhysicsState() {
        std::lock_guard lock(historyMutex_);

        stateHistory_.push_back(savePhysicsState());

        // Limit history size
        if (stateHistory_.size() > 60) {
            stateHistory_.pop_front();
        }
    }

    void PhysicsTimeline::updatePhysicsStats(const Duration frameTime, std::uint32_t steps) {
        // TODO: Temporal, ya que std::atomic<Duration> da muchos problemas
        Duration currentTotalTime = totalPhysicsTime_.load(std::memory_order_relaxed);
        Duration newTotalTime;

        do {
            newTotalTime = currentTotalTime + frameTime;
        } while (!totalPhysicsTime_.compare_exchange_strong(
                currentTotalTime,
                newTotalTime,
                std::memory_order_relaxed,
                std::memory_order_relaxed
                ));
    }
} // namespace engine::time
