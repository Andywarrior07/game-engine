//
// Created by Andres Guerrero on 22-09-25.
//

#include "RealTimeline.h"

namespace engine::time {
    RealTimeline::~RealTimeline() {
        Timeline::stop();
    }

    bool RealTimeline::initialize(const TimelineConfigBase& config) {
        if (state_ != TimelineState::UNINITIALIZED) {
            return false;
        }

        config_ = config;

        // Override configuration to enforce real-time behavior
        config_.timestepConfig.mode = TimestepMode::VARIABLE;
        config_.initialScale = 1.0; // Always 1:1 scale
        config_.startPaused = false; // Never starts paused

        // Initialize timing
        startTime_ = backend_.now();
        lastUpdateTime_ = startTime_;
        currentTime_ = Duration::zero();

        // Pre-allocate timer storage
        timers_.reserve(config_.maxEventQueue);

        state_ = TimelineState::INITIALIZED;

        if (config_.autoStart) {
            return start();
        }

        return true;
    }

    // TODO: Revisar esto
    TimelineUpdateResult RealTimeline::update(Duration deltaTime) {
        if (state_ != TimelineState::RUNNING) {
            return TimelineUpdateResult{};
        }

        // Always use actual elapsed time, ignore provided delta
        const TimeStamp now = backend_.now();
        const Duration actualDelta = std::chrono::duration_cast<Duration>(
            now - lastUpdateTime_);

        // Update timing
        lastUpdateTime_ = now;
        currentTime_ += actualDelta;
        totalElapsedTime_ += actualDelta;
        frameCount_.fetch_add(1, std::memory_order_relaxed);

        // Update frame statistics
        frameStats_.updateFrame(actualDelta);

        // Process timers
        const auto timersProcessed = processTimers(now);

        // Calculate instantaneous metrics
        updateMetrics(actualDelta);

        return TimelineUpdateResult{
            .actualDeltaTime = actualDelta,
            .scaledDeltaTime = actualDelta, // No scaling for real time
            .fixedStepsExecuted = 0, // Variable timestep
            .timersProcessed = timersProcessed,
            .wasThrottled = false,
            .budgetExceeded = false
        };
    }

    void RealTimeline::reset() {
        startTime_ = backend_.now();
        lastUpdateTime_ = startTime_;
        currentTime_ = Duration::zero();
        totalElapsedTime_ = Duration::zero();
        frameCount_.store(0, std::memory_order_relaxed);
        frameStats_.reset();

        clearAllTimers();
    }

    SafeTimerHandle RealTimeline::createTimer(const Duration duration, TimerCallback callback, const bool recurring) {
        std::lock_guard lock(timerMutex_);

        const TimerID id = timerIdCounter_.fetch_add(1, std::memory_order_relaxed);
        const TimeStamp fireTime = backend_.now() + duration;

        RealTimer timer{
            .id = id,
            .fireTime = fireTime,
            .duration = duration,
            .callback = std::move(callback),
            .recurring = recurring,
            .generation = 1
        };

        const TimerHandle handle{id, timer.generation};
        timers_.emplace(id, std::move(timer));

        // Add to priority queue
        timerQueue_.push({fireTime, id});

        return SafeTimerHandle{handle, nullptr}; // TODO: Set timer system
    }

    bool RealTimeline::cancelTimer(const SafeTimerHandle& handle) {
        std::lock_guard lock(timerMutex_);

        const auto it = timers_.find(handle.getId());

        if (it == timers_.end() || it->second.generation != handle.getGeneration()) return false;

        it->second.cancelled = true;
        timers_.erase(it);

        return true;
    }

    void RealTimeline::clearAllTimers() {
        std::lock_guard lock(timerMutex_);
        timers_.clear();

        // Clear priority queue
        while (!timerQueue_.empty()) {
            timerQueue_.pop();
        }
    }

    bool RealTimeline::updateConfig(const TimelineConfigBase& config) {
        // Limited configuration updates for real timeline
        config_ = config;

        // Enforce real-time constraints
        config_.timestepConfig.mode = TimestepMode::VARIABLE;
        config_.initialScale = 1.0;

        return true;
    }

    void RealTimeline::synchronizeWith(const TimeStamp externalTime, const Duration latency) {
        const TimeStamp localTime = backend_.now();
        const Duration offset = std::chrono::duration_cast<Duration>(
            externalTime - localTime) + latency;

        timeSyncOffset_ = offset;
        lastSyncTime_ = localTime;
    }

    bool RealTimeline::waitForTime(const Duration targetTime, const Duration timeout) {
        const TimeStamp startWait = backend_.now();
        const TimeStamp targetTimestamp = startTime_ + targetTime;
        const TimeStamp timeoutTimestamp = startWait + timeout;

        while (backend_.now() < targetTimestamp) {
            if (backend_.now() >= timeoutTimestamp) {
                return false; // Timeout
            }

            // Short sleep to avoid busy waiting
            backend_.sleep(Duration(100)); // 100μs
        }

        return true;
    }

    std::uint32_t RealTimeline::processTimers(const TimeStamp currentTime) {
        std::uint32_t processed = 0;
        std::vector<TimerCallback> callbacks;

        {
            std::lock_guard lock(timerMutex_);

            while (!timerQueue_.empty() &&
                timerQueue_.top().fireTime <= currentTime) {
                const auto entry = timerQueue_.top();
                timerQueue_.pop();

                if (auto it = timers_.find(entry.id); it != timers_.end() && !it->second.cancelled) {
                    callbacks.push_back(it->second.callback);

                    if (it->second.recurring) {
                        // Reschedule recurring timer
                        it->second.fireTime = currentTime + it->second.duration;
                        timerQueue_.push({it->second.fireTime, it->second.id});
                    }
                    else {
                        // Remove one-shot timer
                        timers_.erase(it);
                    }

                    processed++;
                }
            }
        }

        // Execute callbacks outside lock
        for (const auto& callback : callbacks) {
            callback(TimerHandle{}); // TODO: Provide proper handle
        }

        return processed;
    }
} // namespace engine::time