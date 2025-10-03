//
// Created by Andres Guerrero on 30-09-25.
//

#include "TimerSystem.h"

namespace engine::time {
    TimerSystem::~TimerSystem() {
        shutdown();
    }

    bool TimerSystem::initialize() {
        if (isInitialized_) {
            return false;
        }

        // Create timer pool if pooling enabled
        if (config_.enablePooling) {
            TimerPool::Config poolConfig{
                            .initialSize = config_.poolInitialSize,
                            .growthSize = config_.poolGrowthSize,
                            .maxSize = config_.maxTimers,
                            .preallocate = true,
                            .allowGrowth = true
                    };

            timerPool_ = std::make_unique<TimerPool>(memoryManager_, poolConfig);
        }

        // Create handle pool
        handlePool_ = std::make_unique<TimerHandlePool>(config_.maxTimers);

        // Reserve space for registrations
        timers_.reserve(config_.maxTimers);
        timersByTimeline_.reserve(16); // Typical timeline count

        isInitialized_ = true;
        return true;
    }

    void TimerSystem::shutdown() {
        if (!isInitialized_) {
            return;
        }

        // Cancel all timers
        cancelAllTimers();

        // Clear registrations
        {
            std::unique_lock lock(registryMutex_);
            timers_.clear();
            timersByTimeline_.clear();
        }

        // Destroy pools
        timerPool_.reset();
        handlePool_.reset();

        isInitialized_ = false;
    }

    SafeTimerHandle TimerSystem::createTimer(const Timer::Config& config, const TimelineID timelineId) {
        if (!isInitialized_) {
            return SafeTimerHandle{};
        }

        // Allocate timer
        Timer* timer = allocateTimer();
        if (!timer) {
            return SafeTimerHandle{};
        }

        // Initialize timer
        Timer::Config timerConfig = config;
        timerConfig.timelineId = timelineId;

        if (!timer->initialize(timerConfig)) {
            deallocateTimer(timer);
            return SafeTimerHandle{};
        }

        // Allocate handle
        const TimerHandle handle = handlePool_->allocate();
        if (!handle.isValid()) {
            deallocateTimer(timer);
            return SafeTimerHandle{};
        }

        const TimeStamp currentTime = Clock::now();
        calculateExpiration(timer, currentTime);

        // Set timer ID and generation
        timer->id_ = handle.id;
        timer->generation_ = handle.generation;

        // Register timer
        {
            std::unique_lock lock(registryMutex_);

            const TimerRegistration registration{
                            .timer = timer,
                            .handle = handle,
                            .timelineId = timelineId,
                            .createdAt = Clock::now()
                    };

            timers_[handle.id] = registration;
            timersByTimeline_[timelineId].insert(handle.id);
        }

        // Update statistics
        if (config_.trackStatistics) {
            stats_.totalCreated.fetch_add(1, std::memory_order_relaxed);
            stats_.currentActive.fetch_add(1, std::memory_order_relaxed);
            updatePeakActive();
        }

        return SafeTimerHandle{handle, this};
    }

    SafeTimerHandle TimerSystem::createOneShotTimer(
            const Duration duration,
            TimerCallback callback,
            const TimelineID timelineId
            ) {

        // TODO: Revisar que falta Name en el config
        const Timer::Config config{
                        .duration = duration,
                        .callback = std::move(callback),
                        .type = TimerType::ONE_SHOT,
                        .timelineId = timelineId
                };

        return createTimer(config, timelineId);
    }

    SafeTimerHandle TimerSystem::createRecurringTimer(
            const Duration interval,
            TimerCallback callback,
            const TimelineID timelineId
            ) {

        // TODO: Revisar que falta Name en el config
        const Timer::Config config{
                        .duration = interval,
                        .callback = std::move(callback),
                        .type = TimerType::RECURRING,
                        .timelineId = timelineId
                };

        return createTimer(config, timelineId);
    }

    bool TimerSystem::cancelTimer(const SafeTimerHandle& handle) {
        if (!validateHandle(handle.getRawHandle())) {
            return false;
        }

        Timer* timer = nullptr;

        {
            std::unique_lock lock(registryMutex_);

            if (const auto it = timers_.find(handle.getId()); it != timers_.end()) {
                timer = it->second.timer;

                // Remove from timeline index
                auto& timelineTimers = timersByTimeline_[it->second.timelineId];
                timelineTimers.erase(handle.getId());

                // Remove from registry
                timers_.erase(it);
            }
        }

        if (timer) {
            timer->cancel();
            deallocateTimer(timer);
            handlePool_->release(handle.getRawHandle());

            // Update statistics
            if (config_.trackStatistics) {
                stats_.totalCancelled.fetch_add(1, std::memory_order_relaxed);
                stats_.currentActive.fetch_sub(1, std::memory_order_relaxed);
            }

            return true;
        }

        return false;
    }

    bool TimerSystem::pauseTimer(const SafeTimerHandle& handle) const {
        Timer* timer = getTimer(handle.getRawHandle());
        return timer ? timer->pause() : false;
    }

    bool TimerSystem::resumeTimer(const SafeTimerHandle& handle) const {
        Timer* timer = getTimer(handle.getRawHandle());
        return timer ? timer->resume() : false;
    }

    bool TimerSystem::resetTimer(const SafeTimerHandle& handle, const bool restart) const {
        Timer* timer = getTimer(handle.getRawHandle());

        if (!timer) {
            return false;
        }

        timer->reset(restart);

        // Recalculate expiration time after reset
        if (restart) {
            const TimeStamp currentTime = Clock::now();
            calculateExpiration(timer, currentTime);
        }

        return true;
    }

    std::size_t TimerSystem::cancelTimelineTimers(const TimelineID timelineId) {
        std::vector<TimerID> toCancel;

        {
            std::shared_lock lock(registryMutex_);

            if (const auto it = timersByTimeline_.find(timelineId); it != timersByTimeline_.end()) {
                toCancel.assign(it->second.begin(), it->second.end());
            }
        }

        std::size_t cancelled = 0;
        for (const TimerID id : toCancel) {
            if (cancelTimer(SafeTimerHandle{TimerHandle{id, 0}, this})) {
                cancelled++;
            }
        }

        return cancelled;
    }

    std::size_t TimerSystem::cancelAllTimers() {
        std::vector<TimerHandle> handles;

        {
            std::shared_lock lock(registryMutex_);
            handles.reserve(timers_.size());

            for (const auto& reg : timers_ | std::views::values) {
                handles.push_back(reg.handle);
            }
        }

        std::size_t cancelled = 0;
        for (const auto& handle : handles) {
            if (cancelTimer(SafeTimerHandle{handle, this})) {
                cancelled++;
            }
        }

        return cancelled;
    }

    std::size_t TimerSystem::pauseTimelineTimers(const TimelineID timelineId) {
        std::shared_lock lock(registryMutex_);

        std::size_t paused = 0;

        if (const auto it = timersByTimeline_.find(timelineId); it != timersByTimeline_.end()) {
            for (TimerID id : it->second) {
                if (auto timerIt = timers_.find(id); timerIt != timers_.end() &&
                    timerIt->second.timer->pause()) {
                    paused++;
                }
            }
        }

        return paused;
    }

    std::size_t TimerSystem::resumeTimelineTimers(const TimelineID timelineId) {
        std::shared_lock lock(registryMutex_);

        std::size_t resumed = 0;

        if (const auto it = timersByTimeline_.find(timelineId); it != timersByTimeline_.end()) {
            for (TimerID id : it->second) {
                if (auto timerIt = timers_.find(id); timerIt != timers_.end() &&
                    timerIt->second.timer->resume()) {
                    resumed++;
                }
            }
        }

        return resumed;
    }

    bool TimerSystem::validateHandle(const TimerHandle handle) const {
        if (!handle.isValid() || !handlePool_->validate(handle)) {
            return false;
        }

        std::shared_lock lock(registryMutex_);
        const auto it = timers_.find(handle.id);

        return it != timers_.end() &&
                it->second.handle.generation == handle.generation;
    }

    std::size_t TimerSystem::getTimerCount() const {
        std::shared_lock lock(registryMutex_);
        return timers_.size();
    }

    std::size_t TimerSystem::getActiveTimerCount() const {
        return stats_.currentActive.load(std::memory_order_acquire);
    }

    Timer* TimerSystem::allocateTimer() const {
        if (timerPool_) {
            return timerPool_->allocate();
        }
        return new Timer();
    }

    void TimerSystem::deallocateTimer(Timer* timer) const {
        if (!timer)
            return;

        if (timerPool_ && timerPool_->owns(timer)) {
            timerPool_->deallocate(timer);
        } else {
            delete timer;
        }
    }

    Timer* TimerSystem::getTimer(const TimerHandle handle) const {
        if (!validateHandle(handle)) {
            return nullptr;
        }

        std::shared_lock lock(registryMutex_);

        const auto it = timers_.find(handle.id);
        return (it != timers_.end()) ? it->second.timer : nullptr;
    }

    void TimerSystem::updatePeakActive() {
        const auto current = stats_.currentActive.load(std::memory_order_relaxed);
        auto peak = stats_.peakActive.load(std::memory_order_relaxed);

        while (current > peak &&
            !stats_.peakActive.compare_exchange_weak(
                    peak,
                    current,
                    std::memory_order_release,
                    std::memory_order_relaxed
                    )) {}
    }

    void TimerSystem::calculateExpiration(Timer* timer, const TimeStamp currentTime) {
        if (!timer)
            return;

        // Calculate absolute expiration time
        const Duration remaining = timer->getRemainingTime();
        const TimeStamp expirationTime = currentTime + remaining;

        timer->setExpiration(expirationTime);
    }
} // namespace engine::time
