/**
 * @file GameTimeline.cpp
 * @brief Implementation of GameTimeline
 *
 * @author Andres Guerrero
 * @date 2025-09-22
 */

#include "GameTimeline.h"

#include <algorithm>
#include <ranges>

namespace engine::time {
    GameTimeline::GameTimeline(ITimeBackend& backend, const std::string& name) :
        Timeline(TimelineType::GAME_TIME, name)
        , backend_(backend)
        , frameNumber_(0) {}

    GameTimeline::~GameTimeline() {
        Timeline::stop();
    }

    bool GameTimeline::initialize(const TimelineConfigBase& config) {
        if (state_ != TimelineState::UNINITIALIZED) {
            return false;
        }

        config_ = config;

        // Set initial state
        timeScale_ = config_.initialScale;
        isPaused_ = config_.startPaused;

        // Initialize timing
        lastUpdateTime_ = backend_.now();

        // Pre-allocate storage
        timers_.reserve(config_.maxEventQueue);
        // TODO: Revisar el porque esta decision
        // frameHistory_.reserve(MAX_FRAME_HISTORY);

        state_ = TimelineState::INITIALIZED;

        if (config_.autoStart && !isPaused_) {
            return start();
        }

        return true;
    }

    TimelineUpdateResult GameTimeline::update(const Duration deltaTime) {
        if (state_ != TimelineState::RUNNING || isPaused_) {
            return TimelineUpdateResult{};
        }

        // Apply time scaling to the provided delta
        // Note: deltaTime comes pre-processed by TimestepManager
        const auto scaledDelta = Duration(
                static_cast<std::int64_t>(static_cast<double>(deltaTime.count()) * timeScale_.load())
                );

        // Update timing
        currentTime_ += scaledDelta;
        totalElapsedTime_ += scaledDelta;
        lastUpdateTime_ = backend_.now();

        // Process timers
        const auto timersProcessed = processTimers(currentTime_);

        // Build result
        const TimelineUpdateResult result{
                        .actualDeltaTime = deltaTime,
                        .scaledDeltaTime = scaledDelta,
                        .fixedStepsExecuted = 0,
                        // TimestepManager handles this
                        .timersProcessed = timersProcessed,
                        .wasThrottled = false,
                        .budgetExceeded = false
                };

        // Update frame history
        recordFrameHistory(result);

        // Update statistics
        updateStatistics(result);

        frameNumber_++;

        return result;
    }

    SafeTimerHandle GameTimeline::createTimer(
            const Duration duration,
            TimerCallback callback,
            const bool recurring
            ) {
        std::lock_guard lock(timerMutex_);

        const TimerID id = nextTimerId_++;
        const Duration fireTime = currentTime_ + duration;

        GameTimer timer{
                        .id = id,
                        .fireTime = fireTime,
                        .duration = duration,
                        .callback = std::move(callback),
                        .recurring = recurring,
                        .isPaused = false,
                        .pauseTime = std::nullopt,
                        .generation = 1
                };

        const TimerHandle handle{id, timer.generation};
        timers_[id] = std::move(timer);

        return SafeTimerHandle{handle, nullptr};
    }

    bool GameTimeline::cancelTimer(const SafeTimerHandle& handle) {
        std::lock_guard lock(timerMutex_);

        if (const auto it = timers_.find(handle.getId()); it != timers_.end() && it->second.generation == handle.
            getGeneration()) {
            timers_.erase(it);
            return true;
        }

        return false;
    }

    void GameTimeline::clearAllTimers() {
        std::lock_guard lock(timerMutex_);
        timers_.clear();
    }

    std::size_t GameTimeline::getActiveTimerCount() const {
        std::lock_guard lock(timerMutex_);
        return timers_.size();
    }

    GameTimeline::GameTimelineState GameTimeline::saveState() const {
        return GameTimelineState{
                        .currentTime = currentTime_,
                        .totalElapsedTime = totalElapsedTime_,
                        .timeScale = timeScale_.load(),
                        .isPaused = isPaused_.load(),
                        .frameNumber = frameNumber_,
                        .timestepMode = config_.timestepConfig.mode
                };
    }

    void GameTimeline::loadState(const GameTimelineState& state) {
        currentTime_ = state.currentTime;
        totalElapsedTime_ = state.totalElapsedTime;
        timeScale_ = state.timeScale;
        isPaused_ = state.isPaused;
        frameNumber_ = state.frameNumber;
        config_.timestepConfig.mode = state.timestepMode;

        // Clear history when loading state
        {
            std::lock_guard lock(historyMutex_);
            frameHistory_.clear();
        }
    }

    void GameTimeline::startRecording(const std::size_t maxFrames) {
        recordingEnabled_ = true;
        maxRecordedFrames_ = maxFrames;
        recordedFrames_.clear();
        recordedFrames_.reserve(maxFrames);
    }

    void GameTimeline::stopRecording() {
        recordingEnabled_ = false;
    }

    std::vector<GameTimeline::FrameRecord> GameTimeline::getFrameHistory(const std::size_t count) const {
        std::lock_guard lock(historyMutex_);

        std::vector<FrameRecord> result;
        const std::size_t available = std::min(count, frameHistory_.size());

        result.reserve(available);
        for (std::size_t i = 0; i < available; ++i) {
            result.push_back(frameHistory_[frameHistory_.size() - 1 - i]);
        }

        return result;
    }

    bool GameTimeline::waitForTime(const Duration targetTime, const Duration timeout) {
        const auto startWait = backend_.now();
        const auto timeoutTime = startWait + timeout;

        while (currentTime_ < targetTime) {
            if (backend_.now() >= timeoutTime) {
                return false; // Timeout
            }

            if (isPaused_) {
                // Wait for unpause
                std::unique_lock lock(pauseMutex_);
                pauseCondition_.wait_for(
                        lock,
                        std::chrono::milliseconds(10),
                        [this] { return !isPaused_.load(); }
                        );
            }

            backend_.sleep(Duration(1000)); // 1ms sleep
        }

        return true;
    }

    bool GameTimeline::updateConfig(const TimelineConfigBase& config) {
        config_ = config;
        return true;
    }

    bool GameTimeline::setTimestepMode(const TimestepMode mode) {
        config_.timestepConfig.mode = mode;
        return true;
    }

    void GameTimeline::onPause() {
        // Pause all timers
        std::lock_guard lock(timerMutex_);
        for (auto& timer : timers_ | std::views::values) {
            timer.isPaused = true;
            timer.pauseTime = currentTime_;
        }
    }

    void GameTimeline::onResume() {
        // Resume all timers
        std::lock_guard lock(timerMutex_);
        for (auto& timer : timers_ | std::views::values) {
            if (timer.isPaused && timer.pauseTime) {
                const Duration pauseDuration = currentTime_ - *timer.pauseTime;
                timer.fireTime += pauseDuration;
                timer.isPaused = false;
                timer.pauseTime = std::nullopt;
            }
        }

        // Notify waiting threads
        pauseCondition_.notify_all();
    }

    std::uint32_t GameTimeline::processTimers(const Duration currentTime) {
        std::lock_guard lock(timerMutex_);
        std::uint32_t processed = 0;

        std::vector<TimerCallback> toExecute;
        std::vector<TimerID> toRemove;

        for (auto& [id, timer] : timers_) {
            if (!timer.isPaused && currentTime >= timer.fireTime) {
                toExecute.push_back(timer.callback);

                if (timer.recurring) {
                    timer.fireTime = currentTime + timer.duration;
                } else {
                    toRemove.push_back(id);
                }

                processed++;
            }
        }

        // Remove one-shot timers
        for (TimerID id : toRemove) {
            timers_.erase(id);
        }

        // Execute callbacks outside lock to avoid deadlock
        lock.~lock_guard(); // Explicitly unlock
        for (const auto& callback : toExecute) {
            callback(TimerHandle{});
        }

        return processed;
    }

    void GameTimeline::recordFrameHistory(const TimelineUpdateResult& result) {
        std::lock_guard lock(historyMutex_);

        const FrameRecord record{
                        .frameNumber = frameNumber_,
                        .deltaTime = result.actualDeltaTime,
                        .scaledDeltaTime = result.scaledDeltaTime,
                        .timeScale = timeScale_.load(),
                        .wasPaused = false,
                        .timestamp = backend_.now()
                };

        frameHistory_.push_back(record);

        // Limit history size
        if (frameHistory_.size() > MAX_FRAME_HISTORY) {
            frameHistory_.pop_front();
        }

        // Recording support
        if (recordingEnabled_ && recordedFrames_.size() < maxRecordedFrames_) {
            recordedFrames_.push_back(saveState());
        }
    }

    void GameTimeline::updateStatistics(const TimelineUpdateResult& result) {
        frameStats_.update(result.actualDeltaTime, backend_.now());
    }
} // namespace engine::time
