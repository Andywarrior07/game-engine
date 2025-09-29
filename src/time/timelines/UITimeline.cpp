//
// Created by Andres Guerrero on 22-09-25.
//

#include "UITimeline.h"

namespace engine::time {
    UITimeline::~UITimeline() {
        Timeline::stop();
    }

    bool UITimeline::initialize(const TimelineConfigBase& config) {
        if (state_ != TimelineState::UNINITIALIZED) {
            return false;
        }

        config_ = config;

        // UI timeline typically uses variable timestep
        config_.timestepConfig.mode = TimestepMode::VARIABLE;

        // Set initial state
        timeScale_ = config_.initialScale;
        isPaused_ = config_.startPaused;

        // Initialize timing
        lastUpdateTime_ = backend_.now();

        // Pre-allocate storage
        // No necesitamos el .reserve ya que es un map
        // animations_.reserve(32); // Typical UI animation count
        timers_.reserve(config_.maxEventQueue);

        state_ = TimelineState::INITIALIZED;

        if (config_.autoStart) {
            return start();
        }

        return true;
    }

    TimelineUpdateResult UITimeline::update(const Duration deltaTime) {
        if (state_ != TimelineState::RUNNING) {
            return TimelineUpdateResult{};
        }

        // UI timeline can update even when "paused" (for menu animations)
        // but respects its own pause for specific UI states
        // TODO: Comenté !allowUpdateWithPaused_ pq siempre es false, revisar esto
        if (isPaused_ /*&& !allowUpdateWhilePaused_*/) {
            return TimelineUpdateResult{};
        }

        // Apply time scaling for UI effects
        const auto scaledDelta = Duration(
                static_cast<std::int64_t>(deltaTime.count() * timeScale_.load())
                );

        // Clamp to reasonable UI update rate (no need for high precision)
        const Duration clampedDelta = std::min(scaledDelta, Duration(50000)); // 50ms max

        // Update timing
        currentTime_ += clampedDelta;
        totalElapsedTime_ += clampedDelta;
        lastUpdateTime_ = backend_.now();

        // Update animations
        // TODO: Revisar esto
        const auto animationsProcessed = updateAnimations(clampedDelta);

        // Update transitions
        // TODO: Revisar esto
        const bool transitionCompleted = updateTransition(clampedDelta);

        // Process timers
        const auto timersProcessed = processTimers(currentTime_);

        return TimelineUpdateResult{
                        .actualDeltaTime = deltaTime,
                        .scaledDeltaTime = scaledDelta,
                        .fixedStepsExecuted = 0,
                        .timersProcessed = timersProcessed,
                        .wasThrottled = false,
                        .budgetExceeded = false
                };
    }

    std::uint32_t UITimeline::startAnimation(
            const std::string& name,
            const Duration duration,
            UIAnimation::AnimationCallback updateCallback,
            const EasingType easing
            ) {
        std::lock_guard lock(animationMutex_);

        const std::uint32_t id = nextAnimationId_++;

        animations_[id] = UIAnimation{
                        .name = name,
                        .duration = duration,
                        .elapsed = Duration::zero(),
                        .easing = easing,
                        .updateCallback = std::move(updateCallback),
                        .completionCallback = nullptr,
                        .loop = false,
                        .reverse = false,
                        .paused = false,
                        .speed = 1.0f
                };

        return id;
    }

    void UITimeline::stopAnimation(const std::uint32_t id, const bool complete) {
        std::lock_guard lock(animationMutex_);

        if (const auto it = animations_.find(id); it != animations_.end()) {
            if (complete && it->second.updateCallback) {
                it->second.updateCallback(1.0f);
            }

            if (it->second.completionCallback) {
                it->second.completionCallback();
            }

            animations_.erase(it);
        }
    }

    void UITimeline::setAnimationPaused(const std::uint32_t id, const bool paused) {
        std::lock_guard lock(animationMutex_);

        const auto it = animations_.find(id);

        if (it == animations_.end())
            return;

        it->second.paused = paused;
    }

    void UITimeline::setAnimationSpeed(const std::uint32_t id, const float speed) {
        std::lock_guard lock(animationMutex_);

        const auto it = animations_.find(id);

        if (it == animations_.end())
            return;

        it->second.speed = std::max(0.0f, speed);
    }

    void UITimeline::startTransition(
            const std::string& fromState,
            const std::string& toState,
            const Duration duration,
            std::function<void(float)> transitionCallback,
            const EasingType easing
            ) {
        std::lock_guard lock(transitionMutex_);

        activeTransition_ = UITransition{
                        .fromState = fromState,
                        .toState = toState,
                        .duration = duration,
                        .elapsed = Duration::zero(),
                        .easing = easing,
                        .transitionCallback = std::move(transitionCallback),
                        .completionCallback = nullptr
                };
    }

    SafeTimerHandle UITimeline::createTimer(const Duration duration, TimerCallback callback, const bool recurring) {
        std::lock_guard lock(timerMutex_);

        const TimerID id = nextTimerId_++;
        const Duration fireTime = currentTime_ + duration;

        UITimer timer{
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

    float UITimeline::applyEasing(float t, const EasingType type) {
        t = std::clamp(t, 0.0f, 1.0f);

        switch (type) {
            case EasingType::LINEAR:
                return t;

            case EasingType::EASE_IN_QUAD:
                return t * t;

            case EasingType::EASE_OUT_QUAD:
                return t * (2.0f - t);

            case EasingType::EASE_IN_OUT_QUAD:
                return t < 0.5f ? 2.0f * t * t : 1.0f - pow(-2.0f * t + 2.0f, 2.0f) / 2.0f;

            case EasingType::EASE_IN_CUBIC:
                return t * t * t;

            case EasingType::EASE_OUT_CUBIC:
                return 1.0f - pow(1.0f - t, 3.0f);

            case EasingType::EASE_IN_OUT_CUBIC:
                return t < 0.5f ? 4.0f * t * t * t : 1.0f - pow(-2.0f * t + 2.0f, 3.0f) / 2.0f;

            case EasingType::EASE_IN_EXPO:
                return t == 0.0f ? 0.0f : pow(2.0f, 10.0f * t - 10.0f);

            case EasingType::EASE_OUT_EXPO:
                return t == 1.0f ? 1.0f : 1.0f - pow(2.0f, -10.0f * t);

            case EasingType::EASE_IN_OUT_EXPO:
                if (t == 0.0f)
                    return 0.0f;
                if (t == 1.0f)
                    return 1.0f;
                return t < 0.5f
                        ? pow(2.0f, 20.0f * t - 10.0f) / 2.0f
                        : (2.0f - pow(2.0f, -20.0f * t + 10.0f)) / 2.0f;

            case EasingType::EASE_IN_BACK: {
                constexpr float c1 = 1.70158f;
                constexpr float c3 = c1 + 1.0f;
                return c3 * t * t * t - c1 * t * t;
            }

            case EasingType::EASE_OUT_BACK: {
                constexpr float c1 = 1.70158f;
                constexpr float c3 = c1 + 1.0f;
                return 1.0f + c3 * pow(t - 1.0f, 3.0f) + c1 * pow(t - 1.0f, 2.0f);
            }

            case EasingType::EASE_IN_OUT_BACK: {
                constexpr float c1 = 1.70158f;
                constexpr float c2 = c1 * 1.525f;
                return t < 0.5f
                        ? (pow(2.0f * t, 2.0f) * ((c2 + 1.0f) * 2.0f * t - c2)) / 2.0f
                        : (pow(2.0f * t - 2.0f, 2.0f) * ((c2 + 1.0f) * (t * 2.0f - 2.0f) + c2) + 2.0f) / 2.0f;
            }

            case EasingType::EASE_IN_ELASTIC: {
                constexpr float c4 = (2.0f * 3.14159265f) / 3.0f;
                return t == 0.0f
                        ? 0.0f
                        : t == 1.0f
                        ? 1.0f
                        : -pow(2.0f, 10.0f * t - 10.0f) * sin((t * 10.0f - 10.75f) * c4);
            }

            case EasingType::EASE_OUT_ELASTIC: {
                constexpr float c4 = (2.0f * 3.14159265f) / 3.0f;
                return t == 0.0f
                        ? 0.0f
                        : t == 1.0f
                        ? 1.0f
                        : pow(2.0f, -10.0f * t) * sin((t * 10.0f - 0.75f) * c4) + 1.0f;
            }

            case EasingType::EASE_IN_OUT_ELASTIC: {
                constexpr float c5 = (2.0f * 3.14159265f) / 4.5f;

                if (t == 0.0f)
                    return 0.0f;

                if (t == 1.0f)
                    return 1.0f;

                return t < 0.5f
                        ? -(pow(2.0f, 20.0f * t - 10.0f) * sin((20.0f * t - 11.125f) * c5)) / 2.0f
                        : (pow(2.0f, -20.0f * t + 10.0f) * sin((20.0f * t - 11.125f) * c5)) / 2.0f + 1.0f;
            }

            case EasingType::EASE_OUT_BOUNCE:
                return easeOutBounce(t);

            case EasingType::EASE_IN_BOUNCE:
                return 1.0f - easeOutBounce(1.0f - t);

            case EasingType::EASE_IN_OUT_BOUNCE:
                return t < 0.5f
                        ? (1.0f - easeOutBounce(1.0f - 2.0f * t)) / 2.0f
                        : (1.0f + easeOutBounce(2.0f * t - 1.0f)) / 2.0f;

            default:
                return t;
        }
    }

    bool UITimeline::waitForTime(const Duration targetTime, const Duration timeout) {
        const auto startWait = backend_.now();
        const auto timeoutTime = startWait + timeout;

        while (currentTime_ < targetTime) {
            if (backend_.now() >= timeoutTime) {
                return false;
            }
            backend_.sleep(Duration(1000)); // 1ms
        }

        return true;
    }

    float UITimeline::easeOutBounce(float t) {
        constexpr float n1 = 7.5625f;
        constexpr float d1 = 2.75f;

        if (t < 1.0f / d1) {
            return n1 * t * t;
        }

        if (t < 2.0f / d1) {
            t -= 1.5f / d1;
            return n1 * t * t + 0.75f;
        }

        if (t < 2.5f / d1) {
            t -= 2.25f / d1;
            return n1 * t * t + 0.9375f;
        }

        t -= 2.625f / d1;
        return n1 * t * t + 0.984375f;
    }

    std::uint32_t UITimeline::updateAnimations(const Duration deltaTime) {
        std::lock_guard lock(animationMutex_);
        std::uint32_t completed = 0;

        std::vector<std::uint32_t> toRemove;

        for (auto& [id, anim] : animations_) {
            if (anim.paused)
                continue;

            anim.elapsed += Duration(
                    static_cast<std::int64_t>(
                        static_cast<float>(deltaTime.count()) * anim.speed)
                    );

            float progress = static_cast<float>(anim.elapsed.count()) /
                    static_cast<float>(anim.duration.count());

            if (progress >= 1.0f) {
                if (anim.loop) {
                    anim.elapsed = Duration::zero();
                    progress = 0.0f;
                } else {
                    progress = 1.0f;
                    toRemove.push_back(id);
                    completed++;
                }
            }

            if (anim.reverse) {
                progress = 1.0f - progress;
            }

            const float easedProgress = applyEasing(progress, anim.easing);

            if (anim.updateCallback) {
                anim.updateCallback(easedProgress);
            }

            if (progress >= 1.0f && anim.completionCallback) {
                anim.completionCallback();
            }
        }

        for (auto id : toRemove) {
            animations_.erase(id);
        }

        return completed;
    }

    bool UITimeline::updateTransition(const Duration deltaTime) {
        std::lock_guard lock(transitionMutex_);

        if (!activeTransition_) {
            return false;
        }

        activeTransition_->elapsed += deltaTime;

        float progress = static_cast<float>(activeTransition_->elapsed.count()) /
                static_cast<float>(activeTransition_->duration.count());

        if (progress >= 1.0f) {
            // TODO: Revisar esto
            progress = 1.0f;

            if (activeTransition_->transitionCallback) {
                const float eased = applyEasing(1.0f, activeTransition_->easing);
                activeTransition_->transitionCallback(eased);
            }

            if (activeTransition_->completionCallback) {
                activeTransition_->completionCallback();
            }

            activeTransition_ = std::nullopt;
            return true;
        }

        const float easedProgress = applyEasing(progress, activeTransition_->easing);

        if (activeTransition_->transitionCallback) {
            activeTransition_->transitionCallback(easedProgress);
        }

        return false;
    }

    std::uint32_t UITimeline::processTimers(const Duration currentTime) {
        std::lock_guard lock(timerMutex_);
        std::uint32_t processed = 0;

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

                processed++;
            }
        }

        for (TimerID id : toRemove) {
            timers_.erase(id);
        }

        // Execute callbacks outside lock
        for (const auto& callback : toExecute) {
            callback(TimerHandle{});
        }

        return processed;
    }
} // namespace engine::time
