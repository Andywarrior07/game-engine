//
// Created by Andres Guerrero on 23-09-25.
//

#include "AudioTimeline.h"

namespace engine::time {
    AudioTimeline::AudioTimeline(ITimeBackend& backend, const std::string& name) :
        Timeline(TimelineType::AUDIO_TIME, name)
        , backend_(backend)
        , beatDuration_(Duration(500000)) // 120 BPM default
        , samplesProcessed_(0)
        , isPlaying_(false) {

        updateBeatDuration();
    }

    AudioTimeline::~AudioTimeline() {
        Timeline::stop();
    }

    bool AudioTimeline::initialize(const TimelineConfigBase& config) {
        if (state_ != TimelineState::UNINITIALIZED) {
            return false;
        }

        config_ = config;

        // Audio timeline uses variable timestep synced to audio callback
        config_.timestepConfig.mode = TimestepMode::VARIABLE;

        // Audio timeline cannot be time-scaled (would affect pitch)
        config_.initialScale = 1.0;
        timeScale_ = 1.0;

        // Set initial state
        isPaused_ = config_.startPaused;

        // Initialize timing
        lastUpdateTime_ = backend_.now();
        audioStartTime_ = lastUpdateTime_;

        // Pre-allocate storage
        timers_.reserve(config_.maxEventQueue);

        state_ = TimelineState::INITIALIZED;

        if (config_.autoStart) {
            return start();
        }

        return true;
    }

    TimelineUpdateResult AudioTimeline::update(const Duration deltaTime) {
        if (state_ != TimelineState::RUNNING || isPaused_) {
            return TimelineUpdateResult{};
        }

        // Audio always runs at 1:1 time scale
        // deltaTime typically comes from audio callback interval

        // Update timing with latency compensation
        const Duration compensatedDelta = deltaTime - audioConfig_.latency;
        currentTime_ += compensatedDelta;
        totalElapsedTime_ += compensatedDelta;

        // Update musical time
        updateMusicalTime(compensatedDelta);

        // Process scheduled audio events
        const auto eventsProcessed = processAudioEvents(currentTime_);

        // Process timers
        const auto timersProcessed = processTimers(currentTime_);

        // Update sample count
        const auto samplesInDelta =
                static_cast<std::uint32_t>(toSeconds(deltaTime) * audioConfig_.sampleRate);
        samplesProcessed_ += samplesInDelta;

        // Update DSP time
        dspTime_ = static_cast<double>(samplesProcessed_) / audioConfig_.sampleRate;

        lastUpdateTime_ = backend_.now();

        return TimelineUpdateResult{
                        .actualDeltaTime = deltaTime,
                        .scaledDeltaTime = deltaTime,
                        // No scaling for audio
                        .fixedStepsExecuted = 0,
                        .timersProcessed = timersProcessed + eventsProcessed,
                        .wasThrottled = false,
                        .budgetExceeded = false
                };
    }

    void AudioTimeline::setTempo(const float bpm) {
        audioConfig_.tempo = std::clamp(bpm, 20.0f, 999.0f);
        updateBeatDuration();
    }

    void AudioTimeline::setTimeSignature(const std::uint8_t numerator, const std::uint8_t denominator) {
        audioConfig_.beatsPerBar = numerator;
        audioConfig_.beatUnit = denominator;
        updateBeatDuration();
    }

    Duration AudioTimeline::getTimeToNextBeat() const noexcept {
        const float beatProgress = currentMusicalTime_.beatFraction;
        const float remaining = 1.0f - beatProgress;

        return Duration(static_cast<std::int64_t>(static_cast<float>(beatDuration_.count()) * remaining));
    }

    Duration AudioTimeline::getTimeToNextBar() const noexcept {
        const std::uint8_t beatsRemaining =
                audioConfig_.beatsPerBar - currentMusicalTime_.beat + 1;

        return Duration(beatDuration_.count() * beatsRemaining) -
                Duration(
                        static_cast<std::int64_t>(
                            static_cast<float>(beatDuration_.count()) * currentMusicalTime_.beatFraction)
                        );
    }

    std::uint32_t AudioTimeline::scheduleAudioEvent(const AudioEvent& event) {
        std::lock_guard lock(eventMutex_);

        const std::uint32_t id = nextEventId_++;

        Duration scheduleTime = event.triggerTime;

        // Quantize if requested
        if (event.quantized && audioConfig_.quantizeEvents) {
            scheduleTime = quantizeToGrid(scheduleTime);
        }

        audioEvents_[id] = event;
        audioEvents_[id].triggerTime = scheduleTime;

        return id;
    }

    void AudioTimeline::jumpToBar(const std::uint32_t bar) {
        currentMusicalTime_.bar = bar;
        currentMusicalTime_.beat = 1;
        currentMusicalTime_.tick = 0;
        currentMusicalTime_.beatFraction = 0.0f;

        // Calculate time position
        const Duration barDuration = beatDuration_ * audioConfig_.beatsPerBar;
        currentTime_ = barDuration * (bar - 1);
    }

    SafeTimerHandle AudioTimeline::createTimer(const Duration duration, TimerCallback callback, const bool recurring) {
        std::lock_guard lock(timerMutex_);

        const TimerID id = nextTimerId_++;
        const Duration fireTime = currentTime_ + duration;

        AudioTimer timer{
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

    bool AudioTimeline::waitForTime(const Duration targetTime, const Duration timeout) {
        const auto startWait = backend_.now();
        const auto timeoutTime = startWait + timeout;

        while (currentTime_ < targetTime) {
            if (backend_.now() >= timeoutTime) {
                return false;
            }
            backend_.sleep(Duration(100)); // 100μs
        }

        return true;
    }

    void AudioTimeline::updateMusicalTime(const Duration deltaTime) {
        // Track beat position
        lastBeatTime_ += deltaTime;

        while (lastBeatTime_ >= beatDuration_) {
            lastBeatTime_ -= beatDuration_;

            // Advance beat
            currentMusicalTime_.beat++;
            if (currentMusicalTime_.beat > audioConfig_.beatsPerBar) {
                currentMusicalTime_.beat = 1;
                currentMusicalTime_.bar++;

                if (barCallback_) {
                    barCallback_(currentMusicalTime_.bar);
                }
            }

            if (beatCallback_) {
                beatCallback_(currentMusicalTime_.beat);
            }
        }

        // Calculate beat fraction
        currentMusicalTime_.beatFraction =
                static_cast<float>(lastBeatTime_.count()) /
                static_cast<float>(beatDuration_.count());

        // Calculate tick (480 ticks per quarter note is MIDI standard)
        currentMusicalTime_.tick = static_cast<std::uint16_t>(
            currentMusicalTime_.beatFraction * 480);
    }

    std::uint32_t AudioTimeline::processAudioEvents(const Duration currentTime) {
        std::lock_guard lock(eventMutex_);
        std::uint32_t processed = 0;

        std::vector<std::function<void()>> toExecute;
        std::vector<std::uint32_t> toRemove;

        for (auto& [id, event] : audioEvents_) {
            if (currentTime >= event.triggerTime) {
                toExecute.push_back(event.callback);
                toRemove.push_back(id);
                processed++;
            }
        }

        for (auto id : toRemove) {
            audioEvents_.erase(id);
        }

        // Execute callbacks outside lock
        for (const auto& callback : toExecute) {
            if (callback)
                callback();
        }

        return processed;
    }

    std::uint32_t AudioTimeline::processTimers(const Duration currentTime) {
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
