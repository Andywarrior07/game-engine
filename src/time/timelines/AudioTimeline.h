/**
 * @file AudioTimeline.h
 * @brief Timeline for audio synchronization and playback
 * @details Provides a specialized timeline for audio systems, handling
 *          latency compensation, tempo synchronization, and music timing.
 *          Integrates with audio engines for precise sound effect coordination.
 *
 * @author Andres Guerrero
 * @date 23-09-2025
 */

#pragma once

#include "../core/Timeline.h"
#include "../core/ITimeBackend.h"

#include <atomic>
#include <map>

namespace engine::time {
    /**
     * @brief Audio timeline for sound synchronization
     * @details Manages audio playback timing with support for latency compensation,
     *          beat synchronization, and music tempo changes. Designed to integrate
     *          with audio engines while maintaining frame-independent timing.
     *
     * Key features:
     * - Audio latency compensation
     * - Beat and tempo tracking
     * - Music synchronization
     * - Sample-accurate timing
     * - DSP clock integration
     * - MIDI timing support
     *
     * Use cases:
     * - Music playback and looping
     * - Sound effect timing
     * - Beat-synchronized gameplay
     * - Audio fade transitions
     * - Interactive music systems
     * - Rhythm game mechanics
     */
    class AudioTimeline final : public Timeline {
    public:
        /**
         * @brief Musical time representation
         */
        struct MusicalTime {
            std::uint32_t bar{1}; ///< Current bar (measure)
            std::uint8_t beat{1}; ///< Current beat in bar
            std::uint16_t tick{0}; ///< Sub-beat tick (0-479)
            float beatFraction{0.0f}; ///< Fractional beat position
        };

        /**
         * @brief Audio timing configuration
         */
        struct AudioConfig {
            std::uint32_t sampleRate{48000}; ///< Audio sample rate
            std::uint32_t bufferSize{512}; ///< Audio buffer size
            Duration latency{Duration(10000)}; ///< System audio latency
            float tempo{120.0f}; ///< BPM (beats per minute)
            std::uint8_t beatsPerBar{4}; ///< Time signature numerator
            std::uint8_t beatUnit{4}; ///< Time signature denominator
            bool quantizeEvents{false}; ///< Quantize to beat grid
            std::uint8_t quantizeResolution{16}; ///< Quantize to 1/16 notes
        };

        /**
         * @brief Audio event for scheduling
         */
        struct AudioEvent {
            std::string name; ///< Event identifier
            Duration triggerTime; ///< When to trigger
            std::function<void()> callback; ///< Event callback
            bool quantized{false}; ///< Quantize to beat
            std::uint8_t priority{128}; ///< Execution priority
        };

        /**
         * @brief Constructor
         * @param backend Time backend for platform queries
         * @param name Timeline identifier
         */
        explicit AudioTimeline(
                ITimeBackend& backend,
                const std::string& name = "AudioTime"
                );

        /**
         * @brief Destructor
         */
        ~AudioTimeline() override;

        // =============================================================================
        // Timeline Interface Implementation
        // =============================================================================

        /**
         * @brief Initialize audio timeline
         * @param config Timeline configuration
         * @return True if initialization successful
         */
        bool initialize(const TimelineConfigBase& config) override;

        /**
         * @brief Update audio timeline
         * @param deltaTime Time since last update (usually audio callback interval)
         * @return Update result
         */
        TimelineUpdateResult update(Duration deltaTime) override;

        /**
         * @brief Set time scale (disabled for audio timeline)
         * @param scale Ignored (audio always 1:1)
         * @return Always returns false
         */
        bool setTimeScale(TimeScale scale) override {
            // Audio timeline cannot be scaled (would affect pitch)
            return false;
        }

        // =============================================================================
        // Audio-Specific Features
        // =============================================================================

        /**
         * @brief Set audio configuration
         * @param config Audio timing configuration
         */
        void setAudioConfig(const AudioConfig& config) {
            audioConfig_ = config;
            updateBeatDuration();
        }

        /**
         * @brief Get audio configuration
         */
        [[nodiscard]] const AudioConfig& getAudioConfig() const noexcept {
            return audioConfig_;
        }

        /**
         * @brief Set tempo (BPM)
         * @param bpm Beats per minute
         */
        void setTempo(float bpm);

        /**
         * @brief Get current tempo
         */
        [[nodiscard]] float getTempo() const noexcept {
            return audioConfig_.tempo;
        }

        /**
         * @brief Set time signature
         * @param numerator Beats per bar
         * @param denominator Beat unit (4 = quarter note)
         */
        void setTimeSignature(std::uint8_t numerator, std::uint8_t denominator);

        /**
         * @brief Get current musical time
         */
        [[nodiscard]] MusicalTime getMusicalTime() const noexcept {
            return currentMusicalTime_;
        }

        /**
         * @brief Get time to next beat
         */
        [[nodiscard]] Duration getTimeToNextBeat() const noexcept;

        /**
         * @brief Get time to next bar
         */
        [[nodiscard]] Duration getTimeToNextBar() const noexcept;

        /**
         * @brief Schedule audio event
         * @param event Audio event to schedule
         * @return Event ID for cancellation
         */
        std::uint32_t scheduleAudioEvent(const AudioEvent& event);

        /**
         * @brief Cancel audio event
         * @param eventId Event ID to cancel
         */
        void cancelAudioEvent(const std::uint32_t eventId) {
            std::lock_guard lock(eventMutex_);
            audioEvents_.erase(eventId);
        }

        /**
         * @brief Get DSP time (sample-accurate time)
         */
        [[nodiscard]] double getDSPTime() const noexcept {
            return dspTime_;
        }

        /**
         * @brief Get samples processed
         */
        [[nodiscard]] std::uint64_t getSamplesProcessed() const noexcept {
            return samplesProcessed_;
        }

        /**
         * @brief Sync to external clock (e.g., DAW)
         * @param externalTime External time reference
         */
        void syncToExternalClock(const Duration externalTime) {
            const Duration offset = externalTime - currentTime_;
            clockOffset_ = offset;
            lastSyncTime_ = backend_.now();
        }

        /**
         * @brief Start playback
         */
        void startPlayback() {
            isPlaying_ = true;
            playbackStartTime_ = backend_.now();
        }

        /**
         * @brief Stop playback
         */
        void stopPlayback() {
            isPlaying_ = false;
        }

        /**
         * @brief Check if playing
         */
        [[nodiscard]] bool isPlaying() const noexcept {
            return isPlaying_;
        }

        /**
         * @brief Jump to bar
         * @param bar Target bar number
         */
        void jumpToBar(std::uint32_t bar);

        /**
         * @brief Register beat callback
         * @param callback Function called on each beat
         */
        void setBeatCallback(std::function<void(std::uint32_t)> callback) {
            beatCallback_ = std::move(callback);
        }

        /**
         * @brief Register bar callback
         * @param callback Function called on each bar
         */
        void setBarCallback(std::function<void(std::uint32_t)> callback) {
            barCallback_ = std::move(callback);
        }

        // =============================================================================
        // Timer Management
        // =============================================================================

        SafeTimerHandle createTimer(
                Duration duration,
                TimerCallback callback,
                bool recurring
                ) override;

        bool cancelTimer(const SafeTimerHandle& handle) override {
            std::lock_guard lock(timerMutex_);
            return timers_.erase(handle.getId()) > 0;
        }

        void clearAllTimers() override {
            std::lock_guard lock(timerMutex_);
            timers_.clear();
        }

        [[nodiscard]] std::size_t getActiveTimerCount() const override {
            std::lock_guard lock(timerMutex_);
            return timers_.size();
        }

        // Configuration methods
        [[nodiscard]] const TimelineConfigBase& getConfig() const override {
            return config_;
        }

        bool updateConfig(const TimelineConfigBase& config) override {
            config_ = config;
            // Force audio-specific settings
            config_.timestepConfig.mode = TimestepMode::VARIABLE;
            config_.initialScale = 1.0;
            return true;
        }

        [[nodiscard]] const TimestepConfig& getTimestepConfig() const override {
            return config_.timestepConfig;
        }

        bool setTimestepMode(const TimestepMode mode) override {
            // Audio timeline must use variable timestep
            return mode == TimestepMode::VARIABLE;
        }

        bool waitForTime(Duration targetTime, Duration timeout) override;

    private:
        // Private types
        struct AudioTimer {
            TimerID id;
            Duration fireTime;
            Duration duration;
            TimerCallback callback;
            bool recurring;
            TimerGeneration generation;
        };

        // Private members
        ITimeBackend& backend_;
        TimelineConfigBase config_;
        AudioConfig audioConfig_;

        // Musical time
        MusicalTime currentMusicalTime_;
        Duration beatDuration_;
        Duration lastBeatTime_{Duration::zero()};

        // Audio timing
        std::uint64_t samplesProcessed_;
        double dspTime_{0.0};
        TimeStamp audioStartTime_;
        TimeStamp playbackStartTime_;
        bool isPlaying_;

        // Synchronization
        Duration clockOffset_{Duration::zero()};
        TimeStamp lastSyncTime_;

        // Events
        mutable std::mutex eventMutex_;
        std::map<std::uint32_t, AudioEvent> audioEvents_;
        std::atomic<std::uint32_t> nextEventId_{1};

        // Timers
        mutable std::mutex timerMutex_;
        std::unordered_map<TimerID, AudioTimer> timers_;
        TimerID nextTimerId_{1};

        // Callbacks
        std::function<void(std::uint32_t)> beatCallback_;
        std::function<void(std::uint32_t)> barCallback_;

        // Private methods
        void updateBeatDuration() {
            // Calculate beat duration from tempo
            const float beatsPerSecond = audioConfig_.tempo / 60.0f;
            const float secondsPerBeat = 1.0f / beatsPerSecond;
            beatDuration_ = Duration(static_cast<std::int64_t>(secondsPerBeat * 1000000));
        }

        void updateMusicalTime(Duration deltaTime);

        Duration quantizeToGrid(const Duration time) const {
            // Quantize to nearest grid position
            const Duration gridSize = beatDuration_ / audioConfig_.quantizeResolution;
            const std::int64_t gridUnits = time.count() / gridSize.count();
            return gridSize * gridUnits;
        }

        std::uint32_t processAudioEvents(Duration currentTime);

        std::uint32_t processTimers(Duration currentTime);
    };
} // namespace engine::time
