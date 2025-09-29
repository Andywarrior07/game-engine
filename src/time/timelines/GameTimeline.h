/**
 * @file GameTimeline.h
 * @brief Primary timeline for game logic with full control
 * @details Provides the main timeline for gameplay systems with support for
 *          pausing, time scaling, and various timestep modes. This is the
 *          timeline most game systems will interact with.
 *
 * @author Andres Guerrero
 * @date 2025-09-22
 */

#pragma once

#include "../core/Timeline.h"
#include "../core/ITimeBackend.h"
#include "../profiling/TimeStats.h"

#include <deque>
#include <unordered_map>
#include <mutex>
#include <condition_variable>
#include <atomic>

namespace engine::time {
    /**
     * @brief Main gameplay timeline with comprehensive control
     * @details The primary timeline for game logic, supporting pause, resume,
     *          time scaling, and frame interpolation for smooth visuals.
     *          Does NOT handle timestep strategies - that's TimestepManager's job.
     *
     * Key features:
     * - Full pause/resume support
     * - Time scaling (slow-mo, fast-forward)
     * - Frame interpolation for smooth rendering
     * - Save/load state support
     * - Replay recording capability
     * - Timer management
     *
     * Use cases:
     * - Character movement and animation
     * - Game logic and AI updates
     * - Gameplay timer systems
     * - Special effects and particles
     * - World simulation
     *
     * @note This timeline receives already-processed delta time from TimestepManager.
     *       It focuses on game-specific logic, not timestep strategies.
     */
    class GameTimeline final : public Timeline {
    public:
        /**
         * @brief Game timeline state for save/load
         */
        struct GameTimelineState {
            Duration currentTime; ///< Current game time
            Duration totalElapsedTime; ///< Total elapsed time
            TimeScale timeScale; ///< Current time scale
            bool isPaused; ///< Pause state
            std::uint64_t frameNumber; ///< Current frame number
            TimestepMode timestepMode; ///< Active timestep mode
        };

        /**
         * @brief Frame record for history tracking
         */
        struct FrameRecord {
            std::uint64_t frameNumber{}; ///< Frame number
            Duration deltaTime{}; ///< Raw delta time
            Duration scaledDeltaTime{}; ///< Scaled delta time
            TimeScale timeScale{}; ///< Time scale at frame
            bool wasPaused{}; ///< Was paused this frame
            TimeStamp timestamp; ///< Wall clock timestamp
        };

        /**
         * @brief Constructor
         * @param backend Time backend for platform queries
         * @param name Timeline identifier
         */
        explicit GameTimeline(
                ITimeBackend& backend,
                const std::string& name = "GameTime"
                );

        /**
         * @brief Destructor
         */
        ~GameTimeline() override;

        // Disable copy operations
        GameTimeline(const GameTimeline&) = delete;
        GameTimeline& operator=(const GameTimeline&) = delete;

        // =============================================================================
        // Timeline Interface Implementation
        // =============================================================================

        /**
         * @brief Initialize game timeline
         * @param config Timeline configuration
         * @return True if initialization successful
         */
        bool initialize(const TimelineConfigBase& config) override;

        /**
         * @brief Update game timeline
         * @details Receives pre-processed delta time from TimestepManager.
         *          Focuses on applying time scale, managing timers, and
         *          recording frame history.
         *
         * @param deltaTime Time since last update (already processed by TimestepManager)
         * @return Update result with timing information
         */
        TimelineUpdateResult update(Duration deltaTime) override;

        // =============================================================================
        // Timer Management
        // =============================================================================

        /**
         * @brief Create gameplay timer
         * @param duration Timer duration (affected by time scale)
         * @param callback Timer callback function
         * @param recurring True for repeating timer
         * @return Safe timer handle
         */
        SafeTimerHandle createTimer(
                Duration duration,
                TimerCallback callback,
                bool recurring
                ) override;

        /**
         * @brief Cancel a timer
         * @param handle Timer handle to cancel
         * @return True if timer was cancelled successfully
         */
        bool cancelTimer(const SafeTimerHandle& handle) override;

        /**
         * @brief Clear all active timers
         */
        void clearAllTimers() override;

        /**
         * @brief Get count of active timers
         * @return Number of active timers
         */
        [[nodiscard]] std::size_t getActiveTimerCount() const override;

        // =============================================================================
        // Game-Specific Features
        // =============================================================================

        /**
         * @brief Get current frame number
         * @return Frame count since timeline start
         */
        [[nodiscard]] std::uint64_t getFrameNumber() const noexcept {
            return frameNumber_;
        }

        /**
         * @brief Save timeline state
         * @return Serializable state structure
         */
        [[nodiscard]] GameTimelineState saveState() const;

        /**
         * @brief Load timeline state
         * @param state Previously saved state
         */
        void loadState(const GameTimelineState& state);

        /**
         * @brief Enable replay recording
         * @param maxFrames Maximum frames to record (default: 1 minute at 60 FPS)
         */
        void startRecording(std::size_t maxFrames = 3600);

        /**
         * @brief Stop replay recording
         */
        void stopRecording();

        /**
         * @brief Check if currently recording
         * @return True if recording is active
         */
        [[nodiscard]] bool isRecording() const noexcept {
            return recordingEnabled_;
        }

        /**
         * @brief Get frame history for analysis
         * @param count Number of recent frames to retrieve
         * @return Vector of frame records (newest first)
         */
        [[nodiscard]] std::vector<FrameRecord> getFrameHistory(std::size_t count) const;

        /**
         * @brief Wait for specific game time
         * @param targetTime Target game time to wait for
         * @param timeout Maximum time to wait
         * @return True if target time was reached, false on timeout
         */
        bool waitForTime(Duration targetTime, Duration timeout) override;

        // =============================================================================
        // Configuration
        // =============================================================================

        [[nodiscard]] const TimelineConfigBase& getConfig() const override {
            return config_;
        }

        bool updateConfig(const TimelineConfigBase& config) override;

        [[nodiscard]] const TimestepConfig& getTimestepConfig() const override {
            return config_.timestepConfig;
        }

        bool setTimestepMode(TimestepMode mode) override;

    protected:
        /**
         * @brief Called when timeline pauses
         * @details Pauses all active timers and records pause time
         */
        void onPause() override;

        /**
         * @brief Called when timeline resumes
         * @details Resumes all timers and adjusts their fire times
         */
        void onResume() override;

    private:
        // =============================================================================
        // Private Types
        // =============================================================================

        /**
         * @brief Internal timer structure
         */
        struct GameTimer {
            TimerID id; ///< Unique timer ID
            Duration fireTime; ///< When timer should fire
            Duration duration; ///< Timer duration
            TimerCallback callback; ///< Callback function
            bool recurring; ///< Is repeating timer
            bool isPaused; ///< Is currently paused
            std::optional<Duration> pauseTime; ///< Time when paused
            TimerGeneration generation; ///< Generation counter
        };

        // =============================================================================
        // Private Members
        // =============================================================================

        ITimeBackend& backend_; ///< Platform time source
        TimelineConfigBase config_; ///< Configuration

        std::uint64_t frameNumber_; ///< Current frame number

        // Timer management
        mutable std::mutex timerMutex_; ///< Timer synchronization
        std::unordered_map<TimerID, GameTimer> timers_; ///< Active timers
        TimerID nextTimerId_{1}; ///< Timer ID generator

        // Pause support
        mutable std::mutex pauseMutex_; ///< Pause synchronization
        std::condition_variable pauseCondition_; ///< Pause notification

        // Frame history
        mutable std::mutex historyMutex_; ///< History synchronization
        std::deque<FrameRecord> frameHistory_; ///< Frame timing history
        static constexpr std::size_t MAX_FRAME_HISTORY = 300; ///< 5 seconds at 60 FPS

        // Recording support
        std::atomic<bool> recordingEnabled_{false}; ///< Recording state
        std::size_t maxRecordedFrames_{3600}; ///< Max recording length
        std::vector<GameTimelineState> recordedFrames_; ///< Recorded states

        // Statistics
        BasicTimeStats frameStats_; ///< Frame statistics

        // =============================================================================
        // Private Methods
        // =============================================================================

        /**
         * @brief Process expired timers
         * @param currentTime Current game time
         * @return Number of timers processed
         */
        std::uint32_t processTimers(Duration currentTime);

        /**
         * @brief Record frame to history
         * @param result Update result from this frame
         */
        void recordFrameHistory(const TimelineUpdateResult& result);

        /**
         * @brief Update frame statistics
         * @param result Update result from this frame
         */
        void updateStatistics(const TimelineUpdateResult& result);
    };
} // namespace engine::time
