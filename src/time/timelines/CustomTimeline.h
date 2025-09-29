/**
 * @file CustomTimeline.h
 * @brief Flexible user-configurable timeline template
 * @details Provides a customizable timeline that can be configured for specific
 *          game needs. Supports arbitrary scaling, custom update logic, and
 *          flexible behavior suitable for special effects, cutscenes, and mini-games.
 *
 * @author Andres Guerrero
 * @date Created on 2025-09-19
 */

#pragma once

#include "../core/Timeline.h"
#include "../core/ITimeBackend.h"

#include <any>
#include <utility>

namespace engine::time {
    /**
     * @brief Customizable timeline for game-specific needs
     * @details A flexible timeline implementation that can be configured to match
     *          specific gameplay requirements. Supports custom update logic, arbitrary
     *          time manipulation, and special behaviors for unique game features.
     *
     * Key features:
     * - Fully configurable behavior
     * - Custom update callbacks
     * - Arbitrary time manipulation
     * - User-defined data storage
     * - Scriptable through callbacks
     * - Multiple scaling modes
     *
     * Use cases:
     * - Cutscene playback systems
     * - Mini-game time mechanics
     * - Replay systems
     * - Special effect sequences
     * - Tutorial time control
     * - Debug time manipulation
     */

    /**
     * @brief Custom behavior flags
     */
    enum class CustomBehavior : std::uint32_t {
        NONE              = 0, ALLOW_REVERSE = 1 << 0, ///< Can run backwards
        ALLOW_JUMP        = 1 << 1, ///< Can jump to arbitrary time
        LOOP_ON_END       = 1 << 2, ///< Loop when reaching end
        PAUSE_ON_END      = 1 << 3, ///< Auto-pause at end
        SYNC_TO_PARENT    = 1 << 4, ///< Sync with parent timeline
        IGNORE_PAUSE      = 1 << 5, ///< Continue during global pause
        RECORD_HISTORY    = 1 << 6, ///< Record state history
        ALLOW_REWIND      = 1 << 7, ///< Support rewinding
        INTERPOLATE_JUMPS = 1 << 8, ///< Smooth time jumps
        QUANTIZE_TIME     = 1 << 9, ///< Quantize to time grid
        RELATIVE_TIME     = 1 << 10, ///< Use relative time
        CLAMP_SCALE       = 1 << 11, ///< Clamp time scale to limits
    };

    // Bitwise operators for CustomBehavior
    [[nodiscard]] constexpr CustomBehavior operator|(
            CustomBehavior a,
            CustomBehavior b
            ) noexcept {
        return static_cast<CustomBehavior>(
            static_cast<std::uint32_t>(a) | static_cast<std::uint32_t>(b)
        );
    }

    [[nodiscard]] constexpr CustomBehavior operator&(
            CustomBehavior a,
            CustomBehavior b
            ) noexcept {
        return static_cast<CustomBehavior>(
            static_cast<std::uint32_t>(a) & static_cast<std::uint32_t>(b)
        );
    }

    constexpr CustomBehavior operator|=(
            CustomBehavior& a,
            const CustomBehavior b
            ) noexcept {
        a = a | b;
        return a;
    }

    class CustomTimeline final : public Timeline {
    public:
        /**
         * @brief Custom update callback type
         */
        using UpdateCallback = std::function<TimelineUpdateResult(Duration)>;

        /**
         * @brief Custom timeline configuration
         */
        struct CustomConfig {
            std::string identifier; ///< Unique identifier
            CustomBehavior behaviors{CustomBehavior::NONE}; ///< Behavior flags
            Duration maxDuration{Duration::max()}; ///< Maximum duration
            TimeScale minScale{0.0}; ///< Minimum time scale
            TimeScale maxScale{10.0}; ///< Maximum time scale
            Duration quantizeInterval{Duration::zero()}; ///< Time quantization
            UpdateCallback customUpdate; ///< Custom update logic
            std::any userData; ///< User-defined data
        };

        /**
         * @brief Timeline section for sequencing
         */
        struct TimelineSection {
            std::string name; ///< Section identifier
            Duration startTime; ///< Section start
            Duration endTime; ///< Section end
            TimeScale sectionScale{1.0}; ///< Section time scale
            std::function<void()> onEnter; ///< Enter callback
            std::function<void()> onExit; ///< Exit callback
            std::function<void(float)> onUpdate; ///< Update callback
            bool looping{false}; ///< Loop this section
        };

        struct TimelineStateRecord {
            Duration time;
            TimeScale scale;
            bool paused;
            std::optional<std::size_t> section;
        };

        /**
         * @brief Constructor
         * @param backend Time backend for platform queries
         * @param config Custom configuration
         * @param name Timeline identifier
         */
        explicit CustomTimeline(
                ITimeBackend* backend,
                CustomConfig config,
                const std::string& name = "CustomTime"
                ) :
            Timeline(TimelineType::CUSTOM_1, name)
            , backend_(backend)
            , customConfig_(std::move(config))
            , currentSection_(std::nullopt)
            , jumpStartTime_(Duration::zero())
            , jumpTargetTime_(Duration::zero())
            , jumpDuration_(Duration::zero())
            , jumpElapsed_(Duration::zero()) {
            if (!backend_) {
                throw std::invalid_argument("CustomTimeline requires valid backend");
            }
        }

        /**
         * @brief Destructor
         */
        ~CustomTimeline() override {
            Timeline::stop();
        }

        // =============================================================================
        // Timeline Interface Implementation
        // =============================================================================

        /**
         * @brief Initialize custom timeline
         * @param config Timeline configuration
         * @return True if initialization successful
         */
        bool initialize(const TimelineConfigBase& config) override {
            if (state_ != TimelineState::UNINITIALIZED) {
                return false;
            }

            config_ = config;

            // Set initial state
            timeScale_ = config_.initialScale;
            isPaused_ = config_.startPaused;

            // Initialize timing
            lastUpdateTime_ = backend_->now();

            // Pre-allocate storage
            timers_.reserve(config_.maxEventQueue);
            sections_.reserve(16);

            // if (hasCustomBehavior(CustomBehavior::RECORD_HISTORY)) {
            //     stateHistory_.reserve(300); // 5 seconds at 60 FPS
            // }

            state_ = TimelineState::INITIALIZED;

            if (config_.autoStart) {
                return start();
            }

            return true;
        }

        /**
         * @brief Update custom timeline
         * @param deltaTime Time since last update
         * @return Update result from custom logic or default
         */
        TimelineUpdateResult update(const Duration deltaTime) override {
            if (state_ != TimelineState::RUNNING) {
                return TimelineUpdateResult{};
            }

            // Check if should ignore pause
            if (isPaused_ && !hasCustomBehavior(CustomBehavior::IGNORE_PAUSE)) {
                return TimelineUpdateResult{};
            }

            TimelineUpdateResult result;

            // Use custom update if provided
            if (customConfig_.customUpdate) {
                result = customConfig_.customUpdate(deltaTime);
            } else {
                result = defaultUpdate(deltaTime);
            }

            // Apply custom behaviors
            applyCustomBehaviors(result);

            // Update sections
            updateSections(result.scaledDeltaTime);

            // Record history if enabled
            if (hasCustomBehavior(CustomBehavior::RECORD_HISTORY)) {
                recordState();
            }

            // Process timers
            result.timersProcessed = processTimers(currentTime_);

            lastUpdateTime_ = backend_->now();

            return result;
        }

        // =============================================================================
        // Custom Timeline Features
        // =============================================================================

        /**
         * @brief Add timeline section
         * @param section Section definition
         */
        void addSection(const TimelineSection& section) {
            sections_.push_back(section);

            // Sort sections by start time
            std::ranges::sort(
                    sections_,
                    [](const auto& a, const auto& b) {
                        return a.startTime < b.startTime;
                    }
                    );
        }

        /**
         * @brief Jump to specific time
         * @param targetTime Target time position
         * @param interpolate If true, smoothly interpolate
         */
        void jumpToTime(const Duration targetTime, const bool interpolate = false) {
            if (!hasCustomBehavior(CustomBehavior::ALLOW_JUMP)) {
                return;
            }

            if (interpolate && hasCustomBehavior(CustomBehavior::INTERPOLATE_JUMPS)) {
                // Set up interpolation
                jumpStartTime_ = currentTime_;
                jumpTargetTime_ = targetTime;
                jumpDuration_ = Duration(1000000); // 1 second interpolation
                isJumping_ = true;
            } else {
                // Instant jump
                currentTime_ = targetTime;
                totalElapsedTime_ = targetTime;
            }
        }

        /**
         * @brief Rewind timeline
         * @param duration Amount to rewind
         */
        void rewind(const Duration duration) {
            if (!hasCustomBehavior(CustomBehavior::ALLOW_REWIND)) {
                return;
            }

            Duration newTime = currentTime_ - duration;
            if (newTime < Duration::zero()) {
                newTime = Duration::zero();
            }

            jumpToTime(newTime, false);
        }

        /**
         * @brief Set custom update callback
         * @param callback Update function
         */
        void setUpdateCallback(UpdateCallback callback) {
            customConfig_.customUpdate = std::move(callback);
        }

        /**
         * @brief Set user data
         * @param data User-defined data
         */
        void setUserData(std::any data) {
            customConfig_.userData = std::move(data);
        }

        /**
         * @brief Get user data
         * @tparam T Data type
         * @return User data or nullptr if type mismatch
         */
        template <typename T>
        T* getUserData() {
            try {
                return std::any_cast<T>(&customConfig_.userData);
            } catch (const std::bad_any_cast&) {
                return nullptr;
            }
        }

        /**
         * @brief Set custom behavior flags
         * @param behaviors Behavior flags to set
         */
        void setCustomBehaviors(const CustomBehavior behaviors) {
            customConfig_.behaviors = behaviors;
        }

        /**
         * @brief Check if has custom behavior
         * @param behavior Behavior to check
         * @return True if behavior is enabled
         */
        [[nodiscard]] bool hasCustomBehavior(const CustomBehavior behavior) const noexcept {
            return (customConfig_.behaviors & behavior) == behavior;
        }

        /**
         * @brief Get current section
         * @return Current section name or empty
         */
        [[nodiscard]] std::string getCurrentSection() const {
            if (currentSection_.has_value()) {
                return sections_[*currentSection_].name;
            }
            return "";
        }

        /**
         * @brief Set loop duration
         * @param duration Loop duration (0 to disable)
         */
        void setLoopDuration(const Duration duration) {
            customConfig_.maxDuration = duration;

            if (duration > Duration::zero()) {
                customConfig_.behaviors |= CustomBehavior::LOOP_ON_END;

            }
        }

        /**
         * @brief Get state history
         * @param count Number of states to retrieve
         * @return Historical states
         */
        [[nodiscard]] std::vector<TimelineStateRecord> getStateHistory(const std::size_t count) const {
            std::lock_guard lock(historyMutex_);

            std::vector<TimelineStateRecord> result;
            const std::size_t available = std::min(count, stateHistory_.size());

            result.reserve(available);
            for (std::size_t i = 0; i < available; ++i) {
                result.push_back(stateHistory_[stateHistory_.size() - 1 - i]);
            }

            return result;
        }

        // =============================================================================
        // Timer Management
        // =============================================================================

        SafeTimerHandle createTimer(
                const Duration duration,
                TimerCallback callback,
                const bool recurring
                ) override {
            std::lock_guard lock(timerMutex_);

            const TimerID id = nextTimerId_++;
            const Duration fireTime = currentTime_ + duration;

            CustomTimer timer{
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
            return true;
        }

        [[nodiscard]] const TimestepConfig& getTimestepConfig() const override {
            return config_.timestepConfig;
        }

        bool setTimestepMode(const TimestepMode mode) override {
            config_.timestepConfig.mode = mode;
            return true;
        }

        bool waitForTime(const Duration targetTime, const Duration timeout) override {
            const auto startWait = backend_->now();
            const auto timeoutTime = startWait + timeout;

            while (currentTime_ < targetTime) {
                if (backend_->now() >= timeoutTime) {
                    return false;
                }
                backend_->sleep(Duration(100));
            }
            return true;
        }

    private:
        // Private types
        struct CustomTimer {
            TimerID id{};
            Duration fireTime{};
            Duration duration{};
            TimerCallback callback;
            bool recurring{};
            TimerGeneration generation{};
        };

        // Private members
        ITimeBackend* backend_;
        TimelineConfigBase config_;
        CustomConfig customConfig_;

        // Sections
        std::vector<TimelineSection> sections_;
        std::optional<std::size_t> currentSection_;

        // Jump interpolation
        bool isJumping_{false};
        Duration jumpStartTime_;
        Duration jumpTargetTime_;
        Duration jumpDuration_;
        Duration jumpElapsed_;

        // State history
        mutable std::mutex historyMutex_;
        std::deque<TimelineStateRecord> stateHistory_;

        // Timers
        mutable std::mutex timerMutex_;
        std::unordered_map<TimerID, CustomTimer> timers_;
        TimerID nextTimerId_{1};

        // Private methods
        TimelineUpdateResult defaultUpdate(const Duration deltaTime) {
            // Apply time scaling
            const auto scaledDelta = Duration(
                    static_cast<std::int64_t>(static_cast<double>(deltaTime.count()) * timeScale_.load())
                    );

            // Clamp if configured
            Duration finalDelta = scaledDelta;
            if (hasCustomBehavior(CustomBehavior::CLAMP_SCALE)) {
                const TimeScale clampedScale = std::clamp(
                        timeScale_.load(),
                        customConfig_.minScale,
                        customConfig_.maxScale
                        );
                finalDelta = Duration(
                        static_cast<std::int64_t>(static_cast<double>(deltaTime.count()) * clampedScale)
                        );
            }

            // Quantize if configured
            if (hasCustomBehavior(CustomBehavior::QUANTIZE_TIME) &&
                customConfig_.quantizeInterval > Duration::zero()) {
                const auto quanta = finalDelta.count() / customConfig_.quantizeInterval.count();
                finalDelta = customConfig_.quantizeInterval * quanta;
            }

            // Update time
            currentTime_ += finalDelta;
            totalElapsedTime_ += finalDelta;

            return TimelineUpdateResult{
                            .actualDeltaTime = deltaTime,
                            .scaledDeltaTime = finalDelta,
                            .fixedStepsExecuted = 0,
                            .timersProcessed = 0,
                            .wasThrottled = false,
                            .budgetExceeded = false
                    };
        }

        void applyCustomBehaviors(const TimelineUpdateResult& result) {
            // Handle looping
            if (hasCustomBehavior(CustomBehavior::LOOP_ON_END) &&
                currentTime_ >= customConfig_.maxDuration) {
                currentTime_ = Duration::zero();
            }

            // Handle pause on end
            if (hasCustomBehavior(CustomBehavior::PAUSE_ON_END) &&
                currentTime_ >= customConfig_.maxDuration) {
                pause();
            }

            // Handle jump interpolation
            if (isJumping_) {
                jumpElapsed_ += result.actualDeltaTime;

                if (jumpElapsed_ >= jumpDuration_) {
                    currentTime_ = jumpTargetTime_;
                    isJumping_ = false;
                } else {
                    const float t = static_cast<float>(jumpElapsed_.count()) /
                            static_cast<float>(jumpDuration_.count());

                    const std::int64_t interpolated =
                            jumpStartTime_.count() +
                            static_cast<std::int64_t>(
                                (static_cast<float>(jumpTargetTime_.count()) - static_cast<float>(jumpStartTime_.count())) * t
                            );

                    currentTime_ = Duration(interpolated);
                }
            }
        }

        // TODO: Revisar esto
        void updateSections(Duration deltaTime) {
            // Find current section
            std::optional<std::size_t> newSection;

            for (std::size_t i = 0; i < sections_.size(); ++i) {
                if (currentTime_ >= sections_[i].startTime &&
                    currentTime_ < sections_[i].endTime) {
                    newSection = i;
                    break;
                }
            }

            // Handle section transitions
            if (newSection != currentSection_) {
                if (currentSection_.has_value() && sections_[*currentSection_].onExit) {
                    sections_[*currentSection_].onExit();
                }

                if (newSection.has_value() && sections_[*newSection].onEnter) {
                    sections_[*newSection].onEnter();
                }

                currentSection_ = newSection;
            }

            // Update current section
            if (currentSection_.has_value()) {
                const auto& section = sections_[*currentSection_];

                if (section.onUpdate) {
                    const float progress =
                            static_cast<float>((currentTime_ - section.startTime).count()) /
                            static_cast<float>((section.endTime - section.startTime).count());

                    section.onUpdate(std::clamp(progress, 0.0f, 1.0f));
                }

                // Handle section looping
                if (section.looping && currentTime_ >= section.endTime) {
                    currentTime_ = section.startTime;
                }
            }
        }

        void recordState() {
            std::lock_guard lock(historyMutex_);

            stateHistory_.push_back(
                    TimelineStateRecord{
                            .time = currentTime_,
                            .scale = timeScale_.load(),
                            .paused = isPaused_.load(),
                            .section = currentSection_
                    }
                    );

            // Limit history size
            if (stateHistory_.size() > 300) {
                stateHistory_.pop_front();
            }
        }

        std::uint32_t processTimers(const Duration currentTime) {
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
    };
} // namespace engine::time
