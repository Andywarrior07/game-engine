/**
 * @file UITimeline.h
 * @brief UI-specific timeline for menus and HUD animations
 * @details Provides a timeline that runs independently of game pause state,
 *          allowing UI animations and transitions to continue while the game
 *          is paused. Optimized for moderate precision with UI-friendly features.
 *
 * @author Andres Guerrero
 * @date 22-09-2025
 */

#pragma once

#include "../core/Timeline.h"
#include "../core/ITimeBackend.h"

#include <map>
#include <functional>

namespace engine::time {
    /**
     * @brief UI timeline for interface animations and transitions
     * @details Runs independently of game pause state, allowing menus and HUD
     *          to remain responsive. Supports UI-specific features like easing
     *          functions, animation curves, and transition management.
     *
     * Key features:
     * - Independent of game pause
     * - Built-in easing functions
     * - Animation curve support
     * - Transition management
     * - Screen-space timing
     * - Lower precision acceptable
     *
     * Use cases:
     * - Menu animations
     * - HUD element transitions
     * - Button hover effects
     * - Loading screen animations
     * - Notification pop-ups
     * - Tooltip fade-ins
     */
    class UITimeline final : public Timeline {
    public:
        /**
         * @brief Easing function types for UI animations
         */
        enum class EasingType : std::uint8_t {
            LINEAR, EASE_IN_QUAD, EASE_OUT_QUAD,
            EASE_IN_OUT_QUAD, EASE_IN_CUBIC, EASE_OUT_CUBIC,
            EASE_IN_OUT_CUBIC, EASE_IN_EXPO, EASE_OUT_EXPO,
            EASE_IN_OUT_EXPO, EASE_IN_BACK, EASE_OUT_BACK,
            EASE_IN_OUT_BACK, EASE_IN_ELASTIC, EASE_OUT_ELASTIC,
            EASE_IN_OUT_ELASTIC, EASE_IN_BOUNCE, EASE_OUT_BOUNCE,
            EASE_IN_OUT_BOUNCE
        };

        /**
         * @brief UI animation definition
         */
        struct UIAnimation {
            using AnimationCallback = std::function<void(float)>; // Progress [0, 1]
            using CompletionCallback = std::function<void()>;

            std::string name; ///< Animation identifier
            Duration duration; ///< Total duration
            Duration elapsed{Duration::zero()}; ///< Time elapsed
            EasingType easing{EasingType::LINEAR}; ///< Easing function
            AnimationCallback updateCallback; ///< Progress update
            CompletionCallback completionCallback; ///< Completion handler
            bool loop{false}; ///< Loop animation
            bool reverse{false}; ///< Play in reverse
            bool paused{false}; ///< Animation paused
            float speed{1.0f}; ///< Animation speed
        };

        /**
         * @brief UI transition between states
         */
        struct UITransition {
            std::string fromState; ///< Starting state
            std::string toState; ///< Target state
            Duration duration; ///< Transition duration
            Duration elapsed{Duration::zero()}; ///< Time elapsed
            EasingType easing{EasingType::EASE_IN_OUT_CUBIC};
            std::function<void(float)> transitionCallback; ///< Progress callback
            std::function<void()> completionCallback; ///< Completion callback
        };

        /**
         * @brief Constructor
         * @param backend Time backend for platform queries
         * @param name Timeline identifier
         */
        explicit UITimeline(
                ITimeBackend& backend,
                const std::string& name = "UITime"
                ) :
            Timeline(TimelineType::UI_TIME, name)
            , backend_(backend)
            , nextAnimationId_(1)
            , activeTransition_(std::nullopt) {}

        /**
         * @brief Destructor
         */
        ~UITimeline() override;

        // =============================================================================
        // Timeline Interface Implementation
        // =============================================================================

        /**
         * @brief Initialize UI timeline
         * @param config Timeline configuration
         * @return True if initialization successful
         */
        bool initialize(const TimelineConfigBase& config) override;

        /**
         * @brief Update UI timeline
         * @param deltaTime Time since last update
         * @return Update result
         */
        TimelineUpdateResult update(Duration deltaTime) override;

        // =============================================================================
        // UI Animation Management
        // =============================================================================

        /**
         * @brief Start a UI animation
         * @param name Animation identifier
         * @param duration Animation duration
         * @param updateCallback Progress callback (receives 0-1)
         * @param easing Easing function type
         * @return Animation ID
         */
        std::uint32_t startAnimation(
                const std::string& name,
                Duration duration,
                UIAnimation::AnimationCallback updateCallback,
                EasingType easing = EasingType::EASE_IN_OUT_CUBIC
                );

        /**
         * @brief Stop an animation
         * @param id Animation ID
         * @param complete If true, jump to end before stopping
         */
        void stopAnimation(std::uint32_t id, bool complete = false);

        /**
         * @brief Pause/resume animation
         * @param id Animation ID
         * @param paused Pause state
         */
        void setAnimationPaused(std::uint32_t id, bool paused);

        /**
         * @brief Set animation speed
         * @param id Animation ID
         * @param speed Speed multiplier
         */
        void setAnimationSpeed(std::uint32_t id, float speed);

        /**
         * @brief Start a UI state transition
         * @param fromState Starting state name
         * @param toState Target state name
         * @param duration Transition duration
         * @param transitionCallback Progress callback
         * @param easing Easing function
         */
        void startTransition(
                const std::string& fromState,
                const std::string& toState,
                Duration duration,
                std::function<void(float)> transitionCallback,
                EasingType easing = EasingType::EASE_IN_OUT_CUBIC
                );

        /**
         * @brief Cancel active transition
         */
        void cancelTransition() {
            std::lock_guard lock(transitionMutex_);
            activeTransition_ = std::nullopt;
        }

        /**
         * @brief Check if transition is active
         */
        [[nodiscard]] bool hasActiveTransition() const {
            std::lock_guard lock(transitionMutex_);
            return activeTransition_.has_value();
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

        // =============================================================================
        // UI-Specific Features
        // =============================================================================

        /**
         * @brief Set whether UI updates while game is paused
         * @param allow If true, UI continues during game pause
         */
        void setAllowUpdateWhilePaused(const bool allow) {
            allowUpdateWhilePaused_ = allow;
        }

        /**
         * @brief Get animation count
         */
        [[nodiscard]] std::size_t getActiveAnimationCount() const {
            std::lock_guard lock(animationMutex_);
            return animations_.size();
        }

        /**
         * @brief Apply easing function
         * @param t Progress value [0, 1]
         * @param type Easing type
         * @return Eased value [0, 1]
         */
        [[nodiscard]] static float applyEasing(float t, EasingType type);

        // Configuration methods
        [[nodiscard]] const TimelineConfigBase& getConfig() const override { return config_; }

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

        bool waitForTime(Duration targetTime, Duration timeout) override;

    private:
        // Private types
        struct UITimer {
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

        // Animations
        mutable std::mutex animationMutex_;
        std::map<std::uint32_t, UIAnimation> animations_;
        std::atomic<std::uint32_t> nextAnimationId_;

        // Transitions
        mutable std::mutex transitionMutex_;
        std::optional<UITransition> activeTransition_;

        // Timers
        mutable std::mutex timerMutex_;
        std::unordered_map<TimerID, UITimer> timers_;
        TimerID nextTimerId_{1};

        // UI-specific settings
        bool allowUpdateWhilePaused_{true};

        // Private methods
        static float easeOutBounce(float t);

        std::uint32_t updateAnimations(Duration deltaTime);

        bool updateTransition(Duration deltaTime);

        std::uint32_t processTimers(Duration currentTime);
    };
} // namespace engine::time
