/**
 * @file PhysicsTimeline.h
 * @brief Fixed-timestep timeline for deterministic physics simulation
 * @details Provides a timeline specifically designed for physics engines requiring
 *          deterministic behavior. Uses fixed timestep with accumulation to ensure
 *          reproducible physics simulation regardless of frame rate.
 *
 * @author Andrés Guerrero
 * @date 23-09-2025
 */

#pragma once

#include "../core/Timeline.h"
#include "../core/ITimeBackend.h"
#include <queue>
#include <atomic>

namespace engine::time {
    /**
     * @brief Physics timeline for deterministic simulation
     * @details Implements a fixed timestep timeline critical for physics engines.
     *          Ensures deterministic behavior through consistent time steps while
     *          handling timestep accumulation and subdivision for stability.
     *
     * Key features:
     * - Fixed timestep for determinism
     * - Accumulator pattern for smooth updates
     * - Spiral of death prevention
     * - Substep support for accuracy
     * - Interpolation data for rendering
     * - Rollback support for networking
     *
     * Use cases:
     * - Rigid body dynamics
     * - Collision detection
     * - Particle physics
     * - Cloth simulation
     * - Fluid dynamics
     * - Constraint solving
     */
    class PhysicsTimeline final : public Timeline {
    public:
        /**
         * @brief Physics simulation state for interpolation
         */
        struct PhysicsState {
            Duration simulationTime; ///< Current simulation time
            Duration accumulatedTime; ///< Accumulated time
            std::uint64_t stepCount; ///< Total simulation steps
            float interpolationAlpha; ///< Rendering interpolation
            bool inFixedUpdate; ///< Currently in fixed update
        };

        /**
         * @brief Physics step callback type
         */
        using PhysicsStepCallback = std::function<void(Duration fixedDeltaTime)>;

        /**
         * @brief Physics interpolation callback type
         */
        using InterpolationCallback = std::function<void(float alpha)>;

        /**
         * @brief Constructor
         * @param backend Time backend for platform queries
         * @param name Timeline identifier
         */
        explicit PhysicsTimeline(
                ITimeBackend& backend,
                const std::string& name = "PhysicsTime"
                ) :
            Timeline(TimelineType::PHYSICS_TIME, name)
            , backend_(backend)
            , fixedTimestep_(constants::DEFAULT_FIXED_TIMESTEP)
            , maxSubsteps_(constants::MAX_FIXED_ITERATIONS)
            , accumulator_(Duration::zero())
            , stepCount_(0)
            , interpolationAlpha_(0.0f)
            , inFixedUpdate_(false) {}

        /**
         * @brief Destructor
         */
        ~PhysicsTimeline() override;

        // =============================================================================
        // Timeline Interface Implementation
        // =============================================================================

        /**
         * @brief Initialize physics timeline
         * @param config Timeline configuration
         * @return True if initialization successful
         */
        bool initialize(const TimelineConfigBase& config) override;

        /**
         * @brief Update physics timeline
         * @param deltaTime Time since last update
         * @return Update result with fixed steps executed
         */
        TimelineUpdateResult update(Duration deltaTime) override;

        // =============================================================================
        // Physics-Specific Features
        // =============================================================================

        /**
         * @brief Set physics step callback
         * @param callback Function called each physics step
         */
        void setPhysicsStepCallback(PhysicsStepCallback callback) {
            physicsStepCallback_ = std::move(callback);
        }

        /**
         * @brief Set interpolation callback
         * @param callback Function called with interpolation alpha
         */
        void setInterpolationCallback(InterpolationCallback callback) {
            interpolationCallback_ = std::move(callback);
        }

        /**
         * @brief Set fixed timestep
         * @param timestep New fixed timestep duration
         */
        void setFixedTimestep(Duration timestep);

        /**
         * @brief Get current fixed timestep
         */
        [[nodiscard]] Duration getFixedTimestep() const noexcept {
            return fixedTimestep_;
        }

        /**
         * @brief Get interpolation alpha for rendering
         */
        [[nodiscard]] float getInterpolationAlpha() const noexcept {
            return interpolationAlpha_.load(std::memory_order_acquire);
        }

        /**
         * @brief Get accumulator value
         */
        [[nodiscard]] Duration getAccumulator() const noexcept {
            return accumulator_;
        }

        /**
         * @brief Get total physics steps executed
         */
        [[nodiscard]] std::uint64_t getStepCount() const noexcept {
            return stepCount_;
        }

        /**
         * @brief Check if currently in fixed update
         */
        [[nodiscard]] bool isInFixedUpdate() const noexcept {
            return inFixedUpdate_.load(std::memory_order_acquire);
        }

        /**
         * @brief Reset accumulator (use with caution)
         */
        void resetAccumulator() {
            accumulator_ = Duration::zero();
            interpolationAlpha_.store(0.0f, std::memory_order_release);
        }

        /**
         * @brief Force physics step (for debugging)
         * @param steps Number of steps to force
         */
        void forcePhysicsSteps(std::uint32_t steps);

        /**
         * @brief Save physics state for rollback
         * @return Current physics state
         */
        [[nodiscard]] PhysicsState savePhysicsState() const {
            return PhysicsState{
                            .simulationTime = currentTime_,
                            .accumulatedTime = accumulator_,
                            .stepCount = stepCount_,
                            .interpolationAlpha = interpolationAlpha_.load(),
                            .inFixedUpdate = inFixedUpdate_.load()
                    };
        }

        /**
         * @brief Restore physics state (for rollback)
         * @param state Previously saved state
         */
        void restorePhysicsState(const PhysicsState& state);

        /**
         * @brief Get state history for rollback
         * @param steps Number of steps back
         * @return Historical states
         */
        [[nodiscard]] std::vector<PhysicsState> getStateHistory(std::size_t steps) const;

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

        bool updateConfig(const TimelineConfigBase& config) override;

        [[nodiscard]] const TimestepConfig& getTimestepConfig() const override {
            return config_.timestepConfig;
        }

        bool setTimestepMode(const TimestepMode mode) override {
            // Physics timeline must use fixed timestep
            return mode == TimestepMode::FIXED || mode == TimestepMode::HYBRID;
        }

        bool waitForTime(Duration targetTime, Duration timeout) override;

    private:
        // Private types
        struct PhysicsTimer {
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

        // Fixed timestep settings
        Duration fixedTimestep_;
        std::uint8_t maxSubsteps_;
        Duration maxAccumulator_{Duration(200000)}; // 200ms default
        std::uint8_t substeps_{1};
        bool catchUpAfterSpike_{false};

        // Simulation state
        Duration accumulator_;
        std::uint64_t stepCount_;
        std::atomic<float> interpolationAlpha_;
        std::atomic<bool> inFixedUpdate_;
        TimeStamp lastFixedUpdateTime_;

        // Callbacks
        PhysicsStepCallback physicsStepCallback_;
        InterpolationCallback interpolationCallback_;

        // Timers
        mutable std::mutex timerMutex_;
        std::unordered_map<TimerID, PhysicsTimer> timers_;
        TimerID nextTimerId_{1};
        std::size_t processedTimers_{0};

        // State history
        mutable std::mutex historyMutex_;
        std::deque<PhysicsState> stateHistory_;

        // Statistics
        std::atomic<std::uint32_t> spiralOfDeathCount_{0};
        std::atomic<Duration> totalPhysicsTime_{Duration::zero()};

        // Private methods
        void executePhysicsStep(const Duration timestep) const {
            if (physicsStepCallback_) {
                physicsStepCallback_(timestep);
            }
        }

        void processPhysicsTimers(Duration currentTime);

        void recordPhysicsState();

        void updatePhysicsStats(Duration frameTime, std::uint32_t steps);

        // TODO: Revisar esto
        void onSpiralOfDeath(Duration excess) {
            spiralOfDeathCount_.fetch_add(1, std::memory_order_relaxed);

            // Log warning in debug builds
#ifdef _DEBUG
            // Log spiral of death event
#endif
        }
    };
} // namespace engine::time
