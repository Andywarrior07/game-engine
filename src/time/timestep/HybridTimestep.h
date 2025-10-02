/**
 * @file HybridTimestep.h
 * @brief Hybrid timestep combining fixed physics and variable rendering
 * @details Provides the best of both worlds: deterministic physics updates
 *          with fixed timestep and smooth rendering with variable timestep.
 *          Includes interpolation for visual smoothness.
 *
 * @author Development Team
 * @date Created on 2024-01-20
 */

#pragma once

#include "../core/TimeTypes.h"
#include "../core/TimestepMode.h"
#include "VariableTimestep.h"
#include "FixedTimestep.h"
#include "../utils/TimeMath.h"
#include <memory>
#include <functional>
#include <atomic>

namespace engine::time {
    // =============================================================================
    // Hybrid Update Results
    // =============================================================================

    /**
     * @brief Results from hybrid timestep update
     */
    struct HybridUpdateResult {
        std::uint32_t fixedUpdates{0}; ///< Number of fixed updates
        Duration renderDeltaTime{}; ///< Render frame delta
        Duration physicsDeltaTime{}; ///< Physics update delta
        double interpolationAlpha{0.0}; ///< Interpolation factor
        TimestepMode activeMode{TimestepMode::HYBRID}; ///< Current mode
        bool modeChanged{false}; ///< Mode changed this frame
    };

    // =============================================================================
    // Hybrid Statistics
    // =============================================================================

    /**
     * @brief Statistics for hybrid timestep performance
     */
    struct HybridTimestepStats {
        // Component stats
        FixedTimestepStats fixedStats{}; ///< Fixed timestep stats
        VariableTimestepStats variableStats{}; ///< Variable timestep stats

        // Hybrid-specific stats
        std::uint64_t modeSwitches{0}; ///< Total mode switches
        Duration timeInFixedMode{}; ///< Time spent in fixed mode
        Duration timeInVariableMode{}; ///< Time spent in variable mode
        Duration timeInHybridMode{}; ///< Time spent in hybrid mode

        double physicsUtilization{0.0}; ///< Physics CPU usage
        double renderUtilization{0.0}; ///< Render CPU usage
        double interpolationQuality{1.0}; ///< Interpolation quality metric

        TimestepMode currentMode{TimestepMode::HYBRID}; ///< Current active mode
        Duration lastModeSwitch{}; ///< Time since last switch

        /**
         * @brief Reset all statistics
         */
        void reset() noexcept {
            *this = HybridTimestepStats{};
        }
    };

    // =============================================================================
    // Hybrid Callbacks
    // =============================================================================

    /**
     * @brief Callback for physics update
     * @param deltaTime Fixed physics delta
     * @param updateIndex Update index in current frame
     */
    using PhysicsUpdateCallback = std::function<void(Duration deltaTime, std::uint32_t updateIndex)>;

    /**
     * @brief Callback for render update
     * @param deltaTime Variable render delta
     * @param interpolationAlpha Interpolation factor for smoothing
     */
    using RenderUpdateCallback = std::function<void(Duration deltaTime, double interpolationAlpha)>;

    /**
     * @brief Callback for mode change
     * @param oldMode Previous timestep mode
     * @param newMode New timestep mode
     */
    using ModeChangeCallback = std::function<void(TimestepMode oldMode, TimestepMode newMode)>;

    // =============================================================================
    // HybridTimestep Class
    // =============================================================================

    /**
     * @brief Hybrid timestep system combining fixed and variable updates
     * @details Manages both fixed physics updates and variable rendering
     *          with interpolation for smooth visual presentation.
     */
    class HybridTimestep {
    public:
        /**
         * @brief Constructor with configuration
         * @param config Hybrid timestep configuration
         */
        explicit HybridTimestep(const HybridTimestepConfig& config = {}) noexcept;

        /**
         * @brief Destructor
         */
        ~HybridTimestep() = default;

        // Delete copy operations
        HybridTimestep(const HybridTimestep&) = delete;
        HybridTimestep& operator=(const HybridTimestep&) = delete;

        // Allow move operations
        HybridTimestep(HybridTimestep&&) noexcept = default;
        HybridTimestep& operator=(HybridTimestep&&) noexcept = default;

        // =============================================================================
        // Core Update
        // =============================================================================

        /**
         * @brief Update hybrid timestep system
         * @param frameDeltaTime Raw frame delta time
         * @param physicsCallback Callback for physics updates
         * @param renderCallback Callback for render update
         * @return Update result with details
         */
        [[nodiscard]] HybridUpdateResult update(
                Duration frameDeltaTime,
                const PhysicsUpdateCallback& physicsCallback,
                const RenderUpdateCallback& renderCallback
                ) noexcept;

        /**
         * @brief Update with single callback (simplified)
         * @param frameDeltaTime Raw frame delta
         * @param callback Combined update callback
         * @return Number of physics updates performed
         */
        std::uint32_t updateSimple(
                Duration frameDeltaTime,
                const std::function<void(Duration)>& callback
                ) noexcept;

        /**
         * @brief Reset timestep state
         */
        void reset() noexcept;

        /**
         * @brief Force specific timestep mode
         * @param mode Timestep mode to force
         */
        void setMode(TimestepMode mode) noexcept;

        /**
         * @brief Get current timestep mode
         */
        [[nodiscard]] TimestepMode getMode() const noexcept {
            return currentMode_;
        }

        // =============================================================================
        // Time Queries
        // =============================================================================

        /**
         * @brief Get physics delta time
         * @return Fixed physics timestep
         */
        [[nodiscard]] Duration getPhysicsDeltaTime() const noexcept {
            return fixedTimestep_->getFixedDeltaTime();
        }

        /**
         * @brief Get render delta time
         * @return Variable render timestep
         */
        [[nodiscard]] Duration getRenderDeltaTime() const noexcept {
            return variableTimestep_->getDeltaTime();
        }

        /**
         * @brief Get interpolation alpha
         * @return Interpolation factor [0,1]
         */
        [[nodiscard]] double getInterpolationAlpha() const noexcept {
            return currentInterpolationAlpha_;
        }

        /**
         * @brief Get smoothed interpolation alpha
         * @return Smoothed interpolation for visual stability
         */
        [[nodiscard]] double getSmoothedInterpolationAlpha() const noexcept {
            return smoothedInterpolationAlpha_;
        }

        // =============================================================================
        // Configuration
        // =============================================================================

        /**
         * @brief Set time scale for both systems
         * @param scale Time scale factor
         */
        void setTimeScale(TimeScale scale) noexcept;

        /**
         * @brief Set physics update rate
         * @param updatesPerSecond Physics updates per second
         */
        void setPhysicsRate(double updatesPerSecond) noexcept;

        /**
         * @brief Set target render FPS
         * @param fps Target frames per second
         */
        void setTargetFPS(double fps) noexcept;

        /**
         * @brief Enable/disable interpolation
         */
        void setInterpolation(const bool enable) noexcept {
            config_.enableInterpolation = enable;
        }

        /**
         * @brief Enable/disable adaptive quality
         */
        void setAdaptiveQuality(const bool enable) noexcept {
            config_.adaptiveQuality = enable;
        }

        /**
         * @brief Set mode change callback
         */
        void setModeChangeCallback(ModeChangeCallback callback) noexcept {
            modeChangeCallback_ = std::move(callback);
        }

        /**
         * @brief Get configuration
         */
        [[nodiscard]] const HybridTimestepConfig& getConfig() const noexcept {
            return config_;
        }

        /**
         * @brief Update configuration
         */
        void updateConfig(const HybridTimestepConfig& config) noexcept;

        // =============================================================================
        // Statistics
        // =============================================================================

        /**
         * @brief Get hybrid timestep statistics
         */
        [[nodiscard]] HybridTimestepStats getStats() noexcept;

        /**
         * @brief Reset statistics
         */
        void resetStats() noexcept;

        /**
         * @brief Get physics utilization
         * @return CPU utilization for physics [0,1]
         */
        [[nodiscard]] double getPhysicsUtilization() const noexcept {
            return physicsUtilization_;
        }

        /**
         * @brief Get render utilization
         * @return CPU utilization for rendering [0,1]
         */
        [[nodiscard]] double getRenderUtilization() const noexcept {
            return renderUtilization_;
        }

        // =============================================================================
        // Component Access
        // =============================================================================

        /**
         * @brief Get fixed timestep component
         * @return Pointer to fixed timestep (not owned)
         */
        [[nodiscard]] FixedTimestep* getFixedTimestep() const noexcept {
            return fixedTimestep_.get();
        }

        /**
         * @brief Get variable timestep component
         * @return Pointer to variable timestep (not owned)
         */
        [[nodiscard]] VariableTimestep* getVariableTimestep() const noexcept {
            return variableTimestep_.get();
        }

    private:
        // Configuration
        HybridTimestepConfig config_;

        // Timestep components
        std::unique_ptr<FixedTimestep> fixedTimestep_;
        std::unique_ptr<VariableTimestep> variableTimestep_;

        // Current state
        TimestepMode currentMode_{TimestepMode::HYBRID};
        TimestepMode previousMode_{TimestepMode::HYBRID};
        double currentInterpolationAlpha_{0.0};
        double smoothedInterpolationAlpha_{0.0};

        // Performance tracking
        std::atomic<double> physicsUtilization_{0.0};
        std::atomic<double> renderUtilization_{0.0};
        Duration physicsTime_{};
        Duration renderTime_{};

        // Mode switching
        Duration lastModeSwitch_{};
        Duration modeStabilityTimer_{};
        std::uint32_t modeSwitchCount_{0};

        // Statistics
        mutable std::mutex statsMutex_;
        HybridTimestepStats stats_;

        // Callbacks
        ModeChangeCallback modeChangeCallback_;

        // =============================================================================
        // Internal Methods
        // =============================================================================

        /**
         * @brief Update in hybrid mode
         */
        [[nodiscard]] HybridUpdateResult updateHybridMode(
                Duration frameDeltaTime,
                const PhysicsUpdateCallback& physicsCallback,
                const RenderUpdateCallback& renderCallback
                ) noexcept;

        /**
         * @brief Update in fixed-only mode
         */
        [[nodiscard]] HybridUpdateResult updateFixedMode(
                Duration frameDeltaTime,
                const PhysicsUpdateCallback& physicsCallback,
                const RenderUpdateCallback& renderCallback
                ) const noexcept;

        /**
         * @brief Update in variable-only mode
         */
        [[nodiscard]] HybridUpdateResult updateVariableMode(
                Duration frameDeltaTime,
                const PhysicsUpdateCallback& physicsCallback,
                const RenderUpdateCallback& renderCallback
                ) const noexcept;

        /**
         * @brief Check if mode switch is needed
         */
        [[nodiscard]] bool shouldSwitchMode() const noexcept;

        /**
         * @brief Perform mode switch
         */
        void switchMode(TimestepMode newMode) noexcept;

        /**
         * @brief Update interpolation alpha
         */
        void updateInterpolation() noexcept;

        /**
         * @brief Update performance metrics
         */
        void updatePerformanceMetrics(
                Duration frameDelta,
                Duration physicsElapsed,
                Duration renderElapsed
                ) noexcept;

        /**
         * @brief Update statistics
         */
        void updateStatistics(
                const HybridUpdateResult& result,
                Duration frameDelta
                ) noexcept;

        /**
         * @brief Adjust quality based on performance
         */
        void adjustQuality() const noexcept;
    };

} // namespace engine::time
