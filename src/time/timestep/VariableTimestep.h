/**
 * @file VariableTimestep.h
 * @brief Variable timestep implementation for smooth frame rates
 * @details Provides variable timestep updates with delta time clamping
 *          and smoothing for stable gameplay. Ideal for rendering and
 *          non-deterministic systems.
 *
 * @author Development Team
 * @date Created on 2024-01-20
 */

#pragma once

#include "../core/TimeTypes.h"
#include "../core/TimestepMode.h"

#include <deque>
#include <numeric>
#include <atomic>
#include <mutex>

namespace engine::time {
    // =============================================================================
    // Frame Statistics
    // =============================================================================

    /**
     * @brief Statistics for variable timestep performance
     */
    struct VariableTimestepStats {
        Duration currentDeltaTime{}; ///< Current frame delta
        Duration smoothedDeltaTime{}; ///< Smoothed delta
        Duration averageDeltaTime{}; ///< Average over window
        Duration minDeltaTime{Duration::max()}; ///< Minimum recorded
        Duration maxDeltaTime{Duration::min()}; ///< Maximum recorded

        double currentFPS{0.0}; ///< Current FPS
        double averageFPS{0.0}; ///< Average FPS
        double minFPS{std::numeric_limits<double>::max()}; ///< Minimum FPS
        double maxFPS{0.0}; ///< Maximum FPS

        std::uint64_t frameCount{0}; ///< Total frames processed
        std::uint64_t spikeCount{0}; ///< Frame spike count
        Duration totalTime{}; ///< Total elapsed time

        double timeScaleEffective{1.0}; ///< Effective time scale
        double stability{1.0}; ///< Frame timing stability [0-1]

        /**
         * @brief Reset statistics
         */
        void reset() noexcept {
            *this = VariableTimestepStats{};
        }

        /**
         * @brief Calculate stability metric
         */
        void updateStability(const std::deque<Duration>& history) noexcept;
    };

    // =============================================================================
    // VariableTimestep Class
    // =============================================================================

    /**
     * @brief Variable timestep implementation
     * @details Provides smooth, clamped delta times with optional smoothing
     *          and spike detection for stable gameplay.
     */
    class VariableTimestep {
    public:
        /**
         * @brief Constructor with configuration
         * @param config Timestep configuration
         */
        explicit VariableTimestep(const VariableTimestepConfig& config = {}) noexcept;

        /**
         * @brief Destructor
         */
        ~VariableTimestep() = default;

        // Delete copy operations
        VariableTimestep(const VariableTimestep&) = delete;
        VariableTimestep& operator=(const VariableTimestep&) = delete;

        // Allow move operations
        VariableTimestep(VariableTimestep&&) noexcept = default;
        VariableTimestep& operator=(VariableTimestep&&) noexcept = default;

        // =============================================================================
        // Core Update
        // =============================================================================

        /**
         * @brief Update timestep with new frame time
         * @param rawDeltaTime Raw frame delta time
         * @return Processed delta time for this frame
         */
        [[nodiscard]] Duration update(Duration rawDeltaTime) noexcept;

        /**
         * @brief Reset timestep state
         * @details Clears history and resets statistics
         */
        void reset() noexcept;

        /**
         * @brief Advance time by fixed amount
         * @param deltaTime Time to advance
         * @return Scaled and clamped delta
         */
        [[nodiscard]] Duration advance(Duration deltaTime) noexcept;

        // =============================================================================
        // Time Queries
        // =============================================================================

        /**
         * @brief Get current delta time
         * @return Current processed delta time
         */
        [[nodiscard]] Duration getDeltaTime() const noexcept {
            return currentDeltaTime_;
        }

        /**
         * @brief Get smoothed delta time
         * @return Smoothed delta for stable updates
         */
        [[nodiscard]] Duration getSmoothedDeltaTime() const noexcept {
            return smoothedDeltaTime_;
        }

        /**
         * @brief Get unscaled delta time
         * @return Delta time before scaling
         */
        [[nodiscard]] Duration getUnscaledDeltaTime() const noexcept {
            return unscaledDeltaTime_;
        }

        /**
         * @brief Get current FPS
         * @return Frames per second
         */
        [[nodiscard]] double getFPS() const noexcept {
            return currentFPS_;
        }

        /**
         * @brief Get average FPS
         * @return Average FPS over window
         */
        [[nodiscard]] double getAverageFPS() const noexcept {
            return stats_.averageFPS;
        }

        /**
         * @brief Get interpolation alpha
         * @return Sub-frame interpolation factor [0,1]
         */
        [[nodiscard]] double getInterpolationAlpha() const noexcept {
            return config_.interpolationAlpha;
        }

        // =============================================================================
        // Configuration
        // =============================================================================

        /**
         * @brief Set time scale
         * @param scale Time scale factor
         */
        void setTimeScale(TimeScale scale) noexcept;

        /**
         * @brief Get time scale
         */
        [[nodiscard]] TimeScale getTimeScale() const noexcept {
            return config_.timeScale;
        }

        /**
         * @brief Set target FPS
         * @param fps Target frames per second
         */
        void setTargetFPS(double fps) noexcept;

        /**
         * @brief Set smoothing factor
         * @param factor Smoothing factor [0,1]
         */
        void setSmoothingFactor(double factor) noexcept;

        /**
         * @brief Enable/disable smoothing
         */
        void setSmoothing(const bool enable) noexcept {
            config_.enableSmoothing = enable;
        }

        /**
         * @brief Set delta time limits
         * @param minDelta Minimum delta time
         * @param maxDelta Maximum delta time
         */
        void setDeltaLimits(Duration minDelta, Duration maxDelta) noexcept;

        /**
         * @brief Get configuration
         */
        [[nodiscard]] const VariableTimestepConfig& getConfig() const noexcept {
            return config_;
        }

        /**
         * @brief Update configuration
         */
        void updateConfig(const VariableTimestepConfig& config) noexcept;

        // =============================================================================
        // Statistics
        // =============================================================================

        /**
         * @brief Get timestep statistics
         */
        [[nodiscard]] VariableTimestepStats getStats() const noexcept {
            std::lock_guard<std::mutex> lock(statsMutex_);
            return stats_;
        }

        /**
         * @brief Reset statistics
         */
        void resetStats() noexcept;

        /**
         * @brief Check if last frame was a spike
         */
        [[nodiscard]] bool wasSpike() const noexcept {
            return lastFrameWasSpike_;
        }

        /**
         * @brief Get frame timing stability
         * @return Stability metric [0,1] where 1 is perfectly stable
         */
        [[nodiscard]] double getStability() const noexcept {
            return stats_.stability;
        }

    private:
        // Configuration
        VariableTimestepConfig config_;

        // Current state
        Duration currentDeltaTime_{}; ///< Current processed delta
        Duration smoothedDeltaTime_{}; ///< Smoothed delta
        Duration unscaledDeltaTime_{}; ///< Delta before scaling
        double currentFPS_{0.0}; ///< Current FPS

        // History for smoothing
        std::deque<Duration> deltaHistory_; ///< Delta time history
        Duration historySum_{}; ///< Sum of history for average

        // Statistics
        mutable std::mutex statsMutex_; ///< Stats thread safety
        VariableTimestepStats stats_; ///< Performance statistics

        // State flags
        std::atomic<bool> lastFrameWasSpike_{false}; ///< Spike detection
        bool firstFrame_{true}; ///< First frame flag

        // =============================================================================
        // Internal Methods
        // =============================================================================

        /**
         * @brief Clamp delta time to configured limits
         */
        [[nodiscard]] Duration clampDeltaTime(Duration delta) const noexcept;

        /**
         * @brief Apply smoothing to delta time
         */
        [[nodiscard]] Duration smoothDeltaTime(Duration delta) const noexcept;

        /**
         * @brief Detect frame spike
         */
        [[nodiscard]] bool detectSpike(Duration delta) const noexcept;

        /**
         * @brief Update statistics
         */
        void updateStatistics(Duration delta) noexcept;

        /**
         * @brief Calculate FPS from delta
         */
        [[nodiscard]] static double calculateFPS(Duration delta) noexcept;

        /**
         * @brief Update delta history
         */
        void updateHistory(Duration delta) noexcept;
    };

} // namespace engine::time
