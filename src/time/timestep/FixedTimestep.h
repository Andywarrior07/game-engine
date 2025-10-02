/**
 * @file FixedTimestep.h
 * @brief Fixed timestep implementation for deterministic updates
 * @details Provides fixed timestep updates with accumulator pattern
 *          for deterministic physics and gameplay. Ensures consistent
 *          behavior across different frame rates.
 *
 * @author Development Team
 * @date Created on 2024-01-20
 */

#pragma once

#include "../core/TimeTypes.h"
#include "../core/TimestepMode.h"
#include "../utils/TimeMath.h"

#include <functional>
#include <atomic>
#include <mutex>

namespace engine::time {
    // =============================================================================
    // Fixed Timestep Statistics
    // =============================================================================

    /**
     * @brief Statistics for fixed timestep performance
     */
    struct FixedTimestepStats {
        // Update counts
        std::uint64_t totalUpdates{0}; ///< Total fixed updates
        std::uint64_t totalFrames{0}; ///< Total frames processed
        std::uint32_t updatesThisFrame{0}; ///< Updates in current frame
        std::uint32_t maxUpdatesPerFrame{0}; ///< Maximum updates in one frame

        // Timing
        Duration accumulator{}; ///< Current accumulator value
        Duration lastFrameTime{}; ///< Last frame duration
        Duration averageFrameTime{}; ///< Average frame time
        Duration totalTime{}; ///< Total elapsed time

        // Performance
        double updateRate{0.0}; ///< Updates per second
        double frameRate{0.0}; ///< Frames per second
        double efficiency{1.0}; ///< Update efficiency (1.0 = perfect)

        // Issues
        std::uint64_t spiralCount{0}; ///< Death spiral occurrences
        std::uint64_t droppedUpdates{0}; ///< Dropped updates due to limits
        std::uint64_t partialUpdates{0}; ///< Partial update count

        /**
         * @brief Reset statistics
         */
        void reset() noexcept {
            *this = FixedTimestepStats{};
        }

        /**
         * @brief Calculate efficiency metric
         */
        void updateEfficiency() noexcept;
    };

    // =============================================================================
    // Update Callback
    // =============================================================================

    /**
     * @brief Fixed update callback signature
     * @param deltaTime Fixed delta time
     * @param updateIndex Index of update in current frame
     */
    using FixedUpdateCallback = std::function<void(Duration deltaTime, std::uint32_t updateIndex)>;

    // =============================================================================
    // FixedTimestep Class
    // =============================================================================

    /**
     * @brief Fixed timestep implementation
     * @details Implements the accumulator pattern for deterministic updates
     *          with interpolation support for smooth rendering.
     */
    class FixedTimestep {
    public:
        /**
         * @brief Constructor with configuration
         * @param config Timestep configuration
         */
        explicit FixedTimestep(const FixedTimestepConfig& config = {}) noexcept;

        /**
         * @brief Destructor
         */
        ~FixedTimestep() = default;

        // Delete copy operations
        FixedTimestep(const FixedTimestep&) = delete;
        FixedTimestep& operator=(const FixedTimestep&) = delete;

        // Allow move operations
        FixedTimestep(FixedTimestep&&) noexcept = default;
        FixedTimestep& operator=(FixedTimestep&&) noexcept = default;

        // =============================================================================
        // Core Update
        // =============================================================================

        /**
         * @brief Process frame with fixed timestep
         * @param frameDeltaTime Frame delta time
         * @param updateCallback Callback for each fixed update
         * @return Number of fixed updates performed
         */
        std::uint32_t update(
                Duration frameDeltaTime,
                const FixedUpdateCallback& updateCallback
                ) noexcept;

        /**
         * @brief Advance by fixed timestep without callback
         * @param frameDeltaTime Frame delta time
         * @return Number of updates that would be performed
         */
        std::uint32_t advance(Duration frameDeltaTime) noexcept;

        /**
         * @brief Reset timestep state
         * @details Clears accumulator and resets statistics
         */
        void reset() noexcept;

        /**
         * @brief Force a single fixed update
         * @details Bypasses accumulator for immediate update
         */
        void forceUpdate() noexcept;

        // =============================================================================
        // Time Queries
        // =============================================================================

        /**
         * @brief Get fixed delta time
         * @return Fixed timestep duration
         */
        [[nodiscard]] Duration getFixedDeltaTime() const noexcept {
            return scaledFixedDelta_;
        }

        /**
         * @brief Get unscaled fixed delta
         * @return Fixed delta before time scaling
         */
        [[nodiscard]] Duration getUnscaledFixedDelta() const noexcept {
            return config_.timestep;
        }

        /**
         * @brief Get current accumulator value
         * @return Accumulated time waiting for update
         */
        [[nodiscard]] Duration getAccumulator() const noexcept {
            return accumulator_;
        }

        /**
         * @brief Get interpolation alpha
         * @return Interpolation factor [0,1] for rendering
         */
        [[nodiscard]] double getInterpolationAlpha() const noexcept {
            if (!config_.enableInterpolation)
                return 0.0;

            const double ratio = toSeconds(accumulator_) / toSeconds(scaledFixedDelta_);
            return std::clamp(ratio, 0.0, 1.0);
        }

        /**
         * @brief Get updates performed this frame
         * @return Number of fixed updates in current frame
         */
        [[nodiscard]] std::uint32_t getUpdatesThisFrame() const noexcept {
            return updatesThisFrame_;
        }

        /**
         * @brief Check if in spiral of death
         * @return True if experiencing death spiral
         */
        [[nodiscard]] bool isInSpiral() const noexcept {
            return inSpiral_;
        }

        // =============================================================================
        // Configuration
        // =============================================================================

        /**
         * @brief Set fixed timestep
         * @param deltaTime Fixed update interval
         */
        void setTimestep(Duration deltaTime) noexcept;

        /**
         * @brief Set fixed timestep from target rate
         * @param updatesPerSecond Target update rate
         */
        void setUpdateRate(double updatesPerSecond) noexcept;

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
         * @brief Set maximum iterations per frame
         * @param maxIterations Maximum updates allowed per frame
         */
        void setMaxIterations(std::uint8_t maxIterations) noexcept;

        /**
         * @brief Enable/disable interpolation
         */
        void setInterpolation(const bool enable) noexcept {
            config_.enableInterpolation = enable;
        }

        /**
         * @brief Get configuration
         */
        [[nodiscard]] const FixedTimestepConfig& getConfig() const noexcept {
            return config_;
        }

        /**
         * @brief Update configuration
         */
        void updateConfig(const FixedTimestepConfig& config) noexcept;

        // =============================================================================
        // Statistics
        // =============================================================================

        /**
         * @brief Get timestep statistics
         */
        [[nodiscard]] FixedTimestepStats getStats() const noexcept {
            std::lock_guard lock(statsMutex_);
            return stats_;
        }

        /**
         * @brief Reset statistics
         */
        void resetStats() noexcept;

        /**
         * @brief Get update rate (updates per second)
         */
        [[nodiscard]] double getUpdateRate() const noexcept {
            const double seconds = toSeconds(config_.timestep);
            return seconds > 0 ? 1.0 / seconds : 0.0;
        }

    private:
        // Configuration
        FixedTimestepConfig config_;

        // Current state
        Duration accumulator_{}; ///< Time accumulator
        Duration scaledFixedDelta_{}; ///< Scaled fixed delta
        std::uint32_t updatesThisFrame_{0}; ///< Updates in current frame

        // State flags
        std::atomic<bool> inSpiral_{false}; ///< Death spiral detected
        bool firstFrame_{true}; ///< First frame flag

        // Statistics
        mutable std::mutex statsMutex_; ///< Stats thread safety
        FixedTimestepStats stats_; ///< Performance statistics

        // =============================================================================
        // Internal Methods
        // =============================================================================

        /**
         * @brief Process accumulator with callback
         */
        std::uint32_t processAccumulator(
                Duration frameDelta,
                const FixedUpdateCallback& callback
                ) noexcept;

        /**
         * @brief Detect and handle spiral of death
         */
        void handleSpiral() noexcept;

        /**
         * @brief Update statistics
         */
        void updateStatistics(Duration frameDelta, std::uint32_t updates) noexcept;

        /**
         * @brief Clamp accumulator to safe range
         */
        void clampAccumulator() noexcept;
    };

} // namespace engine::time
