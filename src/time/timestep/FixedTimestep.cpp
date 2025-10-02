/**
 * @file FixedTimestep.cpp
 * @brief Implementation of fixed timestep system
 * @details Provides deterministic timing with accumulator pattern
 *
 * @author Development Team
 * @date Created on 2024-01-20
 */

#include "FixedTimestep.h"
#include <algorithm>

namespace engine::time {

    // =============================================================================
    // FixedTimestepStats Implementation
    // =============================================================================

    void FixedTimestepStats::updateEfficiency() noexcept {
        // Calculate efficiency based on update distribution
        // Perfect efficiency = exactly the expected updates per frame
        // Lower efficiency = too many or too few updates

        if (totalFrames == 0 || totalUpdates == 0) {
            efficiency = 1.0;
            return;
        }

        const double avgUpdatesPerFrame = static_cast<double>(totalUpdates) /
                static_cast<double>(totalFrames);
        const double expectedUpdatesPerFrame = toSeconds(averageFrameTime) /
                toSeconds(accumulator);

        if (expectedUpdatesPerFrame > 0) {
            const double ratio = avgUpdatesPerFrame / expectedUpdatesPerFrame;
            // Map ratio to efficiency (1.0 = perfect, 0.0 = very inefficient)
            efficiency = 1.0 - std::abs(1.0 - ratio);
            efficiency = std::clamp(efficiency, 0.0, 1.0);
        }
    }

    // =============================================================================
    // FixedTimestep Implementation
    // =============================================================================

    FixedTimestep::FixedTimestep(const FixedTimestepConfig& config) noexcept :
        config_(config) {

        // Calculate scaled fixed delta
        scaledFixedDelta_ = TimeMath::scale(config_.timestep, config_.timeScale);

        // Initialize accumulator
        accumulator_ = Duration::zero();

        resetStats();
    }

    std::uint32_t FixedTimestep::update(
            const Duration frameDeltaTime,
            const FixedUpdateCallback& updateCallback
            ) noexcept {

        if (!updateCallback) {
            return advance(frameDeltaTime);
        }

        // Handle first frame
        if (firstFrame_) {
            firstFrame_ = false;
            // Don't accumulate on first frame to avoid large initial delta
            accumulator_ = Duration::zero();
        }

        // Process with accumulator
        const std::uint32_t updates = processAccumulator(frameDeltaTime, updateCallback);

        // Update statistics
        updateStatistics(frameDeltaTime, updates);

        return updates;
    }

    std::uint32_t FixedTimestep::advance(const Duration frameDeltaTime) noexcept {
        // Scale frame delta by time scale
        const Duration scaledFrameDelta = TimeMath::scale(frameDeltaTime, config_.timeScale);

        // Add to accumulator
        accumulator_ += scaledFrameDelta;

        // Clamp accumulator
        clampAccumulator();

        // Calculate potential updates without executing
        std::uint32_t potentialUpdates = 0;
        Duration tempAccumulator = accumulator_;

        while (tempAccumulator >= scaledFixedDelta_ &&
            potentialUpdates < config_.maxIterationsPerFrame) {
            tempAccumulator -= scaledFixedDelta_;
            potentialUpdates++;
        }

        return potentialUpdates;
    }

    void FixedTimestep::reset() noexcept {
        accumulator_ = Duration::zero();
        updatesThisFrame_ = 0;
        firstFrame_ = true;
        inSpiral_ = false;

        resetStats();
    }

    void FixedTimestep::forceUpdate() noexcept {
        // Force a single update regardless of accumulator
        updatesThisFrame_ = 1;

        // Update statistics
        std::lock_guard lock(statsMutex_);
        stats_.totalUpdates++;
        stats_.updatesThisFrame = 1;
    }

    void FixedTimestep::setTimestep(const Duration deltaTime) noexcept {
        // Clamp to reasonable range
        config_.timestep = std::clamp(
                deltaTime,
                Duration(1000),
                // 1ms minimum (1000 Hz)
                Duration(100000) // 100ms maximum (10 Hz)
                );

        // Recalculate scaled delta
        scaledFixedDelta_ = TimeMath::scale(config_.timestep, config_.timeScale);
    }

    void FixedTimestep::setUpdateRate(const double updatesPerSecond) noexcept {
        if (updatesPerSecond > 0) {
            const Duration deltaTime = fromSeconds(1.0 / updatesPerSecond);
            setTimestep(deltaTime);
        }
    }

    void FixedTimestep::setTimeScale(const TimeScale scale) noexcept {
        config_.timeScale = std::clamp(scale, 0.0, constants::MAX_TIME_SCALE);

        // Recalculate scaled delta
        scaledFixedDelta_ = TimeMath::scale(config_.timestep, config_.timeScale);
    }

    void FixedTimestep::setMaxIterations(const std::uint8_t maxIterations) noexcept {
        config_.maxIterationsPerFrame = std::clamp(
                maxIterations,
                static_cast<std::uint8_t>(1),
                static_cast<std::uint8_t>(10)
                );
    }

    void FixedTimestep::updateConfig(const FixedTimestepConfig& config) noexcept {
        config_ = config;

        // Validate and adjust
        setTimestep(config_.timestep);
        setTimeScale(config_.timeScale);
        setMaxIterations(config_.maxIterationsPerFrame);

        // Reset state for new configuration
        reset();
    }

    void FixedTimestep::resetStats() noexcept {
        std::lock_guard lock(statsMutex_);
        stats_.reset();
    }

    std::uint32_t FixedTimestep::processAccumulator(
            const Duration frameDelta,
            const FixedUpdateCallback& callback
            ) noexcept {

        // Scale frame delta by time scale
        const Duration scaledFrameDelta = TimeMath::scale(frameDelta, config_.timeScale);

        // Add to accumulator if using accumulator pattern
        if (config_.useAccumulator) {
            accumulator_ += scaledFrameDelta;
        } else {
            // Direct mode: always do exactly one update
            callback(scaledFixedDelta_, 0);
            updatesThisFrame_ = 1;
            return 1;
        }

        // Clamp accumulator to prevent spiral
        clampAccumulator();

        // Check for spiral of death
        if (config_.preventSpiral && accumulator_ >= config_.spiralThreshold) {
            handleSpiral();
        }

        // Process fixed updates
        std::uint32_t updates = 0;

        while (accumulator_ >= scaledFixedDelta_ &&
            updates < config_.maxIterationsPerFrame) {

            // Execute fixed update
            callback(scaledFixedDelta_, updates);

            // Consume from accumulator
            accumulator_ -= scaledFixedDelta_;
            updates++;
        }

        // Handle partial updates if enabled
        if (config_.allowPartialUpdates &&
            accumulator_ > Duration::zero() &&
            updates == 0) {

            // Do a partial update with remaining time
            callback(accumulator_, 0);
            accumulator_ = Duration::zero();
            updates = 1;

            std::lock_guard lock(statsMutex_);
            stats_.partialUpdates++;
        }

        // Track dropped updates
        if (accumulator_ >= scaledFixedDelta_) {
            std::lock_guard lock(statsMutex_);
            const auto dropped = static_cast<std::uint32_t>(
                accumulator_ / scaledFixedDelta_
            );
            stats_.droppedUpdates += dropped;
        }

        updatesThisFrame_ = updates;
        return updates;
    }

    void FixedTimestep::handleSpiral() noexcept {
        inSpiral_ = true;

        // Cap accumulator to prevent runaway
        accumulator_ = std::min(accumulator_, config_.maxAccumulator);

        // Record spiral occurrence
        std::lock_guard lock(statsMutex_);
        stats_.spiralCount++;

        // Optionally reset accumulator to recover
        if (accumulator_ > config_.maxAccumulator * 2) {
            accumulator_ = scaledFixedDelta_; // Reset to one update worth
        }
    }

    void FixedTimestep::updateStatistics(const Duration frameDelta, const std::uint32_t updates) noexcept {
        std::lock_guard lock(statsMutex_);

        // Update counters
        stats_.totalUpdates += updates;
        stats_.totalFrames++;
        stats_.updatesThisFrame = updates;
        stats_.totalTime += frameDelta;

        // Track max updates per frame
        if (updates > stats_.maxUpdatesPerFrame) {
            stats_.maxUpdatesPerFrame = updates;
        }

        // Update timing
        stats_.accumulator = accumulator_;
        stats_.lastFrameTime = frameDelta;

        // Calculate averages
        if (stats_.totalFrames > 0) {
            stats_.averageFrameTime = stats_.totalTime / static_cast<long>(stats_.totalFrames);

            if (const double totalSeconds = toSeconds(stats_.totalTime); totalSeconds > 0) {
                stats_.updateRate = static_cast<double>(stats_.totalUpdates) / totalSeconds;
                stats_.frameRate = static_cast<double>(stats_.totalFrames) / totalSeconds;
            }
        }

        // Update efficiency
        stats_.updateEfficiency();

        // Clear spiral flag if recovered
        if (inSpiral_ && accumulator_ < config_.spiralThreshold) {
            inSpiral_ = false;
        }
    }

    void FixedTimestep::clampAccumulator() noexcept {
        // Clamp to maximum accumulator
        if (config_.maxAccumulator > Duration::zero()) {
            accumulator_ = std::min(accumulator_, config_.maxAccumulator);
        }

        // Prevent negative accumulator
        if (!config_.allowNegativeTime && accumulator_ < Duration::zero()) {
            accumulator_ = Duration::zero();
        }
    }

} // namespace engine::time
