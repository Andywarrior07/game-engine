/**
 * @file VariableTimestep.cpp
 * @brief Implementation of variable timestep system
 * @details Provides smooth frame timing with spike detection
 *
 * @author Development Team
 * @date Created on 2024-01-20
 */

#include "VariableTimestep.h"

#include "../utils/TimeMath.h"

#include <cmath>

namespace engine::time {

    // =============================================================================
    // VariableTimestepStats Implementation
    // =============================================================================

    void VariableTimestepStats::updateStability(const std::deque<Duration>& history) noexcept {
        if (history.size() < 2) {
            stability = 1.0;
            return;
        }

        // Calculate standard deviation
        double sum = 0.0;
        for (const auto& delta : history) {
            sum += toSeconds(delta);
        }
        const double mean = sum / static_cast<double>(history.size());

        double variance = 0.0;
        for (const auto& delta : history) {
            const double diff = toSeconds(delta) - mean;
            variance += diff * diff;
        }
        variance /= static_cast<double>(history.size());

        const double stdDev = std::sqrt(variance);

        // Normalize to 0-1 range (lower std dev = higher stability)
        // Use coefficient of variation for scale-independent measure
        const double cv = (mean > 0) ? (stdDev / mean) : 0.0;

        // Map CV to stability (0.0 CV = 1.0 stability, 0.5+ CV = 0.0 stability)
        stability = std::clamp(1.0 - (cv * 2.0), 0.0, 1.0);
    }

    // =============================================================================
    // VariableTimestep Implementation
    // =============================================================================

    VariableTimestep::VariableTimestep(const VariableTimestepConfig& config) noexcept :
        config_(config) {

        // Reserve space for history
        deltaHistory_.clear();
        if (config_.smoothingWindowSize > 0) {
            // Pre-allocate doesn't work with deque, but clear ensures it's empty
            deltaHistory_.clear();
        }

        // Initialize target frame time from FPS if specified
        if (config_.targetFPS.has_value() && config_.targetFPS.value() > 0) {
            config_.targetFrameTime = fromSeconds(1.0 / config_.targetFPS.value());
        }

        resetStats();
    }

    Duration VariableTimestep::update(const Duration rawDeltaTime) noexcept {
        // Handle first frame
        if (firstFrame_) {
            firstFrame_ = false;
            currentDeltaTime_ = config_.targetFrameTime;
            smoothedDeltaTime_ = currentDeltaTime_;
            unscaledDeltaTime_ = currentDeltaTime_;
            currentFPS_ = calculateFPS(currentDeltaTime_);

            updateHistory(currentDeltaTime_);
            updateStatistics(currentDeltaTime_);

            return currentDeltaTime_;
        }

        // Store unscaled delta
        unscaledDeltaTime_ = rawDeltaTime;

        // Clamp delta time to configured limits
        const Duration clampedDelta = clampDeltaTime(rawDeltaTime);

        // Detect spikes before smoothing
        lastFrameWasSpike_ = detectSpike(clampedDelta);

        // Apply smoothing if enabled
        Duration processedDelta = config_.enableSmoothing
                ? smoothDeltaTime(clampedDelta)
                : clampedDelta;

        // Apply time scaling
        processedDelta = TimeMath::scale(processedDelta, config_.timeScale);

        // Handle negative time if not allowed
        if (!config_.allowNegativeTime && processedDelta < Duration::zero()) {
            processedDelta = Duration::zero();
        }

        // Update state
        currentDeltaTime_ = processedDelta;
        smoothedDeltaTime_ = processedDelta;
        currentFPS_ = calculateFPS(clampedDelta); // FPS from unscaled time

        // Update interpolation alpha for sub-frame interpolation
        if (config_.enableInterpolation && config_.targetFrameTime > Duration::zero()) {
            const double ratio = toSeconds(rawDeltaTime) / toSeconds(config_.targetFrameTime);
            config_.interpolationAlpha = std::fmod(ratio, 1.0);
        }

        // Update statistics
        updateHistory(clampedDelta);
        updateStatistics(clampedDelta);

        return currentDeltaTime_;
    }

    void VariableTimestep::reset() noexcept {
        currentDeltaTime_ = Duration::zero();
        smoothedDeltaTime_ = Duration::zero();
        unscaledDeltaTime_ = Duration::zero();
        currentFPS_ = 0.0;

        deltaHistory_.clear();
        historySum_ = Duration::zero();

        firstFrame_ = true;
        lastFrameWasSpike_ = false;

        resetStats();
    }

    Duration VariableTimestep::advance(const Duration deltaTime) noexcept {
        // Simple advance without history update
        Duration processedDelta = clampDeltaTime(deltaTime);
        processedDelta = TimeMath::scale(processedDelta, config_.timeScale);

        if (!config_.allowNegativeTime && processedDelta < Duration::zero()) {
            processedDelta = Duration::zero();
        }

        currentDeltaTime_ = processedDelta;
        currentFPS_ = calculateFPS(deltaTime);

        return processedDelta;
    }

    void VariableTimestep::setTimeScale(const TimeScale scale) noexcept {
        config_.timeScale = std::clamp(scale, 0.0, constants::MAX_TIME_SCALE);
    }

    void VariableTimestep::setTargetFPS(double fps) noexcept {
        if (fps > 0) {
            config_.targetFPS = fps;
            config_.targetFrameTime = fromSeconds(1.0 / fps);
        } else {
            config_.targetFPS = std::nullopt;
        }
    }

    void VariableTimestep::setSmoothingFactor(const double factor) noexcept {
        config_.smoothingFactor = std::clamp(factor, 0.0, 1.0);
    }

    void VariableTimestep::setDeltaLimits(const Duration minDelta, const Duration maxDelta) noexcept {
        config_.minDeltaTime = std::max(minDelta, Duration(1)); // At least 1 microsecond
        config_.maxDeltaTime = std::min(maxDelta, Duration(1000000)); // At most 1 second

        // Ensure min <= max
        if (config_.minDeltaTime > config_.maxDeltaTime) {
            std::swap(config_.minDeltaTime, config_.maxDeltaTime);
        }
    }

    void VariableTimestep::updateConfig(const VariableTimestepConfig& config) noexcept {
        config_ = config;

        // Validate and adjust if needed
        setDeltaLimits(config_.minDeltaTime, config_.maxDeltaTime);
        setTimeScale(config_.timeScale);
        setSmoothingFactor(config_.smoothingFactor);

        if (config_.targetFPS.has_value()) {
            setTargetFPS(config_.targetFPS.value());
        }
    }

    void VariableTimestep::resetStats() noexcept {
        std::lock_guard lock(statsMutex_);
        stats_.reset();
    }

    Duration VariableTimestep::clampDeltaTime(const Duration delta) const noexcept {
        if (delta < config_.minDeltaTime) {
            return config_.minDeltaTime;
        }
        if (delta > config_.maxDeltaTime) {
            return config_.maxDeltaTime;
        }
        return delta;
    }

    Duration VariableTimestep::smoothDeltaTime(const Duration delta) const noexcept {
        if (!config_.enableSmoothing || deltaHistory_.empty()) {
            return delta;
        }

        // Exponential smoothing
        if (config_.smoothingFactor > 0) {
            const double currentSeconds = toSeconds(delta);
            const double previousSeconds = toSeconds(smoothedDeltaTime_);
            const double smoothed = static_cast<double>(TimeMath::lerp(
                    fromSeconds(previousSeconds),
                    fromSeconds(currentSeconds),
                    config_.smoothingFactor
                    ).count()) / 1000000.0; // Convert back to seconds

            return fromSeconds(smoothed);
        }

        // Moving average smoothing
        if (!deltaHistory_.empty()) {
            return historySum_ / static_cast<long>(deltaHistory_.size());
        }

        return delta;
    }

    bool VariableTimestep::detectSpike(const Duration delta) const noexcept {
        if (!config_.detectSpikes || deltaHistory_.empty()) {
            return false;
        }

        // Compare against average
        const Duration average = historySum_ / static_cast<long>(deltaHistory_.size());
        const double ratio = toSeconds(delta) / toSeconds(average);

        return ratio > config_.spikeThreshold;
    }

    void VariableTimestep::updateStatistics(const Duration delta) noexcept {
        std::lock_guard lock(statsMutex_);

        // Update current values
        stats_.currentDeltaTime = currentDeltaTime_;
        stats_.smoothedDeltaTime = smoothedDeltaTime_;
        stats_.currentFPS = currentFPS_;

        // Update min/max
        if (delta < stats_.minDeltaTime) {
            stats_.minDeltaTime = delta;
        }
        if (delta > stats_.maxDeltaTime) {
            stats_.maxDeltaTime = delta;
        }

        if (const double fps = currentFPS_; fps > 0) {
            stats_.minFPS = std::min(stats_.minFPS, fps);
            stats_.maxFPS = std::max(stats_.maxFPS, fps);
        }

        // Update counters
        stats_.frameCount++;
        stats_.totalTime += delta;

        if (lastFrameWasSpike_) {
            stats_.spikeCount++;
        }

        // Calculate averages
        if (!deltaHistory_.empty()) {
            stats_.averageDeltaTime = historySum_ / static_cast<long>(deltaHistory_.size());
            stats_.averageFPS = calculateFPS(stats_.averageDeltaTime);
        }

        // Update stability
        stats_.updateStability(deltaHistory_);

        // Store effective time scale
        stats_.timeScaleEffective = config_.timeScale;
    }

    double VariableTimestep::calculateFPS(const Duration delta) noexcept {
        const double seconds = toSeconds(delta);
        if (seconds <= 0.0) {
            return 0.0;
        }
        return 1.0 / seconds;
    }

    void VariableTimestep::updateHistory(const Duration delta) noexcept {
        // Add to history
        deltaHistory_.push_back(delta);
        historySum_ += delta;

        // Maintain window size
        while (deltaHistory_.size() > config_.smoothingWindowSize) {
            historySum_ -= deltaHistory_.front();
            deltaHistory_.pop_front();
        }
    }

} // namespace engine::time
