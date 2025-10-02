/**
 * @file HybridTimestep.cpp
 * @brief Implementation of hybrid timestep system
 * @details Combines fixed physics and variable rendering
 *
 * @author Development Team
 * @date Created on 2024-01-20
 */

#include "HybridTimestep.h"

#include <chrono>

namespace engine::time {

    // =============================================================================
    // HybridTimestep Implementation
    // =============================================================================

    HybridTimestep::HybridTimestep(const HybridTimestepConfig& config) noexcept :
        config_(config) {

        // Create timestep components
        fixedTimestep_ = std::make_unique<FixedTimestep>(config.fixedConfig);
        variableTimestep_ = std::make_unique<VariableTimestep>(config.variableConfig);

        // Initialize state
        currentMode_ = TimestepMode::HYBRID;
        previousMode_ = currentMode_;

        resetStats();
    }

    HybridUpdateResult HybridTimestep::update(
            const Duration frameDeltaTime,
            const PhysicsUpdateCallback& physicsCallback,
            const RenderUpdateCallback& renderCallback
            ) noexcept {

        // Check for mode switching if enabled
        if (config_.autoSwitchMode && shouldSwitchMode()) {
            // Determine best mode based on performance

            if (const double currentFPS = variableTimestep_->getFPS(); currentFPS < config_.switchThresholdFPS * 0.8) {
                switchMode(TimestepMode::VARIABLE);
            } else if (currentFPS > config_.switchThresholdFPS * 1.2) {
                switchMode(TimestepMode::HYBRID);
            }
        }

        // Update based on current mode
        HybridUpdateResult result;

        switch (currentMode_) {
            case TimestepMode::FIXED:
                result = updateFixedMode(frameDeltaTime, physicsCallback, renderCallback);
                break;

            case TimestepMode::VARIABLE:
                result = updateVariableMode(frameDeltaTime, physicsCallback, renderCallback);
                break;

            case TimestepMode::HYBRID:
            default:
                result = updateHybridMode(frameDeltaTime, physicsCallback, renderCallback);
                break;
        }

        // Update statistics
        updateStatistics(result, frameDeltaTime);

        // Adjust quality if enabled
        if (config_.adaptiveQuality) {
            adjustQuality();
        }

        return result;
    }

    std::uint32_t HybridTimestep::updateSimple(
            const Duration frameDeltaTime,
            const std::function<void(Duration)>& callback
            ) noexcept {

        if (!callback) {
            return 0;
        }

        // Simplified update using single callback
        const auto result = update(
                frameDeltaTime,
                // Physics callback
                [&callback](const Duration dt, std::uint32_t) {
                    callback(dt);
                },
                // Render callback (no-op for simple mode)
                [](Duration, double) {}
                );

        return result.fixedUpdates;
    }

    void HybridTimestep::reset() noexcept {
        fixedTimestep_->reset();
        variableTimestep_->reset();

        currentInterpolationAlpha_ = 0.0;
        smoothedInterpolationAlpha_ = 0.0;
        physicsTime_ = Duration::zero();
        renderTime_ = Duration::zero();
        lastModeSwitch_ = Duration::zero();
        modeStabilityTimer_ = Duration::zero();

        resetStats();
    }

    void HybridTimestep::setMode(const TimestepMode mode) noexcept {
        if (mode != currentMode_) {
            switchMode(mode);
        }
    }

    void HybridTimestep::setTimeScale(const TimeScale scale) noexcept {
        config_.fixedConfig.timeScale = scale;
        config_.variableConfig.timeScale = scale;

        fixedTimestep_->setTimeScale(scale);
        variableTimestep_->setTimeScale(scale);
    }

    void HybridTimestep::setPhysicsRate(const double updatesPerSecond) noexcept {
        fixedTimestep_->setUpdateRate(updatesPerSecond);
        config_.fixedConfig.timestep = fixedTimestep_->getUnscaledFixedDelta();
    }

    void HybridTimestep::setTargetFPS(double fps) noexcept {
        variableTimestep_->setTargetFPS(fps);
        config_.variableConfig.targetFPS = fps;
    }

    void HybridTimestep::updateConfig(const HybridTimestepConfig& config) noexcept {
        config_ = config;

        fixedTimestep_->updateConfig(config.fixedConfig);
        variableTimestep_->updateConfig(config.variableConfig);

        reset();
    }

    HybridTimestepStats HybridTimestep::getStats() noexcept {
        std::lock_guard lock(statsMutex_);

        stats_.fixedStats = fixedTimestep_->getStats();
        stats_.variableStats = variableTimestep_->getStats();
        stats_.physicsUtilization = physicsUtilization_;
        stats_.renderUtilization = renderUtilization_;
        stats_.currentMode = currentMode_;

        return stats_;
    }

    void HybridTimestep::resetStats() noexcept {
        std::lock_guard lock(statsMutex_);

        stats_.reset();
        fixedTimestep_->resetStats();
        variableTimestep_->resetStats();

        physicsUtilization_ = 0.0;
        renderUtilization_ = 0.0;
    }

    HybridUpdateResult HybridTimestep::updateHybridMode(
            const Duration frameDeltaTime,
            const PhysicsUpdateCallback& physicsCallback,
            const RenderUpdateCallback& renderCallback
            ) noexcept {

        HybridUpdateResult result;
        result.activeMode = TimestepMode::HYBRID;

        // Measure physics time
        const auto physicsStart = Clock::now();

        // Update fixed physics
        if (physicsCallback) {
            result.fixedUpdates = fixedTimestep_->update(frameDeltaTime, physicsCallback);
            result.physicsDeltaTime = fixedTimestep_->getFixedDeltaTime();
        }

        const auto physicsEnd = Clock::now();
        physicsTime_ = std::chrono::duration_cast<Duration>(physicsEnd - physicsStart);

        // Measure render time
        const auto renderStart = Clock::now();

        // Update variable rendering
        result.renderDeltaTime = variableTimestep_->update(frameDeltaTime);

        // Calculate interpolation alpha for smooth rendering
        currentInterpolationAlpha_ = fixedTimestep_->getInterpolationAlpha();

        // Smooth interpolation changes if enabled
        if (config_.smoothInterpolation) {
            smoothedInterpolationAlpha_ = static_cast<double>(TimeMath::lerp(
                    fromSeconds(smoothedInterpolationAlpha_),
                    fromSeconds(currentInterpolationAlpha_),
                    config_.interpolationDamping
                    ).count()) / 1000000.0;
        } else {
            smoothedInterpolationAlpha_ = currentInterpolationAlpha_;
        }

        result.interpolationAlpha = smoothedInterpolationAlpha_;

        // Execute render callback
        if (renderCallback) {
            renderCallback(result.renderDeltaTime, result.interpolationAlpha);
        }

        const auto renderEnd = Clock::now();
        renderTime_ = std::chrono::duration_cast<Duration>(renderEnd - renderStart);

        // Update performance metrics
        updatePerformanceMetrics(frameDeltaTime, physicsTime_, renderTime_);

        return result;
    }

    HybridUpdateResult HybridTimestep::updateFixedMode(
            const Duration frameDeltaTime,
            const PhysicsUpdateCallback& physicsCallback,
            const RenderUpdateCallback& renderCallback
            ) const noexcept {

        HybridUpdateResult result;
        result.activeMode = TimestepMode::FIXED;

        // In fixed-only mode, both physics and rendering use fixed timestep
        result.fixedUpdates = fixedTimestep_->update(
                frameDeltaTime,
                [&](const Duration dt, const std::uint32_t index) {
                    if (physicsCallback) {
                        physicsCallback(dt, index);
                    }
                    if (renderCallback && index == 0) {
                        // Render once per frame with no interpolation
                        renderCallback(dt, 0.0);
                    }
                }
                );

        result.physicsDeltaTime = fixedTimestep_->getFixedDeltaTime();
        result.renderDeltaTime = result.physicsDeltaTime;
        result.interpolationAlpha = 0.0;

        return result;
    }

    HybridUpdateResult HybridTimestep::updateVariableMode(
            const Duration frameDeltaTime,
            const PhysicsUpdateCallback& physicsCallback,
            const RenderUpdateCallback& renderCallback
            ) const noexcept {

        HybridUpdateResult result;
        result.activeMode = TimestepMode::VARIABLE;

        // In variable-only mode, both physics and rendering use variable timestep
        const Duration deltaTime = variableTimestep_->update(frameDeltaTime);

        result.renderDeltaTime = deltaTime;
        result.physicsDeltaTime = deltaTime;
        result.interpolationAlpha = 0.0;

        // Execute callbacks with variable timestep
        if (physicsCallback) {
            physicsCallback(deltaTime, 0);
            result.fixedUpdates = 1;
        }

        if (renderCallback) {
            renderCallback(deltaTime, 0.0);
        }

        return result;
    }

    bool HybridTimestep::shouldSwitchMode() const noexcept {
        // Check stability before switching
        if (modeStabilityTimer_ < config_.switchHysteresis) {
            return false;
        }

        // Check performance metrics
        const double fps = variableTimestep_->getFPS();
        const double stability = variableTimestep_->getStability();

        // Switch conditions based on current mode
        switch (currentMode_) {
            case TimestepMode::HYBRID:
                // Switch to variable if FPS is too low
                return fps < config_.switchThresholdFPS * 0.7;

            case TimestepMode::VARIABLE:
                // Switch back to hybrid if FPS recovered
                return fps > config_.switchThresholdFPS && stability > 0.8;

            case TimestepMode::FIXED:
                // Fixed mode is usually forced, don't auto-switch
                return false;

            default:
                return false;
        }
    }

    void HybridTimestep::switchMode(const TimestepMode newMode) noexcept {
        if (newMode == currentMode_) {
            return;
        }

        previousMode_ = currentMode_;
        currentMode_ = newMode;

        // Reset stability timer
        modeStabilityTimer_ = Duration::zero();
        lastModeSwitch_ = Duration::zero();

        // Update statistics
        {
            std::lock_guard lock(statsMutex_);
            stats_.modeSwitches++;
            stats_.currentMode = newMode;
        }

        // Notify callback
        if (modeChangeCallback_) {
            modeChangeCallback_(previousMode_, currentMode_);
        }

        // Reset interpolation when switching modes
        currentInterpolationAlpha_ = 0.0;
        smoothedInterpolationAlpha_ = 0.0;
    }

    void HybridTimestep::updateInterpolation() noexcept {
        if (!config_.enableInterpolation) {
            currentInterpolationAlpha_ = 0.0;
            smoothedInterpolationAlpha_ = 0.0;
            return;
        }

        // Get interpolation from fixed timestep
        currentInterpolationAlpha_ = fixedTimestep_->getInterpolationAlpha();

        // Apply smoothing
        if (config_.smoothInterpolation) {
            const double delta = currentInterpolationAlpha_ - smoothedInterpolationAlpha_;
            smoothedInterpolationAlpha_ += delta * config_.interpolationDamping;
        } else {
            smoothedInterpolationAlpha_ = currentInterpolationAlpha_;
        }
    }

    void HybridTimestep::updatePerformanceMetrics(
            const Duration frameDelta,
            const Duration physicsElapsed,
            const Duration renderElapsed
            ) noexcept {

        // Calculate utilization
        if (const double frameSeconds = toSeconds(frameDelta); frameSeconds > 0) {
            physicsUtilization_ = toSeconds(physicsElapsed) / frameSeconds;
            renderUtilization_ = toSeconds(renderElapsed) / frameSeconds;

            // Clamp to [0, 1]
            physicsUtilization_ = std::clamp(physicsUtilization_.load(), 0.0, 1.0);
            renderUtilization_ = std::clamp(renderUtilization_.load(), 0.0, 1.0);
        }

        // Update stability timer
        modeStabilityTimer_ += frameDelta;
        lastModeSwitch_ += frameDelta;
    }

    void HybridTimestep::updateStatistics(
            const HybridUpdateResult& result,
            const Duration frameDelta
            ) noexcept {

        std::lock_guard lock(statsMutex_);

        // Update mode timers
        switch (result.activeMode) {
            case TimestepMode::FIXED:
                stats_.timeInFixedMode += frameDelta;
                break;
            case TimestepMode::VARIABLE:
                stats_.timeInVariableMode += frameDelta;
                break;
            case TimestepMode::HYBRID:
                stats_.timeInHybridMode += frameDelta;
                break;
            default:
                break;
        }

        // Update performance metrics
        stats_.physicsUtilization = physicsUtilization_;
        stats_.renderUtilization = renderUtilization_;
        stats_.lastModeSwitch = lastModeSwitch_;

        // Calculate interpolation quality
        if (config_.enableInterpolation && result.activeMode == TimestepMode::HYBRID) {
            // Quality based on how smooth interpolation is
            const double alphaDiff = std::abs(currentInterpolationAlpha_ - smoothedInterpolationAlpha_);
            stats_.interpolationQuality = 1.0 - std::min(alphaDiff * 2.0, 1.0);
        } else {
            stats_.interpolationQuality = 1.0;
        }
    }

    void HybridTimestep::adjustQuality() const noexcept {
        if (!config_.adaptiveQuality) {
            return;
        }

        // Adjust based on utilization
        if (const double totalUtilization = physicsUtilization_ + renderUtilization_; totalUtilization > config_.targetUtilization * 1.1) {
            // Reduce quality - increase physics timestep slightly
            const Duration current = fixedTimestep_->getUnscaledFixedDelta();
            const auto increased = Duration(static_cast<Duration::rep>(static_cast<double>(current.count()) * 1.1));

            fixedTimestep_->setTimestep(increased);
        } else if (totalUtilization < config_.targetUtilization * 0.9) {
            // Increase quality - decrease physics timestep slightly
            const Duration current = fixedTimestep_->getUnscaledFixedDelta();
            const auto decreased = Duration(static_cast<Duration::rep>(static_cast<double>(current.count()) * 0.95));

            fixedTimestep_->setTimestep(decreased);
        }
    }

} // namespace engine::time
