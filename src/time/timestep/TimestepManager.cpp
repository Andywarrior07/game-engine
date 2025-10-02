/**
 * @file TimestepManager.cpp
 * @brief Implementation of central timestep management system
 * @details Manages all timestep strategies with performance optimization
 *
 * @author Development Team
 * @date Created on 2024-01-20
 */

#include "TimestepManager.h"
#include <iostream>

namespace engine::time {

    // =============================================================================
    // TimestepManager Implementation
    // =============================================================================

    TimestepManager::TimestepManager(const TimestepManagerConfig& config) :
        config_(config)
        , currentMode_(config.initialMode)
        , previousMode_(config.initialMode)
        , totalRuntime_(Duration::zero()) {

        // Apply preset if specified
        if (config.preset != TimestepPreset::CUSTOM) {
            applyPreset(config.preset);
        }

        lastModeSwitch_ = Clock::now();
        lastOptimization_ = Clock::now();
    }

    TimestepManager::~TimestepManager() {
        shutdown();
    }

    bool TimestepManager::initialize(engine::memory::MemoryManager* memoryManager) {
        if (initialized_) {
            return true;
        }

        memoryManager_ = memoryManager ? memoryManager : config_.memoryManager;
        if (!memoryManager_) {
            std::cerr << "[TimestepManager] Error: No memory manager provided!" << std::endl;
            return false;
        }

        // Create timestep components
        if (!createComponents()) {
            std::cerr << "[TimestepManager] Failed to create timestep components!" << std::endl;
            return false;
        }

        // Reset metrics
        metrics_.reset();
        totalRuntime_ = Duration::zero();

        initialized_ = true;

        std::cout << "[TimestepManager] Initialized with mode: "
                << getTimestepModeName(currentMode_) << std::endl;

        return true;
    }

    void TimestepManager::shutdown() {
        if (!initialized_) {
            return;
        }

        destroyComponents();

        initialized_ = false;

        std::cout << "[TimestepManager] Shutdown complete" << std::endl;
    }

    TimestepUpdateResult TimestepManager::update(
            const Duration frameDeltaTime,
            const std::function<void(Duration, std::uint32_t)>& fixedUpdate,
            const std::function<void(Duration, double)>& variableUpdate
            ) {

        if (!initialized_) {
            return {};
        }

        // Check pending mode switch
        if (modeSwitchPending_ && canSwitchMode()) {
            performModeSwitch(pendingMode_);
            modeSwitchPending_ = false;
        }

        // Update based on current mode
        TimestepUpdateResult result;

        switch (currentMode_) {
            case TimestepMode::VARIABLE:
                result = updateVariable(frameDeltaTime, variableUpdate);
                break;

            case TimestepMode::FIXED:
                result = updateFixed(frameDeltaTime, fixedUpdate);
                break;

            case TimestepMode::HYBRID:
                result = updateHybrid(frameDeltaTime, fixedUpdate, variableUpdate);
                break;

            default:
                result.mode = currentMode_;
                result.deltaTime = frameDeltaTime;
                break;
        }

        // Update metrics
        updateMetrics(result, frameDeltaTime);

        // Check for optimization
        if (config_.autoOptimize) {
            checkOptimization();
        }

        return result;
    }

    Duration TimestepManager::updateSimple(const Duration frameDeltaTime) {
        const auto result = update(
                frameDeltaTime,
                nullptr,
                nullptr
                );

        return result.deltaTime;
    }

    void TimestepManager::reset() {
        if (variableTimestep_) {
            variableTimestep_->reset();
        }
        if (fixedTimestep_) {
            fixedTimestep_->reset();
        }
        if (hybridTimestep_) {
            hybridTimestep_->reset();
        }

        resetMetrics();
        totalRuntime_ = Duration::zero();
        lastModeSwitch_ = Clock::now();
        lastOptimization_ = Clock::now();
    }

    bool TimestepManager::setMode(const TimestepMode mode, const bool immediate) {
        if (mode == currentMode_) {
            return false;
        }

        if (!config_.allowRuntimeSwitch) {
            return false;
        }

        if (immediate || canSwitchMode()) {
            performModeSwitch(mode);
            return true;
        } else {
            // Queue the switch for later
            pendingMode_ = mode;
            modeSwitchPending_ = true;
            return false;
        }
    }

    void TimestepManager::applyPreset(TimestepPreset preset) {
        const auto presetConfig = getTimestepPreset(preset);

        // Update mode
        currentMode_ = presetConfig.mode;

        // Extract and apply configurations based on mode
        if (auto varConfig = presetConfig.getVariableConfig()) {
            config_.variableConfig = *varConfig;
            if (variableTimestep_) {
                variableTimestep_->updateConfig(config_.variableConfig);
            }
        }

        if (auto fixedConfig = presetConfig.getFixedConfig()) {
            config_.fixedConfig = *fixedConfig;
            if (fixedTimestep_) {
                fixedTimestep_->updateConfig(config_.fixedConfig);
            }
        }

        if (auto hybridConfig = presetConfig.getHybridConfig()) {
            config_.hybridConfig = *hybridConfig;
            if (hybridTimestep_) {
                hybridTimestep_->updateConfig(config_.hybridConfig);
            }
        }

        std::cout << "[TimestepManager] Applied preset: "
                << static_cast<int>(preset) << std::endl;
    }

    bool TimestepManager::canSwitchMode() const noexcept {
        if (!config_.allowRuntimeSwitch) {
            return false;
        }

        const auto now = Clock::now();
        const auto timeSinceSwitch = now - lastModeSwitch_;

        return timeSinceSwitch >= config_.switchCooldown;
    }

    Duration TimestepManager::getDeltaTime() const noexcept {
        switch (currentMode_) {
            case TimestepMode::VARIABLE:
                return variableTimestep_ ? variableTimestep_->getDeltaTime() : Duration::zero();

            case TimestepMode::FIXED:
                return fixedTimestep_ ? fixedTimestep_->getFixedDeltaTime() : Duration::zero();

            case TimestepMode::HYBRID:
                return hybridTimestep_ ? hybridTimestep_->getRenderDeltaTime() : Duration::zero();

            default:
                return Duration::zero();
        }
    }

    Duration TimestepManager::getFixedDeltaTime() const noexcept {
        if (fixedTimestep_) {
            return fixedTimestep_->getFixedDeltaTime();
        }
        if (hybridTimestep_) {
            return hybridTimestep_->getPhysicsDeltaTime();
        }
        return constants::DEFAULT_FIXED_TIMESTEP;
    }

    double TimestepManager::getInterpolationAlpha() const noexcept {
        switch (currentMode_) {
            case TimestepMode::FIXED:
                return fixedTimestep_ ? fixedTimestep_->getInterpolationAlpha() : 0.0;

            case TimestepMode::HYBRID:
                return hybridTimestep_ ? hybridTimestep_->getInterpolationAlpha() : 0.0;

            default:
                return 0.0;
        }
    }

    double TimestepManager::getCurrentFPS() const noexcept {
        switch (currentMode_) {
            case TimestepMode::VARIABLE:
                return variableTimestep_ ? variableTimestep_->getFPS() : 0.0;

            case TimestepMode::HYBRID:
                return hybridTimestep_ ? hybridTimestep_->getVariableTimestep()->getFPS() : 0.0;

            case TimestepMode::FIXED:
                return fixedTimestep_ ? fixedTimestep_->getUpdateRate() : 0.0;

            default:
                return 0.0;
        }
    }

    double TimestepManager::getAverageFPS() const noexcept {
        switch (currentMode_) {
            case TimestepMode::VARIABLE:
                return variableTimestep_ ? variableTimestep_->getAverageFPS() : 0.0;

            case TimestepMode::HYBRID:
                return hybridTimestep_ ? hybridTimestep_->getVariableTimestep()->getAverageFPS() : 0.0;

            case TimestepMode::FIXED:
                return fixedTimestep_ ? fixedTimestep_->getUpdateRate() : 0.0;

            default:
                return 0.0;
        }
    }

    void TimestepManager::setTimeScale(const TimeScale scale) {
        timeScale_ = scale;

        if (variableTimestep_) {
            variableTimestep_->setTimeScale(scale);
        }
        if (fixedTimestep_) {
            fixedTimestep_->setTimeScale(scale);
        }
        if (hybridTimestep_) {
            hybridTimestep_->setTimeScale(scale);
        }
    }

    void TimestepManager::updateVariableConfig(const VariableTimestepConfig& config) {
        config_.variableConfig = config;
        if (variableTimestep_) {
            variableTimestep_->updateConfig(config);
        }
    }

    void TimestepManager::updateFixedConfig(const FixedTimestepConfig& config) {
        config_.fixedConfig = config;
        if (fixedTimestep_) {
            fixedTimestep_->updateConfig(config);
        }
    }

    void TimestepManager::updateHybridConfig(const HybridTimestepConfig& config) {
        config_.hybridConfig = config;
        if (hybridTimestep_) {
            hybridTimestep_->updateConfig(config);
        }
    }

    TimestepMetrics TimestepManager::getMetrics() {
        std::lock_guard lock(metricsMutex_);

        // Update current mode metrics
        metrics_.currentMode = currentMode_;

        if (variableTimestep_) {
            metrics_.variableStats = variableTimestep_->getStats();
        }
        if (fixedTimestep_) {
            metrics_.fixedStats = fixedTimestep_->getStats();
        }
        if (hybridTimestep_) {
            metrics_.hybridStats = hybridTimestep_->getStats();
        }

        return metrics_;
    }

    void TimestepManager::resetMetrics() {
        std::lock_guard lock(metricsMutex_);
        metrics_.reset();

        if (variableTimestep_) {
            variableTimestep_->resetStats();
        }
        if (fixedTimestep_) {
            fixedTimestep_->resetStats();
        }
        if (hybridTimestep_) {
            hybridTimestep_->resetStats();
        }
    }

    void TimestepManager::optimizeNow() {
        checkOptimization();
    }

    bool TimestepManager::createComponents() {
        if (!memoryManager_) {
            return false;
        }

        try {
            // Create variable timestep
            variableTimestep_ = std::make_unique<VariableTimestep>(config_.variableConfig);

            // Create fixed timestep
            fixedTimestep_ = std::make_unique<FixedTimestep>(config_.fixedConfig);

            // Create hybrid timestep
            hybridTimestep_ = std::make_unique<HybridTimestep>(config_.hybridConfig);

            // Apply current time scale
            if (const TimeScale scale = timeScale_.load(); scale != 1.0) {
                setTimeScale(scale);
            }

            return true;
        } catch (const std::exception& e) {
            std::cerr << "[TimestepManager] Failed to create components: "
                    << e.what() << std::endl;
            return false;
        }
    }

    void TimestepManager::destroyComponents() {
        variableTimestep_.reset();
        fixedTimestep_.reset();
        hybridTimestep_.reset();
    }

    TimestepUpdateResult TimestepManager::updateVariable(
            const Duration frameDeltaTime,
            const std::function<void(Duration, double)>& callback
            ) const {

        TimestepUpdateResult result;
        result.mode = TimestepMode::VARIABLE;

        if (!variableTimestep_) {
            result.deltaTime = frameDeltaTime;
            return result;
        }

        // Update variable timestep
        result.deltaTime = variableTimestep_->update(frameDeltaTime);
        result.updateCount = 1;
        result.interpolationAlpha = 0.0;

        // Execute callback
        if (callback) {
            callback(result.deltaTime, 0.0);
        }

        return result;
    }

    TimestepUpdateResult TimestepManager::updateFixed(
            const Duration frameDeltaTime,
            const std::function<void(Duration, std::uint32_t)>& callback
            ) const {

        TimestepUpdateResult result;
        result.mode = TimestepMode::FIXED;

        if (!fixedTimestep_) {
            result.deltaTime = frameDeltaTime;
            return result;
        }

        // Update fixed timestep
        result.updateCount = fixedTimestep_->update(frameDeltaTime, callback);
        result.fixedDeltaTime = fixedTimestep_->getFixedDeltaTime();
        result.deltaTime = result.fixedDeltaTime;
        result.interpolationAlpha = fixedTimestep_->getInterpolationAlpha();

        return result;
    }

    TimestepUpdateResult TimestepManager::updateHybrid(
            const Duration frameDeltaTime,
            const std::function<void(Duration, std::uint32_t)>& fixedCallback,
            const std::function<void(Duration, double)>& variableCallback
            ) const {

        TimestepUpdateResult result;
        result.mode = TimestepMode::HYBRID;

        if (!hybridTimestep_) {
            result.deltaTime = frameDeltaTime;
            return result;
        }

        // Update hybrid timestep
        const auto hybridResult = hybridTimestep_->update(
                frameDeltaTime,
                fixedCallback,
                variableCallback
                );

        result.updateCount = hybridResult.fixedUpdates;
        result.deltaTime = hybridResult.renderDeltaTime;
        result.fixedDeltaTime = hybridResult.physicsDeltaTime;
        result.interpolationAlpha = hybridResult.interpolationAlpha;
        result.modeChanged = hybridResult.modeChanged;

        return result;
    }

    void TimestepManager::performModeSwitch(const TimestepMode newMode) {
        if (newMode == currentMode_) {
            return;
        }

        previousMode_ = currentMode_;

        // Smooth transition if enabled
        if (config_.smoothTransition) {
            smoothTransition(currentMode_, newMode);
        }

        currentMode_ = newMode;
        lastModeSwitch_ = Clock::now();

        // Update metrics
        {
            std::lock_guard lock(metricsMutex_);
            metrics_.modeSwitches++;
            metrics_.currentMode = newMode;
        }

        // Notify callback
        if (config_.onModeChange) {
            config_.onModeChange(previousMode_, currentMode_);
        }

        std::cout << "[TimestepManager] Mode switched from "
                << getTimestepModeName(previousMode_)
                << " to " << getTimestepModeName(currentMode_) << std::endl;
    }

    void TimestepManager::checkOptimization() {
        const auto now = Clock::now();

        if (const auto timeSinceOpt = now - lastOptimization_; timeSinceOpt < config_.optimizationInterval) {
            return;
        }

        lastOptimization_ = now;

        // Determine optimal mode based on metrics

        if (const TimestepMode optimalMode = determineOptimalMode(); optimalMode != currentMode_ && canSwitchMode()) {
            std::cout << "[TimestepManager] Auto-optimizing to "
                    << getTimestepModeName(optimalMode) << std::endl;
            setMode(optimalMode, false);
        }
    }

    TimestepMode TimestepManager::determineOptimalMode() const {
        const double fps = getAverageFPS();
        const double stability = metrics_.stability;

        // High FPS and stable: Variable mode is fine
        if (fps >= 60.0 && stability > 0.8) {
            return TimestepMode::VARIABLE;
        }

        // Low FPS or unstable: Switch to hybrid for better consistency
        if (fps < 45.0 || stability < 0.5) {
            return TimestepMode::HYBRID;
        }

        // Need determinism (e.g., for physics): Fixed mode
        if (metrics_.efficiency < 0.7) {
            return TimestepMode::FIXED;
        }

        // Default to current mode
        return currentMode_;
    }

    void TimestepManager::updateMetrics(const TimestepUpdateResult& result, const Duration frameDelta) {
        std::lock_guard lock(metricsMutex_);

        metrics_.totalRuntime += frameDelta;
        metrics_.totalUpdates += result.updateCount;

        // Calculate average FPS
        if (const double instantFPS = getCurrentFPS(); instantFPS > 0) {
            constexpr double alpha = 0.1; // Smoothing factor
            metrics_.averageFPS = metrics_.averageFPS * (1.0 - alpha) + instantFPS * alpha;
        }

        // Update stability based on mode
        switch (currentMode_) {
            case TimestepMode::VARIABLE:
                if (variableTimestep_) {
                    metrics_.stability = variableTimestep_->getStability();
                }
                break;

            case TimestepMode::FIXED:
                metrics_.stability = 1.0; // Fixed is always stable
                break;

            case TimestepMode::HYBRID:
                if (hybridTimestep_) {
                    const auto stats = hybridTimestep_->getStats();
                    metrics_.stability = stats.interpolationQuality;
                }
                break;
            default:
                break;
        }

        // Calculate efficiency
        if (result.updateCount > 0) {
            const double expectedUpdates = toSeconds(frameDelta) * 60.0; // Assuming 60Hz target
            const auto actualUpdates = static_cast<double>(result.updateCount);
            metrics_.efficiency = std::min(actualUpdates / expectedUpdates, 1.0);
        }
    }

    void TimestepManager::smoothTransition(const TimestepMode from, const TimestepMode to) const {
        // Transfer state between modes for smooth transition

        if (from == TimestepMode::VARIABLE && to == TimestepMode::FIXED) {
            // Variable to Fixed: Start fixed timestep at current rate
            if (variableTimestep_ && fixedTimestep_) {
                if (const Duration currentDelta = variableTimestep_->getDeltaTime(); currentDelta > Duration::zero()) {
                    fixedTimestep_->setUpdateRate(1000000.0 / static_cast<double>(currentDelta.count()));
                }
            }
        } else if (from == TimestepMode::FIXED && to == TimestepMode::VARIABLE) {
            // Fixed to Variable: Initialize variable with fixed rate
            if (fixedTimestep_ && variableTimestep_) {
                const Duration fixedDelta = fixedTimestep_->getFixedDeltaTime();
                // TODO: Revisar por que no se está usando el valor de retorno de advance
                (void)variableTimestep_->advance(fixedDelta);
            }
        } else if (to == TimestepMode::HYBRID) {
            // To Hybrid: Configure based on previous mode
            if (hybridTimestep_) {
                if (from == TimestepMode::VARIABLE && variableTimestep_) {
                    hybridTimestep_->setTargetFPS(variableTimestep_->getFPS());
                } else if (from == TimestepMode::FIXED && fixedTimestep_) {
                    hybridTimestep_->setPhysicsRate(fixedTimestep_->getUpdateRate());
                }
            }
        }
    }

} // namespace engine::time
