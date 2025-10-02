/**
 * @file TimestepManager.h
 * @brief Central management system for all timestep strategies
 * @details Coordinates variable, fixed, and hybrid timestep modes with seamless
 *          switching, performance monitoring, and memory management integration.
 *          Provides a unified interface for game loop timing control.
 *
 * @author Development Team
 * @date Created on 2024-01-20
 */

#pragma once

#include "VariableTimestep.h"
#include "FixedTimestep.h"
#include "HybridTimestep.h"

#include "../core/TimeTypes.h"
#include "../core/TimestepMode.h"

#include "../../memory/MemorySystem.h"

#include <memory>
#include <functional>
#include <atomic>
#include <mutex>

namespace engine::time {

    // =============================================================================
    // Timestep Manager Configuration
    // =============================================================================

    /**
     * @brief Configuration for timestep manager
     */
    struct TimestepManagerConfig {
        // Initial mode
        TimestepMode initialMode{TimestepMode::VARIABLE};
        TimestepPreset preset{TimestepPreset::CASUAL_GAME};

        // Mode configurations
        VariableTimestepConfig variableConfig{};
        FixedTimestepConfig fixedConfig{};
        HybridTimestepConfig hybridConfig{};

        // Switching behavior
        bool allowRuntimeSwitch{true}; ///< Allow mode switching at runtime
        Duration switchCooldown{Duration(1000000)}; ///< Minimum time between switches
        bool smoothTransition{true}; ///< Smooth transition between modes

        // Performance monitoring
        bool trackPerformance{true}; ///< Track timestep performance
        bool autoOptimize{false}; ///< Auto-adjust based on performance
        Duration optimizationInterval{Duration(5000000)}; ///< Check every 5 seconds

        // Memory management
        engine::memory::MemoryManager* memoryManager{nullptr}; ///< Memory manager
        engine::memory::MemoryCategory memoryCategory{ ///< Memory category
                        engine::memory::MemoryCategory::GENERAL
                };

        // Callbacks
        std::function<void(TimestepMode, TimestepMode)> onModeChange;
        std::function<void(const std::string&)> onPerformanceWarning;
    };

    // =============================================================================
    // Timestep Update Result
    // =============================================================================

    /**
     * @brief Result from timestep manager update
     */
    struct TimestepUpdateResult {
        TimestepMode mode{TimestepMode::VARIABLE}; ///< Current mode
        std::uint32_t updateCount{0}; ///< Number of updates
        Duration deltaTime{}; ///< Frame delta time
        Duration fixedDeltaTime{}; ///< Fixed delta (if applicable)
        double interpolationAlpha{0.0}; ///< Interpolation factor
        bool modeChanged{false}; ///< Mode changed this frame

        /**
         * @brief Check if should perform fixed update
         */
        [[nodiscard]] bool shouldUpdate() const noexcept {
            return updateCount > 0;
        }

        /**
         * @brief Check if should interpolate
         */
        [[nodiscard]] bool shouldInterpolate() const noexcept {
            return mode == TimestepMode::HYBRID && interpolationAlpha > 0.0;
        }
    };

    // =============================================================================
    // Performance Metrics
    // =============================================================================

    /**
     * @brief Timestep performance metrics
     */
    struct TimestepMetrics {
        // Mode-specific stats
        VariableTimestepStats variableStats{};
        FixedTimestepStats fixedStats{};
        HybridTimestepStats hybridStats{};

        // Overall metrics
        TimestepMode currentMode{TimestepMode::VARIABLE};
        Duration totalRuntime{};
        std::uint64_t modeSwitches{0};
        std::uint64_t totalUpdates{0};

        // Performance indicators
        double averageFPS{0.0};
        double stability{1.0};
        double efficiency{1.0};

        /**
         * @brief Reset all metrics
         */
        void reset() noexcept {
            variableStats = {};
            fixedStats = {};
            hybridStats = {};
            totalRuntime = Duration::zero();
            modeSwitches = 0;
            totalUpdates = 0;
            averageFPS = 0.0;
            stability = 1.0;
            efficiency = 1.0;
        }
    };

    // =============================================================================
    // TimestepManager Class
    // =============================================================================

    /**
     * @brief Central manager for all timestep strategies
     * @details Provides unified control over game loop timing with support for
     *          runtime mode switching, performance optimization, and monitoring.
     */
    class TimestepManager {
    public:
        /**
         * @brief Constructor with configuration
         * @param config Manager configuration
         */
        explicit TimestepManager(const TimestepManagerConfig& config = {});

        /**
         * @brief Destructor
         */
        ~TimestepManager();

        // Delete copy operations
        TimestepManager(const TimestepManager&) = delete;
        TimestepManager& operator=(const TimestepManager&) = delete;

        // Allow move operations
        TimestepManager(TimestepManager&&) noexcept = default;
        TimestepManager& operator=(TimestepManager&&) noexcept = default;

        // =============================================================================
        // Initialization
        // =============================================================================

        /**
         * @brief Initialize manager with memory system
         * @param memoryManager Memory manager instance
         * @return True if initialization successful
         */
        bool initialize(engine::memory::MemoryManager* memoryManager);

        /**
         * @brief Shutdown and cleanup
         */
        void shutdown();

        /**
         * @brief Check if initialized
         */
        [[nodiscard]] bool isInitialized() const noexcept {
            return initialized_;
        }

        // =============================================================================
        // Core Update
        // =============================================================================

        /**
         * @brief Update timestep system
         * @param frameDeltaTime Raw frame delta time
         * @param fixedUpdate Callback for fixed updates
         * @param variableUpdate Callback for variable updates
         * @return Update result with timing information
         */
        [[nodiscard]] TimestepUpdateResult update(
                Duration frameDeltaTime,
                const std::function<void(Duration, std::uint32_t)>& fixedUpdate = nullptr,
                const std::function<void(Duration, double)>& variableUpdate = nullptr
                );

        /**
         * @brief Simple update without callbacks
         * @param frameDeltaTime Raw frame delta
         * @return Processed delta time
         */
        [[nodiscard]] Duration updateSimple(Duration frameDeltaTime);

        /**
         * @brief Reset all timestep states
         */
        void reset();

        // =============================================================================
        // Mode Management
        // =============================================================================

        /**
         * @brief Set timestep mode
         * @param mode New timestep mode
         * @param immediate Switch immediately or wait for cooldown
         * @return True if mode changed
         */
        bool setMode(TimestepMode mode, bool immediate = false);

        /**
         * @brief Get current timestep mode
         */
        [[nodiscard]] TimestepMode getMode() const noexcept {
            return currentMode_;
        }

        /**
         * @brief Apply timestep preset
         * @param preset Preset configuration
         */
        void applyPreset(TimestepPreset preset);

        /**
         * @brief Check if can switch modes
         */
        [[nodiscard]] bool canSwitchMode() const noexcept;

        // =============================================================================
        // Time Queries
        // =============================================================================

        /**
         * @brief Get current delta time
         * @return Frame delta time based on current mode
         */
        [[nodiscard]] Duration getDeltaTime() const noexcept;

        /**
         * @brief Get fixed delta time
         * @return Fixed timestep duration
         */
        [[nodiscard]] Duration getFixedDeltaTime() const noexcept;

        /**
         * @brief Get interpolation alpha
         * @return Interpolation factor [0,1]
         */
        [[nodiscard]] double getInterpolationAlpha() const noexcept;

        /**
         * @brief Get current FPS
         */
        [[nodiscard]] double getCurrentFPS() const noexcept;

        /**
         * @brief Get average FPS
         */
        [[nodiscard]] double getAverageFPS() const noexcept;

        // =============================================================================
        // Configuration
        // =============================================================================

        /**
         * @brief Set time scale
         * @param scale Time scale factor
         */
        void setTimeScale(TimeScale scale);

        /**
         * @brief Get time scale
         */
        [[nodiscard]] TimeScale getTimeScale() const noexcept {
            return timeScale_;
        }

        /**
         * @brief Update variable timestep config
         */
        void updateVariableConfig(const VariableTimestepConfig& config);

        /**
         * @brief Update fixed timestep config
         */
        void updateFixedConfig(const FixedTimestepConfig& config);

        /**
         * @brief Update hybrid timestep config
         */
        void updateHybridConfig(const HybridTimestepConfig& config);

        /**
         * @brief Get configuration
         */
        [[nodiscard]] const TimestepManagerConfig& getConfig() const noexcept {
            return config_;
        }

        // =============================================================================
        // Performance Monitoring
        // =============================================================================

        /**
         * @brief Get performance metrics
         */
        [[nodiscard]] TimestepMetrics getMetrics();

        /**
         * @brief Reset performance metrics
         */
        void resetMetrics();

        /**
         * @brief Enable/disable auto-optimization
         */
        void setAutoOptimization(const bool enable) {
            config_.autoOptimize = enable;
        }

        /**
         * @brief Force optimization check
         */
        void optimizeNow();

        // =============================================================================
        // Component Access
        // =============================================================================

        /**
         * @brief Get variable timestep component
         * @return Pointer to variable timestep (may be null)
         */
        [[nodiscard]] VariableTimestep* getVariableTimestep() const noexcept {
            return variableTimestep_.get();
        }

        /**
         * @brief Get fixed timestep component
         * @return Pointer to fixed timestep (may be null)
         */
        [[nodiscard]] FixedTimestep* getFixedTimestep() const noexcept {
            return fixedTimestep_.get();
        }

        /**
         * @brief Get hybrid timestep component
         * @return Pointer to hybrid timestep (may be null)
         */
        [[nodiscard]] HybridTimestep* getHybridTimestep() const noexcept {
            return hybridTimestep_.get();
        }

    private:
        // Configuration
        TimestepManagerConfig config_;

        // State
        bool initialized_{false};
        TimestepMode currentMode_{TimestepMode::VARIABLE};
        TimestepMode previousMode_{TimestepMode::VARIABLE};
        std::atomic<TimeScale> timeScale_{1.0};

        // Components
        std::unique_ptr<VariableTimestep> variableTimestep_;
        std::unique_ptr<FixedTimestep> fixedTimestep_;
        std::unique_ptr<HybridTimestep> hybridTimestep_;

        // Mode switching
        TimeStamp lastModeSwitch_;
        std::atomic<bool> modeSwitchPending_{false};
        TimestepMode pendingMode_{TimestepMode::VARIABLE};

        // Performance tracking
        mutable std::mutex metricsMutex_;
        TimestepMetrics metrics_;
        TimeStamp lastOptimization_;
        Duration totalRuntime_;

        // Memory management
        engine::memory::MemoryManager* memoryManager_{nullptr};

        // =============================================================================
        // Internal Methods
        // =============================================================================

        /**
         * @brief Create timestep components
         */
        bool createComponents();

        /**
         * @brief Destroy timestep components
         */
        void destroyComponents();

        /**
         * @brief Update with variable mode
         */
        [[nodiscard]] TimestepUpdateResult updateVariable(
                Duration frameDeltaTime,
                const std::function<void(Duration, double)>& callback
                ) const;

        /**
         * @brief Update with fixed mode
         */
        [[nodiscard]] TimestepUpdateResult updateFixed(
                Duration frameDeltaTime,
                const std::function<void(Duration, std::uint32_t)>& callback
                ) const;

        /**
         * @brief Update with hybrid mode
         */
        [[nodiscard]] TimestepUpdateResult updateHybrid(
                Duration frameDeltaTime,
                const std::function<void(Duration, std::uint32_t)>& fixedCallback,
                const std::function<void(Duration, double)>& variableCallback
                ) const;

        /**
         * @brief Perform mode switch
         */
        void performModeSwitch(TimestepMode newMode);

        /**
         * @brief Check and perform optimization
         */
        void checkOptimization();

        /**
         * @brief Determine optimal mode based on performance
         */
        [[nodiscard]] TimestepMode determineOptimalMode() const;

        /**
         * @brief Update performance metrics
         */
        void updateMetrics(const TimestepUpdateResult& result, Duration frameDelta);

        /**
         * @brief Smooth transition between modes
         */
        void smoothTransition(TimestepMode from, TimestepMode to) const;
    };

} // namespace engine::time
