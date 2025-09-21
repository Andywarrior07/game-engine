/**
 * @file TimeManagerConfig.h
 * @brief Top-level time management module configuration
 * @details Complete system configuration including memory budgets, timeline setups,
 *          and integration settings with other engine systems. This is the main
 *          configuration structure used to initialize the entire time management module.
 *
 * @author Andres Guerrero
 * @date Created on 2025-09-19
 */

#pragma once

#include "../core/TimeConfig.h"

#include <vector>
#include <unordered_map>
#include <memory>
#include <functional>

// Forward declarations for engine integration
namespace engine::memory {
    class MemoryManager;
}

namespace engine::events {
    class EventSystem;
}

namespace engine::time {
    // =============================================================================
    // Integration Points
    // =============================================================================

    /**
     * @brief External system integration configuration
     * @details Defines how the time system integrates with other engine modules.
     *          Provides hooks and callbacks for external system coordination.
     */
    struct IntegrationConfig {
        // Memory system integration
        engine::memory::MemoryManager* memoryManager{nullptr}; ///< Engine memory manager
        bool useEngineMemory{true}; ///< Use engine allocators

        // Event system integration (commented for future)
        // engine::events::EventSystem* eventSystem{nullptr};  ///< Engine event system
        // bool publishTimeEvents{true};                       ///< Publish time events

        // Thread system integration (commented for future)
        // engine::threading::ThreadManager* threadManager{nullptr}; ///< Thread manager
        // bool useEngineThreadPool{true};                          ///< Use engine threads

        // Render system callbacks
        std::function<void(Duration)> onPreRender; ///< Before rendering
        std::function<void(Duration)> onPostRender; ///< After rendering
        std::function<void(float)> getInterpolationFactor; ///< Get interp factor

        // Physics system callbacks
        std::function<void(Duration)> onPrePhysics; ///< Before physics
        std::function<void(Duration)> onPostPhysics; ///< After physics
        std::function<bool()> isPhysicsEnabled; ///< Check physics state

        // Game state callbacks
        std::function<void()> onGamePause; ///< Game paused
        std::function<void()> onGameResume; ///< Game resumed
        std::function<bool()> isGamePaused; ///< Check pause state

        // Network synchronization (future)
        std::function<TimeStamp()> getServerTime; ///< Get server timestamp
        std::function<void(TimeStamp)> syncToServerTime; ///< Sync with server
        std::function<Duration()> getNetworkLatency; ///< Get current latency

        // Performance adaptation
        std::function<void(Duration)> onFrameDropped; ///< Frame drop handler
        std::function<void(float)> onPerformanceWarning; ///< Performance warning
        std::function<bool()> shouldReduceQuality; ///< Quality reduction check
    };

    // =============================================================================
    // Timeline Creation Configuration
    // =============================================================================

    /**
     * @brief Configuration for creating a specific timeline
     * @details Extended configuration for individual timeline instances.
     *          Builds upon TimelineConfigBase with additional settings.
     */
    struct TimelineCreationConfig {
        TimelineConfigBase base;
        std::string name; ///< Timeline identifier
        bool persistent{false}; ///< Survives level changes
        bool networked{false}; ///< Network synchronized
        bool recordable{false}; ///< Support recording
        std::size_t eventBufferSize{1024}; ///< Event buffer size
        std::size_t historyBufferSize{60}; ///< History frames

        // Timer configuration for this timeline
        std::size_t maxTimersForTimeline{512}; ///< Max timers
        bool allowDynamicTimerCreation{true}; ///< Runtime timer creation

        // Callbacks specific to this timeline
        std::function<void(TimelineID)> onTimelineCreated; ///< Creation callback
        std::function<void(TimelineID)> onTimelineDestroyed; ///< Destruction callback
        std::function<void(TimelineID, TimeScale)> onScaleChanged; ///< Scale change
        std::function<void(TimelineID, bool)> onPauseChanged; ///< Pause state change
    };

    // =============================================================================
    // Timeline Registry Configuration
    // =============================================================================

    /**
     * @brief Configuration for all timelines in the system
     * @details Defines which timelines to create and their configurations.
     *          Supports both predefined and runtime timeline creation.
     */
    struct TimelineRegistryConfig {
        // Predefined timeline configurations
        std::vector<TimelineCreationConfig> predefinedTimelines;

        // Default configurations per timeline type
        std::unordered_map<TimelineType, TimelineCreationConfig> defaultConfigs;

        // Runtime timeline limits
        std::size_t maxRuntimeTimelines{4}; ///< Max custom timelines
        bool allowRuntimeTimelineCreation{true}; ///< Allow runtime creation

        /**
         * @brief Get standard timeline registry for typical game
         */
        [[nodiscard]] static TimelineRegistryConfig getStandardRegistry() {
            TimelineRegistryConfig config;

            // Real-time timeline (always running)
            config.predefinedTimelines.push_back({
                .base = {
                    .type = TimelineType::REAL_TIME,
                    .timestepConfig = {
                        .mode = TimestepMode::VARIABLE,
                        .flags = TimestepFlag::CLAMP_DELTA
                    },
                    .initialScale = 1.0,
                    .startPaused = false,
                    .autoStart = true,
                },
                .name = "RealTime",
                .persistent = true,
                .networked = true,
                .maxTimersForTimeline = 256,
                .onTimelineCreated = nullptr,
                .onTimelineDestroyed = nullptr,
                .onScaleChanged = nullptr,
                .onPauseChanged = nullptr
            });

            // Game timeline (main gameplay)
            config.predefinedTimelines.push_back({
                .base = {
                    .type = TimelineType::GAME_TIME,
                    .timestepConfig = getTimestepPreset(TimestepPreset::CASUAL_GAME),
                    .initialScale = 1.0,
                    .startPaused = false,
                    .autoStart = true,
                    .enableInterpolation = true
                },
                .name = "GameTime",
                .persistent = false,
                .recordable = true,
                .maxTimersForTimeline = 2048,
                .onTimelineCreated = nullptr,
                .onTimelineDestroyed = nullptr,
                .onScaleChanged = nullptr,
                .onPauseChanged = nullptr
            });

            // UI timeline (menus and HUD)
            config.predefinedTimelines.push_back({
                .base = {
                    .type = TimelineType::UI_TIME,
                    .timestepConfig = {
                        .mode = TimestepMode::VARIABLE,
                        .flags = TimestepFlag::SMOOTH_DELTA
                    },
                    .initialScale = 1.0,
                    .startPaused = false,
                    .autoStart = true
                },
                .name = "UITime",
                .persistent = true,
                .maxTimersForTimeline = 512,
                .onTimelineCreated = nullptr,
                .onTimelineDestroyed = nullptr,
                .onScaleChanged = nullptr,
                .onPauseChanged = nullptr
            });

            // Physics timeline (deterministic simulation)
            config.predefinedTimelines.push_back({
                .base = {
                    .type = TimelineType::PHYSICS_TIME,
                    .timestepConfig = getTimestepPreset(TimestepPreset::PHYSICS_SIMULATION),
                    .initialScale = 1.0,
                    .startPaused = false,
                    .autoStart = true,
                    .enableInterpolation = true,
                    .enableExtrapolation = true,
                    .maxExtrapolationFactor = 0.1f
                },
                .name = "PhysicsTime",
                .persistent = false,
                .networked = true,
                .maxTimersForTimeline = 128,
                .onTimelineCreated = nullptr,
                .onTimelineDestroyed = nullptr,
                .onScaleChanged = nullptr,
                .onPauseChanged = nullptr
            });

            // Audio timeline (synchronized playback)
            config.predefinedTimelines.push_back({
                .base = {
                    .type = TimelineType::AUDIO_TIME,
                    .timestepConfig = {
                        .mode = TimestepMode::VARIABLE,
                        .flags = TimestepFlag::CLAMP_DELTA
                    },
                    .initialScale = 1.0,
                    .startPaused = false,
                    .autoStart = true
                },
                .name = "AudioTime",
                .persistent = true,
                .maxTimersForTimeline = 256,
                .onTimelineCreated = nullptr,
                .onTimelineDestroyed = nullptr,
                .onScaleChanged = nullptr,
                .onPauseChanged = nullptr
            });

            return config;
        }
    };

    // =============================================================================
    // Performance Scaling Configuration
    // =============================================================================

    /**
     * @brief Dynamic performance scaling settings
     * @details Configures how the time system adapts to performance constraints.
     *          Enables graceful degradation under heavy load.
     */
    struct PerformanceScalingConfig {
        bool enableDynamicScaling{true}; ///< Auto-adjust quality
        Duration targetFrameTime{constants::TARGET_FRAME_TIME_60FPS}; ///< Target frame

        // Scaling thresholds
        float scaleDownThreshold{0.95f}; ///< CPU usage to scale down
        float scaleUpThreshold{0.70f}; ///< CPU usage to scale up
        Duration scaleDecisionInterval{Duration(1000000)}; ///< Decision frequency (1s)

        // Scaling strategies
        bool reduceTimerPrecision{true}; ///< Lower timer accuracy
        bool reduceUpdateFrequency{true}; ///< Skip updates
        bool disableInterpolation{true}; ///< Disable interpolation
        bool mergeTimerBatches{true}; ///< Batch more timers
        bool skipNonCriticalTimelines{true}; ///< Skip low priority

        // Quality levels
        enum class QualityLevel : std::uint8_t {
            ULTRA, ///< Maximum quality
            HIGH, ///< High quality
            MEDIUM, ///< Balanced
            LOW, ///< Performance mode
            MINIMUM ///< Survival mode
        };

        QualityLevel currentQuality{QualityLevel::HIGH};

        // Per-quality settings
        struct QualitySettings {
            std::uint16_t maxActiveTimers;
            std::uint8_t timelineUpdateRate;
            bool interpolationEnabled;
            bool profilingEnabled;
            Duration minTimerResolution;
        };

        std::unordered_map<QualityLevel, QualitySettings> qualityPresets{
            {QualityLevel::ULTRA, {8192, 100, true, true, Duration(100)}},
            {QualityLevel::HIGH, {4096, 100, true, true, Duration(1000)}},
            {QualityLevel::MEDIUM, {2048, 60, true, false, Duration(10000)}},
            {QualityLevel::LOW, {1024, 30, false, false, Duration(16667)}},
            {QualityLevel::MINIMUM, {256, 15, false, false, Duration(33333)}}
        };
    };

    // =============================================================================
    // Debug and Development Configuration
    // =============================================================================

    /**
     * @brief Debug features and development tools
     * @details Settings for debugging, testing, and development features.
     *          Most features are automatically disabled in release builds.
     */
    struct DebugConfig {
        bool enableTimeControl{true}; ///< Allow time manipulation
        bool enableFrameStepping{true}; ///< Single frame advance
        bool enableTimeRewind{false}; ///< Rewind support
        bool enableTimeRecording{true}; ///< Record for replay
        bool enableDetailedLogging{true}; ///< Verbose logging
        bool enableVisualDebugger{false}; ///< Visual timeline view

        // Time control settings
        TimeScale slowMotionScale{0.1f}; ///< Slow-mo factor
        TimeScale fastForwardScale{4.0f}; ///< Fast-forward factor
        Duration rewindBufferDuration{Duration(60000000)}; ///< 60s rewind buffer

        // Debug visualization
        bool showTimelineGraph{false}; ///< Timeline visualization
        bool showTimerQueue{false}; ///< Timer queue view
        bool showPerformanceOverlay{true}; ///< FPS/frame time
        bool showMemoryUsage{false}; ///< Memory statistics

        // Validation and checks
        bool validateTimerHandles{true}; ///< Check timer validity
        bool detectTimelineDeadlocks{true}; ///< Deadlock detection
        bool checkMemoryLeaks{true}; ///< Memory leak detection

#ifdef _DEBUG
        static constexpr bool IS_DEBUG_BUILD = true;
#else
        static constexpr bool IS_DEBUG_BUILD = false;
#endif

        /**
         * @brief Check if debug feature is available
         */
        [[nodiscard]] bool isFeatureEnabled(std::string_view feature) const {
            if constexpr (!IS_DEBUG_BUILD) {
                return false; // Most debug features disabled in release
            }
            // Feature-specific checks
            if (feature == "rewind") return enableTimeRewind;
            if (feature == "stepping") return enableFrameStepping;
            if (feature == "recording") return enableTimeRecording;
            return false;
        }
    };

    // =============================================================================
    // Complete Time Manager Configuration
    // =============================================================================

    /**
     * @brief Master configuration for the entire time management system
     * @details Aggregates all configuration subsystems into a single structure.
     *          This is the main configuration passed to TimeManager::initialize().
     */
    struct TimeManagerConfig {
        // Core configurations
        CoreTimeConfig core; ///< Core system config
        TimelineRegistryConfig timelines; ///< Timeline setup
        IntegrationConfig integration; ///< Engine integration
        PerformanceScalingConfig scaling; ///< Performance scaling
        DebugConfig debug; ///< Debug features

        // System behavior
        bool autoStart{true}; ///< Start automatically
        bool catchExceptions{true}; ///< Exception handling
        Duration initializationTimeout{Duration(5000000)}; ///< Init timeout (5s)

        /**
         * @brief Validate entire configuration
         * @throws std::invalid_argument if configuration is invalid
         */
        void validate() const {
            core.validate();

            // Validate timeline configurations
            for (const auto& timeline : timelines.predefinedTimelines) {
                timeline.validate();
            }

            // Check memory budgets
            const std::size_t totalTimelineMemory =
                timelines.predefinedTimelines.size() * (core.memory.timelineMemory / 8);
            if (totalTimelineMemory > core.memory.timelineMemory) {
                throw std::invalid_argument("Timeline memory requirements exceed budget");
            }

            // Validate integration points
            if (integration.useEngineMemory && !integration.memoryManager) {
                throw std::invalid_argument("Memory manager required when useEngineMemory is true");
            }
        }

        /**
         * @brief Get default configuration for development
         */
        [[nodiscard]] static TimeManagerConfig getDefaultConfig() {
            return {
                .core = CoreTimeConfig::getPlatformOptimized(),
                .timelines = TimelineRegistryConfig::getStandardRegistry(),
                .integration = IntegrationConfig{},
                .scaling = PerformanceScalingConfig{},
                .debug = DebugConfig{},
                .autoStart = true,
                .catchExceptions = true
            };
        }

        /**
         * @brief Get minimal configuration for testing
         */
        [[nodiscard]] static TimeManagerConfig getMinimalConfig() {
            TimeManagerConfig config;
            config.core = CoreTimeConfig::getMinimal();
            config.timelines.predefinedTimelines.clear();

            // Just one game timeline
            config.timelines.predefinedTimelines.push_back({
                .type = TimelineType::GAME_TIME,
                .name = "GameTime",
                .maxTimersForTimeline = 64
            });

            config.scaling.enableDynamicScaling = false;
            config.debug = DebugConfig{};
            config.debug.enableTimeControl = false;

            return config;
        }

        /**
         * @brief Get production configuration
         */
        // TODO: Revisar esto
        [[nodiscard]] static TimeManagerConfig getProductionConfig() {
            auto config = getDefaultConfig();
            config.core.profiling = ProfilingConfig::getReleaseConfig();
            config.debug = DebugConfig{}; // All debug features off
            config.debug.enablePerformanceOverlay = true; // Keep FPS counter
            config.catchExceptions = false; // Let crashes propagate
            return config;
        }
    };
} // namespace engine::time
