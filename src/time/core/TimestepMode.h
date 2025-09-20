/**
 * @file TimestepMode.h
 * @brief Timestep strategy definitions and configuration
 * @details Defines the different timestep modes (Variable, Fixed, Hybrid) and their
 *          configuration parameters. Includes performance vs accuracy trade-off
 *          definitions and mode-specific settings for optimal game loop behavior.
 *
 * @author Andres Guerrero
 * @date Created on 2025-09-19
 */

#pragma once

#include "TimeTypes.h"
#include <string_view>
#include <variant>
#include <optional>

namespace engine::time {
    // =============================================================================
    // Timestep Mode Enumeration
    // =============================================================================

    /**
     * @brief Primary timestep strategies for game loops
     * @details Each mode offers different trade-offs:
     *          - Variable: Smooth visuals, frame-dependent physics
     *          - Fixed: Deterministic simulation, potential visual stuttering
     *          - Hybrid: Best of both worlds with interpolation overhead
     */
    enum class TimestepMode : std::uint8_t {
        VARIABLE = 0, ///< Frame time determines update delta
        FIXED = 1, ///< Constant timestep with accumulation
        HYBRID = 2, ///< Fixed simulation with interpolated rendering

        COUNT = 3, ///< Number of timestep modes
        INVALID = 255 ///< Invalid mode sentinel
    };

    // =============================================================================
    // Timestep Configuration Flags
    // =============================================================================

    /**
     * @brief Configuration flags for timestep behavior
     * @details Fine-tune timestep behavior for specific requirements.
     *          These flags modify how the timestep strategy operates.
     */
    enum class TimestepFlag : std::uint32_t {
        NONE = 0, ///< Default behavior
        CLAMP_DELTA = 1 << 0, ///< Clamp delta to max value
        SMOOTH_DELTA = 1 << 1, ///< Apply smoothing to delta time
        SPIRAL_OF_DEATH_PROTECTION = 1 << 2, ///< Prevent update spiral
        INTERPOLATE_TRANSFORMS = 1 << 3, ///< Interpolate transform positions
        INTERPOLATE_ANIMATIONS = 1 << 4, ///< Interpolate animation states
        COMPENSATE_FRAME_DROPS = 1 << 5, ///< Adjust for dropped frames
        VSYNC_ALIGNED = 1 << 6, ///< Align to display refresh
        ADAPTIVE_TIMESTEP = 1 << 7, ///< Dynamically adjust timestep
        SUBSTEP_PHYSICS = 1 << 8, ///< Use physics substepping
        DETERMINISTIC = 1 << 9, ///< Ensure deterministic behavior
        NETWORK_PREDICTION = 1 << 10, ///< Enable client prediction
        ROLLBACK_SUPPORT = 1 << 11, ///< Support rollback networking
        REPLAY_COMPATIBLE = 1 << 12, ///< Ensure replay compatibility
        DEBUG_STEPPING = 1 << 13, ///< Allow debug step controls
        PROFILE_TIMESTEPS = 1 << 14, ///< Profile timestep performance
    };

    /**
     * @brief Bitwise OR operator for combining flags
     */
    [[nodiscard]] constexpr TimestepFlag operator|(TimestepFlag a, TimestepFlag b) noexcept {
        return static_cast<TimestepFlag>(
            static_cast<std::uint32_t>(a) | static_cast<std::uint32_t>(b)
        );
    }

    /**
     * @brief Bitwise AND operator for checking flags
     */
    [[nodiscard]] constexpr TimestepFlag operator&(TimestepFlag a, TimestepFlag b) noexcept {
        return static_cast<TimestepFlag>(
            static_cast<std::uint32_t>(a) & static_cast<std::uint32_t>(b)
        );
    }

    /**
     * @brief Check if timestep configuration has specific flag
     */
    [[nodiscard]] constexpr bool hasTimestepFlag(const TimestepFlag flags, const TimestepFlag flag) noexcept {
        return (flags & flag) == flag;
    }

    // =============================================================================
    // Interpolation Strategy
    // =============================================================================

    /**
     * @brief Interpolation methods for hybrid timestep mode
     * @details Determines how positions are interpolated between fixed updates.
     *          Critical for smooth visuals with fixed timestep physics.
     */
    enum class InterpolationMethod : std::uint8_t {
        NONE = 0, ///< No interpolation (snapping)
        LINEAR = 1, ///< Linear interpolation
        CUBIC = 2, ///< Cubic spline interpolation
        HERMITE = 3, ///< Hermite spline (smoother)
        PREDICTIVE = 4, ///< Velocity-based prediction
        ADAPTIVE = 5 ///< Context-aware method selection
    };

    // =============================================================================
    // Delta Time Smoothing
    // =============================================================================

    /**
     * @brief Delta time smoothing algorithms
     * @details Reduces frame time variance for more stable updates.
     *          Helps prevent jerky movement from frame spikes.
     */
    enum class SmoothingMethod : std::uint8_t {
        NONE = 0, ///< No smoothing (raw delta)
        MOVING_AVERAGE = 1, ///< Simple moving average
        EXPONENTIAL = 2, ///< Exponential moving average
        MEDIAN_FILTER = 3, ///< Median of recent samples
        KALMAN_FILTER = 4, ///< Kalman filter prediction
        ADAPTIVE_WINDOW = 5 ///< Variable window based on variance
    };

    // =============================================================================
    // Performance vs Accuracy Trade-offs
    // =============================================================================

    /**
     * @brief Preset configurations for common use cases
     * @details Predefined settings optimized for specific scenarios.
     *          Provides good starting points for different game types.
     */
    enum class TimestepPreset : std::uint8_t {
        COMPETITIVE_FPS, ///< Ultra-low latency, high precision
        CASUAL_GAME, ///< Balanced for smooth gameplay
        PHYSICS_SIMULATION, ///< Accuracy over performance
        MOBILE_OPTIMIZED, ///< Battery-efficient settings
        VR_EXPERIENCE, ///< Low latency, high framerate
        CINEMATIC, ///< Smooth visuals, film-like
        NETWORK_GAME, ///< Optimized for online play
        DEBUG_MODE, ///< Full debugging features
        CUSTOM ///< User-defined configuration
    };

    // =============================================================================
    // Variable Timestep Configuration
    // =============================================================================

    /**
     * @brief Configuration for variable timestep mode
     * @details Settings specific to variable timestep behavior.
     *          Provides smooth visuals at the cost of non-determinism.
     */
    struct VariableTimestepConfig {
        Duration maxDeltaTime{constants::MAX_DELTA_TIME}; ///< Maximum allowed delta
        Duration minDeltaTime{constants::MIN_DELTA_TIME}; ///< Minimum allowed delta
        SmoothingMethod smoothing{SmoothingMethod::NONE}; ///< Delta smoothing method
        std::uint8_t smoothingWindowSize{5}; ///< Samples for smoothing
        float smoothingFactor{0.1f}; ///< EMA smoothing factor
        bool compensateFrameDrops{true}; ///< Handle frame drops
        bool useHighPrecisionTimer{true}; ///< Use QPC on Windows
    };

    // =============================================================================
    // Fixed Timestep Configuration
    // =============================================================================

    /**
     * @brief Configuration for fixed timestep mode
     * @details Settings for deterministic fixed timestep updates.
     *          Critical for reproducible physics and networking.
     */
    struct FixedTimestepConfig {
        Duration timestep{constants::DEFAULT_FIXED_TIMESTEP}; ///< Fixed step duration
        std::uint8_t maxIterationsPerFrame{constants::MAX_FIXED_ITERATIONS}; ///< Spiral prevention
        Duration maxAccumulator{Duration(200000)}; ///< Max accumulated time (200ms)
        bool catchUpAfterSpike{true}; ///< Catch up after lag spike
        bool allowPartialUpdates{false}; ///< Process partial timesteps
        std::uint8_t substeps{1}; ///< Physics substeps per update
        float updateSpeedMultiplier{1.0f}; ///< Speed up/slow down updates
    };

    // =============================================================================
    // Hybrid Timestep Configuration
    // =============================================================================

    /**
     * @brief Configuration for hybrid timestep mode
     * @details Combines fixed simulation with interpolated rendering.
     *          Provides both determinism and smooth visuals.
     */
    struct HybridTimestepConfig {
        FixedTimestepConfig fixedConfig; ///< Fixed timestep settings
        VariableTimestepConfig renderConfig; ///< Render timestep settings
        InterpolationMethod interpolation{InterpolationMethod::LINEAR}; ///< Interpolation method
        float extrapolationLimit{0.25f}; ///< Max extrapolation factor
        bool interpolateTransforms{true}; ///< Interpolate positions
        bool interpolateAnimations{true}; ///< Interpolate animations
        bool interpolateParticles{false}; ///< Interpolate particles
        std::uint8_t interpolationBufferSize{2}; ///< States to keep for interp
    };

    // =============================================================================
    // Unified Timestep Configuration
    // =============================================================================

    /**
     * @brief Complete timestep configuration
     * @details Variant holding mode-specific configuration.
     *          Type-safe configuration for any timestep mode.
     */
    struct TimestepConfig {
        TimestepMode mode{TimestepMode::VARIABLE}; ///< Active timestep mode
        TimestepFlag flags{TimestepFlag::CLAMP_DELTA}; ///< Configuration flags

        /**
         * @brief Mode-specific configuration variant
         */
        std::variant<
            VariableTimestepConfig,
            FixedTimestepConfig,
            HybridTimestepConfig
        > modeConfig{VariableTimestepConfig{}};

        /**
         * @brief Get variable timestep config if applicable
         */
        [[nodiscard]] std::optional<VariableTimestepConfig> getVariableConfig() const {
            if (mode == TimestepMode::VARIABLE) {
                return std::get<VariableTimestepConfig>(modeConfig);
            }
            return std::nullopt;
        }

        /**
         * @brief Get fixed timestep config if applicable
         */
        [[nodiscard]] std::optional<FixedTimestepConfig> getFixedConfig() const {
            if (mode == TimestepMode::FIXED) {
                return std::get<FixedTimestepConfig>(modeConfig);
            }
            return std::nullopt;
        }

        /**
         * @brief Get hybrid timestep config if applicable
         */
        [[nodiscard]] std::optional<HybridTimestepConfig> getHybridConfig() const {
            if (mode == TimestepMode::HYBRID) {
                return std::get<HybridTimestepConfig>(modeConfig);
            }
            return std::nullopt;
        }
    };

    // =============================================================================
    // Timestep Preset Configurations
    // =============================================================================

    /**
     * @brief Get preset configuration for specific use case
     * @param preset Desired preset type
     * @return Complete timestep configuration
     */
    [[nodiscard]] inline TimestepConfig getTimestepPreset(const TimestepPreset preset) {
        switch (preset) {
        case TimestepPreset::COMPETITIVE_FPS: {
            // Ultra-responsive for competitive gaming
            return {
                .mode = TimestepMode::VARIABLE,
                .flags = TimestepFlag::CLAMP_DELTA |
                TimestepFlag::COMPENSATE_FRAME_DROPS |
                TimestepFlag::VSYNC_ALIGNED,
                .modeConfig = VariableTimestepConfig{
                    .maxDeltaTime = Duration(8333), // Cap at 120fps equivalent
                    .minDeltaTime = Duration(100),
                    .smoothing = SmoothingMethod::NONE, // Raw input response
                    .smoothingWindowSize = 0,
                    .smoothingFactor = 0.0f,
                    .compensateFrameDrops = false, // Prefer consistency
                    .useHighPrecisionTimer = true
                }
            };
        }

        case TimestepPreset::CASUAL_GAME: {
            // Smooth and forgiving for casual play
            return {
                .mode = TimestepMode::VARIABLE,
                .flags = TimestepFlag::CLAMP_DELTA |
                TimestepFlag::SMOOTH_DELTA |
                TimestepFlag::COMPENSATE_FRAME_DROPS,
                .modeConfig = VariableTimestepConfig{
                    .maxDeltaTime = constants::MAX_DELTA_TIME,
                    .minDeltaTime = constants::MIN_DELTA_TIME,
                    .smoothing = SmoothingMethod::EXPONENTIAL,
                    .smoothingWindowSize = 8,
                    .smoothingFactor = 0.2f,
                    .compensateFrameDrops = true,
                    .useHighPrecisionTimer = true
                }
            };
        }

        case TimestepPreset::PHYSICS_SIMULATION: {
            // Deterministic physics-heavy games
            return {
                .mode = TimestepMode::HYBRID,
                .flags = TimestepFlag::SPIRAL_OF_DEATH_PROTECTION |
                TimestepFlag::INTERPOLATE_TRANSFORMS |
                TimestepFlag::DETERMINISTIC |
                TimestepFlag::SUBSTEP_PHYSICS,
                .modeConfig = HybridTimestepConfig{
                    .fixedConfig = {
                        .timestep = Duration(16667), // 60Hz physics
                        .maxIterationsPerFrame = 3,
                        .maxAccumulator = Duration(50000),
                        .catchUpAfterSpike = false,
                        .allowPartialUpdates = false,
                        .substeps = 2,
                        .updateSpeedMultiplier = 1.0f
                    },
                    .renderConfig = {
                        .maxDeltaTime = constants::MAX_DELTA_TIME,
                        .minDeltaTime = constants::MIN_DELTA_TIME,
                        .smoothing = SmoothingMethod::NONE
                    },
                    .interpolation = InterpolationMethod::HERMITE,
                    .extrapolationLimit = 0.1f,
                    .interpolateTransforms = true,
                    .interpolateAnimations = true,
                    .interpolateParticles = true,
                    .interpolationBufferSize = 3
                }
            };
        }

        case TimestepPreset::MOBILE_OPTIMIZED: {
            // Battery-efficient mobile settings
            return {
                .mode = TimestepMode::VARIABLE,
                .flags = TimestepFlag::CLAMP_DELTA |
                TimestepFlag::ADAPTIVE_TIMESTEP |
                TimestepFlag::COMPENSATE_FRAME_DROPS,
                .modeConfig = VariableTimestepConfig{
                    .maxDeltaTime = Duration(33333), // 30fps cap for battery
                    .minDeltaTime = Duration(16667), // 60fps max
                    .smoothing = SmoothingMethod::MOVING_AVERAGE,
                    .smoothingWindowSize = 4,
                    .smoothingFactor = 0.25f,
                    .compensateFrameDrops = true,
                    .useHighPrecisionTimer = false // Save battery
                }
            };
        }

        case TimestepPreset::VR_EXPERIENCE: {
            // Low latency, high framerate for VR
            return {
                .mode = TimestepMode::FIXED,
                .flags = TimestepFlag::SPIRAL_OF_DEATH_PROTECTION |
                TimestepFlag::VSYNC_ALIGNED,
                .modeConfig = FixedTimestepConfig{
                    .timestep = Duration(11111), // 90Hz for VR
                    .maxIterationsPerFrame = 2,
                    .maxAccumulator = Duration(22222),
                    .catchUpAfterSpike = false,
                    .allowPartialUpdates = false,
                    .substeps = 1,
                    .updateSpeedMultiplier = 1.0f
                }
            };
        }

        case TimestepPreset::NETWORK_GAME: {
            // Optimized for online multiplayer
            return {
                .mode = TimestepMode::FIXED,
                .flags = TimestepFlag::DETERMINISTIC |
                TimestepFlag::NETWORK_PREDICTION |
                TimestepFlag::ROLLBACK_SUPPORT |
                TimestepFlag::REPLAY_COMPATIBLE,
                .modeConfig = FixedTimestepConfig{
                    .timestep = Duration(16667), // 60Hz tick rate
                    .maxIterationsPerFrame = 4,
                    .maxAccumulator = Duration(66668),
                    .catchUpAfterSpike = true,
                    .allowPartialUpdates = false,
                    .substeps = 1,
                    .updateSpeedMultiplier = 1.0f
                }
            };
        }

        default:
        case TimestepPreset::CUSTOM: {
            // Default variable timestep
            return {
                .mode = TimestepMode::VARIABLE,
                .flags = TimestepFlag::CLAMP_DELTA,
                .modeConfig = VariableTimestepConfig{}
            };
        }
        }
    }

    // =============================================================================
    // Utility Functions
    // =============================================================================

    /**
     * @brief Get human-readable name for timestep mode
     */
    [[nodiscard]] constexpr std::string_view getTimestepModeName(const TimestepMode mode) noexcept {
        switch (mode) {
        case TimestepMode::VARIABLE: return "Variable";
        case TimestepMode::FIXED: return "Fixed";
        case TimestepMode::HYBRID: return "Hybrid";
        default: return "Invalid";
        }
    }

    /**
     * @brief Check if mode requires interpolation
     */
    [[nodiscard]] constexpr bool requiresInterpolation(const TimestepMode mode) noexcept {
        return mode == TimestepMode::HYBRID;
    }

    /**
     * @brief Check if mode is deterministic
     */
    [[nodiscard]] constexpr bool isDeterministic(const TimestepMode mode) noexcept {
        return mode == TimestepMode::FIXED || mode == TimestepMode::HYBRID;
    }

    /**
     * @brief Calculate optimal timestep for target framerate
     */
    [[nodiscard]] constexpr Duration calculateTimestepForFramerate(const std::uint16_t fps) noexcept {
        if (fps == 0) return Duration::zero();
        return Duration(1000000 / fps); // Microseconds per frame
    }
} // namespace engine::time
