/**
 * @file SensitivityStage.h
 * @brief Sensitivity and response curve processing stage
 * @author Andrés Guerrero
 * @date 12-09-2025
 *
 * Applies sensitivity scaling and response curves to input values.
 */

#pragma once

#include "../../core/InputEvent.h"

#include <unordered_map>
#include <functional>

namespace engine::input::processing {
    /**
     * @brief Sensitivity configuration
     */
    struct SensitivityConfig {
        // Global sensitivity
        float globalSensitivity = 1.0f;

        // Device-specific sensitivity
        float mouseSensitivity = 1.0f;
        float stickSensitivity = 1.0f;
        float triggerSensitivity = 1.0f;
        float wheelSensitivity = 1.0f;

        // Axis-specific sensitivity
        struct AxisSensitivity {
            float x = 1.0f;
            float y = 1.0f;
            bool invertX = false;
            bool invertY = false;
        };

        AxisSensitivity mouseAxis;
        AxisSensitivity leftStickAxis;
        AxisSensitivity rightStickAxis;

        // Response curves
        ResponseCurve mouseCurve = ResponseCurve::LINEAR;
        ResponseCurve stickCurve = ResponseCurve::LINEAR;
        ResponseCurve triggerCurve = ResponseCurve::LINEAR;

        float mouseCurveStrength = 0.0f; // 0 = linear, 1 = full curve
        float stickCurveStrength = 0.0f;
        float triggerCurveStrength = 0.0f;

        // Acceleration
        bool enableMouseAcceleration = false;
        float mouseAccelerationFactor = 1.5f;
        float mouseAccelerationThreshold = 100.0f; // Pixels/sec

        // Custom curves
        using CustomCurveFunc = std::function<float(float)>;
        std::unordered_map<std::string, CustomCurveFunc> customCurves;

        // Per-device overrides
        std::unordered_map<DeviceID, float> deviceSensitivity;
    };

    /**
     * @brief Sensitivity and response curve processing stage
     *
     * Modifies input values based on sensitivity settings and response curves
     * to provide better control feel.
     */
    class SensitivityStage {
    public:
        SensitivityStage() noexcept;
        explicit SensitivityStage(const SensitivityConfig& config) noexcept;
        ~SensitivityStage() = default;

        // Disable copy, enable move
        SensitivityStage(const SensitivityStage&) = delete;
        SensitivityStage& operator=(const SensitivityStage&) = delete;
        SensitivityStage(SensitivityStage&&) noexcept = default;
        SensitivityStage& operator=(SensitivityStage&&) noexcept = default;

        /**
         * @brief Process a batch of events
         * @param events Events to process (modified in place)
         * @param deltaTime Time since last update
         * @return Number of events processed
         */
        std::size_t process(std::vector<InputEvent>& events, float deltaTime);

        /**
         * @brief Process single event
         * @param event Event to process (modified in place)
         * @return True if event was processed
         */
        bool processEvent(InputEvent& event);

        /**
         * @brief Set sensitivity configuration
         */
        void setConfig(const SensitivityConfig& config) noexcept;

        /**
         * @brief Get current configuration
         */
        [[nodiscard]] const SensitivityConfig& getConfig() const noexcept {
            return config_;
        }

        /**
         * @brief Set global sensitivity
         */
        void setGlobalSensitivity(float sensitivity) noexcept;

        /**
         * @brief Set device-specific sensitivity
         */
        void setDeviceSensitivity(DeviceID deviceId, float sensitivity);

        /**
         * @brief Set mouse sensitivity
         */
        void setMouseSensitivity(float sensitivity, bool applyToX = true, bool applyToY = true) noexcept;

        /**
         * @brief Set stick sensitivity
         */
        void setStickSensitivity(float sensitivity, bool leftStick = true) noexcept;

        /**
         * @brief Set response curve
         */
        void setResponseCurve(ResponseCurve curve, float strength, DeviceType deviceType);

        /**
         * @brief Register custom curve function
         */
        void registerCustomCurve(const std::string& name, SensitivityConfig::CustomCurveFunc func);

        /**
         * @brief Reset to default settings
         */
        void reset() noexcept;

        /**
         * @brief Get statistics
         */
        struct Statistics {
            std::uint64_t totalProcessed = 0;
            std::uint64_t mouseEventsProcessed = 0;
            std::uint64_t stickEventsProcessed = 0;
            std::uint64_t triggerEventsProcessed = 0;
            std::uint64_t curvesApplied = 0;
            std::uint64_t accelerationApplied = 0;

            void reset() noexcept {
                totalProcessed = 0;
                mouseEventsProcessed = 0;
                stickEventsProcessed = 0;
                triggerEventsProcessed = 0;
                curvesApplied = 0;
                accelerationApplied = 0;
            }
        };

        [[nodiscard]] const Statistics& getStatistics() const noexcept {
            return stats_;
        }

        /**
         * @brief Enable/disable sensitivity processing
         */
        void setEnabled(const bool enabled) noexcept {
            enabled_ = enabled;
        }

        [[nodiscard]] bool isEnabled() const noexcept {
            return enabled_;
        }

    private:
        // Configuration
        SensitivityConfig config_;
        bool enabled_ = true;

        // Statistics
        Statistics stats_;

        // Velocity tracking for acceleration
        struct VelocityTracker {
            math::Vec2 lastPosition;
            std::chrono::steady_clock::time_point lastTime;
            float velocity = 0.0f;
            float smoothedVelocity = 0.0f;
        };

        std::unordered_map<DeviceID, VelocityTracker> velocityTrackers_;

        // Curve cache for performance
        struct CurveCache {
            ResponseCurve type;
            float strength;
            std::vector<float> lookupTable;
            static constexpr std::size_t TABLE_SIZE = 256;

            CurveCache() : type(ResponseCurve::LINEAR), strength(0.0f) {
                lookupTable.resize(TABLE_SIZE);
                regenerate();
            }

            void regenerate();
            [[nodiscard]] float apply(float value) const noexcept;
        };

        CurveCache mouseCurveCache_;
        CurveCache stickCurveCache_;
        CurveCache triggerCurveCache_;

        // ============================================================================
        // Processing Methods
        // ============================================================================

        /**
         * @brief Process mouse motion event
         */
        bool processMouseMotion(MouseMotionEventData& data, DeviceID deviceId);

        /**
         * @brief Process gamepad axis event
         */
        bool processGamepadAxis(GamepadAxisEventData& data, DeviceID deviceId);

        /**
         * @brief Process mouse wheel event
         */
        bool processMouseWheel(MouseWheelEventData& data) const;

        /**
         * @brief Apply sensitivity to value
         */
        [[nodiscard]] static float applySensitivity(float value, float sensitivity) noexcept;

        /**
         * @brief Apply sensitivity to 2D vector
         */
        [[nodiscard]] static math::Vec2 applySensitivity(const math::Vec2& value,
                                                         const SensitivityConfig::AxisSensitivity& config) noexcept;

        /**
         * @brief Apply response curve
         */
        [[nodiscard]] static float applyResponseCurve(float value, const CurveCache& cache) noexcept;

        /**
         * @brief Apply mouse acceleration
         */
        [[nodiscard]] math::Vec2 applyMouseAcceleration(const math::Vec2& delta,
                                                     float velocity,
                                                     float deltaTime) const noexcept;

        /**
         * @brief Update velocity tracking
         */
        void updateVelocity(DeviceID deviceId, const math::Vec2& position, float deltaTime);

        /**
         * @brief Get sensitivity for device
         */
        [[nodiscard]] float getDeviceSensitivity(DeviceID deviceId) const noexcept;
    };
} // namespace engine::input::processing
