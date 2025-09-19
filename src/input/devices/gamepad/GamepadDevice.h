/**
 * @file GamepadDevice.h
 * @brief Gamepad/Controller input device implementation
 * @author Andrés Guerrero
 * @date 12-09-2025
 *
 * Handles gamepad input processing including buttons, analog sticks,
 * triggers, rumble, and motion sensors.
 */

#pragma once

#include "GamepadState.h"

#include "../base/InputDevice.h"
#include "../../core/InputEvent.h"

#include <unordered_map>

namespace engine::input {
    /**
     * @brief Gamepad device implementation
     *
     * Manages gamepad input with support for buttons, analog sticks,
     * triggers, rumble feedback, and motion sensors.
     */
    class GamepadDevice final : public InputDeviceBase {
    public:
        /**
         * @brief Gamepad profile for button/axis mapping
         */
        enum class GamepadProfile {
            UNKNOWN,
            XBOX, // Xbox One/Series controller
            PLAYSTATION, // PS4/PS5 controller
            NINTENDO, // Switch Pro controller
            STEAM, // Steam controller
            GENERIC // Generic controller
        };

        /**
         * @brief Constructor
         */
        explicit GamepadDevice() noexcept;

        /**
         * @brief Destructor
         */
        ~GamepadDevice() override = default;

        // ============================================================================
        // InputDevice Interface Implementation
        // ============================================================================

        bool initialize(const DeviceInitParams& params) override;
        void shutdown() override;
        void reset() noexcept override;
        void update(float deltaTime) override;
        bool pollEvents() override;
        bool processRawInput(const void* data, std::size_t size) override;

        // Rumble support
        bool setRumble(float leftMotor, float rightMotor, float duration) override;
        void stopRumble() noexcept override;
        bool playHapticPattern(std::uint32_t pattern, float intensity) override;

        // ============================================================================
        // Gamepad-Specific Methods
        // ============================================================================

        /**
         * @brief Handle button press
         */
        void onButtonPress(GamepadButton button, float pressure = 1.0f);

        /**
         * @brief Handle button release
         */
        void onButtonRelease(GamepadButton button);

        /**
         * @brief Handle stick movement
         */
        void onStickMove(bool leftStick, float x, float y);

        /**
         * @brief Handle trigger movement
         */
        void onTriggerMove(bool leftTrigger, float value);

        /**
         * @brief Handle motion sensor data
         */
        void onMotionUpdate(const math::Vec3& accelerometer, const math::Vec3& gyroscope);

        /**
         * @brief Handle touchpad input (PS4/PS5)
         */
        void onTouchpadUpdate(int touchIndex, bool active, float x, float y, float pressure);

        /**
         * @brief Handle battery status update
         */
        void onBatteryUpdate(float level, bool charging);

        /**
         * @brief Check if button is pressed
         */
        [[nodiscard]] bool isButtonPressed(const GamepadButton button) const noexcept {
            return gamepadState_.isButtonPressed(button);
        }

        /**
         * @brief Check if button was just pressed
         */
        [[nodiscard]] bool isButtonJustPressed(const GamepadButton button) const noexcept {
            return gamepadState_.isButtonJustPressed(button);
        }

        /**
         * @brief Check if button was just released
         */
        [[nodiscard]] bool isButtonJustReleased(const GamepadButton button) const noexcept {
            return gamepadState_.isButtonJustReleased(button);
        }

        /**
         * @brief Get button pressure (for pressure-sensitive buttons)
         */
        [[nodiscard]] float getButtonPressure(const GamepadButton button) const noexcept {
            return gamepadState_.getButtonPressure(button);
        }

        /**
         * @brief Get left stick position
         */
        [[nodiscard]] const math::Vec2& getLeftStick() const noexcept {
            return gamepadState_.leftStick;
        }

        /**
         * @brief Get right stick position
         */
        [[nodiscard]] const math::Vec2& getRightStick() const noexcept {
            return gamepadState_.rightStick;
        }

        /**
         * @brief Get left trigger value
         */
        [[nodiscard]] float getLeftTrigger() const noexcept {
            return gamepadState_.leftTrigger;
        }

        /**
         * @brief Get right trigger value
         */
        [[nodiscard]] float getRightTrigger() const noexcept {
            return gamepadState_.rightTrigger;
        }

        /**
         * @brief Get gamepad state
         */
        [[nodiscard]] const GamepadState& getGamepadState() const noexcept {
            return gamepadState_;
        }

        /**
         * @brief Update snapshot with gamepad state
         */
        void updateSnapshot(InputSnapshot& snapshot) const;

        /**
         * @brief Set deadzone settings
         */
        void setDeadzoneSettings(const DeadzoneSettings& settings) noexcept override;

        /**
         * @brief Set stick sensitivity
         */
        void setStickSensitivity(float sensitivity) noexcept;

        /**
         * @brief Set trigger threshold
         */
        void setTriggerThreshold(float threshold) noexcept;

        /**
         * @brief Get gamepad profile
         */
        [[nodiscard]] GamepadProfile getProfile() const noexcept {
            return profile_;
        }

        /**
         * @brief Set gamepad profile
         */
        void setProfile(GamepadProfile profile) noexcept;

        /**
         * @brief Check if gamepad has motion sensors
         */
        [[nodiscard]] bool hasMotion() const noexcept {
            return gamepadState_.hasMotion;
        }

        /**
         * @brief Check if gamepad has touchpad
         */
        [[nodiscard]] bool hasTouchpad() const noexcept {
            return gamepadState_.hasTouchpad;
        }

        /**
         * @brief Get battery level
         */
        [[nodiscard]] float getBatteryLevel() const noexcept {
            return gamepadState_.batteryLevel;
        }

        /**
         * @brief Check if gamepad is wireless
         */
        [[nodiscard]] bool isWireless() const noexcept {
            return gamepadState_.isWireless;
        }

        /**
         * @brief Enable/disable adaptive triggers (PS5)
         */
        void setAdaptiveTriggers(bool enabled) noexcept;

        /**
         * @brief Set light bar color (PS4/PS5)
         */
        void setLightBarColor(float r, float g, float b) noexcept;

    private:
        // ============================================================================
        // Member Variables
        // ============================================================================

        // State tracking
        GamepadState gamepadState_;
        GamepadState previousState_;

        // Profile and mapping
        GamepadProfile profile_;
        std::unordered_map<std::uint32_t, GamepadButton> buttonMapping_;
        std::unordered_map<std::uint32_t, GamepadAxis> axisMapping_;

        // Settings
        DeadzoneSettings stickDeadzone_;
        DeadzoneSettings triggerDeadzone_;
        float stickSensitivity_;
        float triggerThreshold_;

        // Response curves
        ResponseCurve stickCurve_;
        ResponseCurve triggerCurve_;
        float stickCurveStrength_;
        float triggerCurveStrength_;

        // Rumble
        bool rumbleEnabled_;
        float rumbleIntensityScale_;

        // Features
        bool adaptiveTriggersEnabled_;
        bool lightBarEnabled_;

        struct {
            float r, g, b;
        } lightBarColor_;

        // Calibration
        struct CalibrationData {
            math::Vec2 leftStickCenter;
            math::Vec2 rightStickCenter;
            math::Vec2 leftStickMax;
            math::Vec2 rightStickMax;
            float leftTriggerMin;
            float leftTriggerMax;
            float rightTriggerMin;
            float rightTriggerMax;
            bool isCalibrated;

            CalibrationData() noexcept
                : leftStickCenter(0, 0)
                  , rightStickCenter(0, 0)
                  , leftStickMax(1, 1)
                  , rightStickMax(1, 1)
                  , leftTriggerMin(0)
                  , leftTriggerMax(1)
                  , rightTriggerMin(0)
                  , rightTriggerMax(1)
                  , isCalibrated(false) {
            }
        } calibration_;

        // ============================================================================
        // Private Methods
        // ============================================================================

        /**
         * @brief Clear gamepad state
         */
        void clearState() noexcept;

        /**
         * @brief Apply deadzone to stick input
         */
        math::Vec2 applyStickDeadzone(const math::Vec2& raw, bool leftStick) const noexcept;

        /**
         * @brief Apply deadzone to trigger input
         */
        float applyTriggerDeadzone(float raw) const noexcept;

        /**
         * @brief Apply response curve to input
         */
        static float applyResponseCurve(float value, ResponseCurve curve, float strength) noexcept;

        /**
         * @brief Apply calibration to stick
         */
        math::Vec2 applyStickCalibration(const math::Vec2& raw, bool leftStick) const noexcept;

        /**
         * @brief Apply calibration to trigger
         */
        float applyTriggerCalibration(float raw, bool leftTrigger) const noexcept;

        /**
         * @brief Generate button event
         */
        void generateButtonEvent(InputEventType type, GamepadButton button, float pressure);

        /**
         * @brief Generate axis event
         */
        void generateAxisEvent(GamepadAxis axis, float value, float delta);

        /**
         * @brief Setup default button mappings
         */
        void setupDefaultMappings(GamepadProfile profile);

        /**
         * @brief Send rumble command to backend
         */
        void sendRumbleCommand(float left, float right);

        /**
         * @brief Update D-Pad state from axes (for controllers with analog D-Pads)
         */
        void updateDPadFromAxes(float x, float y);
    };
} // namespace engine::input
