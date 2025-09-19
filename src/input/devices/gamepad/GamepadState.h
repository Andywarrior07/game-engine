/**
 * @file GamepadState.h
 * @brief Complete gamepad state tracking
 * @author Andrés Guerrero
 * @date 12-09-2025
 *
 * Comprehensive gamepad state management without SDL dependencies.
 */

#pragma once

#include "../../core/InputTypes.h"

#include "../../../math/core/MathTypes.h"
#include "../../../math/core/MathFunctions.h"

#include <bitset>
#include <array>
#include <chrono>
#include <string>

namespace engine::input {
    /**
     * @brief Complete gamepad state
     */
    struct GamepadState {
        // Device info
        DeviceID deviceId;
        std::string name;
        bool isConnected;
        std::uint8_t playerIndex;

        // Button states
        std::bitset<static_cast<std::size_t>(GamepadButton::BUTTON_COUNT)> buttons;
        std::bitset<static_cast<std::size_t>(GamepadButton::BUTTON_COUNT)> buttonsJustPressed;
        std::bitset<static_cast<std::size_t>(GamepadButton::BUTTON_COUNT)> buttonsJustReleased;

        // Analog inputs
        math::Vec2 leftStick;
        math::Vec2 rightStick;
        float leftTrigger;
        float rightTrigger;

        // Raw values (before deadzone)
        math::Vec2 leftStickRaw;
        math::Vec2 rightStickRaw;
        float leftTriggerRaw;
        float rightTriggerRaw;

        // D-Pad state (can be analog on some controllers)
        math::Vec2 dpadVector;
        bool dpadUp;
        bool dpadDown;
        bool dpadLeft;
        bool dpadRight;

        // Rumble state
        float leftMotor; // Low frequency (0-1)
        float rightMotor; // High frequency (0-1)
        float rumbleDuration;
        std::chrono::steady_clock::time_point rumbleEndTime;

        // Battery info
        float batteryLevel; // 0-1 (-1 if unknown)
        bool isCharging;
        bool isWireless;

        // Motion sensors (if available)
        math::Vec3 accelerometer;
        math::Vec3 gyroscope;
        bool hasMotion;

        // Touchpad (PS4/PS5 controllers)
        struct TouchpadState {
            math::Vec2 position;
            float pressure;
            bool isActive;
            std::uint32_t touchId;
        };

        std::array<TouchpadState, 2> touchpads{};
        bool hasTouchpad;

        // Button pressure (for pressure-sensitive controllers)
        std::array<float, static_cast<std::size_t>(GamepadButton::BUTTON_COUNT)> buttonPressure{};
        bool hasPressureSensitive;

        // Timestamps
        std::chrono::steady_clock::time_point lastActivityTime;
        std::chrono::steady_clock::time_point connectedTime;

        GamepadState() noexcept
            : deviceId(INVALID_DEVICE_ID)
              , isConnected(false)
              , playerIndex(0)
              , leftStick(0, 0)
              , rightStick(0, 0)
              , leftTrigger(0)
              , rightTrigger(0)
              , leftStickRaw(0, 0)
              , rightStickRaw(0, 0)
              , leftTriggerRaw(0)
              , rightTriggerRaw(0)
              , dpadVector(0, 0)
              , dpadUp(false)
              , dpadDown(false)
              , dpadLeft(false)
              , dpadRight(false)
              , leftMotor(0)
              , rightMotor(0)
              , rumbleDuration(0)
              , batteryLevel(-1)
              , isCharging(false)
              , isWireless(false)
              , accelerometer(0, 0, 0)
              , gyroscope(0, 0, 0)
              , hasMotion(false)
              , hasTouchpad(false)
              , hasPressureSensitive(false) {
            touchpads.fill({math::Vec2(0, 0), 0.0f, false, 0});
            buttonPressure.fill(0.0f);
        }

        /**
         * @brief Check if button is pressed
         */
        [[nodiscard]] bool isButtonPressed(GamepadButton button) const noexcept {
            const auto index = static_cast<std::size_t>(button);
            return index < buttons.size() && buttons[index];
        }

        /**
         * @brief Check if button was just pressed
         */
        [[nodiscard]] bool isButtonJustPressed(GamepadButton button) const noexcept {
            const auto index = static_cast<std::size_t>(button);
            return index < buttonsJustPressed.size() && buttonsJustPressed[index];
        }

        /**
         * @brief Check if button was just released
         */
        [[nodiscard]] bool isButtonJustReleased(GamepadButton button) const noexcept {
            const auto index = static_cast<std::size_t>(button);
            return index < buttonsJustReleased.size() && buttonsJustReleased[index];
        }

        /**
         * @brief Get button pressure (0-1)
         */
        [[nodiscard]] float getButtonPressure(GamepadButton button) const noexcept {
            if (!hasPressureSensitive) {
                return isButtonPressed(button) ? 1.0f : 0.0f;
            }

            const auto index = static_cast<std::size_t>(button);
            return index < buttonPressure.size() ? buttonPressure[index] : 0.0f;
        }

        /**
         * @brief Update button state
         */
        void updateButton(GamepadButton button, const bool pressed, const float pressure = 1.0f) noexcept {
            const auto index = static_cast<std::size_t>(button);
            if (index >= buttons.size()) return;

            const bool wasPressed = buttons[index];
            buttons[index] = pressed;
            buttonPressure[index] = pressed ? pressure : 0.0f;

            if (pressed && !wasPressed) {
                buttonsJustPressed[index] = true;
            }
            else if (!pressed && wasPressed) {
                buttonsJustReleased[index] = true;
            }

            updateActivity();
        }

        /**
         * @brief Update D-Pad from buttons
         */
        void updateDPadFromButtons() noexcept {
            dpadUp = isButtonPressed(GamepadButton::DPAD_UP);
            dpadDown = isButtonPressed(GamepadButton::DPAD_DOWN);
            dpadLeft = isButtonPressed(GamepadButton::DPAD_LEFT);
            dpadRight = isButtonPressed(GamepadButton::DPAD_RIGHT);

            // Convert to vector
            dpadVector.x = 0;
            dpadVector.y = 0;
            if (dpadLeft) dpadVector.x -= 1.0f;
            if (dpadRight) dpadVector.x += 1.0f;
            if (dpadUp) dpadVector.y += 1.0f;
            if (dpadDown) dpadVector.y -= 1.0f;

            // Normalize diagonal movement
            if (glm::length2(dpadVector) > 1.0f) {
                dpadVector = glm::normalize(dpadVector);
            }
        }

        /**
         * @brief Get stick direction as angle
         */
        [[nodiscard]] float getStickAngle(const bool leftStickParam) const noexcept {
            const math::Vec2& stick = leftStickParam ? leftStick : rightStick;
            return std::atan2(stick.y, stick.x);
        }

        /**
         * @brief Get stick magnitude
         */
        [[nodiscard]] float getStickMagnitude(const bool leftStickParam) const noexcept {
            const math::Vec2& stick = leftStickParam ? leftStick : rightStick;
            return glm::length2(stick);
        }

        /**
         * @brief Check if any input is active
         */
        [[nodiscard]] bool hasAnyInput() const noexcept {
            return buttons.any() ||
                glm::length2(leftStick) > 0.01f ||
                glm::length2(rightStick) > 0.01f ||
                leftTrigger > 0.01f ||
                rightTrigger > 0.01f ||
                glm::length2(dpadVector) > 0.01f;
        }

        /**
         * @brief Apply deadzone to analog inputs
         */
        void applyDeadzone(const DeadzoneSettings& settings) noexcept {
            leftStick = settings.apply(leftStickRaw);
            rightStick = settings.apply(rightStickRaw);
            leftTrigger = settings.apply(leftTriggerRaw);
            rightTrigger = settings.apply(rightTriggerRaw);
        }

        /**
         * @brief Clear frame-specific state
         */
        void clearFrameState() noexcept {
            buttonsJustPressed.reset();
            buttonsJustReleased.reset();
        }

        /**
         * @brief Update rumble state
         */
        void updateRumble(const float deltaTime) noexcept {
            if (rumbleDuration > 0) {
                rumbleDuration -= deltaTime;
                if (rumbleDuration <= 0) {
                    leftMotor = 0;
                    rightMotor = 0;
                    rumbleDuration = 0;
                }
            }

            // Check if rumble should end based on time
            if (rumbleEndTime != std::chrono::steady_clock::time_point{} &&
                std::chrono::steady_clock::now() >= rumbleEndTime) {
                leftMotor = 0;
                rightMotor = 0;
                rumbleEndTime = std::chrono::steady_clock::time_point{};
            }
        }

        /**
         * @brief Set rumble
         */
        void setRumble(const float left, const float right, const float duration = 0.0f) noexcept {
            leftMotor = math::clamp(left, 0.0f, 1.0f);
            rightMotor = math::clamp(right, 0.0f, 1.0f);

            if (duration > 0) {
                rumbleDuration = duration;
                rumbleEndTime = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(static_cast<int>(duration * 1000));
            }
            else {
                rumbleDuration = 0;
                rumbleEndTime = std::chrono::steady_clock::time_point{};
            }
        }

        /**
         * @brief Stop rumble
         */
        void stopRumble() noexcept {
            leftMotor = 0;
            rightMotor = 0;
            rumbleDuration = 0;
            rumbleEndTime = std::chrono::steady_clock::time_point{};
        }

        /**
         * @brief Update activity timestamp
         */
        void updateActivity() noexcept {
            lastActivityTime = std::chrono::steady_clock::now();
        }

        /**
         * @brief Get idle time
         */
        [[nodiscard]] float getIdleTime() const noexcept {
            const auto duration = std::chrono::steady_clock::now() - lastActivityTime;

            return std::chrono::duration<float>(duration).count();
        }

        /**
         * @brief Reset to default state
         */
        void reset() noexcept {
            buttons.reset();
            buttonsJustPressed.reset();
            buttonsJustReleased.reset();
            leftStick = rightStick = math::Vec2(0, 0);
            leftStickRaw = rightStickRaw = math::Vec2(0, 0);
            leftTrigger = rightTrigger = 0;
            leftTriggerRaw = rightTriggerRaw = 0;
            dpadVector = math::Vec2(0, 0);
            dpadUp = dpadDown = dpadLeft = dpadRight = false;
            stopRumble();
            buttonPressure.fill(0.0f);

            for (auto& touchpad : touchpads) {
                touchpad.isActive = false;
                touchpad.position = math::Vec2(0, 0);
                touchpad.pressure = 0;
            }
        }
    };
} // namespace engine::input
