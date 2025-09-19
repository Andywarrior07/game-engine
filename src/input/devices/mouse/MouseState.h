/**
 * @file MouseState.h
 * @brief Consolidated input state for deterministic frame-based processing
 * @author Andrés Guerrero
 * @date 10-09-2025
 */

#pragma once

#include "../../core/InputTypes.h"

#include "../../../math/core/MathConstants.h"

#include <bitset>
#include <glm/gtx/norm.hpp>

namespace engine::input {
    /**
     * @brief Complete mouse state
     */
    struct MouseState {
        // Position and movement
        math::Vec2 position; // Current position in screen coords
        math::Vec2 delta; // Movement since last frame
        math::Vec2 normalizedPos; // Position normalized to [0,1]
        math::Vec2 wheelDelta; // Scroll wheel movement

        // Button states
        std::bitset<static_cast<std::size_t>(MouseButton::BUTTON_COUNT)> buttons;
        std::bitset<static_cast<std::size_t>(MouseButton::BUTTON_COUNT)> justPressed;
        std::bitset<static_cast<std::size_t>(MouseButton::BUTTON_COUNT)> justReleased;

        // Cursor state
        CursorMode cursorMode;
        bool isInWindow;

        // Click tracking for double/triple clicks
        std::array<std::uint8_t, static_cast<std::size_t>(MouseButton::BUTTON_COUNT)> clickCounts;
        std::array<InputTimestamp, static_cast<std::size_t>(MouseButton::BUTTON_COUNT)> lastClickTimes;

        MouseState() noexcept
            : position(0, 0)
              , delta(0, 0)
              , normalizedPos(0, 0)
              , wheelDelta(0, 0)
              , cursorMode(CursorMode::NORMAL)
              , isInWindow(true)
              , clickCounts{}
              , lastClickTimes{} {
        }

        /**
         * @brief Check if button is pressed
         */
        [[nodiscard]] bool isButtonPressed(MouseButton button) const noexcept {
            const auto index = static_cast<std::size_t>(button);
            return index < buttons.size() && buttons[index];
        }

        /**
         * @brief Check if button was just pressed
         */
        [[nodiscard]] bool isButtonJustPressed(MouseButton button) const noexcept {
            const auto index = static_cast<std::size_t>(button);
            return index < justPressed.size() && justPressed[index];
        }

        /**
         * @brief Check if button was just released
         */
        [[nodiscard]] bool isButtonJustReleased(MouseButton button) const noexcept {
            const auto index = static_cast<std::size_t>(button);
            return index < justReleased.size() && justReleased[index];
        }

        /**
         * @brief Check if mouse is moving
         */
        [[nodiscard]] bool isMoving(const float threshold = 0.001f) const noexcept {
            return glm::length2(delta) > threshold * threshold;
        }

        /**
         * @brief Get normalized delta for camera control
         */
        [[nodiscard]] math::Vec2 getNormalizedDelta(const float sensitivity = 1.0f) const noexcept {
            return delta * sensitivity;
        }

        /**
         * @brief Clear frame-specific state
         */
        void clearFrameState() noexcept {
            delta = math::VEC2_ZERO;
            wheelDelta = math::VEC2_ZERO;
            justPressed.reset();
            justReleased.reset();
        }

        /**
         * @brief Update button state
         */
        void updateButton(MouseButton button, const bool isPressed) noexcept {
            const auto index = static_cast<std::size_t>(button);
            if (index >= buttons.size()) return;

            const bool wasPressed = buttons[index];
            buttons[index] = isPressed;

            if (isPressed && !wasPressed) {
                justPressed[index] = true;

                // Track clicks for double/triple click detection
                const auto now = InputTimestamp::clock::now();
                const auto timeSinceLastClick = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - lastClickTimes[index]).count();

                if (timeSinceLastClick < 500) {
                    // 500ms double-click threshold
                    clickCounts[index]++;
                }
                else {
                    clickCounts[index] = 1;
                }
                lastClickTimes[index] = now;
            }
            else if (!isPressed && wasPressed) {
                justReleased[index] = true;
            }
        }
    };
} // namespace engine::input
