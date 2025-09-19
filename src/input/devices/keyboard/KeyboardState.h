/**
 * @file KeyboardState.h
 * @brief Consolidated input state for deterministic frame-based processing
 * @author Andrés Guerrero
 * @date 10-09-2025
 */

#pragma once

#include "../../core/InputTypes.h"

#include <bitset>

namespace engine::input {
    /**
     * @brief Complete keyboard state
     */
    struct KeyboardState {
        // Current state of all keys
        std::bitset<static_cast<std::size_t>(KeyCode::KEY_COUNT)> pressed;
        std::bitset<static_cast<std::size_t>(KeyCode::KEY_COUNT)> justPressed;
        std::bitset<static_cast<std::size_t>(KeyCode::KEY_COUNT)> justReleased;

        // Modifier state
        KeyModifier modifiers;

        // Text input buffer
        std::array<char, 256> textBuffer;
        std::size_t textLength;

        KeyboardState() noexcept
            : modifiers(KeyModifier::NONE)
              , textBuffer{}
              , textLength(0) {
        }

        /**
         * @brief Check if key is currently pressed
         */
        [[nodiscard]] bool isKeyPressed(KeyCode key) const noexcept {
            const auto index = static_cast<std::size_t>(key);
            return index < pressed.size() && pressed[index];
        }

        /**
         * @brief Check if key was just pressed this frame
         */
        [[nodiscard]] bool isKeyJustPressed(KeyCode key) const noexcept {
            const auto index = static_cast<std::size_t>(key);
            return index < justPressed.size() && justPressed[index];
        }

        /**
         * @brief Check if key was just released this frame
         */
        [[nodiscard]] bool isKeyJustReleased(KeyCode key) const noexcept {
            const auto index = static_cast<std::size_t>(key);
            return index < justReleased.size() && justReleased[index];
        }

        /**
         * @brief Check if any key is pressed
         */
        [[nodiscard]] bool hasAnyKeyPressed() const noexcept {
            return pressed.any();
        }

        /**
         * @brief Get key hold duration (requires frame tracking)
         */
        [[nodiscard]] float getKeyHoldTime(const KeyCode key, const float frameTime) const noexcept {
            // This would need frame tracking implementation
            return isKeyPressed(key) ? frameTime : 0.0f;
        }

        /**
         * @brief Clear frame-specific state (just pressed/released)
         */
        void clearFrameState() noexcept {
            justPressed.reset();
            justReleased.reset();
            textLength = 0;
        }

        /**
         * @brief Update key state
         */
        void updateKey(KeyCode key, const bool isPressed) noexcept {
            const auto index = static_cast<std::size_t>(key);
            if (index >= pressed.size()) return;

            const bool wasPressed = pressed[index];
            pressed[index] = isPressed;

            if (isPressed && !wasPressed) {
                justPressed[index] = true;
            }
            else if (!isPressed && wasPressed) {
                justReleased[index] = true;
            }
        }
    };
} // namespace engine::input
