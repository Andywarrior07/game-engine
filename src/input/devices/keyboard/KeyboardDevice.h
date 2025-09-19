/**
 * @file KeyboardDevice.h
 * @brief Keyboard input device implementation
 * @author Andrés Guerrero
 * @date 12-09-2025
 *
 * Handles keyboard input processing, key repeat, and text input.
 */

#pragma once

#include "KeyboardState.h"

#include "../base/InputDevice.h"
#include "../../core/InputEvent.h"

#include <unordered_map>
#include <string>

namespace engine::input {
    /**
     * @brief Keyboard device implementation
     *
     * Manages keyboard input with support for key repeat, text input,
     * and modifier tracking.
     */
    class KeyboardDevice final : public InputDeviceBase {
    public:
        /**
         * @brief Constructor
         */
        explicit KeyboardDevice() noexcept;

        /**
         * @brief Destructor
         */
        ~KeyboardDevice() override = default;

        // ============================================================================
        // InputDevice Interface Implementation
        // ============================================================================

        bool initialize(const DeviceInitParams& params) override;
        void shutdown() override;
        void reset() noexcept override;
        void update(float deltaTime) override;
        bool pollEvents() override;
        bool processRawInput(const void* data, std::size_t size) override;

        // Rumble support (not applicable for keyboard)
        bool setRumble(const float leftMotor, const float rightMotor, const float duration) override {
            (void)leftMotor;
            (void)rightMotor;
            (void)duration;
            return false;
        }

        void stopRumble() noexcept override {
        }

        bool playHapticPattern(const std::uint32_t pattern, const float intensity) override {
            (void)pattern;
            (void)intensity;
            return false;
        }

        // ============================================================================
        // Keyboard-Specific Methods
        // ============================================================================

        /**
         * @brief Handle key press event
         */
        void onKeyPress(KeyCode key, KeyModifier modifiers = KeyModifier::NONE,
                        bool isRepeat = false);

        /**
         * @brief Handle key release event
         */
        void onKeyRelease(KeyCode key, KeyModifier modifiers = KeyModifier::NONE);

        /**
         * @brief Handle text input
         */
        void onTextInput(const char* text);

        /**
         * @brief Enable/disable text input mode
         */
        void setTextInputMode(bool enabled) noexcept;

        /**
         * @brief Check if text input is active
         */
        [[nodiscard]] bool isTextInputActive() const noexcept {
            return textInputActive_;
        }

        /**
         * @brief Check if key is pressed
         */
        [[nodiscard]] bool isKeyPressed(const KeyCode key) const noexcept {
            return keyboardState_.isKeyPressed(key);
        }

        /**
         * @brief Check if key was just pressed
         */
        [[nodiscard]] bool isKeyJustPressed(const KeyCode key) const noexcept {
            return keyboardState_.isKeyJustPressed(key);
        }

        /**
         * @brief Check if key was just released
         */
        [[nodiscard]] bool isKeyJustReleased(const KeyCode key) const noexcept {
            return keyboardState_.isKeyJustReleased(key);
        }

        /**
         * @brief Get current modifiers
         */
        [[nodiscard]] KeyModifier getModifiers() const noexcept {
            return keyboardState_.modifiers;
        }

        /**
         * @brief Check if modifier is active
         */
        [[nodiscard]] bool isModifierActive(KeyModifier modifier) const noexcept {
            return hasModifier(keyboardState_.modifiers, modifier);
        }

        /**
         * @brief Get keyboard state
         */
        [[nodiscard]] const KeyboardState& getKeyboardState() const noexcept {
            return keyboardState_;
        }

        /**
         * @brief Update snapshot with keyboard state
         */
        void updateSnapshot(InputSnapshot& snapshot) const;

        /**
         * @brief Set key repeat parameters
         */
        void setKeyRepeatParams(float delay, float rate) noexcept;

        /**
         * @brief Enable/disable key repeat
         */
        void setKeyRepeatEnabled(const bool enabled) noexcept {
            repeatEnabled_ = enabled;
        }

    private:
        // ============================================================================
        // Member Variables
        // ============================================================================

        // State tracking
        KeyboardState keyboardState_;
        KeyboardState previousState_;

        // Key repeat
        struct RepeatKey {
            KeyCode key;
            float timeHeld;
            float nextRepeatTime;

            explicit RepeatKey(const KeyCode k = KeyCode::UNKNOWN) noexcept
                : key(k), timeHeld(0.0f), nextRepeatTime(0.0f) {
            }
        };

        std::unordered_map<KeyCode, RepeatKey> repeatKeys_;
        bool repeatEnabled_;
        float repeatDelay_; // Time before first repeat
        float repeatRate_; // Time between repeats

        // Text input
        bool textInputActive_;
        std::string textBuffer_;
        static constexpr std::size_t MAX_TEXT_BUFFER = 256;

        // Text input handler
        class TextInputHandler {
        public:
            TextInputHandler() noexcept;

            void initialize();
            void shutdown();
            void reset() noexcept;
            static void update(float deltaTime);

            void beginInput();
            void endInput();
            void addText(const std::string& text);
            void handleKeyPress(KeyCode key, KeyModifier modifiers);

            [[nodiscard]] bool hasText() const noexcept { return !buffer_.empty(); }
            [[nodiscard]] const std::string& getText() const noexcept { return buffer_; }
            std::string extractText();

        private:
            std::string buffer_;
            std::string compositionText_;
            bool isComposing_;
            std::size_t cursorPos_;
        };

        TextInputHandler textInputHandler_;

        // ============================================================================
        // Private Methods
        // ============================================================================

        /**
         * @brief Process key repeat
         */
        void processKeyRepeat(float deltaTime);

        /**
         * @brief Clear keyboard state
         */
        void clearState() noexcept;

        /**
         * @brief Update modifier state from key
         */
        void updateModifierFromKey(KeyCode key, bool pressed) noexcept;

        /**
         * @brief Generate keyboard event
         */
        void generateKeyEvent(InputEventType type, KeyCode key,
                              KeyModifier modifiers, bool isRepeat = false);

        /**
         * @brief Generate text event
         */
        void generateTextEvent(const std::string& text);
    };
} // namespace engine::input
