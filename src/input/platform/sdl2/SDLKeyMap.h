/**
 * @file SDLKeyMap.h
 * @brief SDL2 to engine key code mapping
 * @author Game Engine Team
 * @date 2024
 *
 * Provides bidirectional mapping between SDL scancodes and engine key codes.
 * Uses RAII and modern C++ best practices without singleton pattern.
 */

#pragma once

#include "../../core/InputTypes.h"

#include <SDL2/SDL.h>

#include <unordered_map>
#include <memory>
#include <optional>

namespace engine::input {
    /**
     * @brief Maps SDL scancodes to engine key codes and vice versa
     *
     * Non-singleton implementation that can be instantiated as needed.
     * Uses RAII principles and provides efficient bidirectional mapping.
     */
    class SDLKeyMap {
    public:
        /**
         * @brief Construct and initialize all mappings
         */
        SDLKeyMap() {
            // Reserve space for better performance
            sdlToEngineMap_.reserve(256);
            engineToSDLMap_.reserve(256);
            sdlButtonMap_.reserve(32);
            engineButtonMap_.reserve(32);
            sdlAxisMap_.reserve(8);
            engineAxisMap_.reserve(8);

            // Initialize all mappings
            initializeKeyMap();
            initializeButtonMap();
            initializeAxisMap();
        }

        ~SDLKeyMap() = default;

        // Delete copy constructor and assignment for unique ownership semantics
        SDLKeyMap(const SDLKeyMap&) = delete;
        SDLKeyMap& operator=(const SDLKeyMap&) = delete;

        // Allow move semantics
        SDLKeyMap(SDLKeyMap&&) noexcept = default;
        SDLKeyMap& operator=(SDLKeyMap&&) noexcept = default;

        /**
         * @brief Convert SDL scancode to engine key code
         * @param scancode SDL scancode to convert
         * @return Optional containing key code if mapping exists
         */
        [[nodiscard]] std::optional<KeyCode> sdlToEngineKey(const SDL_Scancode scancode) const noexcept {
            if (const auto it = sdlToEngineMap_.find(scancode); it != sdlToEngineMap_.end()) {
                return it->second;
            }
            return std::nullopt;
        }

        /**
         * @brief Convert SDL keycode to engine key code
         * @param keycode SDL keycode to convert
         * @return Optional containing key code if mapping exists
         */
        [[nodiscard]] std::optional<KeyCode> sdlToEngineKey(const SDL_Keycode keycode) const noexcept {
            const SDL_Scancode scancode = SDL_GetScancodeFromKey(keycode);
            return sdlToEngineKey(scancode);
        }

        /**
         * @brief Convert engine key code to SDL scancode
         * @param keyCode Engine key code to convert
         * @return Optional containing SDL scancode if mapping exists
         */
        [[nodiscard]] std::optional<SDL_Scancode> engineToSDL(const KeyCode keyCode) const noexcept {
            if (const auto it = engineToSDLMap_.find(keyCode); it != engineToSDLMap_.end()) {
                return it->second;
            }
            return std::nullopt;
        }

        /**
         * @brief Convert SDL gamepad button to engine button
         * @param button SDL gamepad button to convert
         * @return Optional containing gamepad button if mapping exists
         */
        [[nodiscard]] std::optional<GamepadButton> sdlToEngineButton(
            const SDL_GameControllerButton button) const noexcept {
            if (const auto it = sdlButtonMap_.find(button); it != sdlButtonMap_.end()) {
                return it->second;
            }
            return std::nullopt;
        }

        /**
         * @brief Convert engine gamepad button to SDL button
         * @param button Engine gamepad button to convert
         * @return Optional containing SDL button if mapping exists
         */
        [[nodiscard]] std::optional<SDL_GameControllerButton> engineToSDLButton(
            const GamepadButton button) const noexcept {
            if (const auto it = engineButtonMap_.find(button); it != engineButtonMap_.end()) {
                return it->second;
            }
            return std::nullopt;
        }

        /**
         * @brief Convert SDL gamepad axis to engine axis
         * @param axis SDL gamepad axis to convert
         * @return Optional containing gamepad axis if mapping exists
         */
        [[nodiscard]] std::optional<GamepadAxis> sdlToEngineAxis(const SDL_GameControllerAxis axis) const noexcept {
            if (const auto it = sdlAxisMap_.find(axis); it != sdlAxisMap_.end()) {
                return it->second;
            }
            return std::nullopt;
        }

        /**
         * @brief Convert engine gamepad axis to SDL axis
         * @param axis Engine gamepad axis to convert
         * @return Optional containing SDL axis if mapping exists
         */
        [[nodiscard]] std::optional<SDL_GameControllerAxis> engineToSDLAxis(const GamepadAxis axis) const noexcept {
            if (const auto it = engineAxisMap_.find(axis); it != engineAxisMap_.end()) {
                return it->second;
            }
            return std::nullopt;
        }

        /**
         * @brief Convert SDL key modifiers to engine modifiers
         * @param mod SDL modifier mask
         * @return Engine modifier mask
         */
        static KeyModifier sdlToEngineModifiers(const Uint16 mod) noexcept {
            auto result = KeyModifier::NONE;

            if (mod & KMOD_LSHIFT) result = result | KeyModifier::LEFT_SHIFT;
            if (mod & KMOD_RSHIFT) result = result | KeyModifier::RIGHT_SHIFT;
            if (mod & KMOD_LCTRL) result = result | KeyModifier::LEFT_CTRL;
            if (mod & KMOD_RCTRL) result = result | KeyModifier::RIGHT_CTRL;
            if (mod & KMOD_LALT) result = result | KeyModifier::LEFT_ALT;
            if (mod & KMOD_RALT) result = result | KeyModifier::RIGHT_ALT;
            if (mod & KMOD_LGUI) result = result | KeyModifier::LEFT_SUPER;
            if (mod & KMOD_RGUI) result = result | KeyModifier::RIGHT_SUPER;

            return result;
        }

        /**
         * @brief Convert engine modifiers to SDL modifiers
         * @param mod Engine modifier mask
         * @return SDL modifier mask
         */
        static Uint16 engineToSDLModifiers(const KeyModifier mod) noexcept {
            Uint16 result = 0;

            if (hasModifier(mod, KeyModifier::LEFT_SHIFT)) result |= KMOD_LSHIFT;
            if (hasModifier(mod, KeyModifier::RIGHT_SHIFT)) result |= KMOD_RSHIFT;
            if (hasModifier(mod, KeyModifier::LEFT_CTRL)) result |= KMOD_LCTRL;
            if (hasModifier(mod, KeyModifier::RIGHT_CTRL)) result |= KMOD_RCTRL;
            if (hasModifier(mod, KeyModifier::LEFT_ALT)) result |= KMOD_LALT;
            if (hasModifier(mod, KeyModifier::RIGHT_ALT)) result |= KMOD_RALT;
            if (hasModifier(mod, KeyModifier::LEFT_SUPER)) result |= KMOD_LGUI;
            if (hasModifier(mod, KeyModifier::RIGHT_SUPER)) result |= KMOD_RGUI;

            return result;
        }

        /**
         * @brief Add custom key mapping at runtime
         * @param sdl SDL scancode
         * @param engine Engine key code
         */
        void addCustomKeyMapping(const SDL_Scancode sdl, const KeyCode engine) {
            addKeyMapping(sdl, engine);
        }

        /**
         * @brief Remove key mapping
         * @param engine Engine key code to remove
         */
        void removeKeyMapping(const KeyCode engine) {
            if (const auto it = engineToSDLMap_.find(engine); it != engineToSDLMap_.end()) {
                const SDL_Scancode sdl = it->second;
                engineToSDLMap_.erase(it);
                sdlToEngineMap_.erase(sdl);
            }
        }

        /**
         * @brief Clear all mappings
         */
        void clearMappings() {
            sdlToEngineMap_.clear();
            engineToSDLMap_.clear();
            sdlButtonMap_.clear();
            engineButtonMap_.clear();
            sdlAxisMap_.clear();
            engineAxisMap_.clear();
        }

        /**
         * @brief Reset to default mappings
         */
        void resetToDefaults() {
            clearMappings();
            initializeKeyMap();
            initializeButtonMap();
            initializeAxisMap();
        }

        /**
         * @brief Get total number of key mappings
         */
        [[nodiscard]] size_t getKeyMappingCount() const noexcept {
            return sdlToEngineMap_.size();
        }

        /**
         * @brief Get total number of button mappings
         */
        [[nodiscard]] size_t getButtonMappingCount() const noexcept {
            return sdlButtonMap_.size();
        }

        /**
         * @brief Get total number of axis mappings
         */
        [[nodiscard]] size_t getAxisMappingCount() const noexcept {
            return sdlAxisMap_.size();
        }

    private:
        // Mapping tables
        std::unordered_map<SDL_Scancode, KeyCode> sdlToEngineMap_;
        std::unordered_map<KeyCode, SDL_Scancode> engineToSDLMap_;
        std::unordered_map<SDL_GameControllerButton, GamepadButton> sdlButtonMap_;
        std::unordered_map<GamepadButton, SDL_GameControllerButton> engineButtonMap_;
        std::unordered_map<SDL_GameControllerAxis, GamepadAxis> sdlAxisMap_;
        std::unordered_map<GamepadAxis, SDL_GameControllerAxis> engineAxisMap_;

        /**
         * @brief Initialize keyboard key mappings
         */
        void initializeKeyMap() {
            // Letters
            addKeyMapping(SDL_SCANCODE_A, KeyCode::A);
            addKeyMapping(SDL_SCANCODE_B, KeyCode::B);
            addKeyMapping(SDL_SCANCODE_C, KeyCode::C);
            addKeyMapping(SDL_SCANCODE_D, KeyCode::D);
            addKeyMapping(SDL_SCANCODE_E, KeyCode::E);
            addKeyMapping(SDL_SCANCODE_F, KeyCode::F);
            addKeyMapping(SDL_SCANCODE_G, KeyCode::G);
            addKeyMapping(SDL_SCANCODE_H, KeyCode::H);
            addKeyMapping(SDL_SCANCODE_I, KeyCode::I);
            addKeyMapping(SDL_SCANCODE_J, KeyCode::J);
            addKeyMapping(SDL_SCANCODE_K, KeyCode::K);
            addKeyMapping(SDL_SCANCODE_L, KeyCode::L);
            addKeyMapping(SDL_SCANCODE_M, KeyCode::M);
            addKeyMapping(SDL_SCANCODE_N, KeyCode::N);
            addKeyMapping(SDL_SCANCODE_O, KeyCode::O);
            addKeyMapping(SDL_SCANCODE_P, KeyCode::P);
            addKeyMapping(SDL_SCANCODE_Q, KeyCode::Q);
            addKeyMapping(SDL_SCANCODE_R, KeyCode::R);
            addKeyMapping(SDL_SCANCODE_S, KeyCode::S);
            addKeyMapping(SDL_SCANCODE_T, KeyCode::T);
            addKeyMapping(SDL_SCANCODE_U, KeyCode::U);
            addKeyMapping(SDL_SCANCODE_V, KeyCode::V);
            addKeyMapping(SDL_SCANCODE_W, KeyCode::W);
            addKeyMapping(SDL_SCANCODE_X, KeyCode::X);
            addKeyMapping(SDL_SCANCODE_Y, KeyCode::Y);
            addKeyMapping(SDL_SCANCODE_Z, KeyCode::Z);

            // Numbers
            addKeyMapping(SDL_SCANCODE_0, KeyCode::NUM_0);
            addKeyMapping(SDL_SCANCODE_1, KeyCode::NUM_1);
            addKeyMapping(SDL_SCANCODE_2, KeyCode::NUM_2);
            addKeyMapping(SDL_SCANCODE_3, KeyCode::NUM_3);
            addKeyMapping(SDL_SCANCODE_4, KeyCode::NUM_4);
            addKeyMapping(SDL_SCANCODE_5, KeyCode::NUM_5);
            addKeyMapping(SDL_SCANCODE_6, KeyCode::NUM_6);
            addKeyMapping(SDL_SCANCODE_7, KeyCode::NUM_7);
            addKeyMapping(SDL_SCANCODE_8, KeyCode::NUM_8);
            addKeyMapping(SDL_SCANCODE_9, KeyCode::NUM_9);

            // Function keys
            addKeyMapping(SDL_SCANCODE_F1, KeyCode::F1);
            addKeyMapping(SDL_SCANCODE_F2, KeyCode::F2);
            addKeyMapping(SDL_SCANCODE_F3, KeyCode::F3);
            addKeyMapping(SDL_SCANCODE_F4, KeyCode::F4);
            addKeyMapping(SDL_SCANCODE_F5, KeyCode::F5);
            addKeyMapping(SDL_SCANCODE_F6, KeyCode::F6);
            addKeyMapping(SDL_SCANCODE_F7, KeyCode::F7);
            addKeyMapping(SDL_SCANCODE_F8, KeyCode::F8);
            addKeyMapping(SDL_SCANCODE_F9, KeyCode::F9);
            addKeyMapping(SDL_SCANCODE_F10, KeyCode::F10);
            addKeyMapping(SDL_SCANCODE_F11, KeyCode::F11);
            addKeyMapping(SDL_SCANCODE_F12, KeyCode::F12);
            addKeyMapping(SDL_SCANCODE_F13, KeyCode::F13);
            addKeyMapping(SDL_SCANCODE_F14, KeyCode::F14);
            addKeyMapping(SDL_SCANCODE_F15, KeyCode::F15);
            addKeyMapping(SDL_SCANCODE_F16, KeyCode::F16);
            addKeyMapping(SDL_SCANCODE_F17, KeyCode::F17);
            addKeyMapping(SDL_SCANCODE_F18, KeyCode::F18);
            addKeyMapping(SDL_SCANCODE_F19, KeyCode::F19);
            addKeyMapping(SDL_SCANCODE_F20, KeyCode::F20);
            addKeyMapping(SDL_SCANCODE_F21, KeyCode::F21);
            addKeyMapping(SDL_SCANCODE_F22, KeyCode::F22);
            addKeyMapping(SDL_SCANCODE_F23, KeyCode::F23);
            addKeyMapping(SDL_SCANCODE_F24, KeyCode::F24);

            // Control keys
            addKeyMapping(SDL_SCANCODE_ESCAPE, KeyCode::ESCAPE);
            addKeyMapping(SDL_SCANCODE_RETURN, KeyCode::ENTER);
            addKeyMapping(SDL_SCANCODE_TAB, KeyCode::TAB);
            addKeyMapping(SDL_SCANCODE_BACKSPACE, KeyCode::BACKSPACE);
            addKeyMapping(SDL_SCANCODE_INSERT, KeyCode::INSERT);
            addKeyMapping(SDL_SCANCODE_DELETE, KeyCode::DELETE);
            addKeyMapping(SDL_SCANCODE_RIGHT, KeyCode::RIGHT);
            addKeyMapping(SDL_SCANCODE_LEFT, KeyCode::LEFT);
            addKeyMapping(SDL_SCANCODE_DOWN, KeyCode::DOWN);
            addKeyMapping(SDL_SCANCODE_UP, KeyCode::UP);
            addKeyMapping(SDL_SCANCODE_PAGEUP, KeyCode::PAGE_UP);
            addKeyMapping(SDL_SCANCODE_PAGEDOWN, KeyCode::PAGE_DOWN);
            addKeyMapping(SDL_SCANCODE_HOME, KeyCode::HOME);
            addKeyMapping(SDL_SCANCODE_END, KeyCode::END);

            // Modifiers
            addKeyMapping(SDL_SCANCODE_CAPSLOCK, KeyCode::CAPS_LOCK);
            addKeyMapping(SDL_SCANCODE_SCROLLLOCK, KeyCode::SCROLL_LOCK);
            addKeyMapping(SDL_SCANCODE_NUMLOCKCLEAR, KeyCode::NUM_LOCK);
            addKeyMapping(SDL_SCANCODE_PRINTSCREEN, KeyCode::PRINT_SCREEN);
            addKeyMapping(SDL_SCANCODE_PAUSE, KeyCode::PAUSE);

            addKeyMapping(SDL_SCANCODE_LSHIFT, KeyCode::LEFT_SHIFT);
            addKeyMapping(SDL_SCANCODE_LCTRL, KeyCode::LEFT_CTRL);
            addKeyMapping(SDL_SCANCODE_LALT, KeyCode::LEFT_ALT);
            addKeyMapping(SDL_SCANCODE_LGUI, KeyCode::LEFT_SUPER);
            addKeyMapping(SDL_SCANCODE_RSHIFT, KeyCode::RIGHT_SHIFT);
            addKeyMapping(SDL_SCANCODE_RCTRL, KeyCode::RIGHT_CTRL);
            addKeyMapping(SDL_SCANCODE_RALT, KeyCode::RIGHT_ALT);
            addKeyMapping(SDL_SCANCODE_RGUI, KeyCode::RIGHT_SUPER);

            // Symbols
            addKeyMapping(SDL_SCANCODE_SPACE, KeyCode::SPACE);
            addKeyMapping(SDL_SCANCODE_APOSTROPHE, KeyCode::APOSTROPHE);
            addKeyMapping(SDL_SCANCODE_COMMA, KeyCode::COMMA);
            addKeyMapping(SDL_SCANCODE_MINUS, KeyCode::MINUS);
            addKeyMapping(SDL_SCANCODE_PERIOD, KeyCode::PERIOD);
            addKeyMapping(SDL_SCANCODE_SLASH, KeyCode::SLASH);
            addKeyMapping(SDL_SCANCODE_SEMICOLON, KeyCode::SEMICOLON);
            addKeyMapping(SDL_SCANCODE_EQUALS, KeyCode::EQUAL);
            addKeyMapping(SDL_SCANCODE_LEFTBRACKET, KeyCode::LEFT_BRACKET);
            addKeyMapping(SDL_SCANCODE_BACKSLASH, KeyCode::BACKSLASH);
            addKeyMapping(SDL_SCANCODE_RIGHTBRACKET, KeyCode::RIGHT_BRACKET);
            addKeyMapping(SDL_SCANCODE_GRAVE, KeyCode::GRAVE);

            // Numpad
            addKeyMapping(SDL_SCANCODE_KP_0, KeyCode::KP_0);
            addKeyMapping(SDL_SCANCODE_KP_1, KeyCode::KP_1);
            addKeyMapping(SDL_SCANCODE_KP_2, KeyCode::KP_2);
            addKeyMapping(SDL_SCANCODE_KP_3, KeyCode::KP_3);
            addKeyMapping(SDL_SCANCODE_KP_4, KeyCode::KP_4);
            addKeyMapping(SDL_SCANCODE_KP_5, KeyCode::KP_5);
            addKeyMapping(SDL_SCANCODE_KP_6, KeyCode::KP_6);
            addKeyMapping(SDL_SCANCODE_KP_7, KeyCode::KP_7);
            addKeyMapping(SDL_SCANCODE_KP_8, KeyCode::KP_8);
            addKeyMapping(SDL_SCANCODE_KP_9, KeyCode::KP_9);
            addKeyMapping(SDL_SCANCODE_KP_DECIMAL, KeyCode::KP_DECIMAL);
            addKeyMapping(SDL_SCANCODE_KP_DIVIDE, KeyCode::KP_DIVIDE);
            addKeyMapping(SDL_SCANCODE_KP_MULTIPLY, KeyCode::KP_MULTIPLY);
            addKeyMapping(SDL_SCANCODE_KP_MINUS, KeyCode::KP_SUBTRACT);
            addKeyMapping(SDL_SCANCODE_KP_PLUS, KeyCode::KP_ADD);
            addKeyMapping(SDL_SCANCODE_KP_ENTER, KeyCode::KP_ENTER);
            addKeyMapping(SDL_SCANCODE_KP_EQUALS, KeyCode::KP_EQUAL);

            // Special
            addKeyMapping(SDL_SCANCODE_APPLICATION, KeyCode::APPLICATION);
        }

        /**
         * @brief Initialize gamepad button mappings
         */
        void initializeButtonMap() {
            addButtonMapping(SDL_CONTROLLER_BUTTON_A, GamepadButton::A);
            addButtonMapping(SDL_CONTROLLER_BUTTON_B, GamepadButton::B);
            addButtonMapping(SDL_CONTROLLER_BUTTON_X, GamepadButton::X);
            addButtonMapping(SDL_CONTROLLER_BUTTON_Y, GamepadButton::Y);
            addButtonMapping(SDL_CONTROLLER_BUTTON_BACK, GamepadButton::BACK);
            addButtonMapping(SDL_CONTROLLER_BUTTON_GUIDE, GamepadButton::GUIDE);
            addButtonMapping(SDL_CONTROLLER_BUTTON_START, GamepadButton::START);
            addButtonMapping(SDL_CONTROLLER_BUTTON_LEFTSTICK, GamepadButton::LEFT_STICK);
            addButtonMapping(SDL_CONTROLLER_BUTTON_RIGHTSTICK, GamepadButton::RIGHT_STICK);
            addButtonMapping(SDL_CONTROLLER_BUTTON_LEFTSHOULDER, GamepadButton::LEFT_BUMPER);
            addButtonMapping(SDL_CONTROLLER_BUTTON_RIGHTSHOULDER, GamepadButton::RIGHT_BUMPER);
            addButtonMapping(SDL_CONTROLLER_BUTTON_DPAD_UP, GamepadButton::DPAD_UP);
            addButtonMapping(SDL_CONTROLLER_BUTTON_DPAD_DOWN, GamepadButton::DPAD_DOWN);
            addButtonMapping(SDL_CONTROLLER_BUTTON_DPAD_LEFT, GamepadButton::DPAD_LEFT);
            addButtonMapping(SDL_CONTROLLER_BUTTON_DPAD_RIGHT, GamepadButton::DPAD_RIGHT);

            // Extended buttons with version check
#if SDL_VERSION_ATLEAST(2, 0, 14)
            addButtonMapping(SDL_CONTROLLER_BUTTON_MISC1, GamepadButton::TOUCHPAD);
            addButtonMapping(SDL_CONTROLLER_BUTTON_PADDLE1, GamepadButton::PADDLE_1);
            addButtonMapping(SDL_CONTROLLER_BUTTON_PADDLE2, GamepadButton::PADDLE_2);
            addButtonMapping(SDL_CONTROLLER_BUTTON_PADDLE3, GamepadButton::PADDLE_3);
            addButtonMapping(SDL_CONTROLLER_BUTTON_PADDLE4, GamepadButton::PADDLE_4);
#endif

#if SDL_VERSION_ATLEAST(2, 0, 16)
            addButtonMapping(SDL_CONTROLLER_BUTTON_TOUCHPAD, GamepadButton::TOUCHPAD);
#endif
        }

        /**
         * @brief Initialize gamepad axis mappings
         */
        void initializeAxisMap() {
            addAxisMapping(SDL_CONTROLLER_AXIS_LEFTX, GamepadAxis::LEFT_STICK_X);
            addAxisMapping(SDL_CONTROLLER_AXIS_LEFTY, GamepadAxis::LEFT_STICK_Y);
            addAxisMapping(SDL_CONTROLLER_AXIS_RIGHTX, GamepadAxis::RIGHT_STICK_X);
            addAxisMapping(SDL_CONTROLLER_AXIS_RIGHTY, GamepadAxis::RIGHT_STICK_Y);
            addAxisMapping(SDL_CONTROLLER_AXIS_TRIGGERLEFT, GamepadAxis::LEFT_TRIGGER);
            addAxisMapping(SDL_CONTROLLER_AXIS_TRIGGERRIGHT, GamepadAxis::RIGHT_TRIGGER);
        }

        /**
         * @brief Add bidirectional key mapping
         */
        void addKeyMapping(const SDL_Scancode sdl, const KeyCode engine) {
            sdlToEngineMap_[sdl] = engine;
            engineToSDLMap_[engine] = sdl;
        }

        /**
         * @brief Add bidirectional button mapping
         */
        void addButtonMapping(const SDL_GameControllerButton sdl, const GamepadButton engine) {
            sdlButtonMap_[sdl] = engine;
            engineButtonMap_[engine] = sdl;
        }

        /**
         * @brief Add bidirectional axis mapping
         */
        void addAxisMapping(const SDL_GameControllerAxis sdl, const GamepadAxis engine) {
            sdlAxisMap_[sdl] = engine;
            engineAxisMap_[engine] = sdl;
        }
    };

    /**
     * @brief Factory function to create a key map with default mappings
     * @return Unique pointer to a new SDLKeyMap instance
     */
    [[nodiscard]] inline std::unique_ptr<SDLKeyMap> createDefaultKeyMap() {
        return std::make_unique<SDLKeyMap>();
    }

    /**
     * @brief Shared key map for systems that need shared access
     * @return Shared pointer to a new SDLKeyMap instance
     */
    [[nodiscard]] inline std::shared_ptr<SDLKeyMap> createSharedKeyMap() {
        return std::make_shared<SDLKeyMap>();
    }

    /**
     * @brief Convert SDL mouse button to engine mouse button
     * @param button SDL mouse button
     * @return Engine mouse button
     */
    MouseButton sdlToEngineMouseButton(Uint8 button) noexcept;

    /**
     * @brief Convert engine mouse button to SDL mouse button
     * @param button Engine mouse button
     * @return SDL mouse button
     */
    Uint8 engineToSDLMouseButton(MouseButton button) noexcept;

    /**
     * @brief Check if SDL event is a text input event
     * @param event SDL event
     * @return True if text input event
     */
    bool isTextInputEvent(SDL_Event& event) noexcept;

    /**
     * @brief Get normalized axis value from SDL
     * @param value Raw SDL axis value
     * @return Normalized value (-1 to 1)
     */
    float normalizeAxisValue(Sint16 value) noexcept;

    /**
     * @brief Get normalized trigger value from SDL
     * @param value Raw SDL trigger value
     * @return Normalized value (0 to 1)
     */
    float normalizeTriggerValue(Sint16 value) noexcept;

    /**
     * @brief Convert SDL hat position to D-pad buttons
     * @param hat SDL hat value
     * @param up Output: up button pressed
     * @param down Output: down button pressed
     * @param left Output: left button pressed
     * @param right Output: right button pressed
     */
    void sdlHatToDPad(Uint8 hat, bool& up, bool& down, bool& left, bool& right) noexcept;

    /**
     * @brief Get SDL joystick GUID as string
     * @param joystick SDL joystick
     * @return GUID string
     */
    std::string getJoystickGUID(SDL_Joystick* joystick);

    /**
     * @brief Get SDL game controller GUID as string
     * @param controller SDL game controller
     * @return GUID string
     */
    std::string getControllerGUID(SDL_GameController* controller);

    /**
     * @brief Check if joystick is a game controller
     * @param joystickIndex Joystick index
     * @return True if game controller
     */
    bool isGameController(int joystickIndex) noexcept;

    /**
     * @brief Get controller battery level
     * @param controller SDL game controller
     * @return Battery level (0.0 to 1.0, or -1.0 if unknown)
     */
    float getControllerBatteryLevel([[maybe_unused]] SDL_GameController* controller) noexcept;

    /**
     * @brief Set controller LED color (PS4/PS5)
     * @param controller SDL game controller
     * @param r Red component (0-255)
     * @param g Green component (0-255)
     * @param b Blue component (0-255)
     * @return True if successful
     */
    bool setControllerLED([[maybe_unused]] SDL_GameController* controller,
                          [[maybe_unused]] Uint8 r,
                          [[maybe_unused]] Uint8 g,
                          [[maybe_unused]] Uint8 b) noexcept;

    /**
     * @brief Trigger haptic pulse on controller
     * @param controller SDL game controller
     * @param leftMotor Left motor intensity (0-65535)
     * @param rightMotor Right motor intensity (0-65535)
     * @param duration Duration in milliseconds
     * @return True if successful
     */
    bool triggerControllerRumble(SDL_GameController* controller,
                                 Uint16 leftMotor,
                                 Uint16 rightMotor,
                                 Uint32 duration) noexcept;

    /**
     * @brief Stop controller rumble
     * @param controller SDL game controller
     * @return True if successful
     */
    bool stopControllerRumble([[maybe_unused]] SDL_GameController* controller) noexcept;

    /**
     * @brief Get touchpad state (PS4/PS5)
     * @param controller SDL game controller
     * @param touchpadIndex Touchpad index (usually 0)
     * @param fingerIndex Finger index (0 or 1)
     * @param x Output: X coordinate (0.0 to 1.0)
     * @param y Output: Y coordinate (0.0 to 1.0)
     * @param pressure Output: Pressure (0.0 to 1.0)
     * @return True if touchpad data available
     */
    bool getControllerTouchpad([[maybe_unused]] SDL_GameController* controller,
                               [[maybe_unused]] int touchpadIndex,
                               [[maybe_unused]] int fingerIndex,
                               [[maybe_unused]] float& x,
                               [[maybe_unused]] float& y,
                               [[maybe_unused]] float& pressure) noexcept;

    /**
     * @brief Get gyroscope data from controller
     * @param controller SDL game controller
     * @param pitch Output: Pitch in degrees per second
     * @param yaw Output: Yaw in degrees per second
     * @param roll Output: Roll in degrees per second
     * @return True if gyro data available
     */
    bool getControllerGyro([[maybe_unused]] SDL_GameController* controller,
                           [[maybe_unused]] float& pitch,
                           [[maybe_unused]] float& yaw,
                           [[maybe_unused]] float& roll) noexcept;

    /**
     * @brief Get accelerometer data from controller
     * @param controller SDL game controller
     * @param x Output: X acceleration
     * @param y Output: Y acceleration
     * @param z Output: Z acceleration
     * @return True if accelerometer data available
     */
    bool getControllerAccelerometer([[maybe_unused]] SDL_GameController* controller,
                                    [[maybe_unused]] float& x,
                                    [[maybe_unused]] float& y,
                                    [[maybe_unused]] float& z) noexcept;
} // namespace engine::input
