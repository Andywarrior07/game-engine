/**
 * @file SDLKeyMap.cpp
 * @brief SDL2 to engine key code mapping implementation
 * @author Andrés Guerrero
 * @date 13-09-2025
 *
 * Implementation file for SDLKeyMap class. Since the header contains
 * all inline implementations, this file provides any additional
 * functionality or explicit template instantiations if needed.
 */


#include "SDLKeyMap.h"

#include <SDL2/SDL.h>

namespace engine::input {
    // ============================================================================
    // Static Assertions for SDL Version
    // ============================================================================

    static_assert(SDL_VERSION_ATLEAST(2, 0, 0), "SDL 2.0 or higher required");

    // ============================================================================
    // Additional Helper Functions
    // ============================================================================

    /**
     * @brief Convert SDL mouse button to engine mouse button
     * @param button SDL mouse button
     * @return Engine mouse button
     */
    MouseButton sdlToEngineMouseButton(Uint8 button) noexcept {
        switch (button) {
        case SDL_BUTTON_LEFT: return MouseButton::LEFT;
        case SDL_BUTTON_MIDDLE: return MouseButton::MIDDLE;
        case SDL_BUTTON_RIGHT: return MouseButton::RIGHT;
        case SDL_BUTTON_X1: return MouseButton::THUMB_1;
        case SDL_BUTTON_X2: return MouseButton::THUMB_2;
        default:
            if (button >= 6 && button <= 8) {
                return static_cast<MouseButton>(button);
            }
            return MouseButton::NONE;
        }
    }

    /**
     * @brief Convert engine mouse button to SDL mouse button
     * @param button Engine mouse button
     * @return SDL mouse button
     */
    Uint8 engineToSDLMouseButton(MouseButton button) noexcept {
        switch (button) {
        case MouseButton::LEFT: return SDL_BUTTON_LEFT;
        case MouseButton::MIDDLE: return SDL_BUTTON_MIDDLE;
        case MouseButton::RIGHT: return SDL_BUTTON_RIGHT;
        case MouseButton::THUMB_1: return SDL_BUTTON_X1;
        case MouseButton::THUMB_2: return SDL_BUTTON_X2;
        default:
            if (button >= MouseButton::BUTTON_6 && button <= MouseButton::BUTTON_8) {
                return static_cast<Uint8>(button);
            }
            return 0;
        }
    }

    /**
     * @brief Check if SDL event is a text input event
     * @param event SDL event
     * @return True if text input event
     */
    bool isTextInputEvent(const SDL_Event& event) noexcept {
        return event.type == SDL_TEXTINPUT || event.type == SDL_TEXTEDITING;
    }

    /**
     * @brief Get normalized axis value from SDL
     * @param value Raw SDL axis value
     * @return Normalized value (-1 to 1)
     */
    float normalizeAxisValue(const Sint16 value) noexcept {
        // SDL axis range is -32768 to 32767

        if (value > 0) {
            constexpr float MAX_VALUE = 32767.0f;
            return static_cast<float>(value) / MAX_VALUE;
        }

        if (value < 0) {
            constexpr float MIN_VALUE = -32768.0f;
            return static_cast<float>(value) / -MIN_VALUE;
        }

        return 0.0f;
    }

    /**
     * @brief Get normalized trigger value from SDL
     * @param value Raw SDL trigger value
     * @return Normalized value (0 to 1)
     */
    float normalizeTriggerValue(const Sint16 value) noexcept {
        // SDL trigger range is 0 to 32767
        constexpr float MAX_VALUE = 32767.0f;
        return static_cast<float>(value) / MAX_VALUE;
    }

    /**
     * @brief Convert SDL hat position to D-pad buttons
     * @param hat SDL hat value
     * @param up Output: up button pressed
     * @param down Output: down button pressed
     * @param left Output: left button pressed
     * @param right Output: right button pressed
     */
    void sdlHatToDPad(const Uint8 hat, bool& up, bool& down, bool& left, bool& right) noexcept {
        up = false;
        down = false;
        left = false;
        right = false;

        if (hat & SDL_HAT_UP) {
            up = true;
        }
        if (hat & SDL_HAT_DOWN) {
            down = true;
        }
        if (hat & SDL_HAT_LEFT) {
            left = true;
        }
        if (hat & SDL_HAT_RIGHT) {
            right = true;
        }
    }

    /**
     * @brief Get SDL joystick GUID as string
     * @param joystick SDL joystick
     * @return GUID string
     */
    std::string getJoystickGUID(SDL_Joystick* joystick) {
        if (!joystick) {
            return "";
        }

        const SDL_JoystickGUID guid = SDL_JoystickGetGUID(joystick);
        char guidStr[33];
        SDL_JoystickGetGUIDString(guid, guidStr, sizeof(guidStr));
        return std::string(guidStr);
    }

    /**
     * @brief Get SDL game controller GUID as string
     * @param controller SDL game controller
     * @return GUID string
     */
    std::string getControllerGUID(SDL_GameController* controller) {
        if (!controller) {
            return "";
        }

        SDL_Joystick* joystick = SDL_GameControllerGetJoystick(controller);
        return getJoystickGUID(joystick);
    }

    /**
     * @brief Check if joystick is a game controller
     * @param joystickIndex Joystick index
     * @return True if game controller
     */
    bool isGameController(const int joystickIndex) noexcept {
        return SDL_IsGameController(joystickIndex) == SDL_TRUE;
    }

    /**
     * @brief Get controller battery level
     * @param controller SDL game controller
     * @return Battery level (0.0 to 1.0, or -1.0 if unknown)
     */
    float getControllerBatteryLevel([[maybe_unused]] SDL_GameController* controller) noexcept {
#if SDL_VERSION_ATLEAST(2, 0, 14)
        if (!controller) {
            return -1.0f;
        }

        SDL_Joystick* joystick = SDL_GameControllerGetJoystick(controller);
        if (!joystick) {
            return -1.0f;
        }

        const SDL_JoystickPowerLevel level = SDL_JoystickCurrentPowerLevel(joystick);

        switch (level) {
        case SDL_JOYSTICK_POWER_EMPTY:
            return 0.0f;
        case SDL_JOYSTICK_POWER_LOW:
            return 0.25f;
        case SDL_JOYSTICK_POWER_MEDIUM:
            return 0.5f;
        case SDL_JOYSTICK_POWER_FULL:
            return 1.0f;
        case SDL_JOYSTICK_POWER_WIRED:
            return 1.0f; // Wired, so "full"
        case SDL_JOYSTICK_POWER_MAX:
            return 1.0f;
        case SDL_JOYSTICK_POWER_UNKNOWN:
        default:
            return -1.0f;
        }
#else
        return -1.0f; // Battery level not available in older SDL versions
#endif
    }

    /**
     * @brief Set controller LED color (PS4/PS5)
     * @param controller SDL game controller
     * @param r Red component (0-255)
     * @param g Green component (0-255)
     * @param b Blue component (0-255)
     * @return True if successful
     */
    bool setControllerLED([[maybe_unused]] SDL_GameController* controller,
                          [[maybe_unused]] const Uint8 r,
                          [[maybe_unused]] const Uint8 g,
                          [[maybe_unused]] const Uint8 b) noexcept {
#if SDL_VERSION_ATLEAST(2, 0, 14)
        if (!controller) {
            return false;
        }

        SDL_Joystick* joystick = SDL_GameControllerGetJoystick(controller);
        if (!joystick) {
            return false;
        }

        return SDL_JoystickSetLED(joystick, r, g, b) == 0;
#else
        return false; // LED control not available in older SDL versions
#endif
    }

    /**
     * @brief Trigger haptic pulse on controller
     * @param controller SDL game controller
     * @param leftMotor Left motor intensity (0-65535)
     * @param rightMotor Right motor intensity (0-65535)
     * @param duration Duration in milliseconds
     * @return True if successful
     */
    bool triggerControllerRumble(SDL_GameController* controller,
                                 const Uint16 leftMotor,
                                 const Uint16 rightMotor,
                                 const Uint32 duration) noexcept {
#if SDL_VERSION_ATLEAST(2, 0, 9)
        if (!controller) {
            return false;
        }

        return SDL_GameControllerRumble(controller, leftMotor, rightMotor, duration) == 0;
#else
        // Fallback to haptic API for older SDL versions
        if (!controller) {
            return false;
        }

        SDL_Joystick* joystick = SDL_GameControllerGetJoystick(controller);
        if (!joystick) {
            return false;
        }

        SDL_Haptic* haptic = SDL_HapticOpenFromJoystick(joystick);
        if (!haptic) {
            return false;
        }

        // Check if rumble is supported
        if (!(SDL_HapticQuery(haptic) & SDL_HAPTIC_RUMBLE)) {
            SDL_HapticClose(haptic);
            return false;
        }

        // Initialize rumble
        if (SDL_HapticRumbleInit(haptic) != 0) {
            SDL_HapticClose(haptic);
            return false;
        }

        // Calculate strength (0.0 to 1.0)
        float strength = ((float)leftMotor + (float)rightMotor) / (2.0f * 65535.0f);

        // Play rumble
        bool success = SDL_HapticRumblePlay(haptic, strength, duration) == 0;

        SDL_HapticClose(haptic);
        return success;
#endif
    }

    /**
     * @brief Stop controller rumble
     * @param controller SDL game controller
     * @return True if successful
     */
    bool stopControllerRumble([[maybe_unused]] SDL_GameController* controller) noexcept {
#if SDL_VERSION_ATLEAST(2, 0, 9)
        if (!controller) {
            return false;
        }

        return SDL_GameControllerRumble(controller, 0, 0, 0) == 0;
#else
        return false;
#endif
    }

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
                               [[maybe_unused]] const int touchpadIndex,
                               [[maybe_unused]] const int fingerIndex,
                               [[maybe_unused]] float& x,
                               [[maybe_unused]] float& y,
                               [[maybe_unused]] float& pressure) noexcept {
#if SDL_VERSION_ATLEAST(2, 0, 14)
        if (!controller) {
            return false;
        }

        Uint8 state;
        const int result = SDL_GameControllerGetTouchpadFinger(controller, touchpadIndex, fingerIndex,
                                                         &state, &x, &y, &pressure);
        return result == 0 && state != 0;
#else
        x = 0.0f;
        y = 0.0f;
        pressure = 0.0f;
        return false; // Touchpad not available in older SDL versions
#endif
    }

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
                           [[maybe_unused]] float& roll) noexcept {
#if SDL_VERSION_ATLEAST(2, 0, 14)
        if (!controller) {
            return false;
        }

        float data[3];

        if (const int result = SDL_GameControllerGetSensorData(controller, SDL_SENSOR_GYRO, data, 3); result == 0) {
            pitch = data[0];
            yaw = data[1];
            roll = data[2];
            return true;
        }
#endif
        pitch = 0.0f;
        yaw = 0.0f;
        roll = 0.0f;
        return false;
    }

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
                                    [[maybe_unused]] float& z) noexcept {
#if SDL_VERSION_ATLEAST(2, 0, 14)
        if (!controller) {
            return false;
        }

        float data[3];

        if (const int result = SDL_GameControllerGetSensorData(controller, SDL_SENSOR_ACCEL, data, 3); result == 0) {
            x = data[0];
            y = data[1];
            z = data[2];
            return true;
        }
#endif
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
        return false;
    }
} // namespace engine::input
