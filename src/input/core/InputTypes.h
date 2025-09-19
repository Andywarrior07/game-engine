/**
 * @file InputTypes.h
 * @brief Core type definitions for the input system
 * @author Andrés Guerrero
 * @date 10-09-2024
 *
 * Defines all fundamental input types, enums, and constants used throughout
 * the input system. Platform-agnostic definitions only.
 */

#pragma once

#include "../../math/core/MathTypes.h"

#include <chrono>

namespace engine::input {
    // ============================================================================
    // Forward Declarations
    // ============================================================================

    class InputDevice;
    class InputContext;
    struct InputEvent;
    struct InputSnapshot;

    // ============================================================================
    // Type Aliases
    // ============================================================================

    using DeviceID = std::uint32_t;
    using ActionID = std::uint32_t;
    using PlayerID = std::uint8_t;
    using InputTimestamp = std::chrono::steady_clock::time_point;
    using InputDuration = std::chrono::duration<float>;

    // ============================================================================
    // Constants
    // ============================================================================

    constexpr DeviceID INVALID_DEVICE_ID = 0;
    constexpr ActionID INVALID_ACTION_ID = 0;
    constexpr PlayerID MAX_PLAYERS = 4;
    constexpr std::size_t MAX_SIMULTANEOUS_KEYS = 16;
    // constexpr std::size_t MAX_TOUCH_POINTS = 10;
    constexpr float ANALOG_DEADZONE_DEFAULT = 0.15f;
    constexpr float TRIGGER_THRESHOLD_DEFAULT = 0.1f;
    constexpr float MOUSE_SENSITIVITY_DEFAULT = 1.0f;

    // ============================================================================
    // Device Types
    // ============================================================================

    /**
     * @brief Types of input devices supported by the system
     */
    enum class DeviceType : std::uint8_t {
        NONE = 0,
        KEYBOARD,
        MOUSE,
        GAMEPAD,
        TOUCH,
        JOYSTICK,
        CUSTOM
    };

    /**
     * @brief Device capability flags
     */
    enum class DeviceCapability : std::uint32_t {
        NONE = 0,
        ANALOG_STICKS = 1 << 0,
        ANALOG_TRIGGERS = 1 << 1,
        RUMBLE = 1 << 2,
        GYROSCOPE = 1 << 3,
        ACCELEROMETER = 1 << 4,
        TOUCHPAD = 1 << 5,
        LIGHT_BAR = 1 << 6,
        BATTERY_STATUS = 1 << 7,
        HAPTIC_FEEDBACK = 1 << 8,
        PRESSURE_SENSITIVE = 1 << 9
    };

    inline DeviceCapability operator|(DeviceCapability a, DeviceCapability b) {
        return static_cast<DeviceCapability>(
            static_cast<std::uint32_t>(a) | static_cast<std::uint32_t>(b)
        );
    }

    inline DeviceCapability operator&(DeviceCapability a, DeviceCapability b) {
        return static_cast<DeviceCapability>(
            static_cast<std::uint32_t>(a) & static_cast<std::uint32_t>(b)
        );
    }

    inline bool hasCapability(const DeviceCapability caps, const DeviceCapability cap) {
        return (caps & cap) == cap;
    }

    // ============================================================================
    // Keyboard Input
    // ============================================================================

    /**
     * @brief Keyboard key codes (platform-independent)
     * Based on USB HID usage tables for consistency
     */
    enum class KeyCode : std::uint16_t {
        UNKNOWN = 0,

        // Letters
        A = 4, B, C, D, E, F, G, H, I, J, K, L, M,
        N, O, P, Q, R, S, T, U, V, W, X, Y, Z,

        // Numbers
        NUM_1 = 30, NUM_2, NUM_3, NUM_4, NUM_5,
        NUM_6, NUM_7, NUM_8, NUM_9, NUM_0,

        // Function keys
        F1 = 58, F2, F3, F4, F5, F6, F7, F8,
        F9, F10, F11, F12, F13, F14, F15, F16,
        F17, F18, F19, F20, F21, F22, F23, F24,

        // Control keys
        ESCAPE = 41,
        ENTER = 40,
        TAB = 43,
        BACKSPACE = 42,
        INSERT = 73,
        DELETE = 76,
        RIGHT = 79,
        LEFT = 80,
        DOWN = 81,
        UP = 82,
        PAGE_UP = 75,
        PAGE_DOWN = 78,
        HOME = 74,
        END = 77,

        // Modifiers
        CAPS_LOCK = 57,
        SCROLL_LOCK = 71,
        NUM_LOCK = 83,
        PRINT_SCREEN = 70,
        PAUSE = 72,

        LEFT_SHIFT = 225,
        LEFT_CTRL = 224,
        LEFT_ALT = 226,
        LEFT_SUPER = 227, // Windows/Cmd key
        RIGHT_SHIFT = 229,
        RIGHT_CTRL = 228,
        RIGHT_ALT = 230,
        RIGHT_SUPER = 231,

        // Symbols
        SPACE = 44,
        APOSTROPHE = 52,
        COMMA = 54,
        MINUS = 45,
        PERIOD = 55,
        SLASH = 56,
        SEMICOLON = 51,
        EQUAL = 46,
        LEFT_BRACKET = 47,
        BACKSLASH = 49,
        RIGHT_BRACKET = 48,
        GRAVE = 53,

        // Numpad
        KP_0 = 98, KP_1 = 89, KP_2 = 90, KP_3 = 91,
        KP_4 = 92, KP_5 = 93, KP_6 = 94, KP_7 = 95,
        KP_8 = 96, KP_9 = 97,
        KP_DECIMAL = 99,
        KP_DIVIDE = 84,
        KP_MULTIPLY = 85,
        KP_SUBTRACT = 86,
        KP_ADD = 87,
        KP_ENTER = 88,
        KP_EQUAL = 103,

        // Special
        APPLICATION = 101, // Context menu key

        KEY_COUNT = 256
    };

    /**
     * @brief Keyboard modifier state flags
     */
    enum class KeyModifier : std::uint8_t {
        NONE = 0,
        LEFT_SHIFT = 1 << 0,
        RIGHT_SHIFT = 1 << 1,
        LEFT_CTRL = 1 << 2,
        RIGHT_CTRL = 1 << 3,
        LEFT_ALT = 1 << 4,
        RIGHT_ALT = 1 << 5,
        LEFT_SUPER = 1 << 6,
        RIGHT_SUPER = 1 << 7,

        SHIFT = LEFT_SHIFT | RIGHT_SHIFT,
        CTRL = LEFT_CTRL | RIGHT_CTRL,
        ALT = LEFT_ALT | RIGHT_ALT,
        SUPER = LEFT_SUPER | RIGHT_SUPER
    };

    inline KeyModifier operator|(KeyModifier a, KeyModifier b) {
        return static_cast<KeyModifier>(
            static_cast<std::uint8_t>(a) | static_cast<std::uint8_t>(b)
        );
    }

    inline KeyModifier operator&(KeyModifier a, KeyModifier b) {
        return static_cast<KeyModifier>(
            static_cast<std::uint8_t>(a) & static_cast<std::uint8_t>(b)
        );
    }

    inline bool hasModifier(const KeyModifier mods, const KeyModifier mod) {
        return (mods & mod) == mod;
    }

    // ============================================================================
    // Mouse Input
    // ============================================================================

    /**
     * @brief Mouse button identifiers
     */
    enum class MouseButton : std::uint8_t {
        NONE = 0,
        LEFT = 1,
        MIDDLE = 2,
        RIGHT = 3,
        THUMB_1 = 4, // Back
        THUMB_2 = 5, // Forward
        BUTTON_6 = 6,
        BUTTON_7 = 7,
        BUTTON_8 = 8,

        BUTTON_COUNT = 9
    };

    /**
     * @brief Mouse cursor modes
     */
    enum class CursorMode : std::uint8_t {
        NORMAL = 0, // Visible and free
        HIDDEN, // Hidden but free
        DISABLED, // Hidden and locked (FPS mode)
        CONFINED // Visible but confined to window
    };

    // ============================================================================
    // Gamepad Input
    // ============================================================================

    /**
     * @brief Gamepad button identifiers (Xbox layout reference)
     */
    enum class GamepadButton : std::uint16_t {
        NONE = 0,

        // Face buttons
        A = 1, // Cross on PlayStation
        B = 2, // Circle on PlayStation
        X = 3, // Square on PlayStation
        Y = 4, // Triangle on PlayStation

        // Shoulders
        LEFT_BUMPER = 5, // L1
        RIGHT_BUMPER = 6, // R1

        // Special buttons
        BACK = 7, // Select/Share
        START = 8, // Options
        GUIDE = 9, // Xbox/PS button

        // Stick clicks
        LEFT_STICK = 10, // L3
        RIGHT_STICK = 11, // R3

        // D-Pad
        DPAD_UP = 12,
        DPAD_DOWN = 13,
        DPAD_LEFT = 14,
        DPAD_RIGHT = 15,

        // Extended buttons (for special controllers)
        TOUCHPAD = 16,
        PADDLE_1 = 17,
        PADDLE_2 = 18,
        PADDLE_3 = 19,
        PADDLE_4 = 20,

        BUTTON_COUNT = 21
    };

    /**
     * @brief Gamepad axis identifiers
     */
    enum class GamepadAxis : std::uint8_t {
        NONE = 0,
        LEFT_STICK_X = 1,
        LEFT_STICK_Y = 2,
        RIGHT_STICK_X = 3,
        RIGHT_STICK_Y = 4,
        LEFT_TRIGGER = 5, // L2
        RIGHT_TRIGGER = 6, // R2

        AXIS_COUNT = 7
    };

    /**
     * @brief Gamepad rumble motors
     */
    enum class RumbleMotor : std::uint8_t {
        NONE = 0,
        LEFT = 1, // Low frequency
        RIGHT = 2, // High frequency
        BOTH = 3,
        TRIGGERS = 4 // For adaptive triggers (PS5)
    };

    // ============================================================================
    // Touch Input
    // ============================================================================

    /**
     * @brief Touch phase for gesture recognition
     */
    enum class TouchPhase : std::uint8_t {
        BEGAN = 0,
        MOVED,
        STATIONARY,
        ENDED,
        CANCELLED
    };

    /**
     * @brief Touch gesture types
     */
    enum class GestureType : std::uint8_t {
        NONE = 0,
        TAP,
        DOUBLE_TAP,
        LONG_PRESS,
        SWIPE,
        PINCH,
        ROTATE,
        PAN
    };

    // ============================================================================
    // Input Events
    // ============================================================================

    /**
     * @brief Types of input events
     */
    enum class InputEventType : std::uint8_t {
        NONE = 0,

        // Keyboard
        KEY_PRESSED,
        KEY_RELEASED,
        KEY_REPEAT,
        TEXT_INPUT,

        // Mouse
        MOUSE_MOVED,
        MOUSE_BUTTON_PRESSED,
        MOUSE_BUTTON_RELEASED,
        MOUSE_WHEEL,
        MOUSE_ENTERED,
        MOUSE_LEFT,

        // Gamepad
        GAMEPAD_CONNECTED,
        GAMEPAD_DISCONNECTED,
        GAMEPAD_BUTTON_PRESSED,
        GAMEPAD_BUTTON_RELEASED,
        GAMEPAD_AXIS_MOVED,
        GAMEPAD_TRIGGER_MOVED,

        // Touch
        TOUCH_BEGAN,
        TOUCH_MOVED,
        TOUCH_ENDED,
        TOUCH_CANCELLED,
        GESTURE_RECOGNIZED,

        // System
        FOCUS_GAINED,
        FOCUS_LOST,
        DEVICE_ADDED,
        DEVICE_REMOVED,
        CONTEXT_CHANGED
    };

    /**
     * @brief Input state for binary inputs
     */
    enum class InputState : std::uint8_t {
        NONE = 0,
        PRESSED = 1, // Just pressed this frame
        HELD = 2, // Held for multiple frames
        RELEASED = 3 // Just released this frame
    };

    /**
     * @brief Input context priority levels
     */
    enum class ContextPriority : std::uint8_t {
        LOWEST = 0,
        LOW = 50,
        NORMAL = 100,
        HIGH = 150,
        HIGHEST = 200,
        SYSTEM = 255 // Cannot be overridden
    };

    // ============================================================================
    // Action Mapping
    // ============================================================================

    /**
     * @brief Type of action (binary vs analog)
     */
    enum class ActionType : std::uint8_t {
        BUTTON = 0, // Binary on/off
        AXIS_1D, // Single axis (-1 to 1)
        AXIS_2D, // Two axes (Vector2)
        AXIS_3D // Three axes (Vector3)
    };

    /**
     * @brief Action trigger conditions
     */
    enum class TriggerEvent : std::uint8_t {
        NONE = 0,
        STARTED, // Transition from not triggered to triggered
        ONGOING, // Remains triggered
        COMPLETED, // Transition from triggered to not triggered
        CANCELLED // Action was cancelled
    };

    /**
     * @brief Modifier behavior for actions
     */
    enum class ModifierBehavior : std::uint8_t {
        NONE = 0,
        NEGATE, // Invert the value
        SCALE, // Scale by a factor
        DEADZONE, // Apply deadzone
        CLAMP, // Clamp to range
        SMOOTH // Apply smoothing
    };

    // ============================================================================
    // Utility Structures
    // ============================================================================

    /**
     * @brief Input range for analog values
     */
    struct InputRange {
        float min;
        float max;

        constexpr InputRange() noexcept : min(-1.0f), max(1.0f) {
        }

        constexpr InputRange(const float min, const float max) noexcept : min(min), max(max) {
        }

        [[nodiscard]] float clamp(const float value) const noexcept {
            if (value < min) return min;
            if (value > max) return max;
            return value;
        }

        [[nodiscard]] float normalize(const float value) const noexcept {
            if (const float range = max - min; range > 0.0001f) {
                return (value - min) / range;
            }

            return 0.0f;
        }

        [[nodiscard]] float remap(const float value, const InputRange& target) const noexcept {
            const float normalized = normalize(value);

            return target.min + normalized * (target.max - target.min);
        }
    };

    /**
     * @brief Deadzone configuration
     */
    struct DeadzoneSettings {
        float innerDeadzone; // Below this, input is 0
        float outerDeadzone; // Above this, input is 1

        constexpr DeadzoneSettings() noexcept
            : innerDeadzone(0.15f), outerDeadzone(0.95f) {
        }

        constexpr DeadzoneSettings(const float inner, const float outer) noexcept
            : innerDeadzone(inner), outerDeadzone(outer) {
        }

        [[nodiscard]] float apply(const float value) const noexcept {
            const float absValue = std::abs(value);

            if (absValue < innerDeadzone) {
                return 0.0f;
            }

            if (absValue > outerDeadzone) {
                return value > 0 ? 1.0f : -1.0f;
            }

            // Remap from deadzone range to 0-1
            const float remapped = (absValue - innerDeadzone) / (outerDeadzone - innerDeadzone);

            return remapped * (value > 0 ? 1.0f : -1.0f);
        }

        [[nodiscard]] math::Vec2 apply(const math::Vec2& stick) const noexcept {
            const float magnitude = glm::length(stick);

            if (magnitude < innerDeadzone) {
                return math::Vec2(0.0f, 0.0f);
            }

            if (magnitude > outerDeadzone) {
                return glm::normalize(stick);
            }

            const float remapped = (magnitude - innerDeadzone) / (outerDeadzone - innerDeadzone);

            return glm::normalize(stick) * remapped;
        }
    };

    /**
     * @brief Response curve for analog inputs
     */
    enum class ResponseCurve : std::uint8_t {
        LINEAR = 0,
        QUADRATIC,
        CUBIC,
        EXPONENTIAL,
        LOGARITHMIC,
        CUSTOM
    };

    /**
     * @brief Smoothing filter type
     */
    enum class FilterType : std::uint8_t {
        NONE = 0,
        AVERAGE, // Moving average
        EXPONENTIAL, // Exponential smoothing
        KALMAN, // Kalman filter
        ONE_EURO // 1€ filter (good for cursor/aim)
    };

    /**
     * @brief Input validation flags
     */
    enum class ValidationFlag : std::uint16_t {
        NONE = 0,
        REQUIRE_FOCUS = 1 << 0,
        BLOCK_IN_MENU = 1 << 1,
        BLOCK_IN_CUTSCENE = 1 << 2,
        BLOCK_WHEN_TYPING = 1 << 3,
        CONSUME_INPUT = 1 << 4,
        ALLOW_REMOTE = 1 << 5,
        REQUIRE_EXCLUSIVE = 1 << 6
    };

    inline ValidationFlag operator|(ValidationFlag a, ValidationFlag b) {
        return static_cast<ValidationFlag>(
            static_cast<std::uint16_t>(a) | static_cast<std::uint16_t>(b)
        );
    }

    inline ValidationFlag operator&(ValidationFlag a, ValidationFlag b) {
        return static_cast<ValidationFlag>(
            static_cast<std::uint16_t>(a) & static_cast<std::uint16_t>(b)
        );
    }

    // ============================================================================
    // String Conversion Utilities
    // ============================================================================

    inline const char* deviceTypeToString(const DeviceType type) {
        switch (type) {
        case DeviceType::NONE: return "None";
        case DeviceType::KEYBOARD: return "Keyboard";
        case DeviceType::MOUSE: return "Mouse";
        case DeviceType::GAMEPAD: return "Gamepad";
        case DeviceType::TOUCH: return "Touch";
        case DeviceType::JOYSTICK: return "Joystick";
        case DeviceType::CUSTOM: return "Custom";
        default: return "Unknown";
        }
    }

    inline const char* keyCodeToString(const KeyCode key) {
        switch (key) {
        case KeyCode::A: return "A";
        case KeyCode::B: return "B";
        case KeyCode::C: return "C";
        case KeyCode::D: return "D";
        case KeyCode::E: return "E";
        case KeyCode::F: return "F";
        case KeyCode::G: return "G";
        case KeyCode::H: return "H";
        case KeyCode::I: return "I";
        case KeyCode::J: return "J";
        case KeyCode::K: return "K";
        case KeyCode::L: return "L";
        case KeyCode::M: return "M";
        case KeyCode::N: return "N";
        case KeyCode::O: return "O";
        case KeyCode::P: return "P";
        case KeyCode::Q: return "Q";
        case KeyCode::R: return "R";
        case KeyCode::S: return "S";
        case KeyCode::T: return "T";
        case KeyCode::U: return "U";
        case KeyCode::V: return "V";
        case KeyCode::W: return "W";
        case KeyCode::X: return "X";
        case KeyCode::Y: return "Y";
        case KeyCode::Z: return "Z";
        case KeyCode::NUM_0: return "0";
        case KeyCode::NUM_1: return "1";
        case KeyCode::NUM_2: return "2";
        case KeyCode::NUM_3: return "3";
        case KeyCode::NUM_4: return "4";
        case KeyCode::NUM_5: return "5";
        case KeyCode::NUM_6: return "6";
        case KeyCode::NUM_7: return "7";
        case KeyCode::NUM_8: return "8";
        case KeyCode::NUM_9: return "9";
        case KeyCode::SPACE: return "Space";
        case KeyCode::ENTER: return "Enter";
        case KeyCode::ESCAPE: return "Escape";
        case KeyCode::TAB: return "Tab";
        case KeyCode::BACKSPACE: return "Backspace";
        case KeyCode::LEFT_SHIFT: return "Left Shift";
        case KeyCode::RIGHT_SHIFT: return "Right Shift";
        case KeyCode::LEFT_CTRL: return "Left Ctrl";
        case KeyCode::RIGHT_CTRL: return "Right Ctrl";
        case KeyCode::LEFT_ALT: return "Left Alt";
        case KeyCode::RIGHT_ALT: return "Right Alt";
        case KeyCode::UP: return "Up Arrow";
        case KeyCode::DOWN: return "Down Arrow";
        case KeyCode::LEFT: return "Left Arrow";
        case KeyCode::RIGHT: return "Right Arrow";
        case KeyCode::F1: return "F1";
        case KeyCode::F2: return "F2";
        case KeyCode::F3: return "F3";
        case KeyCode::F4: return "F4";
        case KeyCode::F5: return "F5";
        case KeyCode::F6: return "F6";
        case KeyCode::F7: return "F7";
        case KeyCode::F8: return "F8";
        case KeyCode::F9: return "F9";
        case KeyCode::F10: return "F10";
        case KeyCode::F11: return "F11";
        case KeyCode::F12: return "F12";
        default: return "Unknown";
        }
    }

    inline const char* mouseButtonToString(const MouseButton button) {
        switch (button) {
        case MouseButton::LEFT: return "Left Mouse";
        case MouseButton::MIDDLE: return "Middle Mouse";
        case MouseButton::RIGHT: return "Right Mouse";
        case MouseButton::THUMB_1: return "Mouse Back";
        case MouseButton::THUMB_2: return "Mouse Forward";
        default: return "Unknown Mouse Button";
        }
    }

    inline const char* gamepadButtonToString(const GamepadButton button) {
        switch (button) {
        case GamepadButton::A: return "A/Cross";
        case GamepadButton::B: return "B/Circle";
        case GamepadButton::X: return "X/Square";
        case GamepadButton::Y: return "Y/Triangle";
        case GamepadButton::LEFT_BUMPER: return "Left Bumper";
        case GamepadButton::RIGHT_BUMPER: return "Right Bumper";
        case GamepadButton::BACK: return "Back/Select";
        case GamepadButton::START: return "Start";
        case GamepadButton::GUIDE: return "Guide";
        case GamepadButton::LEFT_STICK: return "Left Stick Click";
        case GamepadButton::RIGHT_STICK: return "Right Stick Click";
        case GamepadButton::DPAD_UP: return "D-Pad Up";
        case GamepadButton::DPAD_DOWN: return "D-Pad Down";
        case GamepadButton::DPAD_LEFT: return "D-Pad Left";
        case GamepadButton::DPAD_RIGHT: return "D-Pad Right";
        default: return "Unknown Gamepad Button";
        }
    }
} // namespace engine::input
