/**
 * @file InputConstants.h
 * @brief Compile-time constants for the input system
 * @author Game Engine Team
 * @date 2024
 *
 * Central location for all input system constants to avoid magic numbers
 * and ensure consistency across the codebase.
 */

#pragma once

#include <cstdint>
#include <chrono>

namespace engine::input {
    // ============================================================================
    // System Limits
    // ============================================================================

    // Maximum supported devices
    constexpr std::uint32_t MAX_INPUT_DEVICES = 32;
    constexpr std::uint32_t MAX_KEYBOARDS = 1;
    constexpr std::uint32_t MAX_MICE = 1;
    constexpr std::uint32_t MAX_GAMEPADS = 4;
    constexpr std::uint32_t MAX_TOUCH_DEVICES = 2;
    constexpr std::uint32_t MAX_CUSTOM_DEVICES = 8;

    // Maximum inputs per device
    constexpr std::uint32_t MAX_KEYS = 512;
    constexpr std::uint32_t MAX_MOUSE_BUTTONS = 8;
    constexpr std::uint32_t MAX_GAMEPAD_BUTTONS = 32;
    constexpr std::uint32_t MAX_GAMEPAD_AXES = 8;
    constexpr std::uint32_t MAX_TOUCH_POINTS = 10;

    // Event limits
    constexpr std::uint32_t MAX_EVENTS_PER_FRAME = 256;
    constexpr std::uint32_t MAX_EVENTS_IN_QUEUE = 1024;
    constexpr std::uint32_t MAX_EVENT_DATA_SIZE = 64; // Bytes

    // Action mapping limits
    constexpr std::uint32_t MAX_ACTIONS = 256;
    constexpr std::uint32_t MAX_BINDINGS_PER_ACTION = 8;
    constexpr std::uint32_t MAX_CONTEXTS = 32;
    constexpr std::uint32_t MAX_BINDINGS_PER_CONTEXT = 128;

    // ============================================================================
    // Default Values
    // ============================================================================

    // Deadzone and thresholds
    constexpr float DEFAULT_STICK_DEADZONE = 0.15f;
    constexpr float DEFAULT_TRIGGER_THRESHOLD = 0.1f;
    constexpr float DEFAULT_MOUSE_WHEEL_SENSITIVITY = 1.0f;
    constexpr float DEFAULT_STICK_SENSITIVITY = 1.0f;

    // Timing values (in seconds)
    constexpr float DEFAULT_KEY_REPEAT_DELAY = 0.5f;
    constexpr float DEFAULT_KEY_REPEAT_RATE = 0.05f;
    constexpr float DEFAULT_DOUBLE_CLICK_TIME = 0.3f;
    constexpr float DEFAULT_TAP_TIME = 0.2f;
    constexpr float DEFAULT_HOLD_TIME = 0.5f;
    constexpr float DEFAULT_SWIPE_TIME = 0.5f;

    // Distance thresholds (in pixels)
    constexpr float DEFAULT_DOUBLE_CLICK_DISTANCE = 5.0f;
    constexpr float DEFAULT_TAP_DISTANCE = 10.0f;
    constexpr float DEFAULT_SWIPE_MIN_DISTANCE = 50.0f;
    constexpr float DEFAULT_DRAG_THRESHOLD = 5.0f;

    // Buffer settings
    constexpr std::uint32_t DEFAULT_INPUT_BUFFER_FRAMES = 6;
    constexpr std::uint32_t DEFAULT_HISTORY_FRAMES = 600; // 10 seconds at 60 FPS
    constexpr std::uint32_t DEFAULT_EVENT_POOL_SIZE = 1024;
    constexpr std::uint32_t DEFAULT_SNAPSHOT_POOL_SIZE = 4;

    // ============================================================================
    // Device ID Ranges
    // ============================================================================

    // Reserved device ID ranges for different device types
    constexpr std::uint32_t DEVICE_ID_KEYBOARD_BASE = 0x00000000;
    constexpr std::uint32_t DEVICE_ID_MOUSE_BASE = 0x00010000;
    constexpr std::uint32_t DEVICE_ID_GAMEPAD_BASE = 0x00020000;
    constexpr std::uint32_t DEVICE_ID_TOUCH_BASE = 0x00030000;
    constexpr std::uint32_t DEVICE_ID_CUSTOM_BASE = 0x00040000;
    constexpr std::uint32_t DEVICE_ID_INVALID = 0xFFFFFFFF;

    // Special device IDs
    constexpr std::uint32_t DEVICE_ID_ANY = 0xFFFFFFFE; // Matches any device
    constexpr std::uint32_t DEVICE_ID_SYSTEM_KEYBOARD = DEVICE_ID_KEYBOARD_BASE;
    constexpr std::uint32_t DEVICE_ID_SYSTEM_MOUSE = DEVICE_ID_MOUSE_BASE;

    // ============================================================================
    // Timing Constants
    // ============================================================================

    using InputClock = std::chrono::steady_clock;
    using InputTimePoint = InputClock::time_point;
    using InputDuration = std::chrono::duration<float>;

    constexpr std::chrono::milliseconds INPUT_TICK_RATE{16}; // ~60 FPS
    constexpr std::chrono::milliseconds INPUT_POLL_RATE{8}; // ~120 Hz polling

    // ============================================================================
    // Math Constants
    // ============================================================================

    constexpr float INPUT_PI = 3.14159265358979323846f;
    constexpr float INPUT_2PI = 6.28318530717958647692f;
    constexpr float INPUT_PI_2 = 1.57079632679489661923f;
    constexpr float INPUT_DEG_TO_RAD = INPUT_PI / 180.0f;
    constexpr float INPUT_RAD_TO_DEG = 180.0f / INPUT_PI;

    // Epsilon for float comparisons
    constexpr float INPUT_EPSILON = 0.0001f;
    constexpr float INPUT_EPSILON_SQUARED = INPUT_EPSILON * INPUT_EPSILON;

    // ============================================================================
    // Bit Flags
    // ============================================================================

    // Event flags
    constexpr std::uint32_t EVENT_FLAG_CONSUMED = 1 << 0;
    constexpr std::uint32_t EVENT_FLAG_SYNTHETIC = 1 << 1;
    constexpr std::uint32_t EVENT_FLAG_FROM_REPLAY = 1 << 2;
    constexpr std::uint32_t EVENT_FLAG_HIGH_PRIORITY = 1 << 3;
    constexpr std::uint32_t EVENT_FLAG_COALESCED = 1 << 4;

    // Device state flags
    constexpr std::uint32_t DEVICE_FLAG_CONNECTED = 1 << 0;
    constexpr std::uint32_t DEVICE_FLAG_ENABLED = 1 << 1;
    constexpr std::uint32_t DEVICE_FLAG_HAS_BATTERY = 1 << 2;
    constexpr std::uint32_t DEVICE_FLAG_IS_WIRELESS = 1 << 3;
    constexpr std::uint32_t DEVICE_FLAG_SUPPORTS_RUMBLE = 1 << 4;
    constexpr std::uint32_t DEVICE_FLAG_SUPPORTS_HAPTICS = 1 << 5;

    // ============================================================================
    // Error Codes
    // ============================================================================

    enum class InputErrorCode : std::int32_t {
        SUCCESS = 0,
        NOT_INITIALIZED = -1,
        ALREADY_INITIALIZED = -2,
        DEVICE_NOT_FOUND = -3,
        INVALID_PARAMETER = -4,
        OUT_OF_MEMORY = -5,
        QUEUE_FULL = -6,
        CONTEXT_NOT_FOUND = -7,
        ACTION_NOT_FOUND = -8,
        BINDING_CONFLICT = -9,
        DEVICE_DISCONNECTED = -10,
        NOT_SUPPORTED = -11,
        TIMEOUT = -12,
        PERMISSION_DENIED = -13,
        FILE_NOT_FOUND = -14,
        PARSE_ERROR = -15
    };

    /**
     * @brief Convert error code to string
     */
    inline const char* getErrorString(const InputErrorCode error) {
        switch (error) {
        case InputErrorCode::SUCCESS: return "Success";
        case InputErrorCode::NOT_INITIALIZED: return "Input system not initialized";
        case InputErrorCode::ALREADY_INITIALIZED: return "Input system already initialized";
        case InputErrorCode::DEVICE_NOT_FOUND: return "Device not found";
        case InputErrorCode::INVALID_PARAMETER: return "Invalid parameter";
        case InputErrorCode::OUT_OF_MEMORY: return "Out of memory";
        case InputErrorCode::QUEUE_FULL: return "Event queue full";
        case InputErrorCode::CONTEXT_NOT_FOUND: return "Context not found";
        case InputErrorCode::ACTION_NOT_FOUND: return "Action not found";
        case InputErrorCode::BINDING_CONFLICT: return "Binding conflict";
        case InputErrorCode::DEVICE_DISCONNECTED: return "Device disconnected";
        case InputErrorCode::NOT_SUPPORTED: return "Operation not supported";
        case InputErrorCode::TIMEOUT: return "Operation timed out";
        case InputErrorCode::PERMISSION_DENIED: return "Permission denied";
        case InputErrorCode::FILE_NOT_FOUND: return "File not found";
        case InputErrorCode::PARSE_ERROR: return "Parse error";
        default: return "Unknown error";
        }
    }

    // ============================================================================
    // String Constants
    // ============================================================================

    // Default context names
    constexpr auto CONTEXT_DEFAULT = "Default";
    constexpr auto CONTEXT_GAMEPLAY = "Gameplay";
    constexpr auto CONTEXT_UI_MENU = "UIMenu";
    constexpr auto CONTEXT_UI_DIALOG = "UIDialog";
    constexpr auto CONTEXT_TEXT_INPUT = "TextInput";
    constexpr auto CONTEXT_CUTSCENE = "Cutscene";
    constexpr auto CONTEXT_DEBUG = "Debug";
    constexpr auto CONTEXT_PAUSED = "Paused";

    // Configuration file names
    constexpr auto DEFAULT_INPUT_CONFIG_FILE = "input_config.json";
    constexpr auto DEFAULT_BINDINGS_FILE = "input_bindings.json";
    constexpr auto DEFAULT_GAMEPAD_DB_FILE = "gamecontrollerdb.txt";
} // namespace engine::input
