//
// Created by Andres Guerrero on 28-07-25.
//

#pragma once

#include <SDL.h>                    // SDL core functionality
#include <functional>               // For std::function callbacks
#include <unordered_map>           // For fast hash-based lookups
#include <unordered_set>           // For action state tracking
#include <vector>                  // For dynamic arrays
#include <string>                  // For configuration and debug
#include <chrono>                  // For timing and input buffering
#include <memory>                  // For smart pointers
#include <optional>                // For optional return values (C++17)
#include <atomic>                  // For thread-safe operations
#include <mutex>                   // For thread synchronization
#include <array>                   // For fixed-size arrays

namespace engine::input {

    // Forward declarations
    class InputManager;
    struct InputBinding;

    // Strong typing for input identifiers - prevents mixing different ID types
    using ActionID = std::uint32_t;
    using AxisID = std::uint32_t;

    // Invalid ID constants for error checking
    constexpr ActionID INVALID_ACTION_ID = 0;
    constexpr AxisID INVALID_AXIS_ID = 0;

    /**
     * @brief Input device types supported by the system
     */
    enum class InputDevice : std::uint8_t {
        KEYBOARD = 0,           // Standard keyboard input
        MOUSE = 1,              // Mouse buttons and movement
        GAMEPAD = 2,            // Game controller (Xbox/PlayStation style)
        JOYSTICK = 3            // Generic joystick (for specialized controllers)
    };

    /**
     * @brief Input event types for fine-grained control
     */
    enum class InputEventType : std::uint8_t {
        PRESSED = 0,            // Button was just pressed this frame
        RELEASED = 1,           // Button was just released this frame
        HELD = 2,               // Button is being held down
        REPEAT = 3              // Key repeat event (for text input)
    };

    /**
     * @brief Mouse button mapping (extends SDL mouse buttons)
     */
    enum class MouseButton : std::uint8_t {
        LEFT = SDL_BUTTON_LEFT,
        MIDDLE = SDL_BUTTON_MIDDLE,
        RIGHT = SDL_BUTTON_RIGHT,
        X1 = SDL_BUTTON_X1,         // Extra mouse button 1
        X2 = SDL_BUTTON_X2          // Extra mouse button 2
    };

    /**
     * @brief Gamepad axis mapping with semantic names
     */
    enum class GamepadAxis : std::uint8_t {
        LEFT_STICK_X = SDL_CONTROLLER_AXIS_LEFTX,
        LEFT_STICK_Y = SDL_CONTROLLER_AXIS_LEFTY,
        RIGHT_STICK_X = SDL_CONTROLLER_AXIS_RIGHTX,
        RIGHT_STICK_Y = SDL_CONTROLLER_AXIS_RIGHTY,
        LEFT_TRIGGER = SDL_CONTROLLER_AXIS_TRIGGERLEFT,
        RIGHT_TRIGGER = SDL_CONTROLLER_AXIS_TRIGGERRIGHT
    };

    /**
     * @brief Configuration for axis behavior and sensitivity
     */
    struct AxisConfig {
        float deadZone = 0.15f;         // Dead zone threshold (0.0-1.0)
        float sensitivity = 1.0f;       // Input sensitivity multiplier
        bool invertAxis = false;        // Whether to invert the axis
        float smoothing = 0.0f;         // Input smoothing factor (0.0-1.0)
        float snapThreshold = 0.8f;     // Threshold for digital snap behavior
    };

    /**
     * @brief Input binding configuration for flexible key mapping
     */
    struct InputBinding {
        InputDevice device;                     // Which device this binding applies to
        std::uint32_t inputCode;               // Device-specific input code
        float weight = 1.0f;                   // Contribution weight for composite inputs
        bool isInverted = false;               // Whether to invert this input

        // Constructor for easy initialization
        InputBinding(InputDevice dev, std::uint32_t code, float w = 1.0f, bool inv = false)
            : device(dev), inputCode(code), weight(w), isInverted(inv) {}
    };

    /**
     * @brief Callback function types for input events
     */
    using ActionCallback = std::function<void(ActionID actionId, InputEventType eventType, float deltaTime)>;
    using AxisCallback = std::function<void(AxisID axisId, float value, float deltaTime)>;

    /**
     * @brief Configuration parameters for InputManager
     */
    struct InputManagerConfig {
        bool enableKeyboardRepeat = false;          // Enable key repeat events
        bool enableMouseCapture = false;            // Capture mouse exclusively
        bool enableGamepadHotswap = true;           // Allow gamepad connect/disconnect
        std::chrono::milliseconds inputBufferTime{100};  // Input buffering window
        float globalAxisSensitivity = 1.0f;        // Global axis sensitivity multiplier
        bool enableInputLogging = false;            // Log input events for debugging
        std::uint32_t maxGamepads = 4;             // Maximum number of gamepads to support
    };

    /**
     * @brief High-performance input manager with flexible action/axis mapping
     *
     * This class provides:
     * - Action-based input system with configurable bindings
     * - Axis input with dead zones, sensitivity, and smoothing
     * - Multi-device support (keyboard, mouse, multiple gamepads)
     * - Input buffering for fighting games and precise timing
     * - Thread-safe input state querying
     * - Hot-swappable controller support
     * - Configurable input profiles for different contexts
     */
    class InputManager {
    public:
        /**
         * @brief Constructor with configuration
         * @param config Configuration parameters for input handling
         */
        explicit InputManager(const InputManagerConfig& config = {});

        /**
         * @brief Destructor - cleanup SDL input subsystems
         */
        ~InputManager();

        // Non-copyable but moveable for flexibility
        InputManager(const InputManager&) = delete;
        InputManager& operator=(const InputManager&) = delete;
        InputManager(InputManager&&) = default;
        InputManager& operator=(InputManager&&) = default;

        /**
         * @brief Initialize the input manager and SDL input subsystems
         * @return true if initialization was successful
         */
        bool initialize();

        /**
         * @brief Shutdown and cleanup all input resources
         */
        void shutdown();

        /**
         * @brief Process SDL input events (call once per frame before update)
         * @param deltaTime Time elapsed since last frame in seconds
         */
        void processEvents(float deltaTime);

        /**
         * @brief Update input state and execute callbacks (call once per frame after processEvents)
         * @param deltaTime Time elapsed since last frame in seconds
         */
        void update(float deltaTime);

        // === ACTION MANAGEMENT ===

        /**
         * @brief Create a new input action with a unique ID
         * @param name Human-readable name for debugging
         * @return ActionID for the created action, or INVALID_ACTION_ID if creation failed
         */
        ActionID createAction(const std::string& name);

        /**
         * @brief Remove an action and all its bindings
         * @param actionId Action to remove
         * @return true if action was successfully removed
         */
        bool removeAction(ActionID actionId);

        /**
         * @brief Bind a keyboard key to an action
         * @param actionId Target action
         * @param keycode SDL keycode to bind
         * @param weight Contribution weight (default: 1.0)
         * @return true if binding was successful
         */
        bool bindKeyToAction(ActionID actionId, SDL_Keycode keycode, float weight = 1.0f);

        /**
         * @brief Bind a mouse button to an action
         * @param actionId Target action
         * @param button Mouse button to bind
         * @param weight Contribution weight (default: 1.0)
         * @return true if binding was successful
         */
        bool bindMouseButtonToAction(ActionID actionId, MouseButton button, float weight = 1.0f);

        /**
         * @brief Bind a gamepad button to an action
         * @param actionId Target action
         * @param gamepadId Gamepad index (0-3)
         * @param button SDL gamepad button to bind
         * @param weight Contribution weight (default: 1.0)
         * @return true if binding was successful
         */
        bool bindGamepadButtonToAction(ActionID actionId, int gamepadId, SDL_GameControllerButton button, float weight = 1.0f);

        /**
         * @brief Set callback function for action events
         * @param actionId Target action
         * @param callback Function to call when action state changes
         */
        void setActionCallback(ActionID actionId, const ActionCallback& callback);

        // === AXIS MANAGEMENT ===

        /**
         * @brief Create a new input axis with a unique ID
         * @param name Human-readable name for debugging
         * @param config Axis configuration (dead zone, sensitivity, etc.)
         * @return AxisID for the created axis, or INVALID_AXIS_ID if creation failed
         */
        AxisID createAxis(const std::string& name, const AxisConfig& config = {});

        /**
         * @brief Remove an axis and all its bindings
         * @param axisId Axis to remove
         * @return true if axis was successfully removed
         */
        bool removeAxis(AxisID axisId);

        /**
         * @brief Bind keyboard keys to an axis (positive/negative)
         * @param axisId Target axis
         * @param positiveKey Key for positive direction
         * @param negativeKey Key for negative direction
         * @param weight Contribution weight (default: 1.0)
         * @return true if binding was successful
         */
        bool bindKeysToAxis(AxisID axisId, SDL_Keycode positiveKey, SDL_Keycode negativeKey, float weight = 1.0f);

        /**
         * @brief Bind gamepad axis to an input axis
         * @param axisId Target axis
         * @param gamepadId Gamepad index (0-3)
         * @param gamepadAxis SDL gamepad axis to bind
         * @param weight Contribution weight (default: 1.0)
         * @return true if binding was successful
         */
        bool bindGamepadAxisToAxis(AxisID axisId, int gamepadId, GamepadAxis gamepadAxis, float weight = 1.0f);

        /**
         * @brief Set callback function for axis value changes
         * @param axisId Target axis
         * @param callback Function to call when axis value changes
         */
        void setAxisCallback(AxisID axisId, const AxisCallback& callback);

        /**
         * @brief Update axis configuration
         * @param axisId Target axis
         * @param config New configuration
         */
        void updateAxisConfig(AxisID axisId, const AxisConfig& config);

        // === INPUT STATE QUERIES ===

        /**
         * @brief Check if an action is currently pressed (this frame)
         * @param actionId Action to check
         * @return true if action was just pressed
         */
        bool isActionPressed(ActionID actionId) const;

        /**
         * @brief Check if an action is currently held down
         * @param actionId Action to check
         * @return true if action is being held
         */
        bool isActionHeld(ActionID actionId) const;

        /**
         * @brief Check if an action was just released (this frame)
         * @param actionId Action to check
         * @return true if action was just released
         */
        bool isActionReleased(ActionID actionId) const;

        /**
         * @brief Get current axis value with all processing applied
         * @param axisId Axis to query
         * @return Processed axis value (-1.0 to 1.0), or 0.0 if axis doesn't exist
         */
        float getAxisValue(AxisID axisId) const;

        /**
         * @brief Get raw axis value before processing (for debugging)
         * @param axisId Axis to query
         * @return Raw axis value, or 0.0 if axis doesn't exist
         */
        float getRawAxisValue(AxisID axisId) const;

        /**
         * @brief Get axis delta (change since last frame)
         * @param axisId Axis to query
         * @return Change in axis value since last frame
         */
        float getAxisDelta(AxisID axisId) const;

        // === DEVICE MANAGEMENT ===

        /**
         * @brief Get number of connected gamepads
         * @return Number of active gamepads
         */
        int getConnectedGamepadCount() const;

        /**
         * @brief Check if a specific gamepad is connected
         * @param gamepadId Gamepad index to check (0-3)
         * @return true if gamepad is connected and functional
         */
        bool isGamepadConnected(int gamepadId) const;

        /**
         * @brief Get name of connected gamepad
         * @param gamepadId Gamepad index (0-3)
         * @return Gamepad name, or empty string if not connected
         */
        std::string getGamepadName(int gamepadId) const;

        /**
         * @brief Enable/disable gamepad rumble
         * @param gamepadId Gamepad index (0-3)
         * @param lowFreq Low frequency rumble (0.0-1.0)
         * @param highFreq High frequency rumble (0.0-1.0)
         * @param durationMs Duration in milliseconds
         * @return true if rumble was set successfully
         */
        bool setGamepadRumble(int gamepadId, float lowFreq, float highFreq, std::uint32_t durationMs);

        // === CONFIGURATION ===

        /**
         * @brief Save current input configuration to file
         * @param filepath Path to save configuration
         * @return true if save was successful
         */
        bool saveConfiguration(const std::string& filepath) const;

        /**
         * @brief Load input configuration from file
         * @param filepath Path to load configuration from
         * @return true if load was successful
         */
        bool loadConfiguration(const std::string& filepath);

        /**
         * @brief Clear all action and axis bindings
         */
        void clearAllBindings();

        /**
         * @brief Get list of all action names for UI
         * @return Vector of action name strings
         */
        std::vector<std::string> getActionNames() const;

        /**
         * @brief Get list of all axis names for UI
         * @return Vector of axis name strings
         */
        std::vector<std::string> getAxisNames() const;

        /**
         * @brief Check if the application should exit
         * @return true if exit was requested through input
         */
        bool shouldExit() const { return shouldExitFlag_.load(std::memory_order_acquire); }

        /**
         * @brief Force exit flag (for programmatic exit)
         * @param exit Whether to exit
         */
        void setShouldExit(bool exit) { shouldExitFlag_.store(exit, std::memory_order_release); }

        // === DEBUG AND PROFILING ===

        /**
         * @brief Get debug information about current input state
         * @return String with detailed input state information
         */
        std::string getDebugInfo() const;

        /**
         * @brief Enable/disable input event logging
         * @param enable Whether to log input events
         */
        void setInputLogging(bool enable) { config_.enableInputLogging = enable; }

        /**
         * @brief Get input statistics for performance monitoring
         * @return String with performance statistics
         */
        std::string getInputStats() const;

    private:
        // === INTERNAL STRUCTURES ===

        /**
         * @brief Internal action state tracking
         */
        struct ActionState {
            std::string name;                           // Human-readable name
            std::vector<InputBinding> bindings;         // All bindings for this action
            bool currentState = false;                  // Current frame state
            bool previousState = false;                 // Previous frame state
            std::chrono::steady_clock::time_point lastActivation;  // Last activation time
            ActionCallback callback;                    // Optional callback function
        };

        /**
         * @brief Internal axis state tracking
         */
        struct AxisState {
            std::string name;                           // Human-readable name
            std::vector<InputBinding> bindings;         // All bindings for this axis
            AxisConfig config;                          // Configuration parameters
            float currentValue = 0.0f;                  // Current processed value
            float previousValue = 0.0f;                 // Previous frame value
            float rawValue = 0.0f;                      // Raw unprocessed value
            float smoothedValue = 0.0f;                 // Smoothed value for gradual changes
            AxisCallback callback;                      // Optional callback function
        };

        /**
         * @brief Gamepad state tracking
         */
        struct GamepadState {
            SDL_GameController* controller = nullptr;   // SDL controller handle
            std::string name;                          // Controller name
            bool isConnected = false;                  // Connection status
            std::array<bool, SDL_CONTROLLER_BUTTON_MAX> buttonStates{};     // Button states
            std::array<float, SDL_CONTROLLER_AXIS_MAX> axisValues{};        // Axis values
            std::chrono::steady_clock::time_point lastRumbleEnd;           // Rumble end time
        };

        // === MEMBER VARIABLES ===

        // Configuration and state
        InputManagerConfig config_;                     // Configuration parameters
        bool initialized_ = false;                      // Initialization state
        std::atomic<bool> shouldExitFlag_{false};      // Thread-safe exit flag

        // ID generation
        ActionID nextActionId_ = 1;                     // Next action ID to assign
        AxisID nextAxisId_ = 1;                         // Next axis ID to assign

        // Input state storage
        std::unordered_map<ActionID, ActionState> actions_;     // All registered actions
        std::unordered_map<AxisID, AxisState> axes_;            // All registered axes

        // Device state
        const std::uint8_t* keyboardState_ = nullptr;   // Current keyboard state from SDL
        std::vector<std::uint8_t> previousKeyboardState_;  // Previous keyboard state
        int keyboardStateSize_ = 0;                     // Size of keyboard state array

        std::uint32_t currentMouseState_ = 0;           // Current mouse button state
        std::uint32_t previousMouseState_ = 0;          // Previous mouse button state

        std::array<GamepadState, 4> gamepads_;          // State for up to 4 gamepads

        // Threading and performance
        mutable std::mutex inputMutex_;                 // Mutex for thread-safe operations
        std::chrono::steady_clock::time_point lastUpdateTime_;  // Last update timestamp

        // Statistics
        mutable std::atomic<std::uint64_t> eventsProcessed_{0}; // Events processed counter
        mutable std::atomic<std::uint64_t> actionsTriggered_{0}; // Actions triggered counter

        // === INTERNAL METHODS ===

        /**
         * @brief Initialize SDL input subsystems
         * @return true if initialization successful
         */
        bool initializeSDL();

        /**
         * @brief Initialize gamepad detection and setup
         */
        void initializeGamepads();

        /**
         * @brief Update keyboard state tracking
         */
        void updateKeyboardState();

        /**
         * @brief Update mouse state tracking
         */
        void updateMouseState();

        /**
         * @brief Update all gamepad states
         */
        void updateGamepadStates();

        /**
         * @brief Process all action bindings and update states
         * @param deltaTime Time elapsed since last frame
         */
        void processActions(float deltaTime);

        /**
         * @brief Process all axis bindings and update values
         * @param deltaTime Time elapsed since last frame
         */
        void processAxes(float deltaTime);

        /**
         * @brief Handle SDL input events
         * @param event SDL event to process
         * @param deltaTime Time elapsed since last frame
         */
        void handleSDLEvent(const SDL_Event& event, float deltaTime);

        /**
         * @brief Process gamepad connection/disconnection
         * @param deviceIndex SDL device index
         * @param connected Whether device was connected or disconnected
         */
        void handleGamepadDeviceChange(int deviceIndex, bool connected);

        /**
         * @brief Apply axis configuration (dead zone, sensitivity, smoothing)
         * @param rawValue Raw input value
         * @param config Axis configuration
         * @param previousValue Previous processed value (for smoothing)
         * @param deltaTime Frame delta time
         * @return Processed axis value
         */
        float processAxisValue(float rawValue, const AxisConfig& config, float previousValue, float deltaTime) const;

        /**
         * @brief Apply dead zone to axis value
         * @param value Input value
         * @param deadZone Dead zone threshold
         * @return Value with dead zone applied
         */
        float applyDeadZone(float value, float deadZone) const;

        /**
         * @brief Apply smoothing to axis value
         * @param currentValue Current frame value
         * @param previousValue Previous frame value
         * @param smoothing Smoothing factor (0.0-1.0)
         * @param deltaTime Frame delta time
         * @return Smoothed value
         */
        float applySmoothing(float currentValue, float previousValue, float smoothing, float deltaTime) const;

        /**
         * @brief Get input value from a specific binding
         * @param binding Input binding to evaluate
         * @return Current input value (0.0-1.0 for buttons, -1.0-1.0 for axes)
         */
        float getBindingValue(const InputBinding& binding) const;

        /**
         * @brief Find gamepad index from SDL instance ID
         * @param instanceId SDL gamepad instance ID
         * @return Gamepad index (0-3), or -1 if not found
         */
        int findGamepadIndex(SDL_JoystickID instanceId) const;

        /**
         * @brief Log input event for debugging
         * @param message Log message
         */
        void logInputEvent(const std::string& message) const;

        /**
         * @brief Validate action ID
         * @param actionId Action ID to validate
         * @return true if action ID is valid
         */
        bool isValidActionId(ActionID actionId) const;

        /**
         * @brief Validate axis ID
         * @param axisId Axis ID to validate
         * @return true if axis ID is valid
         */
        bool isValidAxisId(AxisID axisId) const;
    };

} // namespace engine::input
