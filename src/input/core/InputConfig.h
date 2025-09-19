/**
 * @file InputConfig.h
 * @brief Global configuration settings for the input system
 * @author Game Engine Team
 * @date 2024
 *
 * Centralized configuration for all input system parameters.
 * Supports runtime tweaking and serialization.
 */

#pragma once

#include "../core/InputConstants.h"

#include "../../math/MathSystem.h"

#include <string>

namespace engine::input {
    /**
     * @brief Global input system configuration
     *
     * Contains all tunable parameters for the input system.
     * Can be loaded from JSON/XML or modified at runtime.
     */
    struct InputConfig {
        // ============================================================================
        // Device Configuration
        // ============================================================================

        struct DeviceConfig {
            bool enabled = true;
            float pollRate = 60.0f; // Hz
            float timeout = 5.0f; // Seconds before disconnect

            // Device-specific limits
            std::uint32_t maxDevices = 16;
            std::uint32_t maxEventsPerFrame = 256;

            DeviceConfig() = default;
        };

        struct KeyboardConfig : DeviceConfig {
            bool enableRepeat = true;
            float repeatDelay = 0.5f; // Initial delay before repeat
            float repeatRate = 0.05f; // Repeat interval
            bool blockSystemKeys = false; // Block OS shortcuts

            KeyboardConfig() {
                maxDevices = 1; // Usually only one keyboard
            }
        };

        struct MouseConfig : DeviceConfig {
            float sensitivity = 1.0f;
            float acceleration = 0.0f;
            float wheelSensitivity = 1.0f;
            float doubleClickTime = 0.3f; // Seconds
            float doubleClickDistance = 5.0f; // Pixels
            bool rawInput = true;
            bool invertY = false;
            bool smoothing = false;
            float smoothingFactor = 0.5f;

            MouseConfig() {
                maxDevices = 1;
            }
        };

        struct GamepadConfig : DeviceConfig {
            float stickDeadzone = DEFAULT_STICK_DEADZONE;
            float triggerThreshold = DEFAULT_TRIGGER_THRESHOLD;
            float stickSensitivity = 1.0f;
            float triggerSensitivity = 1.0f;
            bool invertLeftY = false;
            bool invertRightY = false;
            bool swapSticks = false;
            bool rumbleEnabled = true;
            float rumbleScale = 1.0f;
            bool enableDeadzone = true;

            // Response curves
            enum class StickCurve {
                LINEAR,
                QUADRATIC,
                CUBIC,
                EXPONENTIAL,
                CUSTOM
            };

            StickCurve leftStickCurve = StickCurve::LINEAR;
            StickCurve rightStickCurve = StickCurve::LINEAR;

            GamepadConfig() {
                maxDevices = MAX_GAMEPADS;
            }
        };

        struct TouchConfig : DeviceConfig {
            float tapTime = 0.2f; // Max time for tap
            float tapDistance = 10.0f; // Max movement for tap
            float holdTime = 0.5f; // Time before hold triggers
            float swipeMinDistance = 50.0f;
            float swipeMaxTime = 0.5f;
            float pinchThreshold = 0.1f;
            bool multiTouchEnabled = true;
            std::uint32_t maxTouchPoints = MAX_TOUCH_POINTS;

            TouchConfig() {
                maxDevices = 1;
            }
        };

        // ============================================================================
        // Processing Configuration
        // ============================================================================

        struct ProcessingConfig {
            // Event processing
            bool consumeProcessedEvents = true;
            bool allowSyntheticEvents = true;
            std::uint32_t maxEventsPerTick = 1000;

            // Input buffering
            bool enableInputBuffer = true;
            std::uint32_t inputBufferFrames = DEFAULT_INPUT_BUFFER_FRAMES;
            float inputBufferTimeout = 0.5f; // Seconds

            // Filtering
            bool enableNoiseFilter = true;
            float noiseThreshold = 0.01f;

            // Gesture recognition
            bool enableGestures = true;
            float gestureTimeout = 1.0f;

            ProcessingConfig() = default;
        };

        // ============================================================================
        // Action Mapping Configuration
        // ============================================================================

        struct ActionMappingConfig {
            bool allowRuntimeRemapping = true;
            bool requireUniqueBindings = false; // No duplicate bindings
            bool caseSensitiveContexts = false;
            std::uint32_t maxActionsPerContext = 256;
            std::uint32_t maxContexts = 32;
            float defaultHoldTime = 0.0f;
            float defaultRepeatDelay = 0.5f;
            float defaultRepeatRate = 0.1f;

            ActionMappingConfig() = default;
        };

        // ============================================================================
        // Memory Configuration
        // ============================================================================

        struct MemoryConfig {
            // Pool sizes
            std::size_t eventPoolSize = DEFAULT_EVENT_POOL_SIZE;
            std::size_t snapshotPoolSize = DEFAULT_SNAPSHOT_POOL_SIZE;
            std::size_t devicePoolSize = 32;

            // Buffer sizes
            std::size_t eventQueueSize = 512;
            std::size_t historyBufferFrames = DEFAULT_HISTORY_FRAMES;

            // Allocation flags
            bool preallocateAll = true;
            bool allowDynamicGrowth = false;

            MemoryConfig() = default;
        };

        // ============================================================================
        // Debug Configuration
        // ============================================================================

        struct DebugConfig {
            bool enableLogging = false;
            bool logAllEvents = false;
            bool logStateChanges = true;
            bool enableVisualization = false;
            bool showInputOverlay = false;
            bool recordForReplay = false;
            bool validateEvents = true;
            bool trackMemoryUsage = true;
            std::string logFilePath = "input_debug.log";

            DebugConfig() = default;
        };

        // ============================================================================
        // Main Configuration Structure
        // ============================================================================

        KeyboardConfig keyboard;
        MouseConfig mouse;
        GamepadConfig gamepad;
        TouchConfig touch;
        ProcessingConfig processing;
        ActionMappingConfig actionMapping;
        MemoryConfig memory;
        DebugConfig debug;

        // Global settings
        bool enabled = true;
        float globalSensitivity = 1.0f;
        bool blockInputInBackground = true;

        InputConfig() = default;

        // TODO: Implementar estos metodos
        /**
         * @brief Load configuration from file
         */
        bool loadFromFile(const std::string& filepath) {
            // Would implement JSON/XML parsing here
            // For now, just return success
            return true;
        }

        /**
         * @brief Save configuration to file
         */
        bool saveToFile(const std::string& filepath) const {
            // Would implement JSON/XML serialization here
            return true;
        }

        /**
         * @brief Reset to default values
         */
        void reset() {
            *this = InputConfig{};
        }

        /**
         * @brief Apply a preset configuration
         */
        void applyPreset(const std::string& presetName) {
            if (presetName == "FPS") {
                mouse.sensitivity = 2.0f;
                mouse.rawInput = true;
                gamepad.stickDeadzone = 0.1f;
                processing.enableInputBuffer = false;
            }
            else if (presetName == "RPG") {
                mouse.sensitivity = 1.5f;
                gamepad.stickDeadzone = 0.15f;
                processing.enableInputBuffer = true;
                processing.inputBufferFrames = 6;
            }
            else if (presetName == "Fighting") {
                processing.enableInputBuffer = true;
                processing.inputBufferFrames = 10;
                gamepad.stickDeadzone = 0.2f;
            }
            else if (presetName == "Strategy") {
                mouse.sensitivity = 1.0f;
                mouse.acceleration = 0.2f;
                keyboard.enableRepeat = true;
                keyboard.repeatRate = 0.03f;
            }
        }

        /**
         * @brief Validate configuration
         */
        [[nodiscard]] bool validate() const {
            // Check for invalid values
            if (memory.eventPoolSize == 0) return false;
            if (memory.snapshotPoolSize == 0) return false;
            if (processing.maxEventsPerTick == 0) return false;

            // Validate ranges
            if (mouse.sensitivity < 0.1f || mouse.sensitivity > 10.0f) return false;
            if (gamepad.stickDeadzone < 0.0f || gamepad.stickDeadzone > 0.9f) return false;

            return true;
        }

        // TODO: Implementar estos metodos
        /**
         * @brief Get configuration value by path
         */
        template <typename T>
        T getValue(const std::string& path) const {
            // Would implement reflection or property system here
            // Example: "mouse.sensitivity" -> return mouse.sensitivity
            return T{};
        }

        /**
         * @brief Set configuration value by path
         */
        template <typename T>
        void setValue(const std::string& path, const T& value) {
            // Would implement reflection or property system here
            // Example: "mouse.sensitivity", 2.0f -> mouse.sensitivity = 2.0f
        }
    };

    /**
     * @brief Global configuration instance
     */
    class GlobalInputConfig {
    public:
        static GlobalInputConfig& getInstance() {
            static GlobalInputConfig instance;
            return instance;
        }

        InputConfig& getConfig() { return config_; }
        [[nodiscard]] const InputConfig& getConfig() const { return config_; }

        void setConfig(const InputConfig& config) { config_ = config; }

    private:
        GlobalInputConfig() = default;
        InputConfig config_;
    };
} // namespace engine::input
