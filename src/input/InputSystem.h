/**
 * @file InputSystem.h
 * @brief Main input system interface for the game engine
 * @author Andrés Guerrero
 * @date 13-09-2025
 *
 * High-level interface that wraps the input service and provides
 * a simplified API for game code to interact with the input system.
 */

#pragma once

#include "core/InputTypes.h"
#include "core/InputSnapshot.h"

#include "../memory/MemorySystem.h"

#include <memory>
#include <string>
#include <functional>

namespace engine::input {
    // Forward declarations
    class InputService;
    class InputLifecycle;

    namespace debug {
        class InputLogger;
        class InputDebugOverlay;
    }

    namespace utils {
        class ConfigLoader;
        struct InputConfigData;
    }

    /**
     * @brief Main input system configuration
     */
    struct InputSystemConfig {
        // Core settings
        bool enableKeyboard = true;
        bool enableMouse = true;
        bool enableGamepad = true;
        bool enableTouch = true;

        // Performance settings
        std::size_t eventPoolSize = 1024;
        std::size_t snapshotPoolSize = 128;
        std::size_t historyFrameCount = 600;

        // Input processing
        float analogDeadzone = 0.15f;
        float triggerThreshold = 0.1f;
        float mouseSensitivity = 1.0f;
        bool enableInputBuffering = true;
        std::size_t inputBufferFrames = 6;

        // Debug settings
        bool enableDebugLogging = false;
        bool enableDebugOverlay = false;
        bool recordForReplay = false;

        // Configuration file
        // TODO: Revisar esto
        std::string configFilePath;
        bool autoLoadConfig = true;
        bool autoSaveConfig = false;

        InputSystemConfig() = default;
    };

    /**
     * @brief Main input system class
     *
     * Provides a high-level interface to the input system, managing
     * lifecycle, configuration, and access to input state.
     */
    class InputSystem {
    public:
        using ActionCallback = std::function<void(ActionID, const ActionState&)>;
        using EventCallback = std::function<bool(const InputEvent&)>;
        using ContextChangeCallback = std::function<void(const std::string&, const std::string&)>;

        /**
         * @brief Constructor
         * @param memoryManager Memory manager for allocations
         * @param mathSystem Math system for vector operations
         */
        explicit InputSystem(memory::MemoryManager* memoryManager);

        /**
         * @brief Destructor
         */
        ~InputSystem();

        // Disable copy, enable move
        InputSystem(const InputSystem&) = delete;
        InputSystem& operator=(const InputSystem&) = delete;
        InputSystem(InputSystem&&) noexcept;
        InputSystem& operator=(InputSystem&&) noexcept;

        // ============================================================================
        // Lifecycle Management
        // ============================================================================

        /**
         * @brief Initialize the input system
         * @param config System configuration
         * @return True if successful
         */
        bool initialize(const InputSystemConfig& config = {});

        /**
         * @brief Shutdown the input system
         */
        void shutdown();

        /**
         * @brief Update the input system
         * @param deltaTime Time since last update
         */
        void update(float deltaTime) const;

        /**
         * @brief Check if system is initialized
         */
        [[nodiscard]] bool isInitialized() const noexcept;

        /**
         * @brief Pause input processing
         */
        void pause() const;

        /**
         * @brief Resume input processing
         */
        void resume() const;

        /**
         * @brief Check if paused
         */
        [[nodiscard]] bool isPaused() const noexcept;

        // ============================================================================
        // Input State Access
        // ============================================================================

        /**
         * @brief Get current input snapshot
         */
        [[nodiscard]] const InputSnapshot* getSnapshot() const;

        /**
         * @brief Get previous input snapshot
         */
        [[nodiscard]] const InputSnapshot* getPreviousSnapshot() const;

        /**
         * @brief Check if key is pressed
         */
        [[nodiscard]] bool isKeyPressed(KeyCode key) const;

        /**
         * @brief Check if key was just pressed
         */
        [[nodiscard]] bool isKeyJustPressed(KeyCode key) const;

        /**
         * @brief Check if key was just released
         */
        [[nodiscard]] bool isKeyJustReleased(KeyCode key) const;

        /**
         * @brief Check if mouse button is pressed
         */
        [[nodiscard]] bool isMouseButtonPressed(MouseButton button) const;

        /**
         * @brief Get mouse position
         */
        [[nodiscard]] math::Vec2 getMousePosition() const;

        /**
         * @brief Get mouse delta
         */
        [[nodiscard]] math::Vec2 getMouseDelta() const;

        /**
         * @brief Get mouse wheel delta
         */
        [[nodiscard]] math::Vec2 getMouseWheel() const;

        /**
         * @brief Check if gamepad button is pressed
         */
        [[nodiscard]] bool isGamepadButtonPressed(PlayerID player, GamepadButton button) const;

        /**
         * @brief Get gamepad stick position
         */
        [[nodiscard]] math::Vec2 getGamepadStick(PlayerID player, bool leftStick) const;

        /**
         * @brief Get gamepad trigger value
         */
        [[nodiscard]] float getGamepadTrigger(PlayerID player, bool leftTrigger) const;

        // ============================================================================
        // Action System
        // ============================================================================

        /**
         * @brief Register action
         * @param id Action ID
         * @param name Action name
         * @param type Action type
         */
        void registerAction(ActionID id, const std::string& name,
                            ActionType type = ActionType::BUTTON) const;

        /**
         * @brief Bind key to action
         */
        void bindKey(ActionID actionId, KeyCode key,
                     const std::string& context = "Default") const;

        /**
         * @brief Bind mouse button to action
         */
        void bindMouseButton(ActionID actionId, MouseButton button,
                             const std::string& context = "Default") const;

        /**
         * @brief Bind gamepad button to action
         */
        void bindGamepadButton(ActionID actionId, GamepadButton button,
                               const std::string& context = "Default") const;

        /**
         * @brief Bind gamepad axis to action
         */
        void bindGamepadAxis(ActionID actionId, GamepadAxis axis,
                             const std::string& context = "Default") const;

        /**
         * @brief Check if action is triggered
         */
        [[nodiscard]] bool isActionTriggered(ActionID actionId) const;

        /**
         * @brief Get action value
         */
        [[nodiscard]] float getActionValue(ActionID actionId) const;

        /**
         * @brief Get action 2D value
         */
        [[nodiscard]] math::Vec2 getActionValue2D(ActionID actionId) const;

        /**
         * @brief Register action callback
         */
        void registerActionCallback(ActionID actionId, ActionCallback callback) const;

        // ============================================================================
        // Context Management
        // ============================================================================

        /**
         * @brief Push input context
         */
        bool pushContext(const std::string& name,
                         ContextPriority priority = ContextPriority::NORMAL) const;

        /**
         * @brief Pop input context
         */
        bool popContext() const;

        /**
         * @brief Set input context (clear stack and push)
         */
        bool setContext(const std::string& name,
                        ContextPriority priority = ContextPriority::NORMAL) const;

        /**
         * @brief Get current context
         */
        [[nodiscard]] std::string getCurrentContext() const;

        /**
         * @brief Register context change callback
         */
        void registerContextChangeCallback(ContextChangeCallback callback) const;

        // ============================================================================
        // Device Control
        // ============================================================================

        /**
         * @brief Set gamepad rumble
         */
        bool setGamepadRumble(PlayerID player, float leftMotor, float rightMotor,
                              float duration = 0.5f) const;

        /**
         * @brief Stop gamepad rumble
         */
        void stopGamepadRumble(PlayerID player) const;

        /**
         * @brief Enable text input mode
         */
        void enableTextInput() const;

        /**
         * @brief Disable text input mode
         */
        void disableTextInput() const;

        /**
         * @brief Check if text input is active
         */
        [[nodiscard]] bool isTextInputActive() const;

        /**
         * @brief Set relative mouse mode (FPS style)
         */
        bool setRelativeMouseMode(bool enabled) const;

        /**
         * @brief Check if relative mouse mode is enabled
         */
        [[nodiscard]] bool isRelativeMouseMode() const;

        /**
         * @brief Show/hide mouse cursor
         */
        void setMouseCursorVisible(bool visible) const;

        // ============================================================================
        // Configuration
        // ============================================================================

        /**
         * @brief Load configuration from file
         * @param filepath Configuration file path
         * @return True if successful
         */
        bool loadConfiguration(const std::string& filepath);

        /**
         * @brief Save configuration to file
         * @param filepath Output file path
         * @return True if successful
         */
        bool saveConfiguration(const std::string& filepath) const;

        /**
         * @brief Apply configuration preset
         * @param presetName Preset name (FPS, RPG, Strategy, etc.)
         */
        void applyPreset(const std::string& presetName) const;

        /**
         * @brief Get current configuration
         */
        [[nodiscard]] const InputSystemConfig& getConfig() const noexcept {
            return config_;
        }

        // ============================================================================
        // Recording and Replay
        // ============================================================================

        /**
         * @brief Start recording input
         */
        void startRecording() const;

        /**
         * @brief Stop recording
         */
        void stopRecording() const;

        /**
         * @brief Check if recording
         */
        [[nodiscard]] bool isRecording() const;

        /**
         * @brief Save recording to file
         * @param filepath Output file path
         * @return True if successful
         */
        bool saveRecording(const std::string& filepath) const;

        /**
         * @brief Load and play recording
         * @param filepath Recording file path
         * @return True if successful
         */
        bool playRecording(const std::string& filepath) const;

        /**
         * @brief Stop replay
         */
        void stopReplay() const;

        /**
         * @brief Check if replaying
         */
        [[nodiscard]] bool isReplaying() const;

        // ============================================================================
        // Debug
        // ============================================================================

        /**
         * @brief Enable debug logging
         * @param enabled Enable state
         */
        void setDebugLoggingEnabled(bool enabled);

        /**
         * @brief Enable debug overlay
         * @param enabled Enable state
         */
        void setDebugOverlayEnabled(bool enabled);

        /**
         * @brief Toggle debug overlay
         */
        void toggleDebugOverlay() const;

        /**
         * @brief Get debug overlay
         */
        [[nodiscard]] debug::InputDebugOverlay* getDebugOverlay() const;

        /**
         * @brief Register raw event callback
         */
        void registerEventCallback(EventCallback callback) const;

        /**
         * @brief Inject synthetic event
         */
        void injectEvent(const InputEvent& event) const;

        // ============================================================================
        // Statistics
        // ============================================================================

        struct Statistics {
            std::uint64_t totalFrames = 0;
            std::uint64_t totalEvents = 0;
            float averageFrameTime = 0.0f;
            float peakFrameTime = 0.0f;
            std::size_t memoryUsage = 0;

            void reset() noexcept {
                totalFrames = 0;
                totalEvents = 0;
                averageFrameTime = 0.0f;
                peakFrameTime = 0.0f;
                memoryUsage = 0;
            }
        };

        [[nodiscard]] Statistics getStatistics() const;

        void resetStatistics() const;

    private:
        // Implementation pointer pattern for ABI stability
        class Impl;
        std::unique_ptr<Impl> pImpl;

        // Configuration (kept here for quick access)
        InputSystemConfig config_;
    };
} // namespace engine::input
