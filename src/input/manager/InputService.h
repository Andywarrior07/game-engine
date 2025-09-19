/**
 * @file InputService.h
 * @brief Main input service that orchestrates the entire input system
 * @author Andrés Guerrero
 * @date 13-09-2025
 *
 * Central manager for the input system. Coordinates devices, processes events,
 * manages contexts, and generates frame snapshots for deterministic input.
 */

#pragma once

#include "../core/InputTypes.h"
#include "../core/InputSnapshot.h"
#include "../processing/EventProcessor.h"
#include "../processing/SnapshotManager.h"
#include "../devices/base/DeviceService.h"
#include "../platform/sdl2/SDLInputBackend.h"

#include "../../math/MathSystem.h"

#include <memory>
#include <functional>
#include <atomic>

namespace engine::input {
    namespace mapping {
        class ContextStack;
    }

    // Forward declarations
    class ActionMap;
    class ContextStack;
    class InputMemoryPools;
    class MouseLock;

    /**
     * @brief Input service configuration
     */
    struct InputServiceConfig {
        // Memory configuration
        std::size_t eventPoolSize = 1024;
        std::size_t snapshotPoolSize = 128;
        std::size_t historyFrameCount = 600;

        // Device configuration
        bool enableKeyboard = true;
        bool enableMouse = true;
        bool enableGamepad = true;
        bool enableTouch = true;
        std::uint8_t maxGamepads = 4;

        // Processing configuration
        float analogDeadzone = 0.15f;
        float triggerThreshold = 0.1f;
        float mouseSensitivity = 1.0f;
        float gamepadSensitivity = 1.0f;

        // Behavior configuration
        bool consumeProcessedEvents = true;
        bool enableInputBuffering = true;
        std::size_t inputBufferFrames = 6;
        bool enableGestureRecognition = true;

        // Debug configuration
        bool enableDebugLogging = false;
        bool enableInputVisualization = false;
        bool recordInputForReplay = false;
    };

    /**
     * @brief Main input system service
     *
     * Orchestrates all input subsystems using the new modular architecture.
     */
    class InputService {
    public:
        using ActionCallback = std::function<void(ActionID, const ActionState&)>;
        using RawEventCallback = std::function<bool(const InputEvent&)>;
        using ContextChangeCallback = std::function<void(const std::string&, const std::string&)>;

        /**
         * @brief Constructor
         */
        explicit InputService(memory::MemoryManager* memoryManager) noexcept;

        /**
         * @brief Destructor
         */
        ~InputService();

        // Disable copy, enable move
        InputService(const InputService&) = delete;
        InputService& operator=(const InputService&) = delete;
        InputService(InputService&&) noexcept = default;
        InputService& operator=(InputService&&) noexcept = default;

        // ============================================================================
        // Initialization and Lifecycle
        // ============================================================================

        /**
         * @brief Initialize the input service
         */
        bool initialize(const InputServiceConfig& config = {});

        /**
         * @brief Shutdown the input service
         */
        void shutdown();

        /**
         * @brief Process input for current frame
         */
        void tick(float deltaTime);

        /**
         * @brief Check if initialized
         */
        [[nodiscard]] bool isInitialized() const noexcept {
            return initialized_;
        }

        // ============================================================================
        // Snapshot Access
        // ============================================================================

        /**
         * @brief Get current frame's input snapshot
         */
        [[nodiscard]] const InputSnapshot* getSnapshot() const;

        /**
         * @brief Get previous frame's input snapshot
         */
        [[nodiscard]] const InputSnapshot* getPreviousSnapshot() const;

        /**
         * @brief Get frame number
         */
        [[nodiscard]] std::uint64_t getFrameNumber() const noexcept {
            return frameNumber_.load(std::memory_order_acquire);
        }

        // ============================================================================
        // Action Mapping
        // ============================================================================

        /**
         * @brief Register action mapping
         */
        void registerAction(ActionID id, const std::string& name,
                            ActionType type = ActionType::BUTTON) const;

        /**
         * @brief Bind input to action
         */
        void bindInput(ActionID actionId, KeyCode key,
                       const std::string& context = "Default") const;

        void bindInput(ActionID actionId, MouseButton button,
                       const std::string& context = "Default") const;

        void bindInput(ActionID actionId, GamepadButton button,
                       const std::string& context = "Default") const;

        void bindAxis(ActionID actionId, GamepadAxis axis,
                      const std::string& context = "Default") const;

        /**
         * @brief Register action callback
         */
        void registerActionCallback(ActionID actionId, ActionCallback callback) const;

        /**
         * @brief Clear action callbacks
         */
        void clearActionCallbacks(ActionID actionId) const;

        // ============================================================================
        // Context Management
        // ============================================================================

        /**
         * @brief Push new input context
         */
        bool pushContext(const std::string& name, ContextPriority priority = ContextPriority::NORMAL) const;

        /**
         * @brief Pop current context
         */
        bool popContext() const;

        /**
         * @brief Get current context name
         */
        [[nodiscard]] std::string getCurrentContext() const;

        /**
         * @brief Register context change callback
         */
        void registerContextChangeCallback(ContextChangeCallback callback) const;

        // ============================================================================
        // Device Management
        // ============================================================================

        /**
         * @brief Set gamepad rumble
         */
        bool setGamepadRumble(PlayerID player, float leftMotor, float rightMotor,
                              float duration = 0.5f) const;

        /**
         * @brief Enable/disable text input mode
         */
        void setTextInputMode(bool enabled) const;

        /**
         * @brief Set relative mouse mode
         */
        bool setRelativeMouseMode(bool enabled) const;

        /**
         * @brief Get mouse lock manager
         */
        [[nodiscard]] MouseLock* getMouseLock() const noexcept {
            return mouseLock_.get();
        }

        // ============================================================================
        // Raw Event Access
        // ============================================================================

        /**
         * @brief Register raw event callback
         */
        void registerRawEventCallback(RawEventCallback callback) const;

        /**
         * @brief Inject synthetic event
         */
        void injectEvent(const InputEvent& event) const;

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
        [[nodiscard]] bool isRecording() const noexcept;

        /**
         * @brief Export recorded input
         */
        [[nodiscard]] std::vector<std::uint8_t> exportRecording() const;

        /**
         * @brief Import and start replay
         */
        bool startReplay(const std::vector<std::uint8_t>& replayData) const;

        /**
         * @brief Stop replay
         */
        void stopReplay() const;

        /**
         * @brief Check if replaying
         */
        [[nodiscard]] bool isReplaying() const noexcept;

        /**
         * @brief Pause/resume input processing
         */
        void setPaused(const bool paused) noexcept {
            isPaused_.store(paused, std::memory_order_release);
        }

        /**
         * @brief Check if paused
         */
        [[nodiscard]] bool isPaused() const noexcept {
            return isPaused_.load(std::memory_order_acquire);
        }

        // ============================================================================
        // Subsystem Access
        // ============================================================================

        /**
         * @brief Get device service
         */
        [[nodiscard]] DeviceService* getDeviceService() const noexcept {
            return deviceService_.get();
        }

        /**
         * @brief Get event processor
         */
        [[nodiscard]] processing::EventProcessor* getEventProcessor() const noexcept {
            return eventProcessor_.get();
        }

        /**
         * @brief Get snapshot manager
         */
        [[nodiscard]] processing::SnapshotManager* getSnapshotManager() const noexcept {
            return snapshotManager_.get();
        }

        /**
         * @brief Get action map
         */
        [[nodiscard]] ActionMap* getActionMap() const noexcept {
            return actionMap_.get();
        }

        /**
         * @brief Get context stack
         */
        [[nodiscard]] mapping::ContextStack* getContextStack() const noexcept {
            return contextStack_.get();
        }

        /**
         * @brief Get SDL backend
         */
        [[nodiscard]] SDLInputBackend* getBackend() const noexcept {
            return backend_.get();
        }

        // ============================================================================
        // Statistics
        // ============================================================================

        struct Statistics {
            std::uint64_t totalFrames = 0;
            std::uint64_t droppedEvents = 0;
            float averageFrameTime = 0.0f;
            float peakFrameTime = 0.0f;

            void reset() noexcept {
                totalFrames = 0;
                droppedEvents = 0;
                averageFrameTime = 0.0f;
                peakFrameTime = 0.0f;
            }
        };

        [[nodiscard]] const Statistics& getStatistics() const noexcept {
            return stats_;
        }

        void resetStatistics() const noexcept {
            stats_.reset();
        }

    private:
        // ============================================================================
        // Private Methods
        // ============================================================================

        /**
         * @brief Initialize subsystems
         */
        bool initializeSubsystems();

        /**
         * @brief Shutdown subsystems
         */
        void shutdownSubsystems();

        /**
         * @brief Process frame
         */
        void processFrame(float deltaTime) const;

        /**
         * @brief Update statistics
         */
        void updateStatistics(float frameTime) const;

        // ============================================================================
        // Member Variables
        // ============================================================================

        // Core systems
        memory::MemoryManager* memoryManager_;

        // Configuration
        InputServiceConfig config_;
        std::atomic<bool> initialized_{false};

        // Memory pools
        std::unique_ptr<InputMemoryPools> memoryPools_;

        // Subsystems
        std::unique_ptr<DeviceService> deviceService_;
        std::unique_ptr<processing::EventProcessor> eventProcessor_;
        std::unique_ptr<processing::SnapshotManager> snapshotManager_;
        std::unique_ptr<ActionMap> actionMap_;
        std::unique_ptr<mapping::ContextStack> contextStack_;
        std::unique_ptr<SDLInputBackend> backend_;
        std::unique_ptr<MouseLock> mouseLock_;

        // State
        std::atomic<std::uint64_t> frameNumber_{0};
        std::atomic<bool> isPaused_{false};

        // Statistics
        mutable Statistics stats_;
        std::chrono::steady_clock::time_point lastFrameTime_;
    };
} // namespace engine::input
