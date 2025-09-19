/**
 * @file SDLInputBackend.h
 * @brief SDL2 input backend implementation
 * @author Andrés Guerrero
 * @date 13-09-2025
 *
 * Platform abstraction layer for SDL2 input handling.
 * Converts SDL events to engine events with zero allocations.
 */

#pragma once

#include "SDLKeyMap.h"

#include "../../core/InputEvent.h"
#include "../../devices/base/InputDevice.h"

#include <SDL2/SDL.h>

#include <vector>
#include <unordered_map>
#include <queue>
#include <mutex>
#include <memory>
#include <atomic>

namespace engine::input {
    // Forward declarations
    class DeviceService;
    class MouseLock;
    class InputMemoryPools;

    /**
     * @brief SDL backend configuration
     */
    struct SDLBackendConfig {
        // SDL initialization flags
        Uint32 sdlInitFlags = SDL_INIT_EVENTS | SDL_INIT_GAMECONTROLLER | SDL_INIT_HAPTIC;

        // Event processing
        std::uint32_t maxEventsPerPoll = 64; // Max SDL events per poll
        bool filterSystemEvents = true; // Filter OS-specific events
        bool enableTextInput = false; // Enable text input by default

        // Controller settings
        bool autoLoadControllerMappings = true; // Load gamecontrollerdb.txt
        std::string controllerMappingsFile = "gamecontrollerdb.txt";
        bool enableControllerEvents = true;

        // Mouse settings
        bool enableRelativeMode = false; // Start in relative mode
        bool showCursor = true; // Show cursor initially

        // Window events
        bool processWindowEvents = true; // Handle focus, resize, etc.

        // Touch settings
        bool enableTouchEvents = true; // Process touch events
        bool simulateMouseWithTouch = false; // Touch generates mouse events
    };

    /**
     * @brief SDL2 input backend
     *
     * Handles SDL event polling, device management, and event conversion.
     * Thread-safe and designed for zero allocations during runtime.
     */
    class SDLInputBackend {
    public:
        using EventCallback = std::function<void(const InputEvent&)>;
        using DeviceCallback = std::function<void(DeviceID, bool connected)>;

        /**
         * @brief Constructor
         * @param deviceService Device service for device management
         * @param memoryPools Memory pools for allocations
         */
        explicit SDLInputBackend(DeviceService* deviceService,
                                 InputMemoryPools* memoryPools = nullptr) noexcept;

        /**
         * @brief Destructor
         */
        ~SDLInputBackend();

        // Disable copy, enable move
        SDLInputBackend(const SDLInputBackend&) = delete;
        SDLInputBackend& operator=(const SDLInputBackend&) = delete;
        SDLInputBackend(SDLInputBackend&&) noexcept = default;
        SDLInputBackend& operator=(SDLInputBackend&&) noexcept = default;

        // ============================================================================
        // Initialization and Shutdown
        // ============================================================================

        /**
         * @brief Initialize SDL backend
         * @param config Backend configuration
         * @return True if successful
         */
        bool initialize(const SDLBackendConfig& config = {});

        /**
         * @brief Shutdown SDL backend
         */
        void shutdown();

        /**
         * @brief Check if initialized
         */
        [[nodiscard]] bool isInitialized() const noexcept {
            return initialized_;
        }

        // ============================================================================
        // Event Processing
        // ============================================================================

        /**
         * @brief Poll SDL events and convert to engine events
         * @return Number of events processed
         */
        std::size_t pollEvents();

        /**
         * @brief Process single SDL event
         * @param sdlEvent SDL event to process
         * @return True if event was handled
         */
        bool processSDLEvent(const SDL_Event& sdlEvent);

        /**
         * @brief Pump SDL event queue (call from main thread)
         */
        static void pumpEvents();

        /**
         * @brief Get pending engine events
         * @param events Output vector for events
         * @param maxEvents Maximum events to retrieve
         * @return Number of events retrieved
         */
        std::size_t getPendingEvents(std::vector<InputEvent>& events,
                                     std::size_t maxEvents = 64);

        /**
         * @brief Clear all pending events
         */
        void clearPendingEvents();

        // ============================================================================
        // Device Management
        // ============================================================================

        /**
         * @brief Scan for connected devices
         * @return Number of devices found
         */
        std::size_t scanDevices();

        /**
         * @brief Get SDL controller for device
         * @param deviceId Device ID
         * @return SDL controller or nullptr
         */
        [[nodiscard]] SDL_GameController* getController(DeviceID deviceId) const;

        /**
         * @brief Get SDL joystick for device
         * @param deviceId Device ID
         * @return SDL joystick or nullptr
         */
        [[nodiscard]] SDL_Joystick* getJoystick(DeviceID deviceId) const;

        /**
         * @brief Check if device is connected
         * @param deviceId Device ID
         * @return True if connected
         */
        [[nodiscard]] bool isDeviceConnected(DeviceID deviceId) const;

        // ============================================================================
        // Mouse Control
        // ============================================================================

        /**
         * @brief Set mouse lock manager
         * @param mouseLock Mouse lock manager
         */
        void setMouseLock(MouseLock* mouseLock) noexcept {
            mouseLock_ = mouseLock;
        }

        /**
         * @brief Warp mouse to position
         * @param x X coordinate
         * @param y Y coordinate
         */
        void warpMouse(int x, int y) const;

        /**
         * @brief Set relative mouse mode
         * @param enabled Enable relative mode
         * @return True if successful
         */
        static bool setRelativeMouseMode(bool enabled);

        /**
         * @brief Set cursor visibility
         * @param visible Show cursor
         */
        static void setCursorVisibility(bool visible);

        /**
         * @brief Confine cursor to area
         * @param x X coordinate
         * @param y Y coordinate
         * @param width Area width
         * @param height Area height
         * @return True if successful
         */
        bool confineCursor(int x, int y, int width, int height) const;

        // ============================================================================
        // Text Input
        // ============================================================================

        /**
         * @brief Start text input
         */
        static void startTextInput();

        /**
         * @brief Stop text input
         */
        static void stopTextInput();

        /**
         * @brief Check if text input is active
         */
        [[nodiscard]] static bool isTextInputActive() noexcept;

        /**
         * @brief Set text input rectangle (for IME)
         * @param x X coordinate
         * @param y Y coordinate
         * @param width Rectangle width
         * @param height Rectangle height
         */
        static void setTextInputRect(int x, int y, int width, int height);

        // ============================================================================
        // Controller Haptics
        // ============================================================================

        /**
         * @brief Set controller rumble
         * @param deviceId Device ID
         * @param leftMotor Left motor intensity (0-1)
         * @param rightMotor Right motor intensity (0-1)
         * @param duration Duration in milliseconds
         * @return True if successful
         */
        bool setControllerRumble(DeviceID deviceId, float leftMotor,
                                 float rightMotor, Uint32 duration) const;

        /**
         * @brief Stop controller rumble
         * @param deviceId Device ID
         * @return True if successful
         */
        bool stopControllerRumble(DeviceID deviceId) const;

        // ============================================================================
        // Callbacks
        // ============================================================================

        /**
         * @brief Set event callback
         * @param callback Event callback function
         */
        void setEventCallback(EventCallback callback) {
            eventCallback_ = std::move(callback);
        }

        /**
         * @brief Set device callback
         * @param callback Device callback function
         */
        void setDeviceCallback(DeviceCallback callback) {
            deviceCallback_ = std::move(callback);
        }

        // ============================================================================
        // Statistics
        // ============================================================================

        struct Statistics {
            std::atomic<std::uint64_t> totalEventsProcessed{0};
            std::atomic<std::uint64_t> eventsDropped{0};
            std::atomic<std::uint32_t> controllersConnected{0};
            std::atomic<std::uint32_t> joysticksConnected{0};

            void reset() noexcept {
                totalEventsProcessed = 0;
                eventsDropped = 0;
                controllersConnected = 0;
                joysticksConnected = 0;
            }
        };

        [[nodiscard]] const Statistics& getStatistics() const noexcept {
            return stats_;
        }

        void resetStatistics() noexcept {
            stats_.reset();
        }

    private:
        // ============================================================================
        // Internal Types
        // ============================================================================

        struct ControllerInfo {
            SDL_GameController* controller;
            SDL_Joystick* joystick;
            SDL_Haptic* haptic;
            DeviceID deviceId;
            int instanceId;
            std::string name;
            bool isConnected;

            ControllerInfo() noexcept
                : controller(nullptr)
                  , joystick(nullptr)
                  , haptic(nullptr)
                  , deviceId(INVALID_DEVICE_ID)
                  , instanceId(-1)
                  , isConnected(false) {
            }
        };

        // ============================================================================
        // Member Variables
        // ============================================================================

        // Configuration
        SDLBackendConfig config_;
        bool initialized_ = false;

        // Services
        DeviceService* deviceService_;
        MouseLock* mouseLock_ = nullptr;
        InputMemoryPools* memoryPools_ = nullptr;

        // Key mapping
        std::unique_ptr<SDLKeyMap> keyMap_;

        // Event queue
        std::queue<InputEvent> eventQueue_;
        mutable std::mutex eventMutex_;

        // Controller management
        std::unordered_map<int, ControllerInfo> controllers_; // Instance ID -> Info
        std::unordered_map<DeviceID, int> deviceToInstance_; // Device ID -> Instance ID
        mutable std::mutex controllerMutex_;

        // Mouse state
        math::Vec2 lastMousePosition_;
        bool mouseInWindow_ = true;

        // Keyboard state
        KeyModifier currentModifiers_ = KeyModifier::NONE;

        // Callbacks
        EventCallback eventCallback_;
        DeviceCallback deviceCallback_;

        // Statistics
        Statistics stats_;

        // Window info (for normalization)
        int windowWidth_ = 1920;
        int windowHeight_ = 1080;
        SDL_Window* window_ = nullptr;

        // ============================================================================
        // Event Processing Methods
        // ============================================================================

        void processKeyboardEvent(const SDL_Event& event);
        void processMouseButtonEvent(const SDL_Event& event);
        void processMouseMotionEvent(const SDL_Event& event);
        void processMouseWheelEvent(const SDL_Event& event);
        void processControllerButtonEvent(const SDL_Event& event);
        void processControllerAxisEvent(const SDL_Event& event);
        void processControllerDeviceEvent(const SDL_Event& event);
        void processJoystickEvent(const SDL_Event& event);
        void processTouchEvent(const SDL_Event& event);
        void processWindowEvent(const SDL_Event& event);
        void processTextInputEvent(const SDL_Event& event);

        // ============================================================================
        // Helper Methods
        // ============================================================================

        void queueEvent(InputEvent event);
        [[nodiscard]] InputEvent* allocateEvent() const;
        void deallocateEvent(InputEvent* event) const;

        void openController(int deviceIndex);
        void closeController(int instanceId);
        [[nodiscard]] ControllerInfo* findController(int instanceId);
        [[nodiscard]] const ControllerInfo* findController(int instanceId) const;

        [[nodiscard]] math::Vec2 normalizeMousePosition(int x, int y) const noexcept;
        [[nodiscard]] static KeyModifier getSDLModifiers() noexcept;
        void updateModifiers(const SDL_Event& event);

        static bool loadControllerMappings(const std::string& filepath);
        void updateWindowSize();
    };
} // namespace engine::input
