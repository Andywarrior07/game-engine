/**
 * @file DeviceService.h
 * @brief Centralized device management service
 * @author Andrés Guerrero
 * @date 12-09-2025
 *
 * Manages device lifecycle, hot-plugging, registry, and player assignments.
 * Thread-safe device management with automatic resource cleanup.
 */

#pragma once

#include "InputDevice.h"
#include "DeviceState.h"

#include "../../core/InputTypes.h"

#include "../../../memory/manager/MemoryManager.h"

#include <memory>
#include <vector>
#include <unordered_map>
#include <functional>
#include <mutex>
#include <shared_mutex>
#include <atomic>

namespace engine::input {
    // Forward declarations
    class KeyboardDevice;
    class MouseDevice;
    class GamepadDevice;

    /**
     * @brief Device registry entry
     */
    struct DeviceEntry {
        std::unique_ptr<InputDevice> device;
        DeviceID id;
        DeviceType type;
        PlayerID assignedPlayer;
        std::chrono::steady_clock::time_point registrationTime;
        bool isActive;

        DeviceEntry() noexcept
            : id(INVALID_DEVICE_ID)
              , type(DeviceType::NONE)
              , assignedPlayer(0)
              , isActive(false) {
        }
    };

    /**
     * @brief Device discovery info for hot-plugging
     */
    struct DeviceDiscoveryInfo {
        DeviceType type;
        std::string name;
        std::string serialNumber;
        std::uint16_t vendorId;
        std::uint16_t productId;
        void* platformHandle;

        DeviceDiscoveryInfo() noexcept
            : type(DeviceType::NONE)
              , vendorId(0)
              , productId(0)
              , platformHandle(nullptr) {
        }
    };

    /**
     * @brief Centralized device management service
     */
    class DeviceService {
    public:
        using DeviceCallback = std::function<void(InputDevice*)>;
        using DeviceEventCallback = std::function<void(DeviceID, bool connected)>;
        using MemoryAllocator = memory::MemoryManager;

        /**
         * @brief Constructor
         * @param memoryManager Memory manager for device allocation
         */
        explicit DeviceService(MemoryAllocator* memoryManager = nullptr) noexcept;

        /**
         * @brief Destructor
         */
        ~DeviceService();

        // Disable copy
        DeviceService(const DeviceService&) = delete;
        DeviceService& operator=(const DeviceService&) = delete;

        // Enable move
        DeviceService(DeviceService&&) noexcept = default;
        DeviceService& operator=(DeviceService&&) noexcept = default;

        // ============================================================================
        // Initialization and Shutdown
        // ============================================================================

        /**
         * @brief Initialize the device service
         * @return True if successful
         */
        bool initialize();

        /**
         * @brief Shutdown the device service
         */
        void shutdown();

        /**
         * @brief Update all devices
         * @param deltaTime Time since last update
         */
        void update(float deltaTime);

        // ============================================================================
        // Device Registration
        // ============================================================================

        /**
         * @brief Register a new device
         * @param device Device to register (ownership transferred)
         * @return Assigned device ID or INVALID_DEVICE_ID on failure
         */
        DeviceID registerDevice(std::unique_ptr<InputDevice> device);

        /**
         * @brief Unregister a device
         * @param deviceId Device to unregister
         * @return True if successful
         */
        bool unregisterDevice(DeviceID deviceId);

        /**
         * @brief Create and register a device
         * @param info Device discovery information
         * @return Device ID or INVALID_DEVICE_ID on failure
         */
        DeviceID createDevice(const DeviceDiscoveryInfo& info);

        // ============================================================================
        // Device Queries
        // ============================================================================

        /**
         * @brief Get device by ID
         * @param deviceId Device ID
         * @return Device pointer or nullptr
         */
        [[nodiscard]] InputDevice* getDevice(DeviceID deviceId) const;

        /**
         * @brief Get all devices of a specific type
         * @param type Device type
         * @return Vector of device pointers
         */
        [[nodiscard]] std::vector<InputDevice*> getDevicesByType(DeviceType type) const;

        /**
         * @brief Get device for player
         * @param playerIndex Player index
         * @return Primary device for player or nullptr
         */
        [[nodiscard]] InputDevice* getPlayerDevice(PlayerID playerIndex) const;

        /**
         * @brief Get all registered devices
         * @return Vector of all device pointers
         */
        [[nodiscard]] std::vector<InputDevice*> getAllDevices() const;

        /**
         * @brief Get number of registered devices
         */
        [[nodiscard]] std::size_t getDeviceCount() const noexcept;

        /**
         * @brief Check if device exists
         * @param deviceId Device ID to check
         * @return True if device is registered
         */
        [[nodiscard]] bool hasDevice(DeviceID deviceId) const noexcept;

        // ============================================================================
        // Player Assignment
        // ============================================================================

        /**
         * @brief Assign device to player
         * @param deviceId Device ID
         * @param playerIndex Player index
         * @return True if successful
         */
        bool assignDeviceToPlayer(DeviceID deviceId, PlayerID playerIndex);

        /**
         * @brief Unassign device from player
         * @param deviceId Device ID
         * @return True if successful
         */
        bool unassignDevice(DeviceID deviceId);

        /**
         * @brief Get next available player slot
         * @return Player index or MAX_PLAYERS if none available
         */
        [[nodiscard]] PlayerID getNextAvailablePlayer() const noexcept;

        /**
         * @brief Auto-assign device to next available player
         * @param deviceId Device ID
         * @return Assigned player index or MAX_PLAYERS on failure
         */
        PlayerID autoAssignPlayer(DeviceID deviceId);

        // ============================================================================
        // Hot-Plug Support
        // ============================================================================

        /**
         * @brief Handle device connection
         * @param info Device discovery information
         * @return Device ID or INVALID_DEVICE_ID on failure
         */
        DeviceID onDeviceConnected(const DeviceDiscoveryInfo& info);

        /**
         * @brief Handle device disconnection
         * @param deviceId Device that disconnected
         */
        void onDeviceDisconnected(DeviceID deviceId);

        /**
         * @brief Scan for new devices
         * @return Number of new devices found
         */
        std::size_t scanForDevices();

        /**
         * @brief Enable/disable hot-plug detection
         * @param enabled Enable hot-plug
         */
        void setHotPlugEnabled(bool enabled) noexcept;

        // ============================================================================
        // Event Callbacks
        // ============================================================================

        /**
         * @brief Register device connection/disconnection callback
         * @param callback Callback function
         */
        void setDeviceEventCallback(DeviceEventCallback callback);

        /**
         * @brief Process events from all devices
         * @param callback Callback for each event
         */
        void processAllEvents(const std::function<void(const InputEvent&)>& callback);

        // ============================================================================
        // Device Factory Methods
        // ============================================================================

        /**
         * @brief Create keyboard device
         * @param params Initialization parameters
         * @return Keyboard device or nullptr on failure
         */
        std::unique_ptr<KeyboardDevice> createKeyboardDevice(const DeviceInitParams& params);

        /**
         * @brief Create mouse device
         * @param params Initialization parameters
         * @return Mouse device or nullptr on failure
         */
        std::unique_ptr<MouseDevice> createMouseDevice(const DeviceInitParams& params);

        /**
         * @brief Create gamepad device
         * @param params Initialization parameters
         * @return Gamepad device or nullptr on failure
         */
        std::unique_ptr<GamepadDevice> createGamepadDevice(const DeviceInitParams& params);

        // ============================================================================
        // Statistics and Debugging
        // ============================================================================

        /**
         * @brief Get device statistics
         * @param deviceId Device ID
         * @return Device state or empty state if not found
         */
        [[nodiscard]] const DeviceState* getDeviceStatistics(DeviceID deviceId) const;

        /**
         * @brief Reset all device statistics
         */
        void resetStatistics() noexcept;

        /**
         * @brief Generate device report
         * @return String containing device information
         */
        [[nodiscard]] std::string generateDeviceReport() const;

    private:
        // Device registry
        mutable std::shared_mutex deviceMutex_;
        std::unordered_map<DeviceID, DeviceEntry> devices_;
        std::atomic<DeviceID> nextDeviceId_{1};

        // Player assignments
        std::array<DeviceID, MAX_PLAYERS> playerDevices_;
        mutable std::mutex playerMutex_;

        // Memory management
        MemoryAllocator* memoryManager_;
        bool ownsMemoryManager_;

        // Hot-plug support
        std::atomic<bool> hotPlugEnabled_{true};
        DeviceEventCallback deviceEventCallback_;

        // Statistics
        std::atomic<std::uint32_t> totalDevicesCreated_{0};
        std::atomic<std::uint32_t> totalDevicesDestroyed_{0};

        // State
        bool initialized_{false};

        // ============================================================================
        // Internal Helper Methods
        // ============================================================================

        /**
         * @brief Generate next device ID
         * @return Unique device ID
         */
        [[nodiscard]] DeviceID generateDeviceId() noexcept;

        /**
         * @brief Find device entry by ID
         * @param deviceId Device ID
         * @return Device entry pointer or nullptr
         */
        [[nodiscard]] DeviceEntry* findDeviceEntry(DeviceID deviceId);
        [[nodiscard]] const DeviceEntry* findDeviceEntry(DeviceID deviceId) const;

        /**
         * @brief Remove inactive devices
         * @return Number of devices removed
         */
        std::size_t pruneInactiveDevices();

        /**
         * @brief Validate device before registration
         * @param device Device to validate
         * @return True if valid
         */
        [[nodiscard]] static bool validateDevice(const InputDevice* device) noexcept;

        /**
         * @brief Notify device event
         * @param deviceId Device ID
         * @param connected True if connected, false if disconnected
         */
        void notifyDeviceEvent(DeviceID deviceId, bool connected) const;
    };
} // namespace engine::input
