/**
 * @file DeviceService.cpp
 * @brief Centralized device management service implementation
 * @author Andrés Guerrero
 * @date 12-09-2025
 *
 * Implementation of thread-safe device registry and lifecycle management.
 */

#include "DeviceService.h"

#include "../keyboard/KeyboardDevice.h"
#include "../mouse/MouseDevice.h"
#include "../gamepad/GamepadDevice.h"

#include <sstream>
#include <algorithm>
#include <iomanip>

namespace engine::input {
    // ============================================================================
    // Constructor and Destructor
    // ============================================================================

    DeviceService::DeviceService(MemoryAllocator* memoryManager) noexcept
        : memoryManager_(memoryManager)
          , ownsMemoryManager_(false) {
        // Clear player device assignments
        playerDevices_.fill(INVALID_DEVICE_ID);

        // If no memory manager provided, create our own
        if (!memoryManager_) {
            // In production, this would get the global memory manager
            // For now, we'll work without it
            ownsMemoryManager_ = false;
        }
    }

    DeviceService::~DeviceService() {
        shutdown();
    }

    // ============================================================================
    // Initialization and Shutdown
    // ============================================================================

    bool DeviceService::initialize() {
        if (initialized_) {
            return false;
        }

        // Reserve space for devices
        {
            std::unique_lock lock(deviceMutex_);
            devices_.reserve(MAX_INPUT_DEVICES);
        }

        // Clear player assignments
        {
            std::lock_guard lock(playerMutex_);
            playerDevices_.fill(INVALID_DEVICE_ID);
        }

        initialized_ = true;
        return true;
    }

    void DeviceService::shutdown() {
        if (!initialized_) {
            return;
        }

        // Disconnect all devices
        {
            std::unique_lock lock(deviceMutex_);

            for (auto& [id, entry] : devices_) {
                if (entry.device) {
                    entry.device->shutdown();

                    // Notify disconnection
                    notifyDeviceEvent(id, false);
                }
            }

            devices_.clear();
        }

        // Clear player assignments
        {
            std::lock_guard lock(playerMutex_);
            playerDevices_.fill(INVALID_DEVICE_ID);
        }

        initialized_ = false;
    }

    void DeviceService::update(const float deltaTime) {
        std::shared_lock lock(deviceMutex_);

        for (auto& entry : devices_ | std::views::values) {
            if (entry.device && entry.isActive) {
                entry.device->update(deltaTime);

                // Check if device is still connected
                if (!entry.device->isConnected()) {
                    entry.isActive = false;
                }
            }
        }
    }

    // ============================================================================
    // Device Registration
    // ============================================================================

    DeviceID DeviceService::registerDevice(std::unique_ptr<InputDevice> device) {
        if (!device || !validateDevice(device.get())) {
            return INVALID_DEVICE_ID;
        }

        const DeviceID deviceId = generateDeviceId();

        {
            std::unique_lock lock(deviceMutex_);

            // Check if we've hit the device limit
            if (devices_.size() >= MAX_INPUT_DEVICES) {
                pruneInactiveDevices();

                if (devices_.size() >= MAX_INPUT_DEVICES) {
                    return INVALID_DEVICE_ID;
                }
            }

            // Create device entry
            DeviceEntry& entry = devices_[deviceId];
            entry.device = std::move(device);
            entry.id = deviceId;
            entry.type = entry.device->getDeviceType();
            entry.assignedPlayer = entry.device->getPlayerIndex();
            entry.registrationTime = std::chrono::steady_clock::now();
            entry.isActive = true;
        }

        totalDevicesCreated_.fetch_add(1, std::memory_order_relaxed);

        // Notify connection
        notifyDeviceEvent(deviceId, true);

        return deviceId;
    }

    bool DeviceService::unregisterDevice(const DeviceID deviceId) {
        if (deviceId == INVALID_DEVICE_ID) {
            return false;
        }

        std::unique_lock lock(deviceMutex_);

        const auto it = devices_.find(deviceId);
        if (it == devices_.end()) {
            return false;
        }

        // Shutdown device
        if (it->second.device) {
            it->second.device->shutdown();
        }

        // Remove from player assignment
        {
            std::lock_guard playerLock(playerMutex_);
            for (auto& playerId : playerDevices_) {
                if (playerId == deviceId) {
                    playerId = INVALID_DEVICE_ID;
                    break;
                }
            }
        }

        devices_.erase(it);
        totalDevicesDestroyed_.fetch_add(1, std::memory_order_relaxed);

        // Notify disconnection
        notifyDeviceEvent(deviceId, false);

        return true;
    }

    DeviceID DeviceService::createDevice(const DeviceDiscoveryInfo& info) {
        DeviceInitParams params;
        params.name = info.name;
        params.type = info.type;
        params.platformHandle = info.platformHandle;
        params.deviceId = generateDeviceId();

        std::unique_ptr<InputDevice> device;

        switch (info.type) {
        case DeviceType::KEYBOARD:
            device = createKeyboardDevice(params);
            break;

        case DeviceType::MOUSE:
            device = createMouseDevice(params);
            break;

        case DeviceType::GAMEPAD:
            device = createGamepadDevice(params);
            break;

        default:
            return INVALID_DEVICE_ID;
        }

        if (!device) {
            return INVALID_DEVICE_ID;
        }

        return registerDevice(std::move(device));
    }

    // ============================================================================
    // Device Queries
    // ============================================================================

    InputDevice* DeviceService::getDevice(const DeviceID deviceId) const {
        std::shared_lock lock(deviceMutex_);

        if (const auto it = devices_.find(deviceId); it != devices_.end() && it->second.device) {
            return it->second.device.get();
        }

        return nullptr;
    }

    std::vector<InputDevice*> DeviceService::getDevicesByType(const DeviceType type) const {
        std::vector<InputDevice*> result;
        std::shared_lock lock(deviceMutex_);

        for (const auto& entry : devices_ | std::views::values) {
            if (entry.type == type && entry.device && entry.isActive) {
                result.push_back(entry.device.get());
            }
        }

        return result;
    }

    InputDevice* DeviceService::getPlayerDevice(const PlayerID playerIndex) const {
        if (playerIndex >= MAX_PLAYERS) {
            return nullptr;
        }

        std::lock_guard lock(playerMutex_);
        const DeviceID deviceId = playerDevices_[playerIndex];

        if (deviceId == INVALID_DEVICE_ID) {
            return nullptr;
        }

        return getDevice(deviceId);
    }

    std::vector<InputDevice*> DeviceService::getAllDevices() const {
        std::vector<InputDevice*> result;
        std::shared_lock lock(deviceMutex_);

        result.reserve(devices_.size());
        for (const auto& entry : devices_ | std::views::values) {
            if (entry.device && entry.isActive) {
                result.push_back(entry.device.get());
            }
        }

        return result;
    }

    std::size_t DeviceService::getDeviceCount() const noexcept {
        std::shared_lock lock(deviceMutex_);
        return devices_.size();
    }

    bool DeviceService::hasDevice(const DeviceID deviceId) const noexcept {
        std::shared_lock lock(deviceMutex_);
        return devices_.contains(deviceId);
    }

    // ============================================================================
    // Player Assignment
    // ============================================================================

    bool DeviceService::assignDeviceToPlayer(const DeviceID deviceId, const PlayerID playerIndex) {
        if (playerIndex >= MAX_PLAYERS || deviceId == INVALID_DEVICE_ID) {
            return false;
        }

        // Get device
        InputDevice* device = getDevice(deviceId);
        if (!device) {
            return false;
        }

        // Update player assignment
        {
            std::lock_guard lock(playerMutex_);

            // Remove previous assignment for this device
            for (auto& playerId : playerDevices_) {
                if (playerId == deviceId) {
                    playerId = INVALID_DEVICE_ID;
                }
            }

            // Assign to new player
            playerDevices_[playerIndex] = deviceId;
        }

        // Update device
        device->setPlayerIndex(playerIndex);

        // Update entry
        {
            std::unique_lock lock(deviceMutex_);
            if (auto* entry = findDeviceEntry(deviceId)) {
                entry->assignedPlayer = playerIndex;
            }
        }

        return true;
    }

    bool DeviceService::unassignDevice(const DeviceID deviceId) {
        if (deviceId == INVALID_DEVICE_ID) {
            return false;
        }

        // Get device
        InputDevice* device = getDevice(deviceId);
        if (!device) {
            return false;
        }

        // Remove player assignment
        {
            std::lock_guard lock(playerMutex_);
            for (auto& playerId : playerDevices_) {
                if (playerId == deviceId) {
                    playerId = INVALID_DEVICE_ID;
                    break;
                }
            }
        }

        // Update device
        device->setPlayerIndex(0);

        // Update entry
        {
            std::unique_lock lock(deviceMutex_);
            if (auto* entry = findDeviceEntry(deviceId)) {
                entry->assignedPlayer = 0;
            }
        }

        return true;
    }

    PlayerID DeviceService::getNextAvailablePlayer() const noexcept {
        std::lock_guard lock(playerMutex_);

        for (PlayerID i = 0; i < MAX_PLAYERS; ++i) {
            if (playerDevices_[i] == INVALID_DEVICE_ID) {
                return i;
            }
        }

        return MAX_PLAYERS;
    }

    PlayerID DeviceService::autoAssignPlayer(const DeviceID deviceId) {
        if (const PlayerID playerIndex = getNextAvailablePlayer(); playerIndex < MAX_PLAYERS) {
            if (assignDeviceToPlayer(deviceId, playerIndex)) {
                return playerIndex;
            }
        }

        return MAX_PLAYERS;
    }

    // ============================================================================
    // Hot-Plug Support
    // ============================================================================

    DeviceID DeviceService::onDeviceConnected(const DeviceDiscoveryInfo& info) {
        if (!hotPlugEnabled_.load(std::memory_order_relaxed)) {
            return INVALID_DEVICE_ID;
        }

        // Check if device already exists (by serial number)
        if (!info.serialNumber.empty()) {
            std::shared_lock lock(deviceMutex_);
            for (const auto& [id, entry] : devices_) {
                if (entry.device &&
                    entry.device->getDeviceName() == info.name &&
                    entry.device->getConfig("serial") == info.serialNumber) {
                    // Device already registered
                    return id;
                }
            }
        }

        // Create new device
        return createDevice(info);
    }

    void DeviceService::onDeviceDisconnected(const DeviceID deviceId) {
        if (!hotPlugEnabled_.load(std::memory_order_relaxed)) {
            return;
        }

        // Mark device as inactive (don't remove immediately)
        {
            std::unique_lock lock(deviceMutex_);
            if (auto* entry = findDeviceEntry(deviceId)) {
                entry->isActive = false;
            }
        }

        // Notify disconnection
        notifyDeviceEvent(deviceId, false);
    }

    // TODO: Revisar esto
    std::size_t DeviceService::scanForDevices() {
        // This would interface with platform-specific device detection
        // For now, return 0
        return 0;
    }

    void DeviceService::setHotPlugEnabled(const bool enabled) noexcept {
        hotPlugEnabled_.store(enabled, std::memory_order_relaxed);
    }

    // ============================================================================
    // Event Callbacks
    // ============================================================================

    void DeviceService::setDeviceEventCallback(DeviceEventCallback callback) {
        deviceEventCallback_ = std::move(callback);
    }

    void DeviceService::processAllEvents(const std::function<void(const InputEvent&)>& callback) {
        if (!callback) {
            return;
        }

        std::vector<InputEvent> events;
        events.reserve(MAX_EVENTS_PER_FRAME);

        std::shared_lock lock(deviceMutex_);

        for (const auto& entry : devices_ | std::views::values) {
            if (entry.device && entry.isActive) {
                events.clear();
                entry.device->getPendingEvents(events, MAX_EVENTS_PER_FRAME);

                for (const auto& event : events) {
                    callback(event);
                }
            }
        }
    }

    // ============================================================================
    // Device Factory Methods
    // ============================================================================

    // TODO: Revisar todo esto despues de tener todos los archivos
    std::unique_ptr<KeyboardDevice> DeviceService::createKeyboardDevice(const DeviceInitParams& params) {
        // Implementation depends on KeyboardDevice class
        // For now, return nullptr
        return nullptr;
    }

    std::unique_ptr<MouseDevice> DeviceService::createMouseDevice(const DeviceInitParams& params) {
        // Implementation depends on MouseDevice class
        // For now, return nullptr
        return nullptr;
    }

    std::unique_ptr<GamepadDevice> DeviceService::createGamepadDevice(const DeviceInitParams& params) {
        // Implementation depends on GamepadDevice class
        // For now, return nullptr
        return nullptr;
    }

    // ============================================================================
    // Statistics and Debugging
    // ============================================================================

    const DeviceState* DeviceService::getDeviceStatistics(const DeviceID deviceId) const {
        const auto* device = getDevice(deviceId);

        if (!device) {
            return nullptr;
        }

        return device->getState();
    }

    void DeviceService::resetStatistics() noexcept {
        std::unique_lock lock(deviceMutex_);

        for (auto& entry : devices_ | std::views::values) {
            if (entry.device) {
                // Reset device-specific statistics
                entry.device->clearPendingEvents();
            }
        }
    }

    std::string DeviceService::generateDeviceReport() const {
        std::stringstream report;

        report << "=== Device Service Report ===" << std::endl;
        report << "Initialized: " << (initialized_ ? "Yes" : "No") << std::endl;
        report << "Hot-plug Enabled: " << (hotPlugEnabled_.load() ? "Yes" : "No") << std::endl;
        report << "Total Devices Created: " << totalDevicesCreated_.load() << std::endl;
        report << "Total Devices Destroyed: " << totalDevicesDestroyed_.load() << std::endl;
        report << std::endl;

        std::shared_lock lock(deviceMutex_);

        report << "Active Devices: " << devices_.size() << "/" << MAX_INPUT_DEVICES << std::endl;

        for (const auto& [id, entry] : devices_) {
            if (!entry.device) continue;

            report << std::endl;
            report << "Device #" << id << ":" << std::endl;
            report << "  Type: " << deviceTypeToString(entry.type) << std::endl;
            report << "  Name: " << entry.device->getDeviceName() << std::endl;
            report << "  Active: " << (entry.isActive ? "Yes" : "No") << std::endl;
            report << "  Player: " << static_cast<int>(entry.assignedPlayer) << std::endl;

            auto duration = std::chrono::steady_clock::now() - entry.registrationTime;
            const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
            report << "  Connected for: " << seconds << " seconds" << std::endl;

            // Add device state
            const auto& state = entry.device->getState();
            report << state->getStatusString();
        }

        return report.str();
    }

    // ============================================================================
    // Internal Helper Methods
    // ============================================================================

    DeviceID DeviceService::generateDeviceId() noexcept {
        return nextDeviceId_.fetch_add(1, std::memory_order_relaxed);
    }

    DeviceEntry* DeviceService::findDeviceEntry(const DeviceID deviceId) {
        const auto it = devices_.find(deviceId);
        return (it != devices_.end()) ? &it->second : nullptr;
    }

    const DeviceEntry* DeviceService::findDeviceEntry(const DeviceID deviceId) const {
        const auto it = devices_.find(deviceId);
        return (it != devices_.end()) ? &it->second : nullptr;
    }

    std::size_t DeviceService::pruneInactiveDevices() {
        std::size_t removed = 0;

        for (auto it = devices_.begin(); it != devices_.end();) {
            if (!it->second.isActive || !it->second.device) {
                it = devices_.erase(it);
                removed++;
            }
            else {
                ++it;
            }
        }

        return removed;
    }

    bool DeviceService::validateDevice(const InputDevice* device) noexcept {
        if (!device) {
            return false;
        }

        // Check if device type is valid
        if (device->getDeviceType() == DeviceType::NONE) {
            return false;
        }

        // Check if device ID is valid
        if (device->getDeviceId() == INVALID_DEVICE_ID) {
            return false;
        }

        return true;
    }

    void DeviceService::notifyDeviceEvent(const DeviceID deviceId, const bool connected) const {
        if (deviceEventCallback_) {
            deviceEventCallback_(deviceId, connected);
        }
    }
} // namespace engine::input
