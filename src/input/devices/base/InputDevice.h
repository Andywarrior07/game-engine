/**
 * @file InputDevice.h
 * @brief Abstract base class for all input devices
 * @author Andrés Guerrero
 * @date 12-09-2025
 *
 * Defines the interface that all input devices must implement.
 * Provides common functionality for device management and event generation.
 */

#pragma once

#include "DeviceState.h"
#include "DeviceCapabilities.h"

#include "../../core/InputTypes.h"
#include "../../core/InputEvent.h"
#include "../../core/InputSnapshot.h"

#include <string>
#include <functional>
#include <vector>
#include <queue>
#include <mutex>

namespace engine::input {
    /**
     * @brief Device initialization parameters
     */
    struct DeviceInitParams {
        DeviceID deviceId = INVALID_DEVICE_ID;
        std::string name;
        DeviceType type = DeviceType::NONE;
        PlayerID playerIndex = 0;
        bool autoAssignPlayer = true;
        bool enableRumble = true;
        bool enableMotion = false;
        void* platformHandle = nullptr; // Platform-specific handle
    };

    /**
     * @brief Abstract base class for input devices
     */
    class InputDevice {
    public:
        using EventCallback = std::function<void(const InputEvent&)>;
        using StateChangeCallback = std::function<void(DeviceConnectionStat)>;

        /**
         * @brief Virtual destructor
         */
        virtual ~InputDevice() = default;

        // ============================================================================
        // Lifecycle Management
        // ============================================================================

        /**
         * @brief Initialize the device
         * @param params Initialization parameters
         * @return True if successful
         */
        virtual bool initialize(const DeviceInitParams& params) = 0;

        /**
         * @brief Shutdown the device
         */
        virtual void shutdown() = 0;

        /**
         * @brief Reset device to default state
         */
        virtual void reset() noexcept = 0;

        // ============================================================================
        // Device State
        // ============================================================================

        /**
         * @brief Update device state
         * @param deltaTime Time since last update
         */
        virtual void update(float deltaTime) = 0;

        /**
         * @brief Poll device for events
         * @return True if events were generated
         */
        virtual bool pollEvents() = 0;

        /**
         * @brief Process raw input data
         * @param data Raw input data
         * @param size Size of data
         * @return True if processed successfully
         */
        virtual bool processRawInput(const void* data, std::size_t size) = 0;

        /**
         * @brief Get current device state
         */
        [[nodiscard]] virtual const DeviceState* getState() const noexcept = 0;

        /**
         * @brief Check if device is connected
         */
        [[nodiscard]] virtual bool isConnected() const noexcept = 0;

        /**
         * @brief Check if device is active (generating input)
         */
        [[nodiscard]] virtual bool isActive() const noexcept = 0;

        // ============================================================================
        // Device Information
        // ============================================================================

        /**
         * @brief Get device ID
         */
        [[nodiscard]] virtual DeviceID getDeviceId() const noexcept = 0;

        /**
         * @brief Get device type
         */
        [[nodiscard]] virtual DeviceType getDeviceType() const noexcept = 0;

        /**
         * @brief Get device name
         */
        [[nodiscard]] virtual const std::string& getDeviceName() const noexcept = 0;

        /**
         * @brief Get assigned player index
         */
        [[nodiscard]] virtual PlayerID getPlayerIndex() const noexcept = 0;

        /**
         * @brief Set player index
         */
        virtual void setPlayerIndex(PlayerID player) noexcept = 0;

        /**
         * @brief Get device capabilities
         */
        [[nodiscard]] virtual const DeviceCapabilities& getCapabilities() const noexcept = 0;

        // ============================================================================
        // Event Management
        // ============================================================================

        /**
         * @brief Register event callback
         */
        virtual void setEventCallback(EventCallback callback) = 0;

        /**
         * @brief Register state change callback
         */
        virtual void setStateChangeCallback(StateChangeCallback callback) = 0;

        /**
         * @brief Get pending events
         * @param events Output vector for events
         * @param maxEvents Maximum events to retrieve
         * @return Number of events retrieved
         */
        virtual std::size_t getPendingEvents(std::vector<InputEvent>& events,
                                             std::size_t maxEvents = 64) = 0;

        /**
         * @brief Clear all pending events
         */
        virtual void clearPendingEvents() noexcept = 0;

        // ============================================================================
        // Rumble/Haptics
        // ============================================================================

        /**
         * @brief Set rumble intensity
         * @param leftMotor Left motor intensity (0-1)
         * @param rightMotor Right motor intensity (0-1)
         * @param duration Duration in seconds (0 = infinite)
         * @return True if successful
         */
        virtual bool setRumble(float leftMotor, float rightMotor, float duration = 0.0f) = 0;

        /**
         * @brief Stop all rumble
         */
        virtual void stopRumble() noexcept = 0;

        /**
         * @brief Play haptic pattern
         * @param pattern Pattern identifier
         * @param intensity Intensity (0-1)
         * @return True if successful
         */
        virtual bool playHapticPattern(std::uint32_t pattern, float intensity = 1.0f) = 0;

        // ============================================================================
        // Configuration
        // ============================================================================

        /**
         * @brief Set device configuration
         * @param key Configuration key
         * @param value Configuration value
         * @return True if successful
         */
        virtual bool setConfig(const std::string& key, const std::string& value) = 0;

        /**
         * @brief Get device configuration
         * @param key Configuration key
         * @return Configuration value or empty string
         */
        [[nodiscard]] virtual std::string getConfig(const std::string& key) const = 0;

        /**
         * @brief Apply deadzone settings
         * @param settings Deadzone settings to apply
         */
        virtual void setDeadzoneSettings(const DeadzoneSettings& settings) = 0;

        /**
         * @brief Get current deadzone settings
         */
        [[nodiscard]] virtual DeadzoneSettings getDeadzoneSettings() const = 0;

        // ============================================================================
        // Platform-Specific
        // ============================================================================

        /**
         * @brief Get platform-specific handle
         * @return Platform handle or nullptr
         */
        [[nodiscard]] virtual void* getPlatformHandle() const noexcept = 0;

        /**
         * @brief Check if device owns pointer (for memory management)
         * @param ptr Pointer to check
         * @return True if device allocated this memory
         */
        [[nodiscard]] virtual bool ownsPointer(const void* ptr) const noexcept = 0;
    };

    /**
     * @brief Base implementation with common functionality
     */
    class InputDeviceBase : public InputDevice {
    protected:
        // Device identification
        DeviceID deviceId_;
        DeviceType deviceType_;
        std::string deviceName_;
        PlayerID playerIndex_;

        // Device state
        DeviceState state_;
        DeviceCapabilities capabilities_;
        DeviceConnectionStat connectionState_;

        // Event management
        EventCallback eventCallback_;
        StateChangeCallback stateChangeCallback_;
        std::queue<InputEvent> eventQueue_;
        mutable std::mutex eventMutex_;

        // Configuration
        DeadzoneSettings deadzoneSettings_;
        std::unordered_map<std::string, std::string> config_;

        // Platform handle
        void* platformHandle_;

        // Statistics
        std::atomic<std::uint64_t> totalEvents_{0};
        std::atomic<std::uint64_t> droppedEvents_{0};

    public:
        InputDeviceBase() noexcept
            : deviceId_(INVALID_DEVICE_ID)
              , deviceType_(DeviceType::NONE)
              , playerIndex_(0)
              , connectionState_(DeviceConnectionStat::DISCONNECTED)
              , platformHandle_(nullptr) {
        }

        ~InputDeviceBase() override = default;

        // TODO: Revisar esto
        // State queries
        [[nodiscard]] const DeviceState* getState() const noexcept override {
            return &state_;
        }

        [[nodiscard]] bool isConnected() const noexcept override {
            return connectionState_ == DeviceConnectionStat::CONNECTED;
        }

        // TODO: Revisar esto
        [[nodiscard]] bool isActive() const noexcept override {
            return state_.isActive.load(std::memory_order_relaxed);
        }

        // Device information
        [[nodiscard]] DeviceID getDeviceId() const noexcept override {
            return deviceId_;
        }

        [[nodiscard]] DeviceType getDeviceType() const noexcept override {
            return deviceType_;
        }

        [[nodiscard]] const std::string& getDeviceName() const noexcept override {
            return deviceName_;
        }

        [[nodiscard]] PlayerID getPlayerIndex() const noexcept override {
            return playerIndex_;
        }

        void setPlayerIndex(const PlayerID player) noexcept override {
            playerIndex_ = player;
            state_.assignedPlayer = player;
        }

        [[nodiscard]] const DeviceCapabilities& getCapabilities() const noexcept override {
            return capabilities_;
        }

        // TODO: Revisar si cumple la misma funcion que el del SDLInputBackend, ya que este no se usa
        // Event management
        void setEventCallback(EventCallback callback) override {
            eventCallback_ = std::move(callback);
        }

        void setStateChangeCallback(StateChangeCallback callback) override {
            stateChangeCallback_ = std::move(callback);
        }

        std::size_t getPendingEvents(std::vector<InputEvent>& events,
                                     const std::size_t maxEvents) override {
            std::lock_guard lock(eventMutex_);

            std::size_t count = 0;
            while (!eventQueue_.empty() && count < maxEvents) {
                events.push_back(std::move(eventQueue_.front()));
                eventQueue_.pop();
                count++;
            }

            return count;
        }

        void clearPendingEvents() noexcept override {
            std::lock_guard lock(eventMutex_);
            std::queue<InputEvent> empty;
            std::swap(eventQueue_, empty);
        }

        // Configuration
        bool setConfig(const std::string& key, const std::string& value) override {
            config_[key] = value;
            return true;
        }

        [[nodiscard]] std::string getConfig(const std::string& key) const override {
            const auto it = config_.find(key);
            return it != config_.end() ? it->second : "";
        }

        void setDeadzoneSettings(const DeadzoneSettings& settings) override {
            deadzoneSettings_ = settings;
        }

        [[nodiscard]] DeadzoneSettings getDeadzoneSettings() const override {
            return deadzoneSettings_;
        }

        // Platform-specific
        [[nodiscard]] void* getPlatformHandle() const noexcept override {
            return platformHandle_;
        }

        // TODO: Revisar esto
        [[nodiscard]] bool ownsPointer(const void* ptr) const noexcept override {
            return false; // Base implementation doesn't allocate
        }

    protected:
        /**
         * @brief Queue an event
         */
        void queueEvent(InputEvent event) {
            std::lock_guard lock(eventMutex_);

            // Set device ID if not set
            if (event.deviceId == INVALID_DEVICE_ID) {
                event.deviceId = deviceId_;
            }

            // Set timestamp if not set
            if (event.timestamp == InputTimestamp{}) {
                event.timestamp = InputTimestamp::clock::now();
            }

            eventQueue_.push(std::move(event));
            totalEvents_.fetch_add(1, std::memory_order_relaxed);

            // Fire callback if registered
            if (eventCallback_) {
                eventCallback_(eventQueue_.back());
            }

            // Update activity
            state_.updateActivity();
        }

        /**
         * @brief Update connection state
         */
        void updateConnectionState(const DeviceConnectionStat newState) {
            if (connectionState_ != newState) {
                connectionState_ = newState;
                state_.connectionState = newState;

                if (newState == DeviceConnectionStat::CONNECTED) {
                    state_.connectedTime = std::chrono::steady_clock::now();
                }

                if (stateChangeCallback_) {
                    stateChangeCallback_(newState);
                }
            }
        }
    };
} // namespace engine::input
