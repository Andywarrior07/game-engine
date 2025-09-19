/**
 * @file DeviceState.h
 * @brief Generic device state tracking
 * @author Game Engine Team
 * @date 2024
 *
 * Manages common state information for all input devices.
 */

#pragma once

#include "DeviceConnectionStat.h"

#include "../../core/InputTypes.h"
#include "../../core/InputConstants.h"

#include <chrono>
#include <atomic>
#include <string>


namespace engine::input {
    /**
     * @brief Common device state information
     *
     * Base state that all devices share, including connection status,
     * timing information, and error tracking.
     */
    struct DeviceState {
        // Device identification
        DeviceID id;
        DeviceType type;
        std::string name;
        std::string serialNumber;

        // Connection state
        DeviceConnectionStat connectionState;
        std::chrono::steady_clock::time_point connectedTime;
        std::chrono::steady_clock::time_point lastActivityTime;
        std::atomic<bool> isActive;

        // Player assignment
        PlayerID assignedPlayer;
        bool isPlayerControlled;

        // Device properties
        std::uint16_t vendorId;
        std::uint16_t productId;
        std::uint16_t versionNumber;

        // Battery information (if applicable)
        float batteryLevel; // 0-1, -1 if not available
        bool isCharging;
        bool isWireless;

        // Error tracking
        std::uint32_t errorCount;
        InputErrorCode lastError;
        std::string lastErrorMessage;

        // Statistics
        std::uint64_t totalEvents;
        std::uint64_t droppedEvents;
        std::uint64_t totalBytes; // For bandwidth tracking

        // Timing
        float pollRate; // Actual polling rate in Hz
        float latency; // Average latency in ms
        float updateFrequency; // How often device sends updates

        /**
         * @brief Default constructor
         */
        DeviceState() noexcept
            : id()
              , type(DeviceType::NONE)
              , connectionState(DeviceConnectionStat::DISCONNECTED)
              , isActive(false)
              , assignedPlayer(0)
              , isPlayerControlled(false)
              , vendorId(0)
              , productId(0)
              , versionNumber(0)
              , batteryLevel(-1.0f)
              , isCharging(false)
              , isWireless(false)
              , errorCount(0)
              , lastError(InputErrorCode::SUCCESS)
              , totalEvents(0)
              , droppedEvents(0)
              , totalBytes(0)
              , pollRate(0.0f)
              , latency(0.0f)
              , updateFrequency(0.0f) {
        }

        /**
         * @brief Check if device is connected
         */
        bool isConnected() const noexcept {
            return connectionState == DeviceConnectionStat::CONNECTED;
        }

        /**
         * @brief Check if device has battery
         */
        bool hasBattery() const noexcept {
            return batteryLevel >= 0.0f;
        }

        /**
         * @brief Get connection duration
         */
        std::chrono::duration<float> getConnectionDuration() const noexcept {
            if (!isConnected()) return std::chrono::duration<float>(0);
            return std::chrono::steady_clock::now() - connectedTime;
        }

        /**
         * @brief Get time since last activity
         */
        std::chrono::duration<float> getIdleTime() const noexcept {
            return std::chrono::steady_clock::now() - lastActivityTime;
        }

        /**
         * @brief Update activity timestamp
         */
        void updateActivity() noexcept {
            lastActivityTime = std::chrono::steady_clock::now();
            isActive.store(true, std::memory_order_relaxed);
        }

        /**
         * @brief Mark as inactive
         */
        void markInactive() noexcept {
            isActive.store(false, std::memory_order_relaxed);
        }

        /**
         * @brief Record error
         */
        void recordError(const InputErrorCode error, const std::string& message = "") {
            errorCount++;
            lastError = error;
            lastErrorMessage = message;
        }

        /**
         * @brief Clear error state
         */
        void clearError() noexcept {
            lastError = InputErrorCode::SUCCESS;
            lastErrorMessage.clear();
        }

        /**
         * @brief Get device status string
         */
        std::string getStatusString() const {
            std::string status = "Device: " + name + "\n";
            status += "  ID: " + std::to_string(id) + "\n";
            status += "  Type: " + std::string(deviceTypeToString(type)) + "\n";
            status += "  Connected: " + std::string(isConnected() ? "Yes" : "No") + "\n";

            if (isConnected()) {
                auto duration = getConnectionDuration();
                status += "  Connection Time: " + std::to_string(duration.count()) + "s\n";

                auto idle = getIdleTime();
                status += "  Idle Time: " + std::to_string(idle.count()) + "s\n";
            }

            if (hasBattery()) {
                status += "  Battery: " + std::to_string(static_cast<int>(batteryLevel * 100)) + "%";
                if (isCharging) status += " (Charging)";
                status += "\n";
            }

            if (errorCount > 0) {
                status += "  Errors: " + std::to_string(errorCount) + "\n";
                status += "  Last Error: " + std::string(getErrorString(lastError)) + "\n";
            }

            status += "  Total Events: " + std::to_string(totalEvents) + "\n";
            if (droppedEvents > 0) {
                status += "  Dropped Events: " + std::to_string(droppedEvents) + "\n";
            }

            return status;
        }

        /**
         * @brief Reset statistics
         */
        void resetStatistics() noexcept {
            totalEvents = 0;
            droppedEvents = 0;
            totalBytes = 0;
            errorCount = 0;
            clearError();
        }
    };
} // namespace engine::input
