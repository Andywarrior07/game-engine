/**
 * @file DeviceConnectionState.h
 * @brief Abstract base class for all input devices
 * @author Andrés Guerrero
 * @date 16-09-2025
 */

#pragma once

#include <cstdint>

namespace engine::input {
    /**
     * @brief Device connection state
     */
    enum class DeviceConnectionStat : std::uint8_t {
        DISCONNECTED = 0,
        CONNECTING,
        CONNECTED,
        DISCONNECTING,
        ERROR_STATE,
        SUSPENDED
    };
} // namespace engine::input
