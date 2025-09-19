/**
 * @file DeadzoneStage.cpp
 * @brief Deadzone processing stage implementation
 * @author Andrés Guerrero
 * @date 12-09-2025
 */

#include "DeadzoneStage.h"

#include "../../utils/InputUtils.h"

namespace engine::input::processing {
    DeadzoneStage::DeadzoneStage() noexcept = default;

    DeadzoneStage::DeadzoneStage(const DeadzoneConfig& config) noexcept
        : config_(config) {
    }

    std::size_t DeadzoneStage::process(std::vector<InputEvent>& events, const float deltaTime) {
        if (!enabled_ || events.empty()) {
            return 0;
        }

        std::size_t processed = 0;

        // Process events in-place for cache efficiency
        for (auto& event : events) {
            if (processEvent(event)) {
                processed++;
            }
        }

        return processed;
    }

    bool DeadzoneStage::processEvent(InputEvent& event) {
        if (!enabled_ || event.consumed) {
            return false;
        }

        bool processed = false;

        switch (event.type) {
        case InputEventType::GAMEPAD_AXIS_MOVED: {
            if (auto* data = event.getMutableEventData<GamepadAxisEventData>()) {
                processed = processGamepadAxis(*data, event.deviceId);
            }
            break;
        }
        case InputEventType::MOUSE_WHEEL: {
            if (auto* data = event.getMutableEventData<MouseWheelEventData>()) {
                processed = processMouseWheel(*data);
            }
            break;
        }
        default:
            // Event type not handled by deadzone processing
            return false;
        }

        if (processed) {
            stats_.totalProcessed++;
        }

        return processed;
    }

    void DeadzoneStage::setConfig(const DeadzoneConfig& config) noexcept {
        config_ = config;

        // Clear cached values when configuration changes
        cachedValues_.clear();
    }

    void DeadzoneStage::setDeviceDeadzone(const DeviceID deviceId, const DeadzoneSettings& settings) {
        config_.deviceOverrides[deviceId] = settings;
        config_.usePerDeviceSettings = true;
    }

    void DeadzoneStage::removeDeviceDeadzone(const DeviceID deviceId) {
        config_.deviceOverrides.erase(deviceId);

        if (config_.deviceOverrides.empty()) {
            config_.usePerDeviceSettings = false;
        }
    }

    void DeadzoneStage::setStickDeadzone(const DeadzoneSettings& settings, const bool leftStick) noexcept {
        if (leftStick) {
            config_.leftStickDeadzone = settings;
        }
        else {
            config_.rightStickDeadzone = settings;
        }
    }

    void DeadzoneStage::setTriggerDeadzone(const DeadzoneSettings& settings, const bool leftTrigger) noexcept {
        if (leftTrigger) {
            config_.leftTriggerDeadzone = settings;
        }
        else {
            config_.rightTriggerDeadzone = settings;
        }
    }

    void DeadzoneStage::reset() noexcept {
        config_ = DeadzoneConfig{};
        stats_.reset();
        cachedValues_.clear();
    }

    // ============================================================================
    // Private Implementation
    // ============================================================================

    bool DeadzoneStage::processGamepadAxis(GamepadAxisEventData& data, const DeviceID deviceId) {
        const DeadzoneSettings settings = getDeviceDeadzone(deviceId, data.axis);
        const std::uint64_t cacheKey = generateCacheKey(deviceId, data.axis);

        // Get or create cached value entry
        auto& cached = cachedValues_[cacheKey];
        const auto currentTime = std::chrono::steady_clock::now();

        // Calculate delta time for smooth transitions
        const float deltaTime = cached.lastUpdate != std::chrono::steady_clock::time_point{}
                                    ? std::chrono::duration<float>(currentTime - cached.lastUpdate).count()
                                    : 0.016f; // Default to ~60fps

        cached.lastUpdate = currentTime;

        switch (data.axis) {
        case GamepadAxis::LEFT_STICK_X:
        case GamepadAxis::LEFT_STICK_Y: {
            // Reconstruct full stick vector for proper radial deadzone
            math::Vec2 stickValue = cached.lastStickValue;

            if (data.axis == GamepadAxis::LEFT_STICK_X) {
                stickValue.x = data.value;
            }
            else {
                stickValue.y = data.value;
            }

            const math::Vec2 processed = applyStickDeadzone(stickValue, settings, config_.stickDeadzoneType);

            // Update the specific axis value
            const float newValue = (data.axis == GamepadAxis::LEFT_STICK_X) ? processed.x : processed.y;

            // Check if value was filtered to zero
            if (std::abs(data.value) > math::EPSILON && std::abs(newValue) < math::EPSILON) {
                stats_.eventsFiltered++;
            }

            data.value = newValue;
            cached.lastStickValue = processed;
            stats_.sticksProcessed++;
            break;
        }
        case GamepadAxis::RIGHT_STICK_X:
        case GamepadAxis::RIGHT_STICK_Y: {
            // Same logic for right stick
            math::Vec2 stickValue = cached.lastStickValue;

            if (data.axis == GamepadAxis::RIGHT_STICK_X) {
                stickValue.x = data.value;
            }
            else {
                stickValue.y = data.value;
            }

            const DeadzoneSettings rightSettings = config_.usePerDeviceSettings
                                                       ? settings
                                                       : config_.rightStickDeadzone;

            const math::Vec2 processed = applyStickDeadzone(stickValue, rightSettings, config_.stickDeadzoneType);

            const float newValue = (data.axis == GamepadAxis::RIGHT_STICK_X) ? processed.x : processed.y;

            if (std::abs(data.value) > math::EPSILON && std::abs(newValue) < math::EPSILON) {
                stats_.eventsFiltered++;
            }

            data.value = newValue;
            cached.lastStickValue = processed;
            stats_.sticksProcessed++;
            break;
        }
        case GamepadAxis::LEFT_TRIGGER:
        case GamepadAxis::RIGHT_TRIGGER: {
            const DeadzoneSettings triggerSettings = (data.axis == GamepadAxis::LEFT_TRIGGER)
                                                         ? (config_.usePerDeviceSettings
                                                                ? settings
                                                                : config_.leftTriggerDeadzone)
                                                         : (config_.usePerDeviceSettings
                                                                ? settings
                                                                : config_.rightTriggerDeadzone);

            const float processed = applyTriggerDeadzone(data.value, triggerSettings);

            // Apply smooth transition to avoid sudden jumps
            const bool wasInDeadzone = std::abs(cached.lastTriggerValue) < math::EPSILON;
            const bool isInDeadzone = std::abs(processed) < math::EPSILON;

            float finalValue = processed;
            if (wasInDeadzone != isInDeadzone) {
                finalValue = smoothDeadzoneTransition(processed, cached.lastTriggerValue, wasInDeadzone, deltaTime);
            }

            if (std::abs(data.value) > math::EPSILON && std::abs(finalValue) < math::EPSILON) {
                stats_.eventsFiltered++;
            }

            data.value = finalValue;
            cached.lastTriggerValue = finalValue;
            cached.wasInDeadzone = isInDeadzone;
            stats_.triggersProcessed++;
            break;
        }
        default:
            return false;
        }

        return true;
    }

    bool DeadzoneStage::processMouseWheel(MouseWheelEventData& data) {
        const float threshold = config_.mouseWheelDeadzone;
        bool filtered = false;

        // Apply deadzone to both X and Y components
        if (std::abs(data.delta.x) < threshold) {
            if (std::abs(data.delta.x) > math::EPSILON) {
                filtered = true;
            }
            data.delta.x = 0.0f;
        }

        if (std::abs(data.delta.y) < threshold) {
            if (std::abs(data.delta.y) > math::EPSILON) {
                filtered = true;
            }
            data.delta.y = 0.0f;
        }

        if (filtered) {
            stats_.eventsFiltered++;
        }

        stats_.wheelsProcessed++;
        return true;
    }

    math::Vec2 DeadzoneStage::applyStickDeadzone(const math::Vec2& input,
                                                 const DeadzoneSettings& settings,
                                                 const DeadzoneConfig::DeadzoneType type) {
        using namespace engine::input::utils;

        switch (type) {
        case DeadzoneConfig::DeadzoneType::RADIAL:
            return applyRadialDeadzone(input, settings.innerDeadzone, settings.outerDeadzone);

        case DeadzoneConfig::DeadzoneType::AXIAL:
            return applyAxialDeadzone(input, settings.innerDeadzone, settings.innerDeadzone);

        case DeadzoneConfig::DeadzoneType::HYBRID:
            return applyHybridDeadzone(input, settings);

        case DeadzoneConfig::DeadzoneType::SCALED: {
            // Scaled radial preserves input direction while scaling magnitude
            const float magnitude = glm::length(input);
            if (magnitude < settings.innerDeadzone) {
                return math::Vec2(0.0f, 0.0f);
            }

            if (magnitude > settings.outerDeadzone) {
                return input; // Keep original direction and magnitude
            }

            // Scale magnitude to [0, 1] range while preserving direction
            const float scaledMagnitude = (magnitude - settings.innerDeadzone) /
                (settings.outerDeadzone - settings.innerDeadzone);
            return glm::normalize(input) * scaledMagnitude;
        }
        default:
            return applyRadialDeadzone(input, settings.innerDeadzone, settings.outerDeadzone);
        }
    }

    float DeadzoneStage::applyTriggerDeadzone(const float input, const DeadzoneSettings& settings) {
        const float absInput = std::abs(input);

        if (absInput < settings.innerDeadzone) {
            return 0.0f;
        }

        if (absInput > settings.outerDeadzone) {
            return input > 0 ? 1.0f : -1.0f;
        }

        // Linear scaling from deadzone to full range
        const float scaledValue = (absInput - settings.innerDeadzone) /
            (settings.outerDeadzone - settings.innerDeadzone);

        return scaledValue * (input > 0 ? 1.0f : -1.0f);
    }

    DeadzoneSettings DeadzoneStage::getDeviceDeadzone(const DeviceID deviceId, const GamepadAxis axis) const {
        // Check for device-specific override first
        if (config_.usePerDeviceSettings) {
            if (const auto it = config_.deviceOverrides.find(deviceId); it != config_.deviceOverrides.end()) {
                return it->second;
            }
        }

        // Return axis-specific default settings
        switch (axis) {
        case GamepadAxis::LEFT_STICK_X:
        case GamepadAxis::LEFT_STICK_Y:
            return config_.leftStickDeadzone;

        case GamepadAxis::RIGHT_STICK_X:
        case GamepadAxis::RIGHT_STICK_Y:
            return config_.rightStickDeadzone;

        case GamepadAxis::LEFT_TRIGGER:
            return config_.leftTriggerDeadzone;

        case GamepadAxis::RIGHT_TRIGGER:
            return config_.rightTriggerDeadzone;

        default:
            return config_.leftStickDeadzone; // Default fallback
        }
    }

    std::uint64_t DeadzoneStage::generateCacheKey(const DeviceID deviceId, const GamepadAxis axis) noexcept {
        // Combine device ID and axis into unique key
        return (static_cast<std::uint64_t>(deviceId) << 32) | static_cast<std::uint64_t>(axis);
    }

    float DeadzoneStage::smoothDeadzoneTransition(const float current, const float previous,
                                                  const bool wasInDeadzone, const float deltaTime) {
        // Smooth transition when entering/exiting deadzone to avoid jarring jumps

        if (wasInDeadzone && std::abs(current) > math::EPSILON) {
            constexpr float TRANSITION_SPEED = 10.0f;
            // Entering active zone - smoothly ramp up from zero
            const float maxChange = TRANSITION_SPEED * deltaTime;

            if (const float change = std::abs(current - previous); change > maxChange) {
                return previous + (current > previous ? maxChange : -maxChange);
            }
        }

        return current;
    }
} // namespace engine::input::processing
