/**
 * @file SensitivityStage.cpp
 * @brief Sensitivity and response curve processing stage implementation
 * @author Andrés Guerrero
 * @date 12-09-2025
 */

#include "SensitivityStage.h"

#include "../../utils/InputUtils.h"

#include <algorithm>
#include <cmath>

namespace engine::input::processing {
    SensitivityStage::SensitivityStage() noexcept {
        // Initialize curve caches with default settings
        mouseCurveCache_.type = ResponseCurve::LINEAR;
        mouseCurveCache_.strength = 0.0f;
        mouseCurveCache_.regenerate();

        stickCurveCache_.type = ResponseCurve::LINEAR;
        stickCurveCache_.strength = 0.0f;
        stickCurveCache_.regenerate();

        triggerCurveCache_.type = ResponseCurve::LINEAR;
        triggerCurveCache_.strength = 0.0f;
        triggerCurveCache_.regenerate();
    }

    SensitivityStage::SensitivityStage(const SensitivityConfig& config) noexcept
        : config_(config) {
        // Initialize curve caches with config settings
        mouseCurveCache_.type = config_.mouseCurve;
        mouseCurveCache_.strength = config_.mouseCurveStrength;
        mouseCurveCache_.regenerate();

        stickCurveCache_.type = config_.stickCurve;
        stickCurveCache_.strength = config_.stickCurveStrength;
        stickCurveCache_.regenerate();

        triggerCurveCache_.type = config_.triggerCurve;
        triggerCurveCache_.strength = config_.triggerCurveStrength;
        triggerCurveCache_.regenerate();
    }

    // TODO: Revisar porque no se usa deltatime
    std::size_t SensitivityStage::process(std::vector<InputEvent>& events, const float deltaTime) {
        if (!enabled_ || events.empty()) {
            return 0;
        }

        std::size_t processed = 0;

        // Process events in-place for optimal cache performance
        for (auto& event : events) {
            if (processEvent(event)) {
                processed++;
            }
        }

        return processed;
    }

    bool SensitivityStage::processEvent(InputEvent& event) {
        if (!enabled_ || event.consumed) {
            return false;
        }

        bool processed = false;

        switch (event.type) {
        case InputEventType::MOUSE_MOVED: {
            if (auto* data = event.getMutableEventData<MouseMotionEventData>()) {
                processed = processMouseMotion(*data, event.deviceId);
            }
            break;
        }
        case InputEventType::GAMEPAD_AXIS_MOVED:
        case InputEventType::GAMEPAD_TRIGGER_MOVED: {
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
            // Event type not handled by sensitivity processing
            return false;
        }

        if (processed) {
            stats_.totalProcessed++;
        }

        return processed;
    }

    void SensitivityStage::setConfig(const SensitivityConfig& config) noexcept {
        config_ = config;

        // Regenerate curve caches when configuration changes
        if (mouseCurveCache_.type != config_.mouseCurve ||
            mouseCurveCache_.strength != config_.mouseCurveStrength) {
            mouseCurveCache_.type = config_.mouseCurve;
            mouseCurveCache_.strength = config_.mouseCurveStrength;
            mouseCurveCache_.regenerate();
        }

        if (stickCurveCache_.type != config_.stickCurve ||
            stickCurveCache_.strength != config_.stickCurveStrength) {
            stickCurveCache_.type = config_.stickCurve;
            stickCurveCache_.strength = config_.stickCurveStrength;
            stickCurveCache_.regenerate();
        }

        if (triggerCurveCache_.type != config_.triggerCurve ||
            triggerCurveCache_.strength != config_.triggerCurveStrength) {
            triggerCurveCache_.type = config_.triggerCurve;
            triggerCurveCache_.strength = config_.triggerCurveStrength;
            triggerCurveCache_.regenerate();
        }
    }

    void SensitivityStage::setGlobalSensitivity(const float sensitivity) noexcept {
        config_.globalSensitivity = math::clamp(sensitivity, 0.01f, 10.0f);
    }

    void SensitivityStage::setDeviceSensitivity(const DeviceID deviceId, const float sensitivity) {
        config_.deviceSensitivity[deviceId] = math::clamp(sensitivity, 0.01f, 10.0f);
    }

    void SensitivityStage::setMouseSensitivity(const float sensitivity, const bool applyToX,
                                               const bool applyToY) noexcept {
        const float clampedSensitivity = math::clamp(sensitivity, 0.01f, 10.0f);

        if (applyToX && applyToY) {
            config_.mouseSensitivity = clampedSensitivity;
            config_.mouseAxis.x = clampedSensitivity;
            config_.mouseAxis.y = clampedSensitivity;
        }
        else if (applyToX) {
            config_.mouseAxis.x = clampedSensitivity;
        }
        else if (applyToY) {
            config_.mouseAxis.y = clampedSensitivity;
        }
    }

    void SensitivityStage::setStickSensitivity(const float sensitivity, const bool leftStick) noexcept {
        const float clampedSensitivity = math::clamp(sensitivity, 0.01f, 10.0f);

        config_.stickSensitivity = clampedSensitivity;

        if (leftStick) {
            config_.leftStickAxis.x = clampedSensitivity;
            config_.leftStickAxis.y = clampedSensitivity;
        }
        else {
            config_.rightStickAxis.x = clampedSensitivity;
            config_.rightStickAxis.y = clampedSensitivity;
        }
    }

    void SensitivityStage::setResponseCurve(const ResponseCurve curve, const float strength,
                                            const DeviceType deviceType) {
        const float clampedStrength = math::clamp(strength, 0.0f, 1.0f);

        switch (deviceType) {
        case DeviceType::MOUSE:
            config_.mouseCurve = curve;
            config_.mouseCurveStrength = clampedStrength;
            mouseCurveCache_.type = curve;
            mouseCurveCache_.strength = clampedStrength;
            mouseCurveCache_.regenerate();
            break;

        case DeviceType::GAMEPAD:
            config_.stickCurve = curve;
            config_.stickCurveStrength = clampedStrength;
            config_.triggerCurve = curve;
            config_.triggerCurveStrength = clampedStrength;

            stickCurveCache_.type = curve;
            stickCurveCache_.strength = clampedStrength;
            stickCurveCache_.regenerate();

            triggerCurveCache_.type = curve;
            triggerCurveCache_.strength = clampedStrength;
            triggerCurveCache_.regenerate();
            break;

        default:
            // Unsupported device type
            break;
        }
    }

    void SensitivityStage::registerCustomCurve(const std::string& name, SensitivityConfig::CustomCurveFunc func) {
        config_.customCurves[name] = std::move(func);
    }

    void SensitivityStage::reset() noexcept {
        config_ = SensitivityConfig{};
        stats_.reset();
        velocityTrackers_.clear();

        // Reset curve caches to default
        mouseCurveCache_ = CurveCache{};
        stickCurveCache_ = CurveCache{};
        triggerCurveCache_ = CurveCache{};
    }

    // ============================================================================
    // Private Implementation
    // ============================================================================

    // TODO: Revisar esto, quizá dejarla como void
    bool SensitivityStage::processMouseMotion(MouseMotionEventData& data, const DeviceID deviceId) {
        // Get device-specific sensitivity or use global
        const float deviceSensitivity = getDeviceSensitivity(deviceId);
        const float combinedSensitivity = config_.globalSensitivity * config_.mouseSensitivity * deviceSensitivity;

        // Apply axis-specific sensitivity and inversion
        math::Vec2 processedDelta = applySensitivity(data.delta, config_.mouseAxis);
        processedDelta *= combinedSensitivity;

        // Apply response curve
        if (config_.mouseCurve != ResponseCurve::LINEAR && config_.mouseCurveStrength > math::EPSILON) {
            processedDelta.x = applyResponseCurve(processedDelta.x, mouseCurveCache_);
            processedDelta.y = applyResponseCurve(processedDelta.y, mouseCurveCache_);
        }

        // Apply mouse acceleration if enabled
        if (config_.enableMouseAcceleration) {
            updateVelocity(deviceId, data.position, 0.016f); // Assume 60fps for velocity calculation

            if (const auto it = velocityTrackers_.find(deviceId); it != velocityTrackers_.end()) {
                processedDelta = applyMouseAcceleration(processedDelta, it->second.smoothedVelocity, 0.016f);
                stats_.accelerationApplied++;
            }
        }

        // Update event data
        data.delta = processedDelta;
        data.normalizedDelta = processedDelta; // Normalized delta same as delta for mouse

        stats_.mouseEventsProcessed++;
        stats_.curvesApplied += (config_.mouseCurve != ResponseCurve::LINEAR) ? 1 : 0;

        return true;
    }

    bool SensitivityStage::processGamepadAxis(GamepadAxisEventData& data, const DeviceID deviceId) {
        const float deviceSensitivity = getDeviceSensitivity(deviceId);
        float processedValue = data.value;
        bool isTrigger = false;

        // Determine if this is a trigger or stick axis
        switch (data.axis) {
        case GamepadAxis::LEFT_TRIGGER:
        case GamepadAxis::RIGHT_TRIGGER:
            isTrigger = true;
            processedValue = applySensitivity(processedValue,
                                              config_.globalSensitivity * config_.triggerSensitivity *
                                              deviceSensitivity);
            break;

        case GamepadAxis::LEFT_STICK_X:
        case GamepadAxis::LEFT_STICK_Y:
            processedValue = applySensitivity(processedValue,
                                              config_.globalSensitivity * config_.stickSensitivity * deviceSensitivity);

            // Apply axis-specific inversion for left stick
            if (data.axis == GamepadAxis::LEFT_STICK_Y && config_.leftStickAxis.invertY) {
                processedValue = -processedValue;
            }
            if (data.axis == GamepadAxis::LEFT_STICK_X && config_.leftStickAxis.invertX) {
                processedValue = -processedValue;
            }
            break;

        case GamepadAxis::RIGHT_STICK_X:
        case GamepadAxis::RIGHT_STICK_Y:
            processedValue = applySensitivity(processedValue,
                                              config_.globalSensitivity * config_.stickSensitivity * deviceSensitivity);

            // Apply axis-specific inversion for right stick
            if (data.axis == GamepadAxis::RIGHT_STICK_Y && config_.rightStickAxis.invertY) {
                processedValue = -processedValue;
            }
            if (data.axis == GamepadAxis::RIGHT_STICK_X && config_.rightStickAxis.invertX) {
                processedValue = -processedValue;
            }
            break;

        default:
            return false;
        }

        // Apply response curve based on axis type
        if (isTrigger) {
            if (config_.triggerCurve != ResponseCurve::LINEAR && config_.triggerCurveStrength > math::EPSILON) {
                processedValue = applyResponseCurve(processedValue, triggerCurveCache_);
                stats_.curvesApplied++;
            }
            stats_.triggerEventsProcessed++;
        }
        else {
            if (config_.stickCurve != ResponseCurve::LINEAR && config_.stickCurveStrength > math::EPSILON) {
                processedValue = applyResponseCurve(processedValue, stickCurveCache_);
                stats_.curvesApplied++;
            }
            stats_.stickEventsProcessed++;
        }

        // Clamp final value to valid range
        data.value = math::clamp(processedValue, -1.0f, 1.0f);

        return true;
    }

    bool SensitivityStage::processMouseWheel(MouseWheelEventData& data) const {
        // Apply wheel-specific sensitivity
        const float wheelSensitivity = config_.globalSensitivity * config_.wheelSensitivity;

        data.delta.x = applySensitivity(data.delta.x, wheelSensitivity);
        data.delta.y = applySensitivity(data.delta.y, wheelSensitivity);

        return true;
    }

    float SensitivityStage::applySensitivity(const float value, const float sensitivity) noexcept {
        return value * sensitivity;
    }

    math::Vec2 SensitivityStage::applySensitivity(const math::Vec2& value,
                                                  const SensitivityConfig::AxisSensitivity& config) noexcept {
        math::Vec2 result = value;

        result.x *= config.x;
        result.y *= config.y;

        if (config.invertX) result.x = -result.x;
        if (config.invertY) result.y = -result.y;

        return result;
    }

    float SensitivityStage::applyResponseCurve(const float value, const CurveCache& cache) noexcept {
        return cache.apply(value);
    }

    // TODO: Revisar porque nos e usa deltatime
    math::Vec2 SensitivityStage::applyMouseAcceleration(const math::Vec2& delta, const float velocity,
                                                        const float deltaTime) const noexcept {
        using namespace engine::input::utils;

        if (velocity < config_.mouseAccelerationThreshold) {
            return delta; // No acceleration below threshold
        }

        // Calculate acceleration factor based on velocity
        const float velocityRatio = velocity / config_.mouseAccelerationThreshold;
        const float accelerationMultiplier = 1.0f + (config_.mouseAccelerationFactor - 1.0f) * velocityRatio;

        return applyAcceleration(delta, 1.0f, accelerationMultiplier);
    }

    void SensitivityStage::updateVelocity(const DeviceID deviceId, const math::Vec2& position, const float deltaTime) {
        auto& tracker = velocityTrackers_[deviceId];

        if (tracker.lastTime != std::chrono::steady_clock::time_point{}) {
            const math::Vec2 deltaPos = position - tracker.lastPosition;
            const float distance = glm::length(deltaPos);
            tracker.velocity = distance / deltaTime;

            // Apply exponential smoothing to velocity for stable acceleration
            constexpr float smoothingFactor = 0.1f;
            tracker.smoothedVelocity = (1.0f - smoothingFactor) * tracker.smoothedVelocity + smoothingFactor * tracker.
                velocity;
        }

        tracker.lastPosition = position;
        tracker.lastTime = std::chrono::steady_clock::now();
    }

    float SensitivityStage::getDeviceSensitivity(const DeviceID deviceId) const noexcept {
        if (const auto it = config_.deviceSensitivity.find(deviceId); it != config_.deviceSensitivity.end()) {
            return it->second;
        }
        return 1.0f; // Default sensitivity
    }

    // ============================================================================
    // CurveCache Implementation
    // ============================================================================

    void SensitivityStage::CurveCache::regenerate() {
        // Pre-calculate lookup table for performance
        for (std::size_t i = 0; i < TABLE_SIZE; ++i) {
            const float input = static_cast<float>(i) / static_cast<float>(TABLE_SIZE - 1); // [0, 1]
            const float inputSigned = input * 2.0f - 1.0f; // [-1, 1]

            float output = inputSigned;
            const float absInput = std::abs(inputSigned);
            const float sign = inputSigned < 0.0f ? -1.0f : 1.0f;

            switch (type) {
            case ResponseCurve::LINEAR:
                // No change needed
                break;

            case ResponseCurve::QUADRATIC: {
                const float quadratic = absInput * absInput;
                output = sign * math::lerp(absInput, quadratic, strength);
                break;
            }

            case ResponseCurve::CUBIC: {
                const float cubic = absInput * absInput * absInput;
                output = sign * math::lerp(absInput, cubic, strength);
                break;
            }

            case ResponseCurve::EXPONENTIAL: {
                const float exponential = (std::exp(absInput) - 1.0f) / (math::E<float> - 1.0f);
                output = sign * math::lerp(absInput, exponential, strength);
                break;
            }

            case ResponseCurve::LOGARITHMIC: {
                const float logarithmic = std::log(1.0f + absInput * 9.0f) / std::log(10.0f);
                output = sign * math::lerp(absInput, logarithmic, strength);
                break;
            }

            case ResponseCurve::CUSTOM:
                // Custom curves would need to be handled separately
                break;
            }

            lookupTable[i] = math::clamp(output, -1.0f, 1.0f);
        }
    }

    float SensitivityStage::CurveCache::apply(const float value) const noexcept {
        if (type == ResponseCurve::LINEAR || strength < math::EPSILON) {
            return value;
        }

        const float absValue = std::abs(value);
        if (absValue < math::EPSILON) {
            return 0.0f;
        }

        // Map input value to lookup table index
        const float normalizedInput = (absValue + 1.0f) * 0.5f; // Map [-1, 1] to [0, 1]
        const float scaledIndex = normalizedInput * static_cast<float>(TABLE_SIZE - 1);
        const std::size_t index = static_cast<std::size_t>(scaledIndex);

        // Bounds check
        if (index >= TABLE_SIZE - 1) {
            return value > 0.0f ? lookupTable[TABLE_SIZE - 1] : -lookupTable[TABLE_SIZE - 1];
        }

        // Linear interpolation between adjacent table values for smoother curves
        const float fraction = scaledIndex - static_cast<float>(index);
        const float interpolated = lookupTable[index] * (1.0f - fraction) + lookupTable[index + 1] * fraction;

        return value > 0.0f ? std::abs(interpolated) : -std::abs(interpolated);
    }
} // namespace engine::input::processing
