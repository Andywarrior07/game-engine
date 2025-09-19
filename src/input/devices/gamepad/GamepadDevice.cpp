/**
 * @file GamepadDevice.cpp
 * @brief Gamepad/Controller input device implementation
 * @author Andrés Guerrero
 * @date 12-09-2025
 */

#include "GamepadDevice.h"

#include "../../utils/InputUtils.h"
#include "../base/InputDevice.h"

#include <algorithm>

namespace engine::input {
    GamepadDevice::GamepadDevice() noexcept
        : profile_(GamepadProfile::UNKNOWN)
          , stickSensitivity_(1.0f)
          , triggerThreshold_(0.1f)
          , stickCurve_(ResponseCurve::LINEAR)
          , triggerCurve_(ResponseCurve::LINEAR)
          , stickCurveStrength_(0.0f)
          , triggerCurveStrength_(0.0f)
          , rumbleEnabled_(true)
          , rumbleIntensityScale_(1.0f)
          , adaptiveTriggersEnabled_(false)
          , lightBarEnabled_(false)
          , lightBarColor_{1.0f, 1.0f, 1.0f} {
        deviceType_ = DeviceType::GAMEPAD;
        deviceName_ = "Generic Gamepad";

        // Set default deadzone settings
        stickDeadzone_ = DeadzoneSettings(0.15f, 0.95f);
        triggerDeadzone_ = DeadzoneSettings(0.1f, 1.0f);

        // Set capabilities
        capabilities_.setCapability(DeviceCapabilityFlags::DIGITAL_BUTTONS);
        capabilities_.setCapability(DeviceCapabilityFlags::ANALOG_STICKS);
        capabilities_.setCapability(DeviceCapabilityFlags::ANALOG_TRIGGERS);
        capabilities_.setCapability(DeviceCapabilityFlags::DPAD);
        capabilities_.setCapability(DeviceCapabilityFlags::RUMBLE_BASIC);

        auto& specs = capabilities_.getSpecsMutable();
        specs.buttonCount = static_cast<std::uint8_t>(GamepadButton::BUTTON_COUNT);
        specs.axisCount = 6; // 2 sticks + 2 triggers
        specs.hatCount = 1; // D-pad
        specs.rumbleSpec.motorCount = 2;
    }

    bool GamepadDevice::initialize(const DeviceInitParams& params) {
        deviceId_ = params.deviceId;
        deviceName_ = params.name.empty() ? "Generic Gamepad" : params.name;
        playerIndex_ = params.playerIndex;
        platformHandle_ = params.platformHandle;

        // Clear state
        clearState();

        // Setup default mappings based on name detection
        if (deviceName_.find("Xbox") != std::string::npos) {
            profile_ = GamepadProfile::XBOX;
        }
        else if (deviceName_.find("PlayStation") != std::string::npos ||
            deviceName_.find("DualShock") != std::string::npos ||
            deviceName_.find("DualSense") != std::string::npos) {
            profile_ = GamepadProfile::PLAYSTATION;
            capabilities_.setCapability(DeviceCapabilityFlags::TOUCHPAD);
            capabilities_.setCapability(DeviceCapabilityFlags::LIGHT_BAR);
            gamepadState_.hasTouchpad = true;
        }
        else if (deviceName_.find("Nintendo") != std::string::npos ||
            deviceName_.find("Switch") != std::string::npos) {
            profile_ = GamepadProfile::NINTENDO;
        }
        else {
            profile_ = GamepadProfile::GENERIC;
        }

        setupDefaultMappings(profile_);

        // Update gamepad state
        gamepadState_.deviceId = deviceId_;
        gamepadState_.name = deviceName_;
        gamepadState_.playerIndex = playerIndex_;
        gamepadState_.connectedTime = std::chrono::steady_clock::now();
        gamepadState_.isConnected = true;

        // Mark as connected
        updateConnectionState(DeviceConnectionStat::CONNECTED);
        state_.updateActivity();

        return true;
    }

    void GamepadDevice::shutdown() {
        stopRumble();
        clearState();
        gamepadState_.isConnected = false;
        updateConnectionState(DeviceConnectionStat::DISCONNECTED);
    }

    void GamepadDevice::reset() noexcept {
        clearState();
        stopRumble();
    }

    void GamepadDevice::update(const float deltaTime) {
        // Save previous state
        previousState_ = gamepadState_;

        // Clear frame-specific state
        gamepadState_.clearFrameState();

        // Update rumble
        gamepadState_.updateRumble(deltaTime);

        // Update D-Pad from buttons
        gamepadState_.updateDPadFromButtons();

        // Check if we need to stop rumble
        if (gamepadState_.leftMotor == 0 && gamepadState_.rightMotor == 0 && rumbleEnabled_) {
            sendRumbleCommand(0, 0);
        }

        // Update device state
        state_.updateActivity();
    }

    bool GamepadDevice::pollEvents() {
        // Gamepad events are typically pushed, not polled
        return false;
    }

    bool GamepadDevice::processRawInput(const void* data, const std::size_t size) {
        if (!data || size == 0) return false;

        // This would process platform-specific raw input
        return true;
    }

    bool GamepadDevice::setRumble(float leftMotor, float rightMotor, const float duration) {
        if (!rumbleEnabled_) return false;

        // Apply intensity scale
        leftMotor *= rumbleIntensityScale_;
        rightMotor *= rumbleIntensityScale_;

        // Set rumble state
        gamepadState_.setRumble(leftMotor, rightMotor, duration);

        // Send to backend
        sendRumbleCommand(leftMotor, rightMotor);

        return true;
    }

    void GamepadDevice::stopRumble() noexcept {
        gamepadState_.stopRumble();
        sendRumbleCommand(0, 0);
    }

    bool GamepadDevice::playHapticPattern(const std::uint32_t pattern, const float intensity) {
        // Convert pattern to rumble
        // This is a simplified implementation
        float left = 0, right = 0;

        switch (pattern) {
        case 1: // Light tap
            left = 0.2f * intensity;
            right = 0.2f * intensity;
            return setRumble(left, right, 0.1f);

        case 2: // Strong pulse
            left = 0.8f * intensity;
            right = 0.8f * intensity;
            return setRumble(left, right, 0.2f);

        case 3: // Soft vibration
            left = 0.3f * intensity;
            right = 0.1f * intensity;
            return setRumble(left, right, 0.5f);

        default:
            return false;
        }
    }

    void GamepadDevice::onButtonPress(const GamepadButton button, const float pressure) {
        // Update state
        gamepadState_.updateButton(button, true, pressure);

        // Generate event
        generateButtonEvent(InputEventType::GAMEPAD_BUTTON_PRESSED, button, pressure);

        state_.updateActivity();
    }

    void GamepadDevice::onButtonRelease(const GamepadButton button) {
        // Update state
        gamepadState_.updateButton(button, false, 0.0f);

        // Generate event
        generateButtonEvent(InputEventType::GAMEPAD_BUTTON_RELEASED, button, 0.0f);

        state_.updateActivity();
    }

    void GamepadDevice::onStickMove(const bool leftStick, const float x, const float y) {
        math::Vec2 raw(x, y);

        // Apply calibration
        raw = applyStickCalibration(raw, leftStick);

        // Store raw value
        if (leftStick) {
            gamepadState_.leftStickRaw = raw;
        }
        else {
            gamepadState_.rightStickRaw = raw;
        }

        // Apply deadzone
        math::Vec2 processed = applyStickDeadzone(raw, leftStick);

        // Apply response curve
        if (stickCurveStrength_ > 0) {
            if (const float magnitude = processed.length(); magnitude > 0) {
                const float newMagnitude = applyResponseCurve(magnitude, stickCurve_, stickCurveStrength_);
                processed = glm::normalize(processed) * newMagnitude;
            }
        }

        // Apply sensitivity
        processed = processed * stickSensitivity_;

        // Clamp to unit circle
        processed = utils::clampToUnitCircle(processed);

        // Calculate delta
        const math::Vec2 oldValue = leftStick ? previousState_.leftStick : previousState_.rightStick;
        const math::Vec2 delta = processed - oldValue;

        // Update state
        if (leftStick) {
            gamepadState_.leftStick = processed;
        }
        else {
            gamepadState_.rightStick = processed;
        }

        // Generate event
        GamepadAxis axis = leftStick ? GamepadAxis::LEFT_STICK_X : GamepadAxis::RIGHT_STICK_X;
        generateAxisEvent(axis, processed.x, delta.x);

        axis = leftStick ? GamepadAxis::LEFT_STICK_Y : GamepadAxis::RIGHT_STICK_Y;
        generateAxisEvent(axis, processed.y, delta.y);

        state_.updateActivity();
    }

    void GamepadDevice::onTriggerMove(const bool leftTrigger, const float value) {
        // Apply calibration
        const float raw = applyTriggerCalibration(value, leftTrigger);

        // Store raw value
        if (leftTrigger) {
            gamepadState_.leftTriggerRaw = raw;
        }
        else {
            gamepadState_.rightTriggerRaw = raw;
        }

        // Apply deadzone
        float processed = applyTriggerDeadzone(raw);

        // Apply response curve
        if (triggerCurveStrength_ > 0) {
            processed = applyResponseCurve(processed, triggerCurve_, triggerCurveStrength_);
        }

        // Calculate delta
        const float oldValue = leftTrigger ? previousState_.leftTrigger : previousState_.rightTrigger;
        const float delta = processed - oldValue;

        // Update state
        if (leftTrigger) {
            gamepadState_.leftTrigger = processed;
        }
        else {
            gamepadState_.rightTrigger = processed;
        }

        // Generate event
        const GamepadAxis axis = leftTrigger ? GamepadAxis::LEFT_TRIGGER : GamepadAxis::RIGHT_TRIGGER;
        generateAxisEvent(axis, processed, delta);

        // Generate trigger moved event if threshold crossed
        if (std::abs(delta) > 0.01f) {
            InputEvent event;
            event.type = InputEventType::GAMEPAD_TRIGGER_MOVED;
            event.deviceId = deviceId_;
            event.playerId = playerIndex_;
            GamepadAxisEventData data(axis, processed, delta);
            event.data = data;
            queueEvent(std::move(event));
        }

        state_.updateActivity();
    }

    void GamepadDevice::onMotionUpdate(const math::Vec3& accelerometer, const math::Vec3& gyroscope) {
        gamepadState_.accelerometer = accelerometer;
        gamepadState_.gyroscope = gyroscope;
        gamepadState_.hasMotion = true;
        // TODO: Revisar esto
        // Could generate motion events here if needed

        state_.updateActivity();
    }

    void GamepadDevice::onTouchpadUpdate(const int touchIndex, const bool active, const float x, const float y, const float pressure) {
        if (touchIndex < 0 || touchIndex >= 2) return;

        auto& touchpad = gamepadState_.touchpads[touchIndex];
        touchpad.isActive = active;
        touchpad.position = math::Vec2(x, y);
        touchpad.pressure = pressure;

        // Could generate touchpad events here if needed

        state_.updateActivity();
    }

    void GamepadDevice::onBatteryUpdate(const float level, const bool charging) {
        gamepadState_.batteryLevel = math::clamp(level, 0.0f, 1.0f);
        gamepadState_.isCharging = charging;
    }

    void GamepadDevice::updateSnapshot(InputSnapshot& snapshot) const {
        if (playerIndex_ < MAX_PLAYERS) {
            snapshot.gamepads[playerIndex_] = gamepadState_;
        }
    }

    void GamepadDevice::setDeadzoneSettings(const DeadzoneSettings& settings) noexcept {
        stickDeadzone_ = settings;
        InputDeviceBase::setDeadzoneSettings(settings);
    }

    void GamepadDevice::setStickSensitivity(const float sensitivity) noexcept {
        stickSensitivity_ = math::clamp(sensitivity, 0.1f, 3.0f);
    }

    void GamepadDevice::setTriggerThreshold(const float threshold) noexcept {
        triggerThreshold_ = math::clamp(threshold, 0.0f, 0.5f);
    }

    void GamepadDevice::setProfile(const GamepadProfile profile) noexcept {
        profile_ = profile;
        setupDefaultMappings(profile);
    }

    void GamepadDevice::setAdaptiveTriggers(const bool enabled) noexcept {
        adaptiveTriggersEnabled_ = enabled;
        // TODO: Revisar esto
        if (capabilities_.hasCapability(DeviceCapabilityFlags::ADAPTIVE_TRIGGERS)) {
            // Would send command to device
        }
    }

    void GamepadDevice::setLightBarColor(const float r, const float g, const float b) noexcept {
        lightBarColor_.r = math::saturate(r);
        lightBarColor_.g = math::saturate(g);
        lightBarColor_.b = math::saturate(b);
        lightBarEnabled_ = true;

        // TODO: Revisar esto
        if (capabilities_.hasCapability(DeviceCapabilityFlags::LIGHT_BAR)) {
            // Would send command to device
        }
    }

    void GamepadDevice::clearState() noexcept {
        gamepadState_.reset();
    }

    // TODO: Revisar esto urgente
    math::Vec2 GamepadDevice::applyStickDeadzone(const math::Vec2& raw, const bool leftStick) const noexcept {
        (void)leftStick; // Could use different deadzones per stick
        return utils::applyHybridDeadzone(raw, stickDeadzone_);
    }

    float GamepadDevice::applyTriggerDeadzone(const float raw) const noexcept {
        return triggerDeadzone_.apply(raw);
    }

    float GamepadDevice::applyResponseCurve(const float value, const ResponseCurve curve, const float strength) noexcept {
        return utils::applyResponseCurve(value, curve, strength);
    }

    math::Vec2 GamepadDevice::applyStickCalibration(const math::Vec2& raw, const bool leftStick) const noexcept {
        if (!calibration_.isCalibrated) {
            return raw;
        }

        const math::Vec2& center = leftStick ? calibration_.leftStickCenter : calibration_.rightStickCenter;
        const math::Vec2& maxVal = leftStick ? calibration_.leftStickMax : calibration_.rightStickMax;

        // Apply center offset
        math::Vec2 calibrated = raw - center;

        // Apply scale
        calibrated.x /= maxVal.x;
        calibrated.y /= maxVal.y;

        return utils::clampToUnitCircle(calibrated);
    }

    // TODO: Revisar esto
    float GamepadDevice::applyTriggerCalibration(const float raw, const bool leftTrigger) const noexcept {
        if (!calibration_.isCalibrated) {
            return raw;
        }

        const float min = leftTrigger ? calibration_.leftTriggerMin : calibration_.rightTriggerMin;
        const float max = leftTrigger ? calibration_.leftTriggerMax : calibration_.rightTriggerMax;

        // Normalize to 0-1 range
        return math::saturate((raw - min) / (max - min));
    }

    void GamepadDevice::generateButtonEvent(const InputEventType type, const GamepadButton button, const float pressure) {
        const GamepadButtonEventData data(button, pressure);
        InputEvent event(type, deviceId_, data);
        event.playerId = playerIndex_;
        queueEvent(std::move(event));
    }

    void GamepadDevice::generateAxisEvent(const GamepadAxis axis, const float value, const float delta) {
        const GamepadAxisEventData data(axis, value, delta);
        InputEvent event(InputEventType::GAMEPAD_AXIS_MOVED, deviceId_, data);
        event.playerId = playerIndex_;
        queueEvent(std::move(event));
    }

    void GamepadDevice::setupDefaultMappings(const GamepadProfile profile) {
        buttonMapping_.clear();
        axisMapping_.clear();

        switch (profile) {
        case GamepadProfile::XBOX:
            // Xbox button mappings (SDL button indices)
            buttonMapping_[0] = GamepadButton::A;
            buttonMapping_[1] = GamepadButton::B;
            buttonMapping_[2] = GamepadButton::X;
            buttonMapping_[3] = GamepadButton::Y;
            buttonMapping_[4] = GamepadButton::LEFT_BUMPER;
            buttonMapping_[5] = GamepadButton::RIGHT_BUMPER;
            buttonMapping_[6] = GamepadButton::BACK;
            buttonMapping_[7] = GamepadButton::START;
            buttonMapping_[8] = GamepadButton::GUIDE;
            buttonMapping_[9] = GamepadButton::LEFT_STICK;
            buttonMapping_[10] = GamepadButton::RIGHT_STICK;
            buttonMapping_[11] = GamepadButton::DPAD_UP;
            buttonMapping_[12] = GamepadButton::DPAD_DOWN;
            buttonMapping_[13] = GamepadButton::DPAD_LEFT;
            buttonMapping_[14] = GamepadButton::DPAD_RIGHT;
            break;

        case GamepadProfile::PLAYSTATION:
            // PlayStation button mappings
            buttonMapping_[0] = GamepadButton::X; // Cross -> A
            buttonMapping_[1] = GamepadButton::A; // Circle -> B
            buttonMapping_[2] = GamepadButton::Y; // Square -> X
            buttonMapping_[3] = GamepadButton::B; // Triangle -> Y
            buttonMapping_[4] = GamepadButton::LEFT_BUMPER; // L1
            buttonMapping_[5] = GamepadButton::RIGHT_BUMPER; // R1
            buttonMapping_[6] = GamepadButton::BACK; // Share
            buttonMapping_[7] = GamepadButton::START; // Options
            buttonMapping_[8] = GamepadButton::GUIDE; // PS Button
            buttonMapping_[9] = GamepadButton::LEFT_STICK; // L3
            buttonMapping_[10] = GamepadButton::RIGHT_STICK; // R3
            buttonMapping_[11] = GamepadButton::DPAD_UP;
            buttonMapping_[12] = GamepadButton::DPAD_DOWN;
            buttonMapping_[13] = GamepadButton::DPAD_LEFT;
            buttonMapping_[14] = GamepadButton::DPAD_RIGHT;
            buttonMapping_[15] = GamepadButton::TOUCHPAD;
            break;

        case GamepadProfile::NINTENDO:
            // Nintendo button mappings (swapped A/B, X/Y)
            buttonMapping_[0] = GamepadButton::B; // B -> A position
            buttonMapping_[1] = GamepadButton::A; // A -> B position
            buttonMapping_[2] = GamepadButton::Y; // Y -> X position
            buttonMapping_[3] = GamepadButton::X; // X -> Y position
            buttonMapping_[4] = GamepadButton::LEFT_BUMPER; // L
            buttonMapping_[5] = GamepadButton::RIGHT_BUMPER; // R
            buttonMapping_[6] = GamepadButton::BACK; // Minus
            buttonMapping_[7] = GamepadButton::START; // Plus
            buttonMapping_[8] = GamepadButton::GUIDE; // Home
            buttonMapping_[9] = GamepadButton::LEFT_STICK;
            buttonMapping_[10] = GamepadButton::RIGHT_STICK;
            buttonMapping_[11] = GamepadButton::DPAD_UP;
            buttonMapping_[12] = GamepadButton::DPAD_DOWN;
            buttonMapping_[13] = GamepadButton::DPAD_LEFT;
            buttonMapping_[14] = GamepadButton::DPAD_RIGHT;
            break;

        default:
            // Generic/default mappings
            for (std::uint32_t i = 0; i < static_cast<std::uint32_t>(GamepadButton::BUTTON_COUNT); ++i) {
                buttonMapping_[i] = static_cast<GamepadButton>(i);
            }
            break;
        }

        // Axis mappings are generally consistent
        axisMapping_[0] = GamepadAxis::LEFT_STICK_X;
        axisMapping_[1] = GamepadAxis::LEFT_STICK_Y;
        axisMapping_[2] = GamepadAxis::RIGHT_STICK_X;
        axisMapping_[3] = GamepadAxis::RIGHT_STICK_Y;
        axisMapping_[4] = GamepadAxis::LEFT_TRIGGER;
        axisMapping_[5] = GamepadAxis::RIGHT_TRIGGER;
    }

    // TODO: Implementar esto
    void GamepadDevice::sendRumbleCommand(const float left, const float right) {
        // This would interface with the platform backend
        // For now, just update internal state
        (void)left;
        (void)right;
    }

    void GamepadDevice::updateDPadFromAxes(const float x, const float y) {
        // Convert analog D-Pad values to digital
        constexpr float threshold = 0.5f;

        gamepadState_.dpadLeft = x < -threshold;
        gamepadState_.dpadRight = x > threshold;
        gamepadState_.dpadUp = y > threshold;
        gamepadState_.dpadDown = y < -threshold;

        // Update buttons
        if (gamepadState_.dpadUp) {
            onButtonPress(GamepadButton::DPAD_UP);
        }
        else {
            onButtonRelease(GamepadButton::DPAD_UP);
        }

        if (gamepadState_.dpadDown) {
            onButtonPress(GamepadButton::DPAD_DOWN);
        }
        else {
            onButtonRelease(GamepadButton::DPAD_DOWN);
        }

        if (gamepadState_.dpadLeft) {
            onButtonPress(GamepadButton::DPAD_LEFT);
        }
        else {
            onButtonRelease(GamepadButton::DPAD_LEFT);
        }

        if (gamepadState_.dpadRight) {
            onButtonPress(GamepadButton::DPAD_RIGHT);
        }
        else {
            onButtonRelease(GamepadButton::DPAD_RIGHT);
        }
    }
} // namespace engine::input
