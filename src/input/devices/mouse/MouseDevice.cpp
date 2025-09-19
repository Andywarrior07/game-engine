/**
 * @file MouseDevice.cpp
 * @brief Mouse input device implementation
 * @author Andrés Guerrero
 * @date 12-09-2025
 */

#include "MouseDevice.h"
#include "../../core/InputSnapshot.h"
#include "../../utils/InputUtils.h"
#include <cmath>
#include <algorithm>

namespace engine::input {
    MouseDevice::MouseDevice() noexcept
        : sensitivity_(1.0f)
          , accelerationEnabled_(false)
          , accelerationFactor_(1.5f)
          , rawInputEnabled_(false)
          , windowWidth_(1920)
          , windowHeight_(1080)
          , doubleClickTime_(0.3f)
          , doubleClickDistance_(5.0f)
          , xFilter_(1.0f, 0.007f, 1.0f)
          , yFilter_(1.0f, 0.007f, 1.0f)
          , filteringEnabled_(true)
          , accumulatedDelta_(0, 0) {
        deviceType_ = DeviceType::MOUSE;
        deviceName_ = "System Mouse";

        // Set capabilities
        capabilities_.setCapability(DeviceCapabilityFlags::DIGITAL_BUTTONS);
        capabilities_.setCapability(DeviceCapabilityFlags::HIGH_PRECISION);
        capabilities_.setCapability(DeviceCapabilityFlags::USB);

        auto& specs = capabilities_.getSpecsMutable();
        specs.buttonCount = static_cast<std::uint8_t>(MouseButton::BUTTON_COUNT);
        specs.axisCount = 3; // X, Y, Wheel

        auto& perf = capabilities_.getPerformanceMutable();
        perf.maxDPI = 16000; // Modern gaming mouse
        perf.currentDPI = 800; // Default DPI

        // Initialize last click positions
        lastClickPositions_.fill(math::Vec2(0, 0));
    }

    bool MouseDevice::initialize(const DeviceInitParams& params) {
        deviceId_ = params.deviceId;
        deviceName_ = params.name.empty() ? "System Mouse" : params.name;
        playerIndex_ = params.playerIndex;
        platformHandle_ = params.platformHandle;

        // Initialize mouse lock
        mouseLock_.initialize();

        // Clear state
        clearState();

        // Mark as connected
        updateConnectionState(DeviceConnectionStat::CONNECTED);
        state_.updateActivity();

        return true;
    }

    void MouseDevice::shutdown() {
        mouseLock_.shutdown();
        clearState();
        updateConnectionState(DeviceConnectionStat::DISCONNECTED);
    }

    void MouseDevice::reset() noexcept {
        clearState();
        mouseLock_.resetDelta();
        accumulatedDelta_ = math::VEC2_ZERO;
        xFilter_.reset();
        yFilter_.reset();
    }

    void MouseDevice::update(const float deltaTime) {
        // Save previous state
        previousState_ = mouseState_;

        // Clear frame-specific state
        mouseState_.clearFrameState();

        // Update mouse lock
        mouseLock_.update(deltaTime);

        // Update device state
        state_.updateActivity();
    }

    bool MouseDevice::pollEvents() {
        // Mouse events are typically pushed, not polled
        return false;
    }

    // TODO: Esto deberia quitarlo ya que no usaremos raw input, al menos en el corto plazo
    bool MouseDevice::processRawInput(const void* data, const std::size_t size) {
        if (!data || size == 0) return false;

        // This would process platform-specific raw input
        // For now, we assume events come through onMouseMove/onMouseButton
        return true;
    }

    void MouseDevice::onMouseMove(const float x, const float y, const float deltaX, const float deltaY) {
        // Update position
        mouseState_.position = math::Vec2(x, y);

        // Process movement (apply filtering, acceleration, etc.)
        constexpr float dt = 0.016f; // Assume 60 FPS, would get from timer
        math::Vec2 processedDelta = processMovement(deltaX, deltaY, dt);

        // Handle locked mode
        if (mouseLock_.isLocked()) {
            processedDelta = mouseLock_.processLockedMovement(processedDelta.x, processedDelta.y);
        }

        // Handle confined mode
        if (mouseLock_.isConfined()) {
            mouseState_.position = mouseLock_.clampToConfinement(mouseState_.position);
        }

        // Update state
        mouseState_.delta = processedDelta;
        mouseState_.normalizedPos = normalizePosition(mouseState_.position);

        // Generate event
        MouseMotionEventData eventData;
        eventData.position = mouseState_.position;
        eventData.delta = processedDelta;
        eventData.normalizedDelta = math::Vec2(
            processedDelta.x / static_cast<float>(windowWidth_),
            processedDelta.y / static_cast<float>(windowHeight_)
        );
        eventData.isRelative = mouseLock_.isLocked();

        generateMouseEvent(InputEventType::MOUSE_MOVED, eventData);

        state_.updateActivity();
    }

    void MouseDevice::onMouseButtonPress(MouseButton button, const float x, const float y) {
        // Update button state
        mouseState_.updateButton(button, true);

        // Update click count for double-click detection
        math::Vec2 position(x, y);
        updateClickCount(button, position);

        // Generate event
        MouseButtonEventData eventData;
        eventData.button = button;
        eventData.position = position;
        eventData.clickCount = mouseState_.clickCounts[static_cast<std::size_t>(button)];

        generateMouseEvent(InputEventType::MOUSE_BUTTON_PRESSED, eventData);

        state_.updateActivity();
    }

    void MouseDevice::onMouseButtonRelease(MouseButton button, const float x, const float y) {
        // Update button state
        mouseState_.updateButton(button, false);

        // Generate event
        MouseButtonEventData eventData;
        eventData.button = button;
        eventData.position = math::Vec2(x, y);
        eventData.clickCount = mouseState_.clickCounts[static_cast<std::size_t>(button)];

        generateMouseEvent(InputEventType::MOUSE_BUTTON_RELEASED, eventData);

        state_.updateActivity();
    }

    void MouseDevice::onMouseWheel(const float deltaX, const float deltaY) {
        // Update wheel delta
        mouseState_.wheelDelta = math::Vec2(deltaX, deltaY);

        // Generate event
        MouseWheelEventData eventData;
        eventData.delta = mouseState_.wheelDelta;
        eventData.position = mouseState_.position;
        eventData.isPrecise = false; // Would be set based on device

        generateMouseEvent(InputEventType::MOUSE_WHEEL, eventData);

        state_.updateActivity();
    }

    void MouseDevice::onMouseEnter() {
        mouseState_.isInWindow = true;

        // Generate event
        InputEvent event;
        event.type = InputEventType::MOUSE_ENTERED;
        event.deviceId = deviceId_;
        event.playerId = playerIndex_;
        queueEvent(std::move(event));

        state_.updateActivity();
    }

    void MouseDevice::onMouseLeave() {
        mouseState_.isInWindow = false;

        // Generate event
        InputEvent event;
        event.type = InputEventType::MOUSE_LEFT;
        event.deviceId = deviceId_;
        event.playerId = playerIndex_;
        queueEvent(std::move(event));

        state_.updateActivity();
    }

    void MouseDevice::updateSnapshot(InputSnapshot& snapshot) const {
        snapshot.mouse = mouseState_;
    }

    void MouseDevice::setSensitivity(const float sensitivity) noexcept {
        sensitivity_ = math::clamp(sensitivity, 0.1f, 10.0f);
        mouseLock_.setSensitivity(sensitivity_);
    }

    void MouseDevice::setAcceleration(const bool enabled, const float factor) noexcept {
        accelerationEnabled_ = enabled;
        accelerationFactor_ = math::clamp(factor, 1.0f, 3.0f);
    }

    void MouseDevice::setWindowSize(const int width, const int height) noexcept {
        windowWidth_ = std::max(1, width);
        windowHeight_ = std::max(1, height);
        mouseLock_.updateWindowSize(width, height);
    }

    void MouseDevice::clearState() noexcept {
        mouseState_.position = math::VEC2_ZERO;
        mouseState_.delta = math::VEC2_ZERO;
        mouseState_.normalizedPos = math::VEC2_ZERO;
        mouseState_.wheelDelta = math::VEC2_ZERO;
        mouseState_.buttons.reset();
        mouseState_.justPressed.reset();
        mouseState_.justReleased.reset();
        mouseState_.clickCounts.fill(0);
        mouseState_.cursorMode = CursorMode::NORMAL;
        mouseState_.isInWindow = true;
    }

    // TODO: Revisar el dt
    math::Vec2 MouseDevice::processMovement(const float deltaX, const float deltaY, const float dt) {
        math::Vec2 delta(deltaX, deltaY);

        // Apply sensitivity
        delta = delta * sensitivity_;

        // Apply acceleration if enabled
        if (accelerationEnabled_) {
            delta = utils::applyAcceleration(delta, 1.0f, accelerationFactor_);
        }

        // Apply filtering if enabled (smooths jittery input)
        if (filteringEnabled_ && dt > 0) {
            delta.x = xFilter_.filter(delta.x, dt);
            delta.y = yFilter_.filter(delta.y, dt);
        }

        // Add to accumulated delta for sub-pixel precision
        accumulatedDelta_ += delta;

        // Extract integer movement and keep fractional part
        math::Vec2 movement;
        movement.x = std::floor(accumulatedDelta_.x);
        movement.y = std::floor(accumulatedDelta_.y);
        accumulatedDelta_ -= movement;

        return movement;
    }

    void MouseDevice::updateClickCount(MouseButton button, const math::Vec2& position) {
        const auto index = static_cast<std::size_t>(button);
        if (index >= mouseState_.clickCounts.size()) return;

        const auto now = InputTimestamp::clock::now();
        const auto& lastClickTime = mouseState_.lastClickTimes[index];
        const auto& lastClickPos = lastClickPositions_[index];

        // Calculate time since last click
        const auto timeSinceLastClick = std::chrono::duration<float>(now - lastClickTime).count();


        // Calculate distance from last click
        // Check if this is a multi-click
        if (const float distance = (position - lastClickPos).length(); timeSinceLastClick <= doubleClickTime_ && distance <= doubleClickDistance_) {
            mouseState_.clickCounts[index]++;
            if (mouseState_.clickCounts[index] > 3) {
                mouseState_.clickCounts[index] = 1; // Reset after triple-click
            }
        }
        else {
            mouseState_.clickCounts[index] = 1;
        }

        // Update last click info
        mouseState_.lastClickTimes[index] = now;
        lastClickPositions_[index] = position;
    }

    void MouseDevice::generateMouseEvent(const InputEventType type, const MouseButtonEventData& data) {
        InputEvent event(type, deviceId_, data);
        event.playerId = playerIndex_;
        queueEvent(std::move(event));
    }

    void MouseDevice::generateMouseEvent(const InputEventType type, const MouseMotionEventData& data) {
        InputEvent event(type, deviceId_, data);
        event.playerId = playerIndex_;
        queueEvent(std::move(event));
    }

    void MouseDevice::generateMouseEvent(const InputEventType type, const MouseWheelEventData& data) {
        InputEvent event(type, deviceId_, data);
        event.playerId = playerIndex_;
        queueEvent(std::move(event));
    }

    math::Vec2 MouseDevice::normalizePosition(const math::Vec2& position) const noexcept {
        return math::Vec2(
            position.x / static_cast<float>(windowWidth_),
            position.y / static_cast<float>(windowHeight_)
        );
    }
} // namespace engine::input
