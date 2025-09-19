/**
 * @file KeyboardDevice.cpp
 * @brief Keyboard input device implementation
 * @author Andrés Guerrero
 * @date 12-09-2025
 */

#include "KeyboardDevice.h"

#include "../../core/InputSnapshot.h"
#include "../../utils/InputUtils.h"

#include <algorithm>

namespace engine::input {
    // ============================================================================
    // TextInputHandler Implementation
    // ============================================================================

    KeyboardDevice::TextInputHandler::TextInputHandler() noexcept
        : isComposing_(false)
          , cursorPos_(0) {
    }

    void KeyboardDevice::TextInputHandler::initialize() {
        buffer_.clear();
        buffer_.reserve(256);
        compositionText_.clear();
        isComposing_ = false;
        cursorPos_ = 0;
    }

    void KeyboardDevice::TextInputHandler::shutdown() {
        reset();
    }

    void KeyboardDevice::TextInputHandler::reset() noexcept {
        buffer_.clear();
        compositionText_.clear();
        isComposing_ = false;
        cursorPos_ = 0;
    }

    void KeyboardDevice::TextInputHandler::update(const float deltaTime) {
        (void)deltaTime;
        // Could handle composition timeout here
    }

    void KeyboardDevice::TextInputHandler::beginInput() {
        reset();
    }

    void KeyboardDevice::TextInputHandler::endInput() {
        if (isComposing_) {
            // Commit composition
            buffer_ += compositionText_;
            compositionText_.clear();
            isComposing_ = false;
        }
    }

    void KeyboardDevice::TextInputHandler::addText(const std::string& text) {
        if (buffer_.length() + text.length() <= 256) {
            buffer_ += text;
            cursorPos_ = buffer_.length();
        }
    }

    void KeyboardDevice::TextInputHandler::handleKeyPress(const KeyCode key, const KeyModifier modifiers) {
        (void)modifiers;

        switch (key) {
        case KeyCode::BACKSPACE:
            if (!buffer_.empty() && cursorPos_ > 0) {
                buffer_.erase(cursorPos_ - 1, 1);
                cursorPos_--;
            }
            break;

        case KeyCode::DELETE:
            if (cursorPos_ < buffer_.length()) {
                buffer_.erase(cursorPos_, 1);
            }
            break;

        case KeyCode::LEFT:
            if (cursorPos_ > 0) cursorPos_--;
            break;

        case KeyCode::RIGHT:
            if (cursorPos_ < buffer_.length()) cursorPos_++;
            break;

        case KeyCode::HOME:
            cursorPos_ = 0;
            break;

        case KeyCode::END:
            cursorPos_ = buffer_.length();
            break;

        default:
            break;
        }
    }

    std::string KeyboardDevice::TextInputHandler::extractText() {
        std::string result = std::move(buffer_);
        buffer_.clear();
        cursorPos_ = 0;
        return result;
    }

    // ============================================================================
    // KeyboardDevice Implementation
    // ============================================================================

    KeyboardDevice::KeyboardDevice() noexcept
        : repeatEnabled_(true)
          , repeatDelay_(0.5f)
          , repeatRate_(0.05f)
          , textInputActive_(false) {
        deviceType_ = DeviceType::KEYBOARD;
        deviceName_ = "System Keyboard";

        // Set capabilities
        capabilities_.setCapability(DeviceCapabilityFlags::DIGITAL_BUTTONS);
        capabilities_.setCapability(DeviceCapabilityFlags::USB);
        capabilities_.getSpecsMutable().buttonCount = static_cast<std::uint8_t>(KeyCode::KEY_COUNT);
    }

    bool KeyboardDevice::initialize(const DeviceInitParams& params) {
        deviceId_ = params.deviceId;
        deviceName_ = params.name.empty() ? "System Keyboard" : params.name;
        playerIndex_ = params.playerIndex;
        platformHandle_ = params.platformHandle;

        // Initialize text input handler
        textInputHandler_.initialize();

        // Clear state
        clearState();

        // Mark as connected
        updateConnectionState(DeviceConnectionStat::CONNECTED);
        state_.updateActivity();

        return true;
    }

    void KeyboardDevice::shutdown() {
        textInputHandler_.shutdown();
        clearState();
        repeatKeys_.clear();
        updateConnectionState(DeviceConnectionStat::DISCONNECTED);
    }

    void KeyboardDevice::reset() noexcept {
        clearState();
        repeatKeys_.clear();
        textInputHandler_.reset();
        textInputActive_ = false;
    }

    void KeyboardDevice::update(const float deltaTime) {
        // Save previous state
        previousState_ = keyboardState_;

        // Clear frame-specific state
        keyboardState_.clearFrameState();

        // Process key repeat
        if (repeatEnabled_) {
            processKeyRepeat(deltaTime);
        }

        // Update text input
        if (textInputActive_) {
            textInputHandler_.update(deltaTime);
        }

        // Update device state
        state_.updateActivity();
    }

    // TODO: Revisar esto
    bool KeyboardDevice::pollEvents() {
        // Keyboard events are typically pushed, not polled
        // This would interface with the platform backend
        return false;
    }

    // TODO: Revisar esto
    bool KeyboardDevice::processRawInput(const void* data, const std::size_t size) {
        if (!data || size == 0) return false;

        // This would process platform-specific raw input
        // For now, we assume events come through onKeyPress/onKeyRelease
        return true;
    }

    void KeyboardDevice::onKeyPress(const KeyCode key, const KeyModifier modifiers, const bool isRepeat) {
        if (key >= KeyCode::KEY_COUNT) return;

        // Update modifier from key
        updateModifierFromKey(key, true);

        // Update state
        keyboardState_.updateKey(key, true);
        keyboardState_.modifiers = modifiers;

        // Add to repeat tracking if not already repeating
        if (repeatEnabled_ && !isRepeat) {
            RepeatKey& repeatKey = repeatKeys_[key];
            repeatKey.key = key;
            repeatKey.timeHeld = 0.0f;
            repeatKey.nextRepeatTime = repeatDelay_;
        }

        // Generate event
        generateKeyEvent(InputEventType::KEY_PRESSED, key, modifiers, isRepeat);

        // Handle text input
        if (textInputActive_) {
            textInputHandler_.handleKeyPress(key, modifiers);
        }

        state_.updateActivity();
    }

    void KeyboardDevice::onKeyRelease(const KeyCode key, const KeyModifier modifiers) {
        if (key >= KeyCode::KEY_COUNT) return;

        // Update modifier from key
        updateModifierFromKey(key, false);

        // Update state
        keyboardState_.updateKey(key, false);
        keyboardState_.modifiers = modifiers;

        // Remove from repeat tracking
        repeatKeys_.erase(key);

        // Generate event
        generateKeyEvent(InputEventType::KEY_RELEASED, key, modifiers, false);

        state_.updateActivity();
    }

    void KeyboardDevice::onTextInput(const char* text) {
        if (!textInputActive_ || !text) return;

        const std::string str(text);
        textInputHandler_.addText(str);

        // Copy to keyboard state buffer
        const std::size_t len = std::min(str.length(), keyboardState_.textBuffer.size() - 1);
        std::memcpy(keyboardState_.textBuffer.data(), str.c_str(), len);
        keyboardState_.textBuffer[len] = '\0';
        keyboardState_.textLength = len;

        // Generate event
        generateTextEvent(str);

        state_.updateActivity();
    }

    void KeyboardDevice::setTextInputMode(const bool enabled) noexcept {
        textInputActive_ = enabled;

        if (enabled) {
            textInputHandler_.beginInput();
        }
        else {
            textInputHandler_.endInput();
        }
    }

    void KeyboardDevice::updateSnapshot(InputSnapshot& snapshot) const {
        // Copy keyboard state
        snapshot.keyboard = keyboardState_;

        // Add text from text input handler
        if (textInputActive_ && textInputHandler_.hasText()) {
            const auto& text = textInputHandler_.getText();
            const std::size_t len = std::min(text.length(), snapshot.keyboard.textBuffer.size() - 1);
            std::memcpy(snapshot.keyboard.textBuffer.data(), text.c_str(), len);
            snapshot.keyboard.textBuffer[len] = '\0';
            snapshot.keyboard.textLength = len;
        }
    }

    void KeyboardDevice::setKeyRepeatParams(const float delay, const float rate) noexcept {
        repeatDelay_ = std::max(0.0f, delay);
        repeatRate_ = std::max(0.01f, rate);
    }

    void KeyboardDevice::processKeyRepeat(const float deltaTime) {
        std::vector<KeyCode> keysToRepeat;

        for (auto& [key, repeatKey] : repeatKeys_) {
            repeatKey.timeHeld += deltaTime;

            if (repeatKey.timeHeld >= repeatKey.nextRepeatTime) {
                keysToRepeat.push_back(key);
                repeatKey.nextRepeatTime += repeatRate_;
            }
        }

        // Generate repeat events
        for (const KeyCode key : keysToRepeat) {
            onKeyPress(key, keyboardState_.modifiers, true);
        }
    }

    void KeyboardDevice::clearState() noexcept {
        keyboardState_.pressed.reset();
        keyboardState_.justPressed.reset();
        keyboardState_.justReleased.reset();
        keyboardState_.modifiers = KeyModifier::NONE;
        keyboardState_.textLength = 0;
        std::memset(keyboardState_.textBuffer.data(), 0, keyboardState_.textBuffer.size());
    }

    void KeyboardDevice::updateModifierFromKey(const KeyCode key, const bool pressed) noexcept {
        auto modifier = KeyModifier::NONE;

        switch (key) {
        case KeyCode::LEFT_SHIFT: modifier = KeyModifier::LEFT_SHIFT;
            break;
        case KeyCode::RIGHT_SHIFT: modifier = KeyModifier::RIGHT_SHIFT;
            break;
        case KeyCode::LEFT_CTRL: modifier = KeyModifier::LEFT_CTRL;
            break;
        case KeyCode::RIGHT_CTRL: modifier = KeyModifier::RIGHT_CTRL;
            break;
        case KeyCode::LEFT_ALT: modifier = KeyModifier::LEFT_ALT;
            break;
        case KeyCode::RIGHT_ALT: modifier = KeyModifier::RIGHT_ALT;
            break;
        case KeyCode::LEFT_SUPER: modifier = KeyModifier::LEFT_SUPER;
            break;
        case KeyCode::RIGHT_SUPER: modifier = KeyModifier::RIGHT_SUPER;
            break;
        default: return;
        }

        if (pressed) {
            keyboardState_.modifiers = keyboardState_.modifiers | modifier;
        }
        else {
            keyboardState_.modifiers = static_cast<KeyModifier>(
                static_cast<std::uint8_t>(keyboardState_.modifiers) &
                ~static_cast<std::uint8_t>(modifier)
            );
        }
    }

    void KeyboardDevice::generateKeyEvent(const InputEventType type, const KeyCode key,
                                          const KeyModifier modifiers, const bool isRepeat) {
        const KeyboardEventData data(key, modifiers, isRepeat);
        InputEvent event(type, deviceId_, data);
        event.playerId = playerIndex_;

        queueEvent(std::move(event));
    }

    void KeyboardDevice::generateTextEvent(const std::string& text) {
        const TextEventData data(text.c_str(), 0); // Unicode codepoint would be calculated
        InputEvent event(InputEventType::TEXT_INPUT, deviceId_, data);
        event.playerId = playerIndex_;

        queueEvent(std::move(event));
    }
} // namespace engine::input
