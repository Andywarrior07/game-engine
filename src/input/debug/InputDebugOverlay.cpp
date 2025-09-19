/**
 * @file InputDebugOverlay.cpp
 * @brief Visual debug overlay for input system implementation
 * @author Andrés Guerrero
 * @date 13-09-2025
 */

#include "InputDebugOverlay.h"

#include "../core/InputEvent.h"

#include <algorithm>
#include <iomanip>
#include <sstream>

namespace engine::input::debug {
    InputDebugOverlay::InputDebugOverlay() noexcept {
    }

    // ============================================================================
    // Update
    // ============================================================================

    void InputDebugOverlay::update(const InputSnapshot& snapshot, const float deltaTime) {
        if (!config_.enabled) {
            return;
        }

        timeSinceUpdate_ += deltaTime;

        // Update only at specified interval to reduce overhead
        if (timeSinceUpdate_ < config_.updateInterval) {
            return;
        }

        currentSnapshot_ = snapshot;
        timeSinceUpdate_ = 0.0f;

        // Clean up old history entries
        cleanupHistory();

        // Generate debug information
        generateDebugLines();
    }

    void InputDebugOverlay::addEvent(const InputEvent& event) {
        if (!config_.enabled || (config_.enabledSections & OverlaySection::EVENTS) == OverlaySection::NONE) {
            return;
        }

        // Add to event history
        EventHistoryEntry historyEntry;
        historyEntry.event = event;
        historyEntry.timestamp = std::chrono::steady_clock::now();

        eventHistory_.push_back(historyEntry);

        // Keep history size bounded
        if (eventHistory_.size() > config_.maxEventHistory) {
            eventHistory_.pop_front();
        }
    }

    void InputDebugOverlay::addPerformanceSample(const float frameTime) {
        if (!config_.enabled || !config_.showPerformanceGraph) {
            return;
        }

        performanceSamples_.push_back(frameTime);

        // Update statistics
        averageFrameTime_ = (averageFrameTime_ * 0.95f) + (frameTime * 0.05f); // Exponential smoothing
        peakFrameTime_ = std::max(peakFrameTime_, frameTime);

        // Keep sample count bounded
        if (performanceSamples_.size() > config_.graphSamples) {
            performanceSamples_.pop_front();
        }
    }

    void InputDebugOverlay::clearHistory() noexcept {
        eventHistory_.clear();
        actionHistory_.clear();
        performanceSamples_.clear();
        averageFrameTime_ = 0.0f;
        peakFrameTime_ = 0.0f;
    }

    void InputDebugOverlay::addCustomSection(const std::string& name, const std::vector<DebugLine>& lines) {
        customSections_[name] = lines;
    }

    void InputDebugOverlay::removeCustomSection(const std::string& name) {
        customSections_.erase(name);
    }

    void InputDebugOverlay::clearCustomSections() {
        customSections_.clear();
    }

    // ============================================================================
    // Private Implementation
    // ============================================================================

    void InputDebugOverlay::generateDebugLines() {
        debugLines_.clear();
        currentHeight_ = config_.y;

        // Generate sections based on enabled flags
        if ((config_.enabledSections & OverlaySection::KEYBOARD) != OverlaySection::NONE) {
            generateKeyboardSection();
        }

        if ((config_.enabledSections & OverlaySection::MOUSE) != OverlaySection::NONE) {
            generateMouseSection();
        }

        if ((config_.enabledSections & OverlaySection::GAMEPAD) != OverlaySection::NONE) {
            generateGamepadSection();
        }

        if ((config_.enabledSections & OverlaySection::TOUCH) != OverlaySection::NONE) {
            generateTouchSection();
        }

        if ((config_.enabledSections & OverlaySection::ACTIONS) != OverlaySection::NONE) {
            generateActionsSection();
        }

        if ((config_.enabledSections & OverlaySection::EVENTS) != OverlaySection::NONE) {
            generateEventsSection();
        }

        if ((config_.enabledSections & OverlaySection::PERFORMANCE) != OverlaySection::NONE) {
            generatePerformanceSection();
        }

        if ((config_.enabledSections & OverlaySection::CONTEXT) != OverlaySection::NONE) {
            generateContextSection();
        }

        if ((config_.enabledSections & OverlaySection::DEVICES) != OverlaySection::NONE) {
            generateDevicesSection();
        }

        if ((config_.enabledSections & OverlaySection::HISTORY) != OverlaySection::NONE) {
            generateHistorySection();
        }

        // Add custom sections
        for (const auto& [name, lines] : customSections_) {
            addSectionHeader("Custom: " + name);
            for (const auto& line : lines) {
                debugLines_.push_back(line);
                currentHeight_ += config_.lineHeight;
            }
            addSeparator();
        }

        // Update total height
        currentHeight_ += config_.lineHeight; // Extra padding at bottom
    }

    void InputDebugOverlay::generateKeyboardSection() {
        addSectionHeader("Keyboard");

        // Show pressed keys
        bool hasPressed = false;
        std::stringstream pressedKeys;

        for (std::uint16_t i = 0; i < static_cast<std::uint16_t>(KeyCode::KEY_COUNT); ++i) {
            const KeyCode key = static_cast<KeyCode>(i);
            if (currentSnapshot_.keyboard.isKeyPressed(key)) {
                if (hasPressed) pressedKeys << ", ";
                pressedKeys << formatKey(key);
                hasPressed = true;
            }
        }

        if (hasPressed) {
            addLine("Pressed: " + pressedKeys.str(), 10.0f,
                    config_.colors.pressed[0], config_.colors.pressed[1], config_.colors.pressed[2]);
        }
        else {
            addLine("No keys pressed", 10.0f, 0.6f, 0.6f, 0.6f);
        }

        // Show modifiers
        const auto modifiers = currentSnapshot_.keyboard.modifiers;
        if (modifiers != KeyModifier::NONE) {
            std::stringstream modStr;
            if (hasModifier(modifiers, KeyModifier::CTRL)) modStr << "Ctrl ";
            if (hasModifier(modifiers, KeyModifier::SHIFT)) modStr << "Shift ";
            if (hasModifier(modifiers, KeyModifier::ALT)) modStr << "Alt ";
            if (hasModifier(modifiers, KeyModifier::SUPER)) modStr << "Super ";

            addLine("Modifiers: " + modStr.str(), 10.0f,
                    config_.colors.analog[0], config_.colors.analog[1], config_.colors.analog[2]);
        }

        // Show text input state
        if (currentSnapshot_.acceptingTextInput) {
            addLine("Text Input: ACTIVE", 10.0f,
                    config_.colors.pressed[0], config_.colors.pressed[1], config_.colors.pressed[2]);
        }

        addSeparator();
    }

    void InputDebugOverlay::generateMouseSection() {
        addSectionHeader("Mouse");

        // Show position
        addLine("Position: " + formatVector(currentSnapshot_.mouse.position), 10.0f);

        // Show delta if significant
        if (glm::length(currentSnapshot_.mouse.delta) > 0.01f) {
            addLine("Delta: " + formatVector(currentSnapshot_.mouse.delta), 10.0f,
                    config_.colors.analog[0], config_.colors.analog[1], config_.colors.analog[2]);
        }

        // Show pressed buttons
        bool hasPressed = false;
        std::stringstream pressedButtons;

        for (std::uint8_t i = 0; i < static_cast<std::uint8_t>(MouseButton::BUTTON_COUNT); ++i) {
            const MouseButton button = static_cast<MouseButton>(i);
            if (currentSnapshot_.mouse.isButtonPressed(button)) {
                if (hasPressed) pressedButtons << ", ";
                pressedButtons << formatButton(button);
                hasPressed = true;
            }
        }

        if (hasPressed) {
            addLine("Buttons: " + pressedButtons.str(), 10.0f,
                    config_.colors.pressed[0], config_.colors.pressed[1], config_.colors.pressed[2]);
        }

        // Show wheel
        if (glm::length(currentSnapshot_.mouse.wheelDelta) > 0.01f) {
            addLine("Wheel: " + formatVector(currentSnapshot_.mouse.wheelDelta), 10.0f,
                    config_.colors.analog[0], config_.colors.analog[1], config_.colors.analog[2]);
        }

        // Show cursor mode
        const char* modeStr = "Unknown";
        switch (currentSnapshot_.mouse.cursorMode) {
        case CursorMode::NORMAL: modeStr = "Normal";
            break;
        case CursorMode::HIDDEN: modeStr = "Hidden";
            break;
        case CursorMode::DISABLED: modeStr = "Disabled";
            break;
        case CursorMode::CONFINED: modeStr = "Confined";
            break;
        }
        addLine("Cursor: " + std::string(modeStr), 10.0f);

        addSeparator();
    }

    void InputDebugOverlay::generateGamepadSection() {
        addSectionHeader("Gamepads");

        bool anyConnected = false;

        for (PlayerID player = 0; player < MAX_PLAYERS; ++player) {
            const auto* gamepad = currentSnapshot_.getGamepad(player);
            if (!gamepad) continue;

            anyConnected = true;
            addLine("Player " + std::to_string(player) + " (" + gamepad->name + "):", 10.0f,
                    config_.colors.header[0], config_.colors.header[1], config_.colors.header[2], true);

            // Show sticks
            if (glm::length(gamepad->leftStick) > 0.01f) {
                addLine("L-Stick: " + formatVector(gamepad->leftStick), 20.0f,
                        config_.colors.analog[0], config_.colors.analog[1], config_.colors.analog[2]);
            }

            if (glm::length(gamepad->rightStick) > 0.01f) {
                addLine("R-Stick: " + formatVector(gamepad->rightStick), 20.0f,
                        config_.colors.analog[0], config_.colors.analog[1], config_.colors.analog[2]);
            }

            // Show triggers
            if (gamepad->leftTrigger > 0.01f) {
                addLine("L-Trigger: " + formatAxis(gamepad->leftTrigger), 20.0f,
                        config_.colors.analog[0], config_.colors.analog[1], config_.colors.analog[2]);
            }

            if (gamepad->rightTrigger > 0.01f) {
                addLine("R-Trigger: " + formatAxis(gamepad->rightTrigger), 20.0f,
                        config_.colors.analog[0], config_.colors.analog[1], config_.colors.analog[2]);
            }

            // Show pressed buttons
            std::stringstream pressedButtons;
            bool hasPressed = false;

            for (std::uint16_t i = 0; i < static_cast<std::uint16_t>(GamepadButton::BUTTON_COUNT); ++i) {
                const GamepadButton button = static_cast<GamepadButton>(i);
                if (gamepad->isButtonPressed(button)) {
                    if (hasPressed) pressedButtons << ", ";
                    pressedButtons << formatButton(button);
                    hasPressed = true;
                }
            }

            if (hasPressed) {
                addLine("Buttons: " + pressedButtons.str(), 20.0f,
                        config_.colors.pressed[0], config_.colors.pressed[1], config_.colors.pressed[2]);
            }

            // Show rumble
            if (gamepad->leftMotor > 0.01f || gamepad->rightMotor > 0.01f) {
                addLine("Rumble: L=" + formatAxis(gamepad->leftMotor) + " R=" + formatAxis(gamepad->rightMotor) +
                        " (" + std::to_string(gamepad->rumbleDuration) + "s)", 20.0f,
                        config_.colors.analog[0], config_.colors.analog[1], config_.colors.analog[2]);
            }
        }

        if (!anyConnected) {
            addLine("No gamepads connected", 10.0f, 0.6f, 0.6f, 0.6f);
        }

        addSeparator();
    }

    void InputDebugOverlay::generateTouchSection() {
        addSectionHeader("Touch");

        if (currentSnapshot_.touch.activeTouchCount > 0) {
            addLine("Active touches: " + std::to_string(currentSnapshot_.touch.activeTouchCount), 10.0f,
                    config_.colors.pressed[0], config_.colors.pressed[1], config_.colors.pressed[2]);

            for (std::size_t i = 0; i < currentSnapshot_.touch.activeTouchCount && i < 5; ++i) {
                const auto* touch = currentSnapshot_.touch.getTouch(i);
                if (touch) {
                    const char* phaseStr = "Unknown";
                    switch (touch->phase) {
                    case TouchPhase::BEGAN: phaseStr = "Began";
                        break;
                    case TouchPhase::MOVED: phaseStr = "Moved";
                        break;
                    case TouchPhase::STATIONARY: phaseStr = "Stationary";
                        break;
                    case TouchPhase::ENDED: phaseStr = "Ended";
                        break;
                    case TouchPhase::CANCELLED: phaseStr = "Cancelled";
                        break;
                    }

                    addLine("Touch " + std::to_string(touch->id) + ": " + formatVector(touch->position) +
                            " (" + phaseStr + ")", 20.0f);
                }
            }

            // Show last gesture
            if (currentSnapshot_.touch.lastGesture != GestureType::NONE) {
                const char* gestureStr = "Unknown";
                switch (currentSnapshot_.touch.lastGesture) {
                case GestureType::TAP: gestureStr = "Tap";
                    break;
                case GestureType::DOUBLE_TAP: gestureStr = "Double Tap";
                    break;
                case GestureType::LONG_PRESS: gestureStr = "Long Press";
                    break;
                case GestureType::SWIPE: gestureStr = "Swipe";
                    break;
                case GestureType::PINCH: gestureStr = "Pinch";
                    break;
                case GestureType::ROTATE: gestureStr = "Rotate";
                    break;
                case GestureType::PAN: gestureStr = "Pan";
                    break;
                default: break;
                }

                addLine("Last Gesture: " + std::string(gestureStr), 10.0f,
                        config_.colors.analog[0], config_.colors.analog[1], config_.colors.analog[2]);
            }
        }
        else {
            addLine("No active touches", 10.0f, 0.6f, 0.6f, 0.6f);
        }

        addSeparator();
    }

    void InputDebugOverlay::generateActionsSection() {
        addSectionHeader("Actions");

        if (currentSnapshot_.actions.empty()) {
            addLine("No mapped actions", 10.0f, 0.6f, 0.6f, 0.6f);
        }
        else {
            std::size_t activeActions = 0;
            for (const auto& [actionId, actionState] : currentSnapshot_.actions) {
                if (actionState.isTriggered()) {
                    const std::string actionName = "Action" + std::to_string(actionId);
                    // Would normally resolve from action map

                    std::stringstream valueStr;
                    switch (actionState.type) {
                    case ActionType::BUTTON:
                        valueStr << (actionState.buttonValue ? "ON" : "OFF");
                        break;
                    case ActionType::AXIS_1D:
                        valueStr << std::fixed << std::setprecision(2) << actionState.axis1DValue;
                        break;
                    case ActionType::AXIS_2D:
                        valueStr << formatVector(actionState.axis2DValue);
                        break;
                    case ActionType::AXIS_3D:
                        valueStr << "(" << std::fixed << std::setprecision(2)
                            << actionState.axis3DValue.x << ", "
                            << actionState.axis3DValue.y << ", "
                            << actionState.axis3DValue.z << ")";
                        break;
                    }

                    const char* triggerStr = "None";
                    switch (actionState.triggerState) {
                    case TriggerEvent::STARTED: triggerStr = "Started";
                        break;
                    case TriggerEvent::ONGOING: triggerStr = "Ongoing";
                        break;
                    case TriggerEvent::COMPLETED: triggerStr = "Completed";
                        break;
                    case TriggerEvent::CANCELLED: triggerStr = "Cancelled";
                        break;
                    default: break;
                    }

                    addLine(actionName + ": " + valueStr.str() + " [" + triggerStr + "]", 10.0f,
                            config_.colors.pressed[0], config_.colors.pressed[1], config_.colors.pressed[2]);

                    if (actionState.heldDuration > 0.0f) {
                        addLine("  Held: " + std::to_string(actionState.heldDuration) + "s", 20.0f,
                                config_.colors.analog[0], config_.colors.analog[1], config_.colors.analog[2]);
                    }

                    activeActions++;
                }
            }

            if (activeActions == 0) {
                addLine("No active actions", 10.0f, 0.6f, 0.6f, 0.6f);
            }
        }

        addSeparator();
    }

    void InputDebugOverlay::generateEventsSection() {
        addSectionHeader("Recent Events");

        if (eventHistory_.empty()) {
            addLine("No recent events", 10.0f, 0.6f, 0.6f, 0.6f);
        }
        else {
            const auto currentTime = std::chrono::steady_clock::now();
            std::size_t shown = 0;

            // Show most recent events first
            for (auto it = eventHistory_.rbegin(); it != eventHistory_.rend() && shown < 10; ++it) {
                const auto& entry = *it;
                const auto elapsed = std::chrono::duration<float>(currentTime - entry.timestamp).count();

                if (elapsed > config_.historyDuration) {
                    break; // Too old
                }

                // Color based on age (fade out over time)
                const float alpha = std::max(0.3f, 1.0f - (elapsed / config_.historyDuration));

                const char* typeStr = "Unknown";
                switch (entry.event.type) {
                case InputEventType::KEY_PRESSED: typeStr = "KeyPress";
                    break;
                case InputEventType::KEY_RELEASED: typeStr = "KeyRelease";
                    break;
                case InputEventType::MOUSE_MOVED: typeStr = "MouseMove";
                    break;
                case InputEventType::MOUSE_BUTTON_PRESSED: typeStr = "MousePress";
                    break;
                case InputEventType::MOUSE_BUTTON_RELEASED: typeStr = "MouseRelease";
                    break;
                case InputEventType::GAMEPAD_BUTTON_PRESSED: typeStr = "GamepadPress";
                    break;
                case InputEventType::GAMEPAD_BUTTON_RELEASED: typeStr = "GamepadRelease";
                    break;
                case InputEventType::GAMEPAD_AXIS_MOVED: typeStr = "GamepadAxis";
                    break;
                default: break;
                }

                std::stringstream eventStr;
                eventStr << typeStr << " (D" << entry.event.deviceId << ")";
                if (elapsed < 0.5f) {
                    // Show age for very recent events
                    eventStr << " [" << std::fixed << std::setprecision(1) << elapsed << "s]";
                }

                addLine(eventStr.str(), 10.0f, alpha, alpha, alpha);
                shown++;
            }

            if (shown == 0) {
                addLine("No recent events in timeframe", 10.0f, 0.6f, 0.6f, 0.6f);
            }
        }

        addSeparator();
    }

    void InputDebugOverlay::generatePerformanceSection() {
        addSectionHeader("Performance");

        // Show current frame stats
        addLine("Frame: " + std::to_string(currentSnapshot_.frameNumber), 10.0f);
        addLine("Events: " + std::to_string(currentSnapshot_.eventCount), 10.0f);

        if (currentSnapshot_.droppedEvents > 0) {
            addLine("Dropped: " + std::to_string(currentSnapshot_.droppedEvents), 10.0f,
                    config_.colors.released[0], config_.colors.released[1], config_.colors.released[2]);
        }

        // Show timing info
        if (averageFrameTime_ > 0.0f) {
            addLine("Avg Frame: " + std::to_string(averageFrameTime_) + "ms", 10.0f,
                    config_.colors.analog[0], config_.colors.analog[1], config_.colors.analog[2]);
        }

        if (peakFrameTime_ > 0.0f) {
            addLine("Peak Frame: " + std::to_string(peakFrameTime_) + "ms", 10.0f,
                    config_.colors.analog[0], config_.colors.analog[1], config_.colors.analog[2]);
        }

        // Show graph note if enabled
        if (config_.showPerformanceGraph && !performanceSamples_.empty()) {
            addLine("Graph: " + std::to_string(performanceSamples_.size()) + " samples", 10.0f);
        }

        addSeparator();
    }

    void InputDebugOverlay::generateContextSection() {
        addSectionHeader("Context");

        addLine("Active: " + currentSnapshot_.activeContext, 10.0f,
                config_.colors.header[0], config_.colors.header[1], config_.colors.header[2]);

        const char* priorityStr = "Unknown";
        switch (currentSnapshot_.contextPriority) {
        case ContextPriority::LOWEST: priorityStr = "Lowest";
            break;
        case ContextPriority::LOW: priorityStr = "Low";
            break;
        case ContextPriority::NORMAL: priorityStr = "Normal";
            break;
        case ContextPriority::HIGH: priorityStr = "High";
            break;
        case ContextPriority::HIGHEST: priorityStr = "Highest";
            break;
        case ContextPriority::SYSTEM: priorityStr = "System";
            break;
        }
        addLine("Priority: " + std::string(priorityStr), 10.0f);

        addLine("Has Focus: " + std::string(currentSnapshot_.hasFocus ? "Yes" : "No"), 10.0f,
                currentSnapshot_.hasFocus ? config_.colors.pressed[0] : config_.colors.released[0],
                currentSnapshot_.hasFocus ? config_.colors.pressed[1] : config_.colors.released[1],
                currentSnapshot_.hasFocus ? config_.colors.pressed[2] : config_.colors.released[2]);

        if (currentSnapshot_.consumeAllInput) {
            addLine("Mode: Consume All", 10.0f,
                    config_.colors.released[0], config_.colors.released[1], config_.colors.released[2]);
        }

        addSeparator();
    }

    void InputDebugOverlay::generateDevicesSection() {
        addSectionHeader("Devices");

        // Count connected devices
        std::uint32_t keyboardCount = currentSnapshot_.keyboard.hasAnyKeyPressed() ? 1 : 0;
        std::uint32_t mouseCount = currentSnapshot_.mouse.buttons.any() ? 1 : 0;
        std::uint32_t gamepadCount = 0;

        for (const auto& gamepad : currentSnapshot_.gamepads) {
            if (gamepad.isConnected) gamepadCount++;
        }

        std::uint32_t touchCount = currentSnapshot_.touch.activeTouchCount > 0 ? 1 : 0;

        addLine("Keyboard: " + std::to_string(keyboardCount), 10.0f,
                keyboardCount > 0 ? config_.colors.pressed[0] : config_.colors.released[0],
                keyboardCount > 0 ? config_.colors.pressed[1] : config_.colors.released[1],
                keyboardCount > 0 ? config_.colors.pressed[2] : config_.colors.released[2]);

        addLine("Mouse: " + std::to_string(mouseCount), 10.0f,
                mouseCount > 0 ? config_.colors.pressed[0] : config_.colors.released[0],
                mouseCount > 0 ? config_.colors.pressed[1] : config_.colors.released[1],
                mouseCount > 0 ? config_.colors.pressed[2] : config_.colors.released[2]);

        addLine("Gamepads: " + std::to_string(gamepadCount) + "/" + std::to_string(MAX_PLAYERS), 10.0f,
                gamepadCount > 0 ? config_.colors.pressed[0] : config_.colors.released[0],
                gamepadCount > 0 ? config_.colors.pressed[1] : config_.colors.released[1],
                gamepadCount > 0 ? config_.colors.pressed[2] : config_.colors.released[2]);

        addLine("Touch: " + std::to_string(touchCount), 10.0f,
                touchCount > 0 ? config_.colors.pressed[0] : config_.colors.released[0],
                touchCount > 0 ? config_.colors.pressed[1] : config_.colors.released[1],
                touchCount > 0 ? config_.colors.pressed[2] : config_.colors.released[2]);

        addSeparator();
    }

    void InputDebugOverlay::generateHistorySection() {
        addSectionHeader("Action History");

        if (actionHistory_.empty()) {
            addLine("No recent actions", 10.0f, 0.6f, 0.6f, 0.6f);
        }
        else {
            const auto currentTime = std::chrono::steady_clock::now();
            std::size_t shown = 0;

            for (auto it = actionHistory_.rbegin(); it != actionHistory_.rend() && shown < config_.maxActionHistory; ++
                 it) {
                const auto& entry = *it;
                const auto elapsed = std::chrono::duration<float>(currentTime - entry.timestamp).count();

                if (elapsed > config_.historyDuration) {
                    break;
                }

                const float alpha = std::max(0.3f, 1.0f - (elapsed / config_.historyDuration));

                const char* stateStr = "Unknown";
                switch (entry.state.triggerState) {
                case TriggerEvent::STARTED:
                    stateStr = "Started";
                    break;
                case TriggerEvent::ONGOING:
                    stateStr = "Ongoing";
                    break;
                case TriggerEvent::COMPLETED:
                    stateStr = "Completed";
                    break;
                case TriggerEvent::CANCELLED:
                    stateStr = "Cancelled";
                    break;
                default:
                    stateStr = "None";
                    break;
                }

                addLine(entry.actionName + " " + stateStr, 10.0f, alpha, alpha, alpha);
                shown++;
            }
        }

        addSeparator();
    }

    void InputDebugOverlay::addSectionHeader(const std::string& title) {
        addLine("=== " + title + " ===", 0.0f,
                config_.colors.header[0], config_.colors.header[1], config_.colors.header[2], true);
    }

    void InputDebugOverlay::addLine(const std::string& text, const float indent,
                                    const float r, const float g, const float b, const bool bold) {
        debugLines_.emplace_back(text, r, g, b, 1.0f, indent, bold);
        currentHeight_ += config_.lineHeight;
    }

    void InputDebugOverlay::addSeparator() {
        addLine("", 0.0f);
    }

    std::string InputDebugOverlay::formatKey(const KeyCode key) const {
        return keyCodeToString(key);
    }

    std::string InputDebugOverlay::formatButton(const MouseButton button) const {
        return mouseButtonToString(button);
    }

    std::string InputDebugOverlay::formatButton(const GamepadButton button) const {
        return gamepadButtonToString(button);
    }

    std::string InputDebugOverlay::formatAxis(const float value) const {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << value;
        return ss.str();
    }

    std::string InputDebugOverlay::formatVector(const math::Vec2& vec) const {
        std::stringstream ss;
        ss << "(" << std::fixed << std::setprecision(2) << vec.x << ", " << vec.y << ")";
        return ss.str();
    }

    void InputDebugOverlay::cleanupHistory() {
        const auto currentTime = std::chrono::steady_clock::now();
        const auto cutoffTime = currentTime - std::chrono::duration<float>(config_.historyDuration);

        // Clean event history
        eventHistory_.erase(
            std::remove_if(eventHistory_.begin(), eventHistory_.end(),
                           [cutoffTime](const EventHistoryEntry& entry) {
                               return entry.timestamp < cutoffTime;
                           }),
            eventHistory_.end()
        );

        // Clean action history
        actionHistory_.erase(
            std::remove_if(actionHistory_.begin(), actionHistory_.end(),
                           [cutoffTime](const ActionHistoryEntry& entry) {
                               return entry.timestamp < cutoffTime;
                           }),
            actionHistory_.end()
        );
    }
} // namespace engine::input::debug
