/**
 * @file InputBinding.h
 * @brief Input binding structure for action mapping
 * @author Andrés Guerrero
 * @date 11-08-2025
 *
 * Defines how physical inputs map to logical actions with context awareness.
 */

#pragma once

#include "../core/InputTypes.h"

#include <string>

namespace engine::input {
    /**
     * @brief Represents a binding between physical input and action
     */
    struct InputBinding {
        enum class Type : std::uint8_t {
            KEYBOARD,
            MOUSE_BUTTON,
            MOUSE_AXIS,
            GAMEPAD_BUTTON,
            GAMEPAD_AXIS,
            TOUCH,
            COMPOSITE_2D,
            COMPOSITE_3D
        };

        // Binding identification
        ActionID actionId;
        std::string context;
        Type inputType;

        // Input specifiers (union would be more efficient but less clear)
        KeyCode keyCode;
        MouseButton mouseButton;
        GamepadButton gamepadButton;
        GamepadAxis gamepadAxis;

        // Modifiers and conditions
        KeyModifier requiredModifiers;
        float scale; // Scale factor for axis inputs
        bool invert; // Invert axis direction
        bool consumeInput; // Should this binding consume the input?

        // Binding properties
        float priority; // Higher priority bindings are evaluated first
        float holdTime; // Required hold time for activation (0 for instant)
        float repeatDelay; // Delay before repeat starts
        float repeatRate; // Rate of repeat

        // TODO: cuando quieras soportar multi-jugador real tendrás que empezar a setear playerIndex en los bindings al momento de registrarlos.
        std::uint8_t playerIndex; // Player index for gamepad bindings

        InputBinding() noexcept
            : actionId(INVALID_ACTION_ID)
              , context("Default")
              , inputType(Type::KEYBOARD)
              , keyCode(KeyCode::UNKNOWN)
              , mouseButton(MouseButton::NONE)
              , gamepadButton(GamepadButton::NONE)
              , gamepadAxis(GamepadAxis::NONE)
              , requiredModifiers(KeyModifier::NONE)
              , scale(1.0f)
              , invert(false)
              , consumeInput(true)
              , priority(1.0f)
              , holdTime(0.0f)
              , repeatDelay(0.5f)
              , repeatRate(0.1f)
              , playerIndex(0) {
        }

        /**
         * @brief Check if binding matches another binding's input
         */
        [[nodiscard]] bool matchesInput(const InputBinding& other) const noexcept {
            if (inputType != other.inputType) return false;

            switch (inputType) {
            case Type::KEYBOARD:
                return keyCode == other.keyCode;
            case Type::MOUSE_BUTTON:
                return mouseButton == other.mouseButton;
            case Type::GAMEPAD_BUTTON:
                return gamepadButton == other.gamepadButton;
            case Type::GAMEPAD_AXIS:
                return gamepadAxis == other.gamepadAxis;
            default:
                return false;
            }
        }

        /**
         * @brief Check if bindings are equal
         */
        [[nodiscard]] bool equals(const InputBinding& other) const noexcept {
            return actionId == other.actionId &&
                context == other.context &&
                matchesInput(other);
        }

        /**
         * @brief Convert to JSON string
         */
        [[nodiscard]] std::string toJSON() const {
            std::string json = "{";
            json += "\"action\":" + std::to_string(actionId) + ",";
            json += "\"context\":\"" + context + "\",";
            json += "\"type\":" + std::to_string(static_cast<int>(inputType)) + ",";

            switch (inputType) {
            case Type::KEYBOARD:
                json += "\"key\":" + std::to_string(static_cast<int>(keyCode));
                break;
            case Type::MOUSE_BUTTON:
                json += "\"mouseButton\":" + std::to_string(static_cast<int>(mouseButton));
                break;
            case Type::GAMEPAD_BUTTON:
                json += "\"gamepadButton\":" + std::to_string(static_cast<int>(gamepadButton));
                break;
            case Type::GAMEPAD_AXIS:
                json += "\"gamepadAxis\":" + std::to_string(static_cast<int>(gamepadAxis));
                break;
            default:
                break;
            }

            if (scale != 1.0f) {
                json += ",\"scale\":" + std::to_string(scale);
            }
            if (invert) {
                json += ",\"invert\":true";
            }

            json += "}";
            return json;
        }
    };

    /**
     * @brief Composite 2D binding (e.g., WASD movement)
     */
    struct CompositeBinding2D {
        ActionID actionId;
        std::string context;

        // Four-way input
        KeyCode upKey;
        KeyCode downKey;
        KeyCode leftKey;
        KeyCode rightKey;

        // Alternative gamepad mapping
        GamepadAxis xAxis;
        GamepadAxis yAxis;

        // Properties
        bool normalize; // Normalize diagonal movement
        float deadzone; // Deadzone for analog input

        CompositeBinding2D() noexcept
            : actionId(INVALID_ACTION_ID)
              , context("Default")
              , upKey(KeyCode::W)
              , downKey(KeyCode::S)
              , leftKey(KeyCode::A)
              , rightKey(KeyCode::D)
              , xAxis(GamepadAxis::LEFT_STICK_X)
              , yAxis(GamepadAxis::LEFT_STICK_Y)
              , normalize(true)
              , deadzone(0.15f) {
        }
    };
} // namespace engine::input
