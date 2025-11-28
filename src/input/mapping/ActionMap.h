/**
 * @file ActionMap.h
 * @brief Action mapping system for input abstraction
 * @author Game Engine Team
 * @date 2024
 *
 * Maps physical inputs to logical game actions. Supports context-based
 * bindings, composite inputs, and runtime remapping.
 */

#pragma once

#include "InputBinding.h"

#include "../core/InputSnapshot.h"
#include "../core/InputTypes.h"
#include "../devices/keyboard/KeyboardState.h"

#include <algorithm>
#include <functional>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace engine::input {
// ============================================================================
// Action Definition
// ============================================================================

/**
 * @brief Defines a logical action that can be triggered by inputs
 */
struct ActionDefinition {
  ActionID id;
  std::string name;
  ActionType type;
  std::string description;
  std::string category; // For UI grouping

  // Default bindings
  std::vector<InputBinding> defaultBindings;

  // Action properties
  bool consumeInput;          // Should this action consume the input?
  bool allowRebinding;        // Can player rebind this action?
  bool requireExactModifiers; // Must modifiers match exactly?

  ActionDefinition() noexcept
      : id(INVALID_ACTION_ID), type(ActionType::BUTTON), consumeInput(true),
        allowRebinding(true), requireExactModifiers(false) {}

  ActionDefinition(const ActionID id, std::string name,
                   const ActionType type) noexcept
      : id(id), name(std::move(name)), type(type), consumeInput(true),
        allowRebinding(true), requireExactModifiers(false) {}
};

// ============================================================================
// Action Map
// ============================================================================

/**
 * @brief Manages the mapping between inputs and actions
 *
 * Central repository for all action definitions and their bindings.
 * Handles context-aware binding resolution and composite inputs.
 */
class ActionMap {
public:
  ActionMap() noexcept : nextActionId_(1), currentContext_("Default") {}

  /**
   * @brief Initialize the action map
   * @return True if successful
   */
  bool initialize() {
    // Clear any existing data
    actions_.clear();
    actionsByName_.clear();
    contextBindings_.clear();
    actionBindings_.clear();
    compositeBindings2D_.clear();

    if constexpr (requires { actionCallbacks_; }) {
      std::unique_lock lock(callbackMutex_);
      actionCallbacks_.clear();
    }

    nextActionId_ = 1;
    currentContext_ = "Default";

    return true;
  }

  /**
   * @brief Shutdown and cleanup the action map
   */
  void shutdown() {
    // Clear all data structures
    actions_.clear();
    actionsByName_.clear();
    contextBindings_.clear();
    actionBindings_.clear();
    compositeBindings2D_.clear();

    if constexpr (requires { actionCallbacks_; }) {
      std::unique_lock lock(callbackMutex_);
      actionCallbacks_.clear();
    }

    currentContext_.clear();
  }

  /**
   * @brief Set current active context for action resolution
   * @param context Context name to activate
   */
  void setCurrentContext(const std::string &context) {
    currentContext_ = context;
  }

  /**
   * @brief Get current active context
   * @return Current context name
   */
  [[nodiscard]] const std::string &getCurrentContext() const noexcept {
    return currentContext_;
  }

  // ============================================================================
  // Action Registration
  // ============================================================================

  /**
   * @brief Register a new action
   */
  ActionID registerAction(const std::string &name,
                          const ActionType type = ActionType::BUTTON) {
    const ActionID id = nextActionId_++;
    registerAction(id, name, type);

    return id;
  }

  /**
   * @brief Register action with specific ID
   */
  void registerAction(const ActionID id, const std::string &name,
                      const ActionType type) {
    const ActionDefinition def(id, name, type);

    actions_[id] = def;
    actionsByName_[name] = id;
  }

  /**
   * @brief Register full action definition
   */
  void registerAction(const ActionDefinition &definition) {
    actions_[definition.id] = definition;
    actionsByName_[definition.name] = definition.id;

    // Add default bindings
    for (const auto &binding : definition.defaultBindings) {
      addBinding(binding);
    }
  }

  /**
   * @brief Get action definition
   */
  [[nodiscard]] const ActionDefinition *getAction(const ActionID id) const {
    const auto it = actions_.find(id);
    return it != actions_.end() ? &it->second : nullptr;
  }

  /**
   * @brief Get action by name
   */
  [[nodiscard]] const ActionDefinition *
  getAction(const std::string &name) const {
    const auto it = actionsByName_.find(name);

    if (it == actionsByName_.end())
      return nullptr;

    return getAction(it->second);
  }

  /**
   * @brief Get action type
   */
  [[nodiscard]] ActionType getActionType(const ActionID id) const {
    auto *action = getAction(id);

    return action ? action->type : ActionType::BUTTON;
  }

  // ============================================================================
  // Binding Management
  // ============================================================================

  /**
   * @brief Add input binding
   */
  void addBinding(const InputBinding &binding) {
    // Remove any existing binding for this input in the same context
    removeBindingForInput(binding);

    // Add to context-specific bindings
    contextBindings_[binding.context].push_back(binding);

    // Add to action lookup
    actionBindings_[binding.actionId].push_back(binding);

    // Sort by priority
    sortBindings(contextBindings_[binding.context]);
  }

  /**
   * @brief Remove binding
   */
  void removeBinding(const InputBinding &binding) {
    // Remove from context bindings
    if (const auto contextIt = contextBindings_.find(binding.context);
        contextIt != contextBindings_.end()) {
      auto &bindings = contextIt->second;
      std::erase_if(bindings, [&binding](const InputBinding &b) {
        return b.equals(binding);
      });
    }

    // Remove from action bindings
    if (const auto actionIt = actionBindings_.find(binding.actionId);
        actionIt != actionBindings_.end()) {
      auto &bindings = actionIt->second;
      std::erase_if(bindings, [&binding](const InputBinding &b) {
        return b.equals(binding);
      });
    }
  }

  /**
   * @brief Clear all bindings for an action
   */
  void clearActionBindings(ActionID actionId) {
    actionBindings_.erase(actionId);

    // Remove from all context bindings
    for (auto &bindings : contextBindings_ | std::views::values) {
      std::erase_if(bindings, [actionId](const InputBinding &b) {
        return b.actionId == actionId;
      });
    }
  }

  /**
   * @brief Get all bindings for a context
   */
  [[nodiscard]] std::vector<InputBinding>
  getBindingsForContext(const std::string &context) const {
    if (const auto it = contextBindings_.find(context);
        it != contextBindings_.end()) {
      return it->second;
    }

    // Also include global context bindings
    if (const auto globalIt = contextBindings_.find("Global");
        globalIt != contextBindings_.end()) {
      return globalIt->second;
    }

    return {};
  }

  /**
   * @brief Get all bindings for an action
   */
  [[nodiscard]] std::vector<InputBinding>
  getBindingsForAction(const ActionID actionId) const {
    const auto it = actionBindings_.find(actionId);
    return it != actionBindings_.end() ? it->second
                                       : std::vector<InputBinding>{};
  }

  /**
   * @brief Find action bound to specific input
   */
  [[nodiscard]] ActionID
  findActionForInput(const KeyCode key, const std::string &context,
                     const KeyModifier modifiers = KeyModifier::NONE) const {
    for (const auto bindings = getBindingsForContext(context);
         const auto &binding : bindings) {
      if (binding.inputType == InputBinding::Type::KEYBOARD &&
          binding.keyCode == key) {
        // Check modifiers if required
        if (binding.requiredModifiers == KeyModifier::NONE ||
            binding.requiredModifiers == modifiers) {
          return binding.actionId;
        }
      }
    }

    return INVALID_ACTION_ID;
  }

  [[nodiscard]] ActionID findActionForInput(const MouseButton button,
                                            const std::string &context) const {
    for (const auto bindings = getBindingsForContext(context);
         const auto &binding : bindings) {
      if (binding.inputType == InputBinding::Type::MOUSE_BUTTON &&
          binding.mouseButton == button) {
        return binding.actionId;
      }
    }

    return INVALID_ACTION_ID;
  }

  [[nodiscard]] ActionID findActionForInput(const GamepadButton button,
                                            const std::string &context) const {
    for (const auto bindings = getBindingsForContext(context);
         const auto &binding : bindings) {
      if (binding.inputType == InputBinding::Type::GAMEPAD_BUTTON &&
          binding.gamepadButton == button) {
        return binding.actionId;
      }
    }

    return INVALID_ACTION_ID;
  }

  // ============================================================================
  // Composite Bindings
  // ============================================================================

  /**
   * @brief Create composite 2D axis binding (e.g., WASD for movement)
   */
  void createComposite2D(const ActionID actionId, const std::string &context,
                         const KeyCode up, const KeyCode down,
                         const KeyCode left, const KeyCode right) {
    CompositeBinding2D composite;
    composite.actionId = actionId;
    composite.context = context;
    composite.upKey = up;
    composite.downKey = down;
    composite.leftKey = left;
    composite.rightKey = right;

    compositeBindings2D_.push_back(composite);

    // Register as special binding type
    InputBinding binding;
    binding.actionId = actionId;
    binding.context = context;
    binding.inputType = InputBinding::Type::COMPOSITE_2D;
    addBinding(binding);
  }

  /**
   * @brief Evaluate composite 2D binding
   */
  static math::Vec2 evaluateComposite2D(const CompositeBinding2D &composite,
                                        const KeyboardState &keyboard) {
    math::Vec2 result(0, 0);

    if (keyboard.isKeyPressed(composite.leftKey))
      result.x -= 1.0f;
    if (keyboard.isKeyPressed(composite.rightKey))
      result.x += 1.0f;
    if (keyboard.isKeyPressed(composite.downKey))
      result.y -= 1.0f;
    if (keyboard.isKeyPressed(composite.upKey))
      result.y += 1.0f;

    // Normalize diagonal movement
    if (glm::length2(result) > 1.0f) {
      result = glm::normalize(result);
    }

    return result;
  }

  /**
   * @brief Get all composite 2D bindings
   */
  [[nodiscard]] const std::vector<CompositeBinding2D> &
  getComposite2DBindings() const {
    return compositeBindings2D_;
  }

  // ============================================================================
  // Serialization
  // ============================================================================

  /**
   * @brief Save bindings to JSON string
   */
  [[nodiscard]] std::string serializeBindings() const {
    // Simple JSON serialization
    std::string json = "{\n  \"bindings\": [\n";

    bool first = true;
    for (const auto &bindings : contextBindings_ | std::views::values) {
      for (const auto &binding : bindings) {
        if (!first)
          json += ",\n";
        json += "    " + binding.toJSON();
        first = false;
      }
    }

    json += "\n  ]\n}";
    return json;
  }

  // TODO: Implementar esto
  /**
   * @brief Load bindings from JSON string
   */
  bool deserializeBindings(const std::string &json) {
    // Would implement JSON parsing here
    // For now, just a placeholder
    return true;
  }

  // ============================================================================
  // Utility Methods
  // ============================================================================

  /**
   * @brief Get all registered actions
   */
  [[nodiscard]] std::vector<ActionDefinition> getAllActions() const {
    std::vector<ActionDefinition> result;
    result.reserve(actions_.size());

    for (const auto &action : actions_ | std::views::values) {
      result.push_back(action);
    }

    return result;
  }

  /**
   * @brief Get all contexts with bindings
   */
  [[nodiscard]] std::vector<std::string> getAllContexts() const {
    std::vector<std::string> contexts;
    contexts.reserve(contextBindings_.size());

    for (const auto &[context, bindings] : contextBindings_) {
      if (!bindings.empty()) {
        contexts.push_back(context);
      }
    }

    return contexts;
  }

  /**
   * @brief Reset to default bindings
   */
  void resetToDefaults() {
    // Clear all current bindings
    contextBindings_.clear();
    actionBindings_.clear();
    compositeBindings2D_.clear();

    // Re-add default bindings
    for (const auto &action : actions_ | std::views::values) {
      for (const auto &binding : action.defaultBindings) {
        addBinding(binding);
      }
    }
  }

  /**
   * @brief Evalúa el estado de todas las acciones a partir del snapshot actual.
   *
   * Recorre todos los bindings registrados y resuelve los ActionState de cada
   * acción según el estado de teclado, mouse y gamepads en el snapshot.
   *
   * @param snapshot Snapshot de entrada que contiene el estado actual de
   * teclado, mouse y gamepads.
   * @return std::unordered_map<ActionID, ActionState> Mapa de estados de acción
   * indexado por ActionID.
   *
   * @note Esta implementación inicial cubre:
   * - Acciones de teclado (KeyCode → ActionType::BUTTON)
   * - Acciones de mouse (MouseButton → ActionType::BUTTON)
   * - Acciones de gamepad (GamepadButton → ActionType::BUTTON)
   * - Acciones compuestas 2D (WASD → ActionType::AXIS_2D)
   *
   * @todo Incluir soporte para AXIS_1D (sticks, triggers) y AXIS_3D si hay
   * bindings.
   */
  [[nodiscard]] std::unordered_map<ActionID, ActionState>
  evaluateActions(const InputSnapshot &snapshot) const {
    std::unordered_map<ActionID, ActionState> result;

    std::vector<InputBinding> activeBindings;

    // Agregar bindings del contexto actual
    if (auto it = contextBindings_.find(currentContext_);
        it != contextBindings_.end()) {
      activeBindings.insert(activeBindings.end(), it->second.begin(),
                            it->second.end());
    }

    // Agregar bindings globales (siempre activos)
    if (auto it = contextBindings_.find("Global");
        it != contextBindings_.end()) {
      activeBindings.insert(activeBindings.end(), it->second.begin(),
                            it->second.end());
    }

    for (const auto &binding : activeBindings) {
      ActionState state;
      state.actionId = binding.actionId;
      state.type =
          getActionType(binding.actionId); // usa ActionMap::getActionType
      state.triggerState = TriggerEvent::NONE;
      state.heldDuration = 0.0f;
      state.consumed = false;

      switch (binding.inputType) {
      case InputBinding::Type::KEYBOARD: {
        const bool pressed = snapshot.keyboard.isKeyPressed(binding.keyCode);
        const bool justPressed =
            snapshot.keyboard.isKeyJustPressed(binding.keyCode);
        const bool justReleased =
            snapshot.keyboard.isKeyJustReleased(binding.keyCode);

        state.type = ActionType::BUTTON;
        state.buttonValue = pressed;
        if (justPressed)
          state.triggerState = TriggerEvent::STARTED;
        else if (pressed)
          state.triggerState = TriggerEvent::ONGOING;
        else if (justReleased)
          state.triggerState = TriggerEvent::COMPLETED;
        break;
      }

      case InputBinding::Type::MOUSE_BUTTON: {
        const bool pressed =
            snapshot.mouse.isButtonPressed(binding.mouseButton);
        const bool justPressed =
            snapshot.mouse.isButtonJustPressed(binding.mouseButton);
        const bool justReleased =
            snapshot.mouse.isButtonJustReleased(binding.mouseButton);

        state.type = ActionType::BUTTON;
        state.buttonValue = pressed;
        if (justPressed)
          state.triggerState = TriggerEvent::STARTED;
        else if (pressed)
          state.triggerState = TriggerEvent::ONGOING;
        else if (justReleased)
          state.triggerState = TriggerEvent::COMPLETED;
        break;
      }

      case InputBinding::Type::GAMEPAD_BUTTON: {
        const auto &gp = snapshot.gamepads[binding.playerIndex];
        const bool pressed = gp.isButtonPressed(binding.gamepadButton);
        const bool justPressed = gp.isButtonJustPressed(binding.gamepadButton);
        const bool justReleased =
            gp.isButtonJustReleased(binding.gamepadButton);

        state.type = ActionType::BUTTON;
        state.buttonValue = pressed;
        if (justPressed)
          state.triggerState = TriggerEvent::STARTED;
        else if (pressed)
          state.triggerState = TriggerEvent::ONGOING;
        else if (justReleased)
          state.triggerState = TriggerEvent::COMPLETED;
        break;
      }

      case InputBinding::Type::COMPOSITE_2D: {
        for (const auto &composite : compositeBindings2D_) {
          if (composite.actionId == binding.actionId &&
              composite.context == binding.context) {
            auto vec = evaluateComposite2D(composite, snapshot.keyboard);

            state.type = ActionType::AXIS_2D;
            state.axis2DValue = vec;
            state.triggerState = (glm::length2(vec) > 0.0f)
                                     ? TriggerEvent::ONGOING
                                     : TriggerEvent::NONE;
            break;
          }
        }
        break;
      }

      default:
        break;
      }

      // Solo guardamos si hay algo relevante
      if (state.triggerState != TriggerEvent::NONE ||
          state.getFloatValue() > 0.0f) {
        result[state.actionId] = state;
      }
    }

    return result;
  }

  /**
   * @brief Register callback for action
   * @param actionId Action to monitor
   * @param callback Function to call when action state changes
   */
  void registerCallback(
      ActionID actionId,
      std::function<void(ActionID, const ActionState &)> callback) {
    std::unique_lock lock(callbackMutex_);
    actionCallbacks_[actionId].emplace_back(std::move(callback));
  }

  /**
   * @brief Clear all callbacks for action
   * @param actionId Action to clear callbacks for
   */
  void clearCallbacks(ActionID actionId) {
    std::unique_lock lock(callbackMutex_);
    actionCallbacks_.erase(actionId);
  }

  /**
   * @brief Trigger callbacks for action state changes
   * @param actionStates Current frame's action states
   * @param previousActionStates Previous frame's action states (for change
   * detection)
   */
  void triggerCallbacks(
      const std::unordered_map<ActionID, ActionState> &actionStates,
      const std::unordered_map<ActionID, ActionState> &previousActionStates)
      const {
    std::shared_lock lock(callbackMutex_);

    for (const auto &[actionId, currentState] : actionStates) {
      // Check if we have callbacks for this action
      const auto callbackIt = actionCallbacks_.find(actionId);
      if (callbackIt == actionCallbacks_.end())
        continue;

      // Get previous state for comparison
      const auto prevIt = previousActionStates.find(actionId);
      const bool hadPreviousState = prevIt != previousActionStates.end();

      // Determine if we should trigger callbacks
      bool shouldTrigger = false;

      if (!hadPreviousState) {
        // New action, trigger if it has any activity
        shouldTrigger = currentState.triggerState != TriggerEvent::NONE ||
                        currentState.getFloatValue() > 0.0f;
      } else {
        const auto &previousState = prevIt->second;

        // Trigger on state changes
        shouldTrigger =
            currentState.triggerState != previousState.triggerState ||
            (currentState.triggerState == TriggerEvent::STARTED) ||
            (previousState.triggerState == TriggerEvent::COMPLETED);
      }

      if (shouldTrigger) {
        // Call all registered callbacks for this action
        for (const auto &callback : callbackIt->second) {
          try {
            callback(actionId, currentState);
          } catch (const std::exception &e) {
            // Log error but don't crash the game
            // TODO: Add proper logging here
            // logger->error("Action callback failed for action {}: {}",
            // actionId, e.what());
          }
        }
      }
    }
  }

private:
  // ============================================================================
  // Private Methods
  // ============================================================================

  /**
   * @brief Remove any existing binding for the same input
   */
  void removeBindingForInput(const InputBinding &binding) {
    const auto it = contextBindings_.find(binding.context);
    if (it == contextBindings_.end())
      return;

    auto &bindings = it->second;
    std::erase_if(bindings, [&binding](const InputBinding &b) {
      return b.inputType == binding.inputType && b.matchesInput(binding);
    });
  }

  /**
   * @brief Sort bindings by priority
   */
  static void sortBindings(std::vector<InputBinding> &bindings) {
    std::ranges::sort(bindings,
                      [](const InputBinding &a, const InputBinding &b) {
                        return a.priority > b.priority; // Higher priority first
                      });
  }

  // ============================================================================
  // Member Variables
  // ============================================================================

  // Action definitions
  std::unordered_map<ActionID, ActionDefinition> actions_;
  std::unordered_map<std::string, ActionID> actionsByName_;

  // Binding storage
  std::unordered_map<std::string, std::vector<InputBinding>> contextBindings_;
  std::unordered_map<ActionID, std::vector<InputBinding>> actionBindings_;

  // Composite bindings
  std::vector<CompositeBinding2D> compositeBindings2D_;

  // ID generation
  ActionID nextActionId_;

  std::string currentContext_;

  std::unordered_map<
      ActionID, std::vector<std::function<void(ActionID, const ActionState &)>>>
      actionCallbacks_;
  mutable std::shared_mutex callbackMutex_; // Para thread safety
};
} // namespace engine::input
