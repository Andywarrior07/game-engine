/**
 * @file InputContext.h
 * @brief Input context for managing input states (gameplay, menu, etc.)
 * @author Game Engine Team
 * @date 2024
 *
 * Contexts allow different input mappings for different game states.
 */

#pragma once

#include "../core/InputTypes.h"

#include <string>
#include <unordered_set>
#include <utility>

namespace engine::input {
    /**
     * @brief Input context for managing input state transitions
     *
     * Contexts represent different input states like gameplay, menu navigation,
     * dialogue, etc. Each context can have its own set of action bindings.
     */
    class InputContext {
    public:
        /**
         * @brief Constructor
         */
        explicit InputContext(std::string  name,
                              const ContextPriority priority = ContextPriority::NORMAL) noexcept
            : name_(std::move(name))
              , priority_(priority)
              , isActive_(true)
              , consumeAllInput_(false)
              , allowsTextInput_(false) {
        }

        // ============================================================================
        // Properties
        // ============================================================================

        /**
         * @brief Get context name
         */
        [[nodiscard]] const std::string& getName() const noexcept { return name_; }

        /**
         * @brief Get priority
         */
        [[nodiscard]] ContextPriority getPriority() const noexcept { return priority_; }

        /**
         * @brief Set priority
         */
        void setPriority(const ContextPriority priority) noexcept { priority_ = priority; }

        /**
         * @brief Check if context is active
         */
        [[nodiscard]] bool isActive() const noexcept { return isActive_; }

        /**
         * @brief Activate/deactivate context
         */
        void setActive(const bool active) noexcept { isActive_ = active; }

        /**
         * @brief Check if context consumes all input
         */
        [[nodiscard]] bool consumesAllInput() const noexcept { return consumeAllInput_; }

        /**
         * @brief Set whether context consumes all input
         */
        void setConsumeAllInput(const bool consume) noexcept { consumeAllInput_ = consume; }

        /**
         * @brief Check if context allows text input
         */
        [[nodiscard]] bool allowsTextInput() const noexcept { return allowsTextInput_; }

        /**
         * @brief Set whether context allows text input
         */
        void setAllowsTextInput(const bool allow) noexcept { allowsTextInput_ = allow; }

        // ============================================================================
        // Action Management
        // ============================================================================

        /**
         * @brief Add allowed action to context
         */
        void addAllowedAction(const ActionID actionId) {
            allowedActions_.insert(actionId);
        }

        /**
         * @brief Remove allowed action
         */
        void removeAllowedAction(const ActionID actionId) {
            allowedActions_.erase(actionId);
        }

        /**
         * @brief Check if action is allowed in this context
         */
        [[nodiscard]] bool isActionAllowed(const ActionID actionId) const noexcept {
            // If no specific actions defined, allow all
            if (allowedActions_.empty()) return true;
            return allowedActions_.contains(actionId);
        }

        /**
         * @brief Add blocked action
         */
        void addBlockedAction(const ActionID actionId) {
            blockedActions_.insert(actionId);
        }

        /**
         * @brief Check if action is blocked
         */
        [[nodiscard]] bool isActionBlocked(const ActionID actionId) const noexcept {
            return blockedActions_.contains(actionId);
        }

        /**
         * @brief Clear all allowed actions
         */
        void clearAllowedActions() {
            allowedActions_.clear();
        }

        /**
         * @brief Clear all blocked actions
         */
        void clearBlockedActions() {
            blockedActions_.clear();
        }

        // ============================================================================
        // Device Filtering
        // ============================================================================

        /**
         * @brief Add allowed device type
         */
        void addAllowedDevice(const DeviceType type) {
            allowedDevices_.insert(type);
        }

        /**
         * @brief Check if device type is allowed
         */
        [[nodiscard]] bool isDeviceAllowed(const DeviceType type) const noexcept {
            if (allowedDevices_.empty()) return true;
            return allowedDevices_.contains(type);
        }

        /**
         * @brief Add blocked device type
         */
        void addBlockedDevice(const DeviceType type) {
            blockedDevices_.insert(type);
        }

        /**
         * @brief Check if device type is blocked
         */
        [[nodiscard]] bool isDeviceBlocked(const DeviceType type) const noexcept {
            return blockedDevices_.contains(type);
        }

    private:
        std::string name_;
        ContextPriority priority_;
        bool isActive_;
        bool consumeAllInput_;
        bool allowsTextInput_;

        std::unordered_set<ActionID> allowedActions_;
        std::unordered_set<ActionID> blockedActions_;
        std::unordered_set<DeviceType> allowedDevices_;
        std::unordered_set<DeviceType> blockedDevices_;
    };

    // ============================================================================
    // Predefined Contexts
    // ============================================================================

    /**
     * @brief Create gameplay context
     */
    inline std::unique_ptr<InputContext> createGameplayContext() {
        auto context = std::make_unique<InputContext>("Gameplay", ContextPriority::NORMAL);
        context->setAllowsTextInput(false);
        context->setConsumeAllInput(false);
        return context;
    }

    /**
     * @brief Create UI menu context
     */
    inline std::unique_ptr<InputContext> createMenuContext() {
        auto context = std::make_unique<InputContext>("Menu", ContextPriority::HIGH);
        context->setAllowsTextInput(false);
        context->setConsumeAllInput(true); // Menus typically consume all input
        return context;
    }

    /**
     * @brief Create text input context
     */
    inline std::unique_ptr<InputContext> createTextInputContext() {
        auto context = std::make_unique<InputContext>("TextInput", ContextPriority::HIGHEST);
        context->setAllowsTextInput(true);
        context->setConsumeAllInput(true);
        return context;
    }

    /**
     * @brief Create cutscene context
     */
    inline std::unique_ptr<InputContext> createCutsceneContext() {
        auto context = std::make_unique<InputContext>("Cutscene", ContextPriority::HIGH);
        context->setAllowsTextInput(false);
        context->setConsumeAllInput(true);
        // Only allow skip action
        return context;
    }
} // namespace engine::input
