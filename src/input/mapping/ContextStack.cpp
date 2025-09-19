/**
 * @file ContextStack.cpp
 * @brief Context stack implementation for layered input handling
 * @author Andrés Guerrero
 * @date 13-09-2025
 */

#include "ContextStack.h"

#include <algorithm>
#include <ranges>
#include <sstream>

namespace engine::input::mapping {
    ContextStack::ContextStack(MemoryAllocator* memoryManager) noexcept
        : memoryManager_(memoryManager) {
        if (!memoryManager_) {
            // In production, would get global memory manager
            ownsMemoryManager_ = false;
        }

        // Reserve space for common stack depths to avoid frequent reallocations
        contextStack_.reserve(config_.maxStackDepth);
    }

    ContextStack::~ContextStack() {
        shutdown();
    }

    // ============================================================================
    // Initialization and Configuration
    // ============================================================================

    bool ContextStack::initialize(const ContextStackConfig& config) {
        if (initialized_) {
            return false;
        }

        config_ = config;

        // Validate configuration
        if (config_.maxStackDepth == 0) {
            return false;
        }

        // Reserve space for contexts
        contextStack_.reserve(config_.maxStackDepth);

        // Initialize with default context if specified
        if (!config_.defaultContext.empty()) {
            // Create a default gameplay context
            auto defaultCtx = std::make_unique<engine::input::InputContext>(
                config_.defaultContext,
                ContextPriority::NORMAL
            );

            if (!registerContext(std::move(defaultCtx))) {
                return false;
            }

            if (!pushContext(config_.defaultContext)) {
                return false;
            }
        }

        initialized_ = true;
        return true;
    }

    void ContextStack::shutdown() {
        if (!initialized_) {
            return;
        }

        // Clear all callbacks first to prevent notifications during shutdown
        clearCallbacks();

        // Clear the stack
        clearStack();

        // Unregister all contexts
        {
            std::lock_guard lock(registryMutex_);
            contexts_.clear();
        }

        initialized_ = false;
    }

    // ============================================================================
    // Context Registration
    // ============================================================================

    bool ContextStack::registerContext(std::unique_ptr<engine::input::InputContext> context) {
        if (!context) {
            return false;
        }

        const std::string name = context->getName();

        std::lock_guard lock(registryMutex_);

        // Check if context already exists
        if (contexts_.contains(name)) {
            return false;
        }

        contexts_[name] = std::move(context);
        stats_.contextsRegistered++;

        return true;
    }

    bool ContextStack::unregisterContext(const std::string& name) {
        std::lock_guard registryLock(registryMutex_);

        // Check if context is currently on stack
        if (isContextOnStack(name)) {
            return false; // Cannot unregister active context
        }

        const auto it = contexts_.find(name);
        if (it == contexts_.end()) {
            return false;
        }

        contexts_.erase(it);
        if (stats_.contextsRegistered > 0) {
            stats_.contextsRegistered--;
        }

        return true;
    }

    InputContext* ContextStack::getContext(const std::string& name) const {
        std::lock_guard lock(registryMutex_);

        const auto it = contexts_.find(name);
        return it != contexts_.end() ? it->second.get() : nullptr;
    }

    bool ContextStack::hasContext(const std::string& name) const noexcept {
        std::lock_guard lock(registryMutex_);
        return contexts_.contains(name);
    }

    std::vector<std::string> ContextStack::getRegisteredContexts() const {
        std::lock_guard lock(registryMutex_);

        std::vector<std::string> names;
        names.reserve(contexts_.size());

        for (const auto& name : contexts_ | std::views::keys) {
            names.push_back(name);
        }

        return names;
    }

    // ============================================================================
    // Stack Operations
    // ============================================================================

    bool ContextStack::pushContext(const std::string& contextName) {
        // Validate context exists
        auto* context = getContext(contextName);
        if (!context) {
            return false;
        }

        std::lock_guard lock(stackMutex_);

        // Check stack depth limit
        if (contextStack_.size() >= config_.maxStackDepth) {
            return false;
        }

        // Check for duplicates if not allowed
        if (!config_.allowDuplicateContexts && isContextOnStack(contextName)) {
            return false;
        }

        // Deactivate current top context
        if (!contextStack_.empty()) {
            deactivateContext(contextStack_.back());
        }

        // Push new context
        contextStack_.emplace_back(context, contextName);
        auto& newEntry = contextStack_.back();

        // Activate the new context
        activateContext(newEntry);

        // Pause lower contexts if configured
        if (config_.pauseLowerContexts && contextStack_.size() > 1) {
            pauseLowerContexts(contextStack_.size() - 1);
        }

        // Update statistics
        stats_.totalPushes++;
        stats_.maxStackDepth = std::max(stats_.maxStackDepth, static_cast<std::uint32_t>(contextStack_.size()));
        updateStatistics();

        notifyTransition(contextName, ContextTransition::PUSHED);
        notifyTransition(contextName, ContextTransition::ACTIVATED);

        return true;
    }

    bool ContextStack::popContext() {
        std::lock_guard lock(stackMutex_);

        if (contextStack_.empty()) {
            return false;
        }

        // Get context being popped
        const auto& topEntry = contextStack_.back();
        const std::string contextName = topEntry.name;

        // Deactivate context
        deactivateContext(topEntry);

        // Remove from stack
        contextStack_.pop_back();

        // Update statistics
        stats_.totalPops++;
        updateStatistics();

        // Notify transition
        notifyTransition(contextName, ContextTransition::POPPED);
        notifyTransition(contextName, ContextTransition::DEACTIVATED);

        // Activate new top context if any
        if (!contextStack_.empty()) {
            auto& newTop = contextStack_.back();
            activateContext(newTop);
            notifyTransition(newTop.name, ContextTransition::ACTIVATED);

            // Resume contexts that were paused
            if (config_.pauseLowerContexts) {
                resumePausedContexts();
            }
        }

        return true;
    }

    bool ContextStack::replaceContext(const std::string& contextName) {
        // Validate context exists
        auto* context = getContext(contextName);
        if (!context) {
            return false;
        }

        std::lock_guard lock(stackMutex_);

        if (contextStack_.empty()) {
            // If stack is empty, just push
            lock.~lock_guard();
            return pushContext(contextName);
        }

        // Get current top context
        auto& topEntry = contextStack_.back();
        const std::string oldContextName = topEntry.name;

        // Deactivate current context
        deactivateContext(topEntry);

        // Replace with new context
        topEntry.context = context;
        topEntry.name = contextName;
        topEntry.isPaused = false;
        topEntry.pushTime = InputTimestamp::clock::now();

        // Activate new context
        activateContext(topEntry);

        // Update statistics
        stats_.totalTransitions++;
        updateStatistics();

        // Notify transitions
        notifyTransition(oldContextName, ContextTransition::DEACTIVATED);
        notifyTransition(contextName, ContextTransition::REPLACED);
        notifyTransition(contextName, ContextTransition::ACTIVATED);

        return true;
    }

    bool ContextStack::setContext(const std::string& contextName) {
        // Clear entire stack first
        clearStack();

        // Push the new context
        return pushContext(contextName);
    }

    void ContextStack::clearStack() noexcept {
        std::lock_guard lock(stackMutex_);

        // Deactivate and notify all contexts
        while (!contextStack_.empty()) {
            auto& entry = contextStack_.back();
            deactivateContext(entry);
            notifyTransition(entry.name, ContextTransition::DEACTIVATED);
            notifyTransition(entry.name, ContextTransition::POPPED);
            contextStack_.pop_back();
            stats_.totalPops++;
        }

        updateStatistics();
    }

    engine::input::InputContext* ContextStack::getActiveContext() const noexcept {
        std::lock_guard lock(stackMutex_);
        return contextStack_.empty() ? nullptr : contextStack_.back().context;
    }

    std::string ContextStack::getActiveContextName() const noexcept {
        std::lock_guard lock(stackMutex_);
        return contextStack_.empty() ? std::string{} : contextStack_.back().name;
    }

    // ============================================================================
    // Context State Management
    // ============================================================================

    bool ContextStack::pauseContext(const std::string& contextName) {
        std::lock_guard lock(stackMutex_);

        auto* entry = findStackEntry(contextName);
        if (!entry || entry->isPaused) {
            return false;
        }

        entry->isPaused = true;
        entry->context->setActive(false);

        notifyTransition(contextName, ContextTransition::PAUSED);
        return true;
    }

    bool ContextStack::resumeContext(const std::string& contextName) {
        std::lock_guard lock(stackMutex_);

        auto* entry = findStackEntry(contextName);
        if (!entry || !entry->isPaused) {
            return false;
        }

        entry->isPaused = false;

        // Only set active if it's the top of stack or doesn't conflict with pause policy
        if (entry == &contextStack_.back() || !config_.pauseLowerContexts) {
            entry->context->setActive(true);
        }

        notifyTransition(contextName, ContextTransition::RESUMED);
        return true;
    }

    bool ContextStack::isContextActive(const std::string& contextName) const noexcept {
        std::lock_guard lock(stackMutex_);

        if (contextStack_.empty()) {
            return false;
        }

        const auto& topEntry = contextStack_.back();
        return topEntry.name == contextName && !topEntry.isPaused;
    }

    bool ContextStack::isContextOnStack(const std::string& contextName) const noexcept {
        // std::lock_guard lock(stackMutex_);
        return findStackEntry(contextName) != nullptr;
    }

    bool ContextStack::isContextPaused(const std::string& contextName) const noexcept {
        std::lock_guard lock(stackMutex_);

        const auto* entry = findStackEntry(contextName);

        if (entry == nullptr) {
            return false;
        }

        return entry->isPaused;
    }

    // ============================================================================
    // Input Processing
    // ============================================================================

    // TODO: Revisar porque no se usa
    bool ContextStack::processEvent(const InputEvent& event) {
        std::lock_guard lock(stackMutex_);

        // Process from top to bottom of stack
        for (auto it = contextStack_.rbegin(); it != contextStack_.rend(); ++it) {
            const auto& entry = *it;

            // Skip paused contexts
            if (entry.isPaused) {
                continue;
            }

            const auto* context = entry.context;
            if (!context || !context->isActive()) {
                continue;
            }

            // Check if context consumes all input
            if (context->consumesAllInput()) {
                return true; // Event handled and consumed
            }

            // Check device filtering
            // Note: This would need DeviceType from event, which isn't in current InputEvent structure
            // For now, assume all device types are allowed

            // If we reach here and propagation is disabled, stop
            if (!config_.propagateUnhandledInput) {
                return false;
            }
        }

        return false; // Event not handled
    }

    // TODO: Revisar porque no se usa
    void ContextStack::update(const float deltaTime) const {
        std::lock_guard lock(stackMutex_);

        // Update all active contexts
        for (auto& entry : contextStack_) {
            if (!entry.isPaused && entry.context && entry.context->isActive()) {
                // Context update would go here if InputContext had an update method
                // For now, contexts are stateless
            }
        }
    }

    ContextPriority ContextStack::getEffectivePriority() const noexcept {
        std::lock_guard lock(stackMutex_);

        if (contextStack_.empty()) {
            return ContextPriority::NORMAL;
        }

        return contextStack_.back().context->getPriority();
    }

    bool ContextStack::shouldConsumeInput() const noexcept {
        std::lock_guard lock(stackMutex_);

        if (contextStack_.empty()) {
            return false;
        }

        return contextStack_.back().context->consumesAllInput();
    }

    // ============================================================================
    // Transition Callbacks
    // ============================================================================

    void ContextStack::registerTransitionCallback(TransitionCallback callback) {
        std::lock_guard lock(callbackMutex_);
        transitionCallbacks_.push_back(std::move(callback));
    }

    void ContextStack::clearCallbacks() {
        std::lock_guard lock(callbackMutex_);
        transitionCallbacks_.clear();
    }

    // ============================================================================
    // Debugging and Utilities
    // ============================================================================

    std::vector<std::string> ContextStack::getStackContents() const {
        std::lock_guard lock(stackMutex_);

        std::vector<std::string> contents;
        contents.reserve(contextStack_.size());

        for (const auto& entry : contextStack_) {
            std::string entryStr = entry.name;
            if (entry.isPaused) {
                entryStr += " [PAUSED]";
            }
            contents.push_back(entryStr);
        }

        return contents;
    }

    bool ContextStack::validateStack() const noexcept {
        std::lock_guard lock(stackMutex_);

        // Check stack depth
        if (contextStack_.size() > config_.maxStackDepth) {
            return false;
        }

        // Check for valid contexts
        for (const auto& entry : contextStack_) {
            if (!entry.context) {
                return false;
            }

            // Verify context is registered
            if (!hasContext(entry.name)) {
                return false;
            }
        }

        // Check duplicates if not allowed
        if (!config_.allowDuplicateContexts) {
            std::unordered_set<std::string> seen;
            for (const auto& entry : contextStack_) {
                if (seen.contains(entry.name)) {
                    return false;
                }
                seen.insert(entry.name);
            }
        }

        return true;
    }

    std::string ContextStack::debugString() const {
        std::lock_guard lock(stackMutex_);

        std::stringstream ss;
        ss << "ContextStack [" << contextStack_.size() << "/" << config_.maxStackDepth << "]:\n";

        for (std::size_t i = 0; i < contextStack_.size(); ++i) {
            const auto& entry = contextStack_[i];
            ss << "  [" << i << "] " << entry.name;

            if (entry.isPaused) {
                ss << " [PAUSED]";
            }

            if (i == contextStack_.size() - 1) {
                ss << " [TOP]";
            }

            ss << " (Priority: " << static_cast<int>(entry.context->getPriority()) << ")\n";
        }

        return ss.str();
    }

    // ============================================================================
    // Private Implementation
    // ============================================================================

    void ContextStack::notifyTransition(const std::string& contextName, const ContextTransition transition) const {
        std::lock_guard lock(callbackMutex_);

        for (const auto& callback : transitionCallbacks_) {
            if (callback) {
                try {
                    callback(contextName, transition);
                }
                catch (const std::exception&) {
                    // Silently ignore callback exceptions to prevent cascade failures
                }
            }
        }
    }

    void ContextStack::activateContext(StackEntry& entry) {
        if (entry.context) {
            entry.context->setActive(true);
            entry.isPaused = false;
        }
    }

    void ContextStack::deactivateContext(const StackEntry& entry) {
        if (entry.context) {
            entry.context->setActive(false);
        }
    }

    void ContextStack::pauseLowerContexts(const std::size_t topIndex) {
        for (std::size_t i = 0; i < topIndex && i < contextStack_.size(); ++i) {
            if (auto& entry = contextStack_[i]; !entry.isPaused) {
                entry.isPaused = true;
                entry.context->setActive(false);
                notifyTransition(entry.name, ContextTransition::PAUSED);
            }
        }
    }

    void ContextStack::resumePausedContexts() {
        // Resume contexts that were paused due to stacking policy
        for (auto& entry : contextStack_) {
            if (entry.isPaused && !config_.pauseLowerContexts) {
                entry.isPaused = false;
                entry.context->setActive(true);
                notifyTransition(entry.name, ContextTransition::RESUMED);
            }
        }
    }

    ContextStack::StackEntry* ContextStack::findStackEntry(const std::string& name) noexcept {
        const auto it = std::ranges::find_if(contextStack_,
                                             [&name](const StackEntry& entry) { return entry.name == name; });
        return it != contextStack_.end() ? &(*it) : nullptr;
    }

    const ContextStack::StackEntry* ContextStack::findStackEntry(const std::string& name) const noexcept {
        const auto it = std::ranges::find_if(contextStack_,
                                             [&name](const StackEntry& entry) { return entry.name == name; });
        return it != contextStack_.end() ? &(*it) : nullptr;
    }

    // TODO: Revisar porque no se usan from y to
    bool ContextStack::validateTransition(const std::string& from, const std::string& to,
                                          const ContextTransition transition) const {
        if (!config_.validateTransitions) {
            return true;
        }

        // Basic validation rules
        switch (transition) {
        case ContextTransition::PUSHED:
            // Can always push unless stack is full
            return contextStack_.size() < config_.maxStackDepth;

        case ContextTransition::POPPED:
            // Can only pop if stack is not empty
            return !contextStack_.empty();

        case ContextTransition::REPLACED:
            // Can only replace if there's something to replace
            return !contextStack_.empty();

        default:
            return true;
        }
    }

    void ContextStack::updateStatistics() const {
        stats_.totalTransitions++;
    }
} // namespace engine::input::mapping
