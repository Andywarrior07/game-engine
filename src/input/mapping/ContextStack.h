/**
 * @file ContextStack.h
 * @brief Manages layered input contexts for different game states
 * @author Andrés Guerrero
 * @date 13-09-2025
 *
 * Stack-based context management allowing for proper input handling
 * in different game states (gameplay, menu, dialogue, etc).
 */

#pragma once

#include "InputContext.h"

#include "../../memory/manager/MemoryManager.h"

#include <memory>
#include <vector>
#include <string>
#include <unordered_map>
#include <mutex>
#include <functional>

namespace engine::input::mapping {
    /**
     * @brief Context transition event
     */
    enum class ContextTransition : std::uint8_t {
        PUSHED, // New context pushed onto stack
        POPPED, // Context removed from stack
        REPLACED, // Context replaced
        ACTIVATED, // Context became active (top of stack)
        DEACTIVATED, // Context no longer active
        PAUSED, // Context paused (still on stack but not active)
        RESUMED // Context resumed from pause
    };

    /**
     * @brief Context stack configuration
     */
    struct ContextStackConfig {
        std::size_t maxStackDepth = 16; // Maximum context stack depth
        bool allowDuplicateContexts = false; // Allow same context multiple times
        bool pauseLowerContexts = true; // Pause contexts below active one
        bool propagateUnhandledInput = true; // Pass unhandled input to lower contexts
        bool validateTransitions = true; // Validate context transitions
        std::string defaultContext = "Gameplay"; // Default context name
    };

    /**
     * @brief Manages a stack of input contexts
     *
     * Provides layered input handling with proper activation/deactivation,
     * pause/resume, and input propagation between contexts.
     */
    class ContextStack {
    public:
        using MemoryAllocator = engine::memory::MemoryManager;
        using TransitionCallback = std::function<void(const std::string&, ContextTransition)>;

        /**
         * @brief Constructor
         * @param memoryManager Memory manager for allocations
         */
        explicit ContextStack(MemoryAllocator* memoryManager = nullptr) noexcept;

        /**
         * @brief Destructor
         */
        ~ContextStack();

        // Disable copy, enable move
        ContextStack(const ContextStack&) = delete;
        ContextStack& operator=(const ContextStack&) = delete;
        ContextStack(ContextStack&&) noexcept = default;
        ContextStack& operator=(ContextStack&&) noexcept = default;

        // ============================================================================
        // Initialization and Configuration
        // ============================================================================

        /**
         * @brief Initialize the context stack
         * @param config Configuration settings
         * @return True if successful
         */
        bool initialize(const ContextStackConfig& config = {});

        /**
         * @brief Shutdown the context stack
         */
        void shutdown();

        /**
         * @brief Set configuration
         * @param config New configuration
         */
        void setConfig(const ContextStackConfig& config) noexcept {
            config_ = config;
        }

        /**
         * @brief Get current configuration
         */
        [[nodiscard]] const ContextStackConfig& getConfig() const noexcept {
            return config_;
        }

        // ============================================================================
        // Context Registration
        // ============================================================================

        /**
         * @brief Register a context
         * @param context Context to register (ownership transferred)
         * @return True if registered successfully
         */
        bool registerContext(std::unique_ptr<InputContext> context);

        /**
         * @brief Unregister a context
         * @param name Context name
         * @return True if unregistered
         */
        bool unregisterContext(const std::string& name);

        /**
         * @brief Get registered context
         * @param name Context name
         * @return Context pointer or nullptr
         */
        [[nodiscard]] InputContext* getContext(const std::string& name) const;

        /**
         * @brief Check if context is registered
         * @param name Context name
         * @return True if registered
         */
        [[nodiscard]] bool hasContext(const std::string& name) const noexcept;

        /**
         * @brief Get all registered context names
         */
        [[nodiscard]] std::vector<std::string> getRegisteredContexts() const;

        // ============================================================================
        // Stack Operations
        // ============================================================================

        /**
         * @brief Push context onto stack
         * @param contextName Name of context to push
         * @return True if pushed successfully
         */
        bool pushContext(const std::string& contextName);

        /**
         * @brief Pop top context from stack
         * @return True if popped successfully
         */
        bool popContext();

        /**
         * @brief Replace top context
         * @param contextName Name of new context
         * @return True if replaced successfully
         */
        bool replaceContext(const std::string& contextName);

        /**
         * @brief Clear stack and push context
         * @param contextName Name of context to set
         * @return True if successful
         */
        bool setContext(const std::string& contextName);

        /**
         * @brief Clear entire stack
         */
        void clearStack() noexcept;

        /**
         * @brief Get active context (top of stack)
         * @return Active context or nullptr
         */
        [[nodiscard]] InputContext* getActiveContext() const noexcept;

        /**
         * @brief Get active context name
         * @return Name of active context or empty string
         */
        [[nodiscard]] std::string getActiveContextName() const noexcept;

        /**
         * @brief Get stack depth
         */
        [[nodiscard]] std::size_t getStackDepth() const noexcept {
            return contextStack_.size();
        }

        /**
         * @brief Check if stack is empty
         */
        [[nodiscard]] bool isEmpty() const noexcept {
            return contextStack_.empty();
        }

        // ============================================================================
        // Context State Management
        // ============================================================================

        /**
         * @brief Pause context
         * @param contextName Context to pause
         * @return True if paused
         */
        bool pauseContext(const std::string& contextName);

        /**
         * @brief Resume context
         * @param contextName Context to resume
         * @return True if resumed
         */
        bool resumeContext(const std::string& contextName);

        /**
         * @brief Check if context is active
         * @param contextName Context name
         * @return True if context is active (top of stack)
         */
        [[nodiscard]] bool isContextActive(const std::string& contextName) const noexcept;

        /**
         * @brief Check if context is on stack
         * @param contextName Context name
         * @return True if context is anywhere on stack
         */
        [[nodiscard]] bool isContextOnStack(const std::string& contextName) const noexcept;

        /**
         * @brief Check if context is paused
         * @param contextName Context name
         * @return True if context is paused
         */
        [[nodiscard]] bool isContextPaused(const std::string& contextName) const noexcept;

        // ============================================================================
        // Input Processing
        // ============================================================================

        /**
         * @brief Process input event through context stack
         * @param event Input event to process
         * @return True if event was handled
         */
        bool processEvent(const InputEvent& event);

        /**
         * @brief Update all contexts on stack
         * @param deltaTime Time since last update
         */
        void update(float deltaTime) const;

        /**
         * @brief Get effective priority for current context
         * @return Priority of active context
         */
        [[nodiscard]] ContextPriority getEffectivePriority() const noexcept;

        /**
         * @brief Check if input should be consumed at current context
         * @return True if input should be consumed
         */
        [[nodiscard]] bool shouldConsumeInput() const noexcept;

        // ============================================================================
        // Transition Callbacks
        // ============================================================================

        /**
         * @brief Register transition callback
         * @param callback Callback function
         */
        void registerTransitionCallback(TransitionCallback callback);

        /**
         * @brief Clear all callbacks
         */
        void clearCallbacks();

        // ============================================================================
        // Debugging and Utilities
        // ============================================================================

        /**
         * @brief Get stack contents as strings
         * @return Vector of context names from bottom to top
         */
        [[nodiscard]] std::vector<std::string> getStackContents() const;

        /**
         * @brief Validate stack integrity
         * @return True if stack is valid
         */
        [[nodiscard]] bool validateStack() const noexcept;

        /**
         * @brief Generate debug string of stack state
         */
        [[nodiscard]] std::string debugString() const;

        /**
         * @brief Stack statistics
         */
        struct Statistics {
            std::uint64_t totalPushes = 0;
            std::uint64_t totalPops = 0;
            std::uint64_t totalTransitions = 0;
            std::uint32_t maxStackDepth = 0;
            std::uint32_t contextsRegistered = 0;

            void reset() noexcept {
                totalPushes = 0;
                totalPops = 0;
                totalTransitions = 0;
                maxStackDepth = 0;
                contextsRegistered = 0;
            }
        };

        [[nodiscard]] const Statistics& getStatistics() const noexcept {
            return stats_;
        }

        void resetStatistics() const noexcept {
            stats_.reset();
        }

    private:
        // ============================================================================
        // Internal Types
        // ============================================================================

        /**
         * @brief Stack entry for a context
         */
        struct StackEntry {
            InputContext* context;
            std::string name;
            bool isPaused;
            InputTimestamp pushTime;

            StackEntry(InputContext* ctx, const std::string& n) noexcept
                : context(ctx)
                  , name(n)
                  , isPaused(false)
                  , pushTime(InputTimestamp::clock::now()) {
            }
        };

        // ============================================================================
        // Member Variables
        // ============================================================================

        // Configuration
        ContextStackConfig config_;
        bool initialized_ = false;

        // Memory management
        MemoryAllocator* memoryManager_;
        bool ownsMemoryManager_ = false;

        // Context registry
        std::unordered_map<std::string, std::unique_ptr<InputContext>> contexts_;

        // Context stack
        std::vector<StackEntry> contextStack_; // Using vector for iteration

        // Thread safety
        mutable std::mutex stackMutex_;
        mutable std::mutex registryMutex_;

        // Callbacks
        std::vector<TransitionCallback> transitionCallbacks_;
        mutable std::mutex callbackMutex_;

        // Statistics
        mutable Statistics stats_;

        // ============================================================================
        // Internal Methods
        // ============================================================================

        /**
         * @brief Notify transition callbacks
         * @param contextName Context that transitioned
         * @param transition Type of transition
         */
        void notifyTransition(const std::string& contextName,
                              ContextTransition transition) const;

        /**
         * @brief Activate context (called when becoming top of stack)
         * @param entry Stack entry to activate
         */
        static void activateContext(StackEntry& entry);

        /**
         * @brief Deactivate context (called when no longer top of stack)
         * @param entry Stack entry to deactivate
         */
        static void deactivateContext(const StackEntry& entry);

        /**
         * @brief Pause contexts below the given index
         * @param topIndex Index of top context
         */
        void pauseLowerContexts(std::size_t topIndex);

        /**
         * @brief Resume contexts that were paused
         */
        void resumePausedContexts();

        /**
         * @brief Find stack entry by name
         * @param name Context name
         * @return Pointer to entry or nullptr
         */
        [[nodiscard]] StackEntry* findStackEntry(const std::string& name) noexcept;
        [[nodiscard]] const StackEntry* findStackEntry(const std::string& name) const noexcept;

        /**
         * @brief Validate context transition
         * @param from Source context
         * @param to Target context
         * @param transition Transition type
         * @return True if transition is valid
         */
        [[nodiscard]] bool validateTransition(const std::string& from,
                                              const std::string& to,
                                              ContextTransition transition) const;

        /**
         * @brief Update statistics
         */
        void updateStatistics() const;
    };
} // namespace engine::input::mapping
