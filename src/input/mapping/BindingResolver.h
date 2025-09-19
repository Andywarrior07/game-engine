/**
 * @file BindingResolver.h
 * @brief Resolves input bindings to actions based on current input state
 * @author Andrés Guerrero
 * @date 13-09-2025
 *
 * High-performance binding resolution with conflict handling, priority
 * management, and context-aware mapping. Zero allocations in hot paths.
 */

#pragma once

#include "InputBinding.h"

#include "../core/InputEvent.h"
#include "../core/InputSnapshot.h"

#include <vector>
#include <unordered_map>
#include <algorithm>
#include <array>

namespace engine::input::mapping {
    /**
     * @brief Binding conflict resolution strategy
     */
    enum class ConflictResolution : std::uint8_t {
        PRIORITY_BASED, // Higher priority wins
        FIRST_MATCH, // First matching binding wins
        LAST_MATCH, // Last matching binding wins
        ALL_MATCHES, // Trigger all matching actions
        CONTEXT_PRIORITY // Context priority determines winner
    };

    /**
     * @brief Binding evaluation result
     */
    struct BindingMatch {
        const InputBinding* binding;
        float value; // Evaluated value (0-1 for buttons, -1 to 1 for axes)
        float confidence; // Match confidence (0-1)
        bool isActive; // Is binding currently active
        bool wasJustTriggered; // Just became active this frame
        bool wasJustReleased; // Just became inactive this frame

        BindingMatch() noexcept
            : binding(nullptr)
              , value(0.0f)
              , confidence(1.0f)
              , isActive(false)
              , wasJustTriggered(false)
              , wasJustReleased(false) {
        }

        [[nodiscard]] bool isValid() const noexcept {
            return binding != nullptr;
        }
    };

    /**
     * @brief Binding resolver configuration
     */
    struct BindingResolverConfig {
        ConflictResolution conflictStrategy = ConflictResolution::PRIORITY_BASED;
        bool allowSimultaneousBindings = true; // Allow multiple bindings per action
        bool requireExactModifiers = false; // Require exact modifier match
        bool consumeResolvedInputs = true; // Mark inputs as consumed
        float analogThreshold = 0.3f; // Threshold for analog activation
        float holdTimeGranularity = 0.016f; // Hold time check interval
        std::size_t maxBindingsPerAction = 8; // Maximum bindings per action
        std::size_t maxActiveBindings = 64; // Maximum simultaneous active bindings
    };

    /**
     * @brief High-performance binding resolver
     *
     * Resolves input bindings to actions with O(1) average lookup time.
     * Designed for zero allocations during frame processing.
     */
    class BindingResolver {
    public:
        /**
         * @brief Constructor
         */
        explicit BindingResolver() noexcept;

        /**
         * @brief Destructor
         */
        ~BindingResolver() = default;

        // Disable copy, enable move
        BindingResolver(const BindingResolver&) = delete;
        BindingResolver& operator=(const BindingResolver&) = delete;
        BindingResolver(BindingResolver&&) noexcept = default;
        BindingResolver& operator=(BindingResolver&&) noexcept = default;

        // ============================================================================
        // Configuration
        // ============================================================================

        /**
         * @brief Set resolver configuration
         * @param config New configuration
         */
        void setConfig(const BindingResolverConfig& config) noexcept {
            config_ = config;
        }

        /**
         * @brief Get current configuration
         */
        [[nodiscard]] const BindingResolverConfig& getConfig() const noexcept {
            return config_;
        }

        // ============================================================================
        // Binding Management
        // ============================================================================

        /**
         * @brief Add binding to resolver
         * @param binding Binding to add
         * @return True if added successfully
         */
        bool addBinding(const InputBinding& binding);

        /**
         * @brief Remove binding from resolver
         * @param binding Binding to remove
         * @return True if removed
         */
        bool removeBinding(const InputBinding& binding);

        /**
         * @brief Remove all bindings for action
         * @param actionId Action ID
         * @return Number of bindings removed
         */
        std::size_t removeBindingsForAction(ActionID actionId);

        /**
         * @brief Clear all bindings
         */
        void clearBindings() noexcept;

        /**
         * @brief Get all bindings for action
         * @param actionId Action ID
         * @return Vector of bindings
         */
        [[nodiscard]] std::vector<const InputBinding*> getBindingsForAction(ActionID actionId) const;

        /**
         * @brief Get binding count
         */
        [[nodiscard]] std::size_t getBindingCount() const noexcept {
            return bindings_.size();
        }

        // ============================================================================
        // Resolution from Events
        // ============================================================================

        /**
         * @brief Resolve binding from input event
         * @param event Input event
         * @param context Current context name
         * @return Matching binding or empty match
         */
        [[nodiscard]] BindingMatch resolveEvent(const InputEvent& event,
                                                const std::string& context) const;

        /**
         * @brief Resolve all bindings from event
         * @param event Input event
         * @param context Current context name
         * @return All matching bindings
         */
        [[nodiscard]] std::vector<BindingMatch> resolveAllEvents(const InputEvent& event,
                                                                 const std::string& context) const;

        // ============================================================================
        // Resolution from Snapshot
        // ============================================================================

        /**
         * @brief Resolve all active bindings from snapshot
         * @param snapshot Input snapshot
         * @param context Current context name
         * @return Map of action IDs to binding matches
         */
        [[nodiscard]] std::unordered_map<ActionID, BindingMatch>
        resolveSnapshot(const InputSnapshot& snapshot,
                        const std::string& context) const;

        /**
         * @brief Resolve specific action from snapshot
         * @param actionId Action to resolve
         * @param snapshot Input snapshot
         * @param context Current context name
         * @return Binding match or empty match
         */
        [[nodiscard]] BindingMatch resolveAction(ActionID actionId,
                                                 const InputSnapshot& snapshot,
                                                 const std::string& context) const;

        /**
         * @brief Evaluate composite 2D binding
         * @param binding Composite binding
         * @param snapshot Input snapshot
         * @return 2D vector value
         */
        [[nodiscard]] Vector2 evaluateComposite2D(const CompositeBinding2D& binding,
                                                  const InputSnapshot& snapshot) const;

        // ============================================================================
        // Conflict Resolution
        // ============================================================================

        /**
         * @brief Check if bindings conflict
         * @param a First binding
         * @param b Second binding
         * @return True if bindings conflict
         */
        [[nodiscard]] bool hasConflict(const InputBinding& a,
                                       const InputBinding& b) const noexcept;

        /**
         * @brief Resolve conflicts between matches
         * @param matches Binding matches to resolve
         * @return Resolved match based on conflict strategy
         */
        [[nodiscard]] BindingMatch resolveConflicts(const std::vector<BindingMatch>& matches) const;

        // ============================================================================
        // Hold Time Tracking
        // ============================================================================

        /**
         * @brief Update hold times for active bindings
         * @param deltaTime Time since last update
         */
        void updateHoldTimes(float deltaTime);

        /**
         * @brief Reset hold time for binding
         * @param binding Binding to reset
         */
        void resetHoldTime(const InputBinding& binding);

        /**
         * @brief Get current hold time for binding
         * @param binding Binding to check
         * @return Current hold time in seconds
         */
        [[nodiscard]] float getHoldTime(const InputBinding& binding) const;

        // ============================================================================
        // Statistics
        // ============================================================================

        struct Statistics {
            std::uint64_t totalResolutions = 0;
            std::uint64_t conflictsResolved = 0;
            std::uint32_t averageMatchesPerResolution = 0;
            std::uint32_t peakActiveBindings = 0;
            float averageResolutionTime = 0.0f;

            void reset() noexcept {
                totalResolutions = 0;
                conflictsResolved = 0;
                averageMatchesPerResolution = 0;
                peakActiveBindings = 0;
                averageResolutionTime = 0.0f;
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
         * @brief Internal binding state tracking
         */
        struct BindingState {
            InputBinding binding;
            float holdTime;
            bool wasActive;
            InputTimestamp lastActivation;

            BindingState() noexcept
                : holdTime(0.0f)
                  , wasActive(false)
                  , lastActivation(InputTimestamp::clock::now()) {
            }
        };

        /**
         * @brief Fast lookup key for bindings
         */
        struct BindingKey {
            InputBinding::Type type;

            union {
                KeyCode key;
                MouseButton mouseButton;
                GamepadButton gamepadButton;
                GamepadAxis gamepadAxis;
                std::uint32_t value;
            };

            BindingKey() noexcept : type(InputBinding::Type::KEYBOARD), value(0) {
            }

            bool operator==(const BindingKey& other) const noexcept {
                return type == other.type && value == other.value;
            }
        };

        /**
         * @brief Hash function for binding keys
         */
        struct BindingKeyHash {
            std::size_t operator()(const BindingKey& key) const noexcept {
                return std::hash<std::uint32_t>{}(key.value) ^
                    (std::hash<std::uint8_t>{}(static_cast<std::uint8_t>(key.type)) << 1);
            }
        };

        // ============================================================================
        // Member Variables
        // ============================================================================

        // Configuration
        BindingResolverConfig config_;

        // Binding storage
        std::vector<BindingState> bindings_;

        // Fast lookups
        std::unordered_map<ActionID, std::vector<std::size_t>> actionToBindings_;
        std::unordered_map<BindingKey, std::vector<std::size_t>, BindingKeyHash> inputToBindings_;

        // Active binding tracking
        mutable std::array<std::size_t, 64> activeBindingIndices_;
        mutable std::size_t activeBindingCount_ = 0;

        // Composite bindings
        std::vector<CompositeBinding2D> compositeBindings_;

        // Statistics
        mutable Statistics stats_;

        // ============================================================================
        // Internal Methods
        // ============================================================================

        /**
         * @brief Build lookup tables after binding changes
         */
        void rebuildLookupTables();

        /**
         * @brief Create binding key from binding
         */
        [[nodiscard]] BindingKey createKey(const InputBinding& binding) const noexcept;

        /**
         * @brief Check if keyboard input matches binding
         */
        [[nodiscard]] bool matchesKeyboard(const InputBinding& binding,
                                           const KeyboardState& keyboard) const noexcept;

        /**
         * @brief Check if mouse input matches binding
         */
        [[nodiscard]] bool matchesMouse(const InputBinding& binding,
                                        const MouseState& mouse) const noexcept;

        /**
         * @brief Check if gamepad input matches binding
         */
        [[nodiscard]] bool matchesGamepad(const InputBinding& binding,
                                          const GamepadState& gamepad) const noexcept;

        /**
         * @brief Evaluate binding value from snapshot
         */
        [[nodiscard]] float evaluateBinding(const InputBinding& binding,
                                            const InputSnapshot& snapshot) const noexcept;

        /**
         * @brief Check if modifiers match
         */
        [[nodiscard]] bool checkModifiers(KeyModifier required,
                                          KeyModifier current) const noexcept;

        /**
         * @brief Update statistics after resolution
         */
        void updateStatistics(std::size_t matchCount, float resolutionTime) const;
    };
} // namespace engine::input::mapping
