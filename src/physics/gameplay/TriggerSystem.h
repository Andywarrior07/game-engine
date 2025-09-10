/**
 * @file TriggerSystem.h
 * @brief Trigger zone system for gameplay events
 * @details Manages trigger volumes, overlap detection, and event dispatching
 *          for gameplay mechanics like checkpoints, traps, and interactive zones
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

#include "../core/PhysicsTypes.h"

#include <BulletCollision/CollisionDispatch/btGhostObject.h>

#include <functional>
#include <unordered_set>

// TODO: In the future, move it to the module manager, as it is a preset
namespace engine::physics {
    /**
     * @brief Trigger zone types
     */
    enum class TriggerType : std::uint8_t {
        VOLUME, // Simple volume trigger
        CHECKPOINT, // Save/checkpoint trigger
        DAMAGE, // Damage dealing zone
        HEAL, // Healing zone
        TELEPORT, // Teleportation trigger
        LADDER, // Climbable area
        WATER, // Water volume
        WIND, // Wind zone
        DIALOGUE, // Dialogue trigger
        CUTSCENE, // Cutscene trigger
        COLLECTIBLE, // Item pickup
        SWITCH, // Interactive switch
        PRESSURE_PLATE, // Pressure activated
        CUSTOM // User-defined
    };

    /**
     * @brief Trigger activation modes
     */
    enum class TriggerActivation : std::uint8_t {
        ON_ENTER, // Activate on entry
        ON_EXIT, // Activate on exit
        ON_STAY, // Continuous while inside
        ON_ENTER_EXIT, // Both enter and exit
        ON_PRESSURE, // Weight/count based
        ON_INTERACT // Manual interaction required
    };

    /**
     * @brief Trigger filter flags
     */
    enum TriggerFilter : std::uint16_t {
        FILTER_NONE = 0,
        FILTER_PLAYER = 1 << 0,
        FILTER_ENEMY = 1 << 1,
        FILTER_NPC = 1 << 2,
        FILTER_VEHICLE = 1 << 3,
        FILTER_PROJECTILE = 1 << 4,
        FILTER_ITEM = 1 << 5,
        FILTER_ALL = 0xFFFF
    };

    /**
     * @brief Trigger event data
     */
    struct TriggerEvent {
        class TriggerZone* trigger;
        RigidBody* body;
        Vec3 contactPoint;
        Float time;
        bool isEntering;

        TriggerEvent() : trigger(nullptr), body(nullptr), contactPoint(VEC3_ZERO), time(0), isEntering(true) {
        }
    };

    /**
     * @brief Trigger zone callbacks
     */
    using TriggerCallback = std::function<void(const TriggerEvent&)>;

    /**
     * @brief Trigger zone class
     * @details Represents a spatial trigger volume that detects overlaps
     *          and fires events for gameplay mechanics
     */
    class TriggerZone {
    public:
        explicit TriggerZone(const std::string& name, const TriggerType type = TriggerType::VOLUME)
            : m_name(name), m_type(type), m_activation(TriggerActivation::ON_ENTER),
              m_filter(FILTER_ALL), m_enabled(true), m_oneShot(false),
              m_hasBeenTriggered(false), m_cooldownTime(0), m_timeSinceTrigger(0),
              m_ghostObject(nullptr) {
        }

        ~TriggerZone();

        // ============================================================================
        // Initialization
        // ============================================================================

        /**
         * @brief Initialize trigger with shape and position
         */
        bool initialize(CollisionShape* shape, const Vec3& position,
                        PhysicsWorld* world);

        void cleanup();

        // ============================================================================
        // Update
        // ============================================================================

        /**
         * @brief Update trigger zone
         */
        void update(Float deltaTime);

        // ============================================================================
        // Configuration
        // ============================================================================

        void setEnabled(const bool enabled) { m_enabled = enabled; }
        bool isEnabled() const { return m_enabled; }

        void setActivation(const TriggerActivation activation) { m_activation = activation; }
        TriggerActivation getActivation() const { return m_activation; }

        void setFilter(const std::uint16_t filter) { m_filter = filter; }
        std::uint16_t getFilter() const { return m_filter; }

        void setOneShot(const bool oneShot) { m_oneShot = oneShot; }
        bool isOneShot() const { return m_oneShot; }

        void setCooldown(const Float seconds) { m_cooldownTime = seconds; }
        Float getCooldown() const { return m_cooldownTime; }

        void setRequiredWeight(const Float weight) { m_requiredWeight = weight; }
        Float getRequiredWeight() const { return m_requiredWeight; }

        void setRequiredCount(const Int count) { m_requiredCount = count; }
        Int getRequiredCount() const { return m_requiredCount; }

        // ============================================================================
        // Callbacks
        // ============================================================================

        void setOnEnterCallback(const TriggerCallback& callback) {
            m_onEnterCallback = callback;
        }

        void setOnExitCallback(const TriggerCallback& callback) {
            m_onExitCallback = callback;
        }

        void setOnStayCallback(const TriggerCallback& callback) {
            m_onStayCallback = callback;
        }

        void setOnActivateCallback(const TriggerCallback& callback) {
            m_onActivateCallback = callback;
        }

        // ============================================================================
        // Queries
        // ============================================================================

        const std::string& getName() const { return m_name; }
        TriggerType getType() const { return m_type; }

        Vec3 getPosition() const;

        void setPosition(const Vec3& position) const;

        bool isBodyInside(RigidBody* body) const {
            return m_overlappingBodies.contains(body);
        }

        std::size_t getOverlapCount() const { return m_overlappingBodies.size(); }

        const std::unordered_set<RigidBody*>& getOverlappingBodies() const {
            return m_overlappingBodies;
        }

        Float getTotalWeight() const;

        /**
         * @brief Force activation (for manual triggers)
         */
        void activate();

        /**
         * @brief Reset trigger (for one-shot triggers)
         */
        void reset();

    private:
        // Basic properties
        std::string m_name;
        TriggerType m_type;
        TriggerActivation m_activation;
        std::uint16_t m_filter;
        bool m_enabled;
        bool m_oneShot;
        bool m_hasBeenTriggered;

        // Timing
        Float m_cooldownTime;
        Float m_timeSinceTrigger;

        // Pressure plate settings
        Float m_requiredWeight = 0;
        Int m_requiredCount = 1;
        bool m_isActivated = false;

        // Physics objects
        PhysicsWorld* m_world = nullptr;
        CollisionShape* m_shape = nullptr;
        btPairCachingGhostObject* m_ghostObject;

        // Tracking
        std::unordered_set<RigidBody*> m_overlappingBodies;

        // Callbacks
        TriggerCallback m_onEnterCallback;
        TriggerCallback m_onExitCallback;
        TriggerCallback m_onStayCallback;
        TriggerCallback m_onActivateCallback;

        void checkOverlaps(std::unordered_set<RigidBody*>& overlaps) const;

        bool passesFilter(RigidBody* body) const {
            // Check filter flags
            // This would check body type against filter
            return true; // Simplified for now
        }

        void onBodyEnter(RigidBody* body);

        void onBodyExit(RigidBody* body);

        void onBodyStay(RigidBody* body, Float deltaTime);

        void updatePressurePlate();
    };

    /**
     * @brief Manager for trigger zones
     */
    class TriggerSystem {
    public:
        TriggerSystem() = default;

        /**
         * @brief Create a trigger zone
         */
        TriggerZone* createTrigger(const std::string& name, TriggerType type,
                                   CollisionShape* shape, const Vec3& position,
                                   PhysicsWorld* world);

        /**
         * @brief Remove a trigger zone
         */
        void removeTrigger(const std::string& name);

        /**
         * @brief Find trigger by name
         */
        TriggerZone* findTrigger(const std::string& name) const;

        /**
         * @brief Update all triggers
         */
        void update(Float deltaTime);

        /**
         * @brief Get all active triggers
         */
        const std::unordered_set<TriggerZone*>& getActiveTriggers() const {
            return m_activeTriggers;
        }

        /**
         * @brief Clear all triggers
         */
        void clear();

        /**
         * @brief Create common trigger presets
         */
        TriggerZone* createCheckpoint(const std::string& name, const Vec3& position,
                                      Float radius, PhysicsWorld* world);

        TriggerZone* createDamageZone(const std::string& name, const AABB& bounds,
                                      Float damagePerSecond, PhysicsWorld* world);

    private:
        std::unordered_map<std::string, std::unique_ptr<TriggerZone>> m_triggers;
        std::unordered_set<TriggerZone*> m_activeTriggers;
    };
} // namespace engine::physics
