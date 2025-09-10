/**
 * @file TriggerSystem.cpp
 * @brief Trigger zone system for gameplay events
 * @details Manages trigger volumes, overlap detection, and event dispatching
 *          for gameplay mechanics like checkpoints, traps, and interactive zones
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#include "TriggerSystem.h"

#include "../collision/CollisionShapes.h"
#include "../core/PhysicsWorld.h"
#include "../dynamics/RigidBody.h"

namespace engine::physics {
    TriggerZone::~TriggerZone() {
        cleanup();
    }

    bool TriggerZone::initialize(CollisionShape* shape, const Vec3& position,
                                 PhysicsWorld* world) {
        if (!shape || !world) return false;

        m_world = world;
        m_shape = shape;

        // Create ghost object for overlap detection
        m_ghostObject = new btPairCachingGhostObject();
        m_ghostObject->setCollisionShape(shape->getBulletShape());
        m_ghostObject->setCollisionFlags(
            btCollisionObject::CF_NO_CONTACT_RESPONSE |
            btCollisionObject::CF_STATIC_OBJECT
        );

        // Set transform
        btTransform transform;
        transform.setIdentity();
        transform.setOrigin(btVector3(position.x, position.y, position.z));
        m_ghostObject->setWorldTransform(transform);

        // Store user pointer for callbacks
        m_ghostObject->setUserPointer(this);

        // Add to physics world
        if (world->getBulletWorld()) {
            world->getBulletWorld()->addCollisionObject(
                m_ghostObject,
                CollisionGroup::TRIGGER,
                CollisionGroup::ALL
            );
        }

        return true;
    }

    void TriggerZone::cleanup() {
        if (m_world && m_world->getBulletWorld() && m_ghostObject) {
            m_world->getBulletWorld()->removeCollisionObject(m_ghostObject);
        }

        delete m_ghostObject;
        m_ghostObject = nullptr;
    }

    void TriggerZone::update(const Float deltaTime) {
        if (!m_enabled || !m_ghostObject) return;

        // Update cooldown
        if (m_cooldownTime > 0) {
            m_timeSinceTrigger += deltaTime;
            if (m_timeSinceTrigger >= m_cooldownTime) {
                m_timeSinceTrigger = 0;
                m_cooldownTime = 0;
            }
        }

        // Check for overlapping bodies
        std::unordered_set<RigidBody*> currentOverlaps;
        checkOverlaps(currentOverlaps);

        // Process enter events
        for (RigidBody* body : currentOverlaps) {
            if (!m_overlappingBodies.contains(body)) {
                // New overlap - trigger enter
                onBodyEnter(body);
            }
        }

        // Process stay events
        if (m_activation == TriggerActivation::ON_STAY) {
            for (RigidBody* body : currentOverlaps) {
                onBodyStay(body, deltaTime);
            }
        }

        // Process exit events
        for (RigidBody* body : m_overlappingBodies) {
            if (!currentOverlaps.contains(body)) {
                // No longer overlapping - trigger exit
                onBodyExit(body);
            }
        }

        // Update overlapping bodies set
        m_overlappingBodies = currentOverlaps;

        // Handle pressure plate logic
        if (m_type == TriggerType::PRESSURE_PLATE) {
            updatePressurePlate();
        }
    }

    Vec3 TriggerZone::getPosition() const {
        if (!m_ghostObject) return Vec3(0);

        const btTransform& transform = m_ghostObject->getWorldTransform();
        const btVector3& pos = transform.getOrigin();
        return Vec3(pos.x(), pos.y(), pos.z());
    }

    void TriggerZone::setPosition(const Vec3& position) const {
        if (!m_ghostObject) return;

        btTransform transform = m_ghostObject->getWorldTransform();
        transform.setOrigin(btVector3(position.x, position.y, position.z));
        m_ghostObject->setWorldTransform(transform);
    }

    Float TriggerZone::getTotalWeight() const {
        Float total = 0;
        for (const RigidBody* body : m_overlappingBodies) {
            if (body) {
                total += body->getMass();
            }
        }
        return total;
    }

    void TriggerZone::activate() {
        if (!m_enabled || (m_oneShot && m_hasBeenTriggered)) return;

        TriggerEvent event;
        event.trigger = this;
        event.body = nullptr;
        event.contactPoint = getPosition();
        event.time = 0;
        event.isEntering = true;

        if (m_onActivateCallback) {
            m_onActivateCallback(event);
        }

        m_hasBeenTriggered = true;
    }

    void TriggerZone::reset() {
        m_hasBeenTriggered = false;
        m_timeSinceTrigger = 0;
        m_overlappingBodies.clear();
    }

    void TriggerZone::checkOverlaps(std::unordered_set<RigidBody*>& overlaps) const {
        if (!m_ghostObject) return;

        // Check all overlapping pairs
        btManifoldArray manifoldArray;
        btBroadphasePairArray& pairArray =
            m_ghostObject->getOverlappingPairCache()->getOverlappingPairArray();

        for (int i = 0; i < pairArray.size(); ++i) {
            const btBroadphasePair& pair = pairArray[i];

            auto obj0 = static_cast<btCollisionObject*>(pair.m_pProxy0->m_clientObject);
            auto obj1 = static_cast<btCollisionObject*>(pair.m_pProxy1->m_clientObject);

            if (const btCollisionObject* other = (obj0 == m_ghostObject) ? obj1 : obj0; other && !other->isStaticObject()) {
                if (auto body = static_cast<RigidBody*>(other->getUserPointer()); body && passesFilter(body)) {
                    overlaps.insert(body);
                }
            }
        }
    }

    void TriggerZone::onBodyEnter(RigidBody* body) {
        if (m_oneShot && m_hasBeenTriggered) return;
        if (m_cooldownTime > 0 && m_timeSinceTrigger < m_cooldownTime) return;

        if (m_activation == TriggerActivation::ON_ENTER ||
            m_activation == TriggerActivation::ON_ENTER_EXIT) {
            TriggerEvent event;
            event.trigger = this;
            event.body = body;
            event.contactPoint = body->getPosition();
            event.time = 0;
            event.isEntering = true;

            if (m_onEnterCallback) {
                m_onEnterCallback(event);
            }

            m_hasBeenTriggered = true;
            m_timeSinceTrigger = 0;
        }
    }

    void TriggerZone::onBodyExit(RigidBody* body) {
        if (m_activation == TriggerActivation::ON_EXIT ||
            m_activation == TriggerActivation::ON_ENTER_EXIT) {
            TriggerEvent event;
            event.trigger = this;
            event.body = body;
            event.contactPoint = body->getPosition();
            event.time = 0;
            event.isEntering = false;

            if (m_onExitCallback) {
                m_onExitCallback(event);
            }
        }
    }

    void TriggerZone::onBodyStay(RigidBody* body, const Float deltaTime) {
        if (m_oneShot && m_hasBeenTriggered) return;

        TriggerEvent event;
        event.trigger = this;
        event.body = body;
        event.contactPoint = body->getPosition();
        event.time = deltaTime;
        event.isEntering = false;

        if (m_onStayCallback) {
            m_onStayCallback(event);
        }
    }

    void TriggerZone::updatePressurePlate() {
        bool shouldBeActive = false;

        // Check weight requirement
        if (m_requiredWeight > 0) {
            shouldBeActive = getTotalWeight() >= m_requiredWeight;
        }

        // Check count requirement
        if (m_requiredCount > 0) {
            shouldBeActive = shouldBeActive ||
                (static_cast<Int>(m_overlappingBodies.size()) >= m_requiredCount);
        }

        // Trigger activation/deactivation
        if (shouldBeActive && !m_isActivated) {
            m_isActivated = true;
            activate();
        }
        else if (!shouldBeActive && m_isActivated) {
            m_isActivated = false;

            // Fire deactivation event
            TriggerEvent event;
            event.trigger = this;
            event.body = nullptr;
            event.contactPoint = getPosition();
            event.time = 0;
            event.isEntering = false;

            if (m_onExitCallback) {
                m_onExitCallback(event);
            }
        }
    }

    TriggerZone* TriggerSystem::createTrigger(const std::string& name, TriggerType type,
                                              CollisionShape* shape, const Vec3& position,
                                              PhysicsWorld* world) {
        auto trigger = std::make_unique<TriggerZone>(name, type);

        if (!trigger->initialize(shape, position, world))
            return nullptr;

        TriggerZone* ptr = trigger.get();
        m_triggers[name] = std::move(trigger);
        m_activeTriggers.insert(ptr);

        return m_triggers[name].get();
    }

    void TriggerSystem::removeTrigger(const std::string& name) {
        if (const auto it = m_triggers.find(name); it != m_triggers.end()) {
            m_activeTriggers.erase(it->second.get());
            m_triggers.erase(it);
        }
    }

    TriggerZone* TriggerSystem::findTrigger(const std::string& name) const {
        const auto it = m_triggers.find(name);
        return it != m_triggers.end() ? it->second.get() : nullptr;
    }

    void TriggerSystem::update(const Float deltaTime) {
        for (auto& trigger : m_triggers | std::views::values) {
            if (trigger) {
                trigger->update(deltaTime);
            }
        }
    }

    void TriggerSystem::clear() {
        m_triggers.clear();
        m_activeTriggers.clear();
    }

    TriggerZone* TriggerSystem::createCheckpoint(const std::string& name, const Vec3& position,
                                                 const Float radius, PhysicsWorld* world) {
        CollisionShape* sphere = new SphereShape(radius);
        const auto trigger = createTrigger(name, TriggerType::CHECKPOINT,
                                           sphere, position, world); // Need shape creation

        if (!trigger) return nullptr;

        trigger->setActivation(TriggerActivation::ON_ENTER);
        trigger->setFilter(FILTER_PLAYER);
        trigger->setOneShot(true);

        return trigger;
    }

    TriggerZone* TriggerSystem::createDamageZone(const std::string& name, const AABB& bounds,
                                                 Float damagePerSecond, PhysicsWorld* world) {
        const Vec3 center = bounds.getCenter();
        const Float radius = bounds.getDiagonalLength() * 0.5f;

        CollisionShape* sphere = new SphereShape(radius);
        const auto trigger = createTrigger(name, TriggerType::DAMAGE,
                                           sphere, center, world);

        if (trigger) {
            trigger->setActivation(TriggerActivation::ON_STAY);
            trigger->setFilter(FILTER_PLAYER | FILTER_ENEMY | FILTER_NPC);

            // Set damage callback
            // trigger->setOnStayCallback([damagePerSecond](const TriggerEvent& event) {
            //     // Apply damage logic here
            //     // event.body would receive damage
            // });
        }

        return trigger;
    }
} // namespace engine::physics
