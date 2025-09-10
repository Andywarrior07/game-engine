/**
 * @file ComponentBridge.h
 * @brief Bridge between physics system and ECS components
 * @details Synchronizes physics simulation with entity component system
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

#include "../core/PhysicsTypes.h"
#include "../dynamics/RigidBody.h"

#include "../../math/MathSystem.h"

#include <unordered_map>
#include <functional>

namespace engine::physics {
    using namespace engine::math;

    // Forward declarations for ECS types
    using EntityID = std::uint32_t;

    /**
     * @brief Physics component for ECS
     */
    struct PhysicsComponent {
        RigidBody* rigidBody = nullptr;
        CollisionShape* shape = nullptr;

        // Physics properties
        Float mass = 1.0f;
        Vec3 linearVelocity = Vec3(0);
        Vec3 angularVelocity = Vec3(0);
        Vec3 linearFactor = Vec3(1);
        Vec3 angularFactor = Vec3(1);

        // Material properties
        Float friction = 0.5f;
        Float restitution = 0.0f;
        Float linearDamping = 0.0f;
        Float angularDamping = 0.0f;

        // Collision settings
        std::uint16_t collisionGroup = DEFAULT;
        std::uint16_t collisionMask = ALL;

        // State flags
        bool isKinematic = false;
        bool isTrigger = false;
        bool enableCCD = false;
        bool startAsleep = false;

        // CCD settings
        Float ccdMotionThreshold = 0.0f;
        Float ccdSweptSphereRadius = 0.0f;
    };

    /**
     * @brief Transform component for ECS
     */
    struct TransformComponent {
        Vec3 position = Vec3(0);
        Quat rotation = Quat(1, 0, 0, 0);
        Vec3 scale = Vec3(1);

        Mat4 getMatrix() const {
            return glm::translate(Mat4(1.0f), position) *
                glm::mat4_cast(rotation) *
                glm::scale(Mat4(1.0f), scale);
        }
    };

    /**
     * @brief Bridge between physics and ECS
     */
    class PhysicsComponentBridge {
    public:
        PhysicsComponentBridge() = default;

        /**
         * @brief Register entity with physics component
         */
        void registerEntity(EntityID entity, PhysicsComponent* physics,
                            TransformComponent* transform);

        /**
         * @brief Unregister entity
         */
        void unregisterEntity(EntityID entity);

        /**
         * @brief Sync transforms from physics to ECS
         */
        void syncFromPhysics(Float deltaTime);

        /**
         * @brief Sync transforms from ECS to physics (for kinematic bodies)
         */
        void syncToPhysics(Float deltaTime);

        /**
         * @brief Get entity ID from rigid body
         */
        EntityID getEntity(RigidBody* body) const {
            const auto it = bodyToEntity_.find(body);
            return it != bodyToEntity_.end() ? it->second : 0;
        }

        /**
         * @brief Get physics component from entity
         */
        PhysicsComponent* getPhysicsComponent(const EntityID entity) {
            const auto it = entities_.find(entity);
            return it != entities_.end() ? it->second.physics : nullptr;
        }

        /**
         * @brief Get transform component from entity
         */
        TransformComponent* getTransformComponent(const EntityID entity) {
            const auto it = entities_.find(entity);
            return it != entities_.end() ? it->second.transform : nullptr;
        }

        /**
         * @brief Process collision event
         */
        void processCollision(RigidBody* bodyA, RigidBody* bodyB,
                              const CollisionManifold& manifold) const;

        /**
         * @brief Register collision listener
         */
        using CollisionListener = std::function<void(EntityID, EntityID, const CollisionManifold&)>;

        void addCollisionListener(const CollisionListener& listener) {
            collisionListeners_.push_back(listener);
        }

        /**
         * @brief Enable/disable interpolation
         */
        void setInterpolation(const bool enable, const Float factor = 0.8f) {
            interpolationEnabled_ = enable;
            interpolationFactor_ = factor;
        }

        /**
         * @brief Apply impulse to entity
         */
        void applyImpulse(EntityID entity, const Vec3& impulse, const Vec3& point = VEC3_ZERO);

        /**
         * @brief Apply force to entity
         */
        void applyForce(EntityID entity, const Vec3& force, const Vec3& point = VEC3_ZERO);

        /**
         * @brief Set entity velocity
         */
        void setVelocity(EntityID entity, const Vec3& linear, const Vec3& angular = VEC3_ZERO);

        /**
         * @brief Teleport entity
         */
        void teleport(EntityID entity, const Vec3& position, const Quat& rotation = QUAT_IDENTITY);

    private:
        struct EntityBinding {
            EntityID entity;
            PhysicsComponent* physics;
            TransformComponent* transform;
            Float lastSyncTime;
        };

        std::unordered_map<EntityID, EntityBinding> entities_;
        std::unordered_map<RigidBody*, EntityID> bodyToEntity_;
        std::vector<CollisionListener> collisionListeners_;

        bool interpolationEnabled_ = true;
        Float interpolationFactor_ = 0.8f;
    };

    /**
     * @brief Physics component factory
     */
    class PhysicsComponentFactory {
    public:
        /**
         * @brief Create physics component from parameters
         */
        static PhysicsComponent createPhysicsComponent(const BodyCreationParams& params);

        /**
         * @brief Create body parameters from component
         */
        static BodyCreationParams createBodyParams(const PhysicsComponent& component,
                                                   const TransformComponent& transform);
    };
} // namespace engine::physics
