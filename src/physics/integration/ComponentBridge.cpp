/**
 * @file ComponentBridge.cpp
 * @brief Bridge between physics system and ECS components
 * @details Synchronizes physics simulation with entity component system
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#include "ComponentBridge.h"

namespace engine::physics {
    void PhysicsComponentBridge::registerEntity(const EntityID entity, PhysicsComponent* physics,
                                                TransformComponent* transform) {
        if (!physics || !transform) return;

        EntityBinding binding;
        binding.entity = entity;
        binding.physics = physics;
        binding.transform = transform;
        binding.lastSyncTime = 0;

        entities_[entity] = binding;

        if (physics->rigidBody) {
            bodyToEntity_[physics->rigidBody] = entity;
        }
    }

    void PhysicsComponentBridge::unregisterEntity(const EntityID entity) {
        if (const auto it = entities_.find(entity); it != entities_.end()) {
            if (it->second.physics && it->second.physics->rigidBody) {
                bodyToEntity_.erase(it->second.physics->rigidBody);
            }
            entities_.erase(it);
        }
    }

    void PhysicsComponentBridge::syncFromPhysics(const Float deltaTime) {
        for (auto& binding : entities_ | std::views::values) {
            if (!binding.physics->rigidBody || !binding.transform) continue;

            // Skip kinematic bodies (controlled by game logic)
            if (binding.physics->isKinematic) continue;

            // Get physics transform
            Vec3 position = binding.physics->rigidBody->getPosition();
            Quat rotation = binding.physics->rigidBody->getRotation();

            // Apply interpolation if enabled
            if (interpolationEnabled_) {
                const Float alpha = interpolationFactor_;
                position = glm::mix(binding.transform->position, position, alpha);
                rotation = glm::slerp(binding.transform->rotation, rotation, alpha);
            }

            // Update transform component
            binding.transform->position = position;
            binding.transform->rotation = rotation;

            // Store velocities
            binding.physics->linearVelocity = binding.physics->rigidBody->getLinearVelocity();
            binding.physics->angularVelocity = binding.physics->rigidBody->getAngularVelocity();

            binding.lastSyncTime += deltaTime;
        }
    }

    void PhysicsComponentBridge::syncToPhysics(const Float deltaTime) {
        for (const auto& binding : entities_ | std::views::values) {
            if (!binding.physics->rigidBody || !binding.transform) continue;

            // Only sync kinematic bodies
            if (!binding.physics->isKinematic) continue;

            // Calculate velocity from transform change
            Vec3 oldPosition = binding.physics->rigidBody->getPosition();
            Vec3 newPosition = binding.transform->position;
            Vec3 velocity = (newPosition - oldPosition) / deltaTime;

            // Update physics body
            binding.physics->rigidBody->setPosition(newPosition);
            binding.physics->rigidBody->setRotation(binding.transform->rotation);

            // Set velocity for smooth collision response
            if (binding.physics->rigidBody->getType() == BodyType::KINEMATIC) {
                binding.physics->rigidBody->setLinearVelocity(velocity);
            }
        }
    }

    void PhysicsComponentBridge::processCollision(RigidBody* bodyA, RigidBody* bodyB,
                                                  const CollisionManifold& manifold) const {
        const EntityID entityA = getEntity(bodyA);

        if (const EntityID entityB = getEntity(bodyB); entityA && entityB) {
            // Notify collision listeners
            for (auto& listener : collisionListeners_) {
                listener(entityA, entityB, manifold);
            }
        }
    }

    void PhysicsComponentBridge::applyImpulse(const EntityID entity, const Vec3& impulse, const Vec3& point) {
        if (const auto* physics = getPhysicsComponent(entity); physics && physics->rigidBody) {
            if (glm::length2(point) > EPSILON_SQUARED) {
                physics->rigidBody->applyImpulseAtPosition(impulse, point);
            }
            else {
                physics->rigidBody->applyImpulse(impulse);
            }
        }
    }

    void PhysicsComponentBridge::applyForce(const EntityID entity, const Vec3& force, const Vec3& point) {
        if (const auto* physics = getPhysicsComponent(entity); physics && physics->rigidBody) {
            if (glm::length2(point) > EPSILON_SQUARED) {
                physics->rigidBody->applyForceAtPosition(force, point);
            }
            else {
                physics->rigidBody->applyForce(force);
            }
        }
    }

    void PhysicsComponentBridge::setVelocity(const EntityID entity, const Vec3& linear, const Vec3& angular) {
        if (const auto* physics = getPhysicsComponent(entity); physics && physics->rigidBody) {
            physics->rigidBody->setLinearVelocity(linear);
            physics->rigidBody->setAngularVelocity(angular);
        }
    }

    void PhysicsComponentBridge::teleport(const EntityID entity, const Vec3& position, const Quat& rotation) {
        auto* transform = getTransformComponent(entity);
        const auto* physics = getPhysicsComponent(entity);

        if (transform) {
            transform->position = position;
            transform->rotation = rotation;
        }

        if (physics && physics->rigidBody) {
            physics->rigidBody->setPosition(position);
            physics->rigidBody->setRotation(rotation);
            physics->rigidBody->setLinearVelocity(Vec3(0));
            physics->rigidBody->setAngularVelocity(Vec3(0));
        }
    }

    PhysicsComponent PhysicsComponentFactory::createPhysicsComponent(const BodyCreationParams& params) {
        PhysicsComponent component;

        component.mass = params.mass;
        component.linearVelocity = params.linearVelocity;
        component.angularVelocity = params.angularVelocity;
        component.linearFactor = params.linearFactor;
        component.angularFactor = params.angularFactor;

        component.friction = params.material.friction;
        component.restitution = params.material.restitution;
        component.linearDamping = params.material.linearDamping;
        component.angularDamping = params.material.angularDamping;

        component.collisionGroup = params.collisionGroup;
        component.collisionMask = params.collisionMask;

        component.isKinematic = (params.type == BodyType::KINEMATIC);
        component.isTrigger = (params.type == BodyType::GHOST);
        component.enableCCD = params.enableCCD;
        component.startAsleep = params.startAsleep;

        component.ccdMotionThreshold = params.ccdMotionThreshold;
        component.ccdSweptSphereRadius = params.ccdSweptSphereRadius;

        return component;
    }

    BodyCreationParams PhysicsComponentFactory::createBodyParams(const PhysicsComponent& component,
                                                                 const TransformComponent& transform) {
        BodyCreationParams params;

        params.mass = component.mass;
        params.transform = new Transform(
            transform.position,
            transform.rotation,
            transform.scale);
        params.linearVelocity = component.linearVelocity;
        params.angularVelocity = component.angularVelocity;
        params.linearFactor = component.linearFactor;
        params.angularFactor = component.angularFactor;

        params.material.friction = component.friction;
        params.material.restitution = component.restitution;
        params.material.linearDamping = component.linearDamping;
        params.material.angularDamping = component.angularDamping;

        params.collisionGroup = component.collisionGroup;
        params.collisionMask = component.collisionMask;

        if (component.isKinematic) {
            params.type = BodyType::KINEMATIC;
        }
        else if (component.isTrigger) {
            params.type = BodyType::GHOST;
        }
        else if (component.mass > 0) {
            params.type = BodyType::DYNAMIC;
        }
        else {
            params.type = BodyType::STATIC;
        }

        params.enableCCD = component.enableCCD;
        params.startAsleep = component.startAsleep;
        params.ccdMotionThreshold = component.ccdMotionThreshold;
        params.ccdSweptSphereRadius = component.ccdSweptSphereRadius;

        return params;
    }
} // namespace engine::physics
