/**
 * @file RigidBody.h
 * @brief Rigid body dynamics implementation
 * @details Wraps Bullet rigid body with game-specific features, optimizations,
 *          and integration with the transform system
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

#include "../core/PhysicsConfig.h"

#include "../../math/MathSystem.h"

class btRigidBody;
class btMotionState;

namespace engine::physics {
    enum class BodyType : std::uint8_t;
    enum class ActivationState : std::uint8_t;
    using namespace engine::math;

    // Forward declarations
    struct BodyCreationParams;
    struct CollisionManifold;
    class CollisionShape;

    /**
     * @brief Rigid body wrapper class
     * @details Provides high-level interface to Bullet rigid bodies with
     *          automatic transform synchronization and event callbacks
     */
    class RigidBody {
    public:
        friend class PhysicsWorld;
        friend class PhysicsManager;

        RigidBody() : bulletBody_(nullptr), motionState_(nullptr), shape_(nullptr), world_(nullptr),
                      transform_(nullptr), userData_(nullptr), id_(0) {
        }

        ~RigidBody();

        /**
         * @brief Initialize rigid body with parameters
         */
        bool initialize(const BodyCreationParams& params, CollisionShape* shape,
                        PhysicsWorld* world);

        /**
         * @brief Clean up resources
         */
        void cleanup();

        /**
         * @brief Reset rigid body for ObjectPool reuse
         */
        void reset();

        // ============================================================================
        // Transform
        // ============================================================================

        /**
         * @brief Get world position
         */
        Vec3 getPosition() const;

        /**
         * @brief Set world position
         */
        void setPosition(const Vec3& position) const;

        /**
         * @brief Get world rotation
         */
        Quat getRotation() const;

        /**
         * @brief Set world rotation
         */
        void setRotation(const Quat& rotation) const;

        /**
         * @brief Get world transform
         */
        Mat4 getWorldMatrix() const;

        /**
         * @brief Set transform from matrix
         */
        void setWorldMatrix(const Mat4& matrix) const;

        /**
         * @brief Get CCD motion threshold
         * @details Objects moving faster than this threshold will use CCD
         * @return Motion threshold in units per frame
         */
        Float getCCDMotionThreshold() const;

        /**
         * @brief Get CCD swept sphere radius
         * @details Radius of the sphere used for swept collision testing
         * @return Swept sphere radius in world units
         */
        Float getCCDSweptSphereRadius() const;

        // ============================================================================
        // Velocities
        // ============================================================================

        /**
         * @brief Get linear velocity
         */
        Vec3 getLinearVelocity() const;

        /**
         * @brief Set linear velocity
         */
        void setLinearVelocity(const Vec3& velocity) const;

        /**
         * @brief Get angular velocity
         */
        Vec3 getAngularVelocity() const;

        /**
         * @brief Set angular velocity
         */
        void setAngularVelocity(const Vec3& velocity) const;

        /**
         * @brief Get velocity at world point
         */
        Vec3 getVelocityAtWorldPoint(const Vec3& worldPoint) const;

        // ============================================================================
        // Forces and Impulses
        // ============================================================================

        /**
         * @brief Apply force at center of mass
         */
        void applyForce(const Vec3& force) const;

        /**
         * @brief Apply force at world position
         */
        void applyForceAtPosition(const Vec3& force, const Vec3& position) const;

        /**
         * @brief Apply torque
         */
        void applyTorque(const Vec3& torque) const;

        /**
         * @brief Apply impulse at center of mass
         */
        void applyImpulse(const Vec3& impulse) const;

        /**
         * @brief Apply impulse at world position
         */
        void applyImpulseAtPosition(const Vec3& impulse, const Vec3& position) const;

        /**
         * @brief Apply torque impulse
         */
        void applyTorqueImpulse(const Vec3& torque) const;

        /**
         * @brief Clear all forces
         */
        void clearForces() const;

        // ============================================================================
        // Mass Properties
        // ============================================================================

        /**
         * @brief Get mass
         */
        Float getMass() const;

        /**
         * @brief Set mass
         */
        void setMass(Float mass);

        /**
         * @brief Get center of mass
         */
        Vec3 getCenterOfMass() const;

        /**
         * @brief Get local inertia tensor
         */
        Vec3 getLocalInertia() const;

        // ============================================================================
        // Constraints
        // ============================================================================

        /**
         * @brief Set linear motion factors (0 = locked, 1 = free)
         */
        void setLinearFactor(const Vec3& factor);

        Vec3 getLinearFactor() const;

        /**
         * @brief Set angular motion factors (0 = locked, 1 = free)
         */
        void setAngularFactor(const Vec3& factor);

        Vec3 getAngularFactor() const;

        /**
         * @brief Lock/unlock position
         */
        void lockPosition(bool lockX = true, bool lockY = true, bool lockZ = true);

        /**
         * @brief Lock/unlock rotation
         */
        void lockRotation(bool lockX = true, bool lockY = true, bool lockZ = true);

        // ============================================================================
        // Material Properties
        // ============================================================================

        void setFriction(Float friction);

        Float getFriction() const;

        void setRestitution(Float restitution);

        Float getRestitution() const;

        void setDamping(Float linear, Float angular);

        void setRollingFriction(Float friction);

        // ============================================================================
        // Activation and Sleep
        // ============================================================================

        bool isActive() const;

        void activate(const bool forceActivation = false) const;

        void sleep() const;

        void setActivationState(ActivationState state) const;

        ActivationState getActivationState() const;

        void setSleepingThresholds(Float linear, Float angular) const;

        // ============================================================================
        // CCD (Continuous Collision Detection)
        // ============================================================================

        void setCCDMotionThreshold(Float threshold) const;

        void setCCDSweptSphereRadius(Float radius) const;

        // ============================================================================
        // Collision Filtering
        // ============================================================================

        void setCollisionGroup(const std::uint16_t group);

        std::uint16_t getCollisionGroup() const;

        void setCollisionMask(const std::uint16_t mask);

        std::uint16_t getCollisionMask() const;

        // ============================================================================
        // Queries
        // ============================================================================

        /**
         * @brief Get axis-aligned bounding box
         */
        AABB getAABB() const;

        /**
         * @brief Check if point is inside body
         */
        static bool containsPoint(const Vec3& point);

        // ============================================================================
        // Accessors
        // ============================================================================

        BodyType getType() const { return params_.type; }
        CollisionShape* getShape() const { return shape_; }
        btRigidBody* getBulletBody() const { return bulletBody_; }
        Transform* getTransform() const { return transform_; }
        void* getUserData() const { return userData_; }
        void setUserData(void* data) { userData_ = data; }
        std::uint32_t getId() const { return id_; }
        const std::string& getName() const { return params_.name; }

        // ============================================================================
        // Callbacks
        // ============================================================================

        void onCollisionEnter(RigidBody* other, const CollisionManifold& manifold);

        void onCollisionExit(RigidBody* other);

        void onTriggerEnter(RigidBody* other);

        void onTriggerExit(RigidBody* other);

        /**
         * @brief Check if this rigid body is a trigger (no physical collision response)
         */
        bool isTrigger() const;

    private:
        // Bullet objects
        btRigidBody* bulletBody_;
        btMotionState* motionState_;
        CollisionShape* shape_;

        // References
        PhysicsWorld* world_;
        Transform* transform_;

        // Parameters
        BodyCreationParams params_;

        // User data
        void* userData_;
        std::uint32_t id_;

        // Callbacks
        CollisionEnterCallback onCollisionEnter_;
        CollisionExitCallback onCollisionExit_;
        TriggerEnterCallback onTriggerEnter_;
        TriggerExitCallback onTriggerExit_;

        /**
         * @brief Configure body type specific settings
         */
        void configureBodyType(BodyType type) const;
    };
} // namespace engine::physics
