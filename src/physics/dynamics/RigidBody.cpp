/**
 * @file RigidBody.cpp
 * @brief Rigid body dynamics implementation
 * @details Wraps Bullet rigid body with game-specific features, optimizations,
 *          and integration with the transform system
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#include "RigidBody.h"

#include "../core/PhysicsWorld.h"
#include "../collision/CollisionShapes.h"
#include "../integration/BulletIntegration.h"

namespace engine::physics {
    RigidBody::~RigidBody() {
        cleanup();
    }

    bool RigidBody::initialize(const BodyCreationParams& params, CollisionShape* shape,
                               PhysicsWorld* world) {
        if (!shape || !world) return false;

        params_ = params;
        shape_ = shape;
        world_ = world;
        transform_ = params.transform;
        id_ = params.id;

        // Create motion state for transform sync
        if (transform_) {
            motionState_ = new PhysicsMotionState(transform_);
        }
        else {
            motionState_ = new btDefaultMotionState();
        }

        // Calculate local inertia
        btVector3 localInertia(0, 0, 0);
        if (params.type == BodyType::DYNAMIC && params.mass > 0) {
            shape->getBulletShape()->calculateLocalInertia(params.mass, localInertia);

            // Override if specified
            if (params.localInertia != Vec3(0)) {
                localInertia = btVector3(params.localInertia.x,
                                         params.localInertia.y,
                                         params.localInertia.z);
            }
        }

        // Create rigid body
        btRigidBody::btRigidBodyConstructionInfo rbInfo(
            params.mass,
            motionState_,
            shape->getBulletShape(),
            localInertia
        );

        // Set material properties
        rbInfo.m_friction = params.material.friction;
        rbInfo.m_restitution = params.material.restitution;
        rbInfo.m_linearDamping = params.material.linearDamping;
        rbInfo.m_angularDamping = params.material.angularDamping;
        rbInfo.m_rollingFriction = params.material.rollingFriction;
        rbInfo.m_spinningFriction = params.material.spinningFriction;

        bulletBody_ = new btRigidBody(rbInfo);

        // Configure body type
        configureBodyType(params.type);

        // Set initial velocities
        setLinearVelocity(params.linearVelocity);
        setAngularVelocity(params.angularVelocity);

        // Set motion constraints
        setLinearFactor(params.linearFactor);
        setAngularFactor(params.angularFactor);

        // Configure CCD if enabled
        if (params.enableCCD) {
            setCCDMotionThreshold(params.ccdMotionThreshold);
            setCCDSweptSphereRadius(params.ccdSweptSphereRadius);
        }

        // Set activation state
        setActivationState(params.activationState);
        if (params.startAsleep) {
            sleep();
        }

        // Store user data pointer
        bulletBody_->setUserPointer(this);
        userData_ = params.userData;

        // Store callbacks
        onCollisionEnter_ = params.onCollisionEnter;
        onCollisionExit_ = params.onCollisionExit;
        onTriggerEnter_ = params.onTriggerEnter;
        onTriggerExit_ = params.onTriggerExit;

        return true;
    }

    void RigidBody::cleanup() {
        if (bulletBody_) {
            delete bulletBody_;
            bulletBody_ = nullptr;
        }

        if (motionState_) {
            delete motionState_;
            motionState_ = nullptr;
        }

        shape_ = nullptr;
        world_ = nullptr;
        transform_ = nullptr;
    }

    void RigidBody::reset() {
        cleanup();
        params_ = BodyCreationParams(); // Reset to defaults
        userData_ = nullptr;
        id_ = 0;
        onCollisionEnter_ = nullptr;
        onCollisionExit_ = nullptr;
        onTriggerEnter_ = nullptr;
        onTriggerExit_ = nullptr;
    }

    Vec3 RigidBody::getPosition() const {
        if (!bulletBody_) return VEC3_ZERO;

        const btVector3& pos = bulletBody_->getWorldTransform().getOrigin();
        return Vec3(pos.x(), pos.y(), pos.z());
    }

    void RigidBody::setPosition(const Vec3& position) const {
        if (!bulletBody_) return;

        btTransform transform = bulletBody_->getWorldTransform();
        transform.setOrigin(btVector3(position.x, position.y, position.z));
        bulletBody_->setWorldTransform(transform);

        if (params_.type == BodyType::DYNAMIC) {
            bulletBody_->activate();
        }
    }

    Quat RigidBody::getRotation() const {
        if (!bulletBody_) return QUAT_IDENTITY;

        const btQuaternion& rot = bulletBody_->getWorldTransform().getRotation();
        return Quat(rot.w(), rot.x(), rot.y(), rot.z());
    }

    void RigidBody::setRotation(const Quat& rotation) const {
        if (!bulletBody_) return;

        btTransform transform = bulletBody_->getWorldTransform();
        transform.setRotation(btQuaternion(rotation.x, rotation.y, rotation.z, rotation.w));
        bulletBody_->setWorldTransform(transform);

        if (params_.type == BodyType::DYNAMIC) {
            bulletBody_->activate();
        }
    }

    Mat4 RigidBody::getWorldMatrix() const {
        if (!bulletBody_) return IDENTITY_4X4;

        const btTransform transform = bulletBody_->getWorldTransform();
        btScalar matrix[16];
        transform.getOpenGLMatrix(matrix);

        return Mat4(
            matrix[0], matrix[1], matrix[2], matrix[3],
            matrix[4], matrix[5], matrix[6], matrix[7],
            matrix[8], matrix[9], matrix[10], matrix[11],
            matrix[12], matrix[13], matrix[14], matrix[15]
        );
    }

    void RigidBody::setWorldMatrix(const Mat4& matrix) const {
        if (!bulletBody_) return;

        btTransform transform;
        transform.setFromOpenGLMatrix(&matrix[0][0]);
        bulletBody_->setWorldTransform(transform);

        if (params_.type == BodyType::DYNAMIC) {
            bulletBody_->activate();
        }
    }

    Float RigidBody::getCCDMotionThreshold() const {
        if (!bulletBody_) return 0.0f;
        return bulletBody_->getCcdMotionThreshold();
    }

    Float RigidBody::getCCDSweptSphereRadius() const {
        if (!bulletBody_) return 0.0f;
        return bulletBody_->getCcdSweptSphereRadius();
    }

    Vec3 RigidBody::getLinearVelocity() const {
        if (!bulletBody_) return VEC3_ZERO;

        const btVector3& vel = bulletBody_->getLinearVelocity();
        return Vec3(vel.x(), vel.y(), vel.z());
    }

    void RigidBody::setLinearVelocity(const Vec3& velocity) const {
        if (!bulletBody_) return;

        bulletBody_->setLinearVelocity(btVector3(velocity.x, velocity.y, velocity.z));
        bulletBody_->activate();
    }

    Vec3 RigidBody::getAngularVelocity() const {
        if (!bulletBody_) return VEC3_ZERO;

        const btVector3& vel = bulletBody_->getAngularVelocity();
        return Vec3(vel.x(), vel.y(), vel.z());
    }

    void RigidBody::setAngularVelocity(const Vec3& velocity) const {
        if (!bulletBody_) return;

        bulletBody_->setAngularVelocity(btVector3(velocity.x, velocity.y, velocity.z));
        bulletBody_->activate();
    }

    Vec3 RigidBody::getVelocityAtWorldPoint(const Vec3& worldPoint) const {
        if (!bulletBody_) return Vec3(0);

        const btVector3 relPos = btVector3(worldPoint.x, worldPoint.y, worldPoint.z) -
            bulletBody_->getWorldTransform().getOrigin();
        const btVector3 vel = bulletBody_->getVelocityInLocalPoint(relPos);

        return Vec3(vel.x(), vel.y(), vel.z());
    }

    void RigidBody::applyForce(const Vec3& force) const {
        if (!bulletBody_) return;

        bulletBody_->applyCentralForce(btVector3(force.x, force.y, force.z));
        bulletBody_->activate();
    }

    void RigidBody::applyForceAtPosition(const Vec3& force, const Vec3& position) const {
        if (!bulletBody_) return;

        const btVector3 relPos = btVector3(position.x, position.y, position.z) -
            bulletBody_->getWorldTransform().getOrigin();

        bulletBody_->applyForce(btVector3(force.x, force.y, force.z), relPos);
        bulletBody_->activate();
    }

    void RigidBody::applyTorque(const Vec3& torque) const {
        if (!bulletBody_) return;

        bulletBody_->applyTorque(btVector3(torque.x, torque.y, torque.z));
        bulletBody_->activate();
    }

    void RigidBody::applyImpulse(const Vec3& impulse) const {
        if (!bulletBody_) return;

        bulletBody_->applyCentralImpulse(btVector3(impulse.x, impulse.y, impulse.z));
        bulletBody_->activate();
    }

    void RigidBody::applyImpulseAtPosition(const Vec3& impulse, const Vec3& position) const {
        if (!bulletBody_) return;

        const btVector3 relPos = btVector3(position.x, position.y, position.z) -
            bulletBody_->getWorldTransform().getOrigin();

        bulletBody_->applyImpulse(btVector3(impulse.x, impulse.y, impulse.z), relPos);
        bulletBody_->activate();
    }

    void RigidBody::applyTorqueImpulse(const Vec3& torque) const {
        if (!bulletBody_) return;

        bulletBody_->applyTorqueImpulse(btVector3(torque.x, torque.y, torque.z));
        bulletBody_->activate();
    }

    void RigidBody::clearForces() const {
        if (!bulletBody_) return;
        bulletBody_->clearForces();
    }

    Float RigidBody::getMass() const {
        if (!bulletBody_) return 0.0f;

        const Float invMass = bulletBody_->getInvMass();
        return invMass > 0 ? 1.0f / invMass : 0.0f;
    }

    void RigidBody::setMass(const Float mass) {
        if (!bulletBody_ || !shape_) return;

        btVector3 localInertia(0, 0, 0);
        if (mass > 0 && params_.type == BodyType::DYNAMIC) {
            shape_->getBulletShape()->calculateLocalInertia(mass, localInertia);
        }

        bulletBody_->setMassProps(mass, localInertia);
        params_.mass = mass;
    }

    Vec3 RigidBody::getCenterOfMass() const {
        if (!bulletBody_) return VEC3_ZERO;

        const btVector3& com = bulletBody_->getCenterOfMassPosition();
        return Vec3(com.x(), com.y(), com.z());
    }

    Vec3 RigidBody::getLocalInertia() const {
        if (!bulletBody_) return Vec3(0);

        const btVector3& inertia = bulletBody_->getLocalInertia();
        return Vec3(inertia.x(), inertia.y(), inertia.z());
    }

    void RigidBody::setLinearFactor(const Vec3& factor) {
        if (!bulletBody_) return;

        bulletBody_->setLinearFactor(btVector3(factor.x, factor.y, factor.z));
        params_.linearFactor = factor;
    }

    // TODO: Ver este warning que aparece mucho
    Vec3 RigidBody::getLinearFactor() const { return params_.linearFactor; }

    void RigidBody::setAngularFactor(const Vec3& factor) {
        if (!bulletBody_) return;

        bulletBody_->setAngularFactor(btVector3(factor.x, factor.y, factor.z));
        params_.angularFactor = factor;
    }

    Vec3 RigidBody::getAngularFactor() const { return params_.angularFactor; }

    void RigidBody::lockPosition(const bool lockX, const bool lockY, const bool lockZ) {
        const Vec3 factor(lockX ? 0.0f : 1.0f,
                          lockY ? 0.0f : 1.0f,
                          lockZ ? 0.0f : 1.0f);

        setLinearFactor(factor);
    }

    void RigidBody::lockRotation(const bool lockX, const bool lockY, const bool lockZ) {
        const Vec3 factor(lockX ? 0.0f : 1.0f,
                          lockY ? 0.0f : 1.0f,
                          lockZ ? 0.0f : 1.0f);

        setAngularFactor(factor);
    }

    void RigidBody::setFriction(const Float friction) {
        if (!bulletBody_) return;
        bulletBody_->setFriction(friction);
        params_.material.friction = friction;
    }

    Float RigidBody::getFriction() const { return params_.material.friction; }

    void RigidBody::setRestitution(const Float restitution) {
        if (!bulletBody_) return;
        bulletBody_->setRestitution(restitution);
        params_.material.restitution = restitution;
    }

    Float RigidBody::getRestitution() const { return params_.material.restitution; }

    void RigidBody::setDamping(const Float linear, const Float angular) {
        if (!bulletBody_) return;
        bulletBody_->setDamping(linear, angular);
        params_.material.linearDamping = linear;
        params_.material.angularDamping = angular;
    }

    void RigidBody::setRollingFriction(const Float friction) {
        if (!bulletBody_) return;
        bulletBody_->setRollingFriction(friction);
        params_.material.rollingFriction = friction;
    }

    bool RigidBody::isActive() const {
        return bulletBody_ && bulletBody_->isActive();
    }

    void RigidBody::activate(const bool forceActivation) const {
        if (!bulletBody_) return;
        bulletBody_->activate(forceActivation);
    }

    void RigidBody::sleep() const {
        if (!bulletBody_) return;

        bulletBody_->setActivationState(ISLAND_SLEEPING);
    }

    void RigidBody::setActivationState(ActivationState state) const {
        if (!bulletBody_) return;

        bulletBody_->setActivationState(static_cast<int>(state));
    }

    ActivationState RigidBody::getActivationState() const {
        if (!bulletBody_) return ActivationState::ACTIVE_STATE;
        return static_cast<ActivationState>(bulletBody_->getActivationState());
    }

    void RigidBody::setSleepingThresholds(const Float linear, const Float angular) const {
        if (!bulletBody_) return;
        bulletBody_->setSleepingThresholds(linear, angular);
    }

    void RigidBody::setCCDMotionThreshold(const Float threshold) const {
        if (!bulletBody_) return;
        bulletBody_->setCcdMotionThreshold(threshold);
    }

    void RigidBody::setCCDSweptSphereRadius(const Float radius) const {
        if (!bulletBody_) return;
        bulletBody_->setCcdSweptSphereRadius(radius);
    }

    void RigidBody::setCollisionGroup(const std::uint16_t group) {
        params_.collisionGroup = group;
        // Need to re-add to world to update filtering
    }

    std::uint16_t RigidBody::getCollisionGroup() const { return params_.collisionGroup; }

    void RigidBody::setCollisionMask(const std::uint16_t mask) {
        params_.collisionMask = mask;
        // Need to re-add to world to update filtering
    }

    std::uint16_t RigidBody::getCollisionMask() const { return params_.collisionMask; }

    AABB RigidBody::getAABB() const {
        if (!bulletBody_) return AABB();

        btVector3 min, max;
        bulletBody_->getAabb(min, max);

        return AABB(
            Vec3(min.x(), min.y(), min.z()),
            Vec3(max.x(), max.y(), max.z())
        );
    }

    bool RigidBody::containsPoint(const Vec3& point) {
        // Implementation depends on shape type
        return false;
    }

    void RigidBody::onCollisionEnter(RigidBody* other, const CollisionManifold& manifold) {
        if (onCollisionEnter_) {
            onCollisionEnter_(this, other, manifold);
        }
    }

    void RigidBody::onCollisionExit(RigidBody* other) {
        if (onCollisionExit_) {
            onCollisionExit_(this, other);
        }
    }

    void RigidBody::onTriggerEnter(RigidBody* other) {
        if (onTriggerEnter_) {
            onTriggerEnter_(this, other);
        }
    }

    void RigidBody::onTriggerExit(RigidBody* other) {
        if (onTriggerExit_) {
            onTriggerExit_(this, other);
        }
    }

    bool RigidBody::isTrigger() const {
        if (!bulletBody_) return false;
        return (bulletBody_->getCollisionFlags() & btCollisionObject::CF_NO_CONTACT_RESPONSE) != 0;
    }

    void RigidBody::configureBodyType(const BodyType type) const {
        if (!bulletBody_) return;

        switch (type) {
        case BodyType::STATIC:
            bulletBody_->setCollisionFlags(
                bulletBody_->getCollisionFlags() |
                btCollisionObject::CF_STATIC_OBJECT
            );
            break;

        case BodyType::KINEMATIC:
            bulletBody_->setCollisionFlags(
                bulletBody_->getCollisionFlags() |
                btCollisionObject::CF_KINEMATIC_OBJECT
            );
            bulletBody_->setActivationState(DISABLE_DEACTIVATION);
            break;

        case BodyType::DYNAMIC:
            // Default flags
            break;

        case BodyType::GHOST:
            bulletBody_->setCollisionFlags(
                bulletBody_->getCollisionFlags() |
                btCollisionObject::CF_NO_CONTACT_RESPONSE
            );
            break;
        }
    }
} // namespace engine::physics
