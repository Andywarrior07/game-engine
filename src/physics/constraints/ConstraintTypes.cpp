/**
* @file ConstraintTypes.cpp
 * @brief Various constraint types for physics simulation
 * @details Implements joints, hinges, sliders, and other constraint types
 *          for connecting rigid bodies with specific degrees of freedom
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#include "ConstraintTypes.h"

#include "../dynamics/RigidBody.h"

namespace engine::physics {
    void PointToPointConstraint::setPivotA(const Vec3& pivot) {
        pivotInA_ = pivot;
        if (constraint_) {
            constraint_->setPivotA(btVector3(pivot.x, pivot.y, pivot.z));
        }
    }

    void PointToPointConstraint::setPivotB(const Vec3& pivot) {
        pivotInB_ = pivot;
        if (constraint_) {
            constraint_->setPivotB(btVector3(pivot.x, pivot.y, pivot.z));
        }
    }

    void PointToPointConstraint::setImpulseClamp(const Float clamp) const {
        if (constraint_) {
            constraint_->m_setting.m_impulseClamp = clamp;
        }
    }

    void PointToPointConstraint::setDamping(const Float damping) const {
        if (constraint_) {
            constraint_->m_setting.m_damping = damping;
        }
    }

    void PointToPointConstraint::createConstraint() {
        if (!bodyA_ || !bodyA_->getBulletBody()) return;

        const btVector3 pivotA(pivotInA_.x, pivotInA_.y, pivotInA_.z);

        if (bodyB_ && bodyB_->getBulletBody()) {
            const btVector3 pivotB(pivotInB_.x, pivotInB_.y, pivotInB_.z);
            constraint_ = new btPoint2PointConstraint(
                *bodyA_->getBulletBody(), *bodyB_->getBulletBody(),
                pivotA, pivotB
            );
        }
        else {
            constraint_ = new btPoint2PointConstraint(*bodyA_->getBulletBody(), pivotA);
        }

        constraint_->setBreakingImpulseThreshold(breakingImpulse_);
    }

    void HingeConstraint::setLimits(const Float low, const Float high) {
        params_.lowLimit = low;
        params_.highLimit = high;
        if (constraint_) {
            constraint_->setLimit(low, high, params_.softness,
                                  params_.biasFactor, params_.relaxationFactor);
        }
    }

    void HingeConstraint::enableMotor(const bool enable) {
        params_.enableMotor = enable;
        if (constraint_) {
            constraint_->enableMotor(enable);
        }
    }

    void HingeConstraint::setMotorTarget(const Float velocity, const Float maxImpulse) {
        params_.motorTargetVelocity = velocity;
        params_.maxMotorImpulse = maxImpulse;
        if (constraint_) {
            constraint_->setMotorTargetVelocity(velocity);
            constraint_->setMaxMotorImpulse(maxImpulse);
        }
    }

    Float HingeConstraint::getHingeAngle() const {
        return constraint_ ? constraint_->getHingeAngle() : 0.0f;
    }

    Float HingeConstraint::getAngularVelocity() const {
        if (!constraint_) return 0.0f;

        // Calculate angular velocity around hinge axis
        const Vec3 angVelA = bodyA_ ? bodyA_->getAngularVelocity() : Vec3(0);
        const Vec3 angVelB = bodyB_ ? bodyB_->getAngularVelocity() : Vec3(0);
        const Vec3 relAngVel = angVelA - angVelB;

        return glm::dot(relAngVel, params_.axisInA);
    }

    void HingeConstraint::createConstraint() {
        if (!bodyA_ || !bodyA_->getBulletBody()) return;

        btTransform frameA;
        frameA.setIdentity();
        frameA.setOrigin(btVector3(params_.pivotInA.x, params_.pivotInA.y, params_.pivotInA.z));

        // Create rotation from axis
        const btVector3 axis(params_.axisInA.x, params_.axisInA.y, params_.axisInA.z);
        const btQuaternion rotation(axis, 0);
        frameA.setRotation(rotation);

        if (bodyB_ && bodyB_->getBulletBody()) {
            btTransform frameB;
            frameB.setIdentity();
            frameB.setOrigin(btVector3(params_.pivotInB.x, params_.pivotInB.y, params_.pivotInB.z));

            const btVector3 axisB(params_.axisInB.x, params_.axisInB.y, params_.axisInB.z);
            frameB.setRotation(btQuaternion(axisB, 0));

            constraint_ = new btHingeConstraint(
                *bodyA_->getBulletBody(), *bodyB_->getBulletBody(),
                frameA, frameB
            );
        }
        else {
            constraint_ = new btHingeConstraint(*bodyA_->getBulletBody(), frameA);
        }

        constraint_->setLimit(params_.lowLimit, params_.highLimit,
                              params_.softness, params_.biasFactor,
                              params_.relaxationFactor);

        if (params_.enableMotor) {
            constraint_->enableMotor(true);
            constraint_->setMotorTargetVelocity(params_.motorTargetVelocity);
            constraint_->setMaxMotorImpulse(params_.maxMotorImpulse);
        }

        constraint_->setBreakingImpulseThreshold(breakingImpulse_);
    }

    void SliderConstraint::setLinearLimits(const Float lower, const Float upper) {
        params_.lowerLinLimit = lower;
        params_.upperLinLimit = upper;

        if (constraint_) {
            constraint_->setLowerLinLimit(lower);
            constraint_->setUpperLinLimit(upper);
        }
    }

    void SliderConstraint::setAngularLimits(const Float lower, const Float upper) {
        params_.lowerAngLimit = lower;
        params_.upperAngLimit = upper;

        if (constraint_) {
            constraint_->setLowerAngLimit(lower);
            constraint_->setUpperAngLimit(upper);
        }
    }

    void SliderConstraint::setLinearMotor(const bool enable, const Float velocity, const Float maxForce) {
        params_.enableLinearMotor = enable;
        params_.linearMotorVelocity = velocity;
        params_.maxLinearMotorForce = maxForce;

        if (constraint_) {
            constraint_->setPoweredLinMotor(enable);
            constraint_->setTargetLinMotorVelocity(velocity);
            constraint_->setMaxLinMotorForce(maxForce);
        }
    }

    Float SliderConstraint::getLinearPosition() const {
        return constraint_ ? constraint_->getLinearPos() : 0.0f;
    }

    Float SliderConstraint::getAngularPosition() const {
        return constraint_ ? constraint_->getAngularPos() : 0.0f;
    }

    void SliderConstraint::createConstraint() {
        if (!bodyA_ || !bodyA_->getBulletBody()) return;
        if (!bodyB_ || !bodyB_->getBulletBody()) return;

        const btTransform frameA = toBulletTransform(params_.frameInA);
        const btTransform frameB = toBulletTransform(params_.frameInB);

        constraint_ = new btSliderConstraint(
            *bodyA_->getBulletBody(), *bodyB_->getBulletBody(),
            frameA, frameB, true
        );

        constraint_->setLowerLinLimit(params_.lowerLinLimit);
        constraint_->setUpperLinLimit(params_.upperLinLimit);
        constraint_->setLowerAngLimit(params_.lowerAngLimit);
        constraint_->setUpperAngLimit(params_.upperAngLimit);

        constraint_->setDampingDirLin(params_.linearDamping);
        constraint_->setDampingDirAng(params_.angularDamping);

        if (params_.enableLinearMotor) {
            constraint_->setPoweredLinMotor(true);
            constraint_->setTargetLinMotorVelocity(params_.linearMotorVelocity);
            constraint_->setMaxLinMotorForce(params_.maxLinearMotorForce);
        }

        if (params_.enableAngularMotor) {
            constraint_->setPoweredAngMotor(true);
            constraint_->setTargetAngMotorVelocity(params_.angularMotorVelocity);
            constraint_->setMaxAngMotorForce(params_.maxAngularMotorForce);
        }

        constraint_->setBreakingImpulseThreshold(breakingImpulse_);
    }

    btTransform SliderConstraint::toBulletTransform(const Transform& transform) {
        const Vec3 position = transform.getPosition();
        const Quat rotation = transform.getRotation();
        btTransform btTrans;

        btTrans.setOrigin(btVector3(position.x, position.y, position.z));
        btTrans.setRotation(btQuaternion(rotation.x, rotation.y,
                                         rotation.z, rotation.w));
        return btTrans;
    }

    void Generic6DofConstraint::setLinearLimit(const Int axis, const Float lower, const Float upper) {
        if (axis < 0 || axis > 2) return;

        if (axis == 0) params_.linearLowerLimit.x = lower;
        else if (axis == 1) params_.linearLowerLimit.y = lower;
        else params_.linearLowerLimit.z = lower;

        if (axis == 0) params_.linearUpperLimit.x = upper;
        else if (axis == 1) params_.linearUpperLimit.y = upper;
        else params_.linearUpperLimit.z = upper;

        if (constraint_) {
            constraint_->setLimit(axis, lower, upper);
        }
    }

    void Generic6DofConstraint::setAngularLimit(const Int axis, const Float lower, const Float upper) {
        if (axis < 0 || axis > 2) return;

        if (axis == 0) params_.angularLowerLimit.x = lower;
        else if (axis == 1) params_.angularLowerLimit.y = lower;
        else params_.angularLowerLimit.z = lower;

        if (axis == 0) params_.angularUpperLimit.x = upper;
        else if (axis == 1) params_.angularUpperLimit.y = upper;
        else params_.angularUpperLimit.z = upper;

        if (constraint_) {
            constraint_->setLimit(axis + 3, lower, upper);
        }
    }

    void Generic6DofConstraint::setLinearMotor(const Int axis, const bool enable, const Float velocity, const Float maxForce) {
        if (axis < 0 || axis > 2) return;

        params_.enableLinearMotor[axis] = enable;

        if (axis == 0) {
            params_.linearMotorVelocity.x = velocity;
            params_.maxLinearMotorForce.x = maxForce;
        }
        else if (axis == 1) {
            params_.linearMotorVelocity.y = velocity;
            params_.maxLinearMotorForce.y = maxForce;
        }
        else {
            params_.linearMotorVelocity.z = velocity;
            params_.maxLinearMotorForce.z = maxForce;
        }

        if (constraint_) {
            btTranslationalLimitMotor* motor = constraint_->getTranslationalLimitMotor();
            motor->m_enableMotor[axis] = enable;
            motor->m_targetVelocity[axis] = velocity;
            motor->m_maxMotorForce[axis] = maxForce;
        }
    }

    void Generic6DofConstraint::setAngularMotor(const Int axis, const bool enable, const Float velocity, const Float maxForce) {
        if (axis < 0 || axis > 2) return;

        params_.enableAngularMotor[axis] = enable;

        if (axis == 0) {
            params_.angularMotorVelocity.x = velocity;
            params_.maxAngularMotorForce.x = maxForce;
        }
        else if (axis == 1) {
            params_.angularMotorVelocity.y = velocity;
            params_.maxAngularMotorForce.y = maxForce;
        }
        else {
            params_.angularMotorVelocity.z = velocity;
            params_.maxAngularMotorForce.z = maxForce;
        }

        if (constraint_) {
            btRotationalLimitMotor* motor = constraint_->getRotationalLimitMotor(axis);
            motor->m_enableMotor = enable;
            motor->m_targetVelocity = velocity;
            motor->m_maxMotorForce = maxForce;
        }
    }

    Vec3 Generic6DofConstraint::getLinearPosition() const {
        if (!constraint_) return Vec3(0);

        const btVector3 pos = constraint_->getCalculatedTransformA().getOrigin() -
            constraint_->getCalculatedTransformB().getOrigin();
        return Vec3(pos.x(), pos.y(), pos.z());
    }

    Vec3 Generic6DofConstraint::getAngularPosition() const {
        if (!constraint_) return Vec3(0);

        const btVector3 angle = constraint_->getCalculatedTransformA().getRotation().getAxis() *
            constraint_->getCalculatedTransformA().getRotation().getAngle();
        return Vec3(angle.x(), angle.y(), angle.z());
    }

    void Generic6DofConstraint::lockTranslation() {
        setLinearLimit(0, 0, 0);
        setLinearLimit(1, 0, 0);
        setLinearLimit(2, 0, 0);
    }

    void Generic6DofConstraint::lockRotation() {
        setAngularLimit(0, 0, 0);
        setAngularLimit(1, 0, 0);
        setAngularLimit(2, 0, 0);
    }

    void Generic6DofConstraint::createConstraint() {
            if (!bodyA_ || !bodyA_->getBulletBody()) return;
            if (!bodyB_ || !bodyB_->getBulletBody()) return;

            btTransform frameA = toBulletTransform(params_.frameInA);
            btTransform frameB = toBulletTransform(params_.frameInB);

            constraint_ = new btGeneric6DofConstraint(
                *bodyA_->getBulletBody(), *bodyB_->getBulletBody(),
                frameA, frameB, true
            );

            // Set linear limits
            constraint_->setLinearLowerLimit(btVector3(
                params_.linearLowerLimit.x,
                params_.linearLowerLimit.y,
                params_.linearLowerLimit.z
            ));
            constraint_->setLinearUpperLimit(btVector3(
                params_.linearUpperLimit.x,
                params_.linearUpperLimit.y,
                params_.linearUpperLimit.z
            ));

            // Set angular limits
            constraint_->setAngularLowerLimit(btVector3(
                params_.angularLowerLimit.x,
                params_.angularLowerLimit.y,
                params_.angularLowerLimit.z
            ));
            constraint_->setAngularUpperLimit(btVector3(
                params_.angularUpperLimit.x,
                params_.angularUpperLimit.y,
                params_.angularUpperLimit.z
            ));

            // Set linear motors
            btTranslationalLimitMotor* linMotor = constraint_->getTranslationalLimitMotor();
            for (int i = 0; i < 3; ++i) {
                linMotor->m_enableMotor[i] = params_.enableLinearMotor[i];
                linMotor->m_targetVelocity[i] = (&params_.linearMotorVelocity.x)[i];
                linMotor->m_maxMotorForce[i] = (&params_.maxLinearMotorForce.x)[i];
            }

            // Set angular motors
            for (int i = 0; i < 3; ++i) {
                btRotationalLimitMotor* angMotor = constraint_->getRotationalLimitMotor(i);
                angMotor->m_enableMotor = params_.enableAngularMotor[i];
                angMotor->m_targetVelocity = (&params_.angularMotorVelocity.x)[i];
                angMotor->m_maxMotorForce = (&params_.maxAngularMotorForce.x)[i];
            }

            constraint_->setBreakingImpulseThreshold(breakingImpulse_);
        }

    btTransform Generic6DofConstraint::toBulletTransform(const Transform& transform) {
        const Vec3 position = transform.getPosition();
        const Quat rotation = transform.getRotation();
        btTransform btTrans;

        btTrans.setOrigin(btVector3(position.x, position.y, position.z));
        btTrans.setRotation(btQuaternion(rotation.x, rotation.y,
                                         rotation.z, rotation.w));
        return btTrans;
    }

    Generic6DofConstraint::DofParams FixedConstraint::createFixedParams(const Transform& frameInA, const Transform& frameInB) {
        DofParams params;
        params.frameInA = frameInA;
        params.frameInB = frameInB;

        // Lock all axes
        params.linearLowerLimit = Vec3(0);
        params.linearUpperLimit = Vec3(0);
        params.angularLowerLimit = Vec3(0);
        params.angularUpperLimit = Vec3(0);

        return params;
    }
} // namespace engine::physics
