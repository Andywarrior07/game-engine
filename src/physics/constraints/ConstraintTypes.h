/**
 * @file ConstraintTypes.h
 * @brief Various constraint types for physics simulation
 * @details Implements joints, hinges, sliders, and other constraint types
 *          for connecting rigid bodies with specific degrees of freedom
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

#include "../../math/core/MathTypes.h"

#include "../core/PhysicsTypes.h"

namespace engine::physics {
    using namespace engine::math;

    /**
     * @brief Base constraint interface
     */
    class IConstraint {
    public:
        virtual ~IConstraint() = default;

        virtual ConstraintType getType() const = 0;
        virtual void preSolve(Float deltaTime) = 0;
        virtual void solveVelocity(Float deltaTime) = 0;
        virtual bool solvePosition(Float deltaTime) = 0;
        virtual bool isEnabled() const = 0;
        virtual void setEnabled(bool enabled) = 0;
        virtual Float getBreakingImpulse() const = 0;
        virtual void setBreakingImpulse(Float impulse) = 0;
        virtual bool isBroken() const = 0;
        virtual btTypedConstraint* getBulletConstraint() = 0;
    };

    /**
     * @brief Base constraint implementation
     */
    template <typename BulletConstraintType>
    class ConstraintBase : public IConstraint {
    public:
        explicit ConstraintBase(RigidBody* bodyA, RigidBody* bodyB = nullptr)
            : bodyA_(bodyA), bodyB_(bodyB), enabled_(true),
              breakingImpulse_(INFINITY_VALUE<Float>), broken_(false) {
        }

        ~ConstraintBase() override {
            if (constraint_) {
                delete constraint_;
                constraint_ = nullptr;
            }
        }

        bool isEnabled() const override { return enabled_ && !broken_; }

        void setEnabled(bool enabled) override {
            enabled_ = enabled;
            if (constraint_) {
                constraint_->setEnabled(enabled);
            }
        }

        Float getBreakingImpulse() const override { return breakingImpulse_; }

        void setBreakingImpulse(Float impulse) override {
            breakingImpulse_ = impulse;
            if (constraint_) {
                constraint_->setBreakingImpulseThreshold(impulse);
            }
        }

        bool isBroken() const override { return broken_; }

        btTypedConstraint* getBulletConstraint() override { return constraint_; }

        void preSolve(Float deltaTime) override {
            // Check if constraint should break
            if (constraint_ && breakingImpulse_ < INFINITY_VALUE<Float>) {
                Float appliedImpulse = constraint_->getAppliedImpulse();
                if (std::abs(appliedImpulse) > breakingImpulse_) {
                    broken_ = true;
                    constraint_->setEnabled(false);
                }
            }
        }

        void solveVelocity(Float deltaTime) override {
            // Handled by Bullet internally
        }

        bool solvePosition(Float deltaTime) override {
            // Handled by Bullet internally
            return true;
        }

    protected:
        RigidBody* bodyA_;
        RigidBody* bodyB_;
        BulletConstraintType* constraint_ = nullptr;
        bool enabled_;
        Float breakingImpulse_;
        bool broken_;
    };

    /**
     * @brief Point to point constraint (ball joint)
     */
    class PointToPointConstraint final : public ConstraintBase<btPoint2PointConstraint> {
    public:
        PointToPointConstraint(RigidBody* bodyA, const Vec3& pivotInA,
                               RigidBody* bodyB = nullptr, const Vec3& pivotInB = VEC3_ZERO)
            : ConstraintBase(bodyA, bodyB), pivotInA_(pivotInA), pivotInB_(pivotInB) {
            createConstraint();
        }

        ConstraintType getType() const override { return ConstraintType::POINT_TO_POINT; }

        void setPivotA(const Vec3& pivot);

        void setPivotB(const Vec3& pivot);

        Vec3 getPivotInA() const { return pivotInA_; }
        Vec3 getPivotInB() const { return pivotInB_; }

        void setImpulseClamp(Float clamp) const;

        void setDamping(Float damping) const;

    private:
        Vec3 pivotInA_;
        Vec3 pivotInB_;

        void createConstraint();
    };

    /**
     * @brief Hinge constraint (revolute joint)
     */
    class HingeConstraint : public ConstraintBase<btHingeConstraint> {
    public:
        struct HingeParams {
            Vec3 pivotInA = Vec3(0);
            Vec3 pivotInB = Vec3(0);
            Vec3 axisInA = Vec3(0, 1, 0);
            Vec3 axisInB = Vec3(0, 1, 0);
            Float lowLimit = -PI<Float>;
            Float highLimit = PI<Float>;
            Float softness = 0.9f;
            Float biasFactor = 0.3f;
            Float relaxationFactor = 1.0f;
            bool enableMotor = false;
            Float motorTargetVelocity = 0.0f;
            Float maxMotorImpulse = 1.0f;
        };

        HingeConstraint(RigidBody* bodyA, RigidBody* bodyB, const HingeParams& params)
            : ConstraintBase(bodyA, bodyB), params_(params) {
            createConstraint();
        }

        ConstraintType getType() const override { return ConstraintType::HINGE; }

        void setLimits(Float low, Float high);

        void enableMotor(bool enable);

        void setMotorTarget(Float velocity, Float maxImpulse);

        Float getHingeAngle() const;

        Float getAngularVelocity() const;

    private:
        HingeParams params_;

        void createConstraint();
    };

    /**
     * @brief Slider constraint (prismatic joint)
     */
    class SliderConstraint : public ConstraintBase<btSliderConstraint> {
    public:
        struct SliderParams {
            Transform frameInA;
            Transform frameInB;
            Float lowerLinLimit = -1.0f;
            Float upperLinLimit = 1.0f;
            Float lowerAngLimit = 0.0f;
            Float upperAngLimit = 0.0f;
            Float linearDamping = 0.7f;
            Float angularDamping = 0.7f;
            bool enableLinearMotor = false;
            Float linearMotorVelocity = 0.0f;
            Float maxLinearMotorForce = 1.0f;
            bool enableAngularMotor = false;
            Float angularMotorVelocity = 0.0f;
            Float maxAngularMotorForce = 1.0f;
        };

        SliderConstraint(RigidBody* bodyA, RigidBody* bodyB, const SliderParams& params)
            : ConstraintBase(bodyA, bodyB), params_(params) {
            createConstraint();
        }

        ConstraintType getType() const override { return ConstraintType::SLIDER; }

        void setLinearLimits(Float lower, Float upper);

        void setAngularLimits(Float lower, Float upper);

        void setLinearMotor(bool enable, Float velocity, Float maxForce);

        Float getLinearPosition() const;

        Float getAngularPosition() const;

    private:
        SliderParams params_;

        void createConstraint();

        static btTransform toBulletTransform(const Transform& transform);
    };

    /**
     * @brief Generic 6DOF constraint (most flexible)
     */
    class Generic6DofConstraint : public ConstraintBase<btGeneric6DofConstraint> {
    public:
        struct DofParams {
            Transform frameInA;
            Transform frameInB;
            Vec3 linearLowerLimit = Vec3(-INFINITY_VALUE<Float>);
            Vec3 linearUpperLimit = Vec3(INFINITY_VALUE<Float>);
            Vec3 angularLowerLimit = Vec3(-INFINITY_VALUE<Float>);
            Vec3 angularUpperLimit = Vec3(INFINITY_VALUE<Float>);
            Vec3 linearStiffness = Vec3(0);
            Vec3 linearDamping = Vec3(1);
            Vec3 angularStiffness = Vec3(0);
            Vec3 angularDamping = Vec3(1);
            bool enableLinearMotor[3] = {false, false, false};
            Vec3 linearMotorVelocity = Vec3(0);
            Vec3 maxLinearMotorForce = Vec3(0);
            bool enableAngularMotor[3] = {false, false, false};
            Vec3 angularMotorVelocity = Vec3(0);
            Vec3 maxAngularMotorForce = Vec3(0);
        };

        Generic6DofConstraint(RigidBody* bodyA, RigidBody* bodyB, const DofParams& params)
            : ConstraintBase(bodyA, bodyB), params_(params) {
            createConstraint();
        }

        ConstraintType getType() const override { return ConstraintType::GENERIC_6DOF; }

        void setLinearLimit(Int axis, Float lower, Float upper);

        void setAngularLimit(Int axis, Float lower, Float upper);

        void setLinearMotor(Int axis, bool enable, Float velocity, Float maxForce);

        void setAngularMotor(Int axis, bool enable, Float velocity, Float maxForce);

        Vec3 getLinearPosition() const;

        Vec3 getAngularPosition() const;

        /**
         * @brief Lock all linear axes (make it rotation only)
         */
        void lockTranslation();

        /**
         * @brief Lock all angular axes (make it translation only)
         */
        void lockRotation();

    private:
        DofParams params_;

        void createConstraint();

        btTransform toBulletTransform(const Transform& transform);
    };

    /**
     * @brief Fixed constraint (welds two bodies together)
     */
    class FixedConstraint final : public Generic6DofConstraint {
    public:
        FixedConstraint(RigidBody* bodyA, RigidBody* bodyB,
                        const Transform& frameInA = Transform(),
                        const Transform& frameInB = Transform())
            : Generic6DofConstraint(bodyA, bodyB, createFixedParams(frameInA, frameInB)) {
        }

        ConstraintType getType() const override { return ConstraintType::FIXED; }

    private:
        static DofParams createFixedParams(const Transform& frameInA, const Transform& frameInB);
    };
} // namespace engine::physics
