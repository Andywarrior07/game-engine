/**
* @file BulletWrappers.h
 * @brief RAII wrappers for Bullet Physics objects
 * @details Provides safe, RAII-compliant wrappers for Bullet objects
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#include "BulletWrappers.h"

#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletDynamics/ConstraintSolver/btTypedConstraint.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>

namespace engine::physics {
    BulletShapeWrapper::~BulletShapeWrapper() {
        release();
    }

    btCollisionShape* BulletShapeWrapper::release() {
        btCollisionShape* temp = shape_;
        if (shape_) {
            delete shape_;
            shape_ = nullptr;
        }
        return temp;
    }

    void BulletShapeWrapper::reset(btCollisionShape* shape) {
        release();
        shape_ = shape;
    }

    BulletBodyWrapper::~BulletBodyWrapper() {
        release();
    }

    void BulletBodyWrapper::reset(btRigidBody* body, btMotionState* motionState) {
        release();
        body_ = body;
        motionState_ = motionState;
    }

    void BulletBodyWrapper::release() {
        if (body_) {
            delete body_;
            body_ = nullptr;
        }
        if (motionState_) {
            delete motionState_;
            motionState_ = nullptr;
        }
    }

    BulletConstraintWrapper::~BulletConstraintWrapper() {
        release();
    }

    void BulletConstraintWrapper::reset(btTypedConstraint* constraint) {
        release();
        constraint_ = constraint;
    }

    void BulletConstraintWrapper::release() {
        if (constraint_) {
            delete constraint_;
            constraint_ = nullptr;
        }
    }

    std::unique_ptr<btCollisionShape, BulletFactory::ShapeDeleter> BulletFactory::createShape(btCollisionShape* shape) {
        return std::unique_ptr<btCollisionShape, ShapeDeleter>(
            shape, [](const btCollisionShape* s) { delete s; }
        );
    }

    std::unique_ptr<btRigidBody, BulletFactory::BodyDeleter> BulletFactory::createBody(btRigidBody* body) {
        return std::unique_ptr<btRigidBody, BodyDeleter>(
            body, [](btRigidBody* b) {
                if (b->getMotionState()) {
                    delete b->getMotionState();
                }
                delete b;
            }
        );
    }

    std::unique_ptr<btTypedConstraint, BulletFactory::ConstraintDeleter> BulletFactory::createConstraint(
        btTypedConstraint* constraint) {
        return std::unique_ptr<btTypedConstraint, ConstraintDeleter>(
            constraint, [](const btTypedConstraint* c) { delete c; }
        );
    }
} // namespace engine::physics
