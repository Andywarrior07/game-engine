/**
 * @file BulletWrappers.h
 * @brief RAII wrappers for Bullet Physics objects
 * @details Provides safe, RAII-compliant wrappers for Bullet objects
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

#include <functional>
#include <memory>

class btTypedConstraint;
class btRigidBody;
class btMotionState;
class btCollisionShape;

namespace engine::physics {
/**
 * @brief RAII wrapper for btCollisionShape
 */
class BulletShapeWrapper {
public:
  BulletShapeWrapper() : shape_(nullptr) {}

  explicit BulletShapeWrapper(btCollisionShape *shape) : shape_(shape) {}

  BulletShapeWrapper(BulletShapeWrapper &&other) noexcept
      : shape_(other.shape_) {
    other.shape_ = nullptr;
  }

  ~BulletShapeWrapper();

  BulletShapeWrapper &operator=(BulletShapeWrapper &&other) noexcept {
    if (this != &other) {
      release();
      shape_ = other.shape_;
      other.shape_ = nullptr;
    }
    return *this;
  }

  void reset(btCollisionShape *shape = nullptr);

  btCollisionShape *get() const { return shape_; }
  btCollisionShape *operator->() const { return shape_; }
  explicit operator bool() const { return shape_ != nullptr; }

  btCollisionShape *release();

private:
  btCollisionShape *shape_;

  BulletShapeWrapper(const BulletShapeWrapper &) = delete;
  BulletShapeWrapper &operator=(const BulletShapeWrapper &) = delete;
};

/**
 * @brief RAII wrapper for btRigidBody
 */
class BulletBodyWrapper {
public:
  BulletBodyWrapper() : body_(nullptr), motionState_(nullptr) {}

  BulletBodyWrapper(btRigidBody *body, btMotionState *motionState)
      : body_(body), motionState_(motionState) {}

  BulletBodyWrapper(BulletBodyWrapper &&other) noexcept
      : body_(other.body_), motionState_(other.motionState_) {
    other.body_ = nullptr;
    other.motionState_ = nullptr;
  }

  ~BulletBodyWrapper();

  BulletBodyWrapper &operator=(BulletBodyWrapper &&other) noexcept {
    if (this != &other) {
      release();
      body_ = other.body_;
      motionState_ = other.motionState_;
      other.body_ = nullptr;
      other.motionState_ = nullptr;
    }
    return *this;
  }

  void reset(btRigidBody *body = nullptr, btMotionState *motionState = nullptr);

  btRigidBody *get() const { return body_; }
  btRigidBody *operator->() const { return body_; }
  explicit operator bool() const { return body_ != nullptr; }

  void release();

private:
  btRigidBody *body_;
  btMotionState *motionState_;

  BulletBodyWrapper(const BulletBodyWrapper &) = delete;
  BulletBodyWrapper &operator=(const BulletBodyWrapper &) = delete;
};

/**
 * @brief RAII wrapper for btTypedConstraint
 */
class BulletConstraintWrapper {
public:
  BulletConstraintWrapper() : constraint_(nullptr) {}

  explicit BulletConstraintWrapper(btTypedConstraint *constraint)
      : constraint_(constraint) {}

  BulletConstraintWrapper(BulletConstraintWrapper &&other) noexcept
      : constraint_(other.constraint_) {
    other.constraint_ = nullptr;
  }

  ~BulletConstraintWrapper();

  BulletConstraintWrapper &operator=(BulletConstraintWrapper &&other) noexcept {
    if (this != &other) {
      release();
      constraint_ = other.constraint_;
      other.constraint_ = nullptr;
    }
    return *this;
  }

  void reset(btTypedConstraint *constraint = nullptr);

  btTypedConstraint *get() const { return constraint_; }
  btTypedConstraint *operator->() const { return constraint_; }
  explicit operator bool() const { return constraint_ != nullptr; }

  void release();

private:
  btTypedConstraint *constraint_;

  BulletConstraintWrapper(const BulletConstraintWrapper &) = delete;
  BulletConstraintWrapper &operator=(const BulletConstraintWrapper &) = delete;
};

/**
 * @brief Smart pointer factory for Bullet objects
 */
class BulletFactory {
public:
  using ShapeDeleter = std::function<void(btCollisionShape *)>;
  using BodyDeleter = std::function<void(btRigidBody *)>;
  using ConstraintDeleter = std::function<void(btTypedConstraint *)>;

  static std::unique_ptr<btCollisionShape, ShapeDeleter>
  createShape(btCollisionShape *shape);

  static std::unique_ptr<btRigidBody, BodyDeleter>
  createBody(btRigidBody *body);

  static std::unique_ptr<btTypedConstraint, ConstraintDeleter>
  createConstraint(btTypedConstraint *constraint);
};
} // namespace engine::physics
