/**
 * @file BulletIntegration.h
 * @brief Integration layer for Bullet Physics engine
 * @details Provides abstraction and utilities for Bullet Physics integration
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

#include "../core/PhysicsTypes.h"

#include <memory>
#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>

struct btSoftBodyWorldInfo;

namespace engine::physics {
    struct PhysicsConfig;
    using namespace engine::math;

    /**
     * @brief Convert engine vector to Bullet vector
     */
    inline btVector3 toBulletVector(const Vec3& v) {
        return btVector3(v.x, v.y, v.z);
    }

    /**
     * @brief Convert Bullet vector to engine vector
     */
    inline Vec3 fromBulletVector(const btVector3& v) {
        return Vec3(v.x(), v.y(), v.z());
    }

    /**
     * @brief Convert engine quaternion to Bullet quaternion
     */
    inline btQuaternion toBulletQuaternion(const Quat& q) {
        return btQuaternion(q.x, q.y, q.z, q.w);
    }

    /**
     * @brief Convert Bullet quaternion to engine quaternion
     */
    inline Quat fromBulletQuaternion(const btQuaternion& q) {
        return Quat(q.w(), q.x(), q.y(), q.z());
    }

    /**
     * @brief Convert engine transform to Bullet transform
     */
    inline btTransform toBulletTransform(const Transform& t) {
        btTransform transform;

        const Vec3 position = t.getPosition();
        const Quat rotation = t.getRotation();

        transform.setOrigin(toBulletVector(position));
        transform.setRotation(toBulletQuaternion(rotation));

        return transform;
    }

    /**
     * @brief Convert Bullet transform to engine transform
     */
    inline Transform fromBulletTransform(const btTransform& t) {
        Transform transform;

        const Vec3 position = fromBulletVector(t.getOrigin());
        const Quat rotation = fromBulletQuaternion(t.getRotation());
        const auto scale = Vec3(1.0f); // Bullet doesn't support scale in transforms

        transform.setPosition(position);
        transform.setRotation(rotation);
        transform.setScale(scale);

        return transform;
    }

    /**
     * @brief Convert engine matrix to Bullet transform
     */
    inline btTransform toBulletTransform(const Mat4& matrix) {
        btTransform transform;
        transform.setFromOpenGLMatrix(&matrix[0][0]);
        return transform;
    }

    /**
     * @brief Convert Bullet transform to engine matrix
     */
    inline Mat4 fromBulletTransformToMatrix(const btTransform& transform) {
        Mat4 matrix;
        transform.getOpenGLMatrix(&matrix[0][0]);
        return matrix;
    }

    /**
     * @brief Bullet debug drawer implementation
     */
    class BulletDebugDrawer final : public btIDebugDraw {
    public:
        BulletDebugDrawer() : debugMode_(DBG_DrawWireframe | DBG_DrawConstraints) {
        }

        void drawLine(const btVector3& from, const btVector3& to, const btVector3& color) override;

        void drawContactPoint(const btVector3& pointOnB, const btVector3& normalOnB,
                              btScalar distance, int lifeTime, const btVector3& color) override;

        void reportErrorWarning(const char* warningString) override {
            errors_.push_back(warningString);
        }

        void draw3dText(const btVector3& location, const char* textString) override;

        void setDebugMode(const int debugMode) override { debugMode_ = debugMode; }
        int getDebugMode() const override { return debugMode_; }

        void clearLines() override { lines_.clear(); }

        // Additional methods for retrieving debug data
        struct DebugLine {
            Vec3 from, to, color;
        };

        struct ContactPointInfo {
            Vec3 point, normal, color;
            Float distance;
            Int lifetime;
        };

        struct Text3D {
            Vec3 position;
            std::string text;
        };

        const std::vector<DebugLine>& getLines() const { return lines_; }
        const std::vector<ContactPointInfo>& getContactPoints() const { return contactPoints_; }
        const std::vector<std::string>& getErrors() const { return errors_; }
        const std::vector<Text3D>& getText3D() const { return text3D_; }

        void clear();

    private:
        int debugMode_;
        std::vector<DebugLine> lines_;
        std::vector<ContactPointInfo> contactPoints_;
        std::vector<std::string> errors_;
        std::vector<Text3D> text3D_;
    };

    /**
     * @brief Bullet motion state for transform synchronization
     */
    class PhysicsMotionState final : public btMotionState {
    public:
        explicit PhysicsMotionState(Transform* transform)
            : transform_(transform), centerOfMass_(0, 0, 0) {
            if (transform_) {
                const Vec3& pos = transform_->getWorldPosition();
                const Quat& rot = transform_->getWorldRotation();
                initialTransform_.setOrigin(btVector3(pos.x, pos.y, pos.z));
                initialTransform_.setRotation(btQuaternion(rot.x, rot.y, rot.z, rot.w));
            }
        }

        void getWorldTransform(btTransform& worldTrans) const override;

        void setWorldTransform(const btTransform& worldTrans) override;

        void setCenterOfMass(const Vec3& com) { centerOfMass_ = com; }
        [[nodiscard]] const Vec3& getCenterOfMass() const { return centerOfMass_; }

    private:
        Transform* transform_;
        Vec3 centerOfMass_;
        btTransform initialTransform_;
    };

    /**
     * @brief Bullet collision callback for event handling
     */
    class PhysicsCollisionCallback final : public btCollisionWorld::ContactResultCallback {
    public:
        using CollisionCallback = std::function<void(btManifoldPoint&, const btCollisionObject*,
                                                     const btCollisionObject*)>;

        explicit PhysicsCollisionCallback(const CollisionCallback& callback) : callback_(callback) {
        }

        btScalar addSingleResult(btManifoldPoint& cp,
                                 const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0,
                                 const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1) override {
            if (callback_) {
                callback_(cp, colObj0Wrap->getCollisionObject(), colObj1Wrap->getCollisionObject());
            }
            return 0;
        }

    private:
        CollisionCallback callback_;
    };

    /**
     * @brief RAII wrapper for Bullet Physics World components
     * @details Manages the lifetime and dependencies of Bullet Physics components
     * This ensures proper initialization order and cleanup
     */
    struct BulletWorldComponents {
        // Order matters! These must be destroyed in reverse order of construction
        std::unique_ptr<btCollisionConfiguration> collisionConfig;
        std::unique_ptr<btCollisionDispatcher> dispatcher;
        std::unique_ptr<btBroadphaseInterface> broadphase;
        std::unique_ptr<btConstraintSolver> solver;

        // World must be destroyed before its dependencies
        std::unique_ptr<btDiscreteDynamicsWorld> world;

        /**
         * @brief Construct components in proper order
         */
        BulletWorldComponents() = default;

        /**
         * @brief Move constructor for efficient transfer
         */
        BulletWorldComponents(BulletWorldComponents&&) = default;
        BulletWorldComponents& operator=(BulletWorldComponents&&) = default;

        // Prevent copying to avoid double-deletion issues
        BulletWorldComponents(const BulletWorldComponents&) = delete;
        BulletWorldComponents& operator=(const BulletWorldComponents&) = delete;

        /**
         * @brief Check if world is properly initialized
         */
        [[nodiscard]] bool isValid() const {
            return world && collisionConfig && dispatcher && broadphase && solver;
        }
    };

    /**
     * @brief RAII wrapper for Bullet Soft Body World components
     */
    struct BulletSoftWorldComponents {
        // Order matters! These must be destroyed in reverse order of construction
        std::unique_ptr<btSoftBodyRigidBodyCollisionConfiguration> collisionConfig;
        std::unique_ptr<btCollisionDispatcher> dispatcher;
        std::unique_ptr<btBroadphaseInterface> broadphase;
        std::unique_ptr<btConstraintSolver> solver;

        // Soft body world info and world
        std::unique_ptr<btSoftBodyWorldInfo> worldInfo;
        std::unique_ptr<btSoftRigidDynamicsWorld> world;

        /**
         * @brief Construct components in proper order
         */
        BulletSoftWorldComponents() = default;

        /**
         * @brief Move constructor for efficient transfer
         */
        BulletSoftWorldComponents(BulletSoftWorldComponents&&) = default;
        BulletSoftWorldComponents& operator=(BulletSoftWorldComponents&&) = default;

        // Prevent copying
        BulletSoftWorldComponents(const BulletSoftWorldComponents&) = delete;
        BulletSoftWorldComponents& operator=(const BulletSoftWorldComponents&) = delete;

        /**
         * @brief Check if world is properly initialized
         */
        [[nodiscard]] bool isValid() const {
            return world && worldInfo && collisionConfig && dispatcher && broadphase && solver;
        }
    };

    /**
     * @brief Improved Bullet world factory with proper resource management
     * @details Factory class that creates properly configured Bullet Physics worlds
     * with correct component lifetime management
     */
    class BulletWorldFactory {
    public:
        /**
         * @brief Create standard rigid body world with all components
         * @param config Physics world configuration
         * @return Complete world components with proper lifetime management
         */
        static BulletWorldComponents createRigidWorld(const PhysicsConfig& config);

        /**
         * @brief Create soft body world with all components
         * @param config Physics world configuration
         * @return Complete soft world components with proper lifetime management
         */
        static BulletSoftWorldComponents createSoftWorld(const PhysicsConfig& config);

    private:
        /**
         * @brief Create collision configuration based on world type
         */
        static std::unique_ptr<btCollisionConfiguration> createCollisionConfig(bool softBody = false);

        /**
         * @brief Create broadphase interface based on configuration
         */
        static std::unique_ptr<btBroadphaseInterface> createBroadphase(const PhysicsConfig& config);

        /**
         * @brief Create constraint solver
         */
        static std::unique_ptr<btConstraintSolver> createSolver();

        /**
         * @brief Apply advanced configuration settings to world
         */
        static void configureAdvancedSettings(btDiscreteDynamicsWorld* world, const PhysicsConfig& config);
    };

    /**
     * @brief Bullet physics utilities
     */
    class BulletUtils {
    public:
        /**
         * @brief Create collision shape from mesh
         */
        static std::unique_ptr<btCollisionShape> createMeshShape(const std::vector<Vec3>& vertices,
                                                                const std::vector<Int>& indices,
                                                                bool isConvex = false);

        /**
         * @brief Create compound shape from multiple shapes
         */
        static std::unique_ptr<btCompoundShape> createCompoundShape(
            const std::vector<std::pair<std::unique_ptr<btCollisionShape>, Transform>>& shapes);

        /**
         * @brief Calculate inertia tensor
         */
        static Vec3 calculateInertia(const btCollisionShape* shape, Float mass);

        /**
         * @brief Enable CCD for body
         */
        static void enableCCD(btRigidBody* body, Float motionThreshold, Float sweptSphereRadius);

        /**
         * @brief Create rigid body from shape and properties
         */
        static std::unique_ptr<btRigidBody> createRigidBody(
            std::unique_ptr<btCollisionShape> shape,
            Float mass,
            const Transform& transform,
            const Vec3& linearVelocity = Vec3(0),
            const Vec3& angularVelocity = Vec3(0)
        );
    };
} // namespace engine::physics