/**
 * @file BulletIntegration.cpp
 * @brief Integration layer for Bullet Physics engine
 * @details Provides abstraction and utilities for Bullet Physics integration
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#include "BulletIntegration.h"

#include "../core/PhysicsConfig.h"

#include <BulletSoftBody/btSoftBody.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>

namespace engine::physics {

    // ============================================================================
    // BulletDebugDrawer Implementation
    // ============================================================================

    void BulletDebugDrawer::drawLine(const btVector3& from, const btVector3& to, const btVector3& color) {
        lines_.push_back({
            fromBulletVector(from),
            fromBulletVector(to),
            Vec3(color.x(), color.y(), color.z())
        });
    }

    void BulletDebugDrawer::drawContactPoint(const btVector3& pointOnB, const btVector3& normalOnB,
                                             const btScalar distance, const int lifeTime, const btVector3& color) {
        const Vec3 point = fromBulletVector(pointOnB);
        const Vec3 normal = fromBulletVector(normalOnB);
        const Vec3 end = point + normal * distance;

        contactPoints_.push_back({
            point, end, Vec3(color.x(), color.y(), color.z()),
            static_cast<Float>(distance), lifeTime
        });
    }

    void BulletDebugDrawer::draw3dText(const btVector3& location, const char* textString) {
        text3D_.push_back({
            fromBulletVector(location),
            std::string(textString)
        });
    }

    void BulletDebugDrawer::clear() {
        lines_.clear();
        contactPoints_.clear();
        errors_.clear();
        text3D_.clear();
    }

    // ============================================================================
    // PhysicsMotionState Implementation
    // ============================================================================

    void PhysicsMotionState::getWorldTransform(btTransform& worldTrans) const {
        if (transform_) {
            const Vec3& pos = transform_->getWorldPosition();
            const Quat& rot = transform_->getWorldRotation();

            worldTrans.setOrigin(btVector3(pos.x, pos.y, pos.z));
            worldTrans.setRotation(btQuaternion(rot.x, rot.y, rot.z, rot.w));
        }
        else {
            // Fallback si no hay transform asignado
            worldTrans = initialTransform_;
        }
    }

    void PhysicsMotionState::setWorldTransform(const btTransform& worldTrans) {
        if (transform_) {
            const btVector3& origin = worldTrans.getOrigin();
            const btQuaternion& rotation = worldTrans.getRotation();

            transform_->setWorldPosition(Vec3(origin.x(), origin.y(), origin.z()));
            transform_->setRotation(Quat(rotation.w(), rotation.x(),
                                         rotation.y(), rotation.z()));
        }
    }

    // ============================================================================
    // BulletWorldFactory Implementation
    // ============================================================================

    BulletWorldComponents BulletWorldFactory::createRigidWorld(const PhysicsConfig& config) {
        BulletWorldComponents components;

        // Create components in proper dependency order
        components.collisionConfig = createCollisionConfig(false);
        components.dispatcher = std::make_unique<btCollisionDispatcher>(components.collisionConfig.get());
        components.broadphase = createBroadphase(config);
        components.solver = createSolver();

        // Create the dynamics world
        components.world = std::make_unique<btDiscreteDynamicsWorld>(
            components.dispatcher.get(),
            components.broadphase.get(),
            components.solver.get(),
            components.collisionConfig.get()
        );

        // Configure world properties
        components.world->setGravity(btVector3(config.gravity.x, config.gravity.y, config.gravity.z));
        components.world->getSolverInfo().m_numIterations = config.solverIterations;
        components.world->getSolverInfo().m_erp = config.erp;
        components.world->getSolverInfo().m_globalCfm = config.globalCFM;

        // Enable split impulse for better stability
        components.world->getSolverInfo().m_splitImpulse = true;
        components.world->getSolverInfo().m_splitImpulsePenetrationThreshold = -0.02f;

        return components;
    }

    BulletSoftWorldComponents BulletWorldFactory::createSoftWorld(const PhysicsConfig& config) {
        BulletSoftWorldComponents components;

        // Create soft body collision configuration
        components.collisionConfig = std::make_unique<btSoftBodyRigidBodyCollisionConfiguration>();
        components.dispatcher = std::make_unique<btCollisionDispatcher>(components.collisionConfig.get());
        components.broadphase = createBroadphase(config);
        components.solver = createSolver();

        // Create soft body world info
        components.worldInfo = std::make_unique<btSoftBodyWorldInfo>();
        components.worldInfo->m_broadphase = components.broadphase.get();
        components.worldInfo->m_dispatcher = components.dispatcher.get();
        components.worldInfo->m_gravity = btVector3(config.gravity.x, config.gravity.y, config.gravity.z);
        components.worldInfo->m_sparsesdf.Initialize();

        // Create the soft-rigid dynamics world
        components.world = std::make_unique<btSoftRigidDynamicsWorld>(
            components.dispatcher.get(),
            components.broadphase.get(),
            components.solver.get(),
            components.collisionConfig.get()
        );

        // Configure world properties
        components.world->setGravity(btVector3(config.gravity.x, config.gravity.y, config.gravity.z));
        components.world->getSolverInfo().m_numIterations = config.solverIterations;
        components.world->getSolverInfo().m_erp = config.erp;
        components.world->getSolverInfo().m_globalCfm = config.globalCFM;

        // Configure soft body world
        components.world->getWorldInfo().m_gravity = components.world->getGravity();
        components.world->getWorldInfo().m_sparsesdf.Initialize();

        return components;
    }

    std::unique_ptr<btCollisionConfiguration> BulletWorldFactory::createCollisionConfig(const bool softBody) {
        if (softBody) {
            return std::make_unique<btSoftBodyRigidBodyCollisionConfiguration>();
        }
        return std::make_unique<btDefaultCollisionConfiguration>();
    }

    std::unique_ptr<btBroadphaseInterface> BulletWorldFactory::createBroadphase(const PhysicsConfig& config) {
        switch (config.broadphaseType) {
            case BroadphaseType::AXIS_SWEEP_3: {
                const btVector3 worldMin(config.worldMin.x, config.worldMin.y, config.worldMin.z);
                const btVector3 worldMax(config.worldMax.x, config.worldMax.y, config.worldMax.z);
                return std::make_unique<btAxisSweep3>(worldMin, worldMax, config.maxProxies);
            }
            case BroadphaseType::DYNAMIC_AABB_TREE:
            default:
                return std::make_unique<btDbvtBroadphase>();
        }
    }

    std::unique_ptr<btConstraintSolver> BulletWorldFactory::createSolver() {
        return std::make_unique<btSequentialImpulseConstraintSolver>();
    }

    void BulletWorldFactory::configureAdvancedSettings(btDiscreteDynamicsWorld* world, const PhysicsConfig& config) {
        if (!world) return;

        // Basic world properties
        world->setGravity(btVector3(config.gravity.x, config.gravity.y, config.gravity.z));

        // Solver configuration
        btContactSolverInfo& solverInfo = world->getSolverInfo();
        solverInfo.m_numIterations = config.solverIterations;
        solverInfo.m_erp = config.erp;              // Error reduction parameter
        solverInfo.m_globalCfm = config.globalCFM;   // Constraint force mixing
        solverInfo.m_damping = config.solverDamping;
        solverInfo.m_restingContactRestitutionThreshold = 0.0f;

        // Split impulse configuration
        if (config.splitImpulse) {
            solverInfo.m_splitImpulse = true;
            solverInfo.m_erp2 = config.erp2;
            solverInfo.m_splitImpulsePenetrationThreshold = -config.defaultContactBreakingThreshold;
        }

        // Dispatch configuration
        btDispatcherInfo& dispatchInfo = world->getDispatchInfo();
        dispatchInfo.m_allowedCcdPenetration = config.defaultContactBreakingThreshold;
        dispatchInfo.m_enableSatConvex = true;  // Enable SAT for better convex collision

        // CCD configuration
        if (config.enableCCD) {
            dispatchInfo.m_useContinuous = true;
            // Set global CCD parameters
            gContactBreakingThreshold = config.defaultContactBreakingThreshold;
        }

        // Performance optimizations
        if (config.enableSleeping) {
            world->setForceUpdateAllAabbs(false);  // Only update active objects
        }

        // Friction configuration
        if (!config.enableFriction) {
            // This would need to be set per rigid body, not globally
            // We'll handle this in the rigid body creation utilities
        }

        // Debug configuration
        if (config.enableDebugDraw && config.debugDrawMode > 0) {
            // Debug drawer would be set externally
            // This is just marking that debug drawing is requested
        }
    }

    // ============================================================================
    // BulletUtils Implementation
    // ============================================================================

    std::unique_ptr<btCollisionShape> BulletUtils::createMeshShape(const std::vector<Vec3>& vertices,
                                                                  const std::vector<Int>& indices,
                                                                  const bool isConvex) {
        if (vertices.empty()) {
            return nullptr;
        }

        if (isConvex) {
            // Create convex hull shape
            auto shape = std::make_unique<btConvexHullShape>();

            // Add all vertices to the convex hull
            for (const auto& vertex : vertices) {
                shape->addPoint(btVector3(vertex.x, vertex.y, vertex.z));
            }

            // Optimize the convex hull for better performance
            shape->optimizeConvexHull();
            shape->initializePolyhedralFeatures();

            return std::move(shape);
        }

        // Create triangle mesh for concave shapes
        auto triangleMesh = std::make_unique<btTriangleMesh>();

        // Process indices in groups of 3 (triangles)
        for (size_t i = 0; i + 2 < indices.size(); i += 3) {
            if (indices[i] < vertices.size() &&
                indices[i + 1] < vertices.size() &&
                indices[i + 2] < vertices.size()) {

                const Vec3& v0 = vertices[indices[i]];
                const Vec3& v1 = vertices[indices[i + 1]];
                const Vec3& v2 = vertices[indices[i + 2]];

                triangleMesh->addTriangle(
                    btVector3(v0.x, v0.y, v0.z),
                    btVector3(v1.x, v1.y, v1.z),
                    btVector3(v2.x, v2.y, v2.z)
                );
            }
        }

        // Create BVH triangle mesh shape for static meshes
        // Use quantized AABB compression for memory efficiency
        return std::make_unique<btBvhTriangleMeshShape>(triangleMesh.release(), true, true);
    }

    std::unique_ptr<btCompoundShape> BulletUtils::createCompoundShape(
        const std::vector<std::pair<std::unique_ptr<btCollisionShape>, Transform>>& shapes) {

        auto compound = std::make_unique<btCompoundShape>();

        for (const auto& [shape, transform] : shapes) {
            if (shape) {
                const btTransform bulletTransform = toBulletTransform(transform);
                compound->addChildShape(bulletTransform, shape.get());
            }
        }

        // Recalculate local AABB for optimization
        compound->recalculateLocalAabb();

        return compound;
    }

    Vec3 BulletUtils::calculateInertia(const btCollisionShape* shape, const Float mass) {
        btVector3 inertia(0, 0, 0);
        if (shape && mass > 0.0f) {
            shape->calculateLocalInertia(mass, inertia);
        }
        return fromBulletVector(inertia);
    }

    void BulletUtils::enableCCD(btRigidBody* body, const Float motionThreshold, const Float sweptSphereRadius) {
        if (!body) return;

        // Enable continuous collision detection
        body->setCcdMotionThreshold(motionThreshold);
        body->setCcdSweptSphereRadius(sweptSphereRadius);

        // Additional CCD configuration for better stability
        body->setActivationState(DISABLE_DEACTIVATION);
    }

    std::unique_ptr<btRigidBody> BulletUtils::createRigidBody(
        std::unique_ptr<btCollisionShape> shape,
        const Float mass,
        const Transform& transform,
        const Vec3& linearVelocity,
        const Vec3& angularVelocity) {

        if (!shape) {
            return nullptr;
        }

        // Calculate local inertia
        const Vec3 inertia = calculateInertia(shape.get(), mass);
        const btVector3 localInertia = toBulletVector(inertia);

        // Create motion state for transform synchronization
        auto motionState = std::make_unique<PhysicsMotionState>(const_cast<Transform*>(&transform));

        // Create rigid body construction info
        btRigidBody::btRigidBodyConstructionInfo rbInfo(
            mass,
            motionState.release(), // btRigidBody takes ownership
            shape.release(),       // btRigidBody takes ownership
            localInertia
        );

        // Set initial velocities
        auto rigidBody = std::make_unique<btRigidBody>(rbInfo);
        rigidBody->setLinearVelocity(toBulletVector(linearVelocity));
        rigidBody->setAngularVelocity(toBulletVector(angularVelocity));

        // Configure default properties for game objects
        rigidBody->setRestitution(0.2f);           // Slight bounce
        rigidBody->setFriction(0.7f);              // Realistic friction
        rigidBody->setRollingFriction(0.1f);       // Rolling resistance
        rigidBody->setSpinningFriction(0.1f);      // Spinning resistance

        // Enable deactivation for performance
        rigidBody->forceActivationState(ACTIVE_TAG);
        rigidBody->setDeactivationTime(2.0f);

        return rigidBody;
    }

} // namespace engine::physics