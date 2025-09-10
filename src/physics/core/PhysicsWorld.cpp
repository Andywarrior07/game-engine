/**
 * @file PhysicsWorld.cpp
 * @brief Core physics world wrapper and management
 * @details Encapsulates Bullet Physics world with custom memory management,
 *          optimized collision detection, and game-specific features
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#include "PhysicsWorld.h"

#include "PhysicsTypes.h"

#include "../dynamics/RigidBody.h"
#include "../collision/CollisionShapes.h"
#include "../utils/PhysicsDebug.h"

#include <BulletCollision/CollisionDispatch/btSimulationIslandManager.h>

namespace engine::physics {
    MemoryManager* PhysicsWorld::sMemoryManager_ = nullptr;

    PhysicsWorld::PhysicsWorld(MemoryManager& memoryManager, const PhysicsConfig& config)
        : memoryManager_(memoryManager), config_(config), accumulator_(0.0f), interpolationAlpha_(0.0f) {
        initialize();
    }

    PhysicsWorld::~PhysicsWorld() {
        shutdown();
    }

    bool PhysicsWorld::initialize() {
        sMemoryManager_ = &memoryManager_;
        // Set up Bullet's memory allocator to use our system
        btAlignedAllocSetCustom(
            [](const size_t size) -> void* {
                return sMemoryManager_->allocate(size, MemoryCategory::PHYSICS, 16);
            },
            [](void* ptr) {
                sMemoryManager_->deallocate(ptr, MemoryCategory::PHYSICS);
            }
        );

        // Create collision configuration
        collisionConfig_ = std::make_unique<btDefaultCollisionConfiguration>();

        // Create collision dispatcher
        dispatcher_ = std::make_unique<btCollisionDispatcher>(collisionConfig_.get());

        // Create broadphase
        createBroadphase();

        // Create constraint solver
        createSolver();

        // Create dynamics world
        dynamicsWorld_ = std::make_unique<btDiscreteDynamicsWorld>(
            dispatcher_.get(),
            broadphase_.get(),
            solver_.get(),
            collisionConfig_.get()
        );

        // Configure world
        dynamicsWorld_->setGravity(btVector3(
            config_.gravity.x,
            config_.gravity.y,
            config_.gravity.z
        ));

        dynamicsWorld_->getSolverInfo().m_numIterations = config_.solverIterations;
        dynamicsWorld_->getSolverInfo().m_erp = config_.erp;
        dynamicsWorld_->getSolverInfo().m_erp2 = config_.erp2;
        dynamicsWorld_->getSolverInfo().m_globalCfm = config_.globalCFM;
        dynamicsWorld_->getSolverInfo().m_splitImpulse = config_.splitImpulse;
        dynamicsWorld_->getSolverInfo().m_solverMode =
            SOLVER_SIMD | SOLVER_USE_WARMSTARTING;

        if (!config_.enableFriction) {
            dynamicsWorld_->getSolverInfo().m_solverMode |= SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION;
        }

        // Set up internal tick callback for collision events
        dynamicsWorld_->setInternalTickCallback(
            [](btDynamicsWorld* world, btScalar timeStep) {
                static_cast<PhysicsWorld*>(world->getWorldUserInfo())->processCollisionEvents();
            },
            this,
            true // Pre-tick
        );

        dynamicsWorld_->setWorldUserInfo(this);

        // Create debug drawer if enabled
        if (config_.enableDebugDraw) {
            debugDrawer_ = std::make_unique<PhysicsDebugDrawer>();
            dynamicsWorld_->setDebugDrawer(debugDrawer_.get());
        }

        // Allocate pools
        // m_bodyPool.reserve(m_config.initialBodyPoolSize);
        shapePool_.reserve(config_.initialShapePoolSize);
        constraintPool_.reserve(config_.initialConstraintPoolSize);

        initialized_ = true;

        return true;
    }

    void PhysicsWorld::shutdown() {
        if (!initialized_) return;

        // Clear all bodies
        clearAllBodies();

        // Clear all constraints
        clearAllConstraints();

        // Clear collision shapes
        shapePool_.clear();

        // Destroy Bullet objects in reverse order
        dynamicsWorld_.reset();
        solver_.reset();
        broadphase_.reset();
        dispatcher_.reset();
        collisionConfig_.reset();

        initialized_ = false;
    }

    Int PhysicsWorld::stepSimulation(Float deltaTime) {
        if (!initialized_ || deltaTime <= 0.0f) return 0;

        // Cap delta time to prevent spiral of death
        deltaTime = min(deltaTime, 0.25f);

        accumulator_ += deltaTime;
        Int steps = 0;

        // Fixed timestep with interpolation
        while (accumulator_ >= config_.fixedTimeStep && steps < config_.maxSubSteps) {
            // Store previous transforms for interpolation
            storePreviousTransforms();

            // Step simulation
            dynamicsWorld_->stepSimulation(
                config_.fixedTimeStep,
                0, // No substeps, we handle it ourselves
                config_.fixedTimeStep
            );

            accumulator_ -= config_.fixedTimeStep;
            steps++;

            // Process collision callbacks
            processCollisionCallbacks();

            // Update statistics
            updateStatistics();
        }

        // Calculate interpolation alpha for rendering
        interpolationAlpha_ = accumulator_ / config_.fixedTimeStep;

        // Interpolate transforms for smooth rendering
        if (steps > 0) {
            interpolateTransforms();
        }

        return steps;
    }

    void PhysicsWorld::debugStep() {
        if (!initialized_) return;

        storePreviousTransforms();
        dynamicsWorld_->stepSimulation(config_.fixedTimeStep, 0, config_.fixedTimeStep);
        processCollisionCallbacks();
        updateStatistics();
    }

    bool PhysicsWorld::raycast(const Vec3& from, const Vec3& to, RaycastHit& hit,
                               const QueryFilter& filter) const {
        const btVector3 btFrom(from.x, from.y, from.z);
        const btVector3 btTo(to.x, to.y, to.z);

        btCollisionWorld::ClosestRayResultCallback callback(btFrom, btTo);
        callback.m_collisionFilterGroup = filter.categoryMask;
        callback.m_collisionFilterMask = filter.groupMask;

        dynamicsWorld_->rayTest(btFrom, btTo, callback);

        if (callback.hasHit()) {
            hit.point = Vec3(callback.m_hitPointWorld.x(),
                             callback.m_hitPointWorld.y(),
                             callback.m_hitPointWorld.z());
            hit.normal = Vec3(callback.m_hitNormalWorld.x(),
                              callback.m_hitNormalWorld.y(),
                              callback.m_hitNormalWorld.z());
            hit.distance = glm::length(hit.point - from);
            hit.fraction = callback.m_closestHitFraction;
            hit.body = static_cast<RigidBody*>(callback.m_collisionObject->getUserPointer());
            return true;
        }

        return false;
    }

    std::vector<RaycastHit> PhysicsWorld::raycastAll(const Vec3& from, const Vec3& to,
                                                     const QueryFilter& filter) const {
        std::vector<RaycastHit> hits;

        const btVector3 btFrom(from.x, from.y, from.z);
        const btVector3 btTo(to.x, to.y, to.z);

        btCollisionWorld::AllHitsRayResultCallback callback(btFrom, btTo);
        callback.m_collisionFilterGroup = filter.categoryMask;
        callback.m_collisionFilterMask = filter.groupMask;

        dynamicsWorld_->rayTest(btFrom, btTo, callback);

        for (int i = 0; i < callback.m_collisionObjects.size(); ++i) {
            RaycastHit hit;
            hit.point = Vec3(callback.m_hitPointWorld[i].x(),
                             callback.m_hitPointWorld[i].y(),
                             callback.m_hitPointWorld[i].z());
            hit.normal = Vec3(callback.m_hitNormalWorld[i].x(),
                              callback.m_hitNormalWorld[i].y(),
                              callback.m_hitNormalWorld[i].z());
            hit.distance = glm::length(hit.point - from);
            hit.fraction = callback.m_hitFractions[i];
            hit.body = static_cast<RigidBody*>(
                callback.m_collisionObjects[i]->getUserPointer());
            hits.push_back(hit);
        }

        return hits;
    }

    std::vector<RigidBody*> PhysicsWorld::overlapSphere(const Vec3& center, Float radius,
                                                        const QueryFilter& filter) const {
        std::vector<RigidBody*> bodies;

        btSphereShape sphere(radius);
        btTransform transform;
        transform.setIdentity();
        transform.setOrigin(btVector3(center.x, center.y, center.z));

        struct Callback : btCollisionWorld::ContactResultCallback {
            std::vector<RigidBody*>* bodies{};
            std::unordered_set<const btCollisionObject*> added;

            btScalar addSingleResult(btManifoldPoint& cp,
                                     const btCollisionObjectWrapper* colObj0Wrap,
                                     int partId0, int index0,
                                     const btCollisionObjectWrapper* colObj1Wrap,
                                     int partId1, int index1) override {
                if (const btCollisionObject* obj = colObj1Wrap->getCollisionObject(); !added.contains(obj)) {
                    added.insert(obj);
                    bodies->push_back(static_cast<RigidBody*>(obj->getUserPointer()));
                }
                return 0;
            }
        };

        Callback callback;
        callback.bodies = &bodies;
        callback.m_collisionFilterGroup = filter.categoryMask;
        callback.m_collisionFilterMask = filter.groupMask;

        btCollisionObject tempObj;
        tempObj.setCollisionShape(&sphere);
        tempObj.setWorldTransform(transform);

        dynamicsWorld_->contactTest(&tempObj, callback);

        return bodies;
    }

    std::vector<RigidBody*> PhysicsWorld::overlapBox(const Vec3& center, const Vec3& halfExtents,
                                                     const Quat& rotation,
                                                     const QueryFilter& filter) {
        std::vector<RigidBody*> bodies;

        btBoxShape box(btVector3(halfExtents.x, halfExtents.y, halfExtents.z));
        btTransform transform;
        transform.setOrigin(btVector3(center.x, center.y, center.z));
        transform.setRotation(btQuaternion(rotation.x, rotation.y, rotation.z, rotation.w));

        // Similar implementation to overlapSphere
        // ... (code similar to above)

        return bodies;
    }

    void PhysicsWorld::setGravity(const Vec3& gravity) {
        config_.gravity = gravity;
        if (dynamicsWorld_) {
            dynamicsWorld_->setGravity(btVector3(gravity.x, gravity.y, gravity.z));
        }
    }

    void PhysicsWorld::setConfig(const PhysicsConfig& config) {
        config_ = config;
        // Apply relevant changes to existing world
        if (dynamicsWorld_) {
            setGravity(config.gravity);
            dynamicsWorld_->getSolverInfo().m_numIterations = config.solverIterations;
            // ... apply other settings
        }
    }

    void PhysicsWorld::enableDebugDraw(const bool enable) {
        config_.enableDebugDraw = enable;
        if (enable && !debugDrawer_) {
            debugDrawer_ = std::make_unique<PhysicsDebugDrawer>();
            dynamicsWorld_->setDebugDrawer(debugDrawer_.get());
        }
    }

    void PhysicsWorld::debugDrawWorld() const {
        if (debugDrawer_ && dynamicsWorld_) {
            dynamicsWorld_->debugDrawWorld();
        }
    }

    btDiscreteDynamicsWorld* PhysicsWorld::getBulletWorld() const {
        return dynamicsWorld_.get();
    }

    // TODO: Ver como mejorar esto
    void PhysicsWorld::createBroadphase() {
        if (config_.broadphaseType == BroadphaseType::DYNAMIC_AABB_TREE) {
            broadphase_ = std::make_unique<btDbvtBroadphase>();
        }
        else if (config_.broadphaseType == BroadphaseType::AXIS_SWEEP_3) {
            btVector3 worldMin(config_.worldMin.x, config_.worldMin.y, config_.worldMin.z);
            btVector3 worldMax(config_.worldMax.x, config_.worldMax.y, config_.worldMax.z);
            broadphase_ = std::make_unique<bt32BitAxisSweep3>(
                worldMin, worldMax, config_.maxProxies);
        }
        else if (config_.broadphaseType == BroadphaseType::SIMPLE) {
            broadphase_ = std::make_unique<btSimpleBroadphase>(config_.maxProxies);
        }
        else {
            broadphase_ = std::make_unique<btDbvtBroadphase>();
        }
    }

    void PhysicsWorld::createSolver() {
        // Add other solver types as needed
        if (SolverType::SEQUENTIAL_IMPULSE == config_.solverType) {
            solver_ = std::make_unique<btSequentialImpulseConstraintSolver>();
        }
        else {
            solver_ = std::make_unique<btSequentialImpulseConstraintSolver>();
        }
    }

    void PhysicsWorld::storePreviousTransforms() {
        for (RigidBody* body : bodies_) {
            if (!body) continue;

            // Get current transform from the rigid body
            btTransform bulletTransform = body->getBulletBody()->getWorldTransform();

            // Convert to our Transform format
            Transform transform;

            // Position
            const btVector3& origin = bulletTransform.getOrigin();
            transform.setPosition(Vec3(origin.x(), origin.y(), origin.z()));

            // Rotation
            const btQuaternion& rotation = bulletTransform.getRotation();
            transform.setRotation(Quat(rotation.w(), rotation.x(), rotation.y(), rotation.z()));

            // Store in previous transforms map
            previousTransforms_[body] = transform;
        }
    }

    void PhysicsWorld::interpolateTransforms() {
        for (RigidBody* body : bodies_) {
            if (!body) continue;

            // Skip interpolation for kinematic and static bodies
            if (body->getType() == BodyType::STATIC || body->getType() == BodyType::KINEMATIC) {
                continue;
            }

            auto it = previousTransforms_.find(body);
            if (it == previousTransforms_.end()) continue;

            const Transform& previousTransform = it->second;

            // Get current transform
            btTransform currentBulletTransform = body->getBulletBody()->getWorldTransform();

            // Convert current transform
            Transform currentTransform;
            const btVector3& currentOrigin = currentBulletTransform.getOrigin();
            const btQuaternion& currentRotation = currentBulletTransform.getRotation();

            currentTransform.setPosition(Vec3(currentOrigin.x(), currentOrigin.y(), currentOrigin.z()));
            currentTransform.setRotation(Quat(currentRotation.w(), currentRotation.x(),
                                              currentRotation.y(), currentRotation.z()));

            // Interpolate between previous and current
            Vec3 interpolatedPosition = lerp(previousTransform.getPosition(),
                                             currentTransform.getPosition(),
                                             interpolationAlpha_);

            Quat interpolatedRotation = slerp(previousTransform.getRotation(),
                                              currentTransform.getRotation(),
                                              interpolationAlpha_);

            // Set interpolated transform for rendering
            Transform interpolatedTransform;
            interpolatedTransform.setPosition(interpolatedPosition);
            interpolatedTransform.setRotation(interpolatedRotation);

            // Update body's render transform (not physics transform)
            interpolatedTransforms_[body] = interpolatedTransform;
        }
    }

    void PhysicsWorld::updateStatistics() {
        // Reset counters
        stats_.numBodies = bodies_.size();
        stats_.numActiveBodies = 0;
        stats_.numContacts = 0;

        // Count active bodies and gather velocity statistics
        Float totalKineticEnergy = 0.0f;
        for (RigidBody* body : bodies_) {
            if (!body) continue;

            // Check if body is active (awake)
            if (body->getBulletBody()->isActive()) {
                stats_.numActiveBodies++;

                // Calculate kinetic energy for performance metrics
                if (body->getType() == BodyType::DYNAMIC) {
                    Vec3 linearVel = body->getLinearVelocity();
                    Vec3 angularVel = body->getAngularVelocity();
                    Float mass = body->getMass();

                    Float linearKE = 0.5f * mass * glm::dot(linearVel, linearVel);
                    Float angularKE = 0.5f * glm::dot(angularVel, angularVel); // Simplified
                    totalKineticEnergy += linearKE + angularKE;
                }
            }
        }

        // Manifold and contact statistics
        stats_.numManifolds = dispatcher_->getNumManifolds();

        Int totalContacts = 0;
        for (int i = 0; i < stats_.numManifolds; ++i) {
            btPersistentManifold* manifold = dispatcher_->getManifoldByIndexInternal(i);
            totalContacts += manifold->getNumContacts();
        }
        stats_.numContacts = totalContacts;

        // Broadphase statistics
        if (broadphase_->getOverlappingPairCache()) {
            stats_.broadphasePairs = broadphase_->getOverlappingPairCache()->getNumOverlappingPairs();
            stats_.broadphaseProxies = broadphase_->getOverlappingPairCache()->getNumOverlappingPairs() * 2;
        }

        // Constraint statistics
        stats_.numConstraints = constraintPool_.size();

        // Simulation islands (for multithreading performance)
        if (dynamicsWorld_->getSimulationIslandManager()) {
            stats_.numIslands = dynamicsWorld_->getSimulationIslandManager()->getUnionFind().getNumElements();
        }

        // Memory usage estimation
        stats_.memoryUsage = sizeof(PhysicsWorld) +
            (bodies_.size() * sizeof(RigidBody*)) +
            (shapePool_.size() * sizeof(std::unique_ptr<CollisionShape>)) +
            (constraintPool_.size() * sizeof(std::unique_ptr<btTypedConstraint>));

        // Update peak usage
        if (stats_.memoryUsage > stats_.peakMemoryUsage) {
            stats_.peakMemoryUsage = stats_.memoryUsage;
        }

        // Performance metrics
        stats_.kineticEnergy = totalKineticEnergy;
        stats_.averageVelocity = stats_.numActiveBodies > 0
                                     ? std::sqrt(totalKineticEnergy / stats_.numActiveBodies)
                                     : 0.0f;
    }

    void PhysicsWorld::clearAllBodies() {
        // Remove all bodies from Bullet world first
        for (RigidBody* body : bodies_) {
            if (!body) continue;

            btRigidBody* bulletBody = body->getBulletBody();
            if (bulletBody && dynamicsWorld_) {
                // Remove from dynamics world
                dynamicsWorld_->removeRigidBody(bulletBody);

                // Clear any collision callbacks
                bulletBody->setUserPointer(nullptr);

                // Remove any constraints attached to this body
                for (auto it = constraintPool_.begin(); it != constraintPool_.end();) {
                    btTypedConstraint* constraint = it->get();
                    if (constraint &&
                        (&constraint->getRigidBodyA() == bulletBody ||
                            &constraint->getRigidBodyB() == bulletBody)) {
                        dynamicsWorld_->removeConstraint(constraint);
                        it = constraintPool_.erase(it);
                    }
                    else {
                        ++it;
                    }
                }
            }

            // Deallocate the RigidBody using memory manager
            memoryManager_.deallocateObject(body, MemoryCategory::PHYSICS);
        }

        // Clear collections
        bodies_.clear();
        bodyMap_.clear();
        previousTransforms_.clear();
        collisionPairs_.clear();
        previousCollisionPairs_.clear();

        // Reset statistics
        stats_.numBodies = 0;
        stats_.numActiveBodies = 0;
        stats_.numContacts = 0;
        stats_.numManifolds = 0;
    }
} // namespace engine::physics
