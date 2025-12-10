/**
 * @file PhysicsManager.cpp
 * @brief Central physics system manager and coordinator
 * @details Manages physics worlds, object pools, LOD, streaming, and provides
 *          the main interface for game systems to interact with physics
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#include "PhysicsManager.h"

#include "../collision/CollisionDetection.h"
#include "../collision/CollisionFiltering.h"
#include "../collision/CollisionShapes.h"
#include "../core/PhysicsConfig.h"
#include "../core/PhysicsWorld.h"

namespace engine::physics {
    PhysicsManager::PhysicsManager(MemoryManager& memoryManager) : memoryManager_(memoryManager) {}

    PhysicsManager::~PhysicsManager() {
        shutdown();
    }

    bool PhysicsManager::initialize(const PhysicsConfig& config) {
        if (initialized_) {
            return false;
        }

        config_ = config;
        startTime_ = std::chrono::steady_clock::now();

        // Create main physics world
        mainWorld_ = std::make_unique<PhysicsWorld>(memoryManager_, config);
        if (!mainWorld_) {
            return false;
        }

        if (!initializeCollisionDetection()) {
            std::cerr << "Failed to initialize collision detection" << std::endl;
            shutdown(); // Clean up partially initialized systems
            return false;
        }

        // Initialize memory pools using MemoryManager
        initializePools();

        // Initialize subsystems using MemoryManager
        lodSystem_ = memoryManager_.allocateObject<PhysicsLODSystem>(
            MemoryCategory::PHYSICS
        );

        profiler_ = memoryManager_.allocateObject<PhysicsProfiler>(
            MemoryCategory::PHYSICS
        );

        if (!lodSystem_ || !profiler_) {
            std::cerr << "[PhysicsManager] Failed to initialize subsystems\n";
            return false;
        }

        // Start worker threads if multithreading enabled
        if (config.enableMultithreading) {
            startWorkerThreads(config.numThreads);
        }

        initialized_ = true;
        startTime_ = std::chrono::steady_clock::now();

        std::cout << "[PhysicsManager] Initialized successfully\n";
        return true;
    }

    void PhysicsManager::shutdown() {
        if (!initialized_)
            return;

        std::cout << "[PhysicsManager] Starting shutdown sequence..." << std::endl;

        // Stop worker threads
        std::cout << "[PhysicsManager] Stopping worker threads..." << std::endl;
        stopWorkerThreads();

        // Clear all physics objects
        std::cout << "[PhysicsManager] Clearing physics objects..." << std::endl;
        clearAllBodies();
        clearAllConstraints();
        clearAllShapes();

        // Shutdown subsystems using MemoryManager
        std::cout << "[PhysicsManager] Shutting down subsystems..." << std::endl;
        if (lodSystem_) {
            memoryManager_.deallocateObject(lodSystem_, MemoryCategory::PHYSICS);
            lodSystem_ = nullptr;
        }

        if (profiler_) {
            memoryManager_.deallocateObject(profiler_, MemoryCategory::PHYSICS);
            profiler_ = nullptr;
        }

        // Shutdown worlds
        std::cout << "[PhysicsManager] Shutting down main world..." << std::endl;
        if (mainWorld_) {
            mainWorld_->shutdown();
            mainWorld_.reset();
        }

        std::cout << "[PhysicsManager] Shutting down async worlds..." << std::endl;
        for (auto& world : asyncWorlds_) {
            if (world) {
                world->shutdown();
            }
        }
        asyncWorlds_.clear();

        // ========================================================================
        // Shutdown collision detection BEFORE world
        // ========================================================================
        // CRITICAL ORDER: CollisionDetection must be shut down before PhysicsWorld
        // because it may hold references to world's collision objects
        std::cout << "[PhysicsManager] Shutting down collision detection..." << std::endl;
        shutdownCollisionDetection();

        // Clear pools LAST
        std::cout << "[PhysicsManager] Clearing pools..." << std::endl;
        clearPools();

        initialized_ = false;
        std::cout << "[PhysicsManager] Shutdown complete\n";
    }

    void PhysicsManager::update(const Float deltaTime, const Vec3& viewerPosition) {
        if (!initialized_)
            return;

        const auto frameStart = std::chrono::high_resolution_clock::now();

        // Update LOD system
        if (lodSystem_) {
            lodSystem_->update(viewerPosition, deltaTime);
        }

        // Pre-physics callbacks
        for (auto& callback : prePhysicsCallbacks_) {
            callback(deltaTime);
        }

        // Step main physics world
        const Int steps = mainWorld_->stepSimulation(deltaTime);

        // Process async tasks
        processAsyncTasks();

        // Post-physics callbacks
        for (auto& callback : postPhysicsCallbacks_) {
            callback(deltaTime);
        }

        // Update profiler
        if (profiler_) {
            const auto frameEnd = std::chrono::high_resolution_clock::now();
            const Float frameTime = std::chrono::duration<Float, std::milli>(
                frameEnd - frameStart
            ).count();
            profiler_->recordFrame(frameTime, steps);
        }
    }

    void PhysicsManager::fixedUpdate() const {
        if (!initialized_)
            return;

        mainWorld_->stepSimulation(config_.fixedTimeStep);
    }

    RigidBody* PhysicsManager::createRigidBody(const BodyCreationParams& params) {
        if (!initialized_)
            return nullptr;

        // Create collision shape directly (no factory)
        CollisionShape* shape = getOrCreateShape(params.shape);
        if (!shape) {
            std::cerr << "[PhysicsManager] Failed to create collision shape\n";
            return nullptr;
        }

        // Allocate body from pool first, then from MemoryManager if pool empty
        RigidBody* body = bodyPool_.allocate();
        if (!body) {
            body = memoryManager_.allocateObject<RigidBody>(MemoryCategory::PHYSICS);
        }

        if (!body) {
            std::cerr << "[PhysicsManager] Failed to allocate RigidBody\n";
            destroyCollisionShapeInternal(shape); // Clean up shape
            return nullptr;
        }

        // Initialize body
        if (!body->initialize(params, shape, mainWorld_.get())) {
            std::cerr << "[PhysicsManager] Failed to initialize RigidBody\n";

            // Clean up
            if (!bodyPool_.deallocate(body)) {
                memoryManager_.deallocateObject(body, MemoryCategory::PHYSICS);
            }
            destroyCollisionShapeInternal(shape);
            return nullptr;
        }

        // Register body in collections
        bodies_.insert(body);
        if (params.id > 0) {
            bodyIdMap_[params.id] = body;
        }
        if (!params.name.empty()) {
            bodyNameMap_[params.name] = body;
        }

        // Add to physics world
        mainWorld_->getBulletWorld()->addRigidBody(
            body->getBulletBody(),
            params.collisionGroup,
            params.collisionMask
        );

        // Register with LOD system
        if (lodSystem_) {
            lodSystem_->registerBody(body, params.importance);
        }

        return body;
    }

    void PhysicsManager::destroyRigidBody(RigidBody* body) {
        if (!body || !initialized_)
            return;

        // Unregister from LOD system
        if (lodSystem_) {
            lodSystem_->unregisterBody(body);
        }

        // Remove from world
        mainWorld_->getBulletWorld()->removeRigidBody(body->getBulletBody());

        // Get shape before cleanup
        CollisionShape* shape = body->getShape();

        // Unregister from collections
        bodies_.erase(body);

        // Remove from maps
        for (auto it = bodyIdMap_.begin(); it != bodyIdMap_.end();) {
            if (it->second == body) {
                it = bodyIdMap_.erase(it);
            } else {
                ++it;
            }
        }

        for (auto it = bodyNameMap_.begin(); it != bodyNameMap_.end();) {
            if (it->second == body) {
                it = bodyNameMap_.erase(it);
            } else {
                ++it;
            }
        }

        // Cleanup and deallocate body
        body->cleanup();
        if (!bodyPool_.deallocate(body)) {
            memoryManager_.deallocateObject(body, MemoryCategory::PHYSICS);
        }

        // Cleanup shape if not shared
        if (shape) {
            destroyCollisionShapeInternal(shape);
        }
    }

    btTypedConstraint* PhysicsManager::createConstraint(const ConstraintCreationParams& params) {
        if (!initialized_)
            return nullptr;

        btTypedConstraint* constraint = nullptr;

        switch (params.type) {
            case ConstraintType::POINT_TO_POINT: {
                if (params.bodyB) {
                    constraint = new btPoint2PointConstraint(
                        *params.bodyA->getBulletBody(),
                        *params.bodyB->getBulletBody(),
                        btVector3(
                            params.frameInA.getPosition().x,
                            params.frameInA.getPosition().y,
                            params.frameInA.getPosition().z
                        ),
                        btVector3(
                            params.frameInB.getPosition().x,
                            params.frameInB.getPosition().y,
                            params.frameInB.getPosition().z
                        )
                    );
                } else {
                    constraint = new btPoint2PointConstraint(
                        *params.bodyA->getBulletBody(),
                        btVector3(
                            params.frameInA.getPosition().x,
                            params.frameInA.getPosition().y,
                            params.frameInA.getPosition().z
                        )
                    );
                }
                break;
            }

            case ConstraintType::HINGE: {
                // Create hinge constraint
                break;
            }

            case ConstraintType::GENERIC_6DOF: {
                const btTransform frameA;
                // Convert transforms

                btGeneric6DofConstraint* dof = nullptr;
                if (params.bodyB) {
                    const btTransform frameB;
                    dof = new btGeneric6DofConstraint(
                        *params.bodyA->getBulletBody(),
                        *params.bodyB->getBulletBody(),
                        frameA,
                        frameB,
                        true
                    );
                } else {
                    dof = new btGeneric6DofConstraint(
                        *params.bodyA->getBulletBody(),
                        frameA,
                        true
                    );
                }

                // Set limits
                dof->setLinearLowerLimit(
                    btVector3(
                        params.linearLowerLimit.x,
                        params.linearLowerLimit.y,
                        params.linearLowerLimit.z
                    )
                );
                dof->setLinearUpperLimit(
                    btVector3(
                        params.linearUpperLimit.x,
                        params.linearUpperLimit.y,
                        params.linearUpperLimit.z
                    )
                );

                constraint = dof;
                break;
            }
            default:
                std::cerr << "[PhysicsManager] Unsupported constraint type\n";
                break;

                // Add other constraint types...
        }

        if (constraint) {
            constraint->setBreakingImpulseThreshold(params.breakingImpulse);

            mainWorld_->getBulletWorld()->addConstraint(
                constraint,
                params.disableCollisionsBetweenLinkedBodies
            );

            constraints_.push_back(constraint);
        }

        return constraint;
    }

    void PhysicsManager::destroyConstraint(btTypedConstraint* constraint) {
        if (!constraint)
            return;

        mainWorld_->getBulletWorld()->removeConstraint(constraint);

        if (const auto it = std::ranges::find(constraints_, constraint); it != constraints_.end()) {
            constraints_.erase(it);
            delete constraint;
        }
    }

    bool PhysicsManager::raycast(
        const Vec3& from,
        const Vec3& to,
        RaycastHit& hit,
        const QueryFilter& filter
    ) const {
        return mainWorld_->raycast(from, to, hit, filter);
    }

    std::vector<RaycastHit> PhysicsManager::raycastAll(
        const Vec3& from,
        const Vec3& to,
        const QueryFilter& filter
    ) const {
        return mainWorld_->raycastAll(from, to, filter);
    }

    std::vector<RigidBody*> PhysicsManager::overlapSphere(
        const Vec3& center,
        const Float radius,
        const QueryFilter& filter
    ) const {
        return mainWorld_->overlapSphere(center, radius, filter);
    }

    std::vector<RigidBody*> PhysicsManager::overlapBox(
        const Vec3& center,
        const Vec3& halfExtents,
        const Quat& rotation,
        const QueryFilter& filter
    ) const {
        return mainWorld_->overlapBox(center, halfExtents, rotation, filter);
    }

    CharacterController* PhysicsManager::createCharacterController(const CharacterControllerParams& params) {
        const auto controller = memoryManager_.allocateObject<CharacterController>(
            MemoryCategory::PHYSICS,
            params,
            mainWorld_.get()
        );

        if (controller) {
            characterControllers_.push_back(controller);
        }

        return controller;
    }

    void PhysicsManager::destroyCharacterController(CharacterController* controller) {
        if (!controller)
            return;

        if (const auto it = std::ranges::find(characterControllers_, controller); it != characterControllers_.
            end()) {
            characterControllers_.erase(it);
            memoryManager_.deallocateObject(controller, MemoryCategory::PHYSICS);
        }
    }

    VehiclePhysics* PhysicsManager::createVehicle(const VehicleParams& params) {
        const auto vehicle = memoryManager_.allocateObject<VehiclePhysics>(
            MemoryCategory::PHYSICS,
            params,
            mainWorld_.get()
        );

        if (vehicle) {
            vehicles_.push_back(vehicle);

            BodyCreationParams bodyParams;
            bodyParams.shape = ShapeCreationParams::Box(params.chassisSize * 0.5f);
            bodyParams.mass = params.chassisMass;
            bodyParams.type = BodyType::DYNAMIC;
            bodyParams.material.friction = 0.5f;
            bodyParams.material.restitution = 0.1f;
            bodyParams.centerOfMass = params.centerOfMass;

            vehicle->setChassisBody(createRigidBody(bodyParams));

            vehicle->initialize();
        }

        return vehicle;
    }

    void PhysicsManager::destroyVehicle(VehiclePhysics* vehicle) {
        if (!vehicle)
            return;

        if (const auto it = std::ranges::find(vehicles_, vehicle); it != vehicles_.end()) {
            vehicles_.erase(it);
            memoryManager_.deallocateObject(vehicle, MemoryCategory::PHYSICS);
        }
    }

    PhysicsWorld* PhysicsManager::createAsyncWorld(const PhysicsConfig& config) {
        auto world = std::make_unique<PhysicsWorld>(memoryManager_, config);
        asyncWorlds_.push_back(std::move(world));

        return asyncWorlds_.back().get();
    }

    void PhysicsManager::setGravity(const Vec3& gravity) {
        config_.gravity = gravity;
        if (mainWorld_) {
            mainWorld_->setGravity(gravity);
        }
    }

    CollisionFilter* PhysicsManager::getCollisionFilter() const {
        if (!collisionDetection_) {
            return nullptr;
        }
        return collisionDetection_->getFilter();
    }

    bool PhysicsManager::setupRPGCollisionFiltering() const {
        CollisionFilter* filter = getCollisionFilter();
        if (!filter) {
            return false;
        }

        // Asumiendo que tienes este método en CollisionFilter
        filter->setupRPGFiltering();
        return true;
    }

    const PhysicsStats& PhysicsManager::getStatistics() const {
        return mainWorld_ ? mainWorld_->getStatistics() : emptyStats_;
    }

    void PhysicsManager::setBroadPhaseType(BroadphaseType type) const {
        if (collisionDetection_) {
            collisionDetection_->setBroadPhaseType(type);
        }
    }

    void PhysicsManager::setContinuousCollisionDetection(const bool enable) const {
        if (collisionDetection_) {
            collisionDetection_->setEnableCCD(enable);
        }
    }

    std::string PhysicsManager::generatePerformanceReport() const {
        std::stringstream ss;

        ss << "=== Physics Performance Report ===\n";
        ss << "Uptime: " << getUptime() << " seconds\n";
        ss << "Bodies: " << bodies_.size() << "\n";
        ss << "Shapes: " << shapes_.size() << "\n";
        ss << "Constraints: " << constraints_.size() << "\n";

        if (mainWorld_) {
            const auto& stats = mainWorld_->getStatistics();
            ss << "\nWorld Statistics:\n";
            ss << "  Active Bodies: " << stats.numActiveBodies << "/" << stats.numBodies << "\n";
            ss << "  Contacts: " << stats.numContacts << "\n";
            ss << "  Manifolds: " << stats.numManifolds << "\n";
            ss << "  Broadphase Pairs: " << stats.broadphasePairs << "\n";
            ss << "\nTiming (ms):\n";
            ss << "  Step Time: " << stats.stepTime << "\n";
            ss << "  Collision: " << stats.collisionTime << "\n";
            ss << "  Solver: " << stats.solverTime << "\n";
            ss << "  Integration: " << stats.integrationTime << "\n";
        }

        if (profiler_) {
            ss << "\n" << profiler_->getReport();
        }

        ss << "\nMemory Usage:\n";
        ss << "  Body Pool: " << bodyPool_.getUsedCount() << "/"
                << bodyPool_.getCapacity() << "\n";
        ss << "  Shape Pool: " << shapePool_.getUsedCount() << "/"
                << shapePool_.getCapacity() << "\n";

        return ss.str();
    }

    void PhysicsManager::enableDebugDraw(const bool enable) const {
        if (mainWorld_) {
            mainWorld_->enableDebugDraw(enable);
        }
    }

    void PhysicsManager::debugDrawWorld() const {
        if (mainWorld_) {
            mainWorld_->debugDrawWorld();
        }
    }

    void PhysicsManager::initializePools() {
        bodyPool_.initialize(config_.initialBodyPoolSize, &memoryManager_);
        shapePool_.initialize(config_.initialShapePoolSize, &memoryManager_);

        std::cout << "[PhysicsManager] Initialized pools - Bodies: "
                << config_.initialBodyPoolSize
                << ", Shapes: " << config_.initialShapePoolSize << std::endl;
    }

    void PhysicsManager::clearPools() {
        bodyPool_.clear();
        shapePool_.clear();
        std::cout << "[PhysicsManager] Cleared object pools\n";
    }

    CollisionShape* PhysicsManager::getOrCreateShape(const ShapeCreationParams& params) {
        // For now, always create new shape
        // In the future, you could implement shape sharing/caching here
        return createCollisionShapeInternal(params);
    }

    CollisionShape* PhysicsManager::createCollisionShapeInternal(const ShapeCreationParams& params) {
        CollisionShape* shape = nullptr;

        switch (params.type) {
            case ShapeType::BOX:
                // FIXED: params.halfExtents -> params.box.halfExtents
                shape = memoryManager_.allocateObject<BoxShape>(
                    MemoryCategory::PHYSICS,
                    params.box.halfExtents
                );
                break;

            case ShapeType::SPHERE:
                // FIXED: params.radius -> params.sphere.radius
                shape = memoryManager_.allocateObject<SphereShape>(
                    MemoryCategory::PHYSICS,
                    params.sphere.radius
                );
                break;

            case ShapeType::CAPSULE:
                // FIXED: params.radius/height -> params.capsule.radius/height
                shape = memoryManager_.allocateObject<CapsuleShape>(
                    MemoryCategory::PHYSICS,
                    params.capsule.radius,
                    params.capsule.height
                );
                break;

            case ShapeType::CYLINDER:
                // FIXED: params.halfExtents -> params.cylinder.halfExtents
                shape = memoryManager_.allocateObject<CylinderShape>(
                    MemoryCategory::PHYSICS,
                    params.cylinder.halfExtents
                );
                break;

            case ShapeType::CONE:
                // FIXED: params.radius/height -> params.cone.radius/height
                shape = memoryManager_.allocateObject<ConeShape>(
                    MemoryCategory::PHYSICS,
                    params.cone.radius,
                    params.cone.height
                );
                break;

            case ShapeType::CONVEX_HULL:
                // FIXED: params.vertices/vertexCount/vertexStride -> params.convexHull.*
                if (params.convexHull.vertices && params.convexHull.vertexCount > 0) {
                    shape = memoryManager_.allocateObject<ConvexHullShape>(
                        MemoryCategory::PHYSICS,
                        params.convexHull.vertices,
                        params.convexHull.vertexCount,
                        params.convexHull.vertexStride
                    );
                }
                break;

            case ShapeType::TRIANGLE_MESH:
                // FIXED: params.vertices/indices/counts -> params.triangleMesh.*
                if (params.triangleMesh.vertices && params.triangleMesh.indices &&
                    params.triangleMesh.vertexCount > 0 && params.triangleMesh.indexCount > 0) {
                    shape = memoryManager_.allocateObject<TriangleMeshShape>(
                        MemoryCategory::PHYSICS,
                        params.triangleMesh.vertices,
                        params.triangleMesh.vertexCount,
                        params.triangleMesh.indices,
                        params.triangleMesh.indexCount,
                        params.triangleMesh.vertexStride
                    );
                }
                break;

            case ShapeType::HEIGHTFIELD:
                // FIXED: params.heightfieldData/Width/Height -> params.heightfield.*
                if (params.heightfield.heightData &&
                    params.heightfield.width > 0 &&
                    params.heightfield.height > 0) {
                    shape = memoryManager_.allocateObject<HeightfieldShape>(
                        MemoryCategory::PHYSICS,
                        params.heightfield.heightData,
                        params.heightfield.width,
                        params.heightfield.height,
                        params.heightfield.minHeight,
                        params.heightfield.maxHeight,
                        params.heightfield.scale
                    );
                }
                break;

            case ShapeType::COMPOUND:
                shape = createCompoundShapeInternal(params);
                break;

            case ShapeType::PLANE:
                // FIXED: hardcoded values -> params.plane.normal/distance
                shape = memoryManager_.allocateObject<PlaneShape>(
                    MemoryCategory::PHYSICS,
                    params.plane.normal,
                    params.plane.distance
                );
                break;

            default:
                std::cerr << "[PhysicsManager] Unknown shape type: "
                        << static_cast<int>(params.type) << std::endl;
                return nullptr;
        }

        if (shape) {
            shape->setMargin(params.margin);
            shapes_.insert(shape);
        }

        return shape;
    }

    void PhysicsManager::destroyCollisionShapeInternal(CollisionShape* shape) {
        if (!shape)
            return;

        std::cout << "[PhysicsManager] Destroying shape at: " << (shape)
                << " (type: " << static_cast<int>(shape->getType())
                << ", refCount: " << shape->getRefCount() << ")" << std::endl;

        shapes_.erase(shape);

        const int newRefCount = shape->decrementRefCount();

        std::cout << "[PhysicsManager] After decrement, refCount = " << newRefCount << std::endl;

        if (newRefCount > 0) {
            std::cout << "[PhysicsManager] Shape still has " << newRefCount
                    << " references, not deallocating yet" << std::endl;
            return;
        }

        std::cout << "[PhysicsManager] RefCount is 0, deallocating shape..." << std::endl;

        memoryManager_.deallocateObject(shape, MemoryCategory::PHYSICS);

        std::cout << "[PhysicsManager] Shape destroyed successfully" << std::endl;
    }

    CompoundShape* PhysicsManager::createCompoundShapeInternal(const ShapeCreationParams& params) {
        auto* compound = memoryManager_.allocateObject<CompoundShape>(
            MemoryCategory::PHYSICS,
            true // Enable dynamic AABB tree
        );

        if (!compound) {
            return nullptr;
        }

        // FIXED: params.childShapes/childTransforms -> params.compound.*
        for (std::size_t i = 0; i < params.compound.childShapes.size(); ++i) {
            if (CollisionShape* childShape = createCollisionShapeInternal(params.compound.childShapes[i]); childShape && i < params.compound.childTransforms.size()) {
                compound->addChildShape(params.compound.childTransforms[i], childShape);
            }
        }

        return compound;
    }

    void PhysicsManager::clearAllBodies() {
        // Copy the set to avoid iterator invalidation
        for (const auto bodiesCopy = bodies_; RigidBody* body : bodiesCopy) {
            destroyRigidBody(body);
        }

        bodies_.clear();
        bodyIdMap_.clear();
        bodyNameMap_.clear();
    }

    void PhysicsManager::clearAllConstraints() {
        for (auto* constraint : constraints_) {
            if (mainWorld_) {
                mainWorld_->getBulletWorld()->removeConstraint(constraint);
            }
            delete constraint;
        }
        constraints_.clear();
    }

    void PhysicsManager::clearAllShapes() {
        // Copy the set to avoid iterator invalidation
        for (const auto shapesCopy = shapes_; CollisionShape* shape : shapesCopy) {
            destroyCollisionShapeInternal(shape);
        }

        shapes_.clear();
    }

    void PhysicsManager::startWorkerThreads(const int numThreads) {
        stopWorkers_ = false;

        for (int i = 0; i < numThreads; ++i) {
            workerThreads_.emplace_back(
                [this]() {
                    while (!stopWorkers_) {
                        std::unique_lock lock(taskMutex_);
                        taskCondition_.wait(
                            lock,
                            [this]() {
                                return !asyncTasks_.empty() || stopWorkers_;
                            }
                        );

                        if (!asyncTasks_.empty()) {
                            auto task = asyncTasks_.front();
                            asyncTasks_.pop();
                            lock.unlock();

                            task();
                        }
                    }
                }
            );
        }
    }

    void PhysicsManager::stopWorkerThreads() {
        stopWorkers_ = true;
        taskCondition_.notify_all();

        for (auto& thread : workerThreads_) {
            if (thread.joinable()) {
                thread.join();
            }
        }

        workerThreads_.clear();
    }

    Float PhysicsManager::getUptime() const {
        const auto now = std::chrono::steady_clock::now();

        return std::chrono::duration<Float>(now - startTime_).count();
    }

    bool PhysicsManager::initializeCollisionDetection() {
        // ========================================================================
        // Create collision detection instance
        // ========================================================================
        // Using std::make_unique for automatic memory management
        // CollisionDetection constructor will initialize its internal components
        // (broad phase, narrow phase, and collision filter)
        collisionDetection_ = std::make_unique<CollisionDetection>();

        if (!collisionDetection_) {
            std::cout << "Failed to create CollisionDetection instance" << std::endl;
            return false;
        }

        // ========================================================================
        // Initialize collision detection subsystems
        // ========================================================================
        // This calls CollisionDetection::initialize() which sets up:
        // - Dynamic AABB tree broad phase (default)
        // - Narrow phase collision algorithms
        // - Collision filtering system
        // - Collision pair caching structures
        collisionDetection_->initialize();

        // ========================================================================
        // Configure collision detection based on PhysicsConfig
        // ========================================================================

        // Set broad phase algorithm type
        // Dynamic AABB Tree: O(log n) insertion/query, excellent for moving objects
        // Sweep and Prune: O(n) update, better for mostly static scenes
        if (config_.broadphaseType == BroadphaseType::DYNAMIC_AABB_TREE) {
            collisionDetection_->setBroadPhaseType(BroadphaseType::DYNAMIC_AABB_TREE);
        } else if (config_.broadphaseType == BroadphaseType::AXIS_SWEEP_3) {
            collisionDetection_->setBroadPhaseType(BroadphaseType::AXIS_SWEEP_3);
        }

        // ========================================================================
        // Enable/Disable collision detection features
        // ========================================================================

        // Temporal coherence caching: Reuse collision data from previous frame
        // Benefits: ~20-30% faster narrow phase for stable contacts
        // Cost: Extra memory for storing previous frame's collision pairs
        // collisionDetection_->setEnableCaching(config_.enableCollisionCaching);

        // Continuous Collision Detection: Prevents tunneling for fast objects
        // Uses swept sphere tests and conservative advancement
        // Critical for: Projectiles, fast-moving characters, vehicles
        collisionDetection_->setEnableCCD(config_.enableCCD);

        // Sleeping optimization: Skip inactive (non-moving) objects
        // Objects that haven't moved significantly are marked as sleeping
        // Sleeping objects are not tested for collisions with other sleeping objects
        collisionDetection_->setEnableSleeping(config_.enableSleeping);

        // ========================================================================
        // Integrate with Bullet's collision system
        // ========================================================================
        // Your CollisionDetection wraps Bullet but provides:
        // 1. Custom filtering logic (layer-based, group-based, callback-based)
        // 2. Event callbacks (onCollisionEnter, onCollisionExit, onCollisionStay)
        // 3. Advanced queries (swept tests, compound shape handling)
        // 4. Performance monitoring and statistics

        // Get collision filter for configuration
        CollisionFilter* filter = collisionDetection_->getFilter();
        if (!filter) {
            std::cout << "CollisionDetection filter is null" << std::endl;
            return false;
        }

        // ========================================================================
        // Setup default collision layers for RPG games
        // ========================================================================
        // Collision layers define "what can collide with what"
        // This is set up later in setupRPGCollisionFiltering()

        std::cout << "CollisionDetection initialized successfully" << std::endl;
        return true;
    }

    void PhysicsManager::shutdownCollisionDetection() {
        if (!collisionDetection_) {
            return; // Already shut down or never initialized
        }

        // ========================================================================
        // Clean up collision detection resources
        // ========================================================================
        // CollisionDetection::shutdown() will:
        // - Clear all cached collision pairs
        // - Release broad phase memory (AABB tree nodes, etc.)
        // - Release narrow phase algorithm resources
        // - Clear collision filter configuration
        collisionDetection_->shutdown();

        // ========================================================================
        // Release collision detection instance
        // ========================================================================
        // std::unique_ptr automatically calls destructor when reset
        collisionDetection_.reset();

        std::cout << "CollisionDetection shut down" << std::endl;
    }
} // namespace engine::physics
