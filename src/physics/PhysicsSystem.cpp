/**
 * @file PhysicsSystem.cpp
 * @brief Main physics system implementation
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#include "PhysicsSystem.h"
#include <iostream>
#include <BulletSoftBody/btSoftBody.h>

namespace engine::physics {
    // ============================================================================
    // Constructor & Destructor
    // ============================================================================

    PhysicsSystem::PhysicsSystem(MemoryManager& memoryManager)
        : memoryManager_(memoryManager) {
        // Initialize profiler
        m_profiler.setEnabled(true);
    }

    PhysicsSystem::~PhysicsSystem() {
        shutdown();
    }

    // ============================================================================
    // System Lifecycle
    // ============================================================================

    bool PhysicsSystem::initialize(const PhysicsConfig& config) {
        if (m_initialized) {
            logWarning("Physics system already initialized");
            return false;
        }

        auto startTime = std::chrono::high_resolution_clock::now();

        // Verify memory manager is initialized
        if (!memoryManager_.isInitialized()) {
            logError("MemoryManager not initialized - cannot initialize physics system");
            return false;
        }

        // Initialize physics manager with our memory manager reference
        if (!initializePhysicsManager(config)) {
            logError("Failed to initialize PhysicsManager");
            return false;
        }

        // Get main physics world from manager
        m_mainWorld = m_physicsManager->getMainWorld();
        if (!m_mainWorld) {
            logError("Failed to get main physics world");
            return false;
        }

        // Initialize all subsystems
        if (!initializeSubsystems(config)) {
            logError("Failed to initialize physics subsystems");
            cleanupSubsystems();
            return false;
        }

        // Setup default collision filtering
        setupDefaultCollisionFiltering();

        // Register system callbacks
        registerCallbacks();

        auto endTime = std::chrono::high_resolution_clock::now();
        m_initializationTime = std::chrono::duration<Float, std::milli>(endTime - startTime).count();

        m_initialized = true;
        m_config = config;

        logInfo("Physics system initialized in " + std::to_string(m_initializationTime) + "ms");

        return true;
    }

    void PhysicsSystem::shutdown() {
        if (!m_initialized)
            return;

        logInfo("Shutting down physics system...");

        // Cleanup subsystems in reverse order of initialization
        std::cout << "[PhysicsSystem] Cleaning up subsystems..." << std::endl;
        cleanupSubsystems();
        std::cout << "[PhysicsSystem] Subsystems cleaned" << std::endl;

        // Shutdown physics manager
        if (m_physicsManager) {
            std::cout << "[PhysicsSystem] Calling PhysicsManager::shutdown()..." << std::endl;
            m_physicsManager->shutdown();
            std::cout << "[PhysicsSystem] PhysicsManager::shutdown() completed" << std::endl;

            std::cout << "[PhysicsSystem] Deallocating PhysicsManager at address: "
                      << m_physicsManager << std::endl;

            // Deallocate physics manager using our memory manager
            memoryManager_.deallocateObject(m_physicsManager, MemoryCategory::PHYSICS);
            std::cout << "[PhysicsSystem] PhysicsManager deallocated successfully" << std::endl;

            m_physicsManager = nullptr;
        }

        m_mainWorld = nullptr;
        m_initialized = false;

        logInfo("Physics system shut down successfully");
    }

    void PhysicsSystem::update(Float deltaTime) {
        if (!m_initialized)
            return;

        m_profiler.beginFrame();

        // Pre-physics update - prepare for simulation
        m_profiler.startTimer("PrePhysics");
        updatePrePhysics(deltaTime);
        m_profiler.endTimer("PrePhysics");

        // Main physics simulation step
        m_profiler.startTimer("Simulation");
        if (m_physicsManager) {
            // Use explicit parameter to resolve ambiguity
            m_physicsManager->update(deltaTime, VEC3_ZERO); // Use version with viewer position
        }
        m_profiler.endTimer("Simulation");

        // Post-physics update - process results
        m_profiler.startTimer("PostPhysics");
        updatePostPhysics(deltaTime);
        m_profiler.endTimer("PostPhysics");

        m_profiler.endFrame();

        // Update statistics and performance metrics
        updateStatistics(deltaTime);
    }

    void PhysicsSystem::fixedUpdate() {
        if (!m_initialized || !m_physicsManager)
            return;

        m_physicsManager->fixedUpdate();
    }

    // ============================================================================
    // Body Management
    // ============================================================================

    RigidBody* PhysicsSystem::createRigidBody(const BodyCreationParams& params) {
        if (!m_physicsManager)
            return nullptr;
        return m_physicsManager->createRigidBody(params);
    }

    void PhysicsSystem::destroyRigidBody(RigidBody* body) {
        if (!m_physicsManager || !body)
            return;
        m_physicsManager->destroyRigidBody(body);
    }

    SoftBody* PhysicsSystem::createSoftBody(const SoftBodyConfig& config) {
        if (!m_softBodyWorldInfo) {
            logWarning("Soft body world info not initialized");
            return nullptr;
        }

        return memoryManager_.allocateObject<SoftBody>(
            MemoryCategory::PHYSICS,
            m_softBodyWorldInfo.get(),
            config
        );
    }

    void PhysicsSystem::destroySoftBody(SoftBody* body) {
        if (!body)
            return;
        memoryManager_.deallocateObject(body, MemoryCategory::PHYSICS);
    }

    // ============================================================================
    // Constraints
    // ============================================================================

    btTypedConstraint* PhysicsSystem::createConstraint(const ConstraintCreationParams& params) {
        if (!m_physicsManager)
            return nullptr;
        return m_physicsManager->createConstraint(params);
    }

    void PhysicsSystem::destroyConstraint(btTypedConstraint* constraint) {
        if (!m_physicsManager || !constraint)
            return;
        m_physicsManager->destroyConstraint(constraint);
    }

    // ============================================================================
    // Character Controllers
    // ============================================================================

    CharacterController* PhysicsSystem::createCharacterController(const CharacterControllerParams& params) {
        if (!m_physicsManager)
            return nullptr;
        return m_physicsManager->createCharacterController(params);
    }

    void PhysicsSystem::destroyCharacterController(CharacterController* controller) {
        if (!m_physicsManager || !controller)
            return;
        m_physicsManager->destroyCharacterController(controller);
    }

    // ============================================================================
    // Vehicles
    // ============================================================================

    VehiclePhysics* PhysicsSystem::createVehicle(const VehicleParams& params) {
        if (!m_physicsManager)
            return nullptr;
        return m_physicsManager->createVehicle(params);
    }

    void PhysicsSystem::destroyVehicle(VehiclePhysics* vehicle) {
        if (!m_physicsManager || !vehicle)
            return;
        m_physicsManager->destroyVehicle(vehicle);
    }

    // ============================================================================
    // Queries
    // ============================================================================

    bool PhysicsSystem::raycast(
        const Vec3& from,
        const Vec3& to,
        RaycastHit& hit,
        const QueryFilter& filter
    ) {
        if (!m_queries)
            return false;
        return m_queries->raycast(from, to, hit, filter);
    }

    std::vector<RigidBody*> PhysicsSystem::overlapSphere(
        const Vec3& center,
        Float radius,
        const QueryFilter& filter
    ) {
        if (!m_queries)
            return {};
        return m_queries->overlapSphere(center, radius, filter);
    }

    // ============================================================================
    // Configuration
    // ============================================================================

    void PhysicsSystem::setGravity(const Vec3& gravity) {
        if (m_physicsManager) {
            m_physicsManager->setGravity(gravity);
        }
    }

    Vec3 PhysicsSystem::getGravity() const {
        if (m_physicsManager) {
            return m_physicsManager->getGravity();
        }
        return Vec3(0.0f, -9.81f, 0.0f); // Default gravity
    }

    void PhysicsSystem::setTimeScale(Float scale) {
        m_timeScale = clamp(scale, 0.0f, 2.0f);
    }

    void PhysicsSystem::enableDebugDraw(bool enable) {
        if (m_physicsManager) {
            m_physicsManager->enableDebugDraw(enable);
        }
    }

    // ============================================================================
    // Statistics and Performance
    // ============================================================================

    std::string PhysicsSystem::generatePerformanceReport() const {
        std::stringstream ss;

        ss << "=== Physics System Performance Report ===\n\n";

        ss << "System Information:\n";
        ss << "  Initialization Time: " << m_statistics.initializationTime << "ms\n";
        ss << "  Total Simulation Time: " << m_statistics.totalSimulationTime << "s\n";
        ss << "  Time Scale: " << m_timeScale << "\n\n";

        ss << "Performance Metrics:\n";
        ss << "  Average Frame Time: " << m_statistics.averageFrameTime << "ms\n";
        ss << "  Peak Frame Time: " << m_statistics.peakFrameTime << "ms\n";
        ss << "  Simulation Steps: " << m_statistics.simulationSteps << "\n\n";

        ss << "Object Counts:\n";
        ss << "  Rigid Bodies: " << m_statistics.numRigidBodies << "\n";
        ss << "  Soft Bodies: " << m_statistics.numSoftBodies << "\n";
        ss << "  Constraints: " << m_statistics.numConstraints << "\n";
        ss << "  Characters: " << m_statistics.numCharacters << "\n";
        ss << "  Vehicles: " << m_statistics.numVehicles << "\n\n";

        ss << "Collision Statistics:\n";
        ss << "  Collision Pairs: " << m_statistics.numCollisionPairs << "\n";
        ss << "  Active Contacts: " << m_statistics.numContacts << "\n\n";

        ss << "Memory Usage:\n";
        ss << "  Used: " << (m_statistics.memoryUsed / 1024.0f / 1024.0f) << " MB\n";
        ss << "  Allocated: " << (m_statistics.memoryAllocated / 1024.0f / 1024.0f) << " MB\n\n";

        // Add subsystem reports
        if (m_physicsManager) {
            ss << m_physicsManager->generatePerformanceReport() << "\n";
        }
        ss << m_profiler.getReport() << "\n"; // Fixed: use getReport() instead of generateReport()

        return ss.str();
    }

    // ============================================================================
    // Private Implementation Methods
    // ============================================================================

    bool PhysicsSystem::initializePhysicsManager(const PhysicsConfig& config) {
        // Allocate PhysicsManager using our memory manager
        m_physicsManager = memoryManager_.allocateObject<PhysicsManager>(
            MemoryCategory::PHYSICS,
            memoryManager_
        );

        if (!m_physicsManager) {
            logError("Failed to allocate PhysicsManager");
            return false;
        }

        // Initialize the physics manager
        if (!m_physicsManager->initialize(config)) {
            logError("Failed to initialize PhysicsManager");
            memoryManager_.deallocateObject(m_physicsManager, MemoryCategory::PHYSICS);
            m_physicsManager = nullptr;
            return false;
        }

        return true;
    }

    bool PhysicsSystem::initializeSubsystems(const PhysicsConfig& config) {
        try {
            // Initialize query system
            m_queries = std::make_unique<PhysicsQueries>(m_mainWorld);
            if (!m_queries) {
                logError("Failed to create PhysicsQueries");
                return false;
            }

            // Initialize destruction system - pass MemoryManager pointer
            m_destructionSystem = std::make_unique<DestructionSystem>(m_mainWorld, &memoryManager_);
            if (!m_destructionSystem) {
                logError("Failed to create DestructionSystem");
                return false;
            }

            // Initialize trigger system
            m_triggerSystem = std::make_unique<TriggerSystem>();
            if (!m_triggerSystem) {
                logError("Failed to create TriggerSystem");
                return false;
            }

            // Initialize component bridge
            m_componentBridge = std::make_unique<PhysicsComponentBridge>();
            if (!m_componentBridge) {
                logError("Failed to create PhysicsComponentBridge");
                return false;
            }

            // Initialize force registry
            m_forceRegistry = std::make_unique<ForceGeneratorRegistry>();
            if (!m_forceRegistry) {
                logError("Failed to create ForceGeneratorRegistry");
                return false;
            }

            // Initialize soft body world info if soft bodies are enabled
            // For now, always create it since we don't have enableSoftBodies in config
            // You can add this field to PhysicsConfig if needed
            const bool enableSoftBodies = true; // TODO: Add to PhysicsConfig
            if (enableSoftBodies) {
                m_softBodyWorldInfo = std::make_unique<btSoftBodyWorldInfo>();
                if (!m_softBodyWorldInfo) {
                    logError("Failed to create soft body world info");
                    return false;
                }
                // TODO: Proper soft body world info initialization
            }

            logInfo("All physics subsystems initialized successfully");
            return true;
        } catch (const std::exception& e) {
            logError("Exception during subsystem initialization: " + std::string(e.what()));
            return false;
        }
    }

    void PhysicsSystem::cleanupSubsystems() {
        // Clean up in reverse order of initialization
        m_softBodyWorldInfo.reset();
        m_forceRegistry.reset();
        m_componentBridge.reset();
        m_triggerSystem.reset();
        m_destructionSystem.reset();
        m_queries.reset();
    }

    void PhysicsSystem::setupDefaultCollisionFiltering() const {
        if (!m_physicsManager) {
            logError("Cannot setup collision filtering - PhysicsManager not initialized");
            return;
        }

        // Use the method from PhysicsManager
        if (!m_physicsManager->setupRPGCollisionFiltering()) {
            logError("Failed to setup RPG collision filtering");
            return;
        }

        logInfo("Default RPG collision filtering configured successfully");
    }

    void PhysicsSystem::registerCallbacks() {
        if (!m_physicsManager)
            return;

        // Register collision callback
        m_physicsManager->registerPostPhysicsCallback(
            [this](const Float dt) { processCollisions(dt); }
        );
    }

    void PhysicsSystem::updatePrePhysics(const Float deltaTime) const {
        // Apply time scale
        const Float scaledDelta = deltaTime * m_timeScale;

        // Update force generators
        if (m_forceRegistry) {
            m_forceRegistry->updateGenerators(scaledDelta);
        }

        // Sync transforms from ECS to physics
        if (m_componentBridge) {
            m_componentBridge->syncToPhysics(scaledDelta);
        }
    }

    void PhysicsSystem::updatePostPhysics(Float deltaTime) {
        // Sync transforms from physics to ECS
        if (m_componentBridge) {
            m_componentBridge->syncFromPhysics(deltaTime);
        }

        // Update trigger system
        if (m_triggerSystem) {
            m_triggerSystem->update(deltaTime);
        }

        // Update destruction system
        if (m_destructionSystem) {
            m_destructionSystem->update(deltaTime);
        }
    }

    // TODO: Revisar este metodo
    void PhysicsSystem::processCollisions(Float deltaTime) {
        // Process collision events for destruction system
        // This would iterate through collision manifolds and trigger appropriate events
        if (m_destructionSystem && m_physicsManager) {
            // Get collision manifolds from physics world
            // Process destruction events based on collision forces
            // This is where we'd handle things like:
            // - Destructible objects breaking apart
            // - Damage calculations
            // - Sound/particle effects triggers
        }

        if (!m_mainWorld) {
            return;
        }

        // Get collision world from Bullet
        btCollisionWorld* collisionWorld = m_mainWorld->getBulletWorld()->getCollisionWorld();
        if (!collisionWorld) {
            return;
        }

        // Get all collision manifolds (contact points)
        btDispatcher* dispatcher = collisionWorld->getDispatcher();
        int numManifolds = dispatcher->getNumManifolds();

        // ⭐ AQUÍ - Debug de colisiones detectadas
        // if (numManifolds > 0) {
        //     std::cout << "=== Frame Collisions: " << numManifolds << " ===" << std::endl;
        // }

        for (int i = 0; i < numManifolds; ++i) {
            btPersistentManifold* contactManifold = dispatcher->getManifoldByIndexInternal(i);

            const btCollisionObject* obA = contactManifold->getBody0();
            const btCollisionObject* obB = contactManifold->getBody1();

            int numContacts = contactManifold->getNumContacts();

            if (numContacts > 0) {
                // ⭐ AQUÍ - Debug detallado de cada colisión
                // std::cout << "Collision between bodies:" << std::endl;
                // std::cout << "  Body A: " << obA << std::endl;
                // std::cout << "  Body B: " << obB << std::endl;
                // std::cout << "  Contacts: " << numContacts << std::endl;

                // Iterar sobre puntos de contacto
                for (int j = 0; j < numContacts; ++j) {
                    btManifoldPoint& pt = contactManifold->getContactPoint(j);

                    // Solo reportar contactos reales (penetración)
                    if (pt.getDistance() < 0.0f) {
                        const btVector3& ptA = pt.getPositionWorldOnA();
                        const btVector3& ptB = pt.getPositionWorldOnB();
                        const btVector3& normalOnB = pt.m_normalWorldOnB;

                        // std::cout << "    Contact " << j << ":" << std::endl;
                        // std::cout << "      Position A: (" << ptA.x() << ", " << ptA.y() << ", " << ptA.z() << ")" << std::endl;
                        // std::cout << "      Position B: (" << ptB.x() << ", " << ptB.y() << ", " << ptB.z() << ")" << std::endl;
                        // std::cout << "      Normal: (" << normalOnB.x() << ", " << normalOnB.y() << ", " << normalOnB.z() << ")" << std::endl;
                        // std::cout << "      Penetration: " << pt.getDistance() << std::endl;
                        // std::cout << "      Impulse: " << pt.getAppliedImpulse() << std::endl;
                    }
                }
            }
        }

        // Process triggers
        if (m_triggerSystem) {
            m_triggerSystem->update(deltaTime);
        }
    }

    void PhysicsSystem::updateStatistics(Float deltaTime) {
        m_totalSimulationTime += deltaTime;

        // Update profiling statistics
        const auto& profilerStats = m_profiler.getStatistics();
        m_statistics.averageFrameTime = profilerStats.avgTotalTime;
        m_statistics.peakFrameTime = profilerStats.maxTotalTime;

        // Get simulation statistics from physics manager
        if (m_physicsManager) {
            const auto& managerStats = m_physicsManager->getStatistics();
            m_statistics.numRigidBodies = managerStats.numBodies;
            m_statistics.numContacts = managerStats.numContacts;
        }

        // Update system timing
        m_statistics.initializationTime = m_initializationTime;
        m_statistics.totalSimulationTime = m_totalSimulationTime;

        // Update memory statistics
        m_statistics.memoryUsed = memoryManager_.getTotalMemoryUsage();
        m_statistics.memoryAllocated = memoryManager_.getCategoryMemoryUsage(MemoryCategory::PHYSICS);
    }

    // ============================================================================
    // Logging Utilities
    // ============================================================================

    void PhysicsSystem::logInfo(const std::string& message) {
        // TODO: Interface with actual logging system
        std::cout << "[PhysicsSystem] INFO: " << message << std::endl;
    }

    void PhysicsSystem::logWarning(const std::string& message) {
        // TODO: Interface with actual logging system
        std::cout << "[PhysicsSystem] WARNING: " << message << std::endl;
    }

    void PhysicsSystem::logError(const std::string& message) {
        // TODO: Interface with actual logging system
        std::cerr << "[PhysicsSystem] ERROR: " << message << std::endl;
    }
} // namespace engine::physics
