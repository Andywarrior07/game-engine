/**
 * @file PhysicsSystem.h
 * @brief Main physics system header - integrates all physics components
 * @details Central include file and system orchestrator for the entire physics engine
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

// Core Physics
#include "core/PhysicsTypes.h"
#include "core/PhysicsConstants.h"
#include "core/PhysicsConfig.h"
#include "core/PhysicsWorld.h"

// Collision System
#include "collision/BroadPhase.h"
#include "collision/NarrowPhase.h"
#include "collision/CollisionDetection.h"
#include "collision/CollisionFiltering.h"
#include "collision/CollisionShapes.h"

// Dynamics
#include "dynamics/RigidBody.h"
#include "dynamics/SoftBody.h"
#include "dynamics/ForceGenerators.h"
#include "dynamics/Integration.h"

// Constraints
#include "constraints/ConstraintSolver.h"
#include "constraints/ConstraintTypes.h"
#include "constraints/MotorConstraints.h"

// Gameplay Systems
#include "gameplay/CharacterController.h"
#include "gameplay/VehiclePhysics.h"
#include "gameplay/TriggerSystem.h"
#include "gameplay/DestructionSystem.h"

// Integration
#include "integration/BulletIntegration.h"
#include "integration/BulletWrappers.h"
#include "integration/ComponentBridge.h"

// Queries
#include "queries/PhysicsQueries.h"
#include "queries/RaycastSystem.h"

// Streaming and Optimization
#include "streaming/PhysicsLOD.h"

// Management
#include "manager/PhysicsManager.h"
#include "manager/PhysicsProfiler.h"

// Utilities
#include "utils/PhysicsDebug.h"
#include "utils/PhysicsUtils.h"

// External Dependencies
#include "../math/MathSystem.h"
#include "../memory/manager/MemoryManager.h"

#include <memory>
#include <chrono>
#include <sstream>

namespace engine::physics {

    using namespace engine::math;
    using namespace engine::memory;

    /**
     * @brief Main physics system orchestrator
     * @details Coordinates all physics subsystems and provides a unified interface
     *          Now works with dependency injection pattern instead of singletons
     */
    class PhysicsSystem {
    public:
        /**
         * @brief Constructor with dependency injection
         * @param memoryManager Reference to the memory manager instance
         */
        explicit PhysicsSystem(MemoryManager& memoryManager);

        /**
         * @brief Destructor
         */
        ~PhysicsSystem();

        // Non-copyable, movable
        PhysicsSystem(const PhysicsSystem&) = delete;
        PhysicsSystem& operator=(const PhysicsSystem&) = delete;
        PhysicsSystem(PhysicsSystem&&) = default;
        PhysicsSystem& operator=(PhysicsSystem&&) = default;

        // ============================================================================
        // System Lifecycle
        // ============================================================================

        /**
         * @brief Initialize the entire physics system
         * @param config Physics configuration parameters
         * @return true if initialization successful, false otherwise
         */
        bool initialize(const PhysicsConfig& config = PhysicsConfig::ForRPG());

        /**
         * @brief Shutdown the physics system
         * @details Properly cleans up all subsystems and releases memory
         */
        void shutdown();

        /**
         * @brief Main physics update - called every frame
         * @param deltaTime Time elapsed since last update in seconds
         */
        void update(Float deltaTime);

        /**
         * @brief Fixed update for deterministic simulation
         * @details Called at fixed intervals for consistent physics behavior
         */
        void fixedUpdate();

        // ============================================================================
        // Body Management
        // ============================================================================

        /**
         * @brief Create a rigid body with specified parameters
         * @param params Configuration for the rigid body creation
         * @return Pointer to created rigid body, nullptr on failure
         */
        RigidBody* createRigidBody(const BodyCreationParams& params);

        /**
         * @brief Destroy a rigid body and free its memory
         * @param body Pointer to the rigid body to destroy
         */
        void destroyRigidBody(RigidBody* body);

        /**
         * @brief Create a soft body with specified configuration
         * @param config Soft body configuration parameters
         * @return Pointer to created soft body, nullptr on failure
         */
        SoftBody* createSoftBody(const SoftBodyConfig& config);

        /**
         * @brief Destroy a soft body and free its memory
         * @param body Pointer to the soft body to destroy
         */
        void destroySoftBody(SoftBody* body);

        // ============================================================================
        // Constraints
        // ============================================================================

        /**
         * @brief Create a constraint between rigid bodies
         * @param params Constraint creation parameters
         * @return Pointer to created constraint, nullptr on failure
         */
        btTypedConstraint* createConstraint(const ConstraintCreationParams& params);

        /**
         * @brief Destroy a constraint
         * @param constraint Pointer to the constraint to destroy
         */
        void destroyConstraint(btTypedConstraint* constraint);

        // ============================================================================
        // Character Controllers
        // ============================================================================

        /**
         * @brief Create a character controller for player/NPC movement
         * @param params Character controller configuration
         * @return Pointer to created controller, nullptr on failure
         */
        CharacterController* createCharacterController(const CharacterControllerParams& params);

        /**
         * @brief Destroy a character controller
         * @param controller Pointer to the controller to destroy
         */
        void destroyCharacterController(CharacterController* controller);

        // ============================================================================
        // Vehicles
        // ============================================================================

        /**
         * @brief Create a vehicle physics simulation
         * @param params Vehicle configuration parameters
         * @return Pointer to created vehicle, nullptr on failure
         */
        VehiclePhysics* createVehicle(const VehicleParams& params);

        /**
         * @brief Destroy a vehicle
         * @param vehicle Pointer to the vehicle to destroy
         */
        void destroyVehicle(VehiclePhysics* vehicle);

        // ============================================================================
        // Queries
        // ============================================================================

        /**
         * @brief Perform a raycast query
         * @param from Starting point of the ray
         * @param to End point of the ray
         * @param hit Output structure for hit information
         * @param filter Query filtering options
         * @return true if ray hit something, false otherwise
         */
        bool raycast(const Vec3& from, const Vec3& to, RaycastHit& hit,
                    const QueryFilter& filter = QueryFilter());

        /**
         * @brief Perform sphere overlap test
         * @param center Center of the sphere
         * @param radius Radius of the sphere
         * @param filter Query filtering options
         * @return Vector of overlapping rigid bodies
         */
        std::vector<RigidBody*> overlapSphere(const Vec3& center, Float radius,
                                             const QueryFilter& filter = QueryFilter());

        // ============================================================================
        // Subsystem Access
        // ============================================================================

        PhysicsManager* getManager() { return m_physicsManager; }
        PhysicsWorld* getWorld() { return m_mainWorld; }
        PhysicsQueries* getQueries() { return m_queries.get(); }
        DestructionSystem* getDestructionSystem() { return m_destructionSystem.get(); }
        TriggerSystem* getTriggerSystem() { return m_triggerSystem.get(); }
        PhysicsComponentBridge* getComponentBridge() { return m_componentBridge.get(); }
        ForceGeneratorRegistry* getForceRegistry() { return m_forceRegistry.get(); }
        PhysicsProfiler& getProfiler() { return m_profiler; }
        MemoryManager& getMemoryManager() { return memoryManager_; }

        // ============================================================================
        // Configuration
        // ============================================================================

        /**
         * @brief Set world gravity
         * @param gravity New gravity vector
         */
        void setGravity(const Vec3& gravity);

        /**
         * @brief Get current world gravity
         * @return Current gravity vector
         */
        Vec3 getGravity() const;

        /**
         * @brief Set physics time scale
         * @param scale Time scale factor (0.0 to 2.0)
         */
        void setTimeScale(Float scale);

        Float getTimeScale() const { return m_timeScale; }

        /**
         * @brief Enable/disable debug visualization
         * @param enable Whether to enable debug drawing
         */
        void enableDebugDraw(bool enable);

        // ============================================================================
        // Statistics and Performance
        // ============================================================================

        struct SystemStatistics {
            // Performance metrics
            Float averageFrameTime;
            Float peakFrameTime;
            Int simulationSteps;

            // Object counts
            std::size_t numRigidBodies;
            std::size_t numSoftBodies;
            std::size_t numConstraints;
            std::size_t numCharacters;
            std::size_t numVehicles;

            // Collision statistics
            std::size_t numCollisionPairs;
            std::size_t numContacts;

            // Memory usage
            std::size_t memoryUsed;
            std::size_t memoryAllocated;

            // System information
            Float initializationTime;
            Float totalSimulationTime;
        };

        const SystemStatistics& getStatistics() const { return m_statistics; }

        /**
         * @brief Generate comprehensive performance report
         * @return Formatted performance report string
         */
        std::string generatePerformanceReport() const;

    private:
        // ============================================================================
        // Core State
        // ============================================================================

        bool m_initialized = false;
        PhysicsConfig m_config;
        MemoryManager& memoryManager_;              // Reference to injected memory manager
        PhysicsManager* m_physicsManager = nullptr; // Owned physics manager instance
        PhysicsWorld* m_mainWorld = nullptr;        // Quick access to main world
        Float m_timeScale = 1.0f;
        Float m_initializationTime = 0.0f;

        // ============================================================================
        // Subsystems - using unique_ptr for automatic cleanup
        // ============================================================================

        std::unique_ptr<PhysicsQueries> m_queries;
        std::unique_ptr<DestructionSystem> m_destructionSystem;
        std::unique_ptr<TriggerSystem> m_triggerSystem;
        std::unique_ptr<PhysicsComponentBridge> m_componentBridge;
        std::unique_ptr<ForceGeneratorRegistry> m_forceRegistry;
        std::unique_ptr<btSoftBodyWorldInfo> m_softBodyWorldInfo;

        // ============================================================================
        // Profiling and Statistics
        // ============================================================================

        PhysicsProfiler m_profiler;
        SystemStatistics m_statistics;
        Float m_totalSimulationTime = 0.0f;

        // ============================================================================
        // Private Implementation Methods
        // ============================================================================

        /**
         * @brief Initialize the physics manager with our memory manager
         * @param config Physics configuration
         * @return true if successful, false otherwise
         */
        bool initializePhysicsManager(const PhysicsConfig& config);

        /**
         * @brief Initialize all physics subsystems
         * @param config Physics configuration
         * @return true if all subsystems initialized successfully
         */
        bool initializeSubsystems(const PhysicsConfig& config);

        /**
         * @brief Clean up all subsystems in proper order
         */
        void cleanupSubsystems();

        /**
         * @brief Setup default collision filtering for RPG games
         */
        void setupDefaultCollisionFiltering() const;

        /**
         * @brief Register system callbacks
         */
        void registerCallbacks();

        /**
         * @brief Pre-physics update operations
         * @param deltaTime Frame delta time
         */
        void updatePrePhysics(Float deltaTime) const;

        /**
         * @brief Post-physics update operations
         * @param deltaTime Frame delta time
         */
        void updatePostPhysics(Float deltaTime);

        /**
         * @brief Process collision events
         * @param deltaTime Frame delta time
         */
        void processCollisions(Float deltaTime);

        /**
         * @brief Update performance and memory statistics
         * @param deltaTime Frame delta time
         */
        void updateStatistics(Float deltaTime);

        // ============================================================================
        // Logging Utilities
        // ============================================================================

        static void logInfo(const std::string& message);
        static void logWarning(const std::string& message);
        static void logError(const std::string& message);
    };

} // namespace engine::physics