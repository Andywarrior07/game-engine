/**
 * @file PhysicsManager.h
 * @brief Central physics system manager and coordinator
 * @details Manages physics worlds, object pools, LOD, streaming, and provides
 *          the main interface for game systems to interact with physics
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

#include "PhysicsProfiler.h"

#include "../gameplay/CharacterController.h"
#include "../gameplay/VehiclePhysics.h"
#include "../streaming/PhysicsLOD.h"

#include "../../memory/manager/ObjectPool.h"

namespace engine::physics {
    class CollisionFilter;
    class CollisionDetection;
    class CompoundShape;

    /**
     * @brief Central physics system manager
     * @details Manages all physics subsystems, handles multi-threading,
     *          memory pooling, and provides the main API for physics operations
     */
    class PhysicsManager {
    public:
        explicit PhysicsManager(MemoryManager& memoryManager);

        ~PhysicsManager();

        PhysicsManager(const PhysicsManager&) = delete;
        PhysicsManager& operator=(const PhysicsManager&) = delete;

        // ============================================================================
        // Initialization and Shutdown
        // ============================================================================

        /**
         * @brief Initialize physics system with configuration
         * @param config Physics configuration
         */
        bool initialize(const PhysicsConfig& config = PhysicsConfig::ForRPG());

        /**
         * @brief Shutdown physics system and release resources
         */
        void shutdown();

        // ============================================================================
        // Update Loop
        // ============================================================================

        /**
         * @brief Main physics update
         * @param deltaTime Frame time in seconds
         * @param viewerPosition Position for LOD calculations
         */
        void update(Float deltaTime, const Vec3& viewerPosition = VEC3_ZERO);

        void update(const Float deltaTime) {
            update(deltaTime, cachedViewerPosition_);
        }

        /**
         * @brief Set viewer position for LOD calculations
         * @param position Current viewer position
         */
        void setViewerPosition(const Vec3& position) {
            cachedViewerPosition_ = position;
        }

        /**
         * @brief Fixed update for deterministic physics
         */
        void fixedUpdate() const;

        // ============================================================================
        // Body Management
        // ============================================================================

        // TODO: este metodo esta en PhysicsWorld, quiza deberia hacer lo mismo que raycast, y llamar mainworld->createRigidBody
        /**
         * @brief Create a rigid body with proper LOD registration
         */
        RigidBody* createRigidBody(const BodyCreationParams& params);

        // TODO: este metodo esta en PhysicsWorld, quiza deberia hacer lo mismo que raycast, y llamar mainworld->destroyRigidBody
        /**
         * @brief Destroy a rigid body
         */
        void destroyRigidBody(RigidBody* body);

        // TODO: este metodo esta en PhysicsWorld, quiza deberia hacer lo mismo que raycast, y llamar mainworld->findBodyById
        /**
         * @brief Find body by ID
         */
        RigidBody* findBodyById(const std::uint32_t id) const {
            const auto it = bodyIdMap_.find(id);
            return it != bodyIdMap_.end() ? it->second : nullptr;
        }

        /**
         * @brief Find body by name
         */
        RigidBody* findBodyByName(const std::string& name) const {
            const auto it = bodyNameMap_.find(name);
            return it != bodyNameMap_.end() ? it->second : nullptr;
        }

        // ============================================================================
        // Shape Management
        // ============================================================================

        /**
         * @brief Create a collision shape directly (no factory needed)
         */
        CollisionShape* createShape(const ShapeCreationParams& params) {
            return createCollisionShapeInternal(params);
        }

        /**
         * @brief Destroy a collision shape directly (no factory needed)
         */
        void destroyShape(CollisionShape* shape) {
            destroyCollisionShapeInternal(shape);
        }

        // ============================================================================
        // Constraint Management
        // ============================================================================

        /**
         * @brief Create a physics constraint
         */
        btTypedConstraint* createConstraint(const ConstraintCreationParams& params);

        /**
         * @brief Destroy a constraint
         */
        void destroyConstraint(btTypedConstraint* constraint);

        // ============================================================================
        // Collision Queries
        // ============================================================================

        /**
         * @brief Perform a raycast
         */
        bool raycast(const Vec3& from, const Vec3& to, RaycastHit& hit,
                     const QueryFilter& filter = QueryFilter()) const;

        /**
         * @brief Raycast all hits
         */
        std::vector<RaycastHit> raycastAll(const Vec3& from, const Vec3& to,
                                           const QueryFilter& filter = QueryFilter()) const;

        /**
         * @brief Sphere overlap test
         */
        std::vector<RigidBody*> overlapSphere(const Vec3& center, Float radius,
                                              const QueryFilter& filter = QueryFilter()) const;

        /**
         * @brief Box overlap test
         */
        std::vector<RigidBody*> overlapBox(const Vec3& center, const Vec3& halfExtents,
                                           const Quat& rotation = QUAT_IDENTITY,
                                           const QueryFilter& filter = QueryFilter()) const;

        /**
         * @brief Capsule sweep test
         */
        bool sweepCapsule(const Vec3& from, const Vec3& to, Float radius, Float height,
                          SweepResult& result, const QueryFilter& filter = QueryFilter()) {
            // Implementation for capsule sweep
            return false;
        }

        // ============================================================================
        // Character Controllers
        // ============================================================================

        /**
         * @brief Create a character controller
         */
        CharacterController* createCharacterController(const CharacterControllerParams& params);

        /**
         * @brief Destroy a character controller
         */
        void destroyCharacterController(CharacterController* controller);

        // ============================================================================
        // Vehicle Physics
        // ============================================================================

        /**
         * @brief Create a vehicle
         */
        VehiclePhysics* createVehicle(const VehicleParams& params);

        /**
         * @brief Destroy a vehicle
         */
        void destroyVehicle(VehiclePhysics* vehicle);

        // ============================================================================
        // World Management
        // ============================================================================

        PhysicsWorld* getMainWorld() { return mainWorld_.get(); }
        const PhysicsWorld* getMainWorld() const { return mainWorld_.get(); }

        /**
         * @brief Create an async physics world for background simulation
         */
        PhysicsWorld* createAsyncWorld(const PhysicsConfig& config);

        // ============================================================================
        // Collision System Access
        // ============================================================================

        /**
         * @brief Get collision detection system
         * @return Pointer to collision detection system
         */
        CollisionDetection* getCollisionDetection() const {
            return collisionDetection_.get();
        }

        /**
         * @brief Get collision filter for configuration
         * @return Pointer to collision filter, nullptr if not available
         */
        CollisionFilter* getCollisionFilter() const;

        /**
         * @brief Setup RPG collision filtering
         * @return true if setup successful, false otherwise
         */
        bool setupRPGCollisionFiltering() const;

        /**
         * @brief Set broad phase algorithm
         * @param type Broad phase type to use
             */
        void setBroadPhaseType(BroadphaseType type) const;

        /**
         * @brief Enable/disable continuous collision detection
         * @param enable Whether to enable CCD
         */
        void setContinuousCollisionDetection(bool enable) const;

        /**
         * @brief Get collision statistics
         * @return Collision detection statistics
         */
        // CollisionDetection::Statistics getCollisionStatistics() const {
        //     if (!collisionDetection_) {
        //         return CollisionDetection::Statistics{};
        //     }
        //     return collisionDetection_->getStatistics();
        // }

        // ============================================================================
        // Configuration
        // ============================================================================

        const PhysicsConfig& getConfig() const { return config_; }

        void setGravity(const Vec3& gravity);

        Vec3 getGravity() const { return config_.gravity; }

        // ============================================================================
        // Callbacks
        // ============================================================================

        using PhysicsCallback = std::function<void(Float)>;

        void registerPrePhysicsCallback(const PhysicsCallback& callback) {
            prePhysicsCallbacks_.push_back(callback);
        }

        void registerPostPhysicsCallback(const PhysicsCallback& callback) {
            postPhysicsCallbacks_.push_back(callback);
        }

        // ============================================================================
        // Statistics and Profiling
        // ============================================================================

        const PhysicsStats& getStatistics() const;

        PhysicsProfiler* getProfiler() const { return profiler_; }

        /**
         * @brief Generate performance report
         */
        std::string generatePerformanceReport() const;

        // ============================================================================
        // Debug
        // ============================================================================

        void enableDebugDraw(bool enable) const;

        void debugDrawWorld() const;

    private:
        // Core state
        bool initialized_ = false;
        PhysicsConfig config_;
        std::chrono::steady_clock::time_point startTime_;

        // Modules
        MemoryManager& memoryManager_;

        // Physics worlds
        std::unique_ptr<PhysicsWorld> mainWorld_;
        std::vector<std::unique_ptr<PhysicsWorld>> asyncWorlds_;

        // Collision
        std::unique_ptr<CollisionDetection> collisionDetection_;

        // Object management
        std::unordered_set<RigidBody*> bodies_;
        std::unordered_set<CollisionShape*> shapes_;
        std::vector<btTypedConstraint*> constraints_;
        std::vector<CharacterController*> characterControllers_;
        std::vector<VehiclePhysics*> vehicles_;

        // Lookup maps
        std::unordered_map<std::uint32_t, RigidBody*> bodyIdMap_;
        std::unordered_map<std::string, RigidBody*> bodyNameMap_;

        // Subsystems - managed through MemoryManager
        PhysicsLODSystem* lodSystem_ = nullptr;
        PhysicsProfiler* profiler_ = nullptr;

        // Memory pools
        ObjectPool<RigidBody> bodyPool_;
        ObjectPool<CollisionShape> shapePool_;

        // Threading
        std::vector<std::thread> workerThreads_;
        std::atomic<bool> stopWorkers_{false};
        std::queue<std::function<void()>> asyncTasks_;
        std::mutex taskMutex_;
        std::condition_variable taskCondition_;

        // Callbacks
        std::vector<PhysicsCallback> prePhysicsCallbacks_;
        std::vector<PhysicsCallback> postPhysicsCallbacks_;

        Vec3 cachedViewerPosition_{0, 0, 0};

        // Statistics
        PhysicsStats emptyStats_;

        // ============================================================================
        // Private Methods - FIXED AND ADDED
        // ============================================================================

        void initializePools();

        void clearPools();

        /**
         * @brief FIXED: Get existing shape or create new one
         */
        CollisionShape* getOrCreateShape(const ShapeCreationParams& params);

        /**
         * @brief ADDED: Internal shape creation - replaces CollisionShapeFactory
         */
        CollisionShape* createCollisionShapeInternal(const ShapeCreationParams& params);


        /**
         * @brief ADDED: Internal shape destruction
         */
        void destroyCollisionShapeInternal(CollisionShape* shape);

        /**
         * @brief ADDED: Create compound shape internally
         */
        CompoundShape* createCompoundShapeInternal(const ShapeCreationParams& params);

        void clearAllBodies();

        void clearAllConstraints();

        void clearAllShapes();

        void startWorkerThreads(const int numThreads);

        void stopWorkerThreads();

        void processAsyncTasks() {
            // Process completed async tasks
            // Implementation depends on specific async task requirements
        }

        Float getUptime() const;
    };
} // namespace engine::physics
