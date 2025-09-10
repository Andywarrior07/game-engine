/**
 * @file PhysicsWorld.h
 * @brief Core physics world wrapper and management
 * @details Encapsulates Bullet Physics world with custom memory management,
 *          optimized collision detection, and game-specific features
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

#include "PhysicsConfig.h"
#include "PhysicsTypes.h"
//
// #include "../utils/PhysicsDebug.h"

// #include "../../math/MathSystem.h"
// #include "../../memory/MemorySystem.h"

// #include <BulletSoftBody/btSoftRigidDynamicsWorld.h>

#include <memory>
#include <ranges>
#include <unordered_map>
#include <unordered_set>


namespace engine::physics {
    // using namespace engine::math;
    // using namespace engine::memory;
    // Forward declarations
    struct RaycastHit;
    class RigidBody;
    class CollisionShape;
    class PhysicsDebugDrawer;

    /**
     * @brief Main physics world class
     * @details Manages the Bullet physics simulation, providing a high-level
     *          interface for game systems while handling performance optimizations
     */
    class PhysicsWorld {
    public:
        /**
         * @brief Construct physics world with configuration
         */
        explicit PhysicsWorld(MemoryManager& memoryManager, const PhysicsConfig& config = PhysicsConfig());

        ~PhysicsWorld();

        PhysicsWorld(const PhysicsWorld&) = delete;
        PhysicsWorld& operator=(const PhysicsWorld&) = delete;

        // ============================================================================
        // Initialization and Shutdown
        // ============================================================================

        bool initialize();

        void shutdown();

        // ============================================================================
        // Simulation Step
        // ============================================================================

        /**
         * @brief Step physics simulation with fixed timestep and interpolation
         * @param deltaTime Frame delta time in seconds
         * @return Number of simulation steps performed
         */
        Int stepSimulation(Float deltaTime);

        /**
         * @brief Force single simulation step (for debugging)
         */
        void debugStep();

        // ============================================================================
        // Body Management
        // ============================================================================

        // TODO: Revisar esto ya que deberia agregar o quitar rigid body nomas el physicsworld
        // https://chatgpt.com/c/68bcb2eb-9228-8332-aeaa-20bb74587a62
        /**
         * @brief Create and add a rigid body to the world
         */
        // static RigidBody* createRigidBody(const BodyCreationParams& params) {
        //     // This will be implemented in PhysicsManager.h
        //     // For now, return nullptr as placeholder
        //     return nullptr;
        // }

        // TODO: Ver si deberia quitar esto ya que existe en phusics manager.
        /**
         * @brief Remove and destroy a rigid body
         */
        // void destroyRigidBody(RigidBody* body) {
        //     if (!body) return;
        //
        //     // Remove from world
        //     auto it = m_bodies.find(body);
        //     if (it != m_bodies.end()) {
        //         m_bodies.erase(it);
        //         // Actual removal logic in RigidBody.h
        //     }
        // }

        /**
         * @brief Get all bodies in the world
         */
        [[nodiscard]] const std::unordered_set<RigidBody*>& getBodies() const { return bodies_; }

        // TODO: Ver si deberia quitar esto ya que existe en phusics manager.
        /**
         * @brief Find body by ID
         */
        // [[nodiscard]] RigidBody* findBodyById(std::uint32_t id) const {
        //     const auto it = m_bodyMap.find(id);
        //     return it != m_bodyMap.end() ? it->second : nullptr;
        // }

        // ============================================================================
        // Collision Queries
        // ============================================================================

        /**
         * @brief Raycast into the world
         */
        bool raycast(const Vec3& from, const Vec3& to, RaycastHit& hit,
                     const QueryFilter& filter = QueryFilter()) const;

        /**
         * @brief Raycast all hits
         */
        [[nodiscard]] std::vector<RaycastHit> raycastAll(const Vec3& from, const Vec3& to,
                                                         const QueryFilter& filter = QueryFilter()) const;

        /**
         * @brief Sphere overlap test
         */
        [[nodiscard]] std::vector<RigidBody*> overlapSphere(const Vec3& center, Float radius,
                                                            const QueryFilter& filter = QueryFilter()) const;

        /**
         * @brief Box overlap test
         */
        static std::vector<RigidBody*> overlapBox(const Vec3& center, const Vec3& halfExtents,
                                                  const Quat& rotation = QUAT_IDENTITY,
                                                  const QueryFilter& filter = QueryFilter());

        // ============================================================================
        // World Properties
        // ============================================================================

        void setGravity(const Vec3& gravity);

        [[nodiscard]] Vec3 getGravity() const { return config_.gravity; }

        [[nodiscard]] const PhysicsConfig& getConfig() const { return config_; }

        void setConfig(const PhysicsConfig& config);

        // ============================================================================
        // Statistics and Debugging
        // ============================================================================

        [[nodiscard]] const PhysicsStats& getStatistics() const { return stats_; }

        void enableDebugDraw(bool enable);

        void debugDrawWorld() const;

        btDiscreteDynamicsWorld* getBulletWorld() const;

    private:
        MemoryManager& memoryManager_;

        static MemoryManager* sMemoryManager_;

        std::unordered_map<RigidBody*, Transform> interpolatedTransforms_;

        // Configuration
        PhysicsConfig config_;
        bool initialized_ = false;

        // Bullet objects
        std::unique_ptr<btDefaultCollisionConfiguration> collisionConfig_;
        std::unique_ptr<btCollisionDispatcher> dispatcher_;
        std::unique_ptr<btBroadphaseInterface> broadphase_;
        std::unique_ptr<btConstraintSolver> solver_;
        std::unique_ptr<btDiscreteDynamicsWorld> dynamicsWorld_;
        std::unique_ptr<PhysicsDebugDrawer> debugDrawer_;

        // Object pools
        std::unordered_set<RigidBody*> bodies_;
        std::unordered_map<std::uint32_t, RigidBody*> bodyMap_;
        std::vector<std::unique_ptr<CollisionShape>> shapePool_;
        std::vector<std::unique_ptr<btTypedConstraint>> constraintPool_;

        // Collision tracking
        struct CollisionPair {
            RigidBody* bodyA;
            RigidBody* bodyB;

            bool operator==(const CollisionPair& other) const {
                return (bodyA == other.bodyA && bodyB == other.bodyB) ||
                    (bodyA == other.bodyB && bodyB == other.bodyA);
            }
        };

        struct CollisionPairHash {
            std::size_t operator()(const CollisionPair& pair) const {
                return std::hash<void*>()(pair.bodyA) ^ std::hash<void*>()(pair.bodyB);
            }
        };

        std::unordered_map<CollisionPair, CollisionManifold, CollisionPairHash> collisionPairs_;
        std::unordered_map<CollisionPair, CollisionManifold, CollisionPairHash> previousCollisionPairs_;

        // Interpolation
        Float accumulator_;
        Float interpolationAlpha_;
        std::unordered_map<RigidBody*, Transform> previousTransforms_;

        // Statistics
        PhysicsStats stats_;

        // ============================================================================
        // Private Methods
        // ============================================================================

        void createBroadphase();

        void createSolver();

        void processCollisionEvents() {
            // Process collision manifolds
            const int numManifolds = dispatcher_->getNumManifolds();

            for (int i = 0; i < numManifolds; ++i) {
                if (btPersistentManifold* manifold = dispatcher_->getManifoldByIndexInternal(i); manifold->
                    getNumContacts() > 0) {
                    auto* bodyA = static_cast<RigidBody*>(
                        manifold->getBody0()->getUserPointer());
                    const auto bodyB = static_cast<RigidBody*>(
                        manifold->getBody1()->getUserPointer());

                    if (bodyA && bodyB) {
                        CollisionPair pair{bodyA, bodyB};

                        // Convert manifold to our format
                        CollisionManifold ourManifold;
                        ourManifold.bodyA = bodyA;
                        ourManifold.bodyB = bodyB;

                        for (int j = 0; j < manifold->getNumContacts(); ++j) {
                            const btManifoldPoint& pt = manifold->getContactPoint(j);

                            ContactPoint contact;
                            contact.positionWorldOnA = Vec3(pt.m_positionWorldOnA.x(),
                                                            pt.m_positionWorldOnA.y(),
                                                            pt.m_positionWorldOnA.z());
                            contact.positionWorldOnB = Vec3(pt.m_positionWorldOnB.x(),
                                                            pt.m_positionWorldOnB.y(),
                                                            pt.m_positionWorldOnB.z());
                            contact.normalWorldOnB = Vec3(pt.m_normalWorldOnB.x(),
                                                          pt.m_normalWorldOnB.y(),
                                                          pt.m_normalWorldOnB.z());
                            contact.distance = pt.m_distance1;
                            contact.impulse = pt.m_appliedImpulse;
                            contact.bodyA = bodyA;
                            contact.bodyB = bodyB;

                            ourManifold.addContactPoint(contact);
                        }

                        collisionPairs_[pair] = ourManifold;
                    }
                }
            }
        }

        void processCollisionCallbacks() {
            // Check for new collisions (Enter)
            for (const auto& pair : collisionPairs_ | std::views::keys) {
                if (!previousCollisionPairs_.contains(pair)) {
                    // New collision
                    // Call onCollisionEnter callbacks
                }
            }

            // Check for ongoing collisions (Stay)
            for (const auto& pair : collisionPairs_ | std::views::keys) {
                if (previousCollisionPairs_.contains(pair)) {
                    // Ongoing collision
                    // Call onCollisionStay callbacks
                }
            }

            // Check for ended collisions (Exit)
            for (const auto& pair : previousCollisionPairs_ | std::views::keys) {
                if (!collisionPairs_.contains(pair)) {
                    // Collision ended
                    // Call onCollisionExit callbacks
                }
            }

            // Update previous collisions
            previousCollisionPairs_ = collisionPairs_;
            collisionPairs_.clear();
        }

        void storePreviousTransforms();

        void interpolateTransforms();

        void updateStatistics();

        void clearAllBodies();

        void clearAllConstraints() {
            for (auto& constraint : constraintPool_) {
                if (constraint) {
                    dynamicsWorld_->removeConstraint(constraint.get());
                }
            }
            constraintPool_.clear();
        }
    };
} // namespace engine::physics
