/**
 * @file CollisionDetection.h
 * @brief Main collision detection pipeline
 * @details Coordinates broad and narrow phase collision detection,
 *          manages collision pairs, and generates contact manifolds
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

#include "BroadPhase.h"

#include "../core/PhysicsTypes.h"

namespace engine::physics {
    class IBroadPhase;
    class NarrowPhase;
    class CollisionFilter;

    /**
     * @brief Collision pair information
     */
    struct CollisionPairInfo;


    /**
     * @brief Collision detection pipeline
     * @details Manages the complete collision detection process from broad phase
     *          to contact generation, including temporal coherence and caching
     */
    class CollisionDetection {
    public:
        CollisionDetection();

        ~CollisionDetection();

        // ============================================================================
        // Initialization
        // ============================================================================

        void initialize();

        void shutdown();

        // ============================================================================
        // Main Collision Detection
        // ============================================================================

        /**
         * @brief Run complete collision detection pipeline
         */
        void detectCollisions(const std::vector<RigidBody*>& bodies, Float deltaTime);

        /**
         * @brief Detect collision between two specific bodies
         */
        bool detectPairCollision(RigidBody* bodyA, RigidBody* bodyB,
                                 CollisionManifold& manifold) const;

        // ============================================================================
        // Continuous Collision Detection
        // ============================================================================

        /**
         * @brief Perform CCD for fast-moving objects
         */
        bool performCCD(RigidBody* body, const Vec3& targetPosition,
                        Float& timeOfImpact, RigidBody*& hitBody) const;

        // ============================================================================
        // Queries
        // ============================================================================

        /**
         * @brief Get all collision pairs
         */
        const std::unordered_map<CollisionPair, CollisionPairInfo, CollisionPair::Hash>&
        getCollisionPairs() const {
            return collisionPairs_;
        }

        /**
         * @brief Check if two bodies are colliding
         */
        bool areColliding(RigidBody* bodyA, RigidBody* bodyB) const;

        /**
         * @brief Get manifold for collision pair
         */
        const CollisionManifold* getManifold(RigidBody* bodyA, RigidBody* bodyB) const;

        // ============================================================================
        // Configuration
        // ============================================================================

        void setBroadPhaseType(BroadphaseType type);

        void setEnableCaching(const bool enable) { enableCaching_ = enable; }
        void setEnableCCD(const bool enable) { enableCCD_ = enable; }
        void setEnableSleeping(const bool enable) { enableSleeping_ = enable; }

        CollisionFilter* getFilter() { return filter_.get(); }
        const CollisionFilter* getFilter() const { return filter_.get(); }

        // ============================================================================
        // Statistics
        // ============================================================================

        struct Statistics {
            std::size_t numBroadPhasePairs = 0;
            std::size_t numFilteredPairs = 0;
            std::size_t numNarrowPhasePairs = 0;
            std::size_t numCollisions = 0;
            std::size_t numContacts = 0;
            Float broadPhaseTime = 0.0f;
            Float filterTime = 0.0f;
            Float narrowPhaseTime = 0.0f;
            Float totalTime = 0.0f;
        };

        const Statistics& getStatistics() const { return stats_; }

    private:
        // Collision detection components
        std::unique_ptr<IBroadPhase> broadPhase_;
        std::unique_ptr<NarrowPhase> narrowPhase_;
        std::unique_ptr<CollisionFilter> filter_;

        // Collision pairs
        std::vector<CollisionPair> potentialPairs_;
        std::vector<CollisionPair> activePairs_;
        std::unordered_map<CollisionPair, CollisionPairInfo, CollisionPair::Hash> collisionPairs_;
        std::unordered_map<CollisionPair, CollisionPairInfo, CollisionPair::Hash> previousPairs_;

        // Configuration
        bool enableCaching_;
        bool enableCCD_;
        bool enableSleeping_;

        // Statistics
        Statistics stats_;

        void updateBroadPhase(const std::vector<RigidBody*>& bodies) const;

        void filterPairs(const std::vector<CollisionPair>& potential,
                         std::vector<CollisionPair>& active);

        void processCollisionPair(const CollisionPair& pair, Float deltaTime);

        void processCollisionEvents();

        static bool shouldUseCCD(const RigidBody* bodyA, const RigidBody* bodyB);

        static bool performSweptCollision(const RigidBody* movingBody, const Vec3& from, const Vec3& to,
                                          const RigidBody* staticBody, Float& timeOfImpact);

        static void performPairCCD(const RigidBody* bodyA, const RigidBody* bodyB, Float& timeOfImpact);

        void updateStatistics(const std::chrono::high_resolution_clock::time_point& startTime,
                              const std::chrono::high_resolution_clock::time_point& broadPhaseEnd,
                              const std::chrono::high_resolution_clock::time_point& filterEnd,
                              const std::chrono::high_resolution_clock::time_point& narrowPhaseEnd);
    };
} // namespace engine::physics
