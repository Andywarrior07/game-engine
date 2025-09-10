/**
 * @file BroadPhase.h
 * @brief Broad phase collision detection algorithms
 * @details Implements spatial partitioning and sweep-and-prune for efficiently
 *          finding potentially colliding pairs of objects
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

#include "../core/PhysicsTypes.h"
#include "../core/PhysicsConstants.h"
#include "../dynamics/RigidBody.h"
#include <btBulletCollisionCommon.h>
#include <algorithm>
#include <vector>
#include <unordered_set>

namespace engine::physics {
    /**
     * @brief Collision pair identifier
     */
    struct CollisionPair {
        RigidBody* bodyA;
        RigidBody* bodyB;

        CollisionPair(RigidBody* a, RigidBody* b) {
            // Ensure consistent ordering
            if (a < b) {
                bodyA = a;
                bodyB = b;
            }
            else {
                bodyA = b;
                bodyB = a;
            }
        }

        bool operator==(const CollisionPair& other) const {
            return bodyA == other.bodyA && bodyB == other.bodyB;
        }

        struct Hash {
            std::size_t operator()(const CollisionPair& pair) const {
                std::size_t h1 = std::hash<void*>()(pair.bodyA);
                std::size_t h2 = std::hash<void*>()(pair.bodyB);
                return h1 ^ (h2 << 1);
            }
        };
    };

    /**
     * @brief Base broad phase interface
     */
    class IBroadPhase {
    public:
        virtual ~IBroadPhase() = default;

        /**
         * @brief Add body to broad phase
         */
        virtual void addBody(RigidBody* body) = 0;

        /**
         * @brief Remove body from broad phase
         */
        virtual void removeBody(RigidBody* body) = 0;

        /**
         * @brief Update body's bounds in broad phase
         */
        virtual void updateBody(RigidBody* body) = 0;

        /**
         * @brief Get all potentially colliding pairs
         */
        virtual void computePairs(std::vector<CollisionPair>& pairs) = 0;

        /**
         * @brief Query overlapping bodies
         */
        virtual void query(const AABB& aabb, std::vector<RigidBody*>& results) = 0;

        /**
         * @brief Ray query
         */
        virtual void rayQuery(const Vec3& from, const Vec3& to,
                              std::vector<RigidBody*>& results) = 0;

        /**
         * @brief Clear all data
         */
        virtual void clear() = 0;

        /**
         * @brief Get statistics
         */
        virtual std::size_t getProxyCount() const = 0;
        virtual std::size_t getPairCount() const = 0;
    };

    /**
     * @brief Sweep and Prune broad phase implementation
     * @details Efficient for scenes with many static objects and coherent motion
     */
    class SweepAndPruneBroadPhase final : public IBroadPhase {
    public:
        SweepAndPruneBroadPhase();

        void addBody(RigidBody* body) override;

        void removeBody(RigidBody* body) override;

        void updateBody(RigidBody* body) override;

        void computePairs(std::vector<CollisionPair>& pairs) override;

        void query(const AABB& aabb, std::vector<RigidBody*>& results) override;

        void rayQuery(const Vec3& from, const Vec3& to,
                      std::vector<RigidBody*>& results) override;

        void clear() override;

        std::size_t getProxyCount() const override { return proxies_.size(); }
        std::size_t getPairCount() const override { return lastPairCount_; }

    private:
        struct Endpoint {
            Float value;
            std::uint32_t proxyId;
            bool isMin;

            bool operator<(const Endpoint& other) const {
                return value < other.value;
            }
        };

        struct Proxy {
            RigidBody* body = nullptr;
            AABB aabb;
            std::uint32_t id = 0;
        };

        // Endpoints for each axis
        std::vector<Endpoint> endpoints_[3];

        // Proxy storage
        std::unordered_map<std::uint32_t, Proxy> proxies_;
        std::unordered_map<RigidBody*, std::uint32_t> bodyToProxy_;
        std::uint32_t nextProxyId_ = 0;

        // Statistics
        mutable std::size_t lastPairCount_ = 0;

        void insertEndpoint(int axis, const Endpoint& endpoint);

        void sortEndpoints(const int axis) {
            std::sort(endpoints_[axis].begin(), endpoints_[axis].end());
        }

        void updateEndpoints(int axis, std::uint32_t proxyId,
                             const AABB& oldAABB, const AABB& newAABB);

        bool overlapsOnOtherAxes(std::uint32_t proxyA, std::uint32_t proxyB);

        static bool shouldTestPair(const RigidBody* bodyA, const RigidBody* bodyB);
    };

    /**
     * @brief Dynamic AABB Tree broad phase
     * @details Better for dynamic scenes with frequent updates
     */
    class DynamicAABBTreeBroadPhase final : public IBroadPhase {
    public:
        DynamicAABBTreeBroadPhase() : root_(nullptr), nodeCount_(0) {
            nodes_.reserve(collision::DEFAULT_MAX_PROXIES);
        }

        ~DynamicAABBTreeBroadPhase() override;

        void addBody(RigidBody* body) override;

        void removeBody(RigidBody* body) override;

        void updateBody(RigidBody* body) override;

        void computePairs(std::vector<CollisionPair>& pairs) override;

        void query(const AABB& aabb, std::vector<RigidBody*>& results) override;

        void rayQuery(const Vec3& from, const Vec3& to,
                      std::vector<RigidBody*>& results) override;

        void clear() override;

        std::size_t getProxyCount() const override { return bodyToNode_.size(); }
        std::size_t getPairCount() const override { return lastPairCount_; }

        /**
         * @brief Get tree height for diagnostics
         */
        int getTreeHeight() const {
            return root_ ? getHeight(root_) : 0;
        }

        /**
         * @brief Get tree balance for diagnostics
         */
        Float getTreeBalance() const;

    private:
        struct Node {
            AABB aabb;
            RigidBody* body = nullptr;
            Node* parent = nullptr;
            Node* left = nullptr;
            Node* right = nullptr;
            int height = 0;
            bool isLeaf = false;
        };

        Node* root_;
        std::vector<Node> nodes_;
        std::unordered_map<RigidBody*, Node*> bodyToNode_;
        std::size_t nodeCount_;
        mutable std::size_t lastPairCount_ = 0;

        Node* allocateNode() {
            nodes_.emplace_back();
            return &nodes_[nodeCount_++];
        }

        void deallocateNode(Node* node) {
            // In a real implementation, would maintain a free list
        }

        void insertLeaf(Node* leaf);

        void removeLeaf(const Node* leaf);

        Node* findBestSibling(const Node* leaf) const;

        static Float calculateCost(const Node* node, const Node* leaf);

        static void rebalance(Node* node);

        static void computePairsRecursive(Node* nodeA, Node* nodeB,
                                          std::unordered_set<CollisionPair, CollisionPair::Hash>& pairs);

        static void queryRecursive(const Node* node, const AABB& aabb,
                                   std::vector<RigidBody*>& results);

        static void rayQueryRecursive(const Node* node, const Ray& ray,
                                      std::vector<RigidBody*>& results);

        static int getHeight(const Node* node);
    };
} // namespace engine::physics
