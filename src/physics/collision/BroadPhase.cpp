//
// Created by Andres Guerrero on 31-08-25.
//

#include "BroadPhase.h"

namespace engine::physics {
    SweepAndPruneBroadPhase::SweepAndPruneBroadPhase() {
        for (int axis = 0; axis < 3; ++axis) {
            endpoints_[axis].reserve(collision::DEFAULT_MAX_PROXIES * 2);
        }
    }

    void SweepAndPruneBroadPhase::addBody(RigidBody* body) {
        if (!body) return;

        AABB aabb = body->getAABB();

        // Create proxy
        Proxy proxy;
        proxy.body = body;
        proxy.aabb = aabb;
        proxy.id = nextProxyId_++;

        // Add endpoints for each axis
        for (int axis = 0; axis < 3; ++axis) {
            Endpoint minEndpoint;
            minEndpoint.value = aabb.min[axis];
            minEndpoint.proxyId = proxy.id;
            minEndpoint.isMin = true;

            Endpoint maxEndpoint;
            maxEndpoint.value = aabb.max[axis];
            maxEndpoint.proxyId = proxy.id;
            maxEndpoint.isMin = false;

            insertEndpoint(axis, minEndpoint);
            insertEndpoint(axis, maxEndpoint);
        }

        proxies_[proxy.id] = proxy;
        bodyToProxy_[body] = proxy.id;
    }

    void SweepAndPruneBroadPhase::removeBody(RigidBody* body) {
        const auto it = bodyToProxy_.find(body);
        if (it == bodyToProxy_.end()) return;

        std::uint32_t proxyId = it->second;

        // Remove endpoints
        for (int axis = 0; axis < 3; ++axis) {
            std::erase_if(endpoints_[axis],
                          [proxyId](const Endpoint& ep) { return ep.proxyId == proxyId; });
        }

        proxies_.erase(proxyId);
        bodyToProxy_.erase(it);
    }

    void SweepAndPruneBroadPhase::updateBody(RigidBody* body) {
        const auto it = bodyToProxy_.find(body);
        if (it == bodyToProxy_.end()) return;

        const std::uint32_t proxyId = it->second;
        const AABB newAABB = body->getAABB();

        auto& proxy = proxies_[proxyId];
        const AABB oldAABB = proxy.aabb;
        proxy.aabb = newAABB;

        // Update endpoints if AABB changed significantly
        for (int axis = 0; axis < 3; ++axis) {
            updateEndpoints(axis, proxyId, oldAABB, newAABB);
        }
    }

    void SweepAndPruneBroadPhase::computePairs(std::vector<CollisionPair>& pairs) {
        pairs.clear();
        std::unordered_set<CollisionPair, CollisionPair::Hash> uniquePairs;

        // Sort endpoints on primary axis (usually X)
        sortEndpoints(0);

        // Sweep through endpoints
        std::unordered_set<std::uint32_t> activeProxies;

        for (const auto& endpoint : endpoints_[0]) {
            if (endpoint.isMin) {
                // Check against all active proxies
                for (std::uint32_t activeId : activeProxies) {
                    if (overlapsOnOtherAxes(endpoint.proxyId, activeId)) {
                        const auto& proxyA = proxies_[endpoint.proxyId];

                        if (const auto& proxyB = proxies_[activeId]; shouldTestPair(proxyA.body, proxyB.body)) {
                            uniquePairs.insert(CollisionPair(proxyA.body, proxyB.body));
                        }
                    }
                }
                activeProxies.insert(endpoint.proxyId);
            }
            else {
                activeProxies.erase(endpoint.proxyId);
            }
        }

        // Convert to vector
        pairs.reserve(uniquePairs.size());
        for (const auto& pair : uniquePairs) {
            pairs.push_back(pair);
        }
    }

    void SweepAndPruneBroadPhase::query(const AABB& aabb, std::vector<RigidBody*>& results) {
        results.clear();

        for (const auto& proxy : proxies_ | std::views::values) {
            if (proxy.aabb.intersects(aabb)) {
                results.push_back(proxy.body);
            }
        }
    }

    void SweepAndPruneBroadPhase::rayQuery(const Vec3& from, const Vec3& to,
                                           std::vector<RigidBody*>& results) {
        results.clear();

        // Create ray AABB
        AABB rayAABB;
        rayAABB.min = glm::min(from, to);
        rayAABB.max = glm::max(from, to);

        // Query AABB first
        std::vector<RigidBody*> candidates;
        query(rayAABB, candidates);

        // Refine with actual ray test
        for (RigidBody* body : candidates) {
            // Perform ray-AABB test
            AABB bodyAABB = body->getAABB();
            if (intersectRayAABB(Ray(from, glm::normalize(to - from)), bodyAABB)) {
                results.push_back(body);
            }
        }
    }

    void SweepAndPruneBroadPhase::clear() {
        for (int axis = 0; axis < 3; ++axis) {
            endpoints_[axis].clear();
        }
        proxies_.clear();
        bodyToProxy_.clear();
        nextProxyId_ = 0;
    }

    void SweepAndPruneBroadPhase::insertEndpoint(const int axis, const Endpoint& endpoint) {
        endpoints_[axis].push_back(endpoint);
        // Insertion sort to maintain order
        sortEndpoints(axis);
    }

    void SweepAndPruneBroadPhase::updateEndpoints(const int axis, const std::uint32_t proxyId,
                                                  const AABB& oldAABB, const AABB& newAABB) {
        // Find and update min endpoint
        for (auto& ep : endpoints_[axis]) {
            if (ep.proxyId == proxyId) {
                if (ep.isMin) {
                    ep.value = newAABB.min[axis];
                }
                else {
                    ep.value = newAABB.max[axis];
                }
            }
        }

        // Re-sort if bounds changed significantly
        if (std::abs(oldAABB.min[axis] - newAABB.min[axis]) > collision::BROADPHASE_AABB_EXPANSION ||
            std::abs(oldAABB.max[axis] - newAABB.max[axis]) > collision::BROADPHASE_AABB_EXPANSION) {
            sortEndpoints(axis);
        }
    }

    bool SweepAndPruneBroadPhase::overlapsOnOtherAxes(const std::uint32_t proxyA, const std::uint32_t proxyB) {
        const auto& a = proxies_[proxyA].aabb;
        const auto& b = proxies_[proxyB].aabb;

        return (a.min.y <= b.max.y && a.max.y >= b.min.y) &&
            (a.min.z <= b.max.z && a.max.z >= b.min.z);
    }

    bool SweepAndPruneBroadPhase::shouldTestPair(const RigidBody* bodyA, const RigidBody* bodyB) {
        // Skip if both are static
        if (bodyA->getType() == BodyType::STATIC &&
            bodyB->getType() == BodyType::STATIC) {
            return false;
        }

        // Skip if both are sleeping
        if (!bodyA->isActive() && !bodyB->isActive()) {
            return false;
        }

        // Check collision filtering
        const std::uint16_t groupA = bodyA->getCollisionGroup();
        const std::uint16_t groupB = bodyB->getCollisionGroup();
        const std::uint16_t maskA = bodyA->getCollisionMask();
        const std::uint16_t maskB = bodyB->getCollisionMask();

        return (groupA & maskB) && (groupB & maskA);
    }

    DynamicAABBTreeBroadPhase::~DynamicAABBTreeBroadPhase() {
        clear();
    }

    void DynamicAABBTreeBroadPhase::addBody(RigidBody* body) {
        if (!body) return;

        AABB aabb = body->getAABB();
        // Fatten AABB for better temporal coherence
        aabb.expand(collision::BROADPHASE_AABB_EXPANSION);

        Node* node = allocateNode();
        node->aabb = aabb;
        node->body = body;
        node->isLeaf = true;

        insertLeaf(node);
        bodyToNode_[body] = node;
    }

    void DynamicAABBTreeBroadPhase::removeBody(RigidBody* body) {
        const auto it = bodyToNode_.find(body);
        if (it == bodyToNode_.end()) return;

        Node* node = it->second;
        removeLeaf(node);
        deallocateNode(node);
        bodyToNode_.erase(it);
    }

    void DynamicAABBTreeBroadPhase::updateBody(RigidBody* body) {
        const auto it = bodyToNode_.find(body);
        if (it == bodyToNode_.end()) return;

        Node* node = it->second;
        AABB newAABB = body->getAABB();

        // Check if update is needed
        if (node->aabb.contains(newAABB)) {
            return; // Still within fat AABB
        }

        // Remove and reinsert
        removeLeaf(node);
        newAABB.expand(collision::BROADPHASE_AABB_EXPANSION);
        node->aabb = newAABB;
        insertLeaf(node);
    }

    void DynamicAABBTreeBroadPhase::computePairs(std::vector<CollisionPair>& pairs) {
        pairs.clear();
        if (!root_) return;

        // Self-query the tree
        std::unordered_set<CollisionPair, CollisionPair::Hash> uniquePairs;
        computePairsRecursive(root_, root_, uniquePairs);

        pairs.reserve(uniquePairs.size());
        for (const auto& pair : uniquePairs) {
            pairs.push_back(pair);
        }

        lastPairCount_ = pairs.size();
    }

    void DynamicAABBTreeBroadPhase::query(const AABB& aabb, std::vector<RigidBody*>& results) {
        results.clear();
        if (!root_) return;

        queryRecursive(root_, aabb, results);
    }

    void DynamicAABBTreeBroadPhase::rayQuery(const Vec3& from, const Vec3& to,
                                             std::vector<RigidBody*>& results) {
        results.clear();
        if (!root_) return;

        const Ray ray(from, glm::normalize(to - from), glm::length(to - from));
        rayQueryRecursive(root_, ray, results);
    }

    void DynamicAABBTreeBroadPhase::clear() {
        root_ = nullptr;
        nodes_.clear();
        bodyToNode_.clear();
        nodeCount_ = 0;
    }

    Float DynamicAABBTreeBroadPhase::getTreeBalance() const {
        if (!root_) return 1.0f;

        const int leftHeight = root_->left ? getHeight(root_->left) : 0;
        const int rightHeight = root_->right ? getHeight(root_->right) : 0;

        return 1.0f - std::abs(leftHeight - rightHeight) /
            static_cast<Float>(std::max(leftHeight, rightHeight));
    }

    void DynamicAABBTreeBroadPhase::insertLeaf(Node* leaf) {
        if (!root_) {
            root_ = leaf;
            return;
        }

        // Find best sibling
        Node* sibling = findBestSibling(leaf);

        // Create new parent
        Node* oldParent = sibling->parent;
        Node* newParent = allocateNode();
        newParent->parent = oldParent;
        newParent->aabb = AABB();
        newParent->aabb.expandToInclude(leaf->aabb);
        newParent->aabb.expandToInclude(sibling->aabb);
        newParent->height = sibling->height + 1;

        if (oldParent) {
            if (oldParent->left == sibling) {
                oldParent->left = newParent;
            }
            else {
                oldParent->right = newParent;
            }
            newParent->left = sibling;
            newParent->right = leaf;
            sibling->parent = newParent;
            leaf->parent = newParent;
        }
        else {
            newParent->left = sibling;
            newParent->right = leaf;
            sibling->parent = newParent;
            leaf->parent = newParent;
            root_ = newParent;
        }

        // Walk up and rebalance
        rebalance(leaf->parent);
    }

    void DynamicAABBTreeBroadPhase::removeLeaf(const Node* leaf) {
        if (leaf == root_) {
            root_ = nullptr;
            return;
        }

        Node* parent = leaf->parent;
        Node* grandParent = parent->parent;
        Node* sibling = (parent->left == leaf) ? parent->right : parent->left;

        if (grandParent) {
            if (grandParent->left == parent) {
                grandParent->left = sibling;
            }
            else {
                grandParent->right = sibling;
            }
            sibling->parent = grandParent;
            deallocateNode(parent);

            // Rebalance
            rebalance(grandParent);
        }
        else {
            root_ = sibling;
            sibling->parent = nullptr;
            deallocateNode(parent);
        }
    }

    DynamicAABBTreeBroadPhase::Node* DynamicAABBTreeBroadPhase::findBestSibling(const Node* leaf) const {
        Node* current = root_;

        while (!current->isLeaf) {
            const Float leftCost = calculateCost(current->left, leaf);
            const Float rightCost = calculateCost(current->right, leaf);

            current = (leftCost < rightCost) ? current->left : current->right;
        }

        return current;
    }

    Float DynamicAABBTreeBroadPhase::calculateCost(const Node* node, const Node* leaf) {
        AABB combined;
        combined.expandToInclude(node->aabb);
        combined.expandToInclude(leaf->aabb);
        return combined.getSurfaceArea();
    }

    void DynamicAABBTreeBroadPhase::rebalance(Node* node) {
        while (node) {
            node->height = 1 + std::max(
                node->left ? node->left->height : 0,
                node->right ? node->right->height : 0
            );

            node->aabb = AABB();
            if (node->left) node->aabb.expandToInclude(node->left->aabb);
            if (node->right) node->aabb.expandToInclude(node->right->aabb);

            node = node->parent;
        }
    }

    void DynamicAABBTreeBroadPhase::computePairsRecursive(Node* nodeA, Node* nodeB,
                                   std::unordered_set<CollisionPair, CollisionPair::Hash>& pairs) {
        if (!nodeA || !nodeB) return;
        if (nodeA == nodeB) return;

        if (!nodeA->aabb.intersects(nodeB->aabb)) return;

        if (nodeA->isLeaf && nodeB->isLeaf) {
            if (nodeA->body && nodeB->body && nodeA->body != nodeB->body) {
                pairs.insert(CollisionPair(nodeA->body, nodeB->body));
            }
        }
        else if (nodeA->isLeaf) {
            computePairsRecursive(nodeA, nodeB->left, pairs);
            computePairsRecursive(nodeA, nodeB->right, pairs);
        }
        else if (nodeB->isLeaf) {
            computePairsRecursive(nodeA->left, nodeB, pairs);
            computePairsRecursive(nodeA->right, nodeB, pairs);
        }
        else {
            computePairsRecursive(nodeA->left, nodeB->left, pairs);
            computePairsRecursive(nodeA->left, nodeB->right, pairs);
            computePairsRecursive(nodeA->right, nodeB->left, pairs);
            computePairsRecursive(nodeA->right, nodeB->right, pairs);
        }
    }

    void DynamicAABBTreeBroadPhase::queryRecursive(const Node* node, const AABB& aabb,
                                   std::vector<RigidBody*>& results) {
        if (!node) return;

        if (!node->aabb.intersects(aabb)) return;

        if (node->isLeaf) {
            if (node->body) {
                results.push_back(node->body);
            }
        }
        else {
            queryRecursive(node->left, aabb, results);
            queryRecursive(node->right, aabb, results);
        }
    }

    void DynamicAABBTreeBroadPhase::rayQueryRecursive(const Node* node, const Ray& ray,
                                      std::vector<RigidBody*>& results) {
        if (!node) return;

        if (!intersectRayAABB(ray, node->aabb)) return;

        if (node->isLeaf) {
            if (node->body) {
                results.push_back(node->body);
            }
        }
        else {
            rayQueryRecursive(node->left, ray, results);
            rayQueryRecursive(node->right, ray, results);
        }
    }

    int DynamicAABBTreeBroadPhase::getHeight(const Node* node) {
        // if (!node) return 0;
        return node->height;
    }
} // namespace engine::physics
