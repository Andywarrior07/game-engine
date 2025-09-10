/**
 * @file CollisionDetection.cpp
 * @brief Main collision detection pipeline
 * @details Coordinates broad and narrow phase collision detection,
 *          manages collision pairs, and generates contact manifolds
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#include "BroadPhase.h"
#include "CollisionDetection.h"
#include "CollisionFiltering.h"
#include "NarrowPhase.h"

#include <chrono>

namespace engine::physics {
    struct CollisionPairInfo {
        CollisionPair pair;
        CollisionManifold manifold;
        bool isNew = true;
        bool wasColliding = false;
        Float timeOfImpact = 0.0f;
        std::chrono::steady_clock::time_point timestamp;

        explicit CollisionPairInfo(const CollisionPair& p)
            : pair(p), timestamp(std::chrono::steady_clock::now()) {
        }
    };

    CollisionDetection::CollisionDetection()
        : broadPhase_(nullptr), narrowPhase_(nullptr), filter_(nullptr),
          enableCaching_(true), enableCCD_(true), enableSleeping_(true) {
        initialize();
    }

    CollisionDetection::~CollisionDetection() {
        shutdown();
    }

    void CollisionDetection::initialize() {
        // Create default broad phase
        broadPhase_ = std::make_unique<DynamicAABBTreeBroadPhase>();

        // Create narrow phase
        narrowPhase_ = std::make_unique<NarrowPhase>();

        // Create collision filter
        filter_ = std::make_unique<CollisionFilter>();

        // Reserve space for collision pairs
        potentialPairs_.reserve(collision::MAX_COLLISION_PAIRS);
        activePairs_.reserve(collision::MAX_COLLISION_PAIRS);
    }

    void CollisionDetection::shutdown() {
        broadPhase_.reset();
        narrowPhase_.reset();
        filter_.reset();
        collisionPairs_.clear();
        previousPairs_.clear();
    }

    void CollisionDetection::detectCollisions(const std::vector<RigidBody*>& bodies, const Float deltaTime) {
        const auto startTime = std::chrono::high_resolution_clock::now();

        // Store previous collision pairs for temporal coherence
        previousPairs_ = collisionPairs_;
        collisionPairs_.clear();

        // Update broad phase
        updateBroadPhase(bodies);

        const auto broadPhaseEnd = std::chrono::high_resolution_clock::now();

        // Get potential collision pairs
        potentialPairs_.clear();
        broadPhase_->computePairs(potentialPairs_);

        // Filter pairs
        activePairs_.clear();
        filterPairs(potentialPairs_, activePairs_);

        const auto filterEnd = std::chrono::high_resolution_clock::now();

        // Narrow phase collision detection
        for (const auto& pair : activePairs_) {
            processCollisionPair(pair, deltaTime);
        }

        const auto narrowPhaseEnd = std::chrono::high_resolution_clock::now();

        // Process collision callbacks
        processCollisionEvents();

        // Update statistics
        updateStatistics(startTime, broadPhaseEnd, filterEnd, narrowPhaseEnd);
    }

    bool CollisionDetection::detectPairCollision(RigidBody* bodyA, RigidBody* bodyB,
                                 CollisionManifold& manifold) const {
        if (!bodyA || !bodyB) return false;

        // Check filtering
        if (!filter_->shouldCollide(bodyA, bodyB)) {
            return false;
        }

        // Get shapes and transforms
        CollisionShape* shapeA = bodyA->getShape();
        CollisionShape* shapeB = bodyB->getShape();

        if (!shapeA || !shapeB) return false;

        Transform transformA(bodyA->getPosition(), bodyA->getRotation());
        Transform transformB(bodyB->getPosition(), bodyB->getRotation());

        // Narrow phase test
        return narrowPhase_->testCollision(shapeA, transformA,
                                            shapeB, transformB, manifold);
    }

    bool CollisionDetection::performCCD(RigidBody* body, const Vec3& targetPosition,
                        Float& timeOfImpact, RigidBody*& hitBody) const {
        if (!enableCCD_ || !body) return false;

        const Vec3 currentPos = body->getPosition();
        const Vec3 displacement = targetPosition - currentPos;
        const Float distance = glm::length(displacement);

        // Check if CCD is needed
        if (distance < body->getCCDMotionThreshold()) {
            return false;
        }

        // Perform swept collision test
        Vec3 direction = displacement / distance;
        timeOfImpact = 1.0f;
        hitBody = nullptr;

        // Query broad phase for potential collisions along path
        AABB sweepAABB;
        sweepAABB.min = glm::min(currentPos, targetPosition);
        sweepAABB.max = glm::max(currentPos, targetPosition);
        sweepAABB.expand(body->getCCDSweptSphereRadius());

        std::vector<RigidBody*> candidates;
        broadPhase_->query(sweepAABB, candidates);

        // Test each candidate
        for (RigidBody* other : candidates) {
            if (other == body) continue;
            if (!filter_->shouldCollide(body, other)) continue;

            if (Float toi; performSweptCollision(body, currentPos, targetPosition, other, toi)) {
                if (toi < timeOfImpact) {
                    timeOfImpact = toi;
                    hitBody = other;
                }
            }
        }

        return hitBody != nullptr;
    }

    bool CollisionDetection::areColliding(RigidBody* bodyA, RigidBody* bodyB) const {
        const CollisionPair pair(bodyA, bodyB);
        return collisionPairs_.contains(pair);
    }

    const CollisionManifold* CollisionDetection::getManifold(RigidBody* bodyA, RigidBody* bodyB) const {
        const CollisionPair pair(bodyA, bodyB);
        const auto it = collisionPairs_.find(pair);
        return it != collisionPairs_.end() ? &it->second.manifold : nullptr;
    }

    void CollisionDetection::setBroadPhaseType(const BroadphaseType type) {
        switch (type) {
        case BroadphaseType::DYNAMIC_AABB_TREE:
            broadPhase_ = std::make_unique<DynamicAABBTreeBroadPhase>();
            break;
        case BroadphaseType::AXIS_SWEEP_3:
            broadPhase_ = std::make_unique<SweepAndPruneBroadPhase>();
            break;
        default:
            break;
        }
    }

    void CollisionDetection::updateBroadPhase(const std::vector<RigidBody*>& bodies) const {
        for (RigidBody* body : bodies) {
            if (!body) continue;

            // Update sleeping bodies less frequently
            if (enableSleeping_ && !body->isActive()) {
                continue;
            }

            broadPhase_->updateBody(body);
        }
    }

    void CollisionDetection::filterPairs(const std::vector<CollisionPair>& potential,
                         std::vector<CollisionPair>& active) {
        for (const auto& pair : potential) {
            // Apply collision filtering
            if (!filter_->shouldCollide(pair.bodyA, pair.bodyB)) {
                continue;
            }

            // Skip sleeping pairs
            if (enableSleeping_ &&
                !pair.bodyA->isActive() && !pair.bodyB->isActive()) {
                continue;
                }

            // Check if pair was colliding in previous frame (temporal coherence)
            if (enableCaching_) {
                if (auto it = previousPairs_.find(pair); it != previousPairs_.end() && it->second.wasColliding) {
                    // Prioritize previously colliding pairs
                    active.insert(active.begin(), pair);
                    continue;
                }
            }

            active.push_back(pair);
        }
    }

    void CollisionDetection::processCollisionPair(const CollisionPair& pair, Float deltaTime) {
        CollisionManifold manifold;

        // Get shapes and transforms
        CollisionShape* shapeA = pair.bodyA->getShape();
        CollisionShape* shapeB = pair.bodyB->getShape();

        if (!shapeA || !shapeB) return;

        Transform transformA(pair.bodyA->getPosition(), pair.bodyA->getRotation());
        Transform transformB(pair.bodyB->getPosition(), pair.bodyB->getRotation());

        // Narrow phase collision detection
        bool colliding = narrowPhase_->testCollision(
            shapeA, transformA, shapeB, transformB, manifold);

        if (colliding) {
            // Check if this is a new collision
            bool isNew = !previousPairs_.contains(pair);

            // Store collision info
            CollisionPairInfo info(pair);
            info.manifold = manifold;
            info.isNew = isNew;
            info.wasColliding = true;

            // Perform CCD if needed for this pair
            if (enableCCD_ && shouldUseCCD(pair.bodyA, pair.bodyB)) {
                performPairCCD(pair.bodyA, pair.bodyB, info.timeOfImpact);
            }

            collisionPairs_.emplace(pair, info);

            // Set manifold bodies
            manifold.bodyA = pair.bodyA;
            manifold.bodyB = pair.bodyB;
        }
    }

    void CollisionDetection::processCollisionEvents() {
        // Process new collisions (onCollisionEnter)
        for (auto& [pair, info] : collisionPairs_) {
            if (info.isNew) {
                pair.bodyA->onCollisionEnter(pair.bodyB, info.manifold);
                pair.bodyB->onCollisionEnter(pair.bodyA, info.manifold);
            }
        }

        // Process ended collisions (onCollisionExit)
        for (const auto& pair : previousPairs_ | std::views::keys) {
            if (!collisionPairs_.contains(pair)) {
                pair.bodyA->onCollisionExit(pair.bodyB);
                pair.bodyB->onCollisionExit(pair.bodyA);
            }
        }
    }

    bool CollisionDetection::shouldUseCCD(const RigidBody* bodyA, const RigidBody* bodyB) {
        const Float speedA = glm::length(bodyA->getLinearVelocity());
        const Float speedB = glm::length(bodyB->getLinearVelocity());

        return speedA > bodyA->getCCDMotionThreshold() ||
            speedB > bodyB->getCCDMotionThreshold();
    }

    auto CollisionDetection::performSweptCollision(const RigidBody* movingBody, const Vec3& from, const Vec3& to,
                                                   const RigidBody* staticBody, Float& timeOfImpact) -> bool {
        // Simplified swept sphere test
        const Float radius = movingBody->getCCDSweptSphereRadius();
        const Ray ray(from, glm::normalize(to - from), glm::length(to - from));

        AABB targetAABB = staticBody->getAABB();
        targetAABB.expand(radius);

        if (intersectRayAABB(ray, targetAABB)) {
            // Calculate exact time of impact
            // This is simplified - real implementation would use conservative advancement
            timeOfImpact = 0.5f; // Placeholder
            return true;
        }

        return false;
    }

    void CollisionDetection::performPairCCD(const RigidBody* bodyA, const RigidBody* bodyB, Float& timeOfImpact) {
        // Perform continuous collision detection between two bodies
        const Vec3 velA = bodyA->getLinearVelocity();
        const Vec3 velB = bodyB->getLinearVelocity();

        if (const Vec3 relativeVel = velA - velB; glm::length2(relativeVel) < EPSILON_SQUARED) {
            timeOfImpact = 1.0f;
            return;
        }

        // Conservative advancement would go here
        timeOfImpact = 1.0f; // Placeholder
    }

    void CollisionDetection::updateStatistics(const std::chrono::high_resolution_clock::time_point& startTime,
                              const std::chrono::high_resolution_clock::time_point& broadPhaseEnd,
                              const std::chrono::high_resolution_clock::time_point& filterEnd,
                              const std::chrono::high_resolution_clock::time_point& narrowPhaseEnd) {
        const auto now = std::chrono::high_resolution_clock::now();

        stats_.broadPhaseTime = std::chrono::duration<Float, std::milli>(
            broadPhaseEnd - startTime).count();
        stats_.filterTime = std::chrono::duration<Float, std::milli>(
            filterEnd - broadPhaseEnd).count();
        stats_.narrowPhaseTime = std::chrono::duration<Float, std::milli>(
            narrowPhaseEnd - filterEnd).count();
        stats_.totalTime = std::chrono::duration<Float, std::milli>(
            now - startTime).count();

        stats_.numBroadPhasePairs = potentialPairs_.size();
        stats_.numFilteredPairs = activePairs_.size();
        stats_.numNarrowPhasePairs = collisionPairs_.size();
        stats_.numCollisions = collisionPairs_.size();

        std::size_t totalContacts = 0;
        for (const auto& info : collisionPairs_ | std::views::values) {
            totalContacts += info.manifold.numContacts;
        }
        stats_.numContacts = totalContacts;
    }
} // namespace engine::physics
