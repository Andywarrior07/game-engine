/**
 * @file PhysicsQueries.h
 * @brief Physics query and spatial search system
 * @details Provides various query methods for physics world interaction
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

#include "../core/PhysicsTypes.h"
#include "../dynamics/RigidBody.h"

#include <vector>

namespace engine::physics {
    using namespace engine::math;

    /**
     * @brief Contact test result
     */
    struct ContactResult {
        RigidBody* bodyA = nullptr;
        RigidBody* bodyB = nullptr;
        std::vector<ContactPoint> contacts;
        Float totalImpulse = 0.0f;
    };

    /**
     * @brief Physics query system
     */
    class PhysicsQueries {
    public:
        explicit PhysicsQueries(PhysicsWorld* world);

        // ============================================================================
        // Raycast Queries
        // ============================================================================

        /**
         * @brief Cast a ray and get closest hit
         */
        bool raycast(const Vec3& from, const Vec3& to, RaycastHit& hit,
                     const QueryFilter& filter = QueryFilter()) const;

        /**
         * @brief Cast a ray and get all hits
         */
        std::vector<RaycastHit> raycastAll(const Vec3& from, const Vec3& to,
                                           const QueryFilter& filter = QueryFilter()) const;

        // ============================================================================
        // Shape Queries
        // ============================================================================

        /**
         * @brief Test sphere overlap
         */
        std::vector<RigidBody*> overlapSphere(const Vec3& center, Float radius,
                                              const QueryFilter& filter = QueryFilter()) const;

        /**
         * @brief Test box overlap
         */
        std::vector<RigidBody*> overlapBox(const Vec3& center, const Vec3& halfExtents,
                                           const Quat& rotation = Quat(1, 0, 0, 0),
                                           const QueryFilter& filter = QueryFilter()) const;

        /**
         * @brief Test capsule overlap
         */
        std::vector<RigidBody*> overlapCapsule(const Vec3& center, Float radius, Float height,
                                               const Quat& rotation = Quat(1, 0, 0, 0),
                                               const QueryFilter& filter = QueryFilter()) const;

        // ============================================================================
        // Sweep Queries
        // ============================================================================

        /**
         * @brief Sweep a sphere through space
         */
        bool sweepSphere(const Vec3& from, const Vec3& to, Float radius,
                         SweepResult& result, const QueryFilter& filter = QueryFilter()) const;

        /**
         * @brief Sweep a box through space
         */
        bool sweepBox(const Vec3& from, const Vec3& to, const Vec3& halfExtents,
                      const Quat& rotation, SweepResult& result,
                      const QueryFilter& filter = QueryFilter()) const;

        // ============================================================================
        // Contact Queries
        // ============================================================================

        /**
         * @brief Get all contacts for a body
         */
        std::vector<ContactResult> getContacts(const RigidBody* body) const;

        /**
         * @brief Test if two bodies are in contact
         */
        bool areInContact(const RigidBody* bodyA, const RigidBody* bodyB) const;

        // ============================================================================
        // Spatial Queries
        // ============================================================================

        /**
         * @brief Get all bodies within AABB
         */
        std::vector<RigidBody*> getBodiesInAABB(const AABB& aabb,
                                                const QueryFilter& filter = QueryFilter()) const;

        /**
         * @brief Get nearest body to point
         */
        RigidBody* getNearestBody(const Vec3& point, Float maxDistance,
                                  const QueryFilter& filter = QueryFilter());

    private:
        PhysicsWorld* world_;

        /**
         * @brief Callback for contact tests
         */
        struct ContactResultCallback final : btCollisionWorld::ContactResultCallback {
            std::vector<RigidBody*>& bodies;
            const QueryFilter& filter;

            ContactResultCallback(std::vector<RigidBody*>& b, const QueryFilter& f)
                : bodies(b), filter(f) {
            }

            btScalar addSingleResult(btManifoldPoint& cp,
                                     const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0,
                                     const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1) override;
        };
    };
} // namespace engine::physics
