/**
 * @file PhysicsQueries.cpp
 * @brief Physics query and spatial search system
 * @details Provides various query methods for physics world interaction
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#include "PhysicsQueries.h"

#include "../core/PhysicsWorld.h"
#include "../integration/BulletIntegration.h"

namespace engine::physics {
    PhysicsQueries::PhysicsQueries(PhysicsWorld* world) : world_(world) {
    }

    bool PhysicsQueries::raycast(const Vec3& from, const Vec3& to, RaycastHit& hit,
                                 const QueryFilter& filter) const {
        if (!world_) return false;

        const btVector3 rayFrom = toBulletVector(from);
        const btVector3 rayTo = toBulletVector(to);

        btCollisionWorld::ClosestRayResultCallback rayCallback(rayFrom, rayTo);
        rayCallback.m_collisionFilterGroup = filter.groupMask;
        rayCallback.m_collisionFilterMask = filter.categoryMask;

        world_->getBulletWorld()->rayTest(rayFrom, rayTo, rayCallback);

        if (rayCallback.hasHit()) {
            hit.body = static_cast<RigidBody*>(rayCallback.m_collisionObject->getUserPointer());
            hit.point = fromBulletVector(rayCallback.m_hitPointWorld);
            hit.normal = fromBulletVector(rayCallback.m_hitNormalWorld);
            hit.distance = glm::length(hit.point - from);

            return filter.shouldTest(hit.body);
        }

        return false;
    }

    std::vector<RaycastHit> PhysicsQueries::raycastAll(const Vec3& from, const Vec3& to,
                                                       const QueryFilter& filter) const {
        std::vector<RaycastHit> hits;
        if (!world_) return hits;

        const btVector3 rayFrom = toBulletVector(from);
        const btVector3 rayTo = toBulletVector(to);

        btCollisionWorld::AllHitsRayResultCallback rayCallback(rayFrom, rayTo);
        rayCallback.m_collisionFilterGroup = filter.groupMask;
        rayCallback.m_collisionFilterMask = filter.categoryMask;

        world_->getBulletWorld()->rayTest(rayFrom, rayTo, rayCallback);

        for (int i = 0; i < rayCallback.m_collisionObjects.size(); ++i) {
            const auto body = static_cast<RigidBody*>(
                rayCallback.m_collisionObjects[i]->getUserPointer());

            if (filter.shouldTest(body)) {
                RaycastHit hit;
                hit.body = body;
                hit.point = fromBulletVector(rayCallback.m_hitPointWorld[i]);
                hit.normal = fromBulletVector(rayCallback.m_hitNormalWorld[i]);
                hit.distance = glm::length(hit.point - from);
                hits.push_back(hit);
            }
        }

        // Sort by distance
        std::ranges::sort(hits,
                          [](const RaycastHit& a, const RaycastHit& b) {
                              return a.distance < b.distance;
                          });

        return hits;
    }

    std::vector<RigidBody*> PhysicsQueries::overlapSphere(const Vec3& center, Float radius,
                                                          const QueryFilter& filter) const {
        std::vector<RigidBody*> bodies;
        if (!world_) return bodies;

        btSphereShape sphere(radius);

        btCollisionObject obj;
        obj.setCollisionShape(&sphere);

        btTransform transform;
        transform.setIdentity();
        transform.setOrigin(toBulletVector(center));

        obj.setWorldTransform(transform);

        ContactResultCallback callback(bodies, filter);
        world_->getBulletWorld()->contactTest(&obj, callback);

        return bodies;
    }

    std::vector<RigidBody*> PhysicsQueries::overlapBox(const Vec3& center, const Vec3& halfExtents,
                                                       const Quat& rotation,
                                                       const QueryFilter& filter) const {
        std::vector<RigidBody*> bodies;
        if (!world_) return bodies;

        btBoxShape box(toBulletVector(halfExtents));

        btCollisionObject obj;
        obj.setCollisionShape(&box);

        btTransform transform;
        transform.setOrigin(toBulletVector(center));
        transform.setRotation(toBulletQuaternion(rotation));
        obj.setWorldTransform(transform);

        ContactResultCallback callback(bodies, filter);
        world_->getBulletWorld()->contactTest(&obj, callback);

        return bodies;
    }

    std::vector<RigidBody*> PhysicsQueries::overlapCapsule(const Vec3& center, Float radius, Float height,
                                                           const Quat& rotation,
                                                           const QueryFilter& filter) const {
        std::vector<RigidBody*> bodies;
        if (!world_) return bodies;

        btCapsuleShape capsule(radius, height);

        btCollisionObject obj;
        obj.setCollisionShape(&capsule);

        btTransform transform;
        transform.setOrigin(toBulletVector(center));
        transform.setRotation(toBulletQuaternion(rotation));
        obj.setWorldTransform(transform);

        ContactResultCallback callback(bodies, filter);
        world_->getBulletWorld()->contactTest(&obj, callback);

        return bodies;
    }

    bool PhysicsQueries::sweepSphere(const Vec3& from, const Vec3& to, const Float radius,
                                     SweepResult& result, const QueryFilter& filter) const {
        if (!world_) return false;

        const btSphereShape sphere(radius);

        btTransform transformFrom;
        transformFrom.setIdentity();
        transformFrom.setOrigin(toBulletVector(from));

        btTransform transformTo;
        transformTo.setIdentity();
        transformTo.setOrigin(toBulletVector(to));

        btCollisionWorld::ClosestConvexResultCallback sweepCallback(
            toBulletVector(from), toBulletVector(to));
        sweepCallback.m_collisionFilterGroup = filter.groupMask;
        sweepCallback.m_collisionFilterMask = filter.categoryMask;

        world_->getBulletWorld()->convexSweepTest(
            &sphere, transformFrom, transformTo, sweepCallback);

        if (sweepCallback.hasHit()) {
            result.body = static_cast<RigidBody*>(
                sweepCallback.m_hitCollisionObject->getUserPointer());
            result.point = fromBulletVector(sweepCallback.m_hitPointWorld);
            result.normal = fromBulletVector(sweepCallback.m_hitNormalWorld);
            result.fraction = sweepCallback.m_closestHitFraction;

            return filter.shouldTest(result.body);
        }

        return false;
    }

    bool PhysicsQueries::sweepBox(const Vec3& from, const Vec3& to, const Vec3& halfExtents,
                                  const Quat& rotation, SweepResult& result,
                                  const QueryFilter& filter) const {
        if (!world_) return false;

        const btBoxShape box(toBulletVector(halfExtents));

        btTransform transformFrom;
        transformFrom.setOrigin(toBulletVector(from));
        transformFrom.setRotation(toBulletQuaternion(rotation));

        btTransform transformTo;
        transformTo.setOrigin(toBulletVector(to));
        transformTo.setRotation(toBulletQuaternion(rotation));

        btCollisionWorld::ClosestConvexResultCallback sweepCallback(
            toBulletVector(from), toBulletVector(to));
        sweepCallback.m_collisionFilterGroup = filter.groupMask;
        sweepCallback.m_collisionFilterMask = filter.categoryMask;

        world_->getBulletWorld()->convexSweepTest(
            &box, transformFrom, transformTo, sweepCallback);

        if (sweepCallback.hasHit()) {
            result.body = static_cast<RigidBody*>(
                sweepCallback.m_hitCollisionObject->getUserPointer());
            result.point = fromBulletVector(sweepCallback.m_hitPointWorld);
            result.normal = fromBulletVector(sweepCallback.m_hitNormalWorld);
            result.fraction = sweepCallback.m_closestHitFraction;

            return filter.shouldTest(result.body);
        }

        return false;
    }

    std::vector<ContactResult> PhysicsQueries::getContacts(const RigidBody* body) const {
        std::vector<ContactResult> contacts;
        if (!world_ || !body) return contacts;

        // Iterate through manifolds
        btDispatcher* dispatcher = world_->getBulletWorld()->getDispatcher();
        const int numManifolds = dispatcher->getNumManifolds();

        for (int i = 0; i < numManifolds; ++i) {
            btPersistentManifold* manifold = dispatcher->getManifoldByIndexInternal(i);

            const btCollisionObject* obj0 = manifold->getBody0();

            if (const btCollisionObject* obj1 = manifold->getBody1(); obj0 == body->getBulletBody() || obj1 == body->
                getBulletBody()) {
                ContactResult result;
                result.bodyA = static_cast<RigidBody*>(obj0->getUserPointer());
                result.bodyB = static_cast<RigidBody*>(obj1->getUserPointer());

                for (int j = 0; j < manifold->getNumContacts(); ++j) {
                    const btManifoldPoint& pt = manifold->getContactPoint(j);

                    ContactPoint contact;
                    contact.positionWorldOnA = fromBulletVector(pt.getPositionWorldOnA());
                    contact.positionWorldOnB = fromBulletVector(pt.getPositionWorldOnB());
                    contact.normalWorldOnB = fromBulletVector(pt.m_normalWorldOnB);
                    contact.distance = pt.getDistance(); // penetración (negativo si se solapan)
                    contact.impulse = pt.getAppliedImpulse();
                    contact.combinedFriction = pt.m_combinedFriction;
                    contact.combinedRestitution = pt.m_combinedRestitution;

                    contact.bodyA = static_cast<RigidBody*>(obj0->getUserPointer());
                    contact.bodyB = static_cast<RigidBody*>(obj1->getUserPointer());

                    result.contacts.push_back(contact);
                    result.totalImpulse += pt.getAppliedImpulse();
                }

                if (!result.contacts.empty()) {
                    contacts.push_back(result);
                }
            }
        }

        return contacts;
    }

    bool PhysicsQueries::areInContact(const RigidBody* bodyA, const RigidBody* bodyB) const {
        if (!world_ || !bodyA || !bodyB) return false;

        btDispatcher* dispatcher = world_->getBulletWorld()->getDispatcher();
        const int numManifolds = dispatcher->getNumManifolds();

        for (int i = 0; i < numManifolds; ++i) {
            const btPersistentManifold* manifold = dispatcher->getManifoldByIndexInternal(i);

            const btCollisionObject* obj0 = manifold->getBody0();

            if (const btCollisionObject* obj1 = manifold->getBody1(); (obj0 == bodyA->getBulletBody() && obj1 == bodyB->
                    getBulletBody()) ||
                (obj0 == bodyB->getBulletBody() && obj1 == bodyA->getBulletBody())) {
                return manifold->getNumContacts() > 0;
            }
        }

        return false;
    }

    std::vector<RigidBody*> PhysicsQueries::getBodiesInAABB(const AABB& aabb,
                                                            const QueryFilter& filter) const {
        std::vector<RigidBody*> bodies;
        if (!world_) return bodies;

        struct AABBCallback : public btBroadphaseAabbCallback {
            std::vector<RigidBody*>& bodies;
            const QueryFilter& filter;

            AABBCallback(std::vector<RigidBody*>& b, const QueryFilter& f)
                : bodies(b), filter(f) {
            }

            bool process(const btBroadphaseProxy* proxy) override {
                const auto obj = static_cast<btCollisionObject*>(proxy->m_clientObject);

                if (const auto body = static_cast<RigidBody*>(obj->getUserPointer()); filter.shouldTest(body)) {
                    bodies.push_back(body);
                }

                return true;
            }
        };

        AABBCallback callback(bodies, filter);

        world_->getBulletWorld()->getBroadphase()->aabbTest(
            toBulletVector(aabb.min),
            toBulletVector(aabb.max),
            callback
        );

        return bodies;
    }

    RigidBody* PhysicsQueries::getNearestBody(const Vec3& point, const Float maxDistance,
                                  const QueryFilter& filter) {
        const AABB searchBox(
            point - Vec3(maxDistance),
            point + Vec3(maxDistance)
        );

        const std::vector<RigidBody*> candidates = getBodiesInAABB(searchBox, filter);

        RigidBody* nearest = nullptr;
        Float nearestDist = maxDistance * maxDistance;

        for (RigidBody* body : candidates) {
            if (Float dist = glm::length2(body->getPosition() - point); dist < nearestDist) {
                nearestDist = dist;
                nearest = body;
            }
        }

        return nearest;
    }

    btScalar PhysicsQueries::ContactResultCallback::addSingleResult(btManifoldPoint& cp,
        const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0,
        const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1) {
        const auto body = static_cast<RigidBody*>(
            colObj1Wrap->getCollisionObject()->getUserPointer());

        if (filter.shouldTest(body)) {
            bodies.push_back(body);
        }

        return 0;
    }
} // namespace engine::physics
