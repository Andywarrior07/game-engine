/**
 * @file NarrowPhase.cpp
 * @brief Narrow phase collision detection algorithms
 * @details Implements GJK, EPA, and SAT algorithms for precise collision
 *          detection and contact generation between collision shapes
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#include "NarrowPhase.h"

#include "../collision/CollisionShapes.h"

// #include <BulletCollision/CollisionShapes/btConvexShape.h>

namespace engine::physics {
    void Simplex::add(const SupportPoint& point) {
        points_[size_++] = point;
    }

    void Simplex::remove(const int index) {
        for (int i = index; i < size_ - 1; ++i) {
            points_[i] = points_[i + 1];
        }
        size_--;
    }

    bool NarrowPhase::testCollision(CollisionShape* shapeA, Transform& transformA,
                                    CollisionShape* shapeB, Transform& transformB,
                                    CollisionManifold& manifold) {
        manifold.clear();

        // Choose algorithm based on shape types
        const ShapeType typeA = shapeA->getType();
        const ShapeType typeB = shapeB->getType();

        // Special case optimizations
        if (typeA == ShapeType::SPHERE && typeB == ShapeType::SPHERE) {
            return sphereSphereCollision(shapeA, transformA, shapeB, transformB, manifold);
        }

        if (typeA == ShapeType::BOX && typeB == ShapeType::BOX) {
            return boxBoxCollision(shapeA, transformA, shapeB, transformB, manifold);
        }

        if ((typeA == ShapeType::SPHERE && typeB == ShapeType::BOX) ||
            (typeA == ShapeType::BOX && typeB == ShapeType::SPHERE)) {
            return sphereBoxCollision(shapeA, transformA, shapeB, transformB, manifold);
        }

        // General case - use GJK/EPA
        return gjkEpaCollision(shapeA, transformA, shapeB, transformB, manifold);
    }

    bool NarrowPhase::getClosestPoints(CollisionShape* shapeA, const Transform& transformA,
                                       CollisionShape* shapeB, const Transform& transformB,
                                       Vec3& pointA, Vec3& pointB, Float& distance) {
        // Use GJK for distance query
        return gjkDistance(shapeA, transformA, shapeB, transformB, pointA, pointB, distance);
    }

    bool NarrowPhase::pointInShape(const Vec3& point, CollisionShape* shape, const Transform& transform) {
        // Transform point to local space
        const Mat4 worldToLocal = glm::inverse(transform.getWorldMatrix());
        const auto localPoint = Vec3(worldToLocal * Vec4(point, 1.0f));

        switch (shape->getType()) {
        case ShapeType::SPHERE:
            return pointInSphere(localPoint, static_cast<SphereShape*>(shape));
        case ShapeType::BOX:
            return pointInBox(localPoint, static_cast<BoxShape*>(shape));
        case ShapeType::CAPSULE:
            return pointInCapsule(localPoint, static_cast<CapsuleShape*>(shape));
        default:
            // Use GJK for general shapes
            return pointInShapeGJK(point, shape, transform);
        }
    }

    bool NarrowPhase::gjk(CollisionShape* shapeA, const Transform& transformA,
                          CollisionShape* shapeB, const Transform& transformB,
                          Simplex& simplex) {
        // Initial direction
        Vec3 direction = transformB.getPosition() - transformA.getPosition();
        if (glm::length2(direction) < EPSILON_SQUARED) {
            direction = Vec3(1, 0, 0);
        }

        // Get initial support point
        SupportPoint support = getSupportPoint(shapeA, transformA, shapeB, transformB, direction);
        simplex.add(support);

        // New search direction
        direction = -support.point;

        int iterations = 0;
        while (iterations++ < gjkMaxIterations_) {
            // If direction is zero, shapes are touching
            if (glm::length2(direction) < EPSILON_SQUARED) {
                return true;
            }

            // Get new support point
            support = getSupportPoint(shapeA, transformA, shapeB, transformB, direction);

            // Check if we've passed the origin
            if (glm::dot(support.point, direction) < 0) {
                return false; // No collision
            }

            simplex.add(support);

            // Process simplex and get new direction
            if (processSimplex(simplex, direction)) {
                return true; // Collision detected
            }
        }

        return false; // Max iterations reached
    }

    bool NarrowPhase::gjkDistance(CollisionShape* shapeA, const Transform& transformA,
                                  CollisionShape* shapeB, const Transform& transformB,
                                  Vec3& closestPointA, Vec3& closestPointB, Float& distance) {
        Simplex simplex;
        Vec3 direction = transformB.getPosition() - transformA.getPosition();

        if (glm::length2(direction) < EPSILON_SQUARED) {
            direction = Vec3(1, 0, 0);
        }

        // Build initial simplex
        SupportPoint support = getSupportPoint(shapeA, transformA, shapeB, transformB, direction);
        simplex.add(support);

        Vec3 closestPoint = support.point;
        Float lastDistance = glm::length(closestPoint);

        int iterations = 0;
        while (iterations++ < gjkMaxIterations_) {
            direction = -closestPoint;

            if (glm::length2(direction) < EPSILON_SQUARED) {
                // Origins coincide
                distance = 0;
                closestPointA = closestPointB = transformA.getPosition();
                return true;
            }

            direction = glm::normalize(direction);
            support = getSupportPoint(shapeA, transformA, shapeB, transformB, direction);

            Float projection = glm::dot(support.point, direction);
            if (projection - lastDistance < gjkTolerance_) {
                // Converged
                distance = lastDistance;

                // Calculate closest points
                Vec3 baryCoords = getBarycentricCoordinates(simplex, closestPoint);
                closestPointA = closestPointB = Vec3(0);

                for (int i = 0; i < simplex.size(); ++i) {
                    closestPointA += simplex[i].supportA * baryCoords[i];
                    closestPointB += simplex[i].supportB * baryCoords[i];
                }

                return true;
            }

            simplex.add(support);

            // Find closest point on simplex to origin
            closestPoint = getClosestPointOnSimplex(simplex);
            lastDistance = glm::length(closestPoint);

            // Reduce simplex to minimum set
            reduceSimplex(simplex, closestPoint);
        }

        return false; // Failed to converge
    }

    bool NarrowPhase::epa(CollisionShape* shapeA, const Transform& transformA,
                          CollisionShape* shapeB, const Transform& transformB,
                          const Simplex& gjkSimplex, Vec3& normal, Float& depth) {
        // Initialize polytope from GJK simplex
        std::vector<SupportPoint> polytope;
        std::vector<Face> faces;

        // Convert simplex to polytope
        for (int i = 0; i < gjkSimplex.size(); ++i) {
            polytope.push_back(gjkSimplex[i]);
        }

        // Build initial faces
        if (gjkSimplex.size() == 4) {
            // Tetrahedron
            faces.push_back({0, 1, 2});
            faces.push_back({0, 2, 3});
            faces.push_back({0, 3, 1});
            faces.push_back({1, 3, 2});
        }

        int iterations = 0;
        while (iterations++ < epaMaxIterations_) {
            // Find closest face to origin
            int closestFaceIndex = -1;
            Float minDistance = INFINITY_VALUE<Float>;
            Vec3 closestNormal;

            for (int i = 0; i < faces.size(); ++i) {
                const auto& [indices] = faces[i];
                Vec3 a = polytope[indices[0]].point;
                Vec3 b = polytope[indices[1]].point;
                Vec3 c = polytope[indices[2]].point;

                Vec3 faceNormal = glm::normalize(glm::cross(b - a, c - a));

                if (Float distance = glm::dot(faceNormal, a); distance < minDistance) {
                    minDistance = distance;
                    closestFaceIndex = i;
                    closestNormal = faceNormal;
                }
            }

            // Get support point in direction of closest face
            SupportPoint support = getSupportPoint(shapeA, transformA, shapeB, transformB,
                                                   closestNormal);

            if (Float projection = glm::dot(support.point, closestNormal); projection - minDistance < epaTolerance_) {
                // Converged
                normal = closestNormal;
                depth = minDistance;
                return true;
            }

            // Expand polytope
            polytope.push_back(support);

            // Remove faces that can see the new point and add new faces
            expandPolytope(polytope, faces, polytope.size() - 1);
        }

        return false; // Failed to converge
    }

    bool NarrowPhase::sphereSphereCollision(CollisionShape* shapeA, const Transform& transformA,
                                            CollisionShape* shapeB, const Transform& transformB,
                                            CollisionManifold& manifold) {
        auto sphereA = static_cast<SphereShape*>(shapeA);
        auto sphereB = static_cast<SphereShape*>(shapeB);

        Vec3 centerA = transformA.getPosition();
        Vec3 centerB = transformB.getPosition();
        Float radiusA = sphereA->getRadius() * glm::length(transformA.getScale());
        Float radiusB = sphereB->getRadius() * glm::length(transformB.getScale());

        Vec3 diff = centerB - centerA;
        Float distanceSq = glm::length2(diff);
        Float radiusSum = radiusA + radiusB;

        if (distanceSq > radiusSum * radiusSum) {
            return false; // No collision
        }

        Float distance = std::sqrt(distanceSq);
        Vec3 normal = distance > EPSILON ? diff / distance : Vec3(0, 1, 0);

        ContactPoint contact;
        contact.normalWorldOnB = normal;
        contact.distance = distance - radiusSum;
        contact.positionWorldOnA = centerA + normal * radiusA;
        contact.positionWorldOnB = centerB - normal * radiusB;

        manifold.addContactPoint(contact);
        return true;
    }

    bool NarrowPhase::boxBoxCollision(CollisionShape* shapeA, const Transform& transformA,
                                      CollisionShape* shapeB, const Transform& transformB,
                                      CollisionManifold& manifold) {
        auto boxA = static_cast<BoxShape*>(shapeA);
        auto boxB = static_cast<BoxShape*>(shapeB);

        // Get OBBs from transforms
        OBB obbA(transformA.getPosition(),
                 boxA->getBoxHalfExtents() * transformA.getScale(),
                 transformA.getRotation());
        OBB obbB(transformB.getPosition(),
                 boxB->getBoxHalfExtents() * transformB.getScale(),
                 transformB.getRotation());

        // SAT test
        Float minOverlap = INFINITY_VALUE<Float>;
        Vec3 minAxis;

        // Test axes from box A
        auto axesA = obbA.getAxes();
        for (const auto& axis : axesA) {
            Float overlap = getOverlapOnAxis(obbA, obbB, axis);
            if (overlap < 0) return false; // Separating axis found

            if (overlap < minOverlap) {
                minOverlap = overlap;
                minAxis = axis;
            }
        }

        // Test axes from box B
        auto axesB = obbB.getAxes();
        for (const auto& axis : axesB) {
            Float overlap = getOverlapOnAxis(obbA, obbB, axis);
            if (overlap < 0) return false;

            if (overlap < minOverlap) {
                minOverlap = overlap;
                minAxis = axis;
            }
        }

        // Test cross product axes
        for (const auto& axisA : axesA) {
            for (const auto& axisB : axesB) {
                Vec3 axis = glm::cross(axisA, axisB);
                if (glm::length2(axis) < EPSILON_SQUARED) continue;

                axis = glm::normalize(axis);
                Float overlap = getOverlapOnAxis(obbA, obbB, axis);
                if (overlap < 0) return false;

                if (overlap < minOverlap) {
                    minOverlap = overlap;
                    minAxis = axis;
                }
            }
        }

        // Generate contact points
        generateBoxBoxContacts(obbA, obbB, minAxis, minOverlap, manifold);

        return true;
    }

    bool NarrowPhase::sphereBoxCollision(CollisionShape* shapeA, Transform& transformA,
                                         CollisionShape* shapeB, Transform& transformB,
                                         CollisionManifold& manifold) {
        // Ensure sphere is A and box is B
        bool swapped = shapeA->getType() != ShapeType::SPHERE;
        if (swapped) {
            std::swap(shapeA, shapeB);
            std::swap(transformA, transformB);
        }

        auto sphere = static_cast<SphereShape*>(shapeA);
        auto box = static_cast<BoxShape*>(shapeB);

        Vec3 sphereCenter = transformA.getPosition();
        Float sphereRadius = sphere->getRadius() * glm::length(transformA.getScale());

        // Transform sphere center to box's local space
        Mat4 worldToBox = glm::inverse(transformB.getWorldMatrix());
        auto localSphereCenter = Vec3(worldToBox * Vec4(sphereCenter, 1.0f));

        // Find closest point on box to sphere center
        Vec3 boxHalfExtents = box->getBoxHalfExtents();
        Vec3 closestPoint = glm::clamp(localSphereCenter, -boxHalfExtents, boxHalfExtents);

        // Check if closest point is inside sphere
        Vec3 diff = localSphereCenter - closestPoint;
        Float distanceSq = glm::length2(diff);

        if (distanceSq > sphereRadius * sphereRadius) {
            return false; // No collision
        }

        // Transform closest point back to world space
        auto worldClosestPoint = Vec3(transformB.getWorldMatrix() * Vec4(closestPoint, 1.0f));

        Vec3 normal;
        Float penetration;

        if (distanceSq < EPSILON_SQUARED) {
            // Sphere center inside box
            // Find face normal with minimum penetration
            Float minPenetration = INFINITY_VALUE<Float>;
            int minFace = -1;

            for (int i = 0; i < 3; ++i) {
                Float penetrationPos = boxHalfExtents[i] - localSphereCenter[i];
                Float penetrationNeg = boxHalfExtents[i] + localSphereCenter[i];

                if (penetrationPos < minPenetration) {
                    minPenetration = penetrationPos;
                    minFace = i;
                }
                if (penetrationNeg < minPenetration) {
                    minPenetration = penetrationNeg;
                    minFace = i + 3;
                }
            }

            normal = Vec3(0);
            normal[minFace % 3] = minFace < 3 ? 1.0f : -1.0f;
            normal = glm::normalize(Vec3(transformB.getRotation() * normal));
            penetration = -(minPenetration + sphereRadius);
        }
        else {
            Float distance = std::sqrt(distanceSq);
            normal = (sphereCenter - worldClosestPoint) / distance;
            penetration = distance - sphereRadius;
        }

        ContactPoint contact;
        contact.normalWorldOnB = swapped ? -normal : normal;
        contact.distance = penetration;
        contact.positionWorldOnA = swapped ? worldClosestPoint : (sphereCenter - normal * sphereRadius);
        contact.positionWorldOnB = swapped ? (sphereCenter + normal * sphereRadius) : worldClosestPoint;

        manifold.addContactPoint(contact);
        return true;
    }

    SupportPoint NarrowPhase::getSupportPoint(CollisionShape* shapeA, const Transform& transformA,
                                              CollisionShape* shapeB, const Transform& transformB,
                                              const Vec3& direction) {
        const Vec3 supportA = getSupport(shapeA, transformA, direction);
        const Vec3 supportB = getSupport(shapeB, transformB, -direction);

        return SupportPoint(supportA, supportB);
    }

    Vec3 NarrowPhase::getSupport(CollisionShape* shape, const Transform& transform, const Vec3& direction) {
        // Transform direction to local space
        const Mat4 worldToLocal = glm::inverse(transform.getWorldMatrix());
        const Vec3 localDir = glm::normalize(Vec3(worldToLocal * Vec4(direction, 0.0f)));

        Vec3 localSupport;

        switch (shape->getType()) {
        case ShapeType::SPHERE: {
            const auto sphere = static_cast<SphereShape*>(shape);
            localSupport = localDir * sphere->getRadius();
            break;
        }

        case ShapeType::BOX: {
            const auto box = static_cast<BoxShape*>(shape);
            const Vec3 halfExtents = box->getBoxHalfExtents();

            localSupport = Vec3(
                localDir.x > 0 ? halfExtents.x : -halfExtents.x,
                localDir.y > 0 ? halfExtents.y : -halfExtents.y,
                localDir.z > 0 ? halfExtents.z : -halfExtents.z
            );
            break;
        }

        case ShapeType::CAPSULE: {
            const auto capsule = static_cast<CapsuleShape*>(shape);
            const Float radius = capsule->getRadius();
            const Float halfHeight = capsule->getHeight() * 0.5f;

            const auto capCenter = Vec3(0, localDir.y > 0 ? halfHeight : -halfHeight, 0);
            localSupport = capCenter + localDir * radius;

            break;
        }

        default: {
            btVector3 btDir(localDir.x, localDir.y, localDir.z);

            btCollisionShape* btShape = shape->getBulletShape();

            // 1) Intentar upcast a btConvexShape (no requiere RTTI)
            if (auto* convex = dynamic_cast<btConvexShape*>(btShape); convex) {
                btVector3 btSupport = convex->localGetSupportingVertex(btDir);
                localSupport = Vec3(btSupport.x(), btSupport.y(), btSupport.z());
            }
            else {
                // 2) Fallback: aproximación usando AABB (no es exacta pero evita fallos)
                btTransform id;
                id.setIdentity();
                btVector3 aabbMin, aabbMax;
                btShape->getAabb(id, aabbMin, aabbMax);

                localSupport = Vec3(
                    btDir.x() >= 0 ? aabbMax.x() : aabbMin.x(),
                    btDir.y() >= 0 ? aabbMax.y() : aabbMin.y(),
                    btDir.z() >= 0 ? aabbMax.z() : aabbMin.z()
                );
            }
            break;
        }
        }

        // Transform back to world space
        return Vec3(transform.getWorldMatrix() * Vec4(localSupport, 1.0f));
    }

    bool NarrowPhase::processSimplex(Simplex& simplex, Vec3& direction) {
        switch (simplex.size()) {
        case 2: return processLine(simplex, direction);
        case 3: return processTriangle(simplex, direction);
        case 4: return processTetrahedron(simplex, direction);
        default: return false;
        }
    }

    bool NarrowPhase::processLine(Simplex& simplex, Vec3& direction) {
        const Vec3 a = simplex[1].point;
        const Vec3 b = simplex[0].point;
        const Vec3 ab = b - a;

        if (const Vec3 ao = -a; glm::dot(ab, ao) > 0) {
            direction = glm::cross(glm::cross(ab, ao), ab);
        }
        else {
            simplex.remove(0);
            direction = ao;
        }

        return false;
    }

    bool NarrowPhase::processTriangle(Simplex& simplex, Vec3& direction) {
        Vec3 a = simplex[2].point;
        Vec3 b = simplex[1].point;
        Vec3 c = simplex[0].point;

        Vec3 ab = b - a;
        Vec3 ac = c - a;
        Vec3 ao = -a;

        if (Vec3 abc = glm::cross(ab, ac); glm::dot(glm::cross(abc, ac), ao) > 0) {
            if (glm::dot(ac, ao) > 0) {
                simplex.remove(1);
                direction = glm::cross(glm::cross(ac, ao), ac);
            }
            else {
                return processLine(simplex, direction);
            }
        }
        else {
            if (glm::dot(glm::cross(ab, abc), ao) > 0) {
                return processLine(simplex, direction);
            }

            if (glm::dot(abc, ao) > 0) {
                direction = abc;
            }

            else {
                std::swap(simplex[0], simplex[1]);
                direction = -abc;
            }
        }

        return false;
    }

    bool NarrowPhase::processTetrahedron(Simplex& simplex, Vec3& direction) {
        Vec3 a = simplex[3].point;
        Vec3 b = simplex[2].point;
        Vec3 c = simplex[1].point;
        Vec3 d = simplex[0].point;

        Vec3 ab = b - a;
        Vec3 ac = c - a;
        Vec3 ad = d - a;
        Vec3 ao = -a;

        Vec3 abc = glm::cross(ab, ac);
        Vec3 acd = glm::cross(ac, ad);
        Vec3 adb = glm::cross(ad, ab);

        if (glm::dot(abc, ao) > 0) {
            simplex.remove(0);
            return processTriangle(simplex, direction);
        }
        if (glm::dot(acd, ao) > 0) {
            simplex.remove(1);
            return processTriangle(simplex, direction);
        }
        if (glm::dot(adb, ao) > 0) {
            simplex.remove(2);
            return processTriangle(simplex, direction);
        }

        return true; // Origin is inside tetrahedron
    }

    Vec3 NarrowPhase::getClosestPointOnSimplex(const Simplex& simplex) {
        // Implementation depends on simplex size
        // Returns closest point on simplex to origin
        return Vec3(0); // Placeholder
    }

    Vec3 NarrowPhase::getBarycentricCoordinates(const Simplex& simplex, const Vec3& point) {
        // Calculate barycentric coordinates of point relative to simplex
        return Vec3(0); // Placeholder
    }

    void NarrowPhase::reduceSimplex(Simplex& simplex, const Vec3& closestPoint) {
        // Reduce simplex to minimum set that contains closest point
    }

    void NarrowPhase::expandPolytope(std::vector<SupportPoint>& polytope,
                            std::vector<Face>& faces, int newPointIndex) {
        // Expand polytope with new point
    }

    Float NarrowPhase::getOverlapOnAxis(const OBB& obbA, const OBB& obbB, const Vec3& axis) {
        // Project both OBBs onto axis and calculate overlap
        return 0.0f; // Placeholder
    }

    void NarrowPhase::generateBoxBoxContacts(const OBB& obbA, const OBB& obbB,
                                    const Vec3& axis, Float penetration,
                                    CollisionManifold& manifold) {
        // Generate contact points for box-box collision
    }

    bool NarrowPhase::pointInSphere(const Vec3& point, SphereShape* sphere) {
        return glm::length2(point) <= square(sphere->getRadius());
    }

    bool NarrowPhase::pointInBox(const Vec3& point, const BoxShape* box) {
        Vec3 halfExtents = box->getBoxHalfExtents();
        return std::abs(point.x) <= halfExtents.x &&
            std::abs(point.y) <= halfExtents.y &&
            std::abs(point.z) <= halfExtents.z;
    }

    bool NarrowPhase::pointInCapsule(const Vec3& point, const CapsuleShape* capsule) {
        const Float radius = capsule->getRadius();
        const Float halfHeight = capsule->getHeight() * 0.5f;

        // Check cylinder part
        if (std::abs(point.y) <= halfHeight) {
            return point.x * point.x + point.z * point.z <= radius * radius;
        }

        // Check hemisphere caps
        const Vec3 capCenter(0, point.y > 0 ? halfHeight : -halfHeight, 0);
        return glm::length2(point - capCenter) <= radius * radius;
    }

    bool NarrowPhase::pointInShapeGJK(const Vec3& point, CollisionShape* shape, const Transform& transform) {
        // Use GJK to test if point is inside arbitrary shape
        return false; // Placeholder
    }

    bool NarrowPhase::gjkEpaCollision(CollisionShape* shapeA, const Transform& transformA,
                             CollisionShape* shapeB, const Transform& transformB,
                             CollisionManifold& manifold) {
        Simplex simplex;

        // Run GJK
        if (!gjk(shapeA, transformA, shapeB, transformB, simplex)) {
            return false; // No collision
        }

        // Run EPA for penetration depth
        Float depth;
        if (Vec3 normal; epa(shapeA, transformA, shapeB, transformB, simplex, normal, depth)) {
            // Generate contact points
            ContactPoint contact;
            contact.normalWorldOnB = normal;
            contact.distance = -depth;

            // Calculate contact points (simplified)
            contact.positionWorldOnA = transformA.getPosition() - normal * depth * 0.5f;
            contact.positionWorldOnB = transformB.getPosition() + normal * depth * 0.5f;

            manifold.addContactPoint(contact);
            return true;
        }

        return false;
    }
} // namespace engine::physics
