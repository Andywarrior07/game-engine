/**
 * @file NarrowPhase.h
 * @brief Narrow phase collision detection algorithms
 * @details Implements GJK, EPA, and SAT algorithms for precise collision
 *          detection and contact generation between collision shapes
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

#include "../core/PhysicsTypes.h"

// #include <algorithm>
// #include <limits>

namespace engine::physics {
    class CapsuleShape;
    class BoxShape;
    class SphereShape;
    /**
     * @brief Support point for GJK/EPA algorithms
     */
    struct SupportPoint {
        Vec3 point; // Point in Minkowski difference
        Vec3 supportA; // Support point on shape A
        Vec3 supportB; // Support point on shape B

        SupportPoint() = default;

        SupportPoint(const Vec3& a, const Vec3& b)
            : point(a - b), supportA(a), supportB(b) {
        }
    };

    /**
     * @brief Simplex for GJK algorithm
     */
    class Simplex {
    public:
        // TODO: Revisar el parametro que falta inicializar
        Simplex() : size_(0) {
        }

        void add(const SupportPoint& point);

        void remove(int index);

        void clear() { size_ = 0; }

        SupportPoint& operator[](const int index) { return points_[index]; }
        const SupportPoint& operator[](const int index) const { return points_[index]; }

        int size() const { return size_; }

    private:
        SupportPoint points_[4]; // Maximum 4 points for 3D simplex
        int size_;
    };

    /**
     * @brief Narrow phase collision detector
     * @details Implements various algorithms for precise collision detection
     */
    class NarrowPhase {
    public:
        NarrowPhase() : gjkMaxIterations_(64), epaMaxIterations_(64),
                        gjkTolerance_(0.0001f), epaTolerance_(0.0001f) {
        }

        // ============================================================================
        // Main Collision Detection
        // ============================================================================

        /**
         * @brief Test collision between two shapes
         */
        bool testCollision(CollisionShape* shapeA, Transform& transformA,
                           CollisionShape* shapeB, Transform& transformB,
                           CollisionManifold& manifold);

        /**
         * @brief Get closest points between two shapes
         */
        bool getClosestPoints(CollisionShape* shapeA, const Transform& transformA,
                              CollisionShape* shapeB, const Transform& transformB,
                              Vec3& pointA, Vec3& pointB, Float& distance);

        /**
         * @brief Test if point is inside shape
         */
        bool pointInShape(const Vec3& point, CollisionShape* shape, const Transform& transform);

        // ============================================================================
        // GJK Algorithm
        // ============================================================================

        /**
         * @brief GJK collision detection
         */
        bool gjk(CollisionShape* shapeA, const Transform& transformA,
                 CollisionShape* shapeB, const Transform& transformB,
                 Simplex& simplex);

        /**
         * @brief GJK distance query
         */
        bool gjkDistance(CollisionShape* shapeA, const Transform& transformA,
                         CollisionShape* shapeB, const Transform& transformB,
                         Vec3& closestPointA, Vec3& closestPointB, Float& distance);

        // ============================================================================
        // EPA Algorithm
        // ============================================================================

        /**
         * @brief EPA (Expanding Polytope Algorithm) for penetration depth
         */
        bool epa(CollisionShape* shapeA, const Transform& transformA,
                 CollisionShape* shapeB, const Transform& transformB,
                 const Simplex& gjkSimplex, Vec3& normal, Float& depth);

        // ============================================================================
        // Specialized Collision Tests
        // ============================================================================

        /**
         * @brief Sphere-sphere collision
         */
        static bool sphereSphereCollision(CollisionShape* shapeA, const Transform& transformA,
                                          CollisionShape* shapeB, const Transform& transformB,
                                          CollisionManifold& manifold);

        /**
         * @brief Box-box collision using SAT
         */
        bool boxBoxCollision(CollisionShape* shapeA, const Transform& transformA,
                             CollisionShape* shapeB, const Transform& transformB,
                             CollisionManifold& manifold);

        /**
         * @brief Sphere-box collision
         */
        static bool sphereBoxCollision(CollisionShape* shapeA, Transform& transformA,
                                       CollisionShape* shapeB, Transform& transformB,
                                       CollisionManifold& manifold);

    private:
        int gjkMaxIterations_;
        int epaMaxIterations_;
        Float gjkTolerance_;
        Float epaTolerance_;

        struct Face {
            int indices[3];
        };

        /**
         * @brief Get support point in given direction
         */
        static SupportPoint getSupportPoint(CollisionShape* shapeA, const Transform& transformA,
                                            CollisionShape* shapeB, const Transform& transformB,
                                            const Vec3& direction);

        /**
         * @brief Get furthest point on shape in given direction
         */
        static Vec3 getSupport(CollisionShape* shape, const Transform& transform, const Vec3& direction);

        /**
         * @brief Process simplex and determine new search direction
         */
        bool processSimplex(Simplex& simplex, Vec3& direction);

        static bool processLine(Simplex& simplex, Vec3& direction);

        static bool processTriangle(Simplex& simplex, Vec3& direction);

        static bool processTetrahedron(Simplex& simplex, Vec3& direction);

        // Helper functions for EPA and distance queries
        Vec3 getClosestPointOnSimplex(const Simplex& simplex);

        Vec3 getBarycentricCoordinates(const Simplex& simplex, const Vec3& point);

        void reduceSimplex(Simplex& simplex, const Vec3& closestPoint);

        void expandPolytope(std::vector<SupportPoint>& polytope,
                            std::vector<Face>& faces, int newPointIndex);

        Float getOverlapOnAxis(const OBB& obbA, const OBB& obbB, const Vec3& axis);

        void generateBoxBoxContacts(const OBB& obbA, const OBB& obbB,
                                    const Vec3& axis, Float penetration,
                                    CollisionManifold& manifold);

        // Point containment tests
        bool pointInSphere(const Vec3& point, SphereShape* sphere);

        static bool pointInBox(const Vec3& point, const BoxShape* box);

        static bool pointInCapsule(const Vec3& point, const CapsuleShape* capsule);

        bool pointInShapeGJK(const Vec3& point, CollisionShape* shape, const Transform& transform);

        bool gjkEpaCollision(CollisionShape* shapeA, const Transform& transformA,
                             CollisionShape* shapeB, const Transform& transformB,
                             CollisionManifold& manifold);
    };
} // namespace engine::physics
