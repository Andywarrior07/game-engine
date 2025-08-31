/**
 * @file Distance.h
 * @brief Distance and closest point calculations between geometric primitives
 *
 * Provides optimized distance queries and closest point computations
 * for physics simulations and proximity detection.
 */

#pragma once

#include "../core/MathTypes.h"
#include "../core/MathConstants.h"
#include "../core/MathFunctions.h"
#include "Primitives.h"

namespace engine::math {

    /**
     * @brief Result of a closest point query
     */
    struct ClosestPointResult {
        Vec3 pointA;       // Closest point on shape A
        Vec3 pointB;       // Closest point on shape B
        Float distance;     // Distance between closest points
        Float distanceSq;   // Squared distance (avoid sqrt when possible)

        ClosestPointResult() noexcept
            : pointA(0), pointB(0), distance(0), distanceSq(0) {}
    };

    // ============================================================================
    // Point to Primitive Distance
    // ============================================================================

    /**
     * @brief Distance from point to line segment
     */
    [[nodiscard]] inline ClosestPointResult distancePointToSegment(
        const Vec3& point, const LineSegment& segment) noexcept {

        const Vec3 ab = segment.end - segment.start;
        Float t = glm::dot(point - segment.start, ab) / glm::dot(ab, ab);
        t = saturate(t);

        ClosestPointResult result;
        result.pointB = segment.start + ab * t;
        result.pointA = point;
        result.distanceSq = glm::length2(point - result.pointB);
        result.distance = std::sqrt(result.distanceSq);

        return result;
    }

    /**
     * @brief Distance from point to plane
     */
    [[nodiscard]] inline Float distancePointToPlane(
        const Vec3& point, const Plane& plane) noexcept {
        return std::abs(plane.getSignedDistance(point));
    }

    /**
     * @brief Squared distance from point to AABB
     */
    [[nodiscard]] inline Float distancePointToAABBSq(
        const Vec3& point, const AABB& aabb) noexcept {
        const Vec3 closest = aabb.getClosestPoint(point);

        return glm::length2(point - closest);
    }

    /**
     * @brief Distance from point to AABB
     */
    [[nodiscard]] inline Float distancePointToAABB(
        const Vec3& point, const AABB& aabb) noexcept {
        return std::sqrt(distancePointToAABBSq(point, aabb));
    }

    /**
     * @brief Distance from point to sphere
     */
    [[nodiscard]] inline Float distancePointToSphere(
        const Vec3& point, const Sphere& sphere) noexcept {
        const Float dist = glm::length(point - sphere.center);

        return max(0.0f, dist - sphere.radius);
    }

    /**
     * @brief Distance from point to OBB
     */
    [[nodiscard]] inline Float distancePointToOBB(
        const Vec3& point, const OBB& obb) noexcept {

        // Transform point to OBB's local space
        const Mat4 worldToLocal = glm::inverse(obb.getTransformMatrix());
        const auto localPoint = Vec3(worldToLocal * Vec4(point, 1.0f));

        // In local space, OBB is an AABB centered at origin
        const AABB localAABB(-obb.halfExtents, obb.halfExtents);
        return distancePointToAABB(localPoint, localAABB);
    }

    /**
     * @brief Distance from point to triangle
     */
    [[nodiscard]] inline ClosestPointResult distancePointToTriangle(
        const Vec3& point, const Triangle& triangle) noexcept {

        // Compute vectors
        Vec3 ab = triangle.v1 - triangle.v0;
        Vec3 ac = triangle.v2 - triangle.v0;
        Vec3 ap = point - triangle.v0;

        // Compute barycentric coordinates
        Float d1 = glm::dot(ab, ap);
        Float d2 = glm::dot(ac, ap);
        if (d1 <= 0.0f && d2 <= 0.0f) {
            // Closest to vertex v0
            ClosestPointResult result;
            result.pointA = point;
            result.pointB = triangle.v0;
            result.distanceSq = glm::length2(point - triangle.v0);
            result.distance = std::sqrt(result.distanceSq);
            return result;
        }

        Vec3 bp = point - triangle.v1;
        Float d3 = glm::dot(ab, bp);
        Float d4 = glm::dot(ac, bp);
        if (d3 >= 0.0f && d4 <= d3) {
            // Closest to vertex v1
            ClosestPointResult result;
            result.pointA = point;
            result.pointB = triangle.v1;
            result.distanceSq = glm::length2(point - triangle.v1);
            result.distance = std::sqrt(result.distanceSq);
            return result;
        }

        Float vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
            // Closest to edge v0-v1
            Float v = d1 / (d1 - d3);
            ClosestPointResult result;
            result.pointA = point;
            result.pointB = triangle.v0 + v * ab;
            result.distanceSq = glm::length2(point - result.pointB);
            result.distance = std::sqrt(result.distanceSq);
            return result;
        }

        Vec3 cp = point - triangle.v2;
        Float d5 = glm::dot(ab, cp);
        Float d6 = glm::dot(ac, cp);
        if (d6 >= 0.0f && d5 <= d6) {
            // Closest to vertex v2
            ClosestPointResult result;
            result.pointA = point;
            result.pointB = triangle.v2;
            result.distanceSq = glm::length2(point - triangle.v2);
            result.distance = std::sqrt(result.distanceSq);
            return result;
        }

        Float vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
            // Closest to edge v0-v2
            Float w = d2 / (d2 - d6);
            ClosestPointResult result;
            result.pointA = point;
            result.pointB = triangle.v0 + w * ac;
            result.distanceSq = glm::length2(point - result.pointB);
            result.distance = std::sqrt(result.distanceSq);
            return result;
        }

        Float va = d3 * d6 - d5 * d4;
        if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
            // Closest to edge v1-v2
            Float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            ClosestPointResult result;
            result.pointA = point;
            result.pointB = triangle.v1 + w * (triangle.v2 - triangle.v1);
            result.distanceSq = glm::length2(point - result.pointB);
            result.distance = std::sqrt(result.distanceSq);
            return result;
        }

        // Point projects inside triangle
        Float denom = 1.0f / (va + vb + vc);
        Float v = vb * denom;
        Float w = vc * denom;

        ClosestPointResult result;
        result.pointA = point;
        result.pointB = triangle.v0 + ab * v + ac * w;
        result.distanceSq = glm::length2(point - result.pointB);
        result.distance = std::sqrt(result.distanceSq);
        return result;
    }

    /**
     * @brief Distance from point to capsule
     */
    [[nodiscard]] inline Float distancePointToCapsule(
        const Vec3& point, const Capsule& capsule) noexcept {

        const ClosestPointResult segmentResult = distancePointToSegment(point, capsule.getSegment());
        return max(0.0f, segmentResult.distance - capsule.radius);
    }

    // ============================================================================
    // Primitive to Primitive Distance
    // ============================================================================

    /**
     * @brief Distance between two spheres
     */
    [[nodiscard]] inline Float distanceSphereSphere(
        const Sphere& a, const Sphere& b) noexcept {

        const Float centerDist = glm::length(b.center - a.center);
        return max(0.0f, centerDist - a.radius - b.radius);
    }

    /**
     * @brief Distance between sphere and AABB
     */
    [[nodiscard]] inline Float distanceSphereAABB(
        const Sphere& sphere, const AABB& aabb) noexcept {

        const Float distToBox = distancePointToAABB(sphere.center, aabb);
        return max(0.0f, distToBox - sphere.radius);
    }

    /**
     * @brief Distance between sphere and plane
     */
    [[nodiscard]] inline Float distanceSpherePlane(
        const Sphere& sphere, const Plane& plane) noexcept {

        const Float distToPlane = distancePointToPlane(sphere.center, plane);
        return max(0.0f, distToPlane - sphere.radius);
    }

    /**
     * @brief Distance between two AABBs
     */
    [[nodiscard]] inline Float distanceAABBAABB(
        const AABB& a, const AABB& b) noexcept {

        Vec3 delta;
        delta.x = max(0.0f, max(a.min.x - b.max.x, b.min.x - a.max.x));
        delta.y = max(0.0f, max(a.min.y - b.max.y, b.min.y - a.max.y));
        delta.z = max(0.0f, max(a.min.z - b.max.z, b.min.z - a.max.z));

        return glm::length(delta);
    }

    /**
     * @brief Distance between AABB and plane
     */
    [[nodiscard]] inline Float distanceAABBPlane(
        const AABB& aabb, const Plane& plane) noexcept {

        const Vec3 center = aabb.getCenter();
        const Vec3 extents = aabb.getHalfExtents();

        // Project box extents onto plane normal
        const Float projectedExtent = glm::dot(glm::abs(plane.normal), extents);
        const Float signedDist = plane.getSignedDistance(center);

        return max(0.0f, std::abs(signedDist) - projectedExtent);
    }

    /**
     * @brief Closest points between two line segments
     */
    [[nodiscard]] inline ClosestPointResult closestPointsSegmentSegment(
        const LineSegment& seg1, const LineSegment& seg2) noexcept {

        Vec3 d1 = seg1.getDirection();
        Vec3 d2 = seg2.getDirection();
        Vec3 r = seg1.start - seg2.start;

        Float a = glm::dot(d1, d1); // Squared length of seg1
        Float e = glm::dot(d2, d2); // Squared length of seg2
        Float f = glm::dot(d2, r);

        Float s, t;

        // Check if either or both segments degenerate into points
        if (a <= EPSILON && e <= EPSILON) {
            // Both segments degenerate into points
            s = t = 0.0f;
        } else if (a <= EPSILON) {
            // First segment degenerates into a point
            s = 0.0f;
            t = saturate(f / e);
        } else {
            Float c = glm::dot(d1, r);
            if (e <= EPSILON) {
                // Second segment degenerates into a point
                t = 0.0f;
                s = saturate(-c / a);
            } else {
                // General case
                Float b = glm::dot(d1, d2);

                // If segments not parallel, compute closest point on seg1 to seg2
                if (const Float denom = a * e - b * b; denom != 0.0f) {
                    s = saturate((b * f - c * e) / denom);
                } else {
                    s = 0.0f; // Use origin of seg1
                }

                // Compute point on seg2 closest to seg1(s)
                t = (b * s + f) / e;

                // If t out of range, clamp and recompute s
                if (t < 0.0f) {
                    t = 0.0f;
                    s = saturate(-c / a);
                } else if (t > 1.0f) {
                    t = 1.0f;
                    s = saturate((b - c) / a);
                }
            }
        }

        ClosestPointResult result;
        result.pointA = seg1.start + s * d1;
        result.pointB = seg2.start + t * d2;
        result.distanceSq = glm::length2(result.pointB - result.pointA);
        result.distance = std::sqrt(result.distanceSq);

        return result;
    }

    /**
     * @brief Distance between two capsules
     */
    [[nodiscard]] inline Float distanceCapsuleCapsule(
        const Capsule& a, const Capsule& b) noexcept {

        const ClosestPointResult segmentResult = closestPointsSegmentSegment(
            a.getSegment(), b.getSegment()
        );

        return max(0.0f, segmentResult.distance - a.radius - b.radius);
    }

    /**
     * @brief Distance between capsule and sphere
     */
    [[nodiscard]] inline Float distanceCapsuleSphere(
        const Capsule& capsule, const Sphere& sphere) noexcept {

        const ClosestPointResult segmentResult = distancePointToSegment(
            sphere.center, capsule.getSegment()
        );

        return max(0.0f, segmentResult.distance - capsule.radius - sphere.radius);
    }

    /**
     * @brief Distance between OBB and sphere
     */
    [[nodiscard]] inline Float distanceOBBSphere(
        const OBB& obb, const Sphere& sphere) noexcept {

        const Float distToOBB = distancePointToOBB(sphere.center, obb);
        return max(0.0f, distToOBB - sphere.radius);
    }

    /**
     * @brief Squared distance between two OBBs (approximate)
     * Note: Exact OBB-OBB distance is computationally expensive
     */
    [[nodiscard]] inline Float distanceOBBOBBApprox(
        const OBB& a, const OBB& b) noexcept {

        // Convert to AABBs for approximation
        const AABB aabbA = a.toAABB();
        const AABB aabbB = b.toAABB();
        return distanceAABBAABB(aabbA, aabbB);
    }

    // ============================================================================
    // Ray Distance Queries
    // ============================================================================

    /**
     * @brief Distance from ray to point
     */
    [[nodiscard]] inline Float distanceRayPoint(
        const Ray& ray, const Vec3& point) noexcept {

        const Vec3 ap = point - ray.origin;
        Float t = glm::dot(ap, ray.direction);
        t = clamp(t, 0.0f, ray.maxDistance);

        const Vec3 closestPoint = ray.getPoint(t);
        return glm::length(point - closestPoint);
    }

    /**
     * @brief Closest points between ray and line segment
     */
    [[nodiscard]] inline ClosestPointResult closestPointsRaySegment(
        const Ray& ray, const LineSegment& segment) noexcept {

        Vec3 segDir = segment.getDirection();
        Vec3 r = ray.origin - segment.start;

        Float a = glm::dot(ray.direction, ray.direction);
        Float b = glm::dot(ray.direction, segDir);
        Float c = glm::dot(segDir, segDir);
        Float d = glm::dot(ray.direction, r);
        Float e = glm::dot(segDir, r);

        Float det = a * c - b * b;
        Float s, t;

        if (std::abs(det) < EPSILON) {
            // Ray and segment are parallel
            s = 0.0f;
            t = e / c;
            t = saturate(t);
        } else {
            s = (b * e - c * d) / det;
            t = (a * e - b * d) / det;

            // Clamp to valid ranges
            s = clamp(s, 0.0f, ray.maxDistance);
            t = saturate(t);

            // Recompute if clamping occurred
            if (s == 0.0f) {
                t = saturate(e / c);
            } else if (s == ray.maxDistance) {
                Vec3 endPoint = ray.getPoint(ray.maxDistance);
                Vec3 ap = endPoint - segment.start;
                t = saturate(glm::dot(ap, segDir) / c);
            }

            if (t == 0.0f) {
                s = clamp(-d / a, 0.0f, ray.maxDistance);
            } else if (t == 1.0f) {
                Vec3 bp = ray.origin - segment.end;
                s = clamp(glm::dot(bp, ray.direction) / a, 0.0f, ray.maxDistance);
            }
        }

        ClosestPointResult result;
        result.pointA = ray.getPoint(s);
        result.pointB = segment.getPoint(t);
        result.distanceSq = glm::length2(result.pointB - result.pointA);
        result.distance = std::sqrt(result.distanceSq);

        return result;
    }

    // ============================================================================
    // Penetration Depth Calculations
    // ============================================================================

    /**
     * @brief Calculate penetration depth and direction for overlapping spheres
     */
    [[nodiscard]] inline std::pair<Vec3, Float> penetrationSphereSphere(
        const Sphere& a, const Sphere& b) noexcept {

        const Vec3 diff = b.center - a.center;
        const Float dist = glm::length(diff);
        Float penetration = a.radius + b.radius - dist;

        if (penetration <= 0 || dist < EPSILON) {
            return {Vec3(0, 1, 0), 0.0f};
        }

        Vec3 direction = diff / dist;
        return {direction, penetration};
    }

    /**
     * @brief Calculate minimum translation vector for overlapping AABBs
     */
    [[nodiscard]] inline std::pair<Vec3, Float> penetrationAABBAABB(
        const AABB& a, const AABB& b) noexcept {

        if (!a.intersects(b)) {
            return {Vec3(0), 0.0f};
        }

        Vec3 aCenter = a.getCenter();
        Vec3 bCenter = b.getCenter();
        Vec3 distance = bCenter - aCenter;

        Vec3 aExtent = a.getHalfExtents();
        Vec3 bExtent = b.getHalfExtents();

        Vec3 overlap;
        overlap.x = aExtent.x + bExtent.x - std::abs(distance.x);
        overlap.y = aExtent.y + bExtent.y - std::abs(distance.y);
        overlap.z = aExtent.z + bExtent.z - std::abs(distance.z);

        // Find axis of minimum overlap
        Vec3 mtv;
        Float minOverlap;

        if (overlap.x <= overlap.y && overlap.x <= overlap.z) {
            mtv = Vec3(sign(distance.x), 0, 0);
            minOverlap = overlap.x;
        } else if (overlap.y <= overlap.z) {
            mtv = Vec3(0, sign(distance.y), 0);
            minOverlap = overlap.y;
        } else {
            mtv = Vec3(0, 0, sign(distance.z));
            minOverlap = overlap.z;
        }

        return {mtv, minOverlap};
    }

} // namespace engine::math