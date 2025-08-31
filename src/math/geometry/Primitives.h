/**
 * @file Primitives.h
 * @brief Geometric primitive definitions for collision detection and spatial queries
 *
 * Defines fundamental geometric shapes used throughout the engine for
 * physics, rendering bounds, and spatial partitioning.
 */

#pragma once

#include "../core/MathTypes.h"
#include "../core/MathConstants.h"
#include "../core/MathFunctions.h"

#include <array>

namespace engine::math {
    // ============================================================================
    // Ray
    // ============================================================================

    /**
     * @brief Ray representation for ray-casting operations
     */
    struct Ray {
        Vec3 origin;
        Vec3 direction; // Should be normalized
        Float maxDistance = INFINITY_VALUE<Float>;

        Ray() noexcept = default;

        Ray(const Vec3& origin, const Vec3& direction, const Float maxDistance = INFINITY_VALUE<Float>) noexcept
            : origin(origin), direction(glm::normalize(direction)), maxDistance(maxDistance) {
        }

        /**
         * @brief Get point along ray at distance t
         */
        [[nodiscard]] Vec3 getPoint(const Float t) const noexcept {
            return origin + direction * t;
        }

        /**
         * @brief Check if distance is within ray's range
         */
        [[nodiscard]] bool isValidDistance(const Float t) const noexcept {
            return t >= 0.0f && t <= maxDistance;
        }
    };

    // ============================================================================
    // Line Segment
    // ============================================================================

    /**
     * @brief Line segment between two points
     */
    struct LineSegment {
        Vec3 start;
        Vec3 end;

        LineSegment() noexcept = default;

        LineSegment(const Vec3& start, const Vec3& end) noexcept
            : start(start), end(end) {
        }

        [[nodiscard]] Vec3 getDirection() const noexcept {
            return end - start;
        }

        [[nodiscard]] Float getLength() const noexcept {
            return glm::length(end - start);
        }

        [[nodiscard]] Float getLengthSquared() const noexcept {
            return glm::length2(end - start);
        }

        [[nodiscard]] Vec3 getPoint(const Float t) const noexcept {
            return start + (end - start) * t;
        }

        [[nodiscard]] Vec3 getCenter() const noexcept {
            return (start + end) * 0.5f;
        }
    };

    // ============================================================================
    // Plane
    // ============================================================================

    /**
     * @brief Infinite plane representation using normal and distance
     * Equation: ax + by + cz + d = 0, where (a,b,c) is normal and d is distance
     */
    struct Plane {
        Vec3 normal{}; // Normal vector (should be normalized)
        Float distance; // Distance from origin along normal

        Plane() noexcept : normal(0, 1, 0), distance(0) {
        }

        Plane(const Vec3& normal, const Float distance) noexcept
            : normal(glm::normalize(normal)), distance(distance) {
        }

        Plane(const Vec3& normal, const Vec3& point) noexcept
            : normal(glm::normalize(normal)), distance(glm::dot(normal, point)) {
        }

        Plane(const Vec3& p0, const Vec3& p1, const Vec3& p2) noexcept {
            const Vec3 v0 = p1 - p0;
            const Vec3 v1 = p2 - p0;
            normal = glm::normalize(glm::cross(v0, v1));
            distance = glm::dot(normal, p0);
        }

        /**
         * @brief Get signed distance from point to plane
         * Positive = in front, Negative = behind, Zero = on plane
         */
        [[nodiscard]] Float getSignedDistance(const Vec3& point) const noexcept {
            return glm::dot(normal, point) - distance;
        }

        /**
         * @brief Project point onto plane
         */
        [[nodiscard]] Vec3 projectPoint(const Vec3& point) const noexcept {
            const Float signedDist = getSignedDistance(point);
            return point - normal * signedDist;
        }

        /**
         * @brief Check if point is on the positive side of plane
         */
        [[nodiscard]] bool isInFront(const Vec3& point) const noexcept {
            return getSignedDistance(point) > 0;
        }

        /**
         * @brief Flip plane to face opposite direction
         */
        void flip() noexcept {
            normal = -normal;
            distance = -distance;
        }
    };

    // ============================================================================
    // Sphere
    // ============================================================================

    /**
     * @brief Sphere/circle primitive
     */
    struct Sphere {
        Vec3 center;
        Float radius;

        Sphere() noexcept : center(0), radius(1) {
        }

        Sphere(const Vec3& center, const Float radius) noexcept
            : center(center), radius(radius) {
        }

        [[nodiscard]] Float getVolume() const noexcept {
            return (4.0f / 3.0f) * PI<Float> * cube(radius);
        }

        [[nodiscard]] Float getSurfaceArea() const noexcept {
            return 4.0f * PI<Float> * square(radius);
        }

        [[nodiscard]] bool contains(const Vec3& point) const noexcept {
            return glm::length2(point - center) <= square(radius);
        }

        [[nodiscard]] bool contains(const Sphere& other) const noexcept {
            const Float dist = glm::length(other.center - center);
            return dist + other.radius <= radius;
        }

        /**
         * @brief Expand sphere to include point
         */
        void expandToInclude(const Vec3& point) noexcept {
            if (const Float dist = glm::length(point - center); dist > radius) {
                radius = dist;
            }
        }

        /**
         * @brief Merge with another sphere
         */
        void merge(const Sphere& other) noexcept {
            const Vec3 diff = other.center - center;
            const Float dist = glm::length(diff);

            if (dist + other.radius <= radius) {
                return; // This sphere contains other
            }
            if (dist + radius <= other.radius) {
                *this = other; // Other contains this
                return;
            }

            // Spheres partially overlap or are separate
            const Float newRadius = (dist + radius + other.radius) * 0.5f;
            const Float t = (newRadius - radius) / dist;
            center = center + diff * t;
            radius = newRadius;
        }
    };

    // ============================================================================
    // Axis-Aligned Bounding Box (AABB)
    // ============================================================================

    /**
     * @brief Axis-aligned bounding box
     */
    struct AABB {
        Vec3 min;
        Vec3 max;

        AABB() noexcept : min(INFINITY_VALUE<Float>), max(NEG_INFINITY<Float>) {
        }

        AABB(const Vec3& min, const Vec3& max) noexcept : min(min), max(max) {
        }

        /**
         * @brief Create AABB from center and half-extents
         */
        static AABB fromCenterExtents(const Vec3& center, const Vec3& halfExtents) noexcept {
            return AABB(center - halfExtents, center + halfExtents);
        }

        /**
         * @brief Create AABB from array of points
         */
        static AABB fromPoints(const Vec3* points, const std::size_t count) noexcept {
            AABB result;
            for (std::size_t i = 0; i < count; ++i) {
                result.expandToInclude(points[i]);
            }
            return result;
        }

        [[nodiscard]] Vec3 getCenter() const noexcept {
            return (min + max) * 0.5f;
        }

        [[nodiscard]] Vec3 getSize() const noexcept {
            return max - min;
        }

        [[nodiscard]] Vec3 getHalfExtents() const noexcept {
            return getSize() * 0.5f;
        }

        [[nodiscard]] Float getVolume() const noexcept {
            const Vec3 size = getSize();
            return size.x * size.y * size.z;
        }

        [[nodiscard]] Float getSurfaceArea() const noexcept {
            const Vec3 size = getSize();
            return 2.0f * (size.x * size.y + size.x * size.z + size.y * size.z);
        }

        [[nodiscard]] Float getDiagonalLength() const noexcept {
            return glm::length(getSize());
        }

        [[nodiscard]] bool isValid() const noexcept {
            return min.x <= max.x && min.y <= max.y && min.z <= max.z;
        }

        [[nodiscard]] bool contains(const Vec3& point) const noexcept {
            return point.x >= min.x && point.x <= max.x &&
                point.y >= min.y && point.y <= max.y &&
                point.z >= min.z && point.z <= max.z;
        }

        [[nodiscard]] bool contains(const AABB& other) const noexcept {
            return other.min.x >= min.x && other.max.x <= max.x &&
                other.min.y >= min.y && other.max.y <= max.y &&
                other.min.z >= min.z && other.max.z <= max.z;
        }

        [[nodiscard]] bool intersects(const AABB& other) const noexcept {
            return min.x <= other.max.x && max.x >= other.min.x &&
                min.y <= other.max.y && max.y >= other.min.y &&
                min.z <= other.max.z && max.z >= other.min.z;
        }

        /**
         * @brief Get the 8 corner vertices of the AABB
         */
        [[nodiscard]] std::array<Vec3, 8> getCorners() const noexcept {
            return {
                {
                    Vec3(min.x, min.y, min.z),
                    Vec3(max.x, min.y, min.z),
                    Vec3(min.x, max.y, min.z),
                    Vec3(max.x, max.y, min.z),
                    Vec3(min.x, min.y, max.z),
                    Vec3(max.x, min.y, max.z),
                    Vec3(min.x, max.y, max.z),
                    Vec3(max.x, max.y, max.z)
                }
            };
        }

        /**
         * @brief Get closest point on AABB to given point
         */
        [[nodiscard]] Vec3 getClosestPoint(const Vec3& point) const noexcept {
            return glm::clamp(point, min, max);
        }

        /**
         * @brief Expand AABB to include point
         */
        void expandToInclude(const Vec3& point) noexcept {
            min = glm::min(min, point);
            max = glm::max(max, point);
        }

        /**
         * @brief Expand AABB to include another AABB
         */
        void expandToInclude(const AABB& other) noexcept {
            min = glm::min(min, other.min);
            max = glm::max(max, other.max);
        }

        /**
         * @brief Expand AABB by given amount in all directions
         */
        void expand(const Float amount) noexcept {
            const Vec3 expansion(amount);

            min -= expansion;
            max += expansion;
        }

        /**
         * @brief Transform AABB by matrix
         */
        [[nodiscard]] AABB transform(const Mat4& matrix) const noexcept {
            const auto corners = getCorners();
            AABB result;

            for (const auto& corner : corners) {
                Vec4 transformed = matrix * Vec4(corner, 1.0f);
                result.expandToInclude(Vec3(transformed) / transformed.w);
            }
            return result;
        }
    };

    // ============================================================================
    // Oriented Bounding Box (OBB)
    // ============================================================================

    /**
     * @brief Oriented bounding box
     */
    struct OBB {
        Vec3 center;
        Vec3 halfExtents;
        Quat orientation;

        OBB() noexcept : center(0), halfExtents(1), orientation(1, 0, 0, 0) {
        }

        OBB(const Vec3& center, const Vec3& halfExtents, const Quat& orientation = Quat(1, 0, 0, 0)) noexcept
            : center(center), halfExtents(halfExtents), orientation(orientation) {
        }

        /**
         * @brief Create OBB from AABB
         */
        explicit OBB(const AABB& aabb) noexcept
            : center(aabb.getCenter()), halfExtents(aabb.getHalfExtents()), orientation(1, 0, 0, 0) {
        }

        /**
         * @brief Get the transformation matrix for this OBB
         */
        [[nodiscard]] Mat4 getTransformMatrix() const noexcept {
            const Mat4 translation = glm::translate(Mat4(1.0f), center);
            const Mat4 rotation = glm::mat4_cast(orientation);
            const Mat4 scale = glm::scale(Mat4(1.0f), halfExtents * 2.0f);
            return translation * rotation * scale;
        }

        /**
         * @brief Get local axes of the OBB
         */
        [[nodiscard]] std::array<Vec3, 3> getAxes() const noexcept {
            Mat3 rotMatrix = glm::mat3_cast(orientation);
            return {
                {
                    Vec3(rotMatrix[0]),
                    Vec3(rotMatrix[1]),
                    Vec3(rotMatrix[2])
                }
            };
        }

        /**
         * @brief Get the 8 corner vertices
         */
        [[nodiscard]] std::array<Vec3, 8> getCorners() const noexcept {
            const Mat3 rotMatrix = glm::mat3_cast(orientation);
            std::array<Vec3, 8> corners{};

            for (int i = 0; i < 8; ++i) {
                Vec3 localCorner(
                    (i & 1) ? halfExtents.x : -halfExtents.x,
                    (i & 2) ? halfExtents.y : -halfExtents.y,
                    (i & 4) ? halfExtents.z : -halfExtents.z
                );
                corners[i] = center + rotMatrix * localCorner;
            }
            return corners;
        }

        /**
         * @brief Convert to AABB (loses rotation)
         */
        [[nodiscard]] AABB toAABB() const noexcept {
            const auto corners = getCorners();
            return AABB::fromPoints(corners.data(), corners.size());
        }
    };

    // ============================================================================
    // Capsule
    // ============================================================================

    /**
     * @brief Capsule (swept sphere) primitive
     */
    struct Capsule {
        Vec3 start;
        Vec3 end;
        Float radius;

        Capsule() noexcept : start(0, -1, 0), end(0, 1, 0), radius(0.5f) {
        }

        Capsule(const Vec3& start, const Vec3& end, const Float radius) noexcept
            : start(start), end(end), radius(radius) {
        }

        [[nodiscard]] LineSegment getSegment() const noexcept {
            return LineSegment(start, end);
        }

        [[nodiscard]] Float getHeight() const noexcept {
            return glm::length(end - start);
        }

        [[nodiscard]] Vec3 getCenter() const noexcept {
            return (start + end) * 0.5f;
        }

        [[nodiscard]] Float getVolume() const noexcept {
            const Float height = getHeight();
            const Float sphereVolume = (4.0f / 3.0f) * PI<Float> * cube(radius);
            const Float cylinderVolume = PI<Float> * square(radius) * height;
            return sphereVolume + cylinderVolume;
        }
    };

    // ============================================================================
    // Frustum
    // ============================================================================

    /**
     * @brief View frustum for culling operations
     */
    struct Frustum {
        enum PlaneIndex {
            PLANE_NEAR = 0,
            PLANE_FAR = 1,
            PLANE_LEFT = 2,
            PLANE_RIGHT = 3,
            PLANE_TOP = 4,
            PLANE_BOTTOM = 5,
            PLANE_COUNT = 6
        };

        std::array<Plane, PLANE_COUNT> planes;

        Frustum() noexcept = default;

        /**
         * @brief Create frustum from view-projection matrix
         */
        explicit Frustum(const Mat4& viewProj) noexcept {
            extractPlanes(viewProj);
        }

        /**
         * @brief Extract frustum planes from view-projection matrix
         */
        void extractPlanes(const Mat4& viewProj) noexcept {
            // Left plane
            planes[PLANE_LEFT] = Plane(
                Vec3(viewProj[0][3] + viewProj[0][0],
                     viewProj[1][3] + viewProj[1][0],
                     viewProj[2][3] + viewProj[2][0]),
                viewProj[3][3] + viewProj[3][0]
            );

            // Right plane
            planes[PLANE_RIGHT] = Plane(
                Vec3(viewProj[0][3] - viewProj[0][0],
                     viewProj[1][3] - viewProj[1][0],
                     viewProj[2][3] - viewProj[2][0]),
                viewProj[3][3] - viewProj[3][0]
            );

            // Top plane
            planes[PLANE_TOP] = Plane(
                Vec3(viewProj[0][3] - viewProj[0][1],
                     viewProj[1][3] - viewProj[1][1],
                     viewProj[2][3] - viewProj[2][1]),
                viewProj[3][3] - viewProj[3][1]
            );

            // Bottom plane
            planes[PLANE_BOTTOM] = Plane(
                Vec3(viewProj[0][3] + viewProj[0][1],
                     viewProj[1][3] + viewProj[1][1],
                     viewProj[2][3] + viewProj[2][1]),
                viewProj[3][3] + viewProj[3][1]
            );

            // Near plane
            planes[PLANE_NEAR] = Plane(
                Vec3(viewProj[0][3] + viewProj[0][2],
                     viewProj[1][3] + viewProj[1][2],
                     viewProj[2][3] + viewProj[2][2]),
                viewProj[3][3] + viewProj[3][2]
            );

            // Far plane
            planes[PLANE_FAR] = Plane(
                Vec3(viewProj[0][3] - viewProj[0][2],
                     viewProj[1][3] - viewProj[1][2],
                     viewProj[2][3] - viewProj[2][2]),
                viewProj[3][3] - viewProj[3][2]
            );

            // Normalize all planes
            for (auto& plane : planes) {
                const Float invLength = 1.0f / glm::length(plane.normal);

                plane.normal *= invLength;
                plane.distance *= invLength;
            }
        }

        /**
         * @brief Test if point is inside frustum
         */
        [[nodiscard]] bool contains(const Vec3& point) const noexcept {
            return std::ranges::all_of(planes, [&](const auto& plane) {
                return plane.getSignedDistance(point) >= 0;
            });
        }

        /**
         * @brief Test if sphere intersects frustum
         */
        [[nodiscard]] bool intersects(const Sphere& sphere) const noexcept {
            return std::ranges::all_of(
                planes,
                [&](const auto& plane) {
                    return plane.getSignedDistance(sphere.center) >= -sphere.radius;
                }
            );
        }

        /**
         * @brief Test if AABB intersects frustum
         */
        [[nodiscard]] bool intersects(const AABB& aabb) const noexcept {
            for (const auto& plane : planes) {
                Vec3 positive = aabb.min;
                if (plane.normal.x >= 0) positive.x = aabb.max.x;
                if (plane.normal.y >= 0) positive.y = aabb.max.y;
                if (plane.normal.z >= 0) positive.z = aabb.max.z;

                if (plane.getSignedDistance(positive) < 0) {
                    return false;
                }
            }
            return true;
        }
    };

    // ============================================================================
    // Triangle
    // ============================================================================

    /**
     * @brief Triangle primitive
     */
    struct Triangle {
        Vec3 v0, v1, v2;

        Triangle() noexcept = default;

        Triangle(const Vec3& v0, const Vec3& v1, const Vec3& v2) noexcept
            : v0(v0), v1(v1), v2(v2) {
        }

        [[nodiscard]] Vec3 getNormal() const noexcept {
            return glm::normalize(glm::cross(v1 - v0, v2 - v0));
        }

        [[nodiscard]] Float getArea() const noexcept {
            return 0.5f * glm::length(glm::cross(v1 - v0, v2 - v0));
        }

        [[nodiscard]] Vec3 getCenter() const noexcept {
            return (v0 + v1 + v2) / 3.0f;
        }

        /**
         * @brief Get barycentric coordinates of point
         */
        [[nodiscard]] Vec3 getBarycentricCoords(const Vec3& point) const noexcept {
            const Vec3 v0v1 = v1 - v0;
            const Vec3 v0v2 = v2 - v0;
            const Vec3 v0p = point - v0;

            const Float d00 = glm::dot(v0v1, v0v1);
            const Float d01 = glm::dot(v0v1, v0v2);
            const Float d11 = glm::dot(v0v2, v0v2);
            const Float d20 = glm::dot(v0p, v0v1);
            const Float d21 = glm::dot(v0p, v0v2);

            const Float invDenom = 1.0f / (d00 * d11 - d01 * d01);
            const Float v = (d11 * d20 - d01 * d21) * invDenom;
            const Float w = (d00 * d21 - d01 * d20) * invDenom;
            const Float u = 1.0f - v - w;

            return Vec3(u, v, w);
        }
    };
} // namespace engine::math
