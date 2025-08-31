/**
 * @file Octree.h
 * @brief Octree and Quadtree spatial partitioning structures
 * @author Andrés Guerrero
 * @date 27-08-2025
 *
 * Provides efficient spatial queries for collision detection, frustum culling,
 * and proximity searches in 2D and 3D space.
 */

#pragma once

#include "../core/MathTypes.h"
#include "../core/MathConstants.h"
#include "../geometry/Primitives.h"
#include "../geometry/Distance.h"
#include "../geometry/Intersections.h"

#include <vector>
#include <memory>
#include <functional>
#include <queue>


namespace engine::math {
    /**
     * @brief Object stored in spatial partitioning structure
     */
    template <typename BoundsType>
    struct SpatialObject {
        BoundsType bounds;
        void* userData; // Pointer to actual game object
        Int id; // Unique identifier

        SpatialObject() : userData(nullptr), id(0) {
        }

        SpatialObject(const BoundsType& b, void* data, const Int objId)
            : bounds(b), userData(data), id(objId) {
        }
    };

    // ============================================================================
    // Octree (3D Spatial Partitioning)
    // ============================================================================

    /**
     * @brief Octree for 3D spatial partitioning
     *
     * Recursively subdivides 3D space into 8 octants for efficient
     * spatial queries and collision detection.
     */
    template <typename T = SpatialObject<AABB>>
    class Octree {
    public:
        using ObjectType = T;
        using QueryCallback = std::function<void(const ObjectType&)>;

        /**
         * @brief Construct octree with bounds and settings
         */
        explicit Octree(const AABB& bounds, const std::size_t maxDepth = 8,
                        const std::size_t maxObjectsPerNode = 8) noexcept
            : bounds_(bounds)
              , maxDepth_(maxDepth)
              , maxObjectsPerNode_(maxObjectsPerNode)
              , depth_(0)
              , parent_(nullptr) {
        }

        /**
         * @brief Insert object into octree
         */
        bool insert(const ObjectType& object) {
            // Check if object fits in bounds
            if (!bounds_.contains(object.bounds)) {
                return false;
            }

            // If we have room or can't subdivide, add here
            if (objects_.size() < maxObjectsPerNode_ || depth_ >= maxDepth_) {
                objects_.push_back(object);
                return true;
            }

            // Subdivide if needed
            if (!isSubdivided_) {
                subdivide();
            }

            // Try to insert into children
            for (auto& child : children_) {
                if (child->insert(object)) {
                    return true;
                }
            }

            // Doesn't fit perfectly in any child, store here
            objects_.push_back(object);
            return true;
        }

        /**
         * @brief Remove object from octree
         */
        bool remove(Int objectId) {
            // Check this node's objects
            auto it = std::find_if(objects_.begin(), objects_.end(),
                                   [objectId](const ObjectType& obj) { return obj.id == objectId; });

            if (it != objects_.end()) {
                objects_.erase(it);
                return true;
            }

            // Check children
            if (isSubdivided_) {
                for (auto& child : children_) {
                    if (child->remove(objectId)) {
                        return true;
                    }
                }
            }

            return false;
        }

        /**
         * @brief Update object position (remove and reinsert)
         */
        bool update(const Int objectId, const ObjectType& newObject) {
            if (remove(objectId)) {
                return insert(newObject);
            }
            return false;
        }

        /**
         * @brief Query objects within AABB
         */
        void query(const AABB& range, std::vector<ObjectType>& results) const {
            if (!bounds_.intersects(range)) {
                return;
            }

            // Check objects at this node
            for (const auto& obj : objects_) {
                if (range.intersects(obj.bounds)) {
                    results.push_back(obj);
                }
            }

            // Recursively query children
            if (isSubdivided_) {
                for (const auto& child : children_) {
                    child->query(range, results);
                }
            }
        }

        /**
         * @brief Query objects within sphere
         */
        void query(const Sphere& sphere, std::vector<ObjectType>& results) const {
            if (!intersectSphereAABB(sphere, bounds_)) {
                return;
            }

            // Check objects at this node
            for (const auto& obj : objects_) {
                if (intersectSphereAABB(sphere, obj.bounds)) {
                    results.push_back(obj);
                }
            }

            // Recursively query children
            if (isSubdivided_) {
                for (const auto& child : children_) {
                    child->query(sphere, results);
                }
            }
        }

        /**
         * @brief Ray cast through octree
         */
        void raycast(const Ray& ray, std::vector<ObjectType>& results) const {
            if (!intersectRayAABB(ray, bounds_)) {
                return;
            }

            // Check objects at this node
            for (const auto& obj : objects_) {
                if (intersectRayAABB(ray, obj.bounds)) {
                    results.push_back(obj);
                }
            }

            // Recursively query children
            if (isSubdivided_) {
                for (const auto& child : children_) {
                    child->raycast(ray, results);
                }
            }
        }

        /**
         * @brief Frustum culling query
         */
        void queryFrustum(const Frustum& frustum, std::vector<ObjectType>& results) const {
            if (!frustum.intersects(bounds_)) {
                return;
            }

            // Check if completely inside frustum (no need to test children individually)
            bool completelyInside = true;
            for (const auto& plane : frustum.planes) {
                if (plane.getSignedDistance(bounds_.getCenter()) -
                    glm::length(bounds_.getHalfExtents()) < 0) {
                    completelyInside = false;
                    break;
                }
            }

            if (completelyInside) {
                // Add all objects in this subtree
                collectAll(results);
                return;
            }

            // Partially inside - test objects individually
            for (const auto& obj : objects_) {
                if (frustum.intersects(obj.bounds)) {
                    results.push_back(obj);
                }
            }

            // Recursively query children
            if (isSubdivided_) {
                for (const auto& child : children_) {
                    child->queryFrustum(frustum, results);
                }
            }
        }

        /**
         * @brief Find nearest object to a point
         */
        std::optional<ObjectType> findNearest(const Vec3& point) const {
            struct NodeDistance {
                const Octree* node;
                Float distance;

                bool operator>(const NodeDistance& other) const {
                    return distance > other.distance;
                }
            };

            std::priority_queue<NodeDistance, std::vector<NodeDistance>, std::greater<>> queue;
            queue.push({this, distancePointToAABB(point, bounds_)});

            std::optional<ObjectType> nearest;
            Float nearestDist = INFINITY_VALUE<Float>;

            while (!queue.empty()) {
                auto [node, nodeDist] = queue.top();
                queue.pop();

                // Skip if this node is farther than current nearest
                if (nodeDist > nearestDist) {
                    continue;
                }

                // Check objects in this node
                for (const auto& obj : node->objects_) {
                    if (const Float dist = distancePointToAABB(point, obj.bounds); dist < nearestDist) {
                        nearestDist = dist;
                        nearest = obj;
                    }
                }

                // Add children to queue
                if (node->isSubdivided_) {
                    for (const auto& child : node->children_) {
                        if (const Float childDist = distancePointToAABB(point, child->bounds_); childDist < nearestDist) {
                            queue.push({child.get(), childDist});
                        }
                    }
                }
            }

            return nearest;
        }

        /**
         * @brief Clear all objects from tree
         */
        void clear() {
            objects_.clear();
            children_.clear();
            isSubdivided_ = false;
        }

        /**
         * @brief Get total object count
         */
        [[nodiscard]] std::size_t getTotalObjects() const {
            std::size_t count = objects_.size();
            if (isSubdivided_) {
                for (const auto& child : children_) {
                    count += child->getTotalObjects();
                }
            }
            return count;
        }

        /**
         * @brief Get tree depth at this node
         */
        [[nodiscard]] std::size_t getDepth() const { return depth_; }

        /**
         * @brief Get bounds of this node
         */
        [[nodiscard]] const AABB& getBounds() const { return bounds_; }

    private:
        AABB bounds_;
        std::size_t maxDepth_;
        std::size_t maxObjectsPerNode_;
        std::size_t depth_;
        Octree* parent_;

        std::vector<ObjectType> objects_;
        std::array<std::unique_ptr<Octree>, 8> children_;
        bool isSubdivided_ = false;

        /**
         * @brief Subdivide this node into 8 children
         */
        void subdivide() {
            Vec3 center = bounds_.getCenter();

            // Create 8 child octant's
            for (int i = 0; i < 8; ++i) {
                Vec3 childMin = center;
                Vec3 childMax = center;

                // Determine octant position
                if (i & 1) {
                    // Right
                    childMax.x = bounds_.max.x;
                }
                else {
                    // Left
                    childMin.x = bounds_.min.x;
                }

                if (i & 2) {
                    // Top
                    childMax.y = bounds_.max.y;
                }
                else {
                    // Bottom
                    childMin.y = bounds_.min.y;
                }

                if (i & 4) {
                    // Front
                    childMax.z = bounds_.max.z;
                }
                else {
                    // Back
                    childMin.z = bounds_.min.z;
                }

                AABB childBounds(childMin, childMax);
                children_[i] = std::make_unique<Octree>(
                    childBounds, maxDepth_, maxObjectsPerNode_
                );
                children_[i]->depth_ = depth_ + 1;
                children_[i]->parent_ = this;
            }

            isSubdivided_ = true;

            // Move objects to children if possible
            std::vector<ObjectType> remainingObjects;
            for (const auto& obj : objects_) {
                bool inserted = false;
                for (auto& child : children_) {
                    if (child->bounds_.contains(obj.bounds)) {
                        child->objects_.push_back(obj);
                        inserted = true;
                        break;
                    }
                }
                if (!inserted) {
                    remainingObjects.push_back(obj);
                }
            }
            objects_ = std::move(remainingObjects);
        }

        /**
         * @brief Collect all objects in this subtree
         */
        void collectAll(std::vector<ObjectType>& results) const {
            results.insert(results.end(), objects_.begin(), objects_.end());
            if (isSubdivided_) {
                for (const auto& child : children_) {
                    child->collectAll(results);
                }
            }
        }
    };

    // ============================================================================
    // Quadtree (2D Spatial Partitioning)
    // ============================================================================

    /**
     * @brief Simple 2D bounds for quadtree
     */
    struct Bounds2D {
        Vec2 min, max;

        Bounds2D() : min(INFINITY_VALUE<Float>), max(NEG_INFINITY<Float>) {
        }

        Bounds2D(const Vec2& min, const Vec2& max) : min(min), max(max) {
        }

        [[nodiscard]] bool contains(const Bounds2D& other) const {
            return other.min.x >= min.x && other.max.x <= max.x &&
                other.min.y >= min.y && other.max.y <= max.y;
        }

        [[nodiscard]] bool intersects(const Bounds2D& other) const {
            return min.x <= other.max.x && max.x >= other.min.x &&
                min.y <= other.max.y && max.y >= other.min.y;
        }

        [[nodiscard]] Vec2 getCenter() const {
            return (min + max) * 0.5f;
        }
    };

    /**
     * @brief Quadtree for 2D spatial partitioning
     * Similar to Octree but for 2D space (4 children instead of 8)
     */
    template <typename T = SpatialObject<Bounds2D>>
    class Quadtree {
    public:
        using ObjectType = T;

        explicit Quadtree(const Bounds2D& bounds,
                          const std::size_t maxDepth = 8,
                          const std::size_t maxObjectsPerNode = 8) noexcept
            : bounds_(bounds)
              , maxDepth_(maxDepth)
              , maxObjectsPerNode_(maxObjectsPerNode)
              , depth_(0) {
        }

        // Similar interface to Octree but with 2D operations
        // Implementation follows same pattern with 4 quadrants instead of 8 octants

    private:
        Bounds2D bounds_;
        std::size_t maxDepth_;
        std::size_t maxObjectsPerNode_;
        std::size_t depth_;

        std::vector<ObjectType> objects_;
        std::array<std::unique_ptr<Quadtree>, 4> children_;
        bool isSubdivided_ = false;
    };
} // namespace engine::math
