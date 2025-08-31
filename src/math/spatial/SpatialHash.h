/**
 * @file SpatialHash.h
 * @brief Uniform spatial hashing for fast proximity queries
 * @author Andrés Guerrero
 * @date 27-08-2025
 *
 * Provides efficient spatial partitioning using hash tables for
 * uniform-sized objects and broad-phase collision detection.
 */

#pragma once

#include "../core/MathTypes.h"
#include "../core/MathConstants.h"
#include "../geometry/Primitives.h"
#include "../core/FastMath.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <ranges>

namespace engine::math {
    /**
     * @brief Hash function for integer coordinates
     */
    struct SpatialHasher {
        std::size_t operator()(const Vec3i& v) const noexcept {
            // Use FNV-1a hash for good distribution
            std::size_t h = 14695981039346656037ULL;
            h ^= static_cast<std::size_t>(v.x);
            h *= 1099511628211ULL;
            h ^= static_cast<std::size_t>(v.y);
            h *= 1099511628211ULL;
            h ^= static_cast<std::size_t>(v.z);
            h *= 1099511628211ULL;
            return h;
        }

        std::size_t operator()(const Vec2i& v) const noexcept {
            std::size_t h = 14695981039346656037ULL;
            h ^= static_cast<std::size_t>(v.x);
            h *= 1099511628211ULL;
            h ^= static_cast<std::size_t>(v.y);
            h *= 1099511628211ULL;
            return h;
        }
    };

    /**
     * @brief 3D Spatial hash grid for uniform partitioning
     *
     * Efficiently handles objects of similar size using a hash table
     * where space is divided into uniform cells.
     */
    template <typename T>
    class SpatialHash3D {
    public:
        using ObjectID = std::uint32_t;
        using CellCoord = Vec3i;

        /**
         * @brief Object stored in spatial hash
         */
        struct SpatialObject {
            T data;
            AABB bounds;
            ObjectID id;
            std::vector<CellCoord> cells; // Cells this object occupies

            SpatialObject() : id(0) {
            }

            SpatialObject(const T& data, const AABB& bounds, const ObjectID id)
                : data(data), bounds(bounds), id(id) {
            }
        };

        /**
         * @brief Initialize spatial hash with cell size
         * @param cellSize Size of each grid cell (should be ~2x typical object size)
         */
        explicit SpatialHash3D(const Float cellSize = 1.0f) noexcept
            : cellSize_(cellSize), invCellSize_(1.0f / cellSize) {
            cells_.reserve(1024); // Pre-allocate for performance
        }

        /**
         * @brief Insert object into spatial hash
         */
        ObjectID insert(const T& data, const AABB& bounds) {
            ObjectID id = nextId_++;

            SpatialObject obj(data, bounds, id);

            // Calculate cells this object occupies
            const CellCoord minCell = worldToCell(bounds.min);
            const CellCoord maxCell = worldToCell(bounds.max);

            // Insert into all overlapping cells
            for (int z = minCell.z; z <= maxCell.z; ++z) {
                for (int y = minCell.y; y <= maxCell.y; ++y) {
                    for (int x = minCell.x; x <= maxCell.x; ++x) {
                        CellCoord cell(x, y, z);
                        cells_[cell].insert(id);
                        obj.cells.push_back(cell);
                    }
                }
            }

            objects_[id] = std::move(obj);
            return id;
        }

        /**
         * @brief Remove object from spatial hash
         */
        bool remove(ObjectID id) {
            auto it = objects_.find(id);
            if (it == objects_.end()) return false;

            // Remove from all cells
            for (const auto& cell : it->second.cells) {
                cells_[cell].erase(id);
                if (cells_[cell].empty()) {
                    cells_.erase(cell);
                }
            }

            objects_.erase(it);
            return true;
        }

        /**
         * @brief Update object position
         */
        bool update(ObjectID id, const AABB& newBounds) {
            auto it = objects_.find(id);
            if (it == objects_.end()) return false;

            auto& obj = it->second;

            // Check if object moved to different cells
            const CellCoord oldMin = worldToCell(obj.bounds.min);
            const CellCoord oldMax = worldToCell(obj.bounds.max);
            const CellCoord newMin = worldToCell(newBounds.min);
            const CellCoord newMax = worldToCell(newBounds.max);

            if (oldMin == newMin && oldMax == newMax) {
                // Still in same cells, just update bounds
                obj.bounds = newBounds;
                return true;
            }

            // Remove from old cells
            for (const auto& cell : obj.cells) {
                cells_[cell].erase(id);
                if (cells_[cell].empty()) {
                    cells_.erase(cell);
                }
            }
            obj.cells.clear();

            // Add to new cells
            for (int z = newMin.z; z <= newMax.z; ++z) {
                for (int y = newMin.y; y <= newMax.y; ++y) {
                    for (int x = newMin.x; x <= newMax.x; ++x) {
                        CellCoord cell(x, y, z);
                        cells_[cell].insert(id);
                        obj.cells.push_back(cell);
                    }
                }
            }

            obj.bounds = newBounds;
            return true;
        }

        /**
         * @brief Query objects in AABB region
         */
        [[nodiscard]] std::vector<T> query(const AABB& region) const {
            std::vector<T> results;
            std::unordered_set<ObjectID> checked;

            const CellCoord minCell = worldToCell(region.min);
            const CellCoord maxCell = worldToCell(region.max);

            // Check all cells in region
            for (int z = minCell.z; z <= maxCell.z; ++z) {
                for (int y = minCell.y; y <= maxCell.y; ++y) {
                    for (int x = minCell.x; x <= maxCell.x; ++x) {
                        CellCoord cell(x, y, z);

                        auto cellIt = cells_.find(cell);
                        if (cellIt == cells_.end()) continue;

                        // Check each object in cell
                        for (ObjectID id : cellIt->second) {
                            // Skip if already checked
                            if (checked.contains(id)) continue;
                            checked.insert(id);

                            // Precise intersection test
                            if (const auto& obj = objects_.at(id); region.intersects(obj.bounds)) {
                                results.push_back(obj.data);
                            }
                        }
                    }
                }
            }

            return results;
        }

        /**
         * @brief Query objects in sphere region
         */
        [[nodiscard]] std::vector<T> query(const Sphere& sphere) const {
            std::vector<T> results;
            std::unordered_set<ObjectID> checked;

            // Get AABB of sphere
            Vec3 radiusVec(sphere.radius);
            AABB sphereAABB(sphere.center - radiusVec, sphere.center + radiusVec);

            CellCoord minCell = worldToCell(sphereAABB.min);
            CellCoord maxCell = worldToCell(sphereAABB.max);

            for (int z = minCell.z; z <= maxCell.z; ++z) {
                for (int y = minCell.y; y <= maxCell.y; ++y) {
                    for (int x = minCell.x; x <= maxCell.x; ++x) {
                        CellCoord cell(x, y, z);

                        auto cellIt = cells_.find(cell);
                        if (cellIt == cells_.end()) continue;

                        for (ObjectID id : cellIt->second) {
                            if (checked.contains(id)) continue;
                            checked.insert(id);

                            if (const auto& obj = objects_.at(id); intersectSphereAABB(sphere, obj.bounds)) {
                                results.push_back(obj.data);
                            }
                        }
                    }
                }
            }

            return results;
        }

        /**
         * @brief Find nearest neighbor to point
         */
        [[nodiscard]] std::optional<T> findNearest(const Vec3& point) const {
            // Start with cell containing point
            const CellCoord centerCell = worldToCell(point);

            std::optional<T> nearest;
            Float nearestDistSq = INFINITY_VALUE<Float>;
            std::unordered_set<ObjectID> checked;

            // Expand search in rings
            for (int ring = 0; ring < 10; ++ring) {
                bool found = false;

                // Check all cells at this ring distance
                for (int z = -ring; z <= ring; ++z) {
                    for (int y = -ring; y <= ring; ++y) {
                        for (int x = -ring; x <= ring; ++x) {
                            // Skip if not on ring surface
                            if (const int maxCoord = max3(std::abs(x), std::abs(y), std::abs(z)); maxCoord != ring)
                                continue;

                            CellCoord cell = centerCell + CellCoord(x, y, z);

                            auto cellIt = cells_.find(cell);
                            if (cellIt == cells_.end()) continue;

                            for (ObjectID id : cellIt->second) {
                                if (checked.contains(id)) continue;
                                checked.insert(id);

                                const auto& obj = objects_.at(id);

                                if (const Float distSq = distancePointToAABBSq(point, obj.bounds); distSq <
                                    nearestDistSq) {
                                    nearestDistSq = distSq;
                                    nearest = obj.data;
                                    found = true;
                                }
                            }
                        }
                    }
                }

                // Stop expanding if we found objects and ring is beyond nearest
                if (found && static_cast<Float>(ring) * cellSize_ > std::sqrt(nearestDistSq)) {
                    break;
                }
            }

            return nearest;
        }

        /**
         * @brief Find all pairs of potentially colliding objects
         */
        [[nodiscard]] std::vector<std::pair<T, T>> findPotentialPairs() const {
            std::vector<std::pair<T, T>> pairs;
            std::unordered_set<std::uint64_t> checkedPairs;

            for (const auto& objectIds : cells_ | std::views::values) {
                // Check all pairs within this cell
                for (auto it1 = objectIds.begin(); it1 != objectIds.end(); ++it1) {
                    auto it2 = it1;
                    ++it2;
                    for (; it2 != objectIds.end(); ++it2) {
                        // Create unique pair ID
                        std::uint64_t pairId = (*it1 < *it2)
                                                   ? (static_cast<std::uint64_t>(*it1) << 32) | *it2
                                                   : (static_cast<std::uint64_t>(*it2) << 32) | *it1;

                        if (checkedPairs.contains(pairId)) continue;
                        checkedPairs.insert(pairId);

                        const auto& obj1 = objects_.at(*it1);

                        if (const auto& obj2 = objects_.at(*it2); obj1.bounds.intersects(obj2.bounds)) {
                            pairs.emplace_back(obj1.data, obj2.data);
                        }
                    }
                }
            }

            return pairs;
        }

        /**
         * @brief Clear all objects
         */
        void clear() {
            cells_.clear();
            objects_.clear();
            nextId_ = 1;
        }

        /**
         * @brief Get statistics
         */
        [[nodiscard]] std::size_t getObjectCount() const { return objects_.size(); }
        [[nodiscard]] std::size_t getCellCount() const { return cells_.size(); }
        [[nodiscard]] Float getCellSize() const { return cellSize_; }

        /**
         * @brief Get average objects per cell
         */
        [[nodiscard]] Float getAverageObjectsPerCell() const {
            if (cells_.empty()) return 0;

            std::size_t total = 0;
            for (const auto& objects : cells_ | std::views::values) {
                total += objects.size();
            }
            return static_cast<Float>(total) / cells_.size();
        }

    private:
        Float cellSize_;
        Float invCellSize_;
        ObjectID nextId_ = 1;

        std::unordered_map<CellCoord, std::unordered_set<ObjectID>, SpatialHasher> cells_;
        std::unordered_map<ObjectID, SpatialObject> objects_;

        /**
         * @brief Convert world position to cell coordinates
         */
        [[nodiscard]] CellCoord worldToCell(const Vec3& pos) const noexcept {
            return {
                fast::fastFloor(pos.x * invCellSize_),
                fast::fastFloor(pos.y * invCellSize_),
                fast::fastFloor(pos.z * invCellSize_)
            };
        }

        /**
         * @brief Get AABB of a cell
         */
        [[nodiscard]] AABB getCellBounds(const CellCoord& cell) const noexcept {
            const Vec3 min = Vec3(cell) * cellSize_;
            const Vec3 max = min + Vec3(cellSize_);
            return {min, max};
        }
    };

    /**
     * @brief 2D Spatial hash grid
     */
    template <typename T>
    class SpatialHash2D {
    public:
        using ObjectID = std::uint32_t;
        using CellCoord = Vec2i;

        struct SpatialObject {
            T data;
            Vec2 min, max;
            ObjectID id;
            std::vector<CellCoord> cells;

            SpatialObject() : data(), min(0), max(0), id(0) {
            }

            SpatialObject(const T& data, const Vec2& min, const Vec2& max, const ObjectID id)
                : data(data), min(min), max(max), id(id) {
            }
        };

        explicit SpatialHash2D(const Float cellSize = 1.0f) noexcept
            : cellSize_(cellSize), invCellSize_(1.0f / cellSize) {
        }

        ObjectID insert(const T& data, const Vec2& min, const Vec2& max) {
            ObjectID id = nextId_++;

            SpatialObject obj(data, min, max, id);

            const CellCoord minCell = worldToCell(min);
            const CellCoord maxCell = worldToCell(max);

            for (int y = minCell.y; y <= maxCell.y; ++y) {
                for (int x = minCell.x; x <= maxCell.x; ++x) {
                    CellCoord cell(x, y);
                    cells_[cell].insert(id);
                    obj.cells.push_back(cell);
                }
            }

            objects_[id] = std::move(obj);
            return id;
        }

        [[nodiscard]] std::vector<T> query(const Vec2& min, const Vec2& max) const {
            std::vector<T> results;
            std::unordered_set<ObjectID> checked;

            const CellCoord minCell = worldToCell(min);
            const CellCoord maxCell = worldToCell(max);

            for (int y = minCell.y; y <= maxCell.y; ++y) {
                for (int x = minCell.x; x <= maxCell.x; ++x) {
                    CellCoord cell(x, y);

                    auto cellIt = cells_.find(cell);
                    if (cellIt == cells_.end()) continue;

                    for (ObjectID id : cellIt->second) {
                        if (checked.contains(id)) continue;
                        checked.insert(id);

                        if (const auto& obj = objects_.at(id); !(obj.min.x > max.x || obj.max.x < min.x ||
                            obj.min.y > max.y || obj.max.y < min.y)) {
                            results.push_back(obj.data);
                        }
                    }
                }
            }

            return results;
        }

    private:
        Float cellSize_;
        Float invCellSize_;
        ObjectID nextId_ = 1;

        std::unordered_map<CellCoord, std::unordered_set<ObjectID>, SpatialHasher> cells_;
        std::unordered_map<ObjectID, SpatialObject> objects_;

        [[nodiscard]] CellCoord worldToCell(const Vec2& pos) const noexcept {
            return {
                fast::fastFloor(pos.x * invCellSize_),

                fast::fastFloor(pos.y * invCellSize_)
            };
        }
    };
} // namespace engine::math
