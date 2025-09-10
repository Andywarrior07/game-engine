/**
 * @file CollisionFiltering.h
 * @brief Collision filtering and layer management
 * @details Implements collision layers, masks, and custom filtering rules
 *          to control which objects can collide with each other
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

#include "BroadPhase.h"

#include <functional>
#include <unordered_map>
#include <unordered_set>

namespace engine::physics {
    /**
     * @brief Collision layer definition
     */
    struct CollisionLayer {
        std::string name;
        std::uint16_t bit;
        std::uint16_t collidesWith;
        bool enabled = true;

        CollisionLayer(const std::string& n, const std::uint16_t b, const std::uint16_t mask)
            : name(n), bit(b), collidesWith(mask) {
        }

        CollisionLayer() = default;
    };

    /**
     * @brief Custom collision filter callback
     */
    using CollisionFilterCallback = std::function<bool(RigidBody*, RigidBody*)>;

    /**
     * @brief Collision filtering system
     * @details Manages collision layers, groups, and custom filtering rules
     */
    class CollisionFilter {
    public:
        CollisionFilter() {
            initializeDefaultLayers();
        }

        // ============================================================================
        // Layer Management
        // ============================================================================

        /**
         * @brief Define a collision layer
         */
        void defineLayer(const std::string& name, std::uint16_t bit);

        /**
         * @brief Set which layers a layer can collide with
         */
        void setLayerCollision(const std::string& layerA, const std::string& layerB,
                               bool canCollide);


        /**
         * @brief Enable/disable a layer
         */
        void setLayerEnabled(const std::string& layer, bool enabled);

        /**
         * @brief Get layer by name
         */
        CollisionLayer* getLayer(const std::string& name);

        // ============================================================================
        // Collision Matrix
        // ============================================================================

        /**
         * @brief Set up collision matrix for common game setup
         */
        void setupCollisionMatrix(const std::vector<std::pair<std::string, std::string>>& pairs);

        /**
         * @brief Print collision matrix for debugging
         */
        [[nodiscard]] std::string getCollisionMatrixString() const;

        // ============================================================================
        // Filtering
        // ============================================================================

        /**
         * @brief Check if two bodies should collide
         */
        bool shouldCollide(RigidBody* bodyA, RigidBody* bodyB) const;

        /**
         * @brief Add custom filter callback
         */
        void addCustomFilter(const CollisionFilterCallback& filter) {
            customFilters_.push_back(filter);
        }

        /**
         * @brief Clear all custom filters
         */
        void clearCustomFilters() {
            customFilters_.clear();
        }

        // ============================================================================
        // Exclusion Management
        // ============================================================================

        /**
         * @brief Add exclusion pair (bodies that should never collide)
         */
        void addExclusionPair(RigidBody* bodyA, RigidBody* bodyB);

        /**
         * @brief Remove exclusion pair
         */
        void removeExclusionPair(RigidBody* bodyA, RigidBody* bodyB);

        /**
         * @brief Clear all exclusion pairs
         */
        void clearExclusionPairs();

        /**
         * @brief Check if pair is excluded
         */
        bool isExcluded(RigidBody* bodyA, RigidBody* bodyB) const;

        // ============================================================================
        // Presets
        // ============================================================================

        /**
         * @brief Set up filtering for typical RPG game
         */
        void setupRPGFiltering();

        /**
         * @brief Set up filtering for platformer game
         */
        void setupPlatformerFiltering();

    private:
        // Layer storage
        std::unordered_map<std::string, CollisionLayer> layers_;
        std::unordered_map<std::uint16_t, CollisionLayer*> layersByBit_;

        // Custom filters
        std::vector<CollisionFilterCallback> customFilters_;

        // Exclusion pairs
        std::unordered_set<CollisionPair, CollisionPair::Hash> exclusionPairs_;

        void initializeDefaultLayers();

        void clearAllLayers() {
            layers_.clear();
            layersByBit_.clear();
        }

        static bool shouldBodyTypesCollide(BodyType typeA, BodyType typeB);
    };
} // namespace engine::physics
