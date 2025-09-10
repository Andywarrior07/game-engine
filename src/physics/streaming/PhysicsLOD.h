/**
 * @file PhysicsLOD.h
 * @brief Physics level of detail system for performance optimization
 * @details Manages simulation quality based on distance, importance, and
 *          performance budget for large-scale physics simulations
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

#include "../core/PhysicsTypes.h"
#include "../core/PhysicsConstants.h"
#include "../dynamics/RigidBody.h"
#include <unordered_map>
#include <queue>

namespace engine::physics {
    /**
     * @brief LOD level for physics simulation
     */
    enum class PhysicsLODLevel : std::uint8_t {
        FULL = 0, // Full simulation
        SIMPLIFIED = 1, // Simplified shapes, reduced frequency
        MINIMAL = 2, // Basic movement only
        FROZEN = 3, // No simulation, frozen in place
        CULLED = 4 // Completely removed from simulation
    };

    /**
     * @brief LOD configuration for each level
     */
    struct LODConfig {
        Float updateFrequency = 60.0f; // Hz
        Float solverIterations = 10; // Constraint solver iterations
        bool enableCCD = true; // Continuous collision detection
        bool enableFriction = true; // Friction calculations
        bool enableComplexShapes = true; // Use complex collision shapes
        bool enableConstraints = true; // Process constraints
        Float collisionMargin = collision::DEFAULT_MARGIN;

        static LODConfig Full() {
            return LODConfig{60.0f, 10, true, true, true, true, collision::DEFAULT_MARGIN};
        }

        static LODConfig Simplified() {
            return LODConfig{30.0f, 5, false, true, false, true, collision::THICK_MARGIN};
        }

        static LODConfig Minimal() {
            return LODConfig{10.0f, 2, false, false, false, false, collision::THICK_MARGIN * 2.0f};
        }

        static LODConfig Frozen() {
            return LODConfig{0.0f, 0, false, false, false, false, 0.0f};
        }
    };

    /**
     * @brief Physics LOD management system
     * @details Dynamically adjusts simulation quality based on various factors
     *          to maintain performance in large-scale simulations
     */
    class PhysicsLODSystem {
    public:
        PhysicsLODSystem();

        // ============================================================================
        // LOD Management
        // ============================================================================

        /**
         * @brief Update LOD levels for all bodies
         * @param viewerPosition Position of the camera/player
         * @param deltaTime Frame time
         */
        void update(const Vec3& viewerPosition, Float deltaTime);

        /**
         * @brief Register a body with the LOD system
         */
        void registerBody(RigidBody* body, Float importance = 1.0f);

        /**
         * @brief Unregister a body
         */
        void unregisterBody(RigidBody* body);

        /**
         * @brief Set importance of a body (higher = more important)
         */
        void setBodyImportance(RigidBody* body, Float importance);

        /**
         * @brief Get current LOD level of a body
         */
        PhysicsLODLevel getBodyLOD(RigidBody* body) const;

        // ============================================================================
        // Configuration
        // ============================================================================

        /**
         * @brief Set LOD distances
         */
        void setLODDistances(const Float high, const Float medium, const Float low, const Float cull) {
            lodDistances_[0] = high;
            lodDistances_[1] = medium;
            lodDistances_[2] = low;
            lodDistances_[3] = cull;
        }

        /**
         * @brief Set LOD configuration for a level
         */
        void setLODConfig(PhysicsLODLevel level, const LODConfig& config) {
            lodConfigs_[static_cast<int>(level)] = config;
        }

        /**
         * @brief Enable/disable automatic quality adjustment
         */
        void setAutoAdjust(const bool enable) { autoAdjust_ = enable; }

        /**
         * @brief Set target frame time for auto adjustment
         */
        void setTargetFrameTime(const Float ms) { targetFrameTime_ = ms; }

        /**
         * @brief Set maximum number of bodies at full LOD
         */
        void setMaxFullLODBodies(const std::size_t count) { maxFullLODBodies_ = count; }

        // ============================================================================
        // Statistics
        // ============================================================================

        struct Statistics {
            std::size_t totalBodies = 0;
            std::size_t bodiesAtFull = 0;
            std::size_t bodiesAtSimplified = 0;
            std::size_t bodiesAtMinimal = 0;
            std::size_t bodiesFrozen = 0;
            std::size_t bodiesCulled = 0;
            Float averageUpdateFrequency = 0.0f;
            Float qualityFactor = 1.0f;
        };

        Statistics getStatistics() const;

    private:
        struct BodyLODInfo {
            RigidBody* body = nullptr;
            PhysicsLODLevel level = PhysicsLODLevel::FULL;
            Float importance = 1.0f;
            Float timeSinceLastUpdate = 0.0f;
            Float distanceToViewer = 0.0f;
            CollisionShape* originalShape = nullptr; // Store original for LOD switching
            CollisionShape* simplifiedShape = nullptr; // Simplified version
        };

        // Bodies and their LOD info
        std::unordered_map<RigidBody*, BodyLODInfo> bodies_;
        std::unordered_map<PhysicsLODLevel, std::unordered_set<RigidBody*>> bodiesByLevel_;

        // LOD configurations
        LODConfig lodConfigs_[5];
        Float lodDistances_[4] = {
            performance::LOD_DISTANCE_HIGH,
            performance::LOD_DISTANCE_MEDIUM,
            performance::LOD_DISTANCE_LOW,
            performance::LOD_DISTANCE_CULL
        };

        // Viewer info
        Vec3 viewerPosition_;

        // Update control
        Float updateInterval_ = 0.5f; // How often to recalculate LOD levels
        Float timeSinceLastUpdate_ = 0.0f;

        // Performance control
        bool autoAdjust_ = true;
        Float targetFrameTime_ = 16.0f; // Target 60 FPS
        Float currentFrameTime_ = 16.0f;
        Float qualityScale_ = 1.0f; // Global quality multiplier
        std::size_t maxFullLODBodies_ = 100;

        void initializeLODConfigs() {
            lodConfigs_[0] = LODConfig::Full();
            lodConfigs_[1] = LODConfig::Simplified();
            lodConfigs_[2] = LODConfig::Minimal();
            lodConfigs_[3] = LODConfig::Frozen();
            lodConfigs_[4] = LODConfig::Frozen(); // Culled uses frozen config
        }

        void updateAllLODLevels();

        void updateBodiesByLOD(Float deltaTime);

        void applyLODToBody(RigidBody* body, PhysicsLODLevel level);

        void adjustQualityForPerformance(Float deltaTime);
    };
} // namespace engine::physics
