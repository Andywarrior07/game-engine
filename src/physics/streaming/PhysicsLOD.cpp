/**
* @file PhysicsLOD.h
 * @brief Physics level of detail system for performance optimization
 * @details Manages simulation quality based on distance, importance, and
 *          performance budget for large-scale physics simulations
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#include "PhysicsLOD.h"

namespace engine::physics {
    PhysicsLODSystem::PhysicsLODSystem() {
        initializeLODConfigs();
    }

    void PhysicsLODSystem::update(const Vec3& viewerPosition, const Float deltaTime) {
        viewerPosition_ = viewerPosition;
        timeSinceLastUpdate_ += deltaTime;

        // Update LOD levels periodically
        if (timeSinceLastUpdate_ >= updateInterval_) {
            updateAllLODLevels();
            timeSinceLastUpdate_ = 0.0f;
        }

        // Update bodies based on their LOD
        updateBodiesByLOD(deltaTime);

        // Adjust quality based on performance
        if (autoAdjust_) {
            adjustQualityForPerformance(deltaTime);
        }
    }

    void PhysicsLODSystem::registerBody(RigidBody* body, const Float importance) {
        if (!body) return;

        BodyLODInfo info;
        info.body = body;
        info.importance = importance;
        info.level = PhysicsLODLevel::FULL;
        info.timeSinceLastUpdate = 0.0f;

        bodies_[body] = info;
        bodiesByLevel_[PhysicsLODLevel::FULL].insert(body);
    }

    void PhysicsLODSystem::unregisterBody(RigidBody* body) {
        if (const auto it = bodies_.find(body); it != bodies_.end()) {
            bodiesByLevel_[it->second.level].erase(body);
            bodies_.erase(it);
        }
    }

    void PhysicsLODSystem::setBodyImportance(RigidBody* body, const Float importance) {
        if (const auto it = bodies_.find(body); it != bodies_.end()) {
            it->second.importance = importance;
        }
    }

    PhysicsLODLevel PhysicsLODSystem::getBodyLOD(RigidBody* body) const {
        const auto it = bodies_.find(body);
        return it != bodies_.end() ? it->second.level : PhysicsLODLevel::FULL;
    }

    PhysicsLODSystem::Statistics PhysicsLODSystem::getStatistics() const {
        Statistics stats;
        stats.totalBodies = bodies_.size();

        for (const auto& [level, bodies] : bodiesByLevel_) {
            switch (level) {
            case PhysicsLODLevel::FULL:
                stats.bodiesAtFull = bodies.size();
                break;
            case PhysicsLODLevel::SIMPLIFIED:
                stats.bodiesAtSimplified = bodies.size();
                break;
            case PhysicsLODLevel::MINIMAL:
                stats.bodiesAtMinimal = bodies.size();
                break;
            case PhysicsLODLevel::FROZEN:
                stats.bodiesFrozen = bodies.size();
                break;
            case PhysicsLODLevel::CULLED:
                stats.bodiesCulled = bodies.size();
                break;
            }
        }

        // Calculate average update frequency
        Float totalFreq = 0;
        for (const auto& info : bodies_ | std::views::values) {
            totalFreq += lodConfigs_[static_cast<int>(info.level)].updateFrequency;
        }
        stats.averageUpdateFrequency = bodies_.empty() ? 0 : totalFreq / bodies_.size();
        stats.qualityFactor = qualityScale_;

        return stats;
    }

    void PhysicsLODSystem::updateAllLODLevels() {
        // Priority queue to find most important bodies
        using BodyPriority = std::pair<Float, RigidBody*>;
        std::priority_queue<BodyPriority> priorityQueue;

        // Calculate priorities for all bodies
        for (auto& [body, info] : bodies_) {
            if (!body) continue;

            // Calculate distance
            info.distanceToViewer = glm::length(body->getPosition() - viewerPosition_);

            // Calculate priority (lower distance + higher importance = higher priority)
            Float priority = info.importance * 1000.0f / (info.distanceToViewer + 1.0f);

            // Boost priority for active bodies
            if (body->isActive()) {
                priority *= 2.0f;
            }

            // Boost priority for bodies with high velocity
            if (const Float speed = glm::length(body->getLinearVelocity()); speed > 10.0f) {
                priority *= 1.5f;
            }

            priorityQueue.push({priority, body});
        }

        // Assign LOD levels based on priority
        std::size_t fullCount = 0;
        while (!priorityQueue.empty()) {
            auto [priority, body] = priorityQueue.top();
            priorityQueue.pop();

            auto& info = bodies_[body];
            PhysicsLODLevel oldLevel = info.level;
            PhysicsLODLevel newLevel;

            // Determine new level based on distance and limits
            if (info.distanceToViewer > lodDistances_[3]) {
                newLevel = PhysicsLODLevel::CULLED;
            }
            else if (info.distanceToViewer > lodDistances_[2]) {
                newLevel = PhysicsLODLevel::FROZEN;
            }
            else if (info.distanceToViewer > lodDistances_[1]) {
                newLevel = PhysicsLODLevel::MINIMAL;
            }
            else if (info.distanceToViewer > lodDistances_[0] ||
                fullCount >= maxFullLODBodies_) {
                newLevel = PhysicsLODLevel::SIMPLIFIED;
            }
            else {
                newLevel = PhysicsLODLevel::FULL;
                fullCount++;
            }

            // Apply hysteresis to prevent LOD flickering
            if (oldLevel != newLevel) {
                if (static_cast<int>(newLevel) > static_cast<int>(oldLevel)) {
                    constexpr Float hysteresis = 5.0f;
                    // Downgrading LOD - add hysteresis
                    if (const Float adjustedDistance = info.distanceToViewer - hysteresis; adjustedDistance <=
                        lodDistances_[static_cast<int>(oldLevel)]) {
                        newLevel = oldLevel; // Keep current level
                    }
                }
            }

            // Update level if changed
            if (oldLevel != newLevel) {
                bodiesByLevel_[oldLevel].erase(body);
                bodiesByLevel_[newLevel].insert(body);
                info.level = newLevel;

                applyLODToBody(body, newLevel);
            }
        }
    }

    void PhysicsLODSystem::updateBodiesByLOD(const Float deltaTime) {
        // Update bodies at different frequencies based on LOD
        for (auto& info : bodies_ | std::views::values) {
            const LODConfig& config = lodConfigs_[static_cast<int>(info.level)];

            if (config.updateFrequency <= 0) continue; // Frozen/culled

            const Float updateInterval = 1.0f / config.updateFrequency;
            info.timeSinceLastUpdate += deltaTime;

            if (info.timeSinceLastUpdate >= updateInterval) {
                // Body needs update
                info.timeSinceLastUpdate = 0.0f;

                // Apply LOD-specific settings
                // This would interface with the physics world to adjust settings
            }
        }
    }

    void PhysicsLODSystem::applyLODToBody(RigidBody* body, PhysicsLODLevel level) {
        if (!body) return;

        const auto& info = bodies_[body];
        const LODConfig& config = lodConfigs_[static_cast<int>(level)];

        switch (level) {
        case PhysicsLODLevel::FULL:
            // Restore original shape if it was simplified
            if (info.originalShape && body->getShape() != info.originalShape) {
                // body->setShape(info.originalShape);
            }
            body->activate();
            break;

        case PhysicsLODLevel::SIMPLIFIED:
            // Use simplified collision shape
            if (!info.simplifiedShape && info.originalShape) {
                // Create simplified version (e.g., box or sphere)
                // info.simplifiedShape = createSimplifiedShape(info.originalShape);
            }
            if (info.simplifiedShape) {
                // body->setShape(info.simplifiedShape);
            }
            break;

        case PhysicsLODLevel::MINIMAL:
            // Reduce collision detection quality
            body->setCCDMotionThreshold(INFINITY_VALUE<Float>);
            break;

        case PhysicsLODLevel::FROZEN:
            // Put body to sleep
            body->sleep();
            break;

        case PhysicsLODLevel::CULLED:
            // Completely deactivate
            body->setActivationState(ActivationState::DISABLE_SIMULATION_STATE);
            break;
        }
    }

    void PhysicsLODSystem::adjustQualityForPerformance(const Float deltaTime) {
        // Simple feedback controller to maintain target frame time
        const Float error = currentFrameTime_ - targetFrameTime_;
        const Float adjustment = error * 0.01f; // Proportional control

        qualityScale_ = clamp(qualityScale_ - adjustment, 0.1f, 1.0f);

        // Adjust LOD distances based on quality scale
        const Float distanceScale = 2.0f - qualityScale_; // Inverse relationship

        lodDistances_[0] = performance::LOD_DISTANCE_HIGH * distanceScale;
        lodDistances_[1] = performance::LOD_DISTANCE_MEDIUM * distanceScale;
        lodDistances_[2] = performance::LOD_DISTANCE_LOW * distanceScale;
        lodDistances_[3] = performance::LOD_DISTANCE_CULL * distanceScale;

        // Adjust max full LOD bodies
        maxFullLODBodies_ = static_cast<std::size_t>(100 * qualityScale_);
    }
} // namespace engine::physics
