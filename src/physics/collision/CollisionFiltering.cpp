/**
 * @file CollisionFiltering.cpp
 * @brief Collision filtering and layer management
 * @details Implements collision layers, masks, and custom filtering rules
 *          to control which objects can collide with each other
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#include "CollisionFiltering.h"

namespace engine::physics {
    void CollisionFilter::defineLayer(const std::string& name, const std::uint16_t bit) {
        if (bit >= 16) {
            // Invalid bit position
            return;
        }

        layers_[name] = CollisionLayer(name, 1 << bit, ALL);
        layersByBit_[1 << bit] = &layers_[name];
    }

    void CollisionFilter::setLayerCollision(const std::string& layerA, const std::string& layerB,
                                            const bool canCollide) {
        const auto itA = layers_.find(layerA);
        const auto itB = layers_.find(layerB);

        if (itA == layers_.end() || itB == layers_.end()) {
            return;
        }

        if (canCollide) {
            // Enable collision
            itA->second.collidesWith |= itB->second.bit;
            itB->second.collidesWith |= itA->second.bit;
        }
        else {
            // Disable collision
            itA->second.collidesWith &= ~itB->second.bit;
            itB->second.collidesWith &= ~itA->second.bit;
        }
    }

    void CollisionFilter::setLayerEnabled(const std::string& layer, const bool enabled) {
        if (const auto it = layers_.find(layer); it != layers_.end()) {
            it->second.enabled = enabled;
        }
    }

    CollisionLayer* CollisionFilter::getLayer(const std::string& name) {
        const auto it = layers_.find(name);
        return it != layers_.end() ? &it->second : nullptr;
    }

    void CollisionFilter::setupCollisionMatrix(const std::vector<std::pair<std::string, std::string>>& pairs) {
        // First, disable all collisions
        for (auto& layer : layers_ | std::views::values) {
            layer.collidesWith = 0;
        }

        // Then enable specific pairs
        for (const auto& [layerA, layerB] : pairs) {
            setLayerCollision(layerA, layerB, true);
        }
    }

    std::string CollisionFilter::getCollisionMatrixString() const {
        std::stringstream ss;
        ss << "Collision Matrix:\n";
        ss << std::setw(15) << " ";

        // Header
        for (const auto& name : layers_ | std::views::keys) {
            ss << std::setw(10) << name.substr(0, 9);
        }
        ss << "\n";

        // Rows
        for (const auto& [nameA, layerA] : layers_) {
            ss << std::setw(15) << nameA.substr(0, 14);

            for (const auto& layerB : layers_ | std::views::values) {
                const bool collides = (layerA.collidesWith & layerB.bit) != 0;
                ss << std::setw(10) << (collides ? "YES" : "-");
            }
            ss << "\n";
        }

        return ss.str();
    }

    bool CollisionFilter::shouldCollide(RigidBody* bodyA, RigidBody* bodyB) const {
        if (!bodyA || !bodyB) return false;

        // Check if same body
        if (bodyA == bodyB) return false;

        // Check body types
        if (!shouldBodyTypesCollide(bodyA->getType(), bodyB->getType())) {
            return false;
        }

        // Check collision groups and masks
        const std::uint16_t groupA = bodyA->getCollisionGroup();
        const std::uint16_t groupB = bodyB->getCollisionGroup();
        const std::uint16_t maskA = bodyA->getCollisionMask();
        const std::uint16_t maskB = bodyB->getCollisionMask();

        if (const bool groupTest = (groupA & maskB) != 0 && (groupB & maskA) != 0; !groupTest) return false;

        // Check layer collision
        const auto itA = layersByBit_.find(groupA);

        if (const auto itB = layersByBit_.find(groupB); itA != layersByBit_.end() && itB != layersByBit_.end()) {
            if (!itA->second->enabled || !itB->second->enabled) {
                return false;
            }

            if ((itA->second->collidesWith & itB->second->bit) == 0) {
                return false;
            }
        }

        // Check custom filters
        for (const auto& filter : customFilters_) {
            if (!filter(bodyA, bodyB)) {
                return false;
            }
        }

        // Check exclusion pairs
        if (CollisionPair pair(bodyA, bodyB); exclusionPairs_.contains(pair)) {
            return false;
        }

        return true;
    }

    void CollisionFilter::addExclusionPair(RigidBody* bodyA, RigidBody* bodyB) {
        exclusionPairs_.insert(CollisionPair(bodyA, bodyB));
    }

    void CollisionFilter::removeExclusionPair(RigidBody* bodyA, RigidBody* bodyB) {
        exclusionPairs_.erase(CollisionPair(bodyA, bodyB));
    }

    void CollisionFilter::clearExclusionPairs() {
        exclusionPairs_.clear();
    }

    bool CollisionFilter::isExcluded(RigidBody* bodyA, RigidBody* bodyB) const {
        return exclusionPairs_.contains(CollisionPair(bodyA, bodyB));
    }

    void CollisionFilter::setupRPGFiltering() {
        clearAllLayers();

        defineLayer("Player", 0);
        defineLayer("Enemy", 1);
        defineLayer("NPC", 2);
        defineLayer("Static", 3);
        defineLayer("Pickup", 4);
        defineLayer("Trigger", 5);
        defineLayer("Projectile", 6);
        defineLayer("Debris", 7);
        defineLayer("Invisible", 8);

        // Define collision pairs
        std::vector<std::pair<std::string, std::string>> collisionPairs = {
            {"Player", "Enemy"},
            {"Player", "NPC"},
            {"Player", "Static"},
            {"Player", "Pickup"},
            {"Player", "Trigger"},
            {"Enemy", "Enemy"},
            {"Enemy", "Static"},
            {"Enemy", "Projectile"},
            {"NPC", "Static"},
            {"Projectile", "Static"},
            {"Debris", "Static"},
            {"Debris", "Debris"}
        };

        setupCollisionMatrix(collisionPairs);
    }

    void CollisionFilter::setupPlatformerFiltering() {
        clearAllLayers();

        defineLayer("Player", 0);
        defineLayer("Platform", 1);
        defineLayer("OneWayPlatform", 2);
        defineLayer("Enemy", 3);
        defineLayer("Collectible", 4);
        defineLayer("Hazard", 5);
        defineLayer("Moving", 6);

        std::vector<std::pair<std::string, std::string>> collisionPairs = {
            {"Player", "Platform"},
            {"Player", "OneWayPlatform"},
            {"Player", "Enemy"},
            {"Player", "Collectible"},
            {"Player", "Hazard"},
            {"Player", "Moving"},
            {"Enemy", "Platform"},
            {"Enemy", "Moving"}
        };

        setupCollisionMatrix(collisionPairs);

        // Add custom filter for one-way platforms
        addCustomFilter([](RigidBody* bodyA, RigidBody* bodyB) {
            // Check if one is a one-way platform
            bool isOneWayA = bodyA->getCollisionGroup() == (1 << 2);
            bool isOneWayB = bodyB->getCollisionGroup() == (1 << 2);

            if (isOneWayA || isOneWayB) {
                // Only collide if approaching from above
                RigidBody* platform = isOneWayA ? bodyA : bodyB;
                RigidBody* other = isOneWayA ? bodyB : bodyA;

                Vec3 relVel = other->getLinearVelocity() - platform->getLinearVelocity();
                return relVel.y < 0; // Moving down
            }

            return true;
        });
    }

    void CollisionFilter::initializeDefaultLayers() {
        // Set up default collision layers
        defineLayer("Default", 0);
        defineLayer("Static", 1);
        defineLayer("Kinematic", 2);
        defineLayer("Dynamic", 3);
        defineLayer("Trigger", 4);
        defineLayer("Character", 5);
        defineLayer("Vehicle", 6);
        defineLayer("Projectile", 7);

        // All layers collide with all by default
        for (auto& layer : layers_ | std::views::values) {
            layer.collidesWith = CollisionGroup::ALL;
        }
    }

    bool CollisionFilter::shouldBodyTypesCollide(const BodyType typeA, const BodyType typeB) {
        // Static bodies don't collide with each other
        if (typeA == BodyType::STATIC && typeB == BodyType::STATIC) {
            return false;
        }

        // Ghost bodies don't generate contact responses
        // but we still want to detect the collision

        return true;
    }
} // namespace engine::physics
