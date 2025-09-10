// TODO: In the future, move it to the module manager, as it is a preset

/**
 * @file DestructionSystem.h
 * @brief Destruction and fracture simulation system
 * @details Implements breakable objects, fracturing, debris generation,
 *          and structural damage simulation
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

#include "../core/PhysicsTypes.h"
#include "../core/PhysicsConstants.h"
#include "../dynamics/RigidBody.h"
#include "../collision/CollisionShapes.h"
#include "../../math/MathSystem.h"
#include "../../math/utils/Random.h"
#include "../../memory/manager/MemoryManager.h"
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionDispatch/btCompoundCompoundCollisionAlgorithm.h>
#include <functional>
#include <memory>
#include <vector>
#include <unordered_map>

namespace engine::physics {

    using namespace engine::math;
    using namespace engine::memory;

    /**
     * @brief Fracture pattern type
     */
    enum class FracturePattern : std::uint8_t {
        VORONOI,        // Realistic irregular pieces
        UNIFORM_GRID,   // Regular grid pattern
        RADIAL,         // Radial pattern from impact point
        SPIRAL,         // Spiral fracture pattern
        CUSTOM          // User-defined pattern
    };

    /**
     * @brief Destruction material properties
     */
    struct DestructionMaterial {
        Float tensileStrength = 10.0f;      // Maximum tensile stress before breaking
        Float compressiveStrength = 50.0f;  // Maximum compressive stress
        Float shearStrength = 15.0f;        // Maximum shear stress
        Float toughness = 1.0f;             // Resistance to crack propagation
        Float density = 2400.0f;            // kg/m³

        // Fracture properties
        Float fractureEnergy = 100.0f;      // Energy required to create new surfaces
        Float damageAccumulation = 0.8f;    // Damage retention factor (0-1)
        Float debrisScale = 1.0f;           // Scale factor for debris pieces
        Int minFragments = 4;               // Minimum number of fragments
        Int maxFragments = 20;              // Maximum number of fragments

        // Particle effects
        bool generateParticles = true;      // Generate particle effects
        Int particlesPerFragment = 10;      // Number of particles per fragment
        Float particleLifetime = 2.0f;      // Particle lifetime in seconds

        // Sound
        std::string breakSound;             // Sound effect path
        Float soundVolume = 1.0f;          // Sound volume multiplier

        // Material presets
        static DestructionMaterial Glass();
        static DestructionMaterial Concrete();
        static DestructionMaterial Wood();
        static DestructionMaterial Metal();
    };

    /**
     * @brief Fractured piece of a destructible object
     */
    struct FractureFragment {
        std::unique_ptr<RigidBody> body;
        std::unique_ptr<CollisionShape> shape;
        std::vector<Vec3> vertices;
        std::vector<Int> indices;
        Float volume;
        Float mass;
        Vec3 localPosition;     // Position relative to original object
        Quat localRotation;     // Rotation relative to original object
        Float lifetime = -1.0f; // Lifetime before removal (-1 = infinite)

        FractureFragment();
    };

    // Forward declaration
    class DestructionSystem;

    /**
     * @brief Destructible object component
     */
    class DestructibleObject {
    public:
        /**
         * @brief Constructor
         * @param body The rigid body to make destructible
         * @param material The destruction material properties
         * @param memoryManager Memory manager for allocations
         * @param rng Random number generator
         */
        DestructibleObject(RigidBody* body,
                          const DestructionMaterial& material,
                          MemoryManager* memoryManager,
                          Random* rng);

        /**
         * @brief Apply damage to the object
         * @param damage Amount of damage to apply
         * @param impactPoint World position of impact
         * @param impactDirection Direction of impact force
         * @return True if object broke from this damage
         */
        bool applyDamage(Float damage, const Vec3& impactPoint, const Vec3& impactDirection);

        /**
         * @brief Apply impact force and check for breakage
         * @param impactPoint World position of impact
         * @param impulse Impact impulse vector
         * @return True if object broke from this impact
         */
        bool applyImpact(const Vec3& impactPoint, const Vec3& impulse);

        /**
         * @brief Update destructible object
         * @param deltaTime Time since last update
         */
        void update(Float deltaTime);

        /**
         * @brief Check if object is broken
         */
        bool isBroken() const { return m_isBroken; }

        /**
         * @brief Get current accumulated damage
         */
        Float getCurrentDamage() const { return m_currentDamage; }

        /**
         * @brief Get damage as ratio of maximum strength
         */
        Float getDamageRatio() const { return m_currentDamage / m_material.tensileStrength; }

        /**
         * @brief Get fragments created from breaking
         */
        const std::vector<FractureFragment>& getFragments() const { return m_fragments; }

        /**
         * @brief Get time since object broke
         */
        Float getTimeSinceBreak() const { return m_timeSinceBreak; }

        /**
         * @brief Reset object to unbroken state
         */
        void reset();

    private:
        RigidBody* m_originalBody;
        DestructionMaterial m_material;
        Float m_currentDamage;
        bool m_isBroken;
        Float m_timeSinceBreak;

        std::vector<FractureFragment> m_fragments;
        std::vector<Vec3> m_impactPoints;
        std::vector<Vec3> m_impactDirections;

        // Mesh data for fracturing
        std::vector<Vec3> m_originalVertices;
        std::vector<Int> m_originalIndices;
        AABB m_boundingBox;

        // Dependencies
        MemoryManager* m_memoryManager;
        Random* m_rng;

        /**
         * @brief Initialize the destructible object
         */
        void initialize();

        /**
         * @brief Fracture the object into pieces
         */
        void fracture(const Vec3& impactPoint, const Vec3& impactDirection, Float energy);

        /**
         * @brief Generate fracture points using Voronoi pattern
         */
        std::vector<Vec3> generateFracturePoints(const Vec3& impactPoint,
                                                const Vec3& impactDirection,
                                                Int numPoints);

        /**
         * @brief Create fragment bodies from fracture points
         */
        void createFragments(const std::vector<Vec3>& fracturePoints,
                           const Vec3& impactPoint,
                           const Vec3& impactDirection);

        /**
         * @brief Apply velocities to fragments based on impact
         */
        void applyFragmentVelocities(const Vec3& impactPoint,
                                    const Vec3& impactDirection,
                                    Float energy);

        /**
         * @brief Calculate stress at a point
         */
        Float calculateStress(const Vec3& point, const Vec3& direction, Float force);

        /**
         * @brief Generate particle effects at impact point
         */
        void generateParticleEffects(const Vec3& impactPoint, Int numFragments);

        /**
         * @brief Play break sound effect
         */
        void playBreakSound(const Vec3& position);
    };

    /**
     * @brief Manages all destructible objects in the scene
     */
    class DestructionSystem {
    public:
        /**
         * @brief Constructor
         * @param world Physics world reference
         * @param memoryManager Memory manager for allocations
         * @param rng Random number generator (optional, will create if null)
         */
        DestructionSystem(PhysicsWorld* world,
                         MemoryManager* memoryManager,
                         Random* rng = nullptr);

        /**
         * @brief Destructor
         */
        ~DestructionSystem();

        // Disable copy operations
        DestructionSystem(const DestructionSystem&) = delete;
        DestructionSystem& operator=(const DestructionSystem&) = delete;

        /**
         * @brief Register a destructible object
         * @param body The rigid body to make destructible
         * @param material Destruction material properties
         * @return Pointer to the created destructible object
         */
        DestructibleObject* registerDestructible(RigidBody* body,
                                                const DestructionMaterial& material);

        /**
         * @brief Unregister a destructible object
         * @param body The rigid body to remove from destruction system
         */
        void unregisterDestructible(RigidBody* body);

        /**
         * @brief Process collision for potential destruction
         * @param manifold Collision information
         */
        void processCollision(const CollisionManifold& manifold);

        /**
         * @brief Apply explosion damage to all destructibles in range
         * @param center Center of explosion
         * @param radius Effect radius
         * @param force Explosion force magnitude
         */
        void applyExplosion(const Vec3& center, Float radius, Float force);

        /**
         * @brief Update all destructible objects
         * @param deltaTime Time since last update
         */
        void update(Float deltaTime);

        /**
         * @brief Set impact threshold for automatic destruction
         * @param threshold Minimum impact speed to cause damage
         */
        void setImpactThreshold(Float threshold) { m_impactThreshold = threshold; }

        /**
         * @brief Get impact threshold
         */
        Float getImpactThreshold() const { return m_impactThreshold; }

        /**
         * @brief Destruction event callback type
         */
        using DestructionCallback = std::function<void(RigidBody*, DestructibleObject*)>;

        /**
         * @brief Register destruction callback
         * @param callback Function to call when objects are destroyed
         */
        void setDestructionCallback(const DestructionCallback& callback);

        /**
         * @brief System statistics
         */
        struct Statistics {
            std::size_t totalDestructibles;
            std::size_t brokenObjects;
            std::size_t totalFragments;
            Float averageDamage;
        };

        /**
         * @brief Get system statistics
         */
        Statistics getStatistics() const;

    private:
        PhysicsWorld* m_physicsWorld;
        MemoryManager* m_memoryManager;
        std::unique_ptr<Random> m_ownedRng;  // Owned if we created it
        Random* m_rng;                        // Pointer to actual random generator

        std::unordered_map<RigidBody*, std::unique_ptr<DestructibleObject>> m_destructibles;
        std::vector<RigidBody*> m_toRemove;

        Float m_impactThreshold = 5.0f; // Minimum impact speed for damage
        DestructionCallback m_destructionCallback;

        /**
         * @brief Handle object destruction event
         */
        void onObjectDestroyed(RigidBody* body, DestructibleObject* destructible);
    };

    /**
     * @brief Structural integrity simulation for buildings
     */
    class StructuralIntegrity {
    public:
        /**
         * @brief Beam connection data
         */
        struct BeamConnection {
            RigidBody* bodyA;
            RigidBody* bodyB;
            btTypedConstraint* constraint;
            Float maxForce;
            Float currentLoad;
            bool isBroken;
        };

        /**
         * @brief Constructor
         * @param world Physics world reference
         * @param rng Random number generator
         */
        StructuralIntegrity(PhysicsWorld* world, Random* rng);

        /**
         * @brief Add structural connection between two bodies
         * @param bodyA First rigid body
         * @param bodyB Second rigid body
         * @param constraint Physics constraint connecting them
         * @param maxForce Maximum force before connection breaks
         */
        void addConnection(RigidBody* bodyA, RigidBody* bodyB,
                          btTypedConstraint* constraint, Float maxForce);

        /**
         * @brief Update structural integrity simulation
         * @param deltaTime Time since last update
         */
        void update(Float deltaTime);

        /**
         * @brief Get all connections
         */
        const std::vector<BeamConnection>& getConnections() const { return m_connections; }

    private:
        PhysicsWorld* m_world;
        Random* m_rng;
        std::vector<BeamConnection> m_connections;

        /**
         * @brief Propagate failure to adjacent connections
         */
        void propagateFailure(const BeamConnection& broken);
    };

} // namespace engine::physics

//
// /**
//  * @file DestructionSystem.h
//  * @brief Destruction and fracture simulation system
//  * @details Implements breakable objects, fracturing, debris generation,
//  *          and structural damage simulation
//  * @author Andrés Guerrero
//  * @date 31-08-2025
//  */
//
// #pragma once
//
// #include "../core/PhysicsTypes.h"
// #include "../core/PhysicsConstants.h"
// #include "../dynamics/RigidBody.h"
// #include "../collision/CollisionShapes.h"
// #include "../../math/MathSystem.h"
// #include "../../memory/MemorySystem.h"
// #include <btBulletDynamicsCommon.h>
// #include <BulletCollision/CollisionDispatch/btCompoundCompoundCollisionAlgorithm.h>
//
// namespace engine::physics {
//
//     using namespace engine::math;
//     using namespace engine::memory;
//
//     /**
//      * @brief Fracture pattern type
//      */
//     enum class FracturePattern : std::uint8_t {
//         VORONOI,        // Realistic irregular pieces
//         UNIFORM_GRID,   // Regular grid pattern
//         RADIAL,         // Radial pattern from impact point
//         SPIRAL,         // Spiral fracture pattern
//         CUSTOM          // User-defined pattern
//     };
//
//     /**
//      * @brief Destruction material properties
//      */
//     struct DestructionMaterial {
//         Float tensileStrength = 10.0f;      // Maximum tensile stress before breaking
//         Float compressiveStrength = 50.0f;  // Maximum compressive stress
//         Float shearStrength = 15.0f;        // Maximum shear stress
//         Float toughness = 1.0f;             // Resistance to crack propagation
//         Float density = 2400.0f;            // kg/m³
//
//         // Fracture properties
//         Float fractureEnergy = 100.0f;      // Energy required to create new surfaces
//         Float damageAccumulation = 0.8f;    // Damage retention factor (0-1)
//         Float debrisScale = 1.0f;           // Scale factor for debris pieces
//         Int minFragments = 4;               // Minimum number of fragments
//         Int maxFragments = 20;              // Maximum number of fragments
//
//         // Particle effects
//         bool generateParticles = true;      // Generate particle effects
//         Int particlesPerFragment = 10;      // Number of particles per fragment
//         Float particleLifetime = 2.0f;      // Particle lifetime in seconds
//
//         // Sound
//         std::string breakSound;             // Sound effect path
//         Float soundVolume = 1.0f;          // Sound volume multiplier
//
//         // Presets
//         static DestructionMaterial Glass() {
//             DestructionMaterial mat;
//             mat.tensileStrength = 5.0f;
//             mat.compressiveStrength = 100.0f;
//             mat.shearStrength = 3.0f;
//             mat.toughness = 0.1f;
//             mat.density = 2500.0f;
//             mat.minFragments = 10;
//             mat.maxFragments = 50;
//             return mat;
//         }
//
//         static DestructionMaterial Concrete() {
//             DestructionMaterial mat;
//             mat.tensileStrength = 3.0f;
//             mat.compressiveStrength = 30.0f;
//             mat.shearStrength = 5.0f;
//             mat.toughness = 0.5f;
//             mat.density = 2400.0f;
//             mat.minFragments = 5;
//             mat.maxFragments = 15;
//             return mat;
//         }
//
//         static DestructionMaterial Wood() {
//             DestructionMaterial mat;
//             mat.tensileStrength = 40.0f;
//             mat.compressiveStrength = 30.0f;
//             mat.shearStrength = 10.0f;
//             mat.toughness = 2.0f;
//             mat.density = 600.0f;
//             mat.minFragments = 2;
//             mat.maxFragments = 8;
//             return mat;
//         }
//
//         static DestructionMaterial Metal() {
//             DestructionMaterial mat;
//             mat.tensileStrength = 200.0f;
//             mat.compressiveStrength = 200.0f;
//             mat.shearStrength = 100.0f;
//             mat.toughness = 10.0f;
//             mat.density = 7800.0f;
//             mat.minFragments = 2;
//             mat.maxFragments = 5;
//             return mat;
//         }
//     };
//
//     /**
//      * @brief Fractured piece of a destructible object
//      */
//     struct FractureFragment {
//         std::unique_ptr<RigidBody> body;
//         std::unique_ptr<CollisionShape> shape;
//         std::vector<Vec3> vertices;
//         std::vector<Int> indices;
//         Float volume;
//         Float mass;
//         Vec3 localPosition;     // Position relative to original object
//         Quat localRotation;     // Rotation relative to original object
//         Float lifetime = -1.0f; // Lifetime before removal (-1 = infinite)
//
//         FractureFragment() : volume(0), mass(0), localRotation(1,0,0,0) {}
//     };
//
//     /**
//      * @brief Destructible object component
//      */
//     class DestructibleObject {
//     public:
//         DestructibleObject(RigidBody* body, const DestructionMaterial& material)
//             : m_originalBody(body), m_material(material), m_currentDamage(0),
//               m_isBroken(false), m_timeSinceBreak(0) {
//             initialize();
//         }
//
//         /**
//          * @brief Apply damage to the object
//          * @return True if object broke
//          */
//         bool applyDamage(Float damage, const Vec3& impactPoint, const Vec3& impactDirection);
//
//         /**
//          * @brief Apply impact force and check for breakage
//          */
//         bool applyImpact(const Vec3& impactPoint, const Vec3& impulse);
//
//         /**
//          * @brief Update destructible object
//          */
//         void update(Float deltaTime);
//
//         bool isBroken() const { return m_isBroken; }
//         Float getCurrentDamage() const { return m_currentDamage; }
//         Float getDamageRatio() const { return m_currentDamage / m_material.tensileStrength; }
//         const std::vector<FractureFragment>& getFragments() const { return m_fragments; }
//
//         /**
//          * @brief Reset object to unbroken state
//          */
//         void reset() {
//             m_isBroken = false;
//             m_currentDamage = 0;
//             m_timeSinceBreak = 0;
//             m_fragments.clear();
//             m_impactPoints.clear();
//             m_impactDirections.clear();
//
//             // Restore original body
//             // if (m_originalBody) {
//             //     m_originalBody->setEnabled(true);
//             // }
//         }
//
//     private:
//         RigidBody* m_originalBody;
//         DestructionMaterial m_material;
//         Float m_currentDamage;
//         bool m_isBroken;
//         Float m_timeSinceBreak;
//
//         std::vector<FractureFragment> m_fragments;
//         std::vector<Vec3> m_impactPoints;
//         std::vector<Vec3> m_impactDirections;
//
//         // Mesh data for fracturing
//         std::vector<Vec3> m_originalVertices;
//         std::vector<Int> m_originalIndices;
//         AABB m_boundingBox;
//
//         void initialize() {
//             if (!m_originalBody) return;
//
//             // Get mesh data from collision shape
//             // This would need to extract mesh from the shape
//             m_boundingBox = m_originalBody->getAABB();
//         }
//
//         /**
//          * @brief Fracture the object into pieces
//          */
//         void fracture(const Vec3& impactPoint, const Vec3& impactDirection, Float energy) {
//             if (m_isBroken) return;
//
//             m_isBroken = true;
//
//             // Disable original body
//             // if (m_originalBody) {
//             //     m_originalBody->setEnabled(false);
//             // }
//
//             // Calculate number of fragments based on energy
//             Int numFragments = lerp(m_material.minFragments, m_material.maxFragments,
//                                    saturate(energy / m_material.fractureEnergy));
//
//             // Generate fracture pattern
//             std::vector<Vec3> fracturePoints = generateFracturePoints(
//                 impactPoint, impactDirection, numFragments);
//
//             // Create fragments
//             createFragments(fracturePoints, impactPoint, impactDirection);
//
//             // Apply velocities to fragments
//             applyFragmentVelocities(impactPoint, impactDirection, energy);
//
//             // Generate particle effects
//             if (m_material.generateParticles) {
//                 generateParticleEffects(impactPoint, numFragments);
//             }
//
//             // Play sound effect
//             if (!m_material.breakSound.empty()) {
//                 playBreakSound(impactPoint);
//             }
//         }
//
//         /**
//          * @brief Generate fracture points using Voronoi
//          */
//         std::vector<Vec3> generateFracturePoints(const Vec3& impactPoint,
//                                                 const Vec3& impactDirection,
//                                                 Int numPoints) {
//             std::vector<Vec3> points;
//             points.reserve(numPoints);
//
//             // Add impact point
//             points.push_back(impactPoint);
//
//             // Generate random points with bias toward impact
//             for (Int i = 1; i < numPoints; ++i) {
//                 Vec3 randomPoint;
//
//                 // Use normal distribution centered at impact
//                 Float distance = std::abs(Random::gaussian(0.0f, 0.3f));
//                 Float theta = Random::uniform(0.0f, TWO_PI<Float>);
//                 Float phi = Random::uniform(0.0f, PI<Float>);
//
//                 randomPoint.x = impactPoint.x + distance * std::sin(phi) * std::cos(theta);
//                 randomPoint.y = impactPoint.y + distance * std::sin(phi) * std::sin(theta);
//                 randomPoint.z = impactPoint.z + distance * std::cos(phi);
//
//                 // Clamp to bounding box
//                 randomPoint = glm::clamp(randomPoint, m_boundingBox.min, m_boundingBox.max);
//
//                 points.push_back(randomPoint);
//             }
//
//             return points;
//         }
//
//         /**
//          * @brief Create fragment bodies from fracture points
//          */
//         void createFragments(const std::vector<Vec3>& fracturePoints,
//                            const Vec3& impactPoint,
//                            const Vec3& impactDirection) {
//             // This would use a Voronoi diagram or other algorithm to create fragments
//             // For now, create simple box fragments
//
//             Vec3 size = m_boundingBox.getSize();
//             Vec3 fragmentSize = size / std::cbrt(static_cast<Float>(fracturePoints.size()));
//
//             for (const auto& point : fracturePoints) {
//                 FractureFragment fragment;
//
//                 // Create fragment shape (simplified as box)
//                 fragment.shape = std::make_unique<BoxShape>(fragmentSize * 0.5f);
//
//                 // Calculate fragment mass
//                 fragment.volume = fragmentSize.x * fragmentSize.y * fragmentSize.z;
//                 fragment.mass = fragment.volume * m_material.density / 1000.0f;
//
//                 // Create fragment body
//                 BodyCreationParams params;
//                 params.shape = fragment.shape.get();
//                 params.mass = fragment.mass;
//                 params.type = BodyType::DYNAMIC;
//                 params.position = point;
//                 params.material.friction = 0.5f;
//                 params.material.restitution = 0.1f;
//
//                 // Fragment lifetime
//                 fragment.lifetime = m_material.particleLifetime * 2.0f;
//
//                 m_fragments.push_back(std::move(fragment));
//             }
//         }
//
//         /**
//          * @brief Apply velocities to fragments based on impact
//          */
//         void applyFragmentVelocities(const Vec3& impactPoint,
//                                     const Vec3& impactDirection,
//                                     Float energy) {
//             for (auto& fragment : m_fragments) {
//                 if (!fragment.body) continue;
//
//                 Vec3 fragmentCenter = fragment.body->getPosition();
//                 Vec3 toFragment = fragmentCenter - impactPoint;
//                 Float distance = glm::length(toFragment);
//
//                 if (distance > EPSILON) {
//                     toFragment /= distance;
//
//                     // Calculate velocity based on distance from impact
//                     Float velocityMagnitude = energy / (1.0f + distance);
//                     Vec3 velocity = (impactDirection * 0.7f + toFragment * 0.3f) * velocityMagnitude;
//
//                     // Add some randomness
//                     velocity.x += Random::uniform(-1.0f, 1.0f);
//                     velocity.y += Random::uniform(0.0f, 2.0f);
//                     velocity.z += Random::uniform(-1.0f, 1.0f);
//
//                     fragment.body->setLinearVelocity(velocity);
//
//                     // Add Random::angular velocity
//                     Vec3 angularVel(
//                         uniform(-5.0f, 5.0f),
//                         Random::uniform(-5.0f, 5.0f),
//                         Random::uniform(-5.0f, 5.0f)
//                     );
//                     fragment.body->setAngularVelocity(angularVel);
//                 }
//             }
//         }
//
//         /**
//          * @brief Calculate stress at a point
//          */
//         Float calculateStress(const Vec3& point, const Vec3& direction, Float force) {
//             // Simplified stress calculation
//             Float area = 0.01f; // Assumed contact area
//             Float stress = force / area;
//
//             // Apply material-specific factors
//             if (std::abs(direction.y) > 0.7f) {
//                 // Vertical load - use compressive strength
//                 return stress / m_material.compressiveStrength;
//             } else if (std::abs(glm::dot(direction, Vec3(1,0,0))) > 0.7f ||
//                       std::abs(glm::dot(direction, Vec3(0,0,1))) > 0.7f) {
//                 // Shear load
//                 return stress / m_material.shearStrength;
//             } else {
//                 // Tensile load
//                 return stress / m_material.tensileStrength;
//             }
//         }
//
//         void generateParticleEffects(const Vec3& impactPoint, Int numFragments) {
//             // This would interface with particle system
//             // Generate dust, debris particles, etc.
//         }
//
//         void playBreakSound(const Vec3& position) {
//             // This would interface with audio system
//             // Play break sound at position with volume
//         }
//     };
//
//     /**
//      * @brief Manages all destructible objects in the scene
//      */
//     class DestructionSystem {
//     public:
//         DestructionSystem(PhysicsWorld* world, IAllocator* allocator = nullptr)
//             : m_physicsWorld(world), m_allocator(allocator ? allocator :
//                                                   &MemoryManager::getInstance()) {
//             m_destructibles.reserve(256);
//         }
//
//         /**
//          * @brief Register a destructible object
//          */
//         DestructibleObject* registerDestructible(RigidBody* body,
//                                                 const DestructionMaterial& material) {
//             auto destructible = m_allocator->allocateObject<DestructibleObject>(
//                 MemoryCategory::PHYSICS, body, material);
//
//             m_destructibles[body] = std::unique_ptr<DestructibleObject>(destructible);
//             return destructible;
//         }
//
//         /**
//          * @brief Unregister a destructible object
//          */
//         void unregisterDestructible(RigidBody* body) {
//             m_destructibles.erase(body);
//         }
//
//         /**
//          * @brief Process collision for potential destruction
//          */
//         void processCollision(const CollisionManifold& manifold) {
//             // Check if either body is destructible
//             auto processBody = [this, &manifold](RigidBody* body, RigidBody* other, bool isBodyA) {
//                 auto it = m_destructibles.find(body);
//                 if (it != m_destructibles.end()) {
//                     // Calculate impact force
//                     Vec3 relativeVelocity = other->getLinearVelocity() - body->getLinearVelocity();
//                     Float impactSpeed = glm::length(relativeVelocity);
//
//                     if (impactSpeed > m_impactThreshold) {
//                         // Calculate impulse
//                         Float totalMass = body->getMass() + other->getMass();
//                         Float impulse = impactSpeed * totalMass;
//
//                         // Get impact point and normal
//                         Vec3 impactPoint = isBodyA ?
//                             manifold.contacts[0].positionOnA :
//                             manifold.contacts[0].positionOnB;
//                         Vec3 impactNormal = isBodyA ?
//                             manifold.contacts[0].normal :
//                             -manifold.contacts[0].normal;
//
//                         // Apply impact
//                         if (it->second->applyImpact(impactPoint, impactNormal * impulse)) {
//                             onObjectDestroyed(body, it->second.get());
//                         }
//                     }
//                 }
//             };
//
//             processBody(manifold.bodyA, manifold.bodyB, true);
//             processBody(manifold.bodyB, manifold.bodyA, false);
//         }
//
//         /**
//          * @brief Apply explosion damage
//          */
//         void applyExplosion(const Vec3& center, Float radius, Float force) {
//             for (auto& [body, destructible] : m_destructibles) {
//                 Vec3 toObject = body->getPosition() - center;
//                 Float distance = glm::length(toObject);
//
//                 if (distance < radius) {
//                     Float falloff = 1.0f - (distance / radius);
//                     Float damage = force * falloff * falloff;
//
//                     Vec3 direction = distance > EPSILON ? toObject / distance : Vec3(0,1,0);
//
//                     if (destructible->applyDamage(damage, center, direction)) {
//                         onObjectDestroyed(body, destructible.get());
//                     }
//                 }
//             }
//         }
//
//         /**
//          * @brief Update all destructible objects
//          */
//         void update(Float deltaTime) {
//             for (auto& [body, destructible] : m_destructibles) {
//                 destructible->update(deltaTime);
//
//                 // Clean up expired fragments
//                 if (destructible->isBroken()) {
//                     const auto& fragments = destructible->getFragments();
//                     if (fragments.empty() && destructible->getTimeSinceBreak() > 5.0f) {
//                         // Mark for removal
//                         m_toRemove.push_back(body);
//                     }
//                 }
//             }
//
//             // Remove marked objects
//             for (RigidBody* body : m_toRemove) {
//                 m_destructibles.erase(body);
//             }
//             m_toRemove.clear();
//         }
//
//         /**
//          * @brief Set impact threshold for automatic destruction
//          */
//         void setImpactThreshold(Float threshold) { m_impactThreshold = threshold; }
//
//         /**
//          * @brief Register destruction callback
//          */
//         using DestructionCallback = std::function<void(RigidBody*, DestructibleObject*)>;
//         void setDestructionCallback(const DestructionCallback& callback) {
//             m_destructionCallback = callback;
//         }
//
//         /**
//          * @brief Get statistics
//          */
//         struct Statistics {
//             std::size_t totalDestructibles;
//             std::size_t brokenObjects;
//             std::size_t totalFragments;
//             Float averageDamage;
//         };
//
//         Statistics getStatistics() const {
//             Statistics stats;
//             stats.totalDestructibles = m_destructibles.size();
//             stats.brokenObjects = 0;
//             stats.totalFragments = 0;
//             Float totalDamage = 0;
//
//             for (const auto& [body, destructible] : m_destructibles) {
//                 if (destructible->isBroken()) {
//                     stats.brokenObjects++;
//                     stats.totalFragments += destructible->getFragments().size();
//                 }
//                 totalDamage += destructible->getDamageRatio();
//             }
//
//             stats.averageDamage = stats.totalDestructibles > 0 ?
//                 totalDamage / stats.totalDestructibles : 0;
//
//             return stats;
//         }
//
//     private:
//         PhysicsWorld* m_physicsWorld;
//         IAllocator* m_allocator;
//
//         std::unordered_map<RigidBody*, std::unique_ptr<DestructibleObject>> m_destructibles;
//         std::vector<RigidBody*> m_toRemove;
//
//         Float m_impactThreshold = 5.0f; // Minimum impact speed for damage
//         DestructionCallback m_destructionCallback;
//
//         void onObjectDestroyed(RigidBody* body, DestructibleObject* destructible) {
//             // Add fragments to physics world
//             for (const auto& fragment : destructible->getFragments()) {
//                 if (fragment.body) {
//                     // Add to world
//                     // m_physicsWorld->addRigidBody(fragment.body.get());
//                 }
//             }
//
//             // Invoke callback
//             if (m_destructionCallback) {
//                 m_destructionCallback(body, destructible);
//             }
//         }
//
//         Float getTimeSinceBreak() const { return m_timeSinceBreak; }
//         Float m_timeSinceBreak = 0;
//     };
//
//     /**
//      * @brief Structural integrity simulation for buildings
//      */
//     class StructuralIntegrity {
//     public:
//         struct BeamConnection {
//             RigidBody* bodyA;
//             RigidBody* bodyB;
//             btTypedConstraint* constraint;
//             Float maxForce;
//             Float currentLoad;
//             bool isBroken;
//         };
//
//         StructuralIntegrity(PhysicsWorld* world) : world_(world) {}
//
//         /**
//          * @brief Add structural connection
//          */
//         void addConnection(RigidBody* bodyA, RigidBody* bodyB,
//                           btTypedConstraint* constraint, Float maxForce) {
//             BeamConnection connection;
//             connection.bodyA = bodyA;
//             connection.bodyB = bodyB;
//             connection.constraint = constraint;
//             connection.maxForce = maxForce;
//             connection.currentLoad = 0;
//             connection.isBroken = false;
//
//             connections_.push_back(connection);
//         }
//
//         /**
//          * @brief Update structural integrity
//          */
//         void update(Float deltaTime) {
//             // Check each connection for overload
//             for (auto& connection : connections_) {
//                 if (connection.isBroken) continue;
//
//                 // Get constraint force
//                 Float appliedImpulse = connection.constraint->getAppliedImpulse();
//                 connection.currentLoad = std::abs(appliedImpulse);
//
//                 // Check for failure
//                 if (connection.currentLoad > connection.maxForce) {
//                     // Break connection
//                     connection.isBroken = true;
//                     connection.constraint->setEnabled(false);
//
//                     // Propagate failure
//                     propagateFailure(connection);
//                 }
//             }
//
//             // Remove broken connections
//             connections_.erase(
//                 std::remove_if(connections_.begin(), connections_.end(),
//                     [](const BeamConnection& c) { return c.isBroken; }),
//                 connections_.end()
//             );
//         }
//
//     private:
//         PhysicsWorld* world_;
//         std::vector<BeamConnection> connections_;
//
//         Random& rng_;
//
//         void propagateFailure(const BeamConnection& broken) {
//             // Check adjacent connections for increased load
//             for (auto& connection : connections_) {
//                 if (connection.isBroken) continue;
//
//                 if (connection.bodyA == broken.bodyA || connection.bodyA == broken.bodyB ||
//                     connection.bodyB == broken.bodyA || connection.bodyB == broken.bodyB) {
//                     // Increase load on adjacent connections
//                     connection.currentLoad *= 1.5f;
//                 }
//             }
//         }
//     };
//
// } // namespace engine::physics