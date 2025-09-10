/**
 * @file DestructionSystem.cpp
 * @brief Destruction and fracture simulation system implementation
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#include "DestructionSystem.h"

#include "../../math/core/MathFunctions.h"

#include <cassert>
#include <algorithm>

namespace engine::physics {

    // ============================================================================
    // DestructionMaterial Material Presets
    // ============================================================================

    DestructionMaterial DestructionMaterial::Glass() {
        DestructionMaterial mat;
        mat.tensileStrength = 5.0f;
        mat.compressiveStrength = 100.0f;
        mat.shearStrength = 3.0f;
        mat.toughness = 0.1f;
        mat.density = 2500.0f;
        mat.minFragments = 10;
        mat.maxFragments = 50;
        mat.breakSound = "audio/glass_break.wav";
        return mat;
    }

    DestructionMaterial DestructionMaterial::Concrete() {
        DestructionMaterial mat;
        mat.tensileStrength = 3.0f;
        mat.compressiveStrength = 30.0f;
        mat.shearStrength = 5.0f;
        mat.toughness = 0.5f;
        mat.density = 2400.0f;
        mat.minFragments = 5;
        mat.maxFragments = 15;
        mat.breakSound = "audio/concrete_break.wav";
        return mat;
    }

    DestructionMaterial DestructionMaterial::Wood() {
        DestructionMaterial mat;
        mat.tensileStrength = 40.0f;
        mat.compressiveStrength = 30.0f;
        mat.shearStrength = 10.0f;
        mat.toughness = 2.0f;
        mat.density = 600.0f;
        mat.minFragments = 2;
        mat.maxFragments = 8;
        mat.breakSound = "audio/wood_break.wav";
        return mat;
    }

    DestructionMaterial DestructionMaterial::Metal() {
        DestructionMaterial mat;
        mat.tensileStrength = 200.0f;
        mat.compressiveStrength = 200.0f;
        mat.shearStrength = 100.0f;
        mat.toughness = 10.0f;
        mat.density = 7800.0f;
        mat.minFragments = 2;
        mat.maxFragments = 5;
        mat.breakSound = "audio/metal_break.wav";
        return mat;
    }

    // ============================================================================
    // FractureFragment Implementation
    // ============================================================================

    FractureFragment::FractureFragment()
        : volume(0), mass(0), localRotation(1, 0, 0, 0) {
    }

    // ============================================================================
    // DestructibleObject Implementation
    // ============================================================================

    DestructibleObject::DestructibleObject(RigidBody* body,
                                          const DestructionMaterial& material,
                                          MemoryManager* memoryManager,
                                          Random* rng)
        : m_originalBody(body)
        , m_material(material)
        , m_currentDamage(0)
        , m_isBroken(false)
        , m_timeSinceBreak(0)
        , m_memoryManager(memoryManager)
        , m_rng(rng) {

        assert(m_originalBody && "Original body cannot be null");
        assert(m_memoryManager && "Memory manager cannot be null");
        assert(m_rng && "Random number generator cannot be null");

        initialize();
    }

    bool DestructibleObject::applyDamage(Float damage, const Vec3& impactPoint, const Vec3& impactDirection) {
        if (m_isBroken) return false;

        m_currentDamage += damage * m_material.damageAccumulation;
        m_impactPoints.push_back(impactPoint);
        m_impactDirections.push_back(impactDirection);

        // Check if damage exceeds material strength
        if (m_currentDamage >= m_material.tensileStrength) {
            Float energy = m_currentDamage * m_material.fractureEnergy;
            fracture(impactPoint, impactDirection, energy);
            return true;
        }

        return false;
    }

    bool DestructibleObject::applyImpact(const Vec3& impactPoint, const Vec3& impulse) {
        if (m_isBroken) return false;

        Float impactMagnitude = glm::length(impulse);
        Float stress = calculateStress(impactPoint, glm::normalize(impulse), impactMagnitude);

        return applyDamage(stress, impactPoint, glm::normalize(impulse));
    }

    void DestructibleObject::update(Float deltaTime) {
        if (m_isBroken) {
            m_timeSinceBreak += deltaTime;

            // Update fragment lifetimes
            for (auto& fragment : m_fragments) {
                if (fragment.lifetime > 0) {
                    fragment.lifetime -= deltaTime;
                }
            }

            // Remove expired fragments
            m_fragments.erase(
                std::remove_if(m_fragments.begin(), m_fragments.end(),
                    [](const FractureFragment& fragment) {
                        return fragment.lifetime > 0 && fragment.lifetime <= 0;
                    }),
                m_fragments.end()
            );
        }
    }

    void DestructibleObject::reset() {
        m_isBroken = false;
        m_currentDamage = 0;
        m_timeSinceBreak = 0;
        m_fragments.clear();
        m_impactPoints.clear();
        m_impactDirections.clear();

        // Re-enable original body if needed
        // Implementation depends on your physics system
    }

    void DestructibleObject::initialize() {
        if (!m_originalBody) return;

        // Get mesh data from collision shape
        // This would need to extract mesh from the shape
        m_boundingBox = m_originalBody->getAABB();

        // For now, reserve space for potential vertices/indices
        m_originalVertices.reserve(1000);
        m_originalIndices.reserve(3000);
    }

    void DestructibleObject::fracture(const Vec3& impactPoint, const Vec3& impactDirection, Float energy) {
        if (m_isBroken) return;

        m_isBroken = true;
        m_timeSinceBreak = 0;

        // Disable original body
        // Implementation depends on your physics system

        // Calculate number of fragments based on energy
        Float energyRatio = energy / m_material.fractureEnergy;
        Int numFragments = static_cast<Int>(lerp(
            static_cast<Float>(m_material.minFragments),
            static_cast<Float>(m_material.maxFragments),
            saturate(energyRatio)
        ));

        // Generate fracture pattern
        std::vector<Vec3> fracturePoints = generateFracturePoints(
            impactPoint, impactDirection, numFragments);

        // Create fragments
        createFragments(fracturePoints, impactPoint, impactDirection);

        // Apply velocities to fragments
        applyFragmentVelocities(impactPoint, impactDirection, energy);

        // Generate particle effects
        if (m_material.generateParticles) {
            generateParticleEffects(impactPoint, numFragments);
        }

        // Play sound effect
        if (!m_material.breakSound.empty()) {
            playBreakSound(impactPoint);
        }
    }

    std::vector<Vec3> DestructibleObject::generateFracturePoints(const Vec3& impactPoint,
                                                                const Vec3& impactDirection,
                                                                Int numPoints) {
        std::vector<Vec3> points;
        points.reserve(numPoints);

        // Add impact point as first fracture point
        points.push_back(impactPoint);

        // Generate random points with bias toward impact
        for (Int i = 1; i < numPoints; ++i) {
            Vec3 randomPoint;

            // Use normal distribution centered at impact
            Float distance = std::abs(m_rng->gaussian(0.0f, 0.3f));
            Float theta = m_rng->range(0.0f, TWO_PI<Float>);
            Float phi = m_rng->range(0.0f, PI<Float>);

            randomPoint.x = impactPoint.x + distance * std::sin(phi) * std::cos(theta);
            randomPoint.y = impactPoint.y + distance * std::sin(phi) * std::sin(theta);
            randomPoint.z = impactPoint.z + distance * std::cos(phi);

            // Clamp to bounding box
            randomPoint = glm::clamp(randomPoint, m_boundingBox.min, m_boundingBox.max);

            points.push_back(randomPoint);
        }

        return points;
    }

    void DestructibleObject::createFragments(const std::vector<Vec3>& fracturePoints,
                                           const Vec3& impactPoint,
                                           const Vec3& impactDirection) {
        // This would use a Voronoi diagram or other algorithm to create fragments
        // For now, create simple box fragments

        Vec3 size = m_boundingBox.getSize();
        Vec3 fragmentSize = size / std::cbrt(static_cast<Float>(fracturePoints.size()));

        m_fragments.reserve(fracturePoints.size());

        for (const auto& point : fracturePoints) {
            FractureFragment fragment;

            // Create fragment shape (simplified as box)
            fragment.shape = std::make_unique<BoxShape>(fragmentSize * 0.5f);

            // Calculate fragment mass
            fragment.volume = fragmentSize.x * fragmentSize.y * fragmentSize.z;
            fragment.mass = fragment.volume * m_material.density / 1000.0f;

            // Create fragment body
            BodyCreationParams params;
            params.shape = ShapeCreationParams::Box(fragmentSize * 0.5f);
            params.mass = fragment.mass;
            params.type = BodyType::DYNAMIC;

            Transform fragmentTransform;
            fragmentTransform.setPosition(point);
            params.transform = &fragmentTransform;

            params.material.friction = 0.5f;
            params.material.restitution = 0.1f;

            // Create the rigid body
            // fragment.body = m_physicsWorld->createRigidBody(params);

            // Fragment lifetime
            fragment.lifetime = m_material.particleLifetime * 2.0f;

            // Set local transform
            fragment.localPosition = point - m_originalBody->getPosition();
            fragment.localRotation = Quat(1, 0, 0, 0);

            m_fragments.push_back(std::move(fragment));
        }
    }

    void DestructibleObject::applyFragmentVelocities(const Vec3& impactPoint,
                                                    const Vec3& impactDirection,
                                                    Float energy) {
        for (auto& fragment : m_fragments) {
            if (!fragment.body) continue;

            Vec3 fragmentCenter = fragment.body->getPosition();
            Vec3 toFragment = fragmentCenter - impactPoint;
            Float distance = glm::length(toFragment);

            if (distance > EPSILON) {
                toFragment /= distance;

                // Calculate velocity based on distance from impact
                Float velocityMagnitude = energy / (1.0f + distance);
                Vec3 velocity = (impactDirection * 0.7f + toFragment * 0.3f) * velocityMagnitude;

                // Add some randomness
                velocity.x += m_rng->range(-1.0f, 1.0f);
                velocity.y += m_rng->range(0.0f, 2.0f);
                velocity.z += m_rng->range(-1.0f, 1.0f);

                fragment.body->setLinearVelocity(velocity);

                // Add angular velocity
                Vec3 angularVel(
                    m_rng->range(-5.0f, 5.0f),
                    m_rng->range(-5.0f, 5.0f),
                    m_rng->range(-5.0f, 5.0f)
                );
                fragment.body->setAngularVelocity(angularVel);
            }
        }
    }

    Float DestructibleObject::calculateStress(const Vec3& point, const Vec3& direction, Float force) {
        // Simplified stress calculation
        constexpr Float area = 0.01f; // Assumed contact area
        Float stress = force / area;

        // Apply material-specific factors
        if (std::abs(direction.y) > 0.7f) {
            // Vertical load - use compressive strength
            return stress / m_material.compressiveStrength;
        } else if (std::abs(glm::dot(direction, Vec3(1, 0, 0))) > 0.7f ||
                  std::abs(glm::dot(direction, Vec3(0, 0, 1))) > 0.7f) {
            // Shear load
            return stress / m_material.shearStrength;
        } else {
            // Tensile load
            return stress / m_material.tensileStrength;
        }
    }

    void DestructibleObject::generateParticleEffects(const Vec3& impactPoint, Int numFragments) {
        // This would interface with particle system
        // Generate dust, debris particles, etc.
        // Implementation depends on your particle system
    }

    void DestructibleObject::playBreakSound(const Vec3& position) {
        // This would interface with audio system
        // Play break sound at position with volume
        // Implementation depends on your audio system
    }

    // ============================================================================
    // DestructionSystem Implementation
    // ============================================================================

    DestructionSystem::DestructionSystem(PhysicsWorld* world,
                                       MemoryManager* memoryManager,
                                       Random* rng)
        : m_physicsWorld(world)
        , m_memoryManager(memoryManager)
        , m_rng(rng) {

        assert(m_physicsWorld && "Physics world cannot be null");
        assert(m_memoryManager && "Memory manager cannot be null");

        // Create our own random generator if none provided
        if (!m_rng) {
            m_ownedRng = std::make_unique<Random>();
            m_rng = m_ownedRng.get();
        }

        m_destructibles.reserve(256);
        m_toRemove.reserve(64);
    }

    DestructionSystem::~DestructionSystem() {
        // Unique pointers will handle cleanup automatically
    }

    DestructibleObject* DestructionSystem::registerDestructible(RigidBody* body,
                                                               const DestructionMaterial& material) {
        if (!body) return nullptr;

        // Create destructible object using memory manager
        auto destructible = m_memoryManager->allocateObject<DestructibleObject>(
            MemoryCategory::PHYSICS, body, material, m_memoryManager, m_rng);

        if (!destructible) return nullptr;

        m_destructibles[body] = std::unique_ptr<DestructibleObject>(destructible);
        return destructible;
    }

    void DestructionSystem::unregisterDestructible(RigidBody* body) {
        auto it = m_destructibles.find(body);
        if (it != m_destructibles.end()) {
            // Destructor will be called automatically when unique_ptr is destroyed
            m_destructibles.erase(it);
        }
    }

    void DestructionSystem::processCollision(const CollisionManifold& manifold) {
        // Check if either body is destructible
        auto processBody = [this, &manifold](RigidBody* body, RigidBody* other, bool isBodyA) {
            auto it = m_destructibles.find(body);
            if (it != m_destructibles.end()) {
                // Calculate impact force
                Vec3 relativeVelocity = other->getLinearVelocity() - body->getLinearVelocity();
                Float impactSpeed = glm::length(relativeVelocity);

                if (impactSpeed > m_impactThreshold) {
                    // Calculate impulse
                    Float totalMass = body->getMass() + other->getMass();
                    Float impulse = impactSpeed * totalMass;

                    // Get impact point and normal
                    Vec3 impactPoint = isBodyA ?
                        manifold.contacts[0].positionWorldOnA :
                        manifold.contacts[0].positionWorldOnB;
                    Vec3 impactNormal = isBodyA ?
                        manifold.contacts[0].normalWorldOnB :
                        -manifold.contacts[0].normalWorldOnB;

                    // Apply impact
                    if (it->second->applyImpact(impactPoint, impactNormal * impulse)) {
                        onObjectDestroyed(body, it->second.get());
                    }
                }
            }
        };

        processBody(manifold.bodyA, manifold.bodyB, true);
        processBody(manifold.bodyB, manifold.bodyA, false);
    }

    void DestructionSystem::applyExplosion(const Vec3& center, Float radius, Float force) {
        for (auto& [body, destructible] : m_destructibles) {
            Vec3 toObject = body->getPosition() - center;
            Float distance = glm::length(toObject);

            if (distance < radius) {
                Float falloff = 1.0f - (distance / radius);
                Float damage = force * falloff * falloff;

                Vec3 direction = distance > EPSILON ? toObject / distance : Vec3(0, 1, 0);

                if (destructible->applyDamage(damage, center, direction)) {
                    onObjectDestroyed(body, destructible.get());
                }
            }
        }
    }

    void DestructionSystem::update(Float deltaTime) {
        for (auto& [body, destructible] : m_destructibles) {
            destructible->update(deltaTime);

            // Clean up expired fragments
            if (destructible->isBroken()) {
                const auto& fragments = destructible->getFragments();
                if (fragments.empty() && destructible->getTimeSinceBreak() > 5.0f) {
                    // Mark for removal
                    m_toRemove.push_back(body);
                }
            }
        }

        // Remove marked objects
        for (RigidBody* body : m_toRemove) {
            m_destructibles.erase(body);
        }
        m_toRemove.clear();
    }

    void DestructionSystem::setDestructionCallback(const DestructionCallback& callback) {
        m_destructionCallback = callback;
    }

    DestructionSystem::Statistics DestructionSystem::getStatistics() const {
        Statistics stats;
        stats.totalDestructibles = m_destructibles.size();
        stats.brokenObjects = 0;
        stats.totalFragments = 0;
        Float totalDamage = 0;

        for (const auto& [body, destructible] : m_destructibles) {
            if (destructible->isBroken()) {
                stats.brokenObjects++;
                stats.totalFragments += destructible->getFragments().size();
            }
            totalDamage += destructible->getDamageRatio();
        }

        stats.averageDamage = stats.totalDestructibles > 0 ?
            totalDamage / static_cast<Float>(stats.totalDestructibles) : 0;

        return stats;
    }

    void DestructionSystem::onObjectDestroyed(RigidBody* body, DestructibleObject* destructible) {
        // Add fragments to physics world
        for (const auto& fragment : destructible->getFragments()) {
            if (fragment.body) {
                // Add to world - implementation depends on your physics system
                // m_physicsWorld->addRigidBody(fragment.body.get());
            }
        }

        // Invoke callback
        if (m_destructionCallback) {
            m_destructionCallback(body, destructible);
        }
    }

    // ============================================================================
    // StructuralIntegrity Implementation
    // ============================================================================

    StructuralIntegrity::StructuralIntegrity(PhysicsWorld* world, Random* rng)
        : m_world(world), m_rng(rng) {

        assert(m_world && "Physics world cannot be null");
        assert(m_rng && "Random number generator cannot be null");

        m_connections.reserve(128);
    }

    void StructuralIntegrity::addConnection(RigidBody* bodyA, RigidBody* bodyB,
                                          btTypedConstraint* constraint, Float maxForce) {
        if (!bodyA || !bodyB || !constraint) return;

        BeamConnection connection;
        connection.bodyA = bodyA;
        connection.bodyB = bodyB;
        connection.constraint = constraint;
        connection.maxForce = maxForce;
        connection.currentLoad = 0;
        connection.isBroken = false;

        m_connections.push_back(connection);
    }

    void StructuralIntegrity::update(Float deltaTime) {
        // Check each connection for overload
        for (auto& connection : m_connections) {
            if (connection.isBroken) continue;

            // Get constraint force
            Float appliedImpulse = connection.constraint->getAppliedImpulse();
            connection.currentLoad = std::abs(appliedImpulse);

            // Check for failure
            if (connection.currentLoad > connection.maxForce) {
                // Break connection
                connection.isBroken = true;
                connection.constraint->setEnabled(false);

                // Propagate failure
                propagateFailure(connection);
            }
        }

        // Remove broken connections
        m_connections.erase(
            std::remove_if(m_connections.begin(), m_connections.end(),
                [](const BeamConnection& c) { return c.isBroken; }),
            m_connections.end()
        );
    }

    void StructuralIntegrity::propagateFailure(const BeamConnection& broken) {
        // Check adjacent connections for increased load
        for (auto& connection : m_connections) {
            if (connection.isBroken) continue;

            if (connection.bodyA == broken.bodyA || connection.bodyA == broken.bodyB ||
                connection.bodyB == broken.bodyA || connection.bodyB == broken.bodyB) {
                // Increase load on adjacent connections with some randomness
                Float multiplier = m_rng->range(1.2f, 1.8f);
                connection.currentLoad *= multiplier;
            }
        }
    }

} // namespace engine::physics