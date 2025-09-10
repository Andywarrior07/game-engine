/**
 * @file ForceGenerators.cpp
 * @brief Force generators for physics simulation
 * @details Implements various force effects including gravity fields, wind,
 *          explosions, magnets, and custom force fields
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#include "ForceGenerators.h"

#include "../dynamics/RigidBody.h"

namespace engine::physics {
    void GravityGenerator::applyForce(RigidBody* body, Float deltaTime) {
        if (!body || !affects(body)) return;

        if (const Float mass = body->getMass(); mass > 0) {
            body->applyForce(gravity_ * mass);
        }
    }

    bool GravityGenerator::affects(RigidBody* body) const {
        return body && body->getType() == BodyType::DYNAMIC;
    }

    void PlanetaryGravityGenerator::applyForce(RigidBody* body, Float deltaTime) {
        if (!body || !affects(body)) return;

        Vec3 direction = center_ - body->getPosition();
        const Float distance = glm::length(direction);

        if (distance < minDistance_ || distance > maxDistance_) return;

        direction /= distance; // Normalize

        // F = G * m1 * m2 / r²
        const Float bodyMass = body->getMass();
        const Float forceMagnitude = (6.67430e-11f * mass_ * bodyMass) / (distance * distance);

        body->applyForce(direction * forceMagnitude);
    }

    bool PlanetaryGravityGenerator::affects(RigidBody* body) const {
        if (!body || body->getType() != BodyType::DYNAMIC) return false;

        const Float distance = glm::length(body->getPosition() - center_);
        return distance >= minDistance_ && distance <= maxDistance_;
    }

    void WindGenerator::applyForce(RigidBody* body, Float deltaTime) {
        if (!body || !affects(body)) return;

        // Get relative velocity
        Vec3 bodyVelocity = body->getLinearVelocity();
        Vec3 relativeVelocity = windVelocity_ - bodyVelocity;

        // Add turbulence
        Vec3 turbulence = Vec3(
            std::sin(time_ * 2.0f) * turbulence_,
            std::cos(time_ * 3.0f) * turbulence_,
            std::sin(time_ * 1.5f) * turbulence_
        );

        // Add gusts
        Float gustFactor = 1.0f + std::sin(time_ * gustFrequency_) * gustStrength_;

        Vec3 windForce = (relativeVelocity + turbulence) * gustFactor;

        // Apply drag coefficient (simplified)
        AABB aabb = body->getAABB();
        Vec3 size = aabb.getSize();
        Float area = size.x * size.y; // Approximate frontal area

        Float dragCoefficient = 0.5f;
        windForce *= dragCoefficient * area * world::AIR_DENSITY;

        body->applyForce(windForce);
    }

    bool WindGenerator::affects(RigidBody* body) const {
        return body && body->getType() == BodyType::DYNAMIC;
    }

    void ExplosionGenerator::applyForce(RigidBody* body, Float deltaTime) {
        if (!body || !affects(body) || timeRemaining_ <= 0) return;

        Vec3 direction = body->getPosition() - center_;
        const Float distance = glm::length(direction);

        if (distance > radius_) return;

        // Normalize direction
        if (distance > EPSILON) {
            direction /= distance;
        }
        else {
            direction = Vec3(0, 1, 0); // Default up if at center
        }

        // Add upwards modifier
        direction.y += upwardsModifier_;
        direction = glm::normalize(direction);

        // Calculate force with falloff
        Float falloff = 1.0f - (distance / radius_);
        falloff = falloff * falloff; // Square for more dramatic falloff

        const Float forceMagnitude = force_ * falloff;

        // Apply as impulse for explosion effect
        body->applyImpulse(direction * forceMagnitude);

        // Optional: Add torque for tumbling effect
        const Vec3 torque = glm::cross(direction, Vec3(0, 1, 0)) * forceMagnitude * 0.1f;
        body->applyTorqueImpulse(torque);
    }

    bool ExplosionGenerator::affects(RigidBody* body) const {
        if (!body || body->getType() != BodyType::DYNAMIC) return false;

        const Float distance = glm::length(body->getPosition() - center_);
        return distance <= radius_ && timeRemaining_ > 0;
    }

    void ExplosionGenerator::update(const Float deltaTime) {
        timeRemaining_ -= deltaTime;

        if (timeRemaining_ <= 0) {
            active_ = false;
        }
    }

    void MagneticGenerator::applyForce(RigidBody* body, Float deltaTime) {
        if (!body || !affects(body)) return;

        // Check if body is magnetic (using user data or special flag)
        // For now, assume all metal objects are affected

        Vec3 direction = position_ - body->getPosition();
        const Float distance = glm::length(direction);

        if (distance > radius_ || distance < EPSILON) return;

        direction /= distance; // Normalize

        // Magnetic force follows inverse square law
        const Float forceMagnitude = strength_ / (distance * distance);

        // Attraction or repulsion based on polarity
        const Float polarityFactor = (polarity_ == Polarity::NORTH) ? 1.0f : -1.0f;

        body->applyForce(direction * forceMagnitude * polarityFactor);
    }

    bool MagneticGenerator::affects(RigidBody* body) const {
        if (!body || body->getType() != BodyType::DYNAMIC) return false;

        const Float distance = glm::length(body->getPosition() - position_);
        return distance <= radius_;
    }

    void BuoyancyGenerator::applyForce(RigidBody* body, Float deltaTime) {
        if (!body || !affects(body)) return;

        AABB aabb = body->getAABB();

        // Calculate submerged volume (simplified)
        Float currentWaterLevel = waterLevel_ +
            std::sin(time_ * waveFrequency_) * waveAmplitude_;

        Float submergedDepth = currentWaterLevel - aabb.min.y;
        if (submergedDepth <= 0) return; // Not in water

        Float objectHeight = aabb.max.y - aabb.min.y;
        Float submergedRatio = saturate(submergedDepth / objectHeight);

        // Approximate volume
        Vec3 size = aabb.getSize();
        Float volume = size.x * size.y * size.z;
        Float submergedVolume = volume * submergedRatio;

        // Buoyancy force: F = ρ * V * g
        Float buoyancyMagnitude = liquidDensity_ * submergedVolume *
            std::abs(world::GRAVITY_EARTH);

        body->applyForce(Vec3(0, buoyancyMagnitude, 0));

        // Drag in water
        Vec3 velocity = body->getLinearVelocity();
        Vec3 relativeVelocity = velocity - flowVelocity_;
        Float dragCoefficient = 0.5f;
        Vec3 dragForce = -relativeVelocity * dragCoefficient * submergedRatio;

        body->applyForce(dragForce);

        // Angular drag
        Vec3 angularVelocity = body->getAngularVelocity();
        Vec3 angularDrag = -angularVelocity * 0.1f * submergedRatio;
        body->applyTorque(angularDrag);
    }

    bool BuoyancyGenerator::affects(RigidBody* body) const {
        if (!body || body->getType() != BodyType::DYNAMIC) return false;

        const AABB aabb = body->getAABB();
        return aabb.min.y < waterLevel_ + waveAmplitude_;
    }

    void ForceFieldGenerator::applyForce(RigidBody* body, Float deltaTime) {
        if (!body || !affects(body)) return;

        const Vec3 position = body->getPosition();
        const Vec3 force = forceFunction_(position, time_);

        body->applyForce(force);
    }

    bool ForceFieldGenerator::affects(RigidBody* body) const {
        if (!body || body->getType() != BodyType::DYNAMIC) return false;

        return bounds_.contains(body->getPosition());
    }

    Vec3 ForceFieldGenerator::VortexForce(const Vec3& position, Float time) {
        // Circular vortex around Y axis
        const Float radius = std::sqrt(position.x * position.x + position.z * position.z);
        if (radius < EPSILON) return Vec3(0);

        const Vec3 tangent(-position.z / radius, 0, position.x / radius);
        const Float strength = 100.0f / (radius + 1.0f); // Falloff with distance

        return tangent * strength;
    }

    Vec3 ForceFieldGenerator::OscillatingForce(const Vec3& position, const Float time) {
        // Oscillating force field
        constexpr Float amplitude = 50.0f;
        constexpr Float frequency = 2.0f;

        const Vec3 force(
            std::sin(time * frequency) * amplitude,
            std::cos(time * frequency * 0.7f) * amplitude * 0.5f,
            std::sin(time * frequency * 1.3f) * amplitude * 0.8f
        );

        return force;
    }

    Vec3 ForceFieldGenerator::RadialForce(const Vec3& position, const Float time) {
        // Radial push/pull from origin
        const Float distance = glm::length(position);
        if (distance < EPSILON) return Vec3(0);

        const Vec3 direction = position / distance;
        const Float strength = std::sin(time) * 100.0f;

        return direction * strength;
    }

    void SpringGenerator::applyForce(RigidBody* body, Float deltaTime) {
        if (!bodyA_ || !bodyB_) return;

        // Only apply to one of the connected bodies per update
        if (body != bodyA_ && body != bodyB_) return;

        Vec3 posA = bodyA_->getPosition();
        Vec3 posB = bodyB_->getPosition();

        Vec3 displacement = posB - posA;
        Float currentLength = glm::length(displacement);

        if (currentLength < EPSILON) return;

        Vec3 direction = displacement / currentLength;

        // Spring force: F = -k * x
        Float extension = currentLength - restLength_;
        Float springForce = -springConstant_ * extension;

        // Damping force: F = -c * v
        Vec3 velA = bodyA_->getLinearVelocity();
        Vec3 velB = bodyB_->getLinearVelocity();
        Vec3 relativeVelocity = velB - velA;
        Float dampingForce = -damping_ * glm::dot(relativeVelocity, direction);

        Vec3 totalForce = direction * (springForce + dampingForce);

        // Apply equal and opposite forces
        if (body == bodyA_) {
            bodyA_->applyForce(-totalForce);
        }
        else {
            bodyB_->applyForce(totalForce);
        }
    }

    bool SpringGenerator::affects(RigidBody* body) const {
        return body == bodyA_ || body == bodyB_;
    }

    void ForceGeneratorRegistry::addGenerator(const std::shared_ptr<IForceGenerator>& generator) {
        if (generator) {
            generators_.push_back(generator);
        }
    }

    void ForceGeneratorRegistry::removeGenerator(const std::shared_ptr<IForceGenerator>& generator) {
        std::erase(generators_, generator);
    }

    void ForceGeneratorRegistry::applyGenerators(RigidBody* body, const Float deltaTime) const {
        for (auto& generator : generators_) {
            if (generator && generator->isActive()) {
                generator->applyForce(body, deltaTime);
            }
        }
    }

    void ForceGeneratorRegistry::updateGenerators(const Float deltaTime) {
        // Remove expired generators (like explosions)
        std::erase_if(generators_,
                      [](const std::shared_ptr<IForceGenerator>& gen) {
                          const auto explosion = std::dynamic_pointer_cast<ExplosionGenerator>(gen);
                          return explosion && explosion->isExpired();
                      });

        // Update remaining generators
        for (const auto& generator : generators_) {
            if (generator && generator->isActive()) {
                generator->update(deltaTime);
            }
        }
    }

    std::size_t ForceGeneratorRegistry::getGeneratorCount() const {
        return std::ranges::count_if(generators_,
                                     [](const std::shared_ptr<IForceGenerator>& gen) {
                                         return gen && gen->isActive();
                                     });
    }

    void ForceGeneratorRegistry::createExplosion(const Vec3& position, Float force, Float radius) {
        const auto explosion = std::make_shared<ExplosionGenerator>(
            position, force, radius, 0.1f, 0.5f
        );
        addGenerator(explosion);
    }

    std::shared_ptr<WindGenerator> ForceGeneratorRegistry::createWindZone(const Vec3& velocity) {
        auto wind = std::make_shared<WindGenerator>(velocity);
        addGenerator(wind);
        return wind;
    }
} // namespace engine::physics
