/**
 * @file ForceGenerators.h
 * @brief Force generators for physics simulation
 * @details Implements various force effects including gravity fields, wind,
 *          explosions, magnets, and custom force fields
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

#include "../../math/MathSystem.h"
#include "../core/PhysicsTypes.h"
// #include "../core/PhysicsConstants.h"
// #include "../dynamics/RigidBody.h"

// #include <functional>
// #include <vector>

namespace engine::physics {
    using namespace engine::math;

    class RigidBody;

    /**
     * @brief Base force generator interface
     */
    class IForceGenerator {
    public:
        virtual ~IForceGenerator() = default;

        /**
         * @brief Apply force to a body
         * @param body Target body
         * @param deltaTime Time step
         */
        virtual void applyForce(RigidBody* body, Float deltaTime) = 0;

        /**
         * @brief Check if generator affects body
         */
        virtual bool affects(RigidBody* body) const = 0;

        /**
         * @brief Update generator state
         */
        virtual void update(Float deltaTime) = 0;

        /**
         * @brief Check if generator is active
         */
        virtual bool isActive() const { return active_; }
        virtual void setActive(const bool active) { active_ = active; }

    protected:
        bool active_ = true;
    };

    // TODO: In the future, move it to the module manager, as it is a preset
    /**
     * @brief Gravity generator with custom direction and strength
     */
    class GravityGenerator : public IForceGenerator {
    public:
        explicit GravityGenerator(const Vec3& gravity = Vec3(0, world::GRAVITY_EARTH, 0))
            : gravity_(gravity) {
        }

        void applyForce(RigidBody* body, Float deltaTime) override;

        bool affects(RigidBody* body) const override;

        void setGravity(const Vec3& gravity) { gravity_ = gravity; }
        Vec3 getGravity() const { return gravity_; }

    private:
        Vec3 gravity_;
    };

    /**
     * @brief Planetary gravity (point gravity source)
     */
    class PlanetaryGravityGenerator : public IForceGenerator {
    public:
        PlanetaryGravityGenerator(const Vec3& center, const Float mass, const Float radius)
            : center_(center), mass_(mass), radius_(radius),
              minDistance_(radius), maxDistance_(radius * 100.0f) {
        }

        void applyForce(RigidBody* body, Float deltaTime) override;

        bool affects(RigidBody* body) const override;

        void setCenter(const Vec3& center) { center_ = center; }
        void setMass(const Float mass) { mass_ = mass; }

    private:
        Vec3 center_;
        Float mass_;
        Float radius_;
        Float minDistance_;
        Float maxDistance_;
    };

    /**
     * @brief Wind force generator
     */
    class WindGenerator final : public IForceGenerator {
    public:
        explicit WindGenerator(const Vec3& windVelocity = Vec3(10, 0, 0))
            : windVelocity_(windVelocity), turbulence_(0.2f),
              gustStrength_(5.0f), gustFrequency_(0.5f), time_(0) {
        }

        void applyForce(RigidBody* body, Float deltaTime) override;

        bool affects(RigidBody* body) const override;

        void update(const Float deltaTime) override {
            time_ += deltaTime;
        }

        void setWindVelocity(const Vec3& velocity) { windVelocity_ = velocity; }
        void setTurbulence(const Float turbulence) { turbulence_ = turbulence; }
        void setGustStrength(const Float strength) { gustStrength_ = strength; }

    private:
        Vec3 windVelocity_;
        Float turbulence_;
        Float gustStrength_;
        Float gustFrequency_;
        Float time_;
    };

    /**
     * @brief Explosion force generator
     */
    class ExplosionGenerator final : public IForceGenerator {
    public:
        ExplosionGenerator(const Vec3& center, const Float force, const Float radius,
                           const Float duration = 0.1f, const Float upwardsModifier = 0.5f)
            : center_(center), force_(force), radius_(radius),
              duration_(duration), upwardsModifier_(upwardsModifier),
              timeRemaining_(duration) {
        }

        void applyForce(RigidBody* body, Float deltaTime) override;

        bool affects(RigidBody* body) const override;

        void update(Float deltaTime) override;

        bool isExpired() const { return timeRemaining_ <= 0; }

    private:
        Vec3 center_;
        Float force_;
        Float radius_;
        Float duration_;
        Float upwardsModifier_;
        Float timeRemaining_;
    };

    /**
     * @brief Magnetic force generator
     */
    class MagneticGenerator : public IForceGenerator {
    public:
        enum class Polarity { NORTH, SOUTH };

        MagneticGenerator(const Vec3& position, const Float strength, const Float radius,
                          const Polarity polarity = Polarity::NORTH)
            : position_(position), strength_(strength), radius_(radius),
              polarity_(polarity) {
        }

        void applyForce(RigidBody* body, Float deltaTime) override;

        bool affects(RigidBody* body) const override;

        void setPosition(const Vec3& position) { position_ = position; }
        void setStrength(const Float strength) { strength_ = strength; }
        void setPolarity(const Polarity polarity) { polarity_ = polarity; }

    private:
        Vec3 position_;
        Float strength_;
        Float radius_;
        Polarity polarity_;
    };

    /**
     * @brief Buoyancy force generator for water simulation
     */
    class BuoyancyGenerator final : public IForceGenerator {
    public:
        explicit BuoyancyGenerator(const Float waterLevel = 0.0f, const Float liquidDensity = material::DENSITY_WATER,
                                   const Vec3& flowVelocity = Vec3(0))
            : waterLevel_(waterLevel), liquidDensity_(liquidDensity),
              flowVelocity_(flowVelocity), waveAmplitude_(0.5f),
              waveFrequency_(0.5f), time_(0) {
        }

        void applyForce(RigidBody* body, Float deltaTime) override;

        bool affects(RigidBody* body) const override;

        void update(const Float deltaTime) override {
            time_ += deltaTime;
        }

        void setWaterLevel(const Float level) { waterLevel_ = level; }
        void setFlowVelocity(const Vec3& velocity) { flowVelocity_ = velocity; }
        void setWaveAmplitude(const Float amplitude) { waveAmplitude_ = amplitude; }

    private:
        Float waterLevel_;
        Float liquidDensity_;
        Vec3 flowVelocity_;
        Float waveAmplitude_;
        Float waveFrequency_;
        Float time_;
    };

    /**
     * @brief Custom force field generator
     */
    class ForceFieldGenerator final : public IForceGenerator {
    public:
        using ForceFunction = std::function<Vec3(const Vec3& position, Float time)>;

        ForceFieldGenerator(const AABB& bounds, const ForceFunction& forceFunc)
            : bounds_(bounds), forceFunction_(forceFunc), time_(0) {
        }

        void applyForce(RigidBody* body, Float deltaTime) override;

        bool affects(RigidBody* body) const override;

        void update(const Float deltaTime) override {
            time_ += deltaTime;
        }

        void setBounds(const AABB& bounds) { bounds_ = bounds; }
        void setForceFunction(const ForceFunction& func) { forceFunction_ = func; }

        // Example force functions
        static Vec3 VortexForce(const Vec3& position, Float time);

        static Vec3 OscillatingForce(const Vec3& position, Float time);

        static Vec3 RadialForce(const Vec3& position, Float time);

    private:
        AABB bounds_;
        ForceFunction forceFunction_;
        Float time_;
    };

    /**
     * @brief Spring force generator for connecting bodies
     */
    class SpringGenerator : public IForceGenerator {
    public:
        SpringGenerator(RigidBody* bodyA, RigidBody* bodyB,
                        const Float restLength, const Float springConstant, const Float damping = 0.1f)
            : bodyA_(bodyA), bodyB_(bodyB), restLength_(restLength),
              springConstant_(springConstant), damping_(damping) {
        }

        void applyForce(RigidBody* body, Float deltaTime) override;

        bool affects(RigidBody* body) const override;

        void setRestLength(const Float length) { restLength_ = length; }
        void setSpringConstant(const Float k) { springConstant_ = k; }
        void setDamping(const Float damping) { damping_ = damping; }

    private:
        RigidBody* bodyA_;
        RigidBody* bodyB_;
        Float restLength_;
        Float springConstant_;
        Float damping_;
    };

    /**
     * @brief Manager for force generators
     */
    class ForceGeneratorRegistry {
    public:
        ForceGeneratorRegistry() = default;

        /**
         * @brief Register a force generator
         */
        void addGenerator(const std::shared_ptr<IForceGenerator>& generator);

        /**
         * @brief Remove a force generator
         */
        void removeGenerator(const std::shared_ptr<IForceGenerator>& generator);

        /**
         * @brief Apply all generators to a body
         */
        void applyGenerators(RigidBody* body, Float deltaTime) const;

        /**
         * @brief Update all generators
         */
        void updateGenerators(Float deltaTime);

        /**
         * @brief Clear all generators
         */
        void clear() {
            generators_.clear();
        }

        /**
         * @brief Get number of active generators
         */
        std::size_t getGeneratorCount() const;

        /**
         * @brief Create and add an explosion at position
         */
        void createExplosion(const Vec3& position, Float force, Float radius);

        /**
         * @brief Create and add a wind zone
         */
        std::shared_ptr<WindGenerator> createWindZone(const Vec3& velocity);;

    private:
        std::vector<std::shared_ptr<IForceGenerator>> generators_;
    };
} // namespace engine::physics
