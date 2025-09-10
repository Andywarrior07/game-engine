/**
 * @file PhysicsUtils.h
 * @brief Utility functions and helpers for physics calculations
 * @details Provides common physics calculations, conversions, and helper functions
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

#include "../core/PhysicsTypes.h"

namespace engine::physics {
    using namespace engine::math;

    /**
     * @brief Physics utility functions
     */
    class PhysicsUtils {
    public:
        // ============================================================================
        // Inertia Calculations
        // ============================================================================

        /**
         * @brief Calculate inertia tensor for box
         */
        static Vec3 calculateBoxInertia(Float mass, const Vec3& halfExtents);

        /**
         * @brief Calculate inertia tensor for sphere
         */
        static Vec3 calculateSphereInertia(Float mass, Float radius);

        /**
         * @brief Calculate inertia tensor for cylinder
         */
        static Vec3 calculateCylinderInertia(Float mass, Float radius, Float height,
                                             Int axis = 1);

        /**
         * @brief Calculate inertia tensor for capsule
         */
        static Vec3 calculateCapsuleInertia(Float mass, Float radius, Float height,
                                            Int axis = 1);

        /**
         * @brief Calculate inertia tensor for mesh
         */
        static Vec3 calculateMeshInertia(Float mass, const std::vector<Vec3>& vertices,
                                         const std::vector<Int>& indices);

        // ============================================================================
        // Force and Torque Calculations
        // ============================================================================

        /**
         * @brief Calculate drag force
         */
        static Vec3 calculateDragForce(const Vec3& velocity, Float dragCoefficient,
                                       Float crossSectionalArea, Float fluidDensity = world::AIR_DENSITY);

        /**
         * @brief Calculate lift force
         */
        static Vec3 calculateLiftForce(const Vec3& velocity, const Vec3& liftDirection,
                                       Float liftCoefficient, Float wingArea,
                                       Float fluidDensity = world::AIR_DENSITY);

        /**
         * @brief Calculate buoyancy force
         */
        static Vec3 calculateBuoyancyForce(const Float submergedVolume,
                                           const Float fluidDensity = material::DENSITY_WATER,
                                           const Vec3& gravity = Vec3(0, -9.81f, 0)) {
            return -gravity * fluidDensity * submergedVolume;
        }

        /**
         * @brief Calculate spring force (Hooke's law)
         */
        static Vec3 calculateSpringForce(const Vec3& displacement, Float springConstant,
                                         Float damping = 0.0f, const Vec3& velocity = VEC3_ZERO);

        /**
         * @brief Calculate torque from force at position
         */
        static Vec3 calculateTorque(const Vec3& force, const Vec3& position,
                                    const Vec3& centerOfMass = VEC3_ZERO);

        // ============================================================================
        // Collision Response
        // ============================================================================

        /**
         * @brief Calculate impulse for collision response
         */
        static Float calculateCollisionImpulse(const Vec3& relativeVelocity,
                                               const Vec3& collisionNormal,
                                               Float massA, Float massB,
                                               Float restitution);

        /**
         * @brief Calculate friction impulse
         */
        static Vec3 calculateFrictionImpulse(const Vec3& relativeVelocity,
                                             const Vec3& collisionNormal,
                                             Float normalImpulse,
                                             Float staticFriction,
                                             Float dynamicFriction);

        /**
         * @brief Calculate penetration resolution
         */
        static void resolvePenetration(Vec3& positionA, Vec3& positionB,
                                       Float massA, Float massB,
                                       const Vec3& normal, Float penetration,
                                       Float correctionPercent = 0.2f,
                                       Float slop = 0.01f);

        // ============================================================================
        // Energy Calculations
        // ============================================================================

        /**
         * @brief Calculate kinetic energy
         */
        static Float calculateKineticEnergy(Float mass, const Vec3& velocity);

        /**
         * @brief Calculate rotational kinetic energy
         */
        static Float calculateRotationalEnergy(const Vec3& inertia, const Vec3& angularVelocity);

        /**
         * @brief Calculate potential energy
         */
        static Float calculatePotentialEnergy(const Float mass, const Float height,
                                              const Float gravity = world::GRAVITY_EARTH) {
            return mass * gravity * height;
        }

        /**
         * @brief Calculate elastic potential energy
         */
        static Float calculateElasticEnergy(const Float springConstant, const Float displacement) {
            return 0.5f * springConstant * displacement * displacement;
        }

        // ============================================================================
        // Trajectory Calculations
        // ============================================================================

        /**
         * @brief Calculate projectile trajectory
         */
        static Vec3 calculateProjectilePosition(const Vec3& initialPosition,
                                                const Vec3& initialVelocity,
                                                const Float time,
                                                const Vec3& gravity = Vec3(0, -9.81f, 0)) {
            return initialPosition + initialVelocity * time + 0.5f * gravity * time * time;
        }

        /**
         * @brief Calculate launch velocity for target
         */
        static Vec3 calculateLaunchVelocity(const Vec3& start, const Vec3& target,
                                            Float angle, Float gravity = world::GRAVITY_EARTH);

        /**
         * @brief Predict intercept point
         */
        static std::optional<Vec3> predictIntercept(const Vec3& shooterPos,
                                                    Float projectileSpeed,
                                                    const Vec3& targetPos,
                                                    const Vec3& targetVelocity);

        // ============================================================================
        // Constraints and Limits
        // ============================================================================

        /**
         * @brief Apply velocity limits
         */
        static void limitVelocity(Vec3& velocity, Float maxSpeed);

        /**
         * @brief Apply angular velocity limits
         */
        static void limitAngularVelocity(Vec3& angularVelocity, Float maxAngularSpeed);

        /**
         * @brief Clamp position to bounds
         */
        static void clampToBounds(Vec3& position, const AABB& bounds);

        // ============================================================================
        // Interpolation and Smoothing
        // ============================================================================

        /**
         * @brief Smooth damp for physics
         */
        static Vec3 smoothDamp(const Vec3& current, const Vec3& target,
                               Vec3& currentVelocity, Float smoothTime,
                               Float maxSpeed, Float deltaTime);

        /**
         * @brief Predict future position
         */
        static Vec3 predictFuturePosition(const Vec3& position, const Vec3& velocity,
                                          const Vec3& acceleration, const Float time) {
            return position + velocity * time + 0.5f * acceleration * time * time;
        }

        // ============================================================================
        // Helper Functions
        // ============================================================================

        /**
         * @brief Check if body is at rest
         */
        static bool isBodyAtRest(const RigidBody* body, Float linearThreshold = 0.01f,
                                 Float angularThreshold = 0.01f);

        /**
         * @brief Calculate center of mass for multiple bodies
         */
        static Vec3 calculateCenterOfMass(const std::vector<RigidBody*>& bodies);

        /**
         * @brief Calculate moment of inertia for point mass
         */
        static Float calculatePointMassInertia(const Float mass, const Float distance) {
            return mass * distance * distance;
        }

        /**
         * @brief Convert between linear and angular velocity
         */
        static Vec3 linearToAngularVelocity(const Vec3& linearVelocity, const Float radius) {
            return linearVelocity / radius;
        }

        static Vec3 angularToLinearVelocity(const Vec3& angularVelocity, const Float radius) {
            return angularVelocity * radius;
        }

        /**
         * @brief Calculate relative velocity
         */
        static Vec3 calculateRelativeVelocity(const RigidBody* bodyA, const RigidBody* bodyB,
                                              const Vec3& contactPoint);

        /**
         * @brief Calculate separation velocity
         */
        static Float calculateSeparationVelocity(const Vec3& relativeVelocity,
                                                 const Vec3& contactNormal);

    private:
        /**
         * @brief Calculate inertia tensor for tetrahedron
         */
        static Mat3 calculateTetrahedronInertia(const Vec3& v0, const Vec3& v1, const Vec3& v2);
    };
} // namespace engine::physics
