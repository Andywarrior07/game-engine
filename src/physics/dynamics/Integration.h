/**
 * @file Integration.h
 * @brief Physics integration methods for motion simulation
 * @details Implements various numerical integration schemes including
 *          Euler, Verlet, and Runge-Kutta methods
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

#include "../../math/MathSystem.h"

namespace engine::physics {
    using namespace engine::math;

    class RigidBody;

    /**
     * @brief Integration method type
     */
    enum class IntegrationType : std::uint8_t {
        EULER, // Simple, fast, less stable
        SEMI_IMPLICIT_EULER, // Better stability
        VERLET, // Good energy conservation
        VELOCITY_VERLET, // Accurate velocities
        RK4 // 4th order Runge-Kutta, most accurate
    };

    /**
     * @brief State vector for integration
     */
    struct IntegrationState {
        Vec3 position;
        Quat orientation;
        Vec3 linearVelocity;
        Vec3 angularVelocity;
        Vec3 linearAcceleration;
        Vec3 angularAcceleration;

        IntegrationState() : orientation(1, 0, 0, 0) {
        }

        IntegrationState operator+(const IntegrationState& other) const {
            IntegrationState result;
            result.position = position + other.position;
            result.orientation = glm::normalize(orientation + other.orientation);
            result.linearVelocity = linearVelocity + other.linearVelocity;
            result.angularVelocity = angularVelocity + other.angularVelocity;
            result.linearAcceleration = linearAcceleration + other.linearAcceleration;
            result.angularAcceleration = angularAcceleration + other.angularAcceleration;
            return result;
        }

        IntegrationState operator*(Float scalar) const {
            IntegrationState result;
            result.position = position * scalar;
            result.orientation = glm::slerp(Quat(1, 0, 0, 0), orientation, scalar);
            result.linearVelocity = linearVelocity * scalar;
            result.angularVelocity = angularVelocity * scalar;
            result.linearAcceleration = linearAcceleration * scalar;
            result.angularAcceleration = angularAcceleration * scalar;
            return result;
        }
    };

    /**
     * @brief Physics integrator for rigid body dynamics
     */
    class PhysicsIntegrator {
    public:
        explicit PhysicsIntegrator(const IntegrationType type = IntegrationType::SEMI_IMPLICIT_EULER)
            : m_integrationType(type), m_gravity(0, -9.81f, 0) {
        }

        /**
         * @brief Integrate a single rigid body
         */
        void integrate(RigidBody* body, Float deltaTime);

        /**
         * @brief Batch integrate multiple bodies
         */
        void integrateBatch(const std::vector<RigidBody*>& bodies, Float deltaTime);

        void setIntegrationType(const IntegrationType type) { m_integrationType = type; }
        IntegrationType getIntegrationType() const { return m_integrationType; }

        void setGravity(const Vec3& gravity) { m_gravity = gravity; }
        Vec3 getGravity() const { return m_gravity; }

        void setMaxLinearVelocity(const Float max) { m_maxLinearVelocity = max; }
        void setMaxAngularVelocity(const Float max) { m_maxAngularVelocity = max; }

    private:
        IntegrationType m_integrationType;
        Vec3 m_gravity;
        Float m_maxLinearVelocity = 100.0f;
        Float m_maxAngularVelocity = 100.0f;

        // Previous states for Verlet integration
        std::unordered_map<RigidBody*, IntegrationState> m_previousStates;

        /**
         * @brief Simple Euler integration
         * x(t+dt) = x(t) + v(t)*dt
         * v(t+dt) = v(t) + a(t)*dt
         */
        void integrateEuler(const RigidBody* body, Float deltaTime) const;

        /**
         * @brief Semi-implicit Euler (Symplectic Euler)
         * v(t+dt) = v(t) + a(t)*dt
         * x(t+dt) = x(t) + v(t+dt)*dt
         */
        void integrateSemiImplicitEuler(RigidBody* body, Float deltaTime) const;

        /**
         * @brief Verlet integration
         * x(t+dt) = 2*x(t) - x(t-dt) + a(t)*dt^2
         */
        void integrateVerlet(RigidBody* body, Float deltaTime);

        /**
         * @brief Velocity Verlet integration
         * x(t+dt) = x(t) + v(t)*dt + 0.5*a(t)*dt^2
         * v(t+dt) = v(t) + 0.5*(a(t) + a(t+dt))*dt
         */
        void integrateVelocityVerlet(RigidBody* body, Float deltaTime) const;

        /**
         * @brief 4th order Runge-Kutta integration
         */
        void integrateRK4(RigidBody* body, Float deltaTime) const;

        /**
         * @brief Evaluate derivatives for RK4
         */
        IntegrationState evaluate(RigidBody* body, const IntegrationState& state, Float dt) const;

        /**
         * @brief Get state from rigid body
         */
        static IntegrationState getState(const RigidBody* body);

        /**
         * @brief Set state to rigid body
         */
        static void setState(const RigidBody* body, const IntegrationState& state);

        /**
         * @brief Calculate linear acceleration from forces
         */
        Vec3 calculateLinearAcceleration(const RigidBody* body) const;

        /**
         * @brief Calculate angular acceleration from torques
         */
        static Vec3 calculateAngularAcceleration(const RigidBody* body);

        /**
         * @brief Integrate orientation using quaternion
         */
        static Quat integrateOrientation(const Quat& orientation, const Vec3& angularVelocity, Float deltaTime);

        /**
         * @brief Apply damping to velocities
         */
        static void applyDamping(const RigidBody* body, Float deltaTime);

        /**
         * @brief Clamp velocities to maximum values
         */
        void clampVelocities(const RigidBody* body) const;

        /**
         * @brief Get inverse inertia tensor in world space
         */
        static Mat3 getInverseInertiaWorld(const RigidBody* body);
    };

    // TODO: In the future, move it to the module manager, as it is a preset
    /**
     * @brief Specialized integrator for character controllers
     */
    class CharacterIntegrator {
    public:
        CharacterIntegrator() : m_gravity(0, -20.0f, 0) {
        } // Higher gravity for responsive movement

        void integrate(Vec3& position, Vec3& velocity, const Vec3& acceleration,
                       Float deltaTime, bool isGrounded) const;

        void setGravity(const Vec3& gravity) { m_gravity = gravity; }

    private:
        Vec3 m_gravity;
    };

    // TODO: In the future, move it to the module manager, as it is a preset
    /**
     * @brief Specialized integrator for vehicles
     */
    class VehicleIntegrator {
    public:
        static void integrate(Vec3& position, Quat& orientation, Vec3& velocity,
                              Vec3& angularVelocity, const Vec3& force, const Vec3& torque,
                              Float mass, const Mat3& inertia, Float deltaTime);
    };
} // namespace engine::physics
