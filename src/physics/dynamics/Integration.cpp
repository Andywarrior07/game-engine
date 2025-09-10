/**
* @file Integration.h
 * @brief Physics integration methods for motion simulation
 * @details Implements various numerical integration schemes including
 *          Euler, Verlet, and Runge-Kutta methods
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#include "Integration.h"

#include "RigidBody.h"

namespace engine::physics {
    void PhysicsIntegrator::integrate(RigidBody* body, const Float deltaTime) {
        if (!body || body->getType() != BodyType::DYNAMIC) return;

        switch (m_integrationType) {
        case IntegrationType::EULER:
            integrateEuler(body, deltaTime);
            break;

        case IntegrationType::SEMI_IMPLICIT_EULER:
            integrateSemiImplicitEuler(body, deltaTime);
            break;

        case IntegrationType::VERLET:
            integrateVerlet(body, deltaTime);
            break;

        case IntegrationType::VELOCITY_VERLET:
            integrateVelocityVerlet(body, deltaTime);
            break;

        case IntegrationType::RK4:
            integrateRK4(body, deltaTime);
            break;
        }

        // Apply damping
        applyDamping(body, deltaTime);

        // Clamp velocities
        clampVelocities(body);
    }

    void PhysicsIntegrator::integrateBatch(const std::vector<RigidBody*>& bodies, const Float deltaTime) {
        for (RigidBody* body : bodies) {
            integrate(body, deltaTime);
        }
    }

    void PhysicsIntegrator::integrateEuler(const RigidBody* body, const Float deltaTime) const {
        IntegrationState state = getState(body);

        // Calculate accelerations
        Vec3 linearAccel = calculateLinearAcceleration(body);
        Vec3 angularAccel = calculateAngularAcceleration(body);

        // Update position and orientation
        state.position += state.linearVelocity * deltaTime;
        state.orientation = integrateOrientation(state.orientation, state.angularVelocity, deltaTime);

        // Update velocities
        state.linearVelocity += linearAccel * deltaTime;
        state.angularVelocity += angularAccel * deltaTime;

        setState(body, state);
    }

    void PhysicsIntegrator::integrateSemiImplicitEuler(RigidBody* body, Float deltaTime) const {
        IntegrationState state = getState(body);

        // Calculate accelerations
        Vec3 linearAccel = calculateLinearAcceleration(body);
        Vec3 angularAccel = calculateAngularAcceleration(body);

        // Update velocities first
        state.linearVelocity += linearAccel * deltaTime;
        state.angularVelocity += angularAccel * deltaTime;

        // Then update position with new velocities
        state.position += state.linearVelocity * deltaTime;
        state.orientation = integrateOrientation(state.orientation, state.angularVelocity, deltaTime);

        setState(body, state);
    }

    void PhysicsIntegrator::integrateVerlet(RigidBody* body, Float deltaTime) {
        IntegrationState currentState = getState(body);

        // Get or create previous state
        auto it = m_previousStates.find(body);
        if (it == m_previousStates.end()) {
            // First frame, use semi-implicit Euler
            integrateSemiImplicitEuler(body, deltaTime);
            m_previousStates[body] = currentState;
            return;
        }

        IntegrationState& prevState = it->second;

        // Calculate accelerations
        Vec3 linearAccel = calculateLinearAcceleration(body);
        Vec3 angularAccel = calculateAngularAcceleration(body);

        // Verlet position update
        Vec3 newPosition = currentState.position * 2.0f - prevState.position +
            linearAccel * (deltaTime * deltaTime);

        // Calculate velocity from positions
        Vec3 newVelocity = (newPosition - prevState.position) / (2.0f * deltaTime);

        // Angular Verlet
        Vec3 angularDisplacement = currentState.angularVelocity * deltaTime +
            angularAccel * (deltaTime * deltaTime * 0.5f);
        Quat newOrientation = integrateOrientation(currentState.orientation,
                                                   angularDisplacement / deltaTime, deltaTime);

        // Store previous state
        prevState = currentState;

        // Update body
        currentState.position = newPosition;
        currentState.orientation = newOrientation;
        currentState.linearVelocity = newVelocity;

        setState(body, currentState);
    }

    void PhysicsIntegrator::integrateVelocityVerlet(RigidBody* body, Float deltaTime) const {
        IntegrationState state = getState(body);

        // Calculate current accelerations
        Vec3 linearAccel = calculateLinearAcceleration(body);
        Vec3 angularAccel = calculateAngularAcceleration(body);

        // Update positions with current velocity and acceleration
        state.position += state.linearVelocity * deltaTime +
            linearAccel * (0.5f * deltaTime * deltaTime);
        state.orientation = integrateOrientation(state.orientation, state.angularVelocity, deltaTime);

        // Calculate new accelerations at new position
        setState(body, state);
        Vec3 newLinearAccel = calculateLinearAcceleration(body);
        Vec3 newAngularAccel = calculateAngularAcceleration(body);

        // Update velocities with average acceleration
        state.linearVelocity += 0.5f * (linearAccel + newLinearAccel) * deltaTime;
        state.angularVelocity += 0.5f * (angularAccel + newAngularAccel) * deltaTime;

        setState(body, state);
    }

    void PhysicsIntegrator::integrateRK4(RigidBody* body, Float deltaTime) const {
        IntegrationState initial = getState(body);

        // k1 = f(t, y)
        IntegrationState k1 = evaluate(body, initial, 0.0f);

        // k2 = f(t + dt/2, y + k1*dt/2)
        IntegrationState k2 = evaluate(body, initial + k1 * (deltaTime * 0.5f), deltaTime * 0.5f);

        // k3 = f(t + dt/2, y + k2*dt/2)
        IntegrationState k3 = evaluate(body, initial + k2 * (deltaTime * 0.5f), deltaTime * 0.5f);

        // k4 = f(t + dt, y + k3*dt)
        IntegrationState k4 = evaluate(body, initial + k3 * deltaTime, deltaTime);

        // y(t+dt) = y(t) + (k1 + 2*k2 + 2*k3 + k4) * dt/6
        IntegrationState derivative = (k1 + k2 * 2.0f + k3 * 2.0f + k4) * (1.0f / 6.0f);

        IntegrationState final = initial;
        final.position += derivative.position * deltaTime;
        final.orientation = integrateOrientation(initial.orientation,
                                                 derivative.angularVelocity, deltaTime);
        final.linearVelocity += derivative.linearVelocity * deltaTime;
        final.angularVelocity += derivative.angularVelocity * deltaTime;

        setState(body, final);
    }

    IntegrationState PhysicsIntegrator::evaluate(RigidBody* body, const IntegrationState& state, Float dt) const {
        // Temporarily set state
        IntegrationState original = getState(body);
        setState(body, state);

        // Calculate derivatives
        IntegrationState derivative;
        derivative.position = state.linearVelocity;
        derivative.linearVelocity = calculateLinearAcceleration(body);
        derivative.angularVelocity = calculateAngularAcceleration(body);

        // Restore original state
        setState(body, original);

        return derivative;
    }

    IntegrationState PhysicsIntegrator::getState(const RigidBody* body) {
        IntegrationState state;
        state.position = body->getPosition();
        state.orientation = body->getRotation();
        state.linearVelocity = body->getLinearVelocity();
        state.angularVelocity = body->getAngularVelocity();

        return state;
    }

    auto PhysicsIntegrator::setState(const RigidBody* body, const IntegrationState& state) -> void {
        body->setPosition(state.position);
        body->setRotation(state.orientation);
        body->setLinearVelocity(state.linearVelocity);
        body->setAngularVelocity(state.angularVelocity);
    }

    Vec3 PhysicsIntegrator::calculateLinearAcceleration(const RigidBody* body) const {
        const Float mass = body->getMass();
        if (mass <= 0) return Vec3(0);

        // Get total force (this would come from force accumulator)
        auto totalForce = Vec3(0); // Placeholder - should get from body

        // Add gravity
        if (body->getType() == BodyType::DYNAMIC) {
            totalForce += m_gravity * mass;
        }

        return totalForce / mass;
    }

    Vec3 PhysicsIntegrator::calculateAngularAcceleration(const RigidBody* body) {
        if (const Vec3 localInertia = body->getLocalInertia(); glm::length2(localInertia) < EPSILON_SQUARED) return Vec3(0);

        // Get total torque (this would come from torque accumulator)
        const Vec3 totalTorque = VEC3_ZERO; // Placeholder - should get from body

        // Apply inverse inertia tensor
        const Mat3 invInertia = getInverseInertiaWorld(body);
        return invInertia * totalTorque;
    }

    Quat PhysicsIntegrator::integrateOrientation(const Quat& orientation, const Vec3& angularVelocity, const Float deltaTime) {
        // Convert angular velocity to quaternion derivative
        const Quat spin(0, angularVelocity.x, angularVelocity.y, angularVelocity.z);
        const Quat derivative = spin * orientation * 0.5f;

        // Integrate
        const Quat newOrientation = orientation + derivative * deltaTime;

        // Normalize to prevent drift
        return glm::normalize(newOrientation);
    }

    void PhysicsIntegrator::applyDamping(const RigidBody* body, const Float deltaTime) {
        Vec3 linearVel = body->getLinearVelocity();
        Vec3 angularVel = body->getAngularVelocity();

        // Linear damping
        constexpr Float linearDamping = 0.99f; // Should come from body material
        linearVel *= std::pow(linearDamping, deltaTime);

        // Angular damping
        constexpr Float angularDamping = 0.98f; // Should come from body material
        angularVel *= std::pow(angularDamping, deltaTime);

        body->setLinearVelocity(linearVel);
        body->setAngularVelocity(angularVel);
    }

    void PhysicsIntegrator::clampVelocities(const RigidBody* body) const {
        Vec3 linearVel = body->getLinearVelocity();

        if (const Float linearSpeed = glm::length(linearVel); linearSpeed > m_maxLinearVelocity) {
            linearVel = (linearVel / linearSpeed) * m_maxLinearVelocity;
            body->setLinearVelocity(linearVel);
        }

        Vec3 angularVel = body->getAngularVelocity();

        if (const Float angularSpeed = glm::length(angularVel); angularSpeed > m_maxAngularVelocity) {
            angularVel = (angularVel / angularSpeed) * m_maxAngularVelocity;
            body->setAngularVelocity(angularVel);
        }
    }

    Mat3 PhysicsIntegrator::getInverseInertiaWorld(const RigidBody* body) {
        const Vec3 localInertia = body->getLocalInertia();
        if (glm::length2(localInertia) < EPSILON_SQUARED) return Mat3(0);

        Mat3 invInertiaLocal(0);
        invInertiaLocal[0][0] = localInertia.x > 0 ? 1.0f / localInertia.x : 0.0f;
        invInertiaLocal[1][1] = localInertia.y > 0 ? 1.0f / localInertia.y : 0.0f;
        invInertiaLocal[2][2] = localInertia.z > 0 ? 1.0f / localInertia.z : 0.0f;

        const Mat3 rotation = glm::mat3_cast(body->getRotation());
        return rotation * invInertiaLocal * glm::transpose(rotation);
    }

    // TODO: In the future, move it to the module manager, as it is a preset
    void CharacterIntegrator::integrate(Vec3& position, Vec3& velocity, const Vec3& acceleration,
                       const Float deltaTime, const bool isGrounded) const {
        // Apply gravity
        Vec3 totalAccel = acceleration;
        if (!isGrounded) {
            totalAccel += m_gravity;
        }

        // Semi-implicit Euler for stability
        velocity += totalAccel * deltaTime;

        // Apply friction when grounded
        if (isGrounded) {
            constexpr Float friction = 10.0f; // Ground friction

            velocity.x *= std::pow(0.1f, friction * deltaTime);
            velocity.z *= std::pow(0.1f, friction * deltaTime);

            // Prevent sliding on slopes
            if (std::abs(velocity.y) < 0.1f) {
                velocity.y = 0;
            }
        }

        // Clamp velocity
        constexpr Float maxSpeed = 20.0f;

        if (const Float speed = glm::length(Vec2(velocity.x, velocity.z)); speed > maxSpeed) {
            const Float scale = maxSpeed / speed;
            velocity.x *= scale;
            velocity.z *= scale;
        }

        // Terminal velocity
        velocity.y = std::max(velocity.y, -50.0f);

        // Update position
        position += velocity * deltaTime;
    }

    void VehicleIntegrator::integrate(Vec3& position, Quat& orientation, Vec3& velocity,
                              Vec3& angularVelocity, const Vec3& force, const Vec3& torque,
                              const Float mass, const Mat3& inertia, const Float deltaTime) {
        // Linear motion
        const Vec3 acceleration = force / mass;
        velocity += acceleration * deltaTime;

        // Apply drag
        constexpr Float dragCoefficient = 0.3f;
        const Vec3 dragForce = -velocity * glm::length(velocity) * dragCoefficient;
        velocity += dragForce * deltaTime / mass;

        position += velocity * deltaTime;

        // Angular motion
        const Vec3 angularAccel = glm::inverse(inertia) * torque;
        angularVelocity += angularAccel * deltaTime;

        // Angular damping
        angularVelocity *= 0.98f;

        // Update orientation
        const Quat spin(0, angularVelocity.x, angularVelocity.y, angularVelocity.z);
        const Quat derivative = spin * orientation * 0.5f;
        orientation = glm::normalize(orientation + derivative * deltaTime);
    }
} // namespace engine::physics
