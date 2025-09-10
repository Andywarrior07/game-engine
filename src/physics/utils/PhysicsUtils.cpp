/**
 * @file PhysicsUtils.cpp
 * @brief Utility functions and helpers for physics calculations
 * @details Provides common physics calculations, conversions, and helper functions
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#include "PhysicsUtils.h"

#include "../dynamics/RigidBody.h"

namespace engine::physics {
    Vec3 PhysicsUtils::calculateBoxInertia(const Float mass, const Vec3& halfExtents) {
        const Vec3 size = halfExtents * 2.0f;
        const Float x2 = size.x * size.x;
        const Float y2 = size.y * size.y;
        const Float z2 = size.z * size.z;

        return Vec3(
            mass * (y2 + z2) / 12.0f,
            mass * (x2 + z2) / 12.0f,
            mass * (x2 + y2) / 12.0f
        );
    }

    Vec3 PhysicsUtils::calculateSphereInertia(const Float mass, const Float radius) {
        const Float i = 0.4f * mass * radius * radius;

        return Vec3(i, i, i);
    }

    Vec3 PhysicsUtils::calculateCylinderInertia(const Float mass, const Float radius, const Float height,
                                                const Int axis) {
        const Float iRadius = 0.5f * mass * radius * radius;
        const Float iHeight = mass * (3.0f * radius * radius + height * height) / 12.0f;

        switch (axis) {
        case 0: return Vec3(iRadius, iHeight, iHeight); // X-axis
        case 1: return Vec3(iHeight, iRadius, iHeight); // Y-axis
        case 2: return Vec3(iHeight, iHeight, iRadius); // Z-axis
        default: return Vec3(iHeight, iRadius, iHeight);
        }
    }

    Vec3 PhysicsUtils::calculateCapsuleInertia(const Float mass, const Float radius, const Float height,
                                               const Int axis) {
        // Approximate as cylinder + 2 hemispheres
        const Float cylinderMass = mass * height / (height + 4.0f * radius / 3.0f);
        const Float hemisphereMass = (mass - cylinderMass) / 2.0f;

        const Vec3 cylinderInertia = calculateCylinderInertia(cylinderMass, radius, height, axis);
        const Float sphereI = 0.4f * hemisphereMass * radius * radius;
        const Float offset = (height / 2.0f + 3.0f * radius / 8.0f);

        Vec3 capsuleInertia = cylinderInertia;
        if (axis == 1) {
            capsuleInertia.x += 2.0f * (sphereI + hemisphereMass * offset * offset);
            capsuleInertia.z += 2.0f * (sphereI + hemisphereMass * offset * offset);
        }

        return capsuleInertia;
    }

    Vec3 PhysicsUtils::calculateMeshInertia(const Float mass, const std::vector<Vec3>& vertices,
                                            const std::vector<Int>& indices) {
        if (vertices.empty() || indices.empty()) {
            return Vec3(1, 1, 1); // Default
        }

        // Calculate volume and center of mass
        Vec3 centerOfMass(0);
        Float totalVolume = 0;

        for (size_t i = 0; i < indices.size(); i += 3) {
            const Vec3& v0 = vertices[indices[i]];
            const Vec3& v1 = vertices[indices[i + 1]];
            const Vec3& v2 = vertices[indices[i + 2]];

            const Float volume = glm::dot(v0, glm::cross(v1, v2)) / 6.0f;
            totalVolume += volume;
            centerOfMass += (v0 + v1 + v2) * volume / 4.0f;
        }

        centerOfMass /= totalVolume;

        // Calculate inertia tensor
        Mat3 inertiaTensor(0);

        for (size_t i = 0; i < indices.size(); i += 3) {
            Vec3 v0 = vertices[indices[i]] - centerOfMass;
            Vec3 v1 = vertices[indices[i + 1]] - centerOfMass;
            Vec3 v2 = vertices[indices[i + 2]] - centerOfMass;

            const Mat3 tetraInertia = calculateTetrahedronInertia(v0, v1, v2);
            inertiaTensor += tetraInertia;
        }

        const Float density = mass / std::abs(totalVolume);
        inertiaTensor *= density;

        return Vec3(inertiaTensor[0][0], inertiaTensor[1][1], inertiaTensor[2][2]);
    }

    Vec3 PhysicsUtils::calculateDragForce(const Vec3& velocity, const Float dragCoefficient,
                                          const Float crossSectionalArea, const Float fluidDensity) {
        const Float speed = glm::length(velocity);
        if (speed < EPSILON) return Vec3(0);

        const Vec3 direction = -velocity / speed;
        const Float dragMagnitude = 0.5f * fluidDensity * speed * speed *
            dragCoefficient * crossSectionalArea;

        return direction * dragMagnitude;
    }

    Vec3 PhysicsUtils::calculateLiftForce(const Vec3& velocity, const Vec3& liftDirection,
                                          const Float liftCoefficient, const Float wingArea,
                                          const Float fluidDensity) {
        const Float speed = glm::length(velocity);
        if (speed < EPSILON) return Vec3(0);

        const Float liftMagnitude = 0.5f * fluidDensity * speed * speed *
            liftCoefficient * wingArea;

        return glm::normalize(liftDirection) * liftMagnitude;
    }

    Vec3 PhysicsUtils::calculateSpringForce(const Vec3& displacement, const Float springConstant,
                                            const Float damping, const Vec3& velocity) {
        const Vec3 springForce = -springConstant * displacement;
        const Vec3 dampingForce = -damping * velocity;

        return springForce + dampingForce;
    }

    Vec3 PhysicsUtils::calculateTorque(const Vec3& force, const Vec3& position,
                                       const Vec3& centerOfMass) {
        const Vec3 r = position - centerOfMass;

        return glm::cross(r, force);
    }

    Float PhysicsUtils::calculateCollisionImpulse(const Vec3& relativeVelocity,
                                                  const Vec3& collisionNormal,
                                                  const Float massA, const Float massB,
                                                  const Float restitution) {
        const Float velocityAlongNormal = glm::dot(relativeVelocity, collisionNormal);

        if (velocityAlongNormal > 0) return 0; // Bodies separating

        const Float invMassSum = (massA > 0 ? 1.0f / massA : 0) +
            (massB > 0 ? 1.0f / massB : 0);

        if (invMassSum <= 0) return 0;

        const Float impulse = -(1.0f + restitution) * velocityAlongNormal / invMassSum;

        return impulse;
    }

    Vec3 PhysicsUtils::calculateFrictionImpulse(const Vec3& relativeVelocity,
                                                const Vec3& collisionNormal,
                                                const Float normalImpulse,
                                                const Float staticFriction,
                                                const Float dynamicFriction) {
        Vec3 tangent = relativeVelocity - collisionNormal * glm::dot(relativeVelocity, collisionNormal);
        const Float tangentSpeed = glm::length(tangent);

        if (tangentSpeed < EPSILON) return Vec3(0);

        tangent /= tangentSpeed;

        const Float maxStaticFriction = staticFriction * std::abs(normalImpulse);
        Float frictionMagnitude;

        if (tangentSpeed < maxStaticFriction) {
            frictionMagnitude = tangentSpeed; // Static friction
        }
        else {
            frictionMagnitude = dynamicFriction * std::abs(normalImpulse); // Dynamic friction
        }

        return -tangent * frictionMagnitude;
    }

    void PhysicsUtils::resolvePenetration(Vec3& positionA, Vec3& positionB,
                                          const Float massA, const Float massB,
                                          const Vec3& normal, const Float penetration,
                                          const Float correctionPercent,
                                          const Float slop) {
        const Float invMassSum = (massA > 0 ? 1.0f / massA : 0) +
            (massB > 0 ? 1.0f / massB : 0);

        if (invMassSum <= 0 || penetration < slop) return;

        const Vec3 correction = normal * (std::max(penetration - slop, 0.0f) / invMassSum * correctionPercent);

        if (massA > 0) positionA -= correction / massA;
        if (massB > 0) positionB += correction / massB;
    }

    Float PhysicsUtils::calculateKineticEnergy(const Float mass, const Vec3& velocity) {
        return 0.5f * mass * glm::dot(velocity, velocity);
    }

    Float PhysicsUtils::calculateRotationalEnergy(const Vec3& inertia, const Vec3& angularVelocity) {
        return 0.5f * glm::dot(inertia * angularVelocity, angularVelocity);
    }

    Vec3 PhysicsUtils::calculateLaunchVelocity(const Vec3& start, const Vec3& target,
                                               const Float angle, const Float gravity) {
        const Vec3 displacement = target - start;
        const Float distance = glm::length(Vec2(displacement.x, displacement.z));
        const Float height = displacement.y;

        const Float angleRad = glm::radians(angle);
        const Float sinAngle = std::sin(angleRad);
        const Float cosAngle = std::cos(angleRad);
        const Float sin2Angle = std::sin(2.0f * angleRad);

        const Float speed = std::sqrt(gravity * distance * distance /
            (distance * sin2Angle - 2.0f * height * cosAngle * cosAngle));

        const Vec3 direction = glm::normalize(Vec3(displacement.x, 0, displacement.z));
        Vec3 velocity = direction * speed * cosAngle;
        velocity.y = speed * sinAngle;

        return velocity;
    }

    std::optional<Vec3> PhysicsUtils::predictIntercept(const Vec3& shooterPos,
                                                       const Float projectileSpeed,
                                                       const Vec3& targetPos,
                                                       const Vec3& targetVelocity) {
        const Vec3 toTarget = targetPos - shooterPos;
        const Float a = glm::dot(targetVelocity, targetVelocity) - projectileSpeed * projectileSpeed;
        const Float b = 2.0f * glm::dot(targetVelocity, toTarget);
        const Float c = glm::dot(toTarget, toTarget);

        const Float discriminant = b * b - 4.0f * a * c;
        if (discriminant < 0) return std::nullopt;

        const Float t1 = (-b - std::sqrt(discriminant)) / (2.0f * a);
        const Float t2 = (-b + std::sqrt(discriminant)) / (2.0f * a);

        const Float t = t1 > 0 ? t1 : t2;
        if (t < 0) return std::nullopt;

        return targetPos + targetVelocity * t;
    }

    void PhysicsUtils::limitVelocity(Vec3& velocity, const Float maxSpeed) {
        if (const Float speed = glm::length(velocity); speed > maxSpeed) {
            velocity = (velocity / speed) * maxSpeed;
        }
    }

    void PhysicsUtils::limitAngularVelocity(Vec3& angularVelocity, const Float maxAngularSpeed) {
        if (const Float speed = glm::length(angularVelocity); speed > maxAngularSpeed) {
            angularVelocity = (angularVelocity / speed) * maxAngularSpeed;
        }
    }

    void PhysicsUtils::clampToBounds(Vec3& position, const AABB& bounds) {
        position = glm::clamp(position, bounds.min, bounds.max);
    }

    Vec3 PhysicsUtils::smoothDamp(const Vec3& current, const Vec3& target,
                                  Vec3& currentVelocity, Float smoothTime,
                                  const Float maxSpeed, const Float deltaTime) {
        smoothTime = std::max(0.0001f, smoothTime);
        const Float omega = 2.0f / smoothTime;
        const Float x = omega * deltaTime;
        const Float exp = 1.0f / (1.0f + x + 0.48f * x * x + 0.235f * x * x * x);

        Vec3 change = current - target;
        const Vec3 originalTo = target;

        const Float maxChange = maxSpeed * smoothTime;

        if (const Float changeMag = glm::length(change); changeMag > maxChange) {
            change = (change / changeMag) * maxChange;
        }

        const Vec3 targetPos = current - change;
        const Vec3 temp = (currentVelocity + omega * change) * deltaTime;
        currentVelocity = (currentVelocity - omega * temp) * exp;

        Vec3 output = targetPos + (change + temp) * exp;

        // Prevent overshooting
        if (glm::dot(originalTo - current, output - originalTo) > 0) {
            output = originalTo;
            currentVelocity = (output - originalTo) / deltaTime;
        }

        return output;
    }

    bool PhysicsUtils::isBodyAtRest(const RigidBody* body, const Float linearThreshold,
                                    const Float angularThreshold) {
        if (!body) return true;

        const Float linearSpeed = glm::length(body->getLinearVelocity());
        const Float angularSpeed = glm::length(body->getAngularVelocity());

        return linearSpeed < linearThreshold && angularSpeed < angularThreshold;
    }

    Vec3 PhysicsUtils::calculateCenterOfMass(const std::vector<RigidBody*>& bodies) {
        Vec3 centerOfMass(0);
        Float totalMass = 0;

        for (const RigidBody* body : bodies) {
            if (body && body->getMass() > 0) {
                centerOfMass += body->getPosition() * body->getMass();
                totalMass += body->getMass();
            }
        }

        if (totalMass > 0) {
            centerOfMass /= totalMass;
        }

        return centerOfMass;
    }

    Vec3 PhysicsUtils::calculateRelativeVelocity(const RigidBody* bodyA, const RigidBody* bodyB,
                                              const Vec3& contactPoint) {
        const Vec3 velA = bodyA ? bodyA->getVelocityAtWorldPoint(contactPoint) : Vec3(0);
        const Vec3 velB = bodyB ? bodyB->getVelocityAtWorldPoint(contactPoint) : Vec3(0);

        return velA - velB;
    }

    Float PhysicsUtils::calculateSeparationVelocity(const Vec3& relativeVelocity,
                                                 const Vec3& contactNormal) {
        return glm::dot(relativeVelocity, contactNormal);
    }

    Mat3 PhysicsUtils::calculateTetrahedronInertia(const Vec3& v0, const Vec3& v1, const Vec3& v2) {
        Mat3 inertia(0);

        const Float volume = std::abs(glm::dot(v0, glm::cross(v1, v2))) / 6.0f;

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                if (i == j) {
                    // Diagonal elements
                    Float sum = 0;
                    for (int k = 0; k < 3; ++k) {
                        if (k != i) {
                            sum += v0[k] * v0[k] + v1[k] * v1[k] + v2[k] * v2[k] +
                                v0[k] * v1[k] + v0[k] * v2[k] + v1[k] * v2[k];
                        }
                    }
                    inertia[i][i] = volume * sum / 10.0f;
                }
                else {
                    // Off-diagonal elements
                    const Float sum = v0[i] * v0[j] + v1[i] * v1[j] + v2[i] * v2[j] +
                    (v0[i] * v1[j] + v1[i] * v0[j] + v0[i] * v2[j] +
                        v2[i] * v0[j] + v1[i] * v2[j] + v2[i] * v1[j]) / 2.0f;
                    inertia[i][j] = -volume * sum / 10.0f;
                }
            }
        }

        return inertia;
    }
} // namespace engine::physics
