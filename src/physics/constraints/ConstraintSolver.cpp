/**
* @file ConstraintSolver.h
 * @brief Physics constraint solver for joint and contact constraints
 * @details Implements iterative constraint solving using Sequential Impulse method
 *          with warm starting and constraint stabilization
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#include "ConstraintSolver.h"
#include "ConstraintTypes.h"

#include "../dynamics/RigidBody.h"

namespace engine::physics {
    void ContactConstraint::initializeFromManifold(const CollisionManifold& manifold) {
        bodyA = manifold.bodyA;
        bodyB = manifold.bodyB;

        // Combine material properties
        if (bodyA && bodyB) {
            friction = std::sqrt(bodyA->getFriction() * bodyB->getFriction());
            restitution = std::max(bodyA->getRestitution(), bodyB->getRestitution());
        }

        // Copy contact points
        points.clear();
        points.reserve(manifold.numContacts);

        for (Int i = 0; i < static_cast<Int>(manifold.numContacts); ++i) {
            ContactPoint point;
            point.worldPointA = manifold.contacts[i].positionWorldOnA;
            point.worldPointB = manifold.contacts[i].positionWorldOnB;
            point.normal = manifold.contacts[i].normalWorldOnB;
            point.penetration = manifold.contacts[i].distance;

            // Convert to local space for persistence
            point.localPointA = bodyA
                                    ? Vec3(glm::inverse(bodyA->getWorldMatrix()) * Vec4(point.worldPointA, 1.0f))
                                    : point.worldPointA;
            point.localPointB = bodyB
                                    ? Vec3(glm::inverse(bodyB->getWorldMatrix()) * Vec4(point.worldPointB, 1.0f))
                                    : point.worldPointB;

            // Initialize impulses to zero
            point.normalImpulse = 0.0f;
            point.tangentImpulse1 = 0.0f;
            point.tangentImpulse2 = 0.0f;

            points.push_back(point);
        }

        // Calculate friction directions
        calculateTangents();
    }

    void ContactConstraint::calculateTangents() {
        if (points.empty()) return;

        const Vec3& normal = points[0].normal;

        // Build orthonormal basis
        if (std::abs(normal.x) >= 0.57735f) {
            tangent1 = Vec3(normal.y, -normal.x, 0.0f);
        }
        else {
            tangent1 = Vec3(0.0f, normal.z, -normal.y);
        }

        tangent1 = glm::normalize(tangent1);
        tangent2 = glm::cross(normal, tangent1);
    }

    void ConstraintSolver::solve(const std::vector<CollisionManifold>& manifolds,
                                 const std::vector<IConstraint*>& constraints,
                                 const Float deltaTime) {
        if (deltaTime <= 0.0f) return;

        deltaTime_ = deltaTime;
        invDeltaTime_ = 1.0f / deltaTime;

        // Build constraint lists
        buildContactConstraints(manifolds);
        buildJointConstraints(constraints);

        // Pre-solve: calculate constraint properties and warm start
        preSolve();

        // Velocity solve: iteratively solve velocity constraints
        solveVelocityConstraints();

        // Integrate velocities
        integrateVelocities();

        // Position solve: correct position errors
        if (config_.splitImpulse) {
            solvePositionConstraints();
        }

        // Clear accumulated forces
        clearForces();

        // Store impulses for warm starting next frame
        storeImpulses();
    }

    void ConstraintSolver::buildContactConstraints(const std::vector<CollisionManifold>& manifolds) {
        contactConstraints_.clear();

        for (const auto& manifold : manifolds) {
            if (manifold.numContacts == 0) continue;

            ContactConstraint constraint;
            constraint.initializeFromManifold(manifold);

            // Try to restore accumulated impulses for warm starting
            if (config_.enableWarmStarting) {
                std::uint64_t key = getConstraintKey(constraint.bodyA, constraint.bodyB);
                if (auto it = constraintCache_.find(key); it != constraintCache_.end()) {
                    matchAndRestoreImpulses(constraint, it->second);
                }
            }

            contactConstraints_.push_back(constraint);
        }
    }

    void ConstraintSolver::buildJointConstraints(const std::vector<IConstraint*>& constraints) {
        jointConstraints_.clear();

        for (auto* constraint : constraints) {
            if (constraint && constraint->isEnabled()) {
                jointConstraints_.push_back(constraint);
            }
        }
    }

    void ConstraintSolver::preSolve() {
        // Pre-solve contact constraints
        for (auto& constraint : contactConstraints_) {
            preSolveContact(constraint);
        }

        // Pre-solve joint constraints
        for (auto* constraint : jointConstraints_) {
            constraint->preSolve(deltaTime_);
        }
    }

    void ConstraintSolver::preSolveContact(ContactConstraint& constraint) const {
        RigidBody* bodyA = constraint.bodyA;
        RigidBody* bodyB = constraint.bodyB;

        if (!bodyA && !bodyB) return;

        // Get velocities
        Vec3 vA = bodyA ? bodyA->getLinearVelocity() : Vec3(0);
        Vec3 wA = bodyA ? bodyA->getAngularVelocity() : Vec3(0);
        Vec3 vB = bodyB ? bodyB->getLinearVelocity() : Vec3(0);
        Vec3 wB = bodyB ? bodyB->getAngularVelocity() : Vec3(0);

        Float mA = bodyA ? bodyA->getMass() : 0.0f;
        Float mB = bodyB ? bodyB->getMass() : 0.0f;

        Float invMassA = mA > 0 ? 1.0f / mA : 0.0f;
        Float invMassB = mB > 0 ? 1.0f / mB : 0.0f;

        Mat3 invInertiaA = bodyA ? getInverseInertiaWorld(bodyA) : ZERO_3X3;
        Mat3 invInertiaB = bodyB ? getInverseInertiaWorld(bodyB) : ZERO_3X3;

        for (auto& point : constraint.points) {
            // Calculate relative positions
            Vec3 rA = point.worldPointA - (bodyA ? bodyA->getPosition() : point.worldPointA);
            Vec3 rB = point.worldPointB - (bodyB ? bodyB->getPosition() : point.worldPointB);

            // Calculate effective mass
            Vec3 rnA = glm::cross(rA, point.normal);
            Vec3 rnB = glm::cross(rB, point.normal);
            Float kNormal = invMassA + invMassB +
                glm::dot(rnA, invInertiaA * rnA) +
                glm::dot(rnB, invInertiaB * rnB);
            point.massNormal = kNormal > 0 ? 1.0f / kNormal : 0.0f;

            // Calculate tangent masses
            Vec3 rt1A = glm::cross(rA, constraint.tangent1);
            Vec3 rt1B = glm::cross(rB, constraint.tangent1);
            Float kTangent1 = invMassA + invMassB +
                glm::dot(rt1A, invInertiaA * rt1A) +
                glm::dot(rt1B, invInertiaB * rt1B);
            point.massTangent1 = kTangent1 > 0 ? 1.0f / kTangent1 : 0.0f;

            Vec3 rt2A = glm::cross(rA, constraint.tangent2);
            Vec3 rt2B = glm::cross(rB, constraint.tangent2);
            Float kTangent2 = invMassA + invMassB +
                glm::dot(rt2A, invInertiaA * rt2A) +
                glm::dot(rt2B, invInertiaB * rt2B);
            point.massTangent2 = kTangent2 > 0 ? 1.0f / kTangent2 : 0.0f;

            // Calculate position bias (Baumgarte stabilization)
            Float penetrationError = point.penetration - config_.slop;
            point.bias = 0.0f;
            if (penetrationError > 0) {
                point.bias = -config_.baumgarte * invDeltaTime_ * penetrationError;
            }

            // Calculate velocity bias (restitution)
            Vec3 vRel = (vB + glm::cross(wB, rB)) - (vA + glm::cross(wA, rA));
            Float vn = glm::dot(vRel, point.normal);

            point.velocityBias = 0.0f;
            if (config_.enableRestitution && vn < -1.0f) {
                point.velocityBias = -constraint.restitution * vn;
            }

            // Warm start
            if (config_.enableWarmStarting) {
                Vec3 impulse = point.normal * point.normalImpulse +
                    constraint.tangent1 * point.tangentImpulse1 +
                    constraint.tangent2 * point.tangentImpulse2;

                impulse *= config_.warmStarting;

                if (bodyA) {
                    vA -= impulse * invMassA;
                    wA -= invInertiaA * glm::cross(rA, impulse);
                }
                if (bodyB) {
                    vB += impulse * invMassB;
                    wB += invInertiaB * glm::cross(rB, impulse);
                }
            }
        }

        // Update velocities after warm starting
        if (bodyA) {
            bodyA->setLinearVelocity(vA);
            bodyA->setAngularVelocity(wA);
        }
        if (bodyB) {
            bodyB->setLinearVelocity(vB);
            bodyB->setAngularVelocity(wB);
        }
    }

    void ConstraintSolver::solveVelocityConstraints() {
        for (Int iter = 0; iter < config_.velocityIterations; ++iter) {
            // Solve joint constraints
            for (auto* constraint : jointConstraints_) {
                constraint->solveVelocity(deltaTime_);
            }

            // Solve contact constraints
            for (auto& constraint : contactConstraints_) {
                solveContactVelocity(constraint);
            }
        }
    }

    void ConstraintSolver::solveContactVelocity(ContactConstraint& constraint) const {
        RigidBody* bodyA = constraint.bodyA;
        RigidBody* bodyB = constraint.bodyB;

        if (!bodyA && !bodyB) return;

        Vec3 vA = bodyA ? bodyA->getLinearVelocity() : Vec3(0);
        Vec3 wA = bodyA ? bodyA->getAngularVelocity() : Vec3(0);
        Vec3 vB = bodyB ? bodyB->getLinearVelocity() : Vec3(0);
        Vec3 wB = bodyB ? bodyB->getAngularVelocity() : Vec3(0);

        Float invMassA = bodyA && bodyA->getMass() > 0 ? 1.0f / bodyA->getMass() : 0.0f;
        Float invMassB = bodyB && bodyB->getMass() > 0 ? 1.0f / bodyB->getMass() : 0.0f;

        Mat3 invInertiaA = bodyA ? getInverseInertiaWorld(bodyA) : ZERO_3X3;
        Mat3 invInertiaB = bodyB ? getInverseInertiaWorld(bodyB) : ZERO_3X3;

        for (auto& point : constraint.points) {
            Vec3 rA = point.worldPointA - (bodyA ? bodyA->getPosition() : point.worldPointA);
            Vec3 rB = point.worldPointB - (bodyB ? bodyB->getPosition() : point.worldPointB);

            // Calculate relative velocity
            Vec3 vRel = (vB + glm::cross(wB, rB)) - (vA + glm::cross(wA, rA));

            // Solve normal constraint
            Float vn = glm::dot(vRel, point.normal);
            Float lambda = point.massNormal * (point.bias + point.velocityBias - vn);

            Float oldImpulse = point.normalImpulse;
            point.normalImpulse = std::max(oldImpulse + lambda, 0.0f);
            lambda = point.normalImpulse - oldImpulse;

            Vec3 impulse = point.normal * lambda;

            if (bodyA) {
                vA -= impulse * invMassA;
                wA -= invInertiaA * glm::cross(rA, impulse);
            }
            if (bodyB) {
                vB += impulse * invMassB;
                wB += invInertiaB * glm::cross(rB, impulse);
            }

            // Solve friction constraints
            if (config_.enableFriction) {
                vRel = (vB + glm::cross(wB, rB)) - (vA + glm::cross(wA, rA));

                Float maxFriction = constraint.friction * point.normalImpulse;

                // Tangent 1
                Float vt1 = glm::dot(vRel, constraint.tangent1);
                lambda = point.massTangent1 * (-vt1);

                oldImpulse = point.tangentImpulse1;
                point.tangentImpulse1 = clamp(oldImpulse + lambda, -maxFriction, maxFriction);
                lambda = point.tangentImpulse1 - oldImpulse;

                impulse = constraint.tangent1 * lambda;

                if (bodyA) {
                    vA -= impulse * invMassA;
                    wA -= invInertiaA * glm::cross(rA, impulse);
                }
                if (bodyB) {
                    vB += impulse * invMassB;
                    wB += invInertiaB * glm::cross(rB, impulse);
                }

                // Tangent 2
                vRel = (vB + glm::cross(wB, rB)) - (vA + glm::cross(wA, rA));
                Float vt2 = glm::dot(vRel, constraint.tangent2);
                lambda = point.massTangent2 * (-vt2);

                oldImpulse = point.tangentImpulse2;
                point.tangentImpulse2 = clamp(oldImpulse + lambda, -maxFriction, maxFriction);
                lambda = point.tangentImpulse2 - oldImpulse;

                impulse = constraint.tangent2 * lambda;

                if (bodyA) {
                    vA -= impulse * invMassA;
                    wA -= invInertiaA * glm::cross(rA, impulse);
                }
                if (bodyB) {
                    vB += impulse * invMassB;
                    wB += invInertiaB * glm::cross(rB, impulse);
                }
            }
        }

        // Update velocities
        if (bodyA) {
            bodyA->setLinearVelocity(vA);
            bodyA->setAngularVelocity(wA);
        }
        if (bodyB) {
            bodyB->setLinearVelocity(vB);
            bodyB->setAngularVelocity(wB);
        }
    }

    void ConstraintSolver::solvePositionConstraints() {
        for (Int iter = 0; iter < config_.positionIterations; ++iter) {
            bool allSolved = true;

            // Solve joint position constraints
            for (auto* constraint : jointConstraints_) {
                if (!constraint->solvePosition(deltaTime_)) {
                    allSolved = false;
                }
            }

            // Solve contact position constraints
            for (auto& constraint : contactConstraints_) {
                if (!solveContactPosition(constraint)) {
                    allSolved = false;
                }
            }

            if (allSolved) break;
        }
    }

    bool ConstraintSolver::solveContactPosition(ContactConstraint& constraint) const {
        RigidBody* bodyA = constraint.bodyA;
        RigidBody* bodyB = constraint.bodyB;

        if (!bodyA && !bodyB) return true;

        Vec3 posA = bodyA ? bodyA->getPosition() : Vec3(0);
        Quat rotA = bodyA ? bodyA->getRotation() : Quat(1, 0, 0, 0);
        Vec3 posB = bodyB ? bodyB->getPosition() : Vec3(0);
        Quat rotB = bodyB ? bodyB->getRotation() : Quat(1, 0, 0, 0);

        Float invMassA = bodyA && bodyA->getMass() > 0 ? 1.0f / bodyA->getMass() : 0.0f;
        Float invMassB = bodyB && bodyB->getMass() > 0 ? 1.0f / bodyB->getMass() : 0.0f;

        Mat3 invInertiaA = bodyA ? getInverseInertiaWorld(bodyA) : ZERO_3X3;
        Mat3 invInertiaB = bodyB ? getInverseInertiaWorld(bodyB) : ZERO_3X3;

        Float maxError = 0.0f;

        for (auto& point : constraint.points) {
            // Recalculate world positions from local
            Vec3 worldA = Vec3(glm::mat4_cast(rotA) * Vec4(point.localPointA, 1.0f)) + posA;
            Vec3 worldB = Vec3(glm::mat4_cast(rotB) * Vec4(point.localPointB, 1.0f)) + posB;

            Vec3 separation = worldB - worldA;
            Float penetration = glm::dot(separation, point.normal);

            Float error = std::min(penetration + config_.slop, 0.0f);
            maxError = std::min(maxError, error);

            if (error >= 0) continue;

            Vec3 rA = worldA - posA;
            Vec3 rB = worldB - posB;

            // Calculate effective mass
            Vec3 rnA = glm::cross(rA, point.normal);
            Vec3 rnB = glm::cross(rB, point.normal);
            Float kNormal = invMassA + invMassB +
                glm::dot(rnA, invInertiaA * rnA) +
                glm::dot(rnB, invInertiaB * rnB);

            Float massNormal = kNormal > 0 ? 1.0f / kNormal : 0.0f;

            // Calculate position impulse
            Float correction = clamp(-error * 0.9f, 0.0f, config_.maxLinearCorrection);
            Float impulse = massNormal * correction;

            Vec3 correctionVec = point.normal * impulse;

            // Apply position correction
            if (bodyA) {
                posA -= correctionVec * invMassA;
                Vec3 angularImpulse = glm::cross(rA, correctionVec);
                Vec3 deltaRotation = invInertiaA * angularImpulse;
                rotA = glm::normalize(rotA + Quat(0, deltaRotation.x, deltaRotation.y, deltaRotation.z) * rotA * 0.5f);
            }
            if (bodyB) {
                posB += correctionVec * invMassB;
                Vec3 angularImpulse = glm::cross(rB, correctionVec);
                Vec3 deltaRotation = invInertiaB * angularImpulse;
                rotB = glm::normalize(rotB + Quat(0, deltaRotation.x, deltaRotation.y, deltaRotation.z) * rotB * 0.5f);
            }
        }

        // Update positions
        if (bodyA) {
            bodyA->setPosition(posA);
            bodyA->setRotation(rotA);
        }
        if (bodyB) {
            bodyB->setPosition(posB);
            bodyB->setRotation(rotB);
        }

        return maxError >= -0.01f;
    }

    void ConstraintSolver::clearForces() const {
        for (auto& constraint : contactConstraints_) {
            if (constraint.bodyA) constraint.bodyA->clearForces();
            if (constraint.bodyB) constraint.bodyB->clearForces();
        }
    }

    void ConstraintSolver::storeImpulses() {
        constraintCache_.clear();

        for (const auto& constraint : contactConstraints_) {
            std::uint64_t key = getConstraintKey(constraint.bodyA, constraint.bodyB);
            constraintCache_[key] = constraint;
        }
    }

    std::uint64_t ConstraintSolver::getConstraintKey(RigidBody* bodyA, RigidBody* bodyB) {
        std::uint64_t ptrA = reinterpret_cast<std::uint64_t>(bodyA);
        std::uint64_t ptrB = reinterpret_cast<std::uint64_t>(bodyB);

        if (ptrA > ptrB) std::swap(ptrA, ptrB);

        return (ptrA << 32) | (ptrB & 0xFFFFFFFF);
    }

    void ConstraintSolver::matchAndRestoreImpulses(ContactConstraint& current, const ContactConstraint& previous) {
        // Match contact points and restore impulses for warm starting
        for (auto& currentPoint : current.points) {
            Float bestDistance = INFINITY_VALUE<Float>;
            const ContactConstraint::ContactPoint* bestMatch = nullptr;

            for (const auto& prevPoint : previous.points) {
                const Float dist = glm::length2(currentPoint.localPointA - prevPoint.localPointA) +
                    glm::length2(currentPoint.localPointB - prevPoint.localPointB);

                if (dist < bestDistance && dist < 0.01f) {
                    bestDistance = dist;
                    bestMatch = &prevPoint;
                }
            }

            if (bestMatch) {
                currentPoint.normalImpulse = bestMatch->normalImpulse;
                currentPoint.tangentImpulse1 = bestMatch->tangentImpulse1;
                currentPoint.tangentImpulse2 = bestMatch->tangentImpulse2;
            }
        }
    }

    Mat3 ConstraintSolver::getInverseInertiaWorld(const RigidBody* body) {
        // Get local inertia tensor
        const Vec3 localInertia = body->getLocalInertia();
        if (glm::length2(localInertia) < EPSILON_SQUARED) return ZERO_3X3;

        // Create diagonal matrix
        Mat3 invInertiaLocal(0);
        invInertiaLocal[0][0] = localInertia.x > 0 ? 1.0f / localInertia.x : 0.0f;
        invInertiaLocal[1][1] = localInertia.y > 0 ? 1.0f / localInertia.y : 0.0f;
        invInertiaLocal[2][2] = localInertia.z > 0 ? 1.0f / localInertia.z : 0.0f;

        // Transform to world space
        const Mat3 rotation = glm::mat3_cast(body->getRotation());
        return rotation * invInertiaLocal * glm::transpose(rotation);
    }
} // namespace engine::physics
