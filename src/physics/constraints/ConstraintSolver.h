/**
 * @file ConstraintSolver.h
 * @brief Physics constraint solver for joint and contact constraints
 * @details Implements iterative constraint solving using Sequential Impulse method
 *          with warm starting and constraint stabilization
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

#include "../core/PhysicsConstants.h"
// #include "../../math/MathSystem.h"

// #include <vector>
// #include <algorithm>

namespace engine::physics {
    struct CollisionManifold;
    class RigidBody;
    using namespace engine::math;

    class IConstraint;
    /**
     * @brief Constraint solver configuration
     */
    struct SolverConfig {
        Int positionIterations = 10; // Position correction iterations
        Int velocityIterations = 10; // Velocity solver iterations
        Float maxLinearCorrection = 0.2f; // Maximum position correction per step
        Float maxAngularCorrection = 0.1f; // Maximum angular correction
        Float baumgarte = 0.2f; // Error reduction parameter (0-1)
        Float slop = 0.01f; // Allowed penetration
        Float warmStarting = 0.8f; // Warm starting factor (0-1)
        Float damping = 0.9f; // Velocity damping
        bool enableFriction = true;
        bool enableRestitution = true;
        bool enableWarmStarting = true;
        bool splitImpulse = true; // Split position/velocity solving

        SolverConfig() = default;

        static SolverConfig Accurate() {
            SolverConfig config;
            config.positionIterations = 20;
            config.velocityIterations = 20;
            config.warmStarting = 0.95f;
            return config;
        }

        static SolverConfig Fast() {
            SolverConfig config;
            config.positionIterations = 4;
            config.velocityIterations = 4;
            config.warmStarting = 0.5f;
            return config;
        }
    };

    /**
     * @brief Contact constraint for collision response
     */
    struct ContactConstraint {
        RigidBody* bodyA = nullptr;
        RigidBody* bodyB = nullptr;

        struct ContactPoint {
            Vec3 localPointA; // Contact point in A's local space
            Vec3 localPointB; // Contact point in B's local space
            Vec3 worldPointA; // Contact point in world space on A
            Vec3 worldPointB; // Contact point in world space on B
            Vec3 normal; // Contact normal (from A to B)
            Float penetration; // Penetration depth
            Float normalImpulse; // Accumulated normal impulse
            Float tangentImpulse1; // Accumulated friction impulse 1
            Float tangentImpulse2; // Accumulated friction impulse 2
            Float bias; // Position correction bias
            Float massNormal; // Effective mass along normal
            Float massTangent1; // Effective mass along tangent 1
            Float massTangent2; // Effective mass along tangent 2
            Float velocityBias; // Restitution bias
        };

        std::vector<ContactPoint> points;
        Vec3 tangent1; // Friction direction 1
        Vec3 tangent2; // Friction direction 2
        Float friction; // Combined friction
        Float restitution; // Combined restitution

        ContactConstraint() = default;

        /**
         * @brief Initialize from collision manifold
         */
        void initializeFromManifold(const CollisionManifold& manifold);

        void calculateTangents();
    };

    /**
     * @brief Sequential Impulse constraint solver
     * @details Solves constraints using the Sequential Impulse method with
     *          warm starting and split impulse for stable stacking
     */
    class ConstraintSolver {
    public:
        explicit ConstraintSolver(const SolverConfig& config = SolverConfig())
            : config_(config) {
            contactConstraints_.reserve(1024);
            jointConstraints_.reserve(256);
        }

        // ============================================================================
        // Main Solver
        // ============================================================================

        /**
         * @brief Solve all constraints
         */
        void solve(const std::vector<CollisionManifold>& manifolds,
                   const std::vector<IConstraint*>& constraints,
                   Float deltaTime);

        // ============================================================================
        // Configuration
        // ============================================================================

        void setConfig(const SolverConfig& config) { config_ = config; }
        const SolverConfig& getConfig() const { return config_; }

        void setGravity(const Vec3& gravity) { gravity_ = gravity; }

    private:
        SolverConfig config_;
        Vec3 gravity_ = Vec3(0, world::GRAVITY_EARTH, 0);
        Float deltaTime_ = 0.0f;
        Float invDeltaTime_ = 0.0f;

        // Constraint lists
        std::vector<ContactConstraint> contactConstraints_;
        std::vector<IConstraint*> jointConstraints_;

        // Warm starting cache
        std::unordered_map<std::uint64_t, ContactConstraint> constraintCache_;

        // ============================================================================
        // Constraint Building
        // ============================================================================

        void buildContactConstraints(const std::vector<CollisionManifold>& manifolds);

        void buildJointConstraints(const std::vector<IConstraint*>& constraints);

        // ============================================================================
        // Pre-Solve
        // ============================================================================

        void preSolve();

        void preSolveContact(ContactConstraint& constraint) const;

        // ============================================================================
        // Velocity Solver
        // ============================================================================

        void solveVelocityConstraints();

        void solveContactVelocity(ContactConstraint& constraint) const;

        // ============================================================================
        // Position Solver
        // ============================================================================

        void solvePositionConstraints();

        bool solveContactPosition(ContactConstraint& constraint) const;

        // ============================================================================
        // Helper Functions
        // ============================================================================

        void integrateVelocities() {
            // Velocities are integrated in the physics world update
            // This is a placeholder for any additional processing
        }

        void clearForces() const;

        void storeImpulses();

        static std::uint64_t getConstraintKey(RigidBody* bodyA, RigidBody* bodyB);

        static void matchAndRestoreImpulses(ContactConstraint& current, const ContactConstraint& previous);

        static Mat3 getInverseInertiaWorld(const RigidBody* body);
    };
} // namespace engine::physics
