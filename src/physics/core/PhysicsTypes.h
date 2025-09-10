/**
 * @file PhysicsTypes.h
 * @brief Core physics type definitions and enumerations
 * @details Defines fundamental types used throughout the physics system,
 *          including collision categories, physics materials, and body types
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

#include "PhysicsConstants.h"
#include "../../memory/MemorySystem.h"

#include <btBulletDynamicsCommon.h>
#include <functional>

namespace engine::physics {
    using namespace engine::math;
    using namespace engine::memory;

    // Forward declarations
    class RigidBody;
    class PhysicsWorld;
    class CollisionShape;
    struct ContactPoint;
    struct RaycastHit;

    // ============================================================================
    // Enumerations
    // ============================================================================

    /**
     * @brief Types of rigid bodies in the physics simulation
     */
    enum class BodyType : std::uint8_t {
        STATIC = 0, // Immovable objects (terrain, walls)
        KINEMATIC = 1, // Controlled by animation/script (platforms, doors)
        DYNAMIC = 2, // Full physics simulation (characters, projectiles)
        GHOST = 3 // Trigger/sensor objects (no collision response)
    };

    /**
     * @brief Collision shape types
     */
    enum class ShapeType : std::uint8_t {
        BOX,
        SPHERE,
        CAPSULE,
        CYLINDER,
        CONE,
        CONVEX_HULL,
        TRIANGLE_MESH,
        HEIGHTFIELD,
        COMPOUND,
        PLANE
    };

    /**
     * @brief Collision filtering groups (bit flags)
     */
    enum CollisionGroup : std::uint16_t {
        NONE = 0,
        DEFAULT = 1 << 0,
        STATIC = 1 << 1,
        KINEMATIC = 1 << 2,
        DEBRIS = 1 << 3,
        SENSOR = 1 << 4,
        CHARACTER = 1 << 5,
        VEHICLE = 1 << 6,
        PROJECTILE = 1 << 7,
        WATER = 1 << 8,
        TRIGGER = 1 << 9,
        RAGDOLL = 1 << 10,
        PARTICLE = 1 << 11,
        ALL = 0xFFFF
    };

    /**
     * @brief Constraint types for joints
     */
    enum class ConstraintType : std::uint8_t {
        POINT_TO_POINT,
        HINGE,
        SLIDER,
        CONE_TWIST,
        GENERIC_6DOF,
        GENERIC_6DOF_SPRING,
        UNIVERSAL,
        FIXED,
        GEAR
    };

    /**
     * @brief Activation states for physics bodies
     */
    enum class ActivationState : std::uint8_t {
        ACTIVE_STATE = ACTIVE_TAG,
        SLEEPING_STATE = ISLAND_SLEEPING,
        WANTS_DEACTIVATION_STATE = WANTS_DEACTIVATION,
        DISABLE_DEACTIVATION_STATE = DISABLE_DEACTIVATION,
        DISABLE_SIMULATION_STATE = DISABLE_SIMULATION
    };

    /**
     * @brief Debug draw modes
     */
    enum class DebugDrawMode : std::uint32_t {
        NONE = 0,
        WIREFRAME = 1 << 0,
        AABB = 1 << 1,
        CONTACT_POINTS = 1 << 2,
        CONSTRAINTS = 1 << 3,
        CONSTRAINT_LIMITS = 1 << 4,
        NORMALS = 1 << 5,
        FRAMES = 1 << 6,
        ALL = 0xFFFFFFFF
    };

    // ============================================================================
    // Physics Material
    // ============================================================================

    /**
     * @brief Physical material properties
     */
    struct PhysicsMaterial {
        Float friction = material::FRICTION_DEFAULT;
        Float restitution = material::RESTITUTION_DEFAULT; // Bounciness
        Float linearDamping = material::DAMPING_DEFAULT_LINEAR;
        Float angularDamping = material::DAMPING_DEFAULT_ANGULAR;
        Float density = 1.0f; // kg/m³

        // Advanced properties
        Float staticFriction = 0.6f;
        Float dynamicFriction = 0.4f;
        Float rollingFriction = 0.01f;
        Float spinningFriction = 0.01f;

        // Flags
        bool enableAnisotropicFriction = false;
        Vec3 anisotropicFrictionDirection = Vec3(1, 1, 1);

        PhysicsMaterial() = default;

        PhysicsMaterial(const Float friction, const Float restitution, const Float density = 1.0f)
            : friction(friction), restitution(restitution), density(density) {
        }

        // Preset materials
        static PhysicsMaterial Concrete() { return {0.8f, 0.2f, 2400.0f}; }
        static PhysicsMaterial Wood() { return {0.4f, 0.3f, 700.0f}; }
        static PhysicsMaterial Metal() { return {0.3f, 0.4f, 7850.0f}; }
        static PhysicsMaterial Ice() { return {0.05f, 0.1f, 917.0f}; }
        static PhysicsMaterial Rubber() { return {0.9f, 0.8f, 1500.0f}; }
        static PhysicsMaterial Glass() { return {0.2f, 0.6f, 2500.0f}; }
    };

    // ============================================================================
    // Collision Data Structures
    // ============================================================================

    /**
     * @brief Contact point information
     */
    struct ContactPoint {
        Vec3 positionWorldOnA; // Contact position on body A
        Vec3 positionWorldOnB; // Contact position on body B
        Vec3 normalWorldOnB; // Normal pointing from B to A
        Float distance; // Penetration depth (negative = penetrating)
        Float impulse; // Applied impulse magnitude
        Float combinedFriction; // Combined friction coefficient
        Float combinedRestitution; // Combined restitution coefficient

        RigidBody* bodyA = nullptr;
        RigidBody* bodyB = nullptr;

        ContactPoint() : positionWorldOnA(VEC3_ZERO), positionWorldOnB(VEC3_ZERO), normalWorldOnB(VEC3_ZERO),
                         distance(0), impulse(0),
                         combinedFriction(0), combinedRestitution(0) {
        }
    };

    /**
     * @brief Collision manifold containing multiple contact points
     */
    struct CollisionManifold {
        std::array<ContactPoint, collision::MAX_CONTACT_POINTS> contacts;
        std::size_t numContacts = 0;
        RigidBody* bodyA = nullptr;
        RigidBody* bodyB = nullptr;

        void addContactPoint(const ContactPoint& point) {
            if (numContacts < collision::MAX_CONTACT_POINTS) {
                contacts[numContacts++] = point;
            }
        }

        void clear() {
            numContacts = 0;
            bodyA = nullptr;
            bodyB = nullptr;
        }
    };

    /**
     * @brief Raycast hit information
     */
    struct RaycastHit {
        Vec3 point; // Hit point in world space
        Vec3 normal; // Surface normal at hit point
        Float distance; // Distance from ray origin
        Float fraction; // Normalized distance (0-1)
        RigidBody* body = nullptr; // Hit body
        void* userData = nullptr; // Custom user data

        RaycastHit() : point(VEC3_ZERO), normal(VEC3_ZERO), distance(0), fraction(0) {
        }
    };

    /**
     * @brief Sweep test result
     */
    struct SweepResult {
        Vec3 point;
        Vec3 normal;
        Float fraction; // Time of impact (0-1)
        RigidBody* body = nullptr;

        SweepResult() : point(VEC3_ZERO), normal(VEC3_ZERO), fraction(1.0f) {
        }
    };

    /**
     * @brief Overlap query result
     */
    struct OverlapResult {
        std::vector<RigidBody*> bodies;
        std::size_t count = 0;

        void clear() {
            bodies.clear();
            count = 0;
        }
    };

    // ============================================================================
    // Callbacks and Listeners
    // ============================================================================

    /**
     * @brief Collision callback function types
     */
    using CollisionEnterCallback = std::function<void(RigidBody*, RigidBody*, const CollisionManifold&)>;
    using CollisionStayCallback = std::function<void(RigidBody*, RigidBody*, const CollisionManifold&)>;
    using CollisionExitCallback = std::function<void(RigidBody*, RigidBody*)>;
    using TriggerEnterCallback = std::function<void(RigidBody*, RigidBody*)>;
    using TriggerExitCallback = std::function<void(RigidBody*, RigidBody*)>;
    using PreSolveCallback = std::function<void(RigidBody*, RigidBody*, CollisionManifold&)>;
    using PostSolveCallback = std::function<void(RigidBody*, RigidBody*, const ContactPoint&)>;

    /**
     * @brief Physics event listener interface
     */
    class IPhysicsListener {
    public:
        virtual ~IPhysicsListener() = default;

        virtual void onCollisionEnter(RigidBody* bodyA, RigidBody* bodyB,
                                      const CollisionManifold& manifold) = 0;

        virtual void onCollisionStay(RigidBody* bodyA, RigidBody* bodyB,
                                     const CollisionManifold& manifold) = 0;

        virtual void onCollisionExit(RigidBody* bodyA, RigidBody* bodyB) = 0;

        virtual void onTriggerEnter(RigidBody* trigger, RigidBody* other) = 0;

        virtual void onTriggerStay(RigidBody* trigger, RigidBody* other) = 0;

        virtual void onTriggerExit(RigidBody* trigger, RigidBody* other) = 0;

        virtual void onWake(RigidBody* body) = 0;

        virtual void onSleep(RigidBody* body) = 0;
    };

    // ============================================================================
    // Physics Query Filters
    // ============================================================================

    /**
     * @brief Filter for physics queries (raycast, overlap, sweep)
     */
    struct QueryFilter {
        std::uint16_t groupMask = ALL;
        std::uint16_t categoryMask = ALL;
        bool includeStatic = true;
        bool includeKinematic = true;
        bool includeDynamic = true;
        bool includeGhost = false;
        bool includeTriggers = false;

        // Custom filter callback
        std::function<bool(RigidBody*)> customFilter = nullptr;

        QueryFilter() = default;

        explicit QueryFilter(const std::uint16_t mask)
            : groupMask(mask), categoryMask(mask) {
        }

        bool shouldTest(RigidBody* body) const;
    };

    // ============================================================================
    // Physics Statistics
    // ============================================================================

    /**
     * @brief Physics performance statistics
     */
    struct PhysicsStats {
        // Timing
        Float stepTime = 0.0f; // Total step time (ms)
        Float collisionTime = 0.0f; // Collision detection time (ms)
        Float solverTime = 0.0f; // Constraint solving time (ms)
        Float integrationTime = 0.0f; // Integration time (ms)

        // Counts
        std::size_t numBodies = 0;
        std::size_t numActiveBodies = 0;
        std::size_t numContacts = 0;
        std::size_t numManifolds = 0;
        std::size_t numConstraints = 0;
        std::size_t numIslands = 0;

        // Memory
        std::size_t memoryUsage = 0;
        std::size_t peakMemoryUsage = 0;

        // Broadphase
        std::size_t broadphasePairs = 0;
        std::size_t broadphaseProxies = 0;

        Float kineticEnergy = 0.0f;
        Float averageVelocity = 0.0f;

        void reset() {
            *this = PhysicsStats();
        }
    };
} // namespace engine::physics
