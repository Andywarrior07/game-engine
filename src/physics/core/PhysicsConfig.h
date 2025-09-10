/**
 * @file PhysicsConfig.h
 * @brief Physics system configuration and initialization parameters
 * @details Provides configuration structures for physics world initialization,
 *          performance tuning, and runtime settings
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

#include "PhysicsTypes.h"
#include "../../math/MathSystem.h"

#include <string>


namespace engine::physics {
    using namespace engine::math;
    // ============================================================================
    // Physics World Configuration
    // ============================================================================

    /**
     * @brief Broadphase algorithm selection
     */
    enum class BroadphaseType {
        DYNAMIC_AABB_TREE, // btDbvtBroadphase (default, good general purpose)
        AXIS_SWEEP_3, // bt32BitAxisSweep3 (good for many objects)
        SIMPLE, // btSimpleBroadphase (testing only)
        CUDA // GPU accelerated (requires CUDA)
    };

    /**
     * @brief Constraint solver type
     */
    enum class SolverType {
        SEQUENTIAL_IMPULSE, // Default, stable
        MLCP_DANTZIG, // Mixed Linear Complementarity Problem
        MLCP_PROJECTED_GAUSS_SEIDEL,
        NNCG, // Non-smooth Nonlinear Conjugate Gradient
        MULTIBODY // For articulated bodies
    };

    /**
     * @brief Main physics configuration
     */
    struct PhysicsConfig {
        // World settings
        Vec3 gravity = Vec3(0, world::GRAVITY_EARTH, 0);
        Vec3 worldMin = Vec3(-10000, -10000, -10000); // Broadphase bounds
        Vec3 worldMax = Vec3(10000, 10000, 10000);

        // Simulation parameters
        Float fixedTimeStep = world::FIXED_TIMESTEP_30HZ;
        Int maxSubSteps = simulation::MAX_SUBSTEPS;
        Float internalTimeStep = fixedTimeStep / static_cast<Float>(maxSubSteps);

        // Solver settings
        SolverType solverType = SolverType::SEQUENTIAL_IMPULSE;
        Int solverIterations = 10;
        Float solverDamping = 1.0f;
        Float erp = 0.2f; // Error reduction parameter
        Float erp2 = 0.8f; // Error reduction for split impulse
        Float globalCFM = 0.0f; // Constraint force mixing

        // Broadphase settings
        BroadphaseType broadphaseType = BroadphaseType::DYNAMIC_AABB_TREE;
        Int maxProxies = 65536;
        Float aabbExpansion = 0.1f; // AABB margin for CCD

        // Collision settings
        Float defaultContactBreakingThreshold = 0.02f;
        Float defaultContactProcessingThreshold = 0.0f;
        Int maxPersistentManifoldPoolSize = 4096;
        Int maxCollisionAlgorithmPoolSize = 4096;

        // CCD settings
        bool enableCCD = true;
        Float ccdMotionThreshold = collision::CCD_MOTION_THRESHOLD;
        Float ccdSweptSphereRadius = collision::CCD_SWEPT_SPHERE_RADIUS;

        // Sleep settings
        bool enableSleeping = true;
        Float linearSleepingThreshold = 0.8f;
        Float angularSleepingThreshold = 1.0f;
        Float sleepTimeThreshold = 0.5f;

        // Performance settings
        bool enableMultithreading = true;
        Int numThreads = 4;
        bool enableGPU = false;
        bool warmStarting = true;
        bool splitImpulse = false;
        bool enableFriction = true;

        // Memory settings
        std::size_t initialBodyPoolSize = 1024;
        std::size_t initialShapePoolSize = 2048;
        std::size_t initialConstraintPoolSize = 512;
        std::size_t initialManifoldPoolSize = 4096;

        // Debug settings
        bool enableDebugDraw = false;
        std::uint32_t debugDrawMode = 0;
        bool enableProfiling = false;
        bool enableStatistics = false;

        PhysicsConfig() = default;

        /**
         * @brief Create optimized config for different game types
         */
        static PhysicsConfig ForPlatformer() {
            PhysicsConfig config;
            config.gravity = Vec3(0, -20.0f, 0); // Stronger gravity for responsive jumping
            config.solverIterations = 8;
            config.enableCCD = true;
            config.fixedTimeStep = 1.0f / 120.0f; // Higher frequency for precision
            return config;
        }

        static PhysicsConfig ForRPG() {
            PhysicsConfig config;
            config.gravity = Vec3(0, world::GRAVITY_EARTH, 0);
            config.solverIterations = 10;
            config.enableMultithreading = true;
            config.worldMin = Vec3(-5000, -500, -5000); // Smaller vertical range
            config.worldMax = Vec3(5000, 500, 5000);
            return config;
        }

        static PhysicsConfig ForMMORPG() {
            PhysicsConfig config;
            config.broadphaseType = BroadphaseType::AXIS_SWEEP_3; // Better for many objects
            config.maxProxies = 131072; // Support more objects
            config.enableMultithreading = true;
            config.numThreads = 8;
            config.initialBodyPoolSize = 8192;
            config.worldMin = Vec3(-50000, -1000, -50000); // Large world
            config.worldMax = Vec3(50000, 1000, 50000);
            return config;
        }

        static PhysicsConfig ForSoulsLike() {
            PhysicsConfig config;
            config.gravity = Vec3(0, world::GRAVITY_EARTH * 1.2f, 0); // Slightly heavier
            config.solverIterations = 12; // More precision for combat
            config.enableCCD = true;
            config.ccdMotionThreshold = 0.5f; // More aggressive CCD
            config.splitImpulse = true; // Better collision response
            return config;
        }

        static PhysicsConfig ForVehicleGame() {
            PhysicsConfig config;
            config.fixedTimeStep = 1.0f / 120.0f; // High frequency for vehicles
            config.solverIterations = 15; // More iterations for stability
            config.enableFriction = true;
            config.splitImpulse = true;
            config.warmStarting = true;
            return config;
        }
    };

    // ============================================================================
    // Shape Creation Parameters
    // ============================================================================

    /**
     * @brief Parameters for creating collision shapes
     */
    struct ShapeCreationParams {
        ShapeType type = ShapeType::BOX;

        // Shape properties
        Float margin = 0.04f; // Collision margin
        bool isConvex = true;
        bool optimizeConvexHull = true;
        Int maxConvexHullVertices = 256;

        // ============================================================================
        // Shape-specific parameters (union-like access)
        // ============================================================================

        struct BoxParams {
            Vec3 halfExtents = Vec3(0.5f);
        } box;

        struct SphereParams {
            Float radius = 0.5f;
        } sphere;

        struct CapsuleParams {
            Float radius = 0.5f;
            Float height = 1.0f;
        } capsule;

        struct CylinderParams {
            Vec3 halfExtents = Vec3(0.5f, 1.0f, 0.5f); // X=radius, Y=half height, Z=radius
        } cylinder;

        struct ConeParams {
            Float radius = 0.5f;
            Float height = 1.0f;
        } cone;

        struct ConvexHullParams {
            const Float* vertices = nullptr;
            std::size_t vertexCount = 0;
            std::size_t vertexStride = sizeof(Vec3);
        } convexHull;

        struct TriangleMeshParams {
            const Float* vertices = nullptr;
            std::size_t vertexCount = 0;
            std::size_t vertexStride = sizeof(Vec3);
            const std::uint32_t* indices = nullptr;
            std::size_t indexCount = 0;
        } triangleMesh;

        struct HeightfieldParams {
            const Float* heightData = nullptr;
            Int width = 0;
            Int height = 0;
            Float minHeight = 0.0f;
            Float maxHeight = 1.0f;
            Vec3 scale = Vec3(1.0f);
        } heightfield;

        struct PlaneParams {
            Vec3 normal = Vec3(0, 1, 0);
            Float distance = 0.0f;
        } plane;

        struct CompoundParams {
            std::vector<ShapeCreationParams> childShapes;
            std::vector<Transform> childTransforms;
        } compound;

        // ============================================================================
        // Factory methods for convenient creation
        // ============================================================================

        ShapeCreationParams() = default;

        static ShapeCreationParams Box(const Vec3& halfExtents) {
            ShapeCreationParams params;
            params.type = ShapeType::BOX;
            params.box.halfExtents = halfExtents;
            return params;
        }

        static ShapeCreationParams Sphere(const Float radius) {
            ShapeCreationParams params;
            params.type = ShapeType::SPHERE;
            params.sphere.radius = radius;
            return params;
        }

        static ShapeCreationParams Capsule(const Float radius, const Float height) {
            ShapeCreationParams params;
            params.type = ShapeType::CAPSULE;
            params.capsule.radius = radius;
            params.capsule.height = height;
            return params;
        }

        static ShapeCreationParams Cylinder(const Float radius, const Float height) {
            ShapeCreationParams params;
            params.type = ShapeType::CYLINDER;
            params.cylinder.halfExtents = Vec3(radius, height * 0.5f, radius);
            return params;
        }

        static ShapeCreationParams Cone(const Float radius, const Float height) {
            ShapeCreationParams params;
            params.type = ShapeType::CONE;
            params.cone.radius = radius;
            params.cone.height = height;
            return params;
        }

        static ShapeCreationParams ConvexHull(const Float* vertices, const std::size_t vertexCount,
                                              const std::size_t stride = sizeof(Vec3)) {
            ShapeCreationParams params;
            params.type = ShapeType::CONVEX_HULL;
            params.convexHull.vertices = vertices;
            params.convexHull.vertexCount = vertexCount;
            params.convexHull.vertexStride = stride;
            return params;
        }

        static ShapeCreationParams TriangleMesh(const Float* vertices, const std::size_t vertexCount,
                                                const std::uint32_t* indices, const std::size_t indexCount,
                                                const std::size_t stride = sizeof(Vec3)) {
            ShapeCreationParams params;
            params.type = ShapeType::TRIANGLE_MESH;
            params.triangleMesh.vertices = vertices;
            params.triangleMesh.vertexCount = vertexCount;
            params.triangleMesh.indices = indices;
            params.triangleMesh.indexCount = indexCount;
            params.triangleMesh.vertexStride = stride;
            return params;
        }

        static ShapeCreationParams Heightfield(const Float* heightData, const Int width, const Int height,
                                               const Float minHeight, const Float maxHeight,
                                               const Vec3& scale = Vec3(1.0f)) {
            ShapeCreationParams params;
            params.type = ShapeType::HEIGHTFIELD;
            params.heightfield.heightData = heightData;
            params.heightfield.width = width;
            params.heightfield.height = height;
            params.heightfield.minHeight = minHeight;
            params.heightfield.maxHeight = maxHeight;
            params.heightfield.scale = scale;
            return params;
        }

        static ShapeCreationParams Plane(const Vec3& normal = Vec3(0, 1, 0), const Float distance = 0.0f) {
            ShapeCreationParams params;
            params.type = ShapeType::PLANE;
            params.plane.normal = normal;
            params.plane.distance = distance;
            return params;
        }

        static ShapeCreationParams Compound() {
            ShapeCreationParams params;
            params.type = ShapeType::COMPOUND;
            return params;
        }
    };

    // ============================================================================
    // Body Creation Parameters
    // ============================================================================

    /**
     * @brief Parameters for creating rigid bodies
     */
    struct BodyCreationParams {
        // Basic properties
        BodyType type = BodyType::DYNAMIC;
        Transform* transform = nullptr;
        ShapeCreationParams shape;
        PhysicsMaterial material;

        // Mass properties
        Float mass = 1.0f;
        Vec3 localInertia = VEC3_ZERO; // Auto-calculated if zero
        Vec3 centerOfMass = VEC3_ZERO;

        // Initial velocities
        Vec3 linearVelocity = VEC3_ZERO;
        Vec3 angularVelocity = VEC3_ZERO;

        // Collision filtering
        std::uint16_t collisionGroup = DEFAULT;
        std::uint16_t collisionMask = ALL;

        // Constraints
        Vec3 linearFactor = VEC3_ONE; // Constraint linear motion (0 = locked)
        Vec3 angularFactor = VEC3_ONE; // Constraint angular motion (0 = locked)

        // CCD settings
        bool enableCCD = false;
        Float ccdMotionThreshold = collision::CCD_MOTION_THRESHOLD;
        Float ccdSweptSphereRadius = collision::CCD_SWEPT_SPHERE_RADIUS;

        // Activation
        ActivationState activationState = ActivationState::ACTIVE_STATE;
        bool startAsleep = false;

        // ============================================================================
        // LOD (Level of Detail) System - NEW ADDITION
        // ============================================================================

        /**
         * @brief Importance factor for LOD system (0.0 = least important, 10.0+ = most important)
         * @details Controls physics simulation quality based on distance and performance:
         *
         * Importance Levels:
         * - 0.0-0.5: Decorative objects (get culled/simplified first)
         * - 0.5-1.0: Low priority props (basic LOD behavior)
         * - 1.0-3.0: Normal gameplay objects (standard simulation)
         * - 3.0-5.0: Important NPCs/objects (higher quality simulation)
         * - 5.0-10.0: Critical objects (player, bosses, key mechanics)
         * - 10.0+: Never LOD'd (always full simulation)
         *
         * Usage Examples:
         * - Static decorative rocks/trees: 0.2f
         * - Furniture and props: 0.8f
         * - Loot items: 1.5f
         * - Enemy NPCs: 3.0f
         * - Important quest objects: 5.0f
         * - Player character: 10.0f
         * - Boss enemies: 15.0f
         *
         * The LOD system uses this value combined with distance from viewer
         * to determine simulation quality automatically.
         */
        Float importance = 1.0f;

        // Callbacks
        CollisionEnterCallback onCollisionEnter = nullptr;
        CollisionExitCallback onCollisionExit = nullptr;
        TriggerEnterCallback onTriggerEnter = nullptr;
        TriggerExitCallback onTriggerExit = nullptr;

        // User data
        void* userData = nullptr;
        std::string name;
        std::uint32_t id = 0;

        BodyCreationParams() = default;

        // ============================================================================
        // Factory methods with importance settings
        // ============================================================================

        static BodyCreationParams StaticBody(const ShapeCreationParams& shape,
                                             Transform* transform = nullptr,
                                             const Float importance = 0.5f) {
            BodyCreationParams params;
            params.type = BodyType::STATIC;
            params.shape = shape;
            params.transform = transform;
            params.mass = 0.0f;
            params.collisionGroup = STATIC;
            params.importance = importance; // Static objects usually low priority
            return params;
        }

        static BodyCreationParams DynamicBody(const ShapeCreationParams& shape,
                                              const Float mass = 1.0f,
                                              Transform* transform = nullptr,
                                              const Float importance = 1.0f) {
            BodyCreationParams params;
            params.type = BodyType::DYNAMIC;
            params.shape = shape;
            params.mass = mass;
            params.transform = transform;
            params.collisionGroup = DEFAULT;
            params.importance = importance; // Dynamic objects normal priority
            return params;
        }

        static BodyCreationParams KinematicBody(const ShapeCreationParams& shape,
                                                Transform* transform = nullptr,
                                                const Float importance = 2.0f) {
            BodyCreationParams params;
            params.type = BodyType::KINEMATIC;
            params.shape = shape;
            params.transform = transform;
            params.mass = 0.0f;
            params.collisionGroup = KINEMATIC;
            params.activationState = ActivationState::DISABLE_DEACTIVATION_STATE;
            params.importance = importance; // Kinematic objects higher priority
            return params;
        }

        static BodyCreationParams GhostBody(const ShapeCreationParams& shape,
                                            Transform* transform = nullptr,
                                            const Float importance = 0.8f) {
            BodyCreationParams params;
            params.type = BodyType::GHOST;
            params.shape = shape;
            params.transform = transform;
            params.mass = 0.0f;
            params.collisionGroup = SENSOR;
            params.collisionMask = ALL;
            params.importance = importance; // Ghost objects low-medium priority
            return params;
        }

        // ============================================================================
        // Game-specific factory methods
        // ============================================================================

        static BodyCreationParams PlayerBody(const ShapeCreationParams& shape,
                                             const Float mass = 70.0f,
                                             Transform* transform = nullptr) {
            BodyCreationParams params = DynamicBody(shape, mass, transform);
            params.importance = 10.0f; // Maximum priority - never gets LOD'd
            params.name = "Player";
            params.enableCCD = true; // Players need precise collision
            return params;
        }

        static BodyCreationParams EnemyBody(const ShapeCreationParams& shape,
                                            const Float mass = 80.0f,
                                            Transform* transform = nullptr,
                                            const std::string& name = "Enemy") {
            BodyCreationParams params = DynamicBody(shape, mass, transform);
            params.importance = 3.0f; // High priority for gameplay
            params.name = name;
            params.enableCCD = true; // Enemies need good collision
            return params;
        }

        static BodyCreationParams BossBody(const ShapeCreationParams& shape,
                                           const Float mass = 200.0f,
                                           Transform* transform = nullptr,
                                           const std::string& name = "Boss") {
            BodyCreationParams params = DynamicBody(shape, mass, transform);
            params.importance = 15.0f; // Maximum priority
            params.name = name;
            params.enableCCD = true;
            return params;
        }

        static BodyCreationParams PropBody(const ShapeCreationParams& shape,
                                           const Float mass = 5.0f,
                                           Transform* transform = nullptr,
                                           const std::string& name = "Prop") {
            BodyCreationParams params = DynamicBody(shape, mass, transform);
            params.importance = 0.8f; // Low priority for optimization
            params.name = name;
            return params;
        }

        static BodyCreationParams LootBody(const ShapeCreationParams& shape,
                                           Transform* transform = nullptr,
                                           const std::string& name = "Loot") {
            BodyCreationParams params = DynamicBody(shape, 0.1f, transform);
            params.importance = 1.5f; // Medium priority - important for gameplay
            params.name = name;
            return params;
        }

        static BodyCreationParams TerrainBody(const ShapeCreationParams& shape,
                                              Transform* transform = nullptr,
                                              const std::string& name = "Terrain") {
            BodyCreationParams params = StaticBody(shape, transform, 0.2f);
            params.name = name;
            params.importance = 0.2f; // Very low priority for terrain
            return params;
        }
    };

    // ============================================================================
    // Constraint Creation Parameters
    // ============================================================================

    /**
     * @brief Parameters for creating constraints/joints
     */
    struct ConstraintCreationParams {
        ConstraintType type = ConstraintType::GENERIC_6DOF;
        RigidBody* bodyA = nullptr;
        RigidBody* bodyB = nullptr; // Can be null for world constraint

        // Frames (local to each body)
        Transform frameInA;
        Transform frameInB;

        // Limits (interpretation depends on constraint type)
        Vec3 linearLowerLimit = Vec3(-INFINITY_VALUE<Float>);
        Vec3 linearUpperLimit = Vec3(INFINITY_VALUE<Float>);
        Vec3 angularLowerLimit = Vec3(-PI<Float>);
        Vec3 angularUpperLimit = Vec3(PI<Float>);

        // Spring settings (for spring constraints)
        Vec3 linearStiffness = Vec3(0);
        Vec3 linearDamping = Vec3(0);
        Vec3 angularStiffness = Vec3(0);
        Vec3 angularDamping = Vec3(0);

        // Motor settings
        Vec3 motorTargetVelocity = Vec3(0);
        Vec3 motorMaxForce = Vec3(0);
        bool enableMotor[6] = {false};

        // Solver settings
        Float breakingImpulse = INFINITY_VALUE<Float>;
        bool disableCollisionsBetweenLinkedBodies = false;
        Int overrideNumSolverIterations = -1;

        ConstraintCreationParams() = default;

        static ConstraintCreationParams FixedConstraint(RigidBody* bodyA, RigidBody* bodyB,
                                                        const Transform& frameInA,
                                                        const Transform& frameInB) {
            ConstraintCreationParams params;
            params.type = ConstraintType::FIXED;
            params.bodyA = bodyA;
            params.bodyB = bodyB;
            params.frameInA = frameInA;
            params.frameInB = frameInB;
            params.linearLowerLimit = Vec3(0);
            params.linearUpperLimit = Vec3(0);
            params.angularLowerLimit = Vec3(0);
            params.angularUpperLimit = Vec3(0);
            return params;
        }

        static ConstraintCreationParams HingeConstraint(RigidBody* bodyA, RigidBody* bodyB,
                                                        const Vec3& pivotInA, const Vec3& pivotInB,
                                                        const Vec3& axisInA, const Vec3& axisInB,
                                                        const Float minAngle = -PI<Float>,
                                                        const Float maxAngle = PI<Float>) {
            ConstraintCreationParams params;
            params.type = ConstraintType::HINGE;
            params.bodyA = bodyA;
            params.bodyB = bodyB;
            params.frameInA.setPosition(pivotInA);
            params.frameInB.setPosition(pivotInB);
            params.angularLowerLimit = Vec3(minAngle, 0, 0);
            params.angularUpperLimit = Vec3(maxAngle, 0, 0);
            return params;
        }
    };
} // namespace engine::physics
