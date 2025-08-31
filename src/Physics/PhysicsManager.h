// //
// // Created by Andres Guerrero on 11-08-25.
// //
//
// #pragma once
//
// // ========================================================================
// // PHYSICS MANAGER HEADER - PART 1: CORE TYPES AND CONFIGURATION
// // ========================================================================
//
// #include <btBulletDynamicsCommon.h>        // Bullet Physics core
// #include <BulletCollision/CollisionDispatch/btGhostObject.h>  // For triggers
// #include <BulletDynamics/Character/btKinematicCharacterController.h>  // Character control
// #include <BulletSoftBody/btSoftBody.h>      // Soft body dynamics
// #include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
// #include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
// #include <BulletSoftBody/btDefaultSoftBodySolver.h>
// #include <BulletSoftBody/btSoftBodyHelpers.h>
// #include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
//
// #include <functional>                       // For callbacks
// #include <unordered_map>                   // For resource storage
// #include <vector>                          // For dynamic arrays
// #include <string>                          // For names
// #include <memory>                          // For smart pointers
// #include <atomic>                          // For thread-safe counters
// #include <mutex>                           // For thread synchronization
// #include <optional>                        // For optional returns
// #include <queue>                           // For event queues
// #include <chrono>                          // For timing
// #include "../Math-old/MathTypes.h"             // Your GLM-based math types
//
// namespace engine::physics {
//     using namespace engine::math;
//
//     // Forward declarations
//     class PhysicsManager;
//     class RigidBody;
//     class CollisionShape;
//     class PhysicsConstraint;
//     class PhysicsWorld;
//     class CollisionListener;
//     class RaycastResult;
//     class PhysicsDebugDrawer;
//
//     // ========================================================================
//     // TYPE DEFINITIONS AND CONSTANTS
//     // ========================================================================
//
//     // Strong typing for physics identifiers
//     using RigidBodyID = std::uint32_t;
//     using CollisionShapeID = std::uint32_t;
//     using ConstraintID = std::uint32_t;
//     using CollisionPairID = std::uint64_t;
//
//     // Invalid ID constants
//     constexpr RigidBodyID INVALID_BODY_ID = 0;
//     constexpr CollisionShapeID INVALID_SHAPE_ID = 0;
//     constexpr ConstraintID INVALID_CONSTRAINT_ID = 0;
//
//     // Physics constants
//     namespace constants {
//         constexpr float DEFAULT_GRAVITY = -9.81f; // Standard Earth gravity
//         constexpr float DEFAULT_TIMESTEP = 1.0f / 60.0f; // 60 Hz physics
//         constexpr int MAX_SUBSTEPS = 10; // Maximum simulation substeps
//         constexpr float DEFAULT_RESTITUTION = 0.5f; // Bounciness
//         constexpr float DEFAULT_FRICTION = 0.5f; // Surface friction
//         constexpr float DEFAULT_ROLLING_FRICTION = 0.1f; // Rolling resistance
//         constexpr float DEFAULT_LINEAR_DAMPING = 0.1f; // Air resistance
//         constexpr float DEFAULT_ANGULAR_DAMPING = 0.1f; // Rotational damping
//         constexpr float SLEEP_THRESHOLD = 0.8f; // When to put bodies to sleep
//         constexpr float CCD_MOTION_THRESHOLD = 0.5f; // Continuous collision detection threshold
//         constexpr float CCD_SWEPT_SPHERE_RADIUS = 0.2f; // CCD sphere radius
//         constexpr float CONTACT_BREAKING_THRESHOLD = 0.02f; // Distance to break contacts
//     }
//
//     // ========================================================================
//     // COLLISION LAYERS AND FILTERING
//     // ========================================================================
//
//     /**
//      * @brief Collision layers for filtering interactions
//      * Uses bit flags for efficient masking
//      */
//     enum class CollisionLayer : std::uint16_t {
//         NONE = 0,
//         DEFAULT = 1 << 0, // Default collision layer
//         STATIC = 1 << 1, // Static world geometry
//         DYNAMIC = 1 << 2, // Dynamic physics objects
//         PLAYER = 1 << 3, // Player entities
//         ENEMY = 1 << 4, // Enemy entities
//         PROJECTILE = 1 << 5, // Projectiles and bullets
//         TRIGGER = 1 << 6, // Trigger volumes
//         DEBRIS = 1 << 7, // Small debris and particles
//         VEHICLE = 1 << 8, // Vehicles
//         RAGDOLL = 1 << 9, // Ragdoll bodies
//         WATER = 1 << 10, // Water volumes
//         SENSOR = 1 << 11, // Sensor objects (no collision response)
//         UI = 1 << 12, // UI physics elements
//         CUSTOM_1 = 1 << 13, // User-defined layer 1
//         CUSTOM_2 = 1 << 14, // User-defined layer 2
//         ALL = 0xFFFF // Collide with everything
//     };
//
//     // Bitwise operations for collision layers
//     inline CollisionLayer operator|(CollisionLayer a, CollisionLayer b) {
//         return static_cast<CollisionLayer>(static_cast<std::uint16_t>(a) | static_cast<std::uint16_t>(b));
//     }
//
//     inline CollisionLayer operator&(CollisionLayer a, CollisionLayer b) {
//         return static_cast<CollisionLayer>(static_cast<std::uint16_t>(a) & static_cast<std::uint16_t>(b));
//     }
//
//     inline CollisionLayer operator~(CollisionLayer a) {
//         return static_cast<CollisionLayer>(~static_cast<std::uint16_t>(a));
//     }
//
//     /**
//      * @brief Collision filter data for precise collision control
//      */
//     struct CollisionFilter {
//         CollisionLayer layer = CollisionLayer::DEFAULT; // This object's layer
//         CollisionLayer mask = CollisionLayer::ALL; // Layers this object collides with
//
//         bool canCollideWith(const CollisionFilter& other) const {
//             return (static_cast<std::uint16_t>(layer) & static_cast<std::uint16_t>(other.mask)) &&
//                 (static_cast<std::uint16_t>(other.layer) & static_cast<std::uint16_t>(mask));
//         }
//     };
//
//     // ========================================================================
//     // PHYSICS BODY TYPES AND STATES
//     // ========================================================================
//
//     /**
//      * @brief Types of rigid bodies
//      */
//     enum class BodyType : std::uint8_t {
//         STATIC = 0, // Immovable (terrain, buildings)
//         KINEMATIC = 1, // Controlled by animation/code
//         DYNAMIC = 2, // Full physics simulation
//         SOFT = 3 // Soft body (cloth, rope, jelly)
//     };
//
//     /**
//      * @brief Activation states for bodies
//      */
//     enum class ActivationState : std::uint8_t {
//         ACTIVE_STATE = 1, // Fully active
//         SLEEPING_STATE = 2, // Sleeping (optimization)
//         WANTS_DEACTIVATION_STATE = 3, // Will sleep soon
//         DISABLE_DEACTIVATION_STATE = 4, // Never sleep
//         DISABLE_SIMULATION_STATE = 5 // Completely disabled
//     };
//
//     /**
//      * @brief Collision shape types
//      */
//     enum class ShapeType : std::uint8_t {
//         BOX = 0, // Box/cube shape
//         SPHERE = 1, // Sphere shape
//         CAPSULE = 2, // Capsule shape
//         CYLINDER = 3, // Cylinder shape
//         CONE = 4, // Cone shape
//         CONVEX_HULL = 5, // Convex mesh
//         TRIANGLE_MESH = 6, // Concave triangle mesh (static only)
//         COMPOUND = 7, // Multiple shapes combined
//         HEIGHTFIELD = 8, // Terrain heightfield
//         PLANE = 9 // Infinite plane
//     };
//
//     // ========================================================================
//     // COLLISION EVENTS AND CALLBACKS
//     // ========================================================================
//
//     /**
//      * @brief Collision event types
//      */
//     enum class CollisionEventType : std::uint8_t {
//         CONTACT_STARTED = 0, // Collision began
//         CONTACT_ENDED = 1, // Collision ended
//         CONTACT_PERSISTED = 2, // Collision continuing
//         TRIGGER_ENTER = 3, // Entered trigger volume
//         TRIGGER_EXIT = 4, // Exited trigger volume
//         TRIGGER_STAY = 5 // Still in trigger volume
//     };
//
//     /**
//      * @brief Contact point information
//      */
//     struct ContactPoint {
//         Vector3 worldPositionOnA; // Contact position on body A
//         Vector3 worldPositionOnB; // Contact position on body B
//         Vector3 worldNormalOnB; // Contact normal (from B to A)
//         float penetrationDepth = 0.0f; // Penetration distance
//         float appliedImpulse = 0.0f; // Impulse applied
//         float combinedFriction = 0.0f; // Combined friction coefficient
//         float combinedRestitution = 0.0f; // Combined restitution
//         int lifetime = 0; // Frames this contact existed
//     };
//
//     /**
//      * @brief Collision manifold containing all contact points
//      */
//     struct CollisionManifold {
//         RigidBodyID bodyA = INVALID_BODY_ID; // First body
//         RigidBodyID bodyB = INVALID_BODY_ID; // Second body
//         std::vector<ContactPoint> contactPoints; // All contact points
//         CollisionEventType eventType; // Type of collision event
//
//         bool isValid() const {
//             return bodyA != INVALID_BODY_ID && bodyB != INVALID_BODY_ID;
//         }
//
//         CollisionPairID getPairID() const {
//             // Create unique ID for collision pair (order-independent)
//             std::uint32_t minID = std::min(bodyA, bodyB);
//             std::uint32_t maxID = std::max(bodyA, bodyB);
//             return (static_cast<CollisionPairID>(minID) << 32) | maxID;
//         }
//     };
//
//     /**
//      * @brief Callback function types for physics events
//      */
//     using CollisionCallback = std::function<void(const CollisionManifold&)>;
//     using TriggerCallback = std::function<void(RigidBodyID trigger, RigidBodyID other, CollisionEventType type)>;
//     using PreStepCallback = std::function<void(float timeStep)>;
//     using PostStepCallback = std::function<void(float timeStep)>;
//
//     // ========================================================================
//     // PHYSICS MATERIAL PROPERTIES
//     // ========================================================================
//
//     /**
//      * @brief Physical material properties
//      */
//     struct PhysicsMaterial {
//         float friction = constants::DEFAULT_FRICTION; // Static friction
//         float rollingFriction = constants::DEFAULT_ROLLING_FRICTION; // Rolling friction
//         float restitution = constants::DEFAULT_RESTITUTION; // Bounciness (0-1)
//         float density = 1.0f; // Material density kg/m³
//         float linearDamping = constants::DEFAULT_LINEAR_DAMPING; // Linear motion damping
//         float angularDamping = constants::DEFAULT_ANGULAR_DAMPING; // Angular motion damping
//
//         // Preset materials
//         static PhysicsMaterial Rubber() { return {0.9f, 0.2f, 0.8f, 1.5f, 0.1f, 0.1f}; }
//         static PhysicsMaterial Ice() { return {0.05f, 0.01f, 0.1f, 0.9f, 0.05f, 0.05f}; }
//         static PhysicsMaterial Metal() { return {0.7f, 0.1f, 0.3f, 7.8f, 0.05f, 0.05f}; }
//         static PhysicsMaterial Wood() { return {0.6f, 0.3f, 0.4f, 0.7f, 0.1f, 0.1f}; }
//         static PhysicsMaterial Stone() { return {0.8f, 0.4f, 0.2f, 2.5f, 0.01f, 0.01f}; }
//         static PhysicsMaterial Bouncy() { return {0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f}; }
//     };
//
//     // ========================================================================
//     // RIGID BODY CONFIGURATION
//     // ========================================================================
//
//     /**
//      * @brief Configuration for creating rigid bodies
//      */
//     struct RigidBodyConfig {
//         BodyType type = BodyType::DYNAMIC; // Body type
//         Transform3D transform; // Initial transform
//         PhysicsMaterial material; // Physical properties
//         CollisionFilter filter; // Collision filtering
//         float mass = 1.0f; // Mass (0 for static)
//         bool isKinematic = false; // Kinematic control
//         bool isTrigger = false; // Trigger volume (no collision response)
//         bool enableCCD = false; // Continuous collision detection
//         bool allowSleeping = true; // Can be deactivated for optimization
//         Vector3 localInertia{0.0f, 0.0f, 0.0f}; // Custom inertia tensor
//         ActivationState initialState = ActivationState::ACTIVE_STATE;
//
//         // Constraints
//         Vector3 linearFactor{1.0f, 1.0f, 1.0f}; // Linear motion constraints (0 = locked)
//         Vector3 angularFactor{1.0f, 1.0f, 1.0f}; // Angular motion constraints
//
//         // Limits
//         float maxLinearVelocity = 100.0f; // Maximum linear velocity
//         float maxAngularVelocity = 100.0f; // Maximum angular velocity
//     };
//
//     // ========================================================================
//     // RAYCAST AND QUERY STRUCTURES
//     // ========================================================================
//
//     /**
//      * @brief Ray for raycasting
//      */
//     struct Ray {
//         Vector3 origin; // Ray start position
//         Vector3 direction; // Ray direction (should be normalized)
//         float maxDistance = 1000.0f; // Maximum ray distance
//         CollisionFilter filter; // Collision filtering
//         bool stopAtFirstHit = false; // Stop at first hit or get all
//
//         Vector3 getPoint(float distance) const {
//             return origin + direction * distance;
//         }
//     };
//
//     /**
//      * @brief Single raycast hit result
//      */
//     struct RaycastHit {
//         RigidBodyID bodyId = INVALID_BODY_ID; // Hit body ID
//         Vector3 point; // World hit point
//         Vector3 normal; // Surface normal at hit
//         float distance = 0.0f; // Distance from ray origin
//         float fraction = 0.0f; // Fraction along ray (0-1)
//         int triangleIndex = -1; // For mesh collisions
//         void* userData = nullptr; // User data from hit body
//
//         bool isValid() const { return bodyId != INVALID_BODY_ID; }
//     };
//
//     /**
//      * @brief Complete raycast result with all hits
//      */
//     struct RaycastResult {
//         std::vector<RaycastHit> hits; // All hits (sorted by distance)
//         bool hasHit = false; // Quick check for any hits
//
//         const RaycastHit& getClosestHit() const {
//             static RaycastHit invalid;
//             return hasHit && !hits.empty() ? hits[0] : invalid;
//         }
//     };
//
//     /**
//      * @brief Sweep test configuration
//      */
//     struct SweepTest {
//         CollisionShapeID shapeId = INVALID_SHAPE_ID; // Shape to sweep
//         Transform3D startTransform; // Start position/rotation
//         Transform3D endTransform; // End position/rotation
//         CollisionFilter filter; // Collision filtering
//         bool stopAtFirstHit = true; // Stop at first hit
//     };
//
//     /**
//      * @brief Overlap test for volume queries
//      */
//     struct OverlapTest {
//         CollisionShapeID shapeId = INVALID_SHAPE_ID; // Shape to test
//         Transform3D transform; // Shape transform
//         CollisionFilter filter; // Collision filtering
//     };
//
//     // ========================================================================
//     // PHYSICS WORLD CONFIGURATION
//     // ========================================================================
//
//     /**
//      * @brief Physics simulation configuration
//      */
//     struct PhysicsConfig {
//         // Simulation parameters
//         Vector3 gravity{0.0f, constants::DEFAULT_GRAVITY, 0.0f};
//         float fixedTimeStep = constants::DEFAULT_TIMESTEP;
//         int maxSubSteps = constants::MAX_SUBSTEPS;
//         float worldScale = 1.0f; // Bullet works best at 0.05-10 meter scale
//
//         // Collision configuration
//         float contactBreakingThreshold = constants::CONTACT_BREAKING_THRESHOLD;
//         float collisionMargin = 0.04f; // Collision shape margin
//         int solverIterations = 10; // Constraint solver iterations
//         bool enableSoftBody = false; // Enable soft body dynamics
//         bool enableContinuousCollision = true; // CCD for fast objects
//
//         // Optimization
//         bool enableSleeping = true; // Deactivate stationary bodies
//         float sleepLinearThreshold = constants::SLEEP_THRESHOLD;
//         float sleepAngularThreshold = constants::SLEEP_THRESHOLD;
//         float sleepTimeThreshold = 1.0f; // Time before sleeping
//
//         // Broadphase settings
//         Vector3 worldMin{-1000.0f, -1000.0f, -1000.0f};
//         Vector3 worldMax{1000.0f, 1000.0f, 1000.0f};
//         int maxProxies = 32766; // Maximum broadphase proxies
//
//         // Debug rendering
//         bool enableDebugDraw = false; // Enable debug visualization
//         int debugDrawMode = 0; // Debug draw flags
//
//         // Performance
//         bool multithreaded = false; // Use multithreading (experimental)
//         int numThreads = 4; // Number of physics threads
//         bool enableGpuAcceleration = false; // GPU physics (if available)
//
//         // Statistics and profiling
//         bool enableProfiling = false; // Track performance metrics
//         bool enableStatistics = false; // Collect collision statistics
//     };
//
//     // ========================================================================
//     // CONSTRAINT TYPES AND CONFIGURATION
//     // ========================================================================
//
//     /**
//      * @brief Types of physics constraints/joints
//      */
//     enum class ConstraintType : std::uint8_t {
//         POINT_TO_POINT = 0, // Ball socket joint
//         HINGE = 1, // Hinge/revolute joint
//         SLIDER = 2, // Prismatic/slider joint
//         CONE_TWIST = 3, // Cone twist (shoulder)
//         GENERIC_6DOF = 4, // 6 degrees of freedom
//         GENERIC_6DOF_SPRING = 5, // 6DOF with springs
//         FIXED = 6, // Fixed/weld joint
//         GEAR = 7 // Gear constraint
//     };
//
//     /**
//      * @brief Base constraint configuration
//      */
//     struct ConstraintConfig {
//         ConstraintType type = ConstraintType::POINT_TO_POINT;
//         RigidBodyID bodyA = INVALID_BODY_ID; // First body
//         RigidBodyID bodyB = INVALID_BODY_ID; // Second body (optional)
//         Transform3D frameInA; // Constraint frame in body A
//         Transform3D frameInB; // Constraint frame in body B
//         bool disableCollisionsBetweenLinkedBodies = true;
//         float breakingImpulseThreshold = INFINITY; // Force to break constraint
//         bool enableMotor = false; // Enable motor
//         float maxMotorImpulse = 1.0f; // Motor strength
//
//         // Constraint limits (depends on type)
//         Vector3 linearLowerLimit{0.0f, 0.0f, 0.0f};
//         Vector3 linearUpperLimit{0.0f, 0.0f, 0.0f};
//         Vector3 angularLowerLimit{0.0f, 0.0f, 0.0f};
//         Vector3 angularUpperLimit{0.0f, 0.0f, 0.0f};
//
//         // Spring/damper settings
//         Vector3 linearStiffness{0.0f, 0.0f, 0.0f};
//         Vector3 linearDamping{1.0f, 1.0f, 1.0f};
//         Vector3 angularStiffness{0.0f, 0.0f, 0.0f};
//         Vector3 angularDamping{1.0f, 1.0f, 1.0f};
//     };
//
//     // ========================================================================
//     // PHYSICS STATISTICS AND DEBUG INFO
//     // ========================================================================
//
//     /**
//      * @brief Physics simulation statistics
//      */
//     struct PhysicsStatistics {
//         // Performance metrics
//         float simulationTime = 0.0f; // Time spent in physics update
//         float collisionTime = 0.0f; // Time in collision detection
//         float solverTime = 0.0f; // Time in constraint solver
//         float broadphaseTime = 0.0f; // Time in broadphase
//         float narrowphaseTime = 0.0f; // Time in narrowphase
//
//         // Counts
//         int numActiveBodies = 0; // Active rigid bodies
//         int numSleepingBodies = 0; // Sleeping bodies
//         int numContacts = 0; // Active contacts
//         int numManifolds = 0; // Contact manifolds
//         int numConstraints = 0; // Active constraints
//         int numIslands = 0; // Simulation islands
//
//         // Collision statistics
//         int broadphasePairs = 0; // Broadphase overlapping pairs
//         int narrowphaseTests = 0; // Narrowphase collision tests
//         int numRaycasts = 0; // Raycasts performed
//         int numSweepTests = 0; // Sweep tests performed
//
//         // Memory usage
//         std::size_t totalMemory = 0; // Total memory usage
//         std::size_t shapeMemory = 0; // Memory for collision shapes
//         std::size_t bodyMemory = 0; // Memory for rigid bodies
//         std::size_t constraintMemory = 0; // Memory for constraints
//     };
//
//     // ========================================================================
//     // COLLISION SHAPE WRAPPER CLASS
//     // ========================================================================
//
//     /**
//      * @brief Wrapper for Bullet collision shapes with resource management
//      *
//      * Provides a high-level interface to Bullet's collision shapes while
//      * managing memory and providing conversion utilities for GLM math types.
//      */
//     class CollisionShape {
//     public:
//         /**
//          * @brief Create a box collision shape
//          * @param halfExtents Half dimensions of the box
//          * @return Shape ID for the created shape
//          */
//         static std::unique_ptr<CollisionShape> CreateBox(const Vector3& halfExtents);
//
//         /**
//          * @brief Create a sphere collision shape
//          * @param radius Sphere radius
//          * @return Shape ID for the created shape
//          */
//         static std::unique_ptr<CollisionShape> CreateSphere(float radius);
//
//         /**
//          * @brief Create a capsule collision shape
//          * @param radius Capsule radius
//          * @param height Capsule height (excluding caps)
//          * @param axis Capsule axis (0=X, 1=Y, 2=Z)
//          * @return Shape ID for the created shape
//          */
//         static std::unique_ptr<CollisionShape> CreateCapsule(float radius, float height, int axis = 1);
//
//         /**
//          * @brief Create a cylinder collision shape
//          * @param halfExtents Half dimensions (radius in X/Z, half-height in Y)
//          * @param axis Cylinder axis (0=X, 1=Y, 2=Z)
//          * @return Shape ID for the created shape
//          */
//         static std::unique_ptr<CollisionShape> CreateCylinder(const Vector3& halfExtents, int axis = 1);
//
//         /**
//          * @brief Create a cone collision shape
//          * @param radius Cone base radius
//          * @param height Cone height
//          * @param axis Cone axis (0=X, 1=Y, 2=Z)
//          * @return Shape ID for the created shape
//          */
//         static std::unique_ptr<CollisionShape> CreateCone(float radius, float height, int axis = 1);
//
//         /**
//          * @brief Create a convex hull from points
//          * @param points Array of points defining the convex hull
//          * @param optimized Optimize the hull for better performance
//          * @return Shape ID for the created shape
//          */
//         static std::unique_ptr<CollisionShape> CreateConvexHull(const std::vector<Vector3>& points,
//                                                                 bool optimized = true);
//
//         /**
//          * @brief Create a static triangle mesh (concave)
//          * @param vertices Triangle vertices
//          * @param indices Triangle indices (3 per triangle)
//          * @param buildBVH Build bounding volume hierarchy for optimization
//          * @return Shape ID for the created shape
//          */
//         static std::unique_ptr<CollisionShape> CreateTriangleMesh(
//             const std::vector<Vector3>& vertices,
//             const std::vector<int>& indices,
//             bool buildBVH = true
//         );
//
//         /**
//          * @brief Create a heightfield terrain shape
//          * @param width Width of heightfield
//          * @param height Height of heightfield
//          * @param heightData Height values
//          * @param minHeight Minimum height value
//          * @param maxHeight Maximum height value
//          * @param upAxis Up axis (0=X, 1=Y, 2=Z)
//          * @return Shape ID for the created shape
//          */
//         static std::unique_ptr<CollisionShape> CreateHeightfield(
//             int width, int height,
//             const std::vector<float>& heightData,
//             float minHeight, float maxHeight,
//             int upAxis = 1
//         );
//
//         /**
//          * @brief Create a compound shape from multiple child shapes
//          * @param children Child shapes with their local transforms
//          * @return Shape ID for the created shape
//          */
//         static std::unique_ptr<CollisionShape> CreateCompound(
//             const std::vector<std::pair<std::unique_ptr<CollisionShape>, Transform3D>>& children
//         );
//
//         /**
//          * @brief Create an infinite plane shape
//          * @param normal Plane normal
//          * @param offset Distance from origin
//          * @return Shape ID for the created shape
//          */
//         static std::unique_ptr<CollisionShape> CreatePlane(const Vector3& normal, float offset);
//
//         // Destructor
//         ~CollisionShape();
//
//         // Non-copyable but moveable
//         CollisionShape(const CollisionShape&) = delete;
//         CollisionShape& operator=(const CollisionShape&) = delete;
//         CollisionShape(CollisionShape&&) = default;
//         CollisionShape& operator=(CollisionShape&&) = default;
//
//         // === Accessors ===
//
//         CollisionShapeID getId() const noexcept { return id_; }
//         ShapeType getType() const noexcept { return type_; }
//         btCollisionShape* getBulletShape() const noexcept { return bulletShape_.get(); }
//
//         /**
//          * @brief Get the local scaling of the shape
//          * @return Current scale
//          */
//         Vector3 getLocalScaling() const;
//
//         /**
//          * @brief Set the local scaling of the shape
//          * @param scale New scale
//          */
//         void setLocalScaling(const Vector3& scale);
//
//         /**
//          * @brief Calculate volume of the shape
//          * @return Volume in cubic units
//          */
//         float calculateVolume() const;
//
//         /**
//          * @brief Calculate inertia tensor for given mass
//          * @param mass Object mass
//          * @return Local inertia tensor
//          */
//         Vector3 calculateLocalInertia(float mass) const;
//
//         /**
//          * @brief Get the axis-aligned bounding box
//          * @param transform Transform to apply
//          * @return AABB min and max points
//          */
//         std::pair<Vector3, Vector3> getAABB(const Transform3D& transform) const;
//
//         /**
//          * @brief Check if shape is convex
//          * @return true if convex shape
//          */
//         bool isConvex() const noexcept;
//
//         /**
//          * @brief Check if shape is compound
//          * @return true if compound shape
//          */
//         bool isCompound() const noexcept;
//
//         /**
//          * @brief Get memory usage
//          * @return Approximate memory usage in bytes
//          */
//         std::size_t getMemoryUsage() const;
//
//     private:
//         CollisionShape(CollisionShapeID id, ShapeType type, std::unique_ptr<btCollisionShape> shape);
//
//         CollisionShapeID id_; // Unique identifier
//         ShapeType type_; // Shape type
//         std::unique_ptr<btCollisionShape> bulletShape_; // Bullet shape
//         std::unique_ptr<btTriangleMesh> meshData_; // Mesh data (for triangle meshes)
//         std::unique_ptr<btTriangleIndexVertexArray> meshInterface_; // Mesh interface
//
//         static std::atomic<CollisionShapeID> nextShapeId_; // ID generator
//     };
//
//     // ========================================================================
//     // RIGID BODY WRAPPER CLASS
//     // ========================================================================
//
//     /**
//      * @brief High-level wrapper for Bullet rigid bodies
//      *
//      * Manages the lifecycle and properties of physics bodies while providing
//      * a clean interface between the engine and Bullet Physics.
//      */
//     class RigidBody {
//     public:
//         /**
//          * @brief Constructor with configuration
//          * @param id Unique identifier
//          * @param shape Collision shape to use
//          * @param config Body configuration
//          */
//         RigidBody(RigidBodyID id, CollisionShape* shape, const RigidBodyConfig& config);
//
//         /**
//          * @brief Destructor - removes body from world
//          */
//         ~RigidBody();
//
//         // Non-copyable but moveable
//         RigidBody(const RigidBody&) = delete;
//         RigidBody& operator=(const RigidBody&) = delete;
//         RigidBody(RigidBody&&) = default;
//         RigidBody& operator=(RigidBody&&) = default;
//
//         // === Accessors ===
//
//         RigidBodyID getId() const noexcept { return id_; }
//         BodyType getType() const noexcept { return type_; }
//         btRigidBody* getBulletBody() const noexcept { return bulletBody_.get(); }
//         btGhostObject* getGhostObject() const noexcept { return ghostObject_.get(); }
//         bool isTrigger() const noexcept { return isTrigger_; }
//
//         // === Transform Operations ===
//
//         /**
//          * @brief Get world transform
//          * @return Current world transform
//          */
//         Transform3D getTransform() const;
//
//         /**
//          * @brief Set world transform
//          * @param transform New world transform
//          */
//         void setTransform(const Transform3D& transform);
//
//         /**
//          * @brief Get world position
//          * @return Current position
//          */
//         Vector3 getPosition() const;
//
//         /**
//          * @brief Set world position
//          * @param position New position
//          */
//         void setPosition(const Vector3& position);
//
//         /**
//          * @brief Get world rotation
//          * @return Current rotation quaternion
//          */
//         Quaternion getRotation() const;
//
//         /**
//          * @brief Set world rotation
//          * @param rotation New rotation quaternion
//          */
//         void setRotation(const Quaternion& rotation);
//
//         // === Velocity and Forces ===
//
//         /**
//          * @brief Get linear velocity
//          * @return Current linear velocity
//          */
//         Vector3 getLinearVelocity() const;
//
//         /**
//          * @brief Set linear velocity
//          * @param velocity New linear velocity
//          */
//         void setLinearVelocity(const Vector3& velocity);
//
//         /**
//          * @brief Get angular velocity
//          * @return Current angular velocity (rad/s)
//          */
//         Vector3 getAngularVelocity() const;
//
//         /**
//          * @brief Set angular velocity
//          * @param velocity New angular velocity (rad/s)
//          */
//         void setAngularVelocity(const Vector3& velocity);
//
//         /**
//          * @brief Apply force at center of mass
//          * @param force Force vector in world space
//          */
//         void applyForce(const Vector3& force);
//
//         /**
//          * @brief Apply force at specific point
//          * @param force Force vector in world space
//          * @param relativePosition Position relative to body center
//          */
//         void applyForce(const Vector3& force, const Vector3& relativePosition);
//
//         /**
//          * @brief Apply torque
//          * @param torque Torque vector in world space
//          */
//         void applyTorque(const Vector3& torque);
//
//         /**
//          * @brief Apply impulse at center of mass
//          * @param impulse Impulse vector in world space
//          */
//         void applyImpulse(const Vector3& impulse);
//
//         /**
//          * @brief Apply impulse at specific point
//          * @param impulse Impulse vector in world space
//          * @param relativePosition Position relative to body center
//          */
//         void applyImpulse(const Vector3& impulse, const Vector3& relativePosition);
//
//         /**
//          * @brief Apply torque impulse
//          * @param torqueImpulse Torque impulse vector
//          */
//         void applyTorqueImpulse(const Vector3& torqueImpulse);
//
//         /**
//          * @brief Clear all forces and torques
//          */
//         void clearForces();
//
//         // === Physical Properties ===
//
//         /**
//          * @brief Get mass
//          * @return Body mass (0 for static)
//          */
//         float getMass() const;
//
//         /**
//          * @brief Set mass (dynamic bodies only)
//          * @param mass New mass (must be > 0 for dynamic)
//          */
//         void setMass(float mass);
//
//         /**
//          * @brief Get friction coefficient
//          * @return Current friction
//          */
//         float getFriction() const;
//
//         /**
//          * @brief Set friction coefficient
//          * @param friction New friction (0 = no friction)
//          */
//         void setFriction(float friction);
//
//         /**
//          * @brief Get restitution (bounciness)
//          * @return Current restitution
//          */
//         float getRestitution() const;
//
//         /**
//          * @brief Set restitution (bounciness)
//          * @param restitution New restitution (0 = no bounce, 1 = perfect bounce)
//          */
//         void setRestitution(float restitution);
//
//         /**
//          * @brief Get linear damping
//          * @return Current linear damping
//          */
//         float getLinearDamping() const;
//
//         /**
//          * @brief Set linear damping
//          * @param damping New linear damping (0 = no damping)
//          */
//         void setLinearDamping(float damping);
//
//         /**
//          * @brief Get angular damping
//          * @return Current angular damping
//          */
//         float getAngularDamping() const;
//
//         /**
//          * @brief Set angular damping
//          * @param damping New angular damping (0 = no damping)
//          */
//         void setAngularDamping(float damping);
//
//         // === State Management ===
//
//         /**
//          * @brief Get activation state
//          * @return Current activation state
//          */
//         ActivationState getActivationState() const;
//
//         /**
//          * @brief Set activation state
//          * @param state New activation state
//          */
//         void setActivationState(ActivationState state);
//
//         /**
//          * @brief Check if body is active
//          * @return true if active
//          */
//         bool isActive() const;
//
//         /**
//          * @brief Activate the body
//          * @param forceActivation Force activation even if velocity is zero
//          */
//         void activate(bool forceActivation = false);
//
//         /**
//          * @brief Check if body is sleeping
//          * @return true if sleeping
//          */
//         bool isSleeping() const;
//
//         // === Collision Configuration ===
//
//         /**
//          * @brief Get collision filter
//          * @return Current collision filter
//          */
//         const CollisionFilter& getCollisionFilter() const noexcept { return collisionFilter_; }
//
//         /**
//          * @brief Set collision filter
//          * @param filter New collision filter
//          */
//         void setCollisionFilter(const CollisionFilter& filter);
//
//         /**
//          * @brief Enable/disable collision response
//          * @param enabled true to enable collisions
//          */
//         void setCollisionResponseEnabled(bool enabled);
//
//         // === Constraints ===
//
//         /**
//          * @brief Set linear factor (constraint linear motion)
//          * @param factor Linear factor (0 = locked, 1 = free)
//          */
//         void setLinearFactor(const Vector3& factor);
//
//         /**
//          * @brief Get linear factor
//          * @return Current linear factor
//          */
//         Vector3 getLinearFactor() const;
//
//         /**
//          * @brief Set angular factor (constraint angular motion)
//          * @param factor Angular factor (0 = locked, 1 = free)
//          */
//         void setAngularFactor(const Vector3& factor);
//
//         /**
//          * @brief Get angular factor
//          * @return Current angular factor
//          */
//         Vector3 getAngularFactor() const;
//
//         // === User Data ===
//
//         /**
//          * @brief Set user data pointer
//          * @param userData Custom user data
//          */
//         void setUserData(void* userData) { userData_ = userData; }
//
//         /**
//          * @brief Get user data pointer
//          * @return Custom user data
//          */
//         void* getUserData() const noexcept { return userData_; }
//
//         // === Continuous Collision Detection ===
//
//         /**
//          * @brief Enable/disable continuous collision detection
//          * @param enabled true to enable CCD
//          */
//         void setCCDEnabled(bool enabled);
//
//         /**
//          * @brief Set CCD motion threshold
//          * @param threshold Motion threshold for CCD activation
//          */
//         void setCCDMotionThreshold(float threshold);
//
//         /**
//          * @brief Set CCD swept sphere radius
//          * @param radius Sphere radius for CCD
//          */
//         void setCCDSweptSphereRadius(float radius);
//
//         // === Utility Functions ===
//
//         /**
//          * @brief Get axis-aligned bounding box
//          * @return AABB min and max points
//          */
//         std::pair<Vector3, Vector3> getAABB() const;
//
//         /**
//          * @brief Get total energy (kinetic + potential)
//          * @param gravity Gravity vector for potential energy calculation
//          * @return Total energy
//          */
//         float getTotalEnergy(const Vector3& gravity) const;
//
//         /**
//          * @brief Reset body to initial state
//          */
//         void reset();
//
//         CollisionShape* getShape() const noexcept { return shape_; }
//
//     private:
//         RigidBodyID id_; // Unique identifier
//         BodyType type_; // Body type
//         std::unique_ptr<btRigidBody> bulletBody_; // Bullet rigid body
//         std::unique_ptr<btGhostObject> ghostObject_; // Ghost object for triggers
//         std::unique_ptr<btDefaultMotionState> motionState_; // Motion state
//         CollisionShape* shape_; // Collision shape (not owned)
//         CollisionFilter collisionFilter_; // Collision filtering
//         bool isTrigger_ = false; // Is trigger volume
//         void* userData_ = nullptr; // User data pointer
//
//         // Initial state for reset
//         Transform3D initialTransform_; // Initial transform
//         Vector3 initialLinearVelocity_{0.0f, 0.0f, 0.0f}; // Initial linear velocity
//         Vector3 initialAngularVelocity_{0.0f, 0.0f, 0.0f}; // Initial angular velocity
//     };
//
//     // ========================================================================
//     // PHYSICS CONSTRAINT WRAPPER CLASS
//     // ========================================================================
//
//     /**
//      * @brief Wrapper for physics constraints/joints between bodies
//      */
//     class PhysicsConstraint {
//     public:
//         /**
//          * @brief Create constraint from configuration
//          * @param id Unique identifier
//          * @param config Constraint configuration
//          * @param bodyA First body
//          * @param bodyB Second body (optional)
//          */
//         PhysicsConstraint(ConstraintID id, const ConstraintConfig& config,
//                           RigidBody* bodyA, RigidBody* bodyB = nullptr);
//
//         /**
//          * @brief Destructor
//          */
//         ~PhysicsConstraint();
//
//         // Non-copyable but moveable
//         PhysicsConstraint(const PhysicsConstraint&) = delete;
//         PhysicsConstraint& operator=(const PhysicsConstraint&) = delete;
//         PhysicsConstraint(PhysicsConstraint&&) = default;
//         PhysicsConstraint& operator=(PhysicsConstraint&&) = default;
//
//         // === Accessors ===
//
//         ConstraintID getId() const noexcept { return id_; }
//         ConstraintType getType() const noexcept { return type_; }
//         btTypedConstraint* getBulletConstraint() const noexcept { return constraint_.get(); }
//
//         /**
//          * @brief Get connected bodies
//          * @return Pair of body IDs (second may be INVALID_BODY_ID)
//          */
//         std::pair<RigidBodyID, RigidBodyID> getConnectedBodies() const;
//
//         // === Constraint Properties ===
//
//         /**
//          * @brief Enable/disable the constraint
//          * @param enabled true to enable
//          */
//         void setEnabled(bool enabled);
//
//         /**
//          * @brief Check if constraint is enabled
//          * @return true if enabled
//          */
//         bool isEnabled() const;
//
//         /**
//          * @brief Set breaking impulse threshold
//          * @param threshold Impulse needed to break constraint
//          */
//         void setBreakingImpulseThreshold(float threshold);
//
//         /**
//          * @brief Get breaking impulse threshold
//          * @return Breaking impulse threshold
//          */
//         float getBreakingImpulseThreshold() const;
//
//         /**
//          * @brief Check if constraint is broken
//          * @return true if broken
//          */
//         bool isBroken() const;
//
//         /**
//          * @brief Get applied impulse (for monitoring stress)
//          * @return Applied impulse
//          */
//         float getAppliedImpulse() const;
//
//         // === Motor Control (for motorized constraints) ===
//
//         /**
//          * @brief Enable/disable motor
//          * @param enabled true to enable motor
//          */
//         void setMotorEnabled(bool enabled);
//
//         /**
//          * @brief Set motor target velocity
//          * @param velocity Target velocity
//          */
//         void setMotorTargetVelocity(float velocity);
//
//         /**
//          * @brief Set maximum motor impulse
//          * @param impulse Maximum impulse
//          */
//         void setMaxMotorImpulse(float impulse);
//
//         // === Limits (depends on constraint type) ===
//
//         /**
//          * @brief Set linear limits
//          * @param lower Lower limits
//          * @param upper Upper limits
//          */
//         void setLinearLimits(const Vector3& lower, const Vector3& upper);
//
//         /**
//          * @brief Set angular limits
//          * @param lower Lower limits
//          * @param upper Upper limits
//          */
//         void setAngularLimits(const Vector3& lower, const Vector3& upper);
//
//     private:
//         ConstraintID id_; // Unique identifier
//         ConstraintType type_; // Constraint type
//         std::unique_ptr<btTypedConstraint> constraint_; // Bullet constraint
//         RigidBodyID bodyA_; // First body ID
//         RigidBodyID bodyB_; // Second body ID
//         bool isBroken_ = false; // Constraint broken state
//     };
//
//     // ========================================================================
//     // PHYSICS MANAGER CLASS
//     // ========================================================================
//
//     /**
//      * @brief Main physics system manager
//      *
//      * This class manages the entire physics simulation including:
//      * - World creation and configuration
//      * - Rigid body management
//      * - Collision detection and response
//      * - Constraint management
//      * - Raycasting and queries
//      * - Debug visualization
//      */
//     class PhysicsManager {
//     public:
//         /**
//          * @brief Constructor with configuration
//          * @param config Physics configuration
//          */
//         explicit PhysicsManager(const PhysicsConfig& config = {});
//
//         /**
//          * @brief Destructor - cleanup all resources
//          */
//         ~PhysicsManager();
//
//         // Non-copyable and non-moveable (singleton-like)
//         PhysicsManager(const PhysicsManager&) = delete;
//         PhysicsManager& operator=(const PhysicsManager&) = delete;
//         PhysicsManager(PhysicsManager&&) = delete;
//         PhysicsManager& operator=(PhysicsManager&&) = delete;
//
//         /**
//          * @brief Initialize physics system
//          * @return true if successful
//          */
//         bool initialize();
//
//         /**
//          * @brief Shutdown physics system
//          */
//         void shutdown();
//
//         /**
//          * @brief Update physics simulation
//          * @param deltaTime Time since last update
//          */
//         void update(float deltaTime);
//
//         /**
//          * @brief Fixed update for physics (called at fixed timestep)
//          * @param fixedDeltaTime Fixed time step
//          */
//         void fixedUpdate(float fixedDeltaTime);
//
//         // === World Configuration ===
//
//         /**
//          * @brief Set gravity
//          * @param gravity New gravity vector
//          */
//         void setGravity(const Vector3& gravity);
//
//         /**
//          * @brief Get current gravity
//          * @return Gravity vector
//          */
//         Vector3 getGravity() const;
//
//         /**
//          * @brief Update configuration
//          * @param config New configuration
//          */
//         void updateConfig(const PhysicsConfig& config);
//
//         /**
//          * @brief Get current configuration
//          * @return Current configuration
//          */
//         const PhysicsConfig& getConfig() const noexcept { return config_; }
//
//         // === Collision Shape Management ===
//
//         /**
//          * @brief Register a collision shape
//          * @param shape Shape to register
//          * @return Shape ID
//          */
//         CollisionShapeID registerShape(std::unique_ptr<CollisionShape> shape);
//
//         /**
//          * @brief Get collision shape by ID
//          * @param shapeId Shape ID
//          * @return Pointer to shape or nullptr
//          */
//         CollisionShape* getShape(CollisionShapeID shapeId);
//
//         /**
//          * @brief Remove collision shape
//          * @param shapeId Shape to remove
//          * @return true if removed
//          */
//         bool removeShape(CollisionShapeID shapeId);
//
//         // === Rigid Body Management ===
//
//         /**
//          * @brief Create rigid body
//          * @param shapeId Collision shape to use
//          * @param config Body configuration
//          * @return Body ID or INVALID_BODY_ID if failed
//          */
//         RigidBodyID createRigidBody(CollisionShapeID shapeId, const RigidBodyConfig& config);
//
//         /**
//          * @brief Remove rigid body
//          * @param bodyId Body to remove
//          * @return true if removed
//          */
//         bool removeRigidBody(RigidBodyID bodyId);
//
//         /**
//          * @brief Get rigid body by ID
//          * @param bodyId Body ID
//          * @return Pointer to body or nullptr
//          */
//         RigidBody* getRigidBody(RigidBodyID bodyId);
//
//         /**
//          * @brief Get all rigid bodies
//          * @return Vector of all body IDs
//          */
//         std::vector<RigidBodyID> getAllRigidBodies() const;
//
//         // === Constraint Management ===
//
//         /**
//          * @brief Create constraint between bodies
//          * @param config Constraint configuration
//          * @return Constraint ID or INVALID_CONSTRAINT_ID if failed
//          */
//         ConstraintID createConstraint(const ConstraintConfig& config);
//
//         /**
//          * @brief Remove constraint
//          * @param constraintId Constraint to remove
//          * @return true if removed
//          */
//         bool removeConstraint(ConstraintID constraintId);
//
//         /**
//          * @brief Get constraint by ID
//          * @param constraintId Constraint ID
//          * @return Pointer to constraint or nullptr
//          */
//         PhysicsConstraint* getConstraint(ConstraintID constraintId);
//
//         // === Collision Callbacks ===
//
//         /**
//          * @brief Register collision callback
//          * @param callback Callback function
//          */
//         void registerCollisionCallback(const CollisionCallback& callback);
//
//         /**
//          * @brief Register trigger callback
//          * @param callback Callback function
//          */
//         void registerTriggerCallback(const TriggerCallback& callback);
//
//         /**
//          * @brief Register pre-step callback
//          * @param callback Callback function
//          */
//         void registerPreStepCallback(const PreStepCallback& callback);
//
//         /**
//          * @brief Register post-step callback
//          * @param callback Callback function
//          */
//         void registerPostStepCallback(const PostStepCallback& callback);
//
//         // === Raycasting and Queries ===
//
//         /**
//          * @brief Perform raycast
//          * @param ray Ray configuration
//          * @return Raycast result
//          */
//         RaycastResult raycast(const Ray& ray) const;
//
//         /**
//          * @brief Perform sweep test
//          * @param sweep Sweep configuration
//          * @return Sweep results
//          */
//         RaycastResult sweepTest(const SweepTest& sweep) const;
//
//         /**
//          * @brief Perform overlap test
//          * @param overlap Overlap configuration
//          * @return Vector of overlapping body IDs
//          */
//         std::vector<RigidBodyID> overlapTest(const OverlapTest& overlap) const;
//
//         /**
//          * @brief Query bodies in AABB
//          * @param min AABB minimum
//          * @param max AABB maximum
//          * @param filter Collision filter
//          * @return Vector of body IDs in AABB
//          */
//         std::vector<RigidBodyID> queryAABB(const Vector3& min, const Vector3& max,
//                                            const CollisionFilter& filter = {}) const;
//
//         // === Debug and Statistics ===
//
//         /**
//          * @brief Enable/disable debug drawing
//          * @param enabled true to enable
//          */
//         void setDebugDrawEnabled(bool enabled);
//
//         /**
//          * @brief Set debug draw mode flags
//          * @param mode Debug draw mode flags
//          */
//         void setDebugDrawMode(int mode);
//
//         /**
//          * @brief Debug draw the physics world
//          * @param renderer Renderer to use for drawing
//          */
//         void debugDraw(void* renderer);
//
//         /**
//          * @brief Get physics statistics
//          * @return Current statistics
//          */
//         PhysicsStatistics getStatistics() const;
//
//         /**
//          * @brief Get debug info string
//          * @return Debug information
//          */
//         std::string getDebugInfo() const;
//
//         // === Utility Functions ===
//
//         /**
//          * @brief Clear all forces on all bodies
//          */
//         void clearAllForces();
//
//         /**
//          * @brief Reset simulation
//          */
//         void resetSimulation();
//
//         /**
//          * @brief Get number of active bodies
//          * @return Active body count
//          */
//         int getActiveBodyCount() const;
//
//         /**
//          * @brief Get total body count
//          * @return Total body count
//          */
//         int getTotalBodyCount() const;
//
//         /**
//          * @brief Set simulation speed
//          * @param speed Speed multiplier (1.0 = normal)
//          */
//         void setSimulationSpeed(float speed) { simulationSpeed_ = speed; }
//
//         /**
//          * @brief Get simulation speed
//          * @return Current speed multiplier
//          */
//         float getSimulationSpeed() const noexcept { return simulationSpeed_; }
//
//     private:
//         // === Internal Methods ===
//
//         void setupBulletWorld();
//         void cleanupBulletWorld();
//         void processCollisions();
//         void updateStatistics();
//
//         // === Bullet to GLM Conversions ===
//
//         static btVector3 toBullet(const Vector3& v);
//         static Vector3 fromBullet(const btVector3& v);
//         static btQuaternion toBullet(const Quaternion& q);
//         static Quaternion fromBullet(const btQuaternion& q);
//         static btTransform toBullet(const Transform3D& t);
//         static Transform3D fromBullet(const btTransform& t);
//
//         // === Member Variables ===
//
//         // Configuration and state
//         PhysicsConfig config_; // Configuration
//         bool initialized_ = false; // Initialization state
//         float simulationSpeed_ = 1.0f; // Simulation speed multiplier
//         float accumulator_ = 0.0f; // Time accumulator for fixed timestep
//
//         // Bullet world components
//         std::unique_ptr<btDefaultCollisionConfiguration> collisionConfig_;
//         std::unique_ptr<btCollisionDispatcher> dispatcher_;
//         std::unique_ptr<btDbvtBroadphase> broadphase_;
//         std::unique_ptr<btSequentialImpulseConstraintSolver> solver_;
//         std::unique_ptr<btDiscreteDynamicsWorld> dynamicsWorld_;
//         std::unique_ptr<btSoftBodySolver> softBodySolver_;
//         std::unique_ptr<btGhostPairCallback> ghostPairCallback_;
//
//         // Resource storage
//         std::unordered_map<CollisionShapeID, std::unique_ptr<CollisionShape>> shapes_;
//         std::unordered_map<RigidBodyID, std::unique_ptr<RigidBody>> bodies_;
//         std::unordered_map<ConstraintID, std::unique_ptr<PhysicsConstraint>> constraints_;
//
//         // ID generation
//         std::atomic<RigidBodyID> nextBodyId_{1};
//         std::atomic<CollisionShapeID> nextShapeId_{1};
//         std::atomic<ConstraintID> nextConstraintId_{1};
//
//         // Collision tracking
//         std::unordered_map<CollisionPairID, CollisionManifold> activeCollisions_;
//         std::vector<CollisionManifold> collisionEvents_;
//
//         // Callbacks
//         std::vector<CollisionCallback> collisionCallbacks_;
//         std::vector<TriggerCallback> triggerCallbacks_;
//         std::vector<PreStepCallback> preStepCallbacks_;
//         std::vector<PostStepCallback> postStepCallbacks_;
//
//         // Debug
//         std::unique_ptr<PhysicsDebugDrawer> debugDrawer_;
//
//         // Statistics
//         mutable PhysicsStatistics statistics_;
//         std::chrono::high_resolution_clock::time_point lastUpdateTime_;
//
//         // Thread safety
//         mutable std::mutex physicsMutex_;
//
//         friend class CollisionListener;
//         friend class PhysicsDebugDrawer;
//     };
// } // namespace engine::physics
