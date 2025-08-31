// //
// // Created by Andres Guerrero on 11-08-25.
// //
//
// // ========================================================================
// // COLLISION SHAPE IMPLEMENTATION
// // ========================================================================
//
// #include "PhysicsManager.h"
// #include <iostream>
// #include <algorithm>
// #include <cmath>
//
// namespace engine::physics {
//     // Static member initialization
//     std::atomic<CollisionShapeID> CollisionShape::nextShapeId_{1};
//
//     /**
//      * @brief Private constructor for CollisionShape
//      * IMPLEMENTATION: Initializes the shape with proper resource management
//      */
//     CollisionShape::CollisionShape(CollisionShapeID id, ShapeType type, std::unique_ptr<btCollisionShape> shape)
//         : id_(id)
//           , type_(type)
//           , bulletShape_(std::move(shape)) {
//         // Set default margin for better collision detection
//         if (bulletShape_) {
//             bulletShape_->setMargin(0.04f); // 4cm margin for stable collisions
//         }
//     }
//
//     /**
//      * @brief Destructor ensures proper cleanup
//      * IMPLEMENTATION: Releases Bullet resources in correct order
//      */
//     CollisionShape::~CollisionShape() {
//         // Bullet shape is automatically cleaned up by unique_ptr
//         // Mesh data is also automatically cleaned up
//     }
//
//     /**
//      * @brief Create a box collision shape
//      * IMPLEMENTATION: Creates an optimized box shape for Bullet Physics
//      */
//     std::unique_ptr<CollisionShape> CollisionShape::CreateBox(const Vector3& halfExtents) {
//         // Validate input
//         if (halfExtents.x <= 0.0f || halfExtents.y <= 0.0f || halfExtents.z <= 0.0f) {
//             std::cerr << "[Physics] Error: Invalid box half extents" << std::endl;
//             return nullptr;
//         }
//
//         // Create Bullet box shape with half extents
//         auto bulletBox = std::make_unique<btBoxShape>(
//             btVector3(halfExtents.x, halfExtents.y, halfExtents.z)
//         );
//
//         // Create our wrapper
//         CollisionShapeID id = nextShapeId_.fetch_add(1);
//         auto shape = std::unique_ptr<CollisionShape>(
//             new CollisionShape(id, ShapeType::BOX, std::move(bulletBox))
//         );
//
//         std::cout << "[Physics] Created box shape (ID: " << id
//             << ", half extents: " << halfExtents.x << ", " << halfExtents.y << ", " << halfExtents.z << ")" <<
//             std::endl;
//
//         return shape;
//     }
//
//     /**
//      * @brief Create a sphere collision shape
//      * IMPLEMENTATION: Creates an optimized sphere shape for fast collision detection
//      */
//     std::unique_ptr<CollisionShape> CollisionShape::CreateSphere(float radius) {
//         // Validate input
//         if (radius <= 0.0f) {
//             std::cerr << "[Physics] Error: Invalid sphere radius" << std::endl;
//             return nullptr;
//         }
//
//         // Create Bullet sphere shape
//         auto bulletSphere = std::make_unique<btSphereShape>(radius);
//
//         // Create our wrapper
//         CollisionShapeID id = nextShapeId_.fetch_add(1);
//         auto shape = std::unique_ptr<CollisionShape>(
//             new CollisionShape(id, ShapeType::SPHERE, std::move(bulletSphere))
//         );
//
//         std::cout << "[Physics] Created sphere shape (ID: " << id << ", radius: " << radius << ")" << std::endl;
//
//         return shape;
//     }
//
//     /**
//      * @brief Create a capsule collision shape
//      * IMPLEMENTATION: Capsules are excellent for character controllers
//      */
//     std::unique_ptr<CollisionShape> CollisionShape::CreateCapsule(float radius, float height, int axis) {
//         // Validate input
//         if (radius <= 0.0f || height <= 0.0f || axis < 0 || axis > 2) {
//             std::cerr << "[Physics] Error: Invalid capsule parameters" << std::endl;
//             return nullptr;
//         }
//
//         // Create Bullet capsule shape based on axis
//         std::unique_ptr<btCollisionShape> bulletCapsule;
//
//         switch (axis) {
//         case 0: // X axis
//             bulletCapsule = std::make_unique<btCapsuleShapeX>(radius, height);
//             break;
//         case 1: // Y axis (most common for characters)
//             bulletCapsule = std::make_unique<btCapsuleShape>(radius, height);
//             break;
//         case 2: // Z axis
//             bulletCapsule = std::make_unique<btCapsuleShapeZ>(radius, height);
//             break;
//         }
//
//         // Create our wrapper
//         CollisionShapeID id = nextShapeId_.fetch_add(1);
//         auto shape = std::unique_ptr<CollisionShape>(
//             new CollisionShape(id, ShapeType::CAPSULE, std::move(bulletCapsule))
//         );
//
//         std::cout << "[Physics] Created capsule shape (ID: " << id
//             << ", radius: " << radius << ", height: " << height << ", axis: " << axis << ")" << std::endl;
//
//         return shape;
//     }
//
//     /**
//      * @brief Create a cylinder collision shape
//      * IMPLEMENTATION: Cylinders for barrels, pillars, etc.
//      */
//     std::unique_ptr<CollisionShape> CollisionShape::CreateCylinder(const Vector3& halfExtents, int axis) {
//         // Validate input
//         if (halfExtents.x <= 0.0f || halfExtents.y <= 0.0f || halfExtents.z <= 0.0f || axis < 0 || axis > 2) {
//             std::cerr << "[Physics] Error: Invalid cylinder parameters" << std::endl;
//             return nullptr;
//         }
//
//         // Create Bullet cylinder shape based on axis
//         std::unique_ptr<btCollisionShape> bulletCylinder;
//         btVector3 btHalfExtents(halfExtents.x, halfExtents.y, halfExtents.z);
//
//         switch (axis) {
//         case 0: // X axis
//             bulletCylinder = std::make_unique<btCylinderShapeX>(btHalfExtents);
//             break;
//         case 1: // Y axis (default)
//             bulletCylinder = std::make_unique<btCylinderShape>(btHalfExtents);
//             break;
//         case 2: // Z axis
//             bulletCylinder = std::make_unique<btCylinderShapeZ>(btHalfExtents);
//             break;
//         }
//
//         // Create our wrapper
//         CollisionShapeID id = nextShapeId_.fetch_add(1);
//         auto shape = std::unique_ptr<CollisionShape>(
//             new CollisionShape(id, ShapeType::CYLINDER, std::move(bulletCylinder))
//         );
//
//         std::cout << "[Physics] Created cylinder shape (ID: " << id << ", axis: " << axis << ")" << std::endl;
//
//         return shape;
//     }
//
//     /**
//      * @brief Create a cone collision shape
//      * IMPLEMENTATION: Cones for specialized collision volumes
//      */
//     std::unique_ptr<CollisionShape> CollisionShape::CreateCone(float radius, float height, int axis) {
//         // Validate input
//         if (radius <= 0.0f || height <= 0.0f || axis < 0 || axis > 2) {
//             std::cerr << "[Physics] Error: Invalid cone parameters" << std::endl;
//             return nullptr;
//         }
//
//         // Create Bullet cone shape based on axis
//         std::unique_ptr<btCollisionShape> bulletCone;
//
//         switch (axis) {
//         case 0: // X axis
//             bulletCone = std::make_unique<btConeShapeX>(radius, height);
//             break;
//         case 1: // Y axis (default)
//             bulletCone = std::make_unique<btConeShape>(radius, height);
//             break;
//         case 2: // Z axis
//             bulletCone = std::make_unique<btConeShapeZ>(radius, height);
//             break;
//         }
//
//         // Create our wrapper
//         CollisionShapeID id = nextShapeId_.fetch_add(1);
//         auto shape = std::unique_ptr<CollisionShape>(
//             new CollisionShape(id, ShapeType::CONE, std::move(bulletCone))
//         );
//
//         std::cout << "[Physics] Created cone shape (ID: " << id
//             << ", radius: " << radius << ", height: " << height << ", axis: " << axis << ")" << std::endl;
//
//         return shape;
//     }
//
//     /**
//      * @brief Create a convex hull from points
//      * IMPLEMENTATION: Optimized convex hull for complex but convex shapes
//      */
//     std::unique_ptr<CollisionShape>
//     CollisionShape::CreateConvexHull(const std::vector<Vector3>& points, bool optimized) {
//         // Validate input
//         if (points.size() < 4) {
//             // Need at least 4 points for a 3D convex hull
//             std::cerr << "[Physics] Error: Need at least 4 points for convex hull" << std::endl;
//             return nullptr;
//         }
//
//         // Create Bullet convex hull shape
//         auto bulletHull = std::make_unique<btConvexHullShape>();
//
//         // Add points to hull
//         for (const auto& point : points) {
//             bulletHull->addPoint(btVector3(point.x, point.y, point.z), false); // Don't recalculate AABB each time
//         }
//
//         // Recalculate AABB once after all points added
//         bulletHull->recalcLocalAabb();
//
//         // Optimize hull if requested (reduces point count)
//         if (optimized && points.size() > 16) {
//             bulletHull->optimizeConvexHull();
//             std::cout << "[Physics] Optimized convex hull from " << points.size()
//                 << " to " << bulletHull->getNumVertices() << " vertices" << std::endl;
//         }
//
//         // Create our wrapper
//         CollisionShapeID id = nextShapeId_.fetch_add(1);
//         auto shape = std::unique_ptr<CollisionShape>(
//             new CollisionShape(id, ShapeType::CONVEX_HULL, std::move(bulletHull))
//         );
//
//         std::cout << "[Physics] Created convex hull shape (ID: " << id
//             << ", vertices: " << points.size() << ")" << std::endl;
//
//         return shape;
//     }
//
//     /**
//      * @brief Create a static triangle mesh
//      * IMPLEMENTATION: For static level geometry with complex concave shapes
//      */
//     std::unique_ptr<CollisionShape> CollisionShape::CreateTriangleMesh(
//         const std::vector<Vector3>& vertices,
//         const std::vector<int>& indices,
//         bool buildBVH) {
//         // Validate input
//         if (vertices.empty() || indices.empty() || indices.size() % 3 != 0) {
//             std::cerr << "[Physics] Error: Invalid triangle mesh data" << std::endl;
//             return nullptr;
//         }
//
//         // Create mesh data storage (needs to persist)
//         auto meshData = std::make_unique<btTriangleMesh>();
//
//         // Add triangles to mesh
//         for (size_t i = 0; i < indices.size(); i += 3) {
//             const Vector3& v0 = vertices[indices[i]];
//             const Vector3& v1 = vertices[indices[i + 1]];
//             const Vector3& v2 = vertices[indices[i + 2]];
//
//             meshData->addTriangle(
//                 btVector3(v0.x, v0.y, v0.z),
//                 btVector3(v1.x, v1.y, v1.z),
//                 btVector3(v2.x, v2.y, v2.z),
//                 false // Don't remove duplicates (performance)
//             );
//         }
//
//         // Create shape with optional BVH optimization
//         std::unique_ptr<btCollisionShape> bulletMesh;
//
//         if (buildBVH) {
//             // BVH (Bounding Volume Hierarchy) for faster ray/collision tests
//             bulletMesh = std::make_unique<btBvhTriangleMeshShape>(meshData.get(), true, true);
//             std::cout << "[Physics] Built BVH for triangle mesh" << std::endl;
//         }
//         else {
//             // Simple triangle mesh (slower but uses less memory)
//             bulletMesh = std::make_unique<btBvhTriangleMeshShape>(meshData.get(), true);
//         }
//
//         // Create our wrapper (special handling for mesh data)
//         CollisionShapeID id = nextShapeId_.fetch_add(1);
//         auto shape = std::unique_ptr<CollisionShape>(
//             new CollisionShape(id, ShapeType::TRIANGLE_MESH, std::move(bulletMesh))
//         );
//         shape->meshData_ = std::move(meshData); // Store mesh data
//
//         std::cout << "[Physics] Created triangle mesh shape (ID: " << id
//             << ", vertices: " << vertices.size()
//             << ", triangles: " << indices.size() / 3 << ")" << std::endl;
//
//         return shape;
//     }
//
//     /**
//      * @brief Create a heightfield terrain shape
//      * IMPLEMENTATION: Optimized terrain collision from height data
//      */
//     std::unique_ptr<CollisionShape> CollisionShape::CreateHeightfield(
//         int width, int height,
//         const std::vector<float>& heightData,
//         float minHeight, float maxHeight,
//         int upAxis) {
//         // Validate input
//         if (width <= 0 || height <= 0 || heightData.size() != static_cast<size_t>(width * height)) {
//             std::cerr << "[Physics] Error: Invalid heightfield parameters" << std::endl;
//             return nullptr;
//         }
//
//         // Create heightfield shape
//         // Note: Bullet expects the height data to persist, so we need to store it
//         auto bulletHeightfield = std::make_unique<btHeightfieldTerrainShape>(
//             width, height,
//             const_cast<float*>(heightData.data()), // Bullet needs non-const pointer
//             1.0f, // Height scale
//             minHeight, maxHeight,
//             upAxis,
//             PHY_FLOAT, // Data type
//             false // Flip quad edges
//         );
//
//         // Set scale to match world units
//         bulletHeightfield->setLocalScaling(btVector3(1.0f, 1.0f, 1.0f));
//
//         // Create our wrapper
//         CollisionShapeID id = nextShapeId_.fetch_add(1);
//         auto shape = std::unique_ptr<CollisionShape>(
//             new CollisionShape(id, ShapeType::HEIGHTFIELD, std::move(bulletHeightfield))
//         );
//
//         std::cout << "[Physics] Created heightfield shape (ID: " << id
//             << ", size: " << width << "x" << height
//             << ", height range: " << minHeight << " to " << maxHeight << ")" << std::endl;
//
//         return shape;
//     }
//
//     /**
//      * @brief Create a compound shape from multiple child shapes
//      * IMPLEMENTATION: Combine multiple shapes into one complex shape
//      */
//     std::unique_ptr<CollisionShape> CollisionShape::CreateCompound(
//         const std::vector<std::pair<std::unique_ptr<CollisionShape>, Transform3D>>& children) {
//         // Validate input
//         if (children.empty()) {
//             std::cerr << "[Physics] Error: Compound shape needs at least one child" << std::endl;
//             return nullptr;
//         }
//
//         // Create compound shape
//         auto bulletCompound = std::make_unique<btCompoundShape>();
//
//         // Add child shapes
//         for (const auto& [childShape, transform] : children) {
//             if (!childShape || !childShape->getBulletShape()) {
//                 continue;
//             }
//
//             // Convert transform to Bullet
//             btTransform btTrans;
//             btTrans.setOrigin(btVector3(transform.position.x, transform.position.y, transform.position.z));
//             btTrans.setRotation(btQuaternion(transform.rotation.x, transform.rotation.y,
//                                              transform.rotation.z, transform.rotation.w));
//
//             // Add child shape (compound takes ownership)
//             bulletCompound->addChildShape(btTrans, childShape->getBulletShape());
//         }
//
//         // Create our wrapper
//         CollisionShapeID id = nextShapeId_.fetch_add(1);
//         auto shape = std::unique_ptr<CollisionShape>(
//             new CollisionShape(id, ShapeType::COMPOUND, std::move(bulletCompound))
//         );
//
//         std::cout << "[Physics] Created compound shape (ID: " << id
//             << ", children: " << children.size() << ")" << std::endl;
//
//         return shape;
//     }
//
//     /**
//      * @brief Create an infinite plane shape
//      * IMPLEMENTATION: Useful for ground planes and boundaries
//      */
//     std::unique_ptr<CollisionShape> CollisionShape::CreatePlane(const Vector3& normal, float offset) {
//         // Validate normal
//         float length = glm::length(normal);
//         if (length < 0.001f) {
//             std::cerr << "[Physics] Error: Invalid plane normal" << std::endl;
//             return nullptr;
//         }
//
//         // Normalize the normal vector
//         Vector3 normalizedNormal = normal / length;
//
//         // Create plane shape
//         auto bulletPlane = std::make_unique<btStaticPlaneShape>(
//             btVector3(normalizedNormal.x, normalizedNormal.y, normalizedNormal.z),
//             offset
//         );
//
//         // Create our wrapper
//         CollisionShapeID id = nextShapeId_.fetch_add(1);
//         auto shape = std::unique_ptr<CollisionShape>(
//             new CollisionShape(id, ShapeType::PLANE, std::move(bulletPlane))
//         );
//
//         std::cout << "[Physics] Created plane shape (ID: " << id
//             << ", normal: " << normalizedNormal.x << ", " << normalizedNormal.y << ", " << normalizedNormal.z
//             << ", offset: " << offset << ")" << std::endl;
//
//         return shape;
//     }
//
//     /**
//      * @brief Get local scaling of the shape
//      * IMPLEMENTATION: Returns the current scale factors
//      */
//     Vector3 CollisionShape::getLocalScaling() const {
//         if (!bulletShape_) {
//             return Vector3(1.0f, 1.0f, 1.0f);
//         }
//
//         const btVector3& scale = bulletShape_->getLocalScaling();
//         return Vector3(scale.x(), scale.y(), scale.z());
//     }
//
//     /**
//      * @brief Set local scaling of the shape
//      * IMPLEMENTATION: Scales the collision shape
//      */
//     void CollisionShape::setLocalScaling(const Vector3& scale) {
//         if (!bulletShape_) {
//             return;
//         }
//
//         // Validate scale
//         if (scale.x <= 0.0f || scale.y <= 0.0f || scale.z <= 0.0f) {
//             std::cerr << "[Physics] Warning: Invalid scale values, must be positive" << std::endl;
//             return;
//         }
//
//         bulletShape_->setLocalScaling(btVector3(scale.x, scale.y, scale.z));
//     }
//
//     /**
//      * @brief Calculate volume of the shape
//      * IMPLEMENTATION: Approximates volume for mass calculations
//      */
//     float CollisionShape::calculateVolume() const {
//         if (!bulletShape_) {
//             return 0.0f;
//         }
//
//         // Calculate volume based on shape type
//         switch (type_) {
//         case ShapeType::BOX: {
//             auto* box = static_cast<btBoxShape*>(bulletShape_.get());
//             btVector3 halfExtents = box->getHalfExtentsWithMargin();
//             return 8.0f * halfExtents.x() * halfExtents.y() * halfExtents.z();
//         }
//
//         case ShapeType::SPHERE: {
//             auto* sphere = static_cast<btSphereShape*>(bulletShape_.get());
//             float radius = sphere->getRadius();
//             return (4.0f / 3.0f) * math::constants::PI * radius * radius * radius;
//         }
//
//         case ShapeType::CAPSULE: {
//             auto* capsule = static_cast<btCapsuleShape*>(bulletShape_.get());
//             float radius = capsule->getRadius();
//             float height = capsule->getHalfHeight() * 2.0f;
//             // Volume = cylinder + sphere
//             float cylinderVolume = math::constants::PI * radius * radius * height;
//             float sphereVolume = (4.0f / 3.0f) * math::constants::PI * radius * radius * radius;
//             return cylinderVolume + sphereVolume;
//         }
//
//         case ShapeType::CYLINDER: {
//             auto* cylinder = static_cast<btCylinderShape*>(bulletShape_.get());
//             btVector3 halfExtents = cylinder->getHalfExtentsWithMargin();
//             // Assuming Y-axis cylinder
//             float radius = halfExtents.x();
//             float height = halfExtents.y() * 2.0f;
//             return math::constants::PI * radius * radius * height;
//         }
//
//         case ShapeType::CONE: {
//             auto* cone = static_cast<btConeShape*>(bulletShape_.get());
//             float radius = cone->getRadius();
//             float height = cone->getHeight();
//             return (1.0f / 3.0f) * math::constants::PI * radius * radius * height;
//         }
//
//         default: {
//             // For complex shapes, use bounding box approximation
//             btVector3 aabbMin, aabbMax;
//             bulletShape_->getAabb(btTransform::getIdentity(), aabbMin, aabbMax);
//             btVector3 size = aabbMax - aabbMin;
//             return size.x() * size.y() * size.z();
//         }
//         }
//     }
//
//     /**
//      * @brief Calculate local inertia tensor
//      * IMPLEMENTATION: Required for realistic rotation dynamics
//      */
//     Vector3 CollisionShape::calculateLocalInertia(float mass) const {
//         if (!bulletShape_ || mass <= 0.0f) {
//             return Vector3(0.0f, 0.0f, 0.0f);
//         }
//
//         btVector3 inertia(0, 0, 0);
//         bulletShape_->calculateLocalInertia(mass, inertia);
//
//         return Vector3(inertia.x(), inertia.y(), inertia.z());
//     }
//
//     /**
//      * @brief Get axis-aligned bounding box
//      * IMPLEMENTATION: Returns AABB for broad-phase collision detection
//      */
//     std::pair<Vector3, Vector3> CollisionShape::getAABB(const Transform3D& transform) const {
//         if (!bulletShape_) {
//             return {Vector3(0.0f), Vector3(0.0f)};
//         }
//
//         // Convert transform to Bullet
//         btTransform btTrans;
//         btTrans.setOrigin(btVector3(transform.position.x, transform.position.y, transform.position.z));
//         btTrans.setRotation(btQuaternion(transform.rotation.x, transform.rotation.y,
//                                          transform.rotation.z, transform.rotation.w));
//
//         // Get AABB from Bullet
//         btVector3 aabbMin, aabbMax;
//         bulletShape_->getAabb(btTrans, aabbMin, aabbMax);
//
//         return {
//             Vector3(aabbMin.x(), aabbMin.y(), aabbMin.z()),
//             Vector3(aabbMax.x(), aabbMax.y(), aabbMax.z())
//         };
//     }
//
//     /**
//      * @brief Check if shape is convex
//      * IMPLEMENTATION: Important for optimization and algorithm selection
//      */
//     bool CollisionShape::isConvex() const noexcept {
//         if (!bulletShape_) {
//             return false;
//         }
//
//         return bulletShape_->isConvex();
//     }
//
//     /**
//      * @brief Check if shape is compound
//      * IMPLEMENTATION: Compound shapes need special handling
//      */
//     bool CollisionShape::isCompound() const noexcept {
//         return type_ == ShapeType::COMPOUND;
//     }
//
//     /**
//      * @brief Get memory usage
//      * IMPLEMENTATION: Approximates memory footprint for profiling
//      */
//     std::size_t CollisionShape::getMemoryUsage() const {
//         std::size_t totalMemory = sizeof(CollisionShape);
//
//         // Add shape-specific memory
//         if (bulletShape_) {
//             // Base shape size
//             totalMemory += sizeof(*bulletShape_);
//
//             // Add type-specific memory
//             switch (type_) {
//             case ShapeType::TRIANGLE_MESH:
//                 if (meshData_) {
//                     totalMemory += meshData_->getNumTriangles() * sizeof(btVector3) * 3;
//                 }
//                 break;
//
//             case ShapeType::CONVEX_HULL: {
//                 auto* hull = static_cast<btConvexHullShape*>(bulletShape_.get());
//                 totalMemory += hull->getNumVertices() * sizeof(btVector3);
//                 break;
//             }
//
//             case ShapeType::COMPOUND: {
//                 auto* compound = static_cast<btCompoundShape*>(bulletShape_.get());
//                 totalMemory += compound->getNumChildShapes() * sizeof(btCompoundShapeChild);
//                 break;
//             }
//
//             default:
//                 // Simple shapes have minimal additional memory
//                 break;
//             }
//         }
//
//         return totalMemory;
//     }
//
//     // ========================================================================
//     // RIGID BODY IMPLEMENTATION
//     // ========================================================================
//
//     /**
//  * @brief Constructor for RigidBody
//  * IMPLEMENTATION: Creates and configures a Bullet rigid body with all properties
//  */
//     RigidBody::RigidBody(RigidBodyID id, CollisionShape* shape, const RigidBodyConfig& config)
//         : id_(id)
//           , type_(config.type)
//           , shape_(shape)
//           , collisionFilter_(config.filter)
//           , isTrigger_(config.isTrigger)
//           , initialTransform_(config.transform) {
//         if (!shape || !shape->getBulletShape()) {
//             throw std::runtime_error("[Physics] Cannot create rigid body without valid shape");
//         }
//
//         // Store initial velocities for reset functionality
//         initialLinearVelocity_ = Vector3(0.0f, 0.0f, 0.0f);
//         initialAngularVelocity_ = Vector3(0.0f, 0.0f, 0.0f);
//
//         // Convert transform to Bullet
//         btTransform startTransform;
//         startTransform.setOrigin(btVector3(
//             config.transform.position.x,
//             config.transform.position.y,
//             config.transform.position.z
//         ));
//         startTransform.setRotation(btQuaternion(
//             config.transform.rotation.x,
//             config.transform.rotation.y,
//             config.transform.rotation.z,
//             config.transform.rotation.w
//         ));
//
//         if (isTrigger_) {
//             // Create ghost object for triggers (no collision response)
//             ghostObject_ = std::make_unique<btGhostObject>();
//             ghostObject_->setCollisionShape(shape->getBulletShape());
//             ghostObject_->setWorldTransform(startTransform);
//             ghostObject_->setCollisionFlags(
//                 ghostObject_->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE
//             );
//
//             // Set user pointer for callbacks
//             ghostObject_->setUserPointer(this);
//
//             std::cout << "[Physics] Created trigger/ghost object (ID: " << id << ")" << std::endl;
//         }
//         else {
//             // Create motion state for interpolation
//             motionState_ = std::make_unique<btDefaultMotionState>(startTransform);
//
//             // Calculate mass and inertia
//             float mass = (config.type == BodyType::STATIC) ? 0.0f : config.mass;
//             btVector3 localInertia(0, 0, 0);
//
//             if (mass > 0.0f) {
//                 // Dynamic body - calculate inertia if not provided
//                 if (config.localInertia.x == 0.0f && config.localInertia.y == 0.0f && config.localInertia.z == 0.0f) {
//                     shape->getBulletShape()->calculateLocalInertia(mass, localInertia);
//                 }
//                 else {
//                     localInertia = btVector3(config.localInertia.x, config.localInertia.y, config.localInertia.z);
//                 }
//             }
//
//             // Create rigid body
//             btRigidBody::btRigidBodyConstructionInfo rbInfo(
//                 mass,
//                 motionState_.get(),
//                 shape->getBulletShape(),
//                 localInertia
//             );
//
//             // Apply material properties
//             rbInfo.m_friction = config.material.friction;
//             rbInfo.m_rollingFriction = config.material.rollingFriction;
//             rbInfo.m_restitution = config.material.restitution;
//             rbInfo.m_linearDamping = config.material.linearDamping;
//             rbInfo.m_angularDamping = config.material.angularDamping;
//
//             // Create the rigid body
//             bulletBody_ = std::make_unique<btRigidBody>(rbInfo);
//
//             // Configure body type
//             if (config.type == BodyType::KINEMATIC || config.isKinematic) {
//                 bulletBody_->setCollisionFlags(
//                     bulletBody_->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT
//                 );
//                 bulletBody_->setActivationState(DISABLE_DEACTIVATION);
//             }
//
//             // Set motion constraints
//             bulletBody_->setLinearFactor(btVector3(
//                 config.linearFactor.x,
//                 config.linearFactor.y,
//                 config.linearFactor.z
//             ));
//             bulletBody_->setAngularFactor(btVector3(
//                 config.angularFactor.x,
//                 config.angularFactor.y,
//                 config.angularFactor.z
//             ));
//
//             // Configure CCD if enabled
//             if (config.enableCCD && mass > 0.0f) {
//                 bulletBody_->setCcdMotionThreshold(constants::CCD_MOTION_THRESHOLD);
//                 bulletBody_->setCcdSweptSphereRadius(constants::CCD_SWEPT_SPHERE_RADIUS);
//             }
//
//             // Set activation state
//             if (config.initialState != ActivationState::ACTIVE_STATE) {
//                 setActivationState(config.initialState);
//             }
//
//             // Disable sleeping if requested
//             if (!config.allowSleeping) {
//                 bulletBody_->setActivationState(DISABLE_DEACTIVATION);
//             }
//
//             // Set user pointer for callbacks
//             bulletBody_->setUserPointer(this);
//
//             std::cout << "[Physics] Created rigid body (ID: " << id
//                 << ", type: " << static_cast<int>(config.type)
//                 << ", mass: " << mass << ")" << std::endl;
//         }
//     }
//
//     /**
//      * @brief Destructor
//      * IMPLEMENTATION: Proper cleanup of Bullet resources
//      */
//     RigidBody::~RigidBody() {
//         // Bullet objects are automatically cleaned up by unique_ptr
//         std::cout << "[Physics] Destroyed rigid body (ID: " << id_ << ")" << std::endl;
//     }
//
//     /**
//      * @brief Get world transform
//      * IMPLEMENTATION: Retrieves current transform from Bullet
//      */
//     Transform3D RigidBody::getTransform() const {
//         Transform3D transform;
//
//         if (bulletBody_) {
//             btTransform btTrans;
//
//             // Get interpolated transform for smooth rendering
//             if (motionState_) {
//                 motionState_->getWorldTransform(btTrans);
//             }
//             else {
//                 btTrans = bulletBody_->getWorldTransform();
//             }
//
//             // Convert to our transform
//             const btVector3& origin = btTrans.getOrigin();
//             const btQuaternion& rotation = btTrans.getRotation();
//
//             transform.position = Vector3(origin.x(), origin.y(), origin.z());
//             transform.rotation = Quaternion(rotation.w(), rotation.x(), rotation.y(), rotation.z());
//         }
//         else if (ghostObject_) {
//             const btTransform& btTrans = ghostObject_->getWorldTransform();
//             const btVector3& origin = btTrans.getOrigin();
//             const btQuaternion& rotation = btTrans.getRotation();
//
//             transform.position = Vector3(origin.x(), origin.y(), origin.z());
//             transform.rotation = Quaternion(rotation.w(), rotation.x(), rotation.y(), rotation.z());
//         }
//
//         return transform;
//     }
//
//     /**
//      * @brief Set world transform
//      * IMPLEMENTATION: Updates Bullet body transform
//      */
//     void RigidBody::setTransform(const Transform3D& transform) {
//         btTransform btTrans;
//         btTrans.setOrigin(btVector3(transform.position.x, transform.position.y, transform.position.z));
//         btTrans.setRotation(btQuaternion(transform.rotation.x, transform.rotation.y,
//                                          transform.rotation.z, transform.rotation.w));
//
//         if (bulletBody_) {
//             // Clear velocities to prevent unwanted motion
//             bulletBody_->setLinearVelocity(btVector3(0, 0, 0));
//             bulletBody_->setAngularVelocity(btVector3(0, 0, 0));
//
//             // Set transform
//             bulletBody_->setWorldTransform(btTrans);
//
//             // Update motion state for interpolation
//             if (motionState_) {
//                 motionState_->setWorldTransform(btTrans);
//             }
//
//             // Activate body to ensure changes take effect
//             bulletBody_->activate(true);
//         }
//         else if (ghostObject_) {
//             ghostObject_->setWorldTransform(btTrans);
//         }
//     }
//
//     /**
//      * @brief Get world position
//      * IMPLEMENTATION: Quick position access
//      */
//     Vector3 RigidBody::getPosition() const {
//         if (bulletBody_) {
//             const btVector3& origin = bulletBody_->getWorldTransform().getOrigin();
//             return Vector3(origin.x(), origin.y(), origin.z());
//         }
//         else if (ghostObject_) {
//             const btVector3& origin = ghostObject_->getWorldTransform().getOrigin();
//             return Vector3(origin.x(), origin.y(), origin.z());
//         }
//         return Vector3(0.0f, 0.0f, 0.0f);
//     }
//
//     /**
//      * @brief Set world position
//      * IMPLEMENTATION: Updates position while preserving rotation
//      */
//     void RigidBody::setPosition(const Vector3& position) {
//         if (bulletBody_) {
//             btTransform transform = bulletBody_->getWorldTransform();
//             transform.setOrigin(btVector3(position.x, position.y, position.z));
//             bulletBody_->setWorldTransform(transform);
//
//             if (motionState_) {
//                 motionState_->setWorldTransform(transform);
//             }
//
//             bulletBody_->activate(true);
//         }
//         else if (ghostObject_) {
//             btTransform transform = ghostObject_->getWorldTransform();
//             transform.setOrigin(btVector3(position.x, position.y, position.z));
//             ghostObject_->setWorldTransform(transform);
//         }
//     }
//
//     /**
//      * @brief Get world rotation
//      * IMPLEMENTATION: Returns quaternion rotation
//      */
//     Quaternion RigidBody::getRotation() const {
//         if (bulletBody_) {
//             const btQuaternion& rot = bulletBody_->getWorldTransform().getRotation();
//             return Quaternion(rot.w(), rot.x(), rot.y(), rot.z());
//         }
//         else if (ghostObject_) {
//             const btQuaternion& rot = ghostObject_->getWorldTransform().getRotation();
//             return Quaternion(rot.w(), rot.x(), rot.y(), rot.z());
//         }
//         return Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
//     }
//
//     /**
//      * @brief Set world rotation
//      * IMPLEMENTATION: Updates rotation while preserving position
//      */
//     void RigidBody::setRotation(const Quaternion& rotation) {
//         if (bulletBody_) {
//             btTransform transform = bulletBody_->getWorldTransform();
//             transform.setRotation(btQuaternion(rotation.x, rotation.y, rotation.z, rotation.w));
//             bulletBody_->setWorldTransform(transform);
//
//             if (motionState_) {
//                 motionState_->setWorldTransform(transform);
//             }
//
//             bulletBody_->activate(true);
//         }
//         else if (ghostObject_) {
//             btTransform transform = ghostObject_->getWorldTransform();
//             transform.setRotation(btQuaternion(rotation.x, rotation.y, rotation.z, rotation.w));
//             ghostObject_->setWorldTransform(transform);
//         }
//     }
//
//     /**
//      * @brief Get linear velocity
//      * IMPLEMENTATION: Returns current movement velocity
//      */
//     Vector3 RigidBody::getLinearVelocity() const {
//         if (bulletBody_) {
//             const btVector3& vel = bulletBody_->getLinearVelocity();
//             return Vector3(vel.x(), vel.y(), vel.z());
//         }
//         return Vector3(0.0f, 0.0f, 0.0f);
//     }
//
//     /**
//      * @brief Set linear velocity
//      * IMPLEMENTATION: Directly sets movement velocity
//      */
//     void RigidBody::setLinearVelocity(const Vector3& velocity) {
//         if (bulletBody_ && type_ != BodyType::STATIC) {
//             bulletBody_->setLinearVelocity(btVector3(velocity.x, velocity.y, velocity.z));
//             bulletBody_->activate(true);
//         }
//     }
//
//     /**
//      * @brief Get angular velocity
//      * IMPLEMENTATION: Returns rotational velocity in radians/second
//      */
//     Vector3 RigidBody::getAngularVelocity() const {
//         if (bulletBody_) {
//             const btVector3& vel = bulletBody_->getAngularVelocity();
//             return Vector3(vel.x(), vel.y(), vel.z());
//         }
//         return Vector3(0.0f, 0.0f, 0.0f);
//     }
//
//     /**
//      * @brief Set angular velocity
//      * IMPLEMENTATION: Directly sets rotational velocity
//      */
//     void RigidBody::setAngularVelocity(const Vector3& velocity) {
//         if (bulletBody_ && type_ != BodyType::STATIC) {
//             bulletBody_->setAngularVelocity(btVector3(velocity.x, velocity.y, velocity.z));
//             bulletBody_->activate(true);
//         }
//     }
//
//     /**
//      * @brief Apply force at center of mass
//      * IMPLEMENTATION: Accumulates force for next simulation step
//      */
//     void RigidBody::applyForce(const Vector3& force) {
//         if (bulletBody_ && type_ == BodyType::DYNAMIC) {
//             bulletBody_->applyCentralForce(btVector3(force.x, force.y, force.z));
//             bulletBody_->activate(true);
//         }
//     }
//
//     /**
//      * @brief Apply force at specific point
//      * IMPLEMENTATION: Creates torque based on force position
//      */
//     void RigidBody::applyForce(const Vector3& force, const Vector3& relativePosition) {
//         if (bulletBody_ && type_ == BodyType::DYNAMIC) {
//             bulletBody_->applyForce(
//                 btVector3(force.x, force.y, force.z),
//                 btVector3(relativePosition.x, relativePosition.y, relativePosition.z)
//             );
//             bulletBody_->activate(true);
//         }
//     }
//
//     /**
//      * @brief Apply torque
//      * IMPLEMENTATION: Accumulates rotational force
//      */
//     void RigidBody::applyTorque(const Vector3& torque) {
//         if (bulletBody_ && type_ == BodyType::DYNAMIC) {
//             bulletBody_->applyTorque(btVector3(torque.x, torque.y, torque.z));
//             bulletBody_->activate(true);
//         }
//     }
//
//     /**
//      * @brief Apply impulse at center of mass
//      * IMPLEMENTATION: Instant velocity change
//      */
//     void RigidBody::applyImpulse(const Vector3& impulse) {
//         if (bulletBody_ && type_ == BodyType::DYNAMIC) {
//             bulletBody_->applyCentralImpulse(btVector3(impulse.x, impulse.y, impulse.z));
//             bulletBody_->activate(true);
//         }
//     }
//
//     /**
//      * @brief Apply impulse at specific point
//      * IMPLEMENTATION: Instant velocity and angular velocity change
//      */
//     void RigidBody::applyImpulse(const Vector3& impulse, const Vector3& relativePosition) {
//         if (bulletBody_ && type_ == BodyType::DYNAMIC) {
//             bulletBody_->applyImpulse(
//                 btVector3(impulse.x, impulse.y, impulse.z),
//                 btVector3(relativePosition.x, relativePosition.y, relativePosition.z)
//             );
//             bulletBody_->activate(true);
//         }
//     }
//
//     /**
//      * @brief Apply torque impulse
//      * IMPLEMENTATION: Instant angular velocity change
//      */
//     void RigidBody::applyTorqueImpulse(const Vector3& torqueImpulse) {
//         if (bulletBody_ && type_ == BodyType::DYNAMIC) {
//             bulletBody_->applyTorqueImpulse(btVector3(torqueImpulse.x, torqueImpulse.y, torqueImpulse.z));
//             bulletBody_->activate(true);
//         }
//     }
//
//     /**
//      * @brief Clear all forces and torques
//      * IMPLEMENTATION: Resets accumulated forces
//      */
//     void RigidBody::clearForces() {
//         if (bulletBody_) {
//             bulletBody_->clearForces();
//         }
//     }
//
//     /**
//      * @brief Get mass
//      * IMPLEMENTATION: Returns body mass (0 for static bodies)
//      */
//     float RigidBody::getMass() const {
//         if (bulletBody_) {
//             float invMass = bulletBody_->getInvMass();
//             return (invMass > 0.0f) ? (1.0f / invMass) : 0.0f;
//         }
//         return 0.0f;
//     }
//
//     /**
//      * @brief Set mass
//      * IMPLEMENTATION: Updates mass and recalculates inertia
//      */
//     void RigidBody::setMass(float mass) {
//         if (bulletBody_ && type_ == BodyType::DYNAMIC && mass > 0.0f) {
//             // Calculate new inertia
//             btVector3 localInertia(0, 0, 0);
//             bulletBody_->getCollisionShape()->calculateLocalInertia(mass, localInertia);
//
//             // Update mass and inertia
//             bulletBody_->setMassProps(mass, localInertia);
//
//             // Update the inverse mass and inertia tensors
//             bulletBody_->updateInertiaTensor();
//
//             std::cout << "[Physics] Updated body mass (ID: " << id_ << ", mass: " << mass << ")" << std::endl;
//         }
//     }
//
//     /**
//      * @brief Get friction coefficient
//      * IMPLEMENTATION: Returns surface friction
//      */
//     float RigidBody::getFriction() const {
//         if (bulletBody_) {
//             return bulletBody_->getFriction();
//         }
//         else if (ghostObject_) {
//             return ghostObject_->getFriction();
//         }
//         return 0.0f;
//     }
//
//     /**
//      * @brief Set friction coefficient
//      * IMPLEMENTATION: Updates surface friction
//      */
//     void RigidBody::setFriction(float friction) {
//         if (bulletBody_) {
//             bulletBody_->setFriction(friction);
//         }
//         else if (ghostObject_) {
//             ghostObject_->setFriction(friction);
//         }
//     }
//
//     /**
//      * @brief Get restitution
//      * IMPLEMENTATION: Returns bounciness factor
//      */
//     float RigidBody::getRestitution() const {
//         if (bulletBody_) {
//             return bulletBody_->getRestitution();
//         }
//         else if (ghostObject_) {
//             return ghostObject_->getRestitution();
//         }
//         return 0.0f;
//     }
//
//     /**
//      * @brief Set restitution
//      * IMPLEMENTATION: Updates bounciness
//      */
//     void RigidBody::setRestitution(float restitution) {
//         if (bulletBody_) {
//             bulletBody_->setRestitution(restitution);
//         }
//         else if (ghostObject_) {
//             ghostObject_->setRestitution(restitution);
//         }
//     }
//
//     /**
//      * @brief Get linear damping
//      * IMPLEMENTATION: Returns air resistance for linear motion
//      */
//     float RigidBody::getLinearDamping() const {
//         if (bulletBody_) {
//             return bulletBody_->getLinearDamping();
//         }
//         return 0.0f;
//     }
//
//     /**
//      * @brief Set linear damping
//      * IMPLEMENTATION: Updates air resistance
//      */
//     void RigidBody::setLinearDamping(float damping) {
//         if (bulletBody_ && type_ != BodyType::STATIC) {
//             float angularDamping = bulletBody_->getAngularDamping();
//             bulletBody_->setDamping(damping, angularDamping);
//         }
//     }
//
//     /**
//      * @brief Get angular damping
//      * IMPLEMENTATION: Returns rotational resistance
//      */
//     float RigidBody::getAngularDamping() const {
//         if (bulletBody_) {
//             return bulletBody_->getAngularDamping();
//         }
//         return 0.0f;
//     }
//
//     /**
//      * @brief Set angular damping
//      * IMPLEMENTATION: Updates rotational resistance
//      */
//     void RigidBody::setAngularDamping(float damping) {
//         if (bulletBody_ && type_ != BodyType::STATIC) {
//             float linearDamping = bulletBody_->getLinearDamping();
//             bulletBody_->setDamping(linearDamping, damping);
//         }
//     }
//
//     /**
//      * @brief Get activation state
//      * IMPLEMENTATION: Returns current activity state
//      */
//     ActivationState RigidBody::getActivationState() const {
//         if (bulletBody_) {
//             int state = bulletBody_->getActivationState();
//             return static_cast<ActivationState>(state);
//         }
//         return ActivationState::ACTIVE_STATE;
//     }
//
//     /**
//      * @brief Set activation state
//      * IMPLEMENTATION: Controls body sleeping/activity
//      */
//     void RigidBody::setActivationState(ActivationState state) {
//         if (bulletBody_) {
//             bulletBody_->setActivationState(static_cast<int>(state));
//         }
//     }
//
//     /**
//      * @brief Check if body is active
//      * IMPLEMENTATION: Returns true if not sleeping
//      */
//     bool RigidBody::isActive() const {
//         if (bulletBody_) {
//             return bulletBody_->isActive();
//         }
//         return ghostObject_ != nullptr;
//     }
//
//     /**
//      * @brief Activate the body
//      * IMPLEMENTATION: Wakes up sleeping body
//      */
//     void RigidBody::activate(bool forceActivation) {
//         if (bulletBody_) {
//             bulletBody_->activate(forceActivation);
//         }
//     }
//
//     /**
//      * @brief Check if body is sleeping
//      * IMPLEMENTATION: Returns true if deactivated for optimization
//      */
//     bool RigidBody::isSleeping() const {
//         if (bulletBody_) {
//             return !bulletBody_->isActive();
//         }
//         return false;
//     }
//
//     /**
//      * @brief Set collision filter
//      * IMPLEMENTATION: Updates collision layer and mask
//      */
//     void RigidBody::setCollisionFilter(const CollisionFilter& filter) {
//         collisionFilter_ = filter;
//
//         // Note: Actual filter application happens when body is added to world
//         // This is handled by PhysicsManager
//     }
//
//     /**
//      * @brief Enable/disable collision response
//      * IMPLEMENTATION: Makes body a sensor/trigger
//      */
//     void RigidBody::setCollisionResponseEnabled(bool enabled) {
//         if (bulletBody_) {
//             int flags = bulletBody_->getCollisionFlags();
//             if (enabled) {
//                 flags &= ~btCollisionObject::CF_NO_CONTACT_RESPONSE;
//             }
//             else {
//                 flags |= btCollisionObject::CF_NO_CONTACT_RESPONSE;
//             }
//             bulletBody_->setCollisionFlags(flags);
//         }
//         else if (ghostObject_) {
//             int flags = ghostObject_->getCollisionFlags();
//             if (enabled) {
//                 flags &= ~btCollisionObject::CF_NO_CONTACT_RESPONSE;
//             }
//             else {
//                 flags |= btCollisionObject::CF_NO_CONTACT_RESPONSE;
//             }
//             ghostObject_->setCollisionFlags(flags);
//         }
//     }
//
//     /**
//      * @brief Set linear factor
//      * IMPLEMENTATION: Constrains linear motion axes
//      */
//     void RigidBody::setLinearFactor(const Vector3& factor) {
//         if (bulletBody_) {
//             bulletBody_->setLinearFactor(btVector3(factor.x, factor.y, factor.z));
//         }
//     }
//
//     /**
//      * @brief Get linear factor
//      * IMPLEMENTATION: Returns linear motion constraints
//      */
//     Vector3 RigidBody::getLinearFactor() const {
//         if (bulletBody_) {
//             const btVector3& factor = bulletBody_->getLinearFactor();
//             return Vector3(factor.x(), factor.y(), factor.z());
//         }
//         return Vector3(1.0f, 1.0f, 1.0f);
//     }
//
//     /**
//      * @brief Set angular factor
//      * IMPLEMENTATION: Constrains rotational axes
//      */
//     void RigidBody::setAngularFactor(const Vector3& factor) {
//         if (bulletBody_) {
//             bulletBody_->setAngularFactor(btVector3(factor.x, factor.y, factor.z));
//         }
//     }
//
//     /**
//      * @brief Get angular factor
//      * IMPLEMENTATION: Returns rotational constraints
//      */
//     Vector3 RigidBody::getAngularFactor() const {
//         if (bulletBody_) {
//             const btVector3& factor = bulletBody_->getAngularFactor();
//             return Vector3(factor.x(), factor.y(), factor.z());
//         }
//         return Vector3(1.0f, 1.0f, 1.0f);
//     }
//
//     /**
//      * @brief Enable/disable CCD
//      * IMPLEMENTATION: Continuous collision detection for fast objects
//      */
//     void RigidBody::setCCDEnabled(bool enabled) {
//         if (bulletBody_ && type_ == BodyType::DYNAMIC) {
//             if (enabled) {
//                 setCCDMotionThreshold(constants::CCD_MOTION_THRESHOLD);
//                 setCCDSweptSphereRadius(constants::CCD_SWEPT_SPHERE_RADIUS);
//             }
//             else {
//                 setCCDMotionThreshold(0.0f);
//                 setCCDSweptSphereRadius(0.0f);
//             }
//         }
//     }
//
//     /**
//      * @brief Set CCD motion threshold
//      * IMPLEMENTATION: Speed threshold for CCD activation
//      */
//     void RigidBody::setCCDMotionThreshold(float threshold) {
//         if (bulletBody_) {
//             bulletBody_->setCcdMotionThreshold(threshold);
//         }
//     }
//
//     /**
//      * @brief Set CCD swept sphere radius
//      * IMPLEMENTATION: Sphere size for continuous collision
//      */
//     void RigidBody::setCCDSweptSphereRadius(float radius) {
//         if (bulletBody_) {
//             bulletBody_->setCcdSweptSphereRadius(radius);
//         }
//     }
//
//     /**
//      * @brief Get AABB
//      * IMPLEMENTATION: Returns axis-aligned bounding box
//      */
//     std::pair<Vector3, Vector3> RigidBody::getAABB() const {
//         btVector3 aabbMin, aabbMax;
//
//         if (bulletBody_) {
//             bulletBody_->getAabb(aabbMin, aabbMax);
//         }
//         else if (ghostObject_) {
//             ghostObject_->getCollisionShape()->getAabb(
//                 ghostObject_->getWorldTransform(), aabbMin, aabbMax
//             );
//         }
//         else {
//             return {Vector3(0.0f), Vector3(0.0f)};
//         }
//
//         return {
//             Vector3(aabbMin.x(), aabbMin.y(), aabbMin.z()),
//             Vector3(aabbMax.x(), aabbMax.y(), aabbMax.z())
//         };
//     }
//
//     /**
//      * @brief Get total energy
//      * IMPLEMENTATION: Calculates kinetic + potential energy
//      */
//     float RigidBody::getTotalEnergy(const Vector3& gravity) const {
//         if (!bulletBody_ || type_ != BodyType::DYNAMIC) {
//             return 0.0f;
//         }
//
//         float mass = getMass();
//         if (mass <= 0.0f) {
//             return 0.0f;
//         }
//
//         // Kinetic energy = 0.5 * m * v^2
//         Vector3 linearVel = getLinearVelocity();
//         float linearKE = 0.5f * mass * glm::dot(linearVel, linearVel);
//
//         // Rotational kinetic energy = 0.5 * I * w^2
//         Vector3 angularVel = getAngularVelocity();
//         const btVector3& inertia = bulletBody_->getLocalInertia();
//         float rotationalKE = 0.5f * (
//             inertia.x() * angularVel.x * angularVel.x +
//             inertia.y() * angularVel.y * angularVel.y +
//             inertia.z() * angularVel.z * angularVel.z
//         );
//
//         // Potential energy = m * g * h
//         Vector3 position = getPosition();
//         float potentialEnergy = mass * glm::length(gravity) * position.y;
//
//         return linearKE + rotationalKE + potentialEnergy;
//     }
//
//     /**
//      * @brief Reset body to initial state
//      * IMPLEMENTATION: Restores original transform and velocities
//      */
//     void RigidBody::reset() {
//         setTransform(initialTransform_);
//         setLinearVelocity(initialLinearVelocity_);
//         setAngularVelocity(initialAngularVelocity_);
//         clearForces();
//         activate(true);
//
//         std::cout << "[Physics] Reset rigid body to initial state (ID: " << id_ << ")" << std::endl;
//     }
//
//
//     // ========================================================================
//     // PHYSICS CONSTRAINT IMPLEMENTATION
//     // ========================================================================
//
//     /**
//  * @brief Constructor for PhysicsConstraint
//  * IMPLEMENTATION: Creates various types of Bullet constraints based on configuration
//  */
//     PhysicsConstraint::PhysicsConstraint(ConstraintID id, const ConstraintConfig& config,
//                                          RigidBody* bodyA, RigidBody* bodyB)
//         : id_(id)
//           , type_(config.type)
//           , bodyA_(bodyA ? bodyA->getId() : INVALID_BODY_ID)
//           , bodyB_(bodyB ? bodyB->getId() : INVALID_BODY_ID) {
//         if (!bodyA || !bodyA->getBulletBody()) {
//             throw std::runtime_error("[Physics] Constraint requires at least one valid rigid body");
//         }
//
//         btRigidBody* btBodyA = bodyA->getBulletBody();
//         btRigidBody* btBodyB = bodyB ? bodyB->getBulletBody() : nullptr;
//
//         // Convert transforms to Bullet
//         btTransform frameInA, frameInB;
//         frameInA.setOrigin(
//             btVector3(config.frameInA.position.x, config.frameInA.position.y, config.frameInA.position.z));
//         frameInA.setRotation(btQuaternion(config.frameInA.rotation.x, config.frameInA.rotation.y,
//                                           config.frameInA.rotation.z, config.frameInA.rotation.w));
//
//         if (btBodyB) {
//             frameInB.setOrigin(btVector3(config.frameInB.position.x, config.frameInB.position.y,
//                                          config.frameInB.position.z));
//             frameInB.setRotation(btQuaternion(config.frameInB.rotation.x, config.frameInB.rotation.y,
//                                               config.frameInB.rotation.z, config.frameInB.rotation.w));
//         }
//
//         // Create constraint based on type
//         switch (config.type) {
//         case ConstraintType::POINT_TO_POINT: {
//             // Ball socket joint - 3 DOF rotation, no translation
//             if (btBodyB) {
//                 constraint_ = std::make_unique<btPoint2PointConstraint>(
//                     *btBodyA, *btBodyB,
//                     frameInA.getOrigin(), frameInB.getOrigin()
//                 );
//             }
//             else {
//                 constraint_ = std::make_unique<btPoint2PointConstraint>(
//                     *btBodyA, frameInA.getOrigin()
//                 );
//             }
//
//             std::cout << "[Physics] Created point-to-point constraint (ID: " << id << ")" << std::endl;
//             break;
//         }
//
//         case ConstraintType::HINGE: {
//             // Hinge joint - 1 DOF rotation around axis
//             if (btBodyB) {
//                 auto* hinge = new btHingeConstraint(*btBodyA, *btBodyB, frameInA, frameInB);
//
//                 // Set limits if specified
//                 if (config.angularLowerLimit.x != config.angularUpperLimit.x) {
//                     hinge->setLimit(config.angularLowerLimit.x, config.angularUpperLimit.x);
//                 }
//
//                 // Configure motor if enabled
//                 if (config.enableMotor) {
//                     hinge->enableAngularMotor(true, 0.0f, config.maxMotorImpulse);
//                 }
//
//                 constraint_.reset(hinge);
//             }
//             else {
//                 constraint_ = std::make_unique<btHingeConstraint>(*btBodyA, frameInA);
//             }
//
//             std::cout << "[Physics] Created hinge constraint (ID: " << id << ")" << std::endl;
//             break;
//         }
//
//         case ConstraintType::SLIDER: {
//             // Slider/prismatic joint - 1 DOF translation along axis
//             if (btBodyB) {
//                 auto* slider = new btSliderConstraint(*btBodyA, *btBodyB, frameInA, frameInB, true);
//
//                 // Set linear limits
//                 slider->setLowerLinLimit(config.linearLowerLimit.x);
//                 slider->setUpperLinLimit(config.linearUpperLimit.x);
//
//                 // Set angular limits (usually locked for pure sliding)
//                 slider->setLowerAngLimit(config.angularLowerLimit.x);
//                 slider->setUpperAngLimit(config.angularUpperLimit.x);
//
//                 // Configure motor if enabled
//                 if (config.enableMotor) {
//                     slider->setPoweredLinMotor(true);
//                     slider->setMaxLinMotorForce(config.maxMotorImpulse);
//                 }
//
//                 constraint_.reset(slider);
//             }
//             else {
//                 throw std::runtime_error("[Physics] Slider constraint requires two bodies");
//             }
//
//             std::cout << "[Physics] Created slider constraint (ID: " << id << ")" << std::endl;
//             break;
//         }
//
//         case ConstraintType::CONE_TWIST: {
//             // Cone twist joint - like a shoulder joint
//             if (btBodyB) {
//                 auto* coneTwist = new btConeTwistConstraint(*btBodyA, *btBodyB, frameInA, frameInB);
//
//                 // Set swing and twist limits
//                 coneTwist->setLimit(
//                     config.angularUpperLimit.x, // Swing span 1
//                     config.angularUpperLimit.y, // Swing span 2
//                     config.angularUpperLimit.z, // Twist span
//                     config.linearStiffness.x, // Softness
//                     config.linearStiffness.y, // Bias factor
//                     config.linearStiffness.z // Relaxation factor
//                 );
//
//                 constraint_.reset(coneTwist);
//             }
//             else {
//                 constraint_ = std::make_unique<btConeTwistConstraint>(*btBodyA, frameInA);
//             }
//
//             std::cout << "[Physics] Created cone-twist constraint (ID: " << id << ")" << std::endl;
//             break;
//         }
//
//         case ConstraintType::GENERIC_6DOF: {
//             // Generic 6 degrees of freedom joint
//             if (btBodyB) {
//                 auto* generic = new btGeneric6DofConstraint(
//                     *btBodyA, *btBodyB, frameInA, frameInB, true
//                 );
//
//                 // Set linear limits for each axis
//                 generic->setLinearLowerLimit(btVector3(
//                     config.linearLowerLimit.x,
//                     config.linearLowerLimit.y,
//                     config.linearLowerLimit.z
//                 ));
//                 generic->setLinearUpperLimit(btVector3(
//                     config.linearUpperLimit.x,
//                     config.linearUpperLimit.y,
//                     config.linearUpperLimit.z
//                 ));
//
//                 // Set angular limits for each axis
//                 generic->setAngularLowerLimit(btVector3(
//                     config.angularLowerLimit.x,
//                     config.angularLowerLimit.y,
//                     config.angularLowerLimit.z
//                 ));
//                 generic->setAngularUpperLimit(btVector3(
//                     config.angularUpperLimit.x,
//                     config.angularUpperLimit.y,
//                     config.angularUpperLimit.z
//                 ));
//
//                 constraint_.reset(generic);
//             }
//             else {
//                 throw std::runtime_error("[Physics] Generic 6DOF constraint requires two bodies");
//             }
//
//             std::cout << "[Physics] Created generic 6DOF constraint (ID: " << id << ")" << std::endl;
//             break;
//         }
//
//         case ConstraintType::GENERIC_6DOF_SPRING: {
//             // 6DOF with spring/damper on each axis
//             if (btBodyB) {
//                 auto* spring = new btGeneric6DofSpringConstraint(
//                     *btBodyA, *btBodyB, frameInA, frameInB, true
//                 );
//
//                 // Set linear limits
//                 spring->setLinearLowerLimit(btVector3(
//                     config.linearLowerLimit.x,
//                     config.linearLowerLimit.y,
//                     config.linearLowerLimit.z
//                 ));
//                 spring->setLinearUpperLimit(btVector3(
//                     config.linearUpperLimit.x,
//                     config.linearUpperLimit.y,
//                     config.linearUpperLimit.z
//                 ));
//
//                 // Set angular limits
//                 spring->setAngularLowerLimit(btVector3(
//                     config.angularLowerLimit.x,
//                     config.angularLowerLimit.y,
//                     config.angularLowerLimit.z
//                 ));
//                 spring->setAngularUpperLimit(btVector3(
//                     config.angularUpperLimit.x,
//                     config.angularUpperLimit.y,
//                     config.angularUpperLimit.z
//                 ));
//
//                 // Configure springs for linear axes (indices 0, 1, 2)
//                 for (int i = 0; i < 3; ++i) {
//                     if (config.linearStiffness[i] > 0.0f) {
//                         spring->enableSpring(i, true);
//                         spring->setStiffness(i, config.linearStiffness[i]);
//                         spring->setDamping(i, config.linearDamping[i]);
//                     }
//                 }
//
//                 // Configure springs for angular axes (indices 3, 4, 5)
//                 for (int i = 0; i < 3; ++i) {
//                     if (config.angularStiffness[i] > 0.0f) {
//                         spring->enableSpring(i + 3, true);
//                         spring->setStiffness(i + 3, config.angularStiffness[i]);
//                         spring->setDamping(i + 3, config.angularDamping[i]);
//                     }
//                 }
//
//                 constraint_.reset(spring);
//             }
//             else {
//                 throw std::runtime_error("[Physics] Spring 6DOF constraint requires two bodies");
//             }
//
//             std::cout << "[Physics] Created 6DOF spring constraint (ID: " << id << ")" << std::endl;
//             break;
//         }
//
//         case ConstraintType::FIXED: {
//             // Fixed/weld joint - no relative motion
//             if (btBodyB) {
//                 auto* fixed = new btFixedConstraint(*btBodyA, *btBodyB, frameInA, frameInB);
//                 constraint_.reset(fixed);
//             }
//             else {
//                 // Fix to world
//                 auto* generic = new btGeneric6DofConstraint(*btBodyA, frameInA, false);
//
//                 // Lock all axes
//                 generic->setLinearLowerLimit(btVector3(0, 0, 0));
//                 generic->setLinearUpperLimit(btVector3(0, 0, 0));
//                 generic->setAngularLowerLimit(btVector3(0, 0, 0));
//                 generic->setAngularUpperLimit(btVector3(0, 0, 0));
//
//                 constraint_.reset(generic);
//             }
//
//             std::cout << "[Physics] Created fixed constraint (ID: " << id << ")" << std::endl;
//             break;
//         }
//
//         case ConstraintType::GEAR: {
//             // Gear constraint - relates angular velocities
//             if (btBodyB) {
//                 auto* gear = new btGearConstraint(*btBodyA, *btBodyB,
//                                                   frameInA.getBasis().getColumn(0), // Axis A
//                                                   frameInB.getBasis().getColumn(0), // Axis B
//                                                   config.linearStiffness.x); // Gear ratio
//                 constraint_.reset(gear);
//             }
//             else {
//                 throw std::runtime_error("[Physics] Gear constraint requires two bodies");
//             }
//
//             std::cout << "[Physics] Created gear constraint (ID: " << id << ")" << std::endl;
//             break;
//         }
//
//         default:
//             throw std::runtime_error("[Physics] Unknown constraint type");
//         }
//
//         // Apply common constraint settings
//         if (constraint_) {
//             // Set breaking impulse threshold
//             if (config.breakingImpulseThreshold < INFINITY) {
//                 constraint_->setBreakingImpulseThreshold(config.breakingImpulseThreshold);
//             }
//
//             // Disable collision between connected bodies if requested
//             if (config.disableCollisionsBetweenLinkedBodies) {
//                 constraint_->setDbgDrawSize(2.0f); // For debug drawing
//             }
//         }
//     }
//
//     /**
//      * @brief Destructor
//      * IMPLEMENTATION: Cleanup constraint resources
//      */
//     PhysicsConstraint::~PhysicsConstraint() {
//         // Constraint is automatically cleaned up by unique_ptr
//         std::cout << "[Physics] Destroyed constraint (ID: " << id_ << ")" << std::endl;
//     }
//
//     /**
//      * @brief Get connected bodies
//      * IMPLEMENTATION: Returns the IDs of connected rigid bodies
//      */
//     std::pair<RigidBodyID, RigidBodyID> PhysicsConstraint::getConnectedBodies() const {
//         return {bodyA_, bodyB_};
//     }
//
//     /**
//      * @brief Enable/disable constraint
//      * IMPLEMENTATION: Temporarily enables or disables the constraint
//      */
//     void PhysicsConstraint::setEnabled(bool enabled) {
//         if (constraint_) {
//             constraint_->setEnabled(enabled);
//         }
//     }
//
//     /**
//      * @brief Check if constraint is enabled
//      * IMPLEMENTATION: Returns current enabled state
//      */
//     bool PhysicsConstraint::isEnabled() const {
//         if (constraint_) {
//             return constraint_->isEnabled();
//         }
//         return false;
//     }
//
//     /**
//      * @brief Set breaking impulse threshold
//      * IMPLEMENTATION: Force needed to break the constraint
//      */
//     void PhysicsConstraint::setBreakingImpulseThreshold(float threshold) {
//         if (constraint_) {
//             constraint_->setBreakingImpulseThreshold(threshold);
//         }
//     }
//
//     /**
//      * @brief Get breaking impulse threshold
//      * IMPLEMENTATION: Returns current breaking threshold
//      */
//     float PhysicsConstraint::getBreakingImpulseThreshold() const {
//         if (constraint_) {
//             return constraint_->getBreakingImpulseThreshold();
//         }
//         return INFINITY;
//     }
//
//     /**
//      * @brief Check if constraint is broken
//      * IMPLEMENTATION: Returns true if constraint exceeded breaking threshold
//      */
//     bool PhysicsConstraint::isBroken() const {
//         if (!constraint_) {
//             return true;
//         }
//
//         // Check if impulse exceeded breaking threshold
//         float appliedImpulse = constraint_->getAppliedImpulse();
//         float breakingThreshold = constraint_->getBreakingImpulseThreshold();
//
//         if (breakingThreshold < INFINITY && appliedImpulse > breakingThreshold) {
//             const_cast<PhysicsConstraint*>(this)->isBroken_ = true;
//         }
//
//         return isBroken_;
//     }
//
//     /**
//      * @brief Get applied impulse
//      * IMPLEMENTATION: Returns force currently applied by constraint
//      */
//     float PhysicsConstraint::getAppliedImpulse() const {
//         if (constraint_) {
//             return constraint_->getAppliedImpulse();
//         }
//         return 0.0f;
//     }
//
//     /**
//      * @brief Enable/disable motor
//      * IMPLEMENTATION: Controls motorized constraints
//      */
//     void PhysicsConstraint::setMotorEnabled(bool enabled) {
//         if (!constraint_) return;
//
//         switch (type_) {
//         case ConstraintType::HINGE: {
//             auto* hinge = static_cast<btHingeConstraint*>(constraint_.get());
//             hinge->enableAngularMotor(enabled, hinge->getMotorTargetVelocity(), hinge->getMaxMotorImpulse());
//             break;
//         }
//
//         case ConstraintType::SLIDER: {
//             auto* slider = static_cast<btSliderConstraint*>(constraint_.get());
//             slider->setPoweredLinMotor(enabled);
//             break;
//         }
//
//         case ConstraintType::GENERIC_6DOF:
//         case ConstraintType::GENERIC_6DOF_SPRING: {
//             // 6DOF motors are configured per axis
//             auto* generic = static_cast<btGeneric6DofConstraint*>(constraint_.get());
//             for (int i = 0; i < 3; ++i) {
//                 generic->getTranslationalLimitMotor()->m_enableMotor[i] = enabled;
//             }
//             for (int i = 0; i < 3; ++i) {
//                 generic->getRotationalLimitMotor(i)->m_enableMotor = enabled;
//             }
//             break;
//         }
//
//         default:
//             break;
//         }
//     }
//
//     /**
//      * @brief Set motor target velocity
//      * IMPLEMENTATION: Sets desired velocity for motorized constraints
//      */
//     void PhysicsConstraint::setMotorTargetVelocity(float velocity) {
//         if (!constraint_) return;
//
//         switch (type_) {
//         case ConstraintType::HINGE: {
//             auto* hinge = static_cast<btHingeConstraint*>(constraint_.get());
//             hinge->enableAngularMotor(true, velocity, hinge->getMaxMotorImpulse());
//             break;
//         }
//
//         case ConstraintType::SLIDER: {
//             auto* slider = static_cast<btSliderConstraint*>(constraint_.get());
//             slider->setTargetLinMotorVelocity(velocity);
//             break;
//         }
//
//         case ConstraintType::GENERIC_6DOF:
//         case ConstraintType::GENERIC_6DOF_SPRING: {
//             auto* generic = static_cast<btGeneric6DofConstraint*>(constraint_.get());
//             // Set for primary axis (usually Y for rotation)
//             generic->getRotationalLimitMotor(1)->m_targetVelocity = velocity;
//             break;
//         }
//
//         default:
//             break;
//         }
//     }
//
//     /**
//      * @brief Set maximum motor impulse
//      * IMPLEMENTATION: Sets motor strength/force
//      */
//     void PhysicsConstraint::setMaxMotorImpulse(float impulse) {
//         if (!constraint_) return;
//
//         switch (type_) {
//         case ConstraintType::HINGE: {
//             auto* hinge = static_cast<btHingeConstraint*>(constraint_.get());
//             hinge->setMaxMotorImpulse(impulse);
//             break;
//         }
//
//         case ConstraintType::SLIDER: {
//             auto* slider = static_cast<btSliderConstraint*>(constraint_.get());
//             slider->setMaxLinMotorForce(impulse);
//             break;
//         }
//
//         case ConstraintType::GENERIC_6DOF:
//         case ConstraintType::GENERIC_6DOF_SPRING: {
//             auto* generic = static_cast<btGeneric6DofConstraint*>(constraint_.get());
//             for (int i = 0; i < 3; ++i) {
//                 generic->getRotationalLimitMotor(i)->m_maxMotorForce = impulse;
//             }
//             break;
//         }
//
//         default:
//             break;
//         }
//     }
//
//     /**
//      * @brief Set linear limits
//      * IMPLEMENTATION: Constrains translation along axes
//      */
//     void PhysicsConstraint::setLinearLimits(const Vector3& lower, const Vector3& upper) {
//         if (!constraint_) return;
//
//         switch (type_) {
//         case ConstraintType::SLIDER: {
//             auto* slider = static_cast<btSliderConstraint*>(constraint_.get());
//             slider->setLowerLinLimit(lower.x);
//             slider->setUpperLinLimit(upper.x);
//             break;
//         }
//
//         case ConstraintType::GENERIC_6DOF:
//         case ConstraintType::GENERIC_6DOF_SPRING: {
//             auto* generic = static_cast<btGeneric6DofConstraint*>(constraint_.get());
//             generic->setLinearLowerLimit(btVector3(lower.x, lower.y, lower.z));
//             generic->setLinearUpperLimit(btVector3(upper.x, upper.y, upper.z));
//             break;
//         }
//
//         default:
//             std::cerr << "[Physics] Warning: Linear limits not supported for this constraint type" << std::endl;
//             break;
//         }
//     }
//
//     /**
//      * @brief Set angular limits
//      * IMPLEMENTATION: Constrains rotation around axes
//      */
//     void PhysicsConstraint::setAngularLimits(const Vector3& lower, const Vector3& upper) {
//         if (!constraint_) return;
//
//         switch (type_) {
//         case ConstraintType::HINGE: {
//             auto* hinge = static_cast<btHingeConstraint*>(constraint_.get());
//             hinge->setLimit(lower.x, upper.x);
//             break;
//         }
//
//         case ConstraintType::SLIDER: {
//             auto* slider = static_cast<btSliderConstraint*>(constraint_.get());
//             slider->setLowerAngLimit(lower.x);
//             slider->setUpperAngLimit(upper.x);
//             break;
//         }
//
//         case ConstraintType::CONE_TWIST: {
//             auto* coneTwist = static_cast<btConeTwistConstraint*>(constraint_.get());
//             coneTwist->setLimit(upper.x, upper.y, upper.z); // Swing1, Swing2, Twist
//             break;
//         }
//
//         case ConstraintType::GENERIC_6DOF:
//         case ConstraintType::GENERIC_6DOF_SPRING: {
//             auto* generic = static_cast<btGeneric6DofConstraint*>(constraint_.get());
//             generic->setAngularLowerLimit(btVector3(lower.x, lower.y, lower.z));
//             generic->setAngularUpperLimit(btVector3(upper.x, upper.y, upper.z));
//             break;
//         }
//
//         default:
//             std::cerr << "[Physics] Warning: Angular limits not supported for this constraint type" << std::endl;
//             break;
//         }
//     }
//
//     // ========================================================================
//     // COLLISION CALLBACK INTERNAL CLASS
//     // ========================================================================
//
//     /**
//      * @brief Internal collision callback handler for Bullet
//      * Processes collision events and triggers user callbacks
//      */
//     class CollisionCallbackHandler : public btCollisionWorld::ContactResultCallback {
//     public:
//         PhysicsManager* manager;
//         std::vector<CollisionManifold> manifolds;
//
//         CollisionCallbackHandler(PhysicsManager* mgr) : manager(mgr) {
//         }
//
//         btScalar addSingleResult(btManifoldPoint& cp,
//                                  const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0,
//                                  const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1) override {
//             // Get rigid bodies from user pointers
//             RigidBody* bodyA = static_cast<RigidBody*>(colObj0Wrap->getCollisionObject()->getUserPointer());
//             RigidBody* bodyB = static_cast<RigidBody*>(colObj1Wrap->getCollisionObject()->getUserPointer());
//
//             if (!bodyA || !bodyB) return 0;
//
//             // Create contact point
//             ContactPoint contact;
//             contact.worldPositionOnA = Vector3(cp.m_positionWorldOnA.x(), cp.m_positionWorldOnA.y(),
//                                                cp.m_positionWorldOnA.z());
//             contact.worldPositionOnB = Vector3(cp.m_positionWorldOnB.x(), cp.m_positionWorldOnB.y(),
//                                                cp.m_positionWorldOnB.z());
//             contact.worldNormalOnB = Vector3(cp.m_normalWorldOnB.x(), cp.m_normalWorldOnB.y(), cp.m_normalWorldOnB.z());
//             contact.penetrationDepth = cp.m_distance1;
//             contact.appliedImpulse = cp.m_appliedImpulse;
//             contact.combinedFriction = cp.m_combinedFriction;
//             contact.combinedRestitution = cp.m_combinedRestitution;
//             contact.lifetime = cp.m_lifeTime;
//
//             // Find or create manifold for this pair
//             CollisionPairID pairId = 0;
//             if (bodyA->getId() < bodyB->getId()) {
//                 pairId = (static_cast<CollisionPairID>(bodyA->getId()) << 32) | bodyB->getId();
//             }
//             else {
//                 pairId = (static_cast<CollisionPairID>(bodyB->getId()) << 32) | bodyA->getId();
//             }
//
//             // Add to manifolds
//             bool found = false;
//             for (auto& manifold : manifolds) {
//                 if (manifold.getPairID() == pairId) {
//                     manifold.contactPoints.push_back(contact);
//                     found = true;
//                     break;
//                 }
//             }
//
//             if (!found) {
//                 CollisionManifold manifold;
//                 manifold.bodyA = bodyA->getId();
//                 manifold.bodyB = bodyB->getId();
//                 manifold.contactPoints.push_back(contact);
//                 manifold.eventType = (cp.m_lifeTime == 0)
//                                          ? CollisionEventType::CONTACT_STARTED
//                                          : CollisionEventType::CONTACT_PERSISTED;
//                 manifolds.push_back(manifold);
//             }
//
//             return 0;
//         }
//     };
//
//     // ========================================================================
//     // PHYSICS DEBUG DRAWER
//     // ========================================================================
//
//     /**
//      * @brief Debug renderer for physics visualization
//      */
//     class PhysicsDebugDrawer : public btIDebugDraw {
//     private:
//         int debugMode_ = DBG_DrawWireframe | DBG_DrawConstraints | DBG_DrawConstraintLimits;
//         void* renderer_ = nullptr; // SDL_Renderer or other renderer
//
//     public:
//         PhysicsDebugDrawer() = default;
//
//         void setRenderer(void* renderer) { renderer_ = renderer; }
//
//         void drawLine(const btVector3& from, const btVector3& to, const btVector3& color) override {
//             // Implementation depends on rendering system
//             // This would draw lines using SDL_RenderDrawLine or similar
//         }
//
//         void drawContactPoint(const btVector3& pointOnB, const btVector3& normalOnB,
//                               btScalar distance, int lifeTime, const btVector3& color) override {
//             // Draw contact points for debugging
//         }
//
//         void reportErrorWarning(const char* warningString) override {
//             std::cerr << "[Physics Debug] " << warningString << std::endl;
//         }
//
//         void draw3dText(const btVector3& location, const char* textString) override {
//             // Draw text in 3D space (optional)
//         }
//
//         void setDebugMode(int debugMode) override { debugMode_ = debugMode; }
//         int getDebugMode() const override { return debugMode_; }
//     };
//
//     // ========================================================================
//     // PHYSICS MANAGER IMPLEMENTATION
//     // ========================================================================
//
//     /**
//      * @brief Constructor for PhysicsManager
//      * IMPLEMENTATION: Initializes configuration and prepares for physics setup
//      */
//     PhysicsManager::PhysicsManager(const PhysicsConfig& config)
//         : config_(config)
//           , initialized_(false)
//           , simulationSpeed_(1.0f)
//           , accumulator_(0.0f) {
//         std::cout << "[Physics] PhysicsManager created with configuration:" << std::endl;
//         std::cout << "  Gravity: " << config.gravity.x << ", " << config.gravity.y << ", " << config.gravity.z <<
//             std::endl;
//         std::cout << "  Fixed timestep: " << config.fixedTimeStep << "s (" << (1.0f / config.fixedTimeStep) << " Hz)" <<
//             std::endl;
//         std::cout << "  Max substeps: " << config.maxSubSteps << std::endl;
//         std::cout << "  Soft body support: " << (config.enableSoftBody ? "Yes" : "No") << std::endl;
//     }
//
//     /**
//      * @brief Destructor
//      * IMPLEMENTATION: Ensures proper cleanup of all physics resources
//      */
//     PhysicsManager::~PhysicsManager() {
//         shutdown();
//     }
//
//     /**
//      * @brief Initialize physics system
//      * IMPLEMENTATION: Sets up Bullet physics world with all components
//      */
//     bool PhysicsManager::initialize() {
//         if (initialized_) {
//             std::cout << "[Physics] Already initialized" << std::endl;
//             return true;
//         }
//
//         std::lock_guard<std::mutex> lock(physicsMutex_);
//
//         try {
//             std::cout << "[Physics] Initializing physics system..." << std::endl;
//
//             // Setup Bullet world
//             setupBulletWorld();
//
//             // Create debug drawer if enabled
//             if (config_.enableDebugDraw) {
//                 debugDrawer_ = std::make_unique<PhysicsDebugDrawer>();
//                 dynamicsWorld_->setDebugDrawer(debugDrawer_.get());
//                 std::cout << "[Physics] Debug drawer enabled" << std::endl;
//             }
//
//             // Initialize timing
//             lastUpdateTime_ = std::chrono::high_resolution_clock::now();
//
//             initialized_ = true;
//             std::cout << "[Physics] Physics system initialized successfully" << std::endl;
//
//             return true;
//         }
//         catch (const std::exception& e) {
//             std::cerr << "[Physics] Initialization failed: " << e.what() << std::endl;
//             cleanupBulletWorld();
//             return false;
//         }
//     }
//
//     /**
//      * @brief Setup Bullet physics world
//      * IMPLEMENTATION: Creates and configures all Bullet components
//      */
//     void PhysicsManager::setupBulletWorld() {
//         // Create collision configuration
//         if (config_.enableSoftBody) {
//             collisionConfig_ = std::make_unique<btSoftBodyRigidBodyCollisionConfiguration>();
//             std::cout << "[Physics] Using soft body collision configuration" << std::endl;
//         }
//         else {
//             collisionConfig_ = std::make_unique<btDefaultCollisionConfiguration>();
//             std::cout << "[Physics] Using default collision configuration" << std::endl;
//         }
//
//         // Create collision dispatcher
//         dispatcher_ = std::make_unique<btCollisionDispatcher>(collisionConfig_.get());
//
//         // Create broadphase
//         broadphase_ = std::make_unique<btDbvtBroadphase>();
//         std::cout << "[Physics] Created dynamic AABB tree broadphase" << std::endl;
//
//         // Create constraint solver
//         if (config_.multithreaded) {
//             // Note: Multithreaded solver requires additional setup
//             solver_ = std::make_unique<btSequentialImpulseConstraintSolver>();
//             std::cout << "[Physics] Warning: Multithreaded solver not fully implemented, using sequential" << std::endl;
//         }
//         else {
//             solver_ = std::make_unique<btSequentialImpulseConstraintSolver>();
//             std::cout << "[Physics] Using sequential impulse constraint solver" << std::endl;
//         }
//
//         // Create dynamics world
//         if (config_.enableSoftBody) {
//             // Soft body world requires special solver
//             softBodySolver_ = std::make_unique<btDefaultSoftBodySolver>();
//
//             auto* softWorld = new btSoftRigidDynamicsWorld(
//                 dispatcher_.get(),
//                 broadphase_.get(),
//                 solver_.get(),
//                 collisionConfig_.get(),
//                 softBodySolver_.get()
//             );
//
//             dynamicsWorld_.reset(softWorld);
//             std::cout << "[Physics] Created soft-rigid dynamics world" << std::endl;
//         }
//         else {
//             dynamicsWorld_ = std::make_unique<btDiscreteDynamicsWorld>(
//                 dispatcher_.get(),
//                 broadphase_.get(),
//                 solver_.get(),
//                 collisionConfig_.get()
//             );
//             std::cout << "[Physics] Created discrete dynamics world" << std::endl;
//         }
//
//         // Configure world
//         dynamicsWorld_->setGravity(btVector3(config_.gravity.x, config_.gravity.y, config_.gravity.z));
//         dynamicsWorld_->getSolverInfo().m_numIterations = config_.solverIterations;
//
//         // Setup ghost pair callback for triggers
//         ghostPairCallback_ = std::make_unique<btGhostPairCallback>();
//         dynamicsWorld_->getBroadphase()->getOverlappingPairCache()->setInternalGhostPairCallback(
//             ghostPairCallback_.get());
//
//         // Configure CCD if enabled
//         if (config_.enableContinuousCollision) {
//             dynamicsWorld_->getDispatchInfo().m_useContinuous = true;
//             std::cout << "[Physics] Continuous collision detection enabled" << std::endl;
//         }
//
//         std::cout << "[Physics] Bullet world setup complete" << std::endl;
//     }
//
//     /**
//      * @brief Cleanup Bullet world
//      * IMPLEMENTATION: Destroys all Bullet components in correct order
//      */
//     void PhysicsManager::cleanupBulletWorld() {
//         if (dynamicsWorld_) {
//             // Remove all constraints
//             for (int i = dynamicsWorld_->getNumConstraints() - 1; i >= 0; --i) {
//                 btTypedConstraint* constraint = dynamicsWorld_->getConstraint(i);
//                 dynamicsWorld_->removeConstraint(constraint);
//             }
//
//             // Remove all rigid bodies
//             for (int i = dynamicsWorld_->getNumCollisionObjects() - 1; i >= 0; --i) {
//                 btCollisionObject* obj = dynamicsWorld_->getCollisionObjectArray()[i];
//                 dynamicsWorld_->removeCollisionObject(obj);
//             }
//         }
//
//         // Clear storage
//         constraints_.clear();
//         bodies_.clear();
//         shapes_.clear();
//
//         // Destroy Bullet components in reverse order
//         dynamicsWorld_.reset();
//         solver_.reset();
//         broadphase_.reset();
//         dispatcher_.reset();
//         collisionConfig_.reset();
//         softBodySolver_.reset();
//         ghostPairCallback_.reset();
//         debugDrawer_.reset();
//
//         std::cout << "[Physics] Bullet world cleaned up" << std::endl;
//     }
//
//     /**
//      * @brief Shutdown physics system
//      * IMPLEMENTATION: Complete cleanup of physics resources
//      */
//     void PhysicsManager::shutdown() {
//         if (!initialized_) {
//             return;
//         }
//
//         std::lock_guard<std::mutex> lock(physicsMutex_);
//
//         std::cout << "[Physics] Shutting down physics system..." << std::endl;
//
//         cleanupBulletWorld();
//
//         // Clear callbacks
//         collisionCallbacks_.clear();
//         triggerCallbacks_.clear();
//         preStepCallbacks_.clear();
//         postStepCallbacks_.clear();
//
//         // Clear collision tracking
//         activeCollisions_.clear();
//         collisionEvents_.clear();
//
//         initialized_ = false;
//
//         std::cout << "[Physics] Physics system shutdown complete" << std::endl;
//     }
//
//     /**
//      * @brief Update physics simulation
//      * IMPLEMENTATION: Steps the physics simulation with interpolation
//      */
//     void PhysicsManager::update(float deltaTime) {
//         if (!initialized_ || !dynamicsWorld_) {
//             return;
//         }
//
//         // Apply simulation speed
//         deltaTime *= simulationSpeed_;
//
//         // Clamp large time steps to prevent instability
//         deltaTime = std::min(deltaTime, 0.25f);
//
//         // Fixed timestep with interpolation
//         accumulator_ += deltaTime;
//
//         int steps = 0;
//         while (accumulator_ >= config_.fixedTimeStep && steps < config_.maxSubSteps) {
//             // Pre-step callbacks
//             for (const auto& callback : preStepCallbacks_) {
//                 callback(config_.fixedTimeStep);
//             }
//
//             // Step simulation
//             dynamicsWorld_->stepSimulation(config_.fixedTimeStep, 0, config_.fixedTimeStep);
//
//             // Process collisions
//             processCollisions();
//
//             // Post-step callbacks
//             for (const auto& callback : postStepCallbacks_) {
//                 callback(config_.fixedTimeStep);
//             }
//
//             accumulator_ -= config_.fixedTimeStep;
//             steps++;
//         }
//
//         // Update statistics
//         if (config_.enableStatistics) {
//             updateStatistics();
//         }
//     }
//
//     /**
//      * @brief Fixed update for physics
//      * IMPLEMENTATION: Single fixed timestep update
//      */
//     void PhysicsManager::fixedUpdate(float fixedDeltaTime) {
//         if (!initialized_ || !dynamicsWorld_) {
//             return;
//         }
//
//         // Pre-step callbacks
//         for (const auto& callback : preStepCallbacks_) {
//             callback(fixedDeltaTime);
//         }
//
//         // Step simulation with fixed timestep
//         dynamicsWorld_->stepSimulation(fixedDeltaTime, 0, fixedDeltaTime);
//
//         // Process collisions
//         processCollisions();
//
//         // Post-step callbacks
//         for (const auto& callback : postStepCallbacks_) {
//             callback(fixedDeltaTime);
//         }
//
//         // Update statistics
//         if (config_.enableStatistics) {
//             updateStatistics();
//         }
//     }
//
//     /**
//      * @brief Process collision events
//      * IMPLEMENTATION: Detects and reports collision events to callbacks
//      */
//     void PhysicsManager::processCollisions() {
//         if (collisionCallbacks_.empty() && triggerCallbacks_.empty()) {
//             return; // No callbacks registered, skip processing
//         }
//
//         // Create collision handler
//         CollisionCallbackHandler handler(this);
//
//         // Check all collision pairs
//         int numManifolds = dispatcher_->getNumManifolds();
//         for (int i = 0; i < numManifolds; ++i) {
//             btPersistentManifold* manifold = dispatcher_->getManifoldByIndexInternal(i);
//
//             if (manifold->getNumContacts() > 0) {
//                 const btCollisionObject* objA = manifold->getBody0();
//                 const btCollisionObject* objB = manifold->getBody1();
//
//                 RigidBody* bodyA = static_cast<RigidBody*>(objA->getUserPointer());
//                 RigidBody* bodyB = static_cast<RigidBody*>(objB->getUserPointer());
//
//                 if (bodyA && bodyB) {
//                     // Create collision manifold
//                     CollisionManifold collisionManifold;
//                     collisionManifold.bodyA = bodyA->getId();
//                     collisionManifold.bodyB = bodyB->getId();
//
//                     // Collect contact points
//                     for (int j = 0; j < manifold->getNumContacts(); ++j) {
//                         const btManifoldPoint& pt = manifold->getContactPoint(j);
//
//                         ContactPoint contact;
//                         contact.worldPositionOnA = Vector3(pt.m_positionWorldOnA.x(),
//                                                            pt.m_positionWorldOnA.y(),
//                                                            pt.m_positionWorldOnA.z());
//                         contact.worldPositionOnB = Vector3(pt.m_positionWorldOnB.x(),
//                                                            pt.m_positionWorldOnB.y(),
//                                                            pt.m_positionWorldOnB.z());
//                         contact.worldNormalOnB = Vector3(pt.m_normalWorldOnB.x(),
//                                                          pt.m_normalWorldOnB.y(),
//                                                          pt.m_normalWorldOnB.z());
//                         contact.penetrationDepth = pt.m_distance1;
//                         contact.appliedImpulse = pt.m_appliedImpulse;
//                         contact.lifetime = pt.m_lifeTime;
//
//                         collisionManifold.contactPoints.push_back(contact);
//                     }
//
//                     // Determine event type
//                     CollisionPairID pairId = collisionManifold.getPairID();
//                     auto it = activeCollisions_.find(pairId);
//
//                     if (it == activeCollisions_.end()) {
//                         // New collision
//                         collisionManifold.eventType = CollisionEventType::CONTACT_STARTED;
//                         activeCollisions_[pairId] = collisionManifold;
//                     }
//                     else {
//                         // Ongoing collision
//                         collisionManifold.eventType = CollisionEventType::CONTACT_PERSISTED;
//                         activeCollisions_[pairId] = collisionManifold;
//                     }
//
//                     // Trigger callbacks
//                     for (const auto& callback : collisionCallbacks_) {
//                         callback(collisionManifold);
//                     }
//                 }
//             }
//         }
//
//         // Check for ended collisions
//         std::vector<CollisionPairID> endedCollisions;
//         for (const auto& [pairId, manifold] : activeCollisions_) {
//             bool stillColliding = false;
//
//             // Check if this pair still has contacts
//             for (int i = 0; i < numManifolds; ++i) {
//                 btPersistentManifold* btManifold = dispatcher_->getManifoldByIndexInternal(i);
//                 if (btManifold->getNumContacts() > 0) {
//                     const btCollisionObject* objA = btManifold->getBody0();
//                     const btCollisionObject* objB = btManifold->getBody1();
//
//                     RigidBody* bodyA = static_cast<RigidBody*>(objA->getUserPointer());
//                     RigidBody* bodyB = static_cast<RigidBody*>(objB->getUserPointer());
//
//                     if (bodyA && bodyB) {
//                         CollisionPairID checkId = 0;
//                         if (bodyA->getId() < bodyB->getId()) {
//                             checkId = (static_cast<CollisionPairID>(bodyA->getId()) << 32) | bodyB->getId();
//                         }
//                         else {
//                             checkId = (static_cast<CollisionPairID>(bodyB->getId()) << 32) | bodyA->getId();
//                         }
//
//                         if (checkId == pairId) {
//                             stillColliding = true;
//                             break;
//                         }
//                     }
//                 }
//             }
//
//             if (!stillColliding) {
//                 endedCollisions.push_back(pairId);
//
//                 // Trigger end collision callback
//                 CollisionManifold endManifold = manifold;
//                 endManifold.eventType = CollisionEventType::CONTACT_ENDED;
//
//                 for (const auto& callback : collisionCallbacks_) {
//                     callback(endManifold);
//                 }
//             }
//         }
//
//         // Remove ended collisions
//         for (CollisionPairID pairId : endedCollisions) {
//             activeCollisions_.erase(pairId);
//         }
//     }
//
//     /**
//      * @brief Update physics statistics
//      * IMPLEMENTATION: Collects performance and state metrics
//      */
//     void PhysicsManager::updateStatistics() {
//         auto now = std::chrono::high_resolution_clock::now();
//         auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - lastUpdateTime_);
//         statistics_.simulationTime = duration.count() / 1000.0f; // Convert to milliseconds
//
//         // Count active/sleeping bodies
//         statistics_.numActiveBodies = 0;
//         statistics_.numSleepingBodies = 0;
//
//         for (const auto& [id, body] : bodies_) {
//             if (body->isActive()) {
//                 statistics_.numActiveBodies++;
//             }
//             else if (body->isSleeping()) {
//                 statistics_.numSleepingBodies++;
//             }
//         }
//
//         // Get Bullet statistics
//         statistics_.numContacts = 0;
//         statistics_.numManifolds = dispatcher_->getNumManifolds();
//
//         for (int i = 0; i < statistics_.numManifolds; ++i) {
//             btPersistentManifold* manifold = dispatcher_->getManifoldByIndexInternal(i);
//             statistics_.numContacts += manifold->getNumContacts();
//         }
//
//         statistics_.numConstraints = dynamicsWorld_->getNumConstraints();
//         statistics_.broadphasePairs = broadphase_->getOverlappingPairCache()->getNumOverlappingPairs();
//
//         // Calculate memory usage
//         statistics_.totalMemory = 0;
//         statistics_.shapeMemory = shapes_.size() * sizeof(CollisionShape);
//         statistics_.bodyMemory = bodies_.size() * sizeof(RigidBody);
//         statistics_.constraintMemory = constraints_.size() * sizeof(PhysicsConstraint);
//
//         for (const auto& [id, shape] : shapes_) {
//             statistics_.shapeMemory += shape->getMemoryUsage();
//         }
//
//         statistics_.totalMemory = statistics_.shapeMemory + statistics_.bodyMemory + statistics_.constraintMemory;
//
//         lastUpdateTime_ = now;
//     }
//
//     // ========================================================================
//     // WORLD CONFIGURATION
//     // ========================================================================
//
//     /**
//      * @brief Set gravity
//      * IMPLEMENTATION: Updates world gravity vector
//      */
//     void PhysicsManager::setGravity(const Vector3& gravity) {
//         if (dynamicsWorld_) {
//             dynamicsWorld_->setGravity(btVector3(gravity.x, gravity.y, gravity.z));
//             config_.gravity = gravity;
//
//             std::cout << "[Physics] Gravity set to: " << gravity.x << ", " << gravity.y << ", " << gravity.z <<
//                 std::endl;
//         }
//     }
//
//     /**
//      * @brief Get current gravity
//      * IMPLEMENTATION: Returns world gravity vector
//      */
//     Vector3 PhysicsManager::getGravity() const {
//         if (dynamicsWorld_) {
//             const btVector3& gravity = dynamicsWorld_->getGravity();
//             return Vector3(gravity.x(), gravity.y(), gravity.z());
//         }
//         return config_.gravity;
//     }
//
//     /**
//      * @brief Update configuration
//      * IMPLEMENTATION: Applies new configuration to physics world
//      */
//     void PhysicsManager::updateConfig(const PhysicsConfig& config) {
//         config_ = config;
//
//         if (dynamicsWorld_) {
//             // Update gravity
//             setGravity(config.gravity);
//
//             // Update solver iterations
//             dynamicsWorld_->getSolverInfo().m_numIterations = config.solverIterations;
//
//             // Update CCD
//             dynamicsWorld_->getDispatchInfo().m_useContinuous = config.enableContinuousCollision;
//
//             // Update debug drawing
//             if (config.enableDebugDraw && !debugDrawer_) {
//                 debugDrawer_ = std::make_unique<PhysicsDebugDrawer>();
//                 dynamicsWorld_->setDebugDrawer(debugDrawer_.get());
//             }
//             else if (!config.enableDebugDraw && debugDrawer_) {
//                 dynamicsWorld_->setDebugDrawer(nullptr);
//                 debugDrawer_.reset();
//             }
//         }
//     }
//
//     // ========================================================================
//     // COLLISION SHAPE MANAGEMENT
//     // ========================================================================
//
//     /**
//      * @brief Register a collision shape
//      * IMPLEMENTATION: Adds shape to manager's registry
//      */
//     CollisionShapeID PhysicsManager::registerShape(std::unique_ptr<CollisionShape> shape) {
//         if (!shape) {
//             return INVALID_SHAPE_ID;
//         }
//
//         std::lock_guard<std::mutex> lock(physicsMutex_);
//
//         CollisionShapeID shapeId = shape->getId();
//         shapes_[shapeId] = std::move(shape);
//
//         return shapeId;
//     }
//
//     /**
//      * @brief Get collision shape by ID
//      * IMPLEMENTATION: Returns shape pointer or nullptr
//      */
//     CollisionShape* PhysicsManager::getShape(CollisionShapeID shapeId) {
//         auto it = shapes_.find(shapeId);
//         return (it != shapes_.end()) ? it->second.get() : nullptr;
//     }
//
//     /**
//      * @brief Remove collision shape
//      * IMPLEMENTATION: Removes shape if not in use
//      */
//     bool PhysicsManager::removeShape(CollisionShapeID shapeId) {
//         std::lock_guard<std::mutex> lock(physicsMutex_);
//
//         // Check if shape is in use by any body
//         for (const auto& [bodyId, body] : bodies_) {
//             if (body->getShape() && body->getShape()->getId() == shapeId) {
//                 std::cerr << "[Physics] Cannot remove shape " << shapeId << " - still in use by body " << bodyId <<
//                     std::endl;
//                 return false;
//             }
//         }
//
//         auto it = shapes_.find(shapeId);
//         if (it != shapes_.end()) {
//             shapes_.erase(it);
//             std::cout << "[Physics] Removed shape " << shapeId << std::endl;
//             return true;
//         }
//
//         return false;
//     }
//
//     // ========================================================================
//     // RIGID BODY MANAGEMENT
//     // ========================================================================
//
//     /**
//      * @brief Create rigid body
//      * IMPLEMENTATION: Creates body with shape and adds to world
//      */
//     RigidBodyID PhysicsManager::createRigidBody(CollisionShapeID shapeId, const RigidBodyConfig& config) {
//         if (!initialized_ || !dynamicsWorld_) {
//             std::cerr << "[Physics] Cannot create body - system not initialized" << std::endl;
//             return INVALID_BODY_ID;
//         }
//
//         CollisionShape* shape = getShape(shapeId);
//         if (!shape) {
//             std::cerr << "[Physics] Cannot create body - invalid shape ID: " << shapeId << std::endl;
//             return INVALID_BODY_ID;
//         }
//
//         std::lock_guard<std::mutex> lock(physicsMutex_);
//
//         try {
//             // Generate ID and create body
//             RigidBodyID bodyId = nextBodyId_.fetch_add(1);
//             auto body = std::make_unique<RigidBody>(bodyId, shape, config);
//
//             // Add to physics world
//             if (body->isTrigger()) {
//                 // Add ghost object for triggers
//                 dynamicsWorld_->addCollisionObject(
//                     body->getGhostObject(),
//                     static_cast<short>(config.filter.layer),
//                     static_cast<short>(config.filter.mask)
//                 );
//             }
//             else {
//                 // Add rigid body
//                 dynamicsWorld_->addRigidBody(
//                     body->getBulletBody(),
//                     static_cast<short>(config.filter.layer),
//                     static_cast<short>(config.filter.mask)
//                 );
//             }
//
//             // Store body
//             bodies_[bodyId] = std::move(body);
//
//             std::cout << "[Physics] Created rigid body " << bodyId << " with shape " << shapeId << std::endl;
//             return bodyId;
//         }
//         catch (const std::exception& e) {
//             std::cerr << "[Physics] Failed to create rigid body: " << e.what() << std::endl;
//             return INVALID_BODY_ID;
//         }
//     }
//
//     /**
//      * @brief Remove rigid body
//      * IMPLEMENTATION: Removes body from world and storage
//      */
//     bool PhysicsManager::removeRigidBody(RigidBodyID bodyId) {
//         std::lock_guard<std::mutex> lock(physicsMutex_);
//
//         auto it = bodies_.find(bodyId);
//         if (it == bodies_.end()) {
//             return false;
//         }
//
//         RigidBody* body = it->second.get();
//
//         // Remove constraints connected to this body
//         std::vector<ConstraintID> constraintsToRemove;
//         for (const auto& [constraintId, constraint] : constraints_) {
//             auto [bodyA, bodyB] = constraint->getConnectedBodies();
//             if (bodyA == bodyId || bodyB == bodyId) {
//                 constraintsToRemove.push_back(constraintId);
//             }
//         }
//
//         for (ConstraintID constraintId : constraintsToRemove) {
//             removeConstraint(constraintId);
//         }
//
//         // Remove from physics world
//         if (body->isTrigger() && body->getGhostObject()) {
//             dynamicsWorld_->removeCollisionObject(body->getGhostObject());
//         }
//         else if (body->getBulletBody()) {
//             dynamicsWorld_->removeRigidBody(body->getBulletBody());
//         }
//
//         // Remove from storage
//         bodies_.erase(it);
//
//         std::cout << "[Physics] Removed rigid body " << bodyId << std::endl;
//         return true;
//     }
//
//     /**
//      * @brief Get rigid body by ID
//      * IMPLEMENTATION: Returns body pointer or nullptr
//      */
//     RigidBody* PhysicsManager::getRigidBody(RigidBodyID bodyId) {
//         auto it = bodies_.find(bodyId);
//         return (it != bodies_.end()) ? it->second.get() : nullptr;
//     }
//
//     /**
//      * @brief Get all rigid bodies
//      * IMPLEMENTATION: Returns vector of all body IDs
//      */
//     std::vector<RigidBodyID> PhysicsManager::getAllRigidBodies() const {
//         std::vector<RigidBodyID> bodyIds;
//         bodyIds.reserve(bodies_.size());
//
//         for (const auto& [id, body] : bodies_) {
//             bodyIds.push_back(id);
//         }
//
//         return bodyIds;
//     }
//
//     // ========================================================================
//     // CONSTRAINT MANAGEMENT
//     // ========================================================================
//
//     /**
//      * @brief Create constraint between bodies
//      * IMPLEMENTATION: Creates and adds constraint to world
//      */
//     ConstraintID PhysicsManager::createConstraint(const ConstraintConfig& config) {
//         if (!initialized_ || !dynamicsWorld_) {
//             std::cerr << "[Physics] Cannot create constraint - system not initialized" << std::endl;
//             return INVALID_CONSTRAINT_ID;
//         }
//
//         RigidBody* bodyA = getRigidBody(config.bodyA);
//         if (!bodyA) {
//             std::cerr << "[Physics] Cannot create constraint - invalid body A: " << config.bodyA << std::endl;
//             return INVALID_CONSTRAINT_ID;
//         }
//
//         RigidBody* bodyB = nullptr;
//         if (config.bodyB != INVALID_BODY_ID) {
//             bodyB = getRigidBody(config.bodyB);
//             if (!bodyB) {
//                 std::cerr << "[Physics] Cannot create constraint - invalid body B: " << config.bodyB << std::endl;
//                 return INVALID_CONSTRAINT_ID;
//             }
//         }
//
//         std::lock_guard<std::mutex> lock(physicsMutex_);
//
//         try {
//             // Generate ID and create constraint
//             ConstraintID constraintId = nextConstraintId_.fetch_add(1);
//             auto constraint = std::make_unique<PhysicsConstraint>(constraintId, config, bodyA, bodyB);
//
//             // Add to physics world
//             dynamicsWorld_->addConstraint(
//                 constraint->getBulletConstraint(),
//                 config.disableCollisionsBetweenLinkedBodies
//             );
//
//             // Store constraint
//             constraints_[constraintId] = std::move(constraint);
//
//             std::cout << "[Physics] Created constraint " << constraintId
//                 << " between bodies " << config.bodyA
//                 << " and " << (config.bodyB != INVALID_BODY_ID ? std::to_string(config.bodyB) : "world")
//                 << std::endl;
//
//             return constraintId;
//         }
//         catch (const std::exception& e) {
//             std::cerr << "[Physics] Failed to create constraint: " << e.what() << std::endl;
//             return INVALID_CONSTRAINT_ID;
//         }
//     }
//
//     /**
//      * @brief Remove constraint
//      * IMPLEMENTATION: Removes constraint from world and storage
//      */
//     bool PhysicsManager::removeConstraint(ConstraintID constraintId) {
//         std::lock_guard<std::mutex> lock(physicsMutex_);
//
//         auto it = constraints_.find(constraintId);
//         if (it == constraints_.end()) {
//             return false;
//         }
//
//         // Remove from physics world
//         if (it->second->getBulletConstraint()) {
//             dynamicsWorld_->removeConstraint(it->second->getBulletConstraint());
//         }
//
//         // Remove from storage
//         constraints_.erase(it);
//
//         std::cout << "[Physics] Removed constraint " << constraintId << std::endl;
//         return true;
//     }
//
//     /**
//      * @brief Get constraint by ID
//      * IMPLEMENTATION: Returns constraint pointer or nullptr
//      */
//     PhysicsConstraint* PhysicsManager::getConstraint(ConstraintID constraintId) {
//         auto it = constraints_.find(constraintId);
//         return (it != constraints_.end()) ? it->second.get() : nullptr;
//     }
//
//
//     // ========================================================================
//     // RAYCASTING AND QUERIES IMPLEMENTATION
//     // ========================================================================
//
//     /**
//  * @brief Internal raycast callback
//  * Collects all hits or stops at first hit
//  */
//     class RaycastCallback : public btCollisionWorld::RayResultCallback {
//     public:
//         std::vector<RaycastHit> hits;
//         CollisionFilter filter;
//         bool stopAtFirstHit;
//         btVector3 rayFrom;
//         btVector3 rayTo;
//
//         RaycastCallback(const CollisionFilter& f, bool stopFirst, const btVector3& from, const btVector3& to)
//             : filter(f), stopAtFirstHit(stopFirst), rayFrom(from), rayTo(to) {
//             m_collisionFilterGroup = static_cast<short>(f.layer);
//             m_collisionFilterMask = static_cast<short>(f.mask);
//         }
//
//         btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace) override {
//             // Get rigid body from collision object
//             RigidBody* body = static_cast<RigidBody*>(rayResult.m_collisionObject->getUserPointer());
//             if (!body) return 1.0f;
//
//             // Check collision filter
//             if (!filter.canCollideWith(body->getCollisionFilter())) {
//                 return 1.0f; // Continue ray
//             }
//
//             // Create hit result
//             RaycastHit hit;
//             hit.bodyId = body->getId();
//             hit.fraction = rayResult.m_hitFraction;
//
//             // Calculate hit point
//             btVector3 hitPoint = rayFrom.lerp(rayTo, rayResult.m_hitFraction);
//             hit.point = Vector3(hitPoint.x(), hitPoint.y(), hitPoint.z());
//
//             // Get normal
//             if (normalInWorldSpace) {
//                 hit.normal = Vector3(rayResult.m_hitNormalLocal.x(),
//                                      rayResult.m_hitNormalLocal.y(),
//                                      rayResult.m_hitNormalLocal.z());
//             }
//             else {
//                 // Convert local normal to world space
//                 btVector3 worldNormal = rayResult.m_collisionObject->getWorldTransform().getBasis() * rayResult.
//                     m_hitNormalLocal;
//                 hit.normal = Vector3(worldNormal.x(), worldNormal.y(), worldNormal.z());
//             }
//
//             // Calculate distance
//             hit.distance = glm::length(hit.point - Vector3(rayFrom.x(), rayFrom.y(), rayFrom.z()));
//
//             // Store user data
//             hit.userData = body->getUserData();
//
//             // Add to hits
//             hits.push_back(hit);
//
//             // Return fraction to continue (1.0) or stop (current fraction)
//             return stopAtFirstHit ? rayResult.m_hitFraction : 1.0f;
//         }
//     };
//
//     /**
//      * @brief Internal sweep test callback
//      */
//     class SweepCallback : public btCollisionWorld::ConvexResultCallback {
//     public:
//         std::vector<RaycastHit> hits;
//         CollisionFilter filter;
//         bool stopAtFirstHit;
//
//         SweepCallback(const CollisionFilter& f, bool stopFirst)
//             : filter(f), stopAtFirstHit(stopFirst) {
//             m_collisionFilterGroup = static_cast<short>(f.layer);
//             m_collisionFilterMask = static_cast<short>(f.mask);
//         }
//
//         btScalar addSingleResult(btCollisionWorld::LocalConvexResult& convexResult, bool normalInWorldSpace) override {
//             // Get rigid body
//             RigidBody* body = static_cast<RigidBody*>(convexResult.m_hitCollisionObject->getUserPointer());
//             if (!body) return 1.0f;
//
//             // Check collision filter
//             if (!filter.canCollideWith(body->getCollisionFilter())) {
//                 return 1.0f;
//             }
//
//             // Create hit result
//             RaycastHit hit;
//             hit.bodyId = body->getId();
//             hit.fraction = convexResult.m_hitFraction;
//             hit.point = Vector3(convexResult.m_hitPointLocal.x(),
//                                 convexResult.m_hitPointLocal.y(),
//                                 convexResult.m_hitPointLocal.z());
//             hit.normal = Vector3(convexResult.m_hitNormalLocal.x(),
//                                  convexResult.m_hitNormalLocal.y(),
//                                  convexResult.m_hitNormalLocal.z());
//             hit.userData = body->getUserData();
//
//             hits.push_back(hit);
//
//             return stopAtFirstHit ? convexResult.m_hitFraction : 1.0f;
//         }
//     };
//
//     /**
//      * @brief Internal overlap test callback
//      */
//     class OverlapCallback : public btCollisionWorld::ContactResultCallback {
//     public:
//         std::vector<RigidBodyID> overlappingBodies;
//         CollisionFilter filter;
//
//         explicit OverlapCallback(const CollisionFilter& f) : filter(f) {
//             m_collisionFilterGroup = static_cast<short>(f.layer);
//             m_collisionFilterMask = static_cast<short>(f.mask);
//         }
//
//         btScalar addSingleResult(btManifoldPoint& cp,
//                                  const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0,
//                                  const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1) override {
//             // Get the other object (not the query shape)
//             const btCollisionObject* otherObj = colObj1Wrap->getCollisionObject();
//             RigidBody* body = static_cast<RigidBody*>(otherObj->getUserPointer());
//
//             if (body && filter.canCollideWith(body->getCollisionFilter())) {
//                 // Check if already added
//                 if (std::find(overlappingBodies.begin(), overlappingBodies.end(), body->getId()) == overlappingBodies.
//                     end()) {
//                     overlappingBodies.push_back(body->getId());
//                 }
//             }
//
//             return 0; // Not used
//         }
//     };
//
//     /**
//      * @brief Perform raycast
//      * IMPLEMENTATION: Casts a ray and returns all hits
//      */
//     RaycastResult PhysicsManager::raycast(const Ray& ray) const {
//         RaycastResult result;
//
//         if (!initialized_ || !dynamicsWorld_) {
//             return result;
//         }
//
//         // Convert ray to Bullet format
//         btVector3 from(ray.origin.x, ray.origin.y, ray.origin.z);
//         btVector3 to = from + btVector3(ray.direction.x, ray.direction.y, ray.direction.z) * ray.maxDistance;
//
//         // Create callback
//         RaycastCallback callback(ray.filter, ray.stopAtFirstHit, from, to);
//
//         // Perform raycast
//         if (ray.stopAtFirstHit) {
//             dynamicsWorld_->rayTest(from, to, callback);
//         }
//         else {
//             dynamicsWorld_->rayTest(from, to, callback);
//         }
//
//         // Sort hits by distance
//         std::sort(callback.hits.begin(), callback.hits.end(),
//                   [](const RaycastHit& a, const RaycastHit& b) {
//                       return a.distance < b.distance;
//                   });
//
//         result.hits = callback.hits;
//         result.hasHit = !callback.hits.empty();
//
//         // Update statistics
//         const_cast<PhysicsStatistics&>(statistics_).numRaycasts++;
//
//         return result;
//     }
//
//     /**
//      * @brief Perform sweep test
//      * IMPLEMENTATION: Sweeps a shape through space and detects collisions
//      */
//     RaycastResult PhysicsManager::sweepTest(const SweepTest& sweep) const {
//         RaycastResult result;
//
//         if (!initialized_ || !dynamicsWorld_) {
//             return result;
//         }
//
//         CollisionShape* shape = const_cast<PhysicsManager*>(this)->getShape(sweep.shapeId);
//         if (!shape || !shape->getBulletShape()) {
//             std::cerr << "[Physics] Invalid shape for sweep test" << std::endl;
//             return result;
//         }
//
//         // Convert transforms to Bullet
//         btTransform from, to;
//         from.setOrigin(btVector3(sweep.startTransform.position.x,
//                                  sweep.startTransform.position.y,
//                                  sweep.startTransform.position.z));
//         from.setRotation(btQuaternion(sweep.startTransform.rotation.x,
//                                       sweep.startTransform.rotation.y,
//                                       sweep.startTransform.rotation.z,
//                                       sweep.startTransform.rotation.w));
//
//         to.setOrigin(btVector3(sweep.endTransform.position.x,
//                                sweep.endTransform.position.y,
//                                sweep.endTransform.position.z));
//         to.setRotation(btQuaternion(sweep.endTransform.rotation.x,
//                                     sweep.endTransform.rotation.y,
//                                     sweep.endTransform.rotation.z,
//                                     sweep.endTransform.rotation.w));
//
//         // Create callback
//         SweepCallback callback(sweep.filter, sweep.stopAtFirstHit);
//
//         // Perform sweep test
//         if (shape->isConvex()) {
//             btConvexShape* convexShape = static_cast<btConvexShape*>(shape->getBulletShape());
//             dynamicsWorld_->convexSweepTest(convexShape, from, to, callback);
//
//             // Sort hits by distance
//             std::sort(callback.hits.begin(), callback.hits.end(),
//                       [](const RaycastHit& a, const RaycastHit& b) {
//                           return a.fraction < b.fraction;
//                       });
//
//             result.hits = callback.hits;
//             result.hasHit = !callback.hits.empty();
//         }
//         else {
//             std::cerr << "[Physics] Sweep test requires convex shape" << std::endl;
//         }
//
//         // Update statistics
//         const_cast<PhysicsStatistics&>(statistics_).numSweepTests++;
//
//         return result;
//     }
//
//     /**
//      * @brief Perform overlap test
//      * IMPLEMENTATION: Finds all bodies overlapping with a shape
//      */
//     std::vector<RigidBodyID> PhysicsManager::overlapTest(const OverlapTest& overlap) const {
//         std::vector<RigidBodyID> overlappingBodies;
//
//         if (!initialized_ || !dynamicsWorld_) {
//             return overlappingBodies;
//         }
//
//         CollisionShape* shape = const_cast<PhysicsManager*>(this)->getShape(overlap.shapeId);
//         if (!shape || !shape->getBulletShape()) {
//             std::cerr << "[Physics] Invalid shape for overlap test" << std::endl;
//             return overlappingBodies;
//         }
//
//         // Create temporary collision object for query
//         btCollisionObject queryObject;
//         queryObject.setCollisionShape(shape->getBulletShape());
//
//         // Set transform
//         btTransform transform;
//         transform.setOrigin(btVector3(overlap.transform.position.x,
//                                       overlap.transform.position.y,
//                                       overlap.transform.position.z));
//         transform.setRotation(btQuaternion(overlap.transform.rotation.x,
//                                            overlap.transform.rotation.y,
//                                            overlap.transform.rotation.z,
//                                            overlap.transform.rotation.w));
//         queryObject.setWorldTransform(transform);
//
//         // Create callback
//         OverlapCallback callback(overlap.filter);
//
//         // Perform overlap test
//         dynamicsWorld_->contactTest(&queryObject, callback);
//
//         return callback.overlappingBodies;
//     }
//
//     /**
//      * @brief Query bodies in AABB
//      * IMPLEMENTATION: Finds all bodies within an axis-aligned bounding box
//      */
//     std::vector<RigidBodyID> PhysicsManager::queryAABB(const Vector3& min, const Vector3& max,
//                                                        const CollisionFilter& filter) const {
//         std::vector<RigidBodyID> bodiesInAABB;
//
//         if (!initialized_ || !dynamicsWorld_) {
//             return bodiesInAABB;
//         }
//
//         // Broadphase AABB query
//         btVector3 aabbMin(min.x, min.y, min.z);
//         btVector3 aabbMax(max.x, max.y, max.z);
//
//         // Custom callback for AABB query
//         struct AABBCallback : public btBroadphaseAabbCallback {
//             std::vector<RigidBodyID>* bodies;
//             const CollisionFilter* filter;
//
//             bool process(const btBroadphaseProxy* proxy) override {
//                 btCollisionObject* obj = static_cast<btCollisionObject*>(proxy->m_clientObject);
//                 RigidBody* body = static_cast<RigidBody*>(obj->getUserPointer());
//
//                 if (body && filter->canCollideWith(body->getCollisionFilter())) {
//                     bodies->push_back(body->getId());
//                 }
//                 return true; // Continue query
//             }
//         };
//
//         AABBCallback callback;
//         callback.bodies = &bodiesInAABB;
//         callback.filter = &filter;
//
//         dynamicsWorld_->getBroadphase()->aabbTest(aabbMin, aabbMax, callback);
//
//         return bodiesInAABB;
//     }
//
//     // ========================================================================
//     // COLLISION CALLBACKS
//     // ========================================================================
//
//     /**
//      * @brief Register collision callback
//      * IMPLEMENTATION: Adds callback for collision events
//      */
//     void PhysicsManager::registerCollisionCallback(const CollisionCallback& callback) {
//         collisionCallbacks_.push_back(callback);
//         std::cout << "[Physics] Registered collision callback (total: " << collisionCallbacks_.size() << ")" <<
//             std::endl;
//     }
//
//     /**
//      * @brief Register trigger callback
//      * IMPLEMENTATION: Adds callback for trigger events
//      */
//     void PhysicsManager::registerTriggerCallback(const TriggerCallback& callback) {
//         triggerCallbacks_.push_back(callback);
//         std::cout << "[Physics] Registered trigger callback (total: " << triggerCallbacks_.size() << ")" << std::endl;
//     }
//
//     /**
//      * @brief Register pre-step callback
//      * IMPLEMENTATION: Adds callback called before physics step
//      */
//     void PhysicsManager::registerPreStepCallback(const PreStepCallback& callback) {
//         preStepCallbacks_.push_back(callback);
//         std::cout << "[Physics] Registered pre-step callback (total: " << preStepCallbacks_.size() << ")" << std::endl;
//     }
//
//     /**
//      * @brief Register post-step callback
//      * IMPLEMENTATION: Adds callback called after physics step
//      */
//     void PhysicsManager::registerPostStepCallback(const PostStepCallback& callback) {
//         postStepCallbacks_.push_back(callback);
//         std::cout << "[Physics] Registered post-step callback (total: " << postStepCallbacks_.size() << ")" <<
//             std::endl;
//     }
//
//     // ========================================================================
//     // DEBUG AND STATISTICS
//     // ========================================================================
//
//     /**
//      * @brief Enable/disable debug drawing
//      * IMPLEMENTATION: Toggles physics debug visualization
//      */
//     void PhysicsManager::setDebugDrawEnabled(bool enabled) {
//         config_.enableDebugDraw = enabled;
//
//         if (enabled && !debugDrawer_) {
//             debugDrawer_ = std::make_unique<PhysicsDebugDrawer>();
//             if (dynamicsWorld_) {
//                 dynamicsWorld_->setDebugDrawer(debugDrawer_.get());
//             }
//             std::cout << "[Physics] Debug drawing enabled" << std::endl;
//         }
//         else if (!enabled && debugDrawer_) {
//             if (dynamicsWorld_) {
//                 dynamicsWorld_->setDebugDrawer(nullptr);
//             }
//             debugDrawer_.reset();
//             std::cout << "[Physics] Debug drawing disabled" << std::endl;
//         }
//     }
//
//     /**
//      * @brief Set debug draw mode flags
//      * IMPLEMENTATION: Configures what to draw in debug mode
//      */
//     void PhysicsManager::setDebugDrawMode(int mode) {
//         config_.debugDrawMode = mode;
//
//         if (debugDrawer_) {
//             debugDrawer_->setDebugMode(mode);
//         }
//     }
//
//     /**
//      * @brief Debug draw the physics world
//      * IMPLEMENTATION: Renders physics debug visualization
//      */
//     void PhysicsManager::debugDraw(void* renderer) {
//         if (!initialized_ || !dynamicsWorld_ || !debugDrawer_) {
//             return;
//         }
//
//         // Set renderer for drawing
//         debugDrawer_->setRenderer(renderer);
//
//         // Trigger debug drawing
//         dynamicsWorld_->debugDrawWorld();
//     }
//
//     /**
//      * @brief Get physics statistics
//      * IMPLEMENTATION: Returns current performance metrics
//      */
//     PhysicsStatistics PhysicsManager::getStatistics() const {
//         return statistics_;
//     }
//
//     /**
//      * @brief Get debug info string
//      * IMPLEMENTATION: Returns detailed debug information
//      */
//     std::string PhysicsManager::getDebugInfo() const {
//         std::ostringstream oss;
//
//         oss << "=== Physics Manager Debug Info ===\n";
//         oss << "Initialized: " << (initialized_ ? "Yes" : "No") << "\n";
//         oss << "Simulation speed: " << simulationSpeed_ << "x\n";
//         oss << "Fixed timestep: " << config_.fixedTimeStep << "s (" << (1.0f / config_.fixedTimeStep) << " Hz)\n";
//         oss << "Gravity: " << config_.gravity.x << ", " << config_.gravity.y << ", " << config_.gravity.z << "\n\n";
//
//         oss << "=== Resources ===\n";
//         oss << "Collision shapes: " << shapes_.size() << "\n";
//         oss << "Rigid bodies: " << bodies_.size() << "\n";
//         oss << "Constraints: " << constraints_.size() << "\n";
//         oss << "Active collisions: " << activeCollisions_.size() << "\n\n";
//
//         oss << "=== Statistics ===\n";
//         oss << "Active bodies: " << statistics_.numActiveBodies << "\n";
//         oss << "Sleeping bodies: " << statistics_.numSleepingBodies << "\n";
//         oss << "Contact points: " << statistics_.numContacts << "\n";
//         oss << "Contact manifolds: " << statistics_.numManifolds << "\n";
//         oss << "Broadphase pairs: " << statistics_.broadphasePairs << "\n";
//         oss << "Raycasts performed: " << statistics_.numRaycasts << "\n";
//         oss << "Sweep tests performed: " << statistics_.numSweepTests << "\n\n";
//
//         oss << "=== Performance ===\n";
//         oss << "Simulation time: " << statistics_.simulationTime << " ms\n";
//         oss << "Collision time: " << statistics_.collisionTime << " ms\n";
//         oss << "Solver time: " << statistics_.solverTime << " ms\n";
//         oss << "Total memory: " << (statistics_.totalMemory / 1024) << " KB\n";
//
//         return oss.str();
//     }
//
//     // ========================================================================
//     // UTILITY FUNCTIONS
//     // ========================================================================
//
//     /**
//      * @brief Clear all forces on all bodies
//      * IMPLEMENTATION: Removes all accumulated forces
//      */
//     void PhysicsManager::clearAllForces() {
//         if (!dynamicsWorld_) {
//             return;
//         }
//
//         dynamicsWorld_->clearForces();
//
//         // Also clear forces on individual bodies
//         for (auto& [id, body] : bodies_) {
//             body->clearForces();
//         }
//     }
//
//     /**
//      * @brief Reset simulation
//      * IMPLEMENTATION: Resets all bodies to initial state
//      */
//     void PhysicsManager::resetSimulation() {
//         if (!initialized_) {
//             return;
//         }
//
//         std::lock_guard<std::mutex> lock(physicsMutex_);
//
//         std::cout << "[Physics] Resetting simulation..." << std::endl;
//
//         // Reset all bodies to initial state
//         for (auto& [id, body] : bodies_) {
//             body->reset();
//         }
//
//         // Clear collision tracking
//         activeCollisions_.clear();
//         collisionEvents_.clear();
//
//         // Reset timing
//         accumulator_ = 0.0f;
//         lastUpdateTime_ = std::chrono::high_resolution_clock::now();
//
//         // Reset statistics
//         statistics_ = PhysicsStatistics{};
//
//         std::cout << "[Physics] Simulation reset complete" << std::endl;
//     }
//
//     /**
//      * @brief Get number of active bodies
//      * IMPLEMENTATION: Counts bodies that are not sleeping
//      */
//     int PhysicsManager::getActiveBodyCount() const {
//         int count = 0;
//
//         for (const auto& [id, body] : bodies_) {
//             if (body->isActive()) {
//                 count++;
//             }
//         }
//
//         return count;
//     }
//
//     /**
//      * @brief Get total body count
//      * IMPLEMENTATION: Returns total number of bodies
//      */
//     int PhysicsManager::getTotalBodyCount() const {
//         return static_cast<int>(bodies_.size());
//     }
//
//     // ========================================================================
//     // BULLET TO GLM CONVERSIONS (Static Helper Methods)
//     // ========================================================================
//
//     /**
//      * @brief Convert GLM Vector3 to Bullet btVector3
//      * IMPLEMENTATION: Direct component conversion
//      */
//     btVector3 PhysicsManager::toBullet(const Vector3& v) {
//         return btVector3(v.x, v.y, v.z);
//     }
//
//     /**
//      * @brief Convert Bullet btVector3 to GLM Vector3
//      * IMPLEMENTATION: Direct component extraction
//      */
//     Vector3 PhysicsManager::fromBullet(const btVector3& v) {
//         return Vector3(v.x(), v.y(), v.z());
//     }
//
//     /**
//      * @brief Convert GLM Quaternion to Bullet btQuaternion
//      * IMPLEMENTATION: Component conversion with order adjustment
//      */
//     btQuaternion PhysicsManager::toBullet(const Quaternion& q) {
//         // GLM quaternion is (w, x, y, z) but we access as (x, y, z, w)
//         return btQuaternion(q.x, q.y, q.z, q.w);
//     }
//
//     /**
//      * @brief Convert Bullet btQuaternion to GLM Quaternion
//      * IMPLEMENTATION: Component extraction with order adjustment
//      */
//     Quaternion PhysicsManager::fromBullet(const btQuaternion& q) {
//         // Create GLM quaternion (w, x, y, z)
//         return Quaternion(q.w(), q.x(), q.y(), q.z());
//     }
//
//     /**
//      * @brief Convert Transform3D to Bullet btTransform
//      * IMPLEMENTATION: Full transform conversion
//      */
//     btTransform PhysicsManager::toBullet(const Transform3D& t) {
//         btTransform transform;
//         transform.setOrigin(toBullet(t.position));
//         transform.setRotation(toBullet(t.rotation));
//         // Note: Bullet doesn't directly support non-uniform scaling in transforms
//         return transform;
//     }
//
//     /**
//      * @brief Convert Bullet btTransform to Transform3D
//      * IMPLEMENTATION: Full transform extraction
//      */
//     Transform3D PhysicsManager::fromBullet(const btTransform& t) {
//         Transform3D transform;
//         transform.position = fromBullet(t.getOrigin());
//         transform.rotation = fromBullet(t.getRotation());
//         transform.scale = Vector3(1.0f, 1.0f, 1.0f); // Default scale
//         return transform;
//     }
// } // namespace engine::physics
