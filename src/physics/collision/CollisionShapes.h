/**
 * @file CollisionShapes.h
 * @brief Collision shape creation and management
 * @details Provides RAII wrappers for Bullet collision shapes with
 *          automatic memory management and optimization features
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

#include "../core/PhysicsTypes.h"

// #include <btBulletCollisionCommon.h>
// #include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
// #include <BulletCollision/CollisionShapes/btConvexHullShape.h>

namespace engine::physics {
    /**
     * @brief Base collision shape wrapper
     * @details RAII wrapper for Bullet collision shapes with reference counting
     */
    class CollisionShape {
    public:
        explicit CollisionShape(btCollisionShape* shape, const ShapeType type)
            : bulletShape_(shape), type_(type), refCount_(1) {
        }

        virtual ~CollisionShape();

        // FIXED: Default constructor for ObjectPool compatibility
        CollisionShape() : bulletShape_(nullptr), type_(ShapeType::BOX), refCount_(1) {
        }

        // Reference counting for shape sharing
        void addRef() { ++refCount_; }

        void release();

        btCollisionShape* getBulletShape() const { return bulletShape_; }
        ShapeType getType() const { return type_; }

        /**
         * @brief Get shape dimensions
         */
        Vec3 getHalfExtents() const;

        /**
         * @brief Get shape volume
         */
        virtual Float getVolume() const {
            // Default implementation - override for specific shapes
            const Vec3 extents = getHalfExtents() * 2.0f;
            return extents.x * extents.y * extents.z;
        }

        /**
         * @brief Calculate mass from density
         */
        Float calculateMass(const Float density) const {
            return getVolume() * density;
        }

        /**
         * @brief Set collision margin
         */
        void setMargin(const Float margin) const;

        Float getMargin() const;

        /**
         * @brief Set local scaling
         */
        void setLocalScaling(const Vec3& scale) const;

        Vec3 getLocalScaling() const;

        /**
         * @brief Reset shape for reuse (for ObjectPool)
         */
        virtual void reset() {
            if (bulletShape_) {
                delete bulletShape_;
                bulletShape_ = nullptr;
            }
            type_ = ShapeType::BOX;
            refCount_ = 1;
        }

    protected:
        btCollisionShape* bulletShape_;
        ShapeType type_;
        std::atomic<int> refCount_;
    };

    /**
     * @brief Box collision shape
     */
    class BoxShape final : public CollisionShape {
    public:
        explicit BoxShape(const Vec3& halfExtents)
            : CollisionShape(
                  new btBoxShape(btVector3(halfExtents.x, halfExtents.y, halfExtents.z)),
                  ShapeType::BOX
              ),
              halfExtents_(halfExtents) {
        }

        // Default constructor for ObjectPool
        BoxShape() : CollisionShape(), halfExtents_(Vec3(0.5f)) {
        }

        const Vec3& getBoxHalfExtents() const { return halfExtents_; }

        Float getVolume() const override {
            return 8.0f * halfExtents_.x * halfExtents_.y * halfExtents_.z;
        }

        void reset() override;

    private:
        Vec3 halfExtents_;
    };

    /**
     * @brief Sphere collision shape
     */
    class SphereShape final : public CollisionShape {
    public:
        explicit SphereShape(const Float radius)
            : CollisionShape(new btSphereShape(radius), ShapeType::SPHERE),
              radius_(radius) {
        }

        // Default constructor for ObjectPool
        SphereShape() : radius_(0.5f) {
        }

        Float getRadius() const { return radius_; }

        Float getVolume() const override {
            return (4.0f / 3.0f) * PI<Float> * radius_ * radius_ * radius_;
        }

        void reset() override;

    private:
        Float radius_;
    };

    /**
     * @brief Capsule collision shape
     */
    class CapsuleShape final : public CollisionShape {
    public:
        CapsuleShape(const Float radius, const Float height)
            : CollisionShape(new btCapsuleShape(radius, height), ShapeType::CAPSULE),
              radius_(radius), height_(height) {
        }

        // Default constructor for ObjectPool
        CapsuleShape() : radius_(0.5f), height_(1.0f) {
        }

        Float getRadius() const { return radius_; }
        Float getHeight() const { return height_; }

        Float getVolume() const override;

        void reset() override;

    private:
        Float radius_;
        Float height_;
    };

    /**
     * @brief Cylinder collision shape
     */
    class CylinderShape final : public CollisionShape {
    public:
        explicit CylinderShape(const Vec3& halfExtents)
            : CollisionShape(
                  new btCylinderShape(btVector3(halfExtents.x, halfExtents.y, halfExtents.z)),
                  ShapeType::CYLINDER
              ),
              halfExtents_(halfExtents) {
        }

        // Default constructor for ObjectPool
        CylinderShape() : halfExtents_(Vec3(0.5f, 1.0f, 0.5f)) {
        }

        const Vec3& getCylinderHalfExtents() const { return halfExtents_; }

        Float getVolume() const override {
            // Assuming Y is the height axis
            return PI<Float> * halfExtents_.x * halfExtents_.z * halfExtents_.y * 2.0f;
        }

        void reset() override;

    private:
        Vec3 halfExtents_;
    };

    /**
     * @brief Cone collision shape
     */
    class ConeShape final : public CollisionShape {
    public:
        ConeShape(const Float radius, const Float height)
            : CollisionShape(new btConeShape(radius, height), ShapeType::CONE),
              radius_(radius), height_(height) {
        }

        // Default constructor for ObjectPool
        ConeShape() : radius_(0.5f), height_(1.0f) {
        }

        Float getRadius() const { return radius_; }
        Float getHeight() const { return height_; }

        Float getVolume() const override {
            return (1.0f / 3.0f) * PI<Float> * radius_ * radius_ * height_;
        }

        void reset() override;

    private:
        Float radius_;
        Float height_;
    };

    /**
     * @brief Convex hull collision shape
     */
    class ConvexHullShape final : public CollisionShape {
    public:
        ConvexHullShape(const Float* vertices, const std::size_t vertexCount,
                        const std::size_t stride = sizeof(Vec3));

        // Default constructor for ObjectPool
        ConvexHullShape() : CollisionShape(), vertexCount_(0) {
        }

        std::size_t getVertexCount() const { return vertexCount_; }

        void reset() override;

    private:
        std::size_t vertexCount_;
    };

    /**
     * @brief Triangle mesh collision shape (for static bodies only)
     */
    class TriangleMeshShape final : public CollisionShape {
    public:
        TriangleMeshShape(const Float* vertices, const std::size_t vertexCount,
                          const std::uint32_t* indices, const std::size_t indexCount,
                          const std::size_t vertexStride = sizeof(Vec3));

        // Default constructor for ObjectPool
        TriangleMeshShape() : CollisionShape(), meshData_(nullptr), vertexCount_(0), triangleCount_(0) {
        }

        ~TriangleMeshShape() override {
            delete meshData_;
        }

        std::size_t getVertexCount() const { return vertexCount_; }
        std::size_t getTriangleCount() const { return triangleCount_; }

        void reset() override;

    private:
        btTriangleMesh* meshData_;
        std::size_t vertexCount_;
        std::size_t triangleCount_;
    };

    /**
     * @brief Heightfield terrain shape
     */
    class HeightfieldShape final : public CollisionShape {
    public:
        HeightfieldShape(const Float* heightData, const Int width, const Int height,
                         const Float minHeight, const Float maxHeight, const Vec3& scale);

        // Default constructor for ObjectPool
        HeightfieldShape() : width_(0), height_(0) {
        }

        Int getWidth() const { return width_; }
        Int getHeight() const { return height_; }

        Float getHeightAt(Int x, Int z) const;

        void reset() override;

    private:
        std::vector<Float> heightData_;
        Int width_;
        Int height_;
    };

    /**
     * @brief Compound shape for complex objects
     */
    class CompoundShape final : public CollisionShape {
    public:
        explicit CompoundShape(const bool dynamicAabbTree = true)
            : CollisionShape(new btCompoundShape(dynamicAabbTree), ShapeType::COMPOUND) {
        }

        // Default constructor for ObjectPool
        CompoundShape() : CollisionShape() {
            bulletShape_ = new btCompoundShape(true);
            type_ = ShapeType::COMPOUND;
        }

        void addChildShape(const Transform& localTransform, CollisionShape* shape);

        void removeChildShape(const CollisionShape* shape);

        std::size_t getNumChildShapes() const;

        void reset() override;

        ~CompoundShape() override {
            for (auto* shape : childShapes_) {
                shape->release();
            }
        }

    private:
        std::vector<CollisionShape*> childShapes_;
    };

    /**
     * @brief Infinite plane shape
     */
    class PlaneShape final : public CollisionShape {
    public:
        PlaneShape(const Vec3& normal, const Float distance)
            : CollisionShape(
                  new btStaticPlaneShape(btVector3(normal.x, normal.y, normal.z), distance),
                  ShapeType::PLANE
              ),
              m_normal(normal), m_distance(distance) {
        }

        // Default constructor for ObjectPool
        PlaneShape() : CollisionShape(), m_normal(Vec3(0, 1, 0)), m_distance(0.0f) {
        }

        const Vec3& getNormal() const { return m_normal; }
        Float getDistance() const { return m_distance; }

        Float getVolume() const override { return 0.0f; } // Infinite

        void reset() override;

    private:
        Vec3 m_normal;
        Float m_distance;
    };

} // namespace engine::physics
