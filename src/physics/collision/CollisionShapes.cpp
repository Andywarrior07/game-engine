/**
 * @file CollisionShapes.cpp
 * @brief Collision shape creation and management
 * @details Provides RAII wrappers for Bullet collision shapes with
 *          automatic memory management and optimization features
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#include "CollisionShapes.h"

#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>

namespace engine::physics {
    CollisionShape::~CollisionShape() {
        if (bulletShape_) {
            delete bulletShape_;
        }
    }

    void CollisionShape::release() {
        if (--refCount_ == 0) {
            delete this;
        }
    }

    Vec3 CollisionShape::getHalfExtents() const {
        if (!bulletShape_) return Vec3(0);

        btVector3 min, max;
        bulletShape_->getAabb(btTransform::getIdentity(), min, max);
        const btVector3 halfExtents = (max - min) * 0.5f;

        return Vec3(halfExtents.x(), halfExtents.y(), halfExtents.z());
    }

    void CollisionShape::setMargin(const Float margin) const {
        if (bulletShape_) {
            bulletShape_->setMargin(margin);
        }
    }

    Float CollisionShape::getMargin() const {
        return bulletShape_ ? bulletShape_->getMargin() : 0.0f;
    }

    void CollisionShape::setLocalScaling(const Vec3& scale) const {
        if (bulletShape_) {
            bulletShape_->setLocalScaling(btVector3(scale.x, scale.y, scale.z));
        }
    }

    Vec3 CollisionShape::getLocalScaling() const {
        if (!bulletShape_) return Vec3(1);

        const btVector3& scale = bulletShape_->getLocalScaling();
        return Vec3(scale.x(), scale.y(), scale.z());
    }

    void BoxShape::reset() {
        CollisionShape::reset();
        halfExtents_ = Vec3(0.5f);
    }

    void SphereShape::reset() {
        CollisionShape::reset();
        radius_ = 0.5f;
    }

    Float CapsuleShape::getVolume() const {
        // Volume of cylinder + volume of sphere
        const Float cylinderVolume = PI<Float> * radius_ * radius_ * height_;
        const Float sphereVolume = (4.0f / 3.0f) * PI<Float> * radius_ * radius_ * radius_;

        return cylinderVolume + sphereVolume;
    }

    void CapsuleShape::reset() {
        CollisionShape::reset();
        radius_ = 0.5f;
        height_ = 1.0f;
    }

    void CylinderShape::reset() {
        CollisionShape::reset();
        halfExtents_ = Vec3(0.5f, 1.0f, 0.5f);
    }

    void ConeShape::reset() {
        CollisionShape::reset();
        radius_ = 0.5f;
        height_ = 1.0f;
    }

    ConvexHullShape::ConvexHullShape(const Float* vertices, const std::size_t vertexCount,
                        const std::size_t stride)
            : CollisionShape(nullptr, ShapeType::CONVEX_HULL) {
        auto* hull = new btConvexHullShape();

        for (std::size_t i = 0; i < vertexCount; ++i) {
            const auto vertex = reinterpret_cast<const Float*>(
                reinterpret_cast<const char*>(vertices) + i * stride
            );
            hull->addPoint(btVector3(vertex[0], vertex[1], vertex[2]));
        }

        // Optimize hull if too many vertices
        if (vertexCount > 256) {
            hull->optimizeConvexHull();
        }

        bulletShape_ = hull;
        vertexCount_ = vertexCount;
    }

    void ConvexHullShape::reset() {
        CollisionShape::reset();
        vertexCount_ = 0;
    }

    TriangleMeshShape::TriangleMeshShape(const Float* vertices, const std::size_t vertexCount,
                          const std::uint32_t* indices, const std::size_t indexCount,
                          const std::size_t vertexStride)
            : CollisionShape(nullptr, ShapeType::TRIANGLE_MESH) {
        // Create mesh data
        meshData_ = new btTriangleMesh();

        for (std::size_t i = 0; i < indexCount; i += 3) {
            const std::uint32_t i0 = indices[i];
            const std::uint32_t i1 = indices[i + 1];
            const std::uint32_t i2 = indices[i + 2];

            const auto v0 = reinterpret_cast<const Float*>(
                reinterpret_cast<const char*>(vertices) + i0 * vertexStride
            );
            const auto v1 = reinterpret_cast<const Float*>(
                reinterpret_cast<const char*>(vertices) + i1 * vertexStride
            );
            const auto v2 = reinterpret_cast<const Float*>(
                reinterpret_cast<const char*>(vertices) + i2 * vertexStride
            );

            meshData_->addTriangle(
                btVector3(v0[0], v0[1], v0[2]),
                btVector3(v1[0], v1[1], v1[2]),
                btVector3(v2[0], v2[1], v2[2])
            );
        }

        // Create BVH-optimized mesh shape
        bulletShape_ = new btBvhTriangleMeshShape(meshData_, true, true);
        vertexCount_ = vertexCount;
        triangleCount_ = indexCount / 3;
    }

    void TriangleMeshShape::reset() {
        CollisionShape::reset();
        delete meshData_;
        meshData_ = nullptr;
        vertexCount_ = 0;
        triangleCount_ = 0;
    }

    HeightfieldShape::HeightfieldShape(const Float* heightData, const Int width, const Int height,
                         const Float minHeight, const Float maxHeight, const Vec3& scale)
            : CollisionShape(nullptr, ShapeType::HEIGHTFIELD),
              width_(width), height_(height) {
        // Copy height data (Bullet doesn't own it)
        heightData_.resize(width * height);
        std::memcpy(heightData_.data(), heightData, width * height * sizeof(Float));

        bulletShape_ = new btHeightfieldTerrainShape(
            width, height,
            heightData_.data(),
            1.0f, // Height scale factor
            minHeight, maxHeight,
            1, // Y up axis
            PHY_FLOAT,
            false // Flip quad edges
        );

        bulletShape_->setLocalScaling(btVector3(scale.x, scale.y, scale.z));
    }

    Float HeightfieldShape::getHeightAt(const Int x, const Int z) const {
        if (x >= 0 && x < width_ && z >= 0 && z < height_) {
            return heightData_[z * width_ + x];
        }
        return 0.0f;
    }

    void HeightfieldShape::reset() {
        CollisionShape::reset();
        heightData_.clear();
        width_ = 0;
        height_ = 0;
    }

    void CompoundShape::addChildShape(const Transform& localTransform, CollisionShape* shape) {
        if (!shape) return;

        btTransform transform;
        transform.setOrigin(btVector3(
            localTransform.getPosition().x,
            localTransform.getPosition().y,
            localTransform.getPosition().z
        ));
        transform.setRotation(btQuaternion(
            localTransform.getRotation().x,
            localTransform.getRotation().y,
            localTransform.getRotation().z,
            localTransform.getRotation().w
        ));

        static_cast<btCompoundShape*>(bulletShape_)->addChildShape(
            transform, shape->getBulletShape()
        );

        shape->addRef();
        childShapes_.push_back(shape);
    }

    void CompoundShape::removeChildShape(const CollisionShape* shape) {
        if (!shape) return;

        static_cast<btCompoundShape*>(bulletShape_)->removeChildShape(
            shape->getBulletShape()
        );

        if (const auto it = std::ranges::find(childShapes_, shape); it != childShapes_.end()) {
            (*it)->release();
            childShapes_.erase(it);
        }
    }
    std::size_t CompoundShape::getNumChildShapes() const {
        return static_cast<btCompoundShape*>(bulletShape_)->getNumChildShapes();
    }

    void CompoundShape::reset() {
        CollisionShape::reset();
        for (auto* shape : childShapes_) {
            shape->release();
        }

        childShapes_.clear();

        if (bulletShape_) {
            delete bulletShape_;
            bulletShape_ = new btCompoundShape(true);
        }
    }

    void PlaneShape::reset() {
        CollisionShape::reset();
        m_normal = Vec3(0, 1, 0);
        m_distance = 0.0f;
    }
} // namespace engine::physics
