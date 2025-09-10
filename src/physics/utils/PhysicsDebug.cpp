/**
 * @file PhysicsDebug.h
 * @brief Physics debug visualization and diagnostics
 * @details Provides debug drawing, performance visualization, and
 *          diagnostic tools for physics development
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#include "PhysicsDebug.h"

namespace engine::physics {
    PhysicsDebugDrawer::PhysicsDebugDrawer() : debugMode_(0) {
    }

    void PhysicsDebugDrawer::drawLine(const btVector3& from, const btVector3& to,
                                      const btVector3& color) {
        lines_.emplace_back(
            Vec3(from.x(), from.y(), from.z()),
            Vec3(to.x(), to.y(), to.z()),
            Color(color.x(), color.y(), color.z())
        );
    }

    void PhysicsDebugDrawer::drawContactPoint(const btVector3& pointOnB, const btVector3& normalOnB,
                                              const btScalar distance, int lifeTime,
                                              const btVector3& color) {
        const Vec3 p(pointOnB.x(), pointOnB.y(), pointOnB.z());
        const Vec3 n(normalOnB.x(), normalOnB.y(), normalOnB.z());
        const Color c(color.x(), color.y(), color.z());

        // Draw contact point as small cross
        drawCross(p, 0.1f, c);

        // Draw normal
        drawLine(pointOnB, pointOnB + normalOnB * 0.5f, color);

        // Store contact info
        contactPoints_.push_back({p, n, static_cast<Float>(distance)});
    }

    void PhysicsDebugDrawer::drawTriangle(const btVector3& v0, const btVector3& v1, const btVector3& v2,
                                          const btVector3& color, const btScalar alpha) {
        triangles_.emplace_back(
            Vec3(v0.x(), v0.y(), v0.z()),
            Vec3(v1.x(), v1.y(), v1.z()),
            Vec3(v2.x(), v2.y(), v2.z()),
            Color(color.x(), color.y(), color.z(), alpha)
        );
    }

    void PhysicsDebugDrawer::drawAABB(const AABB& aabb, const Color& color) {
        const auto corners = aabb.getCorners();

        // Bottom face
        drawDebugLine(corners[0], corners[1], color);
        drawDebugLine(corners[1], corners[3], color);
        drawDebugLine(corners[3], corners[2], color);
        drawDebugLine(corners[2], corners[0], color);

        // Top face
        drawDebugLine(corners[4], corners[5], color);
        drawDebugLine(corners[5], corners[7], color);
        drawDebugLine(corners[7], corners[6], color);
        drawDebugLine(corners[6], corners[4], color);

        // Vertical edges
        drawDebugLine(corners[0], corners[4], color);
        drawDebugLine(corners[1], corners[5], color);
        drawDebugLine(corners[2], corners[6], color);
        drawDebugLine(corners[3], corners[7], color);
    }

    void PhysicsDebugDrawer::drawSphereWireframe(const Vec3& center, const Float radius,
                        const Color& color, const Int segments) {
        // Draw three circles
        for (int i = 0; i < segments; ++i) {
            const Float angle1 = (i * TWO_PI<Float>) / segments;
            const Float angle2 = ((i + 1) * TWO_PI<Float>) / segments;

            // XY plane
            Vec3 p1 = center + Vec3(std::cos(angle1) * radius, std::sin(angle1) * radius, 0);
            Vec3 p2 = center + Vec3(std::cos(angle2) * radius, std::sin(angle2) * radius, 0);
            drawDebugLine(p1, p2, color);

            // XZ plane
            p1 = center + Vec3(std::cos(angle1) * radius, 0, std::sin(angle1) * radius);
            p2 = center + Vec3(std::cos(angle2) * radius, 0, std::sin(angle2) * radius);
            drawDebugLine(p1, p2, color);

            // YZ plane
            p1 = center + Vec3(0, std::cos(angle1) * radius, std::sin(angle1) * radius);
            p2 = center + Vec3(0, std::cos(angle2) * radius, std::sin(angle2) * radius);
            drawDebugLine(p1, p2, color);
        }
    }

    void PhysicsDebugDrawer::drawCapsuleWireframe(const Vec3& start, const Vec3& end, Float radius,
                         const Color& color) {
        // Draw cylinder body
        const Vec3 axis = glm::normalize(end - start);
        Vec3 perpX = glm::abs(axis.x) < 0.9f ? Vec3(1, 0, 0) : Vec3(0, 1, 0);
        Vec3 perpY = glm::normalize(glm::cross(axis, perpX));
        perpX = glm::cross(perpY, axis);

        constexpr int segments = 16;
        for (int i = 0; i < segments; ++i) {
            const Float angle1 = (i * TWO_PI<Float>) / segments;
            const Float angle2 = ((i + 1) * TWO_PI<Float>) / segments;

            Vec3 offset1 = (perpX * std::cos(angle1) + perpY * std::sin(angle1)) * radius;
            Vec3 offset2 = (perpX * std::cos(angle2) + perpY * std::sin(angle2)) * radius;

            drawDebugLine(start + offset1, start + offset2, color);
            drawDebugLine(end + offset1, end + offset2, color);
            drawDebugLine(start + offset1, end + offset1, color);
        }

        // Draw hemisphere caps
        drawSphereWireframe(start, radius, color, 8);
        drawSphereWireframe(end, radius, color, 8);
    }

    void PhysicsDebugDrawer::drawArrow(const Vec3& from, const Vec3& to,
                       const Color& color, const Float headSize) {
        drawDebugLine(from, to, color);

        const Vec3 dir = glm::normalize(to - from);
        Vec3 perpX = glm::abs(dir.x) < 0.9f ? Vec3(1, 0, 0) : Vec3(0, 1, 0);
        Vec3 perpY = glm::normalize(glm::cross(dir, perpX)) * headSize;
        perpX = glm::cross(perpY, dir) * headSize;

        const Vec3 headBase = to - dir * headSize * 2.0f;

        drawDebugLine(to, headBase + perpX, color);
        drawDebugLine(to, headBase - perpX, color);
        drawDebugLine(to, headBase + perpY, color);
        drawDebugLine(to, headBase - perpY, color);
    }

    void PhysicsDebugDrawer::drawTransformAxes(const Mat4& transform, const Float size) {
        const auto origin = Vec3(transform[3]);
        const Vec3 x = Vec3(transform[0]) * size;
        const Vec3 y = Vec3(transform[1]) * size;
        const Vec3 z = Vec3(transform[2]) * size;

        drawArrow(origin, origin + x, Color::Red, size * 0.1f);
        drawArrow(origin, origin + y, Color::Green, size * 0.1f);
        drawArrow(origin, origin + z, Color::Blue, size * 0.1f);
    }

    void PhysicsDebugDrawer::drawCross(const Vec3& position, const Float size, const Color& color) {
        drawDebugLine(position - Vec3(size, 0, 0), position + Vec3(size, 0, 0), color);
        drawDebugLine(position - Vec3(0, size, 0), position + Vec3(0, size, 0), color);
        drawDebugLine(position - Vec3(0, 0, size), position + Vec3(0, 0, size), color);
    }

    void PhysicsDebugDrawer::drawFrustumWireframe(const Frustum& frustum, const Color& color) {
        // This would calculate and draw frustum corners
        // Implementation depends on frustum representation
    }

    void PhysicsDebugDrawer::drawConstraint(btTypedConstraint* constraint) {
        if (!constraint) return;

        // Get constraint frames
        const btTransform& frameA = constraint->getRigidBodyA().getWorldTransform();
        const btTransform& frameB = constraint->getRigidBodyB().getWorldTransform();

        const Vec3 posA(frameA.getOrigin().x(), frameA.getOrigin().y(), frameA.getOrigin().z());
        const Vec3 posB(frameB.getOrigin().x(), frameB.getOrigin().y(), frameB.getOrigin().z());

        // Draw connection
        drawDebugLine(posA, posB, Color::Magenta);

        // Draw constraint frames
        drawTransformAxes(convertBulletTransform(frameA), 0.5f);
        drawTransformAxes(convertBulletTransform(frameB), 0.5f);
    }

    void PhysicsDebugDrawer::clear() {
        lines_.clear();
        triangles_.clear();
        texts_.clear();
        contactPoints_.clear();
        warnings_.clear();
    }

    void PhysicsDebugDrawer::update(Float deltaTime) const {
        // Update lifetimes and remove expired
        auto updateLifetime = [deltaTime](auto& container) {
            container.erase(
                std::remove_if(container.begin(), container.end(),
                               [deltaTime](auto& item) {
                                   if (item.lifetime > 0) {
                                       item.lifetime -= deltaTime;
                                       return item.lifetime <= 0;
                                   }
                                   return false;
                               }),
                container.end()
            );
        };

        // Note: DebugLine doesn't have lifetime in our simple struct
        // You'd need to add it for timed debug draws
    }

    std::string PhysicsDebugDrawer::getDebugStats() const {
        std::stringstream ss;
        ss << "Debug Draw Stats:\n";
        ss << "  Lines: " << lines_.size() << "\n";
        ss << "  Triangles: " << triangles_.size() << "\n";
        ss << "  Texts: " << texts_.size() << "\n";
        ss << "  Contacts: " << contactPoints_.size() << "\n";

        if (!warnings_.empty()) {
            ss << "Warnings:\n";
            for (const auto& warning : warnings_) {
                ss << "  " << warning << "\n";
            }
        }

        return ss.str();
    }

    Mat4 PhysicsDebugDrawer::convertBulletTransform(const btTransform& transform) {
        btScalar matrix[16];
        transform.getOpenGLMatrix(matrix);

        return Mat4(
            matrix[0], matrix[1], matrix[2], matrix[3],
            matrix[4], matrix[5], matrix[6], matrix[7],
            matrix[8], matrix[9], matrix[10], matrix[11],
            matrix[12], matrix[13], matrix[14], matrix[15]
        );
    }
} // namespace engine::physics
