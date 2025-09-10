/**
 * @file PhysicsDebug.h
 * @brief Physics debug visualization and diagnostics
 * @details Provides debug drawing, performance visualization, and
 *          diagnostic tools for physics development
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

#include "../core/PhysicsTypes.h"
#include <LinearMath/btIDebugDraw.h>
#include <vector>
#include <sstream>

namespace engine::physics {
    /**
     * @brief Debug line for rendering
     */
    struct DebugLine {
        Vec3 from;
        Vec3 to;
        Color color;
        Float lifetime;

        DebugLine(const Vec3& f, const Vec3& t, const Color& c, const Float life = 0.0f)
            : from(f), to(t), color(c), lifetime(life) {
        }
    };

    /**
     * @brief Debug triangle for rendering
     */
    struct DebugTriangle {
        Vec3 v0, v1, v2;
        Color color;
        Float lifetime;

        DebugTriangle(const Vec3& a, const Vec3& b, const Vec3& c,
                      const Color& col, const Float life = 0.0f)
            : v0(a), v1(b), v2(c), color(col), lifetime(life) {
        }
    };

    /**
     * @brief Debug text for rendering
     */
    struct DebugText {
        Vec3 position;
        std::string text;
        Color color;
        Float size;
        Float lifetime;

        DebugText(const Vec3& pos, const std::string& txt,
                  const Color& c, const Float s = 1.0f, const Float life = 0.0f)
            : position(pos), text(txt), color(c), size(s), lifetime(life) {
        }
    };

    /**
     * @brief Physics debug drawer implementation
     * @details Implements Bullet's debug draw interface and provides
     *          additional debug visualization features
     */
    class PhysicsDebugDrawer final : public btIDebugDraw {
    public:
        PhysicsDebugDrawer();

        // ============================================================================
        // Bullet Debug Draw Interface
        // ============================================================================

        void drawLine(const btVector3& from, const btVector3& to,
                      const btVector3& color) override;

        void drawContactPoint(const btVector3& pointOnB, const btVector3& normalOnB,
                              btScalar distance, int lifeTime,
                              const btVector3& color) override;

        void drawTriangle(const btVector3& v0, const btVector3& v1, const btVector3& v2,
                          const btVector3& color, btScalar alpha) override;

        void reportErrorWarning(const char* warningString) override {
            warnings_.push_back(warningString);
        }

        void draw3dText(const btVector3& location, const char* textString) override {
            texts_.emplace_back(
                Vec3(location.x(), location.y(), location.z()),
                textString,
                Color::White
            );
        }

        void setDebugMode(const int debugMode) override {
            debugMode_ = debugMode;
        }

        int getDebugMode() const override {
            return debugMode_;
        }

        // ============================================================================
        // Extended Debug Features
        // ============================================================================

        /**
         * @brief Draw AABB
         */
        void drawAABB(const AABB& aabb, const Color& color = Color::Green);

        /**
         * @brief Draw sphere wireframe
         */
        void drawSphereWireframe(const Vec3& center, Float radius,
                        const Color& color = Color::Blue, Int segments = 16);

        /**
         * @brief Draw capsule
         */
        void drawCapsuleWireframe(const Vec3& start, const Vec3& end, Float radius,
                         const Color& color = Color::Cyan);

        /**
         * @brief Draw arrow
         */
        void drawArrow(const Vec3& from, const Vec3& to,
                       const Color& color = Color::Red, Float headSize = 0.2f);

        /**
         * @brief Draw transform axes
         */
        void drawTransformAxes(const Mat4& transform, Float size = 1.0f);

        /**
         * @brief Draw cross marker
         */
        void drawCross(const Vec3& position, Float size, const Color& color);

        /**
         * @brief Draw frustum
         */
        static void drawFrustumWireframe(const Frustum& frustum, const Color& color = Color::Yellow);

        /**
         * @brief Draw physics constraint
         */
        void drawConstraint(btTypedConstraint* constraint);

        // ============================================================================
        // Render Queue Access
        // ============================================================================

        const std::vector<DebugLine>& getLines() const { return lines_; }
        const std::vector<DebugTriangle>& getTriangles() const { return triangles_; }
        const std::vector<DebugText>& getTexts() const { return texts_; }

        /**
         * @brief Clear all debug primitives
         */
        void clear();

        /**
         * @brief Update debug primitives (remove expired ones)
         */
        void update(Float deltaTime) const;

        /**
         * @brief Get debug statistics string
         */
        std::string getDebugStats() const;

    private:
        int debugMode_;
        std::vector<DebugLine> lines_;
        std::vector<DebugTriangle> triangles_;
        std::vector<DebugText> texts_;
        std::vector<std::string> warnings_;

        struct ContactInfo {
            Vec3 point;
            Vec3 normal;
            Float distance;
        };

        std::vector<ContactInfo> contactPoints_;

        void drawDebugLine(const Vec3& from, const Vec3& to, const Color& color) {
            lines_.emplace_back(from, to, color);
        }

        static Mat4 convertBulletTransform(const btTransform& transform);
    };
} // namespace engine::physics
