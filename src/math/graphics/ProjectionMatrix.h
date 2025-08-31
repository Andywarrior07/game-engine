/**
 * @file ProjectionMatrix.h
 * @brief Projection matrix construction for rendering
 *
 * Provides perspective, orthographic, and specialized projection
 * matrices for OpenGL and Vulkan rendering pipelines.
 */

#pragma once

#include "../core/MathTypes.h"
#include "../core/MathConstants.h"
#include "../geometry/Primitives.h"

namespace engine::math::graphics {
    /**
     * @brief Projection matrix builder for various rendering needs
     */
    class Projection {
    public:
        /**
         * @brief Depth range configuration
         */
        enum class DepthRange {
            ZERO_TO_ONE, // Vulkan/D3D style [0,1]
            NEG_ONE_TO_ONE // OpenGL style [-1,1]
        };

        /**
         * @brief Handedness configuration
         */
        enum class Handedness {
            LEFT_HANDED, // D3D style
            RIGHT_HANDED // OpenGL style
        };

        // ============================================================================
        // Perspective Projections
        // ============================================================================

        /**
         * @brief Create perspective projection matrix
         * @param fovY Vertical field of view in radians
         * @param aspectRatio Width/Height ratio
         * @param nearPlane Near clipping plane distance
         * @param farPlane Far clipping plane distance
         * @param depthRange Depth buffer range
         * @param handedness Coordinate system handedness
         */
        [[nodiscard]] static Mat4 perspective(
            Float fovY,
            Float aspectRatio,
            Float nearPlane,
            Float farPlane,
            DepthRange depthRange = DepthRange::NEG_ONE_TO_ONE,
            Handedness handedness = Handedness::RIGHT_HANDED) noexcept;

        /**
         * @brief Create perspective projection from frustum bounds
         */
        [[nodiscard]] static Mat4 frustum(
            Float left, Float right,
            Float bottom, Float top,
            Float nearPlane, Float farPlane,
            DepthRange depthRange = DepthRange::NEG_ONE_TO_ONE) noexcept;

        /**
         * @brief Create infinite perspective projection (for shadow mapping)
         */
        [[nodiscard]] static Mat4 infinitePerspective(
            Float fovY,
            Float aspectRatio,
            Float nearPlane,
            DepthRange depthRange = DepthRange::NEG_ONE_TO_ONE) noexcept;

        // ============================================================================
        // Orthographic Projections
        // ============================================================================

        /**
         * @brief Create orthographic projection matrix
         */
        [[nodiscard]] static Mat4 orthographic(
            Float left, Float right,
            Float bottom, Float top,
            Float nearPlane, Float farPlane,
            DepthRange depthRange = DepthRange::NEG_ONE_TO_ONE) noexcept;

        /**
         * @brief Create centered orthographic projection
         */
        [[nodiscard]] static Mat4 orthographicCentered(
            Float width, Float height,
            Float nearPlane, Float farPlane,
            DepthRange depthRange = DepthRange::NEG_ONE_TO_ONE) noexcept;

        // ============================================================================
        // Specialized Projections
        // ============================================================================

        /**
         * @brief Create reversed Z projection (better depth precision)
         */
        [[nodiscard]] static Mat4 perspectiveReversedZ(
            Float fovY,
            Float aspectRatio,
            Float nearPlane,
            Float farPlane = INFINITY_VALUE<Float>) noexcept;

        /**
         * @brief Create oblique projection matrix (for portals/mirrors)
         */
        [[nodiscard]] static Mat4 obliquePerspective(
            const Mat4& projection,
            const Vec4& clipPlane) noexcept;

        /**
         * @brief Create jittered projection for temporal anti-aliasing
         */
        [[nodiscard]] static Mat4 jitteredPerspective(
            Float fovY,
            Float aspectRatio,
            Float nearPlane,
            Float farPlane,
            const Vec2& jitter) noexcept;

        // ============================================================================
        // Utility Functions
        // ============================================================================

        /**
         * @brief Extract frustum planes from projection matrix
         */
        [[nodiscard]] static Frustum extractFrustum(const Mat4& viewProjection) noexcept {
            return Frustum(viewProjection);
        }

        /**
         * @brief Calculate field of view from projection matrix
         */
        [[nodiscard]] static Float extractFOV(const Mat4& projection) noexcept;

        /**
         * @brief Calculate aspect ratio from projection matrix
         */
        [[nodiscard]] static Float extractAspectRatio(const Mat4& projection) noexcept {
            return projection[1][1] / projection[0][0];
        }

        /**
         * @brief Calculate near/far planes from projection matrix
         */
        [[nodiscard]] static std::pair<Float, Float> extractNearFar(const Mat4& projection) noexcept;

        /**
         * @brief Convert depth from NDC to view space
         */
        [[nodiscard]] static Float ndcDepthToViewZ(
            const Float ndcDepth,
            const Mat4& projection) noexcept {
            return projection[3][2] / (ndcDepth * projection[2][3] - projection[2][2]);
        }

        /**
         * @brief Convert depth from view space to NDC
         */
        [[nodiscard]] static Float viewZToNDCDepth(
            const Float viewZ,
            const Mat4& projection) noexcept {
            return (projection[2][2] + projection[3][2] / viewZ) / projection[2][3];
        }

        /**
         * @brief Linearize depth buffer value
         */
        [[nodiscard]] static Float linearizeDepth(
            Float depth,
            Float nearPlane,
            Float farPlane,
            DepthRange depthRange = DepthRange::NEG_ONE_TO_ONE) noexcept;
    };
} // namespace engine::math::graphics
