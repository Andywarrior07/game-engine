/**
 * @file ProjectionMatrix.cpp
 * @brief Projection matrix construction for rendering
 *
 * Provides perspective, orthographic, and specialized projection
 * matrices for OpenGL and Vulkan rendering pipelines.
 */

#include "ProjectionMatrix.h"

namespace engine::math::graphics {
    Mat4 Projection::perspective(
        const Float fovY,
        const Float aspectRatio,
        const Float nearPlane,
        const Float farPlane,
        const DepthRange depthRange,
        const Handedness handedness) noexcept {
        const Float tanHalfFovY = std::tan(fovY * 0.5f);

        Mat4 result(0.0f);
        result[0][0] = 1.0f / (aspectRatio * tanHalfFovY);
        result[1][1] = 1.0f / tanHalfFovY;

        if (depthRange == DepthRange::ZERO_TO_ONE) {
            // Vulkan/D3D style [0,1] depth
            if (handedness == Handedness::RIGHT_HANDED) {
                result[2][2] = farPlane / (nearPlane - farPlane);
                result[2][3] = -1.0f;
                result[3][2] = -(farPlane * nearPlane) / (farPlane - nearPlane);
            }
            else {
                result[2][2] = farPlane / (farPlane - nearPlane);
                result[2][3] = 1.0f;
                result[3][2] = -(farPlane * nearPlane) / (farPlane - nearPlane);
            }
        }
        else {
            // OpenGL style [-1,1] depth
            if (handedness == Handedness::RIGHT_HANDED) {
                result[2][2] = -(farPlane + nearPlane) / (farPlane - nearPlane);
                result[2][3] = -1.0f;
                result[3][2] = -(2.0f * farPlane * nearPlane) / (farPlane - nearPlane);
            }
            else {
                result[2][2] = (farPlane + nearPlane) / (farPlane - nearPlane);
                result[2][3] = 1.0f;
                result[3][2] = -(2.0f * farPlane * nearPlane) / (farPlane - nearPlane);
            }
        }

        return result;
    }

    Mat4 Projection::frustum(
        const Float left, const Float right,
        const Float bottom, const Float top,
        const Float nearPlane, const Float farPlane,
        const DepthRange depthRange) noexcept {
        Mat4 result(0.0f);

        result[0][0] = (2.0f * nearPlane) / (right - left);
        result[1][1] = (2.0f * nearPlane) / (top - bottom);
        result[2][0] = (right + left) / (right - left);
        result[2][1] = (top + bottom) / (top - bottom);
        result[2][3] = -1.0f;

        if (depthRange == DepthRange::ZERO_TO_ONE) {
            result[2][2] = farPlane / (nearPlane - farPlane);
            result[3][2] = -(farPlane * nearPlane) / (farPlane - nearPlane);
        }
        else {
            result[2][2] = -(farPlane + nearPlane) / (farPlane - nearPlane);
            result[3][2] = -(2.0f * farPlane * nearPlane) / (farPlane - nearPlane);
        }

        return result;
    }

    Mat4 Projection::infinitePerspective(
        const Float fovY,
        const Float aspectRatio,
        const Float nearPlane,
        const DepthRange depthRange) noexcept {
        const Float range = std::tan(fovY * 0.5f) * nearPlane;
        const Float left = -range * aspectRatio;
        const Float right = range * aspectRatio;
        const Float bottom = -range;
        const Float top = range;

        Mat4 result(0.0f);
        result[0][0] = (2.0f * nearPlane) / (right - left);
        result[1][1] = (2.0f * nearPlane) / (top - bottom);
        result[2][3] = -1.0f;

        if (depthRange == DepthRange::ZERO_TO_ONE) {
            result[2][2] = -1.0f;
            result[3][2] = -nearPlane;
        }
        else {
            result[2][2] = -1.0f;
            result[3][2] = -2.0f * nearPlane;
        }

        return result;
    }

    Mat4 Projection::orthographic(
        const Float left, const Float right,
        const Float bottom, const Float top,
        const Float nearPlane, const Float farPlane,
        const DepthRange depthRange) noexcept {
        Mat4 result(1.0f);

        result[0][0] = 2.0f / (right - left);
        result[1][1] = 2.0f / (top - bottom);
        result[3][0] = -(right + left) / (right - left);
        result[3][1] = -(top + bottom) / (top - bottom);

        if (depthRange == DepthRange::ZERO_TO_ONE) {
            result[2][2] = 1.0f / (nearPlane - farPlane);
            result[3][2] = nearPlane / (nearPlane - farPlane);
        }
        else {
            result[2][2] = -2.0f / (farPlane - nearPlane);
            result[3][2] = -(farPlane + nearPlane) / (farPlane - nearPlane);
        }

        return result;
    }

    Mat4 Projection::orthographicCentered(
        const Float width, const Float height,
        const Float nearPlane, const Float farPlane,
        const DepthRange depthRange) noexcept {
        const Float halfWidth = width * 0.5f;
        const Float halfHeight = height * 0.5f;

        return orthographic(
            -halfWidth, halfWidth,
            -halfHeight, halfHeight,
            nearPlane, farPlane,
            depthRange
        );
    }

    Mat4 Projection::perspectiveReversedZ(
        const Float fovY,
        const Float aspectRatio,
        const Float nearPlane,
        const Float farPlane) noexcept {
        const Float tanHalfFovY = std::tan(fovY * 0.5f);

        Mat4 result(0.0f);
        result[0][0] = 1.0f / (aspectRatio * tanHalfFovY);
        result[1][1] = 1.0f / tanHalfFovY;
        result[2][3] = -1.0f;

        if (std::isinf(farPlane)) {
            // Infinite far plane
            result[2][2] = 0.0f;
            result[3][2] = nearPlane;
        }
        else {
            // Finite far plane
            result[2][2] = nearPlane / (nearPlane - farPlane);
            result[3][2] = (farPlane * nearPlane) / (nearPlane - farPlane);
        }

        return result;
    }

    Mat4 Projection::obliquePerspective(
        const Mat4& projection,
        const Vec4& clipPlane) noexcept {
        Mat4 result = projection;

        // Calculate clip-space corner point opposite the clipping plane
        const Vec4 q = glm::inverse(projection) * Vec4(
            sign(clipPlane.x),
            sign(clipPlane.y),
            1.0f,
            1.0f
        );

        const Vec4 c = clipPlane * (2.0f / glm::dot(clipPlane, q));

        // Replace third row of projection matrix
        result[2] = c - result[3];

        return result;
    }

    Mat4 Projection::jitteredPerspective(
        const Float fovY,
        const Float aspectRatio,
        const Float nearPlane,
        const Float farPlane,
        const Vec2& jitter) noexcept {
        Mat4 proj = perspective(fovY, aspectRatio, nearPlane, farPlane);

        // Apply sub-pixel jitter
        proj[2][0] += jitter.x;
        proj[2][1] += jitter.y;

        return proj;
    }

    Float Projection::extractFOV(const Mat4& projection) noexcept {
        return 2.0f * std::atan(1.0f / projection[1][1]);
    }

    std::pair<Float, Float> Projection::extractNearFar(const Mat4& projection) noexcept {
        Float nearPlane = projection[3][2] / (projection[2][2] - 1.0f);
        Float farPlane = projection[3][2] / (projection[2][2] + 1.0f);

        return {nearPlane, farPlane};
    }

    Float Projection::linearizeDepth(
        const Float depth,
        const Float nearPlane,
        const Float farPlane,
        const DepthRange depthRange) noexcept {
        if (depthRange == DepthRange::ZERO_TO_ONE) {
            return nearPlane * farPlane / (farPlane - depth * (farPlane - nearPlane));
        }

        const Float z = depth * 2.0f - 1.0f; // Back to NDC
        return (2.0f * nearPlane * farPlane) / (farPlane + nearPlane - z * (farPlane - nearPlane));
    }
} // namespace engine::math::graphics
