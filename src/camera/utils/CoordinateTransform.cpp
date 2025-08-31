/**
* @file CoordinateTransform.h
 * @brief Coordinate transformation utilities for camera system
 * @author Andrés Guerrero
 * @date 26-08-2025
 */

#include "CoordinateTransform.h"

#include "../cameras/Camera2D.h"
#include "../cameras/Camera3D.h"

namespace engine::camera {
    Vec3 CoordinateTransform::worldToNDC(const Vec3& worldPos,
                                            const Mat4& viewMatrix,
                                            const Mat4& projectionMatrix) {
        // Transform to view space
        const Vec4 viewPos = viewMatrix * Vec4(worldPos, 1.0f);

        // Transform to clip space
        const Vec4 clipPos = projectionMatrix * viewPos;

        // Perspective divide to get NDC
        if (std::abs(clipPos.w) > 0.001f) {
            return Vec3(
                clipPos.x / clipPos.w,
                clipPos.y / clipPos.w,
                clipPos.z / clipPos.w
            );
        }

        return Vec3(clipPos.x, clipPos.y, clipPos.z);
    }

    Vec2 CoordinateTransform::ndcToScreen(const Vec3& ndc, const Viewport& viewport) {
        return Vec2(
            viewport.getX() + (ndc.x + 1.0f) * 0.5f * viewport.getWidth(),
            viewport.getY() + (1.0f - ndc.y) * 0.5f * viewport.getHeight()
        );
    }

    Vec3 CoordinateTransform::screenToNDC(const Vec2& screenPos, const Viewport& viewport) {
        return Vec3(
            (screenPos.x - viewport.getX()) / viewport.getWidth() * 2.0f - 1.0f,
            1.0f - (screenPos.y - viewport.getY()) / viewport.getHeight() * 2.0f,
            0.0f
        );
    }

    Vec3 CoordinateTransform::ndcToWorld(const Vec3& ndc,
                                            const Mat4& invViewMatrix,
                                            const Mat4& invProjMatrix) {
        // NDC to clip space
        const Vec4 clipPos(ndc.x, ndc.y, ndc.z, 1.0f);

        // Clip to view space
        const Vec4 viewPos = invProjMatrix * clipPos;

        // View to world space
        const Vec4 worldPos = invViewMatrix * viewPos;

        // Homogeneous divide
        if (std::abs(worldPos.w) > 0.001f) {
            return Vec3(
                worldPos.x / worldPos.w,
                worldPos.y / worldPos.w,
                worldPos.z / worldPos.w
            );
        }

        return Vec3(worldPos.x, worldPos.y, worldPos.z);
    }

    std::pair<Vec3, Vec3> CoordinateTransform::screenToRay(const Vec2& screenPos,
                                                                 const Viewport& viewport,
                                                                 const Mat4& invViewMatrix,
                                                                 const Mat4& invProjMatrix) {
        // Convert to NDC
        Vec3 ndcNear = screenToNDC(screenPos, viewport);
        ndcNear.z = -1.0f; // Near plane in NDC

        Vec3 ndcFar = ndcNear;
        ndcFar.z = 1.0f; // Far plane in NDC

        // Transform to world space
        const Vec3 worldNear = ndcToWorld(ndcNear, invViewMatrix, invProjMatrix);
        const Vec3 worldFar = ndcToWorld(ndcFar, invViewMatrix, invProjMatrix);

        // Calculate ray
        Vec3 rayOrigin = worldNear;
        Vec3 rayDirection = glm::normalize(worldFar - worldNear);

        return std::make_pair(rayOrigin, rayDirection);
    }

    Vec2 CoordinateTransform::project(const Vec3& worldPos,
                                         const BaseCamera* camera,
                                         const Viewport& viewport) {
        if (!camera) return Vec2(-1, -1);

        // For 2D cameras, use their transformation
        if (camera->getType() == CameraType::CAMERA_2D) {
            const auto* camera2D = static_cast<const Camera2D*>(camera);
            return camera2D->worldToScreen(Vec2(worldPos.x, worldPos.y), viewport);
        }

        // For 3D cameras
        const auto* camera3D = static_cast<const Camera3D*>(camera);
        return camera3D->worldToScreen(worldPos, viewport);
    }

    Vec3 CoordinateTransform::unproject(const Vec2& screenPos,
                                           const float depth,
                                           const BaseCamera* camera,
                                           const Viewport& viewport) {
        if (!camera) return Vec3(0, 0, 0);

        // For 2D cameras
        if (camera->getType() == CameraType::CAMERA_2D) {
            const auto* camera2D = static_cast<const Camera2D*>(camera);
            const Vec2 worldPos2D = camera2D->screenToWorld(screenPos, viewport);

            return Vec3(worldPos2D.x, worldPos2D.y, depth);
        }

        // For 3D cameras
        const auto* camera3D = static_cast<const Camera3D*>(camera);

        return camera3D->screenToWorld(screenPos, viewport, depth);
    }

    std::pair<Vec2, Vec2> CoordinateTransform::calculateScreenBounds(const CameraBounds& worldBounds,
                                                                           const Mat4& viewMatrix,
                                                                           const Mat4& projectionMatrix,
                                                                           const Viewport& viewport) {
        // Get 8 corners of world bounds
        const Vec3 min = worldBounds.getMin();
        const Vec3 max = worldBounds.getMax();

        Vec3 corners[8] = {
            Vec3(min.x, min.y, min.z),
            Vec3(max.x, min.y, min.z),
            Vec3(min.x, max.y, min.z),
            Vec3(max.x, max.y, min.z),
            Vec3(min.x, min.y, max.z),
            Vec3(max.x, min.y, max.z),
            Vec3(min.x, max.y, max.z),
            Vec3(max.x, max.y, max.z)
        };

        // Transform corners to screen space
        Vec2 screenMin(FLT_MAX, FLT_MAX);
        Vec2 screenMax(-FLT_MAX, -FLT_MAX);

        for (const auto& corner : corners) {
            Vec3 ndc = worldToNDC(corner, viewMatrix, projectionMatrix);

            // Skip if behind camera
            if (ndc.z < -1.0f || ndc.z > 1.0f) continue;

            Vec2 screenPos = ndcToScreen(ndc, viewport);

            screenMin.x = std::min(screenMin.x, screenPos.x);
            screenMin.y = std::min(screenMin.y, screenPos.y);
            screenMax.x = std::max(screenMax.x, screenPos.x);
            screenMax.y = std::max(screenMax.y, screenPos.y);
        }

        return std::make_pair(screenMin, screenMax);
    }

    bool CoordinateTransform::isVisible(const Vec3& worldPos,
                            const BaseCamera* camera,
                            const Viewport& viewport,
                            const float margin = 0.0f) {
        if (!camera) return false;

        const Vec2 screenPos = project(worldPos, camera, viewport);

        // Check if within viewport bounds (with margin)
        return screenPos.x >= -margin &&
               screenPos.x <= viewport.getWidth() + margin &&
               screenPos.y >= -margin &&
               screenPos.y <= viewport.getHeight() + margin;
    }

    float CoordinateTransform::distanceToCamera(const Vec3& worldPos, const BaseCamera* camera) {
        if (!camera) return 0.0f;

        const Vec3 cameraPos = camera->getPosition();
        return glm::length(worldPos - cameraPos);
    }

    float CoordinateTransform::calculateViewDepth(const Vec3& worldPos, const Mat4& viewMatrix) {
        const Vec4 viewPos = viewMatrix * Vec4(worldPos, 1.0f);

        return -viewPos.z; // Negative Z is forward in view space
    }
} // namespace engine::camera
