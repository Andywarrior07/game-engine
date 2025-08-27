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
    Vector3 CoordinateTransform::worldToNDC(const Vector3& worldPos,
                                            const Matrix4& viewMatrix,
                                            const Matrix4& projectionMatrix) {
        // Transform to view space
        const Vector4 viewPos = viewMatrix * Vector4(worldPos, 1.0f);

        // Transform to clip space
        const Vector4 clipPos = projectionMatrix * viewPos;

        // Perspective divide to get NDC
        if (std::abs(clipPos.w) > 0.001f) {
            return Vector3(
                clipPos.x / clipPos.w,
                clipPos.y / clipPos.w,
                clipPos.z / clipPos.w
            );
        }

        return Vector3(clipPos.x, clipPos.y, clipPos.z);
    }

    Vector2 CoordinateTransform::ndcToScreen(const Vector3& ndc, const Viewport& viewport) {
        return Vector2(
            viewport.x + (ndc.x + 1.0f) * 0.5f * viewport.width,
            viewport.y + (1.0f - ndc.y) * 0.5f * viewport.height
        );
    }

    Vector3 CoordinateTransform::screenToNDC(const Vector2& screenPos, const Viewport& viewport) {
        return Vector3(
            (screenPos.x - viewport.x) / viewport.width * 2.0f - 1.0f,
            1.0f - (screenPos.y - viewport.y) / viewport.height * 2.0f,
            0.0f
        );
    }

    Vector3 CoordinateTransform::ndcToWorld(const Vector3& ndc,
                                            const Matrix4& invViewMatrix,
                                            const Matrix4& invProjMatrix) {
        // NDC to clip space
        const Vector4 clipPos(ndc.x, ndc.y, ndc.z, 1.0f);

        // Clip to view space
        const Vector4 viewPos = invProjMatrix * clipPos;

        // View to world space
        const Vector4 worldPos = invViewMatrix * viewPos;

        // Homogeneous divide
        if (std::abs(worldPos.w) > 0.001f) {
            return Vector3(
                worldPos.x / worldPos.w,
                worldPos.y / worldPos.w,
                worldPos.z / worldPos.w
            );
        }

        return Vector3(worldPos.x, worldPos.y, worldPos.z);
    }

    std::pair<Vector3, Vector3> CoordinateTransform::screenToRay(const Vector2& screenPos,
                                                                 const Viewport& viewport,
                                                                 const Matrix4& invViewMatrix,
                                                                 const Matrix4& invProjMatrix) {
        // Convert to NDC
        Vector3 ndcNear = screenToNDC(screenPos, viewport);
        ndcNear.z = -1.0f; // Near plane in NDC

        Vector3 ndcFar = ndcNear;
        ndcFar.z = 1.0f; // Far plane in NDC

        // Transform to world space
        const Vector3 worldNear = ndcToWorld(ndcNear, invViewMatrix, invProjMatrix);
        const Vector3 worldFar = ndcToWorld(ndcFar, invViewMatrix, invProjMatrix);

        // Calculate ray
        Vector3 rayOrigin = worldNear;
        Vector3 rayDirection = glm::normalize(worldFar - worldNear);

        return std::make_pair(rayOrigin, rayDirection);
    }

    Vector2 CoordinateTransform::project(const Vector3& worldPos,
                                         const BaseCamera* camera,
                                         const Viewport& viewport) {
        if (!camera) return Vector2(-1, -1);

        // For 2D cameras, use their transformation
        if (camera->getType() == CameraType::CAMERA_2D) {
            const auto* camera2D = static_cast<const Camera2D*>(camera);
            return camera2D->worldToScreen(Vector2(worldPos.x, worldPos.y), viewport);
        }

        // For 3D cameras
        const auto* camera3D = static_cast<const Camera3D*>(camera);
        return camera3D->worldToScreen(worldPos, viewport);
    }

    Vector3 CoordinateTransform::unproject(const Vector2& screenPos,
                                           const float depth,
                                           const BaseCamera* camera,
                                           const Viewport& viewport) {
        if (!camera) return Vector3(0, 0, 0);

        // For 2D cameras
        if (camera->getType() == CameraType::CAMERA_2D) {
            const auto* camera2D = static_cast<const Camera2D*>(camera);
            const Vector2 worldPos2D = camera2D->screenToWorld(screenPos, viewport);

            return Vector3(worldPos2D.x, worldPos2D.y, depth);
        }

        // For 3D cameras
        const auto* camera3D = static_cast<const Camera3D*>(camera);

        return camera3D->screenToWorld(screenPos, viewport, depth);
    }

    std::pair<Vector2, Vector2> CoordinateTransform::calculateScreenBounds(const CameraBounds& worldBounds,
                                                                           const Matrix4& viewMatrix,
                                                                           const Matrix4& projectionMatrix,
                                                                           const Viewport& viewport) {
        // Get 8 corners of world bounds
        const Vector3 min = worldBounds.getMin();
        const Vector3 max = worldBounds.getMax();

        Vector3 corners[8] = {
            Vector3(min.x, min.y, min.z),
            Vector3(max.x, min.y, min.z),
            Vector3(min.x, max.y, min.z),
            Vector3(max.x, max.y, min.z),
            Vector3(min.x, min.y, max.z),
            Vector3(max.x, min.y, max.z),
            Vector3(min.x, max.y, max.z),
            Vector3(max.x, max.y, max.z)
        };

        // Transform corners to screen space
        Vector2 screenMin(FLT_MAX, FLT_MAX);
        Vector2 screenMax(-FLT_MAX, -FLT_MAX);

        for (const auto& corner : corners) {
            Vector3 ndc = worldToNDC(corner, viewMatrix, projectionMatrix);

            // Skip if behind camera
            if (ndc.z < -1.0f || ndc.z > 1.0f) continue;

            Vector2 screenPos = ndcToScreen(ndc, viewport);

            screenMin.x = std::min(screenMin.x, screenPos.x);
            screenMin.y = std::min(screenMin.y, screenPos.y);
            screenMax.x = std::max(screenMax.x, screenPos.x);
            screenMax.y = std::max(screenMax.y, screenPos.y);
        }

        return std::make_pair(screenMin, screenMax);
    }

    bool CoordinateTransform::isVisible(const Vector3& worldPos,
                            const BaseCamera* camera,
                            const Viewport& viewport,
                            const float margin = 0.0f) {
        if (!camera) return false;

        const Vector2 screenPos = project(worldPos, camera, viewport);

        // Check if within viewport bounds (with margin)
        return screenPos.x >= -margin &&
               screenPos.x <= viewport.width + margin &&
               screenPos.y >= -margin &&
               screenPos.y <= viewport.height + margin;
    }

    float CoordinateTransform::distanceToCamera(const Vector3& worldPos, const BaseCamera* camera) {
        if (!camera) return 0.0f;

        const Vector3 cameraPos = camera->getPosition();
        return glm::length(worldPos - cameraPos);
    }

    float CoordinateTransform::calculateViewDepth(const Vector3& worldPos, const Matrix4& viewMatrix) {
        const Vector4 viewPos = viewMatrix * Vector4(worldPos, 1.0f);

        return -viewPos.z; // Negative Z is forward in view space
    }
} // namespace engine::camera
