/**
 * @file CoordinateTransform.h
 * @brief Coordinate transformation utilities for camera system
 * @author Andrés Guerrero
 * @date 26-08-2025
 */

#pragma once

#include "../core/CameraTypes.h"
#include "../core/Viewport.h"
#include "../cameras/BaseCamera.h"

using namespace engine::math;

namespace engine::camera {

    /**
     * @brief Coordinate transformation utilities
     *
     * Provides utility functions for transforming coordinates between
     * different spaces (world, view, screen, NDC) for both 2D and 3D cameras.
     */
    class CoordinateTransform {
    public:
        /**
         * @brief Transform world position to normalized device coordinates (NDC)
         * @param worldPos World position
         * @param viewMatrix View matrix
         * @param projectionMatrix Projection matrix
         * @return Position in NDC space (-1 to 1)
         */
        static Vector3 worldToNDC(const Vector3& worldPos,
                                 const Matrix4& viewMatrix,
                                 const Matrix4& projectionMatrix);

        /**
         * @brief Transform NDC to screen coordinates
         * @param ndc Normalized device coordinates (-1 to 1)
         * @param viewport Target viewport
         * @return Screen coordinates in pixels
         */
        static Vector2 ndcToScreen(const Vector3& ndc, const Viewport& viewport);

        /**
         * @brief Transform screen coordinates to NDC
         * @param screenPos Screen position in pixels
         * @param viewport Source viewport
         * @return Position in NDC space (-1 to 1)
         */
        static Vector3 screenToNDC(const Vector2& screenPos, const Viewport& viewport);

        /**
         * @brief Transform NDC to world space
         * @param ndc NDC coordinates
         * @param invViewMatrix Inverse view matrix
         * @param invProjMatrix Inverse projection matrix
         * @return World position
         */
        static Vector3 ndcToWorld(const Vector3& ndc,
                                 const Matrix4& invViewMatrix,
                                 const Matrix4& invProjMatrix);

        /**
         * @brief Create ray from screen position
         * @param screenPos Screen position
         * @param viewport Viewport
         * @param invViewMatrix Inverse view matrix
         * @param invProjMatrix Inverse projection matrix
         * @return Pair of (ray origin, ray direction)
         */
        static std::pair<Vector3, Vector3> screenToRay(const Vector2& screenPos,
                                                       const Viewport& viewport,
                                                       const Matrix4& invViewMatrix,
                                                       const Matrix4& invProjMatrix);

        /**
         * @brief Project 3D point to 2D screen space
         * @param worldPos World position
         * @param camera Camera to use for projection
         * @param viewport Target viewport
         * @return Screen position (negative if behind camera)
         */
        static Vector2 project(const Vector3& worldPos,
                              const BaseCamera* camera,
                              const Viewport& viewport);

        /**
         * @brief Unproject screen point to world space
         * @param screenPos Screen position
         * @param depth Depth value (for 3D unprojection)
         * @param camera Camera to use for unprojection
         * @param viewport Source viewport
         * @return World position
         */
        static Vector3 unproject(const Vector2& screenPos,
                                float depth,
                                const BaseCamera* camera,
                                const Viewport& viewport);

        /**
         * @brief Calculate screen-space bounding box for world bounds
         * @param worldBounds World space bounds
         * @param viewMatrix View matrix
         * @param projectionMatrix Projection matrix
         * @param viewport Target viewport
         * @return Screen space bounding box (min, max)
         */
        static std::pair<Vector2, Vector2> calculateScreenBounds(const CameraBounds& worldBounds,
                                                                const Matrix4& viewMatrix,
                                                                const Matrix4& projectionMatrix,
                                                                const Viewport& viewport);

        /**
         * @brief Check if world position is visible in viewport
         * @param worldPos World position
         * @param camera Camera to check against
         * @param viewport Viewport
         * @param margin Screen margin in pixels
         * @return true if position is visible
         */
        static bool isVisible(const Vector3& worldPos,
                            const BaseCamera* camera,
                            const Viewport& viewport,
                            float margin);

        /**
         * @brief Calculate distance from camera to world position
         * @param worldPos World position
         * @param camera Camera
         * @return Distance to camera
         */
        static float distanceToCamera(const Vector3& worldPos, const BaseCamera* camera);

        /**
         * @brief Calculate view-space depth
         * @param worldPos World position
         * @param viewMatrix View matrix
         * @return Depth in view space (negative for points in front)
         */
        static float calculateViewDepth(const Vector3& worldPos, const Matrix4& viewMatrix);
    };

} // namespace engine::camera