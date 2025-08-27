/**
 * @file Camera3D.h
 * @brief 3D camera implementation for 3D scenes
 * @author Andrés Guerrero
 * @date 25-08-2025
 */

#pragma once

#include "BaseCamera.h"

namespace engine::camera {
    /**
     * @brief 3D Camera implementation for 3D scene rendering
     *
     * Provides a full 3D camera with support for:
     * - Perspective and orthographic projection
     * - FPS-style free look controls
     * - Third-person following
     * - Orbital movement
     * - View and projection matrix generation
     */
    class Camera3D final : public BaseCamera {
    public:
        /**
         * @brief Constructor for 3D camera
         * @param id Unique identifier
         * @param name Human-readable name
         * @param perspective Whether to use perspective projection
         */
        Camera3D(CameraID id, std::string name, bool perspective = true);

        /**
         * @brief Destructor
         */
        ~Camera3D() override = default;

        // ========================================================================
        // BASE CAMERA IMPLEMENTATION
        // ========================================================================

        [[nodiscard]] Vector3 getPosition() const override { return position_; }
        void setPosition(const Vector3& position) override;
        void update(float deltaTime) override;
        void reset() override;
        [[nodiscard]] std::string getDebugInfo() const override;
        [[nodiscard]] bool validate() const override;

        // ========================================================================
        // POSITION AND ORIENTATION
        // ========================================================================

        /**
         * @brief Set camera target (look-at point)
         * @param target Point to look at
         */
        void setTarget(const Vector3& target);

        /**
         * @brief Get camera target
         * @return Current target position
         */
        [[nodiscard]] const Vector3& getTarget() const noexcept { return target_; }

        /**
         * @brief Set camera up vector
         * @param up Up direction vector
         */
        void setUp(const Vector3& up) {
            up_ = glm::normalize(up);
        }

        /**
         * @brief Get camera up vector
         * @return Current up direction
         */
        [[nodiscard]] const Vector3& getUp() const noexcept { return up_; }

        /**
         * @brief Get camera forward direction
         * @return Normalized forward vector
         */
        [[nodiscard]] Vector3 getForward() const {
            return glm::normalize(target_ - position_);
        }

        /**
         * @brief Get camera right direction
         * @return Normalized right vector
         */
        [[nodiscard]] Vector3 getRight() const {
            return glm::normalize(glm::cross(getForward(), up_));
        }

        /**
         * @brief Look at specific point
         * @param target Point to look at
         * @param up Up vector (default is world up)
         */
        void lookAt(const Vector3& target, const Vector3& up = math::constants::VEC3_UP) {
            target_ = target;
            up_ = glm::normalize(up);
        }

        // ========================================================================
        // PROJECTION PROPERTIES
        // ========================================================================

        /**
         * @brief Set field of view for perspective cameras
         * @param fov Field of view in degrees
         */
        void setFOV(float fov);

        /**
         * @brief Get field of view
         * @return Current FOV in degrees
         */
        [[nodiscard]] float getFOV() const noexcept { return fov_; }

        /**
         * @brief Set near and far clipping planes
         * @param nearPlane Near clipping distance
         * @param farPlane Far clipping distance
         */
        void setClippingPlanes(float nearPlane, float farPlane);

        /**
         * @brief Get clipping planes
         * @return Pair of (near, far) clipping distances
         */
        [[nodiscard]] std::pair<float, float> getClippingPlanes() const { return {nearPlane_, farPlane_}; }

        /**
         * @brief Set whether to use perspective projection
         * @param perspective true for perspective, false for orthographic
         */
        void setPerspective(const bool perspective) {
            isPerspective_ = perspective;
        }

        /**
         * @brief Check if using perspective projection
         * @return true if perspective, false if orthographic
         */
        [[nodiscard]] bool isPerspective() const noexcept { return isPerspective_; }

        /**
         * @brief Set orthographic size (for orthographic cameras)
         * @param size Orthographic view size
         */
        void setOrthographicSize(const float size) {
            orthographicSize_ = std::max(0.1f, size);
        }

        /**
         * @brief Get orthographic size
         * @return Current orthographic size
         */
        [[nodiscard]] float getOrthographicSize() const noexcept { return orthographicSize_; }

        // ========================================================================
        // FPS-STYLE CONTROLS
        // ========================================================================

        /**
         * @brief Set yaw angle for FPS-style cameras
         * @param yaw Yaw angle in degrees
         */
        void setYaw(float yaw);

        /**
         * @brief Set pitch angel for FPS-style cameras
         * @param pitch Pitch angle in degrees
         */
        void setPitch(float pitch);

        /**
         * @brief Set roll angle
         * @param roll Roll angle in degrees
         */
        void setRoll(float roll);

        /**
         * @brief Get current yaw angle
         * @return Yaw in degrees
         */
        [[nodiscard]] float getYaw() const noexcept { return yaw_; }

        /**
         * @brief Get current pitch angle
         * @return Pitch in degrees
         */
        [[nodiscard]] float getPitch() const noexcept { return pitch_; }

        /**
         * @brief Get current roll angle
         * @return Roll in degrees
         */
        [[nodiscard]] float getRoll() const noexcept { return roll_; }

        /**
         * @brief Set pitch constraints for FPS cameras
         * @param minPitch Minimum pitch angle
         * @param maxPitch Maximum pitch angle
         */
        void setPitchLimits(float minPitch, float maxPitch);

        /**
         * @brief Get pitch limits
         * @return Pair of (min, max) pitch angles
         */
        [[nodiscard]] std::pair<float, float> getPitchLimits() const { return {minPitch_, maxPitch_}; }

        // ========================================================================
        // TARGET FOLLOWING
        // ========================================================================

        /**
         * @brief Set follow target for third-person modes
         * @param target Target position to follow
         */
        void setFollowTarget(const Vector3& target);

        /**
         * @brief Get follow target
         * @return Current follow target
         */
        [[nodiscard]] const Vector3& getFollowTarget() const noexcept { return followTarget_; }

        /**
         * @brief Set distance from follow target
         * @param distance Distance to maintain from target
         */
        void setFollowDistance(const float distance) {
            followDistance_ = std::max(0.1f, distance);
        }

        /**
         *  @brief Get follow distance
         *  @return Current follow distance
         */
        [[nodiscard]] float getFollowDistance() const noexcept { return followDistance_; }

        /**
         * @brief Set follow height offset
         * @param height Height offset from target
         */
        void setFollowHeight(const float height) { followHeight_ = height; }

        /**
         * @brief Get follow height
         * @return Current follow height offset
         */
        [[nodiscard]] float getFollowHeight() const noexcept { return followHeight_; }

        /**
         * @brief Set follow speed
         * @param speed Speed of following movement
         */
        void setFollowSpeed(const float speed) { followSpeed_ = std::max(0.1f, speed); }

        /**
         * @brief Get follow speed
         * @return Current follow speed
         */
        [[nodiscard]] float getFollowSpeed() const noexcept { return followSpeed_; }

        // ========================================================================
        // ORBITAL MOVEMENT
        // ========================================================================

        /**
         * @brief Set orbital radius
         * @param radius Orbital movement radius
         */
        void setOrbitalRadius(const float radius) {
            orbitalRadius_ = std::max(0.1f, radius);
        }

        /**
         * @brief Get orbital radius
         * @return Current orbital radius
         */
        [[nodiscard]] float getOrbitalRadius() const noexcept { return orbitalRadius_; }

        /**
         * @brief Set orbital angle
         * @param angle Orbital angle in radians
         */
        void setOrbitalAngle(const float angle) { orbitalAngle_ = angle; }

        /**
         * @brief Get orbital angle
         * @return Current orbital angle in radians
         */
        [[nodiscard]] float getOrbitalAngle() const noexcept { return orbitalAngle_; }

        /**
         * @brief Set orbital speed
         * @param speed Orbital rotation speed (radians/second)
         */
        void setOrbitalSpeed(const float speed) { orbitalSpeed_ = speed; }

        /**
         * @brief Get orbital speed
         * @return Current orbital
         */
        [[nodiscard]] float getOrbitalSpeed() const noexcept { return orbitalSpeed_; }

        // ========================================================================
        // COORDINATE TRANSFORMATIONS
        // ========================================================================

        /**
         * @brief Convert world position to screen coordinates
         * @param worldPos World position to convert
         * @param viewport Viewport for conversion
         * @return Screen coordinates
         */
        [[nodiscard]] Vector2 worldToScreen(const Vector3& worldPos, const Viewport& viewport) const;

        /**
         * @brief Convert screen coordinates to world ray
         * @param screenPos Screen position
         * @param viewport Viewport for conversion
         * @return Pair of (ray origin, ray direction)
         */
        std::pair<Vector3, Vector3> screenToWorldRay(const Vector2& screenPos, const Viewport& viewport) const;

        /**
         * @brief Convert screen coordinates to world position at given depth
         * @param screenPos Screen position
         * @param viewport Viewport for conversion
         * @param depth Depth in world units
         * @return World position at specified depth
         */
        [[nodiscard]] Vector3 screenToWorld(const Vector2& screenPos, const Viewport& viewport, float depth) const;

        /**
         * @brief Calculate view matrix for this camera
         * @return 4x4 view matrix (for integration with rendering system)
         */
        [[nodiscard]] Matrix4 calculateViewMatrix() const {
            return math::lookAt(position_, target_, up_);
        }

        /**
         * @brief Calculate projection matrix for this camera
         * @param viewport Viewport to calculate projection for
         * @return 4x4 projection matrix
         */
        Matrix4 calculateProjectionMatrix(const Viewport& viewport) const;

        /**
         * @brief Get view-projection matrix
         * @param viewport Viewport for projection
         * @return Combined view-projection matrix
         */
        Matrix4 getViewProjectionMatrix(const Viewport& viewport) const;

    private:
        // ========================================================================
        // 3D CAMERA PROPERTIES
        // ========================================================================

        Vector3 position_{0.0f, 0.0f, 5.0f}; ///< Current camera position
        Vector3 target_{0.0f, 0.0f, 0.0f}; ///< Point camera is looking at
        Vector3 up_{0.0f, 1.0f, 0.0f}; ///< Up direction vector

        // Projection properties
        float fov_ = defaults::CAMERA_FOV; ///< Field of view in degrees
        float nearPlane_ = defaults::CAMERA_NEAR_PLANE; ///< Near clipping plane
        float farPlane_ = defaults::CAMERA_FAR_PLANE; ///< Far clipping plane
        bool isPerspective_ = true; ///< Perspective vs orthographic
        float orthographicSize_ = 10.0f; ///< Size for orthographic projection

        // FPS-style rotation (Euler angles)
        float yaw_ = -90.0f; ///< Horizontal rotation
        float pitch_ = 0.0f; ///< Vertical rotation
        float roll_ = 0.0f; ///< Camera roll
        float minPitch_ = defaults::MIN_PITCH; ///< Minimum pitch constraint
        float maxPitch_ = defaults::MAX_PITCH; ///< Maximum pitch constraint

        // Following properties
        Vector3 followTarget_{0.0f, 0.0f, 0.0f}; ///< Target to follow
        float followDistance_ = 5.0f; ///< Distance from follow target
        float followHeight_ = 2.0f; ///< Height offset from target
        float followSpeed_ = defaults::FOLLOW_SPEED; ///< Speed for following movement

        // Orbital properties
        float orbitalRadius_ = defaults::ORBITAL_RADIUS; ///< Radius for orbital movement
        float orbitalAngle_ = 0.0f; ///< Current orbital angle
        float orbitalSpeed_ = defaults::ORBITAL_SPEED; ///< Speed of orbital movement

        /**
         * @brief Update camera base on current mode
         * @param deltaTime Time step
         */
        void updateMovement(float deltaTime);

        /**
         * @brief Update FPS-style free look camera
         * @param deltaTime Time step
         */
        void updateFreeLook(float deltaTime);

        /**
         * @brief Update target following camera
         * @param deltaTime Time step
         */
        void updateFollowTarget(float deltaTime);

        /**
         * @brief Update orbital camera movement
         * @param deltaTime Time step
         */
        void updateOrbital(float deltaTime);

        /**
         * @brief Update target from Euler angles(yaw, pitch, roll)
         */
        void updateFromEuler();

        /**
         * @brief Apply pitch constraints
         */
        void applyPitchConstraints();
    };
} // namespace engine::camera
