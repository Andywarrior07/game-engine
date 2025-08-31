/**
 * @file Camera2D.h
 * @brief 2D camera implementation for sprite-based games
 * @author Andrés Guerrero
 * @date 24-08-2025
 */

#pragma once

#include "BaseCamera.h"

#include "../core/Viewport.h"

#include <utility>

namespace engine::camera {
    /**
     * @brief 2D Camera implementation for sprite and UI rendering
     *
     * Provides a 2D orthographic camera with support for:
     * - Position, zoom, and rotation transformations
     * - Target following for platformers and top-down games
     * - Smooth transitions and movement
     * - Screen-to-world coordinate transformations
     */
    class Camera2D final : public BaseCamera {
    public:
        /**
         * @brief Constructor for 2D camera
         * @param id Unique identifier
         * @param name Human-readable name
         */
        Camera2D(CameraID id, std::string name);

        /**
         * @brief Destructor
         */
        ~Camera2D() override = default;

        [[nodiscard]] Vec3 getPosition() const override {
            return {position_.x, position_.y, 0.0f};
        }

        // ========================================================================
        // BASE CAMERA IMPLEMENTATION
        // ========================================================================

        void setPosition(const Vec3& position) override { setPosition(Vec2(position.x, position.y)); };
        void update(float deltaTime) override;
        void reset() override;
        [[nodiscard]] std::string getDebugInfo() const override;
        [[nodiscard]] bool validate() const override;


        // ========================================================================
        // 2D SPECIFIC POSITION AND TRANSFORM
        // ========================================================================

        /**
         * @brief Set 2D position
         * @param position New 2D position
         */
        void setPosition(const Vec2& position);

        /**
         * @brief Get 2D position
         * @return Current 2D position
         */
        [[nodiscard]] const Vec2& getPosition2D() const noexcept { return position_; }

        /**
         * @brief Set camera zoom level
         * @param zoom New zoom level (1.0 = normal, 2.0 = 2x zoom)
         */
        void setZoom(float zoom);

        /**
         * @brief Get current zoom level
         * @return Current zoom level
         */
        [[nodiscard]] float getZoom() const noexcept { return zoom_; }

        /**
         * @brief Set target zoom for smooth zooming
         * @param zoom Target zoom level
         */
        void setTargetZoom(const float zoom) {
            targetZoom_ = math::clamp(zoom, minZoom_, maxZoom_);
        }

        /**
         * @brief Get target zoom level
         * @return Target zoom level
         */
        [[nodiscard]] float getTargetZoom() const noexcept { return targetZoom_; }

        /**
         * @brief Set camera rotation in degrees
         * @param rotation New rotation angle
         */
        void setRotation(const float rotation) { rotation_ = rotation; }

        /**
         * @brief Get camera rotation
         * @return Current rotation in degrees
         */
        [[nodiscard]] float getRotation() const noexcept { return rotation_; }

        /**
         * @brief Set target rotation for smooth rotation
         * @param rotation New rotation angle
         */
        void setTargetRotation(const float rotation) { targetRotation_ = rotation; }

        /**
         * @brief Get target rotation
         * @return Target rotation in degrees
         */
        [[nodiscard]] float getTargetRotation() const noexcept { return targetRotation_; }

        /**
         * @brief Set camera offset from center
         * @param offset Offset in screen pixels
         */
        void setOffset(const Vec2& offset) { offset_ = offset; }

        /**
         * @brief Get camera offset
         */
        [[nodiscard]] const Vec2& getOffset() const noexcept { return offset_; }

        // ========================================================================
        // TARGET FOLLOWING
        // ========================================================================

        /**
         * @brief Set target position for following modes
         * @param target Target position to follow
         */
        void setTarget(const Vec2& target);

        /**
         * @brief Get current target position
         * @return Current target position
         */
        [[nodiscard]] const Vec2& getTarget() const noexcept { return targetPosition_; }

        /**
         * @brief Set follow speed for target following
         * @param speed Follow speed (higher = faster following)
         */
        void setFollowSpeed(const float speed) {
            followSpeed_ = std::max(0.1f, speed);
        }

        /**
         * @brief Get follow speed
         * @return Current follow speed
         */
        [[nodiscard]] float getFollowSpeed() const noexcept { return followSpeed_; }

        /**
         * @brief Get camera velocity
         * @return Current velocity
         */
        [[nodiscard]] const Vec2& getVelocity() const noexcept { return velocity_; }

        // ========================================================================
        // ZOOM CONFIGURATION
        // ========================================================================

        /**
         * @brief Set zoom constraints
         * @param minZoom Minimum allowed zoom
         * @param maxZoom Maximum allowed zoom
         */
        void setZoomLimits(float minZoom, float maxZoom);

        /**
         * @brief Get zoom limits
         * @return Pair of (min, max) zoom values
         */
        [[nodiscard]] std::pair<float, float> getZoomLimits() const { return {minZoom_, maxZoom_}; }

        // ========================================================================
        // COORDINATE TRANSFORMATIONS
        // ========================================================================

        /**
         * @brief Convert world position to screen coordinates
         * @param worldPos World position to convert
         * @param viewport Viewport for conversion
         * @return Screen coordinate
         */
        [[nodiscard]] Vec2 worldToScreen(const Vec2& worldPos, const Viewport& viewport) const;

        /**
         * @brief Convert screen coordinates to world position
         * @param screenPos Screen position to convert
         * @param viewport Viewport for conversion
         * @return World coordinates
         */
        [[nodiscard]] Vec2 screenToWorld(const Vec2& screenPos, const Viewport& viewport) const;

        /**
         * @brief Get camera view bounds in world space
         * @param viewport Viewport to calculate bounds for
         * @return Camera bounds that define visible area
         */
        [[nodiscard]] CameraBounds getViewBounds(const Viewport& viewport) const;

        /**
         * @brief Check if world position is visible
         * @param worldPos World position to check
         * @param viewport Current viewport
         * @return true if position is visible
         */
        [[nodiscard]] bool isVisible(const Vec2& worldPos, const Viewport& viewport) const;

    private:
        Vec2 position_{0.0f, 0.0f}; ///< Current camera position
        Vec2 targetPosition_{0.0f, 0.0f}; ///< Target position for following modes
        Vec2 offset_{0.0f, 0.0f}; ///< Offset from center (for UI cameras)

        float zoom_ = 1.0f; ///< Zoom level (1.0 = normal scale)
        float targetZoom_ = 1.0f; ///< Target zoom for smooth zooming
        float rotation_ = 0.0f; ///< Rotation in degrees
        float targetRotation_ = 0.0f; ///< Target rotation for smooth rotation

        float followSpeed_ = defaults::FOLLOW_SPEED; ///< Speed for following target
        Vec2 velocity_{0.0f, 0.0f}; ///< Current movement velocity

        float minZoom_ = defaults::MIN_ZOOM; ///< Minimum zoom level
        float maxZoom_ = defaults::MAX_ZOOM; ///< Maximum zoom level

        /**
         * @brief Update camera based on current mode
         * @param deltaTime Time step
         */
        void updateMovement(float deltaTime);

        /**
         * @brief Update zoom smoothing
         * @param deltaTime Time step
         */
        void updateZoom(float deltaTime);

        /**
         * @brief Update rotation smoothing
         * @param deltaTime Time step
         */
        void updateRotation(float deltaTime);

        /**
         * @brief Apply zoom constraints
         */
        void applyZoomConstraints();
    };
} // namespace engine::camera
