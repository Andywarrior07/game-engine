/**
 * @file BaseCamera.h
 * @brief Base camera class with common functionality for all camera types
 * @author Andrés Guerrero
 * @date 24-08-2025
 */

#pragma once

#include "../core/CameraTypes.h"
#include "../core/CameraBounds.h"

#include <string>

namespace engine::camera {
    /**
     * @brief Abstract base camera class with common functionality
     *
     * This class provides the foundation for all camera types in the system.
     * It includes common properties like position, bounds, smoothing, and callbacks
     * that are shared between 2D and 3D camera implementations.
     */
    class BaseCamera {
    public:
        /**
         * @brief Constructor for base camera
         * @param id Unique identifier for this camera
         * @param name Human-readable name for debugging
         * @param type Type of camera (2D or 3D)
         */
        BaseCamera(CameraID id, std::string name, CameraType type);

        /**
         * @brief Virtual destructor for proper cleanup
         */
        virtual ~BaseCamera() = default;

        // Non-copyable buy moveable
        BaseCamera(const BaseCamera&) = delete;
        BaseCamera& operator=(const BaseCamera&) = delete;
        BaseCamera(BaseCamera&&) = default;
        BaseCamera& operator=(BaseCamera&&) = default;

        // ========================================================================
        // PURE VIRTUAL METHODS
        // ========================================================================

        /**
         * @brief Get camera position (2D cameras return z=0)
         * @return Current camera position
         */
        [[nodiscard]] virtual Vec3 getPosition() const = 0;

        /**
         * @brief Set camera position
         * @param position New camera position
         */
        virtual void setPosition(const Vec3& position) = 0;

        /**
         * @brief Update camera state
         * @param deltaTime Time elapsed since last update
         */
        virtual void update(float deltaTime) = 0;

        /**
         * @brief Reset camera to default state
         */
        virtual void reset() = 0;

        /**
         * @brief Get debug information string
         * @return String with camera debug info
         */
        [[nodiscard]] virtual std::string getDebugInfo() const = 0;

        /**
         * @brief Validate camera state
         * @return true if camera is in valid state
         */
        [[nodiscard]] virtual bool validate() const = 0;

        // ========================================================================
        // ACCESSORS
        // ========================================================================

        /**
         * @brief Get camera ID
         * @return Unique camera identifier
         */
        [[nodiscard]] CameraID getId() const noexcept { return id_; }

        /**
         * @brief Get camera name
         * @return Human-readable camera name
         */
        [[nodiscard]] const std::string& getName() const noexcept { return name_; }

        /**
         * @brief Get camera type
         * @return Type of camera (2D or 3D)
         */
        [[nodiscard]] CameraType getType() const noexcept { return type_; }

        /**
         * @brief Get camera mode
         * @return Current movement mode
         */
        [[nodiscard]] CameraMode getMode() const noexcept { return mode_; }

        /**
         * @brief Check if camera is active
         * @return true if this is the active camera
         */
        [[nodiscard]] bool isActive() const noexcept { return active_; }

        /**
         * @brief Check if camera is enabled
         * @return true if camera is enabled
         */
        [[nodiscard]] bool isEnabled() const noexcept { return enabled_; }

        /**
         * @brief Get camera bounds
         * @return Current movement bounds
         */
        [[nodiscard]] const CameraBounds& getBounds() const noexcept { return bounds_; }

        /**
         * @brief Get smoothing speed
         * @return Current smoothing speed
         */
        [[nodiscard]] float getSmoothingSpeed() const noexcept { return smoothingSpeed_; }

        // ========================================================================
        // MUTATORS
        // ========================================================================

        /**
         * @brief Set camera name
         * @param name New camera name
         */
        void setName(const std::string& name) { name_ = name; }

        /**
         * @brief Set camera movement mode
         * @param mode New movement mode
         */
        void setMode(const CameraMode mode) { mode_ = mode; }

        /**
         * @brief Set active to this camera
         * @param active Whether camera should be active
         */
        void setActive(const bool active) { active_ = active; }

        /**
         * @brief Enable or disable this camera
         * @param active Whether camera should be enabled
         */
        void setEnabled(const bool active) { enabled_ = active; }

        /**
         * @brief Set camera bounds
         * @param bounds Movement bounds for this camera
         */
        void setBounds(CameraBounds bounds) { bounds_ = std::move(bounds); }

        /**
         * @brief Set smoothing speed for camera movement
         * @param speed Smoothing speed (higher = more responsive)
         */
        void setSmoothingSpeed(const float speed) {
            smoothingSpeed_ = std::max(0.1f, speed);
        }

        /**
         * @brief Set camera callback for position changes
         * @param callback Function to call when camera moves
         */
        void setCallback(const CameraCallback& callback) {
            callback_ = callback;
        }

        /**
         * @brief Clear camera callback
         */
        void clearCallback() {
            callback_ = nullptr;
        }

    protected:
        CameraID id_; ///< Unique identifier
        std::string name_; ///< Human-readable name
        CameraType type_; ///< Camera type
        CameraMode mode_; ///< Movement mode
        bool active_; ///< Whether this is the active camera
        bool enabled_; ///< Whether camera is enabled

        CameraBounds bounds_; ///< Movement constraints
        float smoothingSpeed_; ///< Smoothing speed for movement
        CameraCallback callback_; ///< Position change callback

        // ========================================================================
        // HELPER METHODS
        // ========================================================================

        /**
         * @brief Apply camera bounds to position
         * @param position Position to constrain
         * @return Constrained position
         */
        [[nodiscard]] Vec3 applyBounds(const Vec3& position) const {
            return bounds_.clamp(position);
        }

        /**
         * @brief Trigger callback fi set
         * @param position Current position
         * @param target Current target
         */
        void triggerCallback(const Vec3& position, const Vec3& target) const;

        /**
         * @brief Apply smoothing to scalar value
         * @param current Current value
         * @param target Target value
         * @param deltaTime Timestep
         * @return Smoothed value
         */
        [[nodiscard]] float applySmoothing(float current, float target, float deltaTime) const;

        /**
         * @brief Apply smoothing to 2D vector
         * @param current Current vector
         * @param target Target vector
         * @param deltaTime Time step
         * @return Smoothed vector
         */
        [[nodiscard]] Vec2 applySmoothing(const Vec2& current, const Vec2& target, float deltaTime) const;

        /**
         * @brief Apply smoothing to 3D vector
         * @param current Current vector
         * @param target Target vector
         * @param deltaTime Time step
         * @return Smoothed vector
         */
        [[nodiscard]] Vec3 applySmoothing(const Vec3& current, const Vec3& target, float deltaTime) const;
    };
} // namespace engine::camera
