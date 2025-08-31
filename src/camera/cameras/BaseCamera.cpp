//
// Created by Andres Guerrero on 24-08-25.
//

#include "BaseCamera.h"

namespace engine::camera {
    BaseCamera::BaseCamera(const CameraID id, std::string name, const CameraType type)
        : id_(id)
          , name_(std::move(name))
          , type_(type)
          , mode_(CameraMode::STATIC)
          , active_(false)
          , enabled_(true)
          , smoothingSpeed_(defaults::SMOOTHING_SPEED) {
    }

    // ========================================================================
    // PROTECTED HELPER METHODS
    // ========================================================================

    void BaseCamera::triggerCallback(const Vec3& position, const Vec3& target) const {
        if (callback_) {
            callback_(id_, position, target);
        }
    }

    float BaseCamera::applySmoothing(const float current, const float target, const float deltaTime) const {
        if (smoothingSpeed_ <= 0.0f) return target;

        const float lerpFactor = 1.0f - std::exp(-smoothingSpeed_ * deltaTime);
        return current + (target - current) * lerpFactor;
    }

    Vec2 BaseCamera::applySmoothing(const Vec2& current, const Vec2& target, const float deltaTime) const {
        if (smoothingSpeed_ <= 0.0f) return target;

        const float lerpFactor = 1.0f - std::exp(-smoothingSpeed_ * deltaTime);
        return current + (target - current) * lerpFactor;
    }

    Vec3 BaseCamera::applySmoothing(const Vec3& current, const Vec3& target, const float deltaTime) const {
        if (smoothingSpeed_ <= 0.0f) return target;

        const float lerpFactor = 1.0f - std::exp(-smoothingSpeed_ * deltaTime);
        return current + (target - current) * lerpFactor;
    }

    // ========================================================================
    // VIRTUAL METHODS IMPLEMENTATION
    // ========================================================================

    std::string BaseCamera::getDebugInfo() const {
        return "Base Camera [ID: " + std::to_string(id_) +
               ", Name: " + name_ +
               ", Type: " + std::to_string(static_cast<int>(type_)) +
               ", Active: " + (active_ ? "true" : "false") + "]";
    }

    bool BaseCamera::validate() const {
        // Validación básica
        return id_ != INVALID_CAMERA_ID && !name_.empty() && smoothingSpeed_ >= 0.0f;
    }

} //namespace engine::camera
