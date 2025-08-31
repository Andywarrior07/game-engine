/**
* @file CameraTransition.cpp
 * @brief Camera transition state and execution
 * @author Andrés Guerrero
 * @date 26-08-2025
 */

#include "CameraTransition.h"

#include "TransitionEasing.h"

#include "../../math/core/MathFunctions.h"

namespace engine::camera {
    CameraTransition::CameraTransition(const TransitionID id, const CameraID cameraId, const TransitionConfig& config)
        : id_(id)
          , cameraId_(cameraId)
          , config_(config)
          , delayRemaining_(config.delay) {
    }

    float CameraTransition::getEasedProgress() const {
        return TransitionEasing::ease(config_.type, getProgress());
    }

    void CameraTransition::setupPosition(const Vec3& start, const Vec3& target) {
        hasPosition_ = true;
        startPosition_ = start;
        targetPosition_ = config_.relative ? start + target : target;
    }

    void CameraTransition::setupTarget(const Vec3& start, const Vec3& target) {
        hasTarget_ = true;
        startTarget_ = start;
        targetLookAt_ = config_.relative ? start + target : target;
    }

    void CameraTransition::setupZoom(const float start, const float target) {
        hasZoom_ = true;
        startZoom_ = start;
        targetZoom_ = config_.relative ? start + target : target;
    }

    void CameraTransition::setupRotation(const float start, const float target) {
        hasRotation_ = true;
        startRotation_ = start;
        targetRotation_ = config_.relative ? start + target : target;
    }

    void CameraTransition::setupFOV(const float start, const float target) {
        hasFOV_ = true;
        startFOV_ = start;
        targetFOV_ = config_.relative ? start + target : target;
    }

    Vec3 CameraTransition::getInterpolatedPosition() const {
        if (!hasPosition_) return startPosition_;

        const float t = getEasedProgress();
        return math::lerp(startPosition_, targetPosition_, t);
    }

    Vec3 CameraTransition::getInterpolatedTarget() const {
        if (!hasTarget_) return startTarget_;

        const float t = getEasedProgress();
        return math::lerp(startTarget_, targetLookAt_, t);
    }

    float CameraTransition::getInterpolatedZoom() const {
        if (!hasZoom_) return startZoom_;

        const float t = getEasedProgress();
        return startZoom_ + (targetZoom_ - startZoom_) * t;
    }

    float CameraTransition::getInterpolatedRotation() const {
        if (!hasRotation_) return startRotation_;

        const float t = getEasedProgress();
        return startRotation_ + (targetRotation_ - startRotation_) * t;
    }

    float CameraTransition::getInterpolatedFOV() const {
        if (!hasFOV_) return startFOV_;

        const float t = getEasedProgress();
        return startFOV_ + (targetFOV_ - startFOV_) * t;
    }

    void CameraTransition::start() {
        active_ = true;
        currentTime_ = 0.0f;
        delayRemaining_ = config_.delay;
    }

    void CameraTransition::stop(const bool triggerCallback) {
        if (active_ && !isComplete() && triggerCallback && config_.onInterrupt) {
            config_.onInterrupt(cameraId_, id_, false);
        }
        active_ = false;
    }

    bool CameraTransition::update(float deltaTime) {
        if (!active_) return false;

        // Handle delay
        if (delayRemaining_ > 0.0f) {
            delayRemaining_ -= deltaTime;
            if (delayRemaining_ > 0.0f) {
                return false; // Still delaying
            }
            // Delay complete, consume any excess time
            deltaTime = -delayRemaining_;
            delayRemaining_ = 0.0f;
        }

        // Update transition time
        currentTime_ += deltaTime;

        // Check if completed
        if (currentTime_ >= config_.duration) {
            currentTime_ = config_.duration;
            active_ = false;

            // Trigger completion callback
            if (config_.onComplete) {
                config_.onComplete(cameraId_, id_, true);
            }
            return true;
        }

        return false;
    }
} // namespace engine::camera
