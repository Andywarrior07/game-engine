/**
 * @file CameraShake.cpp
 * @brief Camera shake effect implementation
 * @author Andrés Guerrero
 * @date 25-08-2025
 */
#include "CameraShake.h"

namespace engine::camera {
    CameraShake::CameraShake(const CameraID cameraId, const ShakeConfig& config)
        : cameraId_(cameraId)
          , config_(config)
          , currentOffset_(math::constants::VEC3_ZERO)
          , active_(true)
          , basePosition_(math::constants::VEC3_ZERO) {
    }

    float CameraShake::getCurrentIntensity() const {
        if (!active_) return 0.0f;

        float intensity = config_.intensity;

        // Apply fade in
        if (config_.fadeIn && currentTime_ < config_.fadeInDuration) {
            intensity *= (currentTime_ / config_.fadeInDuration);
        }

        // Apply fade out
        if (config_.fadeOut && currentTime_ < config_.fadeOutDuration) {
            if (const float fadeOutStart = config_.duration - config_.fadeOutDuration;
                currentTime_ >= fadeOutStart) {
                const float fadeProgress = (currentTime_ - fadeOutStart) / config_.fadeOutDuration;
                intensity *= (1.0f - fadeProgress);
            }
        }

        return intensity;
    }

    float CameraShake::getProgress() const {
        if (config_.duration <= 0.0f) return 1.0f;

        return std::min(currentTime_ / config_.duration, 1.0f);
    }

    void CameraShake::setConfig(ShakeConfig config) {
        config_ = std::move(config);
    }

    void CameraShake::reset() {
        currentTime_ = 0.0f;
        currentOffset_ = math::constants::VEC3_ZERO;
        active_ = true;
    }

    bool CameraShake::update(const float deltaTime) {
        if (!active_) return false;

        currentTime_ += deltaTime;

        if (config_.duration > 0.0f && currentTime_ >= config_.duration) {
            active_ = false;
            currentOffset_ = math::constants::VEC3_ZERO;

            // Trigger callback
            if (config_.onComplete) {
                config_.onComplete(cameraId_, 0.0f, true);
            }

            return true;
        }

        // Generate new offset
        const float intensity = getCurrentIntensity();
        currentOffset_ = generateOffset(config_.pattern, intensity, currentTime_, config_.frequency);

        // Apply axis scaling
        currentOffset_.x *= config_.axes.x;
        currentOffset_.y *= config_.axes.y;
        currentOffset_.z *= config_.axes.z;

        return false;
    }
} // namespace engine::camera
