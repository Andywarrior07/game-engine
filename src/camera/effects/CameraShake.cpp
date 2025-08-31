/**
 * @file CameraShake.cpp
 * @brief Camera shake effect implementation
 * @author Andrés Guerrero
 * @date 25-08-2025
 */
#include "CameraShake.h"

#include "../../math/geometry/Primitives.h"

#include <random>

namespace engine::camera {
    CameraShake::CameraShake(const CameraID cameraId, const ShakeConfig& config)
        : cameraId_(cameraId)
          , config_(config)
          , currentOffset_(math::VEC3_ZERO)
          , active_(true)
          , basePosition_(math::VEC3_ZERO) {
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
        currentOffset_ = math::VEC3_ZERO;
        active_ = true;
    }

    bool CameraShake::update(const float deltaTime) {
        if (!active_) return false;

        currentTime_ += deltaTime;

        if (config_.duration > 0.0f && currentTime_ >= config_.duration) {
            active_ = false;
            currentOffset_ = math::VEC3_ZERO;

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

    Vec3 CameraShake::generateOffset(ShakePattern pattern, float intensity,
                                      float time, float frequency) {
            thread_local std::mt19937 generator(std::random_device{}());
            thread_local std::uniform_real_distribution distribution(-1.0f, 1.0f);

            Vec3 offset;

            switch (pattern) {
                case ShakePattern::RANDOM: {
                    offset.x = distribution(generator) * intensity;
                    offset.y = distribution(generator) * intensity;
                    offset.z = distribution(generator) * intensity * 0.5f;
                    break;
                }

                case ShakePattern::HORIZONTAL: {
                    offset.x = distribution(generator) * intensity;
                    offset.y = 0.0f;
                    offset.z = 0.0f;
                    break;
                }

                case ShakePattern::VERTICAL: {
                    offset.x = 0.0f;
                    offset.y = distribution(generator) * intensity;
                    offset.z = 0.0f;
                    break;
                }

                case ShakePattern::CIRCULAR: {
                    float angle = time * frequency * 2.0f * M_PI;
                    offset.x = std::cos(angle) * intensity;
                    offset.y = std::sin(angle) * intensity;
                    offset.z = 0.0f;
                    break;
                }

                case ShakePattern::EXPLOSION: {
                    float explosionT = std::max(0.0f, 1.0f - time * 2.0f);
                    float explosionIntensity = intensity * explosionT * explosionT;
                    offset.x = distribution(generator) * explosionIntensity;
                    offset.y = distribution(generator) * explosionIntensity;
                    offset.z = distribution(generator) * explosionIntensity * 0.3f;
                    break;
                }

                case ShakePattern::EARTHQUAKE: {
                    float lowFreq = frequency * 0.3f;
                    float noise1 = std::sin(time * lowFreq * 2.0f * M_PI) * intensity;
                    float noise2 = std::sin(time * lowFreq * 2.0f * M_PI * 1.7f) * intensity * 0.7f;
                    offset.x = noise1 + distribution(generator) * intensity * 0.2f;
                    offset.y = noise2 + distribution(generator) * intensity * 0.1f;
                    offset.z = distribution(generator) * intensity * 0.1f;
                    break;
                }

                case ShakePattern::HANDHELD: {
                    // Low frequency, smooth movement
                    float angle1 = time * frequency;
                    float angle2 = time * frequency * 1.3f;
                    offset.x = std::sin(angle1) * intensity * 0.5f;
                    offset.y = std::sin(angle2) * intensity * 0.3f;
                    offset.z = std::sin(angle1 * 0.7f) * intensity * 0.1f;
                    break;
                }

                case ShakePattern::VIBRATION: {
                    // High frequency, low amplitude
                    offset.x = distribution(generator) * intensity * 0.3f;
                    offset.y = distribution(generator) * intensity * 0.3f;
                    offset.z = 0.0f;
                    break;
                }

                case ShakePattern::IMPACT: {
                    // Single direction with decay
                    float decay = std::max(0.0f, 1.0f - time * 5.0f);
                    offset = Vec3(intensity * decay, 0.0f, 0.0f);
                    break;
                }

                case ShakePattern::WAVE: {
                    float wave = std::sin(time * frequency * 2.0f * M_PI);
                    offset.x = wave * intensity;
                    offset.y = wave * intensity * 0.5f;
                    offset.z = 0.0f;
                    break;
                }

                default:
                    offset = math::VEC3_ZERO;
                    break;
            }

            return offset;
        }
} // namespace engine::camera
