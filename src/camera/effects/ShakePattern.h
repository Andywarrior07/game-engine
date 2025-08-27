/**
 * @file ShakePattern.h
 * @brief Camera shake patterns enumeration
 * @author Andrés Guerrero
 * @date 25-08-2025
 */
#pragma once

#include <cstdint>

namespace engine::camera {
    /**
     * @brief Camera shake patterns for different effects
     *
     * Define various shake patterns that can be applied to cameras
     * for different gameplay scenarios and visual effects.
     */
    enum class ShakePattern : std::uint8_t {
        RANDOM = 0, ///< Random shake in all directions
        HORIZONTAL = 1, ///< Only horizontal shake (left-right)
        VERTICAL = 2, ///< Only vertical shake (up-down)
        CIRCULAR = 3, ///< Circular/orbital shake pattern
        EXPLOSION = 4, ///< Explosion-like shake (rapid decay)
        EARTHQUAKE = 5, ///< Earthquake-like shake (low frequency, high amplitude)
        HANDHELD = 6, ///< Handheld camera simulation
        VIBRATION = 7, ///< High frequency vibration
        IMPACT = 8, ///< Single impact shake
        WAVE = 9 ///< Wave-like oscillation
    };

    /**
     * @brief Convert shake pattern to string for debugging
     * @param pattern Shake pattern to convert
     * @return String representation of shake pattern
    */
    inline const char* shakePatternToString(ShakePattern pattern) {
        switch (pattern) {
        case ShakePattern::RANDOM: return "Random";
        case ShakePattern::HORIZONTAL: return "Horizontal";
        case ShakePattern::VERTICAL: return "Vertical";
        case ShakePattern::CIRCULAR: return "Circular";
        case ShakePattern::EXPLOSION: return "Explosion";
        case ShakePattern::EARTHQUAKE: return "Earthquake";
        case ShakePattern::HANDHELD: return "Handheld";
        case ShakePattern::VIBRATION: return "Vibration";
        case ShakePattern::IMPACT: return "Impact";
        case ShakePattern::WAVE: return "Wave";
        default: return "Unknown";
        }
    }
} // namespace engine::camera
