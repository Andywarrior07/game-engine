/**
 * @file PerlinNoise.h
 * @brief Perlin and Simplex noise generation for procedural content
 * @author Andrés Guerrero
 * @date 27-08-2025
 *
 * Provides high-quality gradient noise implementations for terrain generation,
 * texture synthesis, and procedural animation.
 */

#pragma once

#include "../core/MathTypes.h"

#include <array>

// TODO: Preguntar porque este namespace
namespace engine::math::noise {
    /**
     * @brief Improved Perlin Noise implementation
     *
     * Based on Ken Perlin's improved noise algorithm with better gradient
     * distribution and smoother interpolation.
     */
    class PerlinNoise {
    public:
        /**
         * @brief Initialize with optional seed
         */
        explicit PerlinNoise(UInt seed = 0) noexcept;

        /**
         * @brief Re-initialize with new seed
         */
        void initialize(UInt seed = 0) noexcept;

        /**
         * @brief 1D Perlin noise
         */
        [[nodiscard]] Float noise1D(Float x) const noexcept;

        /**
         * @brief 2D Perlin noise
         */
        [[nodiscard]] Float noise2D(Float x, Float y) const noexcept;

        /**
         * @brief 3D Perlin noise
         */
        [[nodiscard]] Float noise3D(Float x, Float y, Float z) const noexcept;

        /**
         * @brief Fractal Brownian Motion (fBm) - Multiple octaves of noise
         */
        [[nodiscard]] Float fbm2D(Float x, Float y, int octaves = 6,
                                  Float lacunarity = 2.0f, Float gain = 0.5f) const noexcept;

        [[nodiscard]] Float fbm3D(Float x, Float y, Float z, int octaves = 6,
                                  Float lacunarity = 2.0f, Float gain = 0.5f) const noexcept;

        /**
         * @brief Turbulence - Absolute value of noise for billowy effect
         */
        [[nodiscard]] Float turbulence2D(Float x, Float y, int octaves = 6,
                                         Float lacunarity = 2.0f, Float gain = 0.5f) const noexcept;

        /**
         * @brief Ridged noise - Inverted absolute value for ridge-like features
         */
        [[nodiscard]] Float ridged2D(Float x, Float y, int octaves = 6,
                                     Float lacunarity = 2.0f, Float gain = 0.5f) const noexcept;

    private:
        std::array<int, 512> p_; // Permutation table

        /**
         * @brief Fade function for smooth interpolation
         * 6t^5 - 15t^4 + 10t^3
         */
        [[nodiscard]] static Float fade(const Float t) noexcept {
            return ((6 * t - 15) * t + 10) * t * t * t;
        }

        /**
         * @brief 1D gradient function
         */
        [[nodiscard]] static Float grad1D(const int hash, const Float x) noexcept {
            const int h = hash & 1;
            return h == 0 ? x : -x;
        }

        /**
         * @brief 2D gradient function
         */
        [[nodiscard]] static Float grad2D(int hash, Float x, Float y) noexcept;

        /**
         * @brief 3D gradient function
         */
        [[nodiscard]] static Float grad3D(int hash, Float x, Float y, Float z) noexcept;
    };
} // namespace engine::math::noise
