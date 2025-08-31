/**
 * @file SimplexNoise.h
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


namespace engine::math::noise {
    /**
         * @brief Simplex Noise - Faster and smoother than Perlin noise
         *
         * Based on Stefan Gustavson's implementation, which is more efficient
         * than classic Perlin noise and has better visual properties.
         */
    class SimplexNoise {
    public:
        explicit SimplexNoise(UInt seed = 0) noexcept;

        void initialize(UInt seed = 0) noexcept;

        /**
         * @brief 2D Simplex noise
         */
        [[nodiscard]] Float noise2D(Float x, Float y) const noexcept;

        /**
         * @brief 3D Simplex noise
         */
        [[nodiscard]] Float noise3D(Float x, Float y, Float z) const noexcept;

    private:
        std::array<int, 512> perm_;
        std::array<int, 512> permMod12_;

        // Gradient vectors for 3D
        static constexpr std::array<std::array<Float, 3>, 12> grad3_ = {
            {
                {1, 1, 0}, {-1, 1, 0}, {1, -1, 0}, {-1, -1, 0},
                {1, 0, 1}, {-1, 0, 1}, {1, 0, -1}, {-1, 0, -1},
                {0, 1, 1}, {0, -1, 1}, {0, 1, -1}, {0, -1, -1}
            }
        };

        [[nodiscard]] static Float dot(const std::array<Float, 3>& g, const Float x, const Float y) noexcept {
            return g[0] * x + g[1] * y;
        }

        [[nodiscard]] static Float dot(const std::array<Float, 3>& g, const Float x, const Float y,
                                       const Float z) noexcept {
            return g[0] * x + g[1] * y + g[2] * z;
        }
    };
} // namespace engine::math::noise
