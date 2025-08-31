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

#include <utility>

namespace engine::math::noise {
    /**
     * @brief Worley (Cellular) Noise - Creates cell-like patterns
     */
    class WorleyNoise {
    public:
        explicit WorleyNoise(UInt seed = 0);
        /**
         * @brief 2D Worley noise
         * @param x The x-coordinate in normalized space where noise is evaluated
         * @param y The y-coordinate in normalized space where noise is evaluated
         * @param distanceFunc Distance function (0=Euclidean, 1=Manhattan, 2=Chebyshev)
         * @return Distance to nearest point and second nearest
         */
        [[nodiscard]] std::pair<Float, Float> noise2D(Float x, Float y, int distanceFunc = 0) const noexcept;

        /**
         * @brief 3D Worley noise
         */
        [[nodiscard]] std::pair<Float, Float> noise3D(Float x, Float y, Float z, int distanceFunc = 0) const noexcept;

    private:
        UInt seed_;

        /**
         * @brief Hash function for cell coordinates
         */
        [[nodiscard]] UInt hash(int x, int y) const noexcept;

        [[nodiscard]] UInt hash(int x, int y, int z) const noexcept;

        /**
         * @brief Get pseudo-random feature point in cell
         */
        [[nodiscard]] Vec2 getFeaturePoint2D(int x, int y) const noexcept;

        [[nodiscard]] Vec3 getFeaturePoint3D(int x, int y, int z) const noexcept;
    };
} // namespace engine::math::noise
