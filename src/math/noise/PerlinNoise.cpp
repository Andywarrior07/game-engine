/**
 * @file PerlinNoise.cpp
 * @brief Perlin and Simplex noise generation for procedural content
 * @author Andrés Guerrero
 * @date 27-08-2025
 *
 * Provides high-quality gradient noise implementations for terrain generation,
 * texture synthesis, and procedural animation.
 */

#include "PerlinNoise.h"

#include "../core/MathConstants.h"
#include "../core/MathFunctions.h"
#include "../core/FastMath.h"

#include <random>

namespace engine::math::noise {
    PerlinNoise::PerlinNoise(const UInt seed) noexcept {
        initialize(seed);
    }

    void PerlinNoise::initialize(const UInt seed) noexcept {
        // Initialize permutation table
        for (int i = 0; i < 256; ++i) {
            p_[i] = i;
        }

        // Shuffle using seed
        if (seed != 0) {
            std::mt19937 gen(seed);
            std::shuffle(p_.begin(), p_.begin() + 256, gen);
        }

        // Duplicate for overflow
        for (int i = 0; i < 256; ++i) {
            p_[256 + i] = p_[i];
        }
    }

    Float PerlinNoise::noise1D(Float x) const noexcept {
        // Find unit grid cell
        const int X = fast::fastFloor(x) & 255;

        // Relative position in cell
        x -= std::floor(x);

        // Compute fade curve
        const Float u = fade(x);

        // Hash coordinates
        const int A = p_[X];
        const int B = p_[X + 1];

        // Blend results
        return lerp(grad1D(A, x), grad1D(B, x - 1), u);
    }

    Float PerlinNoise::noise2D(Float x, Float y) const noexcept {
        // Find unit grid cell
        const int X = fast::fastFloor(x) & 255;
        const int Y = fast::fastFloor(y) & 255;

        // Relative position in cell
        x -= std::floor(x);
        y -= std::floor(y);

        // Compute fade curves
        const Float u = fade(x);
        const Float v = fade(y);

        // Hash coordinates of 4 corners
        const int A = p_[X] + Y;
        const int AA = p_[A];
        const int AB = p_[A + 1];
        const int B = p_[X + 1] + Y;
        const int BA = p_[B];
        const int BB = p_[B + 1];

        // Blend results from 4 corners
        const Float res = lerp(
            lerp(grad2D(p_[AA], x, y),
                 grad2D(p_[BA], x - 1, y), u),
            lerp(grad2D(p_[AB], x, y - 1),
                 grad2D(p_[BB], x - 1, y - 1), u),
            v
        );

        return res;
    }

    Float PerlinNoise::noise3D(Float x, Float y, Float z) const noexcept {
        // Find unit cube
        const int X = fast::fastFloor(x) & 255;
        const int Y = fast::fastFloor(y) & 255;
        const int Z = fast::fastFloor(z) & 255;

        // Relative position in cube
        x -= std::floor(x);
        y -= std::floor(y);
        z -= std::floor(z);

        // Compute fade curves
        const Float u = fade(x);
        const Float v = fade(y);
        const Float w = fade(z);

        // Hash coordinates of 8 cube corners
        const int A = p_[X] + Y;
        const int AA = p_[A] + Z;
        const int AB = p_[A + 1] + Z;
        const int B = p_[X + 1] + Y;
        const int BA = p_[B] + Z;
        const int BB = p_[B + 1] + Z;

        // Blend results from 8 corners
        const Float res = lerp(
            lerp(lerp(grad3D(p_[AA], x, y, z),
                      grad3D(p_[BA], x - 1, y, z), u),
                 lerp(grad3D(p_[AB], x, y - 1, z),
                      grad3D(p_[BB], x - 1, y - 1, z), u), v),
            lerp(lerp(grad3D(p_[AA + 1], x, y, z - 1),
                      grad3D(p_[BA + 1], x - 1, y, z - 1), u),
                 lerp(grad3D(p_[AB + 1], x, y - 1, z - 1),
                      grad3D(p_[BB + 1], x - 1, y - 1, z - 1), u), v),
            w
        );

        return res;
    }

    Float PerlinNoise::fbm2D(const Float x, const Float y, const int octaves,
                             const Float lacunarity, const Float gain) const noexcept {
        Float value = 0;
        Float amplitude = 1;
        Float frequency = 1;
        Float maxValue = 0;

        for (int i = 0; i < octaves; ++i) {
            value += noise2D(x * frequency, y * frequency) * amplitude;
            maxValue += amplitude;
            amplitude *= gain;
            frequency *= lacunarity;
        }

        return value / maxValue;
    }

    Float PerlinNoise::fbm3D(const Float x, const Float y, const Float z, const int octaves,
                             const Float lacunarity, const Float gain) const noexcept {
        Float value = 0;
        Float amplitude = 1;
        Float frequency = 1;
        Float maxValue = 0;

        for (int i = 0; i < octaves; ++i) {
            value += noise3D(x * frequency, y * frequency, z * frequency) * amplitude;
            maxValue += amplitude;
            amplitude *= gain;
            frequency *= lacunarity;
        }

        return value / maxValue;
    }

    Float PerlinNoise::turbulence2D(const Float x, const Float y, const int octaves,
                                    const Float lacunarity, const Float gain) const noexcept {
        Float value = 0;
        Float amplitude = 1;
        Float frequency = 1;
        Float maxValue = 0;

        for (int i = 0; i < octaves; ++i) {
            value += std::abs(noise2D(x * frequency, y * frequency)) * amplitude;
            maxValue += amplitude;
            amplitude *= gain;
            frequency *= lacunarity;
        }

        return value / maxValue;
    }

    Float PerlinNoise::ridged2D(const Float x, const Float y, const int octaves,
                                const Float lacunarity, const Float gain) const noexcept {
        Float value = 0;
        Float amplitude = 1;
        Float frequency = 1;
        Float maxValue = 0;

        for (int i = 0; i < octaves; ++i) {
            Float n = 1.0f - std::abs(noise2D(x * frequency, y * frequency));
            n = n * n;
            value += n * amplitude;
            maxValue += amplitude;
            amplitude *= gain;
            frequency *= lacunarity;
        }

        return value / maxValue;
    }

    Float PerlinNoise::grad2D(const int hash, const Float x, const Float y) noexcept {
        const int h = hash & 3;
        const Float u = h < 2 ? x : y;
        const Float v = h < 2 ? y : x;

        return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
    }

    Float PerlinNoise::grad3D(const int hash, const Float x, const Float y, const Float z) noexcept {
        const int h = hash & 15;
        const Float u = h < 8 ? x : y;
        const Float v = h < 4 ? y : h == 12 || h == 14 ? x : z;

        return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
    }
} // namespace engine::math::noise
