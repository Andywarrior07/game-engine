/**
 * @file SimplexNoise.cpp
 * @brief Perlin and Simplex noise generation for procedural content
 * @author Andrés Guerrero
 * @date 27-08-2025
 *
 * Provides high-quality gradient noise implementations for terrain generation,
 * texture synthesis, and procedural animation.
 */


#include "SimplexNoise.h"

#include <random>

#include "../core/FastMath.h"

namespace engine::math::noise {
    SimplexNoise::SimplexNoise(const UInt seed) noexcept {
        initialize(seed);
    }

    void SimplexNoise::initialize(const UInt seed) noexcept {
        // Initialize permutation table (similar to Perlin)
        for (int i = 0; i < 256; ++i) {
            perm_[i] = i;
        }

        if (seed != 0) {
            std::mt19937 gen(seed);
            std::shuffle(perm_.begin(), perm_.begin() + 256, gen);
        }

        for (int i = 0; i < 256; ++i) {
            perm_[256 + i] = perm_[i];
            permMod12_[i] = perm_[i] % 12;
            permMod12_[256 + i] = permMod12_[i];
        }
    }

    Float SimplexNoise::noise2D(const Float x, const Float y) const noexcept {
        const Float F2 = 0.5f * (std::sqrt(3.0f) - 1.0f);
        const Float G2 = (3.0f - std::sqrt(3.0f)) / 6.0f;

        // Skew the input space
        const Float s = (x + y) * F2;
        const int i = fast::fastFloor(x + s);
        const int j = fast::fastFloor(y + s);

        const Float t = (i + j) * G2;
        const Float X0 = i - t;
        const Float Y0 = j - t;
        const Float x0 = x - X0;
        const Float y0 = y - Y0;

        // Determine which simplex we're in
        int i1, j1;
        if (x0 > y0) {
            i1 = 1;
            j1 = 0;
        }
        else {
            i1 = 0;
            j1 = 1;
        }

        // Calculate offsets for corners
        const Float x1 = x0 - i1 + G2;
        const Float y1 = y0 - j1 + G2;
        const Float x2 = x0 - 1.0f + 2.0f * G2;
        const Float y2 = y0 - 1.0f + 2.0f * G2;

        // Hash coordinates
        const int ii = i & 255;
        const int jj = j & 255;
        const int gi0 = permMod12_[ii + perm_[jj]];
        const int gi1 = permMod12_[ii + i1 + perm_[jj + j1]];
        const int gi2 = permMod12_[ii + 1 + perm_[jj + 1]];

        // Calculate contributions
        Float t0 = 0.5f - x0 * x0 - y0 * y0;
        Float n0 = 0;
        if (t0 >= 0) {
            t0 *= t0;
            n0 = t0 * t0 * dot(grad3_[gi0], x0, y0);
        }

        Float t1 = 0.5f - x1 * x1 - y1 * y1;
        Float n1 = 0;
        if (t1 >= 0) {
            t1 *= t1;
            n1 = t1 * t1 * dot(grad3_[gi1], x1, y1);
        }

        Float t2 = 0.5f - x2 * x2 - y2 * y2;
        Float n2 = 0;
        if (t2 >= 0) {
            t2 *= t2;
            n2 = t2 * t2 * dot(grad3_[gi2], x2, y2);
        }

        // Scale to [-1, 1]
        return 70.0f * (n0 + n1 + n2);
    }

    Float SimplexNoise::noise3D(const Float x, const Float y, const Float z) const noexcept {
        constexpr Float F3 = 1.0f / 3.0f;
        constexpr Float G3 = 1.0f / 6.0f;

        // Skew the input space
        const Float s = (x + y + z) * F3;
        const int i = fast::fastFloor(x + s);
        const int j = fast::fastFloor(y + s);
        const int k = fast::fastFloor(z + s);

        const Float t = (i + j + k) * G3;
        const Float X0 = i - t;
        const Float Y0 = j - t;
        const Float Z0 = k - t;
        const Float x0 = x - X0;
        const Float y0 = y - Y0;
        const Float z0 = z - Z0;

        // Determine simplex
        int i1, j1, k1;
        int i2, j2, k2;

        if (x0 >= y0) {
            if (y0 >= z0) {
                i1 = 1;
                j1 = 0;
                k1 = 0;
                i2 = 1;
                j2 = 1;
                k2 = 0;
            }
            else if (x0 >= z0) {
                i1 = 1;
                j1 = 0;
                k1 = 0;
                i2 = 1;
                j2 = 0;
                k2 = 1;
            }
            else {
                i1 = 0;
                j1 = 0;
                k1 = 1;
                i2 = 1;
                j2 = 0;
                k2 = 1;
            }
        }
        else {
            if (y0 < z0) {
                i1 = 0;
                j1 = 0;
                k1 = 1;
                i2 = 0;
                j2 = 1;
                k2 = 1;
            }
            else if (x0 < z0) {
                i1 = 0;
                j1 = 1;
                k1 = 0;
                i2 = 0;
                j2 = 1;
                k2 = 1;
            }
            else {
                i1 = 0;
                j1 = 1;
                k1 = 0;
                i2 = 1;
                j2 = 1;
                k2 = 0;
            }
        }

        // Calculate offsets
        const Float x1 = x0 - i1 + G3;
        const Float y1 = y0 - j1 + G3;
        const Float z1 = z0 - k1 + G3;
        const Float x2 = x0 - i2 + 2.0f * G3;
        const Float y2 = y0 - j2 + 2.0f * G3;
        const Float z2 = z0 - k2 + 2.0f * G3;
        const Float x3 = x0 - 1.0f + 3.0f * G3;
        const Float y3 = y0 - 1.0f + 3.0f * G3;
        const Float z3 = z0 - 1.0f + 3.0f * G3;

        // Hash coordinates
        const int ii = i & 255;
        const int jj = j & 255;
        const int kk = k & 255;
        const int gi0 = permMod12_[ii + perm_[jj + perm_[kk]]];
        const int gi1 = permMod12_[ii + i1 + perm_[jj + j1 + perm_[kk + k1]]];
        const int gi2 = permMod12_[ii + i2 + perm_[jj + j2 + perm_[kk + k2]]];
        const int gi3 = permMod12_[ii + 1 + perm_[jj + 1 + perm_[kk + 1]]];

        // Calculate contributions
        Float t0 = 0.6f - x0 * x0 - y0 * y0 - z0 * z0;
        Float n0 = 0;
        if (t0 >= 0) {
            t0 *= t0;
            n0 = t0 * t0 * dot(grad3_[gi0], x0, y0, z0);
        }

        Float t1 = 0.6f - x1 * x1 - y1 * y1 - z1 * z1;
        Float n1 = 0;
        if (t1 >= 0) {
            t1 *= t1;
            n1 = t1 * t1 * dot(grad3_[gi1], x1, y1, z1);
        }

        Float t2 = 0.6f - x2 * x2 - y2 * y2 - z2 * z2;
        Float n2 = 0;
        if (t2 >= 0) {
            t2 *= t2;
            n2 = t2 * t2 * dot(grad3_[gi2], x2, y2, z2);
        }

        Float t3 = 0.6f - x3 * x3 - y3 * y3 - z3 * z3;
        Float n3 = 0;
        if (t3 >= 0) {
            t3 *= t3;
            n3 = t3 * t3 * dot(grad3_[gi3], x3, y3, z3);
        }

        // Scale to [-1, 1]
        return 32.0f * (n0 + n1 + n2 + n3);
    }
} // namespace engine::math::noise