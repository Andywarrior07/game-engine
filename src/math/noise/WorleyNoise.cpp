/**
 * @file PerlinNoise.cpp
 * @brief Perlin and Simplex noise generation for procedural content
 * @author Andrés Guerrero
 * @date 27-08-2025
 *
 * Provides high-quality gradient noise implementations for terrain generation,
 * texture synthesis, and procedural animation.
 */

#include "WorleyNoise.h"

#include "../core/FastMath.h"
#include "../core/MathFunctions.h"

namespace engine::math::noise {
    WorleyNoise::WorleyNoise(const UInt seed) : seed_(seed) {
    }

    std::pair<Float, Float> WorleyNoise::noise2D(const Float x, const Float y, const int distanceFunc) const noexcept {
        const int xi = fast::fastFloor(x);
        const int yi = fast::fastFloor(y);

        Float minDist1 = INFINITY_VALUE<Float>;
        Float minDist2 = INFINITY_VALUE<Float>;

        // Check neighboring cells
        for (int j = -1; j <= 1; ++j) {
            for (int i = -1; i <= 1; ++i) {
                const int cellX = xi + i;
                const int cellY = yi + j;

                // Get feature point in cell
                Vec2 featurePoint = getFeaturePoint2D(cellX, cellY);
                featurePoint.x += cellX;
                featurePoint.y += cellY;

                // Calculate distance
                Float dist;
                if (distanceFunc == 0) {
                    // Euclidean
                    dist = glm::length(Vec2(x, y) - featurePoint);
                }
                else if (distanceFunc == 1) {
                    // Manhattan
                    const Vec2 diff = glm::abs(Vec2(x, y) - featurePoint);
                    dist = diff.x + diff.y;
                }
                else {
                    // Chebyshev
                    const Vec2 diff = glm::abs(Vec2(x, y) - featurePoint);
                    dist = max(diff.x, diff.y);
                }

                // Update nearest distances
                if (dist < minDist1) {
                    minDist2 = minDist1;
                    minDist1 = dist;
                }
                else if (dist < minDist2) {
                    minDist2 = dist;
                }
            }
        }

        return {minDist1, minDist2};
    }

    std::pair<Float, Float> WorleyNoise::noise3D(const Float x, const Float y, const Float z,
                                                 const int distanceFunc) const noexcept {
        const int xi = fast::fastFloor(x);
        const int yi = fast::fastFloor(y);
        const int zi = fast::fastFloor(z);

        Float minDist1 = INFINITY_VALUE<Float>;
        Float minDist2 = INFINITY_VALUE<Float>;

        // Check neighboring cells
        for (int k = -1; k <= 1; ++k) {
            for (int j = -1; j <= 1; ++j) {
                for (int i = -1; i <= 1; ++i) {
                    const int cellX = xi + i;
                    const int cellY = yi + j;
                    const int cellZ = zi + k;

                    // Get feature point in cell
                    Vec3 featurePoint = getFeaturePoint3D(cellX, cellY, cellZ);
                    featurePoint += Vec3(cellX, cellY, cellZ);

                    // Calculate distance
                    Float dist;
                    if (distanceFunc == 0) {
                        // Euclidean
                        dist = glm::length(Vec3(x, y, z) - featurePoint);
                    }
                    else if (distanceFunc == 1) {
                        // Manhattan
                        const Vec3 diff = glm::abs(Vec3(x, y, z) - featurePoint);
                        dist = diff.x + diff.y + diff.z;
                    }
                    else {
                        // Chebyshev
                        const Vec3 diff = glm::abs(Vec3(x, y, z) - featurePoint);
                        dist = max3(diff.x, diff.y, diff.z);
                    }

                    // Update nearest distances
                    if (dist < minDist1) {
                        minDist2 = minDist1;
                        minDist1 = dist;
                    }
                    else if (dist < minDist2) {
                        minDist2 = dist;
                    }
                }
            }
        }

        return {minDist1, minDist2};
    }

    UInt WorleyNoise::hash(const int x, const int y) const noexcept {
        UInt h = seed_;
        h ^= x * 374761393;
        h ^= y * 668265263;
        h = (h ^ (h >> 13)) * 1274126177;

        return h ^ (h >> 16);
    }

    UInt WorleyNoise::hash(const int x, const int y, const int z) const noexcept {
        UInt h = seed_;
        h ^= x * 374761393;
        h ^= y * 668265263;
        h ^= z * 1049445839;
        h = (h ^ (h >> 13)) * 1274126177;

        return h ^ (h >> 16);
    }

    Vec2 WorleyNoise::getFeaturePoint2D(const int x, const int y) const noexcept {
        const UInt h = hash(x, y);
        const Float fx = (h & 0xFFFF) / Float(0xFFFF);
        const Float fy = ((h >> 16) & 0xFFFF) / Float(0xFFFF);

        return Vec2(fx, fy);
    }

    Vec3 WorleyNoise::getFeaturePoint3D(const int x, const int y, const int z) const noexcept {
        const UInt h = hash(x, y, z);
        const Float fx = (h & 0x3FF) / static_cast<Float>(0x3FF);
        const Float fy = ((h >> 10) & 0x3FF) / static_cast<Float>(0x3FF);
        const Float fz = ((h >> 20) & 0x3FF) / static_cast<Float>(0x3FF);

        return Vec3(fx, fy, fz);
    }
} // namespace engine::math::noise
