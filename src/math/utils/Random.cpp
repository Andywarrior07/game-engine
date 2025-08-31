/**
 * @file Random.cpp
 * @brief High-quality random number generation for games
 * @author Andrés Guerrero
 * @date 27-08-2025
 *
 * Provides various random distributions and utilities for procedural generation,
 * gameplay mechanics, and stochastic simulations.
 */

#include "Random.h"

#include "../core/MathFunctions.h"

namespace engine::math {
    Random::Random(Int seed) noexcept {
        if (seed == 0) {
            seed = static_cast<Int>(
                std::chrono::steady_clock::now().time_since_epoch().count()
            );
        }
        setSeed(seed);
    }

    void Random::setSeed(const Int seed) noexcept {
        generator_.seed(seed);
        currentSeed_ = seed;
    }

    Float Random::random() noexcept {
        return uniformReal_(generator_);
    }

    Float Random::range(const Float min, const Float max) noexcept {
        return min + (max - min) * random();
    }

    Int Random::rangeInt(const Int min, const Int max) noexcept {
        std::uniform_int_distribution<Int> dist(min, max);
        return dist(generator_);
    }

    bool Random::randomBool(const Float probability) noexcept {
        return random() < probability;
    }

    Float Random::randomSign() noexcept {
        return randomBool() ? 1.0f : -1.0f;
    }

    Vec2 Random::randomVec2() noexcept {
        return Vec2(random(), random());
    }

    Vec2 Random::randomVec2(const Float min, const Float max) noexcept {
        return Vec2(range(min, max), range(min, max));
    }

    Vec3 Random::randomVec3() noexcept {
        return Vec3(random(), random(), random());
    }

    Vec3 Random::randomVec3(const Float min, const Float max) noexcept {
        return Vec3(range(min, max), range(min, max), range(min, max));
    }

    Vec2 Random::randomUnitVec2() noexcept {
        const Float angle = range(0, TWO_PI<Float>);
        return Vec2(std::cos(angle), std::sin(angle));
    }

    Vec3 Random::randomUnitVec3() noexcept {
        const Float z = range(-1.0f, 1.0f);
        const Float angle = range(0, TWO_PI<Float>);
        const Float r = std::sqrt(1.0f - z * z);
        return Vec3(r * std::cos(angle), r * std::sin(angle), z);
    }

    Vec2 Random::randomInCircle() noexcept {
        const Float angle = range(0, TWO_PI<Float>);
        const Float r = std::sqrt(random()); // Square root for uniform distribution
        return Vec2(r * std::cos(angle), r * std::sin(angle));
    }

    Vec3 Random::randomInSphere() noexcept {
        Vec3 p;
        do {
            p = randomVec3(-1.0f, 1.0f);
        }
        while (glm::length2(p) > 1.0f);
        return p;
    }

    Vec3 Random::randomInHemisphere() noexcept {
        const Vec3 v = randomUnitVec3();
        return Vec3(v.x, v.y, std::abs(v.z));
    }

    Quat Random::randomQuaternion() noexcept {
        // Using Shoemake's algorithm for uniform distribution
        const Float u1 = random();
        const Float u2 = random();
        const Float u3 = random();

        const Float sqrt1MinusU1 = std::sqrt(1.0f - u1);
        const Float sqrtU1 = std::sqrt(u1);

        return Quat(
            sqrt1MinusU1 * std::sin(TWO_PI<Float> * u2),
            sqrt1MinusU1 * std::cos(TWO_PI<Float> * u2),
            sqrtU1 * std::sin(TWO_PI<Float> * u3),
            sqrtU1 * std::cos(TWO_PI<Float> * u3)
        );
    }

    Float Random::gaussian(const Float mean, const Float stddev) noexcept {
        std::normal_distribution dist(mean, stddev);
        return dist(generator_);
    }

    Float Random::exponential(const Float lambda) noexcept {
        std::exponential_distribution dist(lambda);
        return dist(generator_);
    }

    Int Random::poisson(const Float mean) noexcept {
        std::poisson_distribution dist(mean);
        return dist(generator_);
    }

    Float Random::chiSquared(const Float degreesOfFreedom) noexcept {
        std::chi_squared_distribution dist(degreesOfFreedom);
        return dist(generator_);
    }

    std::size_t Random::weightedChoice(const std::vector<Float>& weights) noexcept {
        if (weights.empty()) return 0;

        Float sum = 0;
        for (const Float w : weights) sum += w;

        const Float r = range(0, sum);
        Float cumulative = 0;

        for (std::size_t i = 0; i < weights.size(); ++i) {
            cumulative += weights[i];
            if (r <= cumulative) return i;
        }

        return weights.size() - 1;
    }

    Float Random::noise1D(const Float x) noexcept {
        const Int x0 = static_cast<Int>(std::floor(x));
        const Int x1 = x0 + 1;
        const Float fx = x - x0;

        const Float v0 = pseudoRandom(x0);
        const Float v1 = pseudoRandom(x1);

        return lerp(v0, v1, smoothstep(0.0f, 1.0f, fx));
    }

    Float Random::noise2D(const Float x, const Float y) noexcept {
        const Int x0 = static_cast<Int>(std::floor(x));
        const Int y0 = static_cast<Int>(std::floor(y));
        const Int x1 = x0 + 1;
        const Int y1 = y0 + 1;

        const Float fx = x - x0;
        const Float fy = y - y0;

        const Float v00 = pseudoRandom2D(x0, y0);
        const Float v10 = pseudoRandom2D(x1, y0);
        const Float v01 = pseudoRandom2D(x0, y1);
        const Float v11 = pseudoRandom2D(x1, y1);

        const Float sx = smoothstep(0.0f, 1.0f, fx);
        const Float sy = smoothstep(0.0f, 1.0f, fy);

        const Float a = lerp(v00, v10, sx);
        const Float b = lerp(v01, v11, sx);

        return lerp(a, b, sy);
    }

    Float Random::pseudoRandom(Int x) noexcept {
        x = ((x >> 16) ^ x) * 0x45d9f3b;
        x = ((x >> 16) ^ x) * 0x45d9f3b;
        x = (x >> 16) ^ x;
        return (x & 0x7FFFFFFF) / static_cast<Float>(0x7FFFFFFF);
    }

    Float Random::pseudoRandom2D(const Int x, const Int y) noexcept {
        constexpr Int prime1 = 73856093;
        constexpr Int prime2 = 19349663;
        const Int n = x * prime1 ^ y * prime2;

        return pseudoRandom(n);
    }
} // namespace engine::math
