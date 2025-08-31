/**
 * @file Random.h
 * @brief High-quality random number generation for games
 * @author Andrés Guerrero
 * @date 27-08-2025
 *
 * Provides various random distributions and utilities for procedural generation,
 * gameplay mechanics, and stochastic simulations.
 */

#pragma once

#include "../core/MathTypes.h"

#include <random>
#include <chrono>
#include <array>

namespace engine::math {
    /**
     * @brief Thread-safe random number generator
     *
     * Uses Mersenne Twister for high-quality pseudo-random numbers.
     * Each thread should have its own instance for thread safety.
     */
    class Random {
    public:
        /**
         * @brief Initialize with a specific seed
         */
        explicit Random(Int seed = 0) noexcept;

        /**
         * @brief Set the random seed
         */
        void setSeed(Int seed) noexcept;

        [[nodiscard]] Int getSeed() const noexcept { return currentSeed_; }

        // ============================================================================
        // Basic Random Generation
        // ============================================================================

        /**
         * @brief Random float in range [0, 1)
         */
        [[nodiscard]] Float random() noexcept;

        /**
         * @brief Random float in range [min, max)
         */
        [[nodiscard]] Float range(Float min, Float max) noexcept;

        /**
         * @brief Random integer in range [min, max]
         */
        [[nodiscard]] Int rangeInt(Int min, Int max) noexcept;

        /**
         * @brief Random boolean with probability p of being true
         */
        [[nodiscard]] bool randomBool(Float probability = 0.5f) noexcept;

        /**
         * @brief Random sign (-1 or 1)
         */
        [[nodiscard]] Float randomSign() noexcept;

        // ============================================================================
        // Vector Random Generation
        // ============================================================================

        /**
         * @brief Random 2D vector with components in [0, 1)
         */
        [[nodiscard]] Vec2 randomVec2() noexcept;

        /**
         * @brief Random 2D vector with components in [min, max)
         */
        [[nodiscard]] Vec2 randomVec2(Float min, Float max) noexcept;

        /**
         * @brief Random 3D vector with components in [0, 1)
         */
        [[nodiscard]] Vec3 randomVec3() noexcept;

        /**
         * @brief Random 3D vector with components in [min, max)
         */
        [[nodiscard]] Vec3 randomVec3(Float min, Float max) noexcept;

        /**
         * @brief Random unit vector on 2D circle
         */
        [[nodiscard]] Vec2 randomUnitVec2() noexcept;

        /**
         * @brief Random unit vector on 3D sphere (uniform distribution)
         */
        [[nodiscard]] Vec3 randomUnitVec3() noexcept;

        /**
         * @brief Random point inside unit circle
         */
        [[nodiscard]] Vec2 randomInCircle() noexcept;

        /**
         * @brief Random point inside unit sphere
         */
        [[nodiscard]] Vec3 randomInSphere() noexcept;

        /**
         * @brief Random point in hemisphere (oriented along +Z)
         */
        [[nodiscard]] Vec3 randomInHemisphere() noexcept;

        /**
         * @brief Random quaternion (uniform distribution)
         */
        [[nodiscard]] Quat randomQuaternion() noexcept;

        // ============================================================================
        // Distributions
        // ============================================================================

        /**
         * @brief Gaussian/Normal distribution
         */
        [[nodiscard]] Float gaussian(Float mean = 0.0f, Float stddev = 1.0f) noexcept;

        /**
         * @brief Exponential distribution
         */
        [[nodiscard]] Float exponential(Float lambda = 1.0f) noexcept;

        /**
         * @brief Poisson distribution
         */
        [[nodiscard]] Int poisson(Float mean) noexcept;

        /**
         * @brief Chi-squared distribution
         */
        [[nodiscard]] Float chiSquared(Float degreesOfFreedom) noexcept;

        // ============================================================================
        // Weighted Selection
        // ============================================================================

        /**
         * @brief Select random index based on weights
         */
        [[nodiscard]] std::size_t weightedChoice(const std::vector<Float>& weights) noexcept;

        /**
         * @brief Shuffle a container in-place (Fisher-Yates)
         */
        template <typename Container>
        void shuffle(Container& container) noexcept {
            for (std::size_t i = container.size() - 1; i > 0; --i) {
                std::size_t j = rangeInt(0, static_cast<Int>(i));
                std::swap(container[i], container[j]);
            }
        }

        /**
         * @brief Select random element from container
         */
        template <typename Container>
        [[nodiscard]] auto& randomElement(Container& container) noexcept {
            std::size_t index = rangeInt(0, static_cast<Int>(container.size() - 1));
            return container[index];
        }

        // ============================================================================
        // Perlin-style Gradient Noise (simplified)
        // ============================================================================

        /**
         * @brief 1D smooth noise
         */
        [[nodiscard]] static Float noise1D(Float x) noexcept;

        /**
         * @brief 2D smooth noise
         */
        [[nodiscard]] static Float noise2D(Float x, Float y) noexcept;

    private:
        std::mt19937 generator_;
        std::uniform_real_distribution<Float> uniformReal_{0.0f, 1.0f};
        Int currentSeed_ = 0;

        /**
         * @brief Deterministic pseudo-random for noise functions
         */
        [[nodiscard]] static Float pseudoRandom(Int x) noexcept;

        [[nodiscard]] static Float pseudoRandom2D(Int x, Int y) noexcept;
    };

    // ============================================================================
    // Convenience Functions using Global Instance
    // ============================================================================

    namespace random {
        inline Float value(Random& r) { return r.random(); }
        inline Float range(Random& r, const Float min, const Float max) { return r.range(min, max); }
        inline Int rangeInt(Random& r, const Int min, const Int max) { return r.rangeInt(min, max); }
        inline bool boolean(Random& r, const Float p = 0.5f) { return r.randomBool(p); }
        inline Vec2 unitVec2(Random& r) { return r.randomUnitVec2(); }
        inline Vec3 unitVec3(Random& r) { return r.randomUnitVec3(); }
        inline Vec2 inCircle(Random& r) { return r.randomInCircle(); }
        inline Vec3 inSphere(Random& r) { return r.randomInSphere(); }
        inline Quat quaternion(Random& r) { return r.randomQuaternion(); }
    } // namespace random
} // namespace engine::math
