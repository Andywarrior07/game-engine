/**
 * @file MortonCode.h
 * @brief Morton code (Z-order curve) encoding and decoding
 * @author Andrés Guerrero
 * @date 27-08-2025
 *
 * Provides efficient space-filling curve algorithms for spatial indexing,
 * cache-coherent traversal, and hierarchical spatial data structures.
 */

#pragma once

#include "../core/MathTypes.h"
#include "../geometry/Primitives.h"

#include <array>
#include <algorithm>
#include <functional>
#include <ranges>

namespace engine::math {
    /**
     * @brief Morton code encoder/decoder for Z-order curves
     *
     * Morton codes interleave the bits of coordinates to create a
     * space-filling curve that preserves spatial locality.
     */
    class MortonCode {
    public:
        // ============================================================================
        // 2D Morton Codes (32-bit)
        // ============================================================================

        /**
         * @brief Encode 2D coordinates to 32-bit Morton code
         * Each coordinate uses 16 bits (0-65535 range)
         */
        [[nodiscard]] static UInt encode2D(std::uint16_t x, std::uint16_t y) noexcept;

        /**
         * @brief Decode 32-bit Morton code to 2D coordinates
         */
        [[nodiscard]] static std::pair<std::uint16_t, std::uint16_t> decode2D(UInt morton) noexcept;

        /**
         * @brief Encode normalized 2D coordinates [0,1] to Morton code
         */
        [[nodiscard]] static UInt encode2DNormalized(Float x, Float y) noexcept;

        // ============================================================================
        // 3D Morton Codes (64-bit)
        // ============================================================================

        /**
         * @brief Encode 3D coordinates to 64-bit Morton code
         * Each coordinate uses 21 bits (0-2097151 range)
         */
        [[nodiscard]] static std::uint64_t encode3D(UInt x, UInt y, UInt z) noexcept;

        /**
         * @brief Decode 64-bit Morton code to 3D coordinates
         */
        [[nodiscard]] static std::tuple<UInt, UInt, UInt> decode3D(std::uint64_t morton) noexcept;

        /**
         * @brief Encode normalized 3D coordinates [0,1] to Morton code
         */
        [[nodiscard]] static std::uint64_t encode3DNormalized(Float x, Float y, Float z) noexcept;

        // ============================================================================
        // High-Precision 3D Morton Codes (using 96 bits)
        // ============================================================================

        struct Morton96 {
            std::uint64_t low;
            UInt high;

            Morton96() : low(0), high(0) {
            }

            Morton96(const std::uint64_t l, const UInt h) : low(l), high(h) {
            }

            bool operator<(const Morton96& other) const noexcept {
                return high < other.high || (high == other.high && low < other.low);
            }

            bool operator==(const Morton96& other) const noexcept {
                return high == other.high && low == other.low;
            }
        };

        /**
         * @brief Encode 3D coordinates to 96-bit Morton code
         * Each coordinate uses 32 bits (full range)
         */
        [[nodiscard]] static Morton96 encode3D96(UInt x, UInt y, UInt z) noexcept;

        // ============================================================================
        // Utility Functions
        // ============================================================================

        /**
         * @brief Find common prefix length of two Morton codes
         * Used for building hierarchical structures
         */
        [[nodiscard]] static int commonPrefixLength(std::uint64_t a, std::uint64_t b) noexcept;

        /**
         * @brief Get parent Morton code at specific level
         */
        [[nodiscard]] static std::uint64_t getParent(const std::uint64_t morton, const int level) noexcept {
            const int shift = 3 * level; // 3 bits per level for 3D
            return (morton >> shift) << shift;
        }

        /**
         * @brief Get all children Morton codes
         */
        [[nodiscard]] static std::array<std::uint64_t, 8> getChildren3D(std::uint64_t parent) noexcept;

        /**
         * @brief Convert AABB to Morton code range
         * Returns min and max Morton codes that bound the AABB
         */
        [[nodiscard]] static std::pair<std::uint64_t, std::uint64_t> aabbToMortonRange(
            const AABB& aabb, const AABB& sceneBounds) noexcept;

        /**
         * @brief Sort objects by Morton code for cache-coherent traversal
         */
        template <typename T>
        static void sortByMortonCode(std::vector<T>& objects,
                                     std::function<Vec3(const T&)> getPosition,
                                     const AABB& sceneBounds) {
            // Calculate Morton codes
            std::vector<std::pair<std::uint64_t, std::size_t>> mortonPairs;
            mortonPairs.reserve(objects.size());

            for (std::size_t i = 0; i < objects.size(); ++i) {
                Vec3 pos = getPosition(objects[i]);
                const Vec3 norm = (pos - sceneBounds.min) / (sceneBounds.max - sceneBounds.min);
                std::uint64_t morton = encode3DNormalized(norm.x, norm.y, norm.z);
                mortonPairs.emplace_back(morton, i);
            }

            // Sort by Morton code
            std::ranges::sort(mortonPairs);

            // Reorder objects
            std::vector<T> sorted;
            sorted.reserve(objects.size());
            for (const auto& index : mortonPairs | std::views::values) {
                sorted.push_back(std::move(objects[index]));
            }
            objects = std::move(sorted);
        }

    private:
        /**
         * @brief Expand bits for 2D Morton code
         * Separates bits by one zero: b -> 0b0b0b...
         */
        [[nodiscard]] static UInt expandBits2D(std::uint16_t v) noexcept;

        /**
         * @brief Compact bits from 2D Morton code
         */
        [[nodiscard]] static std::uint16_t compactBits2D(UInt v) noexcept;

        /**
         * @brief Expand bits for 3D Morton code
         * Separates bits by two zeros: b -> 00b00b00b...
         */
        [[nodiscard]] static std::uint64_t expandBits3D(UInt v) noexcept;

        /**
         * @brief Compact bits from 3D Morton code
         */
        [[nodiscard]] static UInt compactBits3D(std::uint64_t v) noexcept;

        /**
         * @brief Expand upper bits for 96-bit Morton code
         */
        [[nodiscard]] static std::uint64_t expandBits3DHigh(UInt v) noexcept;
    };

    /**
     * @brief Radix sort for Morton codes (optimized for spatial data)
     */
    class MortonRadixSort {
    public:
        /**
         * @brief Sort array of Morton codes
         */
        static void sort(std::vector<std::uint64_t>& codes);

        /**
         * @brief Sort objects by their Morton codes
         */
        template <typename T>
        static void sortObjects(std::vector<std::pair<std::uint64_t, T>>& objects) {
            if (objects.size() <= 1) return;

            // Use standard sort for small arrays
            if (objects.size() < 1000) {
                std::sort(objects.begin(), objects.end());
                return;
            }

            // Radix sort for large arrays
            std::vector<std::pair<std::uint64_t, T>> temp(objects.size());

            for (int shift = 0; shift < 64; shift += 8) {
                std::array<std::size_t, 256> histogram = {};

                for (const auto& obj : objects) {
                    const std::uint8_t radix = (obj.first >> shift) & 0xFF;
                    ++histogram[radix];
                }

                std::size_t sum = 0;
                for (std::size_t& count : histogram) {
                    const std::size_t tmp = count;
                    count = sum;
                    sum += tmp;
                }

                for (const auto& obj : objects) {
                    const std::uint8_t radix = (obj.first >> shift) & 0xFF;
                    temp[histogram[radix]++] = obj;
                }

                objects.swap(temp);
            }
        }
    };
} // namespace engine::math
