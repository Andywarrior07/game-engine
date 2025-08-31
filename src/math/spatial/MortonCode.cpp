/**
 * @file MortonCode.cpp
 * @brief Morton code (Z-order curve) encoding and decoding
 * @author Andrés Guerrero
 * @date 27-08-2025
 *
 * Provides efficient space-filling curve algorithms for spatial indexing,
 * cache-coherent traversal, and hierarchical spatial data structures.
 */

#include "MortonCode.h"

namespace engine::math {
    UInt MortonCode::encode2D(const std::uint16_t x, const std::uint16_t y) noexcept {
        UInt morton = 0;
        morton |= expandBits2D(x);
        morton |= expandBits2D(y) << 1;

        return morton;
    }

    std::pair<std::uint16_t, std::uint16_t> MortonCode::decode2D(const UInt morton) noexcept {
        std::uint16_t x = compactBits2D(morton);
        std::uint16_t y = compactBits2D(morton >> 1);

        return {x, y};
    }

    UInt MortonCode::encode2DNormalized(Float x, Float y) noexcept {
        x = saturate(x);
        y = saturate(y);

        const auto xi = static_cast<std::uint16_t>(x * 65535.0f);
        const auto yi = static_cast<std::uint16_t>(y * 65535.0f);

        return encode2D(xi, yi);
    }

    std::uint64_t MortonCode::encode3D(UInt x, UInt y, UInt z) noexcept {
        // Limit to 21 bits per coordinate
        x &= 0x1FFFFF;
        y &= 0x1FFFFF;
        z &= 0x1FFFFF;

        std::uint64_t morton = 0;
        morton |= expandBits3D(x);
        morton |= expandBits3D(y) << 1;
        morton |= expandBits3D(z) << 2;
        return morton;
    }

    std::tuple<UInt, UInt, UInt> MortonCode::decode3D(const std::uint64_t morton) noexcept {
        UInt x = compactBits3D(morton);
        UInt y = compactBits3D(morton >> 1);
        UInt z = compactBits3D(morton >> 2);
        return {x, y, z};
    }

    std::uint64_t MortonCode::encode3DNormalized(Float x, Float y, Float z) noexcept {
        x = saturate(x);
        y = saturate(y);
        z = saturate(z);

        const UInt xi = static_cast<UInt>(x * 2097151.0f);
        const UInt yi = static_cast<UInt>(y * 2097151.0f);
        const UInt zi = static_cast<UInt>(z * 2097151.0f);
        return encode3D(xi, yi, zi);
    }

    MortonCode::Morton96 MortonCode::encode3D96(const UInt x, const UInt y, const UInt z) noexcept {
        Morton96 result;

        // Process lower 21 bits (63 bits total)
        const UInt xLow = x & 0x1FFFFF;
        const UInt yLow = y & 0x1FFFFF;
        const UInt zLow = z & 0x1FFFFF;
        result.low = encode3D(xLow, yLow, zLow);

        // Process upper 11 bits (33 bits total)
        const UInt xHigh = (x >> 21) & 0x7FF;
        const UInt yHigh = (y >> 21) & 0x7FF;
        const UInt zHigh = (z >> 21) & 0x7FF;

        std::uint64_t highPart = 0;
        highPart |= expandBits3DHigh(xHigh);
        highPart |= expandBits3DHigh(yHigh) << 1;
        highPart |= expandBits3DHigh(zHigh) << 2;

        result.high = static_cast<UInt>(highPart);

        return result;
    }

    int MortonCode::commonPrefixLength(const std::uint64_t a, const std::uint64_t b) noexcept {
        if (a == b) return 64;

        const std::uint64_t xor_val = a ^ b;
        return __builtin_clzll(xor_val); // Count leading zeros
    }

    std::array<std::uint64_t, 8> MortonCode::getChildren3D(const std::uint64_t parent) noexcept {
        std::array<std::uint64_t, 8> children{};
        const std::uint64_t base = parent << 3;

        for (int i = 0; i < 8; ++i) {
            children[i] = base | i;
        }

        return children;
    }

    std::pair<std::uint64_t, std::uint64_t> MortonCode::aabbToMortonRange(
        const AABB& aabb, const AABB& sceneBounds) noexcept {
        // Normalize AABB to scene bounds
        const Vec3 minNorm = (aabb.min - sceneBounds.min) / (sceneBounds.max - sceneBounds.min);
        const Vec3 maxNorm = (aabb.max - sceneBounds.min) / (sceneBounds.max - sceneBounds.min);

        std::uint64_t minCode = encode3DNormalized(minNorm.x, minNorm.y, minNorm.z);
        std::uint64_t maxCode = encode3DNormalized(maxNorm.x, maxNorm.y, maxNorm.z);

        return {minCode, maxCode};
    }

    UInt MortonCode::expandBits2D(const std::uint16_t v) noexcept {
        UInt x = v;
        x = (x | (x << 8)) & 0x00FF00FF;
        x = (x | (x << 4)) & 0x0F0F0F0F;
        x = (x | (x << 2)) & 0x33333333;
        x = (x | (x << 1)) & 0x55555555;

        return x;
    }

    std::uint16_t MortonCode::compactBits2D(UInt v) noexcept {
        v &= 0x55555555;
        v = (v | (v >> 1)) & 0x33333333;
        v = (v | (v >> 2)) & 0x0F0F0F0F;
        v = (v | (v >> 4)) & 0x00FF00FF;
        v = (v | (v >> 8)) & 0x0000FFFF;

        return static_cast<std::uint16_t>(v);
    }

    std::uint64_t MortonCode::expandBits3D(const UInt v) noexcept {
        std::uint64_t x = v & 0x1FFFFF; // Mask to 21 bits
        x = (x | (x << 32)) & 0x001F00000000FFFF;
        x = (x | (x << 16)) & 0x001F0000FF0000FF;
        x = (x | (x << 8)) & 0x100F00F00F00F00F;
        x = (x | (x << 4)) & 0x10C30C30C30C30C3;
        x = (x | (x << 2)) & 0x1249249249249249;

        return x;
    }

    UInt MortonCode::compactBits3D(std::uint64_t v) noexcept {
        v &= 0x1249249249249249;
        v = (v | (v >> 2)) & 0x10C30C30C30C30C3;
        v = (v | (v >> 4)) & 0x100F00F00F00F00F;
        v = (v | (v >> 8)) & 0x001F0000FF0000FF;
        v = (v | (v >> 16)) & 0x001F00000000FFFF;
        v = (v | (v >> 32)) & 0x00000000001FFFFF;

        return static_cast<UInt>(v);
    }

    std::uint64_t MortonCode::expandBits3DHigh(const UInt v) noexcept {
        std::uint64_t x = v & 0x7FF; // 11 bits

        x = (x | (x << 16)) & 0x00070000000000FF;
        x = (x | (x << 8)) & 0x0007000700070007;
        x = (x | (x << 4)) & 0x0043004300430043;
        x = (x | (x << 2)) & 0x0109010901090109;

        return x;
    }

    void MortonRadixSort::sort(std::vector<std::uint64_t>& codes) {
        if (codes.size() <= 1) return;

        std::vector<std::uint64_t> temp(codes.size());

        // Radix sort with 8-bit radix (8 passes for 64-bit)
        for (int shift = 0; shift < 64; shift += 8) {
            std::array<std::size_t, 256> histogram = {};

            // Build histogram
            for (const std::uint64_t code : codes) {
                const std::uint8_t radix = (code >> shift) & 0xFF;
                histogram[radix]++;
            }

            // Convert to offsets
            std::size_t sum = 0;
            for (std::size_t& count : histogram) {
                const std::size_t tmp = count;
                count = sum;
                sum += tmp;
            }

            // Place elements
            for (const std::uint64_t code : codes) {
                const std::uint8_t radix = (code >> shift) & 0xFF;
                temp[histogram[radix]++] = code;
            }

            codes.swap(temp);
        }
    }
} // namespace engine::math
