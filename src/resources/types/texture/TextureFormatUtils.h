//
// Created by Andres Guerrero on 21-08-25.
//

#pragma once

#include <string>
#include <algorithm>
#include <unordered_map>

#include "TextureFormat.h"

namespace engine::resources::TextureFormatUtils {

    inline std::uint32_t getBytesPerPixel(TextureFormat format) {
        switch (format) {
            // 8-bit formats (1 byte)
            case TextureFormat::R8_UNORM:
            case TextureFormat::R8_SNORM:
            case TextureFormat::R8_UINT:
            case TextureFormat::R8_SINT:
                return 1;

            // 16-bit formats (2 bytes)
            case TextureFormat::R16_UNORM:
            case TextureFormat::R16_SNORM:
            case TextureFormat::R16_UINT:
            case TextureFormat::R16_SINT:
            case TextureFormat::R16_FLOAT:
            case TextureFormat::RG8_UNORM:
            case TextureFormat::RG8_SNORM:
            case TextureFormat::RG8_UINT:
            case TextureFormat::RG8_SINT:
            case TextureFormat::D16_UNORM:
                return 2;

            // 32-bit formats (4 bytes)
            case TextureFormat::R32_UINT:
            case TextureFormat::R32_SINT:
            case TextureFormat::R32_FLOAT:
            case TextureFormat::RG16_UNORM:
            case TextureFormat::RG16_SNORM:
            case TextureFormat::RG16_UINT:
            case TextureFormat::RG16_SINT:
            case TextureFormat::RG16_FLOAT:
            case TextureFormat::RGBA8_UNORM:
            case TextureFormat::RGBA8_SNORM:
            case TextureFormat::RGBA8_UINT:
            case TextureFormat::RGBA8_SINT:
            case TextureFormat::RGBA8_SRGB:
            case TextureFormat::BGRA8_UNORM:
            case TextureFormat::BGRA8_SRGB:
            case TextureFormat::RGB10A2_UNORM:
            case TextureFormat::RGB10A2_UINT:
            case TextureFormat::RG11B10_FLOAT:
            case TextureFormat::RGB9E5_FLOAT:
            case TextureFormat::D24_UNORM_S8_UINT:
            case TextureFormat::D32_FLOAT:
                return 4;

            // 64-bit formats (8 bytes)
            case TextureFormat::RG32_UINT:
            case TextureFormat::RG32_SINT:
            case TextureFormat::RG32_FLOAT:
            case TextureFormat::RGBA16_UNORM:
            case TextureFormat::RGBA16_SNORM:
            case TextureFormat::RGBA16_UINT:
            case TextureFormat::RGBA16_SINT:
            case TextureFormat::RGBA16_FLOAT:
            case TextureFormat::D32_FLOAT_S8_UINT:
                return 8;

            // 128-bit formats (16 bytes)
            case TextureFormat::RGBA32_UINT:
            case TextureFormat::RGBA32_SINT:
            case TextureFormat::RGBA32_FLOAT:
                return 16;

            // Compressed formats return 0 (use getBlockSize instead)
            default:
                return 0;
        }
    }

    inline bool isCompressed(TextureFormat format) {
        switch (format) {
            // BCn, ETC/EAC y ASTC formats
            case TextureFormat::BC1_UNORM:
            case TextureFormat::BC1_SRGB:
            case TextureFormat::BC2_UNORM:
            case TextureFormat::BC2_SRGB:
            case TextureFormat::BC3_UNORM:
            case TextureFormat::BC3_SRGB:
            case TextureFormat::BC4_UNORM:
            case TextureFormat::BC4_SNORM:
            case TextureFormat::BC5_UNORM:
            case TextureFormat::BC5_SNORM:
            case TextureFormat::BC6H_UF16:
            case TextureFormat::BC6H_SF16:
            case TextureFormat::BC7_UNORM:
            case TextureFormat::BC7_SRGB:
            case TextureFormat::ETC2_RGB8_UNORM:
            case TextureFormat::ETC2_RGB8_SRGB:
            case TextureFormat::ETC2_RGBA8_UNORM:
            case TextureFormat::ETC2_RGBA8_SRGB:
            case TextureFormat::EAC_R11_UNORM:
            case TextureFormat::EAC_R11_SNORM:
            case TextureFormat::EAC_RG11_UNORM:
            case TextureFormat::EAC_RG11_SNORM:
            case TextureFormat::ASTC_4x4_UNORM:
            case TextureFormat::ASTC_4x4_SRGB:
            case TextureFormat::ASTC_5x5_UNORM:
            case TextureFormat::ASTC_5x5_SRGB:
            case TextureFormat::ASTC_6x6_UNORM:
            case TextureFormat::ASTC_6x6_SRGB:
            case TextureFormat::ASTC_8x8_UNORM:
            case TextureFormat::ASTC_8x8_SRGB:
                return true;
            default:
                return false;
        }
    }

    inline bool hasAlpha(TextureFormat format) {
        switch (format) {
            case TextureFormat::RGBA8_UNORM:
            case TextureFormat::RGBA8_SNORM:
            case TextureFormat::RGBA8_UINT:
            case TextureFormat::RGBA8_SINT:
            case TextureFormat::RGBA8_SRGB:
            case TextureFormat::BGRA8_UNORM:
            case TextureFormat::BGRA8_SRGB:
            case TextureFormat::RGB10A2_UNORM:
            case TextureFormat::RGB10A2_UINT:
            case TextureFormat::RGBA16_UNORM:
            case TextureFormat::RGBA16_SNORM:
            case TextureFormat::RGBA16_UINT:
            case TextureFormat::RGBA16_SINT:
            case TextureFormat::RGBA16_FLOAT:
            case TextureFormat::RGBA32_UINT:
            case TextureFormat::RGBA32_SINT:
            case TextureFormat::RGBA32_FLOAT:
            case TextureFormat::BC2_UNORM:
            case TextureFormat::BC2_SRGB:
            case TextureFormat::BC3_UNORM:
            case TextureFormat::BC3_SRGB:
            case TextureFormat::BC7_UNORM:
            case TextureFormat::BC7_SRGB:
            case TextureFormat::ETC2_RGBA8_UNORM:
            case TextureFormat::ETC2_RGBA8_SRGB:
            case TextureFormat::ASTC_4x4_UNORM:
            case TextureFormat::ASTC_4x4_SRGB:
            case TextureFormat::ASTC_5x5_UNORM:
            case TextureFormat::ASTC_5x5_SRGB:
            case TextureFormat::ASTC_6x6_UNORM:
            case TextureFormat::ASTC_6x6_SRGB:
            case TextureFormat::ASTC_8x8_UNORM:
            case TextureFormat::ASTC_8x8_SRGB:
                return true;
            default:
                return false;
        }
    }

    inline bool isDepthStencil(TextureFormat format) {
        switch (format) {
            case TextureFormat::D16_UNORM:
            case TextureFormat::D24_UNORM_S8_UINT:
            case TextureFormat::D32_FLOAT:
            case TextureFormat::D32_FLOAT_S8_UINT:
                return true;
            default:
                return false;
        }
    }

    inline std::uint32_t getBlockSize(TextureFormat format) {
        switch (format) {
            case TextureFormat::BC1_UNORM:
            case TextureFormat::BC1_SRGB:
            case TextureFormat::BC4_UNORM:
            case TextureFormat::BC4_SNORM:
            case TextureFormat::EAC_R11_UNORM:
            case TextureFormat::EAC_R11_SNORM:
            case TextureFormat::ETC2_RGB8_UNORM:
            case TextureFormat::ETC2_RGB8_SRGB:
                return 8;
            case TextureFormat::BC2_UNORM:
            case TextureFormat::BC2_SRGB:
            case TextureFormat::BC3_UNORM:
            case TextureFormat::BC3_SRGB:
            case TextureFormat::BC5_UNORM:
            case TextureFormat::BC5_SNORM:
            case TextureFormat::BC6H_UF16:
            case TextureFormat::BC6H_SF16:
            case TextureFormat::BC7_UNORM:
            case TextureFormat::BC7_SRGB:
            case TextureFormat::EAC_RG11_UNORM:
            case TextureFormat::EAC_RG11_SNORM:
            case TextureFormat::ETC2_RGBA8_UNORM:
            case TextureFormat::ETC2_RGBA8_SRGB:
            case TextureFormat::ASTC_4x4_UNORM:
            case TextureFormat::ASTC_4x4_SRGB:
            case TextureFormat::ASTC_5x5_UNORM:
            case TextureFormat::ASTC_5x5_SRGB:
            case TextureFormat::ASTC_6x6_UNORM:
            case TextureFormat::ASTC_6x6_SRGB:
            case TextureFormat::ASTC_8x8_UNORM:
            case TextureFormat::ASTC_8x8_SRGB:
                return 16;
            default:
                return 0;
        }
    }

    inline std::uint32_t calculateMipLevels(std::uint32_t width, std::uint32_t height) {
        if (width == 0 || height == 0) {
            return 1;
        }

        std::uint32_t maxDim = std::max(width, height);
        std::uint32_t mipLevels = 1;
        while (maxDim > 1) {
            maxDim >>= 1;
            mipLevels++;
        }
        return mipLevels;
    }

    inline void getMipDimensions(std::uint32_t mipLevel, std::uint32_t baseWidth,
                                std::uint32_t baseHeight, std::uint32_t& outWidth,
                                std::uint32_t& outHeight) {
        outWidth = std::max(1u, baseWidth >> mipLevel);
        outHeight = std::max(1u, baseHeight >> mipLevel);
    }
} // namespace engine::resources::TextureFormatUtils