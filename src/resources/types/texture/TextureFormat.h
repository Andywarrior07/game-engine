//
// Created by Andres Guerrero on 21-08-25.
//


#pragma once

#include <string>

namespace engine::resources {
    // Texture format enumeration
    enum class TextureFormat : std::uint32_t {
        UNKNOWN = 0,

        // 8-bit formats
        R8_UNORM,
        R8_SNORM,
        R8_UINT,
        R8_SINT,

        // 16-bit formats
        R16_UNORM,
        R16_SNORM,
        R16_UINT,
        R16_SINT,
        R16_FLOAT,
        RG8_UNORM,
        RG8_SNORM,
        RG8_UINT,
        RG8_SINT,

        // 32-bit formats
        R32_UINT,
        R32_SINT,
        R32_FLOAT,
        RG16_UNORM,
        RG16_SNORM,
        RG16_UINT,
        RG16_SINT,
        RG16_FLOAT,
        RGBA8_UNORM,
        RGBA8_SNORM,
        RGBA8_UINT,
        RGBA8_SINT,
        RGBA8_SRGB,
        BGRA8_UNORM,
        BGRA8_SRGB,
        RGB10A2_UNORM,
        RGB10A2_UINT,
        RG11B10_FLOAT,
        RGB9E5_FLOAT,

        // 64-bit formats
        RG32_UINT,
        RG32_SINT,
        RG32_FLOAT,
        RGBA16_UNORM,
        RGBA16_SNORM,
        RGBA16_UINT,
        RGBA16_SINT,
        RGBA16_FLOAT,

        // 128-bit formats
        RGBA32_UINT,
        RGBA32_SINT,
        RGBA32_FLOAT,

        // Compressed formats - BCn (DX)
        BC1_UNORM, // DXT1
        BC1_SRGB,
        BC2_UNORM, // DXT3
        BC2_SRGB,
        BC3_UNORM, // DXT5
        BC3_SRGB,
        BC4_UNORM, // RGTC1
        BC4_SNORM,
        BC5_UNORM, // RGTC2
        BC5_SNORM,
        BC6H_UF16, // BPTC float
        BC6H_SF16,
        BC7_UNORM, // BPTC
        BC7_SRGB,

        // Compressed formats - ETC/EAC (Mobile)
        ETC2_RGB8_UNORM,
        ETC2_RGB8_SRGB,
        ETC2_RGBA8_UNORM,
        ETC2_RGBA8_SRGB,
        EAC_R11_UNORM,
        EAC_R11_SNORM,
        EAC_RG11_UNORM,
        EAC_RG11_SNORM,

        // Compressed formats - ASTC (Mobile)
        ASTC_4x4_UNORM,
        ASTC_4x4_SRGB,
        ASTC_5x5_UNORM,
        ASTC_5x5_SRGB,
        ASTC_6x6_UNORM,
        ASTC_6x6_SRGB,
        ASTC_8x8_UNORM,
        ASTC_8x8_SRGB,

        // Depth/Stencil formats
        D16_UNORM,
        D24_UNORM_S8_UINT,
        D32_FLOAT,
        D32_FLOAT_S8_UINT,

        COUNT
    };

    // Texture type
    enum class TextureType : std::uint8_t {
        TEXTURE_1D,
        TEXTURE_2D,
        TEXTURE_3D,
        TEXTURE_CUBE,
        TEXTURE_1D_ARRAY,
        TEXTURE_2D_ARRAY,
        TEXTURE_CUBE_ARRAY,
        TEXTURE_2D_MULTISAMPLE,
        TEXTURE_2D_MULTISAMPLE_ARRAY
    };

    // Texture usage flags
    enum class TextureUsage : std::uint32_t {
        NONE = 0,
        SHADER_READ = 1 << 0, // Can be read in shaders
        SHADER_WRITE = 1 << 1, // Can be written in shaders
        RENDER_TARGET = 1 << 2, // Can be used as render target
        DEPTH_STENCIL = 1 << 3, // Can be used as depth/stencil
        CPU_READ = 1 << 4, // Can be read by CPU
        CPU_WRITE = 1 << 5, // Can be written by CPU
        GENERATE_MIPS = 1 << 6, // Auto-generate mipmaps
        CUBEMAP = 1 << 7, // Is a cubemap
        STREAMING = 1 << 8, // Support streaming
        IMMUTABLE = 1 << 9 // Cannot be modified after creation
    };

    // Operators for TextureUsage
    inline TextureUsage operator|(TextureUsage a, TextureUsage b) {
        return static_cast<TextureUsage>(static_cast<std::uint32_t>(a) | static_cast<std::uint32_t>(b));
    }

    inline TextureUsage operator&(TextureUsage a, TextureUsage b) {
        return static_cast<TextureUsage>(static_cast<std::uint32_t>(a) & static_cast<std::uint32_t>(b));
    }

    inline bool hasUsage(const TextureUsage usage, const TextureUsage flag) {
        return (usage & flag) == flag;
    }
} // namespace engine::resources
