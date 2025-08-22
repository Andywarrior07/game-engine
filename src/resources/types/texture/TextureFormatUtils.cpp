//
// Created by Andres Guerrero on 21-08-25.
//

#include "TextureFormatUtils.h"

#include <algorithm>
#include <unordered_map>

namespace engine::resources::TextureFormatUtils {
    // ====================================================================
    // STRING CONVERSION
    // ====================================================================

    const char* toString(const TextureFormat format) {
        static const std::unordered_map<TextureFormat, const char*> formatNames = {
            {TextureFormat::UNKNOWN, "UNKNOWN"},

            // 8-bit formats
            {TextureFormat::R8_UNORM, "R8_UNORM"},
            {TextureFormat::R8_SNORM, "R8_SNORM"},
            {TextureFormat::R8_UINT, "R8_UINT"},
            {TextureFormat::R8_SINT, "R8_SINT"},

            // 16-bit formats
            {TextureFormat::R16_UNORM, "R16_UNORM"},
            {TextureFormat::R16_SNORM, "R16_SNORM"},
            {TextureFormat::R16_UINT, "R16_UINT"},
            {TextureFormat::R16_SINT, "R16_SINT"},
            {TextureFormat::R16_FLOAT, "R16_FLOAT"},
            {TextureFormat::RG8_UNORM, "RG8_UNORM"},
            {TextureFormat::RG8_SNORM, "RG8_SNORM"},
            {TextureFormat::RG8_UINT, "RG8_UINT"},
            {TextureFormat::RG8_SINT, "RG8_SINT"},

            // 32-bit formats
            {TextureFormat::R32_UINT, "R32_UINT"},
            {TextureFormat::R32_SINT, "R32_SINT"},
            {TextureFormat::R32_FLOAT, "R32_FLOAT"},
            {TextureFormat::RG16_UNORM, "RG16_UNORM"},
            {TextureFormat::RG16_SNORM, "RG16_SNORM"},
            {TextureFormat::RG16_UINT, "RG16_UINT"},
            {TextureFormat::RG16_SINT, "RG16_SINT"},
            {TextureFormat::RG16_FLOAT, "RG16_FLOAT"},
            {TextureFormat::RGBA8_UNORM, "RGBA8_UNORM"},
            {TextureFormat::RGBA8_SNORM, "RGBA8_SNORM"},
            {TextureFormat::RGBA8_UINT, "RGBA8_UINT"},
            {TextureFormat::RGBA8_SINT, "RGBA8_SINT"},
            {TextureFormat::RGBA8_SRGB, "RGBA8_SRGB"},
            {TextureFormat::BGRA8_UNORM, "BGRA8_UNORM"},
            {TextureFormat::BGRA8_SRGB, "BGRA8_SRGB"},
            {TextureFormat::RGB10A2_UNORM, "RGB10A2_UNORM"},
            {TextureFormat::RGB10A2_UINT, "RGB10A2_UINT"},
            {TextureFormat::RG11B10_FLOAT, "RG11B10_FLOAT"},
            {TextureFormat::RGB9E5_FLOAT, "RGB9E5_FLOAT"},

            // 64-bit formats
            {TextureFormat::RG32_UINT, "RG32_UINT"},
            {TextureFormat::RG32_SINT, "RG32_SINT"},
            {TextureFormat::RG32_FLOAT, "RG32_FLOAT"},
            {TextureFormat::RGBA16_UNORM, "RGBA16_UNORM"},
            {TextureFormat::RGBA16_SNORM, "RGBA16_SNORM"},
            {TextureFormat::RGBA16_UINT, "RGBA16_UINT"},
            {TextureFormat::RGBA16_SINT, "RGBA16_SINT"},
            {TextureFormat::RGBA16_FLOAT, "RGBA16_FLOAT"},

            // 128-bit formats
            {TextureFormat::RGBA32_UINT, "RGBA32_UINT"},
            {TextureFormat::RGBA32_SINT, "RGBA32_SINT"},
            {TextureFormat::RGBA32_FLOAT, "RGBA32_FLOAT"},

            // BCn compressed formats
            {TextureFormat::BC1_UNORM, "BC1_UNORM"},
            {TextureFormat::BC1_SRGB, "BC1_SRGB"},
            {TextureFormat::BC2_UNORM, "BC2_UNORM"},
            {TextureFormat::BC2_SRGB, "BC2_SRGB"},
            {TextureFormat::BC3_UNORM, "BC3_UNORM"},
            {TextureFormat::BC3_SRGB, "BC3_SRGB"},
            {TextureFormat::BC4_UNORM, "BC4_UNORM"},
            {TextureFormat::BC4_SNORM, "BC4_SNORM"},
            {TextureFormat::BC5_UNORM, "BC5_UNORM"},
            {TextureFormat::BC5_SNORM, "BC5_SNORM"},
            {TextureFormat::BC6H_UF16, "BC6H_UF16"},
            {TextureFormat::BC6H_SF16, "BC6H_SF16"},
            {TextureFormat::BC7_UNORM, "BC7_UNORM"},
            {TextureFormat::BC7_SRGB, "BC7_SRGB"},

            // ETC/EAC compressed formats
            {TextureFormat::ETC2_RGB8_UNORM, "ETC2_RGB8_UNORM"},
            {TextureFormat::ETC2_RGB8_SRGB, "ETC2_RGB8_SRGB"},
            {TextureFormat::ETC2_RGBA8_UNORM, "ETC2_RGBA8_UNORM"},
            {TextureFormat::ETC2_RGBA8_SRGB, "ETC2_RGBA8_SRGB"},
            {TextureFormat::EAC_R11_UNORM, "EAC_R11_UNORM"},
            {TextureFormat::EAC_R11_SNORM, "EAC_R11_SNORM"},
            {TextureFormat::EAC_RG11_UNORM, "EAC_RG11_UNORM"},
            {TextureFormat::EAC_RG11_SNORM, "EAC_RG11_SNORM"},

            // ASTC compressed formats
            {TextureFormat::ASTC_4x4_UNORM, "ASTC_4x4_UNORM"},
            {TextureFormat::ASTC_4x4_SRGB, "ASTC_4x4_SRGB"},
            {TextureFormat::ASTC_5x5_UNORM, "ASTC_5x5_UNORM"},
            {TextureFormat::ASTC_5x5_SRGB, "ASTC_5x5_SRGB"},
            {TextureFormat::ASTC_6x6_UNORM, "ASTC_6x6_UNORM"},
            {TextureFormat::ASTC_6x6_SRGB, "ASTC_6x6_SRGB"},
            {TextureFormat::ASTC_8x8_UNORM, "ASTC_8x8_UNORM"},
            {TextureFormat::ASTC_8x8_SRGB, "ASTC_8x8_SRGB"},

            // Depth/Stencil formats
            {TextureFormat::D16_UNORM, "D16_UNORM"},
            {TextureFormat::D24_UNORM_S8_UINT, "D24_UNORM_S8_UINT"},
            {TextureFormat::D32_FLOAT, "D32_FLOAT"},
            {TextureFormat::D32_FLOAT_S8_UINT, "D32_FLOAT_S8_UINT"}
        };

        auto it = formatNames.find(format);
        return (it != formatNames.end()) ? it->second : "UNKNOWN";
    }

    TextureFormat fromString(const std::string& str) {
        static const std::unordered_map<std::string, TextureFormat> stringToFormat = {
            {"UNKNOWN", TextureFormat::UNKNOWN},

            // 8-bit formats
            {"R8_UNORM", TextureFormat::R8_UNORM},
            {"R8_SNORM", TextureFormat::R8_SNORM},
            {"R8_UINT", TextureFormat::R8_UINT},
            {"R8_SINT", TextureFormat::R8_SINT},

            // 16-bit formats
            {"R16_UNORM", TextureFormat::R16_UNORM},
            {"R16_SNORM", TextureFormat::R16_SNORM},
            {"R16_UINT", TextureFormat::R16_UINT},
            {"R16_SINT", TextureFormat::R16_SINT},
            {"R16_FLOAT", TextureFormat::R16_FLOAT},
            {"RG8_UNORM", TextureFormat::RG8_UNORM},
            {"RG8_SNORM", TextureFormat::RG8_SNORM},
            {"RG8_UINT", TextureFormat::RG8_UINT},
            {"RG8_SINT", TextureFormat::RG8_SINT},

            // 32-bit formats
            {"R32_UINT", TextureFormat::R32_UINT},
            {"R32_SINT", TextureFormat::R32_SINT},
            {"R32_FLOAT", TextureFormat::R32_FLOAT},
            {"RG16_UNORM", TextureFormat::RG16_UNORM},
            {"RG16_SNORM", TextureFormat::RG16_SNORM},
            {"RG16_UINT", TextureFormat::RG16_UINT},
            {"RG16_SINT", TextureFormat::RG16_SINT},
            {"RG16_FLOAT", TextureFormat::RG16_FLOAT},
            {"RGBA8_UNORM", TextureFormat::RGBA8_UNORM},
            {"RGBA8_SNORM", TextureFormat::RGBA8_SNORM},
            {"RGBA8_UINT", TextureFormat::RGBA8_UINT},
            {"RGBA8_SINT", TextureFormat::RGBA8_SINT},
            {"RGBA8_SRGB", TextureFormat::RGBA8_SRGB},
            {"BGRA8_UNORM", TextureFormat::BGRA8_UNORM},
            {"BGRA8_SRGB", TextureFormat::BGRA8_SRGB},
            {"RGB10A2_UNORM", TextureFormat::RGB10A2_UNORM},
            {"RGB10A2_UINT", TextureFormat::RGB10A2_UINT},
            {"RG11B10_FLOAT", TextureFormat::RG11B10_FLOAT},
            {"RGB9E5_FLOAT", TextureFormat::RGB9E5_FLOAT},

            // 64-bit formats
            {"RG32_UINT", TextureFormat::RG32_UINT},
            {"RG32_SINT", TextureFormat::RG32_SINT},
            {"RG32_FLOAT", TextureFormat::RG32_FLOAT},
            {"RGBA16_UNORM", TextureFormat::RGBA16_UNORM},
            {"RGBA16_SNORM", TextureFormat::RGBA16_SNORM},
            {"RGBA16_UINT", TextureFormat::RGBA16_UINT},
            {"RGBA16_SINT", TextureFormat::RGBA16_SINT},
            {"RGBA16_FLOAT", TextureFormat::RGBA16_FLOAT},

            // 128-bit formats
            {"RGBA32_UINT", TextureFormat::RGBA32_UINT},
            {"RGBA32_SINT", TextureFormat::RGBA32_SINT},
            {"RGBA32_FLOAT", TextureFormat::RGBA32_FLOAT},

            // BCn compressed formats
            {"BC1_UNORM", TextureFormat::BC1_UNORM},
            {"BC1_SRGB", TextureFormat::BC1_SRGB},
            {"BC2_UNORM", TextureFormat::BC2_UNORM},
            {"BC2_SRGB", TextureFormat::BC2_SRGB},
            {"BC3_UNORM", TextureFormat::BC3_UNORM},
            {"BC3_SRGB", TextureFormat::BC3_SRGB},
            {"BC4_UNORM", TextureFormat::BC4_UNORM},
            {"BC4_SNORM", TextureFormat::BC4_SNORM},
            {"BC5_UNORM", TextureFormat::BC5_UNORM},
            {"BC5_SNORM", TextureFormat::BC5_SNORM},
            {"BC6H_UF16", TextureFormat::BC6H_UF16},
            {"BC6H_SF16", TextureFormat::BC6H_SF16},
            {"BC7_UNORM", TextureFormat::BC7_UNORM},
            {"BC7_SRGB", TextureFormat::BC7_SRGB},

            // ETC/EAC compressed formats
            {"ETC2_RGB8_UNORM", TextureFormat::ETC2_RGB8_UNORM},
            {"ETC2_RGB8_SRGB", TextureFormat::ETC2_RGB8_SRGB},
            {"ETC2_RGBA8_UNORM", TextureFormat::ETC2_RGBA8_UNORM},
            {"ETC2_RGBA8_SRGB", TextureFormat::ETC2_RGBA8_SRGB},
            {"EAC_R11_UNORM", TextureFormat::EAC_R11_UNORM},
            {"EAC_R11_SNORM", TextureFormat::EAC_R11_SNORM},
            {"EAC_RG11_UNORM", TextureFormat::EAC_RG11_UNORM},
            {"EAC_RG11_SNORM", TextureFormat::EAC_RG11_SNORM},

            // ASTC compressed formats
            {"ASTC_4x4_UNORM", TextureFormat::ASTC_4x4_UNORM},
            {"ASTC_4x4_SRGB", TextureFormat::ASTC_4x4_SRGB},
            {"ASTC_5x5_UNORM", TextureFormat::ASTC_5x5_UNORM},
            {"ASTC_5x5_SRGB", TextureFormat::ASTC_5x5_SRGB},
            {"ASTC_6x6_UNORM", TextureFormat::ASTC_6x6_UNORM},
            {"ASTC_6x6_SRGB", TextureFormat::ASTC_6x6_SRGB},
            {"ASTC_8x8_UNORM", TextureFormat::ASTC_8x8_UNORM},
            {"ASTC_8x8_SRGB", TextureFormat::ASTC_8x8_SRGB},

            // Depth/Stencil formats
            {"D16_UNORM", TextureFormat::D16_UNORM},
            {"D24_UNORM_S8_UINT", TextureFormat::D24_UNORM_S8_UINT},
            {"D32_FLOAT", TextureFormat::D32_FLOAT},
            {"D32_FLOAT_S8_UINT", TextureFormat::D32_FLOAT_S8_UINT}
        };

        auto it = stringToFormat.find(str);
        return (it != stringToFormat.end()) ? it->second : TextureFormat::UNKNOWN;
    }
} // namespace engine::resources::TextureFormatUtils