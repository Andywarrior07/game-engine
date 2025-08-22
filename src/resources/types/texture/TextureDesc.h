//
// Created by Andres Guerrero on 21-08-25.
//

#pragma once
#include "TextureFormat.h"
#include "TextureFormatUtils.h"

namespace engine::resources {
    // Texture filter modes
    enum class TextureFilter : std::uint8_t {
        NEAREST, // Point sampling
        LINEAR, // Bilinear filtering
        NEAREST_MIPMAP_NEAREST, // Point sampling with mipmaps
        LINEAR_MIPMAP_NEAREST, // Bilinear with point mipmap
        NEAREST_MIPMAP_LINEAR, // Point with linear mipmap
        LINEAR_MIPMAP_LINEAR, // Trilinear filtering
        ANISOTROPIC // Anisotropic filtering
    };

    // Texture wrap modes
    enum class TextureWrap : std::uint8_t {
        REPEAT, // Repeat texture
        MIRRORED_REPEAT, // Mirror and repeat
        CLAMP_TO_EDGE, // Clamp to edge pixels
        CLAMP_TO_BORDER, // Clamp to border color
        MIRROR_CLAMP_TO_EDGE // Mirror once then clamp
    };

    // Texture sampler state
    struct TextureSamplerState {
        TextureFilter minFilter = TextureFilter::LINEAR_MIPMAP_LINEAR;
        TextureFilter magFilter = TextureFilter::LINEAR;
        TextureWrap wrapU = TextureWrap::REPEAT;
        TextureWrap wrapV = TextureWrap::REPEAT;
        TextureWrap wrapW = TextureWrap::REPEAT;
        float anisotropy = 1.0f; // Anisotropic filtering level
        float lodBias = 0.0f; // LOD bias
        float minLod = -1000.0f; // Minimum LOD
        float maxLod = 1000.0f; // Maximum LOD
        float borderColor[4] = {0, 0, 0, 0}; // Border color for CLAMP_TO_BORDER
        bool compareMode = false; // Enable comparison mode (for shadows)

        bool operator==(const TextureSamplerState& other) const;
        [[nodiscard]] std::size_t hash() const;
    };

    // Texture description
    struct TextureDesc {
        TextureType type = TextureType::TEXTURE_2D;
        TextureFormat format = TextureFormat::RGBA8_UNORM;
        std::uint32_t width = 1;
        std::uint32_t height = 1;
        std::uint32_t depth = 1; // For 3D textures
        std::uint32_t arraySize = 1; // For array textures
        std::uint32_t mipLevels = 1; // Number of mip levels
        std::uint32_t sampleCount = 1; // For multisampling
        TextureUsage usage = TextureUsage::SHADER_READ;
        TextureSamplerState samplerState;

        // Calculate the required memory size
        [[nodiscard]] std::size_t calculateMemorySize() const;

        // Validate description
        [[nodiscard]] bool validate() const;
    };
} // namespace engine::resources
