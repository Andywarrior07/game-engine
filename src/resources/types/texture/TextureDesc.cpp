//
// Created by Andres Guerrero on 21-08-25.
//

#include "TextureDesc.h"

namespace engine::resources {
    // Implementación de TextureSamplerState::operator==
    bool TextureSamplerState::operator==(const TextureSamplerState& other) const {
        return minFilter == other.minFilter &&
            magFilter == other.magFilter &&
            wrapU == other.wrapU &&
            wrapV == other.wrapV &&
            wrapW == other.wrapW &&
            anisotropy == other.anisotropy &&
            lodBias == other.lodBias &&
            minLod == other.minLod &&
            maxLod == other.maxLod &&
            std::memcmp(borderColor, other.borderColor, sizeof(borderColor)) == 0 &&
            compareMode == other.compareMode;
    }

    // Implementación de TextureSamplerState::hash
    std::size_t TextureSamplerState::hash() const {
        std::size_t hash = 0;

        // Helper para combinar hashes
        auto hashCombine = [](std::size_t& seed, std::size_t value) {
            seed ^= value + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        };

        hashCombine(hash, static_cast<std::size_t>(minFilter));
        hashCombine(hash, static_cast<std::size_t>(magFilter));
        hashCombine(hash, static_cast<std::size_t>(wrapU));
        hashCombine(hash, static_cast<std::size_t>(wrapV));
        hashCombine(hash, static_cast<std::size_t>(wrapW));
        hashCombine(hash, std::hash<float>{}(anisotropy));
        hashCombine(hash, std::hash<float>{}(lodBias));
        hashCombine(hash, std::hash<float>{}(minLod));
        hashCombine(hash, std::hash<float>{}(maxLod));

        // Hash border color
        for (int i = 0; i < 4; ++i) {
            hashCombine(hash, std::hash<float>{}(borderColor[i]));
        }

        hashCombine(hash, std::hash<bool>{}(compareMode));

        return hash;
    }

    // Implementación de TextureDesc::calculateMemorySize
    std::size_t TextureDesc::calculateMemorySize() const {
        if (width == 0 || height == 0 || depth == 0 || arraySize == 0 || mipLevels == 0) {
            return 0;
        }

        std::size_t totalSize = 0;

        // Calcular el tamaño para cada array slice
        for (std::uint32_t arrayIndex = 0; arrayIndex < arraySize; ++arrayIndex) {
            // Calcular el tamaño para cada nivel de mip
            for (std::uint32_t mipLevel = 0; mipLevel < mipLevels; ++mipLevel) {
                std::uint32_t mipWidth = std::max(1u, width >> mipLevel);
                std::uint32_t mipHeight = std::max(1u, height >> mipLevel);
                std::uint32_t mipDepth = std::max(1u, depth >> mipLevel);

                std::size_t mipSize = 0;

                if (TextureFormatUtils::isCompressed(format)) {
                    // Para formatos comprimidos, calcular en bloques
                    std::uint32_t blockSize = TextureFormatUtils::getBlockSize(format);

                    // La mayoría de formatos comprimidos usan bloques de 4x4
                    std::uint32_t blockWidth = 4;
                    std::uint32_t blockHeight = 4;

                    // Ajustar para formatos ASTC que tienen diferentes tamaños de bloque
                    switch (format) {
                    case TextureFormat::ASTC_4x4_UNORM:
                    case TextureFormat::ASTC_4x4_SRGB:
                        blockWidth = 4;
                        blockHeight = 4;
                        break;
                    case TextureFormat::ASTC_5x5_UNORM:
                    case TextureFormat::ASTC_5x5_SRGB:
                        blockWidth = 5;
                        blockHeight = 5;
                        break;
                    case TextureFormat::ASTC_6x6_UNORM:
                    case TextureFormat::ASTC_6x6_SRGB:
                        blockWidth = 6;
                        blockHeight = 6;
                        break;
                    case TextureFormat::ASTC_8x8_UNORM:
                    case TextureFormat::ASTC_8x8_SRGB:
                        blockWidth = 8;
                        blockHeight = 8;
                        break;
                    default:
                        // Otros formatos usan 4x4
                        break;
                    }

                    // Calcular número de bloques
                    std::uint32_t blocksX = (mipWidth + blockWidth - 1) / blockWidth;
                    std::uint32_t blocksY = (mipHeight + blockHeight - 1) / blockHeight;

                    mipSize = blocksX * blocksY * mipDepth * blockSize;
                }
                else {
                    // Para formatos no comprimidos
                    std::uint32_t bytesPerPixel = TextureFormatUtils::getBytesPerPixel(format);
                    mipSize = mipWidth * mipHeight * mipDepth * bytesPerPixel;
                }

                totalSize += mipSize;
            }
        }

        // Para cubemaps, multiplicar por 6 caras (si no está ya incluido en arraySize)
        if (type == TextureType::TEXTURE_CUBE && arraySize == 1) {
            totalSize *= 6;
        }

        // Para multisampling, multiplicar por el número de samples
        if (sampleCount > 1) {
            totalSize *= sampleCount;
        }

        return totalSize;
    }

    // Implementación de TextureDesc::validate
    bool TextureDesc::validate() const {
        // Validar dimensiones básicas
        if (width == 0 || height == 0 || depth == 0) {
            return false;
        }

        // Validar array size
        if (arraySize == 0) {
            return false;
        }

        // Validar mip levels
        if (mipLevels == 0) {
            return false;
        }

        // Validar que mipLevels no exceda el máximo posible
        std::uint32_t maxPossibleMips = TextureFormatUtils::calculateMipLevels(width, height);
        if (mipLevels > maxPossibleMips) {
            return false;
        }

        // Validar sample count
        if (sampleCount == 0 || (sampleCount & (sampleCount - 1)) != 0) {
            // Sample count debe ser potencia de 2
            return false;
        }

        // Validaciones específicas por tipo de textura
        switch (type) {
        case TextureType::TEXTURE_1D:
            if (height != 1 || depth != 1) {
                return false;
            }
            if (sampleCount > 1) {
                return false; // 1D no soporta multisampling
            }
            break;

        case TextureType::TEXTURE_2D:
            if (depth != 1) {
                return false;
            }
            break;

        case TextureType::TEXTURE_3D:
            if (arraySize != 1) {
                return false; // 3D textures no pueden ser arrays
            }
            if (sampleCount > 1) {
                return false; // 3D no soporta multisampling
            }
            break;

        case TextureType::TEXTURE_CUBE:
            if (width != height) {
                return false; // Cubemaps deben ser cuadrados
            }
            if (depth != 1) {
                return false;
            }
            if (arraySize < 1) {
                return false;
            }
            if (sampleCount > 1) {
                return false; // Cubemaps no soportan multisampling
            }
            break;

        case TextureType::TEXTURE_1D_ARRAY:
            if (height != 1 || depth != 1) {
                return false;
            }
            if (sampleCount > 1) {
                return false;
            }
            break;

        case TextureType::TEXTURE_2D_ARRAY:
            if (depth != 1) {
                return false;
            }
            break;

        case TextureType::TEXTURE_CUBE_ARRAY:
            if (width != height) {
                return false;
            }
            if (depth != 1) {
                return false;
            }
            if (arraySize % 6 != 0) {
                return false; // Debe ser múltiplo de 6 (6 caras por cubo)
            }
            if (sampleCount > 1) {
                return false;
            }
            break;

        case TextureType::TEXTURE_2D_MULTISAMPLE:
        case TextureType::TEXTURE_2D_MULTISAMPLE_ARRAY:
            if (depth != 1) {
                return false;
            }
            if (sampleCount <= 1) {
                return false; // Debe tener multisampling
            }
            if (mipLevels != 1) {
                return false; // Multisampled textures no pueden tener mipmaps
            }
            break;
        }

        // Validar formato
        if (format == TextureFormat::UNKNOWN) {
            return false;
        }

        // Validar que formatos depth/stencil tengan el usage correcto
        if (TextureFormatUtils::isDepthStencil(format)) {
            if (!hasUsage(usage, TextureUsage::DEPTH_STENCIL)) {
                return false;
            }
        }

        // Validar que formatos comprimidos no se usen con ciertos tipos
        if (TextureFormatUtils::isCompressed(format)) {
            if (type == TextureType::TEXTURE_3D) {
                return false; // Muchos formatos comprimidos no soportan 3D
            }
            if (sampleCount > 1) {
                return false; // Formatos comprimidos no soportan multisampling
            }
        }

        // Validar usage flags
        if (hasUsage(usage, TextureUsage::RENDER_TARGET) &&
            hasUsage(usage, TextureUsage::DEPTH_STENCIL)) {
            return false; // No puede ser ambos al mismo tiempo
        }

        if (hasUsage(usage, TextureUsage::IMMUTABLE) &&
            (hasUsage(usage, TextureUsage::CPU_WRITE) || hasUsage(usage, TextureUsage::SHADER_WRITE))) {
            return false; // Immutable no puede ser escribible
        }

        return true;
    }
} // namespace engine::rsources
