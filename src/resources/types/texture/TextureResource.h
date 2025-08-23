//
// Created by Andres Guerrero on 20-08-25.
//
#pragma once

#include "TextureFormat.h"

#include <memory>
#include <vector>
#include <mutex>
#include <SDL2/SDL.h>

#include "TextureFormatUtils.h"
#include "TextureDesc.h"
#include "../../core/Resource.h"
#include "../../streaming/StreamableResource.h"

namespace engine::resources {
    /**
     * @brief Texture resource using SDL_image for safe image loading
     *
     * This class uses SDL_image for loading various image formats safely,
     * but stores the data in a renderer-agnostic format. The actual GPU
     * upload is handled by the rendering system separately.
     */
    class TextureResource final : public StreamableResource {
    public:
        // ====================================================================
        // CONSTRUCTION / DESTRUCTION
        // ====================================================================

        /**
         * @brief Constructor
         * @param id Resource ID
         * @param name Resource name
         */
        TextureResource(ResourceID id, const std::string& name);

        /**
         * @brief Destructor
         */
        ~TextureResource() override;

        // Delete copy, allow move
        TextureResource(const TextureResource&) = delete;
        TextureResource& operator=(const TextureResource&) = delete;
        TextureResource(TextureResource&&) = default;
        TextureResource& operator=(TextureResource&&) = default;

        // ====================================================================
        // RESOURCE INTERFACE
        // ====================================================================

        bool load(const std::uint8_t* data, ResourceSize size) override;
        bool unload() override;
        void reload() override;
        ResourceSize getMemoryUsage() const override;
        bool validate() const override;

        // ====================================================================
        // STREAMING INTERFACE
        // ====================================================================

        bool streamIn(std::uint32_t lodLevel) override;
        bool streamOut(std::uint32_t lodLevel) override;
        std::uint32_t getStreamedLODLevel() const override { return currentStreamedMip_; }
        std::uint32_t getMaxLODLevel() const override { return desc_.mipLevels - 1; }
        ResourceSize getStreamedMemoryUsage() const override;

        // ====================================================================
        // SERIALIZATION
        // ====================================================================

        bool serialize(std::vector<std::uint8_t>& buffer) const override;
        bool deserialize(const std::uint8_t* data, ResourceSize size) override;

        // ====================================================================
        // TEXTURE-SPECIFIC INTERFACE
        // ====================================================================

        // Getters
        const TextureDesc& getDescription() const { return desc_; }
        TextureFormat getFormat() const { return desc_.format; }
        TextureType getType() const { return desc_.type; }
        std::uint32_t getWidth(std::uint32_t mipLevel = 0) const;
        std::uint32_t getHeight(std::uint32_t mipLevel = 0) const;
        std::uint32_t getDepth(std::uint32_t mipLevel = 0) const;
        std::uint32_t getMipLevels() const { return desc_.mipLevels; }
        std::uint32_t getArraySize() const { return desc_.arraySize; }

        // Data access (renderer-agnostic)
        const std::uint8_t* getMipData(std::uint32_t mipLevel, std::uint32_t arrayIndex = 0) const;
        std::size_t getMipDataSize(std::uint32_t mipLevel) const;
        // const std::uint8_t* getCubemapFaceData(CubemapFace face, std::uint32_t mipLevel = 0) const;

        // Pixel manipulation
        bool getPixel(std::uint32_t x, std::uint32_t y, float* outColor,
                      std::uint32_t mipLevel = 0, std::uint32_t arrayIndex = 0) const;
        bool setPixel(std::uint32_t x, std::uint32_t y, const float* color,
                      std::uint32_t mipLevel = 0, std::uint32_t arrayIndex = 0);

        // Mipmap generation
        bool generateMipmaps();
        bool hasMipmaps() const { return desc_.mipLevels > 1; }

        // Format conversion
        bool convertFormat(TextureFormat newFormat);
        bool isCompressed() const { return TextureFormatUtils::isCompressed(desc_.format); }

        // Image manipulation
        bool resize(std::uint32_t newWidth, std::uint32_t newHeight);
        bool flipVertical();
        bool flipHorizontal();

        // Metadata
        void setSamplerState(const TextureSamplerState& sampler) { desc_.samplerState = sampler; }
        const TextureSamplerState& getSamplerState() const { return desc_.samplerState; }

        // GPU handle (opaque handle for renderer)
        void setGPUHandle(const std::uint64_t handle) { gpuHandle_ = handle; }
        std::uint64_t getGPUHandle() const { return gpuHandle_; }
        bool isUploadedToGPU() const { return gpuHandle_ != 0; }

        // Static type method
        static ResourceType getStaticType() { return ResourceType::TEXTURE; }

        // SDL surface access (for advanced operations)
        SDL_Surface* getSDLSurface(std::uint32_t mipLevel = 0, std::uint32_t arrayIndex = 0);
        const SDL_Surface* getSDLSurface(std::uint32_t mipLevel = 0, std::uint32_t arrayIndex = 0) const;

    protected:
        // ====================================================================
        // INTERNAL LOADING METHODS
        // ====================================================================

        bool loadFromMemory(const std::uint8_t* data, ResourceSize size);
        bool loadFromFile(const std::string& path);
        bool loadFromSDLSurface(SDL_Surface* surface);

        // Format detection and conversion
        TextureFormat detectFormatFromSDL(SDL_Surface* surface) const;
        SDL_Surface* convertToRGBA32(SDL_Surface* source) const;
        bool convertSDLToInternalFormat(SDL_Surface* surface, std::uint32_t mipLevel, std::uint32_t arrayIndex);

        // Mipmap generation
        SDL_Surface* generateMipLevel(SDL_Surface* source, std::uint32_t targetWidth, std::uint32_t targetHeight);

        // Helper methods
        void allocateMipData(std::uint32_t mipLevel, std::uint32_t arrayIndex = 0);
        void freeMipData(std::uint32_t mipLevel, std::uint32_t arrayIndex = 0);
        void freeAllMipData();
        std::size_t calculateMipIndex(std::uint32_t mipLevel, std::uint32_t arrayIndex) const;
        bool validateMipLevel(std::uint32_t mipLevel) const;
        bool validateArrayIndex(std::uint32_t arrayIndex) const;

    private:
        // Texture description
        TextureDesc desc_;

        // Mipmap data storage
        struct TextureMipData {
            std::unique_ptr<std::uint8_t[]> data;
            SDL_Surface* surface = nullptr; // Optional SDL surface for manipulation
            std::size_t size = 0;
            std::uint32_t width = 0;
            std::uint32_t height = 0;
            std::uint32_t depth = 1;
            std::uint32_t rowPitch = 0;
            std::uint32_t slicePitch = 0;
        };

        std::vector<TextureMipData> mipData_;

        // Streaming state
        std::uint32_t currentStreamedMip_ = UINT32_MAX;
        std::uint32_t requestedStreamMip_ = 0;

        // GPU handle (renderer-specific)
        std::uint64_t gpuHandle_ = 0;

        // Memory tracking
        mutable std::atomic<ResourceSize> currentMemoryUsage_{0};

        // Thread safety
        mutable std::shared_mutex dataMutex_;

        // SDL initialization tracking
        static std::atomic<bool> sdlImageInitialized_;
        static std::mutex sdlInitMutex_;

        // Initialize SDL_image if needed
        static bool ensureSDLImageInitialized();
    };
} // namespace engine::resources
