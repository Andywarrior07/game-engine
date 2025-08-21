//
// Created by Andres Guerrero on 17-08-25.
//

#pragma once

#include <future>
#include <string>

#include "../core/ResourceMetadata.h"
#include "../core/ResourceTypes.h"

namespace engine::resources {
    struct ResourceDataSource {
        std::string path;
        const std::uint8_t* data = nullptr;
        ResourceSize size = 0;
        bool isMemoryMapped = false;
        bool isCompressed = false;
        CompressionType compressionType = CompressionType::NONE;
    };

    struct LoadResult {
        bool success = false;
        std::unique_ptr<std::uint8_t[]> data;
        ResourceSize size = 0;
        std::string error;
        ResourceMetadata metadata;
    };

    class IResourceLoader {
    public:
        virtual ~IResourceLoader() = default;

        virtual bool canLoad(const std::string& path, ResourceType type) const = 0;

        virtual LoadResult load(const ResourceDataSource& source) = 0;

        virtual std::future<LoadResult> loadAsync(const ResourceDataSource& source) {
            return std::async(std::launch::async, [this, source]() {
                return load(source);
            });
        }

        virtual std::vector<std::string> getSupportedExtensions() const = 0;

        virtual int getPriority() const { return 0; }

        virtual bool validate(const std::uint8_t* data, ResourceSize size) const { return true; }

        virtual std::optional<ResourceMetadata> getMetadata(const ResourceDataSource& source) {
            return std::nullopt;
        }
    };

    class FileSystemLoader final : public IResourceLoader {
    public:
        bool canLoad(const std::string& path, ResourceType type) const override;
        LoadResult load(const ResourceDataSource& source) override;
        std::vector<std::string> getSupportedExtensions() const override;

    private:
        LoadResult loadFromFile(const std::string& path);
        LoadResult loadFromMemory(const std::uint8_t* data, ResourceSize size);
    };
}
