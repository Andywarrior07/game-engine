//
// Created by Andres Guerrero on 19-08-25.
//

#pragma once

#include <shared_mutex>
#include <string>
#include <unordered_map>

#include "../core/ResourceMetadata.h"
#include "../core/ResourceTypes.h"

namespace engine::resources {
    class ResourcePackage {
    public:
        struct PackageHeader {
            std::uint32_t magic = 0x52504B47; // "RPKG"
            std::uint32_t version = 1;
            std::uint32_t resourceCount = 0;
            std::uint64_t totalSize = 0;
            std::uint32_t compressionType = 0;
            std::uint32_t checksum = 0;
        };

        struct ResourceEntry {
            ResourceID id;
            ResourceType type;
            std::uint64_t offset;
            std::uint64_t compressedSize;
            std::uint64_t uncompressedSize;
            std::uint32_t checksum;
            std::string name;
        };

        explicit ResourcePackage(const std::string& path) : packagePath_(path) {
        }

        bool load();
        void unload();

        // Query package contents
        bool hasResource(ResourceID id) const;
        std::optional<ResourceEntry> getResourceEntry(ResourceID id) const;
        std::vector<ResourceID> getAllResourceIDs() const;

        // Extract resource data
        std::unique_ptr<std::uint8_t[]> extractResource(ResourceID id, ResourceSize& size);

        // Package creation (editor only)
        static bool createPackage(const std::string& outputPath,
                                  const std::vector<std::pair<std::string, ResourceMetadata>>& resources,
                                  CompressionType compression = CompressionType::ZSTD);

    private:
        std::string packagePath_;
        PackageHeader header_;
        std::unordered_map<ResourceID, ResourceEntry> entries_;
        std::unique_ptr<std::uint8_t[]> packageData_;
        ResourceSize packageSize_ = 0;
        bool isLoaded_ = false;
        mutable std::shared_mutex mutex_;
    };
}
