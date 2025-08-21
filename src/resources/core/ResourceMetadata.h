//
// Created by Andres Guerrero on 16-08-25.
//

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "ResourceTypes.h"

namespace engine::resources {
    struct ResourceDependency {
        ResourceID id;
        ResourceVersion requiredVersion;
        bool isRequired; // if false, it's optional
        bool isWeak; // Weak dependencies don't prevent unloading
    };

    struct ResourceMetadata {
        ResourceID id = INVALID_RESOURCE_ID;
        std::string name;
        std::string path;
        ResourceType type = ResourceType::UNKNOWN;
        ResourceVersion version = 0;
        ResourceSize diskSize = 0;
        ResourceSize memorySize = 0;
        std::uint32_t checksum = 0;

        std::chrono::system_clock::time_point creationTime;
        std::chrono::system_clock::time_point modificationTime;
        std::chrono::system_clock::time_point lastAccessTime;

        std::vector<ResourceDependency> dependencies;
        std::vector<ResourceID> dependents; // Resources that depend on this

        ResourcePriority priority = ResourcePriority::NORMAL;
        ResourceFlags flags = ResourceFlags::NONE;
        CompressionType compression = CompressionType::NONE;
        CachePolicy cachePolicy = CachePolicy::CACHE_MEMORY;

        std::unordered_map<std::string, std::string> properties;

        struct PlatformData {
            std::string platformPath; // Platform-specific file path
            ResourceSize platformSize;
            std::uint32_t platformChecksum;
        };

        std::unordered_map<std::string, PlatformData> platformData;
    };

    struct ResourceStats {
        std::atomic<std::size_t> totalResourceCount{0};
        std::atomic<std::size_t> loadedResourceCount{0};
        std::atomic<ResourceSize> totalMemoryUsage{0};
        std::atomic<ResourceSize> totalDiskUsage{0};
        std::atomic<std::uint64_t> totalLoadTime{0};  // Microseconds
        std::atomic<std::uint64_t> totalProcessTime{0};
        std::atomic<std::size_t> cacheHits{0};
        std::atomic<std::size_t> cacheMisses{0};
        std::atomic<std::size_t> failedLoads{0};
        std::atomic<std::size_t> hotReloads{0};

        // Per-type statistics
        std::array<std::atomic<std::size_t>, static_cast<std::size_t>(ResourceType::COUNT)> typeCount{};
        std::array<std::atomic<ResourceSize>, static_cast<std::size_t>(ResourceType::COUNT)> typeMemory{};
    };
}
