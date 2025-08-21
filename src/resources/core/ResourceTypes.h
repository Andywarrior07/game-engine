//
// Created by Andres Guerrero on 16-08-25.
//
// ========================================================================
// RESOURCE TYPES AND ENUMS
// ========================================================================

#pragma once

#include <cstddef>
#include <cstdint>

namespace engine::resources {
    enum class ResourceType : std::uint8_t {
        UNKNOWN = 0,
        TEXTURE,
        MESH,
        SHADER,
        MATERIAL,
        ANIMATION,
        AUDIO,
        FONT,
        SCRIPT,
        LEVEL,
        PREFAB,
        PHYSICS_MATERIAL,
        PARTICLE_SYSTEM,
        UI_LAYOUT,
        CONFIG,
        COUNT
    };

    enum class ResourcePriority : std::uint8_t {
        LOW = 0, // Background loading
        NORMAL = 1, // Standard priority
        HIGH = 2, // Load soon
        CRITICAL = 3 // Load immediately
    };

    enum class ResourceState : std::uint8_t {
        UNLOADED = 0,     // Not in memory
        LOADING,          // Currently loading
        LOADED,           // In memory, not processed
        PROCESSING,       // Being processed (e.g., decompression)
        READY,           // Ready to use
        FAILED,          // Load/process failed
        UNLOADING        // Being unloaded
    };

    enum class ResourceFlags : std::uint32_t {
        NONE = 0,
        PERSISTENT = 1 << 0,      // Don't unload automatically
        STREAMING = 1 << 1,       // Stream from disk
        COMPRESSED = 1 << 2,      // Compressed on disk
        ASYNC_LOAD = 1 << 3,      // Load asynchronously
        GPU_RESOURCE = 1 << 4,    // Requires GPU upload
        PRELOAD = 1 << 5,         // Load at startup
        SHARED = 1 << 6,          // Shared between contexts
        HOT_RELOAD = 1 << 7,      // Support hot reloading
        GENERATED = 1 << 8,       // Generated at runtime
        CACHED = 1 << 9,          // Cache processed version
        LAZY_LOAD = 1 << 10,      // Load on first use
        MEMORY_MAPPED = 1 << 11   // Use memory mapping
    };

    enum class LoadMode : std::uint8_t {
        SYNC,           // Synchronous loading
        ASYNC,          // Asynchronous loading
        STREAM,         // Streaming load
        MEMORY_MAP      // Memory mapped file
    };

    enum class CompressionType : std::uint8_t {
        NONE = 0,
        ZLIB,
        LZ4,
        ZSTD,
        CUSTOM
    };

    enum class CachePolicy : std::uint8_t {
        NO_CACHE = 0, // Never cache
        CACHE_MEMORY, // Cache in memory
        CACHE_DISK,    // Cache on disk
        CACHE_MEMORY_DISK // Cache in memory and on disk
    };

    inline ResourceFlags operator|(ResourceFlags a, ResourceFlags b) {
        return static_cast<ResourceFlags>(static_cast<std::uint32_t>(a) | static_cast<std::uint32_t>(b));
    }

    inline ResourceFlags operator&(ResourceFlags a, ResourceFlags b) {
        return static_cast<ResourceFlags>(static_cast<std::uint32_t>(a) & static_cast<std::uint32_t>(b));
    }

    inline ResourceFlags operator~(ResourceFlags a) {
        return static_cast<ResourceFlags>(~static_cast<std::uint32_t>(a));
    }

    inline bool hasFlag(ResourceFlags flags, ResourceFlags flag) {
        return (flags & flag) == flag;
    }

    using ResourceID = std::uint64_t;
    using ResourceVersion = std::uint32_t;
    using ResourceSize = std::size_t;

    constexpr ResourceID INVALID_RESOURCE_ID = 0;
}
