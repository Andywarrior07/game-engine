//
// Created by Andres Guerrero on 14-08-25.
//

#pragma once

#include <cstdint>
#include <vector>

#include "../core/Types.h"

namespace engine::memory {
    struct MemoryManagerConfig {
        // Main heap configuration
        MemorySize mainHeapSize = 512 * 1024 * 1024; // 512 MB default
        MemorySize debugHeapSize = 64 * 1024 * 1024; // 64 MB for debug

        // Per-frame allocators
        MemorySize frameStackSize = 16 * 1024 * 1024; // 16 MB per frame
        MemorySize frameLinearSize = 32 * 1024 * 1024; // 32 MB linear per frame
        std::uint8_t frameBufferCount = 3; // Triple buffering

        // Specialized allocators
        MemorySize renderingPoolSize = 128 * 1024 * 1024; // 128 MB for rendering
        MemorySize physicsPoolSize = 64 * 1024 * 1024; // 64 MB for physics
        MemorySize audioRingBufferSize = 32 * 1024 * 1024; // 32 MB for audio streaming
        MemorySize networkBufferSize = 16 * 1024 * 1024; // 16 MB for networking
        MemorySize timelineBaseSize = 64 * 1024 * 1024; // 64 MB base para timeline data
        std::size_t maxConcurrentTimelines = 4; // Máximo 4 timelines concurrentes
        bool enableTimelineTracking = true; // Habilitar tracking detallado
        MemorySize timelineReservedSize = 8 * 1024 * 1024; // 8 MB reservados por timeline

        // Pool configurations
        struct PoolConfig {
            MemorySize blockSize;
            std::size_t blockCount;
            MemoryCategory category;
        };

        std::vector<PoolConfig> customPools;

        // Profiling and debugging
        bool enableProfiling = true; // Track allocations
        bool enableMemoryTagging = true; // Tag allocations with categories
        bool enableLeakDetection = true; // Detect memory leaks
        bool enableBoundsChecking = true; // Check for buffer overruns
        bool fillFreedMemory = true; // Fill freed memory with pattern
        std::uint8_t freedMemoryPattern = 0xDE; // Pattern for freed memory
        std::uint8_t uninitializedPattern = 0xCD; // Pattern for new allocations

        // Memory limits and policies
        MemorySize lowMemoryThreshold = 100 * 1024 * 1024; // 100 MB threshold
        MemorySize criticalMemoryThreshold = 50 * 1024 * 1024; // 50 MB critical
        bool allowSystemFallback = true; // Fall back to system malloc
        bool preallocateMemory = true; // Preallocate all memory upfront
    };
}
