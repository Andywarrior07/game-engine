//
// Created by Andres Guerrero on 12-08-25.
//

#pragma once

namespace engine::memory {
    // Type aliases
    // Memory size type aliases for clarity
    using MemorySize = std::size_t;
    using MemoryOffset = std::ptrdiff_t;
    using AllocationID = std::uint64_t;

    // Constants
    // Platform-specific cache line size (typical x86-64)
    constexpr std::size_t CACHE_LINE_SIZE = 64;
    // Default memory alignment for SIMD operations
    constexpr std::size_t DEFAULT_ALIGNMENT = 16;

    // Enums
    enum class AllocationFlags : std::uint32_t {
        NONE           = 0, ZERO_MEMORY = 1 << 0, // Zero-initialize allocated memory
        PERSISTENT     = 1 << 1, // Memory persists across levels
        TEMPORARY      = 1 << 2, // Short-lived allocation
        GPU_ACCESSIBLE = 1 << 3, // Memory accessible by GPU
        CPU_CACHED     = 1 << 4, // Optimize for CPU cache
        THREAD_LOCAL   = 1 << 5, // Thread-local allocation
        DEBUG_FILL     = 1 << 6, // Fill with debug pattern
        NO_CACHE       = 1 << 7, // Bypass CPU cache (streaming)
        EXECUTABLE     = 1 << 8 // Memory can contain executable code
    };

    enum class MemoryCategory : std::uint8_t {
        GENERAL, // General purpose allocations
        RENDERING, // Graphics and rendering data
        PHYSICS, // Physics simulation data
        AUDIO, // Audio buffers and data
        GAMEPLAY, // Game logic and state
        NETWORKING, // Network buffers
        SCRIPTING, // Script execution memory
        UI, // User interface
        WORLD, // World/level data
        ANIMATION, // Animation data
        PARTICLES, // Particle systems
        AI, // AI and pathfinding
        RESOURCES, // Resource loading
        TIMELINE, // Timeline
        DEBUG, // Debug and development
        COUNT // Number of categories. Allways at the end
    };

    // Inline operators for flags
    // Bitwise operators for allocation flags
    inline AllocationFlags operator|(AllocationFlags a, AllocationFlags b) {
        return static_cast<AllocationFlags>(static_cast<std::uint32_t>(a) | static_cast<std::uint32_t>(b));
    }

    inline AllocationFlags operator&(AllocationFlags a, AllocationFlags b) {
        return static_cast<AllocationFlags>(static_cast<std::uint32_t>(a) & static_cast<std::uint32_t>(b));
    }

    inline bool hasFlags(const AllocationFlags flags, const AllocationFlags flag) {
        return (flags & flag) == flag;
    }
}
