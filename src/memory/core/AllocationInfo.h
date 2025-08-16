//
// Created by Andres Guerrero on 12-08-25.
//

#pragma once

#include <chrono>
#include <string>

#include "Types.h"

namespace engine::memory {
    struct AllocationInfo {
        void* address; // Memory address
        MemorySize size; // Allocation size
        MemorySize alignment; // Alignment requirement
        MemoryCategory category; // Usage category
        AllocationFlags flags; // Allocation flags
        std::chrono::steady_clock::time_point timestamp; // When allocated
        std::string debugName; // Optional debug name
        AllocationID id; // Unique allocation ID

#ifdef _DEBUG
        std::string filename; // Source file (debug only)
        std::uint32_t line; // Source line (debug only)
        std::string function; // Function name (debug only)
#endif
    };
}
