//
// Created by Andres Guerrero on 12-08-25.
//

#pragma once

#include <array>
#include <atomic>

#include "Types.h"

namespace engine::memory {
    // Memory statistics for profiling
    struct MemoryStats {
        std::atomic<MemorySize> totalAllocated{0}; // Total bytes allocated
        std::atomic<MemorySize> totalFreed{0}; // Total bytes freed
        std::atomic<MemorySize> currentUsage{0}; // Current bytes in use
        std::atomic<MemorySize> peakUsage{0}; // Peak memory usage
        std::atomic<std::uint64_t> allocationCount{0}; // Number of allocations
        std::atomic<std::uint64_t> freeCount{0}; // Number of deallocations
        std::atomic<std::uint64_t> failedAllocations{0}; // Failed allocation attempts

        // Per-category statistics
        std::array<std::atomic<MemorySize>, static_cast<std::size_t>(MemoryCategory::COUNT)> categoryUsage{};

        // Timing statistics
        std::atomic<std::uint64_t> totalAllocationTime{0}; // Total time spent allocating (microseconds)
        std::atomic<std::uint64_t> totalFreeTime{0}; // Total time spent freeing (microseconds)

        MemoryStats() = default;
        MemoryStats(const MemoryStats&) = delete;             // prohibir copia
        MemoryStats& operator=(const MemoryStats&) = delete;  // prohibir copia

        MemoryStats(MemoryStats&&) noexcept = default;        // habilitar movimiento
        MemoryStats& operator=(MemoryStats&&) noexcept = default;
    };
}
