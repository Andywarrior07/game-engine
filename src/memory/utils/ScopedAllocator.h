//
// Created by Andres Guerrero on 14-08-25.
//

#pragma once

#include "../allocators/StackAllocator.h"
#include "../core/Types.h"

namespace engine::memory {
    class StackAllocator;

    class ScopedAllocator {
    public:
        explicit ScopedAllocator(StackAllocator& allocator);
        ~ScopedAllocator();

        ScopedAllocator(const ScopedAllocator&) = delete;
        ScopedAllocator& operator=(const ScopedAllocator&) = delete;
        ScopedAllocator(ScopedAllocator&&) = delete;
        ScopedAllocator& operator=(ScopedAllocator&&) = delete;

        void* allocate(MemorySize size, MemorySize requiredAlignment = DEFAULT_ALIGNMENT) const;

    private:
        StackAllocator& allocator;
        StackAllocator::Marker marker;
    };
}
