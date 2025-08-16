//
// Created by Andres Guerrero on 14-08-25.
//

#include "ScopedAllocator.h"

namespace engine::memory {
    ScopedAllocator::ScopedAllocator(StackAllocator& allocator)
        : allocator(allocator)
          , marker(allocator.getMarker()) {
    }

    ScopedAllocator::~ScopedAllocator() {
        allocator.freeToMarker(marker);
    }

    void* ScopedAllocator::allocate(const MemorySize size, const MemorySize requiredAlignment) const {
        return allocator.allocate(size, requiredAlignment);
    }
}
