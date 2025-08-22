//
// Created by Andres Guerrero on 21-08-25.
//

#pragma once
#include "../core/IAllocator.h"
#include <unordered_map>
#include <mutex>

namespace engine::memory {
    class HeapAllocator final : public IAllocator {
    public:
        explicit HeapAllocator(const MemorySize capacity, const char* name = "HeapAllocator")
            : capacity_(capacity), usedMemory_(0), name_(name) {}

        void* allocate(const MemorySize size, MemorySize alignment = DEFAULT_ALIGNMENT,
                      AllocationFlags flags = AllocationFlags::NONE) override {
            std::lock_guard<std::mutex> lock(mutex_);

            if (usedMemory_ + size > capacity_) {
                return nullptr;
            }

            // Usar malloc normal del sistema
            void* ptr = std::malloc(size);
            if (ptr) {
                allocations_[ptr] = size;
                usedMemory_ += size;
                recordAllocation(size);
            }
            return ptr;
        }

        void deallocate(void* ptr) override {
            if (!ptr) return;

            std::lock_guard<std::mutex> lock(mutex_);
            auto it = allocations_.find(ptr);
            if (it != allocations_.end()) {
                usedMemory_ -= it->second;
                recordDeallocation(it->second);
                allocations_.erase(it);
                std::free(ptr);
            }
        }

        MemorySize getCapacity() const override { return capacity_; }
        MemorySize getUsedMemory() const override { return usedMemory_; }
        bool owns(const void* ptr) const override {
            std::lock_guard<std::mutex> lock(mutex_);
            return allocations_.find(const_cast<void*>(ptr)) != allocations_.end();
        }
        MemorySize getAllocationSize(const void* ptr) const override {
            std::lock_guard<std::mutex> lock(mutex_);
            auto it = allocations_.find(const_cast<void*>(ptr));
            return (it != allocations_.end()) ? it->second : 0;
        }
        const char* getName() const override { return name_; }

    private:
        MemorySize capacity_;
        MemorySize usedMemory_;
        const char* name_;
        mutable std::mutex mutex_;
        std::unordered_map<void*, MemorySize> allocations_;
    };
} // namespace engine::memory
