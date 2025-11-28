//
// Created by Andres Guerrero on 21-08-25.
//

#pragma once

#include <cstring>
#include <mutex>
#include <unordered_map>

#ifdef __APPLE__
#include <sys/mman.h>
#endif

#include <iostream>

#include "../core/IAllocator.h"

namespace engine::memory {
class SystemAllocator : public IAllocator {
public:
  explicit SystemAllocator(MemorySize capacity,
                           const char *name = "SystemAllocator")
      : baseMemory_(nullptr), capacity_(capacity), usedMemory_(0), name_(name) {

// En macOS, usar mmap para allocaciones grandes
#ifdef __APPLE__
    // mmap es más flexible en macOS para memoria grande
    baseMemory_ = mmap(nullptr, capacity, PROT_READ | PROT_WRITE,
                       MAP_PRIVATE | MAP_ANON, -1, 0);

    if (baseMemory_ == MAP_FAILED) {
      baseMemory_ = nullptr;
      std::cerr << "[SystemAllocator] mmap failed for "
                << (capacity / (1024.0 * 1024.0)) << " MB" << std::endl;

      // Fallback a malloc para cantidades pequeñas
      if (capacity < 50 * 1024 * 1024) { // Menos de 50MB
        baseMemory_ = std::malloc(capacity);
      }
    }
#else
    // En otras plataformas, usar malloc normal
    baseMemory_ = std::malloc(capacity);
#endif

    if (!baseMemory_) {
      throw std::runtime_error("Failed to allocate system memory");
    }

    currentOffset_ = 0;
  }

  ~SystemAllocator() {
    if (baseMemory_) {
#ifdef __APPLE__
      munmap(baseMemory_, capacity_);
#else
      std::free(baseMemory_);
#endif
    }
  }

  void *allocate(MemorySize size, MemorySize alignment = DEFAULT_ALIGNMENT,
                 AllocationFlags flags = AllocationFlags::NONE) override {
    std::lock_guard<std::mutex> lock(mutex_);

    // Alinear offset actual
    MemorySize alignedOffset = alignSize(currentOffset_, alignment);

    if (alignedOffset + size > capacity_) {
      return nullptr;
    }

    void *ptr = static_cast<char *>(baseMemory_) + alignedOffset;
    allocations_[ptr] = size;
    currentOffset_ = alignedOffset + size;
    usedMemory_ += size;

    if (hasFlags(flags, AllocationFlags::ZERO_MEMORY)) {
      std::memset(ptr, 0, size);
    }

    recordAllocation(size);
    return ptr;
  }

  void deallocate(void *ptr) override {
    if (!ptr)
      return;

    std::lock_guard<std::mutex> lock(mutex_);
    auto it = allocations_.find(ptr);
    if (it != allocations_.end()) {
      usedMemory_ -= it->second;
      recordDeallocation(it->second);
      allocations_.erase(it);
      // No podemos realmente liberar en un allocator lineal
      // Solo tracking
    }
  }

  void reset() override {
    std::lock_guard<std::mutex> lock(mutex_);
    allocations_.clear();
    currentOffset_ = 0;
    usedMemory_ = 0;
  }

  MemorySize getCapacity() const override { return capacity_; }
  MemorySize getUsedMemory() const override { return usedMemory_; }

  bool owns(const void *ptr) const override {
    if (!ptr || !baseMemory_)
      return false;
    const char *charPtr = static_cast<const char *>(ptr);
    const char *basePtr = static_cast<const char *>(baseMemory_);
    return charPtr >= basePtr && charPtr < (basePtr + capacity_);
  }

  MemorySize getAllocationSize(const void *ptr) const override {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = allocations_.find(const_cast<void *>(ptr));
    return (it != allocations_.end()) ? it->second : 0;
  }

  const char *getName() const override { return name_; }

private:
  void *baseMemory_;
  MemorySize capacity_;
  MemorySize currentOffset_;
  MemorySize usedMemory_;
  const char *name_;
  mutable std::mutex mutex_;
  std::unordered_map<void *, MemorySize> allocations_;
};
} // namespace engine::memory
