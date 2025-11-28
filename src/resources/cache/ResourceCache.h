//
// Created by Andres Guerrero on 17-08-25.
//

#pragma once

#include <list>
#include <mutex>

#include "../core/Resource.h"

namespace engine::resources {
template <typename T> struct CacheEntry {
  ResourcePtr<T> resource;
  std::chrono::steady_clock::time_point lastAccessTime;
  std::size_t accessCount = 0;
  ResourceSize memorySize = 0;
};

// LRU cache for resources
template <typename T> class ResourceCache {
public:
  explicit ResourceCache(ResourceSize maxMemory = 0, std::size_t maxCount = 0)
      : maxMemory_(maxMemory), maxCount_(maxCount) {}

  // Add resource to cache
  void insert(ResourceID id, ResourcePtr<T> resource) {
    std::unique_lock lock(mutex_);

    // Remove if already exists
    auto it = cacheMap_.find(id);
    if (it != cacheMap_.end()) {
      currentMemory_ -= it->second->memorySize;
      lruList_.erase(it->second);
      cacheMap_.erase(it);
    }

    // Add to front
    lruList_.push_front({resource, std::chrono::steady_clock::now(), 0,
                         resource->getMemoryUsage()});
    cacheMap_[id] = lruList_.begin();
    currentMemory_ += resource->getMemoryUsage();

    // Evict if necesary
    while (shouldEvict()) {
      evictLRU();
    }
  }

  // Get resource from cache
  std::optional<ResourcePtr<T>> get(ResourceID id) {
    std::unique_lock lock(mutex_);

    auto it = cacheMap_.find(id);
    if (it == cacheMap_.end()) {
      ++cacheMisses_;
      return std::nullopt;
    }

    // Move to front (MRU)
    auto &entry = *it->second;
    entry.lastAccess = std::chrono::steady_clock::now();
    entry.accessCount++;
    lruList_.splice(lruList_.begin(), lruList_, it->second);

    ++cacheHits_;

    return entry.resource;
  }

  // Remove from cache
  void remove(ResourceID id) {
    std::unique_lock lock(mutex_);

    auto it = cacheMap_.find(id);
    if (it != cacheMap_.end()) {
      currentMemory_ -= it->second->memorySize;
      lruList_.erase(it->second);
      cacheMap_.erase(it);
    }
  }

  void clear() {
    std::unique_lock lock(mutex_);
    cacheMap_.clear();
    lruList_.clear();
    currentMemory_ = 0;
  }

  struct CacheStats {
    std::size_t hitCount;
    std::size_t missCount;
    float hitRate;
    ResourceSize memoryUsage;
    std::size_t resourceCount;
  };

  CacheStats getStats() const {
    std::shared_lock lock(mutex_);
    std::size_t total = cacheHits_ + cacheMisses_;

    return {cacheHits_, cacheMisses_,
            total > 0 ? static_cast<float>(cacheHits_) / total : 0.0f,
            currentMemory_, cacheMap_.size()};
  }

  // Set cache limits
  void setMaxMemory(ResourceSize requestedMaxMemory) {
    maxMemory_ = requestedMaxMemory;
  }
  void setMaxCount(std::size_t requestedMaxCount) {
    maxCount_ = requestedMaxCount;
  }

private:
  using CacheList = std::list<CacheEntry<T>>;
  using CacheMap = std::unordered_map<ResourceID, typename CacheList::iterator>;

  CacheList lruList_;
  CacheMap cacheMap_;

  ResourceSize maxMemory_ = 0;
  std::size_t maxCount_ = 0;
  ResourceSize currentMemory_ = 0;

  mutable std::shared_mutex mutex_;

  // Statistics
  mutable std::atomic<std::size_t> cacheHits_{0};
  mutable std::atomic<std::size_t> cacheMisses_{0};

  bool shouldEvict() const {
    return (maxMemory_ > 0 && currentMemory_ > maxMemory_) ||
           (maxCount_ > 0 && cacheMap_.size() > maxCount_);
  }

  void evictLRU() {
    if (lruList_.empty()) {
      return;
    }

    auto &entry = lruList_.back();
    currentMemory_ -= entry.memorySize;
    cacheMap_.erase(entry.resource->getId());
    lruList_.pop_back();
  }
};
} // namespace engine::resources
