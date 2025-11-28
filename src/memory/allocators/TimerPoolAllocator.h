//
// Created by Andres Guerrero on 15-08-25.
//

#pragma once

#include "../allocators/PoolAllocator.h"

#include <array>
#include <atomic>
#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

namespace engine::memory {
/**
 * @brief Specialized pool allocator optimized for Timer objects
 * @details High-performance pool allocator specifically designed for timer
 * management systems. Provides segregated pools for different timer types,
 * batch allocation/deallocation, and optimized memory layouts for cache
 * efficiency.
 *
 * Key features:
 * - Type-specific pools with optimal block sizes
 * - Batch allocation for creating many timers at once
 * - Zero-fragmentation timer lifecycle management
 * - Thread-safe operations with lock-free paths
 * - Automatic pool sizing based on usage patterns
 *
 * Performance characteristics:
 * - O(1) allocation/deallocation
 * - Cache-friendly memory layout
 * - Minimal memory overhead per timer
 * - Lock-free for common operations
 *
 * Ideal for:
 * - Game timer systems (thousands of active timers)
 * - Animation timers with different durations
 * - Recurring event scheduling
 * - Performance-critical timing code
 */
class TimerPoolAllocator final : public IAllocator {
public:
  /**
   * @brief Timer type categories for pool segregation
   */
  enum class TimerType : std::uint8_t {
    ONE_SHOT,  ///< Single-execution timers
    RECURRING, ///< Repeating timers
    ANIMATION, ///< Animation frame timers
    SYSTEM,    ///< System/engine timers
    GAMEPLAY,  ///< Game logic timers
    COUNT      ///< Number of timer types
  };

  /**
   * @brief Pool configuration for each timer type
   */
  struct PoolConfig {
    MemorySize blockSize;     ///< Size of each timer block
    std::size_t initialCount; ///< Initial number of blocks
    std::size_t maxCount;     ///< Maximum blocks (0 = unlimited)
    bool allowGrowth;         ///< Allow pool to grow beyond initial size
    float growthFactor;       ///< Growth factor when expanding (e.g., 1.5f)
  };

  /**
   * @brief Default pool configurations for different timer types
   */
  static constexpr std::array<PoolConfig,
                              static_cast<std::size_t>(TimerType::COUNT)>
      DEFAULT_CONFIGS = {{// ONE_SHOT: Small, frequent allocations
                          {64, 1000, 10000, true, 1.5f},
                          // RECURRING: Medium-sized, persistent timers
                          {128, 500, 5000, true, 1.25f},
                          // ANIMATION: High-frequency, small timers
                          {32, 2000, 20000, true, 2.0f},
                          // SYSTEM: Large, infrequent system timers
                          {256, 100, 1000, true, 1.2f},
                          // GAMEPLAY: Variable size game timers
                          {96, 800, 8000, true, 1.4f}}};

  /**
   * @brief Construct timer pool allocator with default configuration
   * @param name Debug name for the allocator
   */
  explicit TimerPoolAllocator(const char *name = "TimerPoolAllocator")
      : TimerPoolAllocator(DEFAULT_CONFIGS, name) {}

  /**
   * @brief Construct timer pool allocator with custom configuration
   * @param configs Pool configurations for each timer type
   * @param name Debug name for the allocator
   */
  explicit TimerPoolAllocator(
      const std::array<PoolConfig, static_cast<std::size_t>(TimerType::COUNT)>
          &configs,
      const char *name = "TimerPoolAllocator");

  /**
   * @brief Destructor
   */
  ~TimerPoolAllocator() override;

  // Disable copy semantics
  TimerPoolAllocator(const TimerPoolAllocator &) = delete;
  TimerPoolAllocator &operator=(const TimerPoolAllocator &) = delete;

  // Move semantics
  TimerPoolAllocator(TimerPoolAllocator &&other) noexcept
      : configs_(other.configs_), pools_(std::move(other.pools_)),
        name_(other.name_), totalCapacity_(other.totalCapacity_),
        totalUsed_(other.totalUsed_.load()) {
    other.totalCapacity_ = 0;
    other.totalUsed_ = 0;
  }

  TimerPoolAllocator &operator=(TimerPoolAllocator &&other) noexcept;

  /**
   * @brief Allocate memory from appropriate timer pool
   * @param size Size of allocation (determines pool selection)
   * @param alignment Memory alignment requirement
   * @param flags Allocation flags
   * @return Pointer to allocated memory or nullptr on failure
   */
  void *allocate(MemorySize size, MemorySize alignment,
                 AllocationFlags flags = AllocationFlags::NONE) override;

  /**
   * @brief Allocate timer of specific type
   * @param timerType Type of timer to allocate
   * @param flags Allocation flags
   * @return Pointer to allocated timer memory
   */
  void *allocateTimer(TimerType timerType,
                      AllocationFlags flags = AllocationFlags::NONE);

  /**
   * @brief Batch allocate multiple timers of the same type
   * @param timerType Type of timers to allocate
   * @param count Number of timers to allocate
   * @param flags Allocation flags
   * @return Vector of allocated timer pointers (empty on failure)
   */
  std::vector<void *>
  batchAllocateTimers(TimerType timerType, std::size_t count,
                      AllocationFlags flags = AllocationFlags::NONE);

  /**
   * @brief Deallocate timer memory
   * @param ptr Pointer to deallocate
   */
  void deallocate(void *ptr) override;

  /**
   * @brief Batch deallocate multiple timers
   * @param ptrs Vector of pointers to deallocate
   */
  void batchDeallocate(const std::vector<void *> &ptrs);

  /**
   * @brief Get total capacity of all pools
   */
  MemorySize getCapacity() const override { return totalCapacity_; }

  /**
   * @brief Get currently used memory across all pools
   */
  MemorySize getUsedMemory() const override {
    return totalUsed_.load(std::memory_order_acquire);
  }

  /**
   * @brief Reset all pools
   */
  void reset() override;

  /**
   * @brief Check if any pool owns the pointer
   */
  bool owns(const void *ptr) const override;

  /**
   * @brief Get allocation size for a pointer
   */
  MemorySize getAllocationSize(const void *ptr) const override;

  /**
   * @brief Get allocator name
   */
  const char *getName() const override { return name_; }

  // Timer pool specific methods

  /**
   * @brief Get pool for specific timer type
   * @param timerType Timer type
   * @return Pointer to pool or nullptr if invalid type
   */
  PoolAllocator *getPool(TimerType timerType) const {
    if (timerType >= TimerType::COUNT)
      return nullptr;

    return pools_[static_cast<std::size_t>(timerType)].get();
  }

  /**
   * @brief Get pool statistics for specific timer type
   * @param timerType Timer type
   * @return Pool statistics
   */
  MemoryStats &getPoolStats(TimerType timerType) const;

  /**
   * @brief Get utilization for specific timer type
   * @param timerType Timer type
   * @return Utilization ratio (0.0 to 1.0)
   */
  float getPoolUtilization(TimerType timerType) const;

  /**
   * @brief Get number of free timers in specific pool
   * @param timerType Timer type
   * @return Number of free timer slots
   */
  std::size_t getFreeTimerCount(TimerType timerType) const;

  /**
   * @brief Get total number of timers in specific pool
   * @param timerType Timer type
   * @return Total timer capacity
   */
  std::size_t getTotalTimerCount(TimerType timerType) const;

  /**
   * @brief Check if specific pool is full
   * @param timerType Timer type
   * @return True if pool is full
   */
  bool isPoolFull(TimerType timerType) const;

  /**
   * @brief Attempt to grow a specific pool
   * @param timerType Timer type to grow
   * @return True if pool was successfully grown
   */
  bool growPool(TimerType timerType);

  /**
   * @brief Generate detailed memory report for all pools
   * @return Formatted memory report string
   */
  std::string generateDetailedReport() const;

  /**
   * @brief Get overall utilization across all pools
   * @return Overall utilization ratio (0.0 to 1.0)
   */
  float getUtilization() const;

private:
  std::array<PoolConfig, static_cast<std::size_t>(TimerType::COUNT)>
      configs_; ///< Pool configurations
  std::array<std::unique_ptr<PoolAllocator>,
             static_cast<std::size_t>(TimerType::COUNT)>
      pools_;                         ///< Timer pools
  const char *name_;                  ///< Debug name
  MemorySize totalCapacity_;          ///< Total capacity across all pools
  std::atomic<MemorySize> totalUsed_; ///< Total used memory

  /**
   * @brief Allocate from specific pool with error handling
   * @param timerType Pool to allocate from
   * @param flags Allocation flags
   * @return Allocated pointer or nullptr
   */
  void *allocateFromPool(TimerType timerType, AllocationFlags flags);

  /**
   * @brief Select timer type based on allocation size
   * @param size Requested allocation size
   * @return Most appropriate timer type
   */
  TimerType selectTimerTypeBySize(MemorySize size) const;

  /**
   * @brief Get human-readable name for timer type
   * @param timerType Timer type
   * @return Timer type name string
   */
  static const char *getTimerTypeName(TimerType timerType);
};
} // namespace engine::memory
