//
// Created by Andres Guerrero on 19-09-25.
//

#include "TimerPoolAllocator.h"

namespace engine::memory {
    TimerPoolAllocator::TimerPoolAllocator(
        const std::array<PoolConfig, static_cast<std::size_t>(TimerType::COUNT)>& configs,
        const char* name) : configs_(configs)
                            , name_(name)
                            , totalCapacity_(0)
                            , totalUsed_(0) {
        // Initialize pools for each timer type
        for (std::size_t i = 0; i < static_cast<std::size_t>(TimerType::COUNT); ++i) {
            const auto& config = configs_[i];

            try {
                pools_[i] = std::make_unique<PoolAllocator>(
                    config.blockSize,
                    config.initialCount,
                    DEFAULT_ALIGNMENT,
                    (std::string(name_) + "_" + getTimerTypeName(static_cast<TimerType>(i))).c_str()
                );

                totalCapacity_ += config.blockSize * config.initialCount;
            }
            catch (const std::exception& e) {
                std::cerr << "[TimerPoolAllocator] Failed to create pool for type " << i
                    << ": " << e.what() << std::endl;

                // Clean up already created pools
                for (std::size_t j = 0; j < i; ++j) {
                    pools_[j].reset();
                }
                throw;
            }
        }

        // Initialize statistics
        stats_.currentUsage = 0;
        stats_.peakUsage = 0;
        stats_.totalAllocated = 0;
        stats_.allocationCount = 0;

        std::cout << "[TimerPoolAllocator] Initialized with "
            << (static_cast<double>(totalCapacity_) / (1024.0 * 1024.0)) << " MB capacity" << std::endl;
    }

    TimerPoolAllocator::~TimerPoolAllocator() {
#ifdef _DEBUG
        // Check for leaks in each pool
        for (std::size_t i = 0; i < static_cast<std::size_t>(TimerType::COUNT); ++i) {
            if (pools_[i] && pools_[i]->getUsedMemory() > 0) {
                std::cerr << "Warning: TimerPoolAllocator pool " << getTimerTypeName(static_cast<TimerType>(i))
                    << " has " << pools_[i]->getUsedMemory() << " bytes still allocated!" << std::endl;
            }
        }
#endif
    }

    TimerPoolAllocator& TimerPoolAllocator::operator=(TimerPoolAllocator&& other) noexcept {
        if (this != &other) {
            configs_ = other.configs_;
            pools_ = std::move(other.pools_);
            name_ = other.name_;
            totalCapacity_ = other.totalCapacity_;
            totalUsed_ = other.totalUsed_.load();

            other.totalCapacity_ = 0;
            other.totalUsed_ = 0;
        }
        return *this;
    }

    // TODO: Revisar parametro no usado
    void* TimerPoolAllocator::allocate(const MemorySize size, MemorySize alignment, const AllocationFlags flags) {
        // Select appropriate pool based on size
        const TimerType timerType = selectTimerTypeBySize(size);
        return allocateFromPool(timerType, flags);
    }

    void* TimerPoolAllocator::allocateTimer(const TimerType timerType, const AllocationFlags flags) {
        return allocateFromPool(timerType, flags);
    }

    std::vector<void*> TimerPoolAllocator::batchAllocateTimers(const TimerType timerType, const std::size_t count,
                                                               const AllocationFlags flags) {
        std::vector<void*> allocations;
        allocations.reserve(count);

        // Attempt to allocate all timers
        for (std::size_t i = 0; i < count; ++i) {
            void* ptr = allocateFromPool(timerType, flags);
            if (!ptr) {
                // Failed to allocate - clean up already allocated timers
                for (void* allocated : allocations) {
                    deallocate(allocated);
                }
                allocations.clear();
                break;
            }
            allocations.push_back(ptr);
        }

        return allocations;
    }

    void TimerPoolAllocator::deallocate(void* ptr) {
        if (!ptr) return;

        // Find which pool owns this pointer
        for (const auto& pool : pools_) {
            if (pool && pool->owns(ptr)) {
                const MemorySize blockSize = pool->getAllocationSize(ptr);
                pool->deallocate(ptr);

                totalUsed_.fetch_sub(blockSize, std::memory_order_relaxed);
                recordDeallocation(blockSize);
                return;
            }
        }

        // Pointer not found in any pool
#ifdef _DEBUG
        std::cerr << "Error: Attempted to deallocate pointer not owned by TimerPoolAllocator!" << std::endl;
        assert(false && "Invalid deallocation");
#endif
    }

    void TimerPoolAllocator::batchDeallocate(const std::vector<void*>& ptrs) {
        for (void* ptr : ptrs) {
            deallocate(ptr);
        }
    }

    void TimerPoolAllocator::reset() {
        for (auto& pool : pools_) {
            if (pool) {
                pool->reset();
            }
        }

        totalUsed_.store(0, std::memory_order_release);
        stats_.currentUsage.store(0, std::memory_order_release);
    }

    bool TimerPoolAllocator::owns(const void* ptr) const {
        for (const auto& pool : pools_) {
            if (pool && pool->owns(ptr)) {
                return true;
            }
        }
        return false;
    }

    MemorySize TimerPoolAllocator::getAllocationSize(const void* ptr) const {
        for (const auto& pool : pools_) {
            if (pool && pool->owns(ptr)) {
                return pool->getAllocationSize(ptr);
            }
        }
        return 0;
    }

    MemoryStats& TimerPoolAllocator::getPoolStats(TimerType timerType) const {
        static MemoryStats dummy;

        if (timerType >= TimerType::COUNT) {
            return dummy;
        }

        if (const auto& pool = pools_[static_cast<std::size_t>(timerType)]) {
            return const_cast<MemoryStats&>(pool->getStats());
        }

        return dummy;
    }

    float TimerPoolAllocator::getPoolUtilization(TimerType timerType) const {
        if (timerType >= TimerType::COUNT) return 0.0f;

        const auto& pool = pools_[static_cast<std::size_t>(timerType)];
        if (!pool) return 0.0f;

        const MemorySize capacity = pool->getCapacity();
        if (capacity == 0) return 0.0f;

        return static_cast<float>(pool->getUsedMemory()) / static_cast<float>(capacity);
    }

    std::size_t TimerPoolAllocator::getFreeTimerCount(TimerType timerType) const {
        if (timerType >= TimerType::COUNT) return 0;

        const auto& pool = pools_[static_cast<std::size_t>(timerType)];
        return pool ? pool->getFreeBlockCount() : 0;
    }

    std::size_t TimerPoolAllocator::getTotalTimerCount(TimerType timerType) const {
        if (timerType >= TimerType::COUNT) return 0;

        const auto& pool = pools_[static_cast<std::size_t>(timerType)];
        return pool ? pool->getTotalBlockCount() : 0;
    }

    bool TimerPoolAllocator::isPoolFull(TimerType timerType) const {
        if (timerType >= TimerType::COUNT) return true;

        const auto& pool = pools_[static_cast<std::size_t>(timerType)];
        return pool ? pool->isFull() : true;
    }

    bool TimerPoolAllocator::growPool(TimerType timerType) {
        if (timerType >= TimerType::COUNT) return false;

        const auto& config = configs_[static_cast<std::size_t>(timerType)];
        if (!config.allowGrowth) return false;

        auto& pool = pools_[static_cast<std::size_t>(timerType)];
        if (!pool) return false;

        const std::size_t currentCount = pool->getTotalBlockCount();
        std::size_t newCount = currentCount * static_cast<std::size_t>(config.growthFactor);

        // Respect maximum count limit
        if (config.maxCount > 0 && newCount > config.maxCount) {
            newCount = config.maxCount;
        }

        if (newCount <= currentCount) return false; // Can't grow

        try {
            // Create new larger pool
            auto newPool = std::make_unique<PoolAllocator>(
                config.blockSize,
                newCount,
                DEFAULT_ALIGNMENT,
                (std::string(name_) + "_" + getTimerTypeName(timerType) + "_grown").c_str()
            );

            // Update capacity tracking
            const MemorySize additionalCapacity = config.blockSize * (newCount - currentCount);
            totalCapacity_ += additionalCapacity;

            // Replace old pool (this will automatically free unused memory)
            pool = std::move(newPool);

            return true;
        }
        catch (const std::exception& e) {
            std::cerr << "[TimerPoolAllocator] Failed to grow pool for type "
                << getTimerTypeName(timerType) << ": " << e.what() << std::endl;
            return false;
        }
    }

    std::string TimerPoolAllocator::generateDetailedReport() const {
        std::stringstream report;

        report << "=== TimerPoolAllocator Report ===" << std::endl;
        report << "Total Capacity: " << (static_cast<double>(getCapacity()) / (1024.0 * 1024.0)) << " MB" << std::endl;
        report << "Total Used: " << (static_cast<double>(getUsedMemory()) / (1024.0 * 1024.0)) << " MB" << std::endl;
        report << "Overall Utilization: " << (getUtilization() * 100.0f) << "%" << std::endl;
        report << std::endl;

        for (std::size_t i = 0; i < static_cast<std::size_t>(TimerType::COUNT); ++i) {
            const auto type = static_cast<TimerType>(i);
            const auto& pool = pools_[i];

            if (!pool) continue;

            report << getTimerTypeName(type) << " Pool:" << std::endl;
            report << "  Block Size: " << configs_[i].blockSize << " bytes" << std::endl;
            report << "  Total Blocks: " << pool->getTotalBlockCount() << std::endl;
            report << "  Used Blocks: " << (pool->getTotalBlockCount() - pool->getFreeBlockCount()) << std::endl;
            report << "  Free Blocks: " << pool->getFreeBlockCount() << std::endl;
            report << "  Utilization: " << (getPoolUtilization(type) * 100.0f) << "%" << std::endl;
            report << "  Memory: " << (static_cast<double>(pool->getUsedMemory()) / 1024.0) << " / "
                << (static_cast<double>(pool->getCapacity()) / 1024.0) << " KB" << std::endl;
            report << std::endl;
        }

        return report.str();
    }

    float TimerPoolAllocator::getUtilization() const {
        const MemorySize totalCap = getCapacity();
        if (totalCap == 0) return 0.0f;

        return static_cast<float>(getUsedMemory()) / static_cast<float>(totalCap);
    }

    void* TimerPoolAllocator::allocateFromPool(TimerType timerType, const AllocationFlags flags) {
        if (timerType >= TimerType::COUNT) {
            stats_.failedAllocations.fetch_add(1, std::memory_order_relaxed);
            return nullptr;
        }

        const auto& pool = pools_[static_cast<std::size_t>(timerType)];
        if (!pool) {
            stats_.failedAllocations.fetch_add(1, std::memory_order_relaxed);
            return nullptr;
        }

        // Try to allocate from pool
        const MemorySize blockSize = configs_[static_cast<std::size_t>(timerType)].blockSize;
        void* ptr = pool->allocate(blockSize, DEFAULT_ALIGNMENT, flags);

        if (!ptr) {
            // Pool is full - try to grow if allowed
            if (growPool(timerType)) {
                ptr = pool->allocate(blockSize, DEFAULT_ALIGNMENT, flags);
            }

            if (!ptr) {
                stats_.failedAllocations.fetch_add(1, std::memory_order_relaxed);
                return nullptr;
            }
        }

        totalUsed_.fetch_add(blockSize, std::memory_order_relaxed);
        recordAllocation(blockSize);

        return ptr;
    }

    TimerPoolAllocator::TimerType TimerPoolAllocator::selectTimerTypeBySize(const MemorySize size) const {
        // Find the smallest pool that can accommodate the size
        auto bestType = TimerType::ONE_SHOT;
        MemorySize bestFit = configs_[static_cast<std::size_t>(TimerType::ONE_SHOT)].blockSize;

        for (std::size_t i = 0; i < static_cast<std::size_t>(TimerType::COUNT); ++i) {
            if (const MemorySize blockSize = configs_[i].blockSize; blockSize >= size && blockSize < bestFit) {
                bestFit = blockSize;
                bestType = static_cast<TimerType>(i);
            }
        }

        return bestType;
    }

    const char* TimerPoolAllocator::getTimerTypeName(const TimerType timerType) {
        switch (timerType) {
        case TimerType::ONE_SHOT: return "ONE_SHOT";
        case TimerType::RECURRING: return "RECURRING";
        case TimerType::ANIMATION: return "ANIMATION";
        case TimerType::SYSTEM: return "SYSTEM";
        case TimerType::GAMEPLAY: return "GAMEPLAY";
        default: return "UNKNOWN";
        }
    }
} // namespace engine::memory
