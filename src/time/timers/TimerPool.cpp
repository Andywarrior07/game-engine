//
// Created by Andres Guerrero on 29-09-25.
//

#include "TimerPool.h"

namespace engine::time {
    TimerPool::TimerPool(memory::MemoryManager& memoryManager, const Config& config) :
        memoryManager_(memoryManager)
        , config_(config)
        , currentSize_(0)
        , totalAllocated_(0)
        , peakUsage_(0)
        , allocationCount_(0)
        , deallocationCount_(0) {
        initialize();
    }

    TimerPool::~TimerPool() {
        shutdown();
    }

    Timer* TimerPool::allocate() {
        Timer* timer = nullptr;

        // Try fast path without lock
        {
            std::lock_guard lock(availableMutex_);

            if (!availableTimers_.empty()) {
                timer = availableTimers_.top();
                availableTimers_.pop();
            }
        }

        // If no timer available, try to grow
        if (!timer && config_.allowGrowth) {
            timer = growPool();
        }

        if (timer) {
            // Reset timer state
            new(timer) Timer(); // Placement new to reinitialize

            // Update statistics
            const auto inUse = totalAllocated_.fetch_add(
                    1,
                    std::memory_order_relaxed
                    ) + 1;
            updatePeakUsage(inUse);
            allocationCount_.fetch_add(1, std::memory_order_relaxed);
        }

        return timer;
    }

    Timer* TimerPool::allocate(const Timer::Config& config) {
        Timer* timer = allocate();

        if (timer && !timer->initialize(config)) {
            deallocate(timer);
            return nullptr;
        }

        return timer;
    }

    void TimerPool::deallocate(Timer* timer) {
        if (!timer || !owns(timer)) {
            return;
        }

        // Cancel timer before returning to pool
        timer->cancel();

        // Optionally zero memory
        if (config_.zeroMemory) {
            timer->reset(false);
            timer->cancel();
        }

        // Return to available pool
        {
            std::lock_guard lock(availableMutex_);
            availableTimers_.push(timer);
        }

        totalAllocated_.fetch_sub(1, std::memory_order_relaxed);
        deallocationCount_.fetch_add(1, std::memory_order_relaxed);
    }

    std::vector<Timer*> TimerPool::batchAllocate(const std::size_t count) {
        std::vector<Timer*> timers;
        timers.reserve(count);

        for (std::size_t i = 0; i < count; ++i) {
            Timer* timer = allocate();
            if (!timer) {
                break; // Pool exhausted
            }
            timers.push_back(timer);
        }

        return timers;
    }

    void TimerPool::batchDeallocate(const std::vector<Timer*>& timers) {
        for (Timer* timer : timers) {
            deallocate(timer);
        }
    }

    void TimerPool::clear() {
        std::lock_guard lock(poolMutex_);

        // Cancel all active timers
        for (const auto& chunk : memoryChunks_) {
            const auto timers = reinterpret_cast<Timer*>(chunk.memory);
            for (std::size_t i = 0; i < chunk.count; ++i) {
                timers[i].cancel();
            }
        }

        // Reset statistics
        totalAllocated_.store(0);
        allocationCount_.store(0);
        deallocationCount_.store(0);

        // Rebuild available stack
        rebuildAvailableStack();
    }

    bool TimerPool::reserve(const std::size_t additionalCount) {
        std::lock_guard lock(poolMutex_);

        if (const std::size_t targetSize = currentSize_ + additionalCount; targetSize > config_.maxSize) {
            return false;
        }

        return growPoolLocked(additionalCount) != nullptr;
    }

    bool TimerPool::owns(const Timer* timer) const {
        if (!timer)
            return false;

        std::lock_guard lock(poolMutex_);

        for (const auto& chunk : memoryChunks_) {
            const std::uint8_t* start = chunk.memory;
            const std::uint8_t* end = start + (chunk.count * sizeof(Timer));

            if (const auto ptr = reinterpret_cast<const std::uint8_t*>(timer); ptr >= start && ptr < end) {
                // Check alignment
                return ((ptr - start) % sizeof(Timer)) == 0;
            }
        }

        return false;
    }

    std::size_t TimerPool::size() const noexcept {
        return currentSize_.load(std::memory_order_acquire);
    }

    std::size_t TimerPool::allocated() const noexcept {
        return totalAllocated_.load(std::memory_order_acquire);
    }

    std::size_t TimerPool::available() const {
        std::lock_guard lock(availableMutex_);
        return availableTimers_.size();
    }

    float TimerPool::getUtilization() const noexcept {
        const std::size_t total = size();
        if (total == 0)
            return 0.0f;

        const std::size_t used = allocated();
        return static_cast<float>(used) / static_cast<float>(total);
    }

    void TimerPool::initialize() {
        if (config_.preallocate) {
            std::lock_guard lock(poolMutex_);
            growPoolLocked(config_.initialSize);
        }
    }

    void TimerPool::shutdown() {
        clear();

        std::lock_guard lock(poolMutex_);

        // Free all memory chunks
        for (const auto& chunk : memoryChunks_) {
            if (chunk.fromMemoryManager) {
                // Destroy timers before deallocating
                const auto timers = reinterpret_cast<Timer*>(chunk.memory);
                for (std::size_t i = 0; i < chunk.count; ++i) {
                    timers[i].~Timer();
                }

                memoryManager_.deallocate(chunk.memory, config_.memoryCategory);
            } else {
                // Direct delete for fallback allocations
                const Timer* timers = reinterpret_cast<Timer*>(chunk.memory);
                delete[] timers;
            }
        }

        memoryChunks_.clear();
    }

    Timer* TimerPool::growPoolLocked(std::size_t count) {
        if (currentSize_ + count > config_.maxSize) {
            count = config_.maxSize - currentSize_;
            if (count == 0) {
                return nullptr; // Already at max
            }
        }

        // Allocate memory chunk
        std::uint8_t* memory = nullptr;
        bool fromMemoryManager = false;

        memory = static_cast<std::uint8_t*>(
            memoryManager_.allocate(
                    count * sizeof(Timer),
                    config_.memoryCategory,
                    alignof(Timer)
                    )
        );
        fromMemoryManager = (memory != nullptr);

        // Fallback to standard allocation
        if (!memory) {
            const auto timers = new Timer[count];
            memory = reinterpret_cast<std::uint8_t*>(timers);
        }

        // Initialize timers
        auto timers = reinterpret_cast<Timer*>(memory);
        for (std::size_t i = 0; i < count; ++i) {
            new(&timers[i]) Timer(); // Placement new
        }

        // Add to chunks
        memoryChunks_.push_back(
                MemoryChunk{
                        .memory = memory,
                        .count = count,
                        .fromMemoryManager = fromMemoryManager
                }
                );

        // Add to available stack
        {
            std::lock_guard availLock(availableMutex_);
            for (std::size_t i = 0; i < count; ++i) {
                availableTimers_.push(&timers[i]);
            }
        }

        currentSize_.fetch_add(count, std::memory_order_relaxed);

        // Return one timer if this was triggered by allocation
        if (!availableTimers_.empty()) {
            Timer* timer = availableTimers_.top();
            availableTimers_.pop();
            return timer;
        }

        return nullptr;
    }

    Timer* TimerPool::growPool() {
        std::lock_guard lock(poolMutex_);
        return growPoolLocked(config_.growthSize);
    }

    void TimerPool::rebuildAvailableStack() {
        // Clear current stack
        while (!availableTimers_.empty()) {
            availableTimers_.pop();
        }

        // Add all timers back
        for (const auto& chunk : memoryChunks_) {
            auto timers = reinterpret_cast<Timer*>(chunk.memory);
            for (std::size_t i = 0; i < chunk.count; ++i) {
                if (timers[i].getState() == TimerState::INACTIVE ||
                    timers[i].getState() == TimerState::CANCELLED) {
                    availableTimers_.push(&timers[i]);
                }
            }
        }
    }

    void TimerPool::updatePeakUsage(const std::size_t currentUsage) {
        std::size_t peak = peakUsage_.load(std::memory_order_relaxed);
        while (currentUsage > peak &&
            !peakUsage_.compare_exchange_weak(
                    peak,
                    currentUsage,
                    std::memory_order_release,
                    std::memory_order_relaxed
                    )
        ) {}
    }
} // namespace engine::time
