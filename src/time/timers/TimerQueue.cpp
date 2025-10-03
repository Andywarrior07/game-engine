/**
 * @file TimerQueue.cpp
 * @brief Implementation of high-performance timer priority queue
 * @details Binary heap implementation with optimized cache access
 *
 * @author Development Team
 * @date Created on 2024-01-20
 */

#include "TimerQueue.h"

#include <algorithm>
#include <chrono>
#include <iostream>

namespace engine::time {

    // =============================================================================
    // TimerQueue Implementation
    // =============================================================================

    TimerQueue::TimerQueue(const TimerQueueConfig& config) :
        config_(config) {

        // Reserve initial capacity
        if (config_.initialCapacity > 0) {
            heap_.reserve(config_.initialCapacity);
            indexMap_.reserve(config_.initialCapacity);
        }

        stats_.reset();
    }

    TimerQueue::~TimerQueue() {
        shutdown();
    }

    TimerQueue::TimerQueue(TimerQueue&& other) noexcept :
        config_(other.config_)
        , initialized_(other.initialized_)
        , heap_(std::move(other.heap_))
        , indexMap_(std::move(other.indexMap_))
        , memoryManager_(other.memoryManager_) {
        // === Manual Transfer de Atomics ===
        // Usamos memory_order_relaxed porque estamos en move (single-threaded context)
        // El objeto 'other' está siendo destruido, nadie más debería accederlo

        stats_.currentSize.store(
                other.stats_.currentSize.load(std::memory_order_relaxed),
                std::memory_order_relaxed
                );

        stats_.peakSize.store(
                other.stats_.peakSize.load(std::memory_order_relaxed),
                std::memory_order_relaxed
                );

        stats_.totalInsertions.store(
                other.stats_.totalInsertions.load(std::memory_order_relaxed),
                std::memory_order_relaxed
                );

        stats_.totalRemovals.store(
                other.stats_.totalRemovals.load(std::memory_order_relaxed),
                std::memory_order_relaxed
                );

        stats_.totalProcessed.store(
                other.stats_.totalProcessed.load(std::memory_order_relaxed),
                std::memory_order_relaxed
                );

        stats_.totalProcessingTime.store(
                other.stats_.totalProcessingTime.load(std::memory_order_relaxed),
                std::memory_order_relaxed
                );

        stats_.averageInsertTime.store(
                other.stats_.averageInsertTime.load(std::memory_order_relaxed),
                std::memory_order_relaxed
                );

        stats_.averageRemoveTime.store(
                other.stats_.averageRemoveTime.load(std::memory_order_relaxed),
                std::memory_order_relaxed
                );

        stats_.heapResizes.store(
                other.stats_.heapResizes.load(std::memory_order_relaxed),
                std::memory_order_relaxed
                );

        stats_.heapRebuilds.store(
                other.stats_.heapRebuilds.load(std::memory_order_relaxed),
                std::memory_order_relaxed
                );

        // Reset del objeto origen (buena práctica)
        other.initialized_ = false;
        other.memoryManager_ = nullptr;
        other.stats_.reset();
    }

    TimerQueue& TimerQueue::operator=(TimerQueue&& other) noexcept {
        if (this != &other) {
            shutdown();

            config_ = other.config_;
            initialized_ = other.initialized_;
            heap_ = std::move(other.heap_);
            indexMap_ = std::move(other.indexMap_);
            memoryManager_ = other.memoryManager_;

            stats_.currentSize.store(
                    other.stats_.currentSize.load(std::memory_order_relaxed),
                    std::memory_order_relaxed
                    );

            stats_.peakSize.store(
                    other.stats_.peakSize.load(std::memory_order_relaxed),
                    std::memory_order_relaxed
                    );

            stats_.totalInsertions.store(
                    other.stats_.totalInsertions.load(std::memory_order_relaxed),
                    std::memory_order_relaxed
                    );

            stats_.totalRemovals.store(
                    other.stats_.totalRemovals.load(std::memory_order_relaxed),
                    std::memory_order_relaxed
                    );

            stats_.totalProcessed.store(
                    other.stats_.totalProcessed.load(std::memory_order_relaxed),
                    std::memory_order_relaxed
                    );

            stats_.totalProcessingTime.store(
                    other.stats_.totalProcessingTime.load(std::memory_order_relaxed),
                    std::memory_order_relaxed
                    );

            stats_.averageInsertTime.store(
                    other.stats_.averageInsertTime.load(std::memory_order_relaxed),
                    std::memory_order_relaxed
                    );

            stats_.averageRemoveTime.store(
                    other.stats_.averageRemoveTime.load(std::memory_order_relaxed),
                    std::memory_order_relaxed
                    );

            stats_.heapResizes.store(
                    other.stats_.heapResizes.load(std::memory_order_relaxed),
                    std::memory_order_relaxed
                    );

            stats_.heapRebuilds.store(
                    other.stats_.heapRebuilds.load(std::memory_order_relaxed),
                    std::memory_order_relaxed
                    );

            other.initialized_ = false;
            other.memoryManager_ = nullptr;
            other.stats_.reset();
        }
        return *this;
    }

    bool TimerQueue::initialize(engine::memory::MemoryManager* memoryManager) {
        if (initialized_) {
            return true;
        }

        memoryManager_ = memoryManager ? memoryManager : config_.memoryManager;
        if (!memoryManager_) {
            std::cerr << "[TimerQueue] Error: No memory manager provided!" << std::endl;
            return false;
        }

        // Allocate initial capacity using memory manager
        if (config_.initialCapacity > 0) {
            try {
                // Pre-allocate space
                heap_.reserve(config_.initialCapacity);
                indexMap_.reserve(config_.initialCapacity);
            } catch (const std::exception& e) {
                std::cerr << "[TimerQueue] Failed to allocate initial capacity: "
                        << e.what() << std::endl;
                return false;
            }
        }

        initialized_ = true;
        return true;
    }

    void TimerQueue::shutdown() {
        if (!initialized_) {
            return;
        }

        clear();

        // Release memory
        heap_.clear();
        heap_.shrink_to_fit();
        indexMap_.clear();

        initialized_ = false;
    }

    bool TimerQueue::insert(const Timer& timer) {
        std::lock_guard lock(queueMutex_);

        const auto startTime = Clock::now();

        // Check capacity
        if (heap_.size() >= config_.maxCapacity) {
            if (!config_.allowDynamicGrowth) {
                return false;
            }
            if (!growCapacity()) {
                return false;
            }
        }

        // Create entry
        QueueEntry entry;
        entry.timer = std::move(const_cast<Timer&>(timer));
        entry.heapIndex = heap_.size();

        // Add to heap
        heap_.push_back(std::move(entry));

        // Update index map
        indexMap_[timer.getHandle().id] = entry.heapIndex;

        // Restore heap property
        heapifyUp(entry.heapIndex);

        // Update statistics
        stats_.totalInsertions.fetch_add(1, std::memory_order_relaxed);
        stats_.currentSize.store(heap_.size(), std::memory_order_relaxed);

        // Update peak size
        std::size_t currentPeak = stats_.peakSize.load(std::memory_order_relaxed);
        while (heap_.size() > currentPeak &&
            !stats_.peakSize.compare_exchange_weak(currentPeak, heap_.size())) {}

        const auto endTime = Clock::now();
        updateStats(true, std::chrono::duration_cast<Duration>(endTime - startTime));

        return true;
    }

    bool TimerQueue::remove(const TimerHandle& handle) {
        std::lock_guard lock(queueMutex_);

        const auto startTime = Clock::now();

        // Find timer in index map
        const auto it = indexMap_.find(handle.id);
        if (it == indexMap_.end()) {
            return false;
        }

        const std::size_t index = it->second;

        // Verify generation
        if (heap_[index].timer.getHandle().generation != handle.generation) {
            return false;
        }

        // Remove from index map
        indexMap_.erase(it);

        // If not last element, swap with last and reheapify
        if (index != heap_.size() - 1) {
            swapElements(index, heap_.size() - 1);
            heap_.pop_back();

            // Reheapify at swapped position
            if (index < heap_.size()) {
                // Check if we need to go up or down
                if (index > 0 && heap_[index].timer.getExpiration() <
                    heap_[getParent(index)].timer.getExpiration()) {
                    heapifyUp(index);
                } else {
                    heapifyDown(index);
                }
            }
        } else {
            heap_.pop_back();
        }

        // Update statistics
        stats_.totalRemovals.fetch_add(1, std::memory_order_relaxed);
        stats_.currentSize.store(heap_.size(), std::memory_order_relaxed);

        const auto endTime = Clock::now();
        updateStats(false, std::chrono::duration_cast<Duration>(endTime - startTime));

        return true;
    }

    bool TimerQueue::update(const TimerHandle& handle, const TimeStamp newExpiration) {
        std::lock_guard lock(queueMutex_);

        // Find timer
        const auto it = indexMap_.find(handle.id);
        if (it == indexMap_.end()) {
            return false;
        }

        const std::size_t index = it->second;

        // Verify generation
        if (heap_[index].timer.getHandle().generation != handle.generation) {
            return false;
        }

        // Update expiration
        const TimeStamp oldExpiration = heap_[index].timer.getExpiration();
        heap_[index].timer.setExpiration(newExpiration);

        // Reheapify based on change direction
        if (newExpiration < oldExpiration) {
            heapifyUp(index);
        } else if (newExpiration > oldExpiration) {
            heapifyDown(index);
        }

        return true;
    }

    const Timer* TimerQueue::peek() const noexcept {
        std::lock_guard lock(queueMutex_);

        if (heap_.empty()) {
            return nullptr;
        }

        return &heap_[0].timer;
    }

    std::optional<Timer> TimerQueue::pop() {
        std::lock_guard lock(queueMutex_);

        if (heap_.empty()) {
            return std::nullopt;
        }

        // Get top timer
        Timer timer = std::move(heap_[0].timer);

        // Remove from index map
        indexMap_.erase(timer.getHandle().id);

        // Move last element to top
        if (heap_.size() > 1) {
            swapElements(0, heap_.size() - 1);
            heap_.pop_back();
            heapifyDown(0);
        } else {
            heap_.pop_back();
        }

        // Update statistics
        stats_.currentSize.store(heap_.size(), std::memory_order_relaxed);

        return timer;
    }

    std::size_t TimerQueue::processExpired(
            const TimeStamp currentTime,
            const std::function<void(Timer&)>& callback
            ) {

        if (!callback) {
            return 0;
        }

        std::lock_guard lock(queueMutex_);

        const auto startTime = Clock::now();
        std::size_t processed = 0;

        while (!heap_.empty() && heap_[0].timer.getExpiration() <= currentTime) {
            Timer timer = std::move(heap_[0].timer);

            // Remove from queue
            indexMap_.erase(timer.getHandle().id);

            if (heap_.size() > 1) {
                swapElements(0, heap_.size() - 1);
                heap_.pop_back();
                heapifyDown(0);
            } else {
                heap_.pop_back();
            }

            // Process timer
            callback(timer);
            processed++;
        }

        // Update statistics
        stats_.currentSize.store(heap_.size(), std::memory_order_relaxed);
        stats_.totalProcessed.fetch_add(processed, std::memory_order_relaxed);

        const auto endTime = Clock::now();
        const auto processingTime = std::chrono::duration_cast<Duration>(endTime - startTime);

        Duration current = stats_.totalProcessingTime.load(std::memory_order_relaxed);
        while (!stats_.totalProcessingTime.compare_exchange_weak(
                current,
                current + processingTime
                )) {}

        return processed;
    }

    std::size_t TimerQueue::processBatch(
            const TimeStamp currentTime,
            const std::function<void(std::vector<Timer>&)>& batchCallback
            ) {

        if (!batchCallback) {
            return 0;
        }

        std::lock_guard lock(queueMutex_);

        std::vector<Timer> batch;
        batch.reserve(config_.batchSize);

        std::size_t totalProcessed = 0;

        while (!heap_.empty() && heap_[0].timer.getExpiration() <= currentTime) {
            Timer timer = std::move(heap_[0].timer);

            // Remove from queue
            indexMap_.erase(timer.getHandle().id);

            if (heap_.size() > 1) {
                swapElements(0, heap_.size() - 1);
                heap_.pop_back();
                heapifyDown(0);
            } else {
                heap_.pop_back();
            }

            batch.push_back(std::move(timer));

            // Process batch when full
            if (batch.size() >= config_.batchSize) {
                batchCallback(batch);
                totalProcessed += batch.size();
                batch.clear();
            }
        }

        // Process remaining batch
        if (!batch.empty()) {
            batchCallback(batch);
            totalProcessed += batch.size();
        }

        // Update statistics
        stats_.currentSize.store(heap_.size(), std::memory_order_relaxed);
        stats_.totalProcessed.fetch_add(totalProcessed, std::memory_order_relaxed);

        return totalProcessed;
    }

    void TimerQueue::clear() {
        std::lock_guard lock(queueMutex_);

        heap_.clear();
        indexMap_.clear();

        stats_.currentSize.store(0, std::memory_order_relaxed);
    }

    void TimerQueue::reserve(std::size_t capacity) {
        std::lock_guard lock(queueMutex_);

        if (capacity > config_.maxCapacity) {
            capacity = config_.maxCapacity;
        }

        heap_.reserve(capacity);
        indexMap_.reserve(capacity);
    }

    void TimerQueue::rebuild() {
        std::lock_guard lock(queueMutex_);

        if (heap_.size() <= 1) {
            return;
        }

        // Rebuild heap from bottom up
        for (std::size_t i = heap_.size() / 2; i > 0; --i) {
            heapifyDown(i - 1);
        }

        // Rebuild index map
        indexMap_.clear();
        for (std::size_t i = 0; i < heap_.size(); ++i) {
            heap_[i].heapIndex = i;
            indexMap_[heap_[i].timer.getHandle().id] = i;
        }

        stats_.heapRebuilds.fetch_add(1, std::memory_order_relaxed);
    }

    bool TimerQueue::isValid() const {
        std::lock_guard lock(queueMutex_);

        // Validate heap property
        for (std::size_t i = 0; i < heap_.size(); ++i) {
            const std::size_t left = getLeftChild(i);
            const std::size_t right = getRightChild(i);

            if (left < heap_.size() &&
                heap_[i].timer.getExpiration() > heap_[left].timer.getExpiration()) {
                return false;
            }

            if (right < heap_.size() &&
                heap_[i].timer.getExpiration() > heap_[right].timer.getExpiration()) {
                return false;
            }

            // Validate index map
            if (heap_[i].heapIndex != i) {
                return false;
            }
        }

        // Validate index map size
        if (indexMap_.size() != heap_.size()) {
            return false;
        }

        return true;
    }

    Timer* TimerQueue::find(const TimerHandle& handle) {
        std::lock_guard lock(queueMutex_);

        const auto it = indexMap_.find(handle.id);
        if (it == indexMap_.end()) {
            return nullptr;
        }

        const std::size_t index = it->second;
        if (index >= heap_.size()) {
            return nullptr;
        }

        // Verify generation
        if (heap_[index].timer.getHandle().generation != handle.generation) {
            return nullptr;
        }

        return &heap_[index].timer;
    }

    const Timer* TimerQueue::find(const TimerHandle& handle) const {
        return const_cast<TimerQueue*>(this)->find(handle);
    }

    bool TimerQueue::contains(const TimerHandle& handle) const {
        return find(handle) != nullptr;
    }

    Duration TimerQueue::getTimeUntilNext(const TimeStamp currentTime) const {
        std::lock_guard lock(queueMutex_);

        if (heap_.empty()) {
            return Duration::max();
        }

        const TimeStamp nextExpiration = heap_[0].timer.getExpiration();

        if (nextExpiration <= currentTime) {
            return Duration::zero();
        }

        return std::chrono::duration_cast<Duration>(nextExpiration - currentTime);
    }

    void TimerQueue::updateConfig(const TimerQueueConfig& config) {
        std::lock_guard lock(queueMutex_);

        config_ = config;

        // Apply new capacity limits
        if (heap_.capacity() > config_.maxCapacity) {
            heap_.shrink_to_fit();
        }
    }

    void TimerQueue::heapifyUp(std::size_t index) {
        while (index > 0) {
            const std::size_t parent = getParent(index);

            if (heap_[parent].timer.getExpiration() <= heap_[index].timer.getExpiration()) {
                break;
            }

            swapElements(parent, index);
            index = parent;
        }
    }

    void TimerQueue::heapifyDown(std::size_t index) {
        while (true) {
            std::size_t smallest = index;
            const std::size_t left = getLeftChild(index);
            const std::size_t right = getRightChild(index);

            if (left < heap_.size() &&
                heap_[left].timer.getExpiration() < heap_[smallest].timer.getExpiration()) {
                smallest = left;
            }

            if (right < heap_.size() &&
                heap_[right].timer.getExpiration() < heap_[smallest].timer.getExpiration()) {
                smallest = right;
            }

            if (smallest == index) {
                break;
            }

            swapElements(index, smallest);
            index = smallest;
        }
    }

    void TimerQueue::swapElements(const std::size_t i, const std::size_t j) {
        if (i == j)
            return;

        // Swap elements
        std::swap(heap_[i], heap_[j]);

        // Update indices
        heap_[i].heapIndex = i;
        heap_[j].heapIndex = j;

        // Update index map
        indexMap_[heap_[i].timer.getHandle().id] = i;
        indexMap_[heap_[j].timer.getHandle().id] = j;
    }

    void TimerQueue::updateIndexMap(const std::size_t index) {
        if (index < heap_.size()) {
            heap_[index].heapIndex = index;
            indexMap_[heap_[index].timer.getHandle().id] = index;
        }
    }

    bool TimerQueue::growCapacity() {
        const std::size_t currentCapacity = heap_.capacity();
        const auto newCapacity = static_cast<std::size_t>(
            static_cast<float>(currentCapacity) * config_.growthFactor
        );

        if (newCapacity > config_.maxCapacity) {
            return false;
        }

        try {
            heap_.reserve(newCapacity);
            indexMap_.reserve(newCapacity);

            stats_.heapResizes.fetch_add(1, std::memory_order_relaxed);
            return true;
        } catch (...) {
            return false;
        }
    }

    void TimerQueue::updateStats(const bool isInsertion, const Duration operationTime) {
        if (!config_.trackStatistics) {
            return;
        }

        if (isInsertion) {
            // Update average insert time
            if (const auto totalInserts = stats_.totalInsertions.load(std::memory_order_relaxed); totalInserts > 0) {
                const Duration currentAvg = stats_.averageInsertTime.load(std::memory_order_relaxed);
                const Duration newAvg = (currentAvg * (totalInserts - 1) + operationTime) / totalInserts;
                stats_.averageInsertTime.store(newAvg, std::memory_order_relaxed);
            }
        } else {
            // Update average remove time
            if (const auto totalRemoves = stats_.totalRemovals.load(std::memory_order_relaxed); totalRemoves > 0) {
                const Duration currentAvg = stats_.averageRemoveTime.load(std::memory_order_relaxed);
                const Duration newAvg = (currentAvg * (totalRemoves - 1) + operationTime) / totalRemoves;
                stats_.averageRemoveTime.store(newAvg, std::memory_order_relaxed);
            }
        }
    }

} // namespace engine::time
