/**
 * @file InputEventQueue.h
 * @brief Lock-free event queue for high-performance input handling
 * @author Game Engine Team
 * @date 2024
 *
 * Implements a lock-free single-producer multiple-consumer queue
 * optimized for input event processing.
 */

#pragma once

#include "InputConstants.h"
#include "InputEvent.h"

#include <array>
#include <atomic>

namespace engine::input {
/**
 * @brief Lock-free input event queue
 *
 * High-performance queue for passing events between threads.
 * Uses atomic operations for thread-safe access without locks.
 */
template <std::size_t Capacity = MAX_EVENTS_IN_QUEUE> class InputEventQueue {
public:
  static_assert((Capacity & (Capacity - 1)) == 0,
                "Capacity must be power of 2");

  InputEventQueue() noexcept : head_(0), tail_(0), size_(0) {
    // Initialize all slots as empty
    for (auto &slot : slots_) {
      slot.sequence.store(0, std::memory_order_relaxed);
    }
  }

  /**
   * @brief Push event to queue (thread-safe)
   */
  bool push(const InputEvent &event) noexcept {
    // Load current tail position
    std::size_t tail = tail_.load(std::memory_order_relaxed);

    for (;;) {
      // Get slot at tail position
      auto &slot = slots_[tail & (Capacity - 1)];

      // Check if slot is available for writing
      if (const std::size_t seq = slot.sequence.load(std::memory_order_acquire);
          seq == tail) {
        // Try to claim this slot
        if (tail_.compare_exchange_weak(tail, tail + 1,
                                        std::memory_order_relaxed)) {
          // Successfully claimed slot, write event
          slot.event = event;

          // Mark slot as filled
          slot.sequence.store(tail + 1, std::memory_order_release);

          size_.fetch_add(1, std::memory_order_relaxed);
          return true;
        }
      } else if (seq < tail) {
        // Queue is full
        return false;
      } else {
        // Another thread is ahead, retry with new tail
        tail = tail_.load(std::memory_order_relaxed);
      }
    }
  }

  /**
   * @brief Pop event from queue (thread-safe)
   */
  bool pop(InputEvent &event) noexcept {
    // Load current head position
    std::size_t head = head_.load(std::memory_order_relaxed);

    for (;;) {
      // Get slot at head position
      auto &slot = slots_[head & (Capacity - 1)];

      // Check if slot has data
      if (const std::size_t seq = slot.sequence.load(std::memory_order_acquire);
          seq == head + 1) {
        // Try to claim this slot for reading
        if (head_.compare_exchange_weak(head, head + 1,
                                        std::memory_order_relaxed)) {
          // Successfully claimed slot, read event
          event = slot.event;

          // Mark slot as empty (ready for next write at tail + Capacity)
          slot.sequence.store(head + Capacity, std::memory_order_release);

          size_.fetch_sub(1, std::memory_order_relaxed);
          return true;
        }
      } else if (seq < head + 1) {
        // Queue is empty
        return false;
      } else {
        // Another thread is ahead, retry with new head
        head = head_.load(std::memory_order_relaxed);
      }
    }
  }

  /**
   * @brief Try to push event (non-blocking)
   */
  bool tryPush(const InputEvent &event) noexcept { return push(event); }

  /**
   * @brief Try to pop event (non-blocking)
   */
  bool tryPop(InputEvent &event) noexcept { return pop(event); }

  /**
   * @brief Batch push events
   */
  std::size_t pushBatch(const InputEvent *events,
                        const std::size_t count) noexcept {
    std::size_t pushed = 0;
    for (std::size_t i = 0; i < count; ++i) {
      if (!push(events[i])) {
        break;
      }
      pushed++;
    }
    return pushed;
  }

  /**
   * @brief Batch pop events
   */
  std::size_t popBatch(InputEvent *events,
                       const std::size_t maxCount) noexcept {
    std::size_t popped = 0;
    for (std::size_t i = 0; i < maxCount; ++i) {
      if (!pop(events[i])) {
        break;
      }
      popped++;
    }
    return popped;
  }

  /**
   * @brief Get approximate size
   */
  std::size_t size() const noexcept {
    return size_.load(std::memory_order_relaxed);
  }

  /**
   * @brief Check if queue is empty
   */
  bool empty() const noexcept { return size() == 0; }

  /**
   * @brief Check if queue is full
   */
  bool full() const noexcept { return size() >= Capacity - 1; }

  /**
   * @brief Get capacity
   */
  static constexpr std::size_t capacity() noexcept { return Capacity; }

  /**
   * @brief Clear queue (not thread-safe)
   */
  void clear() noexcept {
    InputEvent dummy;
    while (pop(dummy)) {
      // Keep popping until empty
    }

    head_.store(0, std::memory_order_relaxed);
    tail_.store(0, std::memory_order_relaxed);
    size_.store(0, std::memory_order_relaxed);

    for (std::size_t i = 0; i < Capacity; ++i) {
      slots_[i].sequence.store(i, std::memory_order_relaxed);
    }
  }

private:
  struct Slot {
    std::atomic<std::size_t> sequence;
    InputEvent event;

    Slot() noexcept : sequence(0) {}
  };

  // Cache line padding to avoid false sharing
  alignas(64) std::atomic<std::size_t> head_;
  alignas(64) std::atomic<std::size_t> tail_;
  alignas(64) std::atomic<std::size_t> size_;
  alignas(64) std::array<Slot, Capacity> slots_;
};

/**
 * @brief Priority-based event queue
 *
 * Manages multiple queues with different priorities.
 */
class PriorityEventQueue {
public:
  enum class Priority : std::uint8_t {
    CRITICAL = 0, // System events
    HIGH = 1,     // User input
    NORMAL = 2,   // Standard events
    LOW = 3,      // Background events
    COUNT = 4
  };

  PriorityEventQueue() noexcept = default;

  /**
   * @brief Push event with priority
   */
  bool push(const InputEvent &event, Priority priority = Priority::NORMAL) {
    auto index = static_cast<std::size_t>(priority);
    if (index >= queues_.size()) {
      index = static_cast<std::size_t>(Priority::NORMAL);
    }
    return queues_[index].push(event);
  }

  /**
   * @brief Pop highest priority event
   */
  bool pop(InputEvent &event) {
    // Try queues in priority order
    for (auto &queue : queues_) {
      if (queue.pop(event)) {
        return true;
      }
    }
    return false;
  }

  /**
   * @brief Get total size across all queues
   */
  std::size_t size() const noexcept {
    std::size_t total = 0;
    for (const auto &queue : queues_) {
      total += queue.size();
    }
    return total;
  }

  /**
   * @brief Check if all queues are empty
   */
  bool empty() const noexcept {
    return std::all_of(queues_.begin(), queues_.end(),
                       [](const auto &q) { return q.empty(); });
  }

  /**
   * @brief Clear all queues
   */
  void clear() noexcept {
    for (auto &queue : queues_) {
      queue.clear();
    }
  }

  /**
   * @brief Get queue for specific priority
   */
  InputEventQueue<256> &getQueue(Priority priority) {
    const auto index = static_cast<std::size_t>(priority);
    return queues_[index];
  }

private:
  std::array<InputEventQueue<256>, static_cast<std::size_t>(Priority::COUNT)>
      queues_;
};
} // namespace engine::input
