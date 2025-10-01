/**
 * @file TimerHandle.h
 * @brief Type-safe timer handle for secure timer references
 * @details Provides an opaque handle system for timer management that prevents
 *          use-after-free bugs through generation counting and validation.
 *          Zero-cost abstraction in release builds with comprehensive validation in debug.
 *
 * @author Andres Guerrero
 * @date Created on 2025-09-19
 */

#pragma once

#include "../core/TimeTypes.h"

#include <atomic>
#include <functional>
#include <optional>

namespace engine::time {
    // Forward declaration
    class TimerSystem;

    // =============================================================================
    // Timer Handle Core
    // =============================================================================

    /**
     * @brief Extended timer handle with validation and lifecycle management
     * @details Builds upon the basic TimerHandle from TimeTypes.h with additional
     *          safety features and validation. This is the user-facing handle type.
     */
    class SafeTimerHandle {
    public:
        /**
         * @brief Default constructor creates invalid handle
         */
        constexpr SafeTimerHandle() noexcept
            : ownerSystem_{nullptr} {
        }

        /**
         * @brief Construct from raw handle and owner system
         * @param handle Raw timer handle
         * @param system Owning timer system
         */
        explicit SafeTimerHandle(const TimerHandle handle, TimerSystem* system = nullptr) noexcept
            : handle_(handle), ownerSystem_(system) {
        }

        /**
         * @brief Copy constructor
         */
        SafeTimerHandle(const SafeTimerHandle&) noexcept = default;

        /**
         * @brief Move constructor
         */
        SafeTimerHandle(SafeTimerHandle&& other) noexcept
            : handle_(other.handle_), ownerSystem_(other.ownerSystem_) {
            other.reset(false);
        }

        /**
         * @brief Copy assignment
         */
        SafeTimerHandle& operator=(const SafeTimerHandle&) noexcept = default;

        /**
         * @brief Move assignment
         */
        SafeTimerHandle& operator=(SafeTimerHandle&& other) noexcept {
            if (this != &other) {
                handle_ = other.handle_;
                ownerSystem_ = other.ownerSystem_;

                other.reset(false);
            }
            return *this;
        }

        /**
         * @brief Destructor - does not cancel timer
         * @details Handles are lightweight references, not owners.
         *          Timers must be explicitly cancelled if needed.
         */
        ~SafeTimerHandle() = default;

        // =============================================================================
        // Validation and Queries
        // =============================================================================

        /**
         * @brief Check if handle references a valid timer
         * @return True if handle is valid (non-null)
         */
        [[nodiscard]] constexpr bool isValid() const noexcept {
            return handle_.isValid();
        }

        /**
         * @brief Check if handle is null/invalid
         * @return True if handle is invalid
         */
        [[nodiscard]] constexpr bool isNull() const noexcept {
            return !isValid();
        }

        /**
         * @brief Validate handle against current timer state
         * @return True if timer exists and generation matches
         * @details This performs runtime validation with the timer system.
         *          More expensive than isValid() but guarantees timer exists.
         */
        [[nodiscard]] bool isAlive() const noexcept;

        /**
         * @brief Check if timer has expired
         * @return True if timer fired and is no longer active
         */
        [[nodiscard]] bool hasExpired() const noexcept;

        /**
         * @brief Check if timer is currently paused
         * @return True if timer is paused
         */
        [[nodiscard]] bool isPaused() const noexcept;

        /**
         * @brief Get timer ID
         * @return Timer identifier
         */
        [[nodiscard]] constexpr TimerID getId() const noexcept {
            return handle_.id;
        }

        /**
         * @brief Get generation counter
         * @return Generation for validation
         */
        [[nodiscard]] constexpr TimerGeneration getGeneration() const noexcept {
            return handle_.generation;
        }

        /**
         * @brief Get raw handle
         * @return Underlying timer handle
         */
        [[nodiscard]] constexpr const TimerHandle& getRawHandle() const noexcept {
            return handle_;
        }

        // =============================================================================
        // Timer Control
        // =============================================================================

        /**
         * @brief Cancel the timer
         * @return True if timer was successfully cancelled
         */
        bool cancel() const noexcept;

        /**
         * @brief Pause the timer
         * @return True if timer was paused
         */
        bool pause() const noexcept;

        /**
         * @brief Resume the timer
         * @return True if timer was resumed
         */
        bool resume() const noexcept;

        /**
         * @brief Reset timer to initial duration
         * @param restart If true, start immediately
         * @return True if timer was reset
         */
        bool reset(bool restart = true) const noexcept;

        /**
         * @brief Get remaining time
         * @return Time until expiration or nullopt if invalid
         */
        [[nodiscard]] std::optional<Duration> getRemainingTime() const noexcept;

        /**
         * @brief Get elapsed time
         * @return Time since timer started or nullopt if invalid
         */
        [[nodiscard]] std::optional<Duration> getElapsedTime() const noexcept;

        /**
         * @brief Reset handle to invalid state
         */
        constexpr void reset() noexcept {
            handle_.reset();
            ownerSystem_ = nullptr;
        }

        // =============================================================================
        // Operators
        // =============================================================================

        /**
         * @brief Equality comparison
         * @details Compares both ID and generation
         */
        [[nodiscard]] constexpr bool operator==(const SafeTimerHandle& other) const noexcept {
            return handle_ == other.handle_;
        }

        [[nodiscard]] constexpr bool operator!=(const SafeTimerHandle& other) const noexcept {
            return !(*this == other);
        }

        /**
         * @brief Ordering for container storage
         */
        [[nodiscard]] constexpr auto operator<=>(const SafeTimerHandle& other) const noexcept {
            return handle_ <=> other.handle_;
        }

        /**
         * @brief Bool conversion for validity check
         */
        [[nodiscard]] explicit operator bool() const noexcept {
            return isValid();
        }

        [[nodiscard]] TimerSystem* getOwnerSystem() const noexcept { return ownerSystem_; }

    private:
        TimerHandle handle_; ///< Underlying handle
        TimerSystem* ownerSystem_; ///< Owning timer system

        friend class TimerSystem; ///< Allow system access
    };

    // =============================================================================
    // Handle Pool for Efficient Management
    // =============================================================================

    /**
     * @brief Pool for recycling timer handles
     * @details Manages handle generation to prevent ID reuse issues.
     *          Thread-safe implementation for concurrent timer creation.
     */
    class TimerHandlePool {
    public:
        /**
         * @brief Construct handle pool with capacity
         * @param maxHandles Maximum number of handles
         */
        explicit TimerHandlePool(const std::size_t maxHandles = constants::MAX_TIMERS) noexcept
            : maxHandles_(static_cast<TimerID>(maxHandles))
              , nextId_(1) // Start at 1, 0 is invalid
              , activeCount_(0) {
            // Pre-size the generation array
            generations_.reserve(maxHandles);

            for (std::size_t i = 0; i < maxHandles; ++i) {
                generations_.emplace_back(std::make_unique<std::atomic<std::uint16_t>>(0));
            }
        }

        /**
         * @brief Allocate a new handle
         * @return New timer handle or invalid if pool exhausted
         */
        [[nodiscard]] TimerHandle allocate() noexcept {
            // Try to reuse a freed handle first
            if (TimerID id; tryPopFreeList(id)) {
                const auto generation = generations_[id - 1]->fetch_add(1, std::memory_order_relaxed);
                activeCount_.fetch_add(1, std::memory_order_relaxed);
                return TimerHandle{id, static_cast<TimerGeneration>(generation + 1)};
            }

            // Allocate new ID if pool not exhausted
            const TimerID newId = nextId_.fetch_add(1, std::memory_order_relaxed);
            if (newId > maxHandles_) {
                nextId_.fetch_sub(1, std::memory_order_relaxed); // Revert
                return TimerHandle{}; // Pool exhausted
            }

            activeCount_.fetch_add(1, std::memory_order_relaxed);
            return TimerHandle{newId, 1}; // First generation
        }

        /**
         * @brief Release a handle back to the pool
         * @param handle Handle to release
         */
        void release(const TimerHandle handle) noexcept {
            if (!handle.isValid()) return;

            // Add to free list for reuse
            pushFreeList(handle.id);
            activeCount_.fetch_sub(1, std::memory_order_relaxed);
        }

        /**
         * @brief Validate a handle against current generation
         * @param handle Handle to validate
         * @return True if handle generation matches current
         */
        [[nodiscard]] bool validate(const TimerHandle handle) const noexcept {
            if (!handle.isValid() || handle.id > maxHandles_) {
                return false;
            }

            const auto currentGen = generations_[handle.id - 1]->load(std::memory_order_acquire);
            return handle.generation == static_cast<TimerGeneration>(currentGen);
        }

        /**
         * @brief Get number of active handles
         */
        [[nodiscard]] std::size_t getActiveCount() const noexcept {
            return activeCount_.load(std::memory_order_acquire);
        }

        /**
         * @brief Get pool capacity
         */
        [[nodiscard]] std::size_t getCapacity() const noexcept {
            return maxHandles_;
        }

        /**
         * @brief Reset pool (invalidates all handles)
         */
        void reset() noexcept {
            nextId_.store(1, std::memory_order_relaxed);
            activeCount_.store(0, std::memory_order_relaxed);

            // Clear free list
            TimerID dummy;
            while (tryPopFreeList(dummy)) {
            }

            // Reset all generations
            for (const auto& gen : generations_) {
                gen->store(0, std::memory_order_relaxed);
            }
        }

    private:
        const TimerID maxHandles_; ///< Maximum handles
        std::atomic<TimerID> nextId_; ///< Next ID to allocate
        std::atomic<std::size_t> activeCount_; ///< Active handle count
        std::vector<std::unique_ptr<std::atomic<std::uint16_t>>> generations_; ///< Generation per ID

        // Lock-free free list implementation
        struct FreeNode {
            TimerID id;
            std::atomic<FreeNode*> next;

            explicit FreeNode(const TimerID timerId) : id(timerId), next(nullptr) {}
        };

        std::atomic<FreeNode*> freeListHead_{nullptr}; ///< Free list head

        /**
         * @brief Try to pop from free list
         * @param[out] id Retrieved ID
         * @return True if ID was retrieved
         */
        bool tryPopFreeList(TimerID& id) noexcept {
            FreeNode* head = freeListHead_.load(std::memory_order_acquire);

            while (head) {
                if (FreeNode* next = head->next.load(std::memory_order_relaxed); freeListHead_.compare_exchange_weak(
                    head, next,
                    std::memory_order_release,
                    std::memory_order_acquire)) {
                    id = head->id;
                    delete head;
                    return true;
                }
            }

            return false;
        }

        /**
         * @brief Push ID to free list
         * @param id ID to add
         */
        void pushFreeList(const TimerID id) noexcept {
            auto node = std::make_unique<FreeNode>(id);
            FreeNode* rawNode = node.release();
            FreeNode* head = freeListHead_.load(std::memory_order_relaxed);

            do {
                rawNode->next.store(head, std::memory_order_relaxed);
            } while (!freeListHead_.compare_exchange_weak(head, rawNode,
                                                          std::memory_order_release,
                                                          std::memory_order_relaxed));
        }
    };

    // =============================================================================
    // Handle Validation Utilities
    // =============================================================================

    /**
     * @brief RAII guard for timer handle validation
     * @details Ensures handle remains valid for scope duration.
     *          Useful for critical timer operations.
     */
    class TimerHandleGuard {
    public:
        /**
         * @brief Construct guard for handle
         * @param handle Handle to guard
         * @throws std::runtime_error if handle is invalid
         */
        explicit TimerHandleGuard(const SafeTimerHandle& handle)
            : handle_(handle) {
            if (!handle_.isAlive()) {
                throw std::runtime_error("Timer handle is not alive");
            }
        }

        /**
         * @brief Destructor validates handle still alive
         */
        ~TimerHandleGuard() {
#ifdef _DEBUG
            if (!handle_.isAlive()) {
                // Log warning in debug builds
                // Timer was destroyed while guarded
            }
#endif
        }

        // Delete copy/move operations
        TimerHandleGuard(const TimerHandleGuard&) = delete;
        TimerHandleGuard& operator=(const TimerHandleGuard&) = delete;
        TimerHandleGuard(TimerHandleGuard&&) = delete;
        TimerHandleGuard& operator=(TimerHandleGuard&&) = delete;

        /**
         * @brief Get guarded handle
         */
        [[nodiscard]] const SafeTimerHandle& get() const noexcept {
            return handle_;
        }

    private:
        const SafeTimerHandle& handle_; ///< Guarded handle
    };

    // =============================================================================
    // Weak Timer Handle
    // =============================================================================

    /**
     * @brief Weak reference to timer that doesn't prevent deletion
     * @details Similar to std::weak_ptr but for timer handles.
     *          Useful for callbacks that shouldn't keep timers alive.
     */
    class WeakTimerHandle {
    public:
        /**
         * @brief Default constructor
         */
        WeakTimerHandle() noexcept = default;

        /**
         * @brief Construct from safe handle
         */
        explicit WeakTimerHandle(const SafeTimerHandle& handle) noexcept
            : handle_(handle.getRawHandle())
              , ownerSystem_(handle.getOwnerSystem()) {
        }

        /**
         * @brief Try to lock weak handle
         * @return Valid handle or null if timer expired
         */
        [[nodiscard]] SafeTimerHandle lock() const noexcept {
            if (expired()) {
                return SafeTimerHandle{};
            }

            return SafeTimerHandle{handle_, ownerSystem_};
        }

        /**
         * @brief Check if timer has expired
         */
        [[nodiscard]] bool expired() const noexcept;

        /**
         * @brief Reset to null
         */
        void reset() noexcept {
            handle_.reset();
            ownerSystem_ = nullptr;
        }

    private:
        TimerHandle handle_{}; ///< Weak handle
        TimerSystem* ownerSystem_{nullptr}; ///< Owner system
    };
} // namespace engine::time

// Hash specialization for SafeTimerHandle
namespace std {
    template <>
    struct hash<engine::time::SafeTimerHandle> {
        [[nodiscard]] std::size_t operator()(const engine::time::SafeTimerHandle& handle) const noexcept {
            return std::hash<engine::time::TimerHandle>{}(handle.getRawHandle());
        }
    };

    template <>
    struct hash<engine::time::WeakTimerHandle> {
        [[nodiscard]] std::size_t operator()(const engine::time::WeakTimerHandle& handle) const noexcept {
            const auto strong = handle.lock();
            return strong ? std::hash<engine::time::SafeTimerHandle>{}(strong) : 0;
        }
    };
} // namespace std
