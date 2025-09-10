//
// Created by Andres Guerrero on 31-08-25.
//

#pragma once


#include "MemoryManager.h"

#include <cassert>
#include <unordered_set>

/**
 * @brief Object pool adapted for MemoryManager integration
 * @details Uses MemoryManager for allocations instead of direct IAllocator
 */
namespace engine::memory {
    template <typename T>
    class ObjectPool {
    public:
        ObjectPool() = default;
        ~ObjectPool() = default;

        /**
         * @brief Initialize pool with MemoryManager
         * @param capacity Initial capacity of the pool
         * @param memoryManager Pointer to memory manager
         */
        void initialize(std::size_t capacity, MemoryManager* memoryManager) {
            assert(memoryManager && "MemoryManager cannot be null");
            memoryManager_ = memoryManager;

            m_pool.reserve(capacity);
            m_capacity = capacity;

            // Pre-allocate objects using MemoryManager
            for (std::size_t i = 0; i < capacity; ++i) {
                if (T* obj = memoryManager_->allocateObject<T>(memory::MemoryCategory::PHYSICS)) {
                    m_pool.push_back(obj);
                }
            }
        }

        /**
         * @brief Allocate object from pool
         * @return Pointer to allocated object or nullptr if pool is empty
         */
        T* allocate() {
            if (!m_pool.empty()) {
                T* obj = m_pool.back();
                m_pool.pop_back();
                m_used.insert(obj);
                return obj;
            }

            // Pool exhausted - try to allocate new object
            T* obj = memoryManager_->allocateObject<T>(memory::MemoryCategory::PHYSICS);
            if (obj) {
                m_used.insert(obj);
                m_capacity++; // Track dynamic growth
            }

            return obj;
        }

        /**
         * @brief Return object to pool
         * @param obj Object to return to pool
         * @return true if object was returned to pool, false if not recognized
         */
        bool deallocate(T* obj) {
            if (!obj) return false;

            if (m_used.erase(obj) > 0) {
                // Reset object state before returning to pool
                // This assumes T has a reset() method - adjust as needed
                if constexpr (requires { obj->reset(); }) {
                    obj->reset();
                }

                m_pool.push_back(obj);
                return true;
            }
            return false;
        }

        /**
         * @brief Clear pool and deallocate all objects
         */
        void clear() {
            if (!memoryManager_) return;

            // Deallocate pooled objects
            for (T* obj : m_pool) {
                memoryManager_->deallocateObject(obj, memory::MemoryCategory::PHYSICS);
            }

            // Deallocate used objects (this might indicate a leak)
            for (T* obj : m_used) {
#ifdef _DEBUG
                std::cerr << "Warning: ObjectPool clearing object still in use!" << std::endl;
#endif
                memoryManager_->deallocateObject(obj, memory::MemoryCategory::PHYSICS);
            }

            m_pool.clear();
            m_used.clear();
            m_capacity = 0;
        }

        /**
         * @brief Get total capacity (initial + dynamically allocated)
         */
        std::size_t getCapacity() const { return m_capacity; }

        /**
         * @brief Get number of objects currently in use
         */
        std::size_t getUsedCount() const { return m_used.size(); }

        /**
         * @brief Get number of objects available in pool
         */
        std::size_t getAvailableCount() const { return m_pool.size(); }

        /**
         * @brief Check if an object belongs to this pool
         */
        bool owns(const T* obj) const {
            return m_used.contains(const_cast<T*>(obj)) ||
                std::find(m_pool.begin(), m_pool.end(), obj) != m_pool.end();
        }

    private:
        memory::MemoryManager* memoryManager_ = nullptr;
        std::vector<T*> m_pool; ///< Available objects
        std::unordered_set<T*> m_used; ///< Objects in use
        std::size_t m_capacity = 0; ///< Total capacity
    };
} // namespace engine::physics
