//
// Created by Andres Guerrero on 14-08-25.
//

#pragma once

#include <functional>
#include <shared_mutex>
#include <string>
#include <utility>

#include "MemoryConfig.h"
#include "../allocators/LinearAllocator.h"
#include "../allocators/StackAllocator.h"
#include "../allocators/PoolAllocator.h"
#include "../allocators/RingBufferAllocator.h"
#include "../core/IAllocator.h"

namespace engine::memory {
    class MemoryManager {
    public:
        MemoryManager() = default;
        ~MemoryManager();

        MemoryManager(const MemoryManager&) = delete;
        MemoryManager& operator=(const MemoryManager&) = delete;

        bool initialize(const MemoryManagerConfig& requestedConfig);
        void shutdown();
        bool isInitialized() const { return initialized; }

        void* allocate(MemorySize size,
                       MemoryCategory category = MemoryCategory::GENERAL,
                       MemorySize alignment = DEFAULT_ALIGNMENT,
                       AllocationFlags flags = AllocationFlags::NONE);
        void deallocate(void* ptr, MemoryCategory category = MemoryCategory::GENERAL);
        void* reallocate(void* ptr, MemorySize newSize,
                        MemoryCategory category = MemoryCategory::GENERAL,
                        MemorySize alignment = DEFAULT_ALIGNMENT);

        template <typename T, typename... Args>
        T* allocateObject(MemoryCategory category, Args&&... args) {
            void* memory = allocate(sizeof(T), category, alignof(T));
            if (!memory) return nullptr;

            try {
                return new(memory) T(std::forward<Args>(args)...);
            }
            catch (...) {
                deallocate(memory, category);
                throw;
            }
        }

        template <typename T>
        void deallocateObject(T* ptr, const MemoryCategory category) {
            if (!ptr) return;

            ptr->~T();
            deallocate(ptr, category);
        }

        template <typename T>
        T* allocateArray(std::size_t count, MemoryCategory category) {
            MemorySize size = sizeof(T) * count + sizeof(std::size_t);
            void* memory = allocate(size, category, alignof(T));
            if (!memory) return nullptr;

            // Store count at beginning
            *static_cast<std::size_t*>(memory) = count;
            T* array = reinterpret_cast<T*>(static_cast<std::uint8_t*>(memory) + sizeof(std::size_t));

            // Default construct elements
            for (std::size_t i = 0; i < count; ++i) {
                new(&array[i]) T();
            }

            return array;
        }

        template <typename T>
        void deallocateArray(T* array, const MemoryCategory category) {
            if (!array) return;

            // Get count from before array
            void* memory = reinterpret_cast<std::uint8_t*>(array) - sizeof(std::size_t);
            const std::size_t count = *static_cast<std::size_t*>(memory);

            // Destruct elements
            for (std::size_t i = 0; i < count; ++i) {
                array[i].~T();
            }

            deallocate(memory, category);
        }

        StackAllocator& getFrameStackAllocator() const;
        LinearAllocator& getFrameLinearAllocator() const;
        void beginFrame(std::uint64_t frameNumber);
        static void endFrame();

        IAllocator* getAllocator(MemoryCategory category) const;
        void registerAllocator(MemoryCategory category, std::unique_ptr<IAllocator> allocator);

        template <typename T>
        PoolAllocator* createPoolAllocator(std::size_t objectCount,
                                           const MemoryCategory category = MemoryCategory::GENERAL) {
            auto pool = std::make_unique<PoolAllocator>(
                sizeof(T), objectCount, alignof(T),
                (std::string("Pool_") + typeid(T).name()).c_str()
            );

            registerAllocator(category, std::move(pool));

            IAllocator* allocator = getAllocator(category);

            return dynamic_cast<PoolAllocator*>(allocator);
        }

        MemorySize getTotalMemoryUsage() const;
        MemorySize getCategoryMemoryUsage(MemoryCategory category) const;
        const MemoryStats& getGlobalStats() const { return globalStats; }
        std::string generateMemoryReport() const;
        bool dumpAllocations(const std::string& filename) const;
        static std::size_t checkForLeaks() ;

        using MemoryPressureCallback = std::function<void(MemorySize available, MemorySize required)>;
        void registerMemoryPressureCallback(MemoryPressureCallback callback);
        MemorySize performMemoryCleanup(MemorySize targetBytes) const;
        static void getSystemMemoryInfo(MemorySize& totalPhysical,
                                        MemorySize& availablePhysical,
                                        MemorySize& totalVirtual,
                                        MemorySize& availableVirtual);

    private:
        MemoryManagerConfig config;
        bool initialized = false;

        std::unique_ptr<IAllocator> mainHeap;
        std::unique_ptr<IAllocator> debugHeap;

        struct FrameAllocators {
            std::unique_ptr<StackAllocator> stackAllocator;
            std::unique_ptr<LinearAllocator> linearAllocator;
            std::uint64_t frameNumber = 0;
        };

        std::vector<FrameAllocators> frameAllocators;
        std::atomic<std::uint8_t> currentFrameIndex{0};

        std::array<std::unique_ptr<IAllocator>, static_cast<std::size_t>(MemoryCategory::COUNT)> categoryAllocators;

        std::unordered_map<std::string, std::unique_ptr<IAllocator>> customAllocators;

        mutable MemoryStats globalStats;

        // Allocation tracking (debug mode)
#ifdef _DEBUG
        struct AllocationRecord {
            void* address;
            MemorySize size;
            MemoryCategory category;
            std::string file;
            int line;
            std::chrono::steady_clock::time_point timestamp;
            std::thread::id threadId;
        };
        std::unordered_map<void*, AllocationRecord> allocationRecords;
        mutable std::shared_mutex recordsMutex;
#endif

        std::vector<MemoryPressureCallback> memoryPressureCallbacks;
        std::atomic<bool> inMemoryPressure{false};

        mutable std::shared_mutex allocatorsMutex;

        IAllocator* selectAllocator(MemoryCategory category, MemorySize size, AllocationFlags flags) const;
        void updateGlobalStats(const MemoryStats& allocatorStats);
        void handleAllocationFailure(MemorySize size, MemoryCategory category);

#ifdef _DEBUG
        void recordAllocation(void* ptr, MemorySize size, MemoryCategory category,
                              const char* file, int line);
        void removeAllocationRecord(void* ptr);
        void validateAllocation(void* ptr) const;
#endif
    };
}
