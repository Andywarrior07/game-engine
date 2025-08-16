//
// Created by Andres Guerrero on 14-08-25.
//

#include "MemoryManager.h"

#include <cassert>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <ostream>
#include <sstream>
#include <mach/mach_host.h>
#include <mach/mach_init.h>
#include <mach/message.h>
#include <mach/vm_statistics.h>
#include <sys/sysctl.h>

namespace engine::memory {
    MemoryManager::~MemoryManager() {
        shutdown();
    }

    bool MemoryManager::initialize(const MemoryManagerConfig& requestedConfig) {
        if (initialized) {
            std::cerr << "Warning: MemoryManager already initialized!" << std::endl;
            return false;
        }

        config = requestedConfig;

        try {
            mainHeap = std::make_unique<LinearAllocator>(config.mainHeapSize, "MainHeap");

#ifdef _DEBUG
            // Create debug heap for debug allocations
            if (config.enableLeakDetection || config.enableBoundsChecking) {
                debugHeap_ = std::make_unique<LinearAllocator>(config.debugHeapSize, "DebugHeap");
            }
#endif

            frameAllocators.resize(config.frameBufferCount);

            for (std::uint8_t i = 0; i < config.frameBufferCount; ++i) {
                frameAllocators[i].stackAllocator = std::make_unique<StackAllocator>(
                    config.frameStackSize,
                    ("FrameStack_" + std::to_string(i)).c_str()
                );
                frameAllocators[i].linearAllocator = std::make_unique<LinearAllocator>(
                    config.frameLinearSize,
                    ("FrameLinear_" + std::to_string(i)).c_str()
                );
                frameAllocators[i].frameNumber = 0;
            }

            // Create specialized allocators for different categories

            // Rendering pool - for render commands, vertex data, etc.
            if (config.renderingPoolSize > 0) {
                auto renderPool = std::make_unique<PoolAllocator>(
                    1024, // 1KB blocks for render commands
                    config.renderingPoolSize / 1024,
                    DEFAULT_ALIGNMENT,
                    "RenderPool"
                );

                categoryAllocators[static_cast<std::size_t>(MemoryCategory::RENDERING)] = std::move(renderPool);
            }

            // Physics pool - for rigid bodies, colliders, etc.
            if (config.physicsPoolSize > 0) {
                auto physicsPool = std::make_unique<PoolAllocator>(
                    512, // 512 byte blocks for physics objects
                    config.physicsPoolSize / 512,
                    DEFAULT_ALIGNMENT,
                    "PhysicsPool"
                );
            }

            // Audio ring buffer - for streaming audio
            if (config.audioRingBufferSize > 0) {
                auto audioBuffer = std::make_unique<RingBufferAllocator>(
                    config.audioRingBufferSize,
                    "AudioRingBuffer"
                );
                categoryAllocators[static_cast<std::size_t>(MemoryCategory::AUDIO)] = std::move(audioBuffer);
            }

            // Network buffer - for packet data
            if (config.networkBufferSize > 0) {
                auto networkBuffer = std::make_unique<RingBufferAllocator>(
                    config.networkBufferSize,
                    "NetworkBuffer"
                );
                categoryAllocators[static_cast<std::size_t>(MemoryCategory::NETWORKING)] = std::move(networkBuffer);
            }

            // Create custom pools from configuration
            for (const auto& poolConfig : config.customPools) {
                auto pool = std::make_unique<PoolAllocator>(
                    poolConfig.blockSize,
                    poolConfig.blockCount,
                    DEFAULT_ALIGNMENT,
                    ("CustomPool_" + std::to_string(static_cast<int>(poolConfig.category))).c_str()
                );

                if (!categoryAllocators[static_cast<std::size_t>(poolConfig.category)]) {
                    categoryAllocators[static_cast<std::size_t>(poolConfig.category)] = std::move(pool);
                }
            }

            std::memset(&globalStats, 0, sizeof(globalStats));

            initialized = true;

            // Log initialization
            std::cout << "MemoryManager initialized successfully:" << std::endl;
            std::cout << "  Main Heap: " << (config.mainHeapSize / (1024.0 * 1024.0)) << " MB" << std::endl;
            std::cout << "  Frame Buffers: " << static_cast<int>(config.frameBufferCount) << std::endl;
            std::cout << "  Frame Stack: " << (config.frameStackSize / (1024.0 * 1024.0)) << " MB" << std::endl;
            std::cout << "  Frame Linear: " << (config.frameLinearSize / (1024.0 * 1024.0)) << " MB" << std::endl;

            return true;
        }
        catch (const std::exception& e) {
            std::cerr << "Failed to initialize MemoryManager: " << e.what() << std::endl;
            shutdown();

            return false;
        }
    }

    void MemoryManager::shutdown() {
        if (!initialized) {
            return;
        }

#ifdef _DEBUG
        // Check for memory leaks
        std::size_t leakCount = checkForLeaks();
        if (leakCount > 0) {
            std::cerr << "Warning: " << leakCount << " memory leaks detected!" << std::endl;

            // Dump leak information
            for (const auto& [ptr, record] : allocationRecords) {
                std::cerr << "  Leaked " << record.size << " bytes from "
                    << record.file << ":" << record.line << std::endl;
            }
        }
#endif

        // Clear all allocators
        frameAllocators.clear();
        for (auto& allocator : categoryAllocators) {
            allocator.reset();
        }
        customAllocators.clear();
        mainHeap.reset();
        debugHeap.reset();

        initialized = false;

        std::cout << "MemoryManager shut down." << std::endl;
    }

    void* MemoryManager::allocate(MemorySize size, MemoryCategory category, const MemorySize alignment,
                                  const AllocationFlags flags) {
        if (!initialized) {
            std::cerr << "Error: MemoryManager not initialized!" << std::endl;
            return nullptr;
        }

        // Select appropriate allocator
        IAllocator* allocator = selectAllocator(category, size, flags);
        if (!allocator) {
            handleAllocationFailure(size, category);
            return nullptr;
        }

        // Perform allocation
        void* ptr = allocator->allocate(size, alignment, flags);

        if (!ptr) {
            // Try to free some memory and retry
            if (performMemoryCleanup(size) >= size) {
                ptr = allocator->allocate(size, alignment, flags);
            }

            if (!ptr) {
                handleAllocationFailure(size, category);
                return nullptr;
            }
        }

        // Update global statistics
        globalStats.totalAllocated.fetch_add(size, std::memory_order_relaxed);
        globalStats.currentUsage.fetch_add(size, std::memory_order_relaxed);
        globalStats.allocationCount.fetch_add(1, std::memory_order_relaxed);
        globalStats.categoryUsage[static_cast<std::size_t>(category)].fetch_add(size, std::memory_order_relaxed);

        // Update peak usage
        const MemorySize current = globalStats.currentUsage.load(std::memory_order_relaxed);
        MemorySize peak = globalStats.peakUsage.load(std::memory_order_relaxed);
        while (current > peak && !globalStats.peakUsage.compare_exchange_weak(peak, current)) {
            // Keep trying
        }

#ifdef _DEBUG
        // Record allocation for leak detection
        if (config.enableLeakDetection) {
            recordAllocation(ptr, size, category, __FILE__, __LINE__);
        }
#endif

        return ptr;
    }

    void MemoryManager::deallocate(void* ptr, MemoryCategory category) {
        if (!ptr) return;

        if (!initialized) {
            std::cerr << "Error: MemoryManager not initialized!" << std::endl;
            return;
        }

#ifdef _DEBUG
        // Validate allocation
        if (config.enableLeakDetection) {
            validateAllocation(ptr);
            removeAllocationRecord(ptr);
        }
#endif

        // Find the allocator that owns this pointer
        IAllocator* allocator = nullptr;

        // Check category allocator first
        if (categoryAllocators[static_cast<std::size_t>(category)]) {
            if (categoryAllocators[static_cast<std::size_t>(category)]->owns(ptr)) {
                allocator = categoryAllocators[static_cast<std::size_t>(category)].get();
            }
        }

        if (!allocator && mainHeap->owns(ptr)) {
            allocator = mainHeap.get();
        }

        if (!allocator) {
            for (const auto& frame : frameAllocators) {
                if (frame.stackAllocator->owns(ptr)) {
                    allocator = frame.stackAllocator.get();
                    break;
                }
                if (frame.linearAllocator->owns(ptr)) {
                    allocator = frame.linearAllocator.get();
                    break;
                }
            }
        }

        if (!allocator) {
            std::cerr << "Error: Attempted to deallocate unknown pointer!" << std::endl;
            assert(false && "Invalid deallocation");
        }

        const MemorySize size = allocator->getAllocationSize(ptr);

        allocator->deallocate(ptr);

        if (size > 0) {
            globalStats.totalFreed.fetch_add(size, std::memory_order_relaxed);
            globalStats.currentUsage.fetch_sub(size, std::memory_order_relaxed);
            globalStats.freeCount.fetch_add(1, std::memory_order_relaxed);
            globalStats.categoryUsage[static_cast<std::size_t>(category)].fetch_sub(size, std::memory_order_relaxed);
        }
    }

    void* MemoryManager::reallocate(void* ptr, const MemorySize newSize,
                                    const MemoryCategory category,
                                    const MemorySize alignment) {
        if (!ptr) {
            return allocate(newSize, category, alignment);
        }

        if (newSize == 0) {
            deallocate(ptr, category);
            return nullptr;
        }

        // For simplicity, always allocate new and copy
        // In production, try to resize in-place if possible
        void* newPtr = allocate(newSize, category, alignment);
        if (!newPtr) {
            return nullptr;
        }

        // Find original size
        MemorySize oldSize = 0;
        for (const auto& allocator : categoryAllocators) {
            if (allocator && allocator->owns(ptr)) {
                oldSize = allocator->getAllocationSize(ptr);
                break;
            }
        }

        if (oldSize == 0 && mainHeap->owns(ptr)) {
            oldSize = mainHeap->getAllocationSize(ptr);
        }

        // Copy data
        if (oldSize > 0) {
            std::memcpy(newPtr, ptr, std::min(oldSize, newSize));
        }

        // Free old allocation
        deallocate(ptr, category);

        return newPtr;
    }

    StackAllocator& MemoryManager::getFrameStackAllocator() const {
        const std::uint8_t index = currentFrameIndex.load(std::memory_order_acquire);

        return *frameAllocators[index].stackAllocator;
    }

    LinearAllocator& MemoryManager::getFrameLinearAllocator() const {
        const std::uint8_t index = currentFrameIndex.load(std::memory_order_acquire);

        return *frameAllocators[index].linearAllocator;
    }

    void MemoryManager::beginFrame(std::uint64_t frameNumber) {
        // Advance to the next frame buffer
        const std::uint8_t currentIndex = currentFrameIndex.load(std::memory_order_acquire);
        const std::uint8_t nextIndex = (currentIndex + 1) % config.frameBufferCount;

        // Reset the allocators we're about to use (from N frames ago)
        frameAllocators[nextIndex].stackAllocator->reset();
        frameAllocators[nextIndex].linearAllocator->reset();
        frameAllocators[nextIndex].frameNumber = frameNumber;

        // Switch to the next frame
        currentFrameIndex.store(nextIndex, std::memory_order_release);
    }

    void MemoryManager::endFrame() {
        // Frame cleanup if needed
        // Most work is done in beginFrame for the next frame
    }

    IAllocator* MemoryManager::getAllocator(MemoryCategory category) const {
        if (category >= MemoryCategory::COUNT) {
            return nullptr;
        }

        return categoryAllocators[static_cast<std::size_t>(category)].get();
    }

    void MemoryManager::registerAllocator(MemoryCategory category,
        std::unique_ptr<IAllocator> allocator) {
        if (category >= MemoryCategory::COUNT) {
            std::cerr << "Error: Invalid memory category!" << std::endl;
            return;
        }

        categoryAllocators[static_cast<std::size_t>(category)] = std::move(allocator);
    }

    MemorySize MemoryManager::getTotalMemoryUsage() const {
        return globalStats.currentUsage.load(std::memory_order_acquire);
    }

    MemorySize MemoryManager::getCategoryMemoryUsage(MemoryCategory category) const {
        if (category >= MemoryCategory::COUNT) {
            return 0;
        }

        return globalStats.categoryUsage[static_cast<std::size_t>(category)].load(std::memory_order_acquire);
    }

    std::string MemoryManager::generateMemoryReport() const {
        std::stringstream report;

        report << "=== Memory Manager Report ===" << std::endl;
        report << std::endl;

        // Global statistics
        report << "Global Statistics:" << std::endl;
        report << "  Current Usage: " << (getTotalMemoryUsage() / (1024.0 * 1024.0)) << " MB" << std::endl;
        report << "  Peak Usage: " << (globalStats.peakUsage.load() / (1024.0 * 1024.0)) << " MB" << std::endl;
        report << "  Total Allocated: " << (globalStats.totalAllocated.load() / (1024.0 * 1024.0)) << " MB" <<
            std::endl;
        report << "  Total Freed: " << (globalStats.totalFreed.load() / (1024.0 * 1024.0)) << " MB" << std::endl;
        report << "  Allocation Count: " << globalStats.allocationCount.load() << std::endl;
        report << "  Free Count: " << globalStats.freeCount.load() << std::endl;
        report << "  Failed Allocations: " << globalStats.failedAllocations.load() << std::endl;
        report << std::endl;

        // Category breakdown
        report << "Category Usage:" << std::endl;

        for (std::size_t i = 0; i < static_cast<std::size_t>(MemoryCategory::COUNT); ++i) {
            MemorySize usage = globalStats.categoryUsage[i].load();
            if (usage > 0) {
                const char* categoryNames[] = {
                    "GENERAL", "RENDERING", "PHYSICS", "AUDIO", "GAMEPLAY",
                    "NETWORKING", "SCRIPTING", "UI", "WORLD", "ANIMATION",
                    "PARTICLES", "AI", "RESOURCES", "DEBUG"
                };
                report << "  " << std::setw(12) << categoryNames[i] << ": "
                    << std::setw(10) << (usage / (1024.0 * 1024.0)) << " MB" << std::endl;
            }
        }
        report << std::endl;

        // Allocator details
        report << "Allocator Details:" << std::endl;

        if (mainHeap) {
            report << "  Main Heap: "
                << (mainHeap->getUsedMemory() / (1024.0 * 1024.0)) << " / "
                << (mainHeap->getCapacity() / (1024.0 * 1024.0)) << " MB" << std::endl;
        }

        // Frame allocators
        std::uint8_t currentFrame = currentFrameIndex.load();
        for (std::uint8_t i = 0; i < config.frameBufferCount; ++i) {
            std::string marker = (i == currentFrame) ? " [CURRENT]" : "";
            report << "  Frame " << static_cast<int>(i) << " Stack" << marker << ": "
                << (frameAllocators[i].stackAllocator->getUsedMemory() / (1024.0 * 1024.0)) << " / "
                << (frameAllocators[i].stackAllocator->getCapacity() / (1024.0 * 1024.0)) << " MB" << std::endl;
            report << "  Frame " << static_cast<int>(i) << " Linear" << marker << ": "
                << (frameAllocators[i].linearAllocator->getUsedMemory() / (1024.0 * 1024.0)) << " / "
                << (frameAllocators[i].linearAllocator->getCapacity() / (1024.0 * 1024.0)) << " MB" << std::endl;
        }

        return report.str();
    }

    bool MemoryManager::dumpAllocations(const std::string& filename) const {
        std::ofstream file(filename);

        if (!file.is_open()) {
            return false;
        }

        file << generateMemoryReport();

#ifdef _DEBUG
        if (config.enableLeakDetection) {
            file << std::endl;
            file << "=== Active Allocations ===" << std::endl;

            std::shared_lock<std::shared_mutex> lock(recordsMutex);
            for (const auto& [ptr, record] : allocationRecords) {
                file << "Address: " << ptr
                    << ", Size: " << record.size
                    << ", Category: " << static_cast<int>(record.category)
                    << ", File: " << record.file
                    << ", Line: " << record.line
                    << std::endl;
            }
        }
#endif

        file.close();

        return true;
    }

    std::size_t MemoryManager::checkForLeaks() {
#ifdef _DEBUG
        if (config_.enableLeakDetection) {
            std::shared_lock<std::shared_mutex> lock(recordsMutex_);
            return allocationRecords_.size();
        }
#endif
        return 0;
    }

    void MemoryManager::registerMemoryPressureCallback(MemoryPressureCallback callback) {
        memoryPressureCallbacks.push_back(callback);
    }

    MemorySize MemoryManager::performMemoryCleanup(const MemorySize targetBytes) const {
        constexpr MemorySize freedBytes = 0;

        // Trigger memory pressure callbacks
        for (const auto& callback : memoryPressureCallbacks) {
            callback(getTotalMemoryUsage(), targetBytes);
        }

        // Reset non-critical allocators if needed
        // This is a simplified implementation

        return freedBytes;
    }

    void MemoryManager::getSystemMemoryInfo(MemorySize& totalPhysical,
                                            MemorySize& availablePhysical,
                                            MemorySize& totalVirtual,
                                            MemorySize& availableVirtual) {
#ifdef _WIN32
        MEMORYSTATUSEX memStatus;
        memStatus.dwLength = sizeof(memStatus);
        GlobalMemoryStatusEx(&memStatus);

        totalPhysical = memStatus.ullTotalPhys;
        availablePhysical = memStatus.ullAvailPhys;
        totalVirtual = memStatus.ullTotalVirtual;
        availableVirtual = memStatus.ullAvailVirtual;

#elif __APPLE__
        int mib[2];
        size_t length;

        // Get physical memory
        mib[0] = CTL_HW;
        mib[1] = HW_MEMSIZE;
        length = sizeof(totalPhysical);
        sysctl(mib, 2, &totalPhysical, &length, nullptr, 0);

        // Get available memory (approximation)
        vm_size_t page_size;
        vm_statistics64_data_t vm_stat;
        mach_msg_type_number_t host_size = sizeof(vm_stat) / sizeof(natural_t);

        host_page_size(mach_host_self(), &page_size);
        host_statistics64(mach_host_self(), HOST_VM_INFO64, reinterpret_cast<host_info64_t>(&vm_stat), &host_size);

        availablePhysical = (vm_stat.free_count + vm_stat.inactive_count) * page_size;

        // Virtual memory info not easily available on macOS
        totalVirtual = totalPhysical * 2; // Approximation
        availableVirtual = availablePhysical * 2;

#elif __linux__
        struct sysinfo memInfo;
        sysinfo (&memInfo);

        totalPhysical = memInfo.totalram * memInfo.mem_unit;
        availablePhysical = memInfo.freeram * memInfo.mem_unit;
        totalVirtual = memInfo.totalswap * memInfo.mem_unit + totalPhysical;
        availableVirtual = memInfo.freeswap * memInfo.mem_unit + availablePhysical;

#else
        // Unknown platform
        totalPhysical = 0;
        availablePhysical = 0;
        totalVirtual = 0;
        availableVirtual = 0;
#endif
    }

    IAllocator* MemoryManager::selectAllocator(MemoryCategory category, const MemorySize size, const AllocationFlags flags) const {
        // Check for a thread-local flag
        if (hasFlags(flags, AllocationFlags::THREAD_LOCAL)) {
            // Use frame stack for thread-local allocations
            return &getFrameLinearAllocator();
        }

        // Check for a temporary flag
        if (hasFlags(flags, AllocationFlags::TEMPORARY)) {
            // Use main heap for temporary allocations
            return &getFrameLinearAllocator();
        }

        // Check category-specific allocator
        if (category < MemoryCategory::COUNT) {
            if (IAllocator* categoryAllocator = categoryAllocators[static_cast<std::size_t>(category)].get()) {
                return categoryAllocator;
            }
        }

        //  Fall back to the main heap
        return mainHeap.get();
    }

    void MemoryManager::handleAllocationFailure(const MemorySize size, MemoryCategory category) {
        globalStats.failedAllocations.fetch_add(1, std::memory_order_relaxed);

        std::cerr << "Memory allocation failed: " << size << " bytes, category: "
            << static_cast<int>(category) << std::endl;

        // Try to recover by triggering cleanup
        if (const MemorySize freed = performMemoryCleanup(size); freed < size) {
            // Check if we're in a critical memory situation
            MemorySize available = 0;
            if (mainHeap) {
                available = mainHeap->getFreeMemory();
            }

            if (available < config.criticalMemoryThreshold) {
                std::cerr << "CRITICAL: System is out of memory!" << std::endl;

                // In production, this might trigger emergency measures
                // like unloading non-essential resources
            }
        }
    }

#ifdef _DEBUG
    void MemoryManager::recordAllocation(void* ptr, MemorySize size, MemoryCategory category,
                                         const char* file, int line) {
        AllocationRecord record;
        record.address = ptr;
        record.size = size;
        record.category = category;
        record.file = file;
        record.line = line;
        record.timestamp = std::chrono::steady_clock::now();
        record.threadId = std::this_thread::get_id();

        std::unique_lock<std::shared_mutex> lock(recordsMutex);
        allocationRecords[ptr] = record;
    }

    void MemoryManager::removeAllocationRecord(void* ptr) {
        std::unique_lock<std::shared_mutex> lock(recordsMutex);
        allocationRecords.erase(ptr);
    }

    void MemoryManager::validateAllocation(void* ptr) const {
        std::shared_lock<std::shared_mutex> lock(recordsMutex);

        auto it = allocationRecords.find(ptr);
        if (it == allocationRecords.end()) {
            std::cerr << "Error: Attempting to free untracked allocation!" << std::endl;
            assert(false && "Invalid free");
        }
    }
#endif
}
