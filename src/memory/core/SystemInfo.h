//
// Created by Andres Guerrero on 21-08-25.
//

#pragma once
#include <cstddef>
#include <thread>
#include <iostream>

#ifdef _WIN32
#include <windows.h>
#include <psapi.h>
#include <sysinfoapi.h>
#elif __APPLE__
#include <mach/mach.h>
#include <mach/mach_host.h>
#include <sys/sysctl.h>
#include <unistd.h>
#elif __linux__
#include <sys/sysinfo.h>
#include <unistd.h>
#include <fstream>
#include <string>
#endif

namespace engine::memory {
    /**
     * @brief System information for automatic hardware detection
     */
    struct SystemInfo {
        // Memory information
        std::size_t totalPhysicalMemory = 0; // Total RAM in bytes
        std::size_t availablePhysicalMemory = 0; // Available RAM in bytes
        std::size_t totalVirtualMemory = 0; // Total virtual memory
        std::size_t availableVirtualMemory = 0; // Available virtual memory

        // CPU information
        unsigned int physicalCores = 0; // Physical CPU cores
        unsigned int logicalCores = 0; // Logical cores (with HyperThreading)
        unsigned int cacheLineSize = 64; // CPU cache line size

        // Recommended limits based on system
        std::size_t recommendedHeapSize = 0; // Recommended main heap size
        std::size_t recommendedMaxMemory = 0; // Max memory to use (leaving room for OS)
        unsigned int recommendedThreadCount = 0; // Recommended worker threads
    };

    /**
     * @brief Detects system hardware capabilities
     */
    class SystemInfoDetector {
    public:
        /**
         * @brief Get current system information
         * @return SystemInfo structure with detected values
         */
        static SystemInfo detect() {
            SystemInfo info;

            // Detect CPU information
            detectCPUInfo(info);

            // Detect memory information
            detectMemoryInfo(info);

            // Calculate recommended limits
            calculateRecommendedLimits(info);

            return info;
        }

        /**
         * @brief Print system information to console
         */
        static void printSystemInfo(const SystemInfo& info) {
            std::cout << "\n=== SYSTEM INFORMATION ===" << std::endl;

            std::cout << "\nCPU:" << std::endl;
            std::cout << "  Physical Cores: " << info.physicalCores << std::endl;
            std::cout << "  Logical Cores: " << info.logicalCores << std::endl;
            std::cout << "  Cache Line Size: " << info.cacheLineSize << " bytes" << std::endl;

            std::cout << "\nMemory:" << std::endl;
            std::cout << "  Total RAM: " << formatBytes(info.totalPhysicalMemory) << std::endl;
            std::cout << "  Available RAM: " << formatBytes(info.availablePhysicalMemory) << std::endl;
            std::cout << "  Total Virtual: " << formatBytes(info.totalVirtualMemory) << std::endl;
            std::cout << "  Available Virtual: " << formatBytes(info.availableVirtualMemory) << std::endl;

            std::cout << "\nRecommended Limits:" << std::endl;
            std::cout << "  Heap Size: " << formatBytes(info.recommendedHeapSize) << std::endl;
            std::cout << "  Max Memory: " << formatBytes(info.recommendedMaxMemory) << std::endl;
            std::cout << "  Worker Threads: " << info.recommendedThreadCount << std::endl;
            std::cout << "=========================" << std::endl;
        }

    private:
        /**
         * @brief Detect CPU information
         */
        static void detectCPUInfo(SystemInfo& info) {
            // Get logical cores (easiest, cross-platform)
            info.logicalCores = std::thread::hardware_concurrency();
            if (info.logicalCores == 0) {
                info.logicalCores = 1; // Fallback
            }

#ifdef _WIN32
            // Windows CPU detection
            SYSTEM_INFO sysInfo;
            GetSystemInfo(&sysInfo);
            info.logicalCores = sysInfo.dwNumberOfProcessors;

            // Get physical cores (more complex on Windows)
            DWORD bufferSize = 0;
            GetLogicalProcessorInformation(nullptr, &bufferSize);

            if (bufferSize > 0) {
                std::vector<SYSTEM_LOGICAL_PROCESSOR_INFORMATION> buffer(
                    bufferSize / sizeof(SYSTEM_LOGICAL_PROCESSOR_INFORMATION)
                );

                if (GetLogicalProcessorInformation(buffer.data(), &bufferSize)) {
                    info.physicalCores = 0;
                    for (const auto& proc : buffer) {
                        if (proc.Relationship == RelationProcessorCore) {
                            info.physicalCores++;
                        }
                    }
                }
            }

            // Cache line size
            for (const auto& proc : buffer) {
                if (proc.Relationship == RelationCache && proc.Cache.Level == 1) {
                    info.cacheLineSize = proc.Cache.LineSize;
                    break;
                }
            }

#elif __APPLE__
            // macOS CPU detection
            size_t size = sizeof(info.physicalCores);
            sysctlbyname("hw.physicalcpu", &info.physicalCores, &size, nullptr, 0);

            size = sizeof(info.logicalCores);
            sysctlbyname("hw.logicalcpu", &info.logicalCores, &size, nullptr, 0);

            size = sizeof(info.cacheLineSize);
            sysctlbyname("hw.cachelinesize", &info.cacheLineSize, &size, nullptr, 0);

#elif __linux__
            // Linux CPU detection
            info.physicalCores = sysconf(_SC_NPROCESSORS_ONLN);

            // Try to get physical cores from /proc/cpuinfo
            std::ifstream cpuinfo("/proc/cpuinfo");
            std::string line;
            std::set<int> physicalIds;

            while (std::getline(cpuinfo, line)) {
                if (line.find("physical id") != std::string::npos) {
                    int id = std::stoi(line.substr(line.find(":") + 1));
                    physicalIds.insert(id);
                }
            }

            if (!physicalIds.empty()) {
                info.physicalCores = physicalIds.size();
            }

            // Cache line size
            long cacheLineSize = sysconf(_SC_LEVEL1_DCACHE_LINESIZE);
            if (cacheLineSize > 0) {
                info.cacheLineSize = static_cast<unsigned int>(cacheLineSize);
            }
#endif

            // Fallback for physical cores
            if (info.physicalCores == 0) {
                // Assume no HyperThreading if we can't detect
                info.physicalCores = info.logicalCores;
            }
        }

        /**
         * @brief Detect memory information
         */
        static void detectMemoryInfo(SystemInfo& info) {
#ifdef _WIN32
            // Windows memory detection
            MEMORYSTATUSEX memStatus;
            memStatus.dwLength = sizeof(memStatus);

            if (GlobalMemoryStatusEx(&memStatus)) {
                info.totalPhysicalMemory = memStatus.ullTotalPhys;
                info.availablePhysicalMemory = memStatus.ullAvailPhys;
                info.totalVirtualMemory = memStatus.ullTotalVirtual;
                info.availableVirtualMemory = memStatus.ullAvailVirtual;
            }

#elif __APPLE__
            // macOS memory detection
            int mib[2];
            size_t length;

            // Get physical memory
            mib[0] = CTL_HW;
            mib[1] = HW_MEMSIZE;
            length = sizeof(info.totalPhysicalMemory);
            sysctl(mib, 2, &info.totalPhysicalMemory, &length, nullptr, 0);

            // Get available memory
            vm_size_t page_size;
            vm_statistics64_data_t vm_stat;
            mach_msg_type_number_t host_size = sizeof(vm_stat) / sizeof(natural_t);

            host_page_size(mach_host_self(), &page_size);
            host_statistics64(mach_host_self(), HOST_VM_INFO64,
                              (host_info64_t)&vm_stat, &host_size);

            info.availablePhysicalMemory = (vm_stat.free_count +
                vm_stat.inactive_count) * page_size;

            // Virtual memory (approximate)
            info.totalVirtualMemory = info.totalPhysicalMemory * 2;
            info.availableVirtualMemory = info.availablePhysicalMemory * 2;

#elif __linux__
            // Linux memory detection
            struct sysinfo memInfo;
            if (sysinfo(&memInfo) == 0) {
                info.totalPhysicalMemory = memInfo.totalram * memInfo.mem_unit;
                info.availablePhysicalMemory = memInfo.freeram * memInfo.mem_unit;
                info.totalVirtualMemory = (memInfo.totalram + memInfo.totalswap) * memInfo.mem_unit;
                info.availableVirtualMemory = (memInfo.freeram + memInfo.freeswap) * memInfo.mem_unit;
            }
#endif
        }

        /**
         * @brief Calculate recommended limits based on detected hardware
         */
        static void calculateRecommendedLimits(SystemInfo& info) {
            // Recommended heap size (25% of total RAM, max 2GB for 32-bit safety)
            info.recommendedHeapSize = std::min(
                info.totalPhysicalMemory / 4,
                static_cast<std::size_t>(2ULL * 1024 * 1024 * 1024)
            );

            // Recommended max memory (75% of available RAM)
            info.recommendedMaxMemory = static_cast<std::size_t>(
                info.availablePhysicalMemory * 0.75
            );

            // Recommended thread count
            // Use physical cores for CPU-bound work, leave some for OS
            if (info.physicalCores > 2) {
                info.recommendedThreadCount = info.physicalCores - 1;
            }
            else {
                info.recommendedThreadCount = 1;
            }

            // Don't exceed logical cores
            info.recommendedThreadCount = std::min(
                info.recommendedThreadCount,
                info.logicalCores
            );
        }

        /**
         * @brief Format bytes to human-readable string
         */
        static std::string formatBytes(std::size_t bytes) {
            const char* units[] = {"B", "KB", "MB", "GB", "TB"};
            int unitIndex = 0;
            double size = static_cast<double>(bytes);

            while (size >= 1024.0 && unitIndex < 4) {
                size /= 1024.0;
                unitIndex++;
            }

            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%.2f %s", size, units[unitIndex]);
            return std::string(buffer);
        }
    };

    // ========================================================================
    // ENHANCED MEMORY MANAGER CONFIG WITH AUTO-DETECTION
    // ========================================================================

    /**
     * @brief Enhanced configuration that can auto-detect system limits
     */
    struct MemoryManagerAutoConfig : public MemoryManagerConfig {
        bool autoDetectLimits = true; // Auto-detect system capabilities
        float memoryUsagePercent = 0.5f; // Use 50% of available RAM
        float heapSizePercent = 0.25f; // Main heap as % of total allocation

        /**
         * @brief Configure based on system detection
         */
        void configureFromSystem() {
            std::cout << "\n=== MEMORY MANAGER CONFIGURATION ===" << std::endl;

            if (!autoDetectLimits) return;

            SystemInfo sysInfo = SystemInfoDetector::detect();

            // Calcular presupuesto total de memoria
            std::size_t totalBudget = static_cast<std::size_t>(
                sysInfo.availablePhysicalMemory * memoryUsagePercent
            );

            std::cout << "\n[MemoryManager] Auto-Configuration:" << std::endl;
            std::cout << "  Total RAM: " << (sysInfo.totalPhysicalMemory / (1024.0 * 1024.0 * 1024.0)) << " GB" <<
                std::endl;
            std::cout << "  Available RAM: " << (sysInfo.availablePhysicalMemory / (1024.0 * 1024.0 * 1024.0)) << " GB"
                << std::endl;
            std::cout << "  Memory Budget: " << (totalBudget / (1024.0 * 1024.0)) << " MB ("
                << (memoryUsagePercent * 100) << "% of available)" << std::endl;

            // Distribuir presupuesto de memoria
#ifdef __APPLE__
            // macOS tiene restricciones diferentes, ser más conservador
            std::cout << "  [macOS] Adjusting for platform limitations" << std::endl;

            // Limitar allocaciones individuales a 32MB máximo en macOS
            const MemorySize maxSingleAlloc = 32 * 1024 * 1024;

            mainHeapSize = std::min(mainHeapSize, maxSingleAlloc);
            debugHeapSize = std::min(debugHeapSize, static_cast<MemorySize>(8 * 1024 * 1024));
            frameStackSize = std::min(frameStackSize, static_cast<MemorySize>(4 * 1024 * 1024));
            frameLinearSize = std::min(frameLinearSize, static_cast<MemorySize>(4 * 1024 * 1024));
            renderingPoolSize = std::min(renderingPoolSize, static_cast<MemorySize>(16 * 1024 * 1024));
            physicsPoolSize = std::min(physicsPoolSize, static_cast<MemorySize>(8 * 1024 * 1024));
#else
            mainHeapSize = static_cast<std::size_t>(totalBudget * heapSizePercent);
            frameStackSize = static_cast<std::size_t>(totalBudget * 0.05f);
            frameLinearSize = static_cast<std::size_t>(totalBudget * 0.05f);
            renderingPoolSize = static_cast<std::size_t>(totalBudget * 0.20f);
            physicsPoolSize = static_cast<std::size_t>(totalBudget * 0.10f);
#endif
            audioRingBufferSize = static_cast<std::size_t>(totalBudget * 0.05f);
            networkBufferSize = static_cast<std::size_t>(totalBudget * 0.02f);

            // Ajustar frame buffers según CPU
            if (sysInfo.physicalCores >= 8) {
                frameBufferCount = 3;
            }
            else if (sysInfo.physicalCores >= 4) {
                frameBufferCount = 2;
            }
            else {
                frameBufferCount = 1;
            }

            // Establecer umbrales
            lowMemoryThreshold = static_cast<std::size_t>(totalBudget * 0.15f);
            criticalMemoryThreshold = static_cast<std::size_t>(totalBudget * 0.05f);

            std::cout << "  Configured heap: " << (mainHeapSize / (1024.0 * 1024.0)) << " MB" << std::endl;
            std::cout << "  Frame buffers: " << static_cast<int>(frameBufferCount) << std::endl;
            std::cout << "  CPU cores: " << sysInfo.physicalCores << " physical, "
                << sysInfo.logicalCores << " logical" << std::endl;
        }
    };

    // ========================================================================
    // THREAD POOL MANAGER (BONUS)
    // ========================================================================

    /**
     * @brief Simple thread pool for the engine
     */
    class ThreadPoolManager {
    public:
        static ThreadPoolManager& getInstance() {
            static ThreadPoolManager instance;
            return instance;
        }

        /**
         * @brief Initialize thread pool based on system
         */
        void initialize() {
            SystemInfo sysInfo = SystemInfoDetector::detect();

            // Create worker threads based on CPU
            workerThreadCount_ = sysInfo.recommendedThreadCount;

            std::cout << "\nThread Pool Configuration:" << std::endl;
            std::cout << "  Worker Threads: " << workerThreadCount_ << std::endl;
            std::cout << "  Physical Cores: " << sysInfo.physicalCores << std::endl;
            std::cout << "  Logical Cores: " << sysInfo.logicalCores << std::endl;

            // Set thread affinity for better cache usage (platform-specific)
#ifdef _WIN32
            // Windows thread affinity
            for (unsigned int i = 0; i < workerThreadCount_; ++i) {
                DWORD_PTR mask = 1ULL << i;
                // SetThreadAffinityMask for each worker thread
            }
#elif __linux__
            // Linux thread affinity
            for (unsigned int i = 0; i < workerThreadCount_; ++i) {
                cpu_set_t cpuset;
                CPU_ZERO(&cpuset);
                CPU_SET(i, &cpuset);
                // pthread_setaffinity_np for each worker thread
            }
#endif
        }

        unsigned int getWorkerCount() const { return workerThreadCount_; }

    private:
        unsigned int workerThreadCount_ = 1;
    };
} // namespace engine::memory
