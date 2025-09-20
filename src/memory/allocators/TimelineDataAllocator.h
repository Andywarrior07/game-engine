//
// Created by Andres Guerrero on 15-08-25.
//

#pragma once

#include "../allocators/LinearAllocator.h"

#include <chrono>
#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include <unordered_map>
#include <shared_mutex>
#include <memory>

namespace engine::memory {
    /**
     * @brief Specialized linear allocator for timeline data management
     * @details High-performance linear allocator optimized for timeline and temporal data storage.
     *          Designed for data that persists for the entire duration of a timeline, scene, or level.
     *          Provides efficient batch allocation, automatic lifecycle management, and optimized
     *          memory layout for temporal data structures.
     *
     * Key features:
     * - Timeline-scoped memory management
     * - Automatic reset on timeline transitions
     * - Optimized for temporal data patterns
     * - Batch allocation for related data
     * - Memory tagging and tracking
     * - Scene/level transition support
     *
     * Performance characteristics:
     * - O(1) allocation time
     * - Excellent cache locality for sequential access
     * - Zero fragmentation
     * - Automatic memory reclamation on timeline reset
     * - Optimal for read-heavy temporal data
     *
     * Ideal for:
     * - Timeline internal data structures
     * - Timestep calculation buffers
     * - Per-timeline configuration data
     * - Animation curve data
     * - Event sequence storage
     * - Level-persistent temporal state
     */
    class TimelineDataAllocator final : public IAllocator {
    public:
        /**
         * @brief Timeline lifecycle phases
         */
        enum class TimelinePhase : std::uint8_t {
            INITIALIZATION, ///< Timeline being set up
            ACTIVE, ///< Timeline running normally
            PAUSED, ///< Timeline paused
            TRANSITIONING, ///< Timeline transitioning between states
            CLEANUP, ///< Timeline being cleaned up
            INACTIVE ///< Timeline inactive
        };

        /**
         * @brief Data category for memory organization
         */
        enum class DataCategory : std::uint8_t {
            TIMELINE_CORE, ///< Core timeline data structures
            TIMESTEP_DATA, ///< Timestep calculation data
            EVENT_DATA, ///< Event and trigger data
            ANIMATION_DATA, ///< Animation and interpolation data
            CONFIG_DATA, ///< Configuration and settings
            TEMP_CALCULATIONS, ///< Temporary calculation buffers
            DEBUG_DATA, ///< Debug and profiling data
            COUNT ///< Number of categories
        };

        /**
         * @brief Timeline configuration
         */
        struct TimelineConfig {
            std::string name; ///< Timeline identifier
            MemorySize reservedSize; ///< Reserved memory for this timeline
            bool allowGrowth; ///< Allow memory to grow beyond reserved
            float growthFactor; ///< Growth factor when expanding
            std::chrono::milliseconds duration; ///< Expected timeline duration
            bool persistAcrossScenes; ///< Persist across scene changes
        };

        /**
         * @brief Allocation metadata for tracking
         */
        struct AllocationMetadata {
            MemorySize offset{}; ///< Offset in buffer
            MemorySize size{}; ///< Allocation size
            DataCategory category; ///< Data category
            std::string tag; ///< User-defined tag
            std::chrono::steady_clock::time_point timestamp; ///< Allocation time
            std::uint64_t allocationId{}; ///< Unique identifier
        };

        /**
         * @brief Construct timeline data allocator with LinearAllocator instance
         * @param linearAllocator Shared pointer to LinearAllocator instance
         * @param config Timeline configuration
         * @param name Debug name for the allocator
         */
        explicit TimelineDataAllocator(std::shared_ptr<LinearAllocator> linearAllocator,
                                       TimelineConfig config,
                                       const char* name = "TimelineDataAllocator");

        /**
         * @brief Destructor
         */
        ~TimelineDataAllocator() override {
#ifdef _DEBUG
            generateFinalReport();
#endif
        }

        // Disable copy semantics
        TimelineDataAllocator(const TimelineDataAllocator&) = delete;
        TimelineDataAllocator& operator=(const TimelineDataAllocator&) = delete;

        /**
         * @brief Move constructor
         */
        TimelineDataAllocator(TimelineDataAllocator&& other) noexcept
            : linearAllocator_(std::move(other.linearAllocator_))
              , config_(std::move(other.config_))
              , currentPhase_(other.currentPhase_)
              , timelineStartTime_(other.timelineStartTime_)
              , nextAllocationId_(other.nextAllocationId_.load())
              , totalResetsCount_(other.totalResetsCount_)
              , name_(other.name_)
              , allocations_(std::move(other.allocations_))
              , categoryUsage_(other.categoryUsage_) {
            other.currentPhase_ = TimelinePhase::INACTIVE;
            other.nextAllocationId_ = 1;
            other.totalResetsCount_ = 0;
        }

        /**
         * @brief Move assignment operator
         */
        TimelineDataAllocator& operator=(TimelineDataAllocator&& other) noexcept;

        /**
         * @brief Allocate memory with timeline-specific metadata
         * @param size Size of allocation
         * @param alignment Memory alignment
         * @param flags Allocation flags
         * @return Pointer to allocated memory
         */
        void* allocate(MemorySize size, MemorySize alignment = DEFAULT_ALIGNMENT,
                       AllocationFlags flags = AllocationFlags::NONE) override;

        /**
         * @brief Deallocate memory (no-op for linear allocator)
         * @param ptr Pointer to deallocate
         */
        void deallocate(void* ptr) override {
            linearAllocator_->deallocate(ptr);
        }

        /**
         * @brief Get allocator capacity
         * @return Total capacity in bytes
         */
        MemorySize getCapacity() const override {
            return linearAllocator_->getCapacity();
        }

        /**
         * @brief Get used memory
         * @return Used memory in bytes
         */
        MemorySize getUsedMemory() const override {
            return linearAllocator_->getUsedMemory();
        }

        /**
         * @brief Reset allocator
         */
        void reset() override;

        /**
         * @brief Check if allocator owns pointer
         * @param ptr Pointer to check
         * @return True if owned
         */
        bool owns(const void* ptr) const override {
            return linearAllocator_->owns(ptr);
        }

        /**
         * @brief Get allocation size
         * @param ptr Pointer to check
         * @return Size of allocation
         */
        MemorySize getAllocationSize(const void* ptr) const override {
            return linearAllocator_->getAllocationSize(ptr);
        }

        /**
         * @brief Get allocator name
         * @return Name string
         */
        const char* getName() const override {
            return name_;
        }

        /**
         * @brief Allocate memory with specific data category
         * @param size Size of allocation
         * @param category Data category for organization
         * @param tag Optional user tag for debugging
         * @param alignment Memory alignment
         * @param flags Allocation flags
         * @return Pointer to allocated memory
         */
        void* allocateWithCategory(MemorySize size, DataCategory category, const std::string& tag = "",
                                   MemorySize alignment = DEFAULT_ALIGNMENT,
                                   AllocationFlags flags = AllocationFlags::NONE);

        /**
         * @brief Batch allocate related data structures
         * @param allocations Vector of {size, category, tag} tuples
         * @param alignment Memory alignment for all allocations
         * @param flags Allocation flags
         * @return Vector of allocated pointers (empty on failure)
         */
        std::vector<void*> batchAllocate(
            const std::vector<std::tuple<MemorySize, DataCategory, std::string>>& allocations,
            MemorySize alignment = DEFAULT_ALIGNMENT,
            AllocationFlags flags = AllocationFlags::NONE);

        /**
         * @brief Reset timeline data for new timeline
         * @param newConfig New timeline configuration
         */
        void resetForNewTimeline(const TimelineConfig& newConfig);

        /**
         * @brief Set timeline phase
         * @param phase New timeline phase
         */
        void setTimelinePhase(TimelinePhase phase);

        /**
         * @brief Get current timeline phase
         */
        TimelinePhase getTimelinePhase() const {
            return currentPhase_;
        }

        /**
         * @brief Get timeline configuration
         */
        const TimelineConfig& getTimelineConfig() const {
            return config_;
        }

        /**
         * @brief Get free memory
         * @return Free memory in bytes
         */
        MemorySize getFreeMemory() const override {
            return getCapacity() - getUsedMemory();
        }

        /**
         * @brief Get memory usage by category
         * @param category Data category
         * @return Bytes used by category
         */
        MemorySize getCategoryUsage(DataCategory category) const {
            if (category >= DataCategory::COUNT) return 0;

            return categoryUsage_[static_cast<std::size_t>(category)];
        }

        /**
         * @brief Get allocation count by category
         * @param category Data category
         * @return Number of allocations in category
         */
        std::size_t getCategoryAllocationCount(DataCategory category) const;

        /**
         * @brief Get timeline runtime duration
         */
        std::chrono::milliseconds getTimelineRuntime() const {
            const auto now = std::chrono::steady_clock::now();

            return std::chrono::duration_cast<std::chrono::milliseconds>(now - timelineStartTime_);
        }

        /**
         * @brief Generate detailed timeline memory report
         */
        std::string generateTimelineReport() const;

        /**
         * @brief Get memory utilization ratio
         */
        float getUtilization() const {
            const MemorySize capacity = getCapacity();

            if (capacity == 0) return 0.0f;

            return static_cast<float>(getUsedMemory()) / static_cast<float>(capacity);
        }

        /**
         * @brief Find allocations by tag
         * @param tag Tag to search for
         * @return Vector of pointers with matching tag
         */
        std::vector<void*> findAllocationsByTag(const std::string& tag) const;

        /**
         * @brief Get underlying LinearAllocator
         * @return Shared pointer to LinearAllocator
         */
        std::shared_ptr<LinearAllocator> getLinearAllocator() const {
            return linearAllocator_;
        }

    private:
        std::shared_ptr<LinearAllocator> linearAllocator_; ///< Underlying linear allocator
        TimelineConfig config_; ///< Timeline configuration
        TimelinePhase currentPhase_; ///< Current timeline phase
        std::chrono::steady_clock::time_point timelineStartTime_; ///< Timeline start time
        std::atomic<std::uint64_t> nextAllocationId_; ///< Next allocation ID
        std::uint32_t totalResetsCount_; ///< Number of timeline resets
        const char* name_; ///< Allocator name

        // Allocation tracking
        std::unordered_map<void*, AllocationMetadata> allocations_; ///< Active allocations
        mutable std::shared_mutex metadataMutex_; ///< Mutex for metadata access
        std::array<MemorySize, static_cast<std::size_t>(DataCategory::COUNT)> categoryUsage_{}; ///< Usage by category

        /**
         * @brief Track allocation metadata
         */
        void trackAllocation(void* ptr, MemorySize size, DataCategory category, const std::string& tag);

        /**
         * @brief Attempt to grow allocator capacity
         */
        bool attemptGrowth(MemorySize additionalSize) const;

        /**
         * @brief Reserve memory for critical categories
         */
        void reserveMemoryForCategories();

        /**
         * @brief Generate final report on destruction
         */
        void generateFinalReport() const;

        /**
         * @brief Get string representation of timeline phase
         */
        static const char* getPhaseString(TimelinePhase phase);

        /**
         * @brief Get string representation of data category
         */
        static const char* getCategoryString(DataCategory category);
    };

    /**
     * @brief RAII helper for timeline phase management
     */
    class ScopedTimelinePhase {
    public:
        ScopedTimelinePhase(TimelineDataAllocator& allocator, TimelineDataAllocator::TimelinePhase phase)
            : allocator_(allocator)
              , previousPhase_(allocator.getTimelinePhase()) {
            allocator_.setTimelinePhase(phase);
        }

        ~ScopedTimelinePhase() {
            allocator_.setTimelinePhase(previousPhase_);
        }

        ScopedTimelinePhase(const ScopedTimelinePhase&) = delete;
        ScopedTimelinePhase& operator=(const ScopedTimelinePhase&) = delete;
        ScopedTimelinePhase(ScopedTimelinePhase&&) = delete;
        ScopedTimelinePhase& operator=(ScopedTimelinePhase&&) = delete;

    private:
        TimelineDataAllocator& allocator_;
        TimelineDataAllocator::TimelinePhase previousPhase_;
    };

    /**
     * @brief Template helper for typed timeline allocations
     */
    template <typename T>
    class TimelineAllocator {
    public:
        using value_type = T;

        explicit TimelineAllocator(TimelineDataAllocator& allocator,
                                   const TimelineDataAllocator::DataCategory category =
                                       TimelineDataAllocator::DataCategory::TIMELINE_CORE)
            : allocator_(allocator), category_(category) {
        }

        template <typename U>
        explicit TimelineAllocator(const TimelineAllocator<U>& other) noexcept
            : allocator_(other.allocator_), category_(other.category_) {
        }

        T* allocate(std::size_t n, const std::string& tag = "") {
            MemorySize size = sizeof(T) * n;
            void* ptr = allocator_.allocateWithCategory(size, category_, tag, alignof(T));
            return static_cast<T*>(ptr);
        }

        void deallocate(T* ptr, std::size_t n) {
            // TimelineDataAllocator doesn't support individual deallocation
            // Memory is reclaimed on timeline reset
            (void)ptr;
            (void)n; // Suppress unused warnings
        }

        template <typename U>
        bool operator==(const TimelineAllocator<U>& other) const noexcept {
            return &allocator_ == &other.allocator_ && category_ == other.category_;
        }

        template <typename U>
        bool operator!=(const TimelineAllocator<U>& other) const noexcept {
            return !(*this == other);
        }

    private:
        TimelineDataAllocator& allocator_;
        TimelineDataAllocator::DataCategory category_;

        template <typename U>
        friend class TimelineAllocator;
    };

    /**
     * @brief Convenience aliases for common timeline data structures
     */
    template <typename T>
    using TimelineVector = std::vector<T, TimelineAllocator<T>>;

    template <typename Key, typename Value>
    using TimelineMap = std::unordered_map<Key, Value, std::hash<Key>, std::equal_to<Key>,
                                           TimelineAllocator<std::pair<const Key, Value>>>;
}
