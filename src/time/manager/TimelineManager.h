/**
 * @file TimelineManager.h
 * @brief Multiple timeline orchestration and lifecycle management
 * @details Manages creation, destruction, and coordination of multiple timelines.
 *          Provides registration, lookup, and synchronization services while
 *          managing timeline lifecycles tied to game scenes and levels.
 *
 * @author Andres Guerrero
 * @date 2025-09-19
 */

#pragma once

#include "../core/Timeline.h"
#include "../core/ITimeBackend.h"
#include "../core/TimeConfig.h"
#include "../timelines/RealTimeline.h"
#include "../timelines/GameTimeline.h"
#include "../timelines/UITimeline.h"
#include "../timelines/PhysicsTimeline.h"
#include "../timelines/AudioTimeline.h"
#include "../timelines/CustomTimeline.h"
#include "../profiling/TimeStats.h"

#include "../../memory/MemorySystem.h"
#include "../../memory/manager/MemoryManager.h"

#include <unordered_map>
#include <shared_mutex>
#include <vector>
#include <string>
#include <memory>
#include <optional>
#include <functional>

namespace engine::time {
    /**
     * @brief Configuration for creating new timelines
     */
    struct TimelineCreationConfig : TimelineConfigBase {
        std::string name{"Unnamed"}; ///< Timeline name
        bool persistent{false}; ///< Survives scene changes
        std::function<void(TimelineID)> onTimelineCreated; ///< Creation callback
        std::function<void(TimelineID)> onTimelineDestroyed; ///< Destruction callback
    };

    /**
     * @brief Registry configuration for timeline system
     */
    struct TimelineRegistryConfig {
        std::vector<TimelineCreationConfig> predefinedTimelines; ///< Pre-created timelines
        std::size_t maxRuntimeTimelines{16}; ///< Max dynamic timelines
    };

    /**
     * @brief Manages multiple timelines and their lifecycle
     * @details Central orchestrator for all timeline instances in the time system.
     *          Handles creation, destruction, registration, and synchronization of
     *          timelines while managing their lifecycle across game states.
     *
     * Key responsibilities:
     * - Timeline factory and registration
     * - Lifecycle management (scene transitions)
     * - Coordination with TimestepManager
     * - Inter-timeline synchronization
     * - Resource management and cleanup
     * - Performance monitoring
     *
     * Thread-safety:
     * - Multiple readers, single writer for timeline access
     * - Atomic operations for hot paths
     * - Lock-free timeline queries where possible
     *
     * @note Does NOT update timelines directly - that's TimestepManager's job.
     *       Focuses on lifecycle, registration, and coordination.
     */
    class TimelineManager {
    public:
        /**
         * @brief Timeline registry entry
         */
        struct TimelineEntry {
            std::shared_ptr<Timeline> timeline; ///< Timeline instance
            TimelineCreationConfig config; ///< Creation config
            TimeStamp createdAt; ///< Creation timestamp
            TimeStamp lastUpdateAt; ///< Last update time
            std::uint32_t updatePriority; ///< Update order
            bool persistent; ///< Survives scene changes
            bool autoUpdate; ///< Update automatically
        };

        /**
         * @brief Timeline group for batch operations
         */
        struct TimelineGroup {
            std::string name; ///< Group identifier
            std::vector<TimelineID> timelines; ///< Timeline IDs in group
            bool pauseAllowed; ///< Can pause as group
            bool syncEnabled; ///< Synchronize updates
        };

        /**
         * @brief Constructor
         * @param backend Platform time backend
         * @param memoryManager Engine memory manager (optional)
         */
        explicit TimelineManager(
                ITimeBackend& backend,
                engine::memory::MemoryManager* memoryManager = nullptr
                );

        /**
         * @brief Destructor
         */
        ~TimelineManager();

        // Delete copy operations
        TimelineManager(const TimelineManager&) = delete;
        TimelineManager& operator=(const TimelineManager&) = delete;

        // =============================================================================
        // Initialization and Lifecycle
        // =============================================================================

        /**
         * @brief Initialize timeline manager
         * @param config Timeline registry configuration
         * @return True if initialization successful
         */
        bool initialize(const TimelineRegistryConfig& config);

        /**
         * @brief Shutdown timeline manager
         * @details Stops and destroys all timelines, clears all registrations
         */
        void shutdown();

        /**
         * @brief Check if manager is initialized
         * @return True if initialized
         */
        [[nodiscard]] bool isInitialized() const noexcept {
            return isInitialized_.load(std::memory_order_acquire);
        }

        // =============================================================================
        // Timeline Creation and Registration
        // =============================================================================

        /**
         * @brief Create and register a new timeline
         * @param config Timeline creation configuration
         * @return Timeline ID or INVALID_TIMELINE_ID on failure
         */
        TimelineID createTimeline(const TimelineCreationConfig& config);

        /**
         * @brief Destroy a timeline
         * @param id Timeline ID to destroy
         * @return True if timeline was destroyed
         */
        bool destroyTimeline(TimelineID id);

        /**
         * @brief Get timeline by ID
         * @param id Timeline ID
         * @return Timeline pointer or nullptr
         */
        [[nodiscard]] std::shared_ptr<Timeline> getTimeline(TimelineID id) const;

        /**
         * @brief Get timeline by type
         * @param type Timeline type
         * @return First timeline of type or nullptr
         */
        [[nodiscard]] std::shared_ptr<Timeline> getTimelineByType(TimelineType type) const;

        /**
         * @brief Get all timelines of a type
         * @param type Timeline type
         * @return Vector of timeline pointers
         */
        [[nodiscard]] std::vector<std::shared_ptr<Timeline>> getTimelinesByType(TimelineType type) const;

        /**
         * @brief Get all registered timelines
         * @return Vector of all timeline pointers
         */
        [[nodiscard]] std::vector<std::shared_ptr<Timeline>> getAllTimelines() const;

        // =============================================================================
        // Timeline Control
        // =============================================================================

        /**
         * @brief Pause all timelines
         */
        void pauseAllTimelines();

        /**
         * @brief Resume all timelines
         */
        void resumeAllTimelines();

        /**
         * @brief Stop all timelines
         */
        void stopAllTimelines();

        /**
         * @brief Start all timelines
         */
        void startAllTimelines();

        // =============================================================================
        // Timeline Groups
        // =============================================================================

        /**
         * @brief Create timeline group
         * @param name Group name
         * @param timelineIds Timeline IDs to include
         * @return True if group created
         */
        bool createGroup(
                const std::string& name,
                const std::vector<TimelineID>& timelineIds
                );

        /**
         * @brief Destroy timeline group
         * @param groupName Group to destroy
         * @return True if group was destroyed
         */
        bool destroyGroup(const std::string& groupName);

        /**
         * @brief Add timeline to group
         * @param groupName Group name
         * @param timelineId Timeline to add
         * @return True if added successfully
         */
        bool addToGroup(const std::string& groupName, TimelineID timelineId);

        /**
         * @brief Remove timeline from group
         * @param groupName Group name
         * @param timelineId Timeline to remove
         * @return True if removed successfully
         */
        bool removeFromGroup(const std::string& groupName, TimelineID timelineId);

        /**
         * @brief Pause timeline group
         * @param groupName Group to pause
         */
        void pauseGroup(const std::string& groupName);

        /**
         * @brief Resume timeline group
         * @param groupName Group to resume
         */
        void resumeGroup(const std::string& groupName);

        /**
         * @brief Get group info
         * @param groupName Group name
         * @return Group info or nullopt
         */
        [[nodiscard]] std::optional<TimelineGroup> getGroup(const std::string& groupName) const;

        // =============================================================================
        // Scene Management
        // =============================================================================

        /**
         * @brief Handle scene transition
         * @param fromScene Previous scene identifier
         * @param toScene New scene identifier
         */
        void onSceneTransition(const std::string& fromScene, const std::string& toScene);

        /**
         * @brief Mark timeline as persistent
         * @param id Timeline ID
         * @param persistent Persistent flag
         * @return True if set successfully
         */
        bool setTimelinePersistent(TimelineID id, bool persistent);

        // =============================================================================
        // Synchronization
        // =============================================================================

        /**
         * @brief Synchronize two timelines
         * @param primaryId Primary timeline ID
         * @param secondaryId Secondary timeline ID
         * @param offset Time offset between timelines
         * @return True if synchronization established
         */
        bool synchronizeTimelines(
                TimelineID primaryId,
                TimelineID secondaryId,
                Duration offset = Duration::zero()
                ) const;

        // =============================================================================
        // Statistics and Monitoring
        // =============================================================================

        /**
         * @brief Get timeline count
         * @return Total number of timelines
         */
        [[nodiscard]] std::size_t getTimelineCount() const;

        /**
         * @brief Get active timeline count
         * @return Number of running timelines
         */
        [[nodiscard]] std::size_t getActiveTimelineCount() const;

        /**
         * @brief Get timeline statistics
         * @param id Timeline ID
         * @return Timeline statistics or nullopt
         */
        [[nodiscard]] std::optional<TimelineStats> getTimelineStats(TimelineID id) const;

        /**
         * @brief Generate manager report
         * @return Formatted report string
         */
        [[nodiscard]] std::string generateReport() const;

    private:
        // =============================================================================
        // Private Members
        // =============================================================================

        ITimeBackend& backend_; ///< Platform backend
        memory::MemoryManager* memoryManager_; ///< Memory manager
        TimelineRegistryConfig registryConfig_; ///< Registry config

        mutable std::shared_mutex timelinesMutex_; ///< Timeline access mutex
        std::unordered_map<TimelineID, TimelineEntry> timelines_; ///< All timelines
        std::unordered_map<TimelineType, std::vector<TimelineID>> timelinesByType_; ///< Type index

        mutable std::shared_mutex groupsMutex_; ///< Group access mutex
        std::unordered_map<std::string, TimelineGroup> timelineGroups_; ///< Timeline groups

        std::atomic<TimelineID> nextTimelineId_; ///< ID generator
        std::atomic<bool> isInitialized_; ///< Initialization state

        std::unordered_map<TimelineID, TimelineStats> timelineStats_; ///< Statistics

        // =============================================================================
        // Private Methods
        // =============================================================================

        /**
         * @brief Create timeline instance based on type
         * @param config Timeline creation configuration
         * @return Timeline instance or nullptr on failure
         */
        std::shared_ptr<Timeline> createTimelineInstance(const TimelineCreationConfig& config) const;

        /**
         * @brief Create RealTimeline instance
         * @param config Creation configuration
         * @return RealTimeline instance
         */
        std::shared_ptr<RealTimeline> createRealTimeline(const TimelineCreationConfig& config) const;

        /**
         * @brief Create GameTimeline instance
         * @param config Creation configuration
         * @return GameTimeline instance
         */
        std::shared_ptr<GameTimeline> createGameTimeline(const TimelineCreationConfig& config) const;

        /**
         * @brief Create UITimeline instance
         * @param config Creation configuration
         * @return UITimeline instance
         */
        std::shared_ptr<UITimeline> createUITimeline(const TimelineCreationConfig& config) const;

        /**
         * @brief Create PhysicsTimeline instance
         * @param config Creation configuration
         * @return PhysicsTimeline instance
         */
        std::shared_ptr<PhysicsTimeline> createPhysicsTimeline(const TimelineCreationConfig& config) const;

        /**
         * @brief Create AudioTimeline instance
         * @param config Creation configuration
         * @return AudioTimeline instance
         */
        std::shared_ptr<AudioTimeline> createAudioTimeline(const TimelineCreationConfig& config) const;

        /**
         * @brief Create CustomTimeline instance
         * @param config Creation configuration
         * @return CustomTimeline instance
         */
        std::shared_ptr<CustomTimeline> createCustomTimeline(const TimelineCreationConfig& config) const;

        /**
         * @brief Register timeline in manager
         * @param timeline Timeline to register
         * @param config Creation configuration
         * @return Assigned timeline ID
         */
        TimelineID registerTimeline(
                const std::shared_ptr<Timeline>& timeline,
                const TimelineCreationConfig& config
                );

        /**
         * @brief Unregister timeline from manager
         * @param id Timeline ID to unregister
         */
        void unregisterTimeline(TimelineID id);

        /**
         * @brief Get default priority for timeline type
         * @param type Timeline type
         * @return Default priority value
         */
        static std::uint32_t getDefaultPriority(TimelineType type);

        /**
         * @brief Update timeline statistics
         * @param id Timeline ID
         * @param result Update result
         */
        void updateTimelineStats(TimelineID id, const TimelineUpdateResult& result);

        static std::string getTimelineName(TimelineType type);
        static std::string getTimelineStateName(TimelineState state);
    };
} // namespace engine::time
