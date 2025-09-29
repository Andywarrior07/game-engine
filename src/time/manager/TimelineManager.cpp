/**
 * @file TimelineManager.cpp
 * @brief Implementation of TimelineManager - Part 1
 * @details Constructor, initialization, and timeline creation
 *
 * @author Andres Guerrero
 * @date 2025-09-19
 */

#include "TimelineManager.h"

#include <algorithm>
#include <ranges>
#include <sstream>

namespace engine::time {
    TimelineManager::TimelineManager(
            ITimeBackend& backend,
            memory::MemoryManager* memoryManager
            ) :
        backend_(backend)
        , memoryManager_(memoryManager)
        , nextTimelineId_(0)
        , isInitialized_(false) {}

    TimelineManager::~TimelineManager() {
        shutdown();
    }

    bool TimelineManager::initialize(const TimelineRegistryConfig& config) {
        if (isInitialized_.load(std::memory_order_acquire)) {
            return false;
        }

        registryConfig_ = config;

        // Create predefined timelines
        for (const auto& tlConfig : config.predefinedTimelines) {
            if (const TimelineID id = createTimeline(tlConfig); id == constants::INVALID_TIMELINE_ID) {
                // Failed to create predefined timeline - critical error
                shutdown();
                return false;
            }
        }

        // Reserve space for runtime timelines
        {
            std::unique_lock lock(timelinesMutex_);
            timelines_.reserve(
                    config.predefinedTimelines.size() +
                    config.maxRuntimeTimelines
                    );
        }

        isInitialized_.store(true, std::memory_order_release);
        return true;
    }

    void TimelineManager::shutdown() {
        if (!isInitialized_.load(std::memory_order_acquire)) {
            return;
        }

        // Stop all timelines
        stopAllTimelines();

        // Clear registrations
        {
            std::unique_lock lock(timelinesMutex_);
            timelines_.clear();
            timelinesByType_.clear();
            timelineStats_.clear();
        }

        {
            std::unique_lock lock(groupsMutex_);
            timelineGroups_.clear();
        }

        isInitialized_.store(false, std::memory_order_release);
    }

    // =============================================================================
    // Timeline Creation
    // =============================================================================

    TimelineID TimelineManager::createTimeline(const TimelineCreationConfig& config) {
        if (!isInitialized_.load(std::memory_order_acquire)) {
            return constants::INVALID_TIMELINE_ID;
        }

        // Create appropriate timeline instance
        const auto timeline = createTimelineInstance(config);
        if (!timeline) {
            return constants::INVALID_TIMELINE_ID;
        }

        // Initialize timeline
        if (!timeline->initialize(config)) {
            return constants::INVALID_TIMELINE_ID;
        }

        // Register timeline
        const TimelineID id = registerTimeline(timeline, config);

        // Fire creation callback if provided
        if (config.onTimelineCreated) {
            config.onTimelineCreated(id);
        }

        return id;
    }

    std::shared_ptr<Timeline> TimelineManager::createTimelineInstance(
            const TimelineCreationConfig& config
            ) const {
        switch (config.type) {
            case TimelineType::REAL_TIME:
                return createRealTimeline(config);

            case TimelineType::GAME_TIME:
                return createGameTimeline(config);

            case TimelineType::UI_TIME:
                return createUITimeline(config);

            case TimelineType::PHYSICS_TIME:
                return createPhysicsTimeline(config);

            case TimelineType::AUDIO_TIME:
                return createAudioTimeline(config);

            case TimelineType::CUSTOM_1:
            case TimelineType::CUSTOM_2:
            case TimelineType::CUSTOM_3:
                return createCustomTimeline(config);

            default:
                return nullptr;
        }
    }

    std::shared_ptr<RealTimeline> TimelineManager::createRealTimeline(
            const TimelineCreationConfig& config
            ) const {
        return std::make_shared<RealTimeline>(backend_, config.name);
    }

    std::shared_ptr<GameTimeline> TimelineManager::createGameTimeline(
            const TimelineCreationConfig& config
            ) const {
        return std::make_shared<GameTimeline>(backend_, config.name);
    }

    std::shared_ptr<UITimeline> TimelineManager::createUITimeline(
            const TimelineCreationConfig& config
            ) const {
        return std::make_shared<UITimeline>(backend_, config.name);
    }

    std::shared_ptr<PhysicsTimeline> TimelineManager::createPhysicsTimeline(
            const TimelineCreationConfig& config
            ) const {
        return std::make_shared<PhysicsTimeline>(backend_, config.name);
    }

    std::shared_ptr<AudioTimeline> TimelineManager::createAudioTimeline(
            const TimelineCreationConfig& config
            ) const {
        return std::make_shared<AudioTimeline>(backend_, config.name);
    }

    std::shared_ptr<CustomTimeline> TimelineManager::createCustomTimeline(
            const TimelineCreationConfig& config
            ) const {
        CustomTimeline::CustomConfig customConfig;
        customConfig.identifier = config.name;

        return std::make_shared<CustomTimeline>(&backend_, customConfig, config.name);
    }

    TimelineID TimelineManager::registerTimeline(
            const std::shared_ptr<Timeline>& timeline,
            const TimelineCreationConfig& config
            ) {
        std::unique_lock lock(timelinesMutex_);

        const TimelineID id = nextTimelineId_.fetch_add(1, std::memory_order_relaxed);

        const TimelineEntry entry{
                        .timeline = timeline,
                        .config = config,
                        .createdAt = backend_.now(),
                        .lastUpdateAt = backend_.now(),
                        .updatePriority = getDefaultPriority(timeline->getType()),
                        .persistent = config.persistent,
                        .autoUpdate = true
                };

        timelines_[id] = entry;
        timelinesByType_[timeline->getType()].push_back(id);

        // Initialize statistics
        timelineStats_[id] = TimelineStats{};
        timelineStats_[id].id = id;

        return id;
    }

    void TimelineManager::unregisterTimeline(const TimelineID id) {
        std::unique_lock lock(timelinesMutex_);

        const auto it = timelines_.find(id);
        if (it == timelines_.end()) {
            return;
        }

        // Guardar el nombre del timeline antes de borrarlo
        const std::string timelineName = it->second.timeline->getName();

        // Remove from type map
        const auto type = it->second.timeline->getType();
        auto& typeList = timelinesByType_[type];
        std::erase(typeList, id);

        // Remove statistics
        timelineStats_.erase(id);

        // Remove timeline entry
        timelines_.erase(it);

        // Cleanup timeline allocator
        if (memoryManager_) {
            memoryManager_->destroyTimeline(timelineName);
        }
    }

    std::uint32_t TimelineManager::getDefaultPriority(const TimelineType type) {
        // Lower numbers = higher priority
        switch (type) {
            case TimelineType::REAL_TIME:
                return 0; // Highest priority
            case TimelineType::PHYSICS_TIME:
                return 10; // Physics before game logic
            case TimelineType::GAME_TIME:
                return 20; // Core gameplay
            case TimelineType::AUDIO_TIME:
                return 30; // Audio sync
            case TimelineType::UI_TIME:
                return 40; // UI last
            case TimelineType::CUSTOM_1:
            case TimelineType::CUSTOM_2:
            case TimelineType::CUSTOM_3:
                return 50; // Custom timelines
            default:
                return 100; // Lowest priority
        }
    }

    // =============================================================================
    // Timeline Destruction
    // =============================================================================

    bool TimelineManager::destroyTimeline(const TimelineID id) {
        std::unique_lock lock(timelinesMutex_);

        const auto it = timelines_.find(id);
        if (it == timelines_.end()) {
            return false;
        }

        // Fire destruction callback if provided
        if (it->second.config.onTimelineDestroyed) {
            it->second.config.onTimelineDestroyed(id);
        }

        // Stop and destroy the timeline
        it->second.timeline->stop();
        it->second.timeline->destroy();

        // Unregister (removes from all maps)
        lock.unlock();
        unregisterTimeline(id);

        return true;
    }

    // =============================================================================
    // Timeline Access
    // =============================================================================

    std::shared_ptr<Timeline> TimelineManager::getTimeline(const TimelineID id) const {
        std::shared_lock lock(timelinesMutex_);

        const auto it = timelines_.find(id);
        return (it != timelines_.end()) ? it->second.timeline : nullptr;
    }

    std::shared_ptr<Timeline> TimelineManager::getTimelineByType(const TimelineType type) const {
        std::shared_lock lock(timelinesMutex_);

        if (const auto it = timelinesByType_.find(type); it != timelinesByType_.end() && !it->second.empty()) {
            // Return first timeline of this type
            if (const auto timelineIt = timelines_.find(it->second.front()); timelineIt != timelines_.end()) {
                return timelineIt->second.timeline;
            }
        }

        return nullptr;
    }

    std::vector<std::shared_ptr<Timeline>> TimelineManager::getTimelinesByType(const TimelineType type) const {
        std::shared_lock lock(timelinesMutex_);

        std::vector<std::shared_ptr<Timeline>> result;

        if (const auto it = timelinesByType_.find(type); it != timelinesByType_.end()) {
            result.reserve(it->second.size());
            for (TimelineID id : it->second) {
                if (auto timelineIt = timelines_.find(id); timelineIt != timelines_.end()) {
                    result.push_back(timelineIt->second.timeline);
                }
            }
        }

        return result;
    }

    std::vector<std::shared_ptr<Timeline>> TimelineManager::getAllTimelines() const {
        std::shared_lock lock(timelinesMutex_);

        std::vector<std::shared_ptr<Timeline>> result;
        result.reserve(timelines_.size());

        for (const auto& entry : timelines_ | std::views::values) {
            result.push_back(entry.timeline);
        }

        return result;
    }

    // =============================================================================
    // Timeline Control
    // =============================================================================

    void TimelineManager::pauseAllTimelines() {
        std::shared_lock lock(timelinesMutex_);

        for (const auto& entry : timelines_ | std::views::values) {
            if (entry.timeline->canPause()) {
                entry.timeline->pause();
            }
        }
    }

    void TimelineManager::resumeAllTimelines() {
        std::shared_lock lock(timelinesMutex_);

        for (const auto& entry : timelines_ | std::views::values) {
            entry.timeline->resume();
        }
    }

    void TimelineManager::stopAllTimelines() {
        std::shared_lock lock(timelinesMutex_);

        for (const auto& entry : timelines_ | std::views::values) {
            entry.timeline->stop();
        }
    }

    void TimelineManager::startAllTimelines() {
        std::shared_lock lock(timelinesMutex_);

        for (const auto& entry : timelines_ | std::views::values) {
            if (entry.timeline->getState() == TimelineState::INITIALIZED) {
                entry.timeline->start();
            }
        }
    }

    // =============================================================================
    // Timeline Groups
    // =============================================================================

    bool TimelineManager::createGroup(const std::string& name, const std::vector<TimelineID>& timelineIds) {
        std::unique_lock lock(groupsMutex_);

        if (timelineGroups_.contains(name)) {
            return false; // Group already exists
        }

        timelineGroups_[name] = TimelineGroup{
                        .name = name,
                        .timelines = timelineIds,
                        .pauseAllowed = true,
                        .syncEnabled = false
                };

        return true;
    }

    bool TimelineManager::destroyGroup(const std::string& groupName) {
        std::unique_lock lock(groupsMutex_);

        const auto it = timelineGroups_.find(groupName);
        if (it == timelineGroups_.end()) {
            return false;
        }

        timelineGroups_.erase(it);
        return true;
    }

    bool TimelineManager::addToGroup(const std::string& groupName, const TimelineID timelineId) {
        std::unique_lock lock(groupsMutex_);

        const auto it = timelineGroups_.find(groupName);
        if (it == timelineGroups_.end()) {
            return false;
        }

        // Check if timeline already in group
        auto& timelines = it->second.timelines;
        if (std::ranges::find(timelines, timelineId) != timelines.end()) {
            return false; // Already in group
        }

        timelines.push_back(timelineId);
        return true;
    }

    bool TimelineManager::removeFromGroup(const std::string& groupName, const TimelineID timelineId) {
        std::unique_lock lock(groupsMutex_);

        const auto it = timelineGroups_.find(groupName);
        if (it == timelineGroups_.end()) {
            return false;
        }

        auto& timelines = it->second.timelines;
        const auto timelineIt = std::ranges::find(timelines, timelineId);
        if (timelineIt == timelines.end()) {
            return false; // Not in group
        }

        timelines.erase(timelineIt);
        return true;
    }

    void TimelineManager::pauseGroup(const std::string& groupName) {
        std::shared_lock lock(groupsMutex_);

        if (const auto it = timelineGroups_.find(groupName); it != timelineGroups_.end() && it->second.pauseAllowed) {
            for (const TimelineID id : it->second.timelines) {
                if (const auto timeline = getTimeline(id)) {
                    timeline->pause();
                }
            }
        }
    }

    void TimelineManager::resumeGroup(const std::string& groupName) {
        std::shared_lock lock(groupsMutex_);

        if (const auto it = timelineGroups_.find(groupName); it != timelineGroups_.end()) {
            for (const TimelineID id : it->second.timelines) {
                if (const auto timeline = getTimeline(id)) {
                    timeline->resume();
                }
            }
        }
    }

    std::optional<TimelineManager::TimelineGroup> TimelineManager::getGroup(const std::string& groupName) const {
        std::shared_lock lock(groupsMutex_);

        if (const auto it = timelineGroups_.find(groupName); it != timelineGroups_.end()) {
            return it->second;
        }

        return std::nullopt;
    }

    // =============================================================================
    // Scene Management
    // =============================================================================
    // TODO: Revisar esto
    void TimelineManager::onSceneTransition(
            const std::string& fromScene,
            const std::string& toScene
            ) {
        std::vector<TimelineID> toDestroy;

        // Identify non-persistent timelines to destroy
        {
            std::shared_lock lock(timelinesMutex_);

            for (const auto& [id, entry] : timelines_) {
                if (!entry.persistent) {
                    toDestroy.push_back(id);
                }
            }
        }

        // Destroy non-persistent timelines
        for (const TimelineID id : toDestroy) {
            destroyTimeline(id);
        }

        // Reset persistent timelines
        {
            std::shared_lock lock(timelinesMutex_);

            for (const auto& entry : timelines_ | std::views::values) {
                if (entry.persistent) {
                    entry.timeline->reset();
                }
            }
        }
    }

    bool TimelineManager::setTimelinePersistent(const TimelineID id, const bool persistent) {
        std::unique_lock lock(timelinesMutex_);

        const auto it = timelines_.find(id);
        if (it == timelines_.end()) {
            return false;
        }

        it->second.persistent = persistent;
        return true;
    }

    // =============================================================================
    // Synchronization
    // =============================================================================

    bool TimelineManager::synchronizeTimelines(
            const TimelineID primaryId,
            const TimelineID secondaryId,
            const Duration offset
            ) const {
        const auto primary = getTimeline(primaryId);
        const auto secondary = getTimeline(secondaryId);

        if (!primary || !secondary) {
            return false;
        }

        return secondary->synchronizeWith(primary.get(), offset);
    }

    // =============================================================================
    // Statistics and Monitoring
    // =============================================================================

    std::size_t TimelineManager::getTimelineCount() const {
        std::shared_lock lock(timelinesMutex_);
        return timelines_.size();
    }

    std::size_t TimelineManager::getActiveTimelineCount() const {
        std::shared_lock lock(timelinesMutex_);

        return std::ranges::count_if(
                timelines_,
                [](const auto& pair) {
                    return pair.second.timeline->isRunning();
                }
                );
    }

    std::optional<TimelineStats> TimelineManager::getTimelineStats(const TimelineID id) const {
        std::shared_lock lock(timelinesMutex_);

        if (const auto it = timelineStats_.find(id); it != timelineStats_.end()) {
            return it->second;
        }

        return std::nullopt;
    }

    std::string TimelineManager::generateReport() const {
        std::stringstream report;

        report << "=== Timeline Manager Report ===\n\n";

        // Overall statistics
        {
            std::shared_lock lock(timelinesMutex_);

            report << "Total Timelines: " << timelines_.size() << "\n";
            report << "Active Timelines: " << getActiveTimelineCount() << "\n";

            // Per-type breakdown
            report << "\nTimelines by Type:\n";
            for (const auto& [type, ids] : timelinesByType_) {
                report << "  " << getTimelineName(type) << ": " << ids.size() << "\n";
            }
        }

        // Groups
        {
            std::shared_lock lock(groupsMutex_);

            report << "\nTimeline Groups: " << timelineGroups_.size() << "\n";
            for (const auto& [name, group] : timelineGroups_) {
                report << "  " << name << ": " << group.timelines.size() << " timelines\n";
            }
        }

        // Detailed timeline information
        {
            std::shared_lock lock(timelinesMutex_);

            report << "\nDetailed Timeline Information:\n";
            for (const auto& [id, entry] : timelines_) {
                report << "\n  Timeline ID: " << id << "\n";
                report << "    Name: " << entry.timeline->getName() << "\n";
                report << "    Type: " << getTimelineName(entry.timeline->getType()) << "\n";
                report << "    State: " << getTimelineStateName(entry.timeline->getState()) << "\n";
                report << "    Priority: " << entry.updatePriority << "\n";
                report << "    Persistent: " << (entry.persistent ? "Yes" : "No") << "\n";
                report << "    Current Time: " << toSeconds(entry.timeline->getCurrentTime()) << "s\n";
                report << "    Time Scale: " << entry.timeline->getTimeScale() << "x\n";

                // Statistics if available
                if (auto statsIt = timelineStats_.find(id); statsIt != timelineStats_.end()) {
                    const auto& stats = statsIt->second;
                    report << "    Updates: " << stats.updateCount.load() << "\n";
                }
            }
        }

        report << "\n=== End Report ===\n";

        return report.str();
    }

    void TimelineManager::updateTimelineStats(
            const TimelineID id,
            const TimelineUpdateResult& result
            ) {
        std::unique_lock lock(timelinesMutex_);

        auto& stats = timelineStats_[id];
        stats.id = id;
        stats.updateDuration.update(result.actualDeltaTime, backend_.now());
        stats.updateCount.fetch_add(1, std::memory_order_relaxed);
    }

    // =============================================================================
    // Helper Methods
    // =============================================================================

    std::string TimelineManager::getTimelineName(const TimelineType type) {
        switch (type) {
            case TimelineType::REAL_TIME:
                return "RealTime";
            case TimelineType::GAME_TIME:
                return "GameTime";
            case TimelineType::UI_TIME:
                return "UITime";
            case TimelineType::PHYSICS_TIME:
                return "PhysicsTime";
            case TimelineType::AUDIO_TIME:
                return "AudioTime";
            case TimelineType::CUSTOM_1:
                return "Custom1";
            case TimelineType::CUSTOM_2:
                return "Custom2";
            case TimelineType::CUSTOM_3:
                return "Custom3";
            default:
                return "Unknown";
        }
    }

    std::string TimelineManager::getTimelineStateName(const TimelineState state) {
        switch (state) {
            case TimelineState::UNINITIALIZED:
                return "Uninitialized";
            case TimelineState::INITIALIZED:
                return "Initialized";
            case TimelineState::RUNNING:
                return "Running";
            case TimelineState::PAUSED:
                return "Paused";
            case TimelineState::STOPPED:
                return "Stopped";
            case TimelineState::DESTROYED:
                return "Destroyed";
            default:
                return "Unknown";
        }
    }
} // namespace engine::time
