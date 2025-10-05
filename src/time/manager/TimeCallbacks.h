/**
 * @file TimeCallbacks.h
 * @brief Callback definitions and event types for the time management system
 * @details Provides type-safe callback definitions for various time system events
 *          including timeline updates, timer events, performance notifications, and
 *          system state changes. Optimized for minimal overhead with move semantics.
 *
 * @author Development Team
 * @date Created on 2024-01-20
 */

#pragma once

#include "../core/TimeTypes.h"
#include "../core/TimelineType.h"
#include "../core/Timeline.h"

#include <functional>
#include <string>
#include <variant>
#include <optional>

namespace engine::time {

    // Forward declarations
    struct FrameProfileData;

    // =============================================================================
    // Core Callback Types
    // =============================================================================

    /**
     * @brief Timer callback function type
     * @details Called when a timer fires. Receives elapsed time since timer started.
     */
    using TimerCallback = std::function<void(Duration elapsed)>;

    /**
     * @brief Timeline update callback
     * @details Called after timeline updates. Receives delta time and timeline ID.
     */
    using TimelineUpdateCallback = std::function<void(Duration deltaTime, TimelineID id)>;

    /**
     * @brief Timeline state change callback
     * @details Called when timeline state changes (paused, resumed, stopped, etc.)
     */
    using TimelineStateCallback = std::function<void(TimelineID id, TimelineState newState)>;

    /**
     * @brief Time scale change callback
     * @details Called when timeline time scale is modified.
     */
    using TimeScaleCallback = std::function<void(TimelineID id, TimeScale oldScale, TimeScale newScale)>;

    // =============================================================================
    // Performance Callbacks
    // =============================================================================

    /**
     * @brief Frame performance callback
     * @details Called with frame profile data for monitoring.
     */
    using FrameCallback = std::function<void(const FrameProfileData& frameData)>;

    /**
     * @brief FPS update callback
     * @details Called periodically with current FPS measurement.
     */
    using FPSCallback = std::function<void(float currentFPS, float averageFPS)>;

    /**
     * @brief Performance warning callback
     * @details Called when performance issues are detected.
     */
    using PerformanceWarningCallback = std::function<void(const std::string& warning)>;

    /**
     * @brief Budget exceeded callback
     * @details Called when a system exceeds its time budget.
     */
    using BudgetExceededCallback = std::function<void(
            const std::string& systemName,
            Duration actual,
            Duration budget
            )>;

    // =============================================================================
    // System Event Callbacks
    // =============================================================================

    /**
     * @brief System initialization callback
     * @details Called when time system components initialize.
     */
    using SystemInitCallback = std::function<void(const std::string& componentName, bool success)>;

    /**
     * @brief System shutdown callback
     * @details Called when time system components shut down.
     */
    using SystemShutdownCallback = std::function<void(const std::string& componentName)>;

    /**
     * @brief Error callback
     * @details Called when errors occur in the time system.
     */
    using ErrorCallback = std::function<void(const std::string& error, const std::string& component)>;

    // =============================================================================
    // Synchronization Callbacks
    // =============================================================================

    /**
     * @brief Timeline synchronization callback
     * @details Called when timelines are synchronized or desynchronized.
     */
    using SyncCallback = std::function<void(TimelineID primary, TimelineID secondary, bool synced)>;

    /**
     * @brief Clock adjustment callback
     * @details Called when system clock adjustments are detected.
     */
    using ClockAdjustmentCallback = std::function<void(Duration adjustment, TimeStamp when)>;

    /**
     * @brief Network time sync callback
     * @details Called when syncing with network time source.
     */
    using NetworkSyncCallback = std::function<void(
            TimeStamp localTime,
            TimeStamp serverTime,
            Duration latency
            )>;

    // =============================================================================
    // Event Types
    // =============================================================================

    /**
     * @brief Time system event types
     * @details Enumeration of all possible time system events.
     */
    enum class TimeEventType : std::uint32_t {
        // Timeline events
        TIMELINE_CREATED = 0x0001, TIMELINE_DESTROYED = 0x0002, TIMELINE_STARTED = 0x0004,
        TIMELINE_STOPPED = 0x0008, TIMELINE_PAUSED    = 0x0010, TIMELINE_RESUMED = 0x0020,
        TIMELINE_UPDATED = 0x0040, TIMELINE_RESET     = 0x0080,

        // Timer events
        TIMER_CREATED   = 0x0100, TIMER_FIRED = 0x0200, TIMER_CANCELLED = 0x0400,
        TIMER_COMPLETED = 0x0800,

        // Performance events
        FRAME_START  = 0x1000, FRAME_END = 0x2000, FRAME_DROPPED = 0x4000,
        VSYNC_MISSED = 0x8000,

        // System events
        BUDGET_EXCEEDED      = 0x10000, SPIKE_DETECTED = 0x20000, PERFORMANCE_WARNING = 0x40000,
        PERFORMANCE_CRITICAL = 0x80000,

        // Synchronization events
        CLOCK_ADJUSTED = 0x100000, TIME_SYNC_LOST = 0x200000, TIME_SYNC_RESTORED = 0x400000,

        // All events mask
        ALL_EVENTS = 0xFFFFFFFF
    };

    /**
     * @brief Bitwise OR operator for event types
     */
    inline TimeEventType operator|(TimeEventType a, TimeEventType b) {
        return static_cast<TimeEventType>(
            static_cast<std::uint32_t>(a) | static_cast<std::uint32_t>(b)
        );
    }

    /**
     * @brief Bitwise AND operator for event types
     */
    inline TimeEventType operator&(TimeEventType a, TimeEventType b) {
        return static_cast<TimeEventType>(
            static_cast<std::uint32_t>(a) & static_cast<std::uint32_t>(b)
        );
    }

    /**
     * @brief Check if event type is set
     */
    inline bool hasEvent(const TimeEventType mask, const TimeEventType event) {
        return (mask & event) == event;
    }

    // =============================================================================
    // Event Data Structures
    // =============================================================================

    /**
     * @brief Timeline event data
     * @details Contains information about timeline-related events.
     */
    struct TimelineEventData {
        TimelineID timelineId;
        TimelineType type;
        TimelineState state;
        Duration deltaTime;
        TimeScale timeScale;
        std::string name;
    };

    /**
     * @brief Timer event data
     * @details Contains information about timer-related events.
     */
    struct TimerEventData {
        TimerID timerId;
        TimelineID timelineId;
        Duration elapsed;
        Duration remaining;
        std::uint32_t executionCount;
        bool recurring;
    };

    /**
     * @brief Performance event data
     * @details Contains performance-related event information.
     */
    struct PerformanceEventData {
        FrameNumber frameNumber;
        Duration frameTime;
        Duration cpuTime;
        Duration gpuTime;
        float fps;
        bool droppedFrame;
        bool missedVSync;
        std::string systemName;
        float budgetUsage;
    };

    /**
     * @brief Generic time event
     * @details Variant type containing any time system event data.
     */
    using TimeEventData = std::variant<
        TimelineEventData,
        TimerEventData,
        PerformanceEventData,
        std::string // For error messages
    >;

    /**
     * @brief Complete time event structure
     * @details Contains event type, timestamp, and associated data.
     */
    struct TimeEvent {
        TimeEventType type;
        TimeStamp timestamp;
        TimeEventData data;
        std::optional<std::string> message;

        /**
         * @brief Check if event matches type
         */
        [[nodiscard]] bool isType(TimeEventType eventType) const noexcept {
            return hasEvent(type, eventType);
        }

        /**
         * @brief Get event data as specific type
         * @tparam T Data type to retrieve
         * @return Pointer to data or nullptr if wrong type
         */
        template <typename T>
        [[nodiscard]] const T* getDataAs() const noexcept {
            return std::get_if<T>(&data);
        }
    };

    // =============================================================================
    // Callback Management
    // =============================================================================

    /**
     * @brief Callback handle for unregistration
     * @details Opaque handle used to unregister callbacks.
     */
    using CallbackHandle = std::uint64_t;

    /**
     * @brief Invalid callback handle constant
     */
    constexpr CallbackHandle INVALID_CALLBACK_HANDLE = 0;

    /**
     * @brief Callback registration info
     * @details Contains callback function and filtering information.
     */
    template <typename CallbackType>
    struct CallbackRegistration {
        CallbackHandle handle{};
        CallbackType callback;
        TimeEventType eventMask;
        std::optional<TimelineID> timelineFilter;
        std::optional<std::string> nameFilter;
        bool oneShot{};

        /**
         * @brief Check if callback should be invoked for event
         */
        [[nodiscard]] bool shouldInvoke(const TimeEvent& event) const noexcept {
            // Check event type mask
            if (!hasEvent(eventMask, event.type)) {
                return false;
            }

            // Check timeline filter if specified
            if (timelineFilter.has_value()) {
                if (const auto* timelineData = event.getDataAs<TimelineEventData>()) {
                    if (timelineData->timelineId != *timelineFilter) {
                        return false;
                    }
                } else if (const auto* timerData = event.getDataAs<TimerEventData>()) {
                    if (timerData->timelineId != *timelineFilter) {
                        return false;
                    }
                }
            }

            // Check name filter if specified
            if (nameFilter.has_value()) {
                if (const auto* timelineData = event.getDataAs<TimelineEventData>()) {
                    if (timelineData->name != *nameFilter) {
                        return false;
                    }
                } else if (const auto* perfData = event.getDataAs<PerformanceEventData>()) {
                    if (perfData->systemName != *nameFilter) {
                        return false;
                    }
                }
            }

            return true;
        }
    };

    // =============================================================================
    // Callback Builder
    // =============================================================================

    /**
     * @brief Fluent interface for building callback registrations
     * @details Provides a type-safe way to configure callbacks.
     */
    template <typename CallbackType>
    class CallbackBuilder {
    public:
        explicit CallbackBuilder(CallbackType callback) :
            registration_{INVALID_CALLBACK_HANDLE,
                          std::move(callback),
                          TimeEventType::ALL_EVENTS,
                          std::nullopt,
                          std::nullopt,
                          false} {}

        /**
         * @brief Set event type filter
         */
        CallbackBuilder& forEvents(TimeEventType events) {
            registration_.eventMask = events;
            return *this;
        }

        /**
         * @brief Set timeline filter
         */
        CallbackBuilder& forTimeline(TimelineID id) {
            registration_.timelineFilter = id;
            return *this;
        }

        /**
         * @brief Set name filter
         */
        CallbackBuilder& withName(const std::string& name) {
            registration_.nameFilter = name;
            return *this;
        }

        /**
         * @brief Make callback one-shot
         */
        CallbackBuilder& once() {
            registration_.oneShot = true;
            return *this;
        }

        /**
         * @brief Get built registration
         */
        [[nodiscard]] CallbackRegistration<CallbackType> build() const {
            return registration_;
        }

    private:
        CallbackRegistration<CallbackType> registration_;
    };

    /**
     * @brief Create callback builder
     * @tparam CallbackType Type of callback (auto-deduced)
     * @param callback Callback function
     * @return Callback builder instance
     */
    template <typename CallbackType>
    [[nodiscard]] CallbackBuilder<CallbackType> makeCallback(CallbackType callback) {
        return CallbackBuilder<CallbackType>(std::move(callback));
    }

} // namespace engine::time
