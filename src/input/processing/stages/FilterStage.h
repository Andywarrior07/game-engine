/**
 * @file FilterStage.h
 * @brief Input event filtering stage for noise reduction and duplicate removal
 * @author Andrés Guerrero
 * @date 12-09-2025
 *
 * Filters out noise, duplicate events, and applies event coalescing.
 */

#pragma once

#include "../../core/InputEvent.h"
#include "../../core/InputConfig.h"

#include <vector>
#include <unordered_map>
#include <chrono>
#include <deque>

namespace engine::input::processing {
    /**
     * @brief Event filter configuration
     */
    struct FilterConfig {
        // Noise filtering
        bool enableNoiseFilter = true;
        float noiseThreshold = 0.01f; // Minimum change to register

        // Duplicate removal
        bool removeDuplicates = true;
        std::chrono::milliseconds duplicateWindow{16}; // Time window for duplicates

        // Event coalescing
        bool enableCoalescing = true;
        std::uint32_t maxCoalescedEvents = 10;

        // Rate limiting
        bool enableRateLimiting = false;
        std::uint32_t maxEventsPerSecond = 1000;

        // Event type filtering
        bool filterByType = false;
        std::vector<InputEventType> allowedTypes;
        std::vector<InputEventType> blockedTypes;
    };

    /**
     * @brief Input event filtering stage
     *
     * Processes raw events to remove noise, duplicates, and apply coalescing.
     */
    class FilterStage {
    public:
        FilterStage() noexcept;
        explicit FilterStage(const FilterConfig& config) noexcept;
        ~FilterStage() = default;

        // Disable copy, enable move
        FilterStage(const FilterStage&) = delete;
        FilterStage& operator=(const FilterStage&) = delete;
        FilterStage(FilterStage&&) noexcept = default;
        FilterStage& operator=(FilterStage&&) noexcept = default;

        /**
         * @brief Process a batch of events
         * @param events Input/output vector of events
         * @param deltaTime Time since last update
         * @return Number of events after filtering
         */
        std::size_t process(std::vector<InputEvent>& events, float deltaTime);

        /**
         * @brief Process single event
         * @param event Event to process
         * @return True if event should be kept
         */
        [[nodiscard]] bool processEvent(const InputEvent& event);

        /**
         * @brief Set filter configuration
         */
        void setConfig(const FilterConfig& config) noexcept;

        /**
         * @brief Get current configuration
         */
        [[nodiscard]] const FilterConfig& getConfig() const noexcept {
            return config_;
        }

        /**
         * @brief Reset filter state
         */
        void reset() noexcept;

        /**
         * @brief Get filter statistics
         */
        struct Statistics {
            std::uint64_t totalProcessed = 0;
            std::uint64_t totalFiltered = 0;
            std::uint64_t noiseFiltered = 0;
            std::uint64_t duplicatesRemoved = 0;
            std::uint64_t eventsCoalesced = 0;
            std::uint64_t rateLimited = 0;

            void reset() noexcept {
                totalProcessed = 0;
                totalFiltered = 0;
                noiseFiltered = 0;
                duplicatesRemoved = 0;
                eventsCoalesced = 0;
                rateLimited = 0;
            }
        };

        [[nodiscard]] const Statistics& getStatistics() const noexcept {
            return stats_;
        }

        /**
         * @brief Enable/disable specific filter
         */
        void setNoiseFilterEnabled(const bool enabled) noexcept {
            config_.enableNoiseFilter = enabled;
        }

        void setDuplicateRemovalEnabled(const bool enabled) noexcept {
            config_.removeDuplicates = enabled;
        }

        void setCoalescingEnabled(const bool enabled) noexcept {
            config_.enableCoalescing = enabled;
        }

        void setRateLimitingEnabled(const bool enabled) noexcept {
            config_.enableRateLimiting = enabled;
        }

    private:
        // Configuration
        FilterConfig config_;

        // Statistics
        Statistics stats_;

        // Event history for duplicate detection
        struct EventHistory {
            InputEvent event;
            std::chrono::steady_clock::time_point timestamp;
        };

        static constexpr std::size_t MAX_HISTORY_SIZE = 100;
        std::deque<EventHistory> eventHistory_;

        // Last values for noise filtering
        struct LastValue {
            float value = 0;
            math::Vec2 vector2 = math::VEC2_ZERO;
            math::Vec3 vector3 = math::VEC3_ZERO;
            std::chrono::steady_clock::time_point timestamp;
        };

        std::unordered_map<std::uint64_t, LastValue> lastValues_;

        // Rate limiting
        struct RateLimitInfo {
            std::uint32_t eventCount = 0;
            std::chrono::steady_clock::time_point windowStart;
        };

        std::unordered_map<DeviceID, RateLimitInfo> rateLimits_;

        // Coalescing buffer
        struct CoalescedEvent {
            InputEvent baseEvent;
            std::vector<InputEvent> coalescedEvents;
            std::uint32_t count = 1;
        };

        std::unordered_map<std::uint64_t, CoalescedEvent> coalescingBuffer_;

        // ============================================================================
        // Filter Methods
        // ============================================================================

        /**
         * @brief Apply noise filter to event
         * @return True if event passes filter
         */
        [[nodiscard]] bool applyNoiseFilter(const InputEvent& event);

        /**
         * @brief Check if event is duplicate
         * @return True if duplicate
         */
        [[nodiscard]] bool isDuplicate(const InputEvent& event) const;

        /**
         * @brief Apply rate limiting
         * @return True if event passes rate limit
         */
        [[nodiscard]] bool applyRateLimit(const InputEvent& event);

        /**
         * @brief Check if event type is allowed
         * @return True if allowed
         */
        [[nodiscard]] bool isTypeAllowed(const InputEvent& event) const noexcept;

        /**
         * @brief Try to coalesce event
         * @return True if event was coalesced
         */
        bool tryCoalesce(const InputEvent& event);

        /**
         * @brief Flush coalesced events
         */
        void flushCoalescedEvents(std::vector<InputEvent>& output);

        /**
         * @brief Clean old history entries
         */
        void cleanHistory();

        /**
         * @brief Generate unique key for event
         */
        [[nodiscard]] static std::uint64_t generateEventKey(const InputEvent& event) noexcept;

        /**
         * @brief Check if two events are similar enough to coalesce
         */
        [[nodiscard]] static bool canCoalesce(const InputEvent& a, const InputEvent& b) noexcept;

        /**
         * @brief Merge two events
         */
        static void mergeEvents(InputEvent& target, const InputEvent& source) noexcept;
    };
} // namespace engine::input::processing
