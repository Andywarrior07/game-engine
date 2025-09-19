/**
 * @file FilterStage.cpp
 * @brief Input event filtering stage implementation
 * @author Andrés Guerrero
 * @date 12-09-2025
 */

#include "FilterStage.h"

#include <algorithm>
#include <chrono>

namespace engine::input::processing {
    FilterStage::FilterStage() noexcept {
    }

    FilterStage::FilterStage(const FilterConfig& config) noexcept
        : config_(config) {
    }


    // TODO: Revisar porque no se usa deltaTime
    std::size_t FilterStage::process(std::vector<InputEvent>& events, const float deltaTime) {
        if (events.empty()) {
            return 0;
        }

        stats_.totalProcessed += events.size();

        // Clean old history entries before processing
        cleanHistory();

        // Pre-allocate space for filtered events to avoid reallocations
        std::vector<InputEvent> filteredEvents;
        filteredEvents.reserve(events.size());

        // Process each event through filters
        for (auto& event : events) {
            if (processEvent(event)) {
                filteredEvents.push_back(std::move(event));
            }
            else {
                stats_.totalFiltered++;
            }
        }

        // Flush any coalesced events
        flushCoalescedEvents(filteredEvents);

        // Replace original events with filtered ones
        events = std::move(filteredEvents);

        return events.size();
    }

    bool FilterStage::processEvent(const InputEvent& event) {
        // Skip consumed events
        if (event.consumed) {
            return true;
        }

        // Apply event type filtering first (fastest check)
        if (config_.filterByType && !isTypeAllowed(event)) {
            return false;
        }

        // Apply rate limiting
        if (config_.enableRateLimiting && !applyRateLimit(event)) {
            stats_.rateLimited++;
            return false;
        }

        // Apply noise filtering
        if (config_.enableNoiseFilter && !applyNoiseFilter(event)) {
            stats_.noiseFiltered++;
            return false;
        }

        // Check for duplicates
        if (config_.removeDuplicates && isDuplicate(event)) {
            stats_.duplicatesRemoved++;
            return false;
        }

        // Try event coalescing
        if (config_.enableCoalescing && tryCoalesce(event)) {
            stats_.eventsCoalesced++;
            return false; // Event was coalesced, don't add to output yet
        }

        // Add to history for duplicate detection
        if (config_.removeDuplicates) {
            eventHistory_.push_back({event, std::chrono::steady_clock::now()});

            // Keep history size bounded
            if (eventHistory_.size() > MAX_HISTORY_SIZE) {
                eventHistory_.pop_front();
            }
        }

        return true;
    }

    void FilterStage::setConfig(const FilterConfig& config) noexcept {
        config_ = config;
    }

    void FilterStage::reset() noexcept {
        eventHistory_.clear();
        lastValues_.clear();
        rateLimits_.clear();
        coalescingBuffer_.clear();
        stats_.reset();
    }

    // ============================================================================
    // Private Implementation
    // ============================================================================

    bool FilterStage::applyNoiseFilter(const InputEvent& event) {
        const std::uint64_t eventKey = generateEventKey(event);
        auto& lastValue = lastValues_[eventKey];
        const auto currentTime = std::chrono::steady_clock::now();

        bool passesFilter = true;

        switch (event.type) {
        case InputEventType::MOUSE_MOVED: {
            if (const auto* data = event.getEventData<MouseMotionEventData>()) {
                if (const float deltaLength = glm::length(data->delta); deltaLength < config_.noiseThreshold) {
                    passesFilter = false;
                }
                else {
                    lastValue.vector2 = data->delta;
                    lastValue.timestamp = currentTime;
                }
            }
            break;
        }

        case InputEventType::GAMEPAD_AXIS_MOVED: {
            if (const auto* data = event.getEventData<GamepadAxisEventData>()) {
                if (const float valueDiff = std::abs(data->value - lastValue.value); valueDiff < config_.noiseThreshold) {
                    passesFilter = false;
                }
                else {
                    lastValue.value = data->value;
                    lastValue.timestamp = currentTime;
                }
            }
            break;
        }

        case InputEventType::MOUSE_WHEEL: {
            if (const auto* data = event.getEventData<MouseWheelEventData>()) {
                if (const float deltaLength = glm::length(data->delta); deltaLength < config_.noiseThreshold) {
                    passesFilter = false;
                }
                else {
                    lastValue.vector2 = data->delta;
                    lastValue.timestamp = currentTime;
                }
            }
            break;
        }

        default:
            // Other event types pass through noise filter
            break;
        }

        return passesFilter;
    }

    bool FilterStage::isDuplicate(const InputEvent& event) const {
        const auto currentTime = std::chrono::steady_clock::now();

        // Check recent history for duplicates
        for (const auto& historyEntry : eventHistory_) {
            if (currentTime - historyEntry.timestamp > config_.duplicateWindow) {
                continue; // Too old to be a duplicate
            }

            const InputEvent& historicEvent = historyEntry.event;

            // Basic duplicate check
            if (event.type != historicEvent.type || event.deviceId != historicEvent.deviceId) {
                continue;
            }

            // Type-specific duplicate detection
            switch (event.type) {
            case InputEventType::KEY_PRESSED:
            case InputEventType::KEY_RELEASED: {
                const auto* eventData = event.getEventData<KeyboardEventData>();

                if (const auto* historyData = historicEvent.getEventData<KeyboardEventData>(); eventData && historyData &&
                    eventData->key == historyData->key &&
                    eventData->modifiers == historyData->modifiers) {
                    return true;
                }
                break;
            }

            case InputEventType::MOUSE_BUTTON_PRESSED:
            case InputEventType::MOUSE_BUTTON_RELEASED: {
                const auto* eventData = event.getEventData<MouseButtonEventData>();

                if (const auto* historyData = historicEvent.getEventData<MouseButtonEventData>(); eventData && historyData && eventData->button == historyData->button) {
                    // Check if positions are close enough to be considered duplicate
                    if (const float distance = glm::length(eventData->position - historyData->position); distance < 2.0f) {
                        // 2 pixel tolerance
                        return true;
                    }
                }
                break;
            }

            case InputEventType::GAMEPAD_BUTTON_PRESSED:
            case InputEventType::GAMEPAD_BUTTON_RELEASED: {
                const auto* eventData = event.getEventData<GamepadButtonEventData>();

                if (const auto* historyData = historicEvent.getEventData<GamepadButtonEventData>(); eventData && historyData && eventData->button == historyData->button) {
                    return true;
                }
                break;
            }

            case InputEventType::GAMEPAD_AXIS_MOVED: {
                const auto* eventData = event.getEventData<GamepadAxisEventData>();

                if (const auto* historyData = historicEvent.getEventData<GamepadAxisEventData>(); eventData && historyData &&
                    eventData->axis == historyData->axis &&
                    std::abs(eventData->value - historyData->value) < config_.noiseThreshold) {
                    return true;
                }
                break;
            }

            default:
                // Generic duplicate check for other event types
                if (std::memcmp(&event.data, &historicEvent.data, sizeof(event.data)) == 0) {
                    return true;
                }
                break;
            }
        }

        return false;
    }

    bool FilterStage::applyRateLimit(const InputEvent& event) {
        auto& rateLimitInfo = rateLimits_[event.deviceId];

        // Reset counter if window has passed
        if (const auto currentTime = std::chrono::steady_clock::now(); currentTime - rateLimitInfo.windowStart > std::chrono::seconds(1)) {
            rateLimitInfo.eventCount = 0;
            rateLimitInfo.windowStart = currentTime;
        }

        // Check if rate limit exceeded
        if (rateLimitInfo.eventCount >= config_.maxEventsPerSecond) {
            return false;
        }

        rateLimitInfo.eventCount++;
        return true;
    }

    bool FilterStage::isTypeAllowed(const InputEvent& event) const noexcept {
        // If no specific filtering is configured, allow all types
        if (!config_.filterByType) {
            return true;
        }

        // Check blocked types first (more restrictive)
        if (!config_.blockedTypes.empty()) {
            return std::ranges::find(config_.blockedTypes, event.type) == config_.blockedTypes.end();
        }

        // Check allowed types
        if (!config_.allowedTypes.empty()) {
            return std::ranges::find(config_.allowedTypes, event.type) != config_.allowedTypes.end();
        }

        return true;
    }

    bool FilterStage::tryCoalesce(const InputEvent& event) {
        const std::uint64_t eventKey = generateEventKey(event);
        auto& coalescedEvent = coalescingBuffer_[eventKey];

        // Check if this event can be coalesced with an existing one
        if (coalescedEvent.count > 0 && canCoalesce(coalescedEvent.baseEvent, event)) {
            // Merge this event with the existing coalesced event
            mergeEvents(coalescedEvent.baseEvent, event);
            coalescedEvent.coalescedEvents.push_back(event);
            coalescedEvent.count++;

            // Check if we've reached the maximum number of coalesced events
            if (coalescedEvent.count >= config_.maxCoalescedEvents) {
                // Force flush this coalesced event
                return false; // Return false to add the merged event to output
            }

            return true; // Event was coalesced
        }
        else {
            // Start a new coalescing group
            coalescedEvent.baseEvent = event;
            coalescedEvent.coalescedEvents.clear();
            coalescedEvent.count = 1;
            return false; // First event in group goes to output immediately
        }
    }

    void FilterStage::flushCoalescedEvents(std::vector<InputEvent>& output) {
        for (auto& coalescedEvent : coalescingBuffer_ | std::views::values) {
            if (coalescedEvent.count > 1) {
                // Add the merged event to output
                output.push_back(std::move(coalescedEvent.baseEvent));
            }
        }

        coalescingBuffer_.clear();
    }

    void FilterStage::cleanHistory() {
        const auto currentTime = std::chrono::steady_clock::now();
        const auto cutoffTime = currentTime - config_.duplicateWindow - std::chrono::milliseconds(100); // Extra buffer

        // Remove old entries from history
        std::erase_if(eventHistory_,
                      [cutoffTime](const EventHistory& entry) {
                          return entry.timestamp < cutoffTime;
                      });

        // Clean old last values
        for (auto it = lastValues_.begin(); it != lastValues_.end();) {
            if (currentTime - it->second.timestamp > std::chrono::seconds(5)) {
                it = lastValues_.erase(it);
            }
            else {
                ++it;
            }
        }
    }

    std::uint64_t FilterStage::generateEventKey(const InputEvent& event) noexcept {
        // Generate unique key based on event type and device
        std::uint64_t key = static_cast<std::uint64_t>(event.type) << 32;
        key |= static_cast<std::uint64_t>(event.deviceId);

        // Add type-specific data to key for better granularity
        switch (event.type) {
        case InputEventType::GAMEPAD_AXIS_MOVED: {
            if (const auto* data = event.getEventData<GamepadAxisEventData>()) {
                key |= static_cast<std::uint64_t>(data->axis) << 16;
            }
            break;
        }
        case InputEventType::KEY_PRESSED:
        case InputEventType::KEY_RELEASED: {
            if (const auto* data = event.getEventData<KeyboardEventData>()) {
                key |= static_cast<std::uint64_t>(data->key) << 16;
            }
            break;
        }
        case InputEventType::MOUSE_BUTTON_PRESSED:
        case InputEventType::MOUSE_BUTTON_RELEASED: {
            if (const auto* data = event.getEventData<MouseButtonEventData>()) {
                key |= static_cast<std::uint64_t>(data->button) << 16;
            }
            break;
        }
        case InputEventType::GAMEPAD_BUTTON_PRESSED:
        case InputEventType::GAMEPAD_BUTTON_RELEASED: {
            if (const auto* data = event.getEventData<GamepadButtonEventData>()) {
                key |= static_cast<std::uint64_t>(data->button) << 16;
            }
            break;
        }
        default:
            break;
        }

        return key;
    }

    bool FilterStage::canCoalesce(const InputEvent& a, const InputEvent& b) noexcept {
        // Basic compatibility check
        if (a.type != b.type || a.deviceId != b.deviceId) {
            return false;
        }

        // Only certain event types can be coalesced
        switch (a.type) {
        case InputEventType::MOUSE_MOVED: {
            // Mouse motion events can always be coalesced
            return true;
        }
        case InputEventType::GAMEPAD_AXIS_MOVED: {
            const auto* dataA = a.getEventData<GamepadAxisEventData>();
            const auto* dataB = b.getEventData<GamepadAxisEventData>();

            // Can coalesce if same axis
            return dataA && dataB && dataA->axis == dataB->axis;
        }
        case InputEventType::MOUSE_WHEEL: {
            // Wheel events can be coalesced
            return true;
        }
        default:
            // Button presses/releases and other discrete events cannot be coalesced
            return false;
        }
    }

    void FilterStage::mergeEvents(InputEvent& target, const InputEvent& source) noexcept {
        // Merge event data based on type
        switch (target.type) {
        case InputEventType::MOUSE_MOVED: {
            auto* targetData = target.getMutableEventData<MouseMotionEventData>();

            if (const auto* sourceData = source.getEventData<MouseMotionEventData>(); targetData && sourceData) {
                // Accumulate deltas and update final position
                targetData->delta += sourceData->delta;
                targetData->normalizedDelta += sourceData->normalizedDelta;
                targetData->position = sourceData->position; // Use latest position
            }
            break;
        }
        case InputEventType::GAMEPAD_AXIS_MOVED: {
            auto* targetData = target.getMutableEventData<GamepadAxisEventData>();

            if (const auto* sourceData = source.getEventData<GamepadAxisEventData>(); targetData && sourceData && targetData->axis == sourceData->axis) {
                // Use latest value and accumulate delta
                targetData->deltaValue += sourceData->deltaValue;
                targetData->value = sourceData->value; // Use latest value
            }
            break;
        }
        case InputEventType::MOUSE_WHEEL: {
            auto* targetData = target.getMutableEventData<MouseWheelEventData>();

            if (const auto* sourceData = source.getEventData<MouseWheelEventData>(); targetData && sourceData) {
                // Accumulate wheel deltas
                targetData->delta += sourceData->delta;
                targetData->position = sourceData->position; // Use latest position
            }
            break;
        }
        default:
            // Other event types don't support merging
            break;
        }

        // Update timestamp to latest
        target.timestamp = source.timestamp;
    }
} // namespace engine::input::processing
