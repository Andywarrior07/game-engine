/**
 * @file EventProcessor.cpp
 * @brief Central event processing pipeline implementation
 * @author Andrés Guerrero
 * @date 12-09-2025
 */

#include "EventProcessor.h"

#include "stages/FilterStage.h"
#include "stages/DeadzoneStage.h"
#include "stages/SensitivityStage.h"

#include "../../memory/allocators/PoolAllocator.h"

#include <algorithm>
#include <chrono>

namespace engine::input::processing {
    EventProcessor::EventProcessor(DeviceService* deviceService,
                                   MemoryAllocator* memoryManager) noexcept
        : deviceService_(deviceService)
          , memoryManager_(memoryManager) {
        processingBuffer_.reserve(256);

        // TODO: Revisar esto
        // If no memory manager provided, we'll use default allocations
        if (!memoryManager_) {
            // In production, would get global memory manager
            ownsMemoryManager_ = false;
        }
    }

    EventProcessor::~EventProcessor() {
        shutdown();
    }

    bool EventProcessor::initialize(const EventProcessorConfig& config) {
        if (initialized_) {
            return false;
        }

        config_ = config;

        // Create event queues
        inputQueue_ = std::make_unique<InputEventQueue<1024>>();
        priorityQueue_ = std::make_unique<PriorityEventQueue>();

        // Create event pool if using memory pools
        if (config_.useMemoryPools && memoryManager_) {
            eventPool_ = std::make_unique<engine::memory::PoolAllocator>(
                sizeof(InputEvent),
                config_.eventPoolSize,
                alignof(InputEvent),
                "EventPool"
            );
        }

        // Create processing stages
        createStages();

        // Reserve buffer space
        processingBuffer_.reserve(config_.maxEventsPerFrame);

        initialized_ = true;
        return true;
    }

    void EventProcessor::shutdown() {
        if (!initialized_) {
            return;
        }

        // Wait for processing to complete
        while (isProcessing_.load(std::memory_order_acquire)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // Clear queues
        flush();

        // Clear stages
        filterStage_.reset();
        deadzoneStage_.reset();
        sensitivityStage_.reset();
        customStages_.clear();

        // Clear callbacks
        clearCallbacks();

        // Release memory pool
        eventPool_.reset();

        initialized_ = false;
    }

    void EventProcessor::setConfig(const EventProcessorConfig& config) {
        config_ = config;

        // Update stage enable states
        if (deadzoneStage_) {
            deadzoneStage_->setEnabled(config.enableDeadzone);
        }
        if (sensitivityStage_) {
            sensitivityStage_->setEnabled(config.enableSensitivity);
        }
    }

    std::size_t EventProcessor::processEvents(const float deltaTime) {
        if (!initialized_ || !deviceService_) {
            return 0;
        }

        // Set processing flag
        if (bool expected = false; !isProcessing_.compare_exchange_strong(expected, true, std::memory_order_acquire)) {
            return 0; // Already processing
        }

        auto processingStart = std::chrono::steady_clock::now();
        std::size_t totalProcessed = 0;

        // TODO: Revisar esto
        // Collect events from devices
        std::size_t collected = collectDeviceEvents();

        // Process priority queue first
        while (!priorityQueue_->empty()) {
            if (InputEvent event; priorityQueue_->pop(event)) {
                processingBuffer_.push_back(event);
            }
        }

        // Process regular queue
        std::size_t dequeued = 0;
        while (dequeued < config_.maxEventsPerFrame && !inputQueue_->empty()) {
            if (InputEvent event; inputQueue_->pop(event)) {
                processingBuffer_.push_back(event);
                dequeued++;
            }
        }

        // Process events through pipeline
        if (config_.enableBatching && processingBuffer_.size() >= config_.batchSize) {
            // Batch processing
            totalProcessed = processBatch(processingBuffer_);
        }
        else {
            // Individual processing
            for (auto& event : processingBuffer_) {
                if (processEvent(event)) {
                    totalProcessed++;
                }
            }
        }

        // Process through stages if enabled
        if (config_.enableFiltering && filterStage_) {
            filterStage_->process(processingBuffer_, deltaTime);
        }

        if (config_.enableDeadzone && deadzoneStage_) {
            deadzoneStage_->process(processingBuffer_, deltaTime);
        }

        if (config_.enableSensitivity && sensitivityStage_) {
            sensitivityStage_->process(processingBuffer_, deltaTime);
        }

        // Invoke callbacks for processed events
        for (const auto& event : processingBuffer_) {
            if (!event.consumed) {
                invokeCallbacks(event);
            }
        }

        // Clear processing buffer for next frame
        processingBuffer_.clear();

        // Update statistics
        const auto processingEnd = std::chrono::steady_clock::now();
        const auto processingTime = std::chrono::duration<float, std::milli>(processingEnd - processingStart).count();
        updateStatistics(totalProcessed, processingTime);

        // Clear processing flag
        isProcessing_.store(false, std::memory_order_release);

        return totalProcessed;
    }

    bool EventProcessor::processEvent(InputEvent& event) const {
        if (!initialized_) {
            return false;
        }

        return processThroughPipeline(event);
    }

    std::size_t EventProcessor::processBatch(std::vector<InputEvent>& events) const {
        if (!initialized_ || events.empty()) {
            return 0;
        }

        std::size_t processed = 0;

        // Process in parallel if configured
        if (config_.processAsync && config_.workerThreads > 1) {
            std::atomic<std::size_t> atomicProcessed{0};

            std::ranges::for_each(events,
                                  [this, &atomicProcessed](InputEvent& event) {
                                      if (processThroughPipeline(event)) {
                                          atomicProcessed.fetch_add(1, std::memory_order_relaxed);
                                      }
                                  });


            processed = atomicProcessed.load(std::memory_order_acquire);
        }
        else {
            // Sequential processing
            for (auto& event : events) {
                if (processThroughPipeline(event)) {
                    processed++;
                }
            }
        }

        stats_.batchesProcessed++;
        return processed;
    }

    bool EventProcessor::queueEvent(const InputEvent& event) const {
        if (!initialized_ || !inputQueue_) {
            return false;
        }

        bool queued = false;

        // Try to queue based on priority
        if (event.type == InputEventType::DEVICE_ADDED ||
            event.type == InputEventType::DEVICE_REMOVED ||
            event.type == InputEventType::GAMEPAD_CONNECTED ||
            event.type == InputEventType::GAMEPAD_DISCONNECTED) {
            queued = priorityQueue_->push(event, PriorityEventQueue::Priority::CRITICAL);
        }
        else {
            queued = inputQueue_->push(event);
        }

        if (!queued) {
            stats_.eventsDropped++;
        }

        // Update high water mark
        if (const std::size_t queueSize = getQueueSize(); queueSize > stats_.queueHighWaterMark) {
            stats_.queueHighWaterMark = static_cast<std::uint32_t>(queueSize);
        }

        return queued;
    }

    void EventProcessor::flush() {
        if (inputQueue_) {
            inputQueue_->clear();
        }
        if (priorityQueue_) {
            priorityQueue_->clear();
        }
        processingBuffer_.clear();
    }

    void EventProcessor::addStage(const std::string& name, ProcessingStage stage, const int priority) {
        std::lock_guard lock(stagesMutex_);

        // Remove existing stage with same name
        const auto it = std::ranges::remove_if(customStages_,
                                               [&name](const CustomStage& s) { return s.name == name; }).begin();
        customStages_.erase(it, customStages_.end());

        // Add new stage
        customStages_.push_back({name, std::move(stage), priority, true});

        // Sort by priority
        std::sort(customStages_.begin(), customStages_.end());
    }

    void EventProcessor::removeStage(const std::string& name) {
        std::lock_guard lock(stagesMutex_);

        const auto it = std::ranges::remove_if(customStages_,
                                         [&name](const CustomStage& s) { return s.name == name; }).begin();
        customStages_.erase(it, customStages_.end());
    }

    void EventProcessor::setStageEnabled(const std::string& name, const bool enabled) {
        std::lock_guard lock(stagesMutex_);

        for (auto& stage : customStages_) {
            if (stage.name == name) {
                stage.enabled = enabled;
                break;
            }
        }
    }

    FilterStage* EventProcessor::getFilterStage() noexcept {
        return filterStage_.get();
    }

    const FilterStage* EventProcessor::getFilterStage() const noexcept {
        return filterStage_.get();
    }

    DeadzoneStage* EventProcessor::getDeadzoneStage() noexcept {
        return deadzoneStage_.get();
    }

    const DeadzoneStage* EventProcessor::getDeadzoneStage() const noexcept {
        return deadzoneStage_.get();
    }

    SensitivityStage* EventProcessor::getSensitivityStage() noexcept {
        return sensitivityStage_.get();
    }

    const SensitivityStage* EventProcessor::getSensitivityStage() const noexcept {
        return sensitivityStage_.get();
    }

    void EventProcessor::registerEventCallback(EventCallback callback) {
        std::lock_guard lock(callbacksMutex_);
        callbacks_.push_back(std::move(callback));
    }

    void EventProcessor::clearCallbacks() {
        std::lock_guard lock(callbacksMutex_);
        callbacks_.clear();
    }

    void EventProcessor::resetStatistics() const noexcept {
        stats_.reset();
    }

    std::size_t EventProcessor::getQueueSize() const noexcept {
        std::size_t size = 0;
        if (inputQueue_) {
            size += inputQueue_->size();
        }
        if (priorityQueue_) {
            size += priorityQueue_->size();
        }
        return size;
    }

    // ============================================================================
    // Private Methods Implementation
    // ============================================================================

    void EventProcessor::createStages() {
        // Create filter stage
        if (config_.enableFiltering) {
            FilterConfig filterConfig;
            filterConfig.enableNoiseFilter = true;
            filterConfig.removeDuplicates = true;
            filterConfig.enableCoalescing = true;
            filterStage_ = std::make_unique<FilterStage>(filterConfig);
        }

        // Create deadzone stage
        if (config_.enableDeadzone) {
            DeadzoneConfig deadzoneConfig;
            deadzoneStage_ = std::make_unique<DeadzoneStage>(deadzoneConfig);
        }

        // Create sensitivity stage
        if (config_.enableSensitivity) {
            SensitivityConfig sensitivityConfig;
            sensitivityStage_ = std::make_unique<SensitivityStage>(sensitivityConfig);
        }
    }

    bool EventProcessor::processThroughPipeline(InputEvent& event) const {
        // Skip consumed events
        if (event.consumed) {
            return false;
        }

        // Process through custom stages first
        {
            std::lock_guard lock(stagesMutex_);
            for (const auto& stage : customStages_) {
                if (stage.enabled && !stage.stage(event)) {
                    stats_.eventsFiltered++;
                    return false;
                }
            }
        }

        stats_.totalEventsProcessed++;
        return true;
    }

    void EventProcessor::invokeCallbacks(const InputEvent& event) const {
        std::lock_guard lock(callbacksMutex_);
        for (const auto& callback : callbacks_) {
            if (callback) {
                callback(event);

                // If event was consumed by callback, stop processing
                if (event.consumed) {
                    break;
                }
            }
        }
    }

    std::size_t EventProcessor::collectDeviceEvents() const {
        if (!deviceService_) {
            return 0;
        }

        std::size_t collected = 0;

        // Reserve space for device events to avoid allocations
        thread_local std::vector<InputEvent> deviceEvents;
        deviceEvents.clear();
        deviceEvents.reserve(64);

        // Process events from all devices
        deviceService_->processAllEvents([&collected](const InputEvent& event) {
            deviceEvents.push_back(event);
            collected++;
        });

        // Queue collected events
        for (const auto& event : deviceEvents) {
            return queueEvent(event);
        }

        return collected;
    }

    // TODO: Revisar esto
    void EventProcessor::updateStatistics(std::size_t eventsProcessed, float processingTime) {
        // Update average processing time using exponential moving average
        constexpr float alpha = 0.1f;
        stats_.averageProcessingTime = (1.0f - alpha) * stats_.averageProcessingTime + alpha * processingTime;

        // Update peak processing time
        if (processingTime > stats_.peakProcessingTime) {
            stats_.peakProcessingTime = processingTime;
        }

        lastProcessTime_ = std::chrono::steady_clock::now();
    }

    [[nodiscard]] InputEvent* EventProcessor::allocateEvent() const {
        if (eventPool_) {
            if (void* memory = eventPool_->allocate(sizeof(InputEvent), alignof(InputEvent))) {
                return new(memory) InputEvent();
            }
        }

        // Fallback to heap allocation
        return new InputEvent();
    }

    void EventProcessor::deallocateEvent(InputEvent* event) const {
        if (!event) return;

        if (eventPool_ && eventPool_->owns(event)) {
            event->~InputEvent();
            eventPool_->deallocate(event);
        }
        else {
            delete event;
        }
    }
} // namespace engine::input::processing
