/**
 * @file EventProcessor.h
 * @brief Central event processing pipeline
 * @author Andrés Guerrero
 * @date 12-09-2025
 *
 * Coordinates all event processing stages in a configurable pipeline.
 */

#pragma once

#include "../core/InputEvent.h"
#include "../core/InputEventQueue.h"
#include "../devices/base/DeviceService.h"

#include "../../memory/manager/MemoryManager.h"

#include <vector>
#include <memory>
#include <functional>
#include <mutex>

namespace engine::input::processing {

    // Forward declarations
    class FilterStage;
    class DeadzoneStage;
    class SensitivityStage;

    /**
     * @brief Event processor configuration
     */
    struct EventProcessorConfig {
        // Pipeline configuration
        bool enableFiltering = true;
        bool enableDeadzone = true;
        bool enableSensitivity = true;

        // Processing limits
        std::uint32_t maxEventsPerFrame = 256;
        std::uint32_t maxProcessingTime = 5;  // Milliseconds

        // Event batching
        bool enableBatching = true;
        std::uint32_t batchSize = 32;

        // Threading
        bool processAsync = false;
        std::uint32_t workerThreads = 1;

        // Memory
        bool useMemoryPools = true;
        std::size_t eventPoolSize = 1024;
    };

    /**
     * @brief Event processing pipeline
     *
     * Manages the complete event processing pipeline from raw input
     * to processed events ready for consumption.
     */
    class EventProcessor {
    public:
        using EventCallback = std::function<void(const InputEvent&)>;
        using ProcessingStage = std::function<bool(InputEvent&)>;
        using MemoryAllocator = engine::memory::MemoryManager;

        /**
         * @brief Constructor
         * @param deviceService Device service for input devices
         * @param memoryManager Memory manager for allocations
         */
        explicit EventProcessor(DeviceService* deviceService,
                               MemoryAllocator* memoryManager = nullptr) noexcept;

        /**
         * @brief Destructor
         */
        ~EventProcessor();

        // Disable copy, enable move
        EventProcessor(const EventProcessor&) = delete;
        EventProcessor& operator=(const EventProcessor&) = delete;
        EventProcessor(EventProcessor&&) noexcept = default;
        EventProcessor& operator=(EventProcessor&&) noexcept = default;

        // ============================================================================
        // Initialization and Configuration
        // ============================================================================

        /**
         * @brief Initialize the event processor
         */
        bool initialize(const EventProcessorConfig& config = {});

        /**
         * @brief Shutdown the event processor
         */
        void shutdown();

        /**
         * @brief Set configuration
         */
        void setConfig(const EventProcessorConfig& config);

        /**
         * @brief Get current configuration
         */
        [[nodiscard]] const EventProcessorConfig& getConfig() const noexcept {
            return config_;
        }

        // ============================================================================
        // Event Processing
        // ============================================================================

        /**
         * @brief Process all pending events
         * @param deltaTime Time since last update
         * @return Number of events processed
         */
        std::size_t processEvents(float deltaTime);

        /**
         * @brief Process single event through pipeline
         * @param event Event to process
         * @return True if event was processed successfully
         */
        bool processEvent(InputEvent& event) const;

        /**
         * @brief Process batch of events
         * @param events Events to process
         * @return Number of events successfully processed
         */
        std::size_t processBatch(std::vector<InputEvent>& events) const;

        /**
         * @brief Queue event for processing
         * @param event Event to queue
         * @return True if queued successfully
         */
        bool queueEvent(const InputEvent& event) const;

        /**
         * @brief Flush all pending events
         */
        void flush();

        // ============================================================================
        // Pipeline Management
        // ============================================================================

        /**
         * @brief Add custom processing stage
         * @param name Stage name
         * @param stage Processing function
         * @param priority Priority (lower = earlier in pipeline)
         */
        void addStage(const std::string& name, ProcessingStage stage, int priority = 100);

        /**
         * @brief Remove processing stage
         * @param name Stage name
         */
        void removeStage(const std::string& name);

        /**
         * @brief Enable/disable stage
         * @param name Stage name
         * @param enabled Enable state
         */
        void setStageEnabled(const std::string& name, bool enabled);

        /**
         * @brief Get filter stage
         */
        [[nodiscard]] FilterStage* getFilterStage() noexcept;
        [[nodiscard]] const FilterStage* getFilterStage() const noexcept;

        /**
         * @brief Get deadzone stage
         */
        [[nodiscard]] DeadzoneStage* getDeadzoneStage() noexcept;
        [[nodiscard]] const DeadzoneStage* getDeadzoneStage() const noexcept;

        /**
         * @brief Get sensitivity stage
         */
        [[nodiscard]] SensitivityStage* getSensitivityStage() noexcept;
        [[nodiscard]] const SensitivityStage* getSensitivityStage() const noexcept;

        // ============================================================================
        // Event Callbacks
        // ============================================================================

        /**
         * @brief Register event callback
         * @param callback Callback function
         */
        void registerEventCallback(EventCallback callback);

        /**
         * @brief Clear all callbacks
         */
        void clearCallbacks();

        // ============================================================================
        // Statistics and Debugging
        // ============================================================================

        /**
         * @brief Processing statistics
         */
        struct Statistics {
            std::uint64_t totalEventsProcessed = 0;
            std::uint64_t eventsDropped = 0;
            std::uint64_t eventsFiltered = 0;
            std::uint64_t batchesProcessed = 0;
            float averageProcessingTime = 0.0f;
            float peakProcessingTime = 0.0f;
            std::uint32_t queueHighWaterMark = 0;

            void reset() noexcept {
                totalEventsProcessed = 0;
                eventsDropped = 0;
                eventsFiltered = 0;
                batchesProcessed = 0;
                averageProcessingTime = 0.0f;
                peakProcessingTime = 0.0f;
                queueHighWaterMark = 0;
            }
        };

        [[nodiscard]] const Statistics& getStatistics() const noexcept {
            return stats_;
        }

        /**
         * @brief Reset statistics
         */
        void resetStatistics() const noexcept;

        /**
         * @brief Get queue size
         */
        [[nodiscard]] std::size_t getQueueSize() const noexcept;

        /**
         * @brief Check if processing
         */
        [[nodiscard]] bool isProcessing() const noexcept {
            return isProcessing_;
        }

    private:
        // Configuration
        EventProcessorConfig config_;
        bool initialized_ = false;

        // Device service
        DeviceService* deviceService_;

        // Memory management
        MemoryAllocator* memoryManager_;
        bool ownsMemoryManager_ = false;
        std::unique_ptr<engine::memory::PoolAllocator> eventPool_;

        // Event queues
        std::unique_ptr<InputEventQueue<1024>> inputQueue_;
        std::unique_ptr<PriorityEventQueue> priorityQueue_;
        std::vector<InputEvent> processingBuffer_;

        // Processing stages
        std::unique_ptr<FilterStage> filterStage_;
        std::unique_ptr<DeadzoneStage> deadzoneStage_;
        std::unique_ptr<SensitivityStage> sensitivityStage_;

        // Custom stages
        struct CustomStage {
            std::string name;
            ProcessingStage stage;
            int priority;
            bool enabled;

            bool operator<(const CustomStage& other) const noexcept {
                return priority < other.priority;
            }
        };

        std::vector<CustomStage> customStages_;
        mutable std::mutex stagesMutex_;

        // Callbacks
        std::vector<EventCallback> callbacks_;
        mutable std::mutex callbacksMutex_;

        // Statistics
        mutable Statistics stats_;
        std::chrono::steady_clock::time_point lastProcessTime_;

        // State
        std::atomic<bool> isProcessing_{false};

        // ============================================================================
        // Internal Methods
        // ============================================================================

        /**
         * @brief Create processing stages
         */
        void createStages();

        /**
         * @brief Process event through pipeline
         */
        bool processThroughPipeline(InputEvent& event) const;

        /**
         * @brief Invoke callbacks for event
         */
        void invokeCallbacks(const InputEvent& event) const;

        /**
         * @brief Collect events from devices
         */
        std::size_t collectDeviceEvents() const;

        /**
         * @brief Update processing statistics
         */
        void updateStatistics(std::size_t eventsProcessed, float processingTime);

        /**
         * @brief Allocate event from pool
         */
        [[nodiscard]] InputEvent* allocateEvent() const;

        /**
         * @brief Deallocate event to pool
         */
        void deallocateEvent(InputEvent* event) const;
    };

} // namespace engine::input::processing