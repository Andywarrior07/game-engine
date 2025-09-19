/**
 * @file InputLifecycle.h
 * @brief Input system lifecycle management
 * @author Andrés Guerrero
 * @date 13-09-2025
 *
 * Manages the initialization, update, and shutdown lifecycle of the
 * input system and all its subsystems.
 */

#pragma once

#include "../core/InputConfig.h"

#include "../../memory/manager/MemoryManager.h"

#include <vector>
#include <string>
#include <functional>
#include <memory>
#include <atomic>
#include <chrono>

namespace engine::input {
    namespace processing {
        class EventProcessor;
        class SnapshotManager;
    }

    namespace mapping {
        class ContextStack;
    }

    // Forward declarations
    class InputService;
    class DeviceService;
    class ActionMap;
    class SDLInputBackend;
    class InputMemoryPools;

    /**
     * @brief Input system lifecycle state
     */
    enum class LifecycleState : std::uint8_t {
        UNINITIALIZED = 0,
        INITIALIZING,
        INITIALIZED,
        RUNNING,
        PAUSED,
        SHUTTING_DOWN,
        SHUTDOWN,
        ERROR_STATE
    };

    /**
     * @brief Lifecycle event types
     */
    enum class LifecycleEvent : std::uint8_t {
        PRE_INIT,
        POST_INIT,
        PRE_UPDATE,
        POST_UPDATE,
        PRE_SHUTDOWN,
        POST_SHUTDOWN,
        STATE_CHANGED,
        ERROR_OCCURRED,
        MEMORY_PRESSURE,
        PERFORMANCE_WARNING
    };

    /**
     * @brief Lifecycle configuration
     */
    struct LifecycleConfig {
        // Initialization settings
        bool autoInitializeDevices = true;
        bool validateConfiguration = true;
        bool preallocateMemory = true;
        float initializationTimeout = 5.0f; // Seconds

        // Update settings
        float targetUpdateRate = 60.0f; // Hz
        float maxUpdateDelta = 0.1f; // Maximum delta time
        bool fixedTimestep = false; // Use fixed timestep
        float fixedDeltaTime = 0.016667f; // 60 FPS

        // Shutdown settings
        bool gracefulShutdown = true; // Wait for pending operations
        float shutdownTimeout = 2.0f; // Seconds
        bool saveStateOnShutdown = false; // Save input state

        // Error handling
        bool continueOnError = false; // Continue after non-critical errors
        std::uint32_t maxErrorRetries = 3; // Retry count for operations

        // Performance monitoring
        bool enableProfiling = true;
        float performanceWarningThreshold = 0.020f; // 20ms
    };

    /**
     * @brief Manages input system lifecycle
     *
     * Coordinates initialization, updates, and shutdown of all input
     * system components in the correct order with proper error handling.
     */
    class InputLifecycle {
    public:
        using MemoryAllocator = memory::MemoryManager;
        using LifecycleCallback = std::function<void(LifecycleEvent)>;
        using ErrorCallback = std::function<void(const std::string&)>;

        /**
         * @brief Constructor
         * @param memoryManager Memory manager for allocations
         */
        explicit InputLifecycle(MemoryAllocator* memoryManager = nullptr) noexcept;

        /**
         * @brief Destructor
         */
        ~InputLifecycle();

        // Disable copy, enable move
        InputLifecycle(const InputLifecycle&) = delete;
        InputLifecycle& operator=(const InputLifecycle&) = delete;
        InputLifecycle(InputLifecycle&&) noexcept = default;
        InputLifecycle& operator=(InputLifecycle&&) noexcept = default;

        // ============================================================================
        // Lifecycle Management
        // ============================================================================

        /**
         * @brief Initialize the input system
         * @param config System configuration
         * @param lifecycleConfig Lifecycle configuration
         * @return True if successful
         */
        bool initialize(const InputConfig& config,
                        const LifecycleConfig& lifecycleConfig = {});

        /**
         * @brief Start the input system
         * @return True if successful
         */
        bool start();

        /**
         * @brief Update the input system
         * @param deltaTime Time since last update
         * @return True if successful
         */
        bool update(float deltaTime);

        /**
         * @brief Pause the input system
         */
        void pause();

        /**
         * @brief Resume the input system
         */
        void resume();

        /**
         * @brief Shutdown the input system
         * @return True if successful
         */
        bool shutdown();

        /**
         * @brief Reset the input system
         * @return True if successful
         */
        bool reset();

        /**
         * @brief Get current lifecycle state
         */
        [[nodiscard]] LifecycleState getState() const noexcept {
            return state_.load(std::memory_order_acquire);
        }

        /**
         * @brief Check if system is running
         */
        [[nodiscard]] bool isRunning() const noexcept {
            return getState() == LifecycleState::RUNNING;
        }

        /**
         * @brief Check if system is initialized
         */
        [[nodiscard]] bool isInitialized() const noexcept {
            const auto state = getState();

            return state == LifecycleState::INITIALIZED ||
                state == LifecycleState::RUNNING ||
                state == LifecycleState::PAUSED;
        }

        // ============================================================================
        // Subsystem Access
        // ============================================================================

        /**
         * @brief Get input service
         */
        [[nodiscard]] InputService* getInputService() const noexcept {
            return inputService_;
        }

        /**
         * @brief Get device service
         */
        [[nodiscard]] DeviceService* getDeviceService() const noexcept {
            return deviceService_;
        }

        /**
         * @brief Get event processor
         */
        [[nodiscard]] processing::EventProcessor* getEventProcessor() const noexcept {
            return eventProcessor_;
        }

        /**
         * @brief Get snapshot manager
         */
        [[nodiscard]] processing::SnapshotManager* getSnapshotManager() const noexcept {
            return snapshotManager_;
        }

        /**
         * @brief Get action map
         */
        [[nodiscard]] ActionMap* getActionMap() const noexcept {
            return actionMap_;
        }

        /**
         * @brief Get context stack
         */
        [[nodiscard]] mapping::ContextStack* getContextStack() const noexcept {
            return contextStack_;
        }

        // ============================================================================
        // Configuration
        // ============================================================================

        /**
         * @brief Set lifecycle configuration
         * @param config New configuration
         */
        void setLifecycleConfig(const LifecycleConfig& config) noexcept {
            lifecycleConfig_ = config;
        }

        /**
         * @brief Get lifecycle configuration
         */
        [[nodiscard]] const LifecycleConfig& getLifecycleConfig() const noexcept {
            return lifecycleConfig_;
        }

        /**
         * @brief Hot-reload configuration
         * @param config New input configuration
         * @return True if successful
         */
        bool reloadConfiguration(const InputConfig& config);

        // ============================================================================
        // Callbacks
        // ============================================================================

        /**
         * @brief Register lifecycle callback
         * @param callback Callback function
         */
        void registerLifecycleCallback(LifecycleCallback callback);

        /**
         * @brief Register error callback
         * @param callback Error callback function
         */
        void registerErrorCallback(ErrorCallback callback);

        /**
         * @brief Clear all callbacks
         */
        void clearCallbacks();

        // ============================================================================
        // Performance Monitoring
        // ============================================================================

        /**
         * @brief Performance statistics
         */
        struct PerformanceStats {
            float averageUpdateTime = 0.0f; // Milliseconds
            float peakUpdateTime = 0.0f; // Milliseconds
            float averageFrameTime = 0.0f; // Milliseconds
            std::uint32_t slowFrames = 0; // Frames over threshold
            std::uint32_t droppedFrames = 0; // Frames skipped
            std::uint64_t totalFrames = 0; // Total frames processed

            void reset() noexcept {
                averageUpdateTime = 0.0f;
                peakUpdateTime = 0.0f;
                averageFrameTime = 0.0f;
                slowFrames = 0;
                droppedFrames = 0;
                totalFrames = 0;
            }
        };

        /**
         * @brief Get performance statistics
         */
        [[nodiscard]] const PerformanceStats& getPerformanceStats() const noexcept {
            return perfStats_;
        }

        /**
         * @brief Reset performance statistics
         */
        void resetPerformanceStats() noexcept {
            perfStats_.reset();
        }

        // ============================================================================
        // Error Handling
        // ============================================================================

        /**
         * @brief Get last error message
         */
        [[nodiscard]] const std::string& getLastError() const noexcept {
            return lastError_;
        }

        /**
         * @brief Clear error state
         */
        void clearError() noexcept {
            lastError_.clear();
            errorCount_ = 0;
        }

        /**
         * @brief Get error count
         */
        [[nodiscard]] std::uint32_t getErrorCount() const noexcept {
            return errorCount_;
        }

        /**
         * @brief Transition to new state
         * @param newState Target state
         * @return True if transition successful
         */
        bool transitionTo(LifecycleState newState);

    private:
        // ============================================================================
        // Member Variables
        // ============================================================================

        // State
        std::atomic<LifecycleState> state_{LifecycleState::UNINITIALIZED};

        // Configuration
        InputConfig inputConfig_;
        LifecycleConfig lifecycleConfig_;

        // Memory management
        MemoryAllocator* memoryManager_;
        bool ownsMemoryManager_ = false;
        std::unique_ptr<InputMemoryPools> memoryPools_;

        // Subsystem pointers (non-owning)
        InputService* inputService_ = nullptr;
        DeviceService* deviceService_ = nullptr;
        processing::EventProcessor* eventProcessor_ = nullptr;
        processing::SnapshotManager* snapshotManager_ = nullptr;
        ActionMap* actionMap_ = nullptr;
        mapping::ContextStack* contextStack_ = nullptr;
        SDLInputBackend* sdlBackend_ = nullptr;

        // Callbacks
        std::vector<LifecycleCallback> lifecycleCallbacks_;
        std::vector<ErrorCallback> errorCallbacks_;

        // Performance monitoring
        PerformanceStats perfStats_;
        std::chrono::steady_clock::time_point lastUpdateTime_;
        float accumulatedTime_ = 0.0f;

        // Error handling
        std::string lastError_;
        std::atomic<std::uint32_t> errorCount_{0};

        // ============================================================================
        // Internal Methods
        // ============================================================================

        /**
         * @brief Initialize subsystems
         * @return True if successful
         */
        bool initializeSubsystems();

        /**
         * @brief Shutdown subsystems
         */
        void shutdownSubsystems();

        /**
         * @brief Validate system configuration
         * @return True if configuration is valid
         */
        [[nodiscard]] bool validateConfiguration() const;

        /**
         * @brief Update timing information
         * @param deltaTime Frame delta time
         */
        void updateTiming(float deltaTime);

        /**
         * @brief Handle error
         * @param error Error message
         * @param critical True if critical error
         * @return True if error was handled
         */
        bool handleError(const std::string& error, bool critical = false);

        /**
         * @brief Notify lifecycle event
         * @param event Event type
         */
        void notifyLifecycleEvent(LifecycleEvent event);

        /**
         * @brief Notify error
         * @param error Error message
         */
        void notifyError(const std::string& error);

        /**
         * @brief Perform fixed timestep update
         * @param deltaTime Frame delta time
         * @return Number of fixed steps performed
         */
        std::uint32_t performFixedUpdate(float deltaTime);

        /**
         * @brief Update performance statistics
         * @param updateTime Time taken for update
         */
        void updatePerformanceStats(float updateTime);

        /**
         * @brief Check memory pressure
         * @return True if under memory pressure
         */
        [[nodiscard]] bool checkMemoryPressure() const;

        /**
         * @brief Recover from error state
         * @return True if recovery successful
         */
        bool recoverFromError();
    };

    // ============================================================================
    // Lifecycle Guard (RAII)
    // ============================================================================

    /**
     * @brief RAII guard for lifecycle operations
     *
     * Ensures proper state transitions and cleanup on scope exit.
     */
    class LifecycleGuard {
    public:
        /**
         * @brief Constructor
         * @param lifecycle Lifecycle manager
         * @param targetState Target state on construction
         * @param revertState State to revert to on destruction
         */
        LifecycleGuard(InputLifecycle& lifecycle,
                       const LifecycleState targetState,
                       const LifecycleState revertState)
            : lifecycle_(lifecycle)
              , revertState_(revertState)
              , shouldRevert_(true) {
            previousState_ = lifecycle_.getState();
            lifecycle_.transitionTo(targetState);
        }

        /**
         * @brief Destructor - reverts state if needed
         */
        ~LifecycleGuard() {
            if (shouldRevert_) {
                lifecycle_.transitionTo(revertState_);
            }
        }

        /**
         * @brief Commit the state change (prevent revert)
         */
        void commit() noexcept {
            shouldRevert_ = false;
        }

        /**
         * @brief Get previous state
         */
        [[nodiscard]] LifecycleState getPreviousState() const noexcept {
            return previousState_;
        }

    private:
        InputLifecycle& lifecycle_;
        LifecycleState previousState_;
        LifecycleState revertState_;
        bool shouldRevert_;
    };

    // ============================================================================
    // Lifecycle Phase
    // ============================================================================

    /**
     * @brief Represents a phase in the lifecycle
     */
    class LifecyclePhase {
    public:
        using PhaseFunction = std::function<bool()>;

        /**
         * @brief Constructor
         * @param name Phase name
         * @param function Phase function
         * @param timeout Timeout in seconds (0 = no timeout)
         */
        LifecyclePhase(std::string name,
                       PhaseFunction function,
                       const float timeout = 0.0f)
            : name_(std::move(name))
              , function_(std::move(function))
              , timeout_(timeout) {
        }

        /**
         * @brief Execute the phase
         * @return True if successful
         */
        [[nodiscard]] bool execute() const {
            if (!function_) {
                return false;
            }

            if (timeout_ > 0.0f) {
                // Execute with timeout
                const auto start = std::chrono::steady_clock::now();
                const bool result = function_();
                const auto end = std::chrono::steady_clock::now();

                if (const float elapsed = std::chrono::duration<float>(end - start).count(); elapsed > timeout_) {
                    return false; // Timeout exceeded
                }

                return result;
            }
            else {
                // Execute without timeout
                return function_();
            }
        }

        /**
         * @brief Get phase name
         */
        [[nodiscard]] const std::string& getName() const noexcept {
            return name_;
        }

        /**
         * @brief Get timeout
         */
        [[nodiscard]] float getTimeout() const noexcept {
            return timeout_;
        }

    private:
        std::string name_;
        PhaseFunction function_;
        float timeout_;
    };

    // ============================================================================
    // Lifecycle Builder (Fluent Interface)
    // ============================================================================

    /**
     * @brief Builder for configuring lifecycle
     */
    class LifecycleBuilder {
    public:
        LifecycleBuilder() = default;

        /**
         * @brief Set memory manager
         */
        LifecycleBuilder& withMemoryManager(engine::memory::MemoryManager* manager) {
            memoryManager_ = manager;
            return *this;
        }

        /**
         * @brief Set input configuration
         */
        LifecycleBuilder& withInputConfig(const InputConfig& config) {
            inputConfig_ = config;
            return *this;
        }

        /**
         * @brief Set lifecycle configuration
         */
        LifecycleBuilder& withLifecycleConfig(const LifecycleConfig& config) {
            lifecycleConfig_ = config;
            return *this;
        }

        /**
         * @brief Enable profiling
         */
        LifecycleBuilder& withProfiling(const bool enabled = true) {
            lifecycleConfig_.enableProfiling = enabled;
            return *this;
        }

        /**
         * @brief Set update rate
         */
        LifecycleBuilder& withUpdateRate(const float hz) {
            lifecycleConfig_.targetUpdateRate = hz;
            return *this;
        }

        /**
         * @brief Enable fixed timestep
         */
        LifecycleBuilder& withFixedTimestep(const float deltaTime) {
            lifecycleConfig_.fixedTimestep = true;
            lifecycleConfig_.fixedDeltaTime = deltaTime;
            return *this;
        }

        /**
         * @brief Add lifecycle callback
         */
        LifecycleBuilder& withLifecycleCallback(InputLifecycle::LifecycleCallback callback) {
            lifecycleCallbacks_.push_back(std::move(callback));
            return *this;
        }

        /**
         * @brief Add error callback
         */
        LifecycleBuilder& withErrorCallback(InputLifecycle::ErrorCallback callback) {
            errorCallbacks_.push_back(std::move(callback));
            return *this;
        }

        /**
         * @brief Build lifecycle manager
         */
        [[nodiscard]] std::unique_ptr<InputLifecycle> build() {
            auto lifecycle = std::make_unique<InputLifecycle>(memoryManager_);

            // Register callbacks
            for (auto& callback : lifecycleCallbacks_) {
                lifecycle->registerLifecycleCallback(std::move(callback));
            }
            for (auto& callback : errorCallbacks_) {
                lifecycle->registerErrorCallback(std::move(callback));
            }

            // Initialize
            if (!lifecycle->initialize(inputConfig_, lifecycleConfig_)) {
                return nullptr;
            }

            return lifecycle;
        }

    private:
        engine::memory::MemoryManager* memoryManager_ = nullptr;
        InputConfig inputConfig_;
        LifecycleConfig lifecycleConfig_;
        std::vector<InputLifecycle::LifecycleCallback> lifecycleCallbacks_;
        std::vector<InputLifecycle::ErrorCallback> errorCallbacks_;
    };
} // namespace engine::input
