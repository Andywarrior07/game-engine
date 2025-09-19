/**
 * @file InputLifecycle.cpp
 * @brief Input system lifecycle management implementation
 * @author Andrés Guerrero
 * @date 13-09-2025
 *
 * Manages the initialization, update, and shutdown lifecycle of the
 * input system and all its subsystems.
 */

#include "InputLifecycle.h"

#include "../manager/InputService.h"
#include "../devices/base/DeviceService.h"
#include "../processing/EventProcessor.h"
#include "../processing/SnapshotManager.h"
#include "../mapping/ActionMap.h"
#include "../mapping/ContextStack.h"
#include "../platform/sdl2/SDLInputBackend.h"
#include "../core/InputMemoryPools.h"

#include <algorithm>
#include <sstream>
#include <iomanip>

namespace engine::input {
    // ============================================================================
    // InputLifecycle Implementation
    // ============================================================================

    InputLifecycle::InputLifecycle(MemoryAllocator* memoryManager) noexcept
        : memoryManager_(memoryManager)
          , ownsMemoryManager_(false) {
        // Reserve space for callbacks to avoid allocations during runtime
        lifecycleCallbacks_.reserve(8);
        errorCallbacks_.reserve(4);

        // Initialize timing
        lastUpdateTime_ = std::chrono::steady_clock::now();
    }

    InputLifecycle::~InputLifecycle() {
        // Ensure clean shutdown if still initialized
        if (isInitialized()) {
            shutdown();
        }

        // Clean up owned memory manager if applicable
        if (ownsMemoryManager_ && memoryManager_) {
            // In production, would properly shutdown memory manager
            memoryManager_ = nullptr;
        }
    }

    // ============================================================================
    // Lifecycle Management
    // ============================================================================

    bool InputLifecycle::initialize(const InputConfig& config,
                                    const LifecycleConfig& lifecycleConfig) {
        // Check if already initialized
        const auto currentState = state_.load(std::memory_order_acquire);
        if (currentState != LifecycleState::UNINITIALIZED) {
            handleError("Cannot initialize: system is not in UNINITIALIZED state");
            return false;
        }

        // Transition to initializing
        if (!transitionTo(LifecycleState::INITIALIZING)) {
            return false;
        }

        // Store configurations
        inputConfig_ = config;
        lifecycleConfig_ = lifecycleConfig;

        // Notify pre-init
        notifyLifecycleEvent(LifecycleEvent::PRE_INIT);

        // Validate configuration if requested
        if (lifecycleConfig_.validateConfiguration && !validateConfiguration()) {
            handleError("Configuration validation failed", true);
            transitionTo(LifecycleState::ERROR_STATE);
            return false;
        }

        // Initialize with timeout if specified
        bool initSuccess = false;
        if (lifecycleConfig_.initializationTimeout > 0.0f) {
            const auto startTime = std::chrono::steady_clock::now();
            const auto timeout = std::chrono::milliseconds(
                static_cast<long>(lifecycleConfig_.initializationTimeout * 1000)
            );

            initSuccess = initializeSubsystems();

            if (const auto elapsed = std::chrono::steady_clock::now() - startTime; elapsed > timeout) {
                handleError("Initialization timeout exceeded", true);
                shutdownSubsystems();
                transitionTo(LifecycleState::ERROR_STATE);
                return false;
            }
        }
        else {
            initSuccess = initializeSubsystems();
        }

        if (!initSuccess) {
            handleError("Failed to initialize subsystems", true);
            shutdownSubsystems();
            transitionTo(LifecycleState::ERROR_STATE);
            return false;
        }

        // Transition to initialized
        if (!transitionTo(LifecycleState::INITIALIZED)) {
            shutdownSubsystems();
            return false;
        }

        // Notify post-init
        notifyLifecycleEvent(LifecycleEvent::POST_INIT);

        return true;
    }

    bool InputLifecycle::start() {
        const auto currentState = state_.load(std::memory_order_acquire);
        if (currentState != LifecycleState::INITIALIZED) {
            handleError("Cannot start: system is not initialized");
            return false;
        }

        // Transition to running
        if (!transitionTo(LifecycleState::RUNNING)) {
            return false;
        }

        // Reset timing
        lastUpdateTime_ = std::chrono::steady_clock::now();
        accumulatedTime_ = 0.0f;

        // Reset performance stats
        perfStats_.reset();

        return true;
    }

    bool InputLifecycle::update(const float deltaTime) {
        const auto currentState = state_.load(std::memory_order_acquire);

        // Can only update if running
        if (currentState != LifecycleState::RUNNING) {
            if (currentState == LifecycleState::PAUSED) {
                // Skip update while paused
                return true;
            }
            return false;
        }

        const auto updateStart = std::chrono::steady_clock::now();

        // Notify pre-update
        notifyLifecycleEvent(LifecycleEvent::PRE_UPDATE);

        bool updateSuccess = false;

        // Handle fixed timestep if enabled
        if (lifecycleConfig_.fixedTimestep) {
            const std::uint32_t steps = performFixedUpdate(deltaTime);
            updateSuccess = steps > 0;
        }
        else {
            // Variable timestep update
            const float clampedDelta = std::min(deltaTime, lifecycleConfig_.maxUpdateDelta);

            // Update input service
            if (inputService_) {
                inputService_->tick(clampedDelta);
                updateSuccess = true;
            }
        }

        // Update timing
        updateTiming(deltaTime);

        // Calculate update time
        const auto updateEnd = std::chrono::steady_clock::now();
        const float updateTime = std::chrono::duration<float, std::milli>(
            updateEnd - updateStart
        ).count();

        // Update performance stats
        updatePerformanceStats(updateTime);

        // Check for performance warnings
        if (lifecycleConfig_.enableProfiling &&
            updateTime > lifecycleConfig_.performanceWarningThreshold * 1000.0f) {
            notifyLifecycleEvent(LifecycleEvent::PERFORMANCE_WARNING);
        }

        // Check memory pressure
        if (checkMemoryPressure()) {
            notifyLifecycleEvent(LifecycleEvent::MEMORY_PRESSURE);
        }

        // Notify post-update
        notifyLifecycleEvent(LifecycleEvent::POST_UPDATE);

        return updateSuccess;
    }

    void InputLifecycle::pause() {
        const auto currentState = state_.load(std::memory_order_acquire);

        if (currentState != LifecycleState::RUNNING) {
            return;
        }

        if (transitionTo(LifecycleState::PAUSED)) {
            // Pause input service
            if (inputService_) {
                inputService_->setPaused(true);
            }
        }
    }

    void InputLifecycle::resume() {
        const auto currentState = state_.load(std::memory_order_acquire);

        if (currentState != LifecycleState::PAUSED) {
            return;
        }

        if (transitionTo(LifecycleState::RUNNING)) {
            // Resume input service
            if (inputService_) {
                inputService_->setPaused(false);
            }

            // Reset timing to avoid large delta
            lastUpdateTime_ = std::chrono::steady_clock::now();
            accumulatedTime_ = 0.0f;
        }
    }

    bool InputLifecycle::shutdown() {
        const auto currentState = state_.load(std::memory_order_acquire);

        // Can shutdown from most states
        if (currentState == LifecycleState::UNINITIALIZED ||
            currentState == LifecycleState::SHUTDOWN) {
            return true;
        }

        // Transition to shutting down
        if (!transitionTo(LifecycleState::SHUTTING_DOWN)) {
            return false;
        }

        // Notify pre-shutdown
        notifyLifecycleEvent(LifecycleEvent::PRE_SHUTDOWN);

        bool shutdownSuccess = true;

        // Handle graceful shutdown with timeout
        if (lifecycleConfig_.gracefulShutdown && lifecycleConfig_.shutdownTimeout > 0.0f) {
            const auto startTime = std::chrono::steady_clock::now();
            const auto timeout = std::chrono::milliseconds(
                static_cast<long>(lifecycleConfig_.shutdownTimeout * 1000)
            );

            // Wait for pending operations
            while (inputService_ && !inputService_->isPaused()) {
                inputService_->setPaused(true);

                const auto elapsed = std::chrono::steady_clock::now() - startTime;
                if (elapsed > timeout) {
                    handleError("Shutdown timeout exceeded");
                    shutdownSuccess = false;
                    break;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }

        // Save state if configured
        if (lifecycleConfig_.saveStateOnShutdown) {
            // TODO: Implement state saving
            // This would serialize current configuration and state
        }

        // Shutdown subsystems
        shutdownSubsystems();

        // Clear callbacks
        clearCallbacks();

        // Transition to shutdown
        if (!transitionTo(LifecycleState::SHUTDOWN)) {
            shutdownSuccess = false;
        }

        // Notify post-shutdown
        notifyLifecycleEvent(LifecycleEvent::POST_SHUTDOWN);

        return shutdownSuccess;
    }

    bool InputLifecycle::reset() {
        // Shutdown first
        if (!shutdown()) {
            return false;
        }

        // Transition back to uninitialized
        if (!transitionTo(LifecycleState::UNINITIALIZED)) {
            return false;
        }

        // Clear error state
        clearError();

        // Reset performance stats
        perfStats_.reset();

        // Can now reinitialize
        return initialize(inputConfig_, lifecycleConfig_);
    }

    bool InputLifecycle::transitionTo(const LifecycleState newState) {
        const auto currentState = state_.load(std::memory_order_acquire);

        // Validate transition
        bool validTransition = false;

        switch (currentState) {
        case LifecycleState::UNINITIALIZED:
            validTransition = (newState == LifecycleState::INITIALIZING);
            break;

        case LifecycleState::INITIALIZING:
            validTransition = (newState == LifecycleState::INITIALIZED ||
                newState == LifecycleState::ERROR_STATE);
            break;

        case LifecycleState::INITIALIZED:
            validTransition = (newState == LifecycleState::RUNNING ||
                newState == LifecycleState::SHUTTING_DOWN);
            break;

        case LifecycleState::RUNNING:
            validTransition = (newState == LifecycleState::PAUSED ||
                newState == LifecycleState::SHUTTING_DOWN ||
                newState == LifecycleState::ERROR_STATE);
            break;

        case LifecycleState::PAUSED:
            validTransition = (newState == LifecycleState::RUNNING ||
                newState == LifecycleState::SHUTTING_DOWN);
            break;

        case LifecycleState::SHUTTING_DOWN:
            validTransition = (newState == LifecycleState::SHUTDOWN);
            break;

        case LifecycleState::SHUTDOWN:
            validTransition = (newState == LifecycleState::UNINITIALIZED);
            break;

        case LifecycleState::ERROR_STATE:
            validTransition = (newState == LifecycleState::SHUTTING_DOWN ||
                newState == LifecycleState::UNINITIALIZED);
            break;
        }

        if (!validTransition) {
            std::stringstream ss;
            ss << "Invalid state transition from "
                << static_cast<int>(currentState)
                << " to "
                << static_cast<int>(newState);
            handleError(ss.str());
            return false;
        }

        // Perform atomic transition
        bool expected = state_.compare_exchange_strong(
            const_cast<LifecycleState&>(currentState),
            newState,
            std::memory_order_acq_rel
        );

        if (expected) {
            notifyLifecycleEvent(LifecycleEvent::STATE_CHANGED);
        }

        return expected;
    }

    // ============================================================================
    // Configuration
    // ============================================================================

    bool InputLifecycle::reloadConfiguration(const InputConfig& config) {
        const auto currentState = state_.load(std::memory_order_acquire);

        // Can only reload when initialized or running
        if (currentState != LifecycleState::INITIALIZED &&
            currentState != LifecycleState::RUNNING &&
            currentState != LifecycleState::PAUSED) {
            handleError("Cannot reload configuration in current state");
            return false;
        }

        // Store old config for rollback
        const InputConfig oldConfig = inputConfig_;
        inputConfig_ = config;

        // Validate new configuration
        if (lifecycleConfig_.validateConfiguration && !validateConfiguration()) {
            inputConfig_ = oldConfig;
            handleError("New configuration validation failed");
            return false;
        }

        // Apply to input service
        if (inputService_) {
            InputServiceConfig serviceConfig;
            serviceConfig.eventPoolSize = config.memory.eventPoolSize;
            serviceConfig.snapshotPoolSize = config.memory.snapshotPoolSize;
            serviceConfig.historyFrameCount = config.memory.historyBufferFrames;
            serviceConfig.enableKeyboard = config.keyboard.enabled;
            serviceConfig.enableMouse = config.mouse.enabled;
            serviceConfig.enableGamepad = config.gamepad.enabled;
            serviceConfig.enableTouch = config.touch.enabled;
            serviceConfig.analogDeadzone = config.gamepad.stickDeadzone;
            serviceConfig.triggerThreshold = config.gamepad.triggerThreshold;
            serviceConfig.mouseSensitivity = config.mouse.sensitivity;
            serviceConfig.enableInputBuffering = config.processing.enableInputBuffer;
            serviceConfig.inputBufferFrames = config.processing.inputBufferFrames;
            serviceConfig.enableDebugLogging = config.debug.enableLogging;
            serviceConfig.recordInputForReplay = config.debug.recordForReplay;

            // Note: InputService would need a reconfigure method for this to work properly
            // For now, we store the config for next initialization
        }

        return true;
    }

    // ============================================================================
    // Callbacks
    // ============================================================================

    void InputLifecycle::registerLifecycleCallback(LifecycleCallback callback) {
        if (callback) {
            lifecycleCallbacks_.push_back(std::move(callback));
        }
    }

    void InputLifecycle::registerErrorCallback(ErrorCallback callback) {
        if (callback) {
            errorCallbacks_.push_back(std::move(callback));
        }
    }

    void InputLifecycle::clearCallbacks() {
        lifecycleCallbacks_.clear();
        errorCallbacks_.clear();
    }

    // ============================================================================
    // Internal Methods Implementation
    // ============================================================================

    bool InputLifecycle::initializeSubsystems() {
        try {
            // Create memory pools first
            if (!memoryManager_) {
                // In production, would get global memory manager
                // For now, we proceed without it
                ownsMemoryManager_ = false;
            }

            memoryPools_ = std::make_unique<InputMemoryPools>();

            InputMemoryPoolConfig poolConfig;
            poolConfig.eventPoolSize = inputConfig_.memory.eventPoolSize;
            poolConfig.snapshotPoolSize = inputConfig_.memory.snapshotPoolSize;
            poolConfig.preallocateAll = lifecycleConfig_.preallocateMemory;

            if (!memoryPools_->initialize(poolConfig)) {
                handleError("Failed to initialize memory pools");
                return false;
            }

            // Create device service
            auto deviceService = std::make_unique<DeviceService>(memoryManager_);
            if (!deviceService->initialize()) {
                handleError("Failed to initialize device service");
                return false;
            }
            deviceService_ = deviceService.get();

            // Create SDL backend
            auto sdlBackend = std::make_unique<SDLInputBackend>(
                deviceService_,
                memoryPools_.get()
            );

            SDLBackendConfig backendConfig;
            backendConfig.enableControllerEvents = inputConfig_.gamepad.enabled;
            backendConfig.enableTouchEvents = inputConfig_.touch.enabled;
            backendConfig.maxEventsPerPoll = 64;

            if (!sdlBackend->initialize(backendConfig)) {
                handleError("Failed to initialize SDL backend");
                return false;
            }
            sdlBackend_ = sdlBackend.get();

            // Create event processor
            auto eventProcessor = std::make_unique<processing::EventProcessor>(
                deviceService_,
                memoryManager_
            );

            processing::EventProcessorConfig processorConfig;
            processorConfig.maxEventsPerFrame = 256;
            processorConfig.enableFiltering = true;
            processorConfig.enableDeadzone = inputConfig_.gamepad.enableDeadzone;
            processorConfig.enableSensitivity = true;
            processorConfig.useMemoryPools = true;

            if (!eventProcessor->initialize(processorConfig)) {
                handleError("Failed to initialize event processor");
                return false;
            }
            eventProcessor_ = eventProcessor.get();

            // Create action map
            auto actionMap = std::make_unique<ActionMap>();
            if (!actionMap->initialize()) {
                handleError("Failed to initialize action map");
                return false;
            }
            actionMap_ = actionMap.get();

            // Create context stack
            auto contextStack = std::make_unique<mapping::ContextStack>(memoryManager_);

            mapping::ContextStackConfig contextConfig;
            contextConfig.maxStackDepth = 16;
            contextConfig.defaultContext = "Default";

            if (!contextStack->initialize(contextConfig)) {
                handleError("Failed to initialize context stack");
                return false;
            }
            contextStack_ = contextStack.get();

            // Create snapshot manager
            auto snapshotManager = std::make_unique<processing::SnapshotManager>(
                deviceService_,
                actionMap_,
                memoryManager_
            );

            processing::SnapshotManagerConfig snapshotConfig;
            snapshotConfig.historySize = inputConfig_.memory.historyBufferFrames;
            snapshotConfig.enableHistory = inputConfig_.debug.recordForReplay;
            snapshotConfig.useDoubleBuffering = true;
            snapshotConfig.useMemoryPool = true;

            if (!snapshotManager->initialize(snapshotConfig)) {
                handleError("Failed to initialize snapshot manager");
                return false;
            }
            snapshotManager_ = snapshotManager.get();

            // Create input service and transfer ownership of subsystems
            auto inputService = std::make_unique<InputService>(memoryManager_);

            // Note: InputService owns these now, we just keep pointers for direct access
            inputService_ = inputService.get();

            // Initialize input service with the config
            InputServiceConfig serviceConfig;
            serviceConfig.eventPoolSize = inputConfig_.memory.eventPoolSize;
            serviceConfig.snapshotPoolSize = inputConfig_.memory.snapshotPoolSize;
            serviceConfig.historyFrameCount = inputConfig_.memory.historyBufferFrames;
            serviceConfig.enableKeyboard = inputConfig_.keyboard.enabled;
            serviceConfig.enableMouse = inputConfig_.mouse.enabled;
            serviceConfig.enableGamepad = inputConfig_.gamepad.enabled;
            serviceConfig.enableTouch = inputConfig_.touch.enabled;
            serviceConfig.analogDeadzone = inputConfig_.gamepad.stickDeadzone;
            serviceConfig.triggerThreshold = inputConfig_.gamepad.triggerThreshold;
            serviceConfig.mouseSensitivity = inputConfig_.mouse.sensitivity;
            serviceConfig.enableInputBuffering = inputConfig_.processing.enableInputBuffer;
            serviceConfig.inputBufferFrames = inputConfig_.processing.inputBufferFrames;
            serviceConfig.enableDebugLogging = inputConfig_.debug.enableLogging;
            serviceConfig.recordInputForReplay = inputConfig_.debug.recordForReplay;

            if (!inputService_->initialize(serviceConfig)) {
                handleError("Failed to initialize input service");
                return false;
            }

            // Auto-initialize devices if configured
            if (lifecycleConfig_.autoInitializeDevices) {
                deviceService_->scanForDevices();
            }

            return true;
        }
        catch (const std::exception& e) {
            handleError(std::string("Exception during initialization: ") + e.what(), true);
            return false;
        }
        catch (...) {
            handleError("Unknown exception during initialization", true);
            return false;
        }
    }

    void InputLifecycle::shutdownSubsystems() {
        // Shutdown in reverse order of initialization

        if (inputService_) {
            inputService_->shutdown();
            inputService_ = nullptr;
        }

        snapshotManager_ = nullptr;
        contextStack_ = nullptr;
        actionMap_ = nullptr;
        eventProcessor_ = nullptr;
        sdlBackend_ = nullptr;
        deviceService_ = nullptr;

        if (memoryPools_) {
            memoryPools_->shutdown();
            memoryPools_.reset();
        }
    }

    bool InputLifecycle::validateConfiguration() const {
        // Validate memory settings
        if (inputConfig_.memory.eventPoolSize == 0 ||
            inputConfig_.memory.snapshotPoolSize == 0) {
            return false;
        }

        // Validate at least one input device is enabled
        if (!inputConfig_.keyboard.enabled &&
            !inputConfig_.mouse.enabled &&
            !inputConfig_.gamepad.enabled &&
            !inputConfig_.touch.enabled) {
            return false;
        }

        // Validate deadzone and threshold ranges
        if (inputConfig_.gamepad.stickDeadzone < 0.0f ||
            inputConfig_.gamepad.stickDeadzone > 1.0f ||
            inputConfig_.gamepad.triggerThreshold < 0.0f ||
            inputConfig_.gamepad.triggerThreshold > 1.0f) {
            return false;
        }

        // Validate lifecycle config
        if (lifecycleConfig_.targetUpdateRate <= 0.0f ||
            lifecycleConfig_.maxUpdateDelta <= 0.0f) {
            return false;
        }

        if (lifecycleConfig_.fixedTimestep &&
            lifecycleConfig_.fixedDeltaTime <= 0.0f) {
            return false;
        }

        return true;
    }

    void InputLifecycle::updateTiming(const float deltaTime) {
        accumulatedTime_ += deltaTime;

        // Update last update time
        lastUpdateTime_ = std::chrono::steady_clock::now();
    }

    bool InputLifecycle::handleError(const std::string& error, const bool critical) {
        lastError_ = error;
        errorCount_.fetch_add(1, std::memory_order_relaxed);

        // Notify error callbacks
        notifyError(error);
        notifyLifecycleEvent(LifecycleEvent::ERROR_OCCURRED);

        // Handle based on criticality
        if (critical) {
            // Transition to error state
            transitionTo(LifecycleState::ERROR_STATE);

            // Attempt recovery if configured
            if (lifecycleConfig_.continueOnError &&
                errorCount_ < lifecycleConfig_.maxErrorRetries) {
                return recoverFromError();
            }

            return false;
        }

        // Non-critical errors can continue if configured
        return lifecycleConfig_.continueOnError;
    }

    void InputLifecycle::notifyLifecycleEvent(const LifecycleEvent event) {
        for (const auto& callback : lifecycleCallbacks_) {
            if (callback) {
                try {
                    callback(event);
                }
                catch (...) {
                    // Silently ignore callback exceptions
                }
            }
        }
    }

    void InputLifecycle::notifyError(const std::string& error) {
        for (const auto& callback : errorCallbacks_) {
            if (callback) {
                try {
                    callback(error);
                }
                catch (...) {
                    // Silently ignore callback exceptions
                }
            }
        }
    }

    std::uint32_t InputLifecycle::performFixedUpdate(const float deltaTime) {
        accumulatedTime_ += deltaTime;
        std::uint32_t steps = 0;

        // Perform fixed timestep updates
        while (accumulatedTime_ >= lifecycleConfig_.fixedDeltaTime) {
            if (inputService_) {
                inputService_->tick(lifecycleConfig_.fixedDeltaTime);
            }

            accumulatedTime_ -= lifecycleConfig_.fixedDeltaTime;
            steps++;

            // Prevent spiral of death
            if (steps >= 10) {
                // Drop accumulated time to catch up
                accumulatedTime_ = 0.0f;
                perfStats_.droppedFrames++;
                break;
            }
        }

        return steps;
    }

    void InputLifecycle::updatePerformanceStats(const float updateTime) {
        // Update average using exponential moving average
        constexpr float alpha = 0.1f;
        perfStats_.averageUpdateTime = (1.0f - alpha) * perfStats_.averageUpdateTime +
            alpha * updateTime;

        // Update peak
        if (updateTime > perfStats_.peakUpdateTime) {
            perfStats_.peakUpdateTime = updateTime;
        }

        // Check for slow frames
        if (updateTime > lifecycleConfig_.performanceWarningThreshold * 1000.0f) {
            perfStats_.slowFrames++;
        }

        // Update frame time
        const auto now = std::chrono::steady_clock::now();
        const float frameTime = std::chrono::duration<float, std::milli>(
            now - lastUpdateTime_
        ).count();

        perfStats_.averageFrameTime = (1.0f - alpha) * perfStats_.averageFrameTime +
            alpha * frameTime;

        perfStats_.totalFrames++;
    }

    bool InputLifecycle::checkMemoryPressure() const {
        if (!memoryPools_) {
            return false;
        }

        // Check if pools are running low
        const std::size_t availableEvents = memoryPools_->getAvailableEvents();
        const std::size_t availableSnapshots = memoryPools_->getAvailableSnapshots();

        // Consider memory pressure if less than 10% available
        const std::size_t eventThreshold = inputConfig_.memory.eventPoolSize / 10;
        const std::size_t snapshotThreshold = inputConfig_.memory.snapshotPoolSize / 10;

        return (availableEvents < eventThreshold ||
            availableSnapshots < snapshotThreshold);
    }

    bool InputLifecycle::recoverFromError() {
        // Attempt to reset the problematic subsystem
        const auto currentState = state_.load(std::memory_order_acquire);

        if (currentState == LifecycleState::ERROR_STATE) {
            // Try to reinitialize
            if (reset()) {
                clearError();
                return true;
            }
        }

        return false;
    }
} // namespace engine::input
