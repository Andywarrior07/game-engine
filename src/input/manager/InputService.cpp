/**
 * @file InputService.cpp
 * @brief Main input service implementation
 * @author Andrés Guerrero
 * @date 13-09-2025
 */

#include "InputService.h"

#include "../core/InputMemoryPools.h"
#include "../mapping/ActionMap.h"
#include "../mapping/ContextStack.h"
#include "../devices/mouse/MouseLock.h"
#include "../devices/keyboard/KeyboardDevice.h"
#include "../devices/mouse/MouseDevice.h"
#include "../devices/gamepad/GamepadDevice.h"

#include <algorithm>

namespace engine::input {
    InputService::InputService(memory::MemoryManager* memoryManager) noexcept
        : memoryManager_(memoryManager) {
        lastFrameTime_ = std::chrono::steady_clock::now();
    }

    InputService::~InputService() {
        if (initialized_.load(std::memory_order_acquire)) {
            shutdown();
        }
    }

    bool InputService::initialize(const InputServiceConfig& config) {
        if (initialized_.load(std::memory_order_acquire)) {
            return true;
        }

        config_ = config;

        // Initialize subsystems
        if (!initializeSubsystems()) {
            shutdownSubsystems();
            return false;
        }

        // Start with default context
        if (contextStack_) {
            contextStack_->pushContext("Default");
        }

        initialized_.store(true, std::memory_order_release);
        return true;
    }

    void InputService::shutdown() {
        if (!initialized_.load(std::memory_order_acquire)) {
            return;
        }

        // Stop any recording/replay
        if (snapshotManager_) {
            snapshotManager_->stopRecording();
            snapshotManager_->stopReplay();
        }

        // Shutdown subsystems in reverse order
        shutdownSubsystems();

        initialized_.store(false, std::memory_order_release);
    }

    void InputService::tick(const float deltaTime) {
        if (!initialized_.load(std::memory_order_acquire) ||
            isPaused_.load(std::memory_order_acquire)) {
            return;
        }

        const auto frameStart = std::chrono::steady_clock::now();

        // Process the frame
        processFrame(deltaTime);

        // Increment frame counter
        frameNumber_.fetch_add(1, std::memory_order_relaxed);

        // Update statistics
        const auto frameEnd = std::chrono::steady_clock::now();
        const float frameTime = std::chrono::duration<float, std::milli>(frameEnd - frameStart).count();
        updateStatistics(frameTime);

        lastFrameTime_ = frameEnd;
    }

    const InputSnapshot* InputService::getSnapshot() const {
        if (snapshotManager_) {
            return snapshotManager_->getCurrentSnapshot();
        }
        return nullptr;
    }

    const InputSnapshot* InputService::getPreviousSnapshot() const {
        if (snapshotManager_) {
            return snapshotManager_->getPreviousSnapshot();
        }
        return nullptr;
    }

    void InputService::registerAction(const ActionID id, const std::string& name, const ActionType type) const {
        if (actionMap_) {
            actionMap_->registerAction(id, name, type);
        }
    }

    void InputService::bindInput(const ActionID actionId, const KeyCode key, const std::string& context) const {
        if (actionMap_) {
            InputBinding binding;
            binding.actionId = actionId;
            binding.inputType = InputBinding::Type::KEYBOARD;
            binding.keyCode = key;
            binding.context = context;
            actionMap_->addBinding(binding);
        }
    }

    void InputService::bindInput(const ActionID actionId, const MouseButton button, const std::string& context) const {
        if (actionMap_) {
            InputBinding binding;
            binding.actionId = actionId;
            binding.inputType = InputBinding::Type::MOUSE_BUTTON;
            binding.mouseButton = button;
            binding.context = context;
            actionMap_->addBinding(binding);
        }
    }

    void InputService::bindInput(const ActionID actionId, const GamepadButton button,
                                 const std::string& context) const {
        if (actionMap_) {
            InputBinding binding;
            binding.actionId = actionId;
            binding.inputType = InputBinding::Type::GAMEPAD_BUTTON;
            binding.gamepadButton = button;
            binding.context = context;
            actionMap_->addBinding(binding);
        }
    }

    void InputService::bindAxis(const ActionID actionId, const GamepadAxis axis, const std::string& context) const {
        if (actionMap_) {
            InputBinding binding;
            binding.actionId = actionId;
            binding.inputType = InputBinding::Type::GAMEPAD_AXIS;
            binding.gamepadAxis = axis;
            binding.context = context;
            actionMap_->addBinding(binding);
        }
    }

    void InputService::registerActionCallback(const ActionID actionId, ActionCallback callback) const {
        if (actionMap_) {
            actionMap_->registerCallback(actionId, std::move(callback));
        }
    }

    void InputService::clearActionCallbacks(const ActionID actionId) const {
        if (actionMap_) {
            actionMap_->clearCallbacks(actionId);
        }
    }

    bool InputService::pushContext(const std::string& name, ContextPriority priority) const {
        if (contextStack_) {
            // Create and register context if it doesn't exist
            if (!contextStack_->hasContext(name)) {
                auto context = std::make_unique<InputContext>(name, priority);
                contextStack_->registerContext(std::move(context));
            }

            return contextStack_->pushContext(name);
        }

        return false;
    }

    bool InputService::popContext() const {
        if (contextStack_) {
            return contextStack_->popContext();
        }

        return false;
    }

    std::string InputService::getCurrentContext() const {
        if (contextStack_) {
            return contextStack_->getActiveContextName();
        }

        return "";
    }

    void InputService::registerContextChangeCallback(ContextChangeCallback callback) const {
        if (contextStack_) {
            contextStack_->registerTransitionCallback(
                [callback = std::move(callback)](const std::string& context, mapping::ContextTransition transition) {
                    if (transition == mapping::ContextTransition::ACTIVATED) {
                        callback("", context);
                    }
                    else if (transition == mapping::ContextTransition::DEACTIVATED) {
                        callback(context, "");
                    }
                }
            );
        }
    }

    bool InputService::setGamepadRumble(const PlayerID player, const float leftMotor,
                                        const float rightMotor, const float duration) const {
        if (!deviceService_) {
            return false;
        }

        // Find gamepad device for player
        auto* device = deviceService_->getPlayerDevice(player);
        if (!device || device->getDeviceType() != DeviceType::GAMEPAD) {
            return false;
        }

        return device->setRumble(leftMotor, rightMotor, duration);
    }

    void InputService::setTextInputMode(const bool enabled) const {
        if (backend_) {
            if (enabled) {
                backend_->startTextInput();
            }
            else {
                backend_->stopTextInput();
            }
        }
    }

    bool InputService::setRelativeMouseMode(const bool enabled) const {
        if (backend_) {
            return backend_->setRelativeMouseMode(enabled);
        }
        return false;
    }

    void InputService::registerRawEventCallback(RawEventCallback callback) const {
        if (eventProcessor_) {
            eventProcessor_->registerEventCallback(
                [callback = std::move(callback)](const InputEvent& event) {
                    // TODO: Revisar esto
                    callback(event);
                }
            );
        }
    }

    void InputService::injectEvent(const InputEvent& event) const {
        if (eventProcessor_) {
            InputEvent syntheticEvent = event;
            syntheticEvent.synthetic = true;
            syntheticEvent.timestamp = InputTimestamp::clock::now();
            syntheticEvent.frameNumber = frameNumber_.load(std::memory_order_acquire);

            // TODO: Revisar esto
            eventProcessor_->queueEvent(syntheticEvent);
        }
    }

    void InputService::startRecording() const {
        if (snapshotManager_) {
            snapshotManager_->startRecording();
        }
    }

    void InputService::stopRecording() const {
        if (snapshotManager_) {
            snapshotManager_->stopRecording();
        }
    }

    bool InputService::isRecording() const noexcept {
        if (snapshotManager_) {
            return snapshotManager_->isRecording();
        }
        return false;
    }

    std::vector<std::uint8_t> InputService::exportRecording() const {
        if (snapshotManager_) {
            return snapshotManager_->exportHistory();
        }
        return {};
    }

    bool InputService::startReplay(const std::vector<std::uint8_t>& replayData) const {
        if (snapshotManager_) {
            return snapshotManager_->startReplay(replayData);
        }
        return false;
    }

    void InputService::stopReplay() const {
        if (snapshotManager_) {
            snapshotManager_->stopReplay();
        }
    }

    bool InputService::isReplaying() const noexcept {
        if (snapshotManager_) {
            return snapshotManager_->isReplaying();
        }
        return false;
    }

    bool InputService::initializeSubsystems() {
        // Initialize memory pools
        memoryPools_ = std::make_unique<InputMemoryPools>();

        InputMemoryPoolConfig poolConfig;
        poolConfig.eventPoolSize = config_.eventPoolSize;
        poolConfig.snapshotPoolSize = config_.snapshotPoolSize;

        if (!memoryPools_->initialize(poolConfig)) {
            return false;
        }

        // Initialize device service
        deviceService_ = std::make_unique<DeviceService>(memoryManager_);
        if (!deviceService_->initialize()) {
            return false;
        }

        // Initialize SDL backend
        backend_ = std::make_unique<SDLInputBackend>(deviceService_.get(), memoryPools_.get());

        SDLBackendConfig backendConfig;
        backendConfig.enableControllerEvents = config_.enableGamepad;
        backendConfig.enableTouchEvents = config_.enableTouch;
        backendConfig.enableTextInput = false; // Start with text input disabled

        if (!backend_->initialize(backendConfig)) {
            return false;
        }

        // Initialize mouse lock
        mouseLock_ = std::make_unique<MouseLock>();
        mouseLock_->initialize();
        backend_->setMouseLock(mouseLock_.get());

        // Initialize event processor
        eventProcessor_ = std::make_unique<processing::EventProcessor>(
            deviceService_.get(), memoryManager_
        );

        processing::EventProcessorConfig processorConfig;
        processorConfig.maxEventsPerFrame = 256;
        processorConfig.enableFiltering = true;
        processorConfig.enableDeadzone = true;
        processorConfig.enableSensitivity = true;

        if (!eventProcessor_->initialize(processorConfig)) {
            return false;
        }

        // Initialize action map
        actionMap_ = std::make_unique<ActionMap>();

        actionMap_->initialize();
        // TODO: Revisar esto, lo comenté ya que siempre devuelve true
        // if (!actionMap_->initialize()) {
        //     return false;
        // }

        // Initialize context stack
        contextStack_ = std::make_unique<mapping::ContextStack>(memoryManager_);

        mapping::ContextStackConfig contextConfig;
        contextConfig.maxStackDepth = 16;
        contextConfig.defaultContext = "Default";

        if (!contextStack_->initialize(contextConfig)) {
            return false;
        }

        // Initialize snapshot manager
        snapshotManager_ = std::make_unique<processing::SnapshotManager>(
            deviceService_.get(), actionMap_.get(), memoryManager_
        );

        processing::SnapshotManagerConfig snapshotConfig;
        snapshotConfig.historySize = config_.historyFrameCount;
        snapshotConfig.enableHistory = config_.recordInputForReplay;

        if (!snapshotManager_->initialize(snapshotConfig)) {
            return false;
        }

        // Set up event flow: Backend -> EventProcessor -> SnapshotManager
        backend_->setEventCallback(
            [this](const InputEvent& event) {
                if (eventProcessor_) {
                    // TODO: Revisar esto
                    eventProcessor_->queueEvent(event);
                }
            }
        );

        // Create default input devices
        if (config_.enableKeyboard) {
            DeviceInitParams keyboardParams;
            keyboardParams.type = DeviceType::KEYBOARD;
            keyboardParams.name = "Keyboard";

            auto keyboard = std::make_unique<KeyboardDevice>();
            if (keyboard->initialize(keyboardParams)) {
                deviceService_->registerDevice(std::move(keyboard));
            }
        }

        if (config_.enableMouse) {
            DeviceInitParams mouseParams;
            mouseParams.type = DeviceType::MOUSE;
            mouseParams.name = "Mouse";

            if (auto mouse = std::make_unique<MouseDevice>(); mouse->initialize(mouseParams)) {
                deviceService_->registerDevice(std::move(mouse));
            }
        }

        return true;
    }

    void InputService::shutdownSubsystems() {
        // Shutdown in reverse order
        if (snapshotManager_) {
            snapshotManager_->shutdown();
            snapshotManager_.reset();
        }

        if (contextStack_) {
            contextStack_->shutdown();
            contextStack_.reset();
        }

        if (actionMap_) {
            actionMap_->shutdown();
            actionMap_.reset();
        }

        if (eventProcessor_) {
            eventProcessor_->shutdown();
            eventProcessor_.reset();
        }

        if (mouseLock_) {
            mouseLock_->shutdown();
            mouseLock_.reset();
        }

        if (backend_) {
            backend_->shutdown();
            backend_.reset();
        }

        if (deviceService_) {
            deviceService_->shutdown();
            deviceService_.reset();
        }

        if (memoryPools_) {
            memoryPools_->shutdown();
            memoryPools_.reset();
        }
    }

    void InputService::processFrame(const float deltaTime) const {
        // Poll SDL events
        if (backend_) {
            backend_->pollEvents();
        }

        // Process events through pipeline
        if (eventProcessor_) {
            eventProcessor_->processEvents(deltaTime);
        }

        // Begin new snapshot frame
        if (snapshotManager_) {
            snapshotManager_->beginFrame(frameNumber_.load(std::memory_order_acquire), deltaTime);

            // TODO: Revisar esto
            // Update device states in snapshot
            snapshotManager_->updateDeviceStates();

            // Process events into snapshot
            std::vector<InputEvent> events;
            events.reserve(256);

            if (backend_) {
                backend_->getPendingEvents(events, 256);
            }

            if (!events.empty()) {
                // TODO: Revisar esto
                snapshotManager_->processEvents(events);
            }

            // Update action states from current context
            if (contextStack_ && actionMap_) {
                auto* activeContext = contextStack_->getActiveContext();
                if (activeContext) {
                    // Set context in action map
                    actionMap_->setCurrentContext(activeContext->getName());

                    // TODO: Revisar esto
                    // Update action states in snapshot
                    snapshotManager_->updateActionStates();
                }
            }

            // Finalize snapshot
            snapshotManager_->endFrame();
        }

        // Update devices
        if (deviceService_) {
            deviceService_->update(deltaTime);
        }
    }

    void InputService::updateStatistics(const float frameTime) const {
        stats_.totalFrames++;

        // Update average frame time (exponential moving average)
        constexpr float alpha = 0.1f;
        stats_.averageFrameTime = (1.0f - alpha) * stats_.averageFrameTime + alpha * frameTime;

        // Update peak frame time
        if (frameTime > stats_.peakFrameTime) {
            stats_.peakFrameTime = frameTime;
        }
    }
} // namespace engine::input
