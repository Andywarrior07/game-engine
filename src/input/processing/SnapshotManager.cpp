/**
 * @file SnapshotManager.cpp
 * @brief Snapshot management implementation
 * @author Andrés Guerrero
 * @date 13-09-2025
 */

#include "SnapshotManager.h"

#include "../devices/keyboard/KeyboardDevice.h"
#include "../devices/mouse/MouseDevice.h"
#include "../devices/gamepad/GamepadDevice.h"

#include <algorithm>

namespace engine::input::processing {
    SnapshotManager::SnapshotManager(DeviceService* deviceService,
                                     ActionMap* actionMap,
                                     MemoryAllocator* memoryManager) noexcept
        : deviceService_(deviceService)
          , actionMap_(actionMap)
          , memoryManager_(memoryManager)
          , ownsMemoryManager_(false) {
        // Reserve space for event buffer
        eventBuffer_.reserve(256);

        // Initialize snapshot pointers
        currentSnapshot_ = nullptr;
        previousSnapshot_ = nullptr;
    }

    SnapshotManager::~SnapshotManager() {
        shutdown();
    }

    bool SnapshotManager::initialize(const SnapshotManagerConfig& config) {
        if (initialized_) {
            return false;
        }

        config_ = config;

        // Create snapshot buffers
        if (!createBuffers()) {
            return false;
        }

        // Create history if enabled
        if (config_.enableHistory) {
            history_ = std::make_unique<InputHistory>(config_.historySize);
        }

        // Create memory pool for snapshots if configured
        if (config_.useMemoryPool && memoryManager_) {
            snapshotPool_ = std::make_unique<engine::memory::PoolAllocator>(
                sizeof(InputSnapshot),
                config_.snapshotPoolSize,
                alignof(InputSnapshot),
                "SnapshotPool"
            );
        }

        // Reserve recording buffer
        if (config_.enableHistory) {
            recordingBuffer_.reserve(1024 * 1024); // 1MB initial
        }

        initialized_ = true;
        return true;
    }

    void SnapshotManager::shutdown() {
        if (!initialized_) {
            return;
        }

        // Stop any active recording/replay
        stopRecording();
        stopReplay();

        // Clear callbacks
        clearCallbacks();

        // Clear history
        if (history_) {
            history_->clear();
        }

        // Release snapshots
        for (auto& snapshot : snapshots_) {
            snapshot.reset();
        }

        // Release memory pool
        snapshotPool_.reset();

        currentSnapshot_ = nullptr;
        previousSnapshot_ = nullptr;

        initialized_ = false;
    }

    void SnapshotManager::setConfig(const SnapshotManagerConfig& config) {
        config_ = config;

        // Resize history if needed
        if (history_ && config_.historySize != history_->getFrameCount()) {
            history_ = std::make_unique<InputHistory>(config_.historySize);
        }
    }

    bool SnapshotManager::beginFrame(const std::uint64_t frameNumber, const float deltaTime) {
        if (!initialized_) {
            return false;
        }

        std::lock_guard lock(snapshotMutex_);

        // Store previous snapshot
        previousSnapshot_ = currentSnapshot_;

        // Get next buffer for double buffering
        if (config_.useDoubleBuffering) {
            currentBufferIndex_ = (currentBufferIndex_ + 1) % BUFFER_COUNT;
            currentSnapshot_ = snapshots_[currentBufferIndex_].get();
        }
        else {
            currentSnapshot_ = snapshots_[0].get();
        }

        if (!currentSnapshot_) {
            return false;
        }

        // Initialize new frame
        currentSnapshot_->frameNumber = frameNumber;
        currentSnapshot_->deltaTime = deltaTime;
        currentSnapshot_->timestamp = InputTimestamp::clock::now();

        // Clear frame-specific state
        currentSnapshot_->clearFrameState();

        // Copy persistent state from previous frame if available
        if (previousSnapshot_) {
            // Copy button/key states (but not just pressed/released)
            currentSnapshot_->keyboard.pressed = previousSnapshot_->keyboard.pressed;
            currentSnapshot_->mouse.buttons = previousSnapshot_->mouse.buttons;

            // Copy gamepad states
            for (std::size_t i = 0; i < MAX_PLAYERS; ++i) {
                if (previousSnapshot_->gamepads[i].isConnected) {
                    currentSnapshot_->gamepads[i].buttons = previousSnapshot_->gamepads[i].buttons;
                    currentSnapshot_->gamepads[i].isConnected = previousSnapshot_->gamepads[i].isConnected;
                    currentSnapshot_->gamepads[i].deviceId = previousSnapshot_->gamepads[i].deviceId;
                }
            }

            // Copy action states
            currentSnapshot_->actions = previousSnapshot_->actions;
        }

        return true;
    }

    std::size_t SnapshotManager::updateDeviceStates() const {
        if (!initialized_ || !currentSnapshot_ || !deviceService_) {
            return 0;
        }

        const auto updateStart = std::chrono::steady_clock::now();
        std::size_t devicesUpdated = 0;

        // Update keyboard state
        updateKeyboardState(*currentSnapshot_);
        devicesUpdated++;

        // Update mouse state
        updateMouseState(*currentSnapshot_);
        devicesUpdated++;

        // Update gamepad states
        updateGamepadStates(*currentSnapshot_);
        devicesUpdated += MAX_PLAYERS;

        // Update touch state if applicable
        updateTouchState(*currentSnapshot_);

        // Apply interpolation if enabled and previous snapshot exists
        if (config_.interpolateAnalogInputs && previousSnapshot_) {
            constexpr float alpha = 0.5f; // Smoothing factor
            interpolateAnalogValues(*currentSnapshot_, *previousSnapshot_, alpha);
        }

        // Update statistics
        const auto updateEnd = std::chrono::steady_clock::now();
        const float updateTime = std::chrono::duration<float, std::milli>(updateEnd - updateStart).count();
        updateStatistics(0, updateTime);

        return devicesUpdated;
    }

    std::size_t SnapshotManager::updateActionStates() const {
        if (!initialized_ || !currentSnapshot_ || !actionMap_) {
            return 0;
        }

        std::size_t actionsUpdated = 0;

        // Update action states from action map

        for (auto actions = actionMap_->evaluateActions(*currentSnapshot_); const auto& [actionId, actionState] :
             actions) {
            currentSnapshot_->actions[actionId] = actionState;
            actionsUpdated++;
        }

        return actionsUpdated;
    }

    std::size_t SnapshotManager::processEvents(const std::vector<InputEvent>& events) const {
        if (!initialized_ || !currentSnapshot_) {
            return 0;
        }

        std::size_t processed = 0;
        std::size_t dropped = 0;

        for (const auto& event : events) {
            // Skip consumed events
            if (event.consumed) {
                continue;
            }

            // Check event limit
            if (processed >= config_.maxEventsPerSnapshot) {
                dropped++;
                continue;
            }

            // Process event into snapshot
            if (processEventIntoSnapshot(*currentSnapshot_, event)) {
                processed++;
            }
        }

        // Update event counts
        currentSnapshot_->eventCount = static_cast<std::uint32_t>(processed);
        currentSnapshot_->droppedEvents = static_cast<std::uint32_t>(dropped);

        // Update statistics
        if (processed > stats_.peakEventsPerSnapshot) {
            stats_.peakEventsPerSnapshot = static_cast<std::uint32_t>(processed);
        }
        stats_.droppedEvents += dropped;

        return processed;
    }

    const InputSnapshot* SnapshotManager::endFrame() {
        if (!initialized_ || !currentSnapshot_) {
            return nullptr;
        }

        std::lock_guard lock(snapshotMutex_);

        // Finalize snapshot
        currentSnapshot_->nextFrame(currentSnapshot_->deltaTime);

        // Record for history if enabled
        if (history_ && config_.enableHistory) {
            history_->recordSnapshot(*currentSnapshot_);
        }

        // Record for replay if recording
        if (isRecording_.load(std::memory_order_acquire)) {
            recordSnapshot(*currentSnapshot_);
        }

        // Update statistics
        stats_.totalSnapshots++;

        // Invoke callbacks
        invokeCallbacks(*currentSnapshot_);

        // Swap read buffer for double buffering
        if (config_.useDoubleBuffering) {
            readBufferIndex_.store(currentBufferIndex_.load(), std::memory_order_release);
        }

        return currentSnapshot_;
    }

    const InputSnapshot* SnapshotManager::getCurrentSnapshot() const noexcept {
        if (!initialized_) {
            return nullptr;
        }

        if (config_.useDoubleBuffering) {
            return snapshots_[readBufferIndex_.load(std::memory_order_acquire)].get();
        }

        return currentSnapshot_;
    }

    const InputSnapshot* SnapshotManager::getPreviousSnapshot() const noexcept {
        return previousSnapshot_;
    }

    void SnapshotManager::swapBuffers() noexcept {
        if (!config_.useDoubleBuffering) {
            return;
        }

        const std::uint8_t current = currentBufferIndex_.load(std::memory_order_acquire);
        const std::uint8_t next = (current + 1) % BUFFER_COUNT;
        currentBufferIndex_.store(next, std::memory_order_release);
        readBufferIndex_.store(current, std::memory_order_release);
    }

    const InputSnapshot* SnapshotManager::getHistoricalSnapshot(const int frameOffset) const {
        if (!history_) {
            return nullptr;
        }

        std::lock_guard lock(historyMutex_);
        return history_->getSnapshot(frameOffset);
    }

    void SnapshotManager::clearHistory() const noexcept {
        if (history_) {
            std::lock_guard lock(historyMutex_);
            history_->clear();
        }
    }

    std::vector<std::uint8_t> SnapshotManager::exportHistory() const {
        if (!history_) {
            return {};
        }

        std::lock_guard lock(historyMutex_);
        return history_->exportHistory();
    }

    bool SnapshotManager::importHistory(const std::vector<std::uint8_t>& data) const {
        if (!history_ || data.empty()) {
            return false;
        }

        std::lock_guard lock(historyMutex_);

        // Clear existing history
        history_->clear();

        // Parse and import snapshots
        std::size_t offset = 0;

        // Read header
        if (data.size() < sizeof(std::uint64_t) * 2) {
            return false;
        }

        const std::uint64_t snapshotCount = *reinterpret_cast<const std::uint64_t*>(data.data());
        offset += sizeof(std::uint64_t) * 2;

        // Import each snapshot
        for (std::uint64_t i = 0; i < snapshotCount; ++i) {
            if (offset >= data.size()) {
                return false;
            }

            // Create sub-vector for snapshot data
            std::vector snapshotData(data.begin() + static_cast<std::vector<std::uint8_t>::difference_type>(offset)
                                     , data.end());
            InputSnapshot snapshot = InputSnapshot::deserialize(snapshotData);

            if (snapshot.frameNumber == 0) {
                return false; // Deserialization failed
            }

            history_->recordSnapshot(snapshot);

            // Move offset forward (would need actual size from deserialize)
            offset += snapshot.serialize().size();
        }

        return true;
    }

    void SnapshotManager::startRecording() {
        if (isRecording_.load(std::memory_order_acquire)) {
            return;
        }

        recordingBuffer_.clear();
        isRecording_.store(true, std::memory_order_release);
    }

    void SnapshotManager::stopRecording() {
        isRecording_.store(false, std::memory_order_release);
    }

    bool SnapshotManager::startReplay(const std::vector<std::uint8_t>& replayData) {
        if (isReplaying_.load(std::memory_order_acquire) || replayData.empty()) {
            return false;
        }

        // Parse replay data into snapshots
        replayBuffer_.clear();
        replayIndex_ = 0;

        // Import snapshots from replay data
        if (!importHistory(replayData)) {
            return false;
        }

        isReplaying_.store(true, std::memory_order_release);
        return true;
    }

    void SnapshotManager::stopReplay() {
        isReplaying_.store(false, std::memory_order_release);
        replayBuffer_.clear();
        replayIndex_ = 0;
    }

    const InputSnapshot* SnapshotManager::getReplaySnapshot() {
        if (!isReplaying_.load(std::memory_order_acquire)) {
            return nullptr;
        }

        if (replayIndex_ >= replayBuffer_.size()) {
            stopReplay();
            return nullptr;
        }

        return &replayBuffer_[replayIndex_++];
    }

    void SnapshotManager::registerSnapshotCallback(SnapshotCallback callback) {
        std::lock_guard lock(callbackMutex_);
        callbacks_.push_back(std::move(callback));
    }

    void SnapshotManager::clearCallbacks() {
        std::lock_guard lock(callbackMutex_);
        callbacks_.clear();
    }

    void SnapshotManager::resetStatistics() const noexcept {
        stats_.reset();
    }

    bool SnapshotManager::validateSnapshot(const InputSnapshot& snapshot) noexcept {
        // Validate frame number
        if (snapshot.frameNumber == 0) {
            return false;
        }

        // Validate delta time
        if (snapshot.deltaTime < 0.0f || snapshot.deltaTime > 1.0f) {
            return false;
        }

        // Validate gamepad states
        // for (const auto& gamepad : snapshot.gamepads) {
        //     if (gamepad.isConnected) {
        //         // Validate analog values are in range
        //         if (std::abs(gamepad.leftStick.x) > 1.0f ||
        //             std::abs(gamepad.leftStick.y) > 1.0f ||
        //             std::abs(gamepad.rightStick.x) > 1.0f ||
        //             std::abs(gamepad.rightStick.y) > 1.0f) {
        //             return false;
        //         }
        //
        //         // Validate trigger values
        //         if (gamepad.leftTrigger < 0.0f || gamepad.leftTrigger > 1.0f ||
        //             gamepad.rightTrigger < 0.0f || gamepad.rightTrigger > 1.0f) {
        //             return false;
        //         }
        //     }
        // }

        // return true;

        return std::ranges::all_of(snapshot.gamepads, [](const auto& gamepad) {
        if (!gamepad.isConnected) return true;

        if (std::abs(gamepad.leftStick.x) > 1.0f ||
            std::abs(gamepad.leftStick.y) > 1.0f ||
            std::abs(gamepad.rightStick.x) > 1.0f ||
            std::abs(gamepad.rightStick.y) > 1.0f) {
            return false;
        }

        if (gamepad.leftTrigger  < 0.0f || gamepad.leftTrigger  > 1.0f ||
            gamepad.rightTrigger < 0.0f || gamepad.rightTrigger > 1.0f) {
            return false;
        }

        return true;
    });
    }

    // ============================================================================
    // Private Methods Implementation
    // ============================================================================

    // TODO: Revisar esto
    bool SnapshotManager::createBuffers() {
        // Create snapshot buffers
        for (auto& snapshot : snapshots_) {
            if (snapshotPool_) {
                snapshot.reset(allocateSnapshot());
            }
            else {
                snapshot = std::make_unique<InputSnapshot>();
            }

            if (!snapshot) {
                return false;
            }
        }

        // Set initial current snapshot
        currentSnapshot_ = snapshots_[0].get();

        return true;
    }

    void SnapshotManager::updateKeyboardState(InputSnapshot& snapshot) const {
        if (!deviceService_) {
            return;
        }

        // Get keyboard device
        const auto keyboards = deviceService_->getDevicesByType(DeviceType::KEYBOARD);

        if (keyboards.empty()) {
            return;
        }

        // Use first keyboard
        if (const auto* keyboard = dynamic_cast<KeyboardDevice*>(keyboards[0])) {
            keyboard->updateSnapshot(snapshot);
        }
    }

    void SnapshotManager::updateMouseState(InputSnapshot& snapshot) const {
        if (!deviceService_) {
            return;
        }

        // Get mouse device
        const auto mice = deviceService_->getDevicesByType(DeviceType::MOUSE);
        if (mice.empty()) {
            return;
        }

        // Use first mouse
        if (const auto* mouse = dynamic_cast<MouseDevice*>(mice[0])) {
            mouse->updateSnapshot(snapshot);
        }
    }

    void SnapshotManager::updateGamepadStates(InputSnapshot& snapshot) const {
        if (!deviceService_) {
            return;
        }

        // Get gamepad devices
        const auto gamepads = deviceService_->getDevicesByType(DeviceType::GAMEPAD);

        // Update each gamepad
        for (std::size_t i = 0; i < std::min(gamepads.size(), static_cast<std::size_t>(MAX_PLAYERS)); ++i) {
            if (const auto* gamepad = dynamic_cast<GamepadDevice*>(gamepads[i])) {
                gamepad->updateSnapshot(snapshot);
            }
        }
    }

    void SnapshotManager::updateTouchState(InputSnapshot& snapshot) const {
        if (!deviceService_) {
            return;
        }

        // Get touch devices
        if (const auto touchDevices = deviceService_->getDevicesByType(DeviceType::TOUCH); touchDevices.empty()) {
            return;
        }

        // Touch implementation would go here
        // Currently just clear the state
        snapshot.touch.clearFrameState();
    }

    bool SnapshotManager::processEventIntoSnapshot(InputSnapshot& snapshot, const InputEvent& event) {
        switch (event.type) {
        case InputEventType::KEY_PRESSED:
            if (auto* data = event.getEventData<KeyboardEventData>()) {
                snapshot.keyboard.updateKey(data->key, true);
                snapshot.keyboard.modifiers = data->modifiers;
                return true;
            }
            break;

        case InputEventType::KEY_RELEASED:
            if (auto* data = event.getEventData<KeyboardEventData>()) {
                snapshot.keyboard.updateKey(data->key, false);
                snapshot.keyboard.modifiers = data->modifiers;
                return true;
            }
            break;

        case InputEventType::MOUSE_BUTTON_PRESSED:
            if (auto* data = event.getEventData<MouseButtonEventData>()) {
                snapshot.mouse.updateButton(data->button, true);
                snapshot.mouse.position = data->position;
                return true;
            }
            break;

        case InputEventType::MOUSE_BUTTON_RELEASED:
            if (auto* data = event.getEventData<MouseButtonEventData>()) {
                snapshot.mouse.updateButton(data->button, false);
                snapshot.mouse.position = data->position;
                return true;
            }
            break;

        case InputEventType::MOUSE_MOVED:
            if (auto* data = event.getEventData<MouseMotionEventData>()) {
                snapshot.mouse.position = data->position;
                snapshot.mouse.delta = data->delta;
                return true;
            }
            break;

        case InputEventType::MOUSE_WHEEL:
            if (auto* data = event.getEventData<MouseWheelEventData>()) {
                snapshot.mouse.wheelDelta = data->delta;
                return true;
            }
            break;

        case InputEventType::GAMEPAD_BUTTON_PRESSED:
            if (auto* data = event.getEventData<GamepadButtonEventData>()) {
                if (auto* gamepad = snapshot.getGamepadMutable(event.playerId)) {
                    const auto index = static_cast<std::size_t>(data->button);
                    gamepad->buttons.set(index);
                    gamepad->buttonsJustPressed.set(index);
                    return true;
                }
            }
            break;

        case InputEventType::GAMEPAD_BUTTON_RELEASED:
            if (auto* data = event.getEventData<GamepadButtonEventData>()) {
                if (auto* gamepad = snapshot.getGamepadMutable(event.playerId)) {
                    const auto index = static_cast<std::size_t>(data->button);
                    gamepad->buttons.reset(index);
                    gamepad->buttonsJustReleased.set(index);
                    return true;
                }
            }
            break;

        case InputEventType::GAMEPAD_AXIS_MOVED:
            if (auto* data = event.getEventData<GamepadAxisEventData>()) {
                if (auto* gamepad = snapshot.getGamepadMutable(event.playerId)) {
                    switch (data->axis) {
                    case GamepadAxis::LEFT_STICK_X:
                        gamepad->leftStickRaw.x = data->value;
                        break;
                    case GamepadAxis::LEFT_STICK_Y:
                        gamepad->leftStickRaw.y = data->value;
                        break;
                    case GamepadAxis::RIGHT_STICK_X:
                        gamepad->rightStickRaw.x = data->value;
                        break;
                    case GamepadAxis::RIGHT_STICK_Y:
                        gamepad->rightStickRaw.y = data->value;
                        break;
                    case GamepadAxis::LEFT_TRIGGER:
                        gamepad->leftTriggerRaw = data->value;
                        break;
                    case GamepadAxis::RIGHT_TRIGGER:
                        gamepad->rightTriggerRaw = data->value;
                        break;
                    default:
                        break;
                    }
                    return true;
                }
            }
            break;

        default:
            break;
        }

        return false;
    }

    void SnapshotManager::interpolateAnalogValues(InputSnapshot& current,
                                                  const InputSnapshot& previous,
                                                  const float alpha) {
        // Interpolate mouse delta
        current.mouse.delta = current.mouse.delta * alpha + previous.mouse.delta * (1.0f - alpha);

        // Interpolate gamepad analog values
        for (std::size_t i = 0; i < MAX_PLAYERS; ++i) {
            if (current.gamepads[i].isConnected && previous.gamepads[i].isConnected) {
                auto& curr = current.gamepads[i];
                const auto& prev = previous.gamepads[i];

                // Interpolate sticks
                curr.leftStick = curr.leftStick * alpha + prev.leftStick * (1.0f - alpha);
                curr.rightStick = curr.rightStick * alpha + prev.rightStick * (1.0f - alpha);

                // Interpolate triggers
                curr.leftTrigger = curr.leftTrigger * alpha + prev.leftTrigger * (1.0f - alpha);
                curr.rightTrigger = curr.rightTrigger * alpha + prev.rightTrigger * (1.0f - alpha);
            }
        }
    }

    // TODO: Implementar esto, deberia usar el resource manager?
    std::vector<std::uint8_t> SnapshotManager::compressSnapshot(const InputSnapshot& snapshot) const {
        // Simple compression: just serialize for now
        // In production, would use actual compression (zlib, lz4, etc.)
        return snapshot.serialize();
    }

    // TODO: Implementar esto, deberia usar el resource manager?
    InputSnapshot SnapshotManager::decompressSnapshot(const std::vector<std::uint8_t>& data) const {
        // Simple decompression: just deserialize for now
        return InputSnapshot::deserialize(data);
    }

    void SnapshotManager::recordSnapshot(const InputSnapshot& snapshot) {
        if (!isRecording_.load(std::memory_order_acquire)) {
            return;
        }

        auto serialized = snapshot.serialize();

        // Append size and data to recording buffer
        auto size = static_cast<std::uint32_t>(serialized.size());

        recordingBuffer_.insert(recordingBuffer_.end(),
                                reinterpret_cast<std::uint8_t*>(&size),
                                reinterpret_cast<std::uint8_t*>(&size) + sizeof(size));
        recordingBuffer_.insert(recordingBuffer_.end(),
                                serialized.begin(),
                                serialized.end());
    }

    void SnapshotManager::invokeCallbacks(const InputSnapshot& snapshot) const {
        std::lock_guard lock(callbackMutex_);
        for (const auto& callback : callbacks_) {
            if (callback) {
                callback(snapshot);
            }
        }
    }

    void SnapshotManager::updateStatistics(const std::size_t eventsProcessed, const float updateTime) const {
        // Update average events per snapshot
        constexpr float alpha = 0.1f;
        stats_.averageEventsPerSnapshot = static_cast<std::uint32_t>(
            (1.0f - alpha) * stats_.averageEventsPerSnapshot + alpha * eventsProcessed
        );

        // Update average update time
        stats_.averageUpdateTime = (1.0f - alpha) * stats_.averageUpdateTime + alpha * updateTime;

        // Update peak update time
        if (updateTime > stats_.peakUpdateTime) {
            stats_.peakUpdateTime = updateTime;
        }

        // Calculate memory usage
        stats_.memoryUsage = sizeof(InputSnapshot) * snapshots_.size();
        if (history_) {
            stats_.memoryUsage += history_->getFrameCount() * sizeof(InputSnapshot);
        }
    }

    InputSnapshot* SnapshotManager::allocateSnapshot() const {
        if (snapshotPool_) {
            if (void* memory = snapshotPool_->allocate(sizeof(InputSnapshot), alignof(InputSnapshot))) {
                return new(memory) InputSnapshot();
            }
        }

        // TODO: Ver si usar por defecto memory manager
        // TODO: Ver que mierda hacer con el tema de asignar memoria
        // https://grok.com/c/3e5f1bcc-e4fc-48ad-b8b4-f060f01796e6
        // Fallback to heap allocation
        return new InputSnapshot();
    }

    void SnapshotManager::deallocateSnapshot(InputSnapshot* snapshot) const {
        if (!snapshot) return;

        if (snapshotPool_ && snapshotPool_->owns(snapshot)) {
            snapshot->~InputSnapshot();
            snapshotPool_->deallocate(snapshot);
        }
        else {
            delete snapshot;
        }
    }
} // namespace engine::input::processing