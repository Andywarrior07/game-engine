/**
 * @file InputSystem.cpp
 * @brief Main input system implementation
 * @author Andrés Guerrero
 * @date 13-09-2025
 */

#include "InputSystem.h"

#include "devices/base/DeviceService.h"

#include "manager/InputService.h"
#include "manager/InputLifecycle.h"
#include "mapping/ActionMap.h"
#include "mapping/ContextStack.h"

#include "processing/SnapshotManager.h"

#include "platform/sdl2/SDLInputBackend.h"

#include "debug/InputLogger.h"
#include "debug/InputDebugOverlay.h"

#include "utils/ConfigLoader.h"

#include <fstream>
#include <chrono>

namespace engine::input {
    // ============================================================================
    // Private Implementation Class
    // ============================================================================

    class InputSystem::Impl {
    public:
        // Systems
        memory::MemoryManager* memoryManager;

        // Core components
        std::unique_ptr<InputLifecycle> lifecycle;
        std::unique_ptr<InputService> service;

        // Debug components
        std::unique_ptr<debug::InputLogger> logger;
        std::unique_ptr<debug::InputDebugOverlay> overlay;

        // Configuration
        std::unique_ptr<utils::ConfigLoader> configLoader;

        // Recording/Replay state
        struct RecordingState {
            bool isRecording = false;
            bool isReplaying = false;
            std::vector<std::uint8_t> recordingBuffer;
            std::chrono::steady_clock::time_point recordingStartTime;
        } recording;

        // State
        bool initialized = false;
        bool paused = false;
        std::chrono::steady_clock::time_point lastUpdateTime;

        // Statistics
        Statistics stats;
        std::chrono::steady_clock::time_point statsStartTime;
        float frameTimeAccumulator = 0.0f;
        std::uint32_t frameTimeCount = 0;

        // Event callbacks
        std::vector<std::function<bool(const InputEvent&)>> eventCallbacks;
        std::mutex callbackMutex;

        explicit Impl(memory::MemoryManager* mem)
            : memoryManager(mem) {
            lastUpdateTime = std::chrono::steady_clock::now();
            statsStartTime = lastUpdateTime;
        }

        ~Impl() {
            // Ensure clean shutdown
            if (initialized) {
                shutdown();
            }
        }

        void shutdown() {
            // Stop any ongoing operations
            if (recording.isRecording && service) {
                service->stopRecording();
            }
            if (recording.isReplaying && service) {
                service->stopReplay();
            }

            // Shutdown components in reverse order
            if (overlay) {
                overlay.reset();
            }
            if (logger) {
                logger->shutdown();
                logger.reset();
            }
            if (configLoader) {
                configLoader.reset();
            }
            if (service) {
                service->shutdown();
                service.reset();
            }
            if (lifecycle) {
                lifecycle->shutdown();
                lifecycle.reset();
            }

            initialized = false;
        }

        void updateStatistics(const float frameTime) {
            stats.totalFrames++;
            frameTimeAccumulator += frameTime;
            frameTimeCount++;

            // Update average frame time (every 60 frames)
            if (frameTimeCount >= 60) {
                stats.averageFrameTime = frameTimeAccumulator / static_cast<float>(frameTimeCount);
                frameTimeAccumulator = 0.0f;
                frameTimeCount = 0;
            }

            // Update peak frame time
            if (frameTime > stats.peakFrameTime) {
                stats.peakFrameTime = frameTime;
            }

            // Calculate memory usage
            if (service) {
                if (auto* snapshotManager = service->getSnapshotManager()) {
                    stats.memoryUsage = sizeof(InputSnapshot) * 2; // Double buffered snapshots

                    // Add history memory
                    if (const auto* history = snapshotManager->getHistory()) {
                        stats.memoryUsage += history->getFrameCount() * sizeof(InputSnapshot);
                    }
                }
            }
        }
    };

    // ============================================================================
    // InputSystem Implementation
    // ============================================================================

    InputSystem::InputSystem(memory::MemoryManager* memoryManager)
        : pImpl(std::make_unique<Impl>(memoryManager)) {
    }

    InputSystem::~InputSystem() {
        if (pImpl && pImpl->initialized) {
            shutdown();
        }
    }

    InputSystem::InputSystem(InputSystem&&) noexcept = default;
    InputSystem& InputSystem::operator=(InputSystem&&) noexcept = default;

    bool InputSystem::initialize(const InputSystemConfig& config) {
        if (pImpl->initialized) {
            return true;
        }

        config_ = config;

        // Create lifecycle manager
        pImpl->lifecycle = std::make_unique<InputLifecycle>(pImpl->memoryManager);

        // Configure lifecycle
        LifecycleConfig lifecycleConfig;
        lifecycleConfig.autoInitializeDevices = true;
        lifecycleConfig.validateConfiguration = true;
        lifecycleConfig.preallocateMemory = true;
        lifecycleConfig.targetUpdateRate = 60.0f;
        lifecycleConfig.enableProfiling = config.enableDebugLogging;

        // Create input configuration
        InputConfig inputConfig;
        inputConfig.keyboard.enabled = config.enableKeyboard;
        inputConfig.mouse.enabled = config.enableMouse;
        inputConfig.gamepad.enabled = config.enableGamepad;
        inputConfig.touch.enabled = config.enableTouch;
        inputConfig.mouse.sensitivity = config.mouseSensitivity;
        inputConfig.gamepad.stickDeadzone = config.analogDeadzone;
        inputConfig.gamepad.triggerThreshold = config.triggerThreshold;
        inputConfig.memory.eventPoolSize = config.eventPoolSize;
        inputConfig.memory.snapshotPoolSize = config.snapshotPoolSize;
        inputConfig.memory.historyBufferFrames = config.historyFrameCount;
        inputConfig.processing.enableInputBuffer = config.enableInputBuffering;
        inputConfig.processing.inputBufferFrames = config.inputBufferFrames;
        inputConfig.debug.enableLogging = config.enableDebugLogging;
        inputConfig.debug.recordForReplay = config.recordForReplay;

        // Initialize lifecycle
        if (!pImpl->lifecycle->initialize(inputConfig, lifecycleConfig)) {
            pImpl->shutdown();
            return false;
        }

        // Get input service from lifecycle
        pImpl->service = std::unique_ptr<InputService>(pImpl->lifecycle->getInputService());
        if (!pImpl->service) {
            // Create service if lifecycle didn't create it
            pImpl->service = std::make_unique<InputService>(pImpl->memoryManager);

            InputServiceConfig serviceConfig;
            serviceConfig.eventPoolSize = config.eventPoolSize;
            serviceConfig.snapshotPoolSize = config.snapshotPoolSize;
            serviceConfig.historyFrameCount = config.historyFrameCount;
            serviceConfig.enableKeyboard = config.enableKeyboard;
            serviceConfig.enableMouse = config.enableMouse;
            serviceConfig.enableGamepad = config.enableGamepad;
            serviceConfig.enableTouch = config.enableTouch;
            serviceConfig.analogDeadzone = config.analogDeadzone;
            serviceConfig.triggerThreshold = config.triggerThreshold;
            serviceConfig.mouseSensitivity = config.mouseSensitivity;
            serviceConfig.enableInputBuffering = config.enableInputBuffering;
            serviceConfig.inputBufferFrames = config.inputBufferFrames;
            serviceConfig.enableDebugLogging = config.enableDebugLogging;
            serviceConfig.recordInputForReplay = config.recordForReplay;

            if (!pImpl->service->initialize(serviceConfig)) {
                pImpl->shutdown();
                return false;
            }
        }

        // Initialize debug components if enabled
        if (config.enableDebugLogging) {
            pImpl->logger = std::make_unique<debug::InputLogger>();

            debug::LoggerConfig loggerConfig;
            loggerConfig.minLevel = debug::LogLevel::INFO;
            loggerConfig.logToFile = true;
            loggerConfig.logToConsole = false;
            loggerConfig.logFilePath = "input_system.log";
            loggerConfig.maxFileSize = 10 * 1024 * 1024; // 10MB
            loggerConfig.rotateFiles = true;

            if (!pImpl->logger->initialize(loggerConfig)) {
                pImpl->logger.reset();
            }
        }

        if (config.enableDebugOverlay) {
            pImpl->overlay = std::make_unique<debug::InputDebugOverlay>();

            debug::OverlayConfig overlayConfig;
            overlayConfig.enabled = false; // Start disabled
            overlayConfig.enabledSections = debug::OverlaySection::ALL;
            overlayConfig.opacity = 0.8f;

            pImpl->overlay->setConfig(overlayConfig);
        }

        // TODO: URGENTE tengo que pasar pad donde esta la cnofiguración
        /*
        pImpl->configLoader = std::make_unique<utils::ConfigLoader>(
            "data/input/",        // engineDataPath - where core configs are stored
            getUserConfigPath()   // userConfigPath - platform-specific user directory
        );
         */
        // TODO: URGENTE Además preguntar por esto, ya que no se si lo tengo implementado
        /*
        // Add this private method to InputSystem::Impl or as a utility function
std::filesystem::path getUserConfigPath() {
    #ifdef _WIN32
        if (const char* appData = std::getenv("APPDATA")) {
            return std::filesystem::path(appData) / "YourGameEngine" / "input";
        }
    #elif __APPLE__
        if (const char* home = std::getenv("HOME")) {
            return std::filesystem::path(home) / "Library" / "Application Support" / "YourGameEngine" / "input";
        }
    #else
        if (const char* home = std::getenv("HOME")) {
            return std::filesystem::path(home) / ".config" / "yourgameengine" / "input";
        }
    #endif
    return "config/input/"; // Fallback
}
         */
        // TODO: Tambien se podría hacer una validación que si autoLoadConfig esta en true, aplicar configLoader
        // Create config loader
        // pImpl->configLoader = std::make_unique<utils::ConfigLoader>();

        // Load initial configuration if specified
        if (config.autoLoadConfig && !config.configFilePath.empty()) {
            loadConfiguration(config.configFilePath);
        }

        // Start lifecycle
        if (!pImpl->lifecycle->start()) {
            pImpl->shutdown();
            return false;
        }

        pImpl->initialized = true;
        pImpl->statsStartTime = std::chrono::steady_clock::now();

        return true;
    }

    void InputSystem::shutdown() {
        if (!pImpl || !pImpl->initialized) {
            return;
        }

        // Save configuration if requested
        if (config_.autoSaveConfig && !config_.configFilePath.empty()) {
            saveConfiguration(config_.configFilePath);
        }

        pImpl->shutdown();
    }

    void InputSystem::update(const float deltaTime) const {
        if (!pImpl->initialized || pImpl->paused) {
            return;
        }

        const auto updateStart = std::chrono::steady_clock::now();

        // Update lifecycle
        if (pImpl->lifecycle) {
            pImpl->lifecycle->update(deltaTime);
        }

        // Update service
        if (pImpl->service) {
            pImpl->service->tick(deltaTime);
        }

        // Update debug overlay if enabled
        if (pImpl->overlay && pImpl->overlay->isEnabled()) {
            if (auto* snapshot = getSnapshot()) {
                pImpl->overlay->update(*snapshot, deltaTime);
            }
        }

        // Update statistics
        const auto updateEnd = std::chrono::steady_clock::now();
        const float updateTime = std::chrono::duration<float, std::milli>(updateEnd - updateStart).count();

        pImpl->updateStatistics(updateTime);

        // Log frame info if verbose logging
        if (pImpl->logger) {
            pImpl->logger->setFrameNumber(pImpl->stats.totalFrames);
            if (pImpl->stats.totalFrames % 60 == 0) {
                // Log every second at 60fps
                pImpl->logger->logPerformance("InputSystem::update", updateTime, "ms");
            }
        }

        pImpl->lastUpdateTime = updateEnd;
    }

    bool InputSystem::isInitialized() const noexcept {
        return pImpl && pImpl->initialized;
    }

    void InputSystem::pause() const {
        if (pImpl) {
            pImpl->paused = true;
            if (pImpl->service) {
                pImpl->service->setPaused(true);
            }
            if (pImpl->lifecycle) {
                pImpl->lifecycle->pause();
            }
        }
    }

    void InputSystem::resume() const {
        if (pImpl) {
            pImpl->paused = false;
            if (pImpl->service) {
                pImpl->service->setPaused(false);
            }
            if (pImpl->lifecycle) {
                pImpl->lifecycle->resume();
            }
        }
    }

    bool InputSystem::isPaused() const noexcept {
        return pImpl ? pImpl->paused : true;
    }

    // ============================================================================
    // Input State Access
    // ============================================================================

    const InputSnapshot* InputSystem::getSnapshot() const {
        if (!pImpl || !pImpl->service) return nullptr;

        return pImpl->service->getSnapshot();
    }

    const InputSnapshot* InputSystem::getPreviousSnapshot() const {
        if (!pImpl || !pImpl->service) return nullptr;

        return pImpl->service->getPreviousSnapshot();
    }

    bool InputSystem::isKeyPressed(const KeyCode key) const {
        if (auto* snapshot = getSnapshot()) {
            return snapshot->keyboard.isKeyPressed(key);
        }

        return false;
    }

    bool InputSystem::isKeyJustPressed(const KeyCode key) const {
        if (auto* snapshot = getSnapshot()) {
            return snapshot->keyboard.isKeyJustPressed(key);
        }

        return false;
    }

    bool InputSystem::isKeyJustReleased(const KeyCode key) const {
        if (auto* snapshot = getSnapshot()) {
            return snapshot->keyboard.isKeyJustReleased(key);
        }

        return false;
    }

    bool InputSystem::isMouseButtonPressed(const MouseButton button) const {
        if (auto* snapshot = getSnapshot()) {
            return snapshot->mouse.isButtonPressed(button);
        }

        return false;
    }

    math::Vec2 InputSystem::getMousePosition() const {
        if (auto* snapshot = getSnapshot()) {
            return snapshot->mouse.position;
        }

        return math::VEC2_ZERO;
    }

    math::Vec2 InputSystem::getMouseDelta() const {
        if (auto* snapshot = getSnapshot()) {
            return snapshot->mouse.delta;
        }

        return math::VEC2_ZERO;
    }

    math::Vec2 InputSystem::getMouseWheel() const {
        if (auto* snapshot = getSnapshot()) {
            return snapshot->mouse.wheelDelta;
        }

        return math::VEC2_ZERO;
    }

    bool InputSystem::isGamepadButtonPressed(const PlayerID player, const GamepadButton button) const {
        if (auto* snapshot = getSnapshot()) {
            if (auto* gamepad = snapshot->getGamepad(player)) {
                return gamepad->isButtonPressed(button);
            }
        }

        return false;
    }

    math::Vec2 InputSystem::getGamepadStick(const PlayerID player, const bool leftStick) const {
        if (auto* snapshot = getSnapshot()) {
            if (auto* gamepad = snapshot->getGamepad(player)) {
                return leftStick ? gamepad->leftStick : gamepad->rightStick;
            }
        }

        return math::VEC2_ZERO;
    }

    float InputSystem::getGamepadTrigger(const PlayerID player, const bool leftTrigger) const {
        if (auto* snapshot = getSnapshot()) {
            if (auto* gamepad = snapshot->getGamepad(player)) {
                return leftTrigger ? gamepad->leftTrigger : gamepad->rightTrigger;
            }
        }

        return 0.0f;
    }

    // ============================================================================
    // Action System
    // ============================================================================

    void InputSystem::registerAction(const ActionID id, const std::string& name, const ActionType type) const {
        if (pImpl && pImpl->service) {
            pImpl->service->registerAction(id, name, type);
        }
    }

    void InputSystem::bindKey(const ActionID actionId, const KeyCode key, const std::string& context) const {
        if (pImpl && pImpl->service) {
            pImpl->service->bindInput(actionId, key, context);
        }
    }

    void InputSystem::bindMouseButton(const ActionID actionId, const MouseButton button,
                                      const std::string& context) const {
        if (pImpl && pImpl->service) {
            pImpl->service->bindInput(actionId, button, context);
        }
    }

    auto InputSystem::bindGamepadButton(const ActionID actionId, const GamepadButton button,
                                        const std::string& context) const -> void {
        if (pImpl && pImpl->service) {
            pImpl->service->bindInput(actionId, button, context);
        }
    }

    void InputSystem::bindGamepadAxis(const ActionID actionId, const GamepadAxis axis,
                                      const std::string& context) const {
        if (pImpl && pImpl->service) {
            pImpl->service->bindAxis(actionId, axis, context);
        }
    }

    bool InputSystem::isActionTriggered(const ActionID actionId) const {
        if (auto* snapshot = getSnapshot()) {
            return snapshot->isActionTriggered(actionId);
        }

        return false;
    }

    float InputSystem::getActionValue(const ActionID actionId) const {
        if (auto* snapshot = getSnapshot()) {
            return snapshot->getActionValue(actionId);
        }

        return 0.0f;
    }

    math::Vec2 InputSystem::getActionValue2D(const ActionID actionId) const {
        if (auto* snapshot = getSnapshot()) {
            return snapshot->getActionValue2D(actionId);
        }

        return math::VEC2_ZERO;
    }

    void InputSystem::registerActionCallback(const ActionID actionId, ActionCallback callback) const {
        if (pImpl && pImpl->service) {
            pImpl->service->registerActionCallback(actionId, std::move(callback));
        }
    }

    // ============================================================================
    // Context Management
    // ============================================================================

    bool InputSystem::pushContext(const std::string& name, const ContextPriority priority) const {
        if (pImpl && pImpl->service) {
            return pImpl->service->pushContext(name, priority);
        }

        return false;
    }

    bool InputSystem::popContext() const {
        if (pImpl && pImpl->service) {
            return pImpl->service->popContext();
        }

        return false;
    }

    bool InputSystem::setContext(const std::string& name, const ContextPriority priority) const {
        if (pImpl && pImpl->service) {
            // Clear stack and push new context
            if (auto* contextStack = pImpl->service->getContextStack()) {
                contextStack->clearStack();
                return pImpl->service->pushContext(name, priority);
            }
        }
        return false;
    }

    std::string InputSystem::getCurrentContext() const {
        if (pImpl && pImpl->service) {
            return pImpl->service->getCurrentContext();
        }
        return "";
    }

    void InputSystem::registerContextChangeCallback(ContextChangeCallback callback) const {
        if (pImpl && pImpl->service) {
            pImpl->service->registerContextChangeCallback(std::move(callback));
        }
    }

    // ============================================================================
    // Device Control
    // ============================================================================

    bool InputSystem::setGamepadRumble(const PlayerID player, const float leftMotor, const float rightMotor, const float duration) const {
        if (pImpl && pImpl->service) {
            return pImpl->service->setGamepadRumble(player, leftMotor, rightMotor, duration);
        }

        return false;
    }

    void InputSystem::stopGamepadRumble(const PlayerID player) const {
        if (pImpl && pImpl->service) {
            pImpl->service->setGamepadRumble(player, 0.0f, 0.0f, 0.0f);
        }
    }

    void InputSystem::enableTextInput() const {
        if (pImpl && pImpl->service) {
            pImpl->service->setTextInputMode(true);
        }
    }

    void InputSystem::disableTextInput() const {
        if (pImpl && pImpl->service) {
            pImpl->service->setTextInputMode(false);
        }
    }

    bool InputSystem::isTextInputActive() const {
        if (pImpl && pImpl->service && pImpl->service->getBackend()) {
            return pImpl->service->getBackend()->isTextInputActive();
        }

        return false;
    }

    bool InputSystem::setRelativeMouseMode(const bool enabled) const {
        if (pImpl && pImpl->service) {
            return pImpl->service->setRelativeMouseMode(enabled);
        }

        return false;
    }

    // TODO: Revisar e implementar esto
    bool InputSystem::isRelativeMouseMode() const {
        // Check SDL backend for current state
        if (pImpl && pImpl->service && pImpl->service->getBackend()) {
            // Would need to add getter to SDLInputBackend
            return false; // Placeholder
        }
        return false;
    }

    void InputSystem::setMouseCursorVisible(const bool visible) const {
        if (pImpl && pImpl->service && pImpl->service->getBackend()) {
            pImpl->service->getBackend()->setCursorVisibility(visible);
        }
    }

    // ============================================================================
    // Configuration
    // ============================================================================

    bool InputSystem::loadConfiguration(const std::string& filepath) {
        if (!pImpl || !pImpl->configLoader) {
            return false;
        }

        const auto configData = pImpl->configLoader->loadFromFile(filepath);
        if (!configData) {
            if (pImpl->logger) {
                pImpl->logger->error("Failed to load input configuration", filepath);
            }
            return false;
        }

        // Apply to action map
        if (pImpl->service && pImpl->service->getActionMap()) {
            pImpl->configLoader->applyToActionMap(*configData, pImpl->service->getActionMap());
        }

        // Apply to context stack
        if (pImpl->service && pImpl->service->getContextStack()) {
            pImpl->configLoader->applyToContextStack(*configData, pImpl->service->getContextStack());
        }

        // Update device settings
        config_.enableKeyboard = configData->devices.enableKeyboard;
        config_.enableMouse = configData->devices.enableMouse;
        config_.enableGamepad = configData->devices.enableGamepad;
        config_.enableTouch = configData->devices.enableTouch;
        config_.mouseSensitivity = configData->devices.mouseSensitivity;
        config_.analogDeadzone = configData->devices.gamepadDeadzone;
        config_.triggerThreshold = configData->devices.triggerThreshold;

        if (pImpl->logger) {
            pImpl->logger->info("Loaded input configuration", filepath);
        }

        return true;
    }

    bool InputSystem::saveConfiguration(const std::string& filepath) const {
        if (!pImpl || !pImpl->configLoader) {
            return false;
        }

        utils::InputConfigData configData;

        // Extract from action map
        if (pImpl->service && pImpl->service->getActionMap()) {
            configData = pImpl->configLoader->extractFromActionMap(pImpl->service->getActionMap());
        }

        // Set device settings
        configData.devices.enableKeyboard = config_.enableKeyboard;
        configData.devices.enableMouse = config_.enableMouse;
        configData.devices.enableGamepad = config_.enableGamepad;
        configData.devices.enableTouch = config_.enableTouch;
        configData.devices.mouseSensitivity = config_.mouseSensitivity;
        configData.devices.gamepadDeadzone = config_.analogDeadzone;
        configData.devices.triggerThreshold = config_.triggerThreshold;

        const bool success = pImpl->configLoader->saveToFile(configData, filepath);

        if (pImpl->logger) {
            if (success) {
                pImpl->logger->info("Saved input configuration", filepath);
            }
            else {
                pImpl->logger->error("Failed to save input configuration", filepath);
            }
        }

        return success;
    }

    // TODO: Esto para cuando se quiera aplicar un preset, mientras por los errores dejar comentado
    // void InputSystem::applyPreset(const std::string& presetName) const {
    //     if (!pImpl || !pImpl->configLoader) {
    //         return;
    //     }
    //
    //     const utils::InputConfigData configData = utils::ConfigLoader::getDefaultConfiguration(presetName);
    //
    //     // Apply to action map
    //     if (pImpl->service && pImpl->service->getActionMap()) {
    //         pImpl->configLoader->applyToActionMap(configData, pImpl->service->getActionMap());
    //     }
    //
    //     // Apply to context stack
    //     if (pImpl->service && pImpl->service->getContextStack()) {
    //         pImpl->configLoader->applyToContextStack(configData, pImpl->service->getContextStack());
    //     }
    //
    //     if (pImpl->logger) {
    //         pImpl->logger->info("Applied input preset", presetName);
    //     }
    // }

    // ============================================================================
    // Recording and Replay
    // ============================================================================

    void InputSystem::startRecording() const {
        if (!pImpl || !pImpl->service) {
            return;
        }

        pImpl->service->startRecording();
        pImpl->recording.isRecording = true;
        pImpl->recording.recordingStartTime = std::chrono::steady_clock::now();
        pImpl->recording.recordingBuffer.clear();

        if (pImpl->logger) {
            pImpl->logger->info("Started input recording");
        }
    }

    void InputSystem::stopRecording() const {
        if (!pImpl || !pImpl->service) {
            return;
        }

        pImpl->service->stopRecording();
        pImpl->recording.isRecording = false;

        // Export recording data
        pImpl->recording.recordingBuffer = pImpl->service->exportRecording();

        if (pImpl->logger) {
            const auto duration = std::chrono::steady_clock::now() - pImpl->recording.recordingStartTime;
            const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
            pImpl->logger->info("Stopped input recording", std::to_string(seconds) + " seconds");
        }
    }

    bool InputSystem::isRecording() const {
        return pImpl ? pImpl->recording.isRecording : false;
    }

    bool InputSystem::saveRecording(const std::string& filepath) const {
        if (!pImpl || pImpl->recording.recordingBuffer.empty()) {
            return false;
        }

        std::ofstream file(filepath, std::ios::binary);

        if (!file) {
            if (pImpl->logger) {
                pImpl->logger->error("Failed to open recording file", filepath);
            }
            return false;
        }

        file.write(reinterpret_cast<const char*>(pImpl->recording.recordingBuffer.data()),
                   pImpl->recording.recordingBuffer.size());

        if (pImpl->logger) {
            pImpl->logger->info("Saved input recording", filepath);
        }

        return true;
    }

    bool InputSystem::playRecording(const std::string& filepath) const {
        if (!pImpl || !pImpl->service) {
            return false;
        }

        std::ifstream file(filepath, std::ios::binary);
        if (!file) {
            if (pImpl->logger) {
                pImpl->logger->error("Failed to open recording file", filepath);
            }
            return false;
        }

        // Read file into buffer
        file.seekg(0, std::ios::end);

        const std::size_t size = file.tellg();

        file.seekg(0, std::ios::beg);

        std::vector<std::uint8_t> replayData(size);

        file.read(reinterpret_cast<char*>(replayData.data()), size);

        const bool success = pImpl->service->startReplay(replayData);

        if (success) {
            pImpl->recording.isReplaying = true;

            if (pImpl->logger) {
                pImpl->logger->info("Started input replay", filepath);
            }
        }

        return success;
    }

    void InputSystem::stopReplay() const {
        if (!pImpl || !pImpl->service) {
            return;
        }

        pImpl->service->stopReplay();
        pImpl->recording.isReplaying = false;

        if (pImpl->logger) {
            pImpl->logger->info("Stopped input replay");
        }
    }

    bool InputSystem::isReplaying() const {
        return pImpl ? pImpl->recording.isReplaying : false;
    }

    // ============================================================================
    // Debug
    // ============================================================================

    void InputSystem::setDebugLoggingEnabled(const bool enabled) {
        config_.enableDebugLogging = enabled;

        if (pImpl) {
            if (enabled && !pImpl->logger) {
                // Create logger
                pImpl->logger = std::make_unique<debug::InputLogger>();

                debug::LoggerConfig loggerConfig;
                loggerConfig.minLevel = debug::LogLevel::INFO;
                loggerConfig.logToFile = true;
                loggerConfig.logFilePath = "input_system.log";

                pImpl->logger->initialize(loggerConfig);
            }
            else if (!enabled && pImpl->logger) {
                // Shutdown logger
                pImpl->logger->shutdown();
                pImpl->logger.reset();
            }
        }
    }

    void InputSystem::setDebugOverlayEnabled(const bool enabled) {
        config_.enableDebugOverlay = enabled;

        if (pImpl) {
            if (enabled && !pImpl->overlay) {
                // Create overlay
                pImpl->overlay = std::make_unique<debug::InputDebugOverlay>();

                debug::OverlayConfig overlayConfig;
                overlayConfig.enabled = true;
                pImpl->overlay->setConfig(overlayConfig);
            }
            else if (pImpl->overlay) {
                pImpl->overlay->setEnabled(enabled);
            }
        }
    }

    void InputSystem::toggleDebugOverlay() const {
        if (pImpl && pImpl->overlay) {
            pImpl->overlay->toggle();
        }
    }

    debug::InputDebugOverlay* InputSystem::getDebugOverlay() const {
        return pImpl ? pImpl->overlay.get() : nullptr;
    }

    void InputSystem::registerEventCallback(EventCallback callback) const {
        if (pImpl && pImpl->service) {
            pImpl->service->registerRawEventCallback(
                [callback = std::move(callback)](const InputEvent& event) {
                    return callback(event);
                }
            );
        }

        // Also store in our impl for direct event injection
        if (pImpl) {
            std::lock_guard lock(pImpl->callbackMutex);
            pImpl->eventCallbacks.push_back(std::move(callback));
        }
    }

    void InputSystem::injectEvent(const InputEvent& event) const {
        if (pImpl && pImpl->service) {
            pImpl->service->injectEvent(event);
        }

        // Also call direct callbacks
        if (pImpl) {
            std::lock_guard lock(pImpl->callbackMutex);
            for (auto& callback : pImpl->eventCallbacks) {
                if (!callback(event)) {
                    break; // Stop if callback returns false
                }
            }
        }
    }

    // ============================================================================
    // Statistics
    // ============================================================================

    InputSystem::Statistics InputSystem::getStatistics() const {
        if (!pImpl) {
            return Statistics{};
        }

        // Copy statistics to avoid race conditions
        Statistics stats = pImpl->stats;

        // Add additional statistics from service
        if (pImpl->service) {
            const auto& serviceStats = pImpl->service->getStatistics();
            stats.totalEvents = serviceStats.droppedEvents;
        }

        return stats;
    }

    void InputSystem::resetStatistics() const {
        if (!pImpl) {
            return;
        }

        pImpl->stats.reset();
        pImpl->frameTimeAccumulator = 0.0f;
        pImpl->frameTimeCount = 0;
        pImpl->statsStartTime = std::chrono::steady_clock::now();

        // Reset service statistics
        if (pImpl->service) {
            pImpl->service->resetStatistics();
        }

        // Reset lifecycle statistics
        if (pImpl->lifecycle) {
            pImpl->lifecycle->resetPerformanceStats();
        }

        // Reset logger statistics
        if (pImpl->logger) {
            pImpl->logger->resetStatistics();
        }
    }
} // namespace engine::input
