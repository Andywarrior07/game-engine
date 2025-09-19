/**
 * @file SDLInputBackend.cpp
 * @brief SDL2 input backend implementation
 * @author Andrés Guerrero
 * @date 13-09-2025
 */

#include "SDLInputBackend.h"
#include "SDLKeyMap.h"

#include "../../devices/base/InputDevice.h"
#include "../../devices/mouse/MouseLock.h"
#include "../../core/InputMemoryPools.h"
#include "../../devices/base/DeviceService.h"

#include <algorithm>
#include <fstream>

namespace engine::input {
    SDLInputBackend::SDLInputBackend(DeviceService* deviceService,
                                     InputMemoryPools* memoryPools) noexcept
        : deviceService_(deviceService)
          , memoryPools_(memoryPools)
          , lastMousePosition_(math::VEC2_ZERO) {
        // Create key map
        keyMap_ = std::make_unique<SDLKeyMap>();
    }

    SDLInputBackend::~SDLInputBackend() {
        shutdown();
    }

    bool SDLInputBackend::initialize(const SDLBackendConfig& config) {
        if (initialized_) {
            return false;
        }

        config_ = config;

        // Initialize SDL subsystems
        if (SDL_InitSubSystem(config_.sdlInitFlags) != 0) {
            SDL_LogError(SDL_LOG_CATEGORY_INPUT,
                         "Failed to initialize SDL: %s", SDL_GetError());
            return false;
        }

        // Load controller mappings
        if (config_.autoLoadControllerMappings) {
            loadControllerMappings(config_.controllerMappingsFile);
        }

        // Enable/disable text input
        if (config_.enableTextInput) {
            SDL_StartTextInput();
        }
        else {
            SDL_StopTextInput();
        }

        // Set initial mouse state
        SDL_ShowCursor(config_.showCursor ? SDL_ENABLE : SDL_DISABLE);
        if (config_.enableRelativeMode) {
            SDL_SetRelativeMouseMode(SDL_TRUE);
        }

        // Get window for mouse normalization
        window_ = SDL_GetMouseFocus();
        if (window_) {
            updateWindowSize();
        }

        // Scan for initial devices
        scanDevices();

        // Setup mouse lock callbacks if available
        if (mouseLock_) {
            mouseLock_->setPlatformCallbacks(
                [this](const int x, const int y) { warpMouse(x, y); },
                [](const bool enabled) { return setRelativeMouseMode(enabled); },
                [](const bool visible) { setCursorVisibility(visible); },
                [this](const int x, const int y, const int w, const int h) { return confineCursor(x, y, w, h); },
                [this]() {
                    if (window_) {
                        int w, h;
                        SDL_GetWindowSize(window_, &w, &h);
                        return std::make_pair(w, h);
                    }
                    return std::make_pair(windowWidth_, windowHeight_);
                }
            );
        }

        initialized_ = true;
        return true;
    }

    void SDLInputBackend::shutdown() {
        if (!initialized_) {
            return;
        }

        // Close all controllers
        {
            std::lock_guard lock(controllerMutex_);
            for (const auto& info : controllers_ | std::views::values) {
                if (info.haptic) {
                    SDL_HapticClose(info.haptic);
                }
                if (info.controller) {
                    SDL_GameControllerClose(info.controller);
                }
            }
            controllers_.clear();
            deviceToInstance_.clear();
        }

        // Clear event queue
        clearPendingEvents();

        // Stop text input
        SDL_StopTextInput();

        // Reset mouse state
        SDL_SetRelativeMouseMode(SDL_FALSE);
        SDL_ShowCursor(SDL_ENABLE);

        // Quit SDL subsystems
        SDL_QuitSubSystem(config_.sdlInitFlags);

        initialized_ = false;
    }

    std::size_t SDLInputBackend::pollEvents() {
        if (!initialized_) {
            return 0;
        }

        SDL_Event event;
        std::size_t eventsProcessed = 0;

        // Poll up to max events per frame
        while (eventsProcessed < config_.maxEventsPerPoll && SDL_PollEvent(&event)) {
            if (processSDLEvent(event)) {
                eventsProcessed++;
            }
        }

        stats_.totalEventsProcessed.fetch_add(eventsProcessed, std::memory_order_relaxed);
        return eventsProcessed;
    }

    bool SDLInputBackend::processSDLEvent(const SDL_Event& sdlEvent) {
        // Update modifiers for keyboard events
        if (sdlEvent.type == SDL_KEYDOWN || sdlEvent.type == SDL_KEYUP) {
            updateModifiers(sdlEvent);
        }

        switch (sdlEvent.type) {
        // Keyboard events
        case SDL_KEYDOWN:
        case SDL_KEYUP:
            processKeyboardEvent(sdlEvent);
            return true;

        // Mouse events
        case SDL_MOUSEBUTTONDOWN:
        case SDL_MOUSEBUTTONUP:
            processMouseButtonEvent(sdlEvent);
            return true;

        case SDL_MOUSEMOTION:
            processMouseMotionEvent(sdlEvent);
            return true;

        case SDL_MOUSEWHEEL:
            processMouseWheelEvent(sdlEvent);
            return true;

        // Controller events
        case SDL_CONTROLLERBUTTONDOWN:
        case SDL_CONTROLLERBUTTONUP:
            if (config_.enableControllerEvents) {
                processControllerButtonEvent(sdlEvent);
                return true;
            }
            break;

        case SDL_CONTROLLERAXISMOTION:
            if (config_.enableControllerEvents) {
                processControllerAxisEvent(sdlEvent);
                return true;
            }
            break;

        case SDL_CONTROLLERDEVICEADDED:
        case SDL_CONTROLLERDEVICEREMOVED:
        case SDL_CONTROLLERDEVICEREMAPPED:
            if (config_.enableControllerEvents) {
                processControllerDeviceEvent(sdlEvent);
                return true;
            }
            break;

        // Joystick events (for non-controller joysticks)
        case SDL_JOYAXISMOTION:
        case SDL_JOYBALLMOTION:
        case SDL_JOYHATMOTION:
        case SDL_JOYBUTTONDOWN:
        case SDL_JOYBUTTONUP:
        case SDL_JOYDEVICEADDED:
        case SDL_JOYDEVICEREMOVED:
            processJoystickEvent(sdlEvent);
            return true;

        // Touch events
        case SDL_FINGERDOWN:
        case SDL_FINGERUP:
        case SDL_FINGERMOTION:
            if (config_.enableTouchEvents) {
                processTouchEvent(sdlEvent);
                return true;
            }
            break;

        // Text input
        case SDL_TEXTINPUT:
        case SDL_TEXTEDITING:
            processTextInputEvent(sdlEvent);
            return true;

        // Window events
        case SDL_WINDOWEVENT:
            if (config_.processWindowEvents) {
                processWindowEvent(sdlEvent);
                return true;
            }
            break;

        // Application events
        case SDL_QUIT:
            // Generate a system quit event
        {
            InputEvent event;
            event.type = InputEventType::FOCUS_LOST;
            event.deviceId = INVALID_DEVICE_ID;
            event.timestamp = InputTimestamp::clock::now();
            queueEvent(std::move(event));
        }
            return true;

        default:
            break;
        }

        return false;
    }

    void SDLInputBackend::pumpEvents() {
        SDL_PumpEvents();
    }

    std::size_t SDLInputBackend::getPendingEvents(std::vector<InputEvent>& events,
                                                  const std::size_t maxEvents) {
        std::lock_guard lock(eventMutex_);

        std::size_t count = 0;
        while (!eventQueue_.empty() && count < maxEvents) {
            events.push_back(std::move(eventQueue_.front()));
            eventQueue_.pop();
            count++;
        }

        return count;
    }

    void SDLInputBackend::clearPendingEvents() {
        std::lock_guard lock(eventMutex_);
        std::queue<InputEvent> empty;
        std::swap(eventQueue_, empty);
    }

    std::size_t SDLInputBackend::scanDevices() {
        std::size_t devicesFound = 0;

        // Scan for game controllers
        const int numJoysticks = SDL_NumJoysticks();
        for (int i = 0; i < numJoysticks; ++i) {
            if (SDL_IsGameController(i)) {
                openController(i);
                devicesFound++;
            }
        }

        stats_.controllersConnected.store(static_cast<std::uint32_t>(controllers_.size()),
                                          std::memory_order_relaxed);

        return devicesFound;
    }

    SDL_GameController* SDLInputBackend::getController(const DeviceID deviceId) const {
        std::lock_guard lock(controllerMutex_);

        if (const auto it = deviceToInstance_.find(deviceId); it != deviceToInstance_.end()) {
            if (const auto controllerIt = controllers_.find(it->second); controllerIt != controllers_.end()) {
                return controllerIt->second.controller;
            }
        }

        return nullptr;
    }

    SDL_Joystick* SDLInputBackend::getJoystick(const DeviceID deviceId) const {
        std::lock_guard lock(controllerMutex_);

        if (const auto it = deviceToInstance_.find(deviceId); it != deviceToInstance_.end()) {
            if (const auto controllerIt = controllers_.find(it->second); controllerIt != controllers_.end()) {
                return controllerIt->second.joystick;
            }
        }

        return nullptr;
    }

    bool SDLInputBackend::isDeviceConnected(const DeviceID deviceId) const {
        std::lock_guard lock(controllerMutex_);

        if (const auto it = deviceToInstance_.find(deviceId); it != deviceToInstance_.end()) {
            if (const auto controllerIt = controllers_.find(it->second); controllerIt != controllers_.end()) {
                return controllerIt->second.isConnected;
            }
        }

        return false;
    }

    void SDLInputBackend::warpMouse(const int x, const int y) const {
        if (window_) {
            SDL_WarpMouseInWindow(window_, x, y);
        }
        else {
            SDL_WarpMouseGlobal(x, y);
        }
    }

    bool SDLInputBackend::setRelativeMouseMode(const bool enabled) {
        return SDL_SetRelativeMouseMode(enabled ? SDL_TRUE : SDL_FALSE) == 0;
    }

    void SDLInputBackend::setCursorVisibility(const bool visible) {
        SDL_ShowCursor(visible ? SDL_ENABLE : SDL_DISABLE);
    }

    bool SDLInputBackend::confineCursor(const int x, const int y, const int width, const int height) const {
        if (!window_) {
            return false;
        }

        const SDL_Rect rect = {x, y, width, height};
        return SDL_SetWindowMouseRect(window_, &rect) == 0;
    }

    void SDLInputBackend::startTextInput() {
        SDL_StartTextInput();
    }

    void SDLInputBackend::stopTextInput() {
        SDL_StopTextInput();
    }

    bool SDLInputBackend::isTextInputActive() noexcept {
        return SDL_IsTextInputActive() == SDL_TRUE;
    }

    void SDLInputBackend::setTextInputRect(const int x, const int y, const int width, const int height) {
        const SDL_Rect rect = {x, y, width, height};
        SDL_SetTextInputRect(&rect);
    }

    bool SDLInputBackend::setControllerRumble(const DeviceID deviceId, const float leftMotor,
                                              const float rightMotor, const Uint32 duration) const {
        SDL_GameController* controller = getController(deviceId);
        if (!controller) {
            return false;
        }

        // Convert 0-1 range to SDL's 0-65535 range
        const Uint16 leftValue = static_cast<Uint16>(leftMotor * 65535.0f);
        const Uint16 rightValue = static_cast<Uint16>(rightMotor * 65535.0f);

        return SDL_GameControllerRumble(controller, leftValue, rightValue, duration) == 0;
    }

    bool SDLInputBackend::stopControllerRumble(const DeviceID deviceId) const {
        return setControllerRumble(deviceId, 0.0f, 0.0f, 0);
    }

    // ============================================================================
    // Event Processing Methods Implementation
    // ============================================================================

    void SDLInputBackend::processKeyboardEvent(const SDL_Event& event) {
        const auto keyCode = keyMap_->sdlToEngineKey(event.key.keysym.scancode);

        if (!keyCode.has_value()) {
            return;
        }

        InputEvent inputEvent;
        inputEvent.type = (event.type == SDL_KEYDOWN) ? InputEventType::KEY_PRESSED : InputEventType::KEY_RELEASED;
        inputEvent.deviceId = INVALID_DEVICE_ID; // Keyboard doesn't have specific device ID
        inputEvent.timestamp = InputTimestamp::clock::now();

        KeyboardEventData data;
        data.key = keyCode.value();
        data.modifiers = currentModifiers_;
        data.isRepeat = (event.key.repeat != 0);
        data.scancode = event.key.keysym.scancode;

        inputEvent.data = data;

        queueEvent(std::move(inputEvent));
    }

    void SDLInputBackend::processMouseButtonEvent(const SDL_Event& event) {
        InputEvent inputEvent;
        inputEvent.type = (event.type == SDL_MOUSEBUTTONDOWN)
                              ? InputEventType::MOUSE_BUTTON_PRESSED
                              : InputEventType::MOUSE_BUTTON_RELEASED;
        inputEvent.deviceId = INVALID_DEVICE_ID;
        inputEvent.timestamp = InputTimestamp::clock::now();

        MouseButtonEventData data;
        data.button = sdlToEngineMouseButton(event.button.button);
        data.position = math::Vec2(static_cast<float>(event.button.x),
                                   static_cast<float>(event.button.y));
        data.clickCount = event.button.clicks;

        inputEvent.data = data;

        queueEvent(std::move(inputEvent));
    }

    void SDLInputBackend::processMouseMotionEvent(const SDL_Event& event) {
        InputEvent inputEvent;
        inputEvent.type = InputEventType::MOUSE_MOVED;
        inputEvent.deviceId = INVALID_DEVICE_ID;
        inputEvent.timestamp = InputTimestamp::clock::now();

        MouseMotionEventData data;
        data.position = math::Vec2(static_cast<float>(event.motion.x),
                                   static_cast<float>(event.motion.y));
        data.delta = math::Vec2(static_cast<float>(event.motion.xrel),
                                static_cast<float>(event.motion.yrel));
        data.normalizedDelta = normalizeMousePosition(event.motion.xrel, event.motion.yrel);
        data.isRelative = (SDL_GetRelativeMouseMode() == SDL_TRUE);

        inputEvent.data = data;

        lastMousePosition_ = data.position;

        queueEvent(std::move(inputEvent));
    }

    void SDLInputBackend::processMouseWheelEvent(const SDL_Event& event) {
        InputEvent inputEvent;
        inputEvent.type = InputEventType::MOUSE_WHEEL;
        inputEvent.deviceId = INVALID_DEVICE_ID;
        inputEvent.timestamp = InputTimestamp::clock::now();

        MouseWheelEventData data;

        // Handle direction flipping
        const float scrollX = static_cast<float>(event.wheel.x);
        float scrollY = static_cast<float>(event.wheel.y);

        if (event.wheel.direction == SDL_MOUSEWHEEL_FLIPPED) {
            scrollY = -scrollY;
        }

        data.delta = math::Vec2(scrollX, scrollY);
        data.position = lastMousePosition_;
        data.isPrecise = (event.wheel.preciseX != 0.0f || event.wheel.preciseY != 0.0f);

        inputEvent.data = data;

        queueEvent(std::move(inputEvent));
    }

    void SDLInputBackend::processControllerButtonEvent(const SDL_Event& event) {
        const auto* info = findController(event.cbutton.which);

        if (!info) {
            return;
        }

        const auto button = keyMap_->sdlToEngineButton(
            static_cast<SDL_GameControllerButton>(event.cbutton.button));

        if (!button.has_value()) {
            return;
        }

        InputEvent inputEvent;
        inputEvent.type = (event.type == SDL_CONTROLLERBUTTONDOWN)
                              ? InputEventType::GAMEPAD_BUTTON_PRESSED
                              : InputEventType::GAMEPAD_BUTTON_RELEASED;
        inputEvent.deviceId = info->deviceId;
        inputEvent.timestamp = InputTimestamp::clock::now();

        GamepadButtonEventData data;
        data.button = button.value();
        data.pressure = 1.0f; // SDL doesn't provide pressure for buttons

        inputEvent.data = data;

        queueEvent(std::move(inputEvent));
    }

    void SDLInputBackend::processControllerAxisEvent(const SDL_Event& event) {
        const auto* info = findController(event.caxis.which);

        if (!info) {
            return;
        }

        const auto axis = keyMap_->sdlToEngineAxis(
            static_cast<SDL_GameControllerAxis>(event.caxis.axis));

        if (!axis.has_value()) {
            return;
        }

        InputEvent inputEvent;

        // Determine if it's a trigger or stick axis
        const bool isTrigger = (axis.value() == GamepadAxis::LEFT_TRIGGER ||
            axis.value() == GamepadAxis::RIGHT_TRIGGER);

        inputEvent.type = isTrigger ? InputEventType::GAMEPAD_TRIGGER_MOVED : InputEventType::GAMEPAD_AXIS_MOVED;
        inputEvent.deviceId = info->deviceId;
        inputEvent.timestamp = InputTimestamp::clock::now();

        GamepadAxisEventData data;
        data.axis = axis.value();

        // Normalize axis value
        if (isTrigger) {
            data.value = normalizeTriggerValue(event.caxis.value);
        }
        else {
            data.value = normalizeAxisValue(event.caxis.value);
        }

        inputEvent.data = data;

        queueEvent(std::move(inputEvent));
    }

    void SDLInputBackend::processControllerDeviceEvent(const SDL_Event& event) {
        if (event.type == SDL_CONTROLLERDEVICEADDED) {
            openController(event.cdevice.which);
        }
        else if (event.type == SDL_CONTROLLERDEVICEREMOVED) {
            closeController(event.cdevice.which);
        }
    }

    // TODO: Revisar si dejar esto o no
    void SDLInputBackend::processJoystickEvent(const SDL_Event& event) {
        // Handle non-game-controller joysticks
        // For now, we'll skip these as most modern controllers are game controllers
    }

    void SDLInputBackend::processTouchEvent(const SDL_Event& event) {
        InputEvent inputEvent;

        switch (event.type) {
        case SDL_FINGERDOWN:
            inputEvent.type = InputEventType::TOUCH_BEGAN;
            break;
        case SDL_FINGERUP:
            inputEvent.type = InputEventType::TOUCH_ENDED;
            break;
        case SDL_FINGERMOTION:
            inputEvent.type = InputEventType::TOUCH_MOVED;
            break;
        default:
            return;
        }

        inputEvent.deviceId = INVALID_DEVICE_ID;
        inputEvent.timestamp = InputTimestamp::clock::now();

        TouchEventData data;
        data.touchId = static_cast<std::uint32_t>(event.tfinger.fingerId);
        data.position = math::Vec2(event.tfinger.x * windowWidth_,
                                   event.tfinger.y * windowHeight_);
        data.pressure = event.tfinger.pressure;

        if (event.type == SDL_FINGERDOWN) {
            data.phase = TouchPhase::BEGAN;
            data.startPosition = data.position;
        }
        else if (event.type == SDL_FINGERUP) {
            data.phase = TouchPhase::ENDED;
        }
        else {
            data.phase = TouchPhase::MOVED;
            data.delta = math::Vec2(event.tfinger.dx * windowWidth_,
                                    event.tfinger.dy * windowHeight_);
        }

        inputEvent.data = data;

        queueEvent(std::move(inputEvent));
    }

    void SDLInputBackend::processWindowEvent(const SDL_Event& event) {
        switch (event.window.event) {
        case SDL_WINDOWEVENT_FOCUS_GAINED: {
            InputEvent inputEvent;
            inputEvent.type = InputEventType::FOCUS_GAINED;
            inputEvent.deviceId = INVALID_DEVICE_ID;
            inputEvent.timestamp = InputTimestamp::clock::now();
            queueEvent(std::move(inputEvent));
        }
        break;

        case SDL_WINDOWEVENT_FOCUS_LOST: {
            InputEvent inputEvent;
            inputEvent.type = InputEventType::FOCUS_LOST;
            inputEvent.deviceId = INVALID_DEVICE_ID;
            inputEvent.timestamp = InputTimestamp::clock::now();
            queueEvent(std::move(inputEvent));
        }
        break;

        case SDL_WINDOWEVENT_SIZE_CHANGED:
            windowWidth_ = event.window.data1;
            windowHeight_ = event.window.data2;
            if (mouseLock_) {
                mouseLock_->updateWindowSize(windowWidth_, windowHeight_);
            }
            break;

        case SDL_WINDOWEVENT_ENTER:
            mouseInWindow_ = true;
            break;

        case SDL_WINDOWEVENT_LEAVE:
            mouseInWindow_ = false;
            break;

        // TODO: Revisar esto
        default:
            break;
        }
    }

    void SDLInputBackend::processTextInputEvent(const SDL_Event& event) {
        if (event.type == SDL_TEXTINPUT) {
            InputEvent inputEvent;
            inputEvent.type = InputEventType::TEXT_INPUT;
            inputEvent.deviceId = INVALID_DEVICE_ID;
            inputEvent.timestamp = InputTimestamp::clock::now();

            TextEventData data;
            std::strncpy(data.text, event.text.text, sizeof(data.text) - 1);
            data.text[sizeof(data.text) - 1] = '\0';

            // Get first codepoint
            if (const char* text = event.text.text; *text) {
                // Simple UTF-8 to codepoint conversion (handles ASCII for now)
                data.codepoint = static_cast<std::uint32_t>(*text);
            }

            inputEvent.data = data;

            queueEvent(std::move(inputEvent));
        }
    }

    // ============================================================================
    // Helper Methods Implementation
    // ============================================================================

    void SDLInputBackend::queueEvent(InputEvent event) {
        // Fire callback if registered
        if (eventCallback_) {
            eventCallback_(event);
        }

        // Queue event
        std::lock_guard lock(eventMutex_);
        eventQueue_.push(std::move(event));
    }

    InputEvent* SDLInputBackend::allocateEvent() const {
        if (memoryPools_) {
            return memoryPools_->allocateEvent();
        }
        return new InputEvent();
    }

    void SDLInputBackend::deallocateEvent(InputEvent* event) const {
        if (memoryPools_) {
            memoryPools_->deallocateEvent(event);
        }
        else {
            delete event;
        }
    }

    void SDLInputBackend::openController(const int deviceIndex) {
        SDL_GameController* controller = SDL_GameControllerOpen(deviceIndex);
        if (!controller) {
            return;
        }

        SDL_Joystick* joystick = SDL_GameControllerGetJoystick(controller);
        const int instanceId = SDL_JoystickInstanceID(joystick);

        // Check if already open
        if (controllers_.contains(instanceId)) {
            SDL_GameControllerClose(controller);
            return;
        }

        // Create controller info
        ControllerInfo info;
        info.controller = controller;
        info.joystick = joystick;
        info.instanceId = instanceId;
        info.name = SDL_GameControllerName(controller);
        info.isConnected = true;

        // Try to open haptic
        if (SDL_JoystickIsHaptic(joystick)) {
            info.haptic = SDL_HapticOpenFromJoystick(joystick);
            if (info.haptic && SDL_HapticRumbleSupported(info.haptic)) {
                SDL_HapticRumbleInit(info.haptic);
            }
        }

        // Generate device ID
        if (deviceService_) {
            DeviceDiscoveryInfo discoveryInfo;
            discoveryInfo.type = DeviceType::GAMEPAD;
            discoveryInfo.name = info.name;
            discoveryInfo.platformHandle = controller;

            info.deviceId = deviceService_->onDeviceConnected(discoveryInfo);
        }
        else {
            info.deviceId = static_cast<DeviceID>(instanceId);
        }

        // Store controller
        {
            std::lock_guard lock(controllerMutex_);
            controllers_[instanceId] = info;
            deviceToInstance_[info.deviceId] = instanceId;
        }

        // Fire device callback
        if (deviceCallback_) {
            deviceCallback_(info.deviceId, true);
        }

        // Generate connection event
        InputEvent event;
        event.type = InputEventType::GAMEPAD_CONNECTED;
        event.deviceId = info.deviceId;
        event.timestamp = InputTimestamp::clock::now();

        DeviceEventData data;
        data.type = DeviceType::GAMEPAD;
        data.deviceId = info.deviceId;
        data.connected = true;
        event.data = data;

        queueEvent(std::move(event));
    }

    void SDLInputBackend::closeController(const int instanceId) {
        std::lock_guard lock(controllerMutex_);

        const auto it = controllers_.find(instanceId);
        if (it == controllers_.end()) {
            return;
        }

        const ControllerInfo& info = it->second;
        const DeviceID deviceId = info.deviceId;

        // Generate disconnection event
        {
            InputEvent event;
            event.type = InputEventType::GAMEPAD_DISCONNECTED;
            event.deviceId = deviceId;
            event.timestamp = InputTimestamp::clock::now();

            DeviceEventData data;
            data.type = DeviceType::GAMEPAD;
            data.deviceId = deviceId;
            data.connected = false;
            event.data = data;

            queueEvent(std::move(event));
        }

        // Close haptic
        if (info.haptic) {
            SDL_HapticClose(info.haptic);
        }

        // Close controller
        if (info.controller) {
            SDL_GameControllerClose(info.controller);
        }

        // Notify device service
        if (deviceService_) {
            deviceService_->onDeviceDisconnected(deviceId);
        }

        // Fire device callback
        if (deviceCallback_) {
            deviceCallback_(deviceId, false);
        }

        // Remove from maps
        deviceToInstance_.erase(deviceId);
        controllers_.erase(it);
    }

    SDLInputBackend::ControllerInfo* SDLInputBackend::findController(const int instanceId) {
        const auto it = controllers_.find(instanceId);
        return (it != controllers_.end()) ? &it->second : nullptr;
    }

    const SDLInputBackend::ControllerInfo* SDLInputBackend::findController(const int instanceId) const {
        const auto it = controllers_.find(instanceId);
        return (it != controllers_.end()) ? &it->second : nullptr;
    }

    math::Vec2 SDLInputBackend::normalizeMousePosition(const int x, const int y) const noexcept {
        return math::Vec2(
            static_cast<float>(x) / static_cast<float>(windowWidth_),
            static_cast<float>(y) / static_cast<float>(windowHeight_)
        );
    }

    KeyModifier SDLInputBackend::getSDLModifiers() noexcept {
        return SDLKeyMap::sdlToEngineModifiers(SDL_GetModState());
    }

    void SDLInputBackend::updateModifiers(const SDL_Event& event) {
        currentModifiers_ = SDLKeyMap::sdlToEngineModifiers(event.key.keysym.mod);
    }

    bool SDLInputBackend::loadControllerMappings(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            // Try to load from SDL's internal database
            return SDL_GameControllerAddMappingsFromFile(filepath.c_str()) >= 0;
        }
        file.close();

        return SDL_GameControllerAddMappingsFromFile(filepath.c_str()) >= 0;
    }

    void SDLInputBackend::updateWindowSize() {
        if (window_) {
            SDL_GetWindowSize(window_, &windowWidth_, &windowHeight_);
        }
    }
} // namespace engine::input
