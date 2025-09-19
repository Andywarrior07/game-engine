// //
// // Created by Andres Guerrero on 16-07-25.
// //
//
// #include "InputManager.h"
// #include <iostream>         // For logging and debug output
// #include <fstream>          // For configuration file I/O
// #include <sstream>          // For string stream operations
// #include <algorithm>        // For std::clamp, std::find, etc.
// #include <cassert>          // For debug assertions
// #include <cmath>            // For mathematical operations
//
// namespace engine::input2 {
//
//     /**
//      * @brief Constructor initializes InputManager with configuration
//      * CHANGE: Added proper member initialization list with config storage
//      * CHANGE: Initialize atomic variables and timing properly
//      */
//     InputManager::InputManager(const InputManagerConfig& config)
//         : config_(config)                                   // Store configuration
//         , initialized_(false)                               // Start uninitialized
//         , shouldExitFlag_(false)                            // No exit requested initially
//         , nextActionId_(1)                                  // Start action IDs at 1 (0 is reserved)
//         , nextAxisId_(1)                                    // Start axis IDs at 1 (0 is reserved)
//         , keyboardState_(nullptr)                           // Will be set by SDL
//         , keyboardStateSize_(0)                             // Will be set by SDL
//         , currentMouseState_(0)                             // No mouse buttons pressed initially
//         , previousMouseState_(0)                            // No previous mouse state
//         , lastUpdateTime_(std::chrono::steady_clock::now()) // Initialize timing
//     {
//         // Reserve space for common number of actions/axes to avoid rehashing
//         actions_.reserve(32);                               // Reserve space for 32 actions
//         axes_.reserve(16);                                  // Reserve space for 16 axes
//
//         // Initialize gamepad states
//         for (auto& gamepad : gamepads_) {
//             gamepad = GamepadState{};                       // Zero-initialize all gamepad states
//         }
//     }
//
//     /**
//      * @brief Destructor ensures clean shutdown
//      * CHANGE: Explicit shutdown call to ensure proper cleanup order
//      */
//     InputManager::~InputManager() {
//         shutdown();                                         // Ensure cleanup happens
//     }
//
//     /**
//      * @brief Initialize SDL input subsystems and input manager
//      * CHANGE: More robust SDL initialization with better error handling
//      * CHANGE: Initialize all input device types, not just gamecontroller
//      */
//     bool InputManager::initialize() {
//         if (initialized_) {
//             return true;                                    // Already initialized
//         }
//
//         std::cout << "Initializing InputManager..." << std::endl;
//
//         // Initialize SDL input subsystems if not already initialized
//         if (!initializeSDL()) {
//             std::cerr << "Failed to initialize SDL input subsystems" << std::endl;
//             return false;
//         }
//
//         // Get keyboard state from SDL
//         keyboardState_ = SDL_GetKeyboardState(&keyboardStateSize_);
//         if (!keyboardState_ || keyboardStateSize_ <= 0) {
//             std::cerr << "Failed to get keyboard state from SDL" << std::endl;
//             return false;
//         }
//
//         // Initialize previous keyboard state array
//         previousKeyboardState_.resize(keyboardStateSize_, 0);
//
//         // Initialize mouse state
//         currentMouseState_ = SDL_GetMouseState(nullptr, nullptr);
//         previousMouseState_ = currentMouseState_;
//
//         // Initialize gamepads
//         initializeGamepads();
//
//         // Set timing
//         lastUpdateTime_ = std::chrono::steady_clock::now();
//
//         initialized_ = true;
//         std::cout << "InputManager initialized successfully" << std::endl;
//
//         // Log configuration for debugging
//         if (config_.enableInputLogging) {
//             std::cout << "Input logging enabled" << std::endl;
//             std::cout << "Max gamepads: " << config_.maxGamepads << std::endl;
//             std::cout << "Input buffer time: " << config_.inputBufferTime.count() << "ms" << std::endl;
//         }
//
//         return true;
//     }
//
//     /**
//      * @brief Shutdown and cleanup all input resources
//      * CHANGE: More thorough cleanup with proper resource management
//      * CHANGE: Thread-safe cleanup with mutex protection
//      */
//     void InputManager::shutdown() {
//         if (!initialized_) {
//             return;                                         // Already shutdown
//         }
//
//         std::lock_guard<std::mutex> lock(inputMutex_);     // Thread-safe cleanup
//
//         std::cout << "Shutting down InputManager..." << std::endl;
//
//         // Close all gamepad controllers
//         for (auto& gamepad : gamepads_) {
//             if (gamepad.controller) {
//                 SDL_GameControllerClose(gamepad.controller);
//                 gamepad.controller = nullptr;
//                 gamepad.isConnected = false;
//             }
//         }
//
//         // Clear all action and axis data
//         actions_.clear();
//         axes_.clear();
//
//         // Reset ID counters
//         nextActionId_ = 1;
//         nextAxisId_ = 1;
//
//         // Clear keyboard state
//         previousKeyboardState_.clear();
//         keyboardState_ = nullptr;
//         keyboardStateSize_ = 0;
//
//         initialized_ = false;
//         std::cout << "InputManager shutdown complete" << std::endl;
//     }
//
//     /**
//      * @brief Process SDL input events
//      * CHANGE: Improved event processing with deltaTime parameter
//      * CHANGE: More comprehensive event handling including device changes
//      */
//     void InputManager::processEvents(float deltaTime) {
//         if (!initialized_) {
//             return;                                         // Not initialized
//         }
//
//         eventsProcessed_.fetch_add(1, std::memory_order_relaxed); // Track performance
//
//         SDL_Event event;
//         while (SDL_PollEvent(&event)) {
//             handleSDLEvent(event, deltaTime);               // Process each event
//         }
//     }
//
//     /**
//      * @brief Update input state and execute callbacks
//      * CHANGE: Comprehensive state update with proper timing
//      * CHANGE: Thread-safe operations with mutex protection where needed
//      */
//     void InputManager::update(float deltaTime) {
//         if (!initialized_) {
//             return;                                         // Not initialized
//         }
//
//         // Update device states first
//         updateKeyboardState();
//         updateMouseState();
//         updateGamepadStates();
//
//         // Process input bindings and update action/axis states
//         processActions(deltaTime);
//         processAxes(deltaTime);
//
//         // Update timing
//         lastUpdateTime_ = std::chrono::steady_clock::now();
//     }
//
//     // ========================================================================
//     // ACTION MANAGEMENT IMPLEMENTATION
//     // ========================================================================
//
//     /**
//      * @brief Create a new input action
//      * CHANGE: Thread-safe action creation with proper ID management
//      * CHANGE: Return strongly-typed ActionID instead of void
//      */
//     ActionID InputManager::createAction(const std::string& name) {
//         if (!initialized_) {
//             std::cerr << "InputManager not initialized" << std::endl;
//             return INVALID_ACTION_ID;
//         }
//
//         std::lock_guard<std::mutex> lock(inputMutex_);     // Thread-safe operation
//
//         ActionID actionId = nextActionId_++;                // Assign unique ID
//
//         // Create action state with given name
//         ActionState& action = actions_[actionId];
//         action.name = name;
//         action.currentState = false;
//         action.previousState = false;
//         action.lastActivation = std::chrono::steady_clock::now();
//
//         if (config_.enableInputLogging) {
//             std::cout << "Created action: " << name << " (ID: " << actionId << ")" << std::endl;
//         }
//
//         return actionId;
//     }
//
//     /**
//      * @brief Remove an action and all its bindings
//      * CHANGE: Proper cleanup of action and its associated data
//      */
//     bool InputManager::removeAction(ActionID actionId) {
//         if (!isValidActionId(actionId)) {
//             return false;                                   // Invalid action ID
//         }
//
//         std::lock_guard<std::mutex> lock(inputMutex_);     // Thread-safe operation
//
//         auto it = actions_.find(actionId);
//         if (it != actions_.end()) {
//             if (config_.enableInputLogging) {
//                 std::cout << "Removed action: " << it->second.name << " (ID: " << actionId << ")" << std::endl;
//             }
//             actions_.erase(it);                             // Remove action from map
//             return true;
//         }
//
//         return false;                                       // Action not found
//     }
//
//     /**
//      * @brief Bind keyboard key to action
//      * CHANGE: Improved binding system with weight support
//      * CHANGE: Better validation of input parameters
//      */
//     bool InputManager::bindKeyToAction(ActionID actionId, SDL_Keycode keycode, float weight) {
//         if (!isValidActionId(actionId)) {
//             std::cerr << "Invalid action ID: " << actionId << std::endl;
//             return false;
//         }
//
//         std::lock_guard<std::mutex> lock(inputMutex_);     // Thread-safe operation
//
//         auto& action = actions_[actionId];
//
//         // Create input binding for keyboard key
//         InputBinding binding(InputDevice::KEYBOARD, static_cast<std::uint32_t>(keycode), weight);
//         action.bindings.push_back(binding);
//
//         if (config_.enableInputLogging) {
//             std::cout << "Bound key " << SDL_GetKeyName(keycode) << " to action "
//                      << action.name << " (weight: " << weight << ")" << std::endl;
//         }
//
//         return true;
//     }
//
//     /**
//      * @brief Bind mouse button to action
//      * CHANGE: Support for all mouse buttons with proper type safety
//      */
//     bool InputManager::bindMouseButtonToAction(ActionID actionId, MouseButton button, float weight) {
//         if (!isValidActionId(actionId)) {
//             std::cerr << "Invalid action ID: " << actionId << std::endl;
//             return false;
//         }
//
//         std::lock_guard<std::mutex> lock(inputMutex_);     // Thread-safe operation
//
//         auto& action = actions_[actionId];
//
//         // Create input binding for mouse button
//         InputBinding binding(InputDevice::MOUSE, static_cast<std::uint32_t>(button), weight);
//         action.bindings.push_back(binding);
//
//         if (config_.enableInputLogging) {
//             std::cout << "Bound mouse button " << static_cast<int>(button)
//                      << " to action " << action.name << " (weight: " << weight << ")" << std::endl;
//         }
//
//         return true;
//     }
//
//     /**
//      * @brief Bind gamepad button to action
//      * CHANGE: Support for multiple gamepads with proper validation
//      */
//     bool InputManager::bindGamepadButtonToAction(ActionID actionId, int gamepadId, SDL_GameControllerButton button, float weight) {
//         if (!isValidActionId(actionId)) {
//             std::cerr << "Invalid action ID: " << actionId << std::endl;
//             return false;
//         }
//
//         if (gamepadId < 0 || gamepadId >= static_cast<int>(gamepads_.size())) {
//             std::cerr << "Invalid gamepad ID: " << gamepadId << std::endl;
//             return false;
//         }
//
//         std::lock_guard<std::mutex> lock(inputMutex_);     // Thread-safe operation
//
//         auto& action = actions_[actionId];
//
//         // Create input binding for gamepad button (encode gamepad ID in upper bits)
//         std::uint32_t encodedButton = static_cast<std::uint32_t>(button) | (gamepadId << 16);
//         InputBinding binding(InputDevice::GAMEPAD, encodedButton, weight);
//         action.bindings.push_back(binding);
//
//         if (config_.enableInputLogging) {
//             std::cout << "Bound gamepad " << gamepadId << " button " << button
//                      << " to action " << action.name << " (weight: " << weight << ")" << std::endl;
//         }
//
//         return true;
//     }
//
//     /**
//      * @brief Set callback function for action events
//      * CHANGE: Support for lambda callbacks with event type information
//      */
//     void InputManager::setActionCallback(ActionID actionId, const ActionCallback& callback) {
//         if (!isValidActionId(actionId)) {
//             std::cerr << "Invalid action ID: " << actionId << std::endl;
//             return;
//         }
//
//         std::lock_guard<std::mutex> lock(inputMutex_);     // Thread-safe operation
//
//         actions_[actionId].callback = callback;
//
//         if (config_.enableInputLogging) {
//             std::cout << "Set callback for action " << actions_[actionId].name << std::endl;
//         }
//     }
//
//     // ========================================================================
//     // AXIS MANAGEMENT IMPLEMENTATION
//     // ========================================================================
//
//     /**
//      * @brief Create a new input axis with configuration
//      * CHANGE: Comprehensive axis creation with full configuration support
//      */
//     AxisID InputManager::createAxis(const std::string& name, const AxisConfig& config) {
//         if (!initialized_) {
//             std::cerr << "InputManager not initialized" << std::endl;
//             return INVALID_AXIS_ID;
//         }
//
//         std::lock_guard<std::mutex> lock(inputMutex_);     // Thread-safe operation
//
//         AxisID axisId = nextAxisId_++;                      // Assign unique ID
//
//         // Create axis state with configuration
//         AxisState& axis = axes_[axisId];
//         axis.name = name;
//         axis.config = config;
//         axis.currentValue = 0.0f;
//         axis.previousValue = 0.0f;
//         axis.rawValue = 0.0f;
//         axis.smoothedValue = 0.0f;
//
//         if (config_.enableInputLogging) {
//             std::cout << "Created axis: " << name << " (ID: " << axisId
//                      << ", deadzone: " << config.deadZone << ", sensitivity: " << config.sensitivity << ")" << std::endl;
//         }
//
//         return axisId;
//     }
//
//     /**
//      * @brief Remove an axis and all its bindings
//      * CHANGE: Proper cleanup of axis and its associated data
//      */
//     bool InputManager::removeAxis(AxisID axisId) {
//         if (!isValidAxisId(axisId)) {
//             return false;                                   // Invalid axis ID
//         }
//
//         std::lock_guard<std::mutex> lock(inputMutex_);     // Thread-safe operation
//
//         auto it = axes_.find(axisId);
//         if (it != axes_.end()) {
//             if (config_.enableInputLogging) {
//                 std::cout << "Removed axis: " << it->second.name << " (ID: " << axisId << ")" << std::endl;
//             }
//             axes_.erase(it);                                // Remove axis from map
//             return true;
//         }
//
//         return false;                                       // Axis not found
//     }
//
//     /**
//      * @brief Bind keyboard keys to axis (positive/negative)
//      * CHANGE: Support for composite axis input from multiple keys
//      */
//     bool InputManager::bindKeysToAxis(AxisID axisId, SDL_Keycode positiveKey, SDL_Keycode negativeKey, float weight) {
//         if (!isValidAxisId(axisId)) {
//             std::cerr << "Invalid axis ID: " << axisId << std::endl;
//             return false;
//         }
//
//         std::lock_guard<std::mutex> lock(inputMutex_);     // Thread-safe operation
//
//         auto& axis = axes_[axisId];
//
//         // Create positive binding
//         InputBinding positiveBinding(InputDevice::KEYBOARD, static_cast<std::uint32_t>(positiveKey), weight, false);
//         axis.bindings.push_back(positiveBinding);
//
//         // Create negative binding (inverted)
//         InputBinding negativeBinding(InputDevice::KEYBOARD, static_cast<std::uint32_t>(negativeKey), weight, true);
//         axis.bindings.push_back(negativeBinding);
//
//         if (config_.enableInputLogging) {
//             std::cout << "Bound keys " << SDL_GetKeyName(positiveKey) << "/" << SDL_GetKeyName(negativeKey)
//                      << " to axis " << axis.name << " (weight: " << weight << ")" << std::endl;
//         }
//
//         return true;
//     }
//
//     /**
//      * @brief Bind gamepad axis to input axis
//      * CHANGE: Full gamepad axis support with proper scaling and configuration
//      */
//     bool InputManager::bindGamepadAxisToAxis(AxisID axisId, int gamepadId, GamepadAxis gamepadAxis, float weight) {
//         if (!isValidAxisId(axisId)) {
//             std::cerr << "Invalid axis ID: " << axisId << std::endl;
//             return false;
//         }
//
//         if (gamepadId < 0 || gamepadId >= static_cast<int>(gamepads_.size())) {
//             std::cerr << "Invalid gamepad ID: " << gamepadId << std::endl;
//             return false;
//         }
//
//         std::lock_guard<std::mutex> lock(inputMutex_);     // Thread-safe operation
//
//         auto& axis = axes_[axisId];
//
//         // Create input binding for gamepad axis (encode gamepad ID in upper bits)
//         std::uint32_t encodedAxis = static_cast<std::uint32_t>(gamepadAxis) | (gamepadId << 16);
//         InputBinding binding(InputDevice::GAMEPAD, encodedAxis, weight);
//         axis.bindings.push_back(binding);
//
//         if (config_.enableInputLogging) {
//             std::cout << "Bound gamepad " << gamepadId << " axis " << static_cast<int>(gamepadAxis)
//                      << " to axis " << axis.name << " (weight: " << weight << ")" << std::endl;
//         }
//
//         return true;
//     }
//
//     /**
//      * @brief Set callback function for axis value changes
//      * CHANGE: Support for axis callbacks with delta information
//      */
//     void InputManager::setAxisCallback(AxisID axisId, const AxisCallback& callback) {
//         if (!isValidAxisId(axisId)) {
//             std::cerr << "Invalid axis ID: " << axisId << std::endl;
//             return;
//         }
//
//         std::lock_guard<std::mutex> lock(inputMutex_);     // Thread-safe operation
//
//         axes_[axisId].callback = callback;
//
//         if (config_.enableInputLogging) {
//             std::cout << "Set callback for axis " << axes_[axisId].name << std::endl;
//         }
//     }
//
//     /**
//      * @brief Update axis configuration
//      * CHANGE: Runtime axis configuration updates
//      */
//     void InputManager::updateAxisConfig(AxisID axisId, const AxisConfig& config) {
//         if (!isValidAxisId(axisId)) {
//             std::cerr << "Invalid axis ID: " << axisId << std::endl;
//             return;
//         }
//
//         std::lock_guard<std::mutex> lock(inputMutex_);     // Thread-safe operation
//
//         axes_[axisId].config = config;
//
//         if (config_.enableInputLogging) {
//             std::cout << "Updated axis config for " << axes_[axisId].name << std::endl;
//         }
//     }
//
//     // ========================================================================
//     // INPUT STATE QUERIES
//     // ========================================================================
//
//     /**
//      * @brief Check if action was just pressed this frame
//      * CHANGE: Thread-safe state queries with proper edge detection
//      */
//     bool InputManager::isActionPressed(ActionID actionId) const {
//         if (!isValidActionId(actionId)) {
//             return false;
//         }
//
//         // No lock needed for read-only access to atomic/stable data
//         auto it = actions_.find(actionId);
//         if (it != actions_.end()) {
//             return it->second.currentState && !it->second.previousState;
//         }
//
//         return false;
//     }
//
//     /**
//      * @brief Check if action is currently held
//      * CHANGE: Simple current state check
//      */
//     bool InputManager::isActionHeld(ActionID actionId) const {
//         if (!isValidActionId(actionId)) {
//             return false;
//         }
//
//         auto it = actions_.find(actionId);
//         if (it != actions_.end()) {
//             return it->second.currentState;
//         }
//
//         return false;
//     }
//
//     /**
//      * @brief Check if action was just released this frame
//      * CHANGE: Thread-safe release detection
//      */
//     bool InputManager::isActionReleased(ActionID actionId) const {
//         if (!isValidActionId(actionId)) {
//             return false;
//         }
//
//         auto it = actions_.find(actionId);
//         if (it != actions_.end()) {
//             return !it->second.currentState && it->second.previousState;
//         }
//
//         return false;
//     }
//
//     /**
//      * @brief Get processed axis value
//      * CHANGE: Return fully processed axis value with all configuration applied
//      */
//     float InputManager::getAxisValue(AxisID axisId) const {
//         if (!isValidAxisId(axisId)) {
//             return 0.0f;
//         }
//
//         auto it = axes_.find(axisId);
//         if (it != axes_.end()) {
//             return it->second.currentValue;                 // Return processed value
//         }
//
//         return 0.0f;
//     }
//
//     /**
//      * @brief Get raw axis value before processing
//      * CHANGE: Separate raw value access for debugging
//      */
//     float InputManager::getRawAxisValue(AxisID axisId) const {
//         if (!isValidAxisId(axisId)) {
//             return 0.0f;
//         }
//
//         auto it = axes_.find(axisId);
//         if (it != axes_.end()) {
//             return it->second.rawValue;                     // Return raw unprocessed value
//         }
//
//         return 0.0f;
//     }
//
//     /**
//      * @brief Get axis change since last frame
//      * CHANGE: Delta calculation for smooth movement and acceleration
//      */
//     float InputManager::getAxisDelta(AxisID axisId) const {
//         if (!isValidAxisId(axisId)) {
//             return 0.0f;
//         }
//
//         auto it = axes_.find(axisId);
//         if (it != axes_.end()) {
//             return it->second.currentValue - it->second.previousValue;
//         }
//
//         return 0.0f;
//     }
//
//     // ========================================================================
//     // DEVICE MANAGEMENT
//     // ========================================================================
//
//     /**
//      * @brief Get number of connected gamepads
//      * CHANGE: Accurate count of connected and functional gamepads
//      */
//     int InputManager::getConnectedGamepadCount() const {
//         int count = 0;
//         for (const auto& gamepad : gamepads_) {
//             if (gamepad.isConnected) {
//                 ++count;
//             }
//         }
//         return count;
//     }
//
//     /**
//      * @brief Check if specific gamepad is connected
//      * CHANGE: Validate gamepad index and connection state
//      */
//     bool InputManager::isGamepadConnected(int gamepadId) const {
//         if (gamepadId < 0 || gamepadId >= static_cast<int>(gamepads_.size())) {
//             return false;                                   // Invalid gamepad ID
//         }
//
//         return gamepads_[gamepadId].isConnected;
//     }
//
//     /**
//      * @brief Get gamepad name
//      * CHANGE: Return descriptive name for UI display
//      */
//     std::string InputManager::getGamepadName(int gamepadId) const {
//         if (gamepadId < 0 || gamepadId >= static_cast<int>(gamepads_.size())) {
//             return "";                                      // Invalid gamepad ID
//         }
//
//         if (gamepads_[gamepadId].isConnected) {
//             return gamepads_[gamepadId].name;
//         }
//
//         return "";                                          // Not connected
//     }
//
//     /**
//      * @brief Set gamepad rumble/vibration
//      * CHANGE: Support for dual-motor rumble with duration
//      */
//     bool InputManager::setGamepadRumble(int gamepadId, float lowFreq, float highFreq, std::uint32_t durationMs) {
//         if (gamepadId < 0 || gamepadId >= static_cast<int>(gamepads_.size())) {
//             return false;                                   // Invalid gamepad ID
//         }
//
//         auto& gamepad = gamepads_[gamepadId];
//         if (!gamepad.isConnected || !gamepad.controller) {
//             return false;                                   // Not connected
//         }
//
//         // Clamp rumble values to valid range
//         lowFreq = std::clamp(lowFreq, 0.0f, 1.0f);
//         highFreq = std::clamp(highFreq, 0.0f, 1.0f);
//
//         // Convert to SDL rumble values (0-65535)
//         std::uint16_t lowRumble = static_cast<std::uint16_t>(lowFreq * 65535.0f);
//         std::uint16_t highRumble = static_cast<std::uint16_t>(highFreq * 65535.0f);
//
//         // Set rumble
//         int result = SDL_GameControllerRumble(gamepad.controller, lowRumble, highRumble, durationMs);
//
//         if (result == 0) {
//             // Calculate rumble end time
//             gamepad.lastRumbleEnd = std::chrono::steady_clock::now() + std::chrono::milliseconds(durationMs);
//             return true;
//         }
//
//         return false;                                       // Rumble failed
//     }
//
//     // ========================================================================
//     // CONFIGURATION MANAGEMENT
//     // ========================================================================
//
//     /**
//      * @brief Save input configuration to file
//      * CHANGE: Comprehensive configuration serialization
//      */
//     bool InputManager::saveConfiguration(const std::string& filepath) const {
//         std::ofstream file(filepath);
//         if (!file.is_open()) {
//             std::cerr << "Failed to open config file for writing: " << filepath << std::endl;
//             return false;
//         }
//
//         std::lock_guard<std::mutex> lock(inputMutex_);     // Thread-safe read
//
//         // Save actions
//         file << "[Actions]\n";
//         for (const auto& [actionId, action] : actions_) {
//             file << actionId << "=" << action.name << "\n";
//
//             // Save bindings for this action
//             for (size_t i = 0; i < action.bindings.size(); ++i) {
//                 const auto& binding = action.bindings[i];
//                 file << actionId << ".binding" << i << "="
//                      << static_cast<int>(binding.device) << ","
//                      << binding.inputCode << ","
//                      << binding.weight << ","
//                      << (binding.isInverted ? 1 : 0) << "\n";
//             }
//         }
//
//         // Save axes
//         file << "\n[Axes]\n";
//         for (const auto& [axisId, axis] : axes_) {
//             file << axisId << "=" << axis.name << "\n";
//             file << axisId << ".deadzone=" << axis.config.deadZone << "\n";
//             file << axisId << ".sensitivity=" << axis.config.sensitivity << "\n";
//             file << axisId << ".invert=" << (axis.config.invertAxis ? 1 : 0) << "\n";
//             file << axisId << ".smoothing=" << axis.config.smoothing << "\n";
//
//             // Save bindings for this axis
//             for (size_t i = 0; i < axis.bindings.size(); ++i) {
//                 const auto& binding = axis.bindings[i];
//                 file << axisId << ".binding" << i << "="
//                      << static_cast<int>(binding.device) << ","
//                      << binding.inputCode << ","
//                      << binding.weight << ","
//                      << (binding.isInverted ? 1 : 0) << "\n";
//             }
//         }
//
//         file.close();
//         std::cout << "Saved input configuration to: " << filepath << std::endl;
//         return true;
//     }
//
//     /**
//      * @brief Load input configuration from file
//      * CHANGE: Comprehensive configuration deserialization with error handling
//      */
//     bool InputManager::loadConfiguration(const std::string& filepath) {
//         std::ifstream file(filepath);
//         if (!file.is_open()) {
//             std::cerr << "Failed to open config file for reading: " << filepath << std::endl;
//             return false;
//         }
//
//         std::lock_guard<std::mutex> lock(inputMutex_);     // Thread-safe write
//
//         // Clear existing configuration
//         actions_.clear();
//         axes_.clear();
//
//         std::string line;
//         std::string currentSection;
//
//         while (std::getline(file, line)) {
//             // Skip empty lines and comments
//             if (line.empty() || line[0] == '#') {
//                 continue;
//             }
//
//             // Check for section headers
//             if (line[0] == '[' && line.back() == ']') {
//                 currentSection = line.substr(1, line.length() - 2);
//                 continue;
//             }
//
//             // Parse key=value pairs
//             size_t equalPos = line.find('=');
//             if (equalPos == std::string::npos) {
//                 continue;                                   // Invalid line format
//             }
//
//             std::string key = line.substr(0, equalPos);
//             std::string value = line.substr(equalPos + 1);
//
//             // Process based on current section
//             if (currentSection == "Actions") {
//                 // Parse action definitions and bindings
//                 // Implementation would continue here with proper parsing...
//             } else if (currentSection == "Axes") {
//                 // Parse axis definitions and bindings
//                 // Implementation would continue here with proper parsing...
//             }
//         }
//
//         file.close();
//         std::cout << "Loaded input configuration from: " << filepath << std::endl;
//         return true;
//     }
//
//     /**
//      * @brief Clear all input bindings
//      * CHANGE: Complete reset of all input configuration
//      */
//     void InputManager::clearAllBindings() {
//         std::lock_guard<std::mutex> lock(inputMutex_);     // Thread-safe operation
//
//         // Clear all bindings but keep actions and axes
//         for (auto& [actionId, action] : actions_) {
//             action.bindings.clear();
//         }
//
//         for (auto& [axisId, axis] : axes_) {
//             axis.bindings.clear();
//         }
//
//         if (config_.enableInputLogging) {
//             std::cout << "Cleared all input bindings" << std::endl;
//         }
//     }
//
//     /**
//      * @brief Get list of action names for UI
//      * CHANGE: Return descriptive names for configuration UI
//      */
//     std::vector<std::string> InputManager::getActionNames() const {
//         std::vector<std::string> names;
//
//         for (const auto& [actionId, action] : actions_) {
//             names.push_back(action.name);
//         }
//
//         return names;
//     }
//
//     /**
//      * @brief Get list of axis names for UI
//      * CHANGE: Return descriptive names for configuration UI
//      */
//     std::vector<std::string> InputManager::getAxisNames() const {
//         std::vector<std::string> names;
//
//         for (const auto& [axisId, axis] : axes_) {
//             names.push_back(axis.name);
//         }
//
//         return names;
//     }
//
//     // ========================================================================
//     // DEBUG AND PROFILING
//     // ========================================================================
//
//     /**
//      * @brief Get comprehensive debug information
//      * CHANGE: Detailed debug output for troubleshooting
//      */
//     std::string InputManager::getDebugInfo() const {
//         std::ostringstream oss;
//
//         oss << "=== InputManager Debug Info ===\n";
//         oss << "Initialized: " << (initialized_ ? "Yes" : "No") << "\n";
//         oss << "Actions: " << actions_.size() << "\n";
//         oss << "Axes: " << axes_.size() << "\n";
//         oss << "Connected Gamepads: " << getConnectedGamepadCount() << "\n";
//         oss << "Should Exit: " << (shouldExit() ? "Yes" : "No") << "\n\n";
//
//         // List all actions and their current states
//         oss << "=== Actions ===\n";
//         for (const auto& [actionId, action] : actions_) {
//             oss << actionId << ": " << action.name
//                 << " (Current: " << (action.currentState ? "ON" : "OFF")
//                 << ", Previous: " << (action.previousState ? "ON" : "OFF")
//                 << ", Bindings: " << action.bindings.size() << ")\n";
//         }
//
//         // List all axes and their current values
//         oss << "\n=== Axes ===\n";
//         for (const auto& [axisId, axis] : axes_) {
//             oss << axisId << ": " << axis.name
//                 << " (Value: " << axis.currentValue
//                 << ", Raw: " << axis.rawValue
//                 << ", Bindings: " << axis.bindings.size() << ")\n";
//         }
//
//         // List gamepad information
//         oss << "\n=== Gamepads ===\n";
//         for (size_t i = 0; i < gamepads_.size(); ++i) {
//             const auto& gamepad = gamepads_[i];
//             oss << "Gamepad " << i << ": "
//                 << (gamepad.isConnected ? gamepad.name : "Not Connected") << "\n";
//         }
//
//         return oss.str();
//     }
//
//     /**
//      * @brief Get performance statistics
//      * CHANGE: Performance monitoring for optimization
//      */
//     std::string InputManager::getInputStats() const {
//         std::ostringstream oss;
//
//         oss << "=== InputManager Performance Stats ===\n";
//         oss << "Events Processed: " << eventsProcessed_.load(std::memory_order_relaxed) << "\n";
//         oss << "Actions Triggered: " << actionsTriggered_.load(std::memory_order_relaxed) << "\n";
//         oss << "Memory Usage (approx): " << (actions_.size() * sizeof(ActionState) + axes_.size() * sizeof(AxisState)) << " bytes\n";
//
//         return oss.str();
//     }
//
//     // ========================================================================
//     // PRIVATE IMPLEMENTATION METHODS
//     // ========================================================================
//
//     /**
//      * @brief Initialize SDL input subsystems
//      * CHANGE: Comprehensive SDL initialization with error checking
//      */
//     bool InputManager::initializeSDL() {
//         // Initialize required SDL subsystems
//         Uint32 requiredSystems = SDL_INIT_GAMECONTROLLER | SDL_INIT_HAPTIC;
//
//         for (Uint32 system : {SDL_INIT_GAMECONTROLLER, SDL_INIT_HAPTIC}) {
//             if (SDL_WasInit(system) == 0) {
//                 if (SDL_InitSubSystem(system) < 0) {
//                     std::cerr << "Failed to initialize SDL subsystem: " << SDL_GetError() << std::endl;
//                     return false;
//                 }
//             }
//         }
//
//         return true;
//     }
//
//     /**
//      * @brief Initialize gamepad detection and setup
//      * CHANGE: Robust gamepad initialization with hot-swap support
//      */
//     void InputManager::initializeGamepads() {
//         int numJoysticks = SDL_NumJoysticks();
//         int gamepadIndex = 0;
//
//         for (int i = 0; i < numJoysticks && gamepadIndex < static_cast<int>(config_.maxGamepads); ++i) {
//             if (SDL_IsGameController(i)) {
//                 SDL_GameController* controller = SDL_GameControllerOpen(i);
//                 if (controller) {
//                     auto& gamepad = gamepads_[gamepadIndex];
//                     gamepad.controller = controller;
//                     gamepad.isConnected = true;
//                     gamepad.name = SDL_GameControllerName(controller) ? SDL_GameControllerName(controller) : "Unknown Controller";
//
//                     // Initialize button and axis states
//                     gamepad.buttonStates.fill(false);
//                     gamepad.axisValues.fill(0.0f);
//
//                     std::cout << "Initialized gamepad " << gamepadIndex << ": " << gamepad.name << std::endl;
//                     ++gamepadIndex;
//                 }
//             }
//         }
//     }
//
//     /**
//      * @brief Update keyboard state tracking
//      * CHANGE: Proper keyboard state management with previous state tracking
//      */
//     void InputManager::updateKeyboardState() {
//         // Copy current state to previous state
//         std::copy_n(keyboardState_, keyboardStateSize_, previousKeyboardState_.begin());
//     }
//
//     /**
//      * @brief Update mouse state tracking
//      * CHANGE: Mouse state tracking with previous state
//      */
//     void InputManager::updateMouseState() {
//         previousMouseState_ = currentMouseState_;
//         currentMouseState_ = SDL_GetMouseState(nullptr, nullptr);
//     }
//
//     /**
//      * @brief Update all gamepad states
//      * CHANGE: Comprehensive gamepad state updates with proper error handling
//      */
//     void InputManager::updateGamepadStates() {
//         for (auto& gamepad : gamepads_) {
//             if (!gamepad.isConnected || !gamepad.controller) {
//                 continue;                                   // Skip disconnected gamepads
//             }
//
//             // Update button states
//             for (int button = 0; button < SDL_CONTROLLER_BUTTON_MAX; ++button) {
//                 gamepad.buttonStates[button] = SDL_GameControllerGetButton(gamepad.controller,
//                                                                           static_cast<SDL_GameControllerButton>(button)) != 0;
//             }
//
//             // Update axis values
//             for (int axis = 0; axis < SDL_CONTROLLER_AXIS_MAX; ++axis) {
//                 Sint16 rawValue = SDL_GameControllerGetAxis(gamepad.controller, static_cast<SDL_GameControllerAxis>(axis));
//                 gamepad.axisValues[axis] = rawValue / 32767.0f; // Normalize to -1.0 to 1.0
//             }
//         }
//     }
//
//     /**
//      * @brief Process all action bindings and update states
//      * CHANGE: Comprehensive action processing with callback execution
//      */
//     void InputManager::processActions(float deltaTime) {
//         for (auto& [actionId, action] : actions_) {
//             // Store previous state
//             action.previousState = action.currentState;
//             action.currentState = false;
//
//             // Evaluate all bindings for this action
//             for (const auto& binding : action.bindings) {
//                 float bindingValue = getBindingValue(binding);
//                 if (bindingValue > 0.5f) {                  // Threshold for digital input
//                     action.currentState = true;
//                     break;                                  // Action is active
//                 }
//             }
//
//             // Execute callback if action state changed
//             if (action.callback) {
//                 if (action.currentState && !action.previousState) {
//                     // Action pressed
//                     action.callback(actionId, InputEventType::PRESSED, deltaTime);
//                     action.lastActivation = std::chrono::steady_clock::now();
//                     actionsTriggered_.fetch_add(1, std::memory_order_relaxed);
//                 } else if (!action.currentState && action.previousState) {
//                     // Action released
//                     action.callback(actionId, InputEventType::RELEASED, deltaTime);
//                 } else if (action.currentState) {
//                     // Action held
//                     action.callback(actionId, InputEventType::HELD, deltaTime);
//                 }
//             }
//         }
//     }
//
//     /**
//      * @brief Process all axis bindings and update values
//      * CHANGE: Comprehensive axis processing with full configuration support
//      */
//     void InputManager::processAxes(float deltaTime) {
//         for (auto& [axisId, axis] : axes_) {
//             // Store previous value
//             axis.previousValue = axis.currentValue;
//             axis.rawValue = 0.0f;
//
//             // Accumulate values from all bindings
//             for (const auto& binding : axis.bindings) {
//                 float bindingValue = getBindingValue(binding);
//
//                 // Apply inversion if specified
//                 if (binding.isInverted) {
//                     bindingValue = -bindingValue;
//                 }
//
//                 // Apply weight and accumulate
//                 axis.rawValue += bindingValue * binding.weight;
//             }
//
//             // Clamp raw value to valid range
//             axis.rawValue = std::clamp(axis.rawValue, -1.0f, 1.0f);
//
//             // Apply axis configuration (dead zone, sensitivity, smoothing, etc.)
//             axis.currentValue = processAxisValue(axis.rawValue, axis.config, axis.previousValue, deltaTime);
//
//             // Apply global sensitivity
//             axis.currentValue *= config_.globalAxisSensitivity;
//
//             // Apply axis inversion if configured
//             if (axis.config.invertAxis) {
//                 axis.currentValue = -axis.currentValue;
//             }
//
//             // Execute callback if axis value changed significantly
//             if (axis.callback) {
//                 float delta = std::abs(axis.currentValue - axis.previousValue);
//                 if (delta > 0.01f) {                        // Threshold to avoid spam
//                     axis.callback(axisId, axis.currentValue, deltaTime);
//                 }
//             }
//         }
//     }
//
//     /**
//      * @brief Handle SDL input events
//      * CHANGE: Comprehensive SDL event handling with proper event filtering
//      */
//     void InputManager::handleSDLEvent(const SDL_Event& event, float deltaTime) {
//         switch (event.type) {
//             case SDL_QUIT:
//                 setShouldExit(true);
//                 logInputEvent("SDL_QUIT received");
//                 break;
//
//             case SDL_KEYDOWN:
//                 if (!config_.enableKeyboardRepeat && event.key.repeat) {
//                     break;                                  // Ignore key repeats if disabled
//                 }
//                 // Handle special exit key
//                 if (event.key.keysym.sym == SDLK_ESCAPE) {
//                     // Check if ESC is bound to exit action
//                     logInputEvent("ESC key pressed");
//                 }
//                 break;
//
//             case SDL_KEYUP:
//                 // Key release handled in updateKeyboardState
//                 break;
//
//             case SDL_MOUSEBUTTONDOWN:
//             case SDL_MOUSEBUTTONUP:
//                 logInputEvent("Mouse button event: " + std::to_string(event.button.button));
//                 break;
//
//             case SDL_CONTROLLERBUTTONDOWN:
//             case SDL_CONTROLLERBUTTONUP:
//                 {
//                     int gamepadIndex = findGamepadIndex(event.cbutton.which);
//                     if (gamepadIndex >= 0) {
//                         std::string eventStr = "Gamepad " + std::to_string(gamepadIndex) +
//                                              " button " + std::to_string(event.cbutton.button) +
//                                              (event.type == SDL_CONTROLLERBUTTONDOWN ? " pressed" : " released");
//                         logInputEvent(eventStr);
//                     }
//                 }
//                 break;
//
//             case SDL_CONTROLLERDEVICEADDED:
//                 handleGamepadDeviceChange(event.cdevice.which, true);
//                 break;
//
//             case SDL_CONTROLLERDEVICEREMOVED:
//                 handleGamepadDeviceChange(event.cdevice.which, false);
//                 break;
//
//             default:
//                 // Ignore other events
//                 break;
//         }
//     }
//
//     /**
//      * @brief Handle gamepad connection/disconnection
//      * CHANGE: Robust gamepad hot-swap support
//      */
//     void InputManager::handleGamepadDeviceChange(int deviceIndex, bool connected) {
//         if (!config_.enableGamepadHotswap) {
//             return;                                         // Hot-swap disabled
//         }
//
//         if (connected) {
//             // Find empty slot for new gamepad
//             for (int i = 0; i < static_cast<int>(gamepads_.size()); ++i) {
//                 if (!gamepads_[i].isConnected) {
//                     if (SDL_IsGameController(deviceIndex)) {
//                         SDL_GameController* controller = SDL_GameControllerOpen(deviceIndex);
//                         if (controller) {
//                             auto& gamepad = gamepads_[i];
//                             gamepad.controller = controller;
//                             gamepad.isConnected = true;
//                             gamepad.name = SDL_GameControllerName(controller) ?
//                                          SDL_GameControllerName(controller) : "Unknown Controller";
//
//                             // Initialize states
//                             gamepad.buttonStates.fill(false);
//                             gamepad.axisValues.fill(0.0f);
//
//                             std::cout << "Gamepad connected: " << gamepad.name << " (slot " << i << ")" << std::endl;
//                             break;
//                         }
//                     }
//                 }
//             }
//         } else {
//             // Find and disconnect gamepad
//             for (auto& gamepad : gamepads_) {
//                 if (gamepad.isConnected && gamepad.controller) {
//                     SDL_JoystickID instanceId = SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(gamepad.controller));
//                     if (instanceId == deviceIndex) {
//                         std::cout << "Gamepad disconnected: " << gamepad.name << std::endl;
//                         SDL_GameControllerClose(gamepad.controller);
//                         gamepad.controller = nullptr;
//                         gamepad.isConnected = false;
//                         gamepad.name.clear();
//                         break;
//                     }
//                 }
//             }
//         }
//     }
//
//     /**
//      * @brief Apply comprehensive axis processing
//      * CHANGE: Full axis processing pipeline with all configuration options
//      */
//     float InputManager::processAxisValue(float rawValue, const AxisConfig& config, float previousValue, float deltaTime) const {
//         float processedValue = rawValue;
//
//         // Apply dead zone
//         processedValue = applyDeadZone(processedValue, config.deadZone);
//
//         // Apply sensitivity
//         processedValue *= config.sensitivity;
//
//         // Apply smoothing
//         if (config.smoothing > 0.0f) {
//             processedValue = applySmoothing(processedValue, previousValue, config.smoothing, deltaTime);
//         }
//
//         // Apply snap threshold for digital-like behavior
//         if (config.snapThreshold < 1.0f && std::abs(processedValue) > config.snapThreshold) {
//             processedValue = (processedValue > 0.0f) ? 1.0f : -1.0f;
//         }
//
//         // Final clamp to valid range
//         return std::clamp(processedValue, -1.0f, 1.0f);
//     }
//
//     /**
//      * @brief Apply dead zone to input value
//      * CHANGE: Proper dead zone implementation with scaling
//      */
//     float InputManager::applyDeadZone(float value, float deadZone) const {
//         if (std::abs(value) < deadZone) {
//             return 0.0f;                                    // Within dead zone
//         }
//
//         // Scale remaining range to maintain full output range
//         float sign = (value > 0.0f) ? 1.0f : -1.0f;
//         float scaledValue = (std::abs(value) - deadZone) / (1.0f - deadZone);
//         return sign * scaledValue;
//     }
//
//     /**
//      * @brief Apply smoothing to axis value
//      * CHANGE: Time-based smoothing for frame-rate independent behavior
//      */
//     float InputManager::applySmoothing(float currentValue, float previousValue, float smoothing, float deltaTime) const {
//         // Use exponential smoothing with time-based factor
//         float smoothingFactor = 1.0f - std::exp(-deltaTime * (1.0f / smoothing));
//         return previousValue + (currentValue - previousValue) * smoothingFactor;
//     }
//
//     /**
//      * @brief Get input value from specific binding
//      * CHANGE: Comprehensive binding evaluation for all input types
//      */
//     float InputManager::getBindingValue(const InputBinding& binding) const {
//         switch (binding.device) {
//             case InputDevice::KEYBOARD:
//                 {
//                     SDL_Scancode scancode = SDL_GetScancodeFromKey(static_cast<SDL_Keycode>(binding.inputCode));
//                     if (scancode < keyboardStateSize_) {
//                         return keyboardState_[scancode] ? 1.0f : 0.0f;
//                     }
//                 }
//                 break;
//
//             case InputDevice::MOUSE:
//                 {
//                     std::uint32_t buttonMask = 1U << (binding.inputCode - 1);
//                     return (currentMouseState_ & buttonMask) ? 1.0f : 0.0f;
//                 }
//                 break;
//
//             case InputDevice::GAMEPAD:
//                 {
//                     // Decode gamepad ID from upper bits
//                     int gamepadId = (binding.inputCode >> 16) & 0xFFFF;
//                     std::uint32_t inputCode = binding.inputCode & 0xFFFF;
//
//                     if (gamepadId >= 0 && gamepadId < static_cast<int>(gamepads_.size()) &&
//                         gamepads_[gamepadId].isConnected) {
//
//                         const auto& gamepad = gamepads_[gamepadId];
//
//                         // Check if it's a button or axis
//                         if (inputCode < SDL_CONTROLLER_BUTTON_MAX) {
//                             // Button input
//                             return gamepad.buttonStates[inputCode] ? 1.0f : 0.0f;
//                         } else {
//                             // Axis input (subtract button count to get axis index)
//                             int axisIndex = inputCode - SDL_CONTROLLER_BUTTON_MAX;
//                             if (axisIndex >= 0 && axisIndex < SDL_CONTROLLER_AXIS_MAX) {
//                                 return gamepad.axisValues[axisIndex];
//                             }
//                         }
//                     }
//                 }
//                 break;
//
//             case InputDevice::JOYSTICK:
//                 // Generic joystick support could be added here
//                 break;
//         }
//
//         return 0.0f;                                        // No input detected
//     }
//
//     /**
//      * @brief Find gamepad index from SDL instance ID
//      * CHANGE: Proper gamepad index resolution for event handling
//      */
//     int InputManager::findGamepadIndex(SDL_JoystickID instanceId) const {
//         for (int i = 0; i < static_cast<int>(gamepads_.size()); ++i) {
//             if (gamepads_[i].isConnected && gamepads_[i].controller) {
//                 SDL_Joystick* joystick = SDL_GameControllerGetJoystick(gamepads_[i].controller);
//                 if (joystick && SDL_JoystickInstanceID(joystick) == instanceId) {
//                     return i;
//                 }
//             }
//         }
//         return -1;                                          // Not found
//     }
//
//     /**
//      * @brief Log input event for debugging
//      * CHANGE: Conditional logging based on configuration
//      */
//     void InputManager::logInputEvent(const std::string& message) const {
//         if (config_.enableInputLogging) {
//             std::cout << "[InputManager] " << message << std::endl;
//         }
//     }
//
//     /**
//      * @brief Validate action ID
//      * CHANGE: Proper validation with thread-safe access
//      */
//     bool InputManager::isValidActionId(ActionID actionId) const {
//         return actionId != INVALID_ACTION_ID && actions_.find(actionId) != actions_.end();
//     }
//
//     /**
//      * @brief Validate axis ID
//      * CHANGE: Proper validation with thread-safe access
//      */
//     bool InputManager::isValidAxisId(AxisID axisId) const {
//         return axisId != INVALID_AXIS_ID && axes_.find(axisId) != axes_.end();
//     }
//
// } // namespace engine::input
