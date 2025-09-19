// /**
//  * @file CameraInputHandler.h
//  * @brief Camera input handling system using InputManager
//  * @author Andrés Guerrero
//  * @date 26-08-2025
//  */
//
// #pragma once
//
// #include "../core/CameraTypes.h"
// #include "../../Input-old/InputManager.h"
//
// #include <unordered_map>
//
// using namespace engine::input2;
//
// namespace engine::camera {
//
//     // Forward declaration
//     class CameraManager;
//
//     /**
//      * @brief Camera input actions
//      */
//     enum class CameraAction : std::uint8_t {
//         MOVE_FORWARD = 0,
//         MOVE_BACKWARD,
//         MOVE_LEFT,
//         MOVE_RIGHT,
//         MOVE_UP,
//         MOVE_DOWN,
//         ZOOM_IN,
//         ZOOM_OUT,
//         RESET_CAMERA,
//         TOGGLE_MODE,
//         SPEED_UP,
//         SPEED_DOWN,
//         LOOK_LEFT,
//         LOOK_RIGHT
//     };
//
//     /**
//      * @brief Camera input axes
//      */
//     enum class CameraAxis : std::uint8_t {
//         HORIZONTAL = 0,    // Left/Right movement or rotation
//         VERTICAL,          // Forward/Back movement or pitch
//         ELEVATION,         // Up/Down movement
//         YAW,              // Horizontal rotation
//         PITCH,            // Vertical rotation
//         ZOOM              // Zoom in/out
//     };
//
//     /**
//      * @brief Camera input handler using existing InputManager
//      */
//     class CameraInputHandler {
//     public:
//         /**
//          * @brief Constructor
//          * @param cameraManager Camera manager reference
//          * @param inputManager Input manager reference
//          */
//         CameraInputHandler(CameraManager* cameraManager, InputManager* inputManager);
//
//         /**
//          * @brief Setup default camera controls
//          */
//         void setupDefaultBindings();
//
//         /**
//          * @brief Setup keyboard bindings for camera
//          */
//         void setupKeyboardBindings();
//
//         /**
//          * @brief Setup gamepad bindings
//          * @param gamepadId Gamepad index
//          */
//         void setupGamepadBindings(int gamepadId) ;
//
//         /**
//          * @brief Update camera based on input
//          * @param deltaTime Time step
//          */
//         void update(float deltaTime);
//
//         /**
//          * @brief Process mouse input directly (for mouse look)
//          * @param deltaX Mouse X movement
//          * @param deltaY Mouse Y movement
//          */
//         void processMouseDelta(const float deltaX, const float deltaY) {
//             mouseDelta_.x = deltaX;
//             mouseDelta_.y = deltaY;
//         }
//
//         /**
//          * @brief Process scroll input
//          * @param scrollDelta Scroll amount
//          */
//         void processScroll(const float scrollDelta) {
//             scrollDelta_ = scrollDelta;
//         }
//
//         /**
//          * @brief Get movement vector from input
//          * @return Movement vector based on current input
//          */
//         [[nodiscard]] Vec3 getMovementVector() const;
//
//         /**
//          * @brief Check if speed boost is active
//          * @return true if speed up action is held
//          */
//         [[nodiscard]] bool isSpeedBoostActive() const;
//
//         /**
//          * @brief Set mouse sensitivity
//          * @param sensitivity New sensitivity
//          */
//         void setMouseSensitivity(const float sensitivity) {
//             mouseSensitivity_ = sensitivity;
//         }
//
//         /**
//          * @brief Set movement speed
//          * @param speed New speed
//          */
//         void setMoveSpeed(const float speed) {
//             moveSpeed_ = speed;
//         }
//
//     private:
//         CameraManager* cameraManager_;
//         InputManager* inputManager_;
//
//         // Input mappings
//         std::unordered_map<CameraAction, ActionID> actionIds_;
//         std::unordered_map<CameraAxis, AxisID> axisIds_;
//
//         // Settings
//         float mouseSensitivity_ = 1.0f;
//         float moveSpeed_ = 10.0f;
//         float sprintMultiplier_ = 2.0f;
//
//         // Mouse input
//         Vec2 mouseDelta_{0, 0};
//         float scrollDelta_ = 0.0f;
//
//         /**
//          * @brief Setup input callbacks
//          */
//         void setupCallbacks();
//     };
//
// } // namespace engine::camera