/**
* @file CameraInputHandler.cpp
 * @brief Camera input handling system using InputManager
 * @author Andrés Guerrero
 * @date 26-08-2025
 */

#include "CameraInputHandler.h"

#include "../cameras/Camera3D.h"
#include "../manager/CameraManager.h"

namespace engine::camera {
    class Camera3D;
    class BaseCamera;

    CameraInputHandler::CameraInputHandler(CameraManager* cameraManager, InputManager* inputManager)
        : cameraManager_(cameraManager)
          , inputManager_(inputManager) {
        setupDefaultBindings();
    }

    void CameraInputHandler::setupDefaultBindings() {
        if (!inputManager_) return;

        // Create camera actions
        actionIds_[CameraAction::MOVE_FORWARD] = inputManager_->createAction("Camera_MoveForward");
        actionIds_[CameraAction::MOVE_BACKWARD] = inputManager_->createAction("Camera_MoveBackward");
        actionIds_[CameraAction::MOVE_LEFT] = inputManager_->createAction("Camera_MoveLeft");
        actionIds_[CameraAction::MOVE_RIGHT] = inputManager_->createAction("Camera_MoveRight");
        actionIds_[CameraAction::MOVE_UP] = inputManager_->createAction("Camera_MoveUp");
        actionIds_[CameraAction::MOVE_DOWN] = inputManager_->createAction("Camera_MoveDown");
        actionIds_[CameraAction::ZOOM_IN] = inputManager_->createAction("Camera_ZoomIn");
        actionIds_[CameraAction::ZOOM_OUT] = inputManager_->createAction("Camera_ZoomOut");
        actionIds_[CameraAction::RESET_CAMERA] = inputManager_->createAction("Camera_Reset");
        actionIds_[CameraAction::SPEED_UP] = inputManager_->createAction("Camera_SpeedUp");
        actionIds_[CameraAction::LOOK_LEFT] = inputManager_->createAction("Camera_LookLeft");
        actionIds_[CameraAction::LOOK_RIGHT] = inputManager_->createAction("Camera_LookRight");

        // Create camera axes
        AxisConfig axisConfig;
        axisConfig.deadZone = 0.15f;
        axisConfig.sensitivity = 1.0f;

        axisIds_[CameraAxis::HORIZONTAL] = inputManager_->createAxis("Camera_Horizontal", axisConfig);
        axisIds_[CameraAxis::VERTICAL] = inputManager_->createAxis("Camera_Vertical", axisConfig);
        axisIds_[CameraAxis::ELEVATION] = inputManager_->createAxis("Camera_Elevation", axisConfig);
        axisIds_[CameraAxis::YAW] = inputManager_->createAxis("Camera_Yaw", axisConfig);
        axisIds_[CameraAxis::PITCH] = inputManager_->createAxis("Camera_Pitch", axisConfig);
        axisIds_[CameraAxis::ZOOM] = inputManager_->createAxis("Camera_Zoom", axisConfig);

        // Setup default keyboard bindings
        setupKeyboardBindings();

        // Setup callbacks
        setupCallbacks();
    }

    void CameraInputHandler::setupKeyboardBindings() {
        if (!inputManager_) return;

        // Movement - WASD
        inputManager_->bindKeyToAction(actionIds_[CameraAction::MOVE_FORWARD], SDLK_w);
        inputManager_->bindKeyToAction(actionIds_[CameraAction::MOVE_BACKWARD], SDLK_s);
        inputManager_->bindKeyToAction(actionIds_[CameraAction::MOVE_LEFT], SDLK_a);
        inputManager_->bindKeyToAction(actionIds_[CameraAction::MOVE_RIGHT], SDLK_d);

        // Vertical movement
        inputManager_->bindKeyToAction(actionIds_[CameraAction::MOVE_UP], SDLK_SPACE);
        inputManager_->bindKeyToAction(actionIds_[CameraAction::MOVE_DOWN], SDLK_c);

        // Zoom
        inputManager_->bindKeyToAction(actionIds_[CameraAction::ZOOM_IN], SDLK_EQUALS);
        inputManager_->bindKeyToAction(actionIds_[CameraAction::ZOOM_OUT], SDLK_MINUS);

        // Others
        inputManager_->bindKeyToAction(actionIds_[CameraAction::RESET_CAMERA], SDLK_r);
        inputManager_->bindKeyToAction(actionIds_[CameraAction::SPEED_UP], SDLK_LSHIFT);

        // Lean/Look
        inputManager_->bindKeyToAction(actionIds_[CameraAction::LOOK_LEFT], SDLK_q);
        inputManager_->bindKeyToAction(actionIds_[CameraAction::LOOK_RIGHT], SDLK_e);

        // Bind axes for smooth movement
        inputManager_->bindKeysToAxis(axisIds_[CameraAxis::HORIZONTAL], SDLK_d, SDLK_a);
        inputManager_->bindKeysToAxis(axisIds_[CameraAxis::VERTICAL], SDLK_w, SDLK_s);
    }

    void CameraInputHandler::setupGamepadBindings(const int gamepadId = 0) {
        if (!inputManager_) return;

        // Movement with left stick
        inputManager_->bindGamepadAxisToAxis(
            axisIds_[CameraAxis::HORIZONTAL],
            gamepadId,
            GamepadAxis::LEFT_STICK_X
        );

        inputManager_->bindGamepadAxisToAxis(
            axisIds_[CameraAxis::VERTICAL],
            gamepadId,
            GamepadAxis::LEFT_STICK_Y
        );

        // Look with right stick
        inputManager_->bindGamepadAxisToAxis(
            axisIds_[CameraAxis::YAW],
            gamepadId,
            GamepadAxis::RIGHT_STICK_X
        );

        inputManager_->bindGamepadAxisToAxis(
            axisIds_[CameraAxis::PITCH],
            gamepadId,
            GamepadAxis::RIGHT_STICK_Y
        );

        // Zoom with triggers
        inputManager_->bindGamepadAxisToAxis(
            axisIds_[CameraAxis::ZOOM],
            gamepadId,
            GamepadAxis::RIGHT_TRIGGER
        );
    }

    void CameraInputHandler::update(const float deltaTime) {

        if (!cameraManager_ || !inputManager_) return;

        // Get active camera
        BaseCamera* activeCamera = cameraManager_->getActiveCamera();
        if (!activeCamera) return;

        // Get movement input
        Vec3 movement = getMovementVector();

        // Apply speed boost if active
        float currentSpeed = moveSpeed_;
        if (isSpeedBoostActive()) {
            currentSpeed *= sprintMultiplier_;
        }

        // Scale movement by speed and deltaTime
        movement *= currentSpeed * deltaTime;

        // Process movement based on camera type
        cameraManager_->processMovement(movement.x, movement.z, movement.y, deltaTime);

        // Process mouse look if we have mouse delta
        if (mouseDelta_.x != 0.0f || mouseDelta_.y != 0.0f) {
            cameraManager_->processMouseLook(
                mouseDelta_.x * mouseSensitivity_,
                mouseDelta_.y * mouseSensitivity_,
                deltaTime
            );
            // Reset mouse delta after processing
            mouseDelta_ = Vec2(0, 0);
        }

        // Process zoom
        const float zoomAxis = inputManager_->getAxisValue(axisIds_[CameraAxis::ZOOM]);
        if (scrollDelta_ != 0.0f) {
            cameraManager_->processZoom(scrollDelta_);
            scrollDelta_ = 0.0f;
        } else if (std::abs(zoomAxis) > 0.01f) {
            cameraManager_->processZoom(zoomAxis * deltaTime * 5.0f);
        }

        // Handle camera reset
        if (inputManager_->isActionPressed(actionIds_[CameraAction::RESET_CAMERA])) {
            if (activeCamera) {
                activeCamera->reset();
            }
        }
    }

    Vec3 CameraInputHandler::getMovementVector() const  {
        if (!inputManager_) return Vec3(0, 0, 0);

        return Vec3(
            inputManager_->getAxisValue(axisIds_.at(CameraAxis::HORIZONTAL)),
            inputManager_->getAxisValue(axisIds_.at(CameraAxis::ELEVATION)),
            -inputManager_->getAxisValue(axisIds_.at(CameraAxis::VERTICAL))  // Negative for forward
        );
    }

    bool CameraInputHandler::isSpeedBoostActive() const {
        if (!inputManager_) return false;

        return inputManager_->isActionHeld(actionIds_.at(CameraAction::SPEED_UP));
    }

    void CameraInputHandler::setupCallbacks() {
        if (!inputManager_) return;

    // Setup zoom callbacks for discrete zoom actions
    inputManager_->setActionCallback(
        actionIds_[CameraAction::ZOOM_IN],
        [this](ActionID id, InputEventType type, float dt) {
            if (type == InputEventType::PRESSED ||
                type == InputEventType::HELD) {
                if (cameraManager_) {
                    cameraManager_->processZoom(0.1f);
                }
            }
        }
    );

    inputManager_->setActionCallback(
        actionIds_[CameraAction::ZOOM_OUT],
        [this](ActionID id, InputEventType type, float dt) {
            if (type == InputEventType::PRESSED ||
                type == InputEventType::HELD) {
                if (cameraManager_) {
                    cameraManager_->processZoom(-0.1f);
                }
            }
        }
    );

    // Setup look callbacks for discrete look actions (useful for lean mechanics)
    inputManager_->setActionCallback(
        actionIds_[CameraAction::LOOK_LEFT],
        [this](ActionID id, InputEventType type, float dt) {
            if (type == InputEventType::HELD && cameraManager_) {
                if (const auto cam3D = dynamic_cast<Camera3D*>(cameraManager_->getActiveCamera())) {
                    // Apply roll for lean effect
                    cam3D->setRoll(cam3D->getRoll() + 15.0f * dt);
                }
            }
        }
    );

    inputManager_->setActionCallback(
        actionIds_[CameraAction::LOOK_RIGHT],
        [this](ActionID id, const InputEventType type, const float dt) {
            if (type == InputEventType::HELD && cameraManager_) {
                if (const auto cam3D = dynamic_cast<Camera3D*>(cameraManager_->getActiveCamera())) {
                    // Apply roll for lean effect
                    cam3D->setRoll(cam3D->getRoll() - 15.0f * dt);
                }
            }
        }
    );
    }
} // namespace engine::camera
