/**
 * @file PlayerController.cpp
 * @brief Simple player controller implementation
 * @author Andrés Guerrero
 * @date 16-09-2025
 */

#include "PlayerController.h"

#include "./input/InputSystem.h"

namespace game {
    PlayerController::PlayerController(engine::input::InputSystem* inputSystem)
        : inputSystem_(inputSystem)
          , position_(0.0f, 0.0f, 0.0f)
          , velocity_(0.0f, 0.0f, 0.0f)
          , isOnGround_(true)
          , moveSpeed_(5.0f) // 5 units per second
          , jumpForce_(8.0f) // Jump velocity
          , gravity_(-15.0f) // Gravity acceleration
          , groundLevel_(0.0f) // Ground Y position
    {
    }

    bool PlayerController::initialize() {
        if (!inputSystem_ || !inputSystem_->isInitialized()) {
            return false;
        }

        // Set up input bindings for WASD movement and Space jump
        setupInputBindings();

        // Reset position to ground
        position_ = engine::math::Vec3(0.0f, groundLevel_, 0.0f);
        velocity_ = engine::math::Vec3(0.0f, 0.0f, 0.0f);
        isOnGround_ = true;

        return true;
    }

    void PlayerController::setupInputBindings() {
        // Register movement actions
        inputSystem_->registerAction(MOVE_FORWARD, "MoveForward", engine::input::ActionType::BUTTON);
        inputSystem_->registerAction(MOVE_BACKWARD, "MoveBackward", engine::input::ActionType::BUTTON);
        inputSystem_->registerAction(MOVE_LEFT, "MoveLeft", engine::input::ActionType::BUTTON);
        inputSystem_->registerAction(MOVE_RIGHT, "MoveRight", engine::input::ActionType::BUTTON);
        inputSystem_->registerAction(JUMP, "Jump", engine::input::ActionType::BUTTON);

        // Bind keys to actions using the default context
        inputSystem_->bindKey(MOVE_FORWARD, engine::input::KeyCode::W, "Default");
        inputSystem_->bindKey(MOVE_BACKWARD, engine::input::KeyCode::S, "Default");
        inputSystem_->bindKey(MOVE_LEFT, engine::input::KeyCode::A, "Default");
        inputSystem_->bindKey(MOVE_RIGHT, engine::input::KeyCode::D, "Default");
        inputSystem_->bindKey(JUMP, engine::input::KeyCode::SPACE, "Default");

        // Optional: Set up action callbacks for immediate response
        inputSystem_->registerActionCallback(JUMP,
                                             [this](engine::input::ActionID actionId,
                                                    const engine::input::ActionState& state) {
                                                 // Handle jump action when triggered
                                                 if (state.isTriggered() && isOnGround_) {
                                                     velocity_.y = jumpForce_;
                                                     isOnGround_ = false;
                                                 }
                                             });
    }

    void PlayerController::update(const float deltaTime) {
        if (!inputSystem_) {
            return;
        }

        // Process input and update movement
        processMovement(deltaTime);
        processJump();

        // Apply physics
        applyPhysics(deltaTime);
    }

    void PlayerController::processMovement(const float deltaTime) {
        // Get movement input from action system
        engine::math::Vec3 moveDirection(0.0f, 0.0f, 0.0f);

        // Check each movement action
        if (inputSystem_->isActionTriggered(MOVE_FORWARD)) {
            moveDirection.z += 1.0f; // Forward is positive Z
        }
        if (inputSystem_->isActionTriggered(MOVE_BACKWARD)) {
            moveDirection.z -= 1.0f; // Backward is negative Z
        }
        if (inputSystem_->isActionTriggered(MOVE_LEFT)) {
            moveDirection.x -= 1.0f; // Left is negative X
        }
        if (inputSystem_->isActionTriggered(MOVE_RIGHT)) {
            moveDirection.x += 1.0f; // Right is positive X
        }

        // Normalize diagonal movement to prevent faster diagonal speed
        if (moveDirection.x != 0.0f || moveDirection.z != 0.0f) {
            const float length = std::sqrt(moveDirection.x * moveDirection.x +
                moveDirection.z * moveDirection.z);
            if (length > 0.0f) {
                moveDirection.x /= length;
                moveDirection.z /= length;
            }
        }

        // Apply movement speed and time delta
        velocity_.x = moveDirection.x * moveSpeed_;
        velocity_.z = moveDirection.z * moveSpeed_;

        // Update horizontal position
        position_.x += velocity_.x * deltaTime;
        position_.z += velocity_.z * deltaTime;
    }

    void PlayerController::processJump() {
        // Jump is handled via callback, but we can also check directly
        if (inputSystem_->isActionTriggered(JUMP) && isOnGround_) {
            velocity_.y = jumpForce_;
            isOnGround_ = false;
        }
    }

    void PlayerController::applyPhysics(const float deltaTime) {
        // Apply gravity if not on ground
        if (!isOnGround_) {
            velocity_.y += gravity_ * deltaTime;
        }

        // Update vertical position
        position_.y += velocity_.y * deltaTime;

        // Simple ground collision
        if (position_.y <= groundLevel_) {
            position_.y = groundLevel_;

            // Only set on ground if we were falling
            if (velocity_.y <= 0.0f) {
                velocity_.y = 0.0f;
                isOnGround_ = true;
            }
        }
    }
} // namespace game
