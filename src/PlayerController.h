/**
 * @file PlayerController.h
 * @brief Simple player controller using the InputSystem
 * @author Andrés Guerrero
 * @date 16-09-2025
 */

#pragma once

#include "./input/InputSystem.h"
#include "./math/MathSystem.h"

namespace game {
    /**
     * @brief Simple player controller that handles WASD movement and Space jump
     */
    class PlayerController {
    public:
        /**
         * @brief Constructor
         * @param inputSystem Reference to the engine's input system
         */
        explicit PlayerController(engine::input::InputSystem* inputSystem);

        /**
         * @brief Initialize the controller and set up input bindings
         * @return True if successful
         */
        bool initialize();

        /**
         * @brief Update player movement based on input
         * @param deltaTime Time since last update in seconds
         */
        void update(float deltaTime);

        /**
         * @brief Get current player position
         */
        [[nodiscard]] const engine::math::Vec3& getPosition() const noexcept {
            return position_;
        }

        /**
         * @brief Get current player velocity
         */
        [[nodiscard]] const engine::math::Vec3& getVelocity() const noexcept {
            return velocity_;
        }

        /**
         * @brief Check if player is on ground
         */
        [[nodiscard]] bool isOnGround() const noexcept {
            return isOnGround_;
        }

        /**
         * @brief Set movement speed
         */
        void setMoveSpeed(float speed) noexcept {
            moveSpeed_ = speed;
        }

        /**
         * @brief Set jump force
         */
        void setJumpForce(float force) noexcept {
            jumpForce_ = force;
        }

    private:
        // Input system reference
        engine::input::InputSystem* inputSystem_;

        // Player state
        engine::math::Vec3 position_;
        engine::math::Vec3 velocity_;
        bool isOnGround_;

        // Movement parameters
        float moveSpeed_;
        float jumpForce_;
        float gravity_;
        float groundLevel_;

        // Action IDs for input mapping
        static constexpr engine::input::ActionID MOVE_FORWARD = 1;
        static constexpr engine::input::ActionID MOVE_BACKWARD = 2;
        static constexpr engine::input::ActionID MOVE_LEFT = 3;
        static constexpr engine::input::ActionID MOVE_RIGHT = 4;
        static constexpr engine::input::ActionID JUMP = 5;

        /**
         * @brief Set up input action bindings
         */
        void setupInputBindings();

        /**
         * @brief Process movement input
         * @param deltaTime Time delta for frame-rate independent movement
         */
        void processMovement(float deltaTime);

        /**
         * @brief Process jump input
         */
        void processJump();

        /**
         * @brief Apply gravity and ground collision
         * @param deltaTime Time delta
         */
        void applyPhysics(float deltaTime);
    };
} // namespace game