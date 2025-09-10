/**
 * @file CharacterController.h
 * @brief Advanced character controller for RPG/Action games
 * @details Implements capsule-based character movement with climbing,
 *          swimming, dodging, and complex movement mechanics
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

#include "../core/PhysicsTypes.h"

#include <BulletDynamics/Character/btKinematicCharacterController.h>

// TODO: In the future, move it to the module manager, as it is a preset
namespace engine::physics {
    /**
     * @brief Character controller parameters
     */
    struct CharacterControllerParams {
        // Shape
        Float radius = 0.4f; // Capsule radius
        Float height = 1.8f; // Character height
        Float crouchHeight = 0.9f; // Height when crouching
        Float stepHeight = 0.35f; // Maximum step height

        // Movement
        Float walkSpeed = 4.0f; // m/s
        Float runSpeed = 8.0f; // m/s
        Float sprintSpeed = 12.0f; // m/s
        Float crouchSpeed = 2.0f; // m/s
        Float swimSpeed = 3.0f; // m/s
        Float climbSpeed = 2.0f; // m/s

        // Jumping
        Float jumpHeight = 2.0f; // Maximum jump height
        Float jumpForwardSpeed = 5.0f; // Forward speed when jumping
        Float airControl = 0.3f; // Movement control in air (0-1)
        Int maxJumps = 2; // For double/triple jump
        Float coyoteTime = 0.15f; // Grace period for jumping after leaving ground

        // Physics
        Float mass = 70.0f; // kg
        Float pushPower = 2.0f; // Push force multiplier
        Float gravity = -20.0f; // Custom gravity for responsive movement
        Float terminalVelocity = -50.0f; // Maximum fall speed
        Float groundFriction = 8.0f; // Ground deceleration
        Float airFriction = 0.5f; // Air resistance

        // Slopes
        Float maxSlopeAngle = 45.0f; // Maximum walkable slope (degrees)
        Float slideThreshold = 35.0f; // Start sliding on slopes (degrees)
        Float slideSpeed = 5.0f; // Sliding speed multiplier

        // Combat/Dodging (Souls-like)
        Float dodgeDistance = 3.0f; // Dodge roll distance
        Float dodgeSpeed = 15.0f; // Dodge roll speed
        Float dodgeDuration = 0.3f; // Dodge animation time
        Float dodgeInvincibility = 0.2f; // I-frames duration

        // Collision
        std::uint16_t collisionGroup = CollisionGroup::CHARACTER;
        std::uint16_t collisionMask = CollisionGroup::ALL & ~CollisionGroup::CHARACTER;

        // Initial state
        Transform* transform = nullptr;
        Vec3 startPosition = Vec3(0);

        CharacterControllerParams() = default;
    };

    /**
     * @brief Character movement state
     */
    enum class CharacterState : std::uint8_t {
        IDLE,
        WALKING,
        RUNNING,
        SPRINTING,
        JUMPING,
        FALLING,
        LANDING,
        CROUCHING,
        CRAWLING,
        CLIMBING,
        SWIMMING,
        DODGING,
        SLIDING,
        WALL_RUNNING,
        LEDGE_HANGING,
        DISABLED
    };

    /**
     * @brief Advanced character controller
     * @details Implements complex character movement mechanics for
     *          action RPGs, platformers, and Souls-like games
     */
    class CharacterController {
    public:
        CharacterController(const CharacterControllerParams& params, PhysicsWorld* world)
            : params_(params), world_(world), state_(CharacterState::IDLE),
              velocity_(0), isGrounded_(false), jumpCount_(0),
              timeSinceGrounded_(0), dodgeCooldown_(0) {
            initialize();
        }

        ~CharacterController();

        // ============================================================================
        // Update
        // ============================================================================

        /**
         * @brief Update character controller
         * @param deltaTime Frame time
         * @param inputMove Movement input vector (normalized)
         * @param inputLook Look direction (for orientation)
         */
        void update(Float deltaTime, const Vec3& inputMove, const Vec3& inputLook);

        // ============================================================================
        // Actions
        // ============================================================================

        /**
         * @brief Make character jump
         */
        bool jump();

        /**
         * @brief Perform dodge roll
         */
        bool dodge(const Vec3& direction);

        /**
         * @brief Start/stop crouching
         */
        void setCrouching(bool crouch);

        /**
         * @brief Start climbing
         */
        bool startClimbing(const Vec3& climbSurface);

        /**
         * @brief Start swimming
         */
        void startSwimming() {
            if (state_ != CharacterState::SWIMMING) {
                state_ = CharacterState::SWIMMING;
                velocity_.y *= 0.5f; // Reduce vertical velocity
            }
        }

        // ============================================================================
        // Queries
        // ============================================================================

        Vec3 getPosition() const;

        void setPosition(const Vec3& position) const;

        Vec3 getVelocity() const { return velocity_; }
        CharacterState getState() const { return state_; }
        bool isGrounded() const { return isGrounded_; }
        bool isInvincible() const { return invincibilityTimer_ > 0; }

        Float getSpeed() const;

        /**
         * @brief Check if character can fit through space
         */
        static bool canFitThroughSpace(const Vec3& position, Float width, Float height);

    private:
        // Parameters
        CharacterControllerParams params_;
        PhysicsWorld* world_;

        // Bullet objects
        btPairCachingGhostObject* ghostObject_ = nullptr;
        btKinematicCharacterController* controller_ = nullptr;
        btCapsuleShape* shape_ = nullptr;

        // State
        CharacterState state_;
        Vec3 velocity_;
        bool isGrounded_;
        Vec3 groundNormal_;
        Float groundSlope_;

        // Jump state
        Int jumpCount_;
        Float timeSinceGrounded_;

        // Dodge state
        Vec3 dodgeDirection_;
        Float dodgeTimer_;
        Float dodgeCooldown_;
        Float invincibilityTimer_;

        // Climbing state
        Vec3 climbSurface_;
        Vec3 climbPoint_;

        // Wall running state
        Vec3 wallNormal_;
        Float wallRunTimer_;

        // Ledge state
        Vec3 ledgePosition_;
        Vec3 ledgeNormal_;

        // ============================================================================
        // Initialization
        // ============================================================================

        void initialize();

        void cleanup();

        // ============================================================================
        // Movement Handlers
        // ============================================================================

        Vec3 handleGroundMovement(const Vec3& input, Float deltaTime) const;

        Vec3 handleAirMovement(const Vec3& input, Float deltaTime) const;

        Vec3 handleCrouchMovement(const Vec3& input, Float deltaTime) const {
            return input * params_.crouchSpeed;
        }

        Vec3 handleClimbing(const Vec3& input, Float deltaTime) const;

        Vec3 handleSwimming(const Vec3& input, Float deltaTime) const;

        Vec3 handleDodging(Float deltaTime) const;

        Vec3 handleSliding(const Vec3& input, Float deltaTime) const;

        Vec3 handleWallRunning(const Vec3& input, Float deltaTime) const;

        Vec3 handleLedgeHanging(const Vec3& input, Float deltaTime) const;

        // ============================================================================
        // Physics Updates
        // ============================================================================

        void applyGravity(Float deltaTime);

        void moveCharacter(const Vec3& displacement) const;

        void updateOrientation(const Vec3& lookDir) const;

        void updateGroundStatus();

        void handleCollisions() const;

        // ============================================================================
        // State Machine
        // ============================================================================

        void updateStateMachine(const Vec3& input);

        void updateTimers(Float deltaTime);

        // ============================================================================
        // Queries
        // ============================================================================

        static bool canClimb(const Vec3& surface);

        bool canStandUp() const;

        bool checkWallRun(Vec3& wallNormal);

        bool canWallRun(const Vec3& wallNormal);

        Vec3 getForwardDirection() const;

        Float landingTimer_ = 0;
    };
} // namespace engine::physics
