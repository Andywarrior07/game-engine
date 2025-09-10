/**
* @file CharacterController.h
 * @brief Advanced character controller for RPG/Action games
 * @details Implements capsule-based character movement with climbing,
 *          swimming, dodging, and complex movement mechanics
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#include "CharacterController.h"

#include "../core/PhysicsWorld.h"

#include <BulletCollision/CollisionDispatch/btGhostObject.h>

namespace engine::physics {
    CharacterController::~CharacterController() {
        cleanup();
    }

    void CharacterController::update(Float deltaTime, const Vec3& inputMove, const Vec3& inputLook) {
        if (state_ == CharacterState::DISABLED) return;

        // Update timers
        updateTimers(deltaTime);

        // Check ground status
        updateGroundStatus();

        // Handle state transitions
        updateStateMachine(inputMove);

        // Apply movement based on state
        Vec3 moveVelocity = Vec3(0);

        switch (state_) {
        case CharacterState::IDLE:
        case CharacterState::WALKING:
        case CharacterState::RUNNING:
        case CharacterState::SPRINTING:
            moveVelocity = handleGroundMovement(inputMove, deltaTime);
            break;

        case CharacterState::JUMPING:
        case CharacterState::FALLING:
            moveVelocity = handleAirMovement(inputMove, deltaTime);
            break;

        case CharacterState::CROUCHING:
        case CharacterState::CRAWLING:
            moveVelocity = handleCrouchMovement(inputMove, deltaTime);
            break;

        case CharacterState::CLIMBING:
            moveVelocity = handleClimbing(inputMove, deltaTime);
            break;

        case CharacterState::SWIMMING:
            moveVelocity = handleSwimming(inputMove, deltaTime);
            break;

        case CharacterState::DODGING:
            moveVelocity = handleDodging(deltaTime);
            break;

        case CharacterState::SLIDING:
            moveVelocity = handleSliding(inputMove, deltaTime);
            break;

        case CharacterState::WALL_RUNNING:
            moveVelocity = handleWallRunning(inputMove, deltaTime);
            break;

        case CharacterState::LEDGE_HANGING:
            moveVelocity = handleLedgeHanging(inputMove, deltaTime);
            break;
        }

        // Apply gravity (except when climbing or swimming)
        if (state_ != CharacterState::CLIMBING &&
            state_ != CharacterState::SWIMMING &&
            state_ != CharacterState::LEDGE_HANGING) {
            applyGravity(deltaTime);
        }

        // Add movement velocity to current velocity
        velocity_.x = moveVelocity.x;
        velocity_.z = moveVelocity.z;

        // Clamp to terminal velocity
        if (velocity_.y < params_.terminalVelocity) {
            velocity_.y = params_.terminalVelocity;
        }

        // Move the character
        moveCharacter(velocity_ * deltaTime);

        // Update orientation
        if (glm::length(inputLook) > 0.1f) {
            updateOrientation(inputLook);
        }

        // Handle collisions and push objects
        handleCollisions();
    }

    bool CharacterController::jump() {
        // Check if can jump (on ground or within coyote time or multi-jump)
        const bool canJump = isGrounded_ ||
            timeSinceGrounded_ < params_.coyoteTime ||
            (jumpCount_ > 0 && jumpCount_ < params_.maxJumps);

        if (!canJump) return false;

        // Calculate jump velocity from desired height
        // v = sqrt(2 * g * h)
        const Float jumpVelocity = std::sqrt(-2.0f * params_.gravity * params_.jumpHeight);
        velocity_.y = jumpVelocity;

        state_ = CharacterState::JUMPING;
        jumpCount_++;
        isGrounded_ = false;

        return true;
    }

    bool CharacterController::dodge(const Vec3& direction) {
        if (state_ == CharacterState::DODGING || dodgeCooldown_ > 0) {
            return false;
        }

        state_ = CharacterState::DODGING;
        dodgeDirection_ = glm::normalize(direction);
        dodgeTimer_ = params_.dodgeDuration;
        dodgeCooldown_ = 0.5f; // Half second cooldown
        invincibilityTimer_ = params_.dodgeInvincibility;

        return true;
    }

    void CharacterController::setCrouching(const bool crouch) {
        if (crouch && isGrounded_) {
            if (state_ != CharacterState::CROUCHING) {
                state_ = CharacterState::CROUCHING;
                // Reduce capsule height
                if (ghostObject_) {
                    // Update collision shape
                }
            }
        }
        else {
            if (state_ == CharacterState::CROUCHING ||
                state_ == CharacterState::CRAWLING) {
                // Check if there's room to stand up
                if (canStandUp()) {
                    state_ = CharacterState::IDLE;
                    // Restore capsule height
                }
            }
        }
    }

    bool CharacterController::startClimbing(const Vec3& climbSurface) {
        if (!canClimb(climbSurface)) return false;

        state_ = CharacterState::CLIMBING;
        climbSurface_ = climbSurface;
        velocity_ = Vec3(0); // Stop current velocity

        return true;
    }

    Vec3 CharacterController::getPosition() const {
        if (ghostObject_) {
            const btTransform& transform = ghostObject_->getWorldTransform();
            return Vec3(transform.getOrigin().x(),
                        transform.getOrigin().y(),
                        transform.getOrigin().z());
        }
        return params_.startPosition;
    }

    void CharacterController::setPosition(const Vec3& position) const {
        if (ghostObject_) {
            btTransform transform = ghostObject_->getWorldTransform();
            transform.setOrigin(btVector3(position.x, position.y, position.z));
            ghostObject_->setWorldTransform(transform);
        }
    }

    Float CharacterController::getSpeed() const {
        return glm::length(Vec2(velocity_.x, velocity_.z));
    }

    bool CharacterController::canFitThroughSpace(const Vec3& position, Float width, Float height) {
        // Perform collision check with capsule at position
        return true; // Placeholder
    }

    void CharacterController::initialize() {
        // Create capsule shape
        shape_ = new btCapsuleShape(params_.radius, params_.height);

        // Create ghost object for collision detection
        ghostObject_ = new btPairCachingGhostObject();
        ghostObject_->setCollisionShape(shape_);
        ghostObject_->setCollisionFlags(btCollisionObject::CF_CHARACTER_OBJECT);

        // Set initial transform
        btTransform startTransform;
        startTransform.setIdentity();
        startTransform.setOrigin(btVector3(
            params_.startPosition.x,
            params_.startPosition.y,
            params_.startPosition.z
        ));
        ghostObject_->setWorldTransform(startTransform);

        // Create character controller
        controller_ = new btKinematicCharacterController(
            ghostObject_, shape_, params_.stepHeight
        );

        controller_->setGravity(btVector3(0, params_.gravity, 0));
        controller_->setMaxSlope(btRadians(params_.maxSlopeAngle));
        controller_->setJumpSpeed(params_.jumpHeight);
        controller_->setFallSpeed(std::abs(params_.terminalVelocity));

        // Add to physics world
        if (world_) {
            btDiscreteDynamicsWorld* bulletWorld = world_->getBulletWorld();
            bulletWorld->addCollisionObject(
                ghostObject_,
                params_.collisionGroup,
                params_.collisionMask
            );
            bulletWorld->addAction(controller_);
        }
    }

    void CharacterController::cleanup() {
        if (world_ && world_->getBulletWorld()) {
            btDiscreteDynamicsWorld* bulletWorld = world_->getBulletWorld();

            if (controller_) {
                bulletWorld->removeAction(controller_);
                delete controller_;
                controller_ = nullptr;
            }

            if (ghostObject_) {
                bulletWorld->removeCollisionObject(ghostObject_);
                delete ghostObject_;
                ghostObject_ = nullptr;
            }
        }

        if (shape_) {
            delete shape_;
            shape_ = nullptr;
        }
    }

    Vec3 CharacterController::handleGroundMovement(const Vec3& input, const Float deltaTime) const {
        Float speed = params_.walkSpeed;

        if (state_ == CharacterState::RUNNING) {
            speed = params_.runSpeed;
        }
        else if (state_ == CharacterState::SPRINTING) {
            speed = params_.sprintSpeed;
        }

        Vec3 moveDir = input * speed;

        // Apply ground friction
        if (glm::length(input) < 0.1f) {
            moveDir.x = velocity_.x * (1.0f - params_.groundFriction * deltaTime);
            moveDir.z = velocity_.z * (1.0f - params_.groundFriction * deltaTime);
        }

        // Check slope and adjust movement
        if (groundSlope_ > params_.slideThreshold) {
            // Slide down slope
            Vec3 slideDir = groundNormal_;
            slideDir.y = 0;
            slideDir = glm::normalize(slideDir);
            moveDir += slideDir * params_.slideSpeed;
        }

        return moveDir;
    }

    Vec3 CharacterController::handleAirMovement(const Vec3& input, const Float deltaTime) const {
        // Limited air control
        Vec3 moveDir = input * params_.runSpeed * params_.airControl;

        // Apply air friction
        moveDir.x = velocity_.x * (1.0f - params_.airFriction * deltaTime);
        moveDir.z = velocity_.z * (1.0f - params_.airFriction * deltaTime);

        return moveDir;
    }

    Vec3 CharacterController::handleClimbing(const Vec3& input, Float deltaTime) const {
        // Movement along climb surface
        Vec3 moveDir = input * params_.climbSpeed;

        // Stick to surface
        moveDir += climbSurface_ * 2.0f;

        return moveDir;
    }

    Vec3 CharacterController::handleSwimming(const Vec3& input, Float deltaTime) const {
        Vec3 moveDir = input * params_.swimSpeed;

        // Add vertical movement for swimming
        if (input.y != 0) {
            moveDir.y = input.y * params_.swimSpeed * 0.7f;
        }

        // Water resistance
        moveDir *= 0.8f;

        return moveDir;
    }

    Vec3 CharacterController::handleDodging(Float deltaTime) const {
        const Float dodgeProgress = 1.0f - (dodgeTimer_ / params_.dodgeDuration);

        // Ease-out curve for dodge speed
        const Float speed = params_.dodgeSpeed * (1.0f - dodgeProgress * dodgeProgress);

        return dodgeDirection_ * speed;
    }

    Vec3 CharacterController::handleSliding(const Vec3& input, Float deltaTime) const {
        // Slide down slope with some control
        Vec3 slideDir = groundNormal_;
        slideDir.y = 0;
        slideDir = glm::normalize(slideDir);

        Vec3 moveDir = slideDir * params_.slideSpeed;
        moveDir += input * params_.walkSpeed * 0.3f; // Limited control

        return moveDir;
    }

    Vec3 CharacterController::handleWallRunning(const Vec3& input, const Float deltaTime) const {
        // Run along wall
        Vec3 wallDir = glm::cross(wallNormal_, Vec3(0, 1, 0));
        Vec3 moveDir = wallDir * params_.runSpeed;

        // Gradually fall
        moveDir.y = -2.0f * deltaTime;

        return moveDir;
    }

    Vec3 CharacterController::handleLedgeHanging(const Vec3& input, Float deltaTime) const {
        // Move along ledge
        const Vec3 ledgeDir = glm::cross(ledgeNormal_, Vec3(0, 1, 0));
        return ledgeDir * input.x * params_.climbSpeed * 0.5f;
    }

    void CharacterController::applyGravity(Float deltaTime) {
        if (!isGrounded_) {
            velocity_.y += params_.gravity * deltaTime;
        }
        else {
            velocity_.y = min(velocity_.y, 0.0f);
        }
    }

    void CharacterController::moveCharacter(const Vec3& displacement) const {
        if (!controller_) return;

        controller_->setWalkDirection(btVector3(
            displacement.x, displacement.y, displacement.z
        ));
    }

    void CharacterController::updateOrientation(const Vec3& lookDir) const {
        if (glm::length(lookDir) < 0.1f) return;

        const Vec3 forward = glm::normalize(Vec3(lookDir.x, 0, lookDir.z));
        const Float angle = std::atan2(forward.x, forward.z);

        if (ghostObject_) {
            btTransform transform = ghostObject_->getWorldTransform();
            transform.setRotation(btQuaternion(btVector3(0, 1, 0), angle));
            ghostObject_->setWorldTransform(transform);
        }
    }

    void CharacterController::updateGroundStatus() {
        isGrounded_ = controller_ && controller_->onGround();

        if (isGrounded_) {
            jumpCount_ = 0;
            timeSinceGrounded_ = 0;

            // Get ground normal and slope
            // Perform raycast down to get ground info
            RaycastHit hit;

            if (const Vec3 pos = getPosition(); world_->raycast(pos, pos + Vec3(0, -params_.height, 0), hit)) {
                groundNormal_ = hit.normal;
                groundSlope_ = glm::degrees(std::acos(glm::dot(groundNormal_, Vec3(0, 1, 0))));
            }
        }
    }

    void CharacterController::handleCollisions() const {
        if (!ghostObject_) return;

        // Check for overlapping objects that can be pushed
        btManifoldArray manifoldArray;
        btBroadphasePairArray& pairArray =
            ghostObject_->getOverlappingPairCache()->getOverlappingPairArray();

        for (int i = 0; i < pairArray.size(); ++i) {
            const btBroadphasePair& pair = pairArray[i];

            auto obj0 = static_cast<btCollisionObject*>(pair.m_pProxy0->m_clientObject);
            auto obj1 = static_cast<btCollisionObject*>(pair.m_pProxy1->m_clientObject);

            // Check if it's a dynamic rigid body
            if (btCollisionObject* other = (obj0 == ghostObject_) ? obj1 : obj0; other && !other->isStaticOrKinematicObject()) {
                if (btRigidBody* body = btRigidBody::upcast(other)) {
                    // Apply push force
                    Vec3 pushDir = getPosition();
                    btVector3 otherPos = body->getWorldTransform().getOrigin();
                    pushDir = Vec3(otherPos.x(), otherPos.y(), otherPos.z()) - pushDir;
                    pushDir.y = 0;
                    pushDir = glm::normalize(pushDir);

                    const Float pushForce = params_.pushPower * params_.mass;
                    body->applyCentralImpulse(btVector3(
                        pushDir.x * pushForce,
                        0,
                        pushDir.z * pushForce
                    ));
                }
            }
        }
    }

    void CharacterController::updateStateMachine(const Vec3& input) {
        Float inputMagnitude = glm::length(Vec2(input.x, input.z));

        switch (state_) {
        case CharacterState::IDLE:
            if (inputMagnitude > 0.1f) {
                state_ = CharacterState::WALKING;
            }
            if (!isGrounded_) {
                state_ = CharacterState::FALLING;
            }
            break;

        case CharacterState::WALKING:
            if (inputMagnitude < 0.1f) {
                state_ = CharacterState::IDLE;
            }
            if (!isGrounded_) {
                state_ = CharacterState::FALLING;
            }
            break;

        case CharacterState::JUMPING:
            if (velocity_.y <= 0) {
                state_ = CharacterState::FALLING;
            }
            break;

        case CharacterState::FALLING:
            if (isGrounded_) {
                state_ = CharacterState::LANDING;
                landingTimer_ = 0.2f;
            }
            break;

        case CharacterState::LANDING:
            if (landingTimer_ <= 0) {
                state_ = inputMagnitude > 0.1f ? CharacterState::WALKING : CharacterState::IDLE;
            }
            break;

        case CharacterState::DODGING:
            if (dodgeTimer_ <= 0) {
                state_ = isGrounded_ ? CharacterState::IDLE : CharacterState::FALLING;
            }
            break;

        case CharacterState::SLIDING:
            if (groundSlope_ < params_.slideThreshold || !isGrounded_) {
                state_ = isGrounded_ ? CharacterState::IDLE : CharacterState::FALLING;
            }
            break;
        }
    }

    void CharacterController::updateTimers(const Float deltaTime) {
        if (!isGrounded_) {
            timeSinceGrounded_ += deltaTime;
        }

        if (dodgeTimer_ > 0) {
            dodgeTimer_ -= deltaTime;
        }

        if (dodgeCooldown_ > 0) {
            dodgeCooldown_ -= deltaTime;
        }

        if (invincibilityTimer_ > 0) {
            invincibilityTimer_ -= deltaTime;
        }

        if (landingTimer_ > 0) {
            landingTimer_ -= deltaTime;
        }

        if (wallRunTimer_ > 0) {
            wallRunTimer_ -= deltaTime;
        }
    }

    bool CharacterController::canClimb(const Vec3& surface) {
        // Check if surface is climbable (angle, material, etc.)
        const Float angle = glm::degrees(std::acos(glm::dot(surface, Vec3(0, 1, 0))));
        return angle > 45.0f && angle < 135.0f;
    }

    bool CharacterController::canStandUp() const {
        // Check if there's room above to stand
        const Vec3 pos = getPosition();
        const Vec3 checkPos = pos + Vec3(0, params_.height - params_.crouchHeight, 0);

        // Perform sphere overlap check
        const std::vector<RigidBody*> overlaps = world_->overlapSphere(
            checkPos, params_.radius * 0.9f
        );

        return overlaps.empty();
    }

    bool CharacterController::checkWallRun(Vec3& wallNormal) {
        // Check for wall to the left or right
        const Vec3 pos = getPosition();
        const Vec3 forward = getForwardDirection();
        const Vec3 right = glm::cross(forward, Vec3(0, 1, 0));

        // Check right wall
        RaycastHit hit;
        if (world_->raycast(pos, pos + right * (params_.radius * 2.0f), hit)) {
            wallNormal = hit.normal;
            return canWallRun(wallNormal);
        }

        // Check left wall
        if (world_->raycast(pos, pos - right * (params_.radius * 2.0f), hit)) {
            wallNormal = hit.normal;
            return canWallRun(wallNormal);
        }

        return false;
    }

    bool CharacterController::canWallRun(const Vec3& wallNormal) {
        // Check if wall angle is suitable for wall running
        Float angle = glm::degrees(std::acos(std::abs(glm::dot(wallNormal, Vec3(0, 1, 0)))));
        return angle > 80.0f; // Nearly vertical
    }

    Vec3 CharacterController::getForwardDirection() const {
        if (ghostObject_) {
            const btTransform& transform = ghostObject_->getWorldTransform();
            const btQuaternion rot = transform.getRotation();
            auto forward = btVector3(0, 0, 1);
            forward = quatRotate(rot, forward);

            return Vec3(forward.x(), forward.y(), forward.z());
        }

        return Vec3(0, 0, 1);
    }
} // namespace engine::physics
