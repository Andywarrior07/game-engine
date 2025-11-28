// MAIN TEST - PHYSICS SYNCHRONIZED VERSION
// ⭐ FIXES:
// 1. Player visual position now syncs with RigidBody physics position
// 2. WASD movement applies forces to physics body instead of directly moving visual
// 3. Gravity affects player correctly
// 4. Player starts at left side, high up, and falls to ground

#include <SDL.h>
#include <SDL_image.h>
#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <cmath>

// CHANGE: New input system includes
#include "./input/InputSystem.h"

// Memory and resource management
#include "./memory/MemorySystem.h"
#include "./resources/manager/ResourceManager.h"
#include "./resources/types/texture/TextureResource.h"

// Camera system
#include "./camera/CameraSystem.h"

// Animation (kept for now)
#include "Animation/AnimationManager.h"
#include "physics/PhysicsSystem.h"
#include "physics/dynamics/RigidBody.h"

#if defined(__APPLE__)
#include <OpenGL/gl.h>
#else
#include <GL/glew.h>
#include <GL/gl.h>
#endif

using namespace engine::memory;
using namespace engine::resources;
using namespace engine::input;
using namespace engine::math;
using namespace engine::camera;

/**
 * @brief Main game class with PHYSICS SYNCHRONIZED
 * ⭐ KEY CHANGE: playerPosition_ is now READ from physics, not written to independently
 */
class GameDemo {
public:
    GameDemo() = default;
    ~GameDemo() = default;

    static constexpr int WINDOW_WIDTH = 800;
    static constexpr int WINDOW_HEIGHT = 600;
    static constexpr float WORLD_WIDTH = 16.0f;
    static constexpr float WORLD_HEIGHT = 12.0f;
    static constexpr float PLAYER_MOVE_FORCE = 50.0f; // ⭐ Changed from SPEED to FORCE
    static constexpr float CAMERA_SPEED = 8.0f;

    enum Actions : ActionID {
        ACTION_MOVE_FORWARD = 1, ACTION_MOVE_BACKWARD = 2, ACTION_MOVE_LEFT     = 3,
        ACTION_MOVE_RIGHT   = 4, ACTION_EXIT          = 5, ACTION_SWITCH_CAMERA = 6,
        ACTION_ZOOM_IN      = 7, ACTION_ZOOM_OUT      = 8, ACTION_CAMERA_SHAKE  = 9,
        ACTION_CAMERA_UP    = 10, ACTION_CAMERA_DOWN  = 11, ACTION_CAMERA_LEFT  = 12,
        ACTION_CAMERA_RIGHT = 13, ACTION_TOGGLE_GRID  = 14, ACTION_TOGGLE_DEBUG = 15,
        ACTION_PLAYER_MOVE  = 20, ACTION_CAMERA_MOVE  = 21, ACTION_RESET_PLAYER = 22,
        // ⭐ NEW: Reset player position
    };

    bool initialize() {
        // Memory Manager initialization
        std::cout << "=== Initializing Memory Manager ===" << std::endl;

        MemoryManagerAutoConfig memConfig;
        memConfig.autoDetectLimits = true;
        memConfig.memoryUsagePercent = 0.10f;
        memConfig.heapSizePercent = 0.25f;

#ifdef _DEBUG
        const char* quickShutdown = std::getenv("GAME_QUICK_SHUTDOWN");
        if (quickShutdown && std::string(quickShutdown) == "1") {
            std::cout << "Quick shutdown mode enabled - disabling memory checks" << std::endl;
            memConfig.enableLeakDetection = false;
            memConfig.enableBoundsChecking = false;
        } else {
            memConfig.enableLeakDetection = true;
            memConfig.enableBoundsChecking = true;
            std::cout << "Debug mode: Memory checks enabled" << std::endl;
        }
#else
        memConfig.enableLeakDetection = false;
        memConfig.enableBoundsChecking = false;
        std::cout << "Release mode: Memory checks disabled" << std::endl;
#endif

        memoryManager_ = std::make_unique<MemoryManager>();
        if (!memoryManager_->initialize(memConfig)) {
            std::cerr << "Failed to initialize MemoryManager" << std::endl;
            return false;
        }
        std::cout << "✓ MemoryManager initialized successfully" << std::endl;

        // Physics System initialization
        physicsSystem_ = std::make_unique<engine::physics::PhysicsSystem>(*memoryManager_);

        engine::physics::PhysicsConfig config = engine::physics::PhysicsConfig::ForRPG();
        config.gravity = Vec3(0, -19.62f, 0);
        config.enableDebugDraw = true;

        if (!physicsSystem_->initialize(config)) {
            std::cerr << "Failed to initialize Physics System" << std::endl;
            return false;
        }

        // Create physics objects
        if (!createPhysicsObjects()) {
            return false;
        }

        // SDL initialization
        if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) < 0) {
            std::cerr << "Failed to initialize SDL: " << SDL_GetError() << std::endl;
            return false;
        }

        int imgFlags = IMG_INIT_PNG | IMG_INIT_JPG;
        if (!(IMG_Init(imgFlags) & imgFlags)) {
            std::cerr << "Failed to initialize SDL_image: " << IMG_GetError() << std::endl;
            SDL_Quit();
            return false;
        }

        window_ = SDL_CreateWindow(
            "Game Demo - Physics Synchronized",
            SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED,
            WINDOW_WIDTH,
            WINDOW_HEIGHT,
            SDL_WINDOW_SHOWN | SDL_WINDOW_OPENGL
        );
        if (!window_) {
            std::cerr << "Failed to create window: " << SDL_GetError() << std::endl;
            cleanup();
            return false;
        }

        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
        SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
        SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

        context_ = SDL_GL_CreateContext(window_);
        if (!context_) {
            std::cerr << "Failed to create OpenGL context: " << SDL_GetError() << std::endl;
            return false;
        }

        SDL_GL_SetSwapInterval(1);

        if (!initializeOpenGL()) {
            return false;
        }

        // Input System
        std::cout << "\n=== Initializing New Input System ===" << std::endl;

        inputSystem_ = std::make_unique<InputSystem>(memoryManager_.get());

        InputSystemConfig inputConfig;
        inputConfig.enableKeyboard = true;
        inputConfig.enableMouse = true;
        inputConfig.enableGamepad = true;
        inputConfig.enableDebugLogging = false;
        inputConfig.mouseSensitivity = 1.0f;

        if (!inputSystem_->initialize(inputConfig)) {
            std::cerr << "Failed to initialize InputSystem" << std::endl;
            cleanup();
            return false;
        }

        setupInputActions();
        std::cout << "✓ InputSystem initialized successfully" << std::endl;

        // Resource Manager
        std::cout << "\n=== Initializing Resource Manager ===" << std::endl;
        resourceManager_ = std::make_unique<ResourceManager>(*memoryManager_);

        ResourceManagerConfig resourceConfig;
        resourceConfig.maxMemory = 256 * 1024 * 1024;
        resourceConfig.enableHotReload = false;
        resourceConfig.loaderThreadCount = 2;

        if (!resourceManager_->initialize(resourceConfig)) {
            std::cerr << "Failed to initialize ResourceManager" << std::endl;
            cleanup();
            return false;
        }
        resourceManager_->registerResourceType<TextureResource>();
        std::cout << "✓ ResourceManager initialized successfully" << std::endl;

        // Camera Manager
        CameraManagerConfig cameraConfig;
        cameraConfig.enableCameraLogging = true;
        cameraConfig.enableTransitions = true;
        cameraConfig.enableShake = true;
        cameraConfig.maxCameras = 4;
        cameraConfig.defaultSmoothingSpeed = 8.0f;

        cameraManager_ = std::make_unique<CameraManager>(cameraConfig);
        if (!cameraManager_->initialize()) {
            std::cerr << "Failed to initialize CameraManager" << std::endl;
            cleanup();
            return false;
        }

        Viewport viewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
        cameraManager_->setViewport(viewport);

        if (!setupWorldAndCamera()) {
            std::cerr << "Failed to setup world and camera" << std::endl;
            cleanup();
            return false;
        }

        if (!loadGameResources()) {
            std::cerr << "Failed to load game resources" << std::endl;
            cleanup();
            return false;
        }

        // ⭐ CRITICAL: Initialize player visual position from PHYSICS position
        syncPlayerPositionFromPhysics();

        std::cout << "Player initialized at position: (" << playerPosition_.x << ", " << playerPosition_.y << ")" << std::endl;

        // Set camera to follow player
        Camera2D* mainCamera = cameraManager_->getCamera2D(mainCameraId_);
        if (mainCamera) {
            mainCamera->setTarget(playerPosition_);
            mainCamera->setPosition(playerPosition_);
            std::cout << "✓ Camera set to follow player" << std::endl;
        }

        std::cout << "\nGame initialized successfully!" << std::endl;
        printControls();
        return true;
    }

    void setupInputActions() {
        inputSystem_->registerAction(ACTION_MOVE_FORWARD, "MoveForward", ActionType::BUTTON);
        inputSystem_->registerAction(ACTION_MOVE_BACKWARD, "MoveBackward", ActionType::BUTTON);
        inputSystem_->registerAction(ACTION_MOVE_LEFT, "MoveLeft", ActionType::BUTTON);
        inputSystem_->registerAction(ACTION_MOVE_RIGHT, "MoveRight", ActionType::BUTTON);
        inputSystem_->registerAction(ACTION_EXIT, "Exit", ActionType::BUTTON);
        inputSystem_->registerAction(ACTION_SWITCH_CAMERA, "SwitchCamera", ActionType::BUTTON);
        inputSystem_->registerAction(ACTION_ZOOM_IN, "ZoomIn", ActionType::BUTTON);
        inputSystem_->registerAction(ACTION_ZOOM_OUT, "ZoomOut", ActionType::BUTTON);
        inputSystem_->registerAction(ACTION_CAMERA_SHAKE, "CameraShake", ActionType::BUTTON);
        inputSystem_->registerAction(ACTION_CAMERA_UP, "CameraUp", ActionType::BUTTON);
        inputSystem_->registerAction(ACTION_CAMERA_DOWN, "CameraDown", ActionType::BUTTON);
        inputSystem_->registerAction(ACTION_CAMERA_LEFT, "CameraLeft", ActionType::BUTTON);
        inputSystem_->registerAction(ACTION_CAMERA_RIGHT, "CameraRight", ActionType::BUTTON);
        inputSystem_->registerAction(ACTION_TOGGLE_GRID, "ToggleGrid", ActionType::BUTTON);
        inputSystem_->registerAction(ACTION_TOGGLE_DEBUG, "ToggleDebug", ActionType::BUTTON);
        inputSystem_->registerAction(ACTION_PLAYER_MOVE, "PlayerMove", ActionType::AXIS_2D);
        inputSystem_->registerAction(ACTION_CAMERA_MOVE, "CameraMove", ActionType::AXIS_2D);
        inputSystem_->registerAction(ACTION_RESET_PLAYER, "ResetPlayer", ActionType::BUTTON);

        inputSystem_->bindKey(ACTION_MOVE_FORWARD, KeyCode::W, "Default");
        inputSystem_->bindKey(ACTION_MOVE_BACKWARD, KeyCode::S, "Default");
        inputSystem_->bindKey(ACTION_MOVE_LEFT, KeyCode::A, "Default");
        inputSystem_->bindKey(ACTION_MOVE_RIGHT, KeyCode::D, "Default");
        inputSystem_->bindKey(ACTION_EXIT, KeyCode::ESCAPE, "Default");
        inputSystem_->bindKey(ACTION_SWITCH_CAMERA, KeyCode::C, "Default");
        inputSystem_->bindKey(ACTION_ZOOM_IN, KeyCode::EQUAL, "Default");
        inputSystem_->bindKey(ACTION_ZOOM_OUT, KeyCode::MINUS, "Default");
        inputSystem_->bindKey(ACTION_CAMERA_SHAKE, KeyCode::SPACE, "Default");
        inputSystem_->bindKey(ACTION_CAMERA_UP, KeyCode::UP, "Default");
        inputSystem_->bindKey(ACTION_CAMERA_DOWN, KeyCode::DOWN, "Default");
        inputSystem_->bindKey(ACTION_CAMERA_LEFT, KeyCode::LEFT, "Default");
        inputSystem_->bindKey(ACTION_CAMERA_RIGHT, KeyCode::RIGHT, "Default");
        inputSystem_->bindKey(ACTION_TOGGLE_GRID, KeyCode::G, "Default");
        inputSystem_->bindKey(ACTION_TOGGLE_DEBUG, KeyCode::X, "Default");
        inputSystem_->bindKey(ACTION_RESET_PLAYER, KeyCode::R, "Default"); // ⭐ NEW

        inputSystem_->bindGamepadButton(ACTION_MOVE_FORWARD, GamepadButton::DPAD_UP, "Default");
        inputSystem_->bindGamepadButton(ACTION_MOVE_BACKWARD, GamepadButton::DPAD_DOWN, "Default");
        inputSystem_->bindGamepadButton(ACTION_MOVE_LEFT, GamepadButton::DPAD_LEFT, "Default");
        inputSystem_->bindGamepadButton(ACTION_MOVE_RIGHT, GamepadButton::DPAD_RIGHT, "Default");
        inputSystem_->bindGamepadButton(ACTION_SWITCH_CAMERA, GamepadButton::Y, "Default");
        inputSystem_->bindGamepadButton(ACTION_CAMERA_SHAKE, GamepadButton::X, "Default");
        inputSystem_->bindGamepadAxis(ACTION_PLAYER_MOVE, GamepadAxis::LEFT_STICK_X, "Default");
        inputSystem_->bindGamepadAxis(ACTION_CAMERA_MOVE, GamepadAxis::RIGHT_STICK_X, "Default");

        std::cout << "Input actions and bindings configured successfully!" << std::endl;
    }

    bool setupWorldAndCamera() {
        mainCameraId_ = cameraManager_->createCamera2D("MainCamera");
        if (mainCameraId_ == INVALID_CAMERA_ID) {
            std::cerr << "Failed to create main camera" << std::endl;
            return false;
        }

        Camera2D* mainCamera = cameraManager_->getCamera2D(mainCameraId_);
        if (!mainCamera) {
            std::cerr << "Failed to get main camera" << std::endl;
            return false;
        }

        mainCamera->setMode(CameraMode::TOP_DOWN);
        mainCamera->setPosition(Vec2(WORLD_WIDTH / 2, WORLD_HEIGHT / 2));
        mainCamera->setZoom(1.0f);
        mainCamera->setFollowSpeed(6.0f);
        mainCamera->setZoomLimits(0.5f, 2.0f);
        mainCamera->setSmoothingSpeed(8.0f);

        CameraBounds worldBounds(
            Vec3(0.0f, 0.0f, -1.0f),
            Vec3(WORLD_WIDTH, WORLD_HEIGHT, 1.0f)
        );
        worldBounds.setEnabled(true);

        mainCamera->setBounds(worldBounds);
        mainCamera->setMode(CameraMode::FOLLOW_TARGET);

        std::cout << "✓ Main camera configured with world bounds" << std::endl;

        if (!cameraManager_->setActiveCamera(mainCameraId_)) {
            std::cerr << "Failed to set active camera" << std::endl;
            return false;
        }

        freeCameraId_ = cameraManager_->createCamera2D("FreeCamera");
        if (freeCameraId_ != INVALID_CAMERA_ID) {
            if (Camera2D* freeCamera = cameraManager_->getCamera2D(freeCameraId_)) {
                freeCamera->setPosition(Vec2(WORLD_WIDTH / 2, WORLD_HEIGHT / 2));
                freeCamera->setBounds(worldBounds);
                freeCamera->setMode(CameraMode::STATIC);
                std::cout << "✓ Free camera created for manual control" << std::endl;
            }
        }

        return true;
    }

    bool loadGameResources() {
        std::cout << "\n=== Loading Game Resources ===" << std::endl;

        const ResourceID playerTextureId = std::hash<std::string>{}("player_texture");

        playerTextureHandle_ = resourceManager_->loadById<TextureResource>(
            playerTextureId,
            std::string("../assets/player-cube.png"),
            ResourcePriority::HIGH,
            LoadMode::SYNC
        );

        auto playerTexture = getTextureResource(playerTextureHandle_);
        if (!playerTexture) {
            std::cerr << "Failed to load player sprite sheet" << std::endl;
            return false;
        }

        playerTextureHandle_.updateCache(playerTexture);

        std::cout << "✓ Player sprite sheet loaded: " << playerTexture->getWidth()
                << "x" << playerTexture->getHeight() << " pixels" << std::endl;

        SDL_Surface* surface = playerTexture->getSDLSurface();
        if (surface) {
            playerGLTexture_ = surfaceToGLTexture(surface);
            if (playerGLTexture_ == 0) {
                std::cerr << "Failed to create OpenGL texture from surface" << std::endl;
                return false;
            }
            std::cout << "✓ OpenGL texture created for rendering (ID: " << playerGLTexture_ << ")" << std::endl;
        } else {
            std::cerr << "Failed to get SDL surface from texture resource" << std::endl;
            return false;
        }

        currentSpriteFrame_.x = 0;
        currentSpriteFrame_.y = 0;
        currentSpriteFrame_.w = 48;
        currentSpriteFrame_.h = 48;

        return true;
    }

    void run() {
        bool running = true;

        auto lastStatsTime = std::chrono::steady_clock::now();
        auto lastFrameTime = std::chrono::steady_clock::now();
        int frameCount = 0;

        std::cout << "\n=== Starting Game Loop ===" << std::endl;

        while (running) {
            auto currentTime = std::chrono::steady_clock::now();
            auto deltaTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastFrameTime);
            float deltaTime = deltaTimeMs.count() / 1000.0f;
            lastFrameTime = currentTime;

            deltaTime = std::min(deltaTime, 0.016f);

            inputSystem_->update(deltaTime);

            if (inputSystem_->isActionTriggered(ACTION_EXIT)) {
                running = false;
                break;
            }

            processCameraInput(deltaTime);

            // ⭐ UPDATE PHYSICS FIRST
            physicsSystem_->update(deltaTime);

            // ⭐ THEN update game logic (applies forces to physics)
            updateGameLogic(deltaTime);

            // ⭐ SYNC visual position from physics AFTER physics update
            syncPlayerPositionFromPhysics();

            cameraManager_->update(deltaTime);
            render();

            frameCount++;
            if (currentTime - lastStatsTime > std::chrono::seconds(3)) {
                showStats(frameCount, currentTime - lastStatsTime);
                frameCount = 0;
                lastStatsTime = currentTime;
            }

            auto frameTime = std::chrono::steady_clock::now() - currentTime;
            auto targetFrameTime = std::chrono::milliseconds(16);
            if (frameTime < targetFrameTime) {
                std::this_thread::sleep_for(targetFrameTime - frameTime);
            }
        }

        std::cout << "\nGame loop ended" << std::endl;
    }

    void processCameraInput(float deltaTime) {
        if (inputSystem_->isActionTriggered(ACTION_SWITCH_CAMERA)) {
            toggleCameraMode();
        }

        if (inputSystem_->isActionTriggered(ACTION_ZOOM_IN)) {
            cameraManager_->processZoom(1.0f);
        }
        if (inputSystem_->isActionTriggered(ACTION_ZOOM_OUT)) {
            cameraManager_->processZoom(-1.0f);
        }

        if (inputSystem_->isActionTriggered(ACTION_CAMERA_SHAKE)) {
            ShakeConfig shakeConfig = ShakeConfig::explosion(3.0f);
            shakeConfig.duration = 0.5f;
            const CameraID cameraId = cameraManager_->getActiveCameraId();
            cameraManager_->startCameraShake(cameraId, shakeConfig);
        }

        Camera2D* activeCamera = cameraManager_->getCamera2D(cameraManager_->getActiveCameraId());
        if (activeCamera && activeCamera->getMode() == CameraMode::STATIC) {
            float cameraHorizontal = 0.0f;
            float cameraVertical = 0.0f;

            if (inputSystem_->isActionTriggered(ACTION_CAMERA_LEFT)) {
                cameraHorizontal -= 1.0f;
            }
            if (inputSystem_->isActionTriggered(ACTION_CAMERA_RIGHT)) {
                cameraHorizontal += 1.0f;
            }
            if (inputSystem_->isActionTriggered(ACTION_CAMERA_UP)) {
                cameraVertical -= 1.0f;
            }
            if (inputSystem_->isActionTriggered(ACTION_CAMERA_DOWN)) {
                cameraVertical += 1.0f;
            }

            Vec2 gamepadCamera = inputSystem_->getActionValue2D(ACTION_CAMERA_MOVE);
            cameraHorizontal += gamepadCamera.x;
            cameraVertical += gamepadCamera.y;

            if (std::abs(cameraHorizontal) > 0.1f || std::abs(cameraVertical) > 0.1f) {
                Vec2 currentPos = activeCamera->getPosition2D();
                Vec2 movement(
                    cameraHorizontal * CAMERA_SPEED * deltaTime,
                    cameraVertical * CAMERA_SPEED * deltaTime
                );
                activeCamera->setPosition(currentPos + movement);
            }
        }

        if (inputSystem_->isActionTriggered(ACTION_TOGGLE_GRID)) {
            showGrid_ = !showGrid_;
            std::cout << "Grid display: " << (showGrid_ ? "ON" : "OFF") << std::endl;
        }
        if (inputSystem_->isActionTriggered(ACTION_TOGGLE_DEBUG)) {
            showDebugInfo_ = !showDebugInfo_;
            std::cout << "Debug info: " << (showDebugInfo_ ? "ON" : "OFF") << std::endl;
        }
    }

    /**
     * ⭐ CRITICAL CHANGE: Now applies FORCES to physics body instead of moving visual position
     * ⭐ FIXED: Better force application to prevent jitter
     */
    void updateGameLogic(float deltaTime) {
        if (!player_) {
            return;
        }

        // ⭐ Reset player position
        if (inputSystem_->isActionTriggered(ACTION_RESET_PLAYER)) {
            resetPlayerPosition();
            return;
        }

        // ⭐ CRITICAL: Check if player is grounded (touching ground)
        Vec3 playerPos = player_->getPosition();
        Vec3 groundPos = ground_->getPosition();
        float distanceToGround = std::abs(playerPos.y - (groundPos.y + 1.0f)); // +1.0 for ground height + player radius
        bool isGrounded = distanceToGround < 0.1f;                             // Small threshold for "touching"

        Vec3 force{0.0f, 0.0f, 0.0f};
        bool isMoving = false;

        // Calculate force based on input (in 2D plane, Z=0)
        if (inputSystem_->getActionValue(ACTION_MOVE_FORWARD) > 0.5f) {
            force.y += PLAYER_MOVE_FORCE;
            isMoving = true;
        }
        if (inputSystem_->getActionValue(ACTION_MOVE_BACKWARD) > 0.5f) {
            force.y -= PLAYER_MOVE_FORCE;
            isMoving = true;
        }
        if (inputSystem_->getActionValue(ACTION_MOVE_LEFT) > 0.5f) {
            force.x -= PLAYER_MOVE_FORCE;
            isMoving = true;
        }
        if (inputSystem_->getActionValue(ACTION_MOVE_RIGHT) > 0.5f) {
            force.x += PLAYER_MOVE_FORCE;
            isMoving = true;
        }

        if (isMoving && isGrounded) {
            if (force.x != 0.0f && force.y != 0.0f) {
                float length = std::sqrt(force.x * force.x + force.y * force.y);
                if (length > 0.0f) {
                    force.x = (force.x / length) * PLAYER_MOVE_FORCE;
                    force.y = (force.y / length) * PLAYER_MOVE_FORCE;
                }
            }

            Vec3 impulse = force * deltaTime * 10.0f;
            player_->applyImpulse(impulse);

            Vec3 vel = player_->getLinearVelocity();
            vel.x *= 0.85f; // Dampen horizontal velocity
            vel.y *= 0.85f; // Dampen vertical velocity
            player_->setLinearVelocity(vel);
        }

        if (!isMoving && isGrounded) {
            Vec3 vel = player_->getLinearVelocity();
            vel.x *= 0.7f;
            vel.y *= 0.7f;
            player_->setLinearVelocity(vel);
        }

        // ⭐ Clamp max speed regardless of grounded state
        Vec3 vel = player_->getLinearVelocity();
        const float maxSpeed = 8.0f; // Slightly lower max speed
        float speed = std::sqrt(vel.x * vel.x + vel.y * vel.y);
        if (speed > maxSpeed) {
            vel.x = (vel.x / speed) * maxSpeed;
            vel.y = (vel.y / speed) * maxSpeed;
            player_->setLinearVelocity(vel);
        }

        // Update camera target
        Camera2D* followCamera = cameraManager_->getCamera2D(mainCameraId_);
        if (followCamera && followCamera->getMode() == CameraMode::FOLLOW_TARGET) {
            followCamera->setTarget(playerPosition_);
        }

        // Debug output
        static int debugCounter = 0;
        if (++debugCounter % 60 == 0) {
            Vec3 physicsPos = player_->getPosition();
            Vec3 velocity = player_->getLinearVelocity();
            std::cout << "\n=== PHYSICS DEBUG ===" << std::endl;
            std::cout << "Physics Pos: (" << physicsPos.x << ", " << physicsPos.y << ", " << physicsPos.z << ")" << std::endl;
            std::cout << "Visual Pos: (" << playerPosition_.x << ", " << playerPosition_.y << ")" << std::endl;
            std::cout << "Velocity: (" << velocity.x << ", " << velocity.y << ", " << velocity.z << ")" << std::endl;
            std::cout << "Is Grounded: " << isGrounded << std::endl;
            std::cout << "Distance to Ground: " << distanceToGround << std::endl;
            std::cout << "Is Active: " << player_->isActive() << std::endl;
            std::cout << "===================\n" << std::endl;
        }
    }

    /**
     * ⭐ NEW METHOD: Sync visual position from physics RigidBody
     */
    void syncPlayerPositionFromPhysics() {
        if (!player_) {
            return;
        }

        Vec3 physicsPos = player_->getPosition();
        playerPosition_.x = physicsPos.x;
        playerPosition_.y = physicsPos.y;
        // Z is ignored for 2D rendering
    }

    /**
     * ⭐ NEW METHOD: Reset player to starting position
     */
    void resetPlayerPosition() {
        if (!player_) {
            return;
        }

        std::cout << "Resetting player position..." << std::endl;

        // Reset to starting position (left side, high up)
        player_->setPosition(Vec3(3.0f, 10.0f, 0));
        player_->setLinearVelocity(Vec3(0, 0, 0));
        player_->setAngularVelocity(Vec3(0, 0, 0));
        player_->activate(true);

        syncPlayerPositionFromPhysics();

        std::cout << "Player reset to (" << playerPosition_.x << ", " << playerPosition_.y << ")" << std::endl;
    }

    void toggleCameraMode() {
        static bool isFollowMode = true;

        if (isFollowMode) {
            cameraManager_->setActiveCamera(freeCameraId_);
            std::cout << "Camera Mode: FREE CONTROL (use arrow keys)" << std::endl;
        } else {
            cameraManager_->setActiveCamera(mainCameraId_);
            std::cout << "Camera Mode: FOLLOW PLAYER" << std::endl;
        }

        isFollowMode = !isFollowMode;
    }

    void render() {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();

        Camera2D* activeCamera = cameraManager_->getCamera2D(cameraManager_->getActiveCameraId());

        if (activeCamera) {
            Vec2 cameraPos = activeCamera->getPosition2D();
            float zoom = activeCamera->getZoom();

            float halfWidth = (WORLD_WIDTH / 2.0f) / zoom;
            float halfHeight = (WORLD_HEIGHT / 2.0f) / zoom;

            // ⭐ FIXED: Correct Y-axis orientation
            // bottom should be LESS than top (bottom < top)
            glOrtho(
                cameraPos.x - halfWidth,
                // left
                cameraPos.x + halfWidth,
                // right
                cameraPos.y - halfHeight,
                // bottom (LOWER value)
                cameraPos.y + halfHeight,
                // top (HIGHER value)
                -1.0f,
                1.0f
            );
        } else {
            // Fallback: Standard orientation (Y+ = up)
            glOrtho(0, WORLD_WIDTH, 0, WORLD_HEIGHT, -1.0f, 1.0f);
        }

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        if (ground_) {
            renderGround();
        }

        // ⭐ Render player at physics-synced position
        renderPlayerOpenGL();

        // ⭐ NEW: Render physics debug visualization
        if (showDebugInfo_) {
            renderPhysicsDebug();
        }

        SDL_GL_SwapWindow(window_);
    }

    void renderGround() const {
        if (!ground_)
            return;

        Vec3 pos = ground_->getPosition();

        glColor3f(1.0f, 1.0f, 1.0f);

        glPushMatrix();
        glTranslatef(pos.x, pos.y, pos.z);

        glColor3f(0.4f, 0.2f, 0.1f); // Brown

        glBegin(GL_QUADS);
        glVertex2f(-WORLD_WIDTH / 2, -0.5f);
        glVertex2f(WORLD_WIDTH / 2, -0.5f);
        glVertex2f(WORLD_WIDTH / 2, 0.5f);
        glVertex2f(-WORLD_WIDTH / 2, 0.5f);
        glEnd();

        glPopMatrix();

        glColor3f(1.0f, 1.0f, 1.0f);
    }

    void renderPlayerOpenGL() {
        if (playerGLTexture_ == 0)
            return;

        auto playerTexture = getTextureResource(playerTextureHandle_);
        if (!playerTexture)
            return;

        float texWidth = static_cast<float>(playerTexture->getWidth());
        float texHeight = static_cast<float>(playerTexture->getHeight());

        float u_min = currentSpriteFrame_.x / texWidth;
        float v_min = currentSpriteFrame_.y / texHeight;
        float u_max = (currentSpriteFrame_.x + currentSpriteFrame_.w) / texWidth;
        float v_max = (currentSpriteFrame_.y + currentSpriteFrame_.h) / texHeight;

        float texCoords[4] = {u_min, v_min, u_max, v_max};

        float worldUnitPerPixel = WORLD_WIDTH / static_cast<float>(WINDOW_WIDTH);
        float spriteWorldWidth = currentSpriteFrame_.w * worldUnitPerPixel;
        float spriteWorldHeight = currentSpriteFrame_.h * worldUnitPerPixel;

        // ⭐ Use synced position
        renderTexturedQuad(
            playerGLTexture_,
            playerPosition_.x,
            playerPosition_.y,
            spriteWorldWidth,
            spriteWorldHeight,
            texCoords
        );
    }

    /**
     * ⭐ NEW: Render physics debug info (bounding boxes, velocities, etc.)
     */
    void renderPhysicsDebug() {
        if (!player_)
            return;

        Vec3 playerPos = player_->getPosition();
        Vec3 velocity = player_->getLinearVelocity();

        // Draw velocity vector
        if (std::abs(velocity.x) > 0.1f || std::abs(velocity.y) > 0.1f) {
            glColor3f(1.0f, 1.0f, 0.0f); // Yellow
            glLineWidth(2.0f);
            glBegin(GL_LINES);
            glVertex2f(playerPos.x, playerPos.y);
            glVertex2f(playerPos.x + velocity.x * 0.2f, playerPos.y + velocity.y * 0.2f);
            glEnd();
            glLineWidth(1.0f);
        }

        // Draw physics bounding box (cube extents = 0.5)
        glColor3f(0.0f, 1.0f, 0.0f); // Green
        glBegin(GL_LINE_LOOP);
        glVertex2f(playerPos.x - 0.5f, playerPos.y - 0.5f);
        glVertex2f(playerPos.x + 0.5f, playerPos.y - 0.5f);
        glVertex2f(playerPos.x + 0.5f, playerPos.y + 0.5f);
        glVertex2f(playerPos.x - 0.5f, playerPos.y + 0.5f);
        glEnd();

        glColor3f(1.0f, 1.0f, 1.0f); // Reset
    }

    Vec2 getCameraPosition() {
        const BaseCamera* activeCamera = cameraManager_->getActiveCamera();
        if (activeCamera) {
            Vec3 pos3D = activeCamera->getPosition();
            Vec2 cameraWorldPos(pos3D.x, pos3D.y);
            Vec2 renderOffset(
                cameraWorldPos.x - WINDOW_WIDTH / 2.0f,
                cameraWorldPos.y - WINDOW_HEIGHT / 2.0f
            );
            return renderOffset;
        }
        return Vec2{0.0f, 0.0f};
    }

    void showStats(int frameCount, std::chrono::steady_clock::duration elapsed) {
        auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
        double fps = frameCount * 1000.0 / elapsedMs;

        std::cout << "\n--- Game Performance Stats ---" << std::endl;
        std::cout << "FPS: " << fps << std::endl;

        std::cout << "Resource Memory: " << (resourceManager_->getMemoryUsage() / 1024.0 / 1024.0) << " MB" <<
                std::endl;
        std::cout << "Total Memory Usage: " << (memoryManager_->getTotalMemoryUsage() / 1024.0 / 1024.0) << " MB" <<
                std::endl;

        std::cout << "Cameras: " << cameraManager_->getCameraCount() << std::endl;

        const BaseCamera* activeCamera = cameraManager_->getActiveCamera();
        if (activeCamera) {
            Vec3 cameraPos = activeCamera->getPosition();
            std::cout << "Camera Position: (" << cameraPos.x << ", " << cameraPos.y << ")" << std::endl;
        }

        auto inputStats = inputSystem_->getStatistics();
        std::cout << "Input Frames: " << inputStats.totalFrames << std::endl;

        std::cout << "Player Position: (" << playerPosition_.x << ", " << playerPosition_.y << ")" << std::endl;

        if (player_) {
            Vec3 vel = player_->getLinearVelocity();
            std::cout << "Player Velocity: (" << vel.x << ", " << vel.y << ")" << std::endl;
        }
    }

    void printControls() {
        std::cout << "=== GAME CONTROLS (PHYSICS VERSION) ===" << std::endl;
        std::cout << "Player Movement (applies forces):" << std::endl;
        std::cout << "  WASD: Apply movement forces to player" << std::endl;
        std::cout << "  R: Reset player position" << std::endl;
        std::cout << "\nCamera Controls:" << std::endl;
        std::cout << "  C: Switch camera mode (Follow/Free)" << std::endl;
        std::cout << "  Arrow Keys: Manual camera (Free mode only)" << std::endl;
        std::cout << "  +/-: Zoom in/out" << std::endl;
        std::cout << "  Space: Camera shake" << std::endl;
        std::cout << "\nDebug:" << std::endl;
        std::cout << "  X: Toggle debug visualization (velocity vectors, bounding boxes)" << std::endl;
        std::cout << "  G: Toggle grid display" << std::endl;
        std::cout << "  ESC: Quit game" << std::endl;
        std::cout << "========================================" << std::endl;
    }

    ResourcePtr<TextureResource> getTextureResource(const ResourceHandle<TextureResource>& handle) {
        if (!handle.isValid())
            return nullptr;

        if (auto cached = handle.tryGet()) {
            return cached;
        }

        return resourceManager_->getResource<TextureResource>(handle.getId());
    }

    void cleanup() {
        std::cout << "\nCleaning up game resources..." << std::endl;

        if (playerGLTexture_ != 0) {
            glDeleteTextures(1, &playerGLTexture_);
            playerGLTexture_ = 0;
        }

        playerTextureHandle_.reset();
        backgroundTextureHandle_.reset();

        if (cameraManager_) {
            cameraManager_->shutdown();
            cameraManager_.reset();
        }

        if (inputSystem_) {
            inputSystem_->shutdown();
            inputSystem_.reset();
        }

        if (resourceManager_) {
            resourceManager_.reset();
        }

        if (memoryManager_) {
            std::cout << "\n=== Memory Manager Final Report ===" << std::endl;
            std::cout << memoryManager_->generateMemoryReport() << std::endl;

            size_t leaks = engine::memory::MemoryManager::checkForLeaks();
            if (leaks > 0) {
                std::cout << "WARNING: " << leaks << " memory leaks detected!" << std::endl;
            } else {
                std::cout << "✓ No memory leaks detected" << std::endl;
            }

            memoryManager_->shutdown();
            memoryManager_.reset();
        }

        if (context_) {
            SDL_GL_DeleteContext(context_);
            context_ = nullptr;
        }

        if (window_) {
            SDL_DestroyWindow(window_);
            window_ = nullptr;
        }

        IMG_Quit();
        SDL_Quit();

        std::cout << "Cleanup completed!" << std::endl;
    }

private:
    enum class PlayerAnimation {
        IDLE, UP, DOWN,
        LEFT, RIGHT
    };

    SDL_Window* window_ = nullptr;
    SDL_GLContext context_;
    GLuint playerGLTexture_ = 0;

    std::unique_ptr<MemoryManager> memoryManager_;
    std::unique_ptr<ResourceManager> resourceManager_;
    std::unique_ptr<CameraManager> cameraManager_;
    std::unique_ptr<engine::physics::PhysicsSystem> physicsSystem_;
    std::unique_ptr<InputSystem> inputSystem_;

    ResourceHandle<TextureResource> playerTextureHandle_;
    ResourceHandle<TextureResource> backgroundTextureHandle_;

    engine::physics::RigidBody* player_;
    engine::physics::RigidBody* ground_;

    // ⭐ This is now READ from physics, not written to independently
    Vec2 playerPosition_;
    SDL_Rect currentSpriteFrame_;

    CameraID mainCameraId_ = INVALID_CAMERA_ID;
    CameraID freeCameraId_ = INVALID_CAMERA_ID;

    bool showGrid_ = false;
    bool showDebugInfo_ = true;

    bool initializeOpenGL() {
        glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        // ⭐ FIXED: Correct Y-axis orientation (Y+ = up)
        // Standard convention: left, right, bottom, top
        glOrtho(0, WORLD_WIDTH, 0, WORLD_HEIGHT, -1.0f, 1.0f);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glClearColor(0.5f, 0.7f, 1.0f, 1.0f);

        GLenum error = glGetError();
        if (error != GL_NO_ERROR) {
            std::cerr << "OpenGL Error: " << error << std::endl;
            return false;
        }

        return true;
    }

    bool createPhysicsObjects() {
        // ============================================================================
        // GROUND CREATION
        // ============================================================================
        auto groundShape = engine::physics::ShapeCreationParams::Box(Vec3(WORLD_WIDTH / 2, 0.5f, 1.0f));

        engine::physics::BodyCreationParams groundParams = engine::physics::BodyCreationParams::StaticBody(
            groundShape,
            nullptr,
            0.1f
        );
        groundParams.name = "Ground";

        // ⭐ FIXED: High friction material to prevent sliding
        groundParams.material = engine::physics::PhysicsMaterial::Concrete();
        groundParams.material.friction = 1.0f;    // Maximum friction
        groundParams.material.restitution = 0.0f; // No bouncing
        groundParams.material.rollingFriction = 0.5f;

        ground_ = physicsSystem_->createRigidBody(groundParams);

        if (!ground_) {
            std::cerr << "Failed to create ground body" << std::endl;
            return false;
        }

        // ⭐ Position ground at BOTTOM (Y=1)
        ground_->setPosition(Vec3(WORLD_WIDTH / 2, 1.0f, 0));

        std::cout << "✓ Ground positioned at: ("
                << ground_->getPosition().x << ", "
                << ground_->getPosition().y << ", "
                << ground_->getPosition().z << ")" << std::endl;

        // ============================================================================
        // PLAYER CREATION
        // ============================================================================
        auto boxShape = engine::physics::ShapeCreationParams::Box(Vec3(0.5f)); // Con 0.2f queda bien, pero no se mueve.

        engine::physics::BodyCreationParams boxParams = engine::physics::BodyCreationParams::DynamicBody(
            boxShape,
            1.0f,
            nullptr,
            5.0f
        );
        boxParams.name = "Player";

        boxParams.material = engine::physics::PhysicsMaterial::Wood();
        boxParams.material.friction = 2.0f;
        boxParams.material.restitution = 0.0f;
        boxParams.material.rollingFriction = 0.3f;

        boxParams.enableCCD = true;

        player_ = physicsSystem_->createRigidBody(boxParams);

        if (!player_) {
            std::cerr << "Failed to create player body" << std::endl;
            return false;
        }

        player_->setDamping(0.9f, 0.9f);
        player_->setPosition(Vec3(3.0f, 10.0f, 0));
        player_->setAngularFactor(Vec3(0,0,0));
        player_->setSleepingThresholds(0.2f, 0.5f);

        std::cout << "✓ Player positioned at: ("
                << player_->getPosition().x << ", "
                << player_->getPosition().y << ", "
                << player_->getPosition().z << ")" << std::endl;

        // ============================================================================
        // VERIFICATION
        // ============================================================================
        Vec3 playerPos = player_->getPosition();
        Vec3 groundPos = ground_->getPosition();
        float distanceY = playerPos.y - groundPos.y;

        std::cout << "\n=== Physics Setup Verification ===" << std::endl;
        std::cout << "Player Y: " << playerPos.y << std::endl;
        std::cout << "Ground Y: " << groundPos.y << std::endl;
        std::cout << "Vertical Distance: " << distanceY << " units" << std::endl;

        if (distanceY > 0) {
            std::cout << "✅ Player is ABOVE ground - will fall and collide" << std::endl;
        } else {
            std::cout << "❌ WARNING: Player is BELOW ground!" << std::endl;
        }

        std::cout << "\nPhysics Properties:" << std::endl;
        std::cout << "  Player mass: " << player_->getMass() << " kg" << std::endl;
        std::cout << "  Player restitution: " << player_->getRestitution() << std::endl;
        std::cout << "  World gravity: ("
                << physicsSystem_->getGravity().x << ", "
                << physicsSystem_->getGravity().y << ", "
                << physicsSystem_->getGravity().z << ")" << std::endl;
        std::cout << "================================\n" << std::endl;

        return true;
    }

    void renderTexturedQuad(
        GLuint texture,
        float x,
        float y,
        float width,
        float height,
        const float* texCoords = nullptr
    ) {
        float defaultTexCoords[4] = {0.0f, 0.0f, 1.0f, 1.0f};
        const float* tc = texCoords ? texCoords : defaultTexCoords;

        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, texture);

        glPushMatrix();
        glTranslatef(x, y, 0.0f);

        glBegin(GL_QUADS);
        glTexCoord2f(tc[0], tc[3]);
        glVertex2f(-width / 2, -height / 2);

        glTexCoord2f(tc[2], tc[3]);
        glVertex2f(width / 2, -height / 2);

        glTexCoord2f(tc[2], tc[1]);
        glVertex2f(width / 2, height / 2);

        glTexCoord2f(tc[0], tc[1]);
        glVertex2f(-width / 2, height / 2);
        glEnd();

        glPopMatrix();

        glBindTexture(GL_TEXTURE_2D, 0);
        glDisable(GL_TEXTURE_2D);
    }

    GLuint surfaceToGLTexture(SDL_Surface* surface) {
        if (!surface)
            return 0;

        GLuint textureID;
        glGenTextures(1, &textureID);
        glBindTexture(GL_TEXTURE_2D, textureID);

        GLenum format = (surface->format->BytesPerPixel == 4) ? GL_RGBA : GL_RGB;

        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            format,
            surface->w,
            surface->h,
            0,
            format,
            GL_UNSIGNED_BYTE,
            surface->pixels
        );

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        return textureID;
    }
};

int main() {
    std::cout << "=== Game Demo - Physics Synchronized ===" << std::endl;
    std::cout << "WASD applies forces to physics body" << std::endl;
    std::cout << "Player starts at left side, high up, and falls to ground" << std::endl;
    std::cout << "Press R to reset player position" << std::endl;
    std::cout << "Press X to toggle debug visualization" << std::endl;
    std::cout << "\nStarting game..." << std::endl;

    GameDemo game;

    if (!game.initialize()) {
        std::cerr << "\nFailed to initialize game!" << std::endl;
        return -1;
    }

    try {
        game.run();
    } catch (const std::exception& e) {
        std::cerr << "\nException during game loop: " << e.what() << std::endl;
        return -1;
    }

    game.cleanup();

    std::cout << "\nGame demo completed successfully!" << std::endl;
    return 0;
}
