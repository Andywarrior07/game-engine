// MAIN TEST - CON ROTACIÓN DE PLAYER EN SALTO

#include <SDL.h>
#include <SDL_image.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <thread>

#include "./camera/CameraSystem.h"
#include "./input/InputSystem.h"
#include "./memory/MemorySystem.h"
#include "./resources/manager/ResourceManager.h"
#include "./resources/types/texture/TextureResource.h"
#include "Animation/AnimationManager.h"
#include "physics/PhysicsSystem.h"
#include "physics/dynamics/RigidBody.h"

#if defined(__APPLE__)
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#include <GL/glew.h>
#endif

using namespace engine::memory;
using namespace engine::resources;
using namespace engine::input;
using namespace engine::math;
using namespace engine::camera;

/**
 * @brief Main game class with PHYSICS SYNCHRONIZED + ROTATION ON JUMP
 */
class GameDemo {
public:
    GameDemo() = default;
    ~GameDemo() = default;

    static constexpr int WINDOW_WIDTH = 1280;
    static constexpr int WINDOW_HEIGHT = 720;
    static constexpr float WORLD_WIDTH = 30.0f;
    static constexpr float WORLD_HEIGHT = 13.5f;
    static constexpr float PLAYER_MOVE_FORCE = 1000.0f;
    static constexpr float CAMERA_SPEED = 8.0f;
    static constexpr float GRAVITY_FORCE = -30.0f;

    enum Actions : ActionID {
        ACTION_JUMP = 1,
        ACTION_MOVE_BACKWARD = 2,
        ACTION_MOVE_LEFT = 3,
        ACTION_MOVE_RIGHT = 4,
        ACTION_EXIT = 5,
        ACTION_SWITCH_CAMERA = 6,
        ACTION_ZOOM_IN = 7,
        ACTION_ZOOM_OUT = 8,
        ACTION_CAMERA_SHAKE = 9,
        ACTION_CAMERA_UP = 10,
        ACTION_CAMERA_DOWN = 11,
        ACTION_CAMERA_LEFT = 12,
        ACTION_CAMERA_RIGHT = 13,
        ACTION_TOGGLE_GRID = 14,
        ACTION_TOGGLE_DEBUG = 15,
        ACTION_PLAYER_MOVE = 20,
        ACTION_CAMERA_MOVE = 21,
        ACTION_RESET_PLAYER = 22,
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
        config.gravity = Vec3(0, GRAVITY_FORCE, 0);
        config.enableDebugDraw = true;

        if (!physicsSystem_->initialize(config)) {
            std::cerr << "Failed to initialize Physics System" << std::endl;
            return false;
        }

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
                "Game Demo - Physics Synchronized + Rotation",
                SDL_WINDOWPOS_CENTERED,
                SDL_WINDOWPOS_CENTERED,
                WINDOW_WIDTH,
                WINDOW_HEIGHT,
                SDL_WINDOW_SHOWN | SDL_WINDOW_OPENGL);
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
        resourceConfig.gcInterval = std::chrono::seconds(5);

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

        syncPlayerPositionFromPhysics();

        std::cout << "Player initialized at position: (" << playerPosition_.x << ", " << playerPosition_.y << ")" << std::endl;

        if (Camera2D* mainCamera = cameraManager_->getCamera2D(mainCameraId_)) {
            mainCamera->setTarget(playerPosition_);
            mainCamera->setPosition(playerPosition_);
            std::cout << "✓ Camera set to follow player" << std::endl;
        }

        std::cout << "\nGame initialized successfully!" << std::endl;
        printControls();
        return true;
    }

    void setupInputActions() const {
        inputSystem_->registerAction(ACTION_JUMP, "Jump", ActionType::BUTTON);
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

        inputSystem_->bindKey(ACTION_JUMP, KeyCode::SPACE, "Default");
        inputSystem_->bindKey(ACTION_MOVE_BACKWARD, KeyCode::S, "Default");
        inputSystem_->bindKey(ACTION_MOVE_LEFT, KeyCode::A, "Default");
        inputSystem_->bindKey(ACTION_MOVE_RIGHT, KeyCode::D, "Default");
        inputSystem_->bindKey(ACTION_EXIT, KeyCode::ESCAPE, "Default");
        inputSystem_->bindKey(ACTION_SWITCH_CAMERA, KeyCode::C, "Default");
        inputSystem_->bindKey(ACTION_ZOOM_IN, KeyCode::EQUAL, "Default");
        inputSystem_->bindKey(ACTION_ZOOM_OUT, KeyCode::MINUS, "Default");
        inputSystem_->bindKey(ACTION_CAMERA_SHAKE, KeyCode::LEFT_SHIFT, "Default");
        inputSystem_->bindKey(ACTION_CAMERA_UP, KeyCode::UP, "Default");
        inputSystem_->bindKey(ACTION_CAMERA_DOWN, KeyCode::DOWN, "Default");
        inputSystem_->bindKey(ACTION_CAMERA_LEFT, KeyCode::LEFT, "Default");
        inputSystem_->bindKey(ACTION_CAMERA_RIGHT, KeyCode::RIGHT, "Default");
        inputSystem_->bindKey(ACTION_TOGGLE_GRID, KeyCode::G, "Default");
        inputSystem_->bindKey(ACTION_TOGGLE_DEBUG, KeyCode::X, "Default");
        inputSystem_->bindKey(ACTION_RESET_PLAYER, KeyCode::R, "Default");

        inputSystem_->bindGamepadButton(ACTION_JUMP, GamepadButton::DPAD_UP, "Default");
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
                Vec3(WORLD_WIDTH, WORLD_HEIGHT, 1.0f));
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
                std::string("../assets/player_02-uhd.png"),
                ResourcePriority::HIGH,
                LoadMode::SYNC);

        auto playerTexture = getTextureResource(playerTextureHandle_);
        if (!playerTexture) {
            std::cerr << "Failed to load player sprite sheet" << std::endl;
            return false;
        }

        playerTextureHandle_.updateCache(playerTexture);

        std::cout << "✓ Player sprite sheet loaded: " << playerTexture->getWidth()
                  << "x" << playerTexture->getHeight() << " pixels" << std::endl;

        if (SDL_Surface* surface = playerTexture->getSDLSurface()) {
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

        currentSpriteFrame_.x = 130;
        currentSpriteFrame_.y = 0;
        currentSpriteFrame_.w = 120;
        currentSpriteFrame_.h = 120;

        std::cout << "\n=== Loading Spike Texture ===" << std::endl;

        SDL_Surface* spikeSurface = IMG_Load("../assets/spike.png");
        if (!spikeSurface) {
            std::cerr << "Failed to load spike texture: " << IMG_GetError() << std::endl;
            return false;
        }

        std::cout << "✓ Spike texture loaded: "
                  << spikeSurface->w << "x" << spikeSurface->h << " pixels" << std::endl;

        spikeTexture_ = surfaceToGLTexture(spikeSurface);
        SDL_FreeSurface(spikeSurface);

        if (spikeTexture_ == 0) {
            std::cerr << "Failed to create OpenGL texture for spike" << std::endl;
            return false;
        }

        std::cout << "✓ Spike OpenGL texture created (ID: " << spikeTexture_ << ")" << std::endl;

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
            physicsSystem_->update(deltaTime);
            syncPlayerPositionFromPhysics();
            updateGameLogic(deltaTime);
            updatePlayerRotation(deltaTime); // ⭐ NUEVO: Actualizar rotación
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
                        cameraVertical * CAMERA_SPEED * deltaTime);
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

    void updateGameLogic(float deltaTime) {
        if (!player_)
            return;

        Vec3 playerPos = player_->getPosition();
        Vec3 groundPos = ground_->getPosition();

        float playerHalfHeight = 0.8f;
        float groundHalfHeight = 0.5f;

        float expectedDistance = groundHalfHeight + playerHalfHeight;
        float distanceToGround = std::abs(playerPos.y - groundPos.y - expectedDistance);

        bool isGrounded = distanceToGround < 0.2f;

        Vec3 currentVel = player_->getLinearVelocity();

        float targetSpeedX = 0.0f;
        float moveSpeed = 15.0f;

        float inputDir = 0.0f;
        if (inputSystem_->getActionValue(ACTION_MOVE_LEFT) > 0.5f)
            inputDir = -1.0f;
        if (inputSystem_->getActionValue(ACTION_MOVE_RIGHT) > 0.5f)
            inputDir = 1.0f;

        targetSpeedX = inputDir * moveSpeed;

        if (isGrounded) {
            float accelerationGround = 10.0f;
            currentVel.x += (targetSpeedX - currentVel.x) * accelerationGround * deltaTime;
        } else {
            if (inputDir != 0.0f) {
                float accelerationAir = 2.0f;
                bool movingAgainstInertia = (currentVel.x * inputDir < 0);

                if (std::abs(currentVel.x) < moveSpeed || movingAgainstInertia) {
                    currentVel.x += (targetSpeedX - currentVel.x) * accelerationAir * deltaTime;
                }
            }

            currentVel.x *= 0.995f;
        }

        // ⭐ LÓGICA DE SALTO MODIFICADA
        if (inputSystem_->getActionValue(ACTION_JUMP) > 0.5f && isGrounded && !isRotating_) {
            currentVel.y = 0.0f;
            player_->setLinearVelocity(currentVel);

            float jumpForce = 20.0f;
            player_->applyImpulse(Vec3(0, jumpForce, 0));

            // ⭐ INICIAR ROTACIÓN
            startRotation();

            isGrounded = false;
            return;
        }

        if (std::abs(currentVel.x) > 0.01f || std::abs(currentVel.y) > 0.01f) {
            player_->activate(true);
        }

        player_->setLinearVelocity(currentVel);

        if (Camera2D* followCamera = cameraManager_->getCamera2D(mainCameraId_);
            followCamera && followCamera->getMode() == CameraMode::FOLLOW_TARGET) {
            followCamera->setTarget(playerPos);
            followCamera->setSmoothingSpeed(0.0f);
            followCamera->setPosition(playerPos);
        }
    }

    /**
     * ⭐⭐⭐ NUEVA FUNCIÓN: Iniciar la rotación del player
     *
     * Esta función se llama cuando el player salta.
     * Activa el sistema de rotación y resetea el progreso.
     */
    void startRotation() {
        isRotating_ = true;                         // Activamos el flag de "estoy rotando"
        rotationProgress_ = 0.0f;                   // Empezamos desde 0% (0 grados)
        rotationStartAngle_ = playerRotationAngle_; // Guardamos el ángulo inicial

        std::cout << "🔄 Rotation started! Initial angle: " << rotationStartAngle_ << "°" << std::endl;
    }

    /**
     * ⭐⭐⭐ NUEVA FUNCIÓN: Actualizar la rotación del player cada frame
     *
     * Esta función se ejecuta en cada frame del juego loop.
     * Solo hace algo si isRotating_ es true.
     *
     * @param deltaTime - Tiempo transcurrido desde el último frame (en segundos)
     */
    void updatePlayerRotation(float deltaTime) {
        // Si NO estamos rotando, salimos inmediatamente
        if (!isRotating_) {
            return;
        }

        // ⭐ PASO 1: Incrementar el progreso de la rotación
        // rotationSpeed_ define qué tan rápido gira (1.0 = 180° en 1 segundo)
        rotationProgress_ += rotationSpeed_ * deltaTime;

        // ⭐ PASO 2: Verificar si ya completamos la rotación completa
        if (rotationProgress_ >= 1.0f) {
            // Nos pasamos del 100%, así que fijamos exactamente a 100%
            rotationProgress_ = 1.0f;

            // Calculamos el ángulo final exacto (inicial + 180°)
            playerRotationAngle_ = rotationStartAngle_ + 180.0f;

            // Desactivamos la rotación
            isRotating_ = false;

            std::cout << "✅ Rotation completed! Final angle: " << playerRotationAngle_ << "°" << std::endl;
            return;
        }

        // ⭐ PASO 3: Si aún no terminamos, calcular el ángulo intermedio
        // Interpolación lineal: ángulo = inicio + (180° × progreso)
        // Ejemplo: si progreso = 0.5 (50%), ángulo = inicio + 90°
        playerRotationAngle_ = rotationStartAngle_ + (180.0f * rotationProgress_);
    }

    void syncPlayerPositionFromPhysics() {
        if (!player_) {
            return;
        }

        const Vec3 physicsPos = player_->getPosition();
        playerPosition_.x = physicsPos.x;
        playerPosition_.y = physicsPos.y;
    }

    void resetPlayerPosition() {
        if (!player_) {
            return;
        }

        std::cout << "Resetting player position..." << std::endl;

        player_->setPosition(Vec3(3.0f, 90.0f, 0));
        player_->setLinearVelocity(Vec3(0, 0, 0));
        player_->setAngularVelocity(Vec3(0, 0, 0));
        player_->activate(true);

        // ⭐ Resetear también la rotación visual
        playerRotationAngle_ = 0.0f;
        isRotating_ = false;
        rotationProgress_ = 0.0f;

        syncPlayerPositionFromPhysics();

        std::cout << "Player reset to (" << playerPosition_.x << ", " << playerPosition_.y << ")" << std::endl;
    }

    void toggleCameraMode() const {
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
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();

        if (const Camera2D* activeCamera = cameraManager_->getCamera2D(cameraManager_->getActiveCameraId())) {
            const Vec2 cameraPos = activeCamera->getPosition2D();
            const float zoom = activeCamera->getZoom();

            const float halfWidth = (WORLD_WIDTH / 2.0f) / zoom;
            const float halfHeight = (WORLD_HEIGHT / 2.0f) / zoom;
            glOrtho(
                    cameraPos.x - halfWidth,
                    cameraPos.x + halfWidth,
                    cameraPos.y - halfHeight,
                    cameraPos.y + halfHeight,
                    -1.0f,
                    1.0f);
        } else {
            glOrtho(0, WORLD_WIDTH, 0, WORLD_HEIGHT, -1.0f, 1.0f);
        }

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        renderBackgroundGrid();

        if (ground_) {
            renderGround();
        }

        if (spike_ && spikeTexture_ != 0) {
            renderSpike();
        }

        // ⭐ MODIFICADO: Ahora pasamos el ángulo de rotación
        renderPlayerOpenGL();

        if (showDebugInfo_) {
            renderPhysicsDebug();
        }

        SDL_GL_SwapWindow(window_);
    }

    void renderBackgroundGrid() {
        glClearColor(0.1f, 0.0f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glEnable(GL_BLEND);
        glLineWidth(1.0f);
        glColor4f(1.0f, 1.0f, 1.0f, 0.15f);

        float camX = 0;
        float camY = 0;

        if (auto* cam = cameraManager_->getCamera2D(cameraManager_->getActiveCameraId())) {
            camX = cam->getPosition2D().x;
            camY = cam->getPosition2D().y;
        }

        float spacing = 1.0f;
        int gridCount = 40;

        glBegin(GL_LINES);
        float startX = std::floor(camX) - gridCount / 2;
        for (int i = 0; i < gridCount; i++) {
            glVertex2f(startX + i * spacing, camY - 20);
            glVertex2f(startX + i * spacing, camY + 20);
        }
        float startY = std::floor(camY) - gridCount / 2;
        for (int i = 0; i < gridCount; i++) {
            glVertex2f(camX - 20, startY + i * spacing);
            glVertex2f(camX + 20, startY + i * spacing);
        }
        glEnd();
    }

    void renderGround() const {
        if (!ground_)
            return;
        Vec3 pos = ground_->getPosition();

        float yTop = pos.y + 0.5f;
        float drawWidth = WORLD_WIDTH * 50.0f;

        glColor3f(0.05f, 0.0f, 0.2f);
        glBegin(GL_QUADS);
        glVertex2f(pos.x - drawWidth, yTop - 10.0f);
        glVertex2f(pos.x + drawWidth, yTop - 10.0f);
        glVertex2f(pos.x + drawWidth, yTop);
        glVertex2f(pos.x - drawWidth, yTop);
        glEnd();

        glLineWidth(3.0f);
        glColor3f(0.0f, 0.8f, 1.0f);
        glBegin(GL_LINES);
        glVertex2f(pos.x - drawWidth, yTop);
        glVertex2f(pos.x + drawWidth, yTop);
        glEnd();
        glLineWidth(1.0f);
    }

    void renderSpike() {
        if (!spike_ || spikeTexture_ == 0) {
            return;
        }

        const Vec3 physicsPos = spike_->getPosition();
        spikePosition_.x = physicsPos.x;
        spikePosition_.y = physicsPos.y;

        renderTexturedQuad(
                spikeTexture_,
                spikePosition_.x,
                spikePosition_.y,
                spikeWidth_,
                spikeHeight_,
                nullptr,
                0.0f // Sin rotación para el spike
        );
    }

    /**
     * ⭐⭐⭐ FUNCIÓN MODIFICADA: Ahora usa playerRotationAngle_
     */
    void renderPlayerOpenGL() {
        if (playerGLTexture_ == 0)
            return;

        auto playerTexture = getTextureResource(playerTextureHandle_);
        if (!playerTexture)
            return;

        const auto texWidth = static_cast<float>(playerTexture->getWidth());
        const auto texHeight = static_cast<float>(playerTexture->getHeight());

        const float u_min = currentSpriteFrame_.x / texWidth;
        const float v_min = currentSpriteFrame_.y / texHeight;
        const float u_max = static_cast<float>(currentSpriteFrame_.x + currentSpriteFrame_.w) / texWidth;
        const float v_max = static_cast<float>(currentSpriteFrame_.y + currentSpriteFrame_.h) / texHeight;

        const float texCoords[4] = {u_min, v_min, u_max, v_max};

        const float worldUnitPerPixel = WORLD_WIDTH / static_cast<float>(WINDOW_WIDTH);
        const float spriteWorldWidth = currentSpriteFrame_.w * worldUnitPerPixel;
        const float spriteWorldHeight = currentSpriteFrame_.h * worldUnitPerPixel;

        // ⭐ PASAMOS EL ÁNGULO DE ROTACIÓN
        renderTexturedQuad(
                playerGLTexture_,
                playerPosition_.x,
                playerPosition_.y,
                spriteWorldWidth,
                spriteWorldHeight,
                texCoords,
                playerRotationAngle_ // ← NUEVO parámetro
        );
    }

    void renderPhysicsDebug() const {
        if (!player_)
            return;

        const Vec3 playerPos = player_->getPosition();

        if (const Vec3 velocity = player_->getLinearVelocity();
            std::abs(velocity.x) > 0.1f || std::abs(velocity.y) > 0.1f) {
            glColor3f(1.0f, 1.0f, 0.0f);
            glLineWidth(2.0f);
            glBegin(GL_LINES);
            glVertex2f(playerPos.x, playerPos.y);
            glVertex2f(playerPos.x + velocity.x * 0.2f, playerPos.y + velocity.y * 0.2f);
            glEnd();
            glLineWidth(1.0f);
        }

        float spriteHalfSize = 1.2f;

        glColor3f(0.0f, 1.0f, 0.0f);
        glBegin(GL_LINE_LOOP);
        glVertex2f(playerPos.x - spriteHalfSize, playerPos.y - spriteHalfSize);
        glVertex2f(playerPos.x + spriteHalfSize, playerPos.y - spriteHalfSize);
        glVertex2f(playerPos.x + spriteHalfSize, playerPos.y + spriteHalfSize);
        glVertex2f(playerPos.x - spriteHalfSize, playerPos.y + spriteHalfSize);
        glEnd();

        glColor3f(1.0f, 1.0f, 1.0f);
    }

    Vec2 getCameraPosition() const {
        if (const BaseCamera* activeCamera = cameraManager_->getActiveCamera()) {
            Vec3 pos3D = activeCamera->getPosition();
            Vec2 cameraWorldPos(pos3D.x, pos3D.y);
            Vec2 renderOffset(
                    cameraWorldPos.x - WINDOW_WIDTH / 2.0f,
                    cameraWorldPos.y - WINDOW_HEIGHT / 2.0f);
            return renderOffset;
        }
        return Vec2{0.0f, 0.0f};
    }

    void showStats(int frameCount, std::chrono::steady_clock::duration elapsed) const {
        const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
        const double fps = frameCount * 1000.0 / elapsedMs;

        std::cout << "\n--- Game Performance Stats ---" << std::endl;
        std::cout << "FPS: " << fps << std::endl;
        std::cout << "Player Position: (" << playerPosition_.x << ", " << playerPosition_.y << ")" << std::endl;
        std::cout << "Player Rotation: " << playerRotationAngle_ << "° (Rotating: "
                  << (isRotating_ ? "YES" : "NO") << ")" << std::endl;

        if (player_) {
            Vec3 vel = player_->getLinearVelocity();
            std::cout << "Player Velocity: (" << vel.x << ", " << vel.y << ")" << std::endl;
        }
    }

    void printControls() {
        std::cout << "=== GAME CONTROLS ===" << std::endl;
        std::cout << "SPACE: Jump + Rotate 180°" << std::endl;
        std::cout << "A/D: Move left/right" << std::endl;
        std::cout << "R: Reset player" << std::endl;
        std::cout << "X: Toggle debug" << std::endl;
        std::cout << "ESC: Quit" << std::endl;
        std::cout << "=====================" << std::endl;
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

        if (spikeTexture_ != 0) {
            glDeleteTextures(1, &spikeTexture_);
            spikeTexture_ = 0;
            std::cout << "✓ Spike texture cleaned up" << std::endl;
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

        if (physicsSystem_) {
            std::cout << "[PhysicsSystem] Shutting down before memory manager..." << std::endl;
            physicsSystem_->shutdown();
            physicsSystem_.reset();
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
        IDLE,
        UP,
        DOWN,
        LEFT,
        RIGHT
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
    engine::physics::RigidBody* spike_;

    Vec2 playerPosition_;
    SDL_Rect currentSpriteFrame_;

    // ============================================================================
    // ⭐⭐⭐ NUEVAS VARIABLES PARA ROTACIÓN
    // ============================================================================
    float playerRotationAngle_ = 0.0f; ///< Ángulo actual de rotación en GRADOS
    bool isRotating_ = false;          ///< ¿Está el player rotando actualmente?
    float rotationProgress_ = 0.0f;    ///< Progreso de 0.0 a 1.0 (0% a 100%)
    float rotationStartAngle_ = 0.0f;  ///< Ángulo cuando empezó la rotación
    float rotationSpeed_ = 1.0f;       ///< Velocidad de rotación (mayor = más rápido)

    GLuint spikeTexture_ = 0;
    Vec2 spikePosition_;
    float spikeWidth_ = 2.0f;
    float spikeHeight_ = 2.0f;

    CameraID mainCameraId_ = INVALID_CAMERA_ID;
    CameraID freeCameraId_ = INVALID_CAMERA_ID;

    bool showGrid_ = false;
    bool showDebugInfo_ = true;

    bool initializeOpenGL() {
        glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
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
        constexpr float groundWidth = WORLD_WIDTH * 50.0f;
        const auto groundShape = engine::physics::ShapeCreationParams::Box(Vec3(groundWidth, 0.5f, 1.0f));

        engine::physics::BodyCreationParams groundParams = engine::physics::BodyCreationParams::StaticBody(
                groundShape,
                nullptr,
                0.1f);
        groundParams.name = "Ground";

        groundParams.material = engine::physics::PhysicsMaterial::Concrete();
        groundParams.material.friction = 1.0f;
        groundParams.material.restitution = 0.0f;
        groundParams.material.rollingFriction = 0.5f;

        ground_ = physicsSystem_->createRigidBody(groundParams);

        if (!ground_) {
            std::cerr << "Failed to create ground body" << std::endl;
            return false;
        }

        ground_->setPosition(Vec3(WORLD_WIDTH / 2, 1.0f, 0));

        std::cout << "✓ Ground positioned at: ("
                  << ground_->getPosition().x << ", "
                  << ground_->getPosition().y << ", "
                  << ground_->getPosition().z << ")" << std::endl;

        constexpr float boxSize = 0.9f;
        const auto boxShape = engine::physics::ShapeCreationParams::Box(Vec3(boxSize));
        constexpr float adjustedMass = 1.0f;

        engine::physics::BodyCreationParams boxParams = engine::physics::BodyCreationParams::DynamicBody(
                boxShape,
                adjustedMass,
                nullptr,
                5.0f);

        boxParams.name = "Player";
        boxParams.material = engine::physics::PhysicsMaterial::Metal();
        boxParams.material.friction = 0.0f;
        boxParams.material.restitution = 0.0f;
        boxParams.material.rollingFriction = 0.1f;

        boxParams.enableCCD = true;

        player_ = physicsSystem_->createRigidBody(boxParams);

        if (!player_) {
            std::cerr << "Failed to create player body" << std::endl;
            return false;
        }

        player_->setDamping(0.3f, 0.3f);
        player_->setPosition(Vec3(3.0f, 2.5f, 0));
        player_->setAngularFactor(Vec3(0, 0, 0));
        player_->setSleepingThresholds(0.0f, 0.0f);

        std::cout << "✓ Player positioned at: ("
                  << player_->getPosition().x << ", "
                  << player_->getPosition().y << ", "
                  << player_->getPosition().z << ")" << std::endl;

        const Vec3 playerPos = player_->getPosition();
        const Vec3 groundPos = ground_->getPosition();
        const float distanceY = playerPos.y - groundPos.y;

        std::cout << "\n=== Physics Setup Verification ===" << std::endl;
        std::cout << "Player Y: " << playerPos.y << std::endl;
        std::cout << "Ground Y: " << groundPos.y << std::endl;
        std::cout << "Vertical Distance: " << distanceY << " units" << std::endl;

        if (distanceY > 0) {
            std::cout << "✅ Player is ABOVE ground - will fall and collide" << std::endl;
        } else {
            std::cout << "❌ WARNING: Player is BELOW ground!" << std::endl;
        }

        std::cout << "\n=== Creating Spike Obstacle ===" << std::endl;

        const float spikeHalfWidth = spikeWidth_ / 2.0f;
        const float spikeHalfHeight = spikeHeight_ / 2.0f;

        const auto spikeShape = engine::physics::ShapeCreationParams::Box(
                Vec3(spikeHalfWidth, spikeHalfHeight, 0.5f));

        engine::physics::BodyCreationParams spikeParams =
                engine::physics::BodyCreationParams::StaticBody(
                        spikeShape,
                        nullptr,
                        0.1f);

        spikeParams.name = "Spike";
        spikeParams.material = engine::physics::PhysicsMaterial::Concrete();
        spikeParams.material.friction = 1.0f;
        spikeParams.material.restitution = 0.0f;
        spikeParams.material.rollingFriction = 0.5f;

        spike_ = physicsSystem_->createRigidBody(spikeParams);

        if (!spike_) {
            std::cerr << "Failed to create spike body" << std::endl;
            return false;
        }

        const Vec3 spikeWorldPos(10.0f, 2.5f, 0.0f);
        spike_->setPosition(spikeWorldPos);

        spikePosition_.x = spikeWorldPos.x;
        spikePosition_.y = spikeWorldPos.y;

        std::cout << "✓ Spike created and positioned" << std::endl;

        return true;
    }

    /**
     * ⭐⭐⭐ FUNCIÓN MODIFICADA: Ahora acepta un parámetro de rotación
     *
     * @param texture - ID de la textura OpenGL
     * @param x - Posición X en el mundo
     * @param y - Posición Y en el mundo
     * @param width - Ancho del quad
     * @param height - Alto del quad
     * @param texCoords - Coordenadas de textura (opcional)
     * @param rotationDegrees - Ángulo de rotación en GRADOS (⭐ NUEVO)
     */
    void renderTexturedQuad(
            GLuint texture,
            float x,
            float y,
            float width,
            float height,
            const float* texCoords = nullptr,
            float rotationDegrees = 0.0f // ⭐ NUEVO parámetro
    ) {
        float defaultTexCoords[4] = {0.0f, 0.0f, 1.0f, 1.0f};
        const float* tc = texCoords ? texCoords : defaultTexCoords;

        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, texture);

        glPushMatrix();

        // ⭐ PASO 1: Mover al punto donde queremos dibujar
        glTranslatef(x, y, 0.0f);

        // ⭐ PASO 2: Rotar alrededor del CENTRO (sentido horario)
        // Nota: OpenGL rota en sentido ANTIHORARIO por defecto,
        // así que usamos -rotationDegrees para obtener rotación HORARIA
        glRotatef(-rotationDegrees, 0.0f, 0.0f, 1.0f);

        // ⭐ PASO 3: Dibujar el quad centrado en (0,0)
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
                surface->pixels);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        return textureID;
    }
};

int main() {
    std::cout << "=== Game Demo - With 180° Jump Rotation ===" << std::endl;
    std::cout << "Press SPACE to jump and rotate!" << std::endl;
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
