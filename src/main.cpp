// MAIN TEST

#include <SDL.h>
#include <SDL_image.h>
#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <cmath>

#include "./input/InputSystem.h"

#include "./memory/MemorySystem.h"
#include "./resources/manager/ResourceManager.h"
#include "./resources/types/texture/TextureResource.h"

#include "./camera/CameraSystem.h"

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
 */
class GameDemo {
public:
    GameDemo() = default;
    ~GameDemo() = default;

    static constexpr int WINDOW_WIDTH = 1280;   // Más ancho para ver más escena
    static constexpr int WINDOW_HEIGHT = 720;   // Relación 16:9 estándar
    static constexpr float WORLD_WIDTH = 30.0f; // Mundo más ancho (scroll horizontal)
    static constexpr float WORLD_HEIGHT = 13.5f;
    static constexpr float PLAYER_MOVE_FORCE = 1000.0f;
    static constexpr float CAMERA_SPEED = 8.0f;
    static constexpr float GRAVITY_FORCE = -30.0f; // Gravedad pesada (caída rápida)

    enum Actions : ActionID {
        ACTION_JUMP         = 1, ACTION_MOVE_BACKWARD = 2, ACTION_MOVE_LEFT     = 3,
        ACTION_MOVE_RIGHT   = 4, ACTION_EXIT          = 5, ACTION_SWITCH_CAMERA = 6,
        ACTION_ZOOM_IN      = 7, ACTION_ZOOM_OUT      = 8, ACTION_CAMERA_SHAKE  = 9,
        ACTION_CAMERA_UP    = 10, ACTION_CAMERA_DOWN  = 11, ACTION_CAMERA_LEFT  = 12,
        ACTION_CAMERA_RIGHT = 13, ACTION_TOGGLE_GRID  = 14, ACTION_TOGGLE_DEBUG = 15,
        ACTION_PLAYER_MOVE  = 20, ACTION_CAMERA_MOVE  = 21, ACTION_RESET_PLAYER = 22,
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

        // Set camera to follow player
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
            std::string("../assets/player_02-uhd.png"),
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

    void updateGameLogic(float deltaTime) {
        if (!player_)
            return;

        // =========================================================
        // 1. DETECCIÓN DE SUELO (Ajustada a tu nuevo tamaño)
        // =========================================================
        Vec3 playerPos = player_->getPosition();
        Vec3 groundPos = ground_->getPosition();

        float playerHalfHeight = 0.8f; // Un poco menos que el físico para tolerancia
        float groundHalfHeight = 0.5f;

        float expectedDistance = groundHalfHeight + playerHalfHeight;
        float distanceToGround = std::abs(playerPos.y - groundPos.y - expectedDistance);

        // Umbral de tolerancia
        bool isGrounded = distanceToGround < 0.2f;

        // =========================================================
        // 2. OBTENER VELOCIDAD ACTUAL
        // =========================================================
        Vec3 currentVel = player_->getLinearVelocity();

        // =========================================================
        // 3. LÓGICA DE MOVIMIENTO HORIZONTAL (Eje X)
        // =========================================================
        float targetSpeedX = 0.0f;
        float moveSpeed = 15.0f; // Velocidad máxima al correr

        float inputDir = 0.0f;
        if (inputSystem_->getActionValue(ACTION_MOVE_LEFT) > 0.5f)
            inputDir = -1.0f;
        if (inputSystem_->getActionValue(ACTION_MOVE_RIGHT) > 0.5f)
            inputDir = 1.0f;

        targetSpeedX = inputDir * moveSpeed;

        // Aquí está la MAGIA del movimiento (Interpolación)
        if (isGrounded) {
            // EN EL SUELO: Tenemos tracción total.
            // Nos movemos rápidamente hacia la velocidad objetivo (acelerar/frenar rápido)
            // 10.0f * deltaTime es el factor de "agarre"
            float accelerationGround = 10.0f;
            currentVel.x += (targetSpeedX - currentVel.x) * accelerationGround * deltaTime;
        } else {
            // EN EL AIRE: Tenemos inercia.
            // Si el jugador NO toca nada (inputDir == 0), NO frenamos (conserva inercia).
            // Si el jugador toca teclas, le damos un poco de control (pero lento).

            if (inputDir != 0.0f) {
                float accelerationAir = 2.0f; // Mucho menor que en el suelo

                // Solo aplicamos fuerza si no excedemos la velocidad máxima
                // o si estamos intentando ir en contra de la inercia actual
                bool movingAgainstInertia = (currentVel.x * inputDir < 0);

                if (std::abs(currentVel.x) < moveSpeed || movingAgainstInertia) {
                    currentVel.x += (targetSpeedX - currentVel.x) * accelerationAir * deltaTime;
                }
            }

            // Opcional: Rozamiento del aire muy leve (para que no vuele infinitamente si le pegan)
            currentVel.x *= 0.995f;
        }

        // =========================================================
        // 4. LÓGICA DE SALTO (Impulso Instantáneo)
        // =========================================================
        if (inputSystem_->getActionValue(ACTION_JUMP) > 0.5f && isGrounded) {
            // Reiniciamos la velocidad Y para que el salto sea consistente
            // (evita que salte menos si estaba bajando una pendiente)
            currentVel.y = 0.0f;

            // IMPORTANTE: Primero seteamos la velocidad limpia
            player_->setLinearVelocity(currentVel);

            // Luego aplicamos el IMPULSO hacia arriba (método existente en RigidBody.h)
            float jumpForce = 20.0f; // Ajustar según peso (mass)
            player_->applyImpulse(Vec3(0, jumpForce, 0));

            // Marcamos grounded falso manual para evitar doble salto en el mismo frame
            isGrounded = false;

            // Salimos temprano para dejar que la física procese el impulso
            // (o actualizamos currentVel.y manualmente si queremos seguir lógica abajo)
            return;
        }

        // =========================================================
        // 5. APLICAR CAMBIOS
        // =========================================================

        // Solo activamos si hay movimiento significativo para ahorrar CPU
        if (std::abs(currentVel.x) > 0.01f || std::abs(currentVel.y) > 0.01f) {
            player_->activate(true);
        }

        // Guardamos la nueva velocidad calculada
        player_->setLinearVelocity(currentVel);

        // =========================================================
        // 6. CÁMARA (Sin cambios)
        // =========================================================
        if (Camera2D* followCamera = cameraManager_->getCamera2D(mainCameraId_); followCamera && followCamera->getMode() == CameraMode::FOLLOW_TARGET) {
            followCamera->setTarget(playerPos);
            followCamera->setSmoothingSpeed(0.0f);
            followCamera->setPosition(playerPos);
        }
    }

    void syncPlayerPositionFromPhysics() {
        if (!player_) {
            return;
        }

        const Vec3 physicsPos = player_->getPosition();
        playerPosition_.x = physicsPos.x;
        playerPosition_.y = physicsPos.y;
        // Z is ignored for 2D rendering
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
        // glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

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
                1.0f
            );
        } else {
            // Fallback: Standard orientation (Y+ = up)
            glOrtho(0, WORLD_WIDTH, 0, WORLD_HEIGHT, -1.0f, 1.0f);
        }

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        renderBackgroundGrid();

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

    void renderBackgroundGrid() {
        // 1. Color de fondo (Azul/Morado oscuro estilo Stereo Madness)
        // R, G, B, Alpha
        glClearColor(0.1f, 0.0f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 2. Dibujar Grilla
        glEnable(GL_BLEND);
        glLineWidth(1.0f);
        glColor4f(1.0f, 1.0f, 1.0f, 0.15f); // Blanco transparente suave

        float camX = 0;
        float camY = 0;

        // Obtenemos posición de cámara para efecto parallax simple
        if (auto* cam = cameraManager_->getCamera2D(cameraManager_->getActiveCameraId())) {
            camX = cam->getPosition2D().x;
            camY = cam->getPosition2D().y;
        }

        // Dibujamos líneas verticales y horizontales infinitas basadas en la cámara
        float spacing = 1.0f; // 1 unidad de mundo = 1 bloque
        int gridCount = 40;   // Cuantas lineas dibujar alrededor de la cámara

        glBegin(GL_LINES);
        // Verticales
        float startX = std::floor(camX) - gridCount / 2;
        for (int i = 0; i < gridCount; i++) {
            glVertex2f(startX + i * spacing, camY - 20);
            glVertex2f(startX + i * spacing, camY + 20);
        }
        // Horizontales
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

        // CAMBIO AQUÍ: Usamos un valor grande para dibujar, igual que en la física
        // O puedes usar una constante como 1000.0f para asegurar que cubra todo
        float drawWidth = WORLD_WIDTH * 50.0f;

        // 1. Relleno oscuro debajo del suelo
        glColor3f(0.05f, 0.0f, 0.2f);
        glBegin(GL_QUADS);
        glVertex2f(pos.x - drawWidth, yTop - 10.0f);
        glVertex2f(pos.x + drawWidth, yTop - 10.0f);
        glVertex2f(pos.x + drawWidth, yTop);
        glVertex2f(pos.x - drawWidth, yTop);
        glEnd();

        // 2. Línea brillante superior
        glLineWidth(3.0f);
        glColor3f(0.0f, 0.8f, 1.0f);
        glBegin(GL_LINES);
        glVertex2f(pos.x - drawWidth, yTop);
        glVertex2f(pos.x + drawWidth, yTop);
        glEnd();
        glLineWidth(1.0f);
    }

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

        renderTexturedQuad(
            playerGLTexture_,
            playerPosition_.x,
            playerPosition_.y,
            spriteWorldWidth,
            spriteWorldHeight,
            texCoords
        );
    }

    void renderPhysicsDebug() const {
        if (!player_)
            return;

        const Vec3 playerPos = player_->getPosition();

        // Draw velocity vector
        if (const Vec3 velocity = player_->getLinearVelocity(); std::abs(velocity.x) > 0.1f || std::abs(velocity.y) > 0.1f) {
            glColor3f(1.0f, 1.0f, 0.0f); // Yellow
            glLineWidth(2.0f);
            glBegin(GL_LINES);
            glVertex2f(playerPos.x, playerPos.y);
            glVertex2f(playerPos.x + velocity.x * 0.2f, playerPos.y + velocity.y * 0.2f);
            glEnd();
            glLineWidth(1.0f);
        }

        // Draw physics bounding box (cube extents = 0.5)
        float spriteHalfSize = 1.2f; ///< this for player_447-uhd.png,m for player-cube.png use 0.2f, for player_02-uhd.png

        glColor3f(0.0f, 1.0f, 0.0f); // Verde
        glBegin(GL_LINE_LOOP);
        glVertex2f(playerPos.x - spriteHalfSize, playerPos.y - spriteHalfSize);
        glVertex2f(playerPos.x + spriteHalfSize, playerPos.y - spriteHalfSize);
        glVertex2f(playerPos.x + spriteHalfSize, playerPos.y + spriteHalfSize);
        glVertex2f(playerPos.x - spriteHalfSize, playerPos.y + spriteHalfSize);
        glEnd();

        glColor3f(1.0f, 1.0f, 1.0f); // Reset
    }

    Vec2 getCameraPosition() const {
        if (const BaseCamera* activeCamera = cameraManager_->getActiveCamera()) {
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

    void showStats(int frameCount, std::chrono::steady_clock::duration elapsed) const {
        const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
        const double fps = frameCount * 1000.0 / elapsedMs;

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
        constexpr float groundWidth = WORLD_WIDTH * 50.0f;
        const auto groundShape = engine::physics::ShapeCreationParams::Box(Vec3(groundWidth, 0.5f, 1.0f));

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
        constexpr float boxSize = 0.9f;

        const auto boxShape = engine::physics::ShapeCreationParams::Box(Vec3(boxSize)); // Con 0.2f queda bien, pero no se mueve.

        // constexpr float minMass = 5.0f; // Masa mínima para asegurar que no sea demasiado ligero
        constexpr float originalSize = 0.5f;
        constexpr float volumeRatio = (boxSize * boxSize * boxSize) / (originalSize * originalSize * originalSize);
        // constexpr float adjustedMass = std::max(minMass, 10.0f * volumeRatio); // Masa base de 10kg con mínimo de 5kg
        constexpr float adjustedMass = 1.0f;

        engine::physics::BodyCreationParams boxParams = engine::physics::BodyCreationParams::DynamicBody(
            boxShape,
            adjustedMass,
            nullptr,
            5.0f
        );

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
        player_->setPosition(Vec3(3.0f, 1.0f, 0));
        player_->setAngularFactor(Vec3(0, 0, 0));
        player_->setSleepingThresholds(0.0f, 0.0f);

        std::cout << "✓ Player positioned at: ("
                << player_->getPosition().x << ", "
                << player_->getPosition().y << ", "
                << player_->getPosition().z << ")" << std::endl;

        // ============================================================================
        // VERIFICATION
        // ============================================================================
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
