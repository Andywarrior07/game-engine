// MAIN TEST

#include <SDL.h>
#include <SDL_image.h>
#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <cmath>

// CHANGE: Removed old resource manager includes
// #include "./Resource/ResourceManager.h"
// #include "./Texture/TextureResource.h"

// CHANGE: Added new resource and memory management includes
#include "./memory/MemorySystem.h"
#include "./resources/manager/ResourceManager.h"
#include "./resources/types/texture/TextureResource.h"

#include "./Input/InputManager.h"
// CHANGE: Removed animation manager as it depends on old system
// #include "./Animation/AnimationManager.h"
#include "./camera/CameraSystem.h"
// #include "./Camera-old/CameraManager.h"
#include "Animation/AnimationManager.h"
// #include "memory/manager/MemoryManager.h"

// CHANGE: Updated namespaces - removed old resources namespace, added new ones
using namespace engine::memory;
using namespace engine::resources; // Now points to new resource system
using namespace engine::input;
using namespace engine::math;
// using namespace engine::animation;
using namespace engine::camera;

/**
 * @brief Main game class demonstrating integrated use of new resource system
 * CHANGE: Complete refactor to use new ResourceManager and MemoryManager
 * Simplified to show sprite rendering without animation system
 */
class GameDemo {
public:
    GameDemo() = default;
    ~GameDemo() = default;

    static constexpr int WORLD_WIDTH = 4096;
    static constexpr int WORLD_HEIGHT = 3072;

    enum class PlayerAnimation {
        IDLE,
        UP,
        DOWN,
        LEFT,
        RIGHT
    };

    /**
     * @brief Initialize SDL, managers and resources
     * CHANGE: Added MemoryManager and new ResourceManager initialization
     * Removed AnimationManager dependencies
     * @return true if initialization was successful
     */
    bool initialize() {
        // CHANGE: Step 0 - Initialize MemoryManager FIRST
        std::cout << "=== Initializing Memory Manager ===" << std::endl;

        MemoryManagerAutoConfig memConfig;
        memConfig.autoDetectLimits = true;
        memConfig.memoryUsagePercent = 0.10f; // Use 10% of system memory
        memConfig.heapSizePercent = 0.25f; // 25% for main heap
        // CHANGE: Disable expensive debug features in Release mode or with flag
#ifdef _DEBUG
        // Check environment variable for quick disable
        const char* quickShutdown = std::getenv("GAME_QUICK_SHUTDOWN");
        if (quickShutdown && std::string(quickShutdown) == "1") {
            std::cout << "Quick shutdown mode enabled - disabling memory checks" << std::endl;
            memConfig.enableLeakDetection = false;
            memConfig.enableBoundsChecking = false;
        }
        else {
            memConfig.enableLeakDetection = true;
            memConfig.enableLeakDetection = true;
            memConfig.enableBoundsChecking = true;
            memConfig.enableBoundsChecking = true;
            std::cout << "Debug mode: Leak detection and bounds checking enabled" << std::endl;
            std::cout << "  (Set GAME_QUICK_SHUTDOWN=1 to disable for faster exit)" << std::endl;
        }
#else
        memConfig.enableLeakDetection = false;
        memConfig.enableBoundsChecking = false;
        std::cout << "Release mode: Memory checks disabled for performance" << std::endl;
#endif

        memoryManager_ = std::make_unique<MemoryManager>();
        if (!memoryManager_->initialize(memConfig)) {
            std::cerr << "Failed to initialize MemoryManager" << std::endl;
            return false;
        }

        std::cout << "✓ MemoryManager initialized successfully" << std::endl;
        std::cout << "  Total memory allocated: " << (memoryManager_->getTotalMemoryUsage() / 1024.0 / 1024.0) << " MB"
            << std::endl;

        // 1. INITIALIZE SDL
        if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) < 0) {
            std::cerr << "Failed to initialize SDL: " << SDL_GetError() << std::endl;
            return false;
        }

        // 2. INITIALIZE SDL_IMAGE
        int imgFlags = IMG_INIT_PNG | IMG_INIT_JPG;
        if (!(IMG_Init(imgFlags) & imgFlags)) {
            std::cerr << "Failed to initialize SDL_image: " << IMG_GetError() << std::endl;
            SDL_Quit();
            return false;
        }

        // 3. CREATE WINDOW
        window_ = SDL_CreateWindow("Game Demo - New Resource System",
                                   SDL_WINDOWPOS_CENTERED,
                                   SDL_WINDOWPOS_CENTERED,
                                   WINDOW_WIDTH, WINDOW_HEIGHT,
                                   SDL_WINDOW_SHOWN);

        if (!window_) {
            std::cerr << "Failed to create window: " << SDL_GetError() << std::endl;
            cleanup();
            return false;
        }

        // 4. CREATE RENDERER
        renderer_ = SDL_CreateRenderer(window_, -1,
                                       SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

        if (!renderer_) {
            std::cerr << "Failed to create renderer: " << SDL_GetError() << std::endl;
            cleanup();
            return false;
        }

        // CHANGE: Step 5 - Initialize the NEW ResourceManager
        std::cout << "\n=== Initializing Resource Manager ===" << std::endl;

        resourceManager_ = std::make_unique<ResourceManager>(*memoryManager_);

        ResourceManagerConfig resourceConfig;
        resourceConfig.maxMemory = 256 * 1024 * 1024; // 256MB limit
        resourceConfig.enableHotReload = false; // Disable for now
        resourceConfig.loaderThreadCount = 2; // 2 loader threads
        resourceConfig.enableAsyncLoading = false; // Use sync loading for simplicity
        resourceConfig.enableMemoryMapping = false;
        resourceConfig.enableCompression = false;

        if (!resourceManager_->initialize(resourceConfig)) {
            std::cerr << "Failed to initialize ResourceManager" << std::endl;
            cleanup();
            return false;
        }

        // CHANGE: Register TextureResource type with new system
        resourceManager_->registerResourceType<TextureResource>();

        std::cout << "✓ ResourceManager initialized successfully" << std::endl;

        // 6. CONFIGURE AND INITIALIZE INPUTMANAGER
        InputManagerConfig inputConfig;
        inputConfig.enableInputLogging = false;
        inputConfig.enableGamepadHotswap = true;
        inputConfig.maxGamepads = 1;

        inputManager_ = std::make_unique<InputManager>(inputConfig);
        if (!inputManager_->initialize()) {
            std::cerr << "Failed to initialize InputManager" << std::endl;
            cleanup();
            return false;
        }

        // 7. CONFIGURE AND INITIALIZE CAMERAMANAGER
        CameraManagerConfig cameraConfig;
        cameraConfig.enableCameraLogging = true;
        cameraConfig.enableTransitions = true;
        cameraConfig.enableShake = true;
        cameraConfig.maxCameras = 4;
        cameraConfig.defaultSmoothingSpeed = 8.0f;
        cameraConfig.mouseSensitivity = 1.0f;
        cameraConfig.scrollSensitivity = 1.0f;

        cameraManager_ = std::make_unique<CameraManager>(cameraConfig);
        if (!cameraManager_->initialize()) {
            std::cerr << "Failed to initialize CameraManager" << std::endl;
            cleanup();
            return false;
        }

        // 8. CONFIGURE VIEWPORT FOR CAMERA
        Viewport viewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
        cameraManager_->setViewport(viewport);

        // 9. SETUP WORLD AND CAMERA
        if (!setupWorldAndCamera()) {
            std::cerr << "Failed to setup world and camera" << std::endl;
            cleanup();
            return false;
        }

        // 10. SETUP INPUT BINDINGS
        setupInputBindings();

        // CHANGE: 11. LOAD RESOURCES WITH NEW SYSTEM
        if (!loadGameResources()) {
            std::cerr << "Failed to load game resources" << std::endl;
            cleanup();
            return false;
        }

        // CHANGE: Initialize player position
        playerPosition_ = Vec2(WORLD_WIDTH / 2.0f, WORLD_HEIGHT / 2.0f);

        // Set camera to follow player
        Camera2D* mainCamera = cameraManager_->getCamera2D(mainCameraId_);
        if (mainCamera) {
            mainCamera->setTarget(playerPosition_);
            std::cout << "✓ Camera set to follow player at (" << playerPosition_.x
                << ", " << playerPosition_.y << ")" << std::endl;
        }

        std::cout << "\nGame initialized successfully!" << std::endl;
        std::cout << "World size: " << WORLD_WIDTH << "x" << WORLD_HEIGHT << " pixels" << std::endl;
        std::cout << "Viewport: " << WINDOW_WIDTH << "x" << WINDOW_HEIGHT << " pixels" << std::endl;
        return true;
    }

    /**
     * @brief Setup world and camera system
     */
    bool setupWorldAndCamera() {
        // 1. CREATE MAIN CAMERA (2D for this demo)
        mainCameraId_ = cameraManager_->createCamera2D("MainCamera");
        if (mainCameraId_ == INVALID_CAMERA_ID) {
            std::cerr << "Failed to create main camera" << std::endl;
            return false;
        }

        // 2. GET CAMERA AND CONFIGURE IT DIRECTLY (no existe setupTopDownCamera)
        Camera2D* mainCamera = cameraManager_->getCamera2D(mainCameraId_);
        if (!mainCamera) {
            std::cerr << "Failed to get main camera" << std::endl;
            return false;
        }

        // 3. SETUP CAMERA FOR TOP-DOWN VIEW
        mainCamera->setMode(CameraMode::TOP_DOWN);
        mainCamera->setPosition(Vec2(WORLD_WIDTH / 2, WORLD_HEIGHT / 2));
        mainCamera->setZoom(1.0f);
        mainCamera->setFollowSpeed(6.0f);
        mainCamera->setZoomLimits(0.5f, 2.0f);
        mainCamera->setSmoothingSpeed(8.0f);

        // 4. CONFIGURE WORLD BOUNDS (sin el tercer parámetro)
        CameraBounds worldBounds(
            Vec3(0.0f, 0.0f, -1.0f),
            Vec3(WORLD_WIDTH, WORLD_HEIGHT, 1.0f)
        );
        worldBounds.setEnabled(true); // Asegurarse de que estén habilitados

        mainCamera->setBounds(worldBounds);
        mainCamera->setMode(CameraMode::FOLLOW_TARGET);

        std::cout << "✓ Main camera configured with world bounds" << std::endl;

        // 5. ACTIVATE MAIN CAMERA
        if (!cameraManager_->setActiveCamera(mainCameraId_)) {
            std::cerr << "Failed to set active camera" << std::endl;
            return false;
        }

        // 6. CREATE FREE CAMERA FOR TESTING
        freeCameraId_ = cameraManager_->createCamera2D("FreeCamera");
        if (freeCameraId_ != INVALID_CAMERA_ID) {
            Camera2D* freeCamera = cameraManager_->getCamera2D(freeCameraId_);
            if (freeCamera) {
                freeCamera->setPosition(Vec2(WORLD_WIDTH / 2, WORLD_HEIGHT / 2));
                freeCamera->setBounds(worldBounds);
                freeCamera->setMode(CameraMode::STATIC);
                std::cout << "✓ Free camera created for manual control" << std::endl;
            }
        }

        return true;
    }

    /**
     * @brief Setup input bindings
     */
    void setupInputBindings() {
        // === PLAYER MOVEMENT ACTIONS ===
        moveUpAction_ = inputManager_->createAction("MoveUp");
        moveDownAction_ = inputManager_->createAction("MoveDown");
        moveLeftAction_ = inputManager_->createAction("MoveLeft");
        moveRightAction_ = inputManager_->createAction("MoveRight");
        exitAction_ = inputManager_->createAction("Exit");

        // === CAMERA ACTIONS ===
        switchCameraAction_ = inputManager_->createAction("SwitchCamera");
        zoomInAction_ = inputManager_->createAction("ZoomIn");
        zoomOutAction_ = inputManager_->createAction("ZoomOut");
        shakeAction_ = inputManager_->createAction("CameraShake");

        // Bind WASD keys for player movement
        inputManager_->bindKeyToAction(moveUpAction_, SDLK_w);
        inputManager_->bindKeyToAction(moveDownAction_, SDLK_s);
        inputManager_->bindKeyToAction(moveLeftAction_, SDLK_a);
        inputManager_->bindKeyToAction(moveRightAction_, SDLK_d);
        inputManager_->bindKeyToAction(exitAction_, SDLK_ESCAPE);

        // Bind camera controls
        inputManager_->bindKeyToAction(switchCameraAction_, SDLK_c);
        inputManager_->bindKeyToAction(zoomInAction_, SDLK_EQUALS);
        inputManager_->bindKeyToAction(zoomOutAction_, SDLK_MINUS);
        inputManager_->bindKeyToAction(shakeAction_, SDLK_SPACE);

        // Create axes for smooth movement
        AxisConfig axisConfig;
        axisConfig.deadZone = 0.1f;
        axisConfig.sensitivity = 1.0f;

        horizontalAxis_ = inputManager_->createAxis("Horizontal", axisConfig);
        verticalAxis_ = inputManager_->createAxis("Vertical", axisConfig);

        inputManager_->bindKeysToAxis(horizontalAxis_, SDLK_d, SDLK_a);
        inputManager_->bindKeysToAxis(verticalAxis_, SDLK_w, SDLK_s);

        // === MANUAL CAMERA CONTROLS (ARROWS) ===
        cameraHorizontalAxis_ = inputManager_->createAxis("CameraHorizontal", axisConfig);
        cameraVerticalAxis_ = inputManager_->createAxis("CameraVertical", axisConfig);

        inputManager_->bindKeysToAxis(cameraHorizontalAxis_, SDLK_RIGHT, SDLK_LEFT);
        inputManager_->bindKeysToAxis(cameraVerticalAxis_, SDLK_UP, SDLK_DOWN);

        // Bind gamepad if available
        if (inputManager_->isGamepadConnected(0)) {
            inputManager_->bindGamepadButtonToAction(moveUpAction_, 0, SDL_CONTROLLER_BUTTON_DPAD_UP);
            inputManager_->bindGamepadButtonToAction(moveDownAction_, 0, SDL_CONTROLLER_BUTTON_DPAD_DOWN);
            inputManager_->bindGamepadButtonToAction(moveLeftAction_, 0, SDL_CONTROLLER_BUTTON_DPAD_LEFT);
            inputManager_->bindGamepadButtonToAction(moveRightAction_, 0, SDL_CONTROLLER_BUTTON_DPAD_RIGHT);

            inputManager_->bindGamepadAxisToAxis(horizontalAxis_, 0, GamepadAxis::LEFT_STICK_X);
            inputManager_->bindGamepadAxisToAxis(verticalAxis_, 0, GamepadAxis::LEFT_STICK_Y);

            inputManager_->bindGamepadAxisToAxis(cameraHorizontalAxis_, 0, GamepadAxis::RIGHT_STICK_X);
            inputManager_->bindGamepadAxisToAxis(cameraVerticalAxis_, 0, GamepadAxis::RIGHT_STICK_Y);

            inputManager_->bindGamepadButtonToAction(switchCameraAction_, 0, SDL_CONTROLLER_BUTTON_Y);
            inputManager_->bindGamepadButtonToAction(shakeAction_, 0, SDL_CONTROLLER_BUTTON_X);
        }

        std::cout << "Input bindings configured successfully!" << std::endl;
    }

    /**
 * @brief Helper method to get texture resource from handle
 * CHANGE: Added helper to work around ResourceHandle.get() implementation
 */
    ResourcePtr<TextureResource> getTextureResource(const ResourceHandle<TextureResource>& handle) {
        if (!handle.isValid()) return nullptr;

        // Try cache first
        if (auto cached = handle.tryGet()) {
            return cached;
        }

        // Get from ResourceManager
        return resourceManager_->getResource<TextureResource>(handle.getId());
    }

    /**
     * @brief Load game resources using new ResourceManager
     * CHANGE: Complete rewrite to use new resource system
     */
    bool loadGameResources() {
        std::cout << "\n=== Loading Game Resources ===" << std::endl;

        // CHANGE: Load player texture using new ResourceManager
        // Generate a unique ID for the resource
        const ResourceID playerTextureId = std::hash<std::string>{}("player_texture");

        // Load the texture synchronously
        playerTextureHandle_ = resourceManager_->loadById<TextureResource>(
            playerTextureId,
            std::string("../assets/player.png"),
            ResourcePriority::HIGH,
            LoadMode::SYNC
        );

        // Wait for resource to be ready and get the actual resource
        auto playerTexture = getTextureResource(playerTextureHandle_);
        if (!playerTexture) {
            std::cerr << "Failed to load player sprite sheet" << std::endl;
            return false;
        }

        playerTextureHandle_.updateCache(playerTexture);

        std::cout << "✓ Player sprite sheet loaded: " << playerTexture->getWidth()
            << "x" << playerTexture->getHeight() << " pixels" << std::endl;

        // CHANGE: Create SDL texture from loaded resource for rendering
        // Since TextureResource stores data in SDL_Surface, we need to create an SDL_Texture
        SDL_Surface* surface = playerTexture->getSDLSurface();
        if (surface) {
            playerSDLTexture_ = SDL_CreateTextureFromSurface(renderer_, surface);
            if (!playerSDLTexture_) {
                std::cerr << "Failed to create SDL texture from surface: " << SDL_GetError() << std::endl;
                return false;
            }
            std::cout << "✓ SDL texture created for rendering" << std::endl;
        }
        else {
            std::cerr << "Failed to get SDL surface from texture resource" << std::endl;
            return false;
        }

        // CHANGE: Setup sprite frame for idle animation (we'll just show one frame)
        // The idle frames are at Y: 0, each frame is 160x140 pixels
        currentSpriteFrame_.x = 0; // First frame
        currentSpriteFrame_.y = 0; // Idle row
        currentSpriteFrame_.w = 160; // Frame width
        currentSpriteFrame_.h = 140; // Frame height

        // CHANGE: Load background texture (commented out for now as requested)
        std::cout << "\n=== Loading Background Texture ===" << std::endl;

        ResourceID backgroundTextureId = std::hash<std::string>{}("background_texture");
        backgroundTextureHandle_ = resourceManager_->loadById<TextureResource>(
            backgroundTextureId,
            "../assets/background.png",  // You'll need to add this asset
            ResourcePriority::NORMAL,
            LoadMode::SYNC
        );

        auto backgroundTexture = backgroundTextureHandle_.get();
        if (backgroundTexture ) {
            SDL_Surface* bgSurface = backgroundTexture->getSDLSurface();
            if (bgSurface) {
                backgroundSDLTexture_ = SDL_CreateTextureFromSurface(renderer_, bgSurface);
                std::cout << "✓ Background texture loaded and created" << std::endl;
            }
        }

        // CHANGE: Print memory usage after loading resources
        std::cout << "\nResource Memory Usage: " << (resourceManager_->getMemoryUsage() / 1024.0 / 1024.0) << " MB" <<
            std::endl;
        std::cout << "Total Memory Usage: " << (memoryManager_->getTotalMemoryUsage() / 1024.0 / 1024.0) << " MB" <<
            std::endl;

        return true;
    }

    /**
     * @brief Main game loop
     */
    void run() {
        bool running = true;

        auto lastStatsTime = std::chrono::steady_clock::now();
        auto lastFrameTime = std::chrono::steady_clock::now();
        int frameCount = 0;

        std::cout << "\n=== Starting Game Loop ===" << std::endl;
        printControls();

        while (running) {
            auto currentTime = std::chrono::steady_clock::now();
            auto deltaTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastFrameTime);
            float deltaTime = deltaTimeMs.count() / 1000.0f;
            lastFrameTime = currentTime;

            // Clamp delta time
            deltaTime = std::min(deltaTime, 0.016f);

            // 1. PROCESS INPUT EVENTS
            inputManager_->processEvents(deltaTime);

            // 2. UPDATE INPUT MANAGER
            inputManager_->update(deltaTime);

            // 3. CHECK EXIT
            if (inputManager_->shouldExit() || inputManager_->isActionPressed(exitAction_)) {
                running = false;
                break;
            }

            // 4. PROCESS CAMERA INPUT
            processCameraInput(deltaTime);

            // 5. UPDATE GAME LOGIC
            updateGameLogic(deltaTime);

            // 6. UPDATE CAMERA
            cameraManager_->update(deltaTime);

            // CHANGE: 7. No animation manager update needed

            // 8. RENDER
            render();

            // 9. SHOW STATS EVERY 3 SECONDS
            frameCount++;
            if (currentTime - lastStatsTime > std::chrono::seconds(3)) {
                showStats(frameCount, currentTime - lastStatsTime);
                frameCount = 0;
                lastStatsTime = currentTime;
            }

            // 10. FRAMERATE CONTROL (60 FPS)
            auto frameTime = std::chrono::steady_clock::now() - currentTime;
            auto targetFrameTime = std::chrono::milliseconds(16);
            if (frameTime < targetFrameTime) {
                std::this_thread::sleep_for(targetFrameTime - frameTime);
            }
        }

        std::cout << "\nGame loop ended" << std::endl;
    }

    /**
     * @brief Process camera-specific input
     */
    void processCameraInput(float deltaTime) {
        // Switch camera modes
        if (inputManager_->isActionPressed(switchCameraAction_)) {
            toggleCameraMode();
        }

        // Zoom
        if (inputManager_->isActionPressed(zoomInAction_)) {
            cameraManager_->processZoom(1.0f);
        }
        if (inputManager_->isActionPressed(zoomOutAction_)) {
            cameraManager_->processZoom(-1.0f);
        }

        // Camera shake
        if (inputManager_->isActionPressed(shakeAction_)) {
            ShakeConfig shakeConfig = ShakeConfig::explosion(3.0f);
            shakeConfig.duration = 0.5f;
            const CameraID cameraId = cameraManager_->getActiveCameraId();
            cameraManager_->startCameraShake(cameraId, shakeConfig);
        }

        // Manual camera control (only in free mode)
        Camera2D* activeCamera = cameraManager_->getCamera2D(cameraManager_->getActiveCameraId());
        if (activeCamera && activeCamera->getMode() == CameraMode::STATIC) {
            float cameraHorizontal = inputManager_->getAxisValue(cameraHorizontalAxis_);
            float cameraVertical = inputManager_->getAxisValue(cameraVerticalAxis_);

            if (std::abs(cameraHorizontal) > 0.1f || std::abs(cameraVertical) > 0.1f) {
                Vec2 currentPos = activeCamera->getPosition2D();
                Vec2 movement(cameraHorizontal * CAMERA_SPEED * deltaTime,
                                 cameraVertical * CAMERA_SPEED * deltaTime);
                activeCamera->setPosition(currentPos + movement);
            }
        }
    }

    /**
     * @brief Toggle between camera modes
     */
    void toggleCameraMode() {
        static bool isFollowMode = true;

        if (isFollowMode) {
            cameraManager_->setActiveCamera(freeCameraId_);
            std::cout << "Camera Mode: FREE CONTROL (use arrow keys)" << std::endl;
        }
        else {
            cameraManager_->setActiveCamera(mainCameraId_);
            std::cout << "Camera Mode: FOLLOW PLAYER" << std::endl;
        }

        isFollowMode = !isFollowMode;
    }

    /**
     * @brief Update game logic
     * CHANGE: Simplified to just move player sprite without animation system
     */
    void updateGameLogic(float deltaTime) {
        // Movement variables
        Vec2 movement{0.0f, 0.0f};
        bool isMoving = false;

        // DETECT MOVEMENT
        if (inputManager_->isActionHeld(moveUpAction_)) {
            movement.y -= PLAYER_SPEED * deltaTime;
            isMoving = true;
            // CHANGE: Update sprite frame to show UP animation frame
            currentSpriteFrame_.x = 0; // First frame
            currentSpriteFrame_.y = 930; // UP animation row
        }
        if (inputManager_->isActionHeld(moveDownAction_)) {
            movement.y += PLAYER_SPEED * deltaTime;
            isMoving = true;
            // CHANGE: Update sprite frame to show DOWN animation frame
            currentSpriteFrame_.x = 0; // First frame
            currentSpriteFrame_.y = 600; // DOWN animation row
        }
        if (inputManager_->isActionHeld(moveLeftAction_)) {
            movement.x -= PLAYER_SPEED * deltaTime;
            isMoving = true;
            // CHANGE: Update sprite frame to show LEFT animation frame
            currentSpriteFrame_.x = 0; // First frame
            currentSpriteFrame_.y = 772; // LEFT animation row
        }
        if (inputManager_->isActionHeld(moveRightAction_)) {
            movement.x += PLAYER_SPEED * deltaTime;
            isMoving = true;
            // CHANGE: Update sprite frame to show RIGHT animation frame
            currentSpriteFrame_.x = 0; // First frame
            currentSpriteFrame_.y = 1068; // RIGHT animation row
        }

        // If not moving, use idle frame
        if (!isMoving) {
            currentSpriteFrame_.x = 0; // First frame
            currentSpriteFrame_.y = 0; // IDLE animation row
        }

        // UPDATE PLAYER POSITION
        if (isMoving) {
            // Normalize diagonal movement
            if (movement.x != 0.0f && movement.y != 0.0f) {
                float length = std::sqrt(movement.x * movement.x + movement.y * movement.y);
                movement.x /= length;
                movement.y /= length;
                movement.x *= PLAYER_SPEED * deltaTime;
                movement.y *= PLAYER_SPEED * deltaTime;
            }

            // Apply movement
            playerPosition_.x += movement.x;
            playerPosition_.y += movement.y;

            // Clamp to world bounds
            const float SPRITE_HALF_WIDTH = 80.0f;
            const float SPRITE_HALF_HEIGHT = 70.0f;

            playerPosition_.x = std::clamp(playerPosition_.x,
                                           SPRITE_HALF_WIDTH,
                                           static_cast<float>(WORLD_WIDTH) - SPRITE_HALF_WIDTH);
            playerPosition_.y = std::clamp(playerPosition_.y,
                                           SPRITE_HALF_HEIGHT,
                                           static_cast<float>(WORLD_HEIGHT) - SPRITE_HALF_HEIGHT);

            // Update camera target
            Camera2D* followCamera = cameraManager_->getCamera2D(mainCameraId_);
            if (followCamera && followCamera->getMode() == CameraMode::FOLLOW_TARGET) {
                followCamera->setTarget(playerPosition_);
            }
        }
    }

    /**
     * @brief Render the scene
     * CHANGE: Simplified rendering without animation manager
     */
    void render() {
        // 1. CLEAR SCREEN
        SDL_SetRenderDrawColor(renderer_, 30, 60, 30, 255);
        SDL_RenderClear(renderer_);

        // 2. GET CAMERA POSITION FOR RENDERING
        Vec2 cameraOffset = getCameraPosition();

        // 3. RENDER WORLD BACKGROUND
        renderWorldBackground(cameraOffset);

        // CHANGE: 4. RENDER PLAYER SPRITE MANUALLY
        renderPlayer(cameraOffset);

        // 5. RENDER DEBUG UI AND HUD
        renderUI();

        // 6. PRESENT TO SCREEN
        SDL_RenderPresent(renderer_);
    }

    /**
     * @brief Render the player sprite
     * CHANGE: New method to render player without animation system
     */
    void renderPlayer(const Vec2& cameraOffset) {
        if (!playerSDLTexture_) return;

        // Calculate screen position from world position
        int screenX = static_cast<int>(playerPosition_.x - cameraOffset.x - currentSpriteFrame_.w / 2);
        int screenY = static_cast<int>(playerPosition_.y - cameraOffset.y - currentSpriteFrame_.h / 2);

        // Destination rectangle on screen
        SDL_Rect destRect = {
            screenX,
            screenY,
            currentSpriteFrame_.w,
            currentSpriteFrame_.h
        };

        // Render the sprite frame
        SDL_RenderCopy(renderer_, playerSDLTexture_, &currentSpriteFrame_, &destRect);
    }

    /**
     * @brief Get current camera position
     */
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

    /**
     * @brief Render world background
     */
    void renderWorldBackground(const Vec2& cameraOffset) {
        const int TILE_SIZE = 64;
        const int GRASS_VARIANT_COUNT = 3;

        // Calculate visible area
        int startX = static_cast<int>(cameraOffset.x / TILE_SIZE);
        int startY = static_cast<int>(cameraOffset.y / TILE_SIZE);
        int endX = static_cast<int>((cameraOffset.x + WINDOW_WIDTH) / TILE_SIZE) + 1;
        int endY = static_cast<int>((cameraOffset.y + WINDOW_HEIGHT) / TILE_SIZE) + 1;

        // Clamp to world bounds
        startX = std::max(0, startX);
        startY = std::max(0, startY);
        endX = std::min(static_cast<int>(WORLD_WIDTH / TILE_SIZE), endX);
        endY = std::min(static_cast<int>(WORLD_HEIGHT / TILE_SIZE), endY);

        // Render visible tiles
        for (int y = startY; y < endY; ++y) {
            for (int x = startX; x < endX; ++x) {
                int worldX = x * TILE_SIZE;
                int worldY = y * TILE_SIZE;

                int screenX = worldX - static_cast<int>(cameraOffset.x);
                int screenY = worldY - static_cast<int>(cameraOffset.y);

                SDL_Rect tileRect = {screenX, screenY, TILE_SIZE, TILE_SIZE};

                // Grass variants based on position
                int variant = (x + y * 7) % GRASS_VARIANT_COUNT;

                switch (variant) {
                case 0:
                    SDL_SetRenderDrawColor(renderer_, 50, 120, 50, 255);
                    break;
                case 1:
                    SDL_SetRenderDrawColor(renderer_, 45, 115, 45, 255);
                    break;
                case 2:
                    SDL_SetRenderDrawColor(renderer_, 55, 125, 55, 255);
                    break;
                }

                SDL_RenderFillRect(renderer_, &tileRect);

                // Add optional grid lines
                if (showGrid_) {
                    SDL_SetRenderDrawColor(renderer_, 40, 100, 40, 128);
                    SDL_RenderDrawRect(renderer_, &tileRect);
                }
            }
        }

        // Render world boundaries
        renderWorldBoundaries(cameraOffset);
    }

    /**
     * @brief Render visual world boundaries
     */
    void renderWorldBoundaries(const Vec2& cameraOffset) {
        SDL_SetRenderDrawColor(renderer_, 100, 50, 50, 255);

        int leftBorder = -static_cast<int>(cameraOffset.x);
        int topBorder = -static_cast<int>(cameraOffset.y);
        int rightBorder = WORLD_WIDTH - static_cast<int>(cameraOffset.x);
        int bottomBorder = WORLD_HEIGHT - static_cast<int>(cameraOffset.y);

        const int BORDER_WIDTH = 5;

        if (leftBorder >= -BORDER_WIDTH && leftBorder <= WINDOW_WIDTH) {
            SDL_Rect leftRect = {leftBorder, 0, BORDER_WIDTH, WINDOW_HEIGHT};
            SDL_RenderFillRect(renderer_, &leftRect);
        }
        if (rightBorder >= 0 && rightBorder <= WINDOW_WIDTH + BORDER_WIDTH) {
            SDL_Rect rightRect = {rightBorder, 0, BORDER_WIDTH, WINDOW_HEIGHT};
            SDL_RenderFillRect(renderer_, &rightRect);
        }
        if (topBorder >= -BORDER_WIDTH && topBorder <= WINDOW_HEIGHT) {
            SDL_Rect topRect = {0, topBorder, WINDOW_WIDTH, BORDER_WIDTH};
            SDL_RenderFillRect(renderer_, &topRect);
        }
        if (bottomBorder >= 0 && bottomBorder <= WINDOW_HEIGHT + BORDER_WIDTH) {
            SDL_Rect bottomRect = {0, bottomBorder, WINDOW_WIDTH, BORDER_WIDTH};
            SDL_RenderFillRect(renderer_, &bottomRect);
        }
    }

    /**
     * @brief Render UI and debug information
     * CHANGE: Updated to show new resource system info
     */
    void renderUI() {
        // === PLAYER STATUS INDICATOR ===
        SDL_Rect statusRect = {10, 10, 200, 30};

        // Color based on current sprite direction
        if (currentSpriteFrame_.y == 0) {
            SDL_SetRenderDrawColor(renderer_, 255, 255, 255, 128); // White - IDLE
        }
        else if (currentSpriteFrame_.y == 930) {
            SDL_SetRenderDrawColor(renderer_, 0, 255, 0, 128); // Green - UP
        }
        else if (currentSpriteFrame_.y == 600) {
            SDL_SetRenderDrawColor(renderer_, 255, 0, 0, 128); // Red - DOWN
        }
        else if (currentSpriteFrame_.y == 772) {
            SDL_SetRenderDrawColor(renderer_, 0, 0, 255, 128); // Blue - LEFT
        }
        else if (currentSpriteFrame_.y == 1068) {
            SDL_SetRenderDrawColor(renderer_, 255, 255, 0, 128); // Yellow - RIGHT
        }
        SDL_RenderFillRect(renderer_, &statusRect);

        // === CAMERA INDICATOR ===
        SDL_Rect cameraRect = {10, 50, 200, 20};
        const BaseCamera* activeCamera = cameraManager_->getActiveCamera();

        if (activeCamera) {
            if (activeCamera->getId() == mainCameraId_) {
                SDL_SetRenderDrawColor(renderer_, 0, 255, 255, 128); // Cyan = Follow mode
            }
            else {
                SDL_SetRenderDrawColor(renderer_, 255, 0, 255, 128); // Magenta = Free mode
            }
        }
        else {
            SDL_SetRenderDrawColor(renderer_, 128, 128, 128, 128); // Grey = No camera
        }
        SDL_RenderFillRect(renderer_, &cameraRect);

        // === ZOOM INDICATOR ===
        const Camera2D* camera2D = cameraManager_->getCamera2D(cameraManager_->getActiveCameraId());
        if (camera2D) {
            float zoom = camera2D->getZoom();
            int zoomBarWidth = static_cast<int>(zoom * 100);
            zoomBarWidth = std::clamp(zoomBarWidth, 10, 300);

            SDL_Rect zoomRect = {10, 80, zoomBarWidth, 15};
            SDL_SetRenderDrawColor(renderer_, 255, 255, 0, 128);
            SDL_RenderFillRect(renderer_, &zoomRect);
        }

        // === MINI MAP ===
        renderMiniMap();

        // === GAMEPAD INDICATOR ===
        if (inputManager_->isGamepadConnected(0)) {
            SDL_Rect gamepadRect = {WINDOW_WIDTH - 50, 10, 40, 20};
            SDL_SetRenderDrawColor(renderer_, 0, 255, 0, 128);
            SDL_RenderFillRect(renderer_, &gamepadRect);
        }

        // === SHAKE INDICATOR ===
        if (cameraManager_->isCameraShaking(cameraManager_->getActiveCameraId())) {
            SDL_Rect shakeRect = {WINDOW_WIDTH - 100, 10, 40, 20};
            SDL_SetRenderDrawColor(renderer_, 255, 0, 0, 200);
            SDL_RenderFillRect(renderer_, &shakeRect);
        }

        // CHANGE: === MEMORY USAGE INDICATOR ===
        if (showDebugInfo_) {
            // Show memory bar
            float memoryPercent = static_cast<float>(memoryManager_->getTotalMemoryUsage()) /
                (256.0f * 1024.0f * 1024.0f); // Relative to 256MB
            int memBarWidth = static_cast<int>(memoryPercent * 200);
            memBarWidth = std::clamp(memBarWidth, 0, 200);

            SDL_Rect memRect = {10, 110, memBarWidth, 10};
            if (memoryPercent < 0.5f) {
                SDL_SetRenderDrawColor(renderer_, 0, 255, 0, 128); // Green
            }
            else if (memoryPercent < 0.8f) {
                SDL_SetRenderDrawColor(renderer_, 255, 255, 0, 128); // Yellow
            }
            else {
                SDL_SetRenderDrawColor(renderer_, 255, 0, 0, 128); // Red
            }
            SDL_RenderFillRect(renderer_, &memRect);
        }
    }

    /**
     * @brief Render minimap
     */
    void renderMiniMap() {
        const int MINIMAP_SIZE = 150;
        const int MINIMAP_X = WINDOW_WIDTH - MINIMAP_SIZE - 10;
        const int MINIMAP_Y = WINDOW_HEIGHT - MINIMAP_SIZE - 10;

        // Minimap background
        SDL_Rect minimapBg = {MINIMAP_X - 5, MINIMAP_Y - 5, MINIMAP_SIZE + 10, MINIMAP_SIZE + 10};
        SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 180);
        SDL_RenderFillRect(renderer_, &minimapBg);

        // World area in minimap
        SDL_Rect worldRect = {MINIMAP_X, MINIMAP_Y, MINIMAP_SIZE, MINIMAP_SIZE};
        SDL_SetRenderDrawColor(renderer_, 50, 100, 50, 255);
        SDL_RenderFillRect(renderer_, &worldRect);

        // Player position in minimap
        int playerMapX = MINIMAP_X + static_cast<int>((playerPosition_.x / WORLD_WIDTH) * MINIMAP_SIZE);
        int playerMapY = MINIMAP_Y + static_cast<int>((playerPosition_.y / WORLD_HEIGHT) * MINIMAP_SIZE);

        SDL_Rect playerDot = {playerMapX - 3, playerMapY - 3, 6, 6};
        SDL_SetRenderDrawColor(renderer_, 255, 255, 255, 255);
        SDL_RenderFillRect(renderer_, &playerDot);

        // Camera visible area in minimap
        const BaseCamera* activeCamera = cameraManager_->getActiveCamera();
        if (activeCamera) {
            Vec3 cameraPos3D = activeCamera->getPosition();
            Vec2 cameraPos(cameraPos3D.x, cameraPos3D.y);

            const Camera2D* camera2D = cameraManager_->getCamera2D(cameraManager_->getActiveCameraId());
            float zoom = camera2D ? camera2D->getZoom() : 1.0f;

            float visibleWidth = WINDOW_WIDTH / zoom;
            float visibleHeight = WINDOW_HEIGHT / zoom;

            int viewX = MINIMAP_X + static_cast<int>(((cameraPos.x - visibleWidth / 2) / WORLD_WIDTH) * MINIMAP_SIZE);
            int viewY = MINIMAP_Y + static_cast<int>(((cameraPos.y - visibleHeight / 2) / WORLD_HEIGHT) * MINIMAP_SIZE);
            int viewW = static_cast<int>((visibleWidth / WORLD_WIDTH) * MINIMAP_SIZE);
            int viewH = static_cast<int>((visibleHeight / WORLD_HEIGHT) * MINIMAP_SIZE);

            SDL_Rect viewRect = {viewX, viewY, viewW, viewH};
            SDL_SetRenderDrawColor(renderer_, 255, 255, 0, 100);
            SDL_RenderFillRect(renderer_, &viewRect);
        }
    }

    /**
     * @brief Show performance statistics
     * CHANGE: Updated to show new memory and resource system stats
     */
    void showStats(int frameCount, std::chrono::steady_clock::duration elapsed) {
        auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
        double fps = frameCount * 1000.0 / elapsedMs;

        std::cout << "\n--- Game Performance Stats ---" << std::endl;
        std::cout << "FPS: " << fps << std::endl;

        // CHANGE: New ResourceManager stats
        std::cout << "Resource Memory: " << (resourceManager_->getMemoryUsage() / 1024.0 / 1024.0) << " MB" <<
            std::endl;
        std::cout << "Resource Count: " << resourceManager_->getStats().totalResourceCount << std::endl;
        std::cout << "Cache Memory: " << (resourceManager_->getCacheMemoryUsage() / 1024.0 / 1024.0) << " MB" <<
            std::endl;

        // CHANGE: MemoryManager stats
        std::cout << "Total Memory Usage: " << (memoryManager_->getTotalMemoryUsage() / 1024.0 / 1024.0) << " MB" <<
            std::endl;
        std::cout << "General Category: " << (memoryManager_->getCategoryMemoryUsage(MemoryCategory::GENERAL) / 1024.0 /
            1024.0) << " MB" << std::endl;
        std::cout << "Rendering Category: " << (memoryManager_->getCategoryMemoryUsage(MemoryCategory::RENDERING) /
            1024.0 / 1024.0) << " MB" << std::endl;

        // CameraManager stats
        std::cout << "Cameras: " << cameraManager_->getCameraCount() << std::endl;
        std::cout << "Active Transitions: " << cameraManager_->getActiveTransitionCount() << std::endl;
        std::cout << "Camera Memory: " << (cameraManager_->getMemoryUsage() / 1024.0) << " KB" << std::endl;

        // Camera position and zoom
        const BaseCamera* activeCamera = cameraManager_->getActiveCamera();
        if (activeCamera) {
            Vec3 cameraPos = activeCamera->getPosition();
            std::cout << "Camera Position: (" << cameraPos.x << ", " << cameraPos.y << ")" << std::endl;

            const Camera2D* camera2D = cameraManager_->getCamera2D(cameraManager_->getActiveCameraId());
            if (camera2D) {
                std::cout << "Camera Zoom: " << camera2D->getZoom() << "x" << std::endl;
                std::cout << "Camera Mode: " << (camera2D->getMode() == CameraMode::FOLLOW_TARGET ? "FOLLOW" : "FREE")
                    << std::endl;
            }
        }

        // InputManager stats
        if (inputManager_->isGamepadConnected(0)) {
            std::cout << "Gamepad: " << inputManager_->getGamepadName(0) << std::endl;
        }

        // Player position
        std::cout << "Player Position: (" << playerPosition_.x << ", " << playerPosition_.y << ")" << std::endl;
        float worldPercX = (playerPosition_.x / WORLD_WIDTH) * 100.0f;
        float worldPercY = (playerPosition_.y / WORLD_HEIGHT) * 100.0f;
        std::cout << "World Coverage: " << worldPercX << "%, " << worldPercY << "%" << std::endl;
    }

    /**
     * @brief Print controls to console
     */
    void printControls() {
        std::cout << "=== GAME CONTROLS ===" << std::endl;
        std::cout << "Player Movement:" << std::endl;
        std::cout << "  WASD: Move player around the world" << std::endl;
        std::cout << "  Gamepad Left Stick: Move player" << std::endl;
        std::cout << "\nCamera Controls:" << std::endl;
        std::cout << "  C: Switch between FOLLOW and FREE camera modes" << std::endl;
        std::cout << "  Arrow Keys: Manual camera control (FREE mode only)" << std::endl;
        std::cout << "  +/-: Zoom in/out" << std::endl;
        std::cout << "  Space: Camera shake effect" << std::endl;
        std::cout << "  Gamepad Right Stick: Manual camera control" << std::endl;
        std::cout << "  Gamepad Y: Switch camera mode" << std::endl;
        std::cout << "  Gamepad X: Camera shake" << std::endl;
        std::cout << "\nOther:" << std::endl;
        std::cout << "  ESC: Quit game" << std::endl;
        std::cout << "  G: Toggle grid display (Press G key)" << std::endl;
        std::cout << "  D: Toggle debug info" << std::endl;
        std::cout << "\nWorld Size: " << WORLD_WIDTH << "x" << WORLD_HEIGHT << " pixels" << std::endl;
        std::cout << "Viewport: " << WINDOW_WIDTH << "x" << WINDOW_HEIGHT << " pixels" << std::endl;
        std::cout << "=======================" << std::endl;
    }

    /**
     * @brief Clean up resources on exit
     * CHANGE: Updated cleanup for new resource system
     */
    void cleanup() {
        std::cout << "\nCleaning up game resources..." << std::endl;

        // CHANGE: Clean up SDL textures
        if (playerSDLTexture_) {
            SDL_DestroyTexture(playerSDLTexture_);
            playerSDLTexture_ = nullptr;
        }
        if (backgroundSDLTexture_) {
            SDL_DestroyTexture(backgroundSDLTexture_);
            backgroundSDLTexture_ = nullptr;
        }

        // CHANGE: Clean up resource handles
        playerTextureHandle_.reset();
        backgroundTextureHandle_.reset();

        // Clean up managers (in reverse order of initialization)
        if (cameraManager_) {
            std::cout << "Final camera stats: " << cameraManager_->getCameraCount() << " cameras" << std::endl;
            cameraManager_->shutdown();
            cameraManager_.reset();
        }

        if (inputManager_) {
            inputManager_->shutdown();
            inputManager_.reset();
        }

        // CHANGE: Clean up new ResourceManager
        if (resourceManager_) {
            std::cout << "Final resource stats - Memory: " << (resourceManager_->getMemoryUsage() / 1024.0 / 1024.0)
                << " MB, Resources: " << resourceManager_->getStats().totalResourceCount << std::endl;

            // Generate final report
            std::cout << "\n=== Resource Manager Final Report ===" << std::endl;
            std::cout << resourceManager_->generateReport() << std::endl;

            // resourceManager_->shutdown();
            resourceManager_.reset();
        }

        // CHANGE: Clean up MemoryManager (must be last!)
        if (memoryManager_) {
            std::cout << "\n=== Memory Manager Final Report ===" << std::endl;
            std::cout << memoryManager_->generateMemoryReport() << std::endl;

            // Check for leaks
            size_t leaks = engine::memory::MemoryManager::checkForLeaks();
            if (leaks > 0) {
                std::cout << "WARNING: " << leaks << " memory leaks detected!" << std::endl;
            }
            else {
                std::cout << "✓ No memory leaks detected" << std::endl;
            }

            memoryManager_->shutdown();
            memoryManager_.reset();
        }

        // Clean up SDL
        if (renderer_) {
            SDL_DestroyRenderer(renderer_);
            renderer_ = nullptr;
        }

        if (window_) {
            SDL_DestroyWindow(window_);
            window_ = nullptr;
        }

        // Close SDL_image and SDL
        IMG_Quit();
        SDL_Quit();

        std::cout << "Cleanup completed!" << std::endl;
    }

private:
    // === CONSTANTS ===
    static constexpr int WINDOW_WIDTH = 1024;
    static constexpr int WINDOW_HEIGHT = 768;
    static constexpr float PLAYER_SPEED = 300.0f;
    static constexpr float CAMERA_SPEED = 400.0f;

    // === SDL OBJECTS ===
    SDL_Window* window_ = nullptr;
    SDL_Renderer* renderer_ = nullptr;

    // CHANGE: SDL textures for rendering
    SDL_Texture* playerSDLTexture_ = nullptr;
    SDL_Texture* backgroundSDLTexture_ = nullptr;

    // === MANAGERS ===
    // CHANGE: Added MemoryManager and updated ResourceManager
    std::unique_ptr<MemoryManager> memoryManager_;
    std::unique_ptr<ResourceManager> resourceManager_;
    std::unique_ptr<InputManager> inputManager_;
    std::unique_ptr<CameraManager> cameraManager_;

    // === RESOURCE HANDLES ===
    // CHANGE: Using new ResourceHandle system
    ResourceHandle<TextureResource> playerTextureHandle_;
    ResourceHandle<TextureResource> backgroundTextureHandle_;

    // === GAME STATE ===
    // CHANGE: Simplified - no animation system
    Vec2 playerPosition_;
    SDL_Rect currentSpriteFrame_; // Current frame to display from sprite sheet

    // === CAMERA IDS ===
    CameraID mainCameraId_ = INVALID_CAMERA_ID;
    CameraID freeCameraId_ = INVALID_CAMERA_ID;

    // === INPUT ACTION IDS ===
    ActionID moveUpAction_ = INVALID_ACTION_ID;
    ActionID moveDownAction_ = INVALID_ACTION_ID;
    ActionID moveLeftAction_ = INVALID_ACTION_ID;
    ActionID moveRightAction_ = INVALID_ACTION_ID;
    ActionID exitAction_ = INVALID_ACTION_ID;

    // === CAMERA ACTION IDS ===
    ActionID switchCameraAction_ = INVALID_ACTION_ID;
    ActionID zoomInAction_ = INVALID_ACTION_ID;
    ActionID zoomOutAction_ = INVALID_ACTION_ID;
    ActionID shakeAction_ = INVALID_ACTION_ID;

    // === INPUT AXIS IDS ===
    AxisID horizontalAxis_ = INVALID_AXIS_ID;
    AxisID verticalAxis_ = INVALID_AXIS_ID;
    AxisID cameraHorizontalAxis_ = INVALID_AXIS_ID;
    AxisID cameraVerticalAxis_ = INVALID_AXIS_ID;

    // === DEBUG FLAGS ===
    bool showGrid_ = false;
    bool showDebugInfo_ = true;
};

/**
 * @brief Main function
 * CHANGE: Updated description for new resource system
 */
int main() {
    std::cout << "=== Game Demo with New Resource & Memory System ===" << std::endl;
    std::cout << "This demo shows integrated MemoryManager, ResourceManager, InputManager, and CameraManager" <<
        std::endl;
    std::cout << "Features:" << std::endl;
    std::cout << "  - Advanced memory management with allocation tracking" << std::endl;
    std::cout << "  - New resource management system with caching" << std::endl;
    std::cout << "  - Large explorable world (" << GameDemo::WORLD_WIDTH << "x" << GameDemo::WORLD_HEIGHT << " pixels)"
        << std::endl;
    std::cout << "  - Player movement with WASD keys and gamepad" << std::endl;
    std::cout << "  - Advanced camera system with follow and free modes" << std::endl;
    std::cout << "  - Camera zoom, smooth transitions, and shake effects" << std::endl;
    std::cout << "  - Simple sprite rendering (no animation system yet)" << std::endl;
    std::cout << "  - Viewport culling for performance" << std::endl;
    std::cout << "  - Mini-map showing world position" << std::endl;
    std::cout << "  - Performance monitoring and stats" << std::endl;
    std::cout << "\nMake sure you have the following asset:" << std::endl;
    std::cout << "  ../assets/player.png (sprite sheet with specified frame layout)" << std::endl;
    std::cout << "\nStarting game..." << std::endl;

    // Create game instance
    GameDemo game;

    // Initialize
    if (!game.initialize()) {
        std::cerr << "\nFailed to initialize game!" << std::endl;
        std::cerr << "Please ensure:" << std::endl;
        std::cerr << "  1. SDL2 and SDL2_image are properly installed" << std::endl;
        std::cerr << "  2. ../assets/player.png exists and is accessible" << std::endl;
        std::cerr << "  3. The sprite sheet follows the frame layout specifications" << std::endl;
        return -1;
    }

    // Run main loop
    try {
        game.run();
    }
    catch (const std::exception& e) {
        std::cerr << "\nException during game loop: " << e.what() << std::endl;
        return -1;
    }

    // Clean up resources
    game.cleanup();

    std::cout << "\nGame demo completed successfully!" << std::endl;
    std::cout << "Thanks for testing the new resource and memory management system!" << std::endl;
    return 0;
}

// ====================================================================
// TEST main.cpp - Memory Manager Example
// ====================================================================

/*
#include <iostream>
#include <vector>
#include <chrono>
#include "memory/manager/MemoryManager.h"
#include "memory/utils/ScopedAllocator.h"

using namespace engine::memory;

// ========================================================================
// EXAMPLE GAME OBJECTS FOR TESTING
// ========================================================================

// Simula una partícula del sistema de partículas
struct Particle {
    float position[3];
    float velocity[3];
    float color[4];
    float lifetime;

    Particle() {
        std::cout << "  [Particle] Constructor called" << std::endl;
    }

    ~Particle() {
        std::cout << "  [Particle] Destructor called" << std::endl;
    }
};

// Simula un comando de renderizado
struct RenderCommand {
    int meshId;
    float transform[16];
    int textureId;

    RenderCommand() {
        std::cout << "  [RenderCommand] Created" << std::endl;
    }
};

// Simula datos de un enemigo
struct Enemy {
    float health;
    float position[3];
    int state;

    Enemy() : health(100.0f), state(0) {
        std::cout << "  [Enemy] Spawned with 100 HP" << std::endl;
    }

    ~Enemy() {
        std::cout << "  [Enemy] Destroyed" << std::endl;
    }
};

// ========================================================================
// MAIN EXAMPLE
// ========================================================================

int main() {
    std::cout << "=== MEMORY MANAGER EXAMPLE ===" << std::endl << std::endl;

    // ====================================================================
    // PASO 1: CONFIGURACIÓN E INICIALIZACIÓN
    // ====================================================================

    std::cout << "1. INITIALIZING MEMORY MANAGER" << std::endl;
    std::cout << "--------------------------------" << std::endl;

    // Crear instancia del MemoryManager (no singleton, como querías)
    MemoryManager memoryManager;
    MemoryManagerAutoConfig memConfig;

    memConfig.autoDetectLimits = true;
    memConfig.memoryUsagePercent = 0.10f;   // 10% en otras plataformas
    memConfig.heapSizePercent = 0.25f;

    memConfig.enableLeakDetection = true;      // Deshabilitar por ahora
    memConfig.enableBoundsChecking = true;     // Deshabilitar por ahora

    if (!memoryManager.initialize(memConfig)) {
        std::cerr << "Failed to initialize MemoryManager in MAIN" << std::endl;
        return -1;
    }

//  MANUAL CONFIGURATION
    // Configurar el sistema de memoria
//    MemoryManagerConfig config;
//    config.mainHeapSize = 256 * 1024 * 1024;        // 256 MB para el heap principal
//    config.frameStackSize = 4 * 1024 * 1024;        // 4 MB para stack por frame
//    config.frameLinearSize = 8 * 1024 * 1024;       // 8 MB para linear por frame
//    config.frameBufferCount = 2;                     // Double buffering (2 frames)
//    config.renderingPoolSize = 16 * 1024 * 1024;    // 16 MB para pool de rendering
//    config.physicsPoolSize = 8 * 1024 * 1024;       // 8 MB para pool de física
//
//    // Agregar un pool personalizado para partículas
//    config.customPools.push_back({
//        sizeof(Particle),                            // Tamaño de cada partícula
//        100000,                                        // Máximo 100000 partículas
//        MemoryCategory::PARTICLES                   // Categoría PARTICLES
//    });

    // Inicializar el sistema
//    if (!memoryManager.initialize(config)) {
//        std::cerr << "Failed to initialize memory manager!" << std::endl;
//        return -1;
//    }

    std::cout << "✓ Memory Manager initialized successfully" << std::endl;
    std::cout << "  - Main Heap: 256 MB" << std::endl;
    std::cout << "  - Frame Buffers: 2 (double buffering)" << std::endl;
    std::cout << "  - Custom Particle Pool: 100000 particles max" << std::endl;
    std::cout << std::endl;

    // ====================================================================
    // PASO 2: USO DEL POOL ALLOCATOR (para objetos del mismo tamaño)
    // ====================================================================

    std::cout << "2. POOL ALLOCATOR EXAMPLE (Particles)" << std::endl;
    std::cout << "--------------------------------------" << std::endl;

    // El pool para partículas ya fue creado en la configuración
    // Ahora vamos a allocar algunas partículas

    std::vector<Particle*> particles;

    // Crear 5 partículas usando el pool configurado para PARTICLES
    std::cout << "Creating 5 particles from pool:" << std::endl;
    for (int i = 0; i < 5; ++i) {
        // allocateObject usa internamente el pool configurado para PARTICLES
        // El MemoryManager ve que pedimos categoría PARTICLES y usa el pool correspondiente
        Particle* particle = memoryManager.allocateObject<Particle>(MemoryCategory::PARTICLES);

        if (particle) {
            // Inicializar la partícula
            particle->position[0] = i * 10.0f;
            particle->lifetime = 1.0f;
            particles.push_back(particle);
            std::cout << "  ✓ Particle " << i << " allocated at: " << particle << std::endl;
        }
    }

    // Ver uso de memoria actual
    std::cout << "\nMemory usage after creating particles:" << std::endl;
    std::cout << "  - Particles category: "
              << memoryManager.getCategoryMemoryUsage(MemoryCategory::PARTICLES)
              << " bytes" << std::endl;
    std::cout << "  - Total usage: "
              << memoryManager.getTotalMemoryUsage()
              << " bytes" << std::endl;

    // Liberar algunas partículas (simulando que murieron)
    std::cout << "\nDestroying first 2 particles:" << std::endl;
    for (int i = 0; i < 2; ++i) {
        // deallocateObject llama al destructor y devuelve memoria al pool
        memoryManager.deallocateObject(particles[i], MemoryCategory::PARTICLES);
        particles[i] = nullptr;
    }

    // El pool ahora tiene 2 bloques libres que pueden ser reutilizados
    std::cout << "  Pool now has 2 free blocks for reuse" << std::endl;
    std::cout << std::endl;

    // ====================================================================
    // PASO 3: USO DEL STACK ALLOCATOR (para datos temporales LIFO)
    // ====================================================================

    std::cout << "3. STACK ALLOCATOR EXAMPLE (Frame Data)" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    // Simular el inicio de un frame de juego
    memoryManager.beginFrame(1);  // Frame número 1
    std::cout << "Started frame 1" << std::endl;

    // Obtener el stack allocator del frame actual
    StackAllocator& frameStack = memoryManager.getFrameStackAllocator();

    // Guardar un marcador para poder liberar todo de una vez después
    auto stackMarker = frameStack.getMarker();
    std::cout << "  Stack marker saved at position: " << stackMarker << std::endl;

    // Allocar datos temporales para el frame (matrices de transformación)
    std::cout << "\nAllocating temporary frame data:" << std::endl;

    // Allocar espacio para 10 matrices 4x4
    float* matrices = static_cast<float*>(
        frameStack.allocate(sizeof(float) * 16 * 10)  // 10 matrices de 4x4
    );
    std::cout << "  ✓ Allocated 10 transform matrices at: " << matrices << std::endl;

    // Allocar comandos de render temporales
    RenderCommand* commands = static_cast<RenderCommand*>(
        frameStack.allocate(sizeof(RenderCommand) * 20)  // 20 comandos
    );
    std::cout << "  ✓ Allocated 20 render commands at: " << commands << std::endl;

    // Ver uso actual del stack
    std::cout << "\n  Stack usage: " << frameStack.getUsedMemory()
              << " / " << frameStack.getCapacity() << " bytes" << std::endl;

    // Usar ScopedAllocator para una subsección (se libera automáticamente)
    std::cout << "\nUsing ScopedAllocator for temporary calculations:" << std::endl;
    {
        // Todo lo allocado en este scope se libera automáticamente al salir
        ScopedAllocator scoped(frameStack);

        // Allocar memoria temporal para cálculos
        void* tempBuffer = scoped.allocate(1024);  // 1KB temporal
        std::cout << "  ✓ Allocated 1KB temp buffer in scope at: " << tempBuffer << std::endl;

        // Simular algunos cálculos...

    } // <-- Aquí se libera automáticamente el tempBuffer
    std::cout << "  ✓ Scoped memory automatically freed" << std::endl;

    // Liberar todo lo del stack hasta el marcador
    std::cout << "\nFreeing all stack memory to marker:" << std::endl;
    frameStack.freeToMarker(stackMarker);
    std::cout << "  ✓ Stack reset to marker position" << std::endl;
    std::cout << "  Stack usage now: " << frameStack.getUsedMemory() << " bytes" << std::endl;
    std::cout << std::endl;

    // ====================================================================
    // PASO 4: USO DEL LINEAR ALLOCATOR (para carga de nivel)
    // ====================================================================

    std::cout << "4. LINEAR ALLOCATOR EXAMPLE (Level Loading)" << std::endl;
    std::cout << "--------------------------------------------" << std::endl;

    // Obtener el linear allocator del frame
    LinearAllocator& frameLinear = memoryManager.getFrameLinearAllocator();

    std::cout << "Simulating level load - allocating sequential data:" << std::endl;

    // Simular carga de datos de nivel (se cargan una vez, no se liberan individualmente)

    // 1. Cargar datos de geometría
    void* geometryData = frameLinear.allocate(1024 * 100);  // 100 KB de geometría
    std::cout << "  ✓ Loaded 100KB geometry data at: " << geometryData << std::endl;

    // 2. Cargar datos de iluminación
    void* lightingData = frameLinear.allocate(1024 * 50);   // 50 KB de luces
    std::cout << "  ✓ Loaded 50KB lighting data at: " << lightingData << std::endl;

    // 3. Cargar datos de colisión
    void* collisionData = frameLinear.allocate(1024 * 75);  // 75 KB de colisión
    std::cout << "  ✓ Loaded 75KB collision data at: " << collisionData << std::endl;

    // Linear allocator NO puede liberar allocaciones individuales
    std::cout << "\n  Linear usage: " << frameLinear.getUsedMemory()
              << " / " << frameLinear.getCapacity() << " bytes" << std::endl;
    std::cout << "  Allocations made: " << frameLinear.getAllocationCount() << std::endl;

    // La única forma de liberar es hacer reset completo
    std::cout << "\nResetting linear allocator (unloading level):" << std::endl;
    frameLinear.reset();
    std::cout << "  ✓ All linear memory freed at once" << std::endl;
    std::cout << "  Linear usage now: " << frameLinear.getUsedMemory() << " bytes" << std::endl;
    std::cout << std::endl;

    // ====================================================================
    // PASO 5: USO DEL HEAP PRINCIPAL (fallback general)
    // ====================================================================

    std::cout << "5. MAIN HEAP EXAMPLE (General Purpose)" << std::endl;
    std::cout << "---------------------------------------" << std::endl;

    // Cuando pedimos una categoría sin allocator específico, usa el heap principal
    std::cout << "Allocating enemies (no specific pool configured):" << std::endl;

    // Crear enemigos - usa el heap principal porque no hay pool para GAMEPLAY
    Enemy* enemy1 = memoryManager.allocateObject<Enemy>(MemoryCategory::GAMEPLAY);
    std::cout << "  ✓ Enemy 1 allocated at: " << enemy1 << std::endl;

    Enemy* enemy2 = memoryManager.allocateObject<Enemy>(MemoryCategory::GAMEPLAY);
    std::cout << "  ✓ Enemy 2 allocated at: " << enemy2 << std::endl;

    // Ver uso por categoría
    std::cout << "\nMemory usage by category:" << std::endl;
    std::cout << "  - GAMEPLAY: "
              << memoryManager.getCategoryMemoryUsage(MemoryCategory::GAMEPLAY)
              << " bytes" << std::endl;
    std::cout << "  - PARTICLES: "
              << memoryManager.getCategoryMemoryUsage(MemoryCategory::PARTICLES)
              << " bytes" << std::endl;

    // ====================================================================
    // PASO 6: REPORTE DE MEMORIA
    // ====================================================================

    std::cout << "\n6. MEMORY REPORT" << std::endl;
    std::cout << "----------------" << std::endl;

    // Generar reporte completo
    std::string report = memoryManager.generateMemoryReport();
    std::cout << report << std::endl;

    // ====================================================================
    // PASO 7: LIMPIEZA
    // ====================================================================

    std::cout << "7. CLEANUP" << std::endl;
    std::cout << "----------" << std::endl;

    // Liberar enemigos
    std::cout << "Destroying enemies:" << std::endl;
    memoryManager.deallocateObject(enemy1, MemoryCategory::GAMEPLAY);
    memoryManager.deallocateObject(enemy2, MemoryCategory::GAMEPLAY);

    // Liberar partículas restantes
    std::cout << "Destroying remaining particles:" << std::endl;
    for (size_t i = 2; i < particles.size(); ++i) {
        if (particles[i]) {
            memoryManager.deallocateObject(particles[i], MemoryCategory::PARTICLES);
        }
    }

    // Verificar leaks antes de shutdown
    std::size_t leaks = memoryManager.checkForLeaks();
    if (leaks > 0) {
        std::cout << "\n⚠ Warning: " << leaks << " memory leaks detected!" << std::endl;
    } else {
        std::cout << "\n✓ No memory leaks detected" << std::endl;
    }

    // Shutdown del sistema
    std::cout << "\nShutting down memory manager..." << std::endl;
    memoryManager.shutdown();

    std::cout << "\n=== EXAMPLE COMPLETED ===" << std::endl;

    return 0;
}
*/
