// #include <SDL.h>
// #include <SDL_image.h>
// #include <iostream>
// #include <memory>
// #include <chrono>
// #include <thread>
// #include <cmath>
//
// #include "./Resource/ResourceManager.h"
// #include "./Texture/TextureResource.h"
// #include "./Input/InputManager.h"
// #include "./Animation/AnimationManager.h"
// #include "./Camera/CameraManager.h"
//
// using namespace engine::resources;
// using namespace engine::input;
// using namespace engine::animation;
// using namespace engine::camera;
//
// /**
//  * @brief Clase principal del juego que demuestra el uso integrado de todos los managers
//  * CHANGE: Integración completa del CameraManager para mundo expandido
//  */
// class GameDemo {
// public:
//     GameDemo() = default;
//     ~GameDemo() = default;
//
//     static constexpr int WORLD_WIDTH = 4096;                // ← NUEVO: Mundo 4x más grande
//     static constexpr int WORLD_HEIGHT = 3072;
//
//     // Enumeración para animaciones del jugador
//     enum class PlayerAnimation {
//         IDLE,
//         UP,
//         DOWN,
//         LEFT,
//         RIGHT
//     };
//
//     /**
//      * @brief Inicializar SDL, managers y recursos
//      * CHANGE: Agregado CameraManager e incrementado tamaño del mundo
//      * @return true si la inicialización fue exitosa
//      */
//     bool initialize() {
//         // 1. INICIALIZAR SDL
//         if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) < 0) {
//             std::cerr << "Failed to initialize SDL: " << SDL_GetError() << std::endl;
//             return false;
//         }
//
//         // 2. INICIALIZAR SDL_IMAGE
//         int imgFlags = IMG_INIT_PNG | IMG_INIT_JPG;
//         if (!(IMG_Init(imgFlags) & imgFlags)) {
//             std::cerr << "Failed to initialize SDL_image: " << IMG_GetError() << std::endl;
//             SDL_Quit();
//             return false;
//         }
//
//         // 3. CREAR VENTANA
//         window_ = SDL_CreateWindow("Game Demo - World with Camera System",
//                                   SDL_WINDOWPOS_CENTERED,
//                                   SDL_WINDOWPOS_CENTERED,
//                                   WINDOW_WIDTH, WINDOW_HEIGHT,
//                                   SDL_WINDOW_SHOWN);
//
//         if (!window_) {
//             std::cerr << "Failed to create window: " << SDL_GetError() << std::endl;
//             cleanup();
//             return false;
//         }
//
//         // 4. CREAR RENDERER
//         renderer_ = SDL_CreateRenderer(window_, -1,
//                                      SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
//
//         if (!renderer_) {
//             std::cerr << "Failed to create renderer: " << SDL_GetError() << std::endl;
//             cleanup();
//             return false;
//         }
//
//         // 5. CONFIGURAR E INICIALIZAR RESOURCEMANAGER
//         ResourceManagerConfig resourceConfig;
//         resourceConfig.maxMemoryUsage = 256 * 1024 * 1024;    // 256MB límite
//         resourceConfig.maxResources = 1000;                    // 1000 recursos máximo
//         resourceConfig.enableAsyncLoading = true;              // Carga asíncrona habilitada
//         resourceConfig.enableMemoryProfiling = true;          // Profiling habilitado
//
//         resourceManager_ = std::make_unique<ResourceManager>(resourceConfig);
//
//         // 6. REGISTRAR FACTORY PARA TEXTURAS
//         auto textureFactory = std::make_unique<SDLTextureFactory>(renderer_);
//         textureFactory->setDefaultFiltering(true);  // Usar filtrado linear
//         resourceManager_->registerFactory<TextureResource>(std::move(textureFactory));
//
//         // 7. CONFIGURAR E INICIALIZAR INPUTMANAGER
//         InputManagerConfig inputConfig;
//         inputConfig.enableInputLogging = false;               // Reducir logging para mejor performance
//         inputConfig.enableGamepadHotswap = true;               // Hot-swap de gamepads
//         inputConfig.maxGamepads = 1;                           // Solo 1 gamepad para este demo
//
//         inputManager_ = std::make_unique<InputManager>(inputConfig);
//         if (!inputManager_->initialize()) {
//             std::cerr << "Failed to initialize InputManager" << std::endl;
//             cleanup();
//             return false;
//         }
//
//         // 8. CONFIGURAR E INICIALIZAR CAMERAMANAGER ← NUEVO
//         CameraManagerConfig cameraConfig;
//         cameraConfig.enableCameraLogging = true;               // Logging para debug de cámara
//         cameraConfig.enableTransitions = true;                 // Transiciones suaves
//         cameraConfig.enableShake = true;                       // Efectos de shake
//         cameraConfig.maxCameras = 4;                           // Múltiples cámaras
//         cameraConfig.defaultSmoothingSpeed = 8.0f;            // Seguimiento suave del jugador
//         cameraConfig.mouseSensitivity = 1.0f;
//         cameraConfig.scrollSensitivity = 1.0f;
//
//         cameraManager_ = std::make_unique<CameraManager>(cameraConfig);
//         if (!cameraManager_->initialize()) {
//             std::cerr << "Failed to initialize CameraManager" << std::endl;
//             cleanup();
//             return false;
//         }
//
//         // 9. CONFIGURAR E INICIALIZAR ANIMATIONMANAGER
//         AnimationManagerConfig animConfig;
//         animConfig.enableAnimationLogging = false;            // Reducir logging
//         animConfig.enableBatching = true;                      // Batching habilitado
//         animConfig.enableCulling = true;                       // Culling habilitado para mundo grande
//         animConfig.maxInstances = 1000;                        // 1000 instancias máximo
//
//         animationManager_ = std::make_unique<AnimationManager>(animConfig);
//         if (!animationManager_->initialize()) {
//             std::cerr << "Failed to initialize AnimationManager" << std::endl;
//             cleanup();
//             return false;
//         }
//
//         // 10. CONFIGURAR VIEWPORT PARA CÁMARA
//         Viewport viewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
//         cameraManager_->setViewport(viewport);
//
//         // 11. CONFIGURAR MUNDO Y CÁMARA
//         if (!setupWorldAndCamera()) {
//             std::cerr << "Failed to setup world and camera" << std::endl;
//             cleanup();
//             return false;
//         }
//
//         // 12. CONFIGURAR INPUT ACTIONS Y EJES
//         setupInputBindings();
//
//         // 13. CARGAR RECURSOS Y CREAR ANIMACIONES
//         if (!loadPlayerResources()) {
//             std::cerr << "Failed to load player resources" << std::endl;
//             cleanup();
//             return false;
//         }
//
//         std::cout << "Game initialized successfully!" << std::endl;
//         std::cout << "World size: " << WORLD_WIDTH << "x" << WORLD_HEIGHT << " pixels" << std::endl;
//         std::cout << "Viewport: " << WINDOW_WIDTH << "x" << WINDOW_HEIGHT << " pixels" << std::endl;
//         return true;
//     }
//
//     /**
//      * @brief Configurar mundo y sistema de cámaras
//      * CHANGE: Nuevo método para configurar cámaras y límites del mundo
//      */
//     bool setupWorldAndCamera() {
//         // 1. CREAR CÁMARA PRINCIPAL (2D para este demo)
//         mainCameraId_ = cameraManager_->createCamera2D("MainCamera");
//         if (mainCameraId_ == INVALID_CAMERA_ID) {
//             std::cerr << "Failed to create main camera" << std::endl;
//             return false;
//         }
//
//         // 2. CONFIGURAR CÁMARA PARA SEGUIMIENTO DEL JUGADOR
//         if (!cameraManager_->setupTopDownCamera(mainCameraId_, Vector2(WORLD_WIDTH/2, WORLD_HEIGHT/2), 1.0f)) {
//             std::cerr << "Failed to setup top-down camera" << std::endl;
//             return false;
//         }
//
//         // 3. CONFIGURAR LÍMITES DEL MUNDO
//         Vector2 worldMin(0.0f, 0.0f);
//         Vector2 worldMax(WORLD_WIDTH, WORLD_HEIGHT);
//         CameraBounds worldBounds(Vector3(worldMin.x, worldMin.y, 0.0f), Vector3(worldMax.x, worldMax.y, 0.0f), 100.0f);
//
//         Camera2D* mainCamera = cameraManager_->getCamera2D(mainCameraId_);
//         if (mainCamera) {
//             mainCamera->setBounds(worldBounds);
//             mainCamera->setMode(CameraMode::FOLLOW_TARGET);
//             mainCamera->setFollowSpeed(6.0f);  // Seguimiento suave pero responsivo
//             mainCamera->setZoomLimits(0.5f, 2.0f);  // Permite zoom entre 50% y 200%
//
//             std::cout << "✓ Main camera configured with world bounds" << std::endl;
//         }
//
//         // 4. ACTIVAR CÁMARA PRINCIPAL
//         if (!cameraManager_->setActiveCamera(mainCameraId_)) {
//             std::cerr << "Failed to set active camera" << std::endl;
//             return false;
//         }
//
//         // 5. CREAR CÁMARA ALTERNATIVA PARA PRUEBAS
//         freeCameraId_ = cameraManager_->createCamera2D("FreeCamera");
//         if (freeCameraId_ != INVALID_CAMERA_ID) {
//             Camera2D* freeCamera = cameraManager_->getCamera2D(freeCameraId_);
//             if (freeCamera) {
//                 freeCamera->setPosition(Vector2(WORLD_WIDTH/2, WORLD_HEIGHT/2));
//                 freeCamera->setBounds(worldBounds);
//                 freeCamera->setMode(CameraMode::STATIC);
//                 std::cout << "✓ Free camera created for manual control" << std::endl;
//             }
//         }
//
//         return true;
//     }
//
//     /**
//      * @brief Configurar bindings de input para movimiento del jugador
//      * CHANGE: Agregados controles de cámara
//      */
//     void setupInputBindings() {
//         // === ACCIONES DE MOVIMIENTO DEL JUGADOR ===
//         moveUpAction_ = inputManager_->createAction("MoveUp");
//         moveDownAction_ = inputManager_->createAction("MoveDown");
//         moveLeftAction_ = inputManager_->createAction("MoveLeft");
//         moveRightAction_ = inputManager_->createAction("MoveRight");
//         exitAction_ = inputManager_->createAction("Exit");
//
//         // === ACCIONES DE CÁMARA ===
//         switchCameraAction_ = inputManager_->createAction("SwitchCamera");
//         zoomInAction_ = inputManager_->createAction("ZoomIn");
//         zoomOutAction_ = inputManager_->createAction("ZoomOut");
//         shakeAction_ = inputManager_->createAction("CameraShake");
//
//         // Bindear teclas WASD para movimiento del jugador
//         inputManager_->bindKeyToAction(moveUpAction_, SDLK_w);
//         inputManager_->bindKeyToAction(moveDownAction_, SDLK_s);
//         inputManager_->bindKeyToAction(moveLeftAction_, SDLK_a);
//         inputManager_->bindKeyToAction(moveRightAction_, SDLK_d);
//         inputManager_->bindKeyToAction(exitAction_, SDLK_ESCAPE);
//
//         // Bindear controles de cámara
//         inputManager_->bindKeyToAction(switchCameraAction_, SDLK_c);
//         inputManager_->bindKeyToAction(zoomInAction_, SDLK_EQUALS);     // '+' key
//         inputManager_->bindKeyToAction(zoomOutAction_, SDLK_MINUS);     // '-' key
//         inputManager_->bindKeyToAction(shakeAction_, SDLK_SPACE);       // Spacebar para shake
//
//         // Crear ejes para movimiento suave
//         AxisConfig axisConfig;
//         axisConfig.deadZone = 0.1f;
//         axisConfig.sensitivity = 1.0f;
//
//         horizontalAxis_ = inputManager_->createAxis("Horizontal", axisConfig);
//         verticalAxis_ = inputManager_->createAxis("Vertical", axisConfig);
//
//         // Bindear teclas a ejes
//         inputManager_->bindKeysToAxis(horizontalAxis_, SDLK_d, SDLK_a);  // D = positivo, A = negativo
//         inputManager_->bindKeysToAxis(verticalAxis_, SDLK_w, SDLK_s);    // W = positivo, S = negativo
//
//         // === CONTROLES DE CÁMARA MANUAL (FLECHAS) ===
//         cameraHorizontalAxis_ = inputManager_->createAxis("CameraHorizontal", axisConfig);
//         cameraVerticalAxis_ = inputManager_->createAxis("CameraVertical", axisConfig);
//
//         inputManager_->bindKeysToAxis(cameraHorizontalAxis_, SDLK_RIGHT, SDLK_LEFT);
//         inputManager_->bindKeysToAxis(cameraVerticalAxis_, SDLK_UP, SDLK_DOWN);
//
//         // Bindear gamepad si está disponible
//         if (inputManager_->isGamepadConnected(0)) {
//             inputManager_->bindGamepadButtonToAction(moveUpAction_, 0, SDL_CONTROLLER_BUTTON_DPAD_UP);
//             inputManager_->bindGamepadButtonToAction(moveDownAction_, 0, SDL_CONTROLLER_BUTTON_DPAD_DOWN);
//             inputManager_->bindGamepadButtonToAction(moveLeftAction_, 0, SDL_CONTROLLER_BUTTON_DPAD_LEFT);
//             inputManager_->bindGamepadButtonToAction(moveRightAction_, 0, SDL_CONTROLLER_BUTTON_DPAD_RIGHT);
//
//             inputManager_->bindGamepadAxisToAxis(horizontalAxis_, 0, GamepadAxis::LEFT_STICK_X);
//             inputManager_->bindGamepadAxisToAxis(verticalAxis_, 0, GamepadAxis::LEFT_STICK_Y);
//
//             // Cámara con stick derecho
//             inputManager_->bindGamepadAxisToAxis(cameraHorizontalAxis_, 0, GamepadAxis::RIGHT_STICK_X);
//             inputManager_->bindGamepadAxisToAxis(cameraVerticalAxis_, 0, GamepadAxis::RIGHT_STICK_Y);
//
//             // Botones de cámara
//             inputManager_->bindGamepadButtonToAction(switchCameraAction_, 0, SDL_CONTROLLER_BUTTON_Y);
//             inputManager_->bindGamepadButtonToAction(shakeAction_, 0, SDL_CONTROLLER_BUTTON_X);
//         }
//
//         std::cout << "Input bindings configured successfully!" << std::endl;
//         std::cout << "Controls:" << std::endl;
//         std::cout << "  WASD: Move player" << std::endl;
//         std::cout << "  Arrow Keys: Manual camera control" << std::endl;
//         std::cout << "  C: Switch camera mode" << std::endl;
//         std::cout << "  +/-: Zoom in/out" << std::endl;
//         std::cout << "  Space: Camera shake effect" << std::endl;
//         std::cout << "  ESC: Quit" << std::endl;
//     }
//
//     /**
//      * @brief Cargar recursos del jugador y crear animaciones
//      * CHANGE: Posición inicial ajustada al centro del mundo expandido
//      */
//     bool loadPlayerResources() {
//         std::cout << "\n=== Loading Player Resources ===" << std::endl;
//
//         // 1. CARGAR SPRITE SHEET DEL JUGADOR
//         playerTexture_ = resourceManager_->loadResource<TextureResource>("../assets/player.png",
//                                                                         LoadPriority::HIGH);
//
//         if (!playerTexture_.isValid() || !playerTexture_.isReady()) {
//             std::cerr << "Failed to load player sprite sheet" << std::endl;
//             return false;
//         }
//
//         std::cout << "✓ Player sprite sheet loaded: " << playerTexture_->getWidth()
//                  << "x" << playerTexture_->getHeight() << " pixels" << std::endl;
//
//         // 2. CREAR SPRITE SHEET EN ANIMATION MANAGER
//         playerSpriteSheetId_ = animationManager_->createSpriteSheet("PlayerSpriteSheet", playerTexture_);
//         SpriteSheet* spriteSheet = animationManager_->getSpriteSheet(playerSpriteSheetId_);
//
//         if (!spriteSheet) {
//             std::cerr << "Failed to create sprite sheet" << std::endl;
//             return false;
//         }
//
//         // 3. DEFINIR FRAMES PARA CADA ANIMACIÓN
//         // Dimensiones del sprite: 160x140 px
//         const int SPRITE_WIDTH = 160;
//         const int SPRITE_HEIGHT = 140;
//
//         // IDLE Animation (3 frames) - Y: 0
//         for (int i = 0; i < 3; ++i) {
//             spriteSheet->addFrame(SpriteRect(i * SPRITE_WIDTH, 0, SPRITE_WIDTH, SPRITE_HEIGHT));
//         }
//
//         // DOWN Animation (10 frames) - Y: 600
//         for (int i = 0; i < 10; ++i) {
//             spriteSheet->addFrame(SpriteRect(i * SPRITE_WIDTH, 600, SPRITE_WIDTH, SPRITE_HEIGHT));
//         }
//
//         // LEFT Animation (10 frames) - Y: 772
//         for (int i = 0; i < 10; ++i) {
//             spriteSheet->addFrame(SpriteRect(i * SPRITE_WIDTH, 772, SPRITE_WIDTH, SPRITE_HEIGHT));
//         }
//
//         // UP Animation (10 frames) - Y: 930
//         for (int i = 0; i < 10; ++i) {
//             spriteSheet->addFrame(SpriteRect(i * SPRITE_WIDTH, 930, SPRITE_WIDTH, SPRITE_HEIGHT));
//         }
//
//         // RIGHT Animation (10 frames) - Y: 1068
//         for (int i = 0; i < 10; ++i) {
//             spriteSheet->addFrame(SpriteRect(i * SPRITE_WIDTH, 1068, SPRITE_WIDTH, SPRITE_HEIGHT));
//         }
//
//         std::cout << "✓ Added " << spriteSheet->getFrameCount() << " frames to sprite sheet" << std::endl;
//
//         // 4. CREAR ANIMACIONES
//         if (!createPlayerAnimations()) {
//             return false;
//         }
//
//         // 5. CREAR INSTANCIA DEL JUGADOR EN EL CENTRO DEL MUNDO
//         SpriteTransform playerTransform;
//         playerTransform.position = Vector2(WORLD_WIDTH / 2.0f, WORLD_HEIGHT / 2.0f);  // Centro del mundo
//         playerTransform.scale = Vector2(1.0f, 1.0f);
//         playerTransform.rotation = 0.0f;
//         playerTransform.alpha = 255;
//         playerTransform.layer = 1;
//
//         playerInstanceId_ = animationManager_->createInstance(idleAnimationId_, playerTransform);
//
//         if (playerInstanceId_ == INVALID_INSTANCE_ID) {
//             std::cerr << "Failed to create player animation instance" << std::endl;
//             return false;
//         }
//
//         Camera2D* mainCamera = cameraManager_->getCamera2D(mainCameraId_);
//         if (mainCamera) {
//             mainCamera->setTarget(playerTransform.position); // ← AGREGAR ESTA LÍNEA
//             std::cout << "✓ Camera set to follow player at (" << playerTransform.position.x
//                      << ", " << playerTransform.position.y << ")" << std::endl;
//         }
//
//         // Iniciar con animación idle
//         AnimationInstance* playerInstance = animationManager_->getInstance(playerInstanceId_);
//         if (playerInstance) {
//             playerInstance->play();
//             currentAnimation_ = PlayerAnimation::IDLE;
//
//             // Configurar cámara para seguir al jugador
//             Camera2D* mainCamera = cameraManager_->getCamera2D(mainCameraId_);
//             if (mainCamera) {
//                 mainCamera->setTarget(playerTransform.position);
//                 std::cout << "✓ Camera set to follow player at (" << playerTransform.position.x
//                          << ", " << playerTransform.position.y << ")" << std::endl;
//             }
//         }
//
//         std::cout << "✓ Player animation instance created and started" << std::endl;
//         return true;
//     }
//
//     /**
//      * @brief Crear todas las animaciones del jugador
//      * CHANGE: Sin cambios, pero mantiene compatibilidad
//      */
//     bool createPlayerAnimations() {
//         AnimationConfig animConfig;
//         animConfig.frameRate = 8.0f;                   // 8 FPS para animaciones suaves
//         animConfig.mode = AnimationMode::LOOP;         // Loop continuo
//         animConfig.autoPlay = false;                   // Control manual
//
//         // IDLE Animation (frames 0-2)
//         idleAnimationId_ = animationManager_->createAnimation("PlayerIdle", playerSpriteSheetId_, animConfig);
//         Animation* idleAnim = animationManager_->getAnimation(idleAnimationId_);
//         if (idleAnim) {
//             idleAnim->generateSequence(0, 2);  // Frames 0, 1, 2
//             std::cout << "✓ IDLE animation created (frames 0-2)" << std::endl;
//         }
//
//         // DOWN Animation (frames 3-12)
//         downAnimationId_ = animationManager_->createAnimation("PlayerDown", playerSpriteSheetId_, animConfig);
//         Animation* downAnim = animationManager_->getAnimation(downAnimationId_);
//         if (downAnim) {
//             downAnim->generateSequence(3, 12);  // Frames 3-12
//             std::cout << "✓ DOWN animation created (frames 3-12)" << std::endl;
//         }
//
//         // LEFT Animation (frames 13-22)
//         leftAnimationId_ = animationManager_->createAnimation("PlayerLeft", playerSpriteSheetId_, animConfig);
//         Animation* leftAnim = animationManager_->getAnimation(leftAnimationId_);
//         if (leftAnim) {
//             leftAnim->generateSequence(13, 22);  // Frames 13-22
//             std::cout << "✓ LEFT animation created (frames 13-22)" << std::endl;
//         }
//
//         // UP Animation (frames 23-32)
//         upAnimationId_ = animationManager_->createAnimation("PlayerUp", playerSpriteSheetId_, animConfig);
//         Animation* upAnim = animationManager_->getAnimation(upAnimationId_);
//         if (upAnim) {
//             upAnim->generateSequence(23, 32);  // Frames 23-32
//             std::cout << "✓ UP animation created (frames 23-32)" << std::endl;
//         }
//
//         // RIGHT Animation (frames 33-42)
//         rightAnimationId_ = animationManager_->createAnimation("PlayerRight", playerSpriteSheetId_, animConfig);
//         Animation* rightAnim = animationManager_->getAnimation(rightAnimationId_);
//         if (rightAnim) {
//             rightAnim->generateSequence(33, 42);  // Frames 33-42
//             std::cout << "✓ RIGHT animation created (frames 33-42)" << std::endl;
//         }
//
//         return true;
//     }
//
//     /**
//      * @brief Loop principal del juego
//      * CHANGE: Integrado update de cámara y mejor control de tiempo
//      */
//     void run() {
//         bool running = true;
//
//         // Variables de control de tiempo
//         auto lastStatsTime = std::chrono::steady_clock::now();
//         auto lastFrameTime = std::chrono::steady_clock::now();
//         int frameCount = 0;
//
//         std::cout << "\n=== Starting Game Loop ===" << std::endl;
//         printControls();
//
//         while (running) {
//             auto currentTime = std::chrono::steady_clock::now();
//             auto deltaTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastFrameTime);
//             float deltaTime = deltaTimeMs.count() / 1000.0f;  // Convert to seconds
//             lastFrameTime = currentTime;
//
//             // Clamp delta time para evitar saltos grandes
//             deltaTime = std::min(deltaTime, 0.016f);  // Max 16ms (60 FPS mínimo)
//
//             // 1. PROCESAR EVENTOS DE INPUT
//             inputManager_->processEvents(deltaTime);
//
//             // 2. ACTUALIZAR INPUT MANAGER
//             inputManager_->update(deltaTime);
//
//             // 3. VERIFICAR SI DEBE SALIR
//             if (inputManager_->shouldExit() || inputManager_->isActionPressed(exitAction_)) {
//                 running = false;
//                 break;
//             }
//
//             // 4. PROCESAR INPUT DE CÁMARA
//             processCameraInput(deltaTime);
//
//             // 5. ACTUALIZAR LÓGICA DEL JUEGO
//             updateGameLogic(deltaTime);
//
//             // 6. ACTUALIZAR CÁMARA ← NUEVO
//             cameraManager_->update(deltaTime);
//
//             // 7. ACTUALIZAR ANIMACIONES
//             animationManager_->update(deltaTime);
//
//             // 8. RENDERIZAR
//             render();
//
//             // 9. MOSTRAR ESTADÍSTICAS CADA 3 SEGUNDOS
//             frameCount++;
//             if (currentTime - lastStatsTime > std::chrono::seconds(3)) {
//                 showStats(frameCount, currentTime - lastStatsTime);
//                 frameCount = 0;
//                 lastStatsTime = currentTime;
//             }
//
//             // 10. CONTROL DE FRAMERATE (60 FPS)
//             auto frameTime = std::chrono::steady_clock::now() - currentTime;
//             auto targetFrameTime = std::chrono::milliseconds(16); // ~60 FPS
//             if (frameTime < targetFrameTime) {
//                 std::this_thread::sleep_for(targetFrameTime - frameTime);
//             }
//         }
//
//         std::cout << "\nGame loop ended" << std::endl;
//     }
//
//     /**
//      * @brief Procesar input específico de cámara
//      * CHANGE: Nuevo método para manejar controles de cámara
//      */
//     void processCameraInput(float deltaTime) {
//         // Switch entre modos de cámara
//         if (inputManager_->isActionPressed(switchCameraAction_)) {
//             toggleCameraMode();
//         }
//
//         // Zoom
//         if (inputManager_->isActionPressed(zoomInAction_)) {
//             cameraManager_->processZoom(1.0f);  // Zoom in
//         }
//         if (inputManager_->isActionPressed(zoomOutAction_)) {
//             cameraManager_->processZoom(-1.0f); // Zoom out
//         }
//
//         // Camera shake
//         if (inputManager_->isActionPressed(shakeAction_)) {
//             ShakeConfig shakeConfig(2.0f, 0.5f, ShakePattern::EXPLOSION);
//             cameraManager_->startCameraShake(mainCameraId_, shakeConfig);
//         }
//
//         // Control manual de cámara (solo en modo free)
//         Camera2D* activeCamera = cameraManager_->getCamera2D(cameraManager_->getActiveCameraId());
//         if (activeCamera && activeCamera->getMode() == CameraMode::STATIC) {
//             float cameraHorizontal = inputManager_->getAxisValue(cameraHorizontalAxis_);
//             float cameraVertical = inputManager_->getAxisValue(cameraVerticalAxis_);
//
//             if (std::abs(cameraHorizontal) > 0.1f || std::abs(cameraVertical) > 0.1f) {
//                 Vector2 currentPos = activeCamera->getPosition2D();
//                 Vector2 movement(cameraHorizontal * CAMERA_SPEED * deltaTime,
//                                cameraVertical * CAMERA_SPEED * deltaTime);
//                 activeCamera->setPosition(currentPos + movement);
//             }
//         }
//     }
//
//     /**
//      * @brief Alternar entre modos de cámara
//      * CHANGE: Nuevo método para cambiar entre seguimiento y control libre
//      */
//     void toggleCameraMode() {
//         static bool isFollowMode = true;
//
//         if (isFollowMode) {
//             // Cambiar a modo libre
//             cameraManager_->setActiveCamera(freeCameraId_);
//             std::cout << "Camera Mode: FREE CONTROL (use arrow keys)" << std::endl;
//         } else {
//             // Cambiar a modo seguimiento
//             cameraManager_->setActiveCamera(mainCameraId_);
//             std::cout << "Camera Mode: FOLLOW PLAYER" << std::endl;
//         }
//
//         isFollowMode = !isFollowMode;
//     }
//
//     /**
//      * @brief Actualizar lógica del juego
//      * CHANGE: Actualizada posición de la cámara para seguir al jugador
//      */
//     void updateGameLogic(float deltaTime) {
//     // Obtener instancia del jugador
//     AnimationInstance* playerInstance = animationManager_->getInstance(playerInstanceId_);
//     if (!playerInstance) {
//         return;
//     }
//
//     if (!playerInstance->isPlaying()) {
//         playerInstance->play();
//     }
//
//     // PASO 1: TRABAJAR CON COORDENADAS WORLD
//     // =====================================
//
//     // Obtener la posición actual WORLD del jugador (mantenida separadamente)
//     // CAMBIO: Ya no usamos directamente playerTransform.position como world coords
//     static Vector2 playerWorldPosition(WORLD_WIDTH / 2.0f, WORLD_HEIGHT / 2.0f); // Inicializar en centro del mundo solo la primera vez
//
//     // Obtener transform actual (que contiene coordenadas SCREEN)
//     SpriteTransform playerTransform = playerInstance->getTransform();
//
//     // Variables de movimiento
//     Vector2 movement{0.0f, 0.0f};
//     bool isMoving = false;
//     PlayerAnimation newAnimation = currentAnimation_;
//
//     // DETECTAR MOVIMIENTO Y DETERMINAR ANIMACIÓN
//     if (inputManager_->isActionHeld(moveUpAction_)) {
//         movement.y -= PLAYER_SPEED * deltaTime;
//         newAnimation = PlayerAnimation::UP;
//         isMoving = true;
//     }
//     if (inputManager_->isActionHeld(moveDownAction_)) {
//         movement.y += PLAYER_SPEED * deltaTime;
//         newAnimation = PlayerAnimation::DOWN;
//         isMoving = true;
//     }
//     if (inputManager_->isActionHeld(moveLeftAction_)) {
//         movement.x -= PLAYER_SPEED * deltaTime;
//         newAnimation = PlayerAnimation::LEFT;
//         isMoving = true;
//     }
//     if (inputManager_->isActionHeld(moveRightAction_)) {
//         movement.x += PLAYER_SPEED * deltaTime;
//         newAnimation = PlayerAnimation::RIGHT;
//         isMoving = true;
//     }
//
//     // Si no hay movimiento, usar animación idle
//     if (!isMoving) {
//         newAnimation = PlayerAnimation::IDLE;
//     }
//
//     // PASO 2: ACTUALIZAR POSICIÓN WORLD DEL JUGADOR
//     // =============================================
//     if (isMoving) {
//         // Normalizar movimiento diagonal
//         if (movement.x != 0.0f && movement.y != 0.0f) {
//             float length = std::sqrt(movement.x * movement.x + movement.y * movement.y);
//             movement.x /= length;
//             movement.y /= length;
//             movement.x *= PLAYER_SPEED * deltaTime;
//             movement.y *= PLAYER_SPEED * deltaTime;
//         }
//
//         // CAMBIO: Aplicar movimiento a coordenadas WORLD
//         playerWorldPosition.x += movement.x;
//         playerWorldPosition.y += movement.y;
//
//         // CLAMPEAR A LÍMITES DEL MUNDO
//         const float SPRITE_HALF_WIDTH = 80.0f;   // 160/2
//         const float SPRITE_HALF_HEIGHT = 70.0f;  // 140/2
//
//         playerWorldPosition.x = std::clamp(playerWorldPosition.x,
//                                           SPRITE_HALF_WIDTH,
//                                           WORLD_WIDTH - SPRITE_HALF_WIDTH);
//         playerWorldPosition.y = std::clamp(playerWorldPosition.y,
//                                           SPRITE_HALF_HEIGHT,
//                                           WORLD_HEIGHT - SPRITE_HALF_HEIGHT);
//
//         // PASO 3: ACTUALIZAR CÁMARA CON COORDENADAS WORLD
//         // ===============================================
//         Camera2D* followCamera = cameraManager_->getCamera2D(mainCameraId_);
//         if (followCamera && followCamera->getMode() == CameraMode::FOLLOW_TARGET) {
//             followCamera->setTarget(playerWorldPosition); // CAMBIO: Usar world position para cámara
//         }
//     }
//
//     // PASO 4: CONVERTIR A COORDENADAS SCREEN PARA ANIMATIONMANAGER
//     // ============================================================
//
//     // Obtener offset de cámara actual
//     Vector2 cameraOffset = getCameraPosition();
//
//     // CAMBIO: Convertir posición world a screen para el AnimationManager
//     Vector2 playerScreenPosition(
//         playerWorldPosition.x - cameraOffset.x,
//         playerWorldPosition.y - cameraOffset.y
//     );
//
//     // CAMBIO: Actualizar transform con coordenadas SCREEN
//     playerTransform.position = playerScreenPosition;
//     playerInstance->setTransform(playerTransform);
//
//     // DEBUG: Imprimir ambas coordenadas para verificar
//     static int debugCounter = 0;
//     if (debugCounter++ % 60 == 0) { // Cada segundo aprox
//         std::cout << "COORDINATE DEBUG:" << std::endl;
//         std::cout << "  World position: (" << playerWorldPosition.x << ", " << playerWorldPosition.y << ")" << std::endl;
//         std::cout << "  Screen position: (" << playerScreenPosition.x << ", " << playerScreenPosition.y << ")" << std::endl;
//         std::cout << "  Camera offset: (" << cameraOffset.x << ", " << cameraOffset.y << ")" << std::endl;
//     }
//
//     // PASO 5: CAMBIAR ANIMACIÓN SI ES NECESARIO
//     // =========================================
//     if (newAnimation != currentAnimation_) {
//         // IMPORTANTE: switchPlayerAnimation creará una nueva instancia,
//         // así que necesitamos asegurar que también tenga las coordenadas screen correctas
//         switchPlayerAnimation(newAnimation);
//         currentAnimation_ = newAnimation;
//
//         // Después de cambiar animación, asegurar que la nueva instancia tenga coordenadas screen
//         AnimationInstance* newPlayerInstance = animationManager_->getInstance(playerInstanceId_);
//         if (newPlayerInstance) {
//             SpriteTransform newTransform = newPlayerInstance->getTransform();
//             newTransform.position = playerScreenPosition; // CAMBIO: Aplicar screen coords a nueva instancia
//             newPlayerInstance->setTransform(newTransform);
//         }
//     }
// }
//     /**
//      * @brief Cambiar animación del jugador
//      * CHANGE: Sin cambios, mantiene compatibilidad
//      */
//     void switchPlayerAnimation(PlayerAnimation animation) {
//         AnimationInstance* playerInstance = animationManager_->getInstance(playerInstanceId_);
//         if (!playerInstance) return;
//
//         AnimationID newAnimationId = idleAnimationId_;  // Default
//
//         switch (animation) {
//             case PlayerAnimation::IDLE:
//                 newAnimationId = idleAnimationId_;
//                 break;
//             case PlayerAnimation::UP:
//                 newAnimationId = upAnimationId_;
//                 break;
//             case PlayerAnimation::DOWN:
//                 newAnimationId = downAnimationId_;
//                 break;
//             case PlayerAnimation::LEFT:
//                 newAnimationId = leftAnimationId_;
//                 break;
//             case PlayerAnimation::RIGHT:
//                 newAnimationId = rightAnimationId_;
//                 break;
//         }
//
//         // Cambiar animación creando nueva instancia
//         SpriteTransform currentTransform = playerInstance->getTransform();
//
//         // Remover instancia actual
//         animationManager_->removeInstance(playerInstanceId_);
//
//         // Crear nueva instancia con nueva animación
//         playerInstanceId_ = animationManager_->createInstance(newAnimationId, currentTransform);
//
//         // Iniciar nueva animación
//         AnimationInstance* newInstance = animationManager_->getInstance(playerInstanceId_);
//         if (newInstance) {
//             newInstance->play();
//         }
//     }
//
//     /**
//      * @brief Renderizar la escena
//      * CHANGE: Integrado sistema de cámaras y mundo expandido
//      */
//     void render() {
//         // 1. LIMPIAR PANTALLA
//         SDL_SetRenderDrawColor(renderer_, 30, 60, 30, 255);  // Verde oscuro más realista
//         SDL_RenderClear(renderer_);
//
//         // 2. OBTENER POSICIÓN DE CÁMARA PARA RENDERIZADO
//         Vector2 cameraOffset = getCameraPosition();
//
//         // 3. RENDERIZAR BACKGROUND DEL MUNDO
//         renderWorldBackground(cameraOffset);
//
//         // 4. RENDERIZAR TODAS LAS ANIMACIONES CON OFFSET DE CÁMARA
//         Vector2 zeroCameraOffset{0.0f, 0.0f};  // Sin offset porque coordenadas ya están en screen space
//         Vector2 viewportSize{static_cast<float>(WINDOW_WIDTH), static_cast<float>(WINDOW_HEIGHT)};
//         animationManager_->render(renderer_, zeroCameraOffset, viewportSize);
//
//         // 5. RENDERIZAR UI DE DEBUG Y HUD
//         renderUI();
//
//         // 6. PRESENTAR AL BUFFER DE PANTALLA
//         SDL_RenderPresent(renderer_);
//     }
//
//     /**
//      * @brief Obtener posición actual de la cámara
//      * CHANGE: Nuevo método para obtener posición de cámara activa
//      */
//     Vector2 getCameraPosition() {
//         const BaseCamera* activeCamera = cameraManager_->getActiveCamera();
//         if (activeCamera) {
//             Vector3 pos3D = activeCamera->getPosition();
//             // CAMBIO: Calcular offset para centrar la vista en la pantalla
//             Vector2 cameraWorldPos(pos3D.x, pos3D.y);
//             Vector2 renderOffset(
//                 cameraWorldPos.x - WINDOW_WIDTH / 2.0f,
//                 cameraWorldPos.y - WINDOW_HEIGHT / 2.0f
//             );
//             return renderOffset;
//         }
//         return Vector2{0.0f, 0.0f};  // Fallback
//     }
//
//     /**
//      * @brief Renderizar background del mundo expandido
//      * CHANGE: Nuevo método para renderizar mundo grande con culling
//      */
//     void renderWorldBackground(const Vector2& cameraOffset) {
//         const int TILE_SIZE = 64;
//         const int GRASS_VARIANT_COUNT = 3;
//
//         // Calcular área visible basada en cámara
//         int startX = static_cast<int>(cameraOffset.x / TILE_SIZE);
//         int startY = static_cast<int>(cameraOffset.y / TILE_SIZE);
//         int endX = static_cast<int>((cameraOffset.x + WINDOW_WIDTH) / TILE_SIZE) + 1;
//         int endY = static_cast<int>((cameraOffset.y + WINDOW_HEIGHT) / TILE_SIZE) + 1;
//
//         // Clampear a límites del mundo
//         startX = std::max(0, startX);
//         startY = std::max(0, startY);
//         endX = std::min(static_cast<int>(WORLD_WIDTH / TILE_SIZE), endX);
//         endY = std::min(static_cast<int>(WORLD_HEIGHT / TILE_SIZE), endY);
//
//         // Renderizar tiles visibles
//         for (int y = startY; y < endY; ++y) {
//             for (int x = startX; x < endX; ++x) {
//                 int worldX = x * TILE_SIZE;
//                 int worldY = y * TILE_SIZE;
//
//                 // Convertir coordenadas del mundo a coordenadas de pantalla
//                 int screenX = worldX - static_cast<int>(cameraOffset.x);
//                 int screenY = worldY - static_cast<int>(cameraOffset.y);
//
//                 SDL_Rect tileRect = {screenX, screenY, TILE_SIZE, TILE_SIZE};
//
//                 // Variantes de césped basadas en posición
//                 int variant = (x + y * 7) % GRASS_VARIANT_COUNT;
//
//                 switch (variant) {
//                     case 0:
//                         SDL_SetRenderDrawColor(renderer_, 50, 120, 50, 255);  // Verde base
//                         break;
//                     case 1:
//                         SDL_SetRenderDrawColor(renderer_, 45, 115, 45, 255);  // Verde oscuro
//                         break;
//                     case 2:
//                         SDL_SetRenderDrawColor(renderer_, 55, 125, 55, 255);  // Verde claro
//                         break;
//                 }
//
//                 SDL_RenderFillRect(renderer_, &tileRect);
//
//                 // Agregar líneas de grid opcionales
//                 if (showGrid_) {
//                     SDL_SetRenderDrawColor(renderer_, 40, 100, 40, 128);
//                     SDL_RenderDrawRect(renderer_, &tileRect);
//                 }
//             }
//         }
//
//         // Renderizar límites del mundo
//         renderWorldBoundaries(cameraOffset);
//     }
//
//     /**
//      * @brief Renderizar límites visuales del mundo
//      * CHANGE: Nuevo método para mostrar bordes del mundo
//      */
//     void renderWorldBoundaries(const Vector2& cameraOffset) {
//         SDL_SetRenderDrawColor(renderer_, 100, 50, 50, 255);  // Rojo para límites
//
//         // Convertir límites del mundo a coordenadas de pantalla
//         int leftBorder = -static_cast<int>(cameraOffset.x);
//         int topBorder = -static_cast<int>(cameraOffset.y);
//         int rightBorder = WORLD_WIDTH - static_cast<int>(cameraOffset.x);
//         int bottomBorder = WORLD_HEIGHT - static_cast<int>(cameraOffset.y);
//
//         const int BORDER_WIDTH = 5;
//
//         // Solo dibujar bordes si están en el viewport
//         if (leftBorder >= -BORDER_WIDTH && leftBorder <= WINDOW_WIDTH) {
//             SDL_Rect leftRect = {leftBorder, 0, BORDER_WIDTH, WINDOW_HEIGHT};
//             SDL_RenderFillRect(renderer_, &leftRect);
//         }
//         if (rightBorder >= 0 && rightBorder <= WINDOW_WIDTH + BORDER_WIDTH) {
//             SDL_Rect rightRect = {rightBorder, 0, BORDER_WIDTH, WINDOW_HEIGHT};
//             SDL_RenderFillRect(renderer_, &rightRect);
//         }
//         if (topBorder >= -BORDER_WIDTH && topBorder <= WINDOW_HEIGHT) {
//             SDL_Rect topRect = {0, topBorder, WINDOW_WIDTH, BORDER_WIDTH};
//             SDL_RenderFillRect(renderer_, &topRect);
//         }
//         if (bottomBorder >= 0 && bottomBorder <= WINDOW_HEIGHT + BORDER_WIDTH) {
//             SDL_Rect bottomRect = {0, bottomBorder, WINDOW_WIDTH, BORDER_WIDTH};
//             SDL_RenderFillRect(renderer_, &bottomRect);
//         }
//     }
//
//     /**
//      * @brief Renderizar UI y información de debug
//      * CHANGE: Actualizado para mostrar información de cámara
//      */
//     void renderUI() {
//         // === INDICADOR DE ESTADO DEL JUGADOR ===
//         SDL_Rect statusRect = {10, 10, 200, 30};
//
//         // Color basado en animación actual
//         switch (currentAnimation_) {
//             case PlayerAnimation::IDLE:
//                 SDL_SetRenderDrawColor(renderer_, 255, 255, 255, 128);  // Blanco
//                 break;
//             case PlayerAnimation::UP:
//                 SDL_SetRenderDrawColor(renderer_, 0, 255, 0, 128);      // Verde
//                 break;
//             case PlayerAnimation::DOWN:
//                 SDL_SetRenderDrawColor(renderer_, 255, 0, 0, 128);      // Rojo
//                 break;
//             case PlayerAnimation::LEFT:
//                 SDL_SetRenderDrawColor(renderer_, 0, 0, 255, 128);      // Azul
//                 break;
//             case PlayerAnimation::RIGHT:
//                 SDL_SetRenderDrawColor(renderer_, 255, 255, 0, 128);    // Amarillo
//                 break;
//         }
//         SDL_RenderFillRect(renderer_, &statusRect);
//
//         // === INDICADOR DE CÁMARA ===
//         SDL_Rect cameraRect = {10, 50, 200, 20};
//         const BaseCamera* activeCamera = cameraManager_->getActiveCamera();
//
//         if (activeCamera) {
//             if (activeCamera->getId() == mainCameraId_) {
//                 SDL_SetRenderDrawColor(renderer_, 0, 255, 255, 128);    // Cyan = Follow mode
//             } else {
//                 SDL_SetRenderDrawColor(renderer_, 255, 0, 255, 128);    // Magenta = Free mode
//             }
//         } else {
//             SDL_SetRenderDrawColor(renderer_, 128, 128, 128, 128);      // Gris = No camera
//         }
//         SDL_RenderFillRect(renderer_, &cameraRect);
//
//         // === INDICADOR DE ZOOM ===
//         const Camera2D* camera2D = cameraManager_->getCamera2D(cameraManager_->getActiveCameraId());
//         if (camera2D) {
//             float zoom = camera2D->getZoom();
//             int zoomBarWidth = static_cast<int>(zoom * 100);  // 1.0 zoom = 100px width
//             zoomBarWidth = std::clamp(zoomBarWidth, 10, 300);
//
//             SDL_Rect zoomRect = {10, 80, zoomBarWidth, 15};
//             SDL_SetRenderDrawColor(renderer_, 255, 255, 0, 128);  // Amarillo para zoom
//             SDL_RenderFillRect(renderer_, &zoomRect);
//         }
//
//         // === MINI MAPA ===
//         renderMiniMap();
//
//         // === INDICADOR DE GAMEPAD ===
//         if (inputManager_->isGamepadConnected(0)) {
//             SDL_Rect gamepadRect = {WINDOW_WIDTH - 50, 10, 40, 20};
//             SDL_SetRenderDrawColor(renderer_, 0, 255, 0, 128);  // Verde = conectado
//             SDL_RenderFillRect(renderer_, &gamepadRect);
//         }
//
//         // === INDICADOR DE SHAKE ===
//         if (cameraManager_->isCameraShaking(cameraManager_->getActiveCameraId())) {
//             SDL_Rect shakeRect = {WINDOW_WIDTH - 100, 10, 40, 20};
//             SDL_SetRenderDrawColor(renderer_, 255, 0, 0, 200);  // Rojo brillante = shake activo
//             SDL_RenderFillRect(renderer_, &shakeRect);
//         }
//     }
//
//     /**
//      * @brief Renderizar mini mapa del mundo
//      * CHANGE: Nuevo método para mostrar posición en el mundo
//      */
//     void renderMiniMap() {
//         const int MINIMAP_SIZE = 150;
//         const int MINIMAP_X = WINDOW_WIDTH - MINIMAP_SIZE - 10;
//         const int MINIMAP_Y = WINDOW_HEIGHT - MINIMAP_SIZE - 10;
//
//         // Background del minimapa
//         SDL_Rect minimapBg = {MINIMAP_X - 5, MINIMAP_Y - 5, MINIMAP_SIZE + 10, MINIMAP_SIZE + 10};
//         SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 180);
//         SDL_RenderFillRect(renderer_, &minimapBg);
//
//         // Área del mundo en el minimapa
//         SDL_Rect worldRect = {MINIMAP_X, MINIMAP_Y, MINIMAP_SIZE, MINIMAP_SIZE};
//         SDL_SetRenderDrawColor(renderer_, 50, 100, 50, 255);
//         SDL_RenderFillRect(renderer_, &worldRect);
//
//         // Posición del jugador en el minimapa
//         AnimationInstance* playerInstance = animationManager_->getInstance(playerInstanceId_);
//         if (playerInstance) {
//             const Vector2& playerPos = playerInstance->getTransform().position;
//
//             int playerMapX = MINIMAP_X + static_cast<int>((playerPos.x / WORLD_WIDTH) * MINIMAP_SIZE);
//             int playerMapY = MINIMAP_Y + static_cast<int>((playerPos.y / WORLD_HEIGHT) * MINIMAP_SIZE);
//
//             SDL_Rect playerDot = {playerMapX - 3, playerMapY - 3, 6, 6};
//             SDL_SetRenderDrawColor(renderer_, 255, 255, 255, 255);  // Blanco para jugador
//             SDL_RenderFillRect(renderer_, &playerDot);
//         }
//
//         // Área visible de la cámara en el minimapa
//         const BaseCamera* activeCamera = cameraManager_->getActiveCamera();
//         if (activeCamera) {
//             Vector3 cameraPos3D = activeCamera->getPosition();
//             Vector2 cameraPos(cameraPos3D.x, cameraPos3D.y);
//
//             // Calcular área visible (aproximada)
//             const Camera2D* camera2D = cameraManager_->getCamera2D(cameraManager_->getActiveCameraId());
//             float zoom = camera2D ? camera2D->getZoom() : 1.0f;
//
//             float visibleWidth = WINDOW_WIDTH / zoom;
//             float visibleHeight = WINDOW_HEIGHT / zoom;
//
//             int viewX = MINIMAP_X + static_cast<int>(((cameraPos.x - visibleWidth/2) / WORLD_WIDTH) * MINIMAP_SIZE);
//             int viewY = MINIMAP_Y + static_cast<int>(((cameraPos.y - visibleHeight/2) / WORLD_HEIGHT) * MINIMAP_SIZE);
//             int viewW = static_cast<int>((visibleWidth / WORLD_WIDTH) * MINIMAP_SIZE);
//             int viewH = static_cast<int>((visibleHeight / WORLD_HEIGHT) * MINIMAP_SIZE);
//
//             SDL_Rect viewRect = {viewX, viewY, viewW, viewH};
//             SDL_SetRenderDrawColor(renderer_, 255, 255, 0, 100);  // Amarillo semitransparente
//             SDL_RenderFillRect(renderer_, &viewRect);
//         }
//     }
//
//     /**
//      * @brief Mostrar estadísticas de performance
//      * CHANGE: Agregadas estadísticas de cámara
//      */
//     void showStats(int frameCount, std::chrono::steady_clock::duration elapsed) {
//         auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
//         double fps = frameCount * 1000.0 / elapsedMs;
//
//         std::cout << "\n--- Game Performance Stats ---" << std::endl;
//         std::cout << "FPS: " << fps << std::endl;
//
//         // ResourceManager stats
//         std::cout << "Resource Memory: " << (resourceManager_->getMemoryUsage() / 1024.0 / 1024.0) << " MB" << std::endl;
//         std::cout << "Resources Loaded: " << resourceManager_->getResourceCount() << std::endl;
//
//         // AnimationManager stats
//         std::cout << "Animation Instances: " << animationManager_->getInstanceCount() << std::endl;
//         std::cout << "Playing Instances: " << animationManager_->getPlayingInstanceCount() << std::endl;
//         std::cout << "Animation Memory: " << (animationManager_->getMemoryUsage() / 1024.0) << " KB" << std::endl;
//
//         // CameraManager stats ← NUEVO
//         std::cout << "Cameras: " << cameraManager_->getCameraCount() << std::endl;
//         std::cout << "Active Transitions: " << cameraManager_->getActiveTransitionCount() << std::endl;
//         std::cout << "Camera Memory: " << (cameraManager_->getMemoryUsage() / 1024.0) << " KB" << std::endl;
//
//         // Camera position and zoom
//         const BaseCamera* activeCamera = cameraManager_->getActiveCamera();
//         if (activeCamera) {
//             Vector3 cameraPos = activeCamera->getPosition();
//             std::cout << "Camera Position: (" << cameraPos.x << ", " << cameraPos.y << ")" << std::endl;
//
//             const Camera2D* camera2D = cameraManager_->getCamera2D(cameraManager_->getActiveCameraId());
//             if (camera2D) {
//                 std::cout << "Camera Zoom: " << camera2D->getZoom() << "x" << std::endl;
//                 std::cout << "Camera Mode: " << (camera2D->getMode() == CameraMode::FOLLOW_TARGET ? "FOLLOW" : "FREE") << std::endl;
//             }
//         }
//
//         // InputManager stats
//         if (inputManager_->isGamepadConnected(0)) {
//             std::cout << "Gamepad: " << inputManager_->getGamepadName(0) << std::endl;
//         }
//
//         // Player position in world
//         AnimationInstance* playerInstance = animationManager_->getInstance(playerInstanceId_);
//         if (playerInstance) {
//             const auto& pos = playerInstance->getTransform().position;
//             std::cout << "Player Position: (" << pos.x << ", " << pos.y << ")" << std::endl;
//             float worldPercX = (pos.x / WORLD_WIDTH) * 100.0f;
//             float worldPercY = (pos.y / WORLD_HEIGHT) * 100.0f;
//             std::cout << "World Coverage: " << worldPercX << "%, " << worldPercY << "%" << std::endl;
//         }
//     }
//
//     /**
//      * @brief Imprimir controles en consola
//      * CHANGE: Nuevos controles de cámara agregados
//      */
//     void printControls() {
//         std::cout << "=== GAME CONTROLS ===" << std::endl;
//         std::cout << "Player Movement:" << std::endl;
//         std::cout << "  WASD: Move player around the world" << std::endl;
//         std::cout << "  Gamepad Left Stick: Move player" << std::endl;
//         std::cout << "\nCamera Controls:" << std::endl;
//         std::cout << "  C: Switch between FOLLOW and FREE camera modes" << std::endl;
//         std::cout << "  Arrow Keys: Manual camera control (FREE mode only)" << std::endl;
//         std::cout << "  +/-: Zoom in/out" << std::endl;
//         std::cout << "  Space: Camera shake effect" << std::endl;
//         std::cout << "  Gamepad Right Stick: Manual camera control" << std::endl;
//         std::cout << "  Gamepad Y: Switch camera mode" << std::endl;
//         std::cout << "  Gamepad X: Camera shake" << std::endl;
//         std::cout << "\nOther:" << std::endl;
//         std::cout << "  ESC: Quit game" << std::endl;
//         std::cout << "  G: Toggle grid display" << std::endl;
//         std::cout << "\nWorld Size: " << WORLD_WIDTH << "x" << WORLD_HEIGHT << " pixels" << std::endl;
//         std::cout << "Viewport: " << WINDOW_WIDTH << "x" << WINDOW_HEIGHT << " pixels" << std::endl;
//         std::cout << "=======================" << std::endl;
//     }
//
//     /**
//      * @brief Limpiar recursos al cerrar
//      * CHANGE: Agregado cleanup de CameraManager
//      */
//     void cleanup() {
//         std::cout << "\nCleaning up game resources..." << std::endl;
//
//         // 1. LIMPIAR HANDLES DE RECURSOS
//         playerTexture_.reset();
//
//         // 2. LIMPIAR MANAGERS (en orden inverso de inicialización)
//         if (animationManager_) {
//             std::cout << "Final animation stats: " << animationManager_->getInstanceCount() << " instances" << std::endl;
//             animationManager_->shutdown();
//             animationManager_.reset();
//         }
//
//         if (cameraManager_) {  // ← NUEVO
//             std::cout << "Final camera stats: " << cameraManager_->getCameraCount() << " cameras" << std::endl;
//             cameraManager_->shutdown();
//             cameraManager_.reset();
//         }
//
//         if (inputManager_) {
//             inputManager_->shutdown();
//             inputManager_.reset();
//         }
//
//         if (resourceManager_) {
//             std::cout << "Final resource stats - Memory: " << (resourceManager_->getMemoryUsage() / 1024.0 / 1024.0)
//                      << " MB, Resources: " << resourceManager_->getResourceCount() << std::endl;
//             resourceManager_->clear();
//             resourceManager_.reset();
//         }
//
//         // 3. LIMPIAR SDL
//         if (renderer_) {
//             SDL_DestroyRenderer(renderer_);
//             renderer_ = nullptr;
//         }
//
//         if (window_) {
//             SDL_DestroyWindow(window_);
//             window_ = nullptr;
//         }
//
//         // 4. CERRAR SDL_IMAGE Y SDL
//         IMG_Quit();
//         SDL_Quit();
//
//         std::cout << "Cleanup completed!" << std::endl;
//     }
//
// private:
//     // === CONSTANTES DE MUNDO Y VENTANA ===
//     static constexpr int WINDOW_WIDTH = 1024;               // Tamaño de ventana (viewport)
//     static constexpr int WINDOW_HEIGHT = 768;             // ← NUEVO: Mantiene proporción 4:3
//     static constexpr float PLAYER_SPEED = 300.0f;           // ← INCREMENTADO: Más velocidad para mundo grande
//     static constexpr float CAMERA_SPEED = 400.0f;           // ← NUEVO: Velocidad de cámara manual
//
//     // === SDL OBJECTS ===
//     SDL_Window* window_ = nullptr;
//     SDL_Renderer* renderer_ = nullptr;
//
//     // === MANAGERS ===
//     std::unique_ptr<ResourceManager> resourceManager_;
//     std::unique_ptr<InputManager> inputManager_;
//     std::unique_ptr<AnimationManager> animationManager_;
//     std::unique_ptr<CameraManager> cameraManager_;          // ← NUEVO
//
//     // === RESOURCE HANDLES ===
//     ResourceHandle<TextureResource> playerTexture_;
//
//     // === ANIMATION IDS ===
//     SpriteSheetID playerSpriteSheetId_ = INVALID_SPRITESHEET_ID;
//     AnimationID idleAnimationId_ = INVALID_ANIMATION_ID;
//     AnimationID upAnimationId_ = INVALID_ANIMATION_ID;
//     AnimationID downAnimationId_ = INVALID_ANIMATION_ID;
//     AnimationID leftAnimationId_ = INVALID_ANIMATION_ID;
//     AnimationID rightAnimationId_ = INVALID_ANIMATION_ID;
//
//     // === ANIMATION INSTANCE ===
//     AnimationInstanceID playerInstanceId_ = INVALID_INSTANCE_ID;
//     PlayerAnimation currentAnimation_ = PlayerAnimation::IDLE;
//
//     // === CAMERA IDS ===  ← NUEVO
//     CameraID mainCameraId_ = INVALID_CAMERA_ID;             // Cámara que sigue al jugador
//     CameraID freeCameraId_ = INVALID_CAMERA_ID;             // Cámara de control libre
//
//     // === INPUT ACTION IDS ===
//     ActionID moveUpAction_ = INVALID_ACTION_ID;
//     ActionID moveDownAction_ = INVALID_ACTION_ID;
//     ActionID moveLeftAction_ = INVALID_ACTION_ID;
//     ActionID moveRightAction_ = INVALID_ACTION_ID;
//     ActionID exitAction_ = INVALID_ACTION_ID;
//
//     // === CAMERA ACTION IDS ===  ← NUEVO
//     ActionID switchCameraAction_ = INVALID_ACTION_ID;       // Cambiar modo de cámara
//     ActionID zoomInAction_ = INVALID_ACTION_ID;             // Zoom in
//     ActionID zoomOutAction_ = INVALID_ACTION_ID;            // Zoom out
//     ActionID shakeAction_ = INVALID_ACTION_ID;              // Camera shake
//
//     // === INPUT AXIS IDS ===
//     AxisID horizontalAxis_ = INVALID_AXIS_ID;               // Movimiento horizontal del jugador
//     AxisID verticalAxis_ = INVALID_AXIS_ID;                 // Movimiento vertical del jugador
//     AxisID cameraHorizontalAxis_ = INVALID_AXIS_ID;         // ← NUEVO: Control manual de cámara
//     AxisID cameraVerticalAxis_ = INVALID_AXIS_ID;           // ← NUEVO: Control manual de cámara
//
//     // === DEBUG FLAGS ===  ← NUEVO
//     bool showGrid_ = false;                                 // Mostrar grid de debug
//     bool showDebugInfo_ = true;                             // Mostrar información de debug
// };
//
// /**
//  * @brief Función principal
//  * CHANGE: Actualizada descripción para incluir sistema de cámaras
//  */
// int main(int argc, char* argv[]) {
//     std::cout << "=== Game Demo with Camera System ===" << std::endl;
//     std::cout << "This demo shows integrated ResourceManager, InputManager, AnimationManager, and CameraManager" << std::endl;
//     std::cout << "Features:" << std::endl;
//     std::cout << "  - Large explorable world (" << GameDemo::WORLD_WIDTH << "x" << GameDemo::WORLD_HEIGHT << " pixels)" << std::endl;
//     std::cout << "  - Player movement with WASD keys and gamepad" << std::endl;
//     std::cout << "  - Advanced camera system with follow and free modes" << std::endl;
//     std::cout << "  - Camera zoom, smooth transitions, and shake effects" << std::endl;
//     std::cout << "  - Animated sprite with different directions" << std::endl;
//     std::cout << "  - Viewport culling for performance" << std::endl;
//     std::cout << "  - Mini-map showing world position" << std::endl;
//     std::cout << "  - Resource management with TextureResource" << std::endl;
//     std::cout << "  - Performance monitoring and stats" << std::endl;
//     std::cout << "\nMake sure you have the following asset:" << std::endl;
//     std::cout << "  assets/player.png (sprite sheet with specified frame layout)" << std::endl;
//     std::cout << "\nStarting game..." << std::endl;
//
//     // Crear instancia del juego
//     GameDemo game;
//
//     // Inicializar
//     if (!game.initialize()) {
//         std::cerr << "\nFailed to initialize game!" << std::endl;
//         std::cerr << "Please ensure:" << std::endl;
//         std::cerr << "  1. SDL2 and SDL2_image are properly installed" << std::endl;
//         std::cerr << "  2. assets/player.png exists and is accessible" << std::endl;
//         std::cerr << "  3. The sprite sheet follows the frame layout specifications" << std::endl;
//         return -1;
//     }
//
//     // Ejecutar loop principal
//     try {
//         game.run();
//     } catch (const std::exception& e) {
//         std::cerr << "\nException during game loop: " << e.what() << std::endl;
//         return -1;
//     }
//
//     // Limpiar recursos
//     game.cleanup();
//
//     std::cout << "\nGame demo completed successfully!" << std::endl;
//     std::cout << "Thanks for testing the integrated camera system!" << std::endl;
//     return 0;
// }

// ====================================================================
// TEST main.cpp - Memory Manager Example
// ====================================================================

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

    // Configurar el sistema de memoria
    MemoryManagerConfig config;
    config.mainHeapSize = 256 * 1024 * 1024;        // 256 MB para el heap principal
    config.frameStackSize = 4 * 1024 * 1024;        // 4 MB para stack por frame
    config.frameLinearSize = 8 * 1024 * 1024;       // 8 MB para linear por frame
    config.frameBufferCount = 2;                     // Double buffering (2 frames)
    config.renderingPoolSize = 16 * 1024 * 1024;    // 16 MB para pool de rendering
    config.physicsPoolSize = 8 * 1024 * 1024;       // 8 MB para pool de física

    // Agregar un pool personalizado para partículas
    config.customPools.push_back({
        sizeof(Particle),                            // Tamaño de cada partícula
        100000,                                        // Máximo 100000 partículas
        MemoryCategory::PARTICLES                   // Categoría PARTICLES
    });

    // Inicializar el sistema
    if (!memoryManager.initialize(config)) {
        std::cerr << "Failed to initialize memory manager!" << std::endl;
        return -1;
    }

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
