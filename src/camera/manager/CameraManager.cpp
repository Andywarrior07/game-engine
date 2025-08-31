/**
 * @file CameraManager.cpp
 * @brief Camera management system implementation
 * @author Engine Development Team
 * @date 2025
 */

#include "CameraManager.h"
#include "../cameras/Camera2D.h"
#include "../cameras/Camera3D.h"
#include "../effects/CameraShake.h"
#include "../transitions/CameraTransition.h"
#include "../input/CameraInputHandler.h"
#include <iostream>
#include <algorithm>

namespace engine::camera {
    // ========================================================================
    // CONSTRUCTOR & DESTRUCTOR
    // ========================================================================

    CameraManager::CameraManager(const CameraManagerConfig& config)
        : config_(config)
          , registry_(std::make_unique<CameraRegistry>())
          , lastUpdateTime_(std::chrono::steady_clock::now()) {
        // Reserve space for transitions and shakes
        transitions_.reserve(config_.maxTransitions);
        shakeStates_.reserve(config_.maxShakeEffects);
    }

    CameraManager::~CameraManager() {
        shutdown();
    }

    // ========================================================================
    // LIFECYCLE
    // ========================================================================

    bool CameraManager::initialize() {
        if (initialized_) {
            return true;
        }

        std::lock_guard lock(managerMutex_);

        if (config_.enableCameraLogging) {
            std::cout << "[CameraManager] Initializing..." << std::endl;
        }

        // Validate configuration
        if (!config_.validate()) {
            std::cerr << "[CameraManager] Invalid configuration" << std::endl;
            return false;
        }

        // Initialize timing
        lastUpdateTime_ = std::chrono::steady_clock::now();

        initialized_ = true;

        if (config_.enableCameraLogging) {
            std::cout << "[CameraManager] Initialized successfully" << std::endl;
        }

        return true;
    }

    void CameraManager::shutdown() {
        if (!initialized_) {
            return;
        }

        std::lock_guard lock(managerMutex_);

        if (config_.enableCameraLogging) {
            std::cout << "[CameraManager] Shutting down..." << std::endl;
        }

        // Clear all resources
        registry_->clear();
        transitions_.clear();
        shakeStates_.clear();

        // Reset state
        activeCameraId_ = INVALID_CAMERA_ID;
        nextCameraId_ = 1;
        nextTransitionId_ = 1;
        initialized_ = false;

        if (config_.enableCameraLogging) {
            std::cout << "[CameraManager] Shutdown complete" << std::endl;
        }
    }

    void CameraManager::update(const float deltaTime) {
        if (!initialized_) {
            return;
        }

        std::lock_guard lock(managerMutex_);

        const auto startTime = std::chrono::steady_clock::now();

        // Update input handler if available
        if (inputHandler_) {
            inputHandler_->update(deltaTime);
        }

        // Update all enabled cameras
        registry_->forEachCamera([&](BaseCamera* camera) {
            if (camera && camera->isEnabled()) {
                camera->update(deltaTime);
                camerasUpdated_.fetch_add(1, std::memory_order_relaxed);
            }
        });

        // Update transitions
        updateTransitions(deltaTime);

        // Update shake effects
        updateShakeEffects(deltaTime);

        // Cleanup completed transitions and inactive shakes
        cleanupCompletedTransitions();
        cleanupInactiveShakes();

        lastUpdateTime_ = startTime;
    }

    // ========================================================================
    // CAMERA CREATION AND MANAGEMENT
    // ========================================================================

    CameraID CameraManager::createCamera2D(const std::string& name) {
        if (!initialized_) {
            std::cerr << "[CameraManager] Not initialized" << std::endl;
            return INVALID_CAMERA_ID;
        }

        std::lock_guard lock(managerMutex_);

        // Check camera limit
        if (registry_->getCount() >= static_cast<size_t>(config_.maxCameras)) {
            std::cerr << "[CameraManager] Maximum camera limit reached" << std::endl;
            return INVALID_CAMERA_ID;
        }

        CameraID cameraId = nextCameraId_++;
        std::string cameraName = name.empty() ? ("Camera2D_" + std::to_string(cameraId)) : name;

        // Create camera
        auto camera = std::make_unique<Camera2D>(cameraId, cameraName);
        camera->setSmoothingSpeed(config_.defaultSmoothingSpeed);

        // Register camera
        const CameraID registeredId = registry_->registerCamera(std::move(camera));

        if (config_.enableCameraLogging) {
            std::cout << "[CameraManager] Created 2D camera: " << cameraName
                << " (ID: " << registeredId << ")" << std::endl;
        }

        return registeredId;
    }

    CameraID CameraManager::createCamera3D(const std::string& name, bool perspective) {
        if (!initialized_) {
            std::cerr << "[CameraManager] Not initialized" << std::endl;
            return INVALID_CAMERA_ID;
        }

        std::lock_guard lock(managerMutex_);

        // Check camera limit
        if (registry_->getCount() >= static_cast<size_t>(config_.maxCameras)) {
            std::cerr << "[CameraManager] Maximum camera limit reached" << std::endl;
            return INVALID_CAMERA_ID;
        }

        CameraID cameraId = nextCameraId_++;
        std::string cameraName = name.empty() ? ("Camera3D_" + std::to_string(cameraId)) : name;

        // Create camera
        auto camera = std::make_unique<Camera3D>(cameraId, cameraName, perspective);
        camera->setSmoothingSpeed(config_.defaultSmoothingSpeed);
        camera->setFOV(config_.defaultFOV);
        camera->setClippingPlanes(config_.defaultNearPlane, config_.defaultFarPlane);

        // Register camera
        const CameraID registeredId = registry_->registerCamera(std::move(camera));

        if (config_.enableCameraLogging) {
            std::cout << "[CameraManager] Created 3D camera: " << cameraName
                << " (ID: " << registeredId << ", Perspective: "
                << (perspective ? "Yes" : "No") << ")" << std::endl;
        }

        return registeredId;
    }

    bool CameraManager::removeCamera(const CameraID cameraId) {
        if (!initialized_) {
            return false;
        }

        std::lock_guard lock(managerMutex_);

        // Stop all transitions and shakes for this camera
        stopAllTransitions(cameraId);
        stopCameraShake(cameraId);

        // Clear active camera if it's being removed
        if (activeCameraId_ == cameraId) {
            activeCameraId_ = INVALID_CAMERA_ID;
        }

        // Unregister from registry
        const bool result = registry_->unregisterCamera(cameraId);

        if (result && config_.enableCameraLogging) {
            std::cout << "[CameraManager] Removed camera ID: " << cameraId << std::endl;
        }

        return result;
    }

    BaseCamera* CameraManager::getCamera(const CameraID cameraId) {
        return registry_->getCamera(cameraId);
    }

    const BaseCamera* CameraManager::getCamera(const CameraID cameraId) const {
        return registry_->getCamera(cameraId);
    }

    CameraID CameraManager::getCameraByName(const std::string& name) const {
        return registry_->getCameraByName(name);
    }

    Camera2D* CameraManager::getCamera2D(const CameraID cameraId) {
        BaseCamera* camera = getCamera(cameraId);

        if (camera && camera->getType() != CameraType::CAMERA_2D) {
            return nullptr;
        }

        return static_cast<Camera2D*>(camera);
    }

    Camera3D* CameraManager::getCamera3D(const CameraID cameraId) {
        BaseCamera* camera = getCamera(cameraId);

        if (!camera ||
            (camera->getType() != CameraType::CAMERA_3D_PERSPECTIVE &&
                camera->getType() != CameraType::CAMERA_3D_ORTHOGRAPHIC)) {
            return nullptr;
        }

        return static_cast<Camera3D*>(camera);
    }

    // ========================================================================
    // ACTIVE CAMERA MANAGEMENT
    // ========================================================================

    bool CameraManager::setActiveCamera(const CameraID cameraId) {
        if (cameraId != INVALID_CAMERA_ID && !registry_->hasCamera(cameraId)) {
            return false;
        }

        std::lock_guard lock(managerMutex_);

        // Deactivate current active camera
        if (activeCameraId_ != INVALID_CAMERA_ID) {
            if (BaseCamera* oldCamera = registry_->getCamera(activeCameraId_)) {
                oldCamera->setActive(false);
            }
        }

        // Activate new camera
        activeCameraId_ = cameraId;

        if (cameraId != INVALID_CAMERA_ID) {
            if (BaseCamera* newCamera = registry_->getCamera(cameraId)) {
                newCamera->setActive(true);

                if (config_.enableCameraLogging) {
                    std::cout << "[CameraManager] Set active camera: " << newCamera->getName()
                        << " (ID: " << cameraId << ")" << std::endl;
                }
            }
        }
        else {
            if (config_.enableCameraLogging) {
                std::cout << "[CameraManager] Cleared active camera" << std::endl;
            }
        }

        return true;
    }

    bool CameraManager::setActiveCamera(const std::string& name) {
        const CameraID cameraId = getCameraByName(name);
        return setActiveCamera(cameraId);
    }

    BaseCamera* CameraManager::getActiveCamera() {
        return getCamera(activeCameraId_);
    }

    const BaseCamera* CameraManager::getActiveCamera() const {
        return getCamera(activeCameraId_);
    }

    // ========================================================================
    // CAMERA TRANSITIONS
    // ========================================================================

    TransitionID CameraManager::transitionToPosition(CameraID cameraId, const Vec3& targetPosition,
                                                     const TransitionConfig& config) {
        if (!initialized_ || !config_.enableTransitions) {
            return INVALID_TRANSITION_ID;
        }

        const BaseCamera* camera = getCamera(cameraId);
        if (!camera) {
            return INVALID_TRANSITION_ID;
        }

        std::lock_guard lock(managerMutex_);

        // Check transition limit
        if (transitions_.size() >= static_cast<size_t>(config_.maxTransitions)) {
            std::cerr << "[CameraManager] Maximum transition limit reached" << std::endl;
            return INVALID_TRANSITION_ID;
        }

        TransitionID transitionId = nextTransitionId_++;

        // Create transition
        auto transition = std::make_unique<CameraTransition>(transitionId, cameraId, config);
        transition->setupPosition(camera->getPosition(), targetPosition);
        transition->start();

        transitions_[transitionId] = std::move(transition);

        if (config_.enableCameraLogging) {
            std::cout << "[CameraManager] Started position transition for camera " << camera->getName()
                << " (Transition ID: " << transitionId << ")" << std::endl;
        }

        return transitionId;
    }

    TransitionID CameraManager::transitionToTarget(CameraID cameraId, const Vec3& targetLookAt,
                                                   const TransitionConfig& config) {
        if (!initialized_ || !config_.enableTransitions) {
            return INVALID_TRANSITION_ID;
        }

        const Camera3D* camera = getCamera3D(cameraId);
        if (!camera) {
            std::cerr << "[CameraManager] Target transition only supported for 3D cameras" << std::endl;
            return INVALID_TRANSITION_ID;
        }

        std::lock_guard lock(managerMutex_);

        // Check transition limit
        if (transitions_.size() >= static_cast<size_t>(config_.maxTransitions)) {
            std::cerr << "[CameraManager] Maximum transition limit reached" << std::endl;
            return INVALID_TRANSITION_ID;
        }

        TransitionID transitionId = nextTransitionId_++;

        // Create transition
        auto transition = std::make_unique<CameraTransition>(transitionId, cameraId, config);
        transition->setupTarget(camera->getTarget(), targetLookAt);
        transition->start();

        transitions_[transitionId] = std::move(transition);

        if (config_.enableCameraLogging) {
            std::cout << "[CameraManager] Started target transition for camera " << camera->getName()
                << " (Transition ID: " << transitionId << ")" << std::endl;
        }

        return transitionId;
    }

    TransitionID CameraManager::transitionBetweenCameras(CameraID fromCameraId, const CameraID toCameraId,
                                                         const TransitionConfig& config) {
        if (!initialized_ || !config_.enableTransitions) {
            return INVALID_TRANSITION_ID;
        }

        BaseCamera* fromCamera = getCamera(fromCameraId);
        BaseCamera* toCamera = getCamera(toCameraId);

        if (!fromCamera || !toCamera) {
            return INVALID_TRANSITION_ID;
        }

        std::lock_guard lock(managerMutex_);

        // Check transition limit
        if (transitions_.size() >= static_cast<size_t>(config_.maxTransitions)) {
            std::cerr << "[CameraManager] Maximum transition limit reached" << std::endl;
            return INVALID_TRANSITION_ID;
        }

        TransitionID transitionId = nextTransitionId_++;

        // Create transition
        auto transition = std::make_unique<CameraTransition>(transitionId, fromCameraId, config);

        // Setup position transition
        transition->setupPosition(fromCamera->getPosition(), toCamera->getPosition());

        // Setup target transition for 3D cameras
        const Camera3D* fromCam3D = dynamic_cast<Camera3D*>(fromCamera);
        if (const Camera3D* toCam3D = dynamic_cast<Camera3D*>(toCamera); fromCam3D && toCam3D) {
            transition->setupTarget(fromCam3D->getTarget(), toCam3D->getTarget());
        }

        // Setup zoom transition for 2D cameras
        const Camera2D* fromCam2D = dynamic_cast<Camera2D*>(fromCamera);
        if (const Camera2D* toCam2D = dynamic_cast<Camera2D*>(toCamera); fromCam2D && toCam2D) {
            transition->setupZoom(fromCam2D->getZoom(), toCam2D->getZoom());
        }

        transition->start();
        transitions_[transitionId] = std::move(transition);

        if (config_.enableCameraLogging) {
            std::cout << "[CameraManager] Started camera transition from " << fromCamera->getName()
                << " to " << toCamera->getName() << " (Transition ID: " << transitionId << ")" << std::endl;
        }

        return transitionId;
    }

    bool CameraManager::stopTransition(const TransitionID transitionId) {
        if (transitionId == INVALID_TRANSITION_ID) {
            return false;
        }

        std::lock_guard lock(managerMutex_);

        const auto it = transitions_.find(transitionId);
        if (it == transitions_.end()) return false;

        it->second->stop();

        if (config_.enableCameraLogging) {
            std::cout << "[CameraManager] Stopped transition ID: " << transitionId << std::endl;
        }

        return true;
    }

    int CameraManager::stopAllTransitions(const CameraID cameraId) {
        std::lock_guard lock(managerMutex_);

        int stoppedCount = 0;

        if (cameraId == INVALID_CAMERA_ID) {
            // Stop all transitions
            for (const auto& transition : transitions_ | std::views::values) {
                if (transition->isActive()) {
                    transition->stop();
                    stoppedCount++;
                }
            }
        }
        else {
            // Stop transitions for specific camera
            for (const auto& transition : transitions_ | std::views::values) {
                if (transition->getCameraId() == cameraId && transition->isActive()) {
                    transition->stop();
                    stoppedCount++;
                }
            }
        }

        if (stoppedCount > 0 && config_.enableCameraLogging) {
            std::cout << "[CameraManager] Stopped " << stoppedCount << " transitions" << std::endl;
        }

        return stoppedCount;
    }

    bool CameraManager::isTransitionActive(const TransitionID transitionId) const {
        if (transitionId == INVALID_TRANSITION_ID) {
            return false;
        }

        std::lock_guard lock(managerMutex_);

        const auto it = transitions_.find(transitionId);
        return (it != transitions_.end()) && it->second->isActive();
    }

    // ========================================================================
    // CAMERA EFFECTS
    // ========================================================================

    bool CameraManager::startCameraShake(CameraID cameraId, const ShakeConfig& config) {
        if (!initialized_ || !config_.enableShake) {
            return false;
        }

        const BaseCamera* camera = getCamera(cameraId);
        if (!camera) {
            return false;
        }

        std::lock_guard lock(managerMutex_);

        // Create or update shake state
        auto shake = std::make_unique<CameraShake>(cameraId, config);
        shake->setBasePosition(camera->getPosition());
        shake->setConfig(config);

        shakeStates_[cameraId] = std::move(shake);

        if (config_.enableCameraLogging) {
            std::cout << "[CameraManager] Started shake effect for camera " << camera->getName()
                << " (Intensity: " << config.intensity << ", Duration: " << config.duration << "s)"
                << std::endl;
        }

        return true;
    }

    bool CameraManager::stopCameraShake(const CameraID cameraId) {
        std::lock_guard lock(managerMutex_);

        const auto it = shakeStates_.find(cameraId);

        if (it == shakeStates_.end()) return false;

        it->second->stop();

        // Restore original position
        if (BaseCamera* camera = getCamera(cameraId)) {
            camera->setPosition(it->second->getBasePosition());
        }

        shakeStates_.erase(it);

        if (config_.enableCameraLogging) {
            std::cout << "[CameraManager] Stopped shake effect for camera ID: " << cameraId << std::endl;
        }

        return true;
    }

    int CameraManager::stopAllShakes() {
        std::lock_guard lock(managerMutex_);

        const int stoppedCount = static_cast<int>(shakeStates_.size());

        for (auto& [cameraId, shake] : shakeStates_) {
            shake->stop();

            // Restore original position
            if (BaseCamera* camera = getCamera(cameraId)) {
                camera->setPosition(shake->getBasePosition());
            }
        }

        shakeStates_.clear();

        if (stoppedCount > 0 && config_.enableCameraLogging) {
            std::cout << "[CameraManager] Stopped " << stoppedCount << " shake effects" << std::endl;
        }

        return stoppedCount;
    }

    bool CameraManager::updateShakeIntensity(const CameraID cameraId, const float intensity) {
        std::lock_guard lock(managerMutex_);

        const auto it = shakeStates_.find(cameraId);

        if (it == shakeStates_.end()) return false;

        if (!it->second->isActive()) return false;

        ShakeConfig newConfig = it->second->getConfig();
        newConfig.intensity = std::max(0.0f, intensity);
        it->second->setConfig(newConfig);

        return true;
    }

    bool CameraManager::isCameraShaking(const CameraID cameraId) const {
        std::lock_guard lock(managerMutex_);

        const auto it = shakeStates_.find(cameraId);
        return (it != shakeStates_.end()) && it->second->isActive();
    }

    // ========================================================================
    // INPUT HANDLING
    // ========================================================================

    void CameraManager::setInputManager(InputManager* inputManager) {
        inputManager_ = inputManager;
        if (inputManager_) {
            inputHandler_ = std::make_unique<CameraInputHandler>(this, inputManager_);
        }
    }

    void CameraManager::processMouseLook(const float deltaX, const float deltaY, const float deltaTime) {
        BaseCamera* activeCamera = getActiveCamera();
        if (!activeCamera) {
            return;
        }

        const auto camera3D = dynamic_cast<Camera3D*>(activeCamera);

        if (!camera3D || (camera3D->getMode() != CameraMode::FREE_LOOK &&
            camera3D->getMode() != CameraMode::FOLLOW_TARGET)) {
            return;
        };

        // Apply sensitivity
        const float yawDelta = deltaX * config_.mouseSensitivity * deltaTime;
        const float pitchDelta = deltaY * config_.mouseSensitivity * deltaTime;

        // Update camera angles
        const float newYaw = camera3D->getYaw() + yawDelta;
        const float newPitch = camera3D->getPitch() - pitchDelta; // Invert Y for natural feel

        camera3D->setYaw(newYaw);
        camera3D->setPitch(newPitch);
    }

    void CameraManager::processZoom(const float zoomDelta) {
        BaseCamera* activeCamera = getActiveCamera();
        if (!activeCamera) {
            return;
        }

        const float scaledDelta = zoomDelta * config_.scrollSensitivity;

        if (const auto camera2D = dynamic_cast<Camera2D*>(activeCamera)) {
            // 2D camera zoom
            const float currentZoom = camera2D->getZoom();
            const float newZoom = currentZoom * (1.0f + scaledDelta * 0.1f); // 10% per scroll unit
            camera2D->setZoom(newZoom);
        }
        else if (const auto camera3D = dynamic_cast<Camera3D*>(activeCamera)) {
            // 3D camera zoom
            if (camera3D->getMode() == CameraMode::FOLLOW_TARGET ||
                camera3D->getMode() == CameraMode::ORBITAL) {
                // Adjust follow distance
                const float currentDistance = camera3D->getFollowDistance();
                const float newDistance = currentDistance * (1.0f + scaledDelta * 0.1f);
                camera3D->setFollowDistance(std::max(0.5f, newDistance));
            }
            else {
                // Adjust FOV
                const float currentFOV = camera3D->getFOV();
                const float newFOV = currentFOV - scaledDelta * 5.0f; // 5 degrees per scroll unit
                camera3D->setFOV(newFOV);
            }
        }
    }

    void CameraManager::processMovement(const float forward, const float right, const float up, const float deltaTime) {
        BaseCamera* activeCamera = getActiveCamera();
        if (!activeCamera) {
            return;
        }

        if (const auto camera3D = dynamic_cast<Camera3D*>(activeCamera)) {
            if (camera3D->getMode() == CameraMode::FREE_LOOK) {
                // FPS-style movement
                const Vec3 forwardDir = camera3D->getForward();
                const Vec3 rightDir = camera3D->getRight();
                const Vec3 upDir = camera3D->getUp();

                // Calculate movement vector
                Vec3 movement = forwardDir * forward + rightDir * right + upDir * up;
                movement *= 10.0f * deltaTime; // Movement speed

                // Apply movement
                const Vec3 newPosition = camera3D->getPosition() + movement;
                camera3D->setPosition(newPosition);
            }
        }
        else if (const auto camera2D = dynamic_cast<Camera2D*>(activeCamera)) {
            // 2D camera movement
            Vec2 movement(right, -forward); // Invert forward for screen coordinates
            movement *= 100.0f * deltaTime; // Movement speed

            const Vec2 newPosition = camera2D->getPosition2D() + movement;
            camera2D->setPosition(newPosition);
        }
    }

    // ========================================================================
    // VIEWPORT AND TRANSFORMATIONS
    // ========================================================================

    Vec2 CameraManager::worldToScreen(const Vec3& worldPos) const {
        const BaseCamera* activeCamera = getActiveCamera();
        if (!activeCamera) {
            return math::VEC2_ZERO;
        }

        if (const auto camera2D = dynamic_cast<const Camera2D*>(activeCamera)) {
            return camera2D->worldToScreen(Vec2(worldPos.x, worldPos.y), viewport_);
        }
        else if (const auto camera3D = dynamic_cast<const Camera3D*>(activeCamera)) {
            return camera3D->worldToScreen(worldPos, viewport_);
        }

        return math::VEC2_ZERO;
    }

    Vec3 CameraManager::screenToWorld(const Vec2& screenPos, const float depth) const {
        const BaseCamera* activeCamera = getActiveCamera();
        if (!activeCamera) {
            return math::VEC3_ZERO;
        }

        if (const auto camera2D = dynamic_cast<const Camera2D*>(activeCamera)) {
            const Vec2 worldPos2D = camera2D->screenToWorld(screenPos, viewport_);
            return Vec3(worldPos2D.x, worldPos2D.y, depth);
        }

        if (const auto camera3D = dynamic_cast<const Camera3D*>(activeCamera)) {
            return camera3D->screenToWorld(screenPos, viewport_, depth);
        }

        return math::VEC3_ZERO;
    }

    math::AABB CameraManager::getViewBounds() const {
        const BaseCamera* activeCamera = getActiveCamera();
        if (!activeCamera) {
            return math::AABB{}; // AABB vacío (inválido por defecto)
        }

        if (const auto camera2D = dynamic_cast<const Camera2D*>(activeCamera)) {
            // Asumiendo que Camera2D tiene un método similar que devuelve AABB
            const CameraBounds bounds = camera2D->getViewBounds(viewport_);
            return bounds.getAABB();
        }

        if (const auto camera3D = dynamic_cast<const Camera3D*>(activeCamera)) {
            const Vec3 position = camera3D->getPosition();
            constexpr float distance = 10.0f;

            return math::AABB(
                position - Vec3(distance, distance, distance),
                position + Vec3(distance, distance, distance)
            );
        }

        return math::AABB{}; // AABB vacío
    }

    // ========================================================================
    // CONFIGURATION AND STATISTICS
    // ========================================================================

    std::size_t CameraManager::getCameraCount() const {
        return registry_->getCount();
    }

    std::size_t CameraManager::getActiveTransitionCount() const {
        std::lock_guard lock(managerMutex_);

        std::size_t count = 0;
        for (const auto& transition : transitions_ | std::views::values) {
            if (transition->isActive()) {
                ++count;
            }
        }
        return count;
    }

    std::size_t CameraManager::getActiveShakeCount() const {
        std::lock_guard lock(managerMutex_);

        std::size_t count = 0;
        for (const auto& shake : shakeStates_ | std::views::values) {
            if (shake->isActive()) {
                ++count;
            }
        }
        return count;
    }

    std::size_t CameraManager::getMemoryUsage() const {
        std::lock_guard lock(managerMutex_);

        std::size_t usage = sizeof(CameraManager);
        usage += registry_->getCount() * sizeof(BaseCamera); // Approximate
        usage += transitions_.size() * sizeof(CameraTransition);
        usage += shakeStates_.size() * sizeof(CameraShake);

        return usage;
    }

    std::string CameraManager::getPerformanceStats() const {
        std::ostringstream oss;

        oss << "=== CameraManager Performance Stats ===" << std::endl;
        oss << "Cameras Updated: " << camerasUpdated_.load(std::memory_order_relaxed) << std::endl;
        oss << "Transitions Processed: " << transitionsProcessed_.load(std::memory_order_relaxed) << std::endl;
        oss << "Memory Usage: " << getMemoryUsage() << " bytes" << std::endl;

        const auto now = std::chrono::steady_clock::now();
        const auto timeSinceUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastUpdateTime_);
        oss << "Time Since Last Update: " << timeSinceUpdate.count() << "ms" << std::endl;

        return oss.str();
    }

    void CameraManager::clearAllCameras() {
        std::lock_guard lock(managerMutex_);

        registry_->clear();
        transitions_.clear();
        shakeStates_.clear();

        nextCameraId_ = 1;
        nextTransitionId_ = 1;
        activeCameraId_ = INVALID_CAMERA_ID;

        if (config_.enableCameraLogging) {
            std::cout << "[CameraManager] Cleared all cameras" << std::endl;
        }
    }

    // ========================================================================
    // PRIVATE IMPLEMENTATION METHODS
    // ========================================================================

    void CameraManager::updateTransitions(const float deltaTime) {
        for (const auto& transition : transitions_ | std::views::values) {
            if (transition->isActive()) {
                const bool completed = transition->update(deltaTime);

                if (!completed) {
                    // Apply interpolated values to camera
                    if (BaseCamera* camera = getCamera(transition->getCameraId())) {
                        // Position
                        camera->setPosition(transition->getInterpolatedPosition());

                        // Target for 3D cameras
                        if (const auto cam3D = dynamic_cast<Camera3D*>(camera)) {
                            cam3D->setTarget(transition->getInterpolatedTarget());

                            // FOV if transitioning
                            if (const float fov = transition->getInterpolatedFOV(); fov > 0.0f) {
                                cam3D->setFOV(fov);
                            }
                        }

                        // Zoom for 2D cameras
                        if (const auto cam2D = dynamic_cast<Camera2D*>(camera)) {
                            if (const float zoom = transition->getInterpolatedZoom(); zoom > 0.0f) {
                                cam2D->setZoom(zoom);
                            }

                            // Rotation if transitioning
                            const float rotation = transition->getInterpolatedRotation();
                            cam2D->setRotation(rotation);
                        }
                    }
                }

                if (completed) {
                    transitionsProcessed_.fetch_add(1, std::memory_order_relaxed);
                }
            }
        }
    }

    void CameraManager::updateShakeEffects(const float deltaTime) {
        for (auto& [cameraId, shake] : shakeStates_) {
            std::cout << "Processing shake for camera " << cameraId << std::endl;
            if (shake->isActive()) {
                const bool completed = shake->update(deltaTime);
                if (!completed) {
                    // ESTO ES LO QUE FALTA - aplicar el offset a la cámara
                    if (BaseCamera* camera = getCamera(cameraId)) {
                        Vec3 shakenPosition = shake->getShakenPosition();
                        camera->setPosition(shakenPosition);
                    }
                }
                else {
                    // Si completó, restaurar posición original
                    if (BaseCamera* camera = getCamera(cameraId)) {
                        camera->setPosition(shake->getBasePosition());
                    }
                }
            }
        }
    }

    void CameraManager::cleanupCompletedTransitions() {
        auto it = transitions_.begin();
        while (it != transitions_.end()) {
            if (!it->second->isActive() && it->second->isComplete()) {
                it = transitions_.erase(it);
            }
            else {
                ++it;
            }
        }
    }

    void CameraManager::cleanupInactiveShakes() {
        auto it = shakeStates_.begin();
        while (it != shakeStates_.end()) {
            if (!it->second->isActive()) {
                // Restore original position before removing shake
                if (BaseCamera* camera = getCamera(it->first)) {
                    camera->setPosition(it->second->getBasePosition());
                }
                it = shakeStates_.erase(it);
            }
            else {
                ++it;
            }
        }
    }

    void CameraManager::logCameraEvent(const std::string& message) const {
        if (config_.enableCameraLogging) {
            std::cout << "[CameraManager] " << message << std::endl;
        }
    }
} // namespace engine::camera
