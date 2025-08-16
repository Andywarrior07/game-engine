//
// Created by Andres Guerrero on 06-08-25.
//

#include "AnimationManager.h"
#include <iostream>         // For logging and debug output
#include <algorithm>        // For std::sort, std::find, etc.
#include <cassert>          // For debug assertions
#include <cmath>            // For mathematical operations
#include <sstream>          // For string stream operations

namespace engine::animation {

    // ========================================================================
    // SPRITE SHEET IMPLEMENTATION
    // ========================================================================

    /**
     * @brief Constructor for SpriteSheet
     * CHANGE: Initialize with texture resource handle for integration with ResourceManager
     */
    SpriteSheet::SpriteSheet(SpriteSheetID id, std::string name, engine::resources::ResourceHandle<engine::resources::TextureResource> texture)
        : id_(id)                                           // Store unique identifier
        , name_(std::move(name))                           // Move name to avoid copy
        , texture_(std::move(texture))                     // Move texture handle
    {
        // Reserve space for common number of frames to avoid frequent reallocations
        frames_.reserve(64);                               // Most sprite sheets have < 64 frames
    }

    /**
     * @brief Check if sprite sheet is ready for use
     * CHANGE: Proper validation of texture resource state
     */
    bool SpriteSheet::isReady() const noexcept {
        return texture_.isValid() &&                       // Texture handle is valid
               texture_.isReady() &&                       // Texture is loaded
               !frames_.empty();                           // Has at least one frame defined
    }

    /**
     * @brief Get frame rectangle by index with bounds checking
     * CHANGE: Safe frame access with validation
     */
    SpriteRect SpriteSheet::getFrame(int frameIndex) const {
        if (frameIndex >= 0 && frameIndex < static_cast<int>(frames_.size())) {
            return frames_[frameIndex];                     // Return valid frame
        }
        return SpriteRect{};                               // Return empty rect for invalid index
    }

    /**
     * @brief Add single frame to sprite sheet
     * CHANGE: Return frame index for easy reference
     */
    int SpriteSheet::addFrame(const SpriteRect& frame) {
        frames_.push_back(frame);                          // Add frame to collection
        return static_cast<int>(frames_.size()) - 1;       // Return index of added frame
    }

    /**
     * @brief Add multiple frames at once
     * CHANGE: Bulk operation for efficiency
     */
    int SpriteSheet::addFrames(const std::vector<SpriteRect>& frames) {
        int addedCount = static_cast<int>(frames.size());
        frames_.reserve(frames_.size() + addedCount);      // Reserve space to avoid reallocations

        for (const auto& frame : frames) {
            frames_.push_back(frame);                      // Add each frame
        }

        return addedCount;                                 // Return number of frames added
    }

    /**
     * @brief Generate frames from grid layout automatically
     * CHANGE: Utility function for common sprite sheet layouts
     */
    int SpriteSheet::generateGridFrames(int columns, int rows, int frameWidth, int frameHeight,
                                       int startX, int startY, int spacingX, int spacingY) {
        if (columns <= 0 || rows <= 0 || frameWidth <= 0 || frameHeight <= 0) {
            return 0;                                      // Invalid parameters
        }

        int frameCount = columns * rows;
        frames_.reserve(frames_.size() + frameCount);      // Reserve space for all frames

        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < columns; ++col) {
                // Calculate frame position accounting for spacing
                int x = startX + col * (frameWidth + spacingX);
                int y = startY + row * (frameHeight + spacingY);

                frames_.emplace_back(x, y, frameWidth, frameHeight);  // Add frame directly
            }
        }

        return frameCount;                                 // Return number of frames generated
    }

    /**
     * @brief Clear all frames from sprite sheet
     * CHANGE: Complete frame reset
     */
    void SpriteSheet::clearFrames() {
        frames_.clear();                                   // Remove all frames
        frames_.shrink_to_fit();                          // Release memory
    }

    /**
     * @brief Remove specific frame by index
     * CHANGE: Safe frame removal with validation
     */
    bool SpriteSheet::removeFrame(int frameIndex) {
        if (frameIndex >= 0 && frameIndex < static_cast<int>(frames_.size())) {
            frames_.erase(frames_.begin() + frameIndex);   // Remove frame at index
            return true;
        }
        return false;                                      // Invalid index
    }

    /**
     * @brief Get texture dimensions for validation
     * CHANGE: Safe access to texture properties
     */
    Vector2 SpriteSheet::getTextureDimensions() const {
        if (texture_.isReady()) {
            return Vector2(static_cast<float>(texture_->getWidth()),
                          static_cast<float>(texture_->getHeight()));
        }
        return Vector2{0.0f, 0.0f};                       // Return zero size if texture not ready
    }

    /**
     * @brief Validate all frames are within texture bounds
     * CHANGE: Comprehensive validation for debugging
     */
    bool SpriteSheet::validateFrames() const {
        if (!texture_.isReady()) {
            return false;                                  // Can't validate without texture
        }

        int textureWidth = texture_->getWidth();
        int textureHeight = texture_->getHeight();

        for (const auto& frame : frames_) {
            // Check if frame is completely within texture bounds
            if (frame.x < 0 || frame.y < 0 ||
                frame.x + frame.width > textureWidth ||
                frame.y + frame.height > textureHeight) {
                return false;                              // Frame extends outside texture
            }
        }

        return true;                                       // All frames are valid
    }

    /**
     * @brief Calculate memory usage of sprite sheet
     * CHANGE: Memory tracking for optimization
     */
    std::size_t SpriteSheet::getMemoryUsage() const {
        std::size_t frameMemory = frames_.size() * sizeof(SpriteRect);  // Frame definitions
        std::size_t nameMemory = name_.capacity();                      // String memory

        // Note: Texture memory is tracked by ResourceManager, not counted here
        return frameMemory + nameMemory;
    }

    // ========================================================================
    // ANIMATION IMPLEMENTATION
    // ========================================================================

    /**
     * @brief Constructor for Animation
     * CHANGE: Initialize with comprehensive configuration
     */
    Animation::Animation(AnimationID id, std::string name, SpriteSheetID spriteSheetId, const AnimationConfig& config)
        : id_(id)                                          // Store unique identifier
        , name_(std::move(name))                          // Move name to avoid copy
        , spriteSheetId_(spriteSheetId)                   // Associated sprite sheet
        , config_(config)                                 // Animation configuration
    {
        // Reserve space for typical animation frame count
        frameSequence_.reserve(32);                       // Most animations have < 32 frames
    }

    /**
     * @brief Get frame index from sequence position with bounds checking
     * CHANGE: Safe sequence access with validation
     */
    int Animation::getFrameIndex(int sequenceIndex) const {
        if (sequenceIndex >= 0 && sequenceIndex < static_cast<int>(frameSequence_.size())) {
            return frameSequence_[sequenceIndex];         // Return valid frame index
        }
        return -1;                                        // Invalid sequence index
    }

    /**
     * @brief Calculate total animation duration
     * CHANGE: Account for animation configuration and speed
     */
    float Animation::getTotalDuration() const noexcept {
        if (frameSequence_.empty() || config_.frameRate <= 0.0f) {
            return 0.0f;                                  // No duration if no frames or invalid frame rate
        }

        return static_cast<float>(frameSequence_.size()) / config_.frameRate;  // Duration in seconds
    }

    /**
     * @brief Set complete frame sequence
     * CHANGE: Replace entire sequence efficiently
     */
    void Animation::setFrameSequence(const std::vector<int>& frameIndices) {
        frameSequence_ = frameIndices;                    // Copy new sequence
    }

    /**
     * @brief Add frame to end of sequence
     * CHANGE: Simple frame addition
     */
    void Animation::addFrame(int frameIndex) {
        frameSequence_.push_back(frameIndex);             // Add to end of sequence
    }

    /**
     * @brief Insert frame at specific position in sequence
     * CHANGE: Flexible frame insertion with validation
     */
    bool Animation::insertFrame(int sequenceIndex, int frameIndex) {
        if (sequenceIndex >= 0 && sequenceIndex <= static_cast<int>(frameSequence_.size())) {
            frameSequence_.insert(frameSequence_.begin() + sequenceIndex, frameIndex);  // Insert at position
            return true;
        }
        return false;                                     // Invalid insertion position
    }

    /**
     * @brief Remove frame from sequence
     * CHANGE: Safe frame removal with validation
     */
    bool Animation::removeFrame(int sequenceIndex) {
        if (sequenceIndex >= 0 && sequenceIndex < static_cast<int>(frameSequence_.size())) {
            frameSequence_.erase(frameSequence_.begin() + sequenceIndex);  // Remove from sequence
            return true;
        }
        return false;                                     // Invalid sequence index
    }

    /**
     * @brief Generate frame sequence from range
     * CHANGE: Utility for creating sequential animations
     */
    void Animation::generateSequence(int startFrame, int endFrame, int step) {
        if (step <= 0) {
            return;                                       // Invalid step value
        }

        frameSequence_.clear();                           // Clear existing sequence

        if (startFrame <= endFrame) {
            // Forward sequence
            for (int frame = startFrame; frame <= endFrame; frame += step) {
                frameSequence_.push_back(frame);
            }
        } else {
            // Reverse sequence
            for (int frame = startFrame; frame >= endFrame; frame -= step) {
                frameSequence_.push_back(frame);
            }
        }
    }

    /**
     * @brief Clear all frames from sequence
     * CHANGE: Complete sequence reset
     */
    void Animation::clearSequence() {
        frameSequence_.clear();                           // Remove all frames
        frameSequence_.shrink_to_fit();                   // Release memory
    }

    /**
     * @brief Validate animation against sprite sheet
     * CHANGE: Comprehensive validation for debugging
     */
    bool Animation::validate(const SpriteSheet& spriteSheet) const {
        if (frameSequence_.empty()) {
            return false;                                 // No frames to validate
        }

        int maxFrame = spriteSheet.getFrameCount() - 1;

        for (int frameIndex : frameSequence_) {
            if (frameIndex < 0 || frameIndex > maxFrame) {
                return false;                             // Frame index out of bounds
            }
        }

        return true;                                      // All frame indices are valid
    }

    // ========================================================================
    // ANIMATION INSTANCE IMPLEMENTATION
    // ========================================================================

    /**
     * @brief Constructor for AnimationInstance
     * CHANGE: Initialize with complete state tracking
     */
    AnimationInstance::AnimationInstance(AnimationInstanceID id, AnimationID animationId, const SpriteTransform& transform)
        : id_(id)                                         // Store unique identifier
        , animationId_(animationId)                       // Associated animation
        , transform_(transform)                           // Initial transform data
        , state_(AnimationState::STOPPED)                // Start in stopped state
        , currentFrame_(0)                               // Start at first frame
        , currentTime_(0.0f)                             // Zero elapsed time
        , loopCount_(0)                                  // No loops completed
        , playingForward_(true)                          // Forward direction initially
        , speed_(1.0f)                                   // Normal playback speed
    {
        // Constructor body empty - all initialization in member list
    }

    /**
     * @brief Get animation progress (0.0 to 1.0)
     * CHANGE: Calculate progress based on current state
     */
    float AnimationInstance::getProgress() const noexcept {
        // Progress calculation depends on current animation state
        // This will be implemented when we have access to animation data
        return 0.0f;  // Placeholder - actual implementation needs animation reference
    }

    /**
     * @brief Start or resume animation playback
     * CHANGE: Proper state management with callback triggering
     */
    void AnimationInstance::play() {
        if (state_ == AnimationState::STOPPED) {
            // Starting from stopped - reset to beginning
            currentFrame_ = 0;
            currentTime_ = 0.0f;
            playingForward_ = true;
            state_ = AnimationState::PLAYING;
            triggerCallback(AnimationEventType::STARTED);
        } else if (state_ == AnimationState::PAUSED) {
            // Resuming from pause
            state_ = AnimationState::PLAYING;
            triggerCallback(AnimationEventType::RESUMED);
        }
        // If already playing, do nothing
    }

    /**
     * @brief Pause animation playback
     * CHANGE: State transition with callback
     */
    void AnimationInstance::pause() {
        if (state_ == AnimationState::PLAYING) {
            state_ = AnimationState::PAUSED;
            triggerCallback(AnimationEventType::PAUSED);
        }
    }

    /**
     * @brief Stop animation and reset to beginning
     * CHANGE: Complete state reset
     */
    void AnimationInstance::stop() {
        state_ = AnimationState::STOPPED;
        currentFrame_ = 0;
        currentTime_ = 0.0f;
        loopCount_ = 0;
        playingForward_ = true;
    }

    /**
     * @brief Restart animation from beginning
     * CHANGE: Reset and immediately start playing
     */
    void AnimationInstance::restart() {
        stop();                                           // Reset state
        play();                                           // Start playing
    }

    /**
     * @brief Set current frame manually
     * CHANGE: Direct frame control with validation
     */
    void AnimationInstance::setCurrentFrame(int frameIndex) {
        if (frameIndex >= 0) {  // Basic validation - full validation needs animation reference
            int oldFrame = currentFrame_;
            currentFrame_ = frameIndex;
            triggerFrameCallback(oldFrame, frameIndex);
        }
    }

    /**
     * @brief Set animation progress (0.0 to 1.0)
     * CHANGE: Direct progress control
     */
    void AnimationInstance::setProgress(float progress) {
        // Clamp progress to valid range
        progress = std::clamp(progress, 0.0f, 1.0f);

        // Progress implementation needs animation reference for frame calculation
        // This is a placeholder that will be completed in the update method
    }

    /**
     * @brief Update animation state and return if frame changed
     * CHANGE: Comprehensive animation update with all playback modes
     */
    bool AnimationInstance::update(float deltaTime, const Animation& animation) {
        if (state_ != AnimationState::PLAYING) {
            return false;                                 // Not playing, no update needed
        }

        const auto& config = animation.getConfig();
        int frameCount = animation.getFrameCount();

        if (frameCount <= 0) {
            return false;                                 // No frames to animate
        }

        // Store old frame for change detection
        int oldFrame = currentFrame_;

        // Calculate time step with speed multiplier
        float effectiveSpeed = config.speed * speed_;
        float frameTime = deltaTime * effectiveSpeed;
        currentTime_ += frameTime;

        // Calculate frame duration
        float frameDuration = animation.getFrameDuration();

        // Update frame based on animation mode
        bool frameChanged = false;

        switch (config.mode) {
            case AnimationMode::ONCE:
                {
                    // Play once and stop at last frame
                    int targetFrame = static_cast<int>(currentTime_ / frameDuration);
                    if (targetFrame >= frameCount) {
                        currentFrame_ = frameCount - 1;   // Clamp to last frame
                        state_ = AnimationState::FINISHED;
                        triggerCallback(AnimationEventType::FINISHED);
                    } else {
                        currentFrame_ = targetFrame;
                    }
                }
                break;

            case AnimationMode::LOOP:
                {
                    // Loop continuously
                    float totalDuration = animation.getTotalDuration();
                    if (currentTime_ >= totalDuration) {
                        currentTime_ -= totalDuration;    // Wrap time
                        loopCount_++;
                        triggerCallback(AnimationEventType::LOOPED);
                    }
                    currentFrame_ = static_cast<int>(currentTime_ / frameDuration) % frameCount;
                }
                break;

            case AnimationMode::PING_PONG:
                {
                    // Play forward then backward
                    int totalFrames = frameCount * 2 - 2;  // Account for ping-pong
                    if (totalFrames <= 0) {
                        currentFrame_ = 0;
                        break;
                    }

                    float totalDuration = totalFrames * frameDuration;
                    if (currentTime_ >= totalDuration) {
                        currentTime_ -= totalDuration;    // Wrap time
                        loopCount_++;
                        triggerCallback(AnimationEventType::LOOPED);
                    }

                    int pingPongFrame = static_cast<int>(currentTime_ / frameDuration) % totalFrames;
                    if (pingPongFrame < frameCount) {
                        currentFrame_ = pingPongFrame;    // Forward direction
                        playingForward_ = true;
                    } else {
                        currentFrame_ = totalFrames - pingPongFrame;  // Backward direction
                        playingForward_ = false;
                    }
                }
                break;

            case AnimationMode::REVERSE:
                {
                    // Play in reverse once
                    int targetFrame = frameCount - 1 - static_cast<int>(currentTime_ / frameDuration);
                    if (targetFrame < 0) {
                        currentFrame_ = 0;                // Clamp to first frame
                        state_ = AnimationState::FINISHED;
                        triggerCallback(AnimationEventType::FINISHED);
                    } else {
                        currentFrame_ = targetFrame;
                    }
                }
                break;

            case AnimationMode::REVERSE_LOOP:
                {
                    // Loop in reverse continuously
                    float totalDuration = animation.getTotalDuration();
                    if (currentTime_ >= totalDuration) {
                        currentTime_ -= totalDuration;    // Wrap time
                        loopCount_++;
                        triggerCallback(AnimationEventType::LOOPED);
                    }
                    currentFrame_ = frameCount - 1 - (static_cast<int>(currentTime_ / frameDuration) % frameCount);
                }
                break;
        }

        // Check if frame actually changed
        frameChanged = (currentFrame_ != oldFrame);
        if (frameChanged) {
            triggerFrameCallback(oldFrame, currentFrame_);
            triggerCallback(AnimationEventType::FRAME_CHANGED);
        }

        return frameChanged;
    }

    /**
     * @brief Trigger animation event callback
     * CHANGE: Safe callback execution with null checking
     */
    void AnimationInstance::triggerCallback(AnimationEventType eventType) {
        if (callback_) {
            callback_(id_, eventType, currentFrame_);
        }
    }

    /**
     * @brief Trigger frame change callback
     * CHANGE: Dedicated frame change notification
     */
    void AnimationInstance::triggerFrameCallback(int oldFrame, int newFrame) {
        if (frameCallback_) {
            frameCallback_(id_, oldFrame, newFrame);
        }
    }

    // ========================================================================
    // ANIMATION MANAGER IMPLEMENTATION
    // ========================================================================

    /**
     * @brief Constructor for AnimationManager
     * CHANGE: Initialize with configuration and proper member setup
     */
    AnimationManager::AnimationManager(const AnimationManagerConfig& config)
        : config_(config)                                 // Store configuration
        , initialized_(false)                            // Start uninitialized
        , nextSpriteSheetId_(1)                          // Start IDs at 1 (0 is reserved)
        , nextAnimationId_(1)
        , nextInstanceId_(1)
        , lastUpdateTime_(std::chrono::steady_clock::now())  // Initialize timing
    {
        // Reserve space for common numbers of resources
        spriteSheets_.reserve(64);                        // Typical game has < 64 sprite sheets
        animations_.reserve(256);                         // Typical game has < 256 animations
        instances_.reserve(config_.maxInstances);         // Reserve for max instances

        // Reserve space for render batches
        renderBatches_.reserve(32);                       // Most games need < 32 batches
        visibleInstances_.reserve(config_.maxInstances);  // Reserve for visibility culling
    }

    /**
     * @brief Destructor ensures clean shutdown
     * CHANGE: Explicit shutdown call for proper cleanup order
     */
    AnimationManager::~AnimationManager() {
        shutdown();
    }

    /**
     * @brief Initialize animation manager
     * CHANGE: Proper initialization with validation
     */
    bool AnimationManager::initialize() {
        if (initialized_) {
            return true;                                  // Already initialized
        }

        std::cout << "Initializing AnimationManager..." << std::endl;

        // Initialize timing
        lastUpdateTime_ = std::chrono::steady_clock::now();

        initialized_ = true;
        std::cout << "AnimationManager initialized successfully" << std::endl;

        // Log configuration for debugging
        if (config_.enableAnimationLogging) {
            std::cout << "Animation logging enabled" << std::endl;
            std::cout << "Max instances: " << config_.maxInstances << std::endl;
            std::cout << "Batching enabled: " << (config_.enableBatching ? "Yes" : "No") << std::endl;
            std::cout << "Culling enabled: " << (config_.enableCulling ? "Yes" : "No") << std::endl;
        }

        return true;
    }

    /**
     * @brief Shutdown and cleanup all resources
     * CHANGE: Comprehensive cleanup with thread safety
     */
    void AnimationManager::shutdown() {
        if (!initialized_) {
            return;                                       // Already shutdown
        }

        std::lock_guard<std::mutex> lock(animationMutex_);  // Thread-safe cleanup

        std::cout << "Shutting down AnimationManager..." << std::endl;

        // Clear all instances first
        instances_.clear();

        // Clear animations
        animations_.clear();

        // Clear sprite sheets
        spriteSheets_.clear();

        // Clear render batches
        renderBatches_.clear();
        visibleInstances_.clear();

        // Reset ID counters
        nextSpriteSheetId_ = 1;
        nextAnimationId_ = 1;
        nextInstanceId_ = 1;

        initialized_ = false;
        std::cout << "AnimationManager shutdown complete" << std::endl;
    }

    /**
     * @brief Update all animation instances
     * CHANGE: Efficient batch update with frame tracking
     */
    void AnimationManager::update(float deltaTime) {
        if (!initialized_) {
            return;                                       // Not initialized
        }

        // Update all active instances
        for (auto& [instanceId, instance] : instances_) {
            if (instance->isPlaying()) {
                // Get associated animation
                auto animIt = animations_.find(instance->getAnimationId());
                if (animIt != animations_.end()) {
                    // Update instance with animation data
                    bool frameChanged = instance->update(deltaTime, *animIt->second);
                    if (frameChanged) {
                        instancesUpdated_.fetch_add(1, std::memory_order_relaxed);
                    }
                }
            }
        }

        // Update timing
        lastUpdateTime_ = std::chrono::steady_clock::now();
    }

    /**
     * @brief Render all visible animation instances
     * CHANGE: Optimized rendering with batching and culling
     */
    void AnimationManager::render(SDL_Renderer* renderer, const Vector2& cameraPosition, const Vector2& viewportSize) {
        if (!initialized_ || !renderer) {
            return;                                       // Not initialized or invalid renderer
        }

        // Update visibility culling if enabled
        if (config_.enableCulling) {
            updateCulling(cameraPosition, viewportSize);
        } else {
            // If culling disabled, all instances are visible
            visibleInstances_.clear();
            visibleInstances_.reserve(instances_.size());
            for (const auto& [instanceId, instance] : instances_) {
                visibleInstances_.push_back(instanceId);
            }
        }

        // Sort instances by rendering layer for proper depth ordering
        sortInstancesByLayer();

        if (config_.enableBatching) {
            // Use batched rendering for better performance
            prepareRenderBatches();

            for (const auto& batch : renderBatches_) {
                if (batch->count > 0) {
                    renderBatch(renderer, *batch);
                    drawCalls_.fetch_add(1, std::memory_order_relaxed);
                }
            }
        } else {
            // Render instances individually (simpler but less efficient)
            for (AnimationInstanceID instanceId : visibleInstances_) {
                auto instanceIt = instances_.find(instanceId);
                if (instanceIt == instances_.end()) continue;

                const auto& instance = *instanceIt->second;

                // Get animation and sprite sheet
                auto animIt = animations_.find(instance.getAnimationId());
                if (animIt == animations_.end()) continue;

                const auto& animation = *animIt->second;
                auto spriteSheetIt = spriteSheets_.find(animation.getSpriteSheetId());
                if (spriteSheetIt == spriteSheets_.end()) continue;

                const auto& spriteSheet = *spriteSheetIt->second;
                if (!spriteSheet.isReady()) continue;

                // Get current frame rectangle
                int frameIndex = animation.getFrameIndex(instance.getCurrentFrame());
                if (frameIndex < 0) continue;

                SpriteRect frameRect = spriteSheet.getFrame(frameIndex);
                if (frameRect.width <= 0 || frameRect.height <= 0) continue;

                // Calculate source and destination rectangles
                SDL_Rect srcRect = frameRect.toSDLRect();
                SDL_Rect dstRect = calculateDestRect(instance, spriteSheet, frameRect);

                // Set up rendering state
                const auto& transform = instance.getTransform();
                SDL_SetTextureAlphaMod(spriteSheet.getTexture()->getSDLTexture(), transform.alpha);
                SDL_SetTextureColorMod(spriteSheet.getTexture()->getSDLTexture(),
                                     transform.tint.r, transform.tint.g, transform.tint.b);

                // Handle sprite flipping
                SDL_RendererFlip flip = SDL_FLIP_NONE;
                if (static_cast<int>(transform.flip) & static_cast<int>(SpriteFlip::HORIZONTAL)) {
                    flip = static_cast<SDL_RendererFlip>(flip | SDL_FLIP_HORIZONTAL);
                }
                if (static_cast<int>(transform.flip) & static_cast<int>(SpriteFlip::VERTICAL)) {
                    flip = static_cast<SDL_RendererFlip>(flip | SDL_FLIP_VERTICAL);
                }

                // Render the sprite
                SDL_RenderCopyEx(renderer, spriteSheet.getTexture()->getSDLTexture(),
                               &srcRect, &dstRect, static_cast<double>(transform.rotation),
                               nullptr, flip);

                drawCalls_.fetch_add(1, std::memory_order_relaxed);
                framesRendered_.fetch_add(1, std::memory_order_relaxed);
            }
        }
    }

    /**
     * @brief Create sprite sheet from texture resource
     * CHANGE: Integration with ResourceManager texture system
     */
    SpriteSheetID AnimationManager::createSpriteSheet(const std::string& name, engine::resources::ResourceHandle<engine::resources::TextureResource> texture) {
        if (!initialized_) {
            std::cerr << "AnimationManager not initialized" << std::endl;
            return INVALID_SPRITESHEET_ID;
        }

        std::lock_guard<std::mutex> lock(animationMutex_);  // Thread-safe operation

        SpriteSheetID spriteSheetId = nextSpriteSheetId_++;

        auto spriteSheet = std::make_unique<SpriteSheet>(spriteSheetId, name, std::move(texture));
        spriteSheets_[spriteSheetId] = std::move(spriteSheet);

        if (config_.enableAnimationLogging) {
            std::cout << "Created sprite sheet: " << name << " (ID: " << spriteSheetId << ")" << std::endl;
        }

        return spriteSheetId;
    }

    /**
     * @brief Remove sprite sheet and all associated animations
     * CHANGE: Cascade deletion of dependent resources
     */
    bool AnimationManager::removeSpriteSheet(SpriteSheetID spriteSheetId) {
        if (!isValidSpriteSheetId(spriteSheetId)) {
            return false;
        }

        std::lock_guard<std::mutex> lock(animationMutex_);

        // Remove all animations that use this sprite sheet
        std::vector<AnimationID> animationsToRemove;
        for (const auto& [animId, animation] : animations_) {
            if (animation->getSpriteSheetId() == spriteSheetId) {
                animationsToRemove.push_back(animId);
            }
        }

        for (AnimationID animId : animationsToRemove) {
            removeAnimation(animId);  // This will also remove instances
        }

        // Remove the sprite sheet
        auto it = spriteSheets_.find(spriteSheetId);
        if (it != spriteSheets_.end()) {
            if (config_.enableAnimationLogging) {
                std::cout << "Removed sprite sheet: " << it->second->getName() << std::endl;
            }
            spriteSheets_.erase(it);
            return true;
        }

        return false;
    }

    /**
     * @brief Get sprite sheet by ID
     * CHANGE: Safe pointer access with validation
     */
    SpriteSheet* AnimationManager::getSpriteSheet(SpriteSheetID spriteSheetId) {
        auto it = spriteSheets_.find(spriteSheetId);
        return (it != spriteSheets_.end()) ? it->second.get() : nullptr;
    }

    /**
     * @brief Get sprite sheet by ID (const version)
     * CHANGE: Const-correct access
     */
    const SpriteSheet* AnimationManager::getSpriteSheet(SpriteSheetID spriteSheetId) const {
        auto it = spriteSheets_.find(spriteSheetId);
        return (it != spriteSheets_.end()) ? it->second.get() : nullptr;
    }

    /**
     * @brief Create animation from sprite sheet
     * CHANGE: Full animation creation with validation
     */
    AnimationID AnimationManager::createAnimation(const std::string& name, SpriteSheetID spriteSheetId, const AnimationConfig& config) {
        if (!initialized_) {
            std::cerr << "AnimationManager not initialized" << std::endl;
            return INVALID_ANIMATION_ID;
        }

        if (!isValidSpriteSheetId(spriteSheetId)) {
            std::cerr << "Invalid sprite sheet ID: " << spriteSheetId << std::endl;
            return INVALID_ANIMATION_ID;
        }

        std::lock_guard<std::mutex> lock(animationMutex_);

        AnimationID animationId = nextAnimationId_++;

        auto animation = std::make_unique<Animation>(animationId, name, spriteSheetId, config);
        animations_[animationId] = std::move(animation);

        if (config_.enableAnimationLogging) {
            std::cout << "Created animation: " << name << " (ID: " << animationId << ")" << std::endl;
        }

        return animationId;
    }

    /**
     * @brief Remove animation and all its instances
     * CHANGE: Cascade deletion of animation instances
     */
    bool AnimationManager::removeAnimation(AnimationID animationId) {
        if (!isValidAnimationId(animationId)) {
            return false;
        }

        std::lock_guard<std::mutex> lock(animationMutex_);

        // Remove all instances of this animation
        removeAllInstancesOfAnimation(animationId);

        // Remove the animation
        auto it = animations_.find(animationId);
        if (it != animations_.end()) {
            if (config_.enableAnimationLogging) {
                std::cout << "Removed animation: " << it->second->getName() << std::endl;
            }
            animations_.erase(it);
            return true;
        }

        return false;
    }

    /**
     * @brief Get animation by ID
     * CHANGE: Safe pointer access with validation
     */
    Animation* AnimationManager::getAnimation(AnimationID animationId) {
        auto it = animations_.find(animationId);
        return (it != animations_.end()) ? it->second.get() : nullptr;
    }

    /**
     * @brief Get animation by ID (const version)
     * CHANGE: Const-correct access
     */
    const Animation* AnimationManager::getAnimation(AnimationID animationId) const {
        auto it = animations_.find(animationId);
        return (it != animations_.end()) ? it->second.get() : nullptr;
    }

    /**
     * @brief Create animation instance
     * CHANGE: Instance creation with proper initialization
     */
    AnimationInstanceID AnimationManager::createInstance(AnimationID animationId, const SpriteTransform& transform) {
        if (!initialized_) {
            std::cerr << "AnimationManager not initialized" << std::endl;
            return INVALID_INSTANCE_ID;
        }

        if (!isValidAnimationId(animationId)) {
            std::cerr << "Invalid animation ID: " << animationId << std::endl;
            return INVALID_INSTANCE_ID;
        }

        if (instances_.size() >= static_cast<size_t>(config_.maxInstances)) {
            std::cerr << "Maximum number of animation instances reached" << std::endl;
            return INVALID_INSTANCE_ID;
        }

        std::lock_guard<std::mutex> lock(animationMutex_);

        AnimationInstanceID instanceId = nextInstanceId_++;

        auto instance = std::make_unique<AnimationInstance>(instanceId, animationId, transform);

        // Auto-play if configured
        const Animation* animation = getAnimation(animationId);
        if (animation && animation->getConfig().autoPlay) {
            instance->play();
        }

        instances_[instanceId] = std::move(instance);

        if (config_.enableAnimationLogging) {
            std::cout << "Created animation instance (ID: " << instanceId << ") for animation " << animationId << std::endl;
        }

        return instanceId;
    }

    /**
     * @brief Remove animation instance
     * CHANGE: Safe instance removal with validation
     */
    bool AnimationManager::removeInstance(AnimationInstanceID instanceId) {
        if (!isValidInstanceId(instanceId)) {
            return false;
        }

        std::lock_guard<std::mutex> lock(animationMutex_);

        auto it = instances_.find(instanceId);
        if (it != instances_.end()) {
            if (config_.enableAnimationLogging) {
                std::cout << "Removed animation instance (ID: " << instanceId << ")" << std::endl;
            }
            instances_.erase(it);
            return true;
        }

        return false;
    }

    /**
     * @brief Get animation instance by ID
     * CHANGE: Safe pointer access with validation
     */
    AnimationInstance* AnimationManager::getInstance(AnimationInstanceID instanceId) {
        auto it = instances_.find(instanceId);
        return (it != instances_.end()) ? it->second.get() : nullptr;
    }

    /**
     * @brief Get animation instance by ID (const version)
     * CHANGE: Const-correct access
     */
    const AnimationInstance* AnimationManager::getInstance(AnimationInstanceID instanceId) const {
        auto it = instances_.find(instanceId);
        return (it != instances_.end()) ? it->second.get() : nullptr;
    }

    /**
     * @brief Get all instances of a specific animation
     * CHANGE: Utility for batch operations on animation instances
     */
    std::vector<AnimationInstanceID> AnimationManager::getInstancesOfAnimation(AnimationID animationId) const {
        std::vector<AnimationInstanceID> result;

        for (const auto& [instanceId, instance] : instances_) {
            if (instance->getAnimationId() == animationId) {
                result.push_back(instanceId);
            }
        }

        return result;
    }

    /**
     * @brief Remove all instances of a specific animation
     * CHANGE: Bulk removal operation
     */
    int AnimationManager::removeAllInstancesOfAnimation(AnimationID animationId) {
        std::vector<AnimationInstanceID> instancesToRemove = getInstancesOfAnimation(animationId);

        for (AnimationInstanceID instanceId : instancesToRemove) {
            instances_.erase(instanceId);
        }

        return static_cast<int>(instancesToRemove.size());
    }

    /**
     * @brief Play all instances of an animation
     * CHANGE: Batch playback control
     */
    void AnimationManager::playAllInstances(AnimationID animationId) {
        for (auto& [instanceId, instance] : instances_) {
            if (instance->getAnimationId() == animationId) {
                instance->play();
            }
        }
    }

    /**
     * @brief Pause all instances of an animation
     * CHANGE: Batch pause control
     */
    void AnimationManager::pauseAllInstances(AnimationID animationId) {
        for (auto& [instanceId, instance] : instances_) {
            if (instance->getAnimationId() == animationId) {
                instance->pause();
            }
        }
    }

    /**
     * @brief Stop all instances of an animation
     * CHANGE: Batch stop control
     */
    void AnimationManager::stopAllInstances(AnimationID animationId) {
        for (auto& [instanceId, instance] : instances_) {
            if (instance->getAnimationId() == animationId) {
                instance->stop();
            }
        }
    }

    /**
     * @brief Clear all animation instances
     * CHANGE: Complete instance cleanup
     */
    void AnimationManager::clearAllInstances() {
        std::lock_guard<std::mutex> lock(animationMutex_);
        instances_.clear();

        if (config_.enableAnimationLogging) {
            std::cout << "Cleared all animation instances" << std::endl;
        }
    }

    /**
     * @brief Get number of active animation instances
     * CHANGE: Performance monitoring
     */
    std::size_t AnimationManager::getInstanceCount() const {
        return instances_.size();
    }

    /**
     * @brief Get number of playing animation instances
     * CHANGE: Performance monitoring for active animations
     */
    std::size_t AnimationManager::getPlayingInstanceCount() const {
        std::size_t count = 0;
        for (const auto& [instanceId, instance] : instances_) {
            if (instance->isPlaying()) {
                ++count;
            }
        }
        return count;
    }

    /**
     * @brief Get memory usage statistics
     * CHANGE: Memory tracking for optimization
     */
    std::size_t AnimationManager::getMemoryUsage() const {
        std::size_t totalMemory = 0;

        // Calculate sprite sheet memory
        for (const auto& [id, spriteSheet] : spriteSheets_) {
            totalMemory += spriteSheet->getMemoryUsage();
        }

        // Calculate animation memory
        totalMemory += animations_.size() * sizeof(Animation);

        // Calculate instance memory
        totalMemory += instances_.size() * sizeof(AnimationInstance);

        // Calculate render batch memory
        for (const auto& batch : renderBatches_) {
            totalMemory += batch->capacity * (sizeof(SDL_Rect) * 2);  // src + dst rects
        }

        return totalMemory;
    }

    /**
     * @brief Get debug information about animation system
     * CHANGE: Comprehensive debug output
     */
    std::string AnimationManager::getDebugInfo() const {
        std::ostringstream oss;

        oss << "=== AnimationManager Debug Info ===\n";
        oss << "Initialized: " << (initialized_ ? "Yes" : "No") << "\n";
        oss << "Sprite Sheets: " << spriteSheets_.size() << "\n";
        oss << "Animations: " << animations_.size() << "\n";
        oss << "Instances: " << instances_.size() << "\n";
        oss << "Playing Instances: " << getPlayingInstanceCount() << "\n";
        oss << "Memory Usage: " << (getMemoryUsage() / 1024) << " KB\n\n";

        // List sprite sheets
        oss << "=== Sprite Sheets ===\n";
        for (const auto& [id, spriteSheet] : spriteSheets_) {
            oss << id << ": " << spriteSheet->getName()
                << " (Frames: " << spriteSheet->getFrameCount()
                << ", Ready: " << (spriteSheet->isReady() ? "Yes" : "No") << ")\n";
        }

        // List animations
        oss << "\n=== Animations ===\n";
        for (const auto& [id, animation] : animations_) {
            oss << id << ": " << animation->getName()
                << " (Frames: " << animation->getFrameCount()
                << ", Duration: " << animation->getTotalDuration() << "s)\n";
        }

        return oss.str();
    }

    /**
     * @brief Get performance statistics
     * CHANGE: Performance monitoring for optimization
     */
    std::string AnimationManager::getPerformanceStats() const {
        std::ostringstream oss;

        oss << "=== AnimationManager Performance Stats ===\n";
        oss << "Frames Rendered: " << framesRendered_.load(std::memory_order_relaxed) << "\n";
        oss << "Draw Calls: " << drawCalls_.load(std::memory_order_relaxed) << "\n";
        oss << "Instances Updated: " << instancesUpdated_.load(std::memory_order_relaxed) << "\n";
        oss << "Render Batches: " << renderBatches_.size() << "\n";
        oss << "Visible Instances: " << visibleInstances_.size() << "\n";

        return oss.str();
    }

    // ========================================================================
    // PRIVATE HELPER METHODS
    // ========================================================================

    /**
     * @brief Validate sprite sheet ID
     * CHANGE: ID validation for safety
     */
    bool AnimationManager::isValidSpriteSheetId(SpriteSheetID spriteSheetId) const {
        return spriteSheetId != INVALID_SPRITESHEET_ID &&
               spriteSheets_.find(spriteSheetId) != spriteSheets_.end();
    }

    /**
     * @brief Validate animation ID
     * CHANGE: ID validation for safety
     */
    bool AnimationManager::isValidAnimationId(AnimationID animationId) const {
        return animationId != INVALID_ANIMATION_ID &&
               animations_.find(animationId) != animations_.end();
    }

    /**
     * @brief Validate animation instance ID
     * CHANGE: ID validation for safety
     */
    bool AnimationManager::isValidInstanceId(AnimationInstanceID instanceId) const {
        return instanceId != INVALID_INSTANCE_ID &&
               instances_.find(instanceId) != instances_.end();
    }

    /**
     * @brief Update frustum culling for instances
     * CHANGE: Efficient visibility culling for large numbers of sprites
     */
    void AnimationManager::updateCulling(const Vector2& cameraPosition, const Vector2& viewportSize) {
        visibleInstances_.clear();

        for (const auto& [instanceId, instance] : instances_) {
            if (isInstanceVisible(*instance, cameraPosition, viewportSize)) {
                visibleInstances_.push_back(instanceId);
            }
        }
    }

    /**
     * @brief Prepare render batches for efficient rendering
     * CHANGE: Batch sprites by texture for fewer draw calls
     */
    void AnimationManager::prepareRenderBatches() {
        std::cout << "Preparing render batches for " << visibleInstances_.size() << " visible instances" << std::endl;

        // Clear existing batches
        for (const auto& batch : renderBatches_) {
            batch->count = 0;
        }

        // Group instances by texture
        std::unordered_map<SpriteSheetID, std::vector<AnimationInstanceID>> spriteSheetGroups;

        for (AnimationInstanceID instanceId : visibleInstances_) {
            auto instanceIt = instances_.find(instanceId);
            if (instanceIt == instances_.end()) continue;

            const auto& instance = *instanceIt->second;
            auto animIt = animations_.find(instance.getAnimationId());
            if (animIt == animations_.end()) continue;

            const auto& animation = *animIt->second;
            SpriteSheetID spriteSheetId = animation.getSpriteSheetId();

            auto spriteSheetIt = spriteSheets_.find(spriteSheetId);
            if (spriteSheetIt == spriteSheets_.end()) continue;

            const auto& spriteSheet = *spriteSheetIt->second;
            if (!spriteSheet.isReady()) continue;

            spriteSheetGroups[spriteSheetId].push_back(instanceId);
        }

        // Create or update render batches
        size_t batchIndex = 0;
        for (const auto& [spriteSheetId, instanceIds] : spriteSheetGroups) {
            if (instanceIds.empty()) continue;

            auto spriteSheetIt = spriteSheets_.find(spriteSheetId);
            if (spriteSheetIt == spriteSheets_.end()) continue;
            const auto& spriteSheet = *spriteSheetIt->second;

            // Get or create batch
            if (batchIndex >= renderBatches_.size()) {
                renderBatches_.push_back(std::make_unique<RenderBatch>());
            }

            auto& batch = *renderBatches_[batchIndex];

            batch.texture = spriteSheet.getTexture();

            // Ensure batch has enough capacity
            int requiredCapacity = static_cast<int>(instanceIds.size());
            if (batch.capacity < requiredCapacity) {
                delete[] batch.srcRects;
                delete[] batch.dstRects;
                batch.srcRects = new SDL_Rect[requiredCapacity];
                batch.dstRects = new SDL_Rect[requiredCapacity];
                batch.capacity = requiredCapacity;
            }

            // Fill batch data
            batch.instances = instanceIds;
            batch.count = requiredCapacity;

            // Prepare rectangles for batch rendering
            for (int i = 0; i < requiredCapacity; ++i) {
                AnimationInstanceID instanceId = instanceIds[i];
                auto instanceIt = instances_.find(instanceId);
                if (instanceIt == instances_.end()) continue;

                const auto& instance = *instanceIt->second;
                auto animIt = animations_.find(instance.getAnimationId());
                if (animIt == animations_.end()) continue;

                const auto& animation = *animIt->second;
                auto spriteSheetIt = spriteSheets_.find(animation.getSpriteSheetId());
                if (spriteSheetIt == spriteSheets_.end()) continue;

                const auto& spriteSheet = *spriteSheetIt->second;

                // Get current frame
                int frameIndex = animation.getFrameIndex(instance.getCurrentFrame());
                if (frameIndex < 0) continue;

                SpriteRect frameRect = spriteSheet.getFrame(frameIndex);

                // Set source and destination rectangles
                batch.srcRects[i] = frameRect.toSDLRect();
                batch.dstRects[i] = calculateDestRect(instance, spriteSheet, frameRect);
            }

            ++batchIndex;
        }
    }

    /**
     * @brief Render a single batch
     * CHANGE: Efficient batch rendering using SDL_RenderCopy array functions
     */
    void AnimationManager::renderBatch(SDL_Renderer* renderer, const RenderBatch& batch) {
        std::cout << "Rendering batch with " << batch.count << " sprites" << std::endl;

        if (batch.count <= 0 || !batch.texture.isReady()) {
            std::cout << "ERROR: Batch invalid - count: " << batch.count
                      << ", texture ready: " << batch.texture.isReady() << std::endl;
            return;
        }

        // Set up texture for rendering
        SDL_Texture* sdlTexture = batch.texture->getSDLTexture();

        // Render all sprites in batch with single call
        // Note: SDL doesn't have RenderCopyMultiple, so we'll render individually but grouped
        for (int i = 0; i < batch.count; ++i) {
            SDL_RenderCopy(renderer, sdlTexture, &batch.srcRects[i], &batch.dstRects[i]);
        }

        framesRendered_.fetch_add(batch.count, std::memory_order_relaxed);
    }

    /**
     * @brief Calculate destination rectangle for sprite rendering
     * CHANGE: Handle transform, scale, and positioning
     */
    SDL_Rect AnimationManager::calculateDestRect(const AnimationInstance& instance, const SpriteSheet& spriteSheet, const SpriteRect& frameRect) const {
        const auto& transform = instance.getTransform();

        // Calculate scaled dimensions
        int scaledWidth = static_cast<int>(frameRect.width * transform.scale.x);
        int scaledHeight = static_cast<int>(frameRect.height * transform.scale.y);

        // Calculate position accounting for origin
        int x = static_cast<int>(transform.position.x - transform.origin.x * scaledWidth);
        int y = static_cast<int>(transform.position.y - transform.origin.y * scaledHeight);

        return {x, y, scaledWidth, scaledHeight};
    }

    /**
     * @brief Check if instance is visible within camera view
     * CHANGE: Frustum culling with margin for smooth scrolling
     */
    bool AnimationManager::isInstanceVisible(const AnimationInstance& instance, const Vector2& cameraPosition, const Vector2& viewportSize) const {
        const auto& transform = instance.getTransform();

        // Calculate sprite bounds (simplified bounding box)
        float spriteWidth = 64.0f * transform.scale.x;   // Assume 64px default sprite size
        float spriteHeight = 64.0f * transform.scale.y;

        // Calculate camera bounds with margin
        float leftBound = cameraPosition.x - config_.cullMargin.x;
        float rightBound = cameraPosition.x + viewportSize.x + config_.cullMargin.x;
        float topBound = cameraPosition.y - config_.cullMargin.y;
        float bottomBound = cameraPosition.y + viewportSize.y + config_.cullMargin.y;

        // Check if sprite intersects with camera view
        return (transform.position.x + spriteWidth >= leftBound &&
                transform.position.x <= rightBound &&
                transform.position.y + spriteHeight >= topBound &&
                transform.position.y <= bottomBound);
    }

    /**
     * @brief Log animation event for debugging
     * CHANGE: Conditional logging based on configuration
     */
    void AnimationManager::logAnimationEvent(const std::string& message) const {
        if (config_.enableAnimationLogging) {
            std::cout << "[AnimationManager] " << message << std::endl;
        }
    }

    /**
     * @brief Sort instances by rendering layer
     * CHANGE: Depth sorting for proper rendering order
     */
    void AnimationManager::sortInstancesByLayer() {
        std::sort(visibleInstances_.begin(), visibleInstances_.end(),
                 [this](AnimationInstanceID a, AnimationInstanceID b) {
                     auto instanceA = instances_.find(a);
                     auto instanceB = instances_.find(b);

                     if (instanceA == instances_.end() || instanceB == instances_.end()) {
                         return false;
                     }

                     return instanceA->second->getTransform().layer < instanceB->second->getTransform().layer;
                 });
    }

    /**
     * @brief Cleanup unused render batches
     * CHANGE: Memory management for render batches
     */
    void AnimationManager::cleanupRenderBatches() {
        // Remove empty batches to free memory
        renderBatches_.erase(
            std::remove_if(renderBatches_.begin(), renderBatches_.end(),
                          [](const std::unique_ptr<RenderBatch>& batch) {
                              return batch->count == 0;
                          }),
            renderBatches_.end());
    }

} // namespace engine::animation                                 // Not