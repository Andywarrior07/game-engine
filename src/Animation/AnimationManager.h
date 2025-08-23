// //
// // Created by Andres Guerrero on 06-08-25.
// //
//
// #pragma once
//
// #include <SDL.h>                    // SDL core functionality
// #include <functional>               // For std::function callbacks
// #include <unordered_map>           // For fast hash-based lookups
// #include <vector>                  // For dynamic arrays
// #include <string>                  // For names and paths
// #include <chrono>                  // For timing and frame duration
// #include <memory>                  // For smart pointers
// #include <optional>                // For optional return values (C++17)
// #include <atomic>                  // For thread-safe operations
// #include <mutex>                   // For thread synchronization
// #include <array>                   // For fixed-size arrays
//
// #include "../Math/MathTypes.h"
// #include "../resources/core/ResourceHandle.h"
// #include "../resources/types/texture/TextureResource.h"
//
// namespace engine::animation {
//     using Vector2 = math::Vector2;
//
//     // Forward declarations
//     class AnimationManager;
//     class SpriteSheet;
//     class Animation;
//     class AnimationInstance;
//
//     // Strong typing for animation identifiers
//     using AnimationID = std::uint32_t;
//     using SpriteSheetID = std::uint32_t;
//     using AnimationInstanceID = std::uint32_t;
//
//     // Invalid ID constants for error checking
//     constexpr AnimationID INVALID_ANIMATION_ID = 0;
//     constexpr SpriteSheetID INVALID_SPRITESHEET_ID = 0;
//     constexpr AnimationInstanceID INVALID_INSTANCE_ID = 0;
//
//     /**
//      * @brief Rectangle structure for sprite frame definitions
//      */
//     struct SpriteRect {
//         int x = 0;                      // X position in sprite sheet
//         int y = 0;                      // Y position in sprite sheet
//         int width = 0;                  // Width of sprite frame
//         int height = 0;                 // Height of sprite frame
//
//         // Constructor for easy initialization
//         SpriteRect() = default;
//         SpriteRect(int x_, int y_, int w, int h) : x(x_), y(y_), width(w), height(h) {}
//
//         // Convert to SDL_Rect
//         SDL_Rect toSDLRect() const { return {x, y, width, height}; }
//     };
//
//     /**
//      * @brief Animation playback modes
//      */
//     enum class AnimationMode : std::uint8_t {
//         ONCE = 0,                   // Play once and stop at last frame
//         LOOP = 1,                   // Loop continuously
//         PING_PONG = 2,              // Play forward then backward continuously
//         REVERSE = 3,                // Play in reverse once
//         REVERSE_LOOP = 4            // Loop in reverse continuously
//     };
//
//     /**
//      * @brief Animation state for tracking playback
//      */
//     enum class AnimationState : std::uint8_t {
//         STOPPED = 0,                // Animation is not playing
//         PLAYING = 1,                // Animation is currently playing
//         PAUSED = 2,                 // Animation is paused
//         FINISHED = 3                // Animation completed (for non-looping animations)
//     };
//
//     /**
//      * @brief Sprite flipping options
//      */
//     enum class SpriteFlip : std::uint8_t {
//         NONE = 0,                   // No flipping
//         HORIZONTAL = 1,             // Flip horizontally (mirror)
//         VERTICAL = 2,               // Flip vertically
//         BOTH = 3                    // Flip both horizontally and vertically
//     };
//
//     /**
//      * @brief Animation event types for callbacks
//      */
//     enum class AnimationEventType : std::uint8_t {
//         STARTED = 0,                // Animation just started playing
//         FINISHED = 1,               // Animation finished (non-looping)
//         LOOPED = 2,                 // Animation completed a loop cycle
//         FRAME_CHANGED = 3,          // Current frame changed
//         PAUSED = 4,                 // Animation was paused
//         RESUMED = 5                 // Animation was resumed
//     };
//
//     /**
//      * @brief Configuration for animation behavior
//      */
//     struct AnimationConfig {
//         float frameRate = 12.0f;                    // Frames per second
//         AnimationMode mode = AnimationMode::LOOP;   // Playback mode
//         bool autoPlay = false;                      // Start playing immediately
//         int startFrame = 0;                         // Frame to start from
//         int endFrame = -1;                          // Frame to end at (-1 = last frame)
//         float speed = 1.0f;                         // Playback speed multiplier
//         int priority = 0;                           // Animation priority (higher = more important)
//     };
//
//     /**
//      * @brief Transform data for sprite rendering
//      */
//     struct SpriteTransform {
//         Vector2 position{0.0f, 0.0f};              // World position
//         Vector2 scale{1.0f, 1.0f};                 // Scale factors
//         float rotation = 0.0f;                      // Rotation in degrees
//         Vector2 origin{0.0f, 0.0f};                // Origin point for rotation/scaling
//         SpriteFlip flip = SpriteFlip::NONE;         // Sprite flipping
//         std::uint8_t alpha = 255;                   // Alpha transparency (0-255)
//         SDL_Color tint{255, 255, 255, 255};        // Color tint
//         int layer = 0;                              // Rendering layer/depth
//     };
//
//     /**
//      * @brief Callback function types for animation events
//      */
//     using AnimationCallback = std::function<void(AnimationInstanceID instanceId, AnimationEventType eventType, int frameIndex)>;
//     using FrameCallback = std::function<void(AnimationInstanceID instanceId, int oldFrame, int newFrame)>;
//
//     /**
//      * @brief Configuration parameters for AnimationManager
//      */
//     struct AnimationManagerConfig {
//         bool enableAnimationLogging = false;        // Log animation events for debugging
//         bool enableFrameCallback = true;            // Enable per-frame callbacks
//         int maxInstances = 10000;                   // Maximum number of animation instances
//         bool enableBatching = false;                 // Enable draw call batching
//         bool enableCulling = true;                  // Enable frustum culling
//         Vector2 cullMargin{100.0f, 100.0f};        // Margin for culling calculations
//         std::chrono::milliseconds updateInterval{16}; // Update interval for animations
//     };
//
//     /**
//      * @brief Sprite sheet data containing texture and frame information
//      *
//      * A sprite sheet contains a texture and metadata about how it's divided into frames.
//      * This allows efficient rendering of animated sprites from a single texture.
//      */
//     class SpriteSheet {
//     public:
//         /**
//          * @brief Constructor for sprite sheet
//          * @param id Unique identifier for this sprite sheet
//          * @param name Human-readable name
//          * @param texture Handle to the texture resource
//          */
//         SpriteSheet(SpriteSheetID id, std::string name, engine::resources::ResourceHandle<engine::resources::TextureResource> texture);
//
//         /**
//          * @brief Destructor
//          */
//         ~SpriteSheet() = default;
//
//         // Non-copyable but moveable
//         SpriteSheet(const SpriteSheet&) = delete;
//         SpriteSheet& operator=(const SpriteSheet&) = delete;
//         SpriteSheet(SpriteSheet&&) = default;
//         SpriteSheet& operator=(SpriteSheet&&) = default;
//
//         // === Accessors ===
//
//         SpriteSheetID getId() const noexcept { return id_; }
//         const std::string& getName() const noexcept { return name_; }
//
//         /**
//          * @brief Get the texture resource handle
//          * @return Texture handle (may be invalid if texture failed to load)
//          */
//         const engine::resources::ResourceHandle<engine::resources::TextureResource>& getTexture() const noexcept { return texture_; }
//
//         /**
//          * @brief Check if sprite sheet is ready for use
//          * @return true if texture is loaded and frames are defined
//          */
//         bool isReady() const noexcept;
//
//         /**
//          * @brief Get number of frames in this sprite sheet
//          * @return Total frame count
//          */
//         int getFrameCount() const noexcept { return static_cast<int>(frames_.size()); }
//
//         /**
//          * @brief Get frame rectangle by index
//          * @param frameIndex Index of frame to get
//          * @return Frame rectangle, or empty rect if index is invalid
//          */
//         SpriteRect getFrame(int frameIndex) const;
//
//         /**
//          * @brief Get all frame rectangles
//          * @return Vector of all frame rectangles
//          */
//         const std::vector<SpriteRect>& getAllFrames() const noexcept { return frames_; }
//
//         // === Frame Management ===
//
//         /**
//          * @brief Add a single frame to the sprite sheet
//          * @param frame Frame rectangle to add
//          * @return Index of added frame
//          */
//         int addFrame(const SpriteRect& frame);
//
//         /**
//          * @brief Add multiple frames to the sprite sheet
//          * @param frames Vector of frame rectangles to add
//          * @return Number of frames added
//          */
//         int addFrames(const std::vector<SpriteRect>& frames);
//
//         /**
//          * @brief Generate frames automatically from grid layout
//          * @param columns Number of columns in grid
//          * @param rows Number of rows in grid
//          * @param frameWidth Width of each frame
//          * @param frameHeight Height of each frame
//          * @param startX Starting X position (default: 0)
//          * @param startY Starting Y position (default: 0)
//          * @param spacingX Horizontal spacing between frames (default: 0)
//          * @param spacingY Vertical spacing between frames (default: 0)
//          * @return Number of frames generated
//          */
//         int generateGridFrames(int columns, int rows, int frameWidth, int frameHeight,
//                               int startX = 0, int startY = 0, int spacingX = 0, int spacingY = 0);
//
//         /**
//          * @brief Clear all frames
//          */
//         void clearFrames();
//
//         /**
//          * @brief Remove frame at specific index
//          * @param frameIndex Index of frame to remove
//          * @return true if frame was removed
//          */
//         bool removeFrame(int frameIndex);
//
//         // === Utilities ===
//
//         /**
//          * @brief Get texture dimensions
//          * @return Vector2 with width and height, or (0,0) if texture not ready
//          */
//         Vector2 getTextureDimensions() const;
//
//         /**
//          * @brief Validate that all frames are within texture bounds
//          * @return true if all frames are valid
//          */
//         bool validateFrames() const;
//
//         /**
//          * @brief Get memory usage information
//          * @return Approximate memory usage in bytes
//          */
//         std::size_t getMemoryUsage() const;
//
//     private:
//         SpriteSheetID id_;                                                          // Unique identifier
//         std::string name_;                                                          // Human-readable name
//         engine::resources::ResourceHandle<engine::resources::TextureResource> texture_;  // Texture resource
//         std::vector<SpriteRect> frames_;                                           // Frame definitions
//     };
//
//     /**
//      * @brief Animation definition containing frame sequence and timing
//      *
//      * An animation defines a sequence of frames from a sprite sheet with timing information.
//      * Multiple animation instances can use the same animation definition.
//      */
//     class Animation {
//     public:
//         /**
//          * @brief Constructor for animation
//          * @param id Unique identifier for this animation
//          * @param name Human-readable name
//          * @param spriteSheetId ID of sprite sheet to use
//          * @param config Animation configuration
//          */
//         Animation(AnimationID id, std::string name, SpriteSheetID spriteSheetId, const AnimationConfig& config = {});
//
//         /**
//          * @brief Destructor
//          */
//         ~Animation() = default;
//
//         // Non-copyable but moveable
//         Animation(const Animation&) = delete;
//         Animation& operator=(const Animation&) = delete;
//         Animation(Animation&&) = default;
//         Animation& operator=(Animation&&) = default;
//
//         // === Accessors ===
//
//         AnimationID getId() const noexcept { return id_; }
//         const std::string& getName() const noexcept { return name_; }
//         SpriteSheetID getSpriteSheetId() const noexcept { return spriteSheetId_; }
//         const AnimationConfig& getConfig() const noexcept { return config_; }
//
//         /**
//          * @brief Get frame count for this animation
//          * @return Number of frames in sequence
//          */
//         int getFrameCount() const noexcept { return static_cast<int>(frameSequence_.size()); }
//
//         /**
//          * @brief Get frame index at specific position in sequence
//          * @param sequenceIndex Position in animation sequence
//          * @return Frame index in sprite sheet, or -1 if invalid
//          */
//         int getFrameIndex(int sequenceIndex) const;
//
//         /**
//          * @brief Get all frame indices in sequence
//          * @return Vector of frame indices
//          */
//         const std::vector<int>& getFrameSequence() const noexcept { return frameSequence_; }
//
//         /**
//          * @brief Get duration of single frame in seconds
//          * @return Frame duration based on frame rate
//          */
//         float getFrameDuration() const noexcept { return 1.0f / config_.frameRate; }
//
//         /**
//          * @brief Get total animation duration in seconds
//          * @return Total duration for one playthrough
//          */
//         float getTotalDuration() const noexcept;
//
//         // === Frame Sequence Management ===
//
//         /**
//          * @brief Set frame sequence for animation
//          * @param frameIndices Vector of frame indices from sprite sheet
//          */
//         void setFrameSequence(const std::vector<int>& frameIndices);
//
//         /**
//          * @brief Add frame to end of sequence
//          * @param frameIndex Frame index to add
//          */
//         void addFrame(int frameIndex);
//
//         /**
//          * @brief Insert frame at specific position
//          * @param sequenceIndex Position to insert at
//          * @param frameIndex Frame index to insert
//          * @return true if insertion was successful
//          */
//         bool insertFrame(int sequenceIndex, int frameIndex);
//
//         /**
//          * @brief Remove frame from sequence
//          * @param sequenceIndex Position to remove from
//          * @return true if removal was successful
//          */
//         bool removeFrame(int sequenceIndex);
//
//         /**
//          * @brief Generate sequence from range of frame indices
//          * @param startFrame First frame index (inclusive)
//          * @param endFrame Last frame index (inclusive)
//          * @param step Step between frames (default: 1)
//          */
//         void generateSequence(int startFrame, int endFrame, int step = 1);
//
//         /**
//          * @brief Clear frame sequence
//          */
//         void clearSequence();
//
//         // === Configuration ===
//
//         /**
//          * @brief Update animation configuration
//          * @param config New configuration
//          */
//         void updateConfig(const AnimationConfig& config) { config_ = config; }
//
//         /**
//          * @brief Set frame rate
//          * @param frameRate New frame rate in FPS
//          */
//         void setFrameRate(const float frameRate) { config_.frameRate = frameRate; }
//
//         /**
//          * @brief Set animation mode
//          * @param mode New animation mode
//          */
//         void setMode(const AnimationMode mode) { config_.mode = mode; }
//
//         /**
//          * @brief Set playback speed
//          * @param speed Speed multiplier (1.0 = normal speed)
//          */
//         void setSpeed(const float speed) { config_.speed = speed; }
//
//         // === Validation ===
//
//         /**
//          * @brief Validate animation against sprite sheet
//          * @param spriteSheet Sprite sheet to validate against
//          * @return true if all frame indices are valid
//          */
//         bool validate(const SpriteSheet& spriteSheet) const;
//
//     private:
//         AnimationID id_;                            // Unique identifier
//         std::string name_;                          // Human-readable name
//         SpriteSheetID spriteSheetId_;              // Associated sprite sheet
//         AnimationConfig config_;                    // Animation configuration
//         std::vector<int> frameSequence_;           // Sequence of frame indices
//     };
//
//     /**
//      * @brief Animation instance for runtime playback
//      *
//      * Represents a playing instance of an animation with its own state and transform.
//      * Multiple instances can play the same animation simultaneously.
//      */
//     class AnimationInstance {
//     public:
//         /**
//          * @brief Constructor for animation instance
//          * @param id Unique identifier for this instance
//          * @param animationId ID of animation to play
//          * @param transform Initial transform data
//          */
//         AnimationInstance(AnimationInstanceID id, AnimationID animationId, const SpriteTransform& transform = {});
//
//         /**
//          * @brief Destructor
//          */
//         ~AnimationInstance() = default;
//
//         // Non-copyable but moveable
//         AnimationInstance(const AnimationInstance&) = delete;
//         AnimationInstance& operator=(const AnimationInstance&) = delete;
//         AnimationInstance(AnimationInstance&&) = default;
//         AnimationInstance& operator=(AnimationInstance&&) = default;
//
//         // === Accessors ===
//
//         AnimationInstanceID getId() const noexcept { return id_; }
//         AnimationID getAnimationId() const noexcept { return animationId_; }
//
//         /**
//          * @brief Get current animation state
//          * @return Current state (playing, paused, etc.)
//          */
//         AnimationState getState() const noexcept { return state_; }
//
//         /**
//          * @brief Get current frame index in animation sequence
//          * @return Current frame index
//          */
//         int getCurrentFrame() const noexcept { return currentFrame_; }
//
//         /**
//          * @brief Get current playback time
//          * @return Time since animation started in seconds
//          */
//         float getCurrentTime() const noexcept { return currentTime_; }
//
//         /**
//          * @brief Get progress through current loop (0.0 to 1.0)
//          * @return Progress value
//          */
//         float getProgress() const noexcept;
//
//         /**
//          * @brief Get number of completed loops
//          * @return Loop count
//          */
//         int getLoopCount() const noexcept { return loopCount_; }
//
//         /**
//          * @brief Check if animation is playing
//          * @return true if currently playing
//          */
//         bool isPlaying() const noexcept { return state_ == AnimationState::PLAYING; }
//
//         /**
//          * @brief Check if animation is finished
//          * @return true if finished (non-looping animations only)
//          */
//         bool isFinished() const noexcept { return state_ == AnimationState::FINISHED; }
//
//         // === Transform Management ===
//
//         /**
//          * @brief Get sprite transform data
//          * @return Current transform
//          */
//         const SpriteTransform& getTransform() const noexcept { return transform_; }
//
//         /**
//          * @brief Set sprite transform data
//          * @param transform New transform data
//          */
//         void setTransform(const SpriteTransform& transform) { transform_ = transform; }
//
//         /**
//          * @brief Set position
//          * @param position New position
//          */
//         void setPosition(const Vector2& position) { transform_.position = position; }
//
//         /**
//          * @brief Set scale
//          * @param scale New scale factors
//          */
//         void setScale(const Vector2& scale) { transform_.scale = scale; }
//
//         /**
//          * @brief Set rotation
//          * @param rotation New rotation in degrees
//          */
//         void setRotation(const float rotation) { transform_.rotation = rotation; }
//
//         /**
//          * @brief Set alpha transparency
//          * @param alpha Alpha value (0-255)
//          */
//         void setAlpha(const std::uint8_t alpha) { transform_.alpha = alpha; }
//
//         /**
//          * @brief Set color tint
//          * @param tint Color tint
//          */
//         void setTint(const SDL_Color& tint) { transform_.tint = tint; }
//
//         /**
//          * @brief Set sprite flipping
//          * @param flip Flip mode
//          */
//         void setFlip(const SpriteFlip flip) { transform_.flip = flip; }
//
//         /**
//          * @brief Set rendering layer
//          * @param layer Layer/depth value
//          */
//         void setLayer(const int layer) { transform_.layer = layer; }
//
//         // === Playback Control ===
//
//         /**
//          * @brief Start or resume animation playback
//          */
//         void play();
//
//         /**
//          * @brief Pause animation playback
//          */
//         void pause();
//
//         /**
//          * @brief Stop animation and reset to beginning
//          */
//         void stop();
//
//         /**
//          * @brief Restart animation from beginning
//          */
//         void restart();
//
//         /**
//          * @brief Set current frame
//          * @param frameIndex Frame index to jump to
//          */
//         void setCurrentFrame(int frameIndex);
//
//         /**
//          * @brief Set playback progress (0.0 to 1.0)
//          * @param progress Progress through animation
//          */
//         void setProgress(float progress);
//
//         // === Runtime Configuration ===
//
//         /**
//          * @brief Set playback speed multiplier
//          * @param speed Speed multiplier (1.0 = normal)
//          */
//         void setSpeed(const float speed) { speed_ = speed; }
//
//         /**
//          * @brief Get current speed multiplier
//          * @return Speed multiplier
//          */
//         float getSpeed() const noexcept { return speed_; }
//
//         /**
//          * @brief Set animation callback
//          * @param callback Callback function for animation events
//          */
//         void setCallback(const AnimationCallback& callback) { callback_ = callback; }
//
//         /**
//          * @brief Set frame change callback
//          * @param callback Callback function for frame changes
//          */
//         void setFrameCallback(const FrameCallback& callback) { frameCallback_ = callback; }
//
//         // === Update and Rendering ===
//
//         /**
//          * @brief Update animation state
//          * @param deltaTime Time elapsed since last update in seconds
//          * @param animation Animation definition to use for updating
//          * @return true if frame changed this update
//          */
//         bool update(float deltaTime, const Animation& animation);
//
//     private:
//         AnimationInstanceID id_;                    // Unique identifier
//         AnimationID animationId_;                   // Associated animation
//         SpriteTransform transform_;                 // Transform data for rendering
//
//         // Playback state
//         AnimationState state_ = AnimationState::STOPPED;  // Current state
//         int currentFrame_ = 0;                      // Current frame in sequence
//         float currentTime_ = 0.0f;                  // Current playback time
//         int loopCount_ = 0;                         // Number of completed loops
//         bool playingForward_ = true;                // Direction for ping-pong mode
//
//         // Runtime configuration
//         float speed_ = 1.0f;                        // Playback speed multiplier
//
//         // Callbacks
//         AnimationCallback callback_;                // Animation event callback
//         FrameCallback frameCallback_;               // Frame change callback
//
//         // Internal helpers
//         void triggerCallback(AnimationEventType eventType);
//         void triggerFrameCallback(int oldFrame, int newFrame);
//     };
//
//     /**
//      * @brief High-performance 2D sprite animation manager
//      *
//      * This class provides:
//      * - Sprite sheet management with texture resource integration
//      * - Animation definition and sequencing
//      * - Multiple simultaneous animation instances
//      * - Flexible playback modes (loop, ping-pong, etc.)
//      * - Event callbacks for animation events
//      * - Batch rendering for performance
//      * - Memory-efficient resource sharing
//      */
//     class AnimationManager {
//     public:
//         /**
//          * @brief Constructor with configuration
//          * @param config Configuration parameters
//          */
//         explicit AnimationManager(const AnimationManagerConfig& config = {});
//
//         /**
//          * @brief Destructor - cleanup all resources
//          */
//         ~AnimationManager();
//
//         // Non-copyable but moveable
//         AnimationManager(const AnimationManager&) = delete;
//         AnimationManager& operator=(const AnimationManager&) = delete;
//         AnimationManager(AnimationManager&&) = default;
//         AnimationManager& operator=(AnimationManager&&) = default;
//
//         /**
//          * @brief Initialize the animation manager
//          * @return true if initialization was successful
//          */
//         bool initialize();
//
//         /**
//          * @brief Shutdown and cleanup all resources
//          */
//         void shutdown();
//
//         /**
//          * @brief Update all active animation instances
//          * @param deltaTime Time elapsed since last update in seconds
//          */
//         void update(float deltaTime);
//
//         /**
//          * @brief Render all visible animation instances
//          * @param renderer SDL renderer to use for drawing
//          * @param cameraPosition Camera position for culling (optional)
//          * @param viewportSize Viewport size for culling (optional)
//          */
//         void render(SDL_Renderer* renderer, const Vector2& cameraPosition = {}, const Vector2& viewportSize = {});
//
//         // === Sprite Sheet Management ===
//
//         /**
//          * @brief Create sprite sheet from texture resource
//          * @param name Human-readable name for sprite sheet
//          * @param texture Texture resource handle
//          * @return SpriteSheetID for created sprite sheet, or INVALID_SPRITESHEET_ID if failed
//          */
//         SpriteSheetID createSpriteSheet(const std::string& name, engine::resources::ResourceHandle<engine::resources::TextureResource> texture);
//
//         /**
//          * @brief Remove sprite sheet and all associated animations
//          * @param spriteSheetId Sprite sheet to remove
//          * @return true if sprite sheet was removed
//          */
//         bool removeSpriteSheet(SpriteSheetID spriteSheetId);
//
//         /**
//          * @brief Get sprite sheet by ID
//          * @param spriteSheetId Sprite sheet ID
//          * @return Pointer to sprite sheet, or nullptr if not found
//          */
//         SpriteSheet* getSpriteSheet(SpriteSheetID spriteSheetId);
//
//         /**
//          * @brief Get sprite sheet by ID (const version)
//          * @param spriteSheetId Sprite sheet ID
//          * @return Const pointer to sprite sheet, or nullptr if not found
//          */
//         const SpriteSheet* getSpriteSheet(SpriteSheetID spriteSheetId) const;
//
//         // === Animation Management ===
//
//         /**
//          * @brief Create animation from sprite sheet
//          * @param name Human-readable name for animation
//          * @param spriteSheetId Sprite sheet to use
//          * @param config Animation configuration
//          * @return AnimationID for created animation, or INVALID_ANIMATION_ID if failed
//          */
//         AnimationID createAnimation(const std::string& name, SpriteSheetID spriteSheetId, const AnimationConfig& config = {});
//
//         /**
//          * @brief Remove animation and all its instances
//          * @param animationId Animation to remove
//          * @return true if animation was removed
//          */
//         bool removeAnimation(AnimationID animationId);
//
//         /**
//          * @brief Get animation by ID
//          * @param animationId Animation ID
//          * @return Pointer to animation, or nullptr if not found
//          */
//         Animation* getAnimation(AnimationID animationId);
//
//         /**
//          * @brief Get animation by ID (const version)
//          * @param animationId Animation ID
//          * @return Const pointer to animation, or nullptr if not found
//          */
//         const Animation* getAnimation(AnimationID animationId) const;
//
//         // === Animation Instance Management ===
//
//         /**
//          * @brief Create animation instance
//          * @param animationId Animation to instantiate
//          * @param transform Initial transform data
//          * @return AnimationInstanceID for created instance, or INVALID_INSTANCE_ID if failed
//          */
//         AnimationInstanceID createInstance(AnimationID animationId, const SpriteTransform& transform = {});
//
//         /**
//          * @brief Remove animation instance
//          * @param instanceId Instance to remove
//          * @return true if instance was removed
//          */
//         bool removeInstance(AnimationInstanceID instanceId);
//
//         /**
//          * @brief Get animation instance by ID
//          * @param instanceId Instance ID
//          * @return Pointer to instance, or nullptr if not found
//          */
//         AnimationInstance* getInstance(AnimationInstanceID instanceId);
//
//         /**
//          * @brief Get animation instance by ID (const version)
//          * @param instanceId Instance ID
//          * @return Const pointer to instance, or nullptr if not found
//          */
//         const AnimationInstance* getInstance(AnimationInstanceID instanceId) const;
//
//         /**
//          * @brief Get all instances of a specific animation
//          * @param animationId Animation ID to search for
//          * @return Vector of instance IDs
//          */
//         std::vector<AnimationInstanceID> getInstancesOfAnimation(AnimationID animationId) const;
//
//         /**
//          * @brief Remove all instances of a specific animation
//          * @param animationId Animation ID
//          * @return Number of instances removed
//          */
//         int removeAllInstancesOfAnimation(AnimationID animationId);
//
//         // === Batch Operations ===
//
//         /**
//          * @brief Play all instances of an animation
//          * @param animationId Animation ID
//          */
//         void playAllInstances(AnimationID animationId);
//
//         /**
//          * @brief Pause all instances of an animation
//          * @param animationId Animation ID
//          */
//         void pauseAllInstances(AnimationID animationId);
//
//         /**
//          * @brief Stop all instances of an animation
//          * @param animationId Animation ID
//          */
//         void stopAllInstances(AnimationID animationId);
//
//         /**
//          * @brief Clear all animation instances
//          */
//         void clearAllInstances();
//
//         // === Configuration and Stats ===
//
//         /**
//          * @brief Get current configuration
//          * @return Current configuration
//          */
//         const AnimationManagerConfig& getConfig() const noexcept { return config_; }
//
//         /**
//          * @brief Update configuration
//          * @param config New configuration
//          */
//         void updateConfig(const AnimationManagerConfig& config) { config_ = config; }
//
//         /**
//          * @brief Get number of active animation instances
//          * @return Number of instances
//          */
//         std::size_t getInstanceCount() const;
//
//         /**
//          * @brief Get number of playing animation instances
//          * @return Number of playing instances
//          */
//         std::size_t getPlayingInstanceCount() const;
//
//         /**
//          * @brief Get memory usage statistics
//          * @return Approximate memory usage in bytes
//          */
//         std::size_t getMemoryUsage() const;
//
//         /**
//          * @brief Get debug information about animation system
//          * @return String with detailed debug information
//          */
//         std::string getDebugInfo() const;
//
//         /**
//          * @brief Get performance statistics
//          * @return String with performance metrics
//          */
//         std::string getPerformanceStats() const;
//
//     private:
//         // === Internal Structures ===
//
//         /**
//          * @brief Render batch for efficient drawing
//          */
//         struct RenderBatch {
//             engine::resources::ResourceHandle<engine::resources::TextureResource> texture;  // Shared texture
//             std::vector<AnimationInstanceID> instances;     // Instances using this texture
//             SDL_Rect* srcRects = nullptr;                   // Source rectangles array
//             SDL_Rect* dstRects = nullptr;                   // Destination rectangles array
//             int count = 0;                                  // Number of sprites in batch
//             int capacity = 0;                               // Maximum capacity
//
//             RenderBatch() = default;
//             ~RenderBatch() {
//                 delete[] srcRects;
//                 delete[] dstRects;
//             }
//
//             // Non-copyable but moveable
//             RenderBatch(const RenderBatch&) = delete;
//             RenderBatch& operator=(const RenderBatch&) = delete;
//             RenderBatch(RenderBatch&& other) noexcept
//                 : texture(std::move(other.texture))
//                 , instances(std::move(other.instances))
//                 , srcRects(other.srcRects)
//                 , dstRects(other.dstRects)
//                 , count(other.count)
//                 , capacity(other.capacity) {
//                 other.srcRects = nullptr;
//                 other.dstRects = nullptr;
//                 other.count = 0;
//                 other.capacity = 0;
//             }
//         };
//
//         // === Member Variables ===
//
//         // Configuration and state
//         AnimationManagerConfig config_;                     // Configuration parameters
//         bool initialized_ = false;                          // Initialization state
//
//         // ID generation
//         SpriteSheetID nextSpriteSheetId_ = 1;              // Next sprite sheet ID
//         AnimationID nextAnimationId_ = 1;                  // Next animation ID
//         AnimationInstanceID nextInstanceId_ = 1;           // Next instance ID
//
//         // Resource storage
//         std::unordered_map<SpriteSheetID, std::unique_ptr<SpriteSheet>> spriteSheets_;    // All sprite sheets
//         std::unordered_map<AnimationID, std::unique_ptr<Animation>> animations_;          // All animations
//         std::unordered_map<AnimationInstanceID, std::unique_ptr<AnimationInstance>> instances_;  // All instances
//
//         // Rendering optimization
//         std::vector<std::unique_ptr<RenderBatch>> renderBatches_;  // Batches for efficient rendering
//         std::vector<AnimationInstanceID> visibleInstances_;       // Currently visible instances
//
//         // Threading and performance
//         mutable std::mutex animationMutex_;                // Mutex for thread-safe operations
//         std::chrono::steady_clock::time_point lastUpdateTime_;    // Last update timestamp
//
//         // Statistics
//         mutable std::atomic<std::uint64_t> framesRendered_{0};    // Total frames rendered
//         mutable std::atomic<std::uint64_t> drawCalls_{0};         // Total draw calls made
//         mutable std::atomic<std::uint64_t> instancesUpdated_{0};  // Total instances updated
//
//         // === Internal Methods ===
//
//         /**
//          * @brief Validate sprite sheet ID
//          * @param spriteSheetId ID to validate
//          * @return true if ID is valid
//          */
//         bool isValidSpriteSheetId(SpriteSheetID spriteSheetId) const;
//
//         /**
//          * @brief Validate animation ID
//          * @param animationId ID to validate
//          * @return true if ID is valid
//          */
//         bool isValidAnimationId(AnimationID animationId) const;
//
//         /**
//          * @brief Validate animation instance ID
//          * @param instanceId ID to validate
//          * @return true if ID is valid
//          */
//         bool isValidInstanceId(AnimationInstanceID instanceId) const;
//
//         /**
//          * @brief Update frustum culling for instances
//          * @param cameraPosition Camera position for culling
//          * @param viewportSize Viewport size for culling
//          */
//         void updateCulling(const Vector2& cameraPosition, const Vector2& viewportSize);
//
//         /**
//          * @brief Prepare render batches for efficient rendering
//          */
//         void prepareRenderBatches();
//
//         /**
//          * @brief Render a single batch
//          * @param renderer SDL renderer
//          * @param batch Render batch to draw
//          */
//         void renderBatch(SDL_Renderer* renderer, const RenderBatch& batch);
//
//         /**
//          * @brief Calculate destination rectangle for sprite rendering
//          * @param instance Animation instance
//          * @param spriteSheet Sprite sheet containing the sprite
//          * @param frameRect Source frame rectangle
//          * @return Destination rectangle for rendering
//          */
//         SDL_Rect calculateDestRect(const AnimationInstance& instance, const SpriteSheet& spriteSheet, const SpriteRect& frameRect) const;
//
//         /**
//          * @brief Check if instance is visible within camera view
//          * @param instance Animation instance to check
//          * @param cameraPosition Camera position
//          * @param viewportSize Viewport size
//          * @return true if instance should be rendered
//          */
//         bool isInstanceVisible(const AnimationInstance& instance, const Vector2& cameraPosition, const Vector2& viewportSize) const;
//
//         /**
//          * @brief Log animation event for debugging
//          * @param message Log message
//          */
//         void logAnimationEvent(const std::string& message) const;
//
//         /**
//          * @brief Sort instances by rendering layer
//          */
//         void sortInstancesByLayer();
//
//         /**
//          * @brief Cleanup unused render batches
//          */
//         void cleanupRenderBatches();
//     };
//
// } // namespace engine::animation