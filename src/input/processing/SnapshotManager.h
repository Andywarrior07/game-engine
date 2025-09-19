/**
 * @file SnapshotManager.h
 * @brief Manages input snapshots for deterministic frame-based processing
 * @author AndrÃ©s Guerrero
 * @date 13-09-2025
 *
 * Coordinates snapshot creation, updates, and history management for
 * deterministic input handling. Critical for replay systems and networking.
 */

#pragma once

#include "../core/InputSnapshot.h"
#include "../devices/base/DeviceService.h"
#include "../mapping/ActionMap.h"

#include "../../memory/manager/MemoryManager.h"

#include <memory>
#include <atomic>
#include <mutex>
#include <array>

namespace engine::input::processing {
    /**
     * @brief Snapshot manager configuration
     */
    struct SnapshotManagerConfig {
        // History settings
        std::size_t historySize = 600; // 10 seconds at 60fps
        bool enableHistory = true; // Enable history recording
        bool enableCompression = false; // Compress history data

        // Performance settings
        bool useDoubleBuffering = true; // Double buffer for thread safety
        bool preallocateHistory = true; // Preallocate history buffer
        std::size_t maxEventsPerSnapshot = 256; // Maximum events to process

        // Snapshot features
        bool trackActionStates = true; // Track action mapping states
        bool trackDeviceStates = true; // Track raw device states
        bool interpolateAnalogInputs = true; // Smooth analog inputs

        // Memory settings
        bool useMemoryPool = true; // Use memory pool for snapshots
        std::size_t snapshotPoolSize = 128; // Pool size for snapshots
    };

    /**
     * @brief Manages input snapshots for deterministic processing
     *
     * Thread-safe snapshot management with double buffering, history tracking,
     * and efficient memory usage through pooling.
     */
    class SnapshotManager {
    public:
        using MemoryAllocator = engine::memory::MemoryManager;
        using SnapshotCallback = std::function<void(const InputSnapshot&)>;

        /**
         * @brief Constructor
         * @param deviceService Device service for input devices
         * @param actionMap Action mapping system
         * @param memoryManager Memory manager for allocations
         */
        explicit SnapshotManager(DeviceService* deviceService,
                                 ActionMap* actionMap,
                                 MemoryAllocator* memoryManager = nullptr) noexcept;

        /**
         * @brief Destructor
         */
        ~SnapshotManager();

        // Disable copy, enable move
        SnapshotManager(const SnapshotManager&) = delete;
        SnapshotManager& operator=(const SnapshotManager&) = delete;
        SnapshotManager(SnapshotManager&&) noexcept = default;
        SnapshotManager& operator=(SnapshotManager&&) noexcept = default;

        // ============================================================================
        // Initialization and Configuration
        // ============================================================================

        /**
         * @brief Initialize the snapshot manager
         * @param config Configuration settings
         * @return True if successful
         */
        bool initialize(const SnapshotManagerConfig& config = {});

        /**
         * @brief Shutdown the snapshot manager
         */
        void shutdown();

        /**
         * @brief Set configuration
         * @param config New configuration
         */
        void setConfig(const SnapshotManagerConfig& config);

        /**
         * @brief Get current configuration
         */
        [[nodiscard]] const SnapshotManagerConfig& getConfig() const noexcept {
            return config_;
        }

        // ============================================================================
        // Snapshot Management
        // ============================================================================

        /**
         * @brief Begin new frame and prepare snapshot
         * @param frameNumber Frame number
         * @param deltaTime Time since last frame
         * @return True if successful
         */
        bool beginFrame(std::uint64_t frameNumber, float deltaTime);

        /**
         * @brief Update current snapshot with device states
         * @return Number of devices updated
         */
        std::size_t updateDeviceStates() const;

        /**
         * @brief Update current snapshot with action states
         * @return Number of actions updated
         */
        std::size_t updateActionStates() const;

        /**
         * @brief Process input events into snapshot
         * @param events Events to process
         * @return Number of events processed
         */
        std::size_t processEvents(const std::vector<InputEvent>& events) const;

        /**
         * @brief Finalize current frame snapshot
         * @return Finalized snapshot pointer
         */
        const InputSnapshot* endFrame();

        /**
         * @brief Get current snapshot (read-only)
         */
        [[nodiscard]] const InputSnapshot* getCurrentSnapshot() const noexcept;

        /**
         * @brief Get previous snapshot (read-only)
         */
        [[nodiscard]] const InputSnapshot* getPreviousSnapshot() const noexcept;

        /**
         * @brief Swap buffers (for double buffering)
         */
        void swapBuffers() noexcept;

        // ============================================================================
        // History Management
        // ============================================================================

        /**
         * @brief Get input history
         */
        [[nodiscard]] InputHistory* getHistory() noexcept {
            return history_.get();
        }

        [[nodiscard]] const InputHistory* getHistory() const noexcept {
            return history_.get();
        }

        /**
         * @brief Get snapshot from history
         * @param frameOffset Negative offset from current frame
         */
        [[nodiscard]] const InputSnapshot* getHistoricalSnapshot(int frameOffset) const;

        /**
         * @brief Clear history
         */
        void clearHistory() const noexcept;

        /**
         * @brief Export history for replay
         * @return Serialized history data
         */
        [[nodiscard]] std::vector<std::uint8_t> exportHistory() const;

        /**
         * @brief Import history from replay
         * @param data Serialized history data
         * @return True if successful
         */
        bool importHistory(const std::vector<std::uint8_t>& data) const;

        // ============================================================================
        // Replay Support
        // ============================================================================

        /**
         * @brief Start recording for replay
         */
        void startRecording();

        /**
         * @brief Stop recording
         */
        void stopRecording();

        /**
         * @brief Check if recording
         */
        [[nodiscard]] bool isRecording() const noexcept {
            return isRecording_.load(std::memory_order_acquire);
        }

        /**
         * @brief Start replay playback
         * @param replayData Replay data to play
         * @return True if successful
         */
        bool startReplay(const std::vector<std::uint8_t>& replayData);

        /**
         * @brief Stop replay playback
         */
        void stopReplay();

        /**
         * @brief Check if replaying
         */
        [[nodiscard]] bool isReplaying() const noexcept {
            return isReplaying_.load(std::memory_order_acquire);
        }

        /**
         * @brief Get next replay snapshot
         * @return Snapshot or nullptr if replay finished
         */
        [[nodiscard]] const InputSnapshot* getReplaySnapshot();

        // ============================================================================
        // Callbacks and Events
        // ============================================================================

        /**
         * @brief Register snapshot callback
         * @param callback Callback function
         */
        void registerSnapshotCallback(SnapshotCallback callback);

        /**
         * @brief Clear all callbacks
         */
        void clearCallbacks();

        // ============================================================================
        // Statistics and Debugging
        // ============================================================================

        /**
         * @brief Snapshot statistics
         */
        struct Statistics {
            std::uint64_t totalSnapshots = 0;
            std::uint64_t droppedEvents = 0;
            std::uint32_t averageEventsPerSnapshot = 0;
            std::uint32_t peakEventsPerSnapshot = 0;
            float averageUpdateTime = 0.0f;
            float peakUpdateTime = 0.0f;
            std::size_t memoryUsage = 0;

            void reset() noexcept {
                totalSnapshots = 0;
                droppedEvents = 0;
                averageEventsPerSnapshot = 0;
                peakEventsPerSnapshot = 0;
                averageUpdateTime = 0.0f;
                peakUpdateTime = 0.0f;
                memoryUsage = 0;
            }
        };

        [[nodiscard]] const Statistics& getStatistics() const noexcept {
            return stats_;
        }

        /**
         * @brief Reset statistics
         */
        void resetStatistics() const noexcept;

        /**
         * @brief Validate snapshot integrity
         * @param snapshot Snapshot to validate
         * @return True if valid
         */
        [[nodiscard]] static bool validateSnapshot(const InputSnapshot& snapshot) noexcept;

    private:
        // ============================================================================
        // Member Variables
        // ============================================================================

        // Configuration
        SnapshotManagerConfig config_;
        bool initialized_ = false;

        // Services
        DeviceService* deviceService_;
        ActionMap* actionMap_;
        MemoryAllocator* memoryManager_;
        bool ownsMemoryManager_ = false;

        // Snapshot buffers (double buffering)
        static constexpr std::size_t BUFFER_COUNT = 2;
        std::array<std::unique_ptr<InputSnapshot>, BUFFER_COUNT> snapshots_;
        std::atomic<std::uint8_t> currentBufferIndex_{0};
        std::atomic<std::uint8_t> readBufferIndex_{1};

        // Current working snapshot
        InputSnapshot* currentSnapshot_ = nullptr;
        InputSnapshot* previousSnapshot_ = nullptr;

        // History management
        std::unique_ptr<InputHistory> history_;

        // Recording/Replay
        std::atomic<bool> isRecording_{false};
        std::atomic<bool> isReplaying_{false};
        std::vector<std::uint8_t> recordingBuffer_;
        std::vector<InputSnapshot> replayBuffer_;
        std::size_t replayIndex_ = 0;

        // Memory pool for snapshots
        std::unique_ptr<memory::PoolAllocator> snapshotPool_;

        // Thread safety
        mutable std::mutex snapshotMutex_;
        mutable std::mutex historyMutex_;
        mutable std::mutex callbackMutex_;

        // Callbacks
        std::vector<SnapshotCallback> callbacks_;

        // Statistics
        mutable Statistics stats_;
        std::chrono::steady_clock::time_point lastUpdateTime_;

        // Temporary buffers for processing
        std::vector<InputEvent> eventBuffer_;

        // ============================================================================
        // Internal Methods
        // ============================================================================

        /**
         * @brief Create snapshot buffers
         */
        bool createBuffers();

        /**
         * @brief Update keyboard state in snapshot
         */
        void updateKeyboardState(InputSnapshot& snapshot) const;

        /**
         * @brief Update mouse state in snapshot
         */
        void updateMouseState(InputSnapshot& snapshot) const;

        /**
         * @brief Update gamepad states in snapshot
         */
        void updateGamepadStates(InputSnapshot& snapshot) const;

        /**
         * @brief Update touch state in snapshot
         */
        void updateTouchState(InputSnapshot& snapshot) const;

        /**
         * @brief Process single event into snapshot
         * @param snapshot Target snapshot
         * @param event Event to process
         * @return True if processed
         */
        static bool processEventIntoSnapshot(InputSnapshot& snapshot, const InputEvent& event);

        /**
         * @brief Interpolate analog values between snapshots
         * @param current Current snapshot
         * @param previous Previous snapshot
         * @param alpha Interpolation factor
         */
        static void interpolateAnalogValues(InputSnapshot& current,
                                            const InputSnapshot& previous,
                                            float alpha);

        /**
         * @brief Compress snapshot for storage
         * @param snapshot Snapshot to compress
         * @return Compressed data
         */
        [[nodiscard]] std::vector<std::uint8_t> compressSnapshot(const InputSnapshot& snapshot) const;

        /**
         * @brief Decompress snapshot
         * @param data Compressed data
         * @return Decompressed snapshot
         */
        [[nodiscard]] InputSnapshot decompressSnapshot(const std::vector<std::uint8_t>& data) const;

        /**
         * @brief Record snapshot for replay
         * @param snapshot Snapshot to record
         */
        void recordSnapshot(const InputSnapshot& snapshot);

        /**
         * @brief Invoke snapshot callbacks
         * @param snapshot Snapshot to pass to callbacks
         */
        void invokeCallbacks(const InputSnapshot& snapshot) const;

        /**
         * @brief Update statistics
         * @param eventsProcessed Number of events processed
         * @param updateTime Time taken to update
         */
        void updateStatistics(std::size_t eventsProcessed, float updateTime) const;

        /**
         * @brief Allocate snapshot from pool
         */
        [[nodiscard]] InputSnapshot* allocateSnapshot() const;

        /**
         * @brief Deallocate snapshot to pool
         */
        void deallocateSnapshot(InputSnapshot* snapshot) const;
    };
} // namespace engine::input::processing