/**
 * @file CameraDebugger.h
 * @brief Debug utilities for camera system
 * @author Andrés Guerrero
 * @date 26-08-2025
 */

#pragma once

#include "../core/CameraTypes.h"
#include "../manager/CameraManager.h"
#include <iomanip>
#include <vector>

namespace engine::camera {

    /**
     * @brief Debug statistics for camera system
     */
    struct CameraDebugStats {
        std::size_t totalCameras = 0;          ///< Total number of cameras
        std::size_t activeCameras = 0;         ///< Number of active cameras
        std::size_t enabledCameras = 0;        ///< Number of enabled cameras
        std::size_t activeTransitions = 0;     ///< Active transitions
        std::size_t activeShakes = 0;          ///< Active shake effects
        float averageUpdateTime = 0.0f;        ///< Average update time (ms)
        float peakUpdateTime = 0.0f;           ///< Peak update time (ms)
        std::size_t memoryUsage = 0;           ///< Memory usage (bytes)
        std::uint64_t totalUpdates = 0;        ///< Total update calls
    };

    /**
     * @brief Debug visualization options
     */
    struct DebugVisualizationOptions {
        bool showFrustum = true;               ///< Show camera frustum
        bool showBounds = true;                ///< Show camera bounds
        bool showPath = false;                 ///< Show camera path
        bool showTarget = true;                ///< Show look-at target
        bool showAxes = true;                  ///< Show camera axes
        bool showGrid = false;                 ///< Show world grid
        bool showStats = true;                 ///< Show statistics overlay
        Vector3 frustumColor{1, 1, 0};         ///< Frustum color
        Vector3 boundsColor{0, 1, 0};          ///< Bounds color
        Vector3 pathColor{0, 0, 1};            ///< Path color
        float lineWidth = 1.0f;                ///< Debug line width
    };

    /**
     * @brief Camera debugging utilities
     *
     * Provides debugging and visualization tools for the camera system,
     * including statistics, state inspection, and visual debugging aids.
     */
    class CameraDebugger {
    public:
        /**
         * @brief Constructor
         * @param manager Camera manager to debug
         */
        explicit CameraDebugger(CameraManager* manager);

        // ========================================================================
        // STATISTICS
        // ========================================================================

        /**
         * @brief Get debug statistics
         * @return Current debug statistics
         */
        CameraDebugStats getStats() const;

        /**
         * @brief Generate statistics report
         * @return Formatted statistics string
         */
        std::string generateStatsReport() const;

        // ========================================================================
        // STATE INSPECTION
        // ========================================================================

        /**
         * @brief Get detailed camera information
         * @param cameraId Camera to inspect
         * @return Detailed camera state string
         */
        std::string inspectCamera(CameraID cameraId) const;

        /**
         * @brief Get all camera summaries
         * @return Summary of all cameras
         */
        std::string getAllCameraSummaries() const;

        // ========================================================================
        // PATH TRACKING
        // ========================================================================

        /**
         * @brief Start recording camera path
         * @param cameraId Camera to track
         */
        void startPathRecording(CameraID cameraId);

        /**
         * @brief Stop recording camera path
         */
        void stopPathRecording() {
            isRecording_ = false;
        }

        /**
         * @brief Get recorded path
         * @return Vector of recorded positions
         */
        const std::vector<Vector3>& getRecordedPath() const {
            return pathHistory_;
        }

        /**
         * @brief Clear recorded path
         */
        void clearPath() {
            pathHistory_.clear();
        }

        // ========================================================================
        // UPDATE TRACKING
        // ========================================================================

        /**
         * @brief Track update timing
         * @param deltaTime Update delta time
         */
        void trackUpdate(float deltaTime);

        /**
         * @brief Reset timing statistics
         */
        void resetStats();

        // ========================================================================
        // VALIDATION
        // ========================================================================

        /**
         * @brief Validate camera system
         * @return Validation report
         */
        std::string validateSystem() const;

        // ========================================================================
        // VISUALIZATION HELPERS
        // ========================================================================

        /**
         * @brief Get camera frustum vertices for rendering
         * @param cameraId Camera ID
         * @param viewport Current viewport
         * @return Frustum vertices (8 points)
         */
        std::vector<Vector3> getFrustumVertices(CameraID cameraId, const Viewport& viewport) const;

        /**
         * @brief Get debug lines for visualization
         * @param options Visualization options
         * @return Vector of line segments (pairs of points)
         */
        std::vector<std::pair<Vector3, Vector3>> getDebugLines(const DebugVisualizationOptions& options) const;

        // ========================================================================
        // CONFIGURATION
        // ========================================================================

        /**
         * @brief Get visualization options
         * @return Current visualization options
         */
        const DebugVisualizationOptions& getVisualizationOptions() const {
            return visualizationOptions_;
        }

        /**
         * @brief Set visualization options
         * @param options New visualization options
         */
        void setVisualizationOptions(DebugVisualizationOptions options) {
            visualizationOptions_ = std::move(options);
        }

        /**
         * @brief Set maximum path history size
         * @param maxSize Maximum number of positions to record
         */
        void setMaxPathHistory(const std::size_t maxSize) {
            maxPathHistory_ = maxSize;
        }

    private:
        CameraManager* cameraManager_;                        ///< Camera manager reference
        DebugVisualizationOptions visualizationOptions_;      ///< Visualization settings

        // Path tracking
        std::vector<Vector3> pathHistory_;                    ///< Recorded camera path
        CameraID recordingCameraId_ = INVALID_CAMERA_ID;     ///< Camera being recorded
        bool isRecording_ = false;                            ///< Whether recording is active
        std::size_t maxPathHistory_ = 1000;                   ///< Maximum path history size

        // Statistics
        std::uint64_t updateCount_ = 0;                       ///< Total update count
        float updateTimeSum_ = 0.0f;                          ///< Sum of update times
        float peakUpdateTime_ = 0.0f;                         ///< Peak update time

        /**
         * @brief Calculate average update time
         * @return Average time in milliseconds
         */
        float calculateAverageUpdateTime() const {
            if (updateCount_ == 0) return 0.0f;
            return updateTimeSum_ / static_cast<float>(updateCount_);
        }

        /**
         * @brief Format bytes for display
         * @param bytes Number of bytes
         * @return Formatted string
         */
        static std::string formatBytes(std::size_t bytes);
    };

} // namespace engine::camera