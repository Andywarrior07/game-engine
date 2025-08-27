/**
 * @file CameraDebugger.cpp
 * @brief Debug utilities for camera system
 * @author Andrés Guerrero
 * @date 26-08-2025
 */
#include "CameraDebugger.h"

#include "../manager/CameraRegistry.h"
#include "../cameras/Camera2D.h"
#include "../cameras/Camera3D.h"

namespace engine::camera {
    CameraDebugger::CameraDebugger(CameraManager* manager)
                : cameraManager_(manager) {
        pathHistory_.reserve(1000);
    }

    CameraDebugStats CameraDebugger::getStats() const {
        CameraDebugStats stats;

        if (!cameraManager_) return stats;

        stats.totalCameras = cameraManager_->getCameraCount();
        stats.activeTransitions = cameraManager_->getActiveTransitionCount();
        stats.activeShakes = cameraManager_->getActiveShakeCount();
        stats.memoryUsage = cameraManager_->getMemoryUsage();

        // Count active and enabled cameras
        for (const auto cameraIds = cameraManager_->getRegistry().getAllCameraIds(); const CameraID id : cameraIds) {
            if (const auto* camera = cameraManager_->getCamera(id)) {
                if (camera->isActive()) stats.activeCameras++;
                if (camera->isEnabled()) stats.enabledCameras++;
            }
        }

        // Calculate timing stats
        stats.averageUpdateTime = calculateAverageUpdateTime();
        stats.peakUpdateTime = peakUpdateTime_;
        stats.totalUpdates = updateCount_;

        return stats;
    }

    std::string CameraDebugger::generateStatsReport() const {
        const CameraDebugStats stats = getStats();
        std::ostringstream oss;

        oss << "=== Camera System Statistics ===\n";
        oss << "Cameras: " << stats.activeCameras << "/" << stats.totalCameras << " active\n";
        oss << "Enabled: " << stats.enabledCameras << "/" << stats.totalCameras << "\n";
        oss << "Transitions: " << stats.activeTransitions << " active\n";
        oss << "Shake Effects: " << stats.activeShakes << " active\n";
        oss << "Memory Usage: " << formatBytes(stats.memoryUsage) << "\n";
        oss << "Update Time: " << std::fixed << std::setprecision(2)
            << stats.averageUpdateTime << "ms (avg), "
            << stats.peakUpdateTime << "ms (peak)\n";
        oss << "Total Updates: " << stats.totalUpdates << "\n";

        return oss.str();
    }

    std::string CameraDebugger::inspectCamera(const CameraID cameraId) const {
        if (!cameraManager_) return "No camera manager";

        const auto camera = cameraManager_->getCamera(cameraId);
        if (!camera) return "Camera not found";

        std::ostringstream oss;
        oss << "=== Camera " << cameraId << " ===\n";
        oss << "Name: " << camera->getName() << "\n";
        oss << "Type: " << cameraTypeToString(camera->getType()) << "\n";
        oss << "Mode: " << cameraModeToString(camera->getMode()) << "\n";
        oss << "Active: " << (camera->isActive() ? "Yes" : "No") << "\n";
        oss << "Enabled: " << (camera->isEnabled() ? "Yes" : "No") << "\n";

        const Vector3 pos = camera->getPosition();
        oss << "Position: (" << pos.x << ", " << pos.y << ", " << pos.z << ")\n";

        // Type-specific information
        if (camera->getType() == CameraType::CAMERA_2D) {
            const auto* cam2D = static_cast<Camera2D*>(camera);
            oss << "Zoom: " << cam2D->getZoom() << "\n";
            oss << "Rotation: " << cam2D->getRotation() << "°\n";
        } else {
            const auto* cam3D = static_cast<Camera3D*>(camera);
            oss << "FOV: " << cam3D->getFOV() << "°\n";
            oss << "Projection: " << (cam3D->isPerspective() ? "Perspective" : "Orthographic") << "\n";

            const Vector3 target = cam3D->getTarget();
            oss << "Target: (" << target.x << ", " << target.y << ", " << target.z << ")\n";
            oss << "Yaw: " << cam3D->getYaw() << "°\n";
            oss << "Pitch: " << cam3D->getPitch() << "°\n";
        }

        return oss.str();
    }

    std::string CameraDebugger::getAllCameraSummaries() const {
        if (!cameraManager_) return "No camera manager";

        std::ostringstream oss;
        oss << "=== Camera Summary ===\n";

        for (const auto cameraIds = cameraManager_->getRegistry().getAllCameraIds(); CameraID id : cameraIds) {
            if (const auto* camera = cameraManager_->getCamera(id)) {
                oss << "[" << id << "] " << camera->getName()
                    << " (" << cameraTypeToString(camera->getType()) << ")";

                if (camera->isActive()) oss << " [ACTIVE]";
                if (!camera->isEnabled()) oss << " [DISABLED]";

                oss << "\n";
            }
        }

        return oss.str();
    }

    void CameraDebugger::startPathRecording(const CameraID cameraId) {
        recordingCameraId_ = cameraId;
        pathHistory_.clear();
        isRecording_ = true;
    }

    void CameraDebugger::trackUpdate(const float deltaTime) {
        updateCount_++;

        const float updateTimeMs = deltaTime * 1000.0f;
        updateTimeSum_ += updateTimeMs;

        if (updateTimeMs > peakUpdateTime_) {
            peakUpdateTime_ = updateTimeMs;
        }

        // Record camera path if enabled
        if (isRecording_ && cameraManager_) {
            if (const auto* camera = cameraManager_->getCamera(recordingCameraId_)) {
                pathHistory_.push_back(camera->getPosition());

                // Limit history size
                if (pathHistory_.size() > maxPathHistory_) {
                    pathHistory_.erase(pathHistory_.begin());
                }
            }
        }
    }

    void CameraDebugger::resetStats() {
        updateCount_ = 0;
        updateTimeSum_ = 0.0f;
        peakUpdateTime_ = 0.0f;
    }

    std::string CameraDebugger::validateSystem() const {
        if (!cameraManager_) return "No camera manager";

        std::ostringstream oss;
        oss << "=== Camera System Validation ===\n";

        bool hasIssues = false;

        // Check initialization
        if (!cameraManager_->isInitialized()) {
            oss << "[ERROR] Camera manager not initialized\n";
            hasIssues = true;
        }

        // Check active camera
        if (const CameraID activeId = cameraManager_->getActiveCameraId(); activeId != INVALID_CAMERA_ID) {
            if (!cameraManager_->getCamera(activeId)) {
                oss << "[ERROR] Active camera ID " << activeId << " does not exist\n";
                hasIssues = true;
            }
        }

        // Validate each camera
        for (const auto cameraIds = cameraManager_->getRegistry().getAllCameraIds(); const CameraID id : cameraIds) {
            if (const auto* camera = cameraManager_->getCamera(id)) {
                if (!camera->validate()) {
                    oss << "[WARNING] Camera " << id << " (" << camera->getName()
                        << ") failed validation\n";
                    hasIssues = true;
                }
            }
        }

        // Check configuration
        if (!cameraManager_->getConfig().validate()) {
            oss << "[ERROR] Invalid camera manager configuration\n";
            hasIssues = true;
        }

        if (!hasIssues) {
            oss << "All validation checks passed\n";
        }

        return oss.str();
    }

    std::vector<Vector3> CameraDebugger::getFrustumVertices(CameraID cameraId, const Viewport& viewport) const {
            std::vector<Vector3> vertices;

            if (!cameraManager_) return vertices;

            BaseCamera* camera = cameraManager_->getCamera(cameraId);
            if (!camera || camera->getType() == CameraType::CAMERA_2D) {
                return vertices; // No frustum for 2D cameras
            }

            const Camera3D* cam3D = static_cast<Camera3D*>(camera);

            // Calculate frustum corners
            float aspect = viewport.getAspectRatio();
            auto [nearPlane, farPlane] = cam3D->getClippingPlanes();

            if (cam3D->isPerspective()) {
                float fovRad = cam3D->getFOV() * math::constants::DEG_TO_RAD;
                float tanHalfFov = std::tan(fovRad * 0.5f);

                float nearHeight = 2.0f * tanHalfFov * nearPlane;
                float nearWidth = nearHeight * aspect;
                float farHeight = 2.0f * tanHalfFov * farPlane;
                float farWidth = farHeight * aspect;

                Vector3 pos = cam3D->getPosition();
                Vector3 forward = cam3D->getForward();
                Vector3 right = cam3D->getRight();
                Vector3 up = cam3D->getUp();

                // Near plane corners
                Vector3 nearCenter = pos + forward * nearPlane;
                vertices.push_back(nearCenter + up * (nearHeight * 0.5f) - right * (nearWidth * 0.5f));
                vertices.push_back(nearCenter + up * (nearHeight * 0.5f) + right * (nearWidth * 0.5f));
                vertices.push_back(nearCenter - up * (nearHeight * 0.5f) + right * (nearWidth * 0.5f));
                vertices.push_back(nearCenter - up * (nearHeight * 0.5f) - right * (nearWidth * 0.5f));

                // Far plane corners
                Vector3 farCenter = pos + forward * farPlane;
                vertices.push_back(farCenter + up * (farHeight * 0.5f) - right * (farWidth * 0.5f));
                vertices.push_back(farCenter + up * (farHeight * 0.5f) + right * (farWidth * 0.5f));
                vertices.push_back(farCenter - up * (farHeight * 0.5f) + right * (farWidth * 0.5f));
                vertices.push_back(farCenter - up * (farHeight * 0.5f) - right * (farWidth * 0.5f));
            } else {
                // Orthographic frustum
                float size = cam3D->getOrthographicSize();
                float width = size * aspect;
                float height = size;

                Vector3 pos = cam3D->getPosition();
                Vector3 forward = cam3D->getForward();
                Vector3 right = cam3D->getRight();
                Vector3 up = cam3D->getUp();

                // Near plane corners
                Vector3 nearCenter = pos + forward * nearPlane;
                vertices.push_back(nearCenter + up * height - right * width);
                vertices.push_back(nearCenter + up * height + right * width);
                vertices.push_back(nearCenter - up * height + right * width);
                vertices.push_back(nearCenter - up * height - right * width);

                // Far plane corners
                Vector3 farCenter = pos + forward * farPlane;
                vertices.push_back(farCenter + up * height - right * width);
                vertices.push_back(farCenter + up * height + right * width);
                vertices.push_back(farCenter - up * height + right * width);
                vertices.push_back(farCenter - up * height - right * width);
            }

            return vertices;
        }

    std::vector<std::pair<Vector3, Vector3>> CameraDebugger::getDebugLines(const DebugVisualizationOptions& options) const {
        std::vector<std::pair<Vector3, Vector3>> lines;

        if (!cameraManager_) return lines;

        const CameraID activeId = cameraManager_->getActiveCameraId();
        if (activeId == INVALID_CAMERA_ID) return lines;

        BaseCamera* camera = cameraManager_->getCamera(activeId);
        if (!camera) return lines;

        Vector3 pos = camera->getPosition();

        // Camera axes
        if (options.showAxes) {
            if (camera->getType() != CameraType::CAMERA_2D) {
                const float axisLength = 1.0f;
                const Camera3D* cam3D = static_cast<Camera3D*>(camera);
                const Vector3 forward = cam3D->getForward() * axisLength;
                const Vector3 right = cam3D->getRight() * axisLength;
                const Vector3 up = cam3D->getUp() * axisLength;

                lines.push_back({pos, pos + forward}); // Z (blue)
                lines.push_back({pos, pos + right});   // X (red)
                lines.push_back({pos, pos + up});      // Y (green)
            }
        }

        // Look-at target
        if (options.showTarget && camera->getType() != CameraType::CAMERA_2D) {
            const Camera3D* cam3D = static_cast<Camera3D*>(camera);
            lines.push_back({pos, cam3D->getTarget()});
        }

        // Path history
        if (options.showPath && pathHistory_.size() > 1) {
            for (size_t i = 1; i < pathHistory_.size(); ++i) {
                lines.push_back({pathHistory_[i - 1], pathHistory_[i]});
            }
        }

        return lines;
    }

    std::string CameraDebugger::formatBytes(const std::size_t bytes) {
        const char* units[] = {"B", "KB", "MB", "GB"};
        int unitIndex = 0;
        double size = static_cast<double>(bytes);

        while (size >= 1024.0 && unitIndex < 3) {
            size /= 1024.0;
            unitIndex++;
        }

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << size << " " << units[unitIndex];
        return oss.str();
    }
} // namespace engine::camera