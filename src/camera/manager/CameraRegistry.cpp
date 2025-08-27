/**
 * @file CameraRegistry.cpp
 * @brief Camera registry for managing camera instances
 * @author Andrés Guerrero
 * @date 26-08-2025
 */

#include "CameraRegistry.h"

namespace engine::camera {
    CameraID CameraRegistry::   registerCamera(std::unique_ptr<BaseCamera> camera) {
        if (!camera) return INVALID_CAMERA_ID;

        std::lock_guard<std::mutex> lock(mutex_);

        CameraID id = camera->getId();
        std::string name = camera->getName();

        // Check for duplicate ID
        if (cameras_.contains(id)) {
            return INVALID_CAMERA_ID;
        }

        // Check for duplicate name
        if (nameToId_.contains(name)) {
            // Generate unique name
            name = generateUniqueName(name);
            camera->setName(name);
        }

        // Register camera
        cameras_.emplace(id, std::make_unique<CameraEntry>(std::move(camera)));
        nameToId_[name] = id;

        // Update type index
        typeIndex_[cameras_[id]->type].push_back(id);

        return id;
    }

    bool CameraRegistry::unregisterCamera(const CameraID id) {
        std::lock_guard lock(mutex_);

        const auto it = cameras_.find(id);

        if (it == cameras_.end()) {
            return false;
        }

        // Remove from name map
        nameToId_.erase(it->second->name);

        // Remove from type index
        auto& typeVec = typeIndex_[it->second->type];
        std::erase(typeVec, id);

        // Remove camera
        cameras_.erase(it);

        return true;
    }

    BaseCamera* CameraRegistry::getCamera(const CameraID id) {
        std::lock_guard lock(mutex_);

        const auto it = cameras_.find(id);

        if (it == cameras_.end()) {
            return nullptr;
        }

        it->second->accessCount++;
        return it->second->camera.get();
    }

    const BaseCamera* CameraRegistry::getCamera(const CameraID id) const {
        std::lock_guard lock(mutex_);

        const auto it = cameras_.find(id);
        return it != cameras_.end() ? it->second->camera.get() : nullptr;
    }

    CameraID CameraRegistry::getCameraByName(const std::string& name) const {
        std::lock_guard lock(mutex_);

        const auto it = nameToId_.find(name);
        return it != nameToId_.end() ? it->second : INVALID_CAMERA_ID;
    }

    bool CameraRegistry::hasCamera(const CameraID id) const {
        std::lock_guard lock(mutex_);
        return cameras_.contains(id);
    }

    bool CameraRegistry::hasCamera(const std::string& name) const {
        std::lock_guard lock(mutex_);
        return nameToId_.contains(name);
    }

    std::size_t CameraRegistry::getCount() const {
        std::lock_guard lock(mutex_);
        return cameras_.size();
    }

    std::vector<CameraID> CameraRegistry::getAllCameraIds() const {
        std::lock_guard lock(mutex_);

        std::vector<CameraID> ids;
        ids.reserve(cameras_.size());

        for (const auto& id : cameras_ | std::views::keys) {
            ids.push_back(id);
        }

        return ids;
    }

    std::vector<std::string> CameraRegistry::getAllCameraNames() const {
        std::lock_guard lock(mutex_);

        std::vector<std::string> names;
        names.reserve(cameras_.size());

        for (const auto& entry : cameras_ | std::views::values) {
            names.push_back(entry->name);
        }

        return names;
    }

    std::vector<CameraID> CameraRegistry::getCamerasByType(const CameraType type) const {
        std::lock_guard lock(mutex_);

        const auto it = typeIndex_.find(type);

        if (it == typeIndex_.end()) {
            return {};
        }

        return it->second;
    }

    void CameraRegistry::clear() {
        std::lock_guard lock(mutex_);
        cameras_.clear();
        nameToId_.clear();
        typeIndex_.clear();
    }

    std::size_t CameraRegistry::getAccessCount(const CameraID id) const {
        std::lock_guard lock(mutex_);

        const auto it = cameras_.find(id);
        return it != cameras_.end() ? it->second->accessCount : 0;
    }

    void CameraRegistry::resetStatistics() {
        std::lock_guard lock(mutex_);

        for (const auto& entry : cameras_ | std::views::values) {
            entry->accessCount = 0;
        }
    }

    std::string CameraRegistry::generateUniqueName(const std::string& baseName) const {
        int suffix = 1;
        std::string uniqueName;

        do {
            uniqueName = baseName + "_" + std::to_string(suffix++);
        }
        while (nameToId_.contains(uniqueName));

        return uniqueName;
    }
} // namespace engine::camera
