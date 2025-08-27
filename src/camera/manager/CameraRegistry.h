/**
 * @file CameraRegistry.h
 * @brief Camera registry for managing camera instances
 * @author Andrés Guerrero
 * @date 26-08-2025
 */

#pragma once

#include "../core/CameraTypes.h"
#include "../cameras/BaseCamera.h"

#include <unordered_map>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <ranges>

namespace engine::camera {
    /**
     * @brief Camera registry for efficient camera management
     *
     * Manages camera instances with fast lookup by ID or name,
     * handles camera lifecycle, and provides iteration capabilities.
     */
    class CameraRegistry {
    public:
        /**
         * @brief Camera entry in registry
         */
        struct CameraEntry {
            std::unique_ptr<BaseCamera> camera; ///< Camera instance
            std::string name; ///< Camera name
            CameraType type; ///< Camera type
            std::size_t accessCount = 0; ///< Access counter for statistics

            /**
             * @brief Constructor
             * @param cam Camera instance
             */
            explicit CameraEntry(std::unique_ptr<BaseCamera> cam)
                : camera(std::move(cam))
                  , name(camera->getName())
                  , type(camera->getType()) {
            }
        };

        /**
         * @brief Default constructor
         */
        CameraRegistry() = default;

        /**
         * @brief Destructor
         */
        ~CameraRegistry() = default;

        // Non-copyable but moveable
        CameraRegistry(const CameraRegistry&) = delete;
        CameraRegistry& operator=(const CameraRegistry&) = delete;
        CameraRegistry(CameraRegistry&&) = default;
        CameraRegistry& operator=(CameraRegistry&&) = default;

        // ========================================================================
        // CAMERA MANAGEMENT
        // ========================================================================

        /**
         * @brief Register a camera
         * @param camera Camera to register
         * @return Camera ID if successful, INVALID_CAMERA_ID if failed
         */
        CameraID registerCamera(std::unique_ptr<BaseCamera> camera);

        /**
         * @brief Unregister a camera
         * @param id Camera ID to unregister
         * @return true if camera was unregistered
         */
        bool unregisterCamera(CameraID id);

        /**
         * @brief Get camera by ID
         * @param id Camera ID
         * @return Pointer to camera, or nullptr if not found
         */
        BaseCamera* getCamera(CameraID id);

        /**
         * @brief Get camera by ID (const version)
         * @param id Camera ID
         * @return Const pointer to camera, or nullptr if not found
         */
        const BaseCamera* getCamera(CameraID id) const;

        /**
         * @brief Get camera by name
         * @param name Camera name
         * @return Camera ID if found, INVALID_CAMERA_ID if not found
         */
        CameraID getCameraByName(const std::string& name) const;

        /**
         * @brief Check if camera exists
         * @param id Camera ID
         * @return true if camera exists
         */
        bool hasCamera(CameraID id) const;

        /**
         * @brief Check if camera name exists
         * @param name Camera name
         * @return true if name exists
         */
        bool hasCamera(const std::string& name) const;

        // ========================================================================
        // ITERATION AND QUERIES
        // ========================================================================

        /**
         * @brief Get number of cameras
         * @return Total camera count
         */
        std::size_t getCount() const;

        /**
         * @brief Get all camera IDs
         * @return Vector of camera IDs
         */
        std::vector<CameraID> getAllCameraIds() const;

        /**
         * @brief Get all camera names
         * @return Vector of camera names
         */
        std::vector<std::string> getAllCameraNames() const;

        /**
         * @brief Get cameras by type
         * @param type Camera type to filter
         * @return Vector of camera IDs of given type
         */
        std::vector<CameraID> getCamerasByType(CameraType type) const;

        /**
         * @brief Apply function to all cameras
         * @param func Function to apply to each camera
         */
        template <typename Func>
        void forEachCamera(Func func) {
            std::lock_guard lock(mutex_);

            for (const auto& entry : cameras_ | std::views::values) {
                func(entry->camera.get());
            }
        }

        /**
         * @brief Apply function to all cameras (const version)
         * @param func Function to apply to each camera
         */
        template <typename Func>
        void forEachCamera(Func func) const {
            std::lock_guard lock(mutex_);

            for (const auto& entry : cameras_ | std::views::values) {
                func(entry->camera.get());
            }
        }

        /**
         * @brief Clear all cameras
         */
        void clear();

        // ========================================================================
        // STATISTICS
        // ========================================================================

        /**
         * @brief Get access count for a camera
         * @param id Camera ID
         * @return Number of times camera was accessed
         */
        std::size_t getAccessCount(CameraID id) const;

        /**
         * @brief Reset access statistics
         */
        void resetStatistics();

    private:
        mutable std::mutex mutex_; ///< Mutex for thread safety
        std::unordered_map<CameraID, std::unique_ptr<CameraEntry>> cameras_; ///< Camera storage
        std::unordered_map<std::string, CameraID> nameToId_; ///< Name to ID mapping
        std::unordered_map<CameraType, std::vector<CameraID>> typeIndex_; ///< Type-based index

        /**
         * @brief Generate unique name for duplicate names
         * @param baseName Base name to make unique
         * @return Unique name
         */
        std::string generateUniqueName(const std::string& baseName) const;
    };
} // namespace engine::camera
