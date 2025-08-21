//
// Created by Andres Guerrero on 20-08-25.
//

#pragma once
#include <string>

#include "../core/Resource.h"

namespace engine::resources {
    // Streaming state for resources
    enum class StreamingState : std::uint8_t {
        NOT_STREAMING = 0,
        STREAMING_IN, // Loading from disk
        STREAMING_OUT, // Unloading to free memory
        RESIDENT, // Fully in memory
        PARTIALLY_LOADED // Some LODs/mips loaded
    };

    // Streaming priority
    enum class StreamingPriority : std::uint8_t {
        BACKGROUND = 0, // Load when idle
        LOW, // Load eventually
        NORMAL, // Standard streaming
        HIGH, // Load soon
        IMMEDIATE // Load right now
    };

    // LOD (Level of Detail) information
    struct LODInfo {
        std::uint32_t level = 0; // LOD level (0 = highest detail)
        ResourceSize memorySize = 0; // Memory required for this LOD
        float distanceThreshold = 0.0f; // Distance at which this LOD should be used
        bool isLoaded = false; // Whether this LOD is currently loaded
    };

    /**
     * @brief Base class for resources that support streaming and LOD management
     *
     * This class provides the interface for resources that can be partially loaded
     * into memory based on various factors like distance, importance, or memory pressure.
     * Examples include textures (mip levels), meshes (geometry LODs), and audio (quality levels).
     */
    class StreamableResource : public Resource {
    public:
        /**
         * @brief Constructor
         * @param id Unique resource identifier
         * @param name Resource name
         * @param type Resource type
         */
        StreamableResource(ResourceID id, const std::string& name, ResourceType type);

        /**
         * @brief Virtual destructor
         */
        ~StreamableResource() override = default;

        StreamableResource(StreamableResource&&) = default;

        // ====================================================================
        // STREAMING INTERFACE (Pure Virtual - Must be implemented)
        // ====================================================================

        /**
         * @brief Stream in a specific LOD level
         * @param lodLevel Level to stream in (0 = highest detail)
         * @return true if successful
         */
        virtual bool streamIn(std::uint32_t lodLevel) = 0;

        /**
         * @brief Stream out LOD levels to free memory
         * @param lodLevel Minimum level to keep (UINT32_MAX = stream out everything)
         * @return true if successful
         */
        virtual bool streamOut(std::uint32_t lodLevel) = 0;

        /**
         * @brief Get currently loaded LOD level
         * @return Current LOD level (UINT32_MAX if nothing loaded)
         */
        virtual std::uint32_t getStreamedLODLevel() const = 0;

        /**
         * @brief Get maximum LOD level available
         * @return Maximum LOD level (0 means only one level)
         */
        virtual std::uint32_t getMaxLODLevel() const = 0;

        /**
         * @brief Get memory usage of currently streamed data
         * @return Memory size in bytes
         */
        virtual ResourceSize getStreamedMemoryUsage() const = 0;

        // ====================================================================
        // OPTIONAL STREAMING INTERFACE (Can be overridden)
        // ====================================================================

        /**
         * @brief Check if a specific LOD level is loaded
         * @param lodLevel LOD level to check
         * @return true if loaded
         */
        virtual bool isLODLoaded(std::uint32_t lodLevel) const;

        /**
         * @brief Get information about a specific LOD level
         * @param lodLevel LOD level
         * @return LOD information
         */
        virtual LODInfo getLODInfo(std::uint32_t lodLevel) const;

        /**
         * @brief Get total memory required for all LODs
         * @return Total memory size in bytes
         */
        virtual ResourceSize getTotalMemoryRequired() const;

        /**
         * @brief Prefetch a LOD level (non-blocking hint)
         * @param lodLevel LOD level to prefetch
         */
        virtual void prefetchLOD(std::uint32_t lodLevel);

        /**
         * @brief Check if resource supports asynchronous streaming
         * @return true if async streaming is supported
         */
        virtual bool supportsAsyncStreaming() const { return true; }

        // ====================================================================
        // STREAMING STATE MANAGEMENT
        // ====================================================================

        /**
         * @brief Get current streaming state
         * @return Current streaming state
         */
        StreamingState getStreamingState() const;

        /**
         * @brief Set streaming state
         * @param state New streaming state
         */
        void setStreamingState(StreamingState state);

        /**
         * @brief Get streaming priority
         * @return Current streaming priority
         */
        StreamingPriority getStreamingPriority() const;

        /**
         * @brief Set streaming priority
         * @param priority New streaming priority
         */
        void setStreamingPriority(StreamingPriority priority);

        /**
         * @brief Get importance factor for streaming decisions
         * @return Importance value (0.0 = not important, 1.0 = very important)
         */
        float getImportance() const;

        /**
         * @brief Set importance factor
         * @param importance Importance value (0.0 to 1.0)
         */
        void setImportance(float importance);

        /**
         * @brief Get last access time
         * @return Time point of last access
         */
        std::chrono::steady_clock::time_point getLastAccessTime() const;

        /**
         * @brief Update last access time to now
         */
        void updateLastAccessTime() const;

        // ====================================================================
        // STREAMING HINTS
        // ====================================================================

        /**
         * @brief Suggest a LOD level based on a quality metric
         * @param qualityMetric Quality value (0.0 = lowest, 1.0 = highest)
         * @return Suggested LOD level
         */
        virtual std::uint32_t suggestLODLevel(float qualityMetric) const;

        /**
         * @brief Check if resource should be streamed out
         * @param memoryPressure Memory pressure level (0.0 = no pressure, 1.0 = critical)
         * @return true if should be considered for streaming out
         */
        virtual bool shouldStreamOut(float memoryPressure) const;

        /**
         * @brief Get streaming cost estimate
         * @param lodLevel LOD level to stream
         * @return Estimated time in milliseconds
         */
        virtual float getStreamingCost(std::uint32_t lodLevel) const;

    protected:
        // Streaming state
        std::atomic<StreamingState> streamingState_;
        std::atomic<StreamingPriority> streamingPriority_;
        std::atomic<std::uint32_t> currentLOD_;

        // Importance and access tracking
        std::atomic<float> importance_;
        mutable std::atomic<std::chrono::steady_clock::time_point> lastAccessTime_;

        // Streaming statistics
        std::atomic<std::uint32_t> streamInCount_;
        std::atomic<std::uint32_t> streamOutCount_;
        std::atomic<std::uint64_t> totalStreamTime_; // Microseconds

        // Helper methods for derived classes
        void recordStreamingOperation(bool isStreamIn, std::chrono::microseconds duration);
        bool validateLODLevel(std::uint32_t lodLevel) const;
    };
}
