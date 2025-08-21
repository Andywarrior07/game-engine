//
// Created by Andres Guerrero on 20-08-25.
//

#pragma once

#include "../core/ResourceTypes.h"
#include <atomic>
#include <vector>
#include <functional>
#include <memory>
#include <future>
#include <chrono>

namespace engine::resources {

    // Forward declarations
    class Resource;
    class ResourceManager;

    // Resource task types
    enum class ResourceTaskType : std::uint8_t {
        LOAD,
        UNLOAD,
        PROCESS,
        COMPRESS,
        DECOMPRESS,
        VALIDATE,
        SERIALIZE,
        DESERIALIZE,
        STREAM_IN,
        STREAM_OUT,
        GENERATE_MIPS,
        OPTIMIZE
    };

    // Resource task status
    enum class ResourceTaskStatus : std::uint8_t {
        PENDING,
        RUNNING,
        COMPLETED,
        FAILED,
        CANCELLED
    };

    /**
     * @brief Base class for resource-related tasks
     *
     * This abstract class represents a unit of work related to resource management.
     * Tasks can have dependencies and are executed by the ResourceTaskScheduler.
     */
    class ResourceTask {
    public:
        using TaskID = std::uint64_t;
        using CompletionCallback = std::function<void(ResourceTask*)>;
        using ProgressCallback = std::function<void(float)>;

        /**
         * @brief Constructor
         * @param id Unique task identifier
         * @param type Task type
         * @param priority Task priority
         */
        ResourceTask(TaskID id, ResourceTaskType type, ResourcePriority priority);

        /**
         * @brief Virtual destructor
         */
        virtual ~ResourceTask() = default;

        /**
         * @brief Execute the task
         * @return true if successful, false otherwise
         */
        virtual bool execute() = 0;

        /**
         * @brief Cancel the task
         */
        virtual void cancel();

        /**
         * @brief Get estimated execution time
         * @return Estimated time in milliseconds
         */
        virtual std::chrono::milliseconds getEstimatedTime() const { return std::chrono::milliseconds(0); }

        /**
         * @brief Get task progress (0.0 to 1.0)
         * @return Current progress
         */
        virtual float getProgress() const { return progress_.load(std::memory_order_acquire); }

        // Getters
        TaskID getId() const { return id_; }
        ResourceTaskType getType() const { return type_; }
        ResourcePriority getPriority() const { return priority_; }
        ResourceTaskStatus getStatus() const { return status_.load(std::memory_order_acquire); }
        const std::string& getName() const { return name_; }
        CompletionCallback getCompletionCallback() const { return completionCallback_; }

        // Setters
        void setName(const std::string& name) { name_ = name; }
        void setCompletionCallback(CompletionCallback callback) { completionCallback_ = callback; }
        void setProgressCallback(ProgressCallback callback) { progressCallback_ = callback; }

        // Dependencies
        void addDependency(TaskID taskId);
        void removeDependency(TaskID taskId);
        const std::vector<TaskID>& getDependencies() const { return dependencies_; }
        bool hasDependencies() const { return !dependencies_.empty(); }

        // Error handling
        const std::string& getErrorMessage() const { return errorMessage_; }
        bool hasError() const { return !errorMessage_.empty(); }

        // Helper methods for derived classes
        void setStatus(ResourceTaskStatus status);
        void setError(const std::string& error);

    protected:
        // Task identification
        TaskID id_;
        ResourceTaskType type_;
        ResourcePriority priority_;
        std::string name_;

        // Task state
        std::atomic<ResourceTaskStatus> status_;
        std::atomic<float> progress_{0.0f};

        // Callbacks
        CompletionCallback completionCallback_;
        ProgressCallback progressCallback_;

        // Dependencies
        std::vector<TaskID> dependencies_;

        // Error handling
        std::string errorMessage_;

        // Helper methods for derived classes
        void updateProgress(float progress);
    };

    // ========================================================================
    // CONCRETE TASK IMPLEMENTATIONS
    // ========================================================================

    /**
     * @brief Task for loading a resource from disk
     */
    class LoadResourceTask : public ResourceTask {
    public:
        LoadResourceTask(TaskID id,
                        const std::string& path,
                        ResourceType resourceType,
                        ResourcePriority priority = ResourcePriority::NORMAL);

        bool execute() override;
        std::chrono::milliseconds getEstimatedTime() const override;

        void setResourceManager(ResourceManager* manager) { resourceManager_ = manager; }
        void setTargetResource(std::shared_ptr<Resource> resource) { targetResource_ = resource; }

        const std::string& getPath() const { return path_; }
        ResourceType getResourceType() const { return resourceType_; }
        std::shared_ptr<Resource> getLoadedResource() const { return targetResource_; }

    private:
        std::string path_;
        ResourceType resourceType_;
        ResourceManager* resourceManager_ = nullptr;
        std::shared_ptr<Resource> targetResource_;
        std::unique_ptr<std::uint8_t[]> loadedData_;
        std::size_t dataSize_ = 0;
    };

    /**
     * @brief Task for unloading a resource
     */
    class UnloadResourceTask : public ResourceTask {
    public:
        UnloadResourceTask(TaskID id,
                          const std::shared_ptr<Resource>& resource,
                          ResourcePriority priority = ResourcePriority::LOW);

        bool execute() override;
        std::chrono::milliseconds getEstimatedTime() const override;

    private:
        std::shared_ptr<Resource> resource_;
    };

    /**
     * @brief Task for compressing resource data
     */
    class CompressResourceTask : public ResourceTask {
    public:
        CompressResourceTask(TaskID id,
                           const std::uint8_t* data,
                           std::size_t size,
                           CompressionType compressionType,
                           ResourcePriority priority = ResourcePriority::LOW);

        bool execute() override;
        std::chrono::milliseconds getEstimatedTime() const override;

        const std::uint8_t* getCompressedData() const { return compressedData_.get(); }
        std::size_t getCompressedSize() const { return compressedSize_; }

    private:
        const std::uint8_t* sourceData_;
        std::size_t sourceSize_;
        CompressionType compressionType_;
        std::unique_ptr<std::uint8_t[]> compressedData_;
        std::size_t compressedSize_ = 0;
    };

    /**
     * @brief Task for decompressing resource data
     */
    class DecompressResourceTask : public ResourceTask {
    public:
        DecompressResourceTask(TaskID id,
                             const std::uint8_t* compressedData,
                             std::size_t compressedSize,
                             std::size_t uncompressedSize,
                             CompressionType compressionType,
                             ResourcePriority priority = ResourcePriority::NORMAL);

        bool execute() override;
        std::chrono::milliseconds getEstimatedTime() const override;

        const std::uint8_t* getDecompressedData() const { return decompressedData_.get(); }
        std::size_t getDecompressedSize() const { return decompressedSize_; }

    private:
        const std::uint8_t* compressedData_;
        std::size_t compressedSize_;
        std::size_t expectedUncompressedSize_;
        CompressionType compressionType_;
        std::unique_ptr<std::uint8_t[]> decompressedData_;
        std::size_t decompressedSize_ = 0;
    };

    /**
     * @brief Task for validating resource data
     */
    class ValidateResourceTask : public ResourceTask {
    public:
        ValidateResourceTask(TaskID id,
                           const std::shared_ptr<Resource>& resource,
                           ResourcePriority priority = ResourcePriority::LOW);

        bool execute() override;
        std::chrono::milliseconds getEstimatedTime() const override;

        bool isValid() const { return isValid_; }
        const std::vector<std::string>& getValidationErrors() const { return validationErrors_; }

    private:
        std::shared_ptr<Resource> resource_;
        bool isValid_ = false;
        std::vector<std::string> validationErrors_;
    };

    /**
     * @brief Task for serializing a resource
     */
    class SerializeResourceTask : public ResourceTask {
    public:
        SerializeResourceTask(TaskID id,
                            const std::shared_ptr<Resource>& resource,
                            ResourcePriority priority = ResourcePriority::LOW);

        bool execute() override;
        std::chrono::milliseconds getEstimatedTime() const override;

        const std::vector<std::uint8_t>& getSerializedData() const { return serializedData_; }

    private:
        std::shared_ptr<Resource> resource_;
        std::vector<std::uint8_t> serializedData_;
    };

    /**
     * @brief Task for deserializing resource data
     */
    class DeserializeResourceTask : public ResourceTask {
    public:
        DeserializeResourceTask(TaskID id,
                              const std::shared_ptr<Resource>& resource,
                              const std::uint8_t* data,
                              std::size_t size,
                              ResourcePriority priority = ResourcePriority::NORMAL);

        bool execute() override;
        std::chrono::milliseconds getEstimatedTime() const override;

    private:
        std::shared_ptr<Resource> resource_;
        const std::uint8_t* data_;
        std::size_t dataSize_;
    };

    /**
     * @brief Batch task that executes multiple tasks
     */
    class BatchResourceTask : public ResourceTask {
    public:
        BatchResourceTask(TaskID id,
                         ResourcePriority priority = ResourcePriority::NORMAL);

        void addTask(std::unique_ptr<ResourceTask> task);
        bool execute() override;
        std::chrono::milliseconds getEstimatedTime() const override;

        std::size_t getTaskCount() const { return tasks_.size(); }
        const std::vector<std::unique_ptr<ResourceTask>>& getTasks() const { return tasks_; }

    private:
        std::vector<std::unique_ptr<ResourceTask>> tasks_;
        bool stopOnError_ = true;
    };

} // namespace engine::resources
