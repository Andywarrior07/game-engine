//
// Created by Andres Guerrero on 20-08-25.
//

#include "ResourceTask.h"
#include "../core/Resource.h"
#include <algorithm>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace engine::resources {
// ====================================================================
// BASE RESOURCE TASK
// ====================================================================

ResourceTask::ResourceTask(const TaskID id, ResourceTaskType type,
                           const ResourcePriority priority)
    : id_(id), type_(type), priority_(priority),
      status_(ResourceTaskStatus::PENDING) {
  // Generate the default name based on type
  const char *typeNames[] = {"Load",      "Unload",       "Process",
                             "Compress",  "Decompress",   "Validate",
                             "Serialize", "Deserialize",  "StreamIn",
                             "StreamOut", "GenerateMips", "Optimize"};

  if (static_cast<std::size_t>(type) < std::size(typeNames)) {
    name_ = std::string(typeNames[static_cast<std::size_t>(type)]) + "_" +
            std::to_string(id);
  } else {
    name_ = "Task_" + std::to_string(id);
  }
}

void ResourceTask::cancel() {
  // Can only cancel if not already running or completed
  if (const auto currentStatus = status_.load(std::memory_order_acquire);
      currentStatus == ResourceTaskStatus::PENDING) {
    setStatus(ResourceTaskStatus::CANCELLED);
  }
}

void ResourceTask::addDependency(const TaskID taskId) {
  // Check for duplicate
  if (std::ranges::find(dependencies_, taskId) == dependencies_.end()) {
    dependencies_.push_back(taskId);
  }
}

void ResourceTask::removeDependency(const TaskID taskId) {
  std::erase(dependencies_, taskId);
}

void ResourceTask::setStatus(const ResourceTaskStatus status) {
  status_.store(status, std::memory_order_release);

  // Trigger completion callback
  if (status == ResourceTaskStatus::COMPLETED ||
      status == ResourceTaskStatus::FAILED ||
      status == ResourceTaskStatus::CANCELLED) {
    if (completionCallback_) {
      completionCallback_(this);
    }
  }
}

void ResourceTask::updateProgress(float progress) {
  progress = std::clamp(progress, 0.0f, 1.0f);
  progress_.store(progress, std::memory_order_release);

  if (progressCallback_) {
    progressCallback_(progress);
  }
}

void ResourceTask::setError(const std::string &error) {
  errorMessage_ = error;
  if (!error.empty()) {
    std::cerr << "Task error [" << name_ << "]: " << error << std::endl;
  }
}

// ====================================================================
// LOAD RESOURCE TASK
// ====================================================================

LoadResourceTask::LoadResourceTask(const TaskID id, const std::string &path,
                                   const ResourceType resourceType,
                                   const ResourcePriority priority)
    : ResourceTask(id, ResourceTaskType::LOAD, priority), path_(path),
      resourceType_(resourceType) {
  setName("Load_" + path);
}

bool LoadResourceTask::execute() {
  setStatus(ResourceTaskStatus::RUNNING);
  updateProgress(0.0f);

  try {
    // Check if a file exists
    if (!std::filesystem::exists(path_)) {
      setError("File not found: " + path_);
      setStatus(ResourceTaskStatus::FAILED);
      return false;
    }

    updateProgress(0.1f);

    // Get file size
    const std::size_t fileSize = std::filesystem::file_size(path_);
    if (fileSize == 0) {
      setError("File is empty: " + path_);
      setStatus(ResourceTaskStatus::FAILED);
      return false;
    }

    updateProgress(0.2f);

    // Allocate memory for file data
    loadedData_ = std::make_unique<std::uint8_t[]>(fileSize);
    dataSize_ = fileSize;

    // Read file
    std::ifstream file(path_, std::ios::binary);
    if (!file.is_open()) {
      setError("Failed to open file: " + path_);
      setStatus(ResourceTaskStatus::FAILED);
      return false;
    }

    updateProgress(0.3f);

    file.read(reinterpret_cast<char *>(loadedData_.get()), fileSize);
    if (!file) {
      setError("Failed to read file: " + path_);
      setStatus(ResourceTaskStatus::FAILED);
      return false;
    }

    file.close();
    updateProgress(0.7f);

    // If we have a target resource, load the data into it
    if (targetResource_) {
      if (!targetResource_->load(loadedData_.get(), dataSize_)) {
        setError("Resource failed to load data");
        setStatus(ResourceTaskStatus::FAILED);
        return false;
      }
      targetResource_->setState(ResourceState::READY);
    }

    updateProgress(1.0f);
    setStatus(ResourceTaskStatus::COMPLETED);
    return true;
  } catch (const std::exception &e) {
    setError(std::string("Exception during load: ") + e.what());
    setStatus(ResourceTaskStatus::FAILED);
    return false;
  }
}

std::chrono::milliseconds LoadResourceTask::getEstimatedTime() const {
  // Estimate based on file size (assume 100 MB/s read speed)
  if (std::filesystem::exists(path_)) {
    const std::size_t fileSize = std::filesystem::file_size(path_);
    // Convert to milliseconds assuming 100 MB/s
    return std::chrono::milliseconds(fileSize / (100 * 1024));
  }
  return std::chrono::milliseconds(100); // Default estimate
}

// ====================================================================
// UNLOAD RESOURCE TASK
// ====================================================================

UnloadResourceTask::UnloadResourceTask(
    const TaskID id, const std::shared_ptr<Resource> &resource,
    const ResourcePriority priority)
    : ResourceTask(id, ResourceTaskType::UNLOAD, priority),
      resource_(resource) {
  if (resource) {
    setName("Unload_" + resource->getName());
  }
}

bool UnloadResourceTask::execute() {
  setStatus(ResourceTaskStatus::RUNNING);

  if (!resource_) {
    setError("No resource to unload");
    setStatus(ResourceTaskStatus::FAILED);
    return false;
  }

  try {
    updateProgress(0.5f);

    const bool result = resource_->unload();

    updateProgress(1.0f);

    if (result) {
      setStatus(ResourceTaskStatus::COMPLETED);
    } else {
      setError("Resource unload failed");
      setStatus(ResourceTaskStatus::FAILED);
    }

    return result;
  } catch (const std::exception &e) {
    setError(std::string("Exception during unload: ") + e.what());
    setStatus(ResourceTaskStatus::FAILED);
    return false;
  }
}

std::chrono::milliseconds UnloadResourceTask::getEstimatedTime() const {
  // Unloading is usually fast
  return std::chrono::milliseconds(10);
}

// ====================================================================
// COMPRESS RESOURCE TASK
// ====================================================================

CompressResourceTask::CompressResourceTask(
    const TaskID id, const std::uint8_t *data, const std::size_t size,
    const CompressionType compressionType, const ResourcePriority priority)
    : ResourceTask(id, ResourceTaskType::COMPRESS, priority), sourceData_(data),
      sourceSize_(size), compressionType_(compressionType) {
  setName("Compress_" + std::to_string(size) + "_bytes");
}

bool CompressResourceTask::execute() {
  setStatus(ResourceTaskStatus::RUNNING);

  if (!sourceData_ || sourceSize_ == 0) {
    setError("No data to compress");
    setStatus(ResourceTaskStatus::FAILED);
    return false;
  }

  try {
    updateProgress(0.1f);

    // Compression implementation would go here
    // For now; this is a placeholder that just copies the data
    // In production, integrate with compression libraries (zlib, lz4, zstd)

    switch (compressionType_) {
    case CompressionType::NONE:
      // Just copy the data
      compressedData_ = std::make_unique<std::uint8_t[]>(sourceSize_);
      std::memcpy(compressedData_.get(), sourceData_, sourceSize_);
      compressedSize_ = sourceSize_;
      break;

    case CompressionType::ZLIB:
    case CompressionType::LZ4:
    case CompressionType::ZSTD:
      // Placeholder - would use actual compression library
      setError("Compression type not implemented");
      setStatus(ResourceTaskStatus::FAILED);
      return false;

    default:
      setError("Unknown compression type");
      setStatus(ResourceTaskStatus::FAILED);
      return false;
    }

    updateProgress(1.0f);
    setStatus(ResourceTaskStatus::COMPLETED);
    return true;
  } catch (const std::exception &e) {
    setError(std::string("Exception during compression: ") + e.what());
    setStatus(ResourceTaskStatus::FAILED);
    return false;
  }
}

std::chrono::milliseconds CompressResourceTask::getEstimatedTime() const {
  // Estimate based on data size (assume 50 MB/s compression speed)
  return std::chrono::milliseconds(sourceSize_ / (50 * 1024));
}

// ====================================================================
// DECOMPRESS RESOURCE TASK
// ====================================================================

DecompressResourceTask::DecompressResourceTask(
    const TaskID id, const std::uint8_t *compressedData,
    const std::size_t compressedSize, const std::size_t uncompressedSize,
    const CompressionType compressionType, const ResourcePriority priority)
    : ResourceTask(id, ResourceTaskType::DECOMPRESS, priority),
      compressedData_(compressedData), compressedSize_(compressedSize),
      expectedUncompressedSize_(uncompressedSize),
      compressionType_(compressionType) {
  setName("Decompress_" + std::to_string(compressedSize) + "_bytes");
}

bool DecompressResourceTask::execute() {
  setStatus(ResourceTaskStatus::RUNNING);

  if (!compressedData_ || compressedSize_ == 0) {
    setError("No data to decompress");
    setStatus(ResourceTaskStatus::FAILED);
    return false;
  }

  try {
    updateProgress(0.1f);

    // Decompression implementation would go here
    switch (compressionType_) {
    case CompressionType::NONE:
      // Just copy the data
      decompressedData_ = std::make_unique<std::uint8_t[]>(compressedSize_);
      std::memcpy(decompressedData_.get(), compressedData_, compressedSize_);
      decompressedSize_ = compressedSize_;
      break;

    case CompressionType::ZLIB:
    case CompressionType::LZ4:
    case CompressionType::ZSTD:
      // Placeholder - would use actual decompression library
      setError("Decompression type not implemented");
      setStatus(ResourceTaskStatus::FAILED);
      return false;

    default:
      setError("Unknown compression type");
      setStatus(ResourceTaskStatus::FAILED);
      return false;
    }

    updateProgress(1.0f);
    setStatus(ResourceTaskStatus::COMPLETED);
    return true;
  } catch (const std::exception &e) {
    setError(std::string("Exception during decompression: ") + e.what());
    setStatus(ResourceTaskStatus::FAILED);
    return false;
  }
}

std::chrono::milliseconds DecompressResourceTask::getEstimatedTime() const {
  // Decompression is typically faster than compression
  return std::chrono::milliseconds(compressedSize_ / (100 * 1024));
}

// ====================================================================
// VALIDATE RESOURCE TASK
// ====================================================================

ValidateResourceTask::ValidateResourceTask(
    const TaskID id, const std::shared_ptr<Resource> &resource,
    const ResourcePriority priority)
    : ResourceTask(id, ResourceTaskType::VALIDATE, priority),
      resource_(resource) {
  if (resource) {
    setName("Validate_" + resource->getName());
  }
}

bool ValidateResourceTask::execute() {
  setStatus(ResourceTaskStatus::RUNNING);

  if (!resource_) {
    setError("No resource to validate");
    setStatus(ResourceTaskStatus::FAILED);
    return false;
  }

  try {
    updateProgress(0.5f);

    // Perform validation
    isValid_ = resource_->validate();

    if (!isValid_) {
      validationErrors_.emplace_back("Resource validation failed");
    }

    updateProgress(1.0f);
    setStatus(ResourceTaskStatus::COMPLETED);
    return true; // Task completed even if the resource is invalid
  } catch (const std::exception &e) {
    setError(std::string("Exception during validation: ") + e.what());
    setStatus(ResourceTaskStatus::FAILED);
    return false;
  }
}

std::chrono::milliseconds ValidateResourceTask::getEstimatedTime() const {
  // Validation is usually fast
  return std::chrono::milliseconds(50);
}

// ====================================================================
// SERIALIZE RESOURCE TASK
// ====================================================================

SerializeResourceTask::SerializeResourceTask(
    const TaskID id, const std::shared_ptr<Resource> &resource,
    const ResourcePriority priority)
    : ResourceTask(id, ResourceTaskType::SERIALIZE, priority),
      resource_(resource) {
  if (resource) {
    setName("Serialize_" + resource->getName());
  }
}

bool SerializeResourceTask::execute() {
  setStatus(ResourceTaskStatus::RUNNING);

  if (!resource_) {
    setError("No resource to serialize");
    setStatus(ResourceTaskStatus::FAILED);
    return false;
  }

  try {
    updateProgress(0.5f);

    const bool result = resource_->serialize(serializedData_);

    updateProgress(1.0f);

    if (result) {
      setStatus(ResourceTaskStatus::COMPLETED);
    } else {
      setError("Serialization failed");
      setStatus(ResourceTaskStatus::FAILED);
    }

    return result;
  } catch (const std::exception &e) {
    setError(std::string("Exception during serialization: ") + e.what());
    setStatus(ResourceTaskStatus::FAILED);
    return false;
  }
}

std::chrono::milliseconds SerializeResourceTask::getEstimatedTime() const {
  if (resource_) {
    // Estimate based on resource memory usage
    return std::chrono::milliseconds(resource_->getMemoryUsage() / (50 * 1024));
  }
  return std::chrono::milliseconds(100);
}

// ====================================================================
// DESERIALIZE RESOURCE TASK
// ====================================================================

DeserializeResourceTask::DeserializeResourceTask(
    const TaskID id, const std::shared_ptr<Resource> &resource,
    const std::uint8_t *data, const std::size_t size,
    const ResourcePriority priority)
    : ResourceTask(id, ResourceTaskType::DESERIALIZE, priority),
      resource_(resource), data_(data), dataSize_(size) {
  if (resource) {
    setName("Deserialize_" + resource->getName());
  }
}

bool DeserializeResourceTask::execute() {
  setStatus(ResourceTaskStatus::RUNNING);

  if (!resource_) {
    setError("No resource to deserialize into");
    setStatus(ResourceTaskStatus::FAILED);
    return false;
  }

  if (!data_ || dataSize_ == 0) {
    setError("No data to deserialize");
    setStatus(ResourceTaskStatus::FAILED);
    return false;
  }

  try {
    updateProgress(0.5f);

    const bool result = resource_->deserialize(data_, dataSize_);

    updateProgress(1.0f);

    if (result) {
      setStatus(ResourceTaskStatus::COMPLETED);
    } else {
      setError("Deserialization failed");
      setStatus(ResourceTaskStatus::FAILED);
    }

    return result;
  } catch (const std::exception &e) {
    setError(std::string("Exception during deserialization: ") + e.what());
    setStatus(ResourceTaskStatus::FAILED);
    return false;
  }
}

std::chrono::milliseconds DeserializeResourceTask::getEstimatedTime() const {
  // Estimate based on data size
  return std::chrono::milliseconds(dataSize_ / (100 * 1024));
}

// ====================================================================
// BATCH RESOURCE TASK
// ====================================================================

BatchResourceTask::BatchResourceTask(const TaskID id,
                                     const ResourcePriority priority)
    : ResourceTask(id, ResourceTaskType::PROCESS, priority) {
  setName("Batch_" + std::to_string(id));
}

void BatchResourceTask::addTask(std::unique_ptr<ResourceTask> task) {
  if (task) {
    tasks_.push_back(std::move(task));
  }
}

bool BatchResourceTask::execute() {
  setStatus(ResourceTaskStatus::RUNNING);

  if (tasks_.empty()) {
    setStatus(ResourceTaskStatus::COMPLETED);
    return true;
  }

  bool allSuccessful = true;

  for (std::size_t i = 0; i < tasks_.size(); ++i) {
    const auto &task = tasks_[i];

    // Update overall progress
    const float taskProgress =
        static_cast<float>(i) / static_cast<float>(tasks_.size());
    updateProgress(taskProgress);

    // Execute task
    if (const bool success = task->execute(); !success) {
      allSuccessful = false;
      setError("Task " + task->getName() +
               " failed: " + task->getErrorMessage());

      if (stopOnError_) {
        setStatus(ResourceTaskStatus::FAILED);
        return false;
      }
    }
  }

  updateProgress(1.0f);
  setStatus(allSuccessful ? ResourceTaskStatus::COMPLETED
                          : ResourceTaskStatus::FAILED);
  return allSuccessful;
}

std::chrono::milliseconds BatchResourceTask::getEstimatedTime() const {
  std::chrono::milliseconds total(0);

  for (const auto &task : tasks_) {
    total += task->getEstimatedTime();
  }

  return total;
}
} // namespace engine::resources
