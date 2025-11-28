//
// Created by Andres Guerrero on 20-08-25.
//

#include "StreamableResource.h"
#include <cmath>

namespace engine::resources {
// ====================================================================
// CONSTRUCTOR / DESTRUCTOR
// ====================================================================

StreamableResource::StreamableResource(const ResourceID id,
                                       const std::string &name,
                                       const ResourceType type)
    : Resource(id, name, type), streamingState_(StreamingState::NOT_STREAMING),
      streamingPriority_(StreamingPriority::NORMAL), currentLOD_(UINT32_MAX),
      importance_(1.0f), lastAccessTime_(std::chrono::steady_clock::now()),
      streamInCount_(0), streamOutCount_(0), totalStreamTime_(0) {}

// StreamableResource::~StreamableResource() {
//     // Ensure resource is properly cleaned up
//     // Derived classes should handle their specific cleanup
// }

// ====================================================================
// OPTIONAL STREAMING INTERFACE IMPLEMENTATION
// ====================================================================

bool StreamableResource::isLODLoaded(const std::uint32_t lodLevel) const {
  if (!validateLODLevel(lodLevel)) {
    return false;
  }

  const std::uint32_t currentLODLevel =
      currentLOD_.load(std::memory_order_acquire);

  // If nothing is loaded
  if (currentLODLevel == UINT32_MAX) {
    return false;
  }

  // LOD is loaded if current LOD is equal or better (lower number = higher
  // detail)
  return currentLODLevel <= lodLevel;
}

LODInfo StreamableResource::getLODInfo(const std::uint32_t lodLevel) const {
  LODInfo info;
  info.level = lodLevel;
  info.isLoaded = isLODLoaded(lodLevel);

  // Default implementation - derived classes should override for accurate info
  // Estimate memory size based on LOD level (each level roughly half the size)
  ResourceSize baseSize = getMemoryUsage();
  if (baseSize > 0 && lodLevel > 0) {
    info.memorySize = baseSize >> lodLevel; // Divide by 2^lodLevel
  } else {
    info.memorySize = baseSize;
  }

  // Default distance thresholds (can be overridden)
  info.distanceThreshold = lodLevel * 10.0f;

  return info;
}

ResourceSize StreamableResource::getTotalMemoryRequired() const {
  ResourceSize total = 0;
  const std::uint32_t maxLOD = getMaxLODLevel();

  for (std::uint32_t lod = 0; lod <= maxLOD; ++lod) {
    const LODInfo info = getLODInfo(lod);
    total += info.memorySize;
  }

  return total;
}

void StreamableResource::prefetchLOD(const std::uint32_t lodLevel) {
  // Default implementation does nothing
  // Derived classes can implement prefetching logic
  (void)lodLevel; // Suppress unused parameter warning
}

// ====================================================================
// STREAMING STATE MANAGEMENT
// ====================================================================

StreamingState StreamableResource::getStreamingState() const {
  return streamingState_.load(std::memory_order_acquire);
}

void StreamableResource::setStreamingState(const StreamingState state) {
  streamingState_.store(state, std::memory_order_release);
}

StreamingPriority StreamableResource::getStreamingPriority() const {
  return streamingPriority_.load(std::memory_order_acquire);
}

void StreamableResource::setStreamingPriority(
    const StreamingPriority priority) {
  streamingPriority_.store(priority, std::memory_order_release);
}

float StreamableResource::getImportance() const {
  return importance_.load(std::memory_order_acquire);
}

void StreamableResource::setImportance(const float requiredImportance) {
  importance_.store(std::clamp(requiredImportance, 0.0f, 1.0f),
                    std::memory_order_release);
}

std::chrono::steady_clock::time_point
StreamableResource::getLastAccessTime() const {
  return lastAccessTime_.load(std::memory_order_acquire);
}

void StreamableResource::updateLastAccessTime() const {
  lastAccessTime_.store(std::chrono::steady_clock::now(),
                        std::memory_order_release);
}

// ====================================================================
// STREAMING HINTS
// ====================================================================

std::uint32_t StreamableResource::suggestLODLevel(float qualityMetric) const {
  qualityMetric = std::clamp(qualityMetric, 0.0f, 1.0f);
  const std::uint32_t maxLOD = getMaxLODLevel();

  if (maxLOD == 0) {
    return 0; // Only one LOD available
  }

  // Map quality metric to LOD level
  // 1.0 = LOD 0 (highest), 0.0 = max LOD (lowest)
  const float lodFloat = (1.0f - qualityMetric) * static_cast<float>(maxLOD);
  const auto suggestedLOD = static_cast<std::uint32_t>(std::round(lodFloat));

  return std::min(suggestedLOD, maxLOD);
}

bool StreamableResource::shouldStreamOut(float memoryPressure) const {
  memoryPressure = std::clamp(memoryPressure, 0.0f, 1.0f);

  // Don't stream out if:
  // - Resource is persistent
  // - Resource has high importance
  // - Resource was recently accessed

  if (hasFlag(metadata_.flags, ResourceFlags::PERSISTENT)) {
    return false;
  }

  const float resourceImportance = getImportance();
  if (resourceImportance > 0.8f) {
    return false; // High importance resources stay in memory
  }

  // Check last access time
  const auto now = std::chrono::steady_clock::now();
  const auto lastAccess = getLastAccessTime();

  // If accessed within last 5 seconds and memory pressure is not critical
  if (const auto timeSinceAccess =
          std::chrono::duration_cast<std::chrono::seconds>(now - lastAccess);
      timeSinceAccess.count() < 5 && memoryPressure < 0.9f) {
    return false;
  }

  // More likely to stream out with higher memory pressure and lower importance
  const float streamOutProbability =
      memoryPressure * (1.0f - resourceImportance);

  // Simple threshold - in production, could use more sophisticated heuristics
  return streamOutProbability > 0.5f;
}

float StreamableResource::getStreamingCost(const std::uint32_t lodLevel) const {
  // Estimate streaming cost in milliseconds
  // This is a simple estimate - derived classes should provide accurate costs

  const LODInfo info = getLODInfo(lodLevel);

  // Assume 100 MB/s streaming speed as baseline
  constexpr float streamingSpeed =
      100.0f * 1024.0f * 1024.0f; // Bytes per second

  const float seconds = static_cast<float>(info.memorySize) / streamingSpeed;
  float milliseconds = seconds * 1000.0f;

  // Add overhead based on priority
  const StreamingPriority priority = getStreamingPriority();
  switch (priority) {
  case StreamingPriority::IMMEDIATE:
    break; // No additional overhead
  case StreamingPriority::HIGH:
    milliseconds *= 1.1f;
    break;
  case StreamingPriority::NORMAL:
    milliseconds *= 1.25f;
    break;
  case StreamingPriority::LOW:
    milliseconds *= 1.5f;
    break;
  case StreamingPriority::BACKGROUND:
    milliseconds *= 2.0f;
    break;
  }

  return milliseconds;
}

// ====================================================================
// PROTECTED HELPER METHODS
// ====================================================================

void StreamableResource::recordStreamingOperation(
    const bool isStreamIn, const std::chrono::microseconds duration) {
  if (isStreamIn) {
    streamInCount_.fetch_add(1, std::memory_order_relaxed);
  } else {
    streamOutCount_.fetch_add(1, std::memory_order_relaxed);
  }

  totalStreamTime_.fetch_add(duration.count(), std::memory_order_relaxed);
}

bool StreamableResource::validateLODLevel(const std::uint32_t lodLevel) const {
  return lodLevel <= getMaxLODLevel();
}
} // namespace engine::resources
