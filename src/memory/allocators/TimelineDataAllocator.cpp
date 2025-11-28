//
// Created by Andres Guerrero on 19-09-25.
//

#include "TimelineDataAllocator.h"

#include <algorithm>
#include <mutex>
#include <ranges>

namespace engine::memory {
TimelineDataAllocator::TimelineDataAllocator(
    std::shared_ptr<LinearAllocator> linearAllocator, TimelineConfig config,
    const char *name)
    : linearAllocator_(std::move(linearAllocator)), config_(std::move(config)),
      currentPhase_(TimelinePhase::INITIALIZATION),
      timelineStartTime_(std::chrono::steady_clock::now()),
      nextAllocationId_(1), totalResetsCount_(0), name_(name) {
  if (!linearAllocator_) {
    throw std::invalid_argument("LinearAllocator cannot be null");
  }

  // Initialize category usage tracking
  for (std::size_t i = 0; i < static_cast<std::size_t>(DataCategory::COUNT);
       ++i) {
    categoryUsage_[i] = 0;
  }

  // Reserve memory for critical categories if needed
  if (config_.reservedSize > 0) {
    reserveMemoryForCategories();
  }

  std::cout << "[TimelineDataAllocator] Initialized timeline '" << config_.name
            << "' with "
            << (static_cast<double>(linearAllocator_->getCapacity()) /
                (1024.0 * 1024.0))
            << " MB" << std::endl;
}

TimelineDataAllocator &
TimelineDataAllocator::operator=(TimelineDataAllocator &&other) noexcept {
  if (this != &other) {
    linearAllocator_ = std::move(other.linearAllocator_);
    config_ = std::move(other.config_);
    currentPhase_ = other.currentPhase_;
    timelineStartTime_ = other.timelineStartTime_;
    nextAllocationId_ = other.nextAllocationId_.load();
    totalResetsCount_ = other.totalResetsCount_;
    allocations_ = std::move(other.allocations_);
    categoryUsage_ = other.categoryUsage_;
    name_ = other.name_;

    other.currentPhase_ = TimelinePhase::INACTIVE;
    other.nextAllocationId_ = 1;
    other.totalResetsCount_ = 0;
  }
  return *this;
}

void *TimelineDataAllocator::allocate(const MemorySize size,
                                      const MemorySize alignment,
                                      const AllocationFlags flags) {
  return allocateWithCategory(size, DataCategory::TIMELINE_CORE, "", alignment,
                              flags);
}

void TimelineDataAllocator::reset() {
  std::unique_lock lock(metadataMutex_);

  linearAllocator_->reset();

  // Clear tracking data
  allocations_.clear();
  for (std::size_t i = 0; i < static_cast<std::size_t>(DataCategory::COUNT);
       ++i) {
    categoryUsage_[i] = 0;
  }

  nextAllocationId_ = 1;
}

void *TimelineDataAllocator::allocateWithCategory(const MemorySize size,
                                                  const DataCategory category,
                                                  const std::string &tag,
                                                  const MemorySize alignment,
                                                  const AllocationFlags flags) {
  // Check timeline phase
  if (currentPhase_ == TimelinePhase::INACTIVE ||
      currentPhase_ == TimelinePhase::CLEANUP) {
    std::cerr << "Warning: Allocating from inactive timeline!" << std::endl;
  }

  // TODO: Arreglar esto ya que ptr es void
  // Attempt allocation from linear allocator
  void *ptr = linearAllocator_->allocate(size, alignment, flags);

  if (!ptr) {
    // Try to grow if allowed
    if (config_.allowGrowth && attemptGrowth(size)) {
      ptr = linearAllocator_->allocate(size, alignment, flags);
    }

    if (!ptr) {
      return nullptr;
    }
  }

  // Track allocation metadata
  trackAllocation(ptr, size, category, tag);

  return ptr;
}

std::vector<void *> TimelineDataAllocator::batchAllocate(
    const std::vector<std::tuple<MemorySize, DataCategory, std::string>>
        &allocations,
    const MemorySize alignment, const AllocationFlags flags) {
  std::vector<void *> results;
  results.reserve(allocations.size());

  // Calculate total size needed
  MemorySize totalSize = 0;
  for (const auto &[size, category, tag] : allocations) {
    totalSize += alignSize(size, alignment);
  }

  // TODO: Revisar esto
  // Check if we have enough space
  if (getFreeMemory() < totalSize) {
    if (!config_.allowGrowth || !attemptGrowth(totalSize)) {
      return {}; // Return empty vector on failure
    }
  }

  // Allocate each item
  for (const auto &[size, category, tag] : allocations) {
    // TODO: Revisar esto, ptr es void
    void *ptr = allocateWithCategory(size, category, tag, alignment, flags);
    if (!ptr) {
      // Cleanup partial allocation
      // Note: Linear allocator doesn't support individual deallocation,
      // but we track this for debugging
      std::cerr << "Warning: Batch allocation partially failed!" << std::endl;
      break;
    }
    results.push_back(ptr);
  }

  return results;
}

void TimelineDataAllocator::resetForNewTimeline(
    const TimelineConfig &newConfig) {
  std::unique_lock lock(metadataMutex_);

  // Store old timeline info for reporting
#ifdef _DEBUG
  if (!allocations_.empty()) {
    std::cout << "[TimelineDataAllocator] Resetting timeline '" << config_.name
              << "' with " << allocations_.size() << " active allocations"
              << std::endl;
  }
#endif

  // Reset base allocator
  linearAllocator_->reset();

  // Update configuration
  config_ = newConfig;
  currentPhase_ = TimelinePhase::INITIALIZATION;
  timelineStartTime_ = std::chrono::steady_clock::now();

  // Clear tracking data
  allocations_.clear();
  for (std::size_t i = 0; i < static_cast<std::size_t>(DataCategory::COUNT);
       ++i) {
    categoryUsage_[i] = 0;
  }

  nextAllocationId_ = 1;
  totalResetsCount_++;

  // Reserve memory for new timeline
  if (config_.reservedSize > 0) {
    reserveMemoryForCategories();
  }
}

void TimelineDataAllocator::setTimelinePhase(const TimelinePhase phase) {
  currentPhase_ = phase;

#ifdef _DEBUG
  std::cout << "[TimelineDataAllocator] Timeline '" << config_.name
            << "' phase changed to " << getPhaseString(phase) << std::endl;
#endif
}

std::size_t TimelineDataAllocator::getCategoryAllocationCount(
    const DataCategory category) const {
  std::shared_lock lock(metadataMutex_);

  std::size_t count = 0;

  for (const auto &metadata : allocations_ | std::views::values) {
    if (metadata.category == category) {
      count++;
    }
  }

  return count;
}

std::string TimelineDataAllocator::generateTimelineReport() const {
  std::shared_lock lock(metadataMutex_);
  std::stringstream report;

  report << "=== Timeline Data Allocator Report ===" << std::endl;
  report << "Timeline: " << config_.name << std::endl;
  report << "Phase: " << getPhaseString(currentPhase_) << std::endl;
  report << "Runtime: " << getTimelineRuntime().count() << " ms" << std::endl;
  report << "Total Resets: " << totalResetsCount_ << std::endl;
  report << std::endl;

  // Memory usage
  report << "Memory Usage:" << std::endl;
  report << "  Capacity: "
         << (static_cast<double>(getCapacity()) / (1024.0 * 1024.0)) << " MB"
         << std::endl;
  report << "  Used: "
         << (static_cast<double>(getUsedMemory()) / (1024.0 * 1024.0)) << " MB"
         << std::endl;
  report << "  Free: "
         << (static_cast<double>(getFreeMemory()) / (1024.0 * 1024.0)) << " MB"
         << std::endl;
  report << "  Utilization: " << (getUtilization() * 100.0f) << "%"
         << std::endl;
  report << std::endl;

  // Category breakdown
  report << "Category Usage:" << std::endl;
  for (std::size_t i = 0; i < static_cast<std::size_t>(DataCategory::COUNT);
       ++i) {
    const auto category = static_cast<DataCategory>(i);
    const MemorySize usage = getCategoryUsage(category);

    if (const std::size_t count = getCategoryAllocationCount(category);
        usage > 0 || count > 0) {
      report << "  " << getCategoryString(category) << ": "
             << (static_cast<double>(usage) / 1024.0) << " KB (" << count
             << " allocations)" << std::endl;
    }
  }
  report << std::endl;

  // Recent allocations (last 10)
  report << "Recent Allocations:" << std::endl;
  std::vector<std::pair<void *, AllocationMetadata>> recent;
  for (const auto &entry : allocations_) {
    recent.emplace_back(entry);
  }

  // Sort by allocation ID (most recent first)
  std::ranges::sort(recent, [](const auto &a, const auto &b) {
    return a.second.allocationId > b.second.allocationId;
  });

  for (std::size_t i = 0;
       i < std::min(recent.size(), static_cast<std::size_t>(10)); ++i) {
    const auto &metadata = recent[i].second;
    report << "  ID " << metadata.allocationId << ": " << metadata.size
           << " bytes"
           << ", " << getCategoryString(metadata.category);
    if (!metadata.tag.empty()) {
      report << " (" << metadata.tag << ")";
    }
    report << std::endl;
  }

  return report.str();
}

std::vector<void *>
TimelineDataAllocator::findAllocationsByTag(const std::string &tag) const {
  std::shared_lock lock(metadataMutex_);
  std::vector<void *> results;

  for (const auto &[ptr, metadata] : allocations_) {
    if (metadata.tag == tag) {
      results.push_back(ptr);
    }
  }

  return results;
}

void TimelineDataAllocator::trackAllocation(void *ptr, const MemorySize size,
                                            DataCategory category,
                                            const std::string &tag) {
  std::unique_lock lock(metadataMutex_);

  AllocationMetadata metadata;
  metadata.offset = linearAllocator_->getOffset(ptr);
  metadata.size = size;
  metadata.category = category;
  metadata.tag = tag;
  metadata.timestamp = std::chrono::steady_clock::now();
  metadata.allocationId =
      nextAllocationId_.fetch_add(1, std::memory_order_acq_rel);

  allocations_[ptr] = metadata;
  categoryUsage_[static_cast<std::size_t>(category)] += size;
}

// TODO: Revisar esto
bool TimelineDataAllocator::attemptGrowth(
    const MemorySize additionalSize) const {
  if (!config_.allowGrowth)
    return false;

  const MemorySize currentCapacity = getCapacity();

  // TODO: Revisar esto
  // Ensure we have at least the additional size needed
  if (MemorySize newCapacity =
          currentCapacity * static_cast<std::size_t>(config_.growthFactor);
      newCapacity < currentCapacity + additionalSize) {
    newCapacity = currentCapacity + additionalSize;
  }

  // For LinearAllocator, we can't actually grow in place
  // This would require creating a new larger buffer and copying data
  // For now, return false to indicate growth failed
  std::cerr << "[TimelineDataAllocator] Growth needed but not implemented for "
               "LinearAllocator base"
            << std::endl;
  return false;
}

void TimelineDataAllocator::reserveMemoryForCategories() {
  // Pre-allocate small amounts for critical categories to ensure availability
  constexpr MemorySize CORE_RESERVE = 1024;     // 1KB for core timeline data
  constexpr MemorySize CONFIG_RESERVE = 512;    // 512B for configuration
  constexpr MemorySize TIMESTEP_RESERVE = 2048; // 2KB for timestep calculations

  // Reserve memory to ensure critical allocations don't fail
  const void *corePtr = allocateWithCategory(
      CORE_RESERVE, DataCategory::TIMELINE_CORE, "reserved_core");
  const void *configPtr = allocateWithCategory(
      CONFIG_RESERVE, DataCategory::CONFIG_DATA, "reserved_config");
  const void *timestepPtr = allocateWithCategory(
      TIMESTEP_RESERVE, DataCategory::TIMESTEP_DATA, "reserved_timestep");

  // These reservations remain allocated to guarantee space
  // They can be used by critical systems when needed
  (void)corePtr;
  (void)configPtr;
  (void)timestepPtr; // Suppress unused warnings
}

void TimelineDataAllocator::generateFinalReport() const {
  if (config_.name.empty())
    return;

  std::cout << "[TimelineDataAllocator] Final report for timeline '"
            << config_.name << "':" << std::endl;
  std::cout << "  Total runtime: " << getTimelineRuntime().count() << " ms"
            << std::endl;
  // std::cout << "  Peak usage: " << (linearAllocator_->stats_.peakUsage.load()
  // / (1024.0 * 1024.0)) << " MB" << std::endl;
  std::cout << "  Total resets: " << totalResetsCount_ << std::endl;

  if (!allocations_.empty()) {
    std::cout << "  Warning: " << allocations_.size()
              << " allocations not cleaned up!" << std::endl;
  }
}

const char *TimelineDataAllocator::getPhaseString(const TimelinePhase phase) {
  switch (phase) {
  case TimelinePhase::INITIALIZATION:
    return "INITIALIZATION";
  case TimelinePhase::ACTIVE:
    return "ACTIVE";
  case TimelinePhase::PAUSED:
    return "PAUSED";
  case TimelinePhase::TRANSITIONING:
    return "TRANSITIONING";
  case TimelinePhase::CLEANUP:
    return "CLEANUP";
  case TimelinePhase::INACTIVE:
    return "INACTIVE";
  default:
    return "UNKNOWN";
  }
}

const char *
TimelineDataAllocator::getCategoryString(const DataCategory category) {
  switch (category) {
  case DataCategory::TIMELINE_CORE:
    return "TIMELINE_CORE";
  case DataCategory::TIMESTEP_DATA:
    return "TIMESTEP_DATA";
  case DataCategory::EVENT_DATA:
    return "EVENT_DATA";
  case DataCategory::ANIMATION_DATA:
    return "ANIMATION_DATA";
  case DataCategory::CONFIG_DATA:
    return "CONFIG_DATA";
  case DataCategory::TEMP_CALCULATIONS:
    return "TEMP_CALCULATIONS";
  case DataCategory::DEBUG_DATA:
    return "DEBUG_DATA";
  default:
    return "UNKNOWN";
  }
}
} // namespace engine::memory
