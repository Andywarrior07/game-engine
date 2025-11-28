//
// Created by Andres Guerrero on 19-08-25.
//

#include "ResourcePackage.h"

#include <cstring>
#include <fstream>
#include <mutex>
#include <ranges>

namespace engine::resources {
bool ResourcePackage::load() {
  if (isLoaded_)
    return true;

  std::unique_lock lock(mutex_);

  std::ifstream file(packagePath_, std::ios::binary);
  if (!file.is_open()) {
    return false;
  }

  // Read header
  file.read(reinterpret_cast<char *>(&header_), sizeof(PackageHeader));

  // Validate magic number
  if (header_.magic != 0x52504B47) {
    return false;
  }

  // Read entries
  for (std::uint32_t i = 0; i < header_.resourceCount; ++i) {
    ResourceEntry entry;

    // Read entry data
    file.read(reinterpret_cast<char *>(&entry.id), sizeof(entry.id));
    file.read(reinterpret_cast<char *>(&entry.type), sizeof(entry.type));
    file.read(reinterpret_cast<char *>(&entry.offset), sizeof(entry.offset));
    file.read(reinterpret_cast<char *>(&entry.compressedSize),
              sizeof(entry.compressedSize));
    file.read(reinterpret_cast<char *>(&entry.uncompressedSize),
              sizeof(entry.uncompressedSize));
    file.read(reinterpret_cast<char *>(&entry.checksum),
              sizeof(entry.checksum));

    // Read name length and name
    std::uint32_t nameLength;
    file.read(reinterpret_cast<char *>(&nameLength), sizeof(nameLength));
    entry.name.resize(nameLength);
    file.read(entry.name.data(), nameLength);

    entries_[entry.id] = entry;
  }

  // Load the entire package into memory (for fast access)
  // In production might use memory mapping instead
  packageSize_ = header_.totalSize;
  packageData_ = std::make_unique<std::uint8_t[]>(packageSize_);

  file.seekg(0);
  file.read(reinterpret_cast<char *>(packageData_.get()), packageSize_);

  isLoaded_ = true;

  return true;
}

void ResourcePackage::unload() {
  std::unique_lock lock(mutex_);

  entries_.clear();
  packageData_.reset();
  packageSize_ = 0;
  isLoaded_ = false;
}

bool ResourcePackage::hasResource(ResourceID id) const {
  std::shared_lock lock(mutex_);

  return entries_.contains(id);
}

std::optional<ResourcePackage::ResourceEntry>
ResourcePackage::getResourceEntry(ResourceID id) const {
  std::shared_lock lock(mutex_);

  if (const auto it = entries_.find(id); it != entries_.end()) {
    return it->second;
  }

  return std::nullopt;
}

std::vector<ResourceID> ResourcePackage::getAllResourceIDs() const {
  std::shared_lock lock(mutex_);

  std::vector<ResourceID> ids;
  ids.reserve(entries_.size());

  for (const auto &id : entries_ | std::views::keys) {
    ids.push_back(id);
  }

  return ids;
}

std::unique_ptr<std::uint8_t[]>
ResourcePackage::extractResource(ResourceID id, ResourceSize &size) {
  std::shared_lock lock(mutex_);

  if (!packageData_) {
    size = 0;
    return nullptr;
  }

  const auto it = entries_.find(id);
  if (it == entries_.end()) {
    size = 0;
    return nullptr;
  }

  const auto &entry = it->second;

  // Extract data from package
  auto data = std::make_unique<std::uint8_t[]>(entry.uncompressedSize);

  if (entry.compressedSize == entry.uncompressedSize) {
    // No compression, direct copy
    std::memcpy(data.get(), packageData_.get() + entry.offset,
                entry.uncompressedSize);
  } else {
    // Decompress data
    // This would integrate with a compression library (LZ4, ZSTD, etc.)
    // For now, just copy compressed data as placeholder
    std::memcpy(data.get(), packageData_.get() + entry.offset,
                entry.uncompressedSize);
  }

  size = entry.uncompressedSize;

  return data;
}

bool ResourcePackage::createPackage(
    const std::string &outputPath,
    const std::vector<std::pair<std::string, ResourceMetadata>> &resources,
    CompressionType compression) {
  std::ofstream file(outputPath, std::ios::binary);
  if (!file.is_open()) {
    return false;
  }

  PackageHeader header;
  header.resourceCount = static_cast<std::uint32_t>(resources.size());
  header.compressionType = static_cast<std::uint32_t>(compression);

  // Write placeholder header
  std::streampos headerPos = file.tellp();
  file.write(reinterpret_cast<const char *>(&header), sizeof(PackageHeader));

  std::uint64_t currentOffset =
      sizeof(PackageHeader) +
      (resources.size() * (sizeof(ResourceEntry) + 256)); // Estimate entry size

  std::vector<ResourceEntry> entries;

  // Write resources
  for (const auto &[path, metadata] : resources) {
    ResourceEntry entry;
    entry.id = metadata.id;
    entry.type = metadata.type;
    entry.name = metadata.name;
    entry.offset = currentOffset;

    // Load resource data
    std::ifstream resourceFile(path, std::ios::binary | std::ios::ate);
    if (!resourceFile.is_open()) {
      continue;
    }

    entry.uncompressedSize = static_cast<std::uint64_t>(resourceFile.tellg());
    resourceFile.seekg(0);

    std::vector<std::uint8_t> data(entry.uncompressedSize);
    resourceFile.read(reinterpret_cast<char *>(data.data()),
                      entry.uncompressedSize);

    // Compress if needed
    if (compression != CompressionType::NONE) {
      // Compress data here
      // For now, no compression
      entry.compressedSize = entry.uncompressedSize;
    } else {
      entry.compressedSize = entry.uncompressedSize;
    }

    // Calculate checksum (simple sum for now)
    entry.checksum = 0;
    for (std::size_t i = 0; i < data.size(); ++i) {
      entry.checksum += data[i];
    }

    // Write to package at current offset
    file.seekp(currentOffset);
    file.write(reinterpret_cast<const char *>(data.data()),
               entry.uncompressedSize);

    currentOffset += entry.uncompressedSize;
    entries.push_back(entry);
  }

  header.totalSize = currentOffset;

  // Write entry table after header
  file.seekp(sizeof(PackageHeader));
  for (const auto &entry : entries) {
    file.write(reinterpret_cast<const char *>(&entry.id), sizeof(entry.id));
    file.write(reinterpret_cast<const char *>(&entry.type), sizeof(entry.type));
    file.write(reinterpret_cast<const char *>(&entry.offset),
               sizeof(entry.offset));
    file.write(reinterpret_cast<const char *>(&entry.compressedSize),
               sizeof(entry.compressedSize));
    file.write(reinterpret_cast<const char *>(&entry.uncompressedSize),
               sizeof(entry.uncompressedSize));
    file.write(reinterpret_cast<const char *>(&entry.checksum),
               sizeof(entry.checksum));

    auto nameLength = static_cast<std::uint32_t>(entry.name.length());
    file.write(reinterpret_cast<const char *>(&nameLength), sizeof(nameLength));
    file.write(entry.name.data(), nameLength);
  }

  // Update header with final values
  file.seekp(headerPos);
  file.write(reinterpret_cast<const char *>(&header), sizeof(PackageHeader));

  return true;
}
} // namespace engine::resources
