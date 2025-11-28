//
// Created by Andres Guerrero on 20-08-25.
//

#include "ResourceRegistry.h"

#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <ranges>
#include <sstream>

namespace engine::resources {
// ====================================================================
// RESOURCE TYPE REGISTRATION
// ====================================================================

void ResourceRegistry::registerResourceType(
    ResourceType type, const std::string &name,
    const std::vector<std::string> &extensions) {
  TypeInfo info;
  info.name = name;
  info.extensions = extensions;

  registerResourceType(type, info);
}

void ResourceRegistry::registerResourceType(ResourceType type,
                                            const TypeInfo &info) {
  std::unique_lock lock(mutex_);

  // Store type info
  typeInfo_[type] = info;

  // Map extensions to type
  for (const auto &ext : info.extensions) {
    std::string normalizedExt = normalizeExtension(ext);
    extensionToType_[normalizedExt] = type;
  }

  std::cout << "Registered resource type: " << info.name << " with "
            << info.extensions.size() << " extensions" << std::endl;
}

void ResourceRegistry::unregisterResourceType(ResourceType type) {
  std::unique_lock lock(mutex_);

  if (const auto it = typeInfo_.find(type); it != typeInfo_.end()) {
    // Remove extension mappings
    for (const auto &ext : it->second.extensions) {
      std::string normalizedExt = normalizeExtension(ext);
      extensionToType_.erase(normalizedExt);
    }

    // Remove type info
    typeInfo_.erase(it);

    // Remove associated loaders
    loaders_.erase(type);
  }
}

bool ResourceRegistry::updateResourceType(ResourceType type,
                                          const TypeInfo &info) {
  std::unique_lock lock(mutex_);

  const auto it = typeInfo_.find(type);
  if (it == typeInfo_.end()) {
    return false;
  }

  // Remove old extension mappings
  for (const auto &ext : it->second.extensions) {
    std::string normalizedExt = normalizeExtension(ext);
    extensionToType_.erase(normalizedExt);
  }

  // Update type info
  it->second = info;

  // Add new extension mappings
  for (const auto &ext : info.extensions) {
    std::string normalizedExt = normalizeExtension(ext);
    extensionToType_[normalizedExt] = type;
  }

  return true;
}

// ====================================================================
// RESOURCE TYPE QUERIES
// ====================================================================

std::optional<ResourceType>
ResourceRegistry::getTypeFromExtension(const std::string &extension) const {
  std::shared_lock lock(mutex_);

  const std::string normalizedExt = normalizeExtension(extension);

  if (const auto it = extensionToType_.find(normalizedExt);
      it != extensionToType_.end()) {
    return it->second;
  }

  return std::nullopt;
}

std::optional<ResourceType>
ResourceRegistry::getTypeFromPath(const std::string &filepath) const {
  std::string ext = extractExtension(filepath);
  if (!ext.empty()) {
    return getTypeFromExtension(ext);
  }
  return std::nullopt;
}

std::optional<ResourceRegistry::TypeInfo>
ResourceRegistry::getTypeInfo(ResourceType type) const {
  std::shared_lock lock(mutex_);

  if (const auto it = typeInfo_.find(type); it != typeInfo_.end()) {
    return it->second;
  }

  return std::nullopt;
}

std::vector<ResourceType> ResourceRegistry::getRegisteredTypes() const {
  std::shared_lock lock(mutex_);

  std::vector<ResourceType> types;
  types.reserve(typeInfo_.size());

  for (const auto &type : typeInfo_ | std::views::keys) {
    types.push_back(type);
  }

  return types;
}

bool ResourceRegistry::isTypeRegistered(const ResourceType type) const {
  std::shared_lock lock(mutex_);
  return typeInfo_.contains(type);
}

std::vector<std::string>
ResourceRegistry::getExtensionsForType(ResourceType type) const {
  std::shared_lock lock(mutex_);

  auto it = typeInfo_.find(type);
  if (it != typeInfo_.end()) {
    return it->second.extensions;
  }

  return {};
}

// ====================================================================
// PATH ALIAS MANAGEMENT
// ====================================================================

void ResourceRegistry::registerPath(const std::string &alias,
                                    const std::string &path,
                                    const std::string &description) {
  std::unique_lock lock(mutex_);

  PathAlias pathAlias;
  pathAlias.path = path;
  pathAlias.description = description;
  pathAlias.isRelative = !std::filesystem::path(path).is_absolute();

  pathAliases_[alias] = pathAlias;

  std::cout << "Registered path alias: @" << alias << " -> " << path
            << std::endl;
}

void ResourceRegistry::unregisterPath(const std::string &alias) {
  std::unique_lock lock(mutex_);
  pathAliases_.erase(alias);
}

std::string ResourceRegistry::resolvePath(const std::string &path) const {
  // Check if path starts with an alias marker (@)
  if (path.empty() || path[0] != '@') {
    return path;
  }

  // Find the end of the alias
  size_t separatorPos = path.find('/');
  if (separatorPos == std::string::npos) {
    separatorPos = path.find('\\');
  }

  std::string alias;
  std::string remainder;

  if (separatorPos != std::string::npos) {
    alias = path.substr(1, separatorPos - 1); // Skip the @ symbol
    remainder = path.substr(separatorPos);
  } else {
    alias = path.substr(1); // Skip the @ symbol
  }

  std::shared_lock lock(mutex_);

  auto it = pathAliases_.find(alias);
  if (it != pathAliases_.end()) {
    return it->second.path + remainder;
  }

  // If alias not found, return original path
  return path;
}

std::optional<ResourceRegistry::PathAlias>
ResourceRegistry::getPathAlias(const std::string &alias) const {
  std::shared_lock lock(mutex_);

  auto it = pathAliases_.find(alias);
  if (it != pathAliases_.end()) {
    return it->second;
  }

  return std::nullopt;
}

std::unordered_map<std::string, ResourceRegistry::PathAlias>
ResourceRegistry::getAllPathAliases() const {
  std::shared_lock lock(mutex_);
  return pathAliases_;
}

// ====================================================================
// LOADER REGISTRATION
// ====================================================================

void ResourceRegistry::registerLoader(ResourceType type,
                                      const LoaderInfo &info) {
  std::unique_lock lock(mutex_);

  loaders_[type].push_back(info);

  // Sort by priority (higher priority first)
  std::sort(loaders_[type].begin(), loaders_[type].end(),
            [](const LoaderInfo &a, const LoaderInfo &b) {
              return a.priority > b.priority;
            });
}

std::vector<ResourceRegistry::LoaderInfo>
ResourceRegistry::getLoadersForType(ResourceType type) const {
  std::shared_lock lock(mutex_);

  auto it = loaders_.find(type);
  if (it != loaders_.end()) {
    return it->second;
  }

  return {};
}

// ====================================================================
// UTILITY METHODS
// ====================================================================

std::string ResourceRegistry::normalizeExtension(const std::string &extension) {
  if (extension.empty()) {
    return extension;
  }

  // Ensure extension starts with a dot
  if (extension[0] != '.') {
    return "." + extension;
  }

  return extension;
}

std::string ResourceRegistry::extractExtension(const std::string &filepath) {
  std::filesystem::path path(filepath);
  std::string ext = path.extension().string();

  // Convert to lowercase for case-insensitive comparison
  std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

  return ext;
}

void ResourceRegistry::clear() {
  std::unique_lock lock(mutex_);

  typeInfo_.clear();
  extensionToType_.clear();
  pathAliases_.clear();
  loaders_.clear();
}

void ResourceRegistry::resetToDefaults() {
  clear();
  initializeDefaults();
}

std::string ResourceRegistry::generateReport() const {
  std::shared_lock lock(mutex_);

  std::stringstream report;

  report << "=== Resource Registry Report ===" << std::endl;
  report << std::endl;

  // Resource types
  report << "Registered Resource Types:" << std::endl;
  for (const auto &[type, info] : typeInfo_) {
    report << "  " << std::setw(15) << info.name << ": ";
    report << info.extensions.size() << " extensions";
    if (info.supportsStreaming)
      report << " [Streaming]";
    if (info.supportsCompression)
      report << " [Compression]";
    if (!info.supportsCaching)
      report << " [No-Cache]";
    report << std::endl;

    if (!info.description.empty()) {
      report << "    Description: " << info.description << std::endl;
    }

    report << "    Extensions: ";
    for (size_t i = 0; i < info.extensions.size(); ++i) {
      if (i > 0)
        report << ", ";
      report << info.extensions[i];
    }
    report << std::endl;
  }
  report << std::endl;

  // Path aliases
  report << "Path Aliases:" << std::endl;
  for (const auto &[alias, info] : pathAliases_) {
    report << "  @" << std::setw(12) << alias << " -> " << info.path;
    if (!info.description.empty()) {
      report << " (" << info.description << ")";
    }
    report << std::endl;
  }
  report << std::endl;

  // Statistics
  report << "Statistics:" << std::endl;
  report << "  Total Resource Types: " << typeInfo_.size() << std::endl;
  report << "  Total Extensions: " << extensionToType_.size() << std::endl;
  report << "  Total Path Aliases: " << pathAliases_.size() << std::endl;
  report << "  Total Loaders: " << loaders_.size() << std::endl;

  return report.str();
}

// ====================================================================
// PRIVATE METHODS
// ====================================================================

void ResourceRegistry::initializeDefaults() {
  // Register texture formats
  TypeInfo textureInfo;
  textureInfo.name = "Texture";
  textureInfo.description = "2D/3D image data for rendering";
  textureInfo.extensions = {".png", ".jpg", ".jpeg", ".tga", ".bmp",
                            ".dds", ".ktx", ".ktx2", ".hdr", ".exr"};
  textureInfo.supportsStreaming = true;
  textureInfo.supportsCompression = true;
  textureInfo.supportsCaching = true;
  textureInfo.averageSize = 4 * 1024 * 1024; // 4 MB average
  registerResourceType(ResourceType::TEXTURE, textureInfo);

  // Register mesh formats
  TypeInfo meshInfo;
  meshInfo.name = "Mesh";
  meshInfo.description = "3D geometry data";
  meshInfo.extensions = {".obj", ".fbx", ".gltf", ".glb",
                         ".dae", ".3ds", ".mesh"};
  meshInfo.supportsStreaming = true;
  meshInfo.supportsCompression = true;
  meshInfo.supportsCaching = true;
  meshInfo.averageSize = 2 * 1024 * 1024; // 2 MB average
  registerResourceType(ResourceType::MESH, meshInfo);

  // Register shader formats
  TypeInfo shaderInfo;
  shaderInfo.name = "Shader";
  shaderInfo.description = "GPU shader programs";
  shaderInfo.extensions = {".vert", ".frag", ".geom", ".tesc",   ".tese",
                           ".comp", ".glsl", ".hlsl", ".shader", ".spv"};
  shaderInfo.supportsStreaming = false;
  shaderInfo.supportsCompression = false;
  shaderInfo.supportsCaching = true;
  shaderInfo.averageSize = 64 * 1024; // 64 KB average
  registerResourceType(ResourceType::SHADER, shaderInfo);

  // Register material formats
  TypeInfo materialInfo;
  materialInfo.name = "Material";
  materialInfo.description = "Material definitions and properties";
  materialInfo.extensions = {".mat", ".material", ".mtl"};
  materialInfo.supportsStreaming = false;
  materialInfo.supportsCompression = false;
  materialInfo.supportsCaching = true;
  materialInfo.averageSize = 16 * 1024; // 16 KB average
  registerResourceType(ResourceType::MATERIAL, materialInfo);

  // Register animation formats
  TypeInfo animationInfo;
  animationInfo.name = "Animation";
  animationInfo.description =
      "Animation data for skeletal and property animation";
  animationInfo.extensions = {".anim", ".animation", ".bvh", ".fbx"};
  animationInfo.supportsStreaming = true;
  animationInfo.supportsCompression = true;
  animationInfo.supportsCaching = true;
  animationInfo.averageSize = 512 * 1024; // 512 KB average
  registerResourceType(ResourceType::ANIMATION, animationInfo);

  // Register audio formats
  TypeInfo audioInfo;
  audioInfo.name = "Audio";
  audioInfo.description = "Audio files for sound effects and music";
  audioInfo.extensions = {".wav", ".mp3", ".ogg", ".flac", ".aac", ".m4a"};
  audioInfo.supportsStreaming = true;
  audioInfo.supportsCompression = true;
  audioInfo.supportsCaching = false;       // Audio often streamed
  audioInfo.averageSize = 5 * 1024 * 1024; // 5 MB average
  registerResourceType(ResourceType::AUDIO, audioInfo);

  // Register font formats
  TypeInfo fontInfo;
  fontInfo.name = "Font";
  fontInfo.description = "Font files for text rendering";
  fontInfo.extensions = {".ttf", ".otf", ".fnt", ".fon"};
  fontInfo.supportsStreaming = false;
  fontInfo.supportsCompression = false;
  fontInfo.supportsCaching = true;
  fontInfo.averageSize = 256 * 1024; // 256 KB average
  registerResourceType(ResourceType::FONT, fontInfo);

  // Register script formats
  TypeInfo scriptInfo;
  scriptInfo.name = "Script";
  scriptInfo.description = "Script files for gameplay logic";
  scriptInfo.extensions = {".lua", ".js", ".py", ".cs", ".script"};
  scriptInfo.supportsStreaming = false;
  scriptInfo.supportsCompression = false;
  scriptInfo.supportsCaching = true;
  scriptInfo.averageSize = 32 * 1024; // 32 KB average
  registerResourceType(ResourceType::SCRIPT, scriptInfo);

  // Register level formats
  TypeInfo levelInfo;
  levelInfo.name = "Level";
  levelInfo.description = "Level/scene data files";
  levelInfo.extensions = {".level", ".map", ".scene", ".world"};
  levelInfo.supportsStreaming = true;
  levelInfo.supportsCompression = true;
  levelInfo.supportsCaching = false;        // Levels are usually large
  levelInfo.averageSize = 50 * 1024 * 1024; // 50 MB average
  registerResourceType(ResourceType::LEVEL, levelInfo);

  // Register prefab formats
  TypeInfo prefabInfo;
  prefabInfo.name = "Prefab";
  prefabInfo.description = "Prefabricated game objects";
  prefabInfo.extensions = {".prefab", ".blueprint"};
  prefabInfo.supportsStreaming = false;
  prefabInfo.supportsCompression = true;
  prefabInfo.supportsCaching = true;
  prefabInfo.averageSize = 128 * 1024; // 128 KB average
  registerResourceType(ResourceType::PREFAB, prefabInfo);

  // Register config formats
  TypeInfo configInfo;
  configInfo.name = "Config";
  configInfo.description = "Configuration and data files";
  configInfo.extensions = {".json", ".xml", ".yaml", ".ini", ".cfg", ".config"};
  configInfo.supportsStreaming = false;
  configInfo.supportsCompression = false;
  configInfo.supportsCaching = true;
  configInfo.averageSize = 8 * 1024; // 8 KB average
  registerResourceType(ResourceType::CONFIG, configInfo);

  // Register default path aliases
  registerPath("assets", "assets/", "Main assets directory");
  registerPath("textures", "assets/textures/", "Texture assets");
  registerPath("models", "assets/models/", "3D model assets");
  registerPath("shaders", "assets/shaders/", "Shader programs");
  registerPath("materials", "assets/materials/", "Material definitions");
  registerPath("animations", "assets/animations/", "Animation data");
  registerPath("audio", "assets/audio/", "Audio files");
  registerPath("fonts", "assets/fonts/", "Font files");
  registerPath("scripts", "assets/scripts/", "Script files");
  registerPath("levels", "assets/levels/", "Level data");
  registerPath("prefabs", "assets/prefabs/", "Prefab definitions");
  registerPath("config", "config/", "Configuration files");
  registerPath("cache", "cache/", "Cache directory");
  registerPath("temp", "temp/", "Temporary files");
}

} // namespace engine::resources
