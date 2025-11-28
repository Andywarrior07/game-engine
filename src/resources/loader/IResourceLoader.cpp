//
// Created by Andres Guerrero on 17-08-25.
//

#include "IResourceLoader.h"

#include <cstring>
#include <filesystem>
#include <fstream>

namespace engine::resources {
bool FileSystemLoader::canLoad(const std::string &path,
                               ResourceType type) const {
  // Check if a file exists
  return std::filesystem::exists(path);
}

LoadResult FileSystemLoader::load(const ResourceDataSource &source) {
  LoadResult result;

  try {
    // Validación mejorada de entrada
    if (source.path.empty() && !source.data) {
      result.error =
          "No valid data source provided: both path and data are empty";
      return result;
    }

    if (source.data && source.size == 0) {
      result.error =
          "Invalid data source: data pointer provided but size is zero";
      return result;
    }

    if (source.data) {
      result = loadFromMemory(source.data, source.size);
    } else {
      result = loadFromFile(source.path);
    }

    if (!result.success) {
      return result;
    }

    // Handle compression if needed
    if (source.isCompressed) {
      // Decompress data based on compression type
      // This would integrate with a compression library
      // For now, we'll leave it as a placeholder
    }
  } catch (const std::exception &e) {
    result.success = false;
    result.error = std::string("Exception during load: ") + e.what();
  }

  return result;
}

std::vector<std::string> FileSystemLoader::getSupportedExtensions() const {
  // This loader supports all extensions as it's the generic file loader
  return {"*"};
}

LoadResult FileSystemLoader::loadFromFile(const std::string &path) {
  LoadResult result;

  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file.is_open()) {
    result.error = "Failed to open file: " + path;
    return result;
  }

  // Get file size
  result.size = static_cast<ResourceSize>(file.tellg());
  file.seekg(0, std::ios::beg);

  // Allocate memory
  result.data = std::make_unique<std::uint8_t[]>(result.size);

  // Read file
  file.read(reinterpret_cast<char *>(result.data.get()), result.size);

  if (!file) {
    result.success = false;
    result.error = "Failed to read file: " + path;
    result.data.reset();
    result.size = 0;

    return result;
  }

  result.success = true;

  result.metadata.path = path;
  result.metadata.diskSize = result.size;

  auto fsPath = std::filesystem::path(path);
  if (std::filesystem::exists(fsPath)) {
    auto fileTime = std::filesystem::last_write_time(fsPath);
    result.metadata.modificationTime =
        std::chrono::system_clock::now(); // Simplified conversion
  }

  return result;
}

LoadResult FileSystemLoader::loadFromMemory(const std::uint8_t *data,
                                            const ResourceSize size) {
  LoadResult result;

  try {
    static_assert(std::is_unsigned_v<ResourceSize>,
                  "ResourceSize should be unsigned");

    // Validación de parámetros de entrada
    if (data == nullptr) {
      result.success = false;
      result.error = "Invalid memory source: null data pointer";
      return result;
    }

    if (size == 0) {
      result.success = false;
      result.error = "Invalid memory source: zero size";
      return result;
    }

    // Verificar límites de tamaño para evitar problemas de memoria
    if (constexpr ResourceSize MAX_MEMORY_SIZE = 1024 * 1024 * 1024;
        size > MAX_MEMORY_SIZE) {
      result.success = false;
      result.error =
          "Memory source too large: " + std::to_string(size) + " bytes";
      return result;
    }

    // Reservar memoria de manera segura
    try {
      result.data = std::make_unique<std::uint8_t[]>(size);
    } catch (const std::bad_alloc &e) {
      result.success = false;
      result.error =
          "Failed to allocate memory for resource: " + std::string(e.what());
      return result;
    }

    // Copiar datos de manera segura
    std::memcpy(result.data.get(), data, size);

    // Configurar resultado exitoso
    result.size = size;
    result.success = true;
    result.metadata.memorySize = size;

  } catch (const std::exception &e) {
    result.success = false;
    result.error = "Exception during memory load: " + std::string(e.what());
    result.data.reset(); // Limpiar memoria parcialmente asignada
  } catch (...) {
    result.success = false;
    result.error = "Unknown exception during memory load";
    result.data.reset();
  }

  return result;
}
} // namespace engine::resources
