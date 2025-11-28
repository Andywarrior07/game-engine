//
// Created by Andres Guerrero on 20-08-25.
//

#include "TextureResource.h"

#include <SDL2/SDL_image.h>
#include <algorithm>
#include <filesystem>
#include <iostream>

namespace engine::resources {
// Static members
std::atomic<bool> TextureResource::sdlImageInitialized_{false};
std::mutex TextureResource::sdlInitMutex_;

// ====================================================================
// CONSTRUCTION / DESTRUCTION
// ====================================================================

TextureResource::TextureResource(const ResourceID id, const std::string &name)
    : StreamableResource(id, name, ResourceType::TEXTURE) {
  desc_ = TextureDesc();

  // Ensure SDL_image is initialized
  ensureSDLImageInitialized();
}

TextureResource::~TextureResource() {
  if (state_ != ResourceState::UNLOADED) {
    TextureResource::unload();
  }
}

// ====================================================================
// SDL INITIALIZATION
// ====================================================================

bool TextureResource::ensureSDLImageInitialized() {
  if (sdlImageInitialized_.load(std::memory_order_acquire)) {
    return true;
  }

  std::lock_guard<std::mutex> lock(sdlInitMutex_);

  // Double-check after acquiring lock
  if (sdlImageInitialized_.load(std::memory_order_acquire)) {
    return true;
  }

  // Initialize SDL_image with all common formats
  constexpr int imgFlags =
      IMG_INIT_PNG | IMG_INIT_JPG | IMG_INIT_TIF | IMG_INIT_WEBP;

  if (const int initialized = IMG_Init(imgFlags);
      (initialized & imgFlags) != imgFlags) {
    std::cerr << "SDL_image initialization failed: " << IMG_GetError()
              << std::endl;
    return false;
  }

  sdlImageInitialized_.store(true, std::memory_order_release);

  std::cout << "SDL_image initialized successfully" << std::endl;
  std::cout << "Supported formats: PNG, JPEG, TIFF, WebP" << std::endl;

  return true;
}

// ====================================================================
// RESOURCE INTERFACE
// ====================================================================

bool TextureResource::load(const std::uint8_t *data, const ResourceSize size) {
  if (!data || size == 0) {
    std::cerr << "TextureResource::load - Invalid data" << std::endl;
    return false;
  }

  std::unique_lock lock(dataMutex_);

  setState(ResourceState::LOADING);

  const bool result = loadFromMemory(data, size);

  if (result) {
    setState(ResourceState::READY);
    currentMemoryUsage_ = desc_.calculateMemorySize();
  } else {
    setState(ResourceState::FAILED);
  }

  return result;
}

bool TextureResource::unload() {
  std::unique_lock lock(dataMutex_);

  // Free all mipmap data
  freeAllMipData();

  // Reset GPU handle
  gpuHandle_ = 0;

  // Reset streaming state
  currentStreamedMip_ = UINT32_MAX;
  requestedStreamMip_ = 0;

  // Update memory usage
  currentMemoryUsage_ = 0;

  setState(ResourceState::UNLOADED);

  return true;
}

void TextureResource::reload() {
  Resource::reload();

  if (gpuHandle_ != 0) {
    // Mark GPU resource as needing update
    // This would be handled by the renderer
  }
}

ResourceSize TextureResource::getMemoryUsage() const {
  return currentMemoryUsage_.load(std::memory_order_acquire);
}

bool TextureResource::validate() const {
  std::shared_lock lock(dataMutex_);

  if (!desc_.validate()) {
    return false;
  }

  if (mipData_.empty()) {
    return false;
  }

  for (const auto &mip : mipData_) {
    if (mip.data && mip.size == 0) {
      return false;
    }
  }

  return state_ == ResourceState::READY;
}

// ====================================================================
// LOADING IMPLEMENTATION
// ====================================================================

bool TextureResource::loadFromMemory(const std::uint8_t *data,
                                     ResourceSize size) {
  // Create SDL_RWops from memory
  SDL_RWops *rwops = SDL_RWFromConstMem(data, static_cast<int>(size));
  if (!rwops) {
    std::cerr << "Failed to create SDL_RWops from memory" << std::endl;
    return false;
  }

  // Load image using SDL_image
  SDL_Surface *surface = IMG_Load_RW(rwops, 1); // 1 = free rwops

  if (!surface) {
    std::cerr << "Failed to load image: " << IMG_GetError() << std::endl;
    return false;
  }

  // Convert to RGBA32 for consistency
  SDL_Surface *rgbaSurface = convertToRGBA32(surface);
  SDL_FreeSurface(surface);

  if (!rgbaSurface) {
    std::cerr << "Failed to convert image to RGBA32" << std::endl;
    return false;
  }

  // Load from SDL surface
  bool result = loadFromSDLSurface(rgbaSurface);

  // Keep the surface for potential manipulation
  if (result && !mipData_.empty()) {
    mipData_[0].surface = rgbaSurface;
  } else {
    SDL_FreeSurface(rgbaSurface);
  }

  return result;
}

bool TextureResource::loadFromFile(const std::string &path) {
  if (!std::filesystem::exists(path)) {
    std::cerr << "File not found: " << path << std::endl;
    return false;
  }

  // Load image using SDL_image
  SDL_Surface *surface = IMG_Load(path.c_str());

  if (!surface) {
    std::cerr << "Failed to load image: " << IMG_GetError() << std::endl;
    return false;
  }

  // Convert to RGBA32
  SDL_Surface *rgbaSurface = convertToRGBA32(surface);
  SDL_FreeSurface(surface);

  if (!rgbaSurface) {
    return false;
  }

  bool result = loadFromSDLSurface(rgbaSurface);

  if (result && !mipData_.empty()) {
    mipData_[0].surface = rgbaSurface;
  } else {
    SDL_FreeSurface(rgbaSurface);
  }

  return result;
}

bool TextureResource::loadFromSDLSurface(SDL_Surface *surface) {
  if (!surface) {
    return false;
  }

  // Update description
  desc_.type = TextureType::TEXTURE_2D;
  desc_.format = detectFormatFromSDL(surface);
  desc_.width = surface->w;
  desc_.height = surface->h;
  desc_.depth = 1;
  desc_.arraySize = 1;
  desc_.mipLevels = 1; // Start with one mip level

  // Allocate mip data
  mipData_.resize(1);

  // Convert SDL surface to internal format
  if (!convertSDLToInternalFormat(surface, 0, 0)) {
    return false;
  }

  // Set as fully loaded
  currentStreamedMip_ = 0;

  return true;
}

// ====================================================================
// FORMAT DETECTION AND CONVERSION
// ====================================================================

TextureFormat TextureResource::detectFormatFromSDL(SDL_Surface *surface) const {
  if (!surface || !surface->format) {
    return TextureFormat::UNKNOWN;
  }

  // For now, we always convert to RGBA8
  // In the future, could detect and preserve original format
  return TextureFormat::RGBA8_UNORM;
}

SDL_Surface *TextureResource::convertToRGBA32(SDL_Surface *source) const {
  if (!source) {
    return nullptr;
  }

  // Define RGBA32 format
  Uint32 rmask, gmask, bmask, amask;

#if SDL_BYTEORDER == SDL_BIG_ENDIAN
  rmask = 0xff000000;
  gmask = 0x00ff0000;
  bmask = 0x0000ff00;
  amask = 0x000000ff;
#else
  rmask = 0x000000ff;
  gmask = 0x0000ff00;
  bmask = 0x00ff0000;
  amask = 0xff000000;
#endif

  SDL_Surface *converted = SDL_CreateRGBSurface(0, // flags
                                                source->w, source->h,
                                                32, // depth
                                                rmask, gmask, bmask, amask);

  if (!converted) {
    std::cerr << "Failed to create RGBA surface: " << SDL_GetError()
              << std::endl;
    return nullptr;
  }

  // Convert the surface
  if (SDL_BlitSurface(source, nullptr, converted, nullptr) != 0) {
    std::cerr << "Failed to convert surface: " << SDL_GetError() << std::endl;
    SDL_FreeSurface(converted);
    return nullptr;
  }

  return converted;
}

bool TextureResource::convertSDLToInternalFormat(SDL_Surface *surface,
                                                 std::uint32_t mipLevel,
                                                 std::uint32_t arrayIndex) {
  if (!surface) {
    return false;
  }

  std::size_t index = calculateMipIndex(mipLevel, arrayIndex);
  if (index >= mipData_.size()) {
    mipData_.resize(index + 1);
  }

  // Lock surface if needed
  if (SDL_MUSTLOCK(surface)) {
    SDL_LockSurface(surface);
  }

  // Calculate data size
  std::size_t dataSize = surface->w * surface->h * 4; // RGBA

  // Allocate internal storage
  mipData_[index].data = std::make_unique<std::uint8_t[]>(dataSize);
  mipData_[index].size = dataSize;
  mipData_[index].width = surface->w;
  mipData_[index].height = surface->h;
  mipData_[index].depth = 1;
  mipData_[index].rowPitch = surface->pitch;
  mipData_[index].slicePitch = dataSize;

  // Copy pixel data
  memcpy(mipData_[index].data.get(), surface->pixels, dataSize);

  // Unlock surface
  if (SDL_MUSTLOCK(surface)) {
    SDL_UnlockSurface(surface);
  }

  return true;
}

// ====================================================================
// STREAMING INTERFACE
// ====================================================================

bool TextureResource::streamIn(const std::uint32_t lodLevel) {
  std::unique_lock lock(dataMutex_);

  if (lodLevel >= desc_.mipLevels) {
    return false;
  }

  if (currentStreamedMip_ <= lodLevel) {
    return true; // Already streamed
  }

  setStreamingState(StreamingState::STREAMING_IN);

  // Load mip levels from the highest detail to requested LOD
  for (std::uint32_t mip = lodLevel; mip < desc_.mipLevels; ++mip) {
    if (currentStreamedMip_ == UINT32_MAX || mip < currentStreamedMip_) {
      for (std::uint32_t array = 0; array < desc_.arraySize; ++array) {
        allocateMipData(mip, array);
        // In production, load from disk or compressed data
      }
    }
  }

  currentStreamedMip_ = lodLevel;
  setStreamingState(StreamingState::RESIDENT);

  currentMemoryUsage_ = getStreamedMemoryUsage();

  return true;
}

bool TextureResource::streamOut(std::uint32_t lodLevel) {
  std::unique_lock lock(dataMutex_);

  if (lodLevel >= desc_.mipLevels) {
    lodLevel = desc_.mipLevels - 1;
  }

  if (currentStreamedMip_ >= lodLevel) {
    return true; // Already at or below requested level
  }

  setStreamingState(StreamingState::STREAMING_OUT);

  // Free mip levels below the requested LOD
  for (std::uint32_t mip = 0; mip < lodLevel && mip < currentStreamedMip_;
       ++mip) {
    for (std::uint32_t array = 0; array < desc_.arraySize; ++array) {
      freeMipData(mip, array);
    }
  }

  currentStreamedMip_ = lodLevel;

  if (currentStreamedMip_ == UINT32_MAX) {
    setStreamingState(StreamingState::NOT_STREAMING);
  } else {
    setStreamingState(StreamingState::PARTIALLY_LOADED);
  }

  currentMemoryUsage_ = getStreamedMemoryUsage();

  return true;
}

ResourceSize TextureResource::getStreamedMemoryUsage() const {
  std::shared_lock lock(dataMutex_);

  if (currentStreamedMip_ == UINT32_MAX) {
    return 0;
  }

  ResourceSize totalSize = 0;

  for (std::uint32_t mip = currentStreamedMip_; mip < desc_.mipLevels; ++mip) {
    for (std::uint32_t array = 0; array < desc_.arraySize; ++array) {
      std::size_t index = calculateMipIndex(mip, array);
      if (index < mipData_.size() && mipData_[index].data) {
        totalSize += mipData_[index].size;
      }
    }
  }

  return totalSize;
}

// ====================================================================
// SERIALIZATION
// ====================================================================

bool TextureResource::serialize(std::vector<std::uint8_t> &buffer) const {
  // For now, use PNG format for serialization
  std::shared_lock lock(dataMutex_);

  if (mipData_.empty() || !mipData_[0].surface) {
    return false;
  }

  // Use SDL_image to save to memory
  // This would require SDL_image 2.0.5+ for IMG_SavePNG_RW
  // For now, return false as placeholder

  return false;
}

bool TextureResource::deserialize(const std::uint8_t *data, ResourceSize size) {
  return load(data, size);
}

// ====================================================================
// TEXTURE-SPECIFIC METHODS
// ====================================================================

std::uint32_t TextureResource::getWidth(std::uint32_t mipLevel) const {
  if (mipLevel >= desc_.mipLevels) {
    return 0;
  }
  return std::max(1u, desc_.width >> mipLevel);
}

std::uint32_t TextureResource::getHeight(std::uint32_t mipLevel) const {
  if (mipLevel >= desc_.mipLevels) {
    return 0;
  }
  return std::max(1u, desc_.height >> mipLevel);
}

std::uint32_t TextureResource::getDepth(std::uint32_t mipLevel) const {
  if (mipLevel >= desc_.mipLevels) {
    return 0;
  }
  return std::max(1u, desc_.depth >> mipLevel);
}

const std::uint8_t *
TextureResource::getMipData(std::uint32_t mipLevel,
                            std::uint32_t arrayIndex) const {
  std::shared_lock lock(dataMutex_);

  if (!validateMipLevel(mipLevel) || !validateArrayIndex(arrayIndex)) {
    return nullptr;
  }

  std::size_t index = calculateMipIndex(mipLevel, arrayIndex);
  if (index >= mipData_.size()) {
    return nullptr;
  }

  return mipData_[index].data.get();
}

std::size_t TextureResource::getMipDataSize(std::uint32_t mipLevel) const {
  std::shared_lock lock(dataMutex_);

  if (!validateMipLevel(mipLevel)) {
    return 0;
  }

  std::size_t index = calculateMipIndex(mipLevel, 0);
  if (index >= mipData_.size()) {
    return 0;
  }

  return mipData_[index].size;
}

// const std::uint8_t* TextureResource::getCubemapFaceData(CubemapFace face,
// std::uint32_t mipLevel) const {
//     if (desc_.type != TextureType::TEXTURE_CUBE) {
//         return nullptr;
//     }
//
//     std::uint32_t faceIndex = static_cast<std::uint32_t>(face);
//     return getMipData(mipLevel, faceIndex);
// }

// ====================================================================
// PIXEL MANIPULATION
// ====================================================================

bool TextureResource::getPixel(std::uint32_t x, std::uint32_t y,
                               float *outColor, std::uint32_t mipLevel,
                               std::uint32_t arrayIndex) const {
  std::shared_lock lock(dataMutex_);

  if (!outColor || TextureFormatUtils::isCompressed(desc_.format)) {
    return false;
  }

  std::uint32_t mipWidth = getWidth(mipLevel);
  std::uint32_t mipHeight = getHeight(mipLevel);

  if (x >= mipWidth || y >= mipHeight) {
    return false;
  }

  const std::uint8_t *data = getMipData(mipLevel, arrayIndex);
  if (!data) {
    return false;
  }

  // Assuming RGBA8 format
  std::size_t pixelOffset = (y * mipWidth + x) * 4;
  const std::uint8_t *pixelData = data + pixelOffset;

  outColor[0] = pixelData[0] / 255.0f;
  outColor[1] = pixelData[1] / 255.0f;
  outColor[2] = pixelData[2] / 255.0f;
  outColor[3] = pixelData[3] / 255.0f;

  return true;
}

bool TextureResource::setPixel(std::uint32_t x, std::uint32_t y,
                               const float *color, std::uint32_t mipLevel,
                               std::uint32_t arrayIndex) {
  std::unique_lock lock(dataMutex_);

  if (!color || TextureFormatUtils::isCompressed(desc_.format)) {
    return false;
  }

  if (hasUsage(desc_.usage, TextureUsage::IMMUTABLE)) {
    return false;
  }

  std::uint32_t mipWidth = getWidth(mipLevel);
  std::uint32_t mipHeight = getHeight(mipLevel);

  if (x >= mipWidth || y >= mipHeight) {
    return false;
  }

  std::size_t index = calculateMipIndex(mipLevel, arrayIndex);
  if (index >= mipData_.size() || !mipData_[index].data) {
    return false;
  }

  std::uint8_t *data = mipData_[index].data.get();

  // Assuming RGBA8 format
  std::size_t pixelOffset = (y * mipWidth + x) * 4;
  std::uint8_t *pixelData = data + pixelOffset;

  pixelData[0] =
      static_cast<std::uint8_t>(std::clamp(color[0] * 255.0f, 0.0f, 255.0f));
  pixelData[1] =
      static_cast<std::uint8_t>(std::clamp(color[1] * 255.0f, 0.0f, 255.0f));
  pixelData[2] =
      static_cast<std::uint8_t>(std::clamp(color[2] * 255.0f, 0.0f, 255.0f));
  pixelData[3] =
      static_cast<std::uint8_t>(std::clamp(color[3] * 255.0f, 0.0f, 255.0f));

  // Update SDL surface if it exists
  if (index < mipData_.size() && mipData_[index].surface) {
    // Update the surface pixel data too
    SDL_LockSurface(mipData_[index].surface);
    std::uint8_t *surfacePixels =
        static_cast<std::uint8_t *>(mipData_[index].surface->pixels);
    memcpy(surfacePixels + pixelOffset, pixelData, 4);
    SDL_UnlockSurface(mipData_[index].surface);
  }

  return true;
}

// ====================================================================
// MIPMAP GENERATION
// ====================================================================

bool TextureResource::generateMipmaps() {
  std::unique_lock lock(dataMutex_);

  if (TextureFormatUtils::isCompressed(desc_.format)) {
    return false;
  }

  // Ensure we have the base level
  if (mipData_.empty() || !mipData_[0].data) {
    return false;
  }

  // Calculate number of mip levels
  std::uint32_t maxMips =
      TextureFormatUtils::calculateMipLevels(desc_.width, desc_.height);

  // Resize mip data array
  if (mipData_.size() < maxMips * desc_.arraySize) {
    mipData_.resize(maxMips * desc_.arraySize);
  }

  // Generate each mip level using SDL
  for (std::uint32_t array = 0; array < desc_.arraySize; ++array) {
    SDL_Surface *currentLevel = nullptr;

    // Get or create base level surface
    std::size_t baseIndex = calculateMipIndex(0, array);
    if (mipData_[baseIndex].surface) {
      currentLevel = mipData_[baseIndex].surface;
    } else {
      // Create surface from raw data
      currentLevel = SDL_CreateRGBSurfaceFrom(
          mipData_[baseIndex].data.get(), mipData_[baseIndex].width,
          mipData_[baseIndex].height,
          32,                            // bits per pixel
          mipData_[baseIndex].width * 4, // pitch
          0x000000ff,                    // Rmask
          0x0000ff00,                    // Gmask
          0x00ff0000,                    // Bmask
          0xff000000                     // Amask
      );

      if (!currentLevel) {
        return false;
      }

      mipData_[baseIndex].surface = currentLevel;
    }

    // Generate each subsequent mip level
    for (std::uint32_t mip = 1; mip < maxMips; ++mip) {
      std::uint32_t mipWidth = std::max(1u, desc_.width >> mip);
      std::uint32_t mipHeight = std::max(1u, desc_.height >> mip);

      SDL_Surface *nextLevel =
          generateMipLevel(currentLevel, mipWidth, mipHeight);
      if (!nextLevel) {
        return false;
      }

      // Store the mip level
      std::size_t mipIndex = calculateMipIndex(mip, array);
      convertSDLToInternalFormat(nextLevel, mip, array);
      mipData_[mipIndex].surface = nextLevel;

      // Don't free the surface, keep it for potential future use
    }
  }

  desc_.mipLevels = maxMips;
  currentMemoryUsage_ = desc_.calculateMemorySize();

  return true;
}

SDL_Surface *TextureResource::generateMipLevel(SDL_Surface *source,
                                               std::uint32_t targetWidth,
                                               std::uint32_t targetHeight) {
  if (!source) {
    return nullptr;
  }

  // Create target surface
  SDL_Surface *target = SDL_CreateRGBSurface(
      0, targetWidth, targetHeight, source->format->BitsPerPixel,
      source->format->Rmask, source->format->Gmask, source->format->Bmask,
      source->format->Amask);

  if (!target) {
    return nullptr;
  }

  // Use SDL's built-in scaling (bilinear filtering)
  if (SDL_BlitScaled(source, nullptr, target, nullptr) != 0) {
    SDL_FreeSurface(target);
    return nullptr;
  }

  return target;
}

// ====================================================================
// FORMAT CONVERSION
// ====================================================================

bool TextureResource::convertFormat(TextureFormat newFormat) {
  // SDL_image primarily works with RGBA formats
  // For other formats, would need additional conversion libraries

  if (newFormat == desc_.format) {
    return true;
  }

  // Currently only support RGBA8
  if (newFormat != TextureFormat::RGBA8_UNORM) {
    std::cerr << "Format conversion not supported for target format"
              << std::endl;
    return false;
  }

  return true;
}

// ====================================================================
// IMAGE MANIPULATION
// ====================================================================

bool TextureResource::resize(std::uint32_t newWidth, std::uint32_t newHeight) {
  std::unique_lock lock(dataMutex_);

  if (TextureFormatUtils::isCompressed(desc_.format)) {
    return false;
  }

  for (std::uint32_t array = 0; array < desc_.arraySize; ++array) {
    std::size_t srcIndex = calculateMipIndex(0, array);
    if (srcIndex >= mipData_.size() || !mipData_[srcIndex].data) {
      continue;
    }

    // Get or create SDL surface
    SDL_Surface *srcSurface = mipData_[srcIndex].surface;
    if (!srcSurface) {
      srcSurface = SDL_CreateRGBSurfaceFrom(
          mipData_[srcIndex].data.get(), mipData_[srcIndex].width,
          mipData_[srcIndex].height, 32, mipData_[srcIndex].width * 4,
          0x000000ff, 0x0000ff00, 0x00ff0000, 0xff000000);

      if (!srcSurface) {
        return false;
      }
    }

    // Create resized surface
    SDL_Surface *resized =
        SDL_CreateRGBSurface(0, newWidth, newHeight, 32, 0x000000ff, 0x0000ff00,
                             0x00ff0000, 0xff000000);

    if (!resized) {
      if (!mipData_[srcIndex].surface) {
        SDL_FreeSurface(srcSurface);
      }
      return false;
    }

    // Scale the surface
    if (SDL_BlitScaled(srcSurface, nullptr, resized, nullptr) != 0) {
      SDL_FreeSurface(resized);
      if (!mipData_[srcIndex].surface) {
        SDL_FreeSurface(srcSurface);
      }
      return false;
    }

    // Clean up old surface if we created it
    if (!mipData_[srcIndex].surface) {
      SDL_FreeSurface(srcSurface);
    } else {
      SDL_FreeSurface(mipData_[srcIndex].surface);
    }

    // Store new surface and data
    mipData_[srcIndex].surface = resized;
    convertSDLToInternalFormat(resized, 0, array);
  }

  // Update description
  desc_.width = newWidth;
  desc_.height = newHeight;
  desc_.mipLevels = 1; // Reset to single mip

  currentMemoryUsage_ = desc_.calculateMemorySize();

  return true;
}

bool TextureResource::flipVertical() {
  std::unique_lock lock(dataMutex_);

  for (std::uint32_t mip = 0; mip < desc_.mipLevels; ++mip) {
    for (std::uint32_t array = 0; array < desc_.arraySize; ++array) {
      std::size_t index = calculateMipIndex(mip, array);
      if (index >= mipData_.size() || !mipData_[index].data) {
        continue;
      }

      std::uint32_t width = mipData_[index].width;
      std::uint32_t height = mipData_[index].height;
      std::uint32_t rowSize = width * 4; // RGBA

      std::unique_ptr<std::uint8_t[]> tempRow =
          std::make_unique<std::uint8_t[]>(rowSize);
      std::uint8_t *data = mipData_[index].data.get();

      // Swap rows
      for (std::uint32_t y = 0; y < height / 2; ++y) {
        std::uint8_t *topRow = data + y * rowSize;
        std::uint8_t *bottomRow = data + (height - 1 - y) * rowSize;

        memcpy(tempRow.get(), topRow, rowSize);
        memcpy(topRow, bottomRow, rowSize);
        memcpy(bottomRow, tempRow.get(), rowSize);
      }

      // Update SDL surface if it exists
      if (mipData_[index].surface) {
        SDL_LockSurface(mipData_[index].surface);
        memcpy(mipData_[index].surface->pixels, data, mipData_[index].size);
        SDL_UnlockSurface(mipData_[index].surface);
      }
    }
  }

  return true;
}

bool TextureResource::flipHorizontal() {
  std::unique_lock lock(dataMutex_);

  for (std::uint32_t mip = 0; mip < desc_.mipLevels; ++mip) {
    for (std::uint32_t array = 0; array < desc_.arraySize; ++array) {
      std::size_t index = calculateMipIndex(mip, array);
      if (index >= mipData_.size() || !mipData_[index].data) {
        continue;
      }

      std::uint32_t width = mipData_[index].width;
      std::uint32_t height = mipData_[index].height;
      std::uint8_t *data = mipData_[index].data.get();

      // Flip each row horizontally
      for (std::uint32_t y = 0; y < height; ++y) {
        std::uint8_t *row = data + y * width * 4;

        for (std::uint32_t x = 0; x < width / 2; ++x) {
          std::uint8_t *leftPixel = row + x * 4;
          std::uint8_t *rightPixel = row + (width - 1 - x) * 4;

          // Swap pixels (RGBA)
          for (int i = 0; i < 4; ++i) {
            std::swap(leftPixel[i], rightPixel[i]);
          }
        }
      }

      // Update SDL surface if it exists
      if (mipData_[index].surface) {
        SDL_LockSurface(mipData_[index].surface);
        memcpy(mipData_[index].surface->pixels, data, mipData_[index].size);
        SDL_UnlockSurface(mipData_[index].surface);
      }
    }
  }

  return true;
}

// ====================================================================
// SDL SURFACE ACCESS
// ====================================================================

SDL_Surface *TextureResource::getSDLSurface(std::uint32_t mipLevel,
                                            std::uint32_t arrayIndex) {
  std::unique_lock lock(dataMutex_);

  std::size_t index = calculateMipIndex(mipLevel, arrayIndex);
  if (index >= mipData_.size()) {
    return nullptr;
  }

  // Create surface if it doesn't exist but we have data
  if (!mipData_[index].surface && mipData_[index].data) {
    mipData_[index].surface = SDL_CreateRGBSurfaceFrom(
        mipData_[index].data.get(), mipData_[index].width,
        mipData_[index].height, 32, mipData_[index].width * 4, 0x000000ff,
        0x0000ff00, 0x00ff0000, 0xff000000);
  }

  return mipData_[index].surface;
}

const SDL_Surface *
TextureResource::getSDLSurface(std::uint32_t mipLevel,
                               std::uint32_t arrayIndex) const {
  std::shared_lock lock(dataMutex_);

  std::size_t index = calculateMipIndex(mipLevel, arrayIndex);
  if (index >= mipData_.size()) {
    return nullptr;
  }

  return mipData_[index].surface;
}

// ====================================================================
// HELPER METHODS
// ====================================================================

void TextureResource::allocateMipData(std::uint32_t mipLevel,
                                      std::uint32_t arrayIndex) {
  std::size_t index = calculateMipIndex(mipLevel, arrayIndex);

  if (index >= mipData_.size()) {
    mipData_.resize(index + 1);
  }

  std::uint32_t mipWidth = getWidth(mipLevel);
  std::uint32_t mipHeight = getHeight(mipLevel);
  std::uint32_t mipDepth = getDepth(mipLevel);

  std::size_t mipSize;
  if (TextureFormatUtils::isCompressed(desc_.format)) {
    std::uint32_t blockSize = TextureFormatUtils::getBlockSize(desc_.format);
    std::uint32_t blocksX = (mipWidth + 3) / 4;
    std::uint32_t blocksY = (mipHeight + 3) / 4;
    mipSize = blocksX * blocksY * blockSize * mipDepth;
  } else {
    std::uint32_t bpp = TextureFormatUtils::getBytesPerPixel(desc_.format);
    mipSize = mipWidth * mipHeight * mipDepth * bpp;
  }

  mipData_[index].data = std::make_unique<std::uint8_t[]>(mipSize);
  mipData_[index].size = mipSize;
  mipData_[index].width = mipWidth;
  mipData_[index].height = mipHeight;
  mipData_[index].depth = mipDepth;

  if (!TextureFormatUtils::isCompressed(desc_.format)) {
    std::uint32_t bpp = TextureFormatUtils::getBytesPerPixel(desc_.format);
    mipData_[index].rowPitch = mipWidth * bpp;
    mipData_[index].slicePitch = mipWidth * mipHeight * bpp;
  }
}

void TextureResource::freeMipData(std::uint32_t mipLevel,
                                  std::uint32_t arrayIndex) {
  std::size_t index = calculateMipIndex(mipLevel, arrayIndex);

  if (index < mipData_.size()) {
    if (mipData_[index].surface) {
      SDL_FreeSurface(mipData_[index].surface);
      mipData_[index].surface = nullptr;
    }
    mipData_[index].data.reset();
    mipData_[index].size = 0;
  }
}

void TextureResource::freeAllMipData() {
  for (auto &mip : mipData_) {
    if (mip.surface) {
      SDL_FreeSurface(mip.surface);
      mip.surface = nullptr;
    }
    mip.data.reset();
    mip.size = 0;
  }
  mipData_.clear();
}

std::size_t TextureResource::calculateMipIndex(std::uint32_t mipLevel,
                                               std::uint32_t arrayIndex) const {
  return arrayIndex * desc_.mipLevels + mipLevel;
}

bool TextureResource::validateMipLevel(std::uint32_t mipLevel) const {
  return mipLevel < desc_.mipLevels;
}

bool TextureResource::validateArrayIndex(std::uint32_t arrayIndex) const {
  return arrayIndex < desc_.arraySize;
}
} // namespace engine::resources
