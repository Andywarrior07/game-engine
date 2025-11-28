/**
 * @file MathSystem.h
 * @brief Main mathematical system for the game engine
 *
 * Central hub for all mathematical operations, providing a unified interface
 * to core math functionality, geometry, transforms, and spatial queries.
 */

#pragma once

// Core math components
#include "core/FastMath.h"
#include "core/MathConstants.h"
#include "core/MathFunctions.h"
#include "core/MathTypes.h"

// Geometry primitives and operations
#include "geometry/Distance.h"
#include "geometry/Intersections.h"
#include "geometry/Primitives.h"

// Transform system
#include "transform/Transform.h"
#include "transform/TransformHierarchy.h"

// Interpolation and animation
#include "interpolation/Curves.h"
#include "interpolation/Easing.h"
#include "interpolation/Interpolation.h"

// Animation
#include "animation/AnimationMath.h"
#include "animation/DualQuaternion.h"

// Graphics
#include "graphics/ColorSpace.h"
#include "graphics/ProjectionMatrix.h"
#include "graphics/TangentSpace.h"

// Spatial partitioning
#include "spatial/MortonCode.h"
#include "spatial/Octree.h"
#include "spatial/SpatialHash.h"

// Noise generation
#include "noise/PerlinNoise.h"
#include "noise/SimplexNoise.h"
#include "noise/WorleyNoise.h"

// SIMD optimizations
#include "simd/SimdMath.h"
#include "simd/SimdTypes.h"

// Utilities
#include "utils/MathDebug.h"
#include "utils/Random.h"

#include <memory>
#include <unordered_map>

namespace engine::math {
/**
 * @brief Main mathematical system manager
 *
 * Provides centralized initialization, configuration, and management
 * of mathematical subsystems. Integrates with MemoryManager for
 * optimized allocations.
 */
class MathSystem {
public:
  explicit MathSystem(std::unique_ptr<noise::PerlinNoise> perlin,
                      std::unique_ptr<noise::SimplexNoise> simplex,
                      std::unique_ptr<noise::WorleyNoise> worley,
                      std::unique_ptr<TransformHierarchy> transformHierarchy)
      : random_(std::make_unique<Random>()), perlinNoise_(std::move(perlin)),
        simplexNoise_(std::move(simplex)), worleyNoise_(std::move(worley)),
        transformHierarchy_(std::move(transformHierarchy)) {
    initialize();
  };

  ~MathSystem();

  MathSystem(const MathSystem &) = delete;
  MathSystem &operator=(const MathSystem &) = delete;
  MathSystem(MathSystem &&) = default;
  MathSystem &operator=(MathSystem &&) = default;

  // ============================================================================
  // System Management
  // ============================================================================

  /**
   * @brief Initialize the math system
   */
  bool initialize() {
    if (initialized_)
      return true;

    // Initialize noise generators
    // perlinNoise_ = std::make_unique<noise::PerlinNoise>();
    // simplexNoise_ = std::make_unique<noise::SimplexNoise>();
    // worleyNoise_ = std::make_unique<noise::WorleyNoise>();

    // Set up math precision settings
    configurePrecision();

    // Validate SIMD support
    detectSIMDSupport();

    // Initialize spatial systems
    // transformHierarchy_ = std::make_unique<TransformHierarchy>();

    initialized_ = true;
    return true;
  }

  /**
   * @brief Shutdown the math system
   */
  void shutdown() {
    if (!initialized_)
      return;

    // Clean up any allocated resources
    transformPool_.clear();
    perlinNoise_.reset();
    simplexNoise_.reset();
    worleyNoise_.reset();
    transformHierarchy_.reset();

    initialized_ = false;
  }

  /**
   * @brief Check if system is initialized
   */
  [[nodiscard]] bool isInitialized() const noexcept { return initialized_; }

  // ============================================================================
  // Subsystem Access
  // ============================================================================

  [[nodiscard]] TransformHierarchy &getTransformHierarchy() {
    return *transformHierarchy_;
  }

  [[nodiscard]] noise::PerlinNoise &getPerlinNoise() { return *perlinNoise_; }

  [[nodiscard]] noise::SimplexNoise &getSimplexNoise() {
    return *simplexNoise_;
  }

  [[nodiscard]] noise::WorleyNoise &getWorleyNoise() { return *worleyNoise_; }

  // ============================================================================
  // Transform Pool Management
  // ============================================================================

  /**
   * @brief Allocate a new transform from the pool
   */
  Transform *allocateTransform() {
    auto transform = std::make_unique<Transform>();
    Transform *ptr = transform.get();
    transformPool_[ptr] = std::move(transform);
    return ptr;
  }

  /**
   * @brief Deallocate a transform back to the pool
   */
  void deallocateTransform(Transform *transform) {
    if (transform) {
      transformPool_.erase(transform);
    }
  }

  /**
   * @brief Get transform pool size
   */
  [[nodiscard]] std::size_t getTransformPoolSize() const noexcept {
    return transformPool_.size();
  }

  // ============================================================================
  // Configuration
  // ============================================================================

  struct Config {
    bool useFastMath = false;         // Use fast approximations
    bool enableSIMD = true;           // Enable SIMD optimizations
    Float epsilonTolerance = EPSILON; // Global epsilon for comparisons
    bool validateMatrices = true;     // Validate matrix operations in debug
  };

  /**
   * @brief Set math system configuration
   */
  void setConfig(const Config &config) { config_ = config; }

  /**
   * @brief Get current configuration
   */
  [[nodiscard]] const Config &getConfig() const noexcept { return config_; }

  // ============================================================================
  // SIMD Support Detection
  // ============================================================================

  struct SIMDSupport {
    bool sse = false;
    bool sse2 = false;
    bool sse3 = false;
    bool ssse3 = false;
    bool sse41 = false;
    bool sse42 = false;
    bool avx = false;
    bool avx2 = false;
    bool avx512 = false;
  };

  /**
   * @brief Get detected SIMD support
   */
  [[nodiscard]] const SIMDSupport &getSIMDSupport() const noexcept {
    return simdSupport_;
  }

  // ============================================================================
  // Utility Functions
  // ============================================================================

  [[nodiscard]] bool validateMatrix(const Mat4 &matrix) const noexcept {
#ifdef _DEBUG
    if (!config_.validateMatrices)
      return true;

    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        if (std::isnan(matrix[i][j]))
          return false;
        if (std::isinf(matrix[i][j]))
          return false;
      }
    }

    Float det = glm::determinant(matrix);
    if (std::abs(det) < EPSILON)
      return false;

    return true;
#else
    return true;
#endif
  }

  [[nodiscard]] static Mat4 composeTransform(const Vec3 &position,
                                             const Quat &rotation,
                                             const Vec3 &scale) noexcept {
    Mat4 T = glm::translate(Mat4(1.0f), position);
    Mat4 R = glm::mat4_cast(rotation);
    Mat4 S = glm::scale(Mat4(1.0f), scale);
    return T * R * S;
  }

  [[nodiscard]] static bool decomposeTransform(const Mat4 &matrix,
                                               Vec3 &position, Quat &rotation,
                                               Vec3 &scale) noexcept {
    return Transform::decompose(matrix, position, rotation, scale);
  }

  [[nodiscard]] static Mat4 buildViewMatrix(const Vec3 &position,
                                            const Quat &orientation) noexcept {
    Mat4 rotation = glm::mat4_cast(glm::conjugate(orientation));
    Mat4 translation = glm::translate(Mat4(1.0f), -position);
    return rotation * translation;
  }

  [[nodiscard]] static Mat4 buildLookAtMatrix(const Vec3 &eye,
                                              const Vec3 &center,
                                              const Vec3 &up) noexcept {
    return glm::lookAt(eye, center, up);
  }

  [[nodiscard]] static Mat4 buildPerspectiveMatrix(Float fovRadians,
                                                   Float aspectRatio,
                                                   Float nearPlane,
                                                   Float farPlane) noexcept {
    return glm::perspective(fovRadians, aspectRatio, nearPlane, farPlane);
  }

  [[nodiscard]] static Mat4 buildOrthographicMatrix(Float left, Float right,
                                                    Float bottom, Float top,
                                                    Float nearPlane,
                                                    Float farPlane) noexcept {
    return glm::ortho(left, right, bottom, top, nearPlane, farPlane);
  }

  [[nodiscard]] static Mat3
  calculateNormalMatrix(const Mat4 &modelMatrix) noexcept {
    return glm::transpose(glm::inverse(Mat3(modelMatrix)));
  }

  [[nodiscard]] static Mat3 calculateTBNMatrix(const Vec3 &tangent,
                                               const Vec3 &bitangent,
                                               const Vec3 &normal) noexcept {
    return Mat3(tangent, bitangent, normal);
  }

  [[nodiscard]] static Mat4 lookAt(const Vec3 &eye, const Vec3 &center,
                                   const Vec3 &up) noexcept {
    return buildLookAtMatrix(eye, center, up);
  }

  [[nodiscard]] static Mat4 perspective(Float fov, Float aspect,
                                        Float nearPlane,
                                        Float farPlane) noexcept {
    return buildPerspectiveMatrix(fov, aspect, nearPlane, farPlane);
  }

  [[nodiscard]] static Mat4 ortho(Float left, Float right, Float bottom,
                                  Float top, Float nearPlane,
                                  Float farPlane) noexcept {
    return buildOrthographicMatrix(left, right, bottom, top, nearPlane,
                                   farPlane);
  }

private:
  bool initialized_ = false;
  Config config_;
  SIMDSupport simdSupport_;

  std::unordered_map<Transform *, std::unique_ptr<Transform>> transformPool_;

  std::unique_ptr<TransformHierarchy> transformHierarchy_;
  std::unique_ptr<noise::PerlinNoise> perlinNoise_;
  std::unique_ptr<noise::SimplexNoise> simplexNoise_;
  std::unique_ptr<noise::WorleyNoise> worleyNoise_;
  std::unique_ptr<Random> random_;

  void configurePrecision() noexcept {
#ifdef _MSC_VER
    _controlfp(_RC_NEAR, _MCW_RC);
#endif
  }

  void detectSIMDSupport() noexcept {
#ifdef __x86_64__
    int cpuInfo[4];

#ifdef _MSC_VER
    __cpuid(cpuInfo, 1);
// #else
//             __cpuid(1, cpuInfo[0], cpuInfo[1], cpuInfo[2], cpuInfo[3]);
#endif

    simdSupport_.sse = (cpuInfo[3] & (1 << 25)) != 0;
    simdSupport_.sse2 = (cpuInfo[3] & (1 << 26)) != 0;
    simdSupport_.sse3 = (cpuInfo[2] & (1 << 0)) != 0;
    simdSupport_.ssse3 = (cpuInfo[2] & (1 << 9)) != 0;
    simdSupport_.sse41 = (cpuInfo[2] & (1 << 19)) != 0;
    simdSupport_.sse42 = (cpuInfo[2] & (1 << 20)) != 0;
    simdSupport_.avx = (cpuInfo[2] & (1 << 28)) != 0;

#ifdef _MSC_VER
    __cpuid(cpuInfo, 7);
// #else
//             __cpuid_count(7, 0, cpuInfo[0], cpuInfo[1], cpuInfo[2],
//             cpuInfo[3]);
#endif

    simdSupport_.avx2 = (cpuInfo[1] & (1 << 5)) != 0;
    simdSupport_.avx512 = (cpuInfo[1] & (1 << 16)) != 0;
#endif
  }
};
} // namespace engine::math
