/**
 * @file RaycastSystem.h
 * @brief Advanced raycasting system with caching and batch processing
 * @details Provides optimized raycasting with various shapes, filtering, and callbacks
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

#include "PhysicsQueries.h"

#include <queue>
#include <future>
#include <thread>

namespace engine::physics {
    using namespace engine::math;

    /**
     * @brief Ray definition with additional properties
     */
    struct RayDefinition {
        Vec3 origin;
        Vec3 direction; // Should be normalized
        Float maxDistance = INFINITY_VALUE<Float>;
        Float thickness = 0.0f; // For thick raycasts

        RayDefinition() = default;

        RayDefinition(const Vec3& o, const Vec3& d, const Float dist = INFINITY_VALUE<Float>)
            : origin(o), direction(glm::normalize(d)), maxDistance(dist) {
        }

        Vec3 getPoint(const Float t) const {
            return origin + direction * t;
        }
    };

    /**
     * @brief Extended raycast hit information
     */
    struct RaycastHitExtended : RaycastHit {
        Int shapeIndex = -1; // Sub-shape index for compound shapes
        Int materialId = -1; // Material ID at hit point
        Vec3 barycentricCoord; // Barycentric coordinates on triangle
        Float friction = 0.5f; // Surface friction
        Float restitution = 0.0f; // Surface restitution

        RaycastHitExtended() = default;

        explicit RaycastHitExtended(const RaycastHit& hit) : RaycastHit(hit), barycentricCoord(VEC3_ZERO) {
        }
    };

    /**
     * @brief Batch raycast request
     */
    struct BatchRaycastRequest {
        RayDefinition ray;
        QueryFilter filter;
        std::function<void(const RaycastHitExtended&)> callback;
        void* userData = nullptr;
        Int priority = 0; // Higher priority processed first

        BatchRaycastRequest() = default;

        bool operator<(const BatchRaycastRequest& other) const {
            // priority_queue by default puts largest on top, así que invertimos si quieres "higher priority first"
            return priority < other.priority;
        }
    };

    /**
     * @brief Raycast cache entry
     */
    struct RaycastCacheEntry {
        RayDefinition ray;
        RaycastHitExtended hit;
        bool hasHit;
        std::chrono::steady_clock::time_point timestamp;

        bool isValid(const Float maxAge = 0.1f) const {
            const auto now = std::chrono::steady_clock::now();
            const Float age = std::chrono::duration<Float>(now - timestamp).count();
            return age < maxAge;
        }
    };

    /**
     * @brief Advanced raycast system with optimization features
     */
    class RaycastSystem {
    public:
        explicit RaycastSystem(PhysicsWorld* world, Random& rng);

        ~RaycastSystem();

        // ============================================================================
        // Single Raycasts
        // ============================================================================

        /**
         * @brief Perform a single raycast
         */
        bool raycast(const RayDefinition& ray, RaycastHitExtended& hit,
                     const QueryFilter& filter = QueryFilter());

        /**
         * @brief Perform a thick raycast (capsule sweep)
         */
        bool thickRaycast(const RayDefinition& ray, Float radius,
                          RaycastHitExtended& hit,
                          const QueryFilter& filter = QueryFilter());

        /**
         * @brief Perform cone cast (multiple rays in cone pattern)
         */
        std::vector<RaycastHitExtended> coneCast(const Vec3& origin,
                                                 const Vec3& direction,
                                                 Float angle,
                                                 Float maxDistance,
                                                 Int numRays,
                                                 const QueryFilter& filter = QueryFilter());

        /**
         * @brief Perform scatter cast (random rays from point)
         */
        std::vector<RaycastHitExtended> scatterCast(const Vec3& origin,
                                                    Int numRays,
                                                    Float maxDistance,
                                                    const QueryFilter& filter = QueryFilter());

        // ============================================================================
        // Multiple Raycasts
        // ============================================================================

        /**
         * @brief Get all hits along a ray
         */
        std::vector<RaycastHitExtended> raycastAll(const RayDefinition& ray,
                                                   const QueryFilter& filter = QueryFilter());

        /**
         * @brief Perform penetrating raycast (goes through objects)
         */
        std::vector<RaycastHitExtended> penetratingRaycast(const RayDefinition& ray,
                                                           Int maxHits = 10,
                                                           const QueryFilter& filter = QueryFilter());

        // ============================================================================
        // Batch Processing
        // ============================================================================

        /**
         * @brief Start batch processor threads
         */
        void startBatchProcessor(Int numThreads = 2);

        /**
         * @brief Stop batch processor
         */
        void stopBatchProcessor();

        /**
         * @brief Submit batch raycast request
         */
        void submitBatchRequest(const BatchRaycastRequest& request);

        /**
         * @brief Process all pending batch requests synchronously
         */
        void processPendingBatches();

        // ============================================================================
        // Specialized Queries
        // ============================================================================

        /**
         * @brief Find ground below position
         */
        bool findGround(const Vec3& position, Float maxDistance,
                        RaycastHitExtended& groundHit,
                        const QueryFilter& filter = QueryFilter());

        /**
         * @brief Check line of sight between two points
         */
        bool hasLineOfSight(const Vec3& from, const Vec3& to,
                            const QueryFilter& filter = QueryFilter());

        /**
         * @brief Calculate visibility from point
         */
        Float calculateVisibility(const Vec3& observer, const Vec3& target,
                                  Float targetRadius = 0.5f,
                                  Int numSamples = 8);

        /**
         * @brief Find reflection point
         */
        bool findReflection(const RayDefinition& ray, Int maxBounces,
                            std::vector<RaycastHitExtended>& reflectionPath,
                            const QueryFilter& filter = QueryFilter());

        // ============================================================================
        // Configuration
        // ============================================================================

        void setCacheEnabled(const bool enable) { m_cacheEnabled = enable; }
        void setCacheMaxSize(const std::size_t size) { m_maxCacheSize = size; }
        void clearCache() { m_cache.clear(); }

        Float getCacheHitRate() const {
            const Float total = m_stats.cacheHits + m_stats.cacheMisses;
            return total > 0 ? m_stats.cacheHits / total : 0.0f;
        }

        // ============================================================================
        // Statistics
        // ============================================================================

        struct Statistics {
            std::size_t totalRaycasts = 0;
            std::size_t cacheHits = 0;
            std::size_t cacheMisses = 0;
            std::size_t batchProcessed = 0;
            Float averageRaycastTime = 0.0f;
        };

        const Statistics& getStatistics() const { return m_stats; }
        void resetStatistics() { m_stats = Statistics(); }

    private:
        PhysicsWorld* m_world;
        PhysicsQueries m_queries;

        Random& rng_;

        // Cache system
        bool m_cacheEnabled;
        std::size_t m_maxCacheSize;
        std::unordered_map<std::size_t, RaycastCacheEntry> m_cache;
        Float m_cacheHitRate;

        // Batch processing
        std::atomic<bool> m_batchProcessorRunning{false};
        std::vector<std::thread> m_batchThreads;
        std::priority_queue<BatchRaycastRequest> m_batchQueue;
        std::mutex m_batchMutex;
        std::condition_variable m_batchCondition;

        // Statistics
        Statistics m_stats;

        /**
         * @brief Extract extended information from hit
         */
        static void extractExtendedInfo(RaycastHitExtended& hit, const RayDefinition& ray);

        /**
         * @brief Generate cache key for ray
         */
        static std::size_t generateCacheKey(const RayDefinition& ray, const QueryFilter& filter);

        /**
         * @brief Check cache for ray result
         */
        RaycastCacheEntry* checkCache(const RayDefinition& ray, const QueryFilter& filter);

        /**
         * @brief Update cache with new result
         */
        void updateCache(const RayDefinition& ray, const QueryFilter& filter,
                         const RaycastHitExtended& hit, bool hasHit);

        /**
         * @brief Evict oldest cache entry
         */
        void evictOldestCacheEntry();

        /**
         * @brief Process batch requests in thread
         */
        void processBatchRequests();
    };
} // namespace engine::physics
