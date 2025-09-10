/**
 * @file RaycastSystem.cpp
 * @brief Advanced raycasting system with caching and batch processing
 * @details Provides optimized raycasting with various shapes, filtering, and callbacks
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#include "RaycastSystem.h"

namespace engine::physics {
    RaycastSystem::RaycastSystem(PhysicsWorld* world, Random& rng)
        : m_world(world), m_queries(world), rng_(rng), m_cacheEnabled(true),
          m_maxCacheSize(1024), m_cacheHitRate(0.0f) {
        m_cache.reserve(m_maxCacheSize);
    }

    RaycastSystem::~RaycastSystem() {
        stopBatchProcessor();
    }

    bool RaycastSystem::raycast(const RayDefinition& ray, RaycastHitExtended& hit,
                                const QueryFilter& filter) {
        // Check cache first
        if (m_cacheEnabled) {
            if (const auto cacheHit = checkCache(ray, filter)) {
                hit = cacheHit->hit;
                m_stats.cacheHits++;
                return cacheHit->hasHit;
            }

            m_stats.cacheMisses++;
        }

        // Perform actual raycast
        const Vec3 to = ray.origin + ray.direction * ray.maxDistance;
        RaycastHit basicHit;
        const bool hasHit = m_queries.raycast(ray.origin, to, basicHit, filter);

        // Convert to extended hit
        hit = RaycastHitExtended(basicHit);
        if (hasHit) {
            extractExtendedInfo(hit, ray);
        }

        // Update cache
        if (m_cacheEnabled) {
            updateCache(ray, filter, hit, hasHit);
        }

        m_stats.totalRaycasts++;
        return hasHit;
    }

    bool RaycastSystem::thickRaycast(const RayDefinition& ray, const Float radius,
                                     RaycastHitExtended& hit,
                                     const QueryFilter& filter) {
        if (radius <= 0.0f) {
            return raycast(ray, hit, filter);
        }

        // Perform capsule sweep
        const Vec3 to = ray.origin + ray.direction * ray.maxDistance;
        Vec3 delta = to - ray.origin;
        const float sweepLength = delta.length();

        SweepResult sweep;
        const bool hasHit = m_queries.sweepSphere(ray.origin, to, radius, sweep, filter);

        if (hasHit) {
            hit.body = sweep.body;
            hit.point = sweep.point;
            hit.normal = sweep.normal;
            hit.distance = sweep.fraction * sweepLength;
            extractExtendedInfo(hit, ray);
        }

        return hasHit;
    }

    std::vector<RaycastHitExtended> RaycastSystem::coneCast(const Vec3& origin,
                                                            const Vec3& direction,
                                                            Float angle,
                                                            Float maxDistance,
                                                            Int numRays,
                                                            const QueryFilter& filter) {
        std::vector<RaycastHitExtended> hits;
        hits.reserve(numRays);

        // Generate rays in cone pattern
        Vec3 forward = glm::normalize(direction);
        Vec3 right, up;
        buildOrthonormalBasis(forward, right, up);

        for (Int i = 0; i < numRays; ++i) {
            Float t = static_cast<Float>(i) / (numRays - 1);
            Float phi = t * TWO_PI<Float>;
            Float theta = angle * std::sqrt(rng_.range(0.0f, 1.0f));

            Vec3 rayDir = forward * std::cos(theta) +
                (right * std::cos(phi) + up * std::sin(phi)) * std::sin(theta);

            RayDefinition ray(origin, rayDir, maxDistance);

            if (RaycastHitExtended hit; raycast(ray, hit, filter)) {
                hits.push_back(hit);
            }
        }

        return hits;
    }

    std::vector<RaycastHitExtended> RaycastSystem::scatterCast(const Vec3& origin,
                                                               Int numRays,
                                                               Float maxDistance,
                                                               const QueryFilter& filter) {
        std::vector<RaycastHitExtended> hits;
        hits.reserve(numRays);

        for (Int i = 0; i < numRays; ++i) {
            // Generate random direction on unit sphere
            Float theta = rng_.range(0.0f, TWO_PI<Float>);
            Float phi = std::acos(rng_.range(-1.0f, 1.0f));

            Vec3 direction(
                std::sin(phi) * std::cos(theta),
                std::sin(phi) * std::sin(theta),
                std::cos(phi)
            );

            RayDefinition ray(origin, direction, maxDistance);

            if (RaycastHitExtended hit; raycast(ray, hit, filter)) {
                hits.push_back(hit);
            }
        }

        return hits;
    }

    std::vector<RaycastHitExtended> RaycastSystem::raycastAll(const RayDefinition& ray,
                                                              const QueryFilter& filter) {
        const Vec3 to = ray.origin + ray.direction * ray.maxDistance;
        const std::vector<RaycastHit> basicHits = m_queries.raycastAll(ray.origin, to, filter);

        std::vector<RaycastHitExtended> extendedHits;
        extendedHits.reserve(basicHits.size());

        for (const auto& basicHit : basicHits) {
            RaycastHitExtended hit(basicHit);
            extractExtendedInfo(hit, ray);
            extendedHits.push_back(hit);
        }

        return extendedHits;
    }

    std::vector<RaycastHitExtended> RaycastSystem::penetratingRaycast(const RayDefinition& ray,
                                                                      const Int maxHits,
                                                                      const QueryFilter& filter) {
        std::vector<RaycastHitExtended> hits;
        hits.reserve(maxHits);

        RayDefinition currentRay = ray;
        Float remainingDistance = ray.maxDistance;

        while (hits.size() < static_cast<unsigned long>(maxHits) && remainingDistance > EPSILON) {
            RaycastHitExtended hit;
            if (!raycast(currentRay, hit, filter)) {
                break;
            }

            hits.push_back(hit);

            // Move ray origin past the hit point
            currentRay.origin = hit.point + currentRay.direction * 0.001f;
            remainingDistance -= hit.distance;
            currentRay.maxDistance = remainingDistance;
        }

        return hits;
    }

    void RaycastSystem::startBatchProcessor(const Int numThreads) {
        if (m_batchProcessorRunning) return;

        m_batchProcessorRunning = true;

        for (Int i = 0; i < numThreads; ++i) {
            m_batchThreads.emplace_back([this]() {
                processBatchRequests();
            });
        }
    }

    void RaycastSystem::stopBatchProcessor() {
        if (!m_batchProcessorRunning) return;

        m_batchProcessorRunning = false;
        m_batchCondition.notify_all();

        for (auto& thread : m_batchThreads) {
            if (thread.joinable()) {
                thread.join();
            }
        }

        m_batchThreads.clear();
    }

    void RaycastSystem::submitBatchRequest(const BatchRaycastRequest& request) {
        {
            std::lock_guard lock(m_batchMutex);
            m_batchQueue.push(request);
        }
        m_batchCondition.notify_one();
    }

    void RaycastSystem::processPendingBatches() {
        std::vector<BatchRaycastRequest> tempQueue;

        {
            std::lock_guard lock(m_batchMutex);

            // mover todos los elementos de la priority_queue a un vector
            while (!m_batchQueue.empty()) {
                tempQueue.push_back(m_batchQueue.top());
                m_batchQueue.pop();
            }
        }

        // ordenar vector para mantener prioridad (si es que quieres)
        std::ranges::sort(tempQueue, [](const auto& a, const auto& b) {
            return a.priority > b.priority; // higher priority first
        });

        // procesar
        for (auto& request : tempQueue) {
            if (RaycastHitExtended hit; raycast(request.ray, hit, request.filter)) {
                if (request.callback) request.callback(hit);
            }
        }
    }

    bool RaycastSystem::findGround(const Vec3& position, const Float maxDistance,
                                   RaycastHitExtended& groundHit,
                                   const QueryFilter& filter) {
        const RayDefinition ray(position, Vec3(0, -1, 0), maxDistance);
        return raycast(ray, groundHit, filter);
    }

    bool RaycastSystem::hasLineOfSight(const Vec3& from, const Vec3& to,
                                       const QueryFilter& filter) {
        Vec3 direction = to - from;
        Float distance = glm::length(direction);
        direction /= distance;

        RayDefinition ray(from, direction, distance);
        RaycastHitExtended hit;

        return !raycast(ray, hit, filter);
    }

    Float RaycastSystem::calculateVisibility(const Vec3& observer, const Vec3& target,
                                             const Float targetRadius,
                                             const Int numSamples) {
        Int visibleSamples = 0;

        // Sample points on target sphere
        for (Int i = 0; i < numSamples; ++i) {
            const Float theta = rng_.range(0.0f, TWO_PI<Float>);
            const Float phi = rng_.range(0.0f, PI<Float>);

            Vec3 samplePoint = target + Vec3(
                targetRadius * std::sin(phi) * std::cos(theta),
                targetRadius * std::sin(phi) * std::sin(theta),
                targetRadius * std::cos(phi)
            );

            if (hasLineOfSight(observer, samplePoint)) {
                visibleSamples++;
            }
        }

        return static_cast<Float>(visibleSamples) / numSamples;
    }

    bool RaycastSystem::findReflection(const RayDefinition& ray, Int maxBounces,
                                       std::vector<RaycastHitExtended>& reflectionPath,
                                       const QueryFilter& filter) {
        reflectionPath.clear();
        reflectionPath.reserve(maxBounces);

        RayDefinition currentRay = ray;

        for (Int i = 0; i < maxBounces; ++i) {
            RaycastHitExtended hit;
            if (!raycast(currentRay, hit, filter)) {
                break;
            }

            reflectionPath.push_back(hit);

            // Calculate reflection direction
            Vec3 reflectDir = glm::reflect(currentRay.direction, hit.normal);
            currentRay.origin = hit.point + hit.normal * 0.001f;
            currentRay.direction = reflectDir;
        }

        return !reflectionPath.empty();
    }

    void RaycastSystem::extractExtendedInfo(RaycastHitExtended& hit, const RayDefinition& ray) {
        if (!hit.body) return;

        // Get material properties
        hit.friction = hit.body->getFriction();
        hit.restitution = hit.body->getRestitution();

        // Calculate texture coordinates if mesh
        // This would require mesh data access

        // Calculate barycentric coordinates for triangle hits
        // This would require triangle information
    }

    std::size_t RaycastSystem::generateCacheKey(const RayDefinition& ray, const QueryFilter& filter) {
        std::size_t key = 0;

        // Hash ray origin and direction
        key ^= std::hash<Float>()(ray.origin.x) << 1;
        key ^= std::hash<Float>()(ray.origin.y) << 2;
        key ^= std::hash<Float>()(ray.origin.z) << 3;
        key ^= std::hash<Float>()(ray.direction.x) << 4;
        key ^= std::hash<Float>()(ray.direction.y) << 5;
        key ^= std::hash<Float>()(ray.direction.z) << 6;
        key ^= std::hash<Float>()(ray.maxDistance) << 7;

        // Hash filter settings
        key ^= std::hash<std::uint16_t>()(filter.groupMask) << 8;
        key ^= std::hash<std::uint16_t>()(filter.categoryMask) << 9;

        return key;
    }

    RaycastCacheEntry* RaycastSystem::checkCache(const RayDefinition& ray, const QueryFilter& filter) {
        const std::size_t key = generateCacheKey(ray, filter);

        const auto it = m_cache.find(key);
        if (it != m_cache.end() && it->second.isValid()) {
            return &it->second;
        }

        return nullptr;
    }

    void RaycastSystem::updateCache(const RayDefinition& ray, const QueryFilter& filter,
                         const RaycastHitExtended& hit, const bool hasHit) {
        const std::size_t key = generateCacheKey(ray, filter);

        RaycastCacheEntry entry;
        entry.ray = ray;
        entry.hit = hit;
        entry.hasHit = hasHit;
        entry.timestamp = std::chrono::steady_clock::now();

        m_cache[key] = entry;

        // Evict old entries if cache is too large
        if (m_cache.size() > m_maxCacheSize) {
            evictOldestCacheEntry();
        }
    }

    void RaycastSystem::evictOldestCacheEntry() {
        if (m_cache.empty()) return;

        auto oldest = m_cache.begin();
        auto oldestTime = oldest->second.timestamp;

        for (auto it = m_cache.begin(); it != m_cache.end(); ++it) {
            if (it->second.timestamp < oldestTime) {
                oldest = it;
                oldestTime = it->second.timestamp;
            }
        }

        m_cache.erase(oldest);
    }

    void RaycastSystem::processBatchRequests() {
        while (m_batchProcessorRunning) {
            std::unique_lock lock(m_batchMutex);

            m_batchCondition.wait(lock, [this]() {
                return !m_batchQueue.empty() || !m_batchProcessorRunning;
            });

            if (!m_batchProcessorRunning) break;

            if (!m_batchQueue.empty()) {
                BatchRaycastRequest request = m_batchQueue.top();
                m_batchQueue.pop();
                lock.unlock();

                // Process request
                if (RaycastHitExtended hit; raycast(request.ray, hit, request.filter)) {
                    if (request.callback) {
                        request.callback(hit);
                    }
                }

                m_stats.batchProcessed++;
            }
        }
    }
} // namespace engine::physics
