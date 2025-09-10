/**
 * @file PhysicsProfiler.h
 * @brief Physics performance profiling and statistics
 * @details Tracks performance metrics, identifies bottlenecks, and provides
 *          optimization insights for the physics simulation
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

#include "../core/PhysicsTypes.h"

namespace engine::physics {
    /**
     * @brief Performance sample for a single frame
     */
    struct PerformanceSample {
        Float totalTime = 0.0f; // Total physics time (ms)
        Float stepTime = 0.0f; // Simulation step time
        Float collisionTime = 0.0f; // Collision detection time
        Float solverTime = 0.0f; // Constraint solving time
        Float integrationTime = 0.0f; // Integration time
        Float broadphaseTime = 0.0f; // Broadphase time
        Float narrowphaseTime = 0.0f; // Narrowphase time
        Float updateTime = 0.0f; // Transform update time

        Int stepCount = 0; // Number of simulation steps
        Int bodyCount = 0; // Active body count
        Int contactCount = 0; // Contact point count
        Int manifoldCount = 0; // Manifold count
        Int constraintCount = 0; // Active constraint count
        Int islandCount = 0; // Simulation island count

        std::chrono::steady_clock::time_point timestamp;
    };

    /**
     * @brief Scoped timer for profiling code sections
     */
    class ScopedTimer {
    public:
        explicit ScopedTimer(Float& target);

        ~ScopedTimer();

    private:
        Float& target_;
        std::chrono::high_resolution_clock::time_point start_;
    };

    /**
     * @brief Physics performance profiler
     * @details Comprehensive profiling system for tracking physics performance,
     *          identifying bottlenecks, and providing optimization suggestions
     */
    class PhysicsProfiler {
    public:
        PhysicsProfiler();

        // ============================================================================
        // Profiling Control
        // ============================================================================

        void setEnabled(const bool enabled) { enabled_ = enabled; }
        bool isEnabled() const { return enabled_; }

        void beginFrame();

        void endFrame();

        // ============================================================================
        // Timing Functions
        // ============================================================================

        void startTimer(const std::string& name);

        void endTimer(const std::string& name);

        static ScopedTimer scopedTimer(Float& target) {
            return ScopedTimer(target);
        }

        // ============================================================================
        // Recording Methods
        // ============================================================================

        void recordFrame(Float frameTime, Int steps);

        void recordSimulationStep(const Float stepTime) {
            if (!enabled_) return;
            currentSample_.stepTime += stepTime;
        }

        void recordCollisionDetection(Float time, Int manifolds, Int contacts);

        void recordSolver(const Float time /*, Int iterations */) {
            if (!enabled_) return;

            currentSample_.solverTime += time;
            // Could track iterations if needed
        }

        void recordBodyCount(const Int active /*, Int total */) {
            if (!enabled_) return;
            currentSample_.bodyCount = active;
        }

        void recordConstraintCount(const Int count) {
            if (!enabled_) return;
            currentSample_.constraintCount = count;
        }

        void recordIslandCount(const Int count) {
            if (!enabled_) return;
            currentSample_.islandCount = count;
        }

        // ============================================================================
        // Statistics
        // ============================================================================

        struct Statistics {
            // Timing stats (ms)
            Float avgTotalTime = 0.0f;
            Float minTotalTime = INFINITY_VALUE<Float>;
            Float maxTotalTime = 0.0f;
            Float avgStepTime = 0.0f;
            Float avgCollisionTime = 0.0f;
            Float avgSolverTime = 0.0f;

            // Count stats
            Float avgBodyCount = 0.0f;
            Float avgContactCount = 0.0f;
            Float avgConstraintCount = 0.0f;

            // Performance metrics
            Float fps = 0.0f;
            Float physicsLoad = 0.0f; // Percentage of frame time
            Float efficiency = 0.0f; // Bodies per millisecond
        };

        const Statistics& getStatistics() const { return stats_; }

        /**
         * @brief Get detailed performance report
         */
        std::string getReport() const;

        /**
         * @brief Get CSV data for logging
         */
        std::string getCSVData() const;

        /**
         * @brief Clear all samples
         */
        void clear();

        // ============================================================================
        // Performance Analysis
        // ============================================================================

        /**
         * @brief Check if physics is taking too much frame time
         */
        bool isBottleneck() const {
            return stats_.physicsLoad > 20.0f; // More than 20% of frame
        }

        /**
         * @brief Get bottleneck analysis
         */
        std::string getBottleneckAnalysis() const;

        /**
         * @brief Get spike analysis for frame drops
         */
        std::vector<std::pair<std::size_t, Float>> getSpikes(Float threshold = 33.0f) const;

        size_t sampleCount() const { return size_; }

    private:
        bool enabled_;
        std::uint64_t frameCount_ = 0;

        // Timing
        std::chrono::high_resolution_clock::time_point frameStart_;
        std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point> timers_;
        std::unordered_map<std::string, Float> timerResults_;

        // Samples
        std::vector<PerformanceSample> samples_;
        size_t head_;
        size_t size_;
        PerformanceSample currentSample_;

        // Statistics
        Statistics stats_;

        // Analysis
        std::vector<std::string> performanceIssues_;
        std::vector<std::string> optimizationSuggestions_;

        void updateStatistics();

        void detectPerformanceIssues();
    };
} // namespace engine::physics
