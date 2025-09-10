/**
 * @file PhysicsProfiler.cpp
 * @brief Physics performance profiling and statistics
 * @details Tracks performance metrics, identifies bottlenecks, and provides
 *          optimization insights for the physics simulation
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#include "PhysicsProfiler.h"

namespace engine::physics {
    ScopedTimer::ScopedTimer(Float& target)
        : target_(target), start_(std::chrono::high_resolution_clock::now()) {
    }

    ScopedTimer::~ScopedTimer() {
        const auto end = std::chrono::high_resolution_clock::now();
        const Float duration = std::chrono::duration<Float, std::milli>(end - start_).count();
        target_ += duration;
    }

    PhysicsProfiler::PhysicsProfiler() : enabled_(true),
                                         samples_(debug::PROFILER_SAMPLE_COUNT),
                                         head_(0), size_(0) {
        currentSample_ = PerformanceSample();
    }

    void PhysicsProfiler::beginFrame() {
        if (!enabled_) return;

        frameStart_ = std::chrono::high_resolution_clock::now();
        currentSample_ = PerformanceSample();
        currentSample_.timestamp = std::chrono::steady_clock::now();
    }

    void PhysicsProfiler::endFrame() {
        if (!enabled_) return;

        const auto frameEnd = std::chrono::high_resolution_clock::now();
        currentSample_.totalTime = std::chrono::duration<Float, std::milli>(
            frameEnd - frameStart_).count();

        // Store sample
        samples_[head_] = currentSample_;
        head_ = (head_ + 1) % samples_.size();
        size_ = std::min(size_ + 1, samples_.size());

        // Update statistics
        updateStatistics();

        // Check for performance issues
        detectPerformanceIssues();

        frameCount_++;
    }

    void PhysicsProfiler::startTimer(const std::string& name) {
        if (!enabled_) return;

        timers_[name] = std::chrono::high_resolution_clock::now();
    }

    void PhysicsProfiler::endTimer(const std::string& name) {
        if (!enabled_) return;

        if (const auto it = timers_.find(name); it != timers_.end()) {
            const auto end = std::chrono::high_resolution_clock::now();
            const Float duration = std::chrono::duration<Float, std::milli>(
                end - it->second).count();

            timerResults_[name] = duration;
            timers_.erase(it);

            // Map to appropriate category
            if (name == "Broadphase") currentSample_.broadphaseTime = duration;
            else if (name == "Narrowphase") currentSample_.narrowphaseTime = duration;
            else if (name == "Solver") currentSample_.solverTime = duration;
            else if (name == "Integration") currentSample_.integrationTime = duration;
            else if (name == "Collision") currentSample_.collisionTime = duration;
            else if (name == "Update") currentSample_.updateTime = duration;
        }
    }

    void PhysicsProfiler::recordFrame(const Float frameTime, const Int steps) {
        if (!enabled_) return;

        currentSample_.totalTime = frameTime;
        currentSample_.stepCount = steps;
        endFrame();
    }

    void PhysicsProfiler::recordCollisionDetection(const Float time, const Int manifolds, const Int contacts) {
        if (!enabled_) return;

        currentSample_.collisionTime += time;
        currentSample_.manifoldCount = manifolds;
        currentSample_.contactCount = contacts;
    }

    std::string PhysicsProfiler::getReport() const {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2);

        ss << "=== Physics Performance Report ===\n";
        ss << "Samples: " << samples_.size() << " frames\n\n";

        ss << "Timing (ms):\n";
        ss << "  Total:      " << std::setw(8) << stats_.avgTotalTime
            << " (min: " << stats_.minTotalTime
            << ", max: " << stats_.maxTotalTime << ")\n";
        ss << "  Step:       " << std::setw(8) << stats_.avgStepTime << "\n";
        ss << "  Collision:  " << std::setw(8) << stats_.avgCollisionTime << "\n";
        ss << "  Solver:     " << std::setw(8) << stats_.avgSolverTime << "\n";

        ss << "\nCounts:\n";
        ss << "  Bodies:     " << std::setw(8) << stats_.avgBodyCount << "\n";
        ss << "  Contacts:   " << std::setw(8) << stats_.avgContactCount << "\n";
        ss << "  Constraints:" << std::setw(8) << stats_.avgConstraintCount << "\n";

        ss << "\nPerformance:\n";
        ss << "  Physics FPS:" << std::setw(8) << stats_.fps << "\n";
        ss << "  Load:       " << std::setw(8) << stats_.physicsLoad << "%\n";
        ss << "  Efficiency: " << std::setw(8) << stats_.efficiency << " bodies/ms\n";

        if (!performanceIssues_.empty()) {
            ss << "\nPerformance Issues:\n";
            for (const auto& issue : performanceIssues_) {
                ss << "  - " << issue << "\n";
            }
        }

        if (!optimizationSuggestions_.empty()) {
            ss << "\nOptimization Suggestions:\n";
            for (const auto& suggestion : optimizationSuggestions_) {
                ss << "  - " << suggestion << "\n";
            }
        }

        return ss.str();
    }

    std::string PhysicsProfiler::getCSVData() const {
        std::stringstream ss;

        // Header
        ss << "Timestamp,TotalTime,StepTime,CollisionTime,SolverTime,"
            << "Bodies,Contacts,Constraints\n";

        // Data rows
        for (const auto& sample : samples_) {
            const auto timeT = std::chrono::system_clock::to_time_t(
                std::chrono::system_clock::now());

            ss << timeT << ","
                << sample.totalTime << ","
                << sample.stepTime << ","
                << sample.collisionTime << ","
                << sample.solverTime << ","
                << sample.bodyCount << ","
                << sample.contactCount << ","
                << sample.constraintCount << "\n";
        }

        return ss.str();
    }

    void PhysicsProfiler::clear() {
        samples_.clear();
        currentSample_ = PerformanceSample();
        stats_ = Statistics();
        performanceIssues_.clear();
        optimizationSuggestions_.clear();
        frameCount_ = 0;
    }

    std::string PhysicsProfiler::getBottleneckAnalysis() const {
        std::stringstream ss;

        if (samples_.empty()) {
            return "No data available for analysis";
        }

        // Analyze time distribution
        if (const Float totalTime = stats_.avgTotalTime; totalTime > EPSILON) {
            const Float collisionPercent = (stats_.avgCollisionTime / totalTime) * 100.0f;
            const Float solverPercent = (stats_.avgSolverTime / totalTime) * 100.0f;
            const Float stepPercent = (stats_.avgStepTime / totalTime) * 100.0f;

            ss << "Time Distribution:\n";
            ss << "  Collision: " << collisionPercent << "%\n";
            ss << "  Solver:    " << solverPercent << "%\n";
            ss << "  Step:      " << stepPercent << "%\n";

            // Identify primary bottleneck
            if (collisionPercent > 50.0f) {
                ss << "\nBottleneck: Collision Detection\n";
                ss << "Suggestions:\n";
                ss << "  - Reduce collision shape complexity\n";
                ss << "  - Use simpler shapes for distant objects\n";
                ss << "  - Implement better spatial partitioning\n";
                ss << "  - Increase collision margins\n";
            }
            else if (solverPercent > 50.0f) {
                ss << "\nBottleneck: Constraint Solver\n";
                ss << "Suggestions:\n";
                ss << "  - Reduce solver iterations\n";
                ss << "  - Simplify constraints\n";
                ss << "  - Use warm starting\n";
                ss << "  - Break constraint chains\n";
            }
            else if (stats_.avgBodyCount > 1000) {
                ss << "\nHigh body count detected\n";
                ss << "Suggestions:\n";
                ss << "  - Implement LOD system\n";
                ss << "  - Sleep distant bodies\n";
                ss << "  - Merge static geometry\n";
                ss << "  - Use instancing for similar objects\n";
            }
        }

        return ss.str();
    }

    std::vector<std::pair<std::size_t, Float>> PhysicsProfiler::getSpikes(const Float threshold) const {
        std::vector<std::pair<std::size_t, Float>> spikes;

        for (std::size_t i = 0; i < samples_.size(); ++i) {
            if (samples_[i].totalTime > threshold) {
                spikes.emplace_back(i, samples_[i].totalTime);
            }
        }

        return spikes;
    }

    void PhysicsProfiler::updateStatistics() {
        if (samples_.empty()) return;

        // Reset stats
        stats_ = Statistics();
        stats_.minTotalTime = INFINITY_VALUE<Float>;

        // Calculate averages
        for (const auto& sample : samples_) {
            stats_.avgTotalTime += sample.totalTime;
            stats_.avgStepTime += sample.stepTime;
            stats_.avgCollisionTime += sample.collisionTime;
            stats_.avgSolverTime += sample.solverTime;
            stats_.avgBodyCount += static_cast<Float>(sample.bodyCount);
            stats_.avgContactCount += static_cast<Float>(sample.contactCount);
            stats_.avgConstraintCount += static_cast<Float>(sample.constraintCount);

            stats_.minTotalTime = min(stats_.minTotalTime, sample.totalTime);
            stats_.maxTotalTime = max(stats_.maxTotalTime, sample.totalTime);
        }

        const Float sampleCount = static_cast<Float>(samples_.size());
        stats_.avgTotalTime /= sampleCount;
        stats_.avgStepTime /= sampleCount;
        stats_.avgCollisionTime /= sampleCount;
        stats_.avgSolverTime /= sampleCount;
        stats_.avgBodyCount /= sampleCount;
        stats_.avgContactCount /= sampleCount;
        stats_.avgConstraintCount /= sampleCount;

        // Calculate derived metrics
        if (stats_.avgTotalTime > EPSILON) {
            stats_.fps = 1000.0f / stats_.avgTotalTime;
            stats_.physicsLoad = (stats_.avgTotalTime / 16.67f) * 100.0f; // Assume 60 FPS target
            stats_.efficiency = stats_.avgBodyCount / stats_.avgTotalTime;
        }
    }

    void PhysicsProfiler::detectPerformanceIssues() {
        performanceIssues_.clear();
        optimizationSuggestions_.clear();

        // Check for high frame time
        if (stats_.avgTotalTime > 16.0f) {
            performanceIssues_.push_back("Physics taking more than 16ms per frame");
            optimizationSuggestions_.push_back("Consider reducing simulation frequency");
        }

        // Check for spikes
        int spikeCount = 0;
        for (const auto& sample : samples_) {
            if (sample.totalTime > stats_.avgTotalTime * 2.0f) {
                spikeCount++;
            }
        }

        if (spikeCount > samples_.size() * 0.1f) {
            performanceIssues_.push_back("Frequent frame time spikes detected");
            optimizationSuggestions_.push_back("Check for periodic expensive operations");
        }

        // Check collision efficiency
        if (stats_.avgContactCount > stats_.avgBodyCount * 10) {
            performanceIssues_.push_back("Excessive contact points");
            optimizationSuggestions_.push_back("Optimize collision shapes or broadphase");
        }

        // Check solver load
        if (stats_.avgSolverTime > stats_.avgTotalTime * 0.5f) {
            performanceIssues_.push_back("Solver taking majority of frame time");
            optimizationSuggestions_.push_back("Reduce solver iterations or constraint count");
        }

        // Check body count
        if (stats_.avgBodyCount > 5000) {
            performanceIssues_.push_back("Very high body count");
            optimizationSuggestions_.push_back("Implement aggressive LOD and culling");
        }
    }
} // namespace engine::physics
