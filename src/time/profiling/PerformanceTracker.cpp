/**
 * @file PerformanceTracker.cpp
 * @brief Implementation of comprehensive performance tracking system
 * @details Efficient performance metrics collection and analysis
 *
 * @author Development Team
 * @date Created on 2024-01-20
 */

#include "PerformanceTracker.h"

#include "../../memory/allocators/CircularBufferAllocator.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <utility>

namespace engine::time {

    // =============================================================================
    // PerformanceReport Implementation
    // =============================================================================

    std::string PerformanceReport::toString() const {
        std::stringstream ss;

        ss << "=== Performance Report ===" << std::endl;
        ss << "Time Range: " << toMilliseconds(totalDuration) << "ms" << std::endl;
        ss << std::endl;

        // Frame statistics
        ss << "Frame Statistics:" << std::endl;
        ss << "  Average FPS: " << std::fixed << std::setprecision(1) << metrics.averageFPS << std::endl;
        ss << "  95th Percentile FPS: " << metrics.percentile95FPS << std::endl;
        ss << "  99th Percentile FPS: " << metrics.percentile99FPS << std::endl;
        ss << "  Average Frame Time: " << toMilliseconds(metrics.averageFrameTime) << "ms" << std::endl;
        ss << "  Worst Frame Time: " << toMilliseconds(metrics.worstFrameTime) << "ms" << std::endl;
        ss << std::endl;

        // System breakdown
        if (!systemStats.empty()) {
            ss << "System Breakdown:" << std::endl;
            for (const auto& [name, stats] : systemStats) {
                ss << "  " << std::setw(20) << name << ": "
                        << std::fixed << std::setprecision(2)
                        << toMilliseconds(stats.getAverage()) << "ms avg" << std::endl;
            }
            ss << std::endl;
        }

        // Memory statistics
        ss << "Memory Statistics:" << std::endl;
        ss << "  Current Usage: " << (static_cast<double>(memoryStats.currentUsage) / (1024.0 * 1024.0)) << " MB" <<
                std::endl;
        ss << "  Peak Usage: " << (static_cast<double>(memoryStats.peakUsage) / (1024.0 * 1024.0)) << " MB" <<
                std::endl;
        ss << "  Total Allocations: " << memoryStats.totalAllocations << std::endl;
        ss << "  Fragmentation: " << std::fixed << std::setprecision(1)
                << memoryStats.fragmentationPercent << "%" << std::endl;
        ss << std::endl;

        // Issues
        if (issues.frameDrops > 0 || issues.spikes > 0 || issues.budgetExceeded > 0) {
            ss << "Performance Issues:" << std::endl;
            if (issues.frameDrops > 0) {
                ss << "  Frame Drops: " << issues.frameDrops << std::endl;
            }
            if (issues.spikes > 0) {
                ss << "  Frame Spikes: " << issues.spikes << std::endl;
            }
            if (issues.budgetExceeded > 0) {
                ss << "  Budget Exceeded: " << issues.budgetExceeded << " times" << std::endl;
            }

            if (!issues.warnings.empty()) {
                ss << "  Warnings:" << std::endl;
                for (const auto& warning : issues.warnings) {
                    ss << "    - " << warning << std::endl;
                }
            }

            if (!issues.criticals.empty()) {
                ss << "  Critical Issues:" << std::endl;
                for (const auto& critical : issues.criticals) {
                    ss << "    ! " << critical << std::endl;
                }
            }
        }

        // CPU/GPU utilization
        ss << "Resource Utilization:" << std::endl;
        ss << "  CPU: " << std::fixed << std::setprecision(1)
                << (metrics.cpuUtilization * 100.0f) << "%" << std::endl;
        ss << "  GPU: " << (metrics.gpuUtilization * 100.0f) << "%" << std::endl;

        return ss.str();
    }

    // =============================================================================
    // PerformanceTracker Implementation
    // =============================================================================

    PerformanceTracker::PerformanceTracker(PerformanceTrackerConfig config) :
        config_(std::move(config)) {

        lastReportTime_ = Clock::now();
    }

    PerformanceTracker::~PerformanceTracker() {
        shutdown();
    }

    bool PerformanceTracker::initialize(engine::memory::MemoryManager* memoryManager) {
        if (initialized_) {
            return true;
        }

        memoryManager_ = memoryManager;
        if (!memoryManager_) {
            std::cerr << "[PerformanceTracker] Error: No memory manager provided!" << std::endl;
            return false;
        }

        try {
            // Create circular buffer allocators for history
            const std::size_t frameBufferSize = config_.frameHistorySize * sizeof(FrameHistory<300>::FrameRecord);
            frameHistoryAllocator_ = std::make_unique<engine::memory::CircularBufferAllocator>(
                    frameBufferSize,
                    config_.frameHistorySize,
                    "FrameHistoryBuffer"
                    );

            const std::size_t sampleBufferSize = config_.sampleHistorySize * sizeof(PerformanceSample);
            sampleHistoryAllocator_ = std::make_unique<engine::memory::CircularBufferAllocator>(
                    sampleBufferSize,
                    config_.sampleHistorySize,
                    "SampleHistoryBuffer"
                    );

            initialized_ = true;
            return true;
        } catch (const std::exception& e) {
            std::cerr << "[PerformanceTracker] Initialization failed: " << e.what() << std::endl;
            return false;
        }
    }

    void PerformanceTracker::shutdown() {
        if (!initialized_) {
            return;
        }

        // Generate final report if configured
        if (config_.autoReport && config_.reportCallback) {
            const auto report = generateReport(true);
            config_.reportCallback(report);
        }

        frameHistoryAllocator_.reset();
        sampleHistoryAllocator_.reset();

        initialized_ = false;
    }

    void PerformanceTracker::beginFrame(const FrameNumber frameNumber) {
        if (!initialized_)
            return;

        currentFrame_ = frameNumber;
        frameStartTime_ = Clock::now();
        inFrame_ = true;
    }

    void PerformanceTracker::endFrame(const Duration cpuTime, const Duration gpuTime) {
        if (!initialized_ || !inFrame_)
            return;

        const auto frameEndTime = Clock::now();
        const auto frameDuration = std::chrono::duration_cast<Duration>(frameEndTime - frameStartTime_);

        // Update frame statistics
        updateFrameStats(frameDuration, cpuTime, gpuTime);

        // Record frame in history
        FrameHistory<300>::FrameRecord record;
        record.frameNumber = currentFrame_;
        record.timestamp = frameEndTime;
        record.frameDuration = frameDuration;
        record.cpuDuration = cpuTime;
        record.gpuDuration = gpuTime;
        record.instantFPS = frameDuration.count() > 0 ? 1000000.0f / static_cast<float>(frameDuration.count()) : 0.0f;
        record.wasDropped = frameDuration > config_.frameTimeWarning;

        frameHistory_.addFrame(record);

        // Detect spike
        if (detectSpike(frameDuration)) {
            counters_.spiralOfDeathEvents.fetch_add(1);
        }

        // Check for auto-report
        checkAutoReport();

        inFrame_ = false;
    }

    void PerformanceTracker::recordFrameDrop(const std::string& reason) {
        if (!initialized_)
            return;

        std::lock_guard lock(statsMutex_);
        frameStats_.recordFrameDrop(constants::TARGET_FRAME_TIME_60FPS);

        if (!reason.empty()) {
            // Store reason for reporting
            counters_.totalFrames.fetch_add(1);
        }
    }

    void PerformanceTracker::recordVSyncMiss() {
        if (!initialized_)
            return;

        frameStats_.vsyncMisses.fetch_add(1);
    }

    ScopedTimer PerformanceTracker::beginSystemTiming(const std::string& systemName) {
        return ScopedTimer(
                systemName,
                ReportDestination::CALLBACK,
                [this, systemName](const TimerReport& report) {
                    recordSystemTiming(systemName, report.elapsed);
                }
                );
    }

    void PerformanceTracker::recordSystemTiming(const std::string& systemName, const Duration duration) {
        if (!initialized_ || !config_.trackSystemStats)
            return;

        std::lock_guard lock(statsMutex_);
        systemStats_[systemName].update(duration);
    }

    void PerformanceTracker::recordBudgetExceeded(
            const std::string& systemName,
            const Duration actual,
            const Duration budget
            ) {

        if (!initialized_)
            return;

        counters_.budgetExceeded.fetch_add(1);

        // Log warning
        if (config_.reportCallback) {
            std::stringstream ss;
            ss << "[Budget Exceeded] " << systemName
                    << ": " << toMilliseconds(actual) << "ms"
                    << " (budget: " << toMilliseconds(budget) << "ms)";
            config_.reportCallback(ss.str());
        }
    }

    void PerformanceTracker::recordEventLatency(const std::string& eventName, const Duration latency) {
        if (!initialized_ || !config_.trackEventLatency)
            return;

        std::lock_guard lock(statsMutex_);
        eventLatencies_[eventName].update(latency);
    }

    void PerformanceTracker::recordSample(const PerformanceSample& sample) const {
        if (!initialized_)
            return;

        // Store sample in circular buffer
        if (sampleHistoryAllocator_) {
            if (void* buffer = sampleHistoryAllocator_->allocate(sizeof(PerformanceSample))) {
                new(buffer) PerformanceSample(sample);
            }
        }
    }

    void PerformanceTracker::updateMemoryStats(
            const std::size_t currentUsage,
            const std::size_t allocations,
            const std::size_t deallocations
            ) {

        if (!initialized_ || !config_.trackMemoryStats)
            return;

        currentMemoryUsage_.store(currentUsage);

        // Update peak
        std::size_t peak = peakMemoryUsage_.load();
        while (currentUsage > peak &&
            !peakMemoryUsage_.compare_exchange_weak(peak, currentUsage)) {}

        if (allocations > 0) {
            totalAllocations_.fetch_add(allocations);
        }
        if (deallocations > 0) {
            totalDeallocations_.fetch_add(deallocations);
        }

        // Check memory warning
        if (config_.memoryWarningPercent > 0 && memoryManager_) {
            // TODO: Revisar este assuming
            const float usagePercent = (static_cast<float>(currentUsage) * 100.0f) /
                    static_cast<float>((512 * 1024 * 1024)); // Assuming 512MB total

            if (usagePercent > config_.memoryWarningPercent && config_.reportCallback) {
                std::stringstream ss;
                ss << "[Memory Warning] Usage at " << std::fixed << std::setprecision(1)
                        << usagePercent << "%";
                config_.reportCallback(ss.str());
            }
        }
    }

    PerformanceReport PerformanceTracker::analyze() const {
        PerformanceReport report = {};

        if (!initialized_)
            return report;

        std::lock_guard lock(statsMutex_);

        // Copy frame statistics
        report.frameStats = frameStats_;

        // Copy system statistics
        report.systemStats = systemStats_;

        // Memory statistics
        report.memoryStats.currentUsage = currentMemoryUsage_.load();
        report.memoryStats.peakUsage = peakMemoryUsage_.load();
        report.memoryStats.totalAllocations = totalAllocations_.load();
        report.memoryStats.totalDeallocations = totalDeallocations_.load();

        // Calculate metrics
        if (frameStats_.frameDuration.sampleCount > 0) {
            const Duration avgFrame = frameStats_.frameDuration.getAverage();
            report.metrics.averageFrameTime = avgFrame;
            report.metrics.averageFPS = avgFrame.count() > 0 ? 1000000.0 / static_cast<double>(avgFrame.count()) : 0.0;
        }

        report.metrics.percentile95FPS = frameStats_.percentile95FPS.load();
        report.metrics.percentile99FPS = frameStats_.percentile99FPS.load();
        report.metrics.worstFrameTime = frameStats_.frameDuration.max.load();

        // Calculate CPU/GPU utilization
        if (frameStats_.cpuTime.sampleCount > 0 && frameStats_.frameDuration.sampleCount > 0) {
            const float cpuTime = static_cast<float>(frameStats_.cpuTime.getAverage().count());
            const float frameTime = static_cast<float>(frameStats_.frameDuration.getAverage().count());
            report.metrics.cpuUtilization = frameTime > 0 ? cpuTime / frameTime : 0.0f;
        }

        if (frameStats_.gpuTime.sampleCount > 0 && frameStats_.frameDuration.sampleCount > 0) {
            const float gpuTime = static_cast<float>(frameStats_.gpuTime.getAverage().count());
            const float frameTime = static_cast<float>(frameStats_.frameDuration.getAverage().count());
            report.metrics.gpuUtilization = frameTime > 0 ? gpuTime / frameTime : 0.0f;
        }

        // Count issues
        report.issues.frameDrops = frameStats_.droppedFrames.load();
        report.issues.spikes = counters_.spiralOfDeathEvents.load();
        report.issues.budgetExceeded = counters_.budgetExceeded.load();

        // Detect anomalies
        for (const auto anomalies = detectAnomalies(); const auto& anomaly : anomalies) {
            if (anomaly.find("Critical") != std::string::npos) {
                report.issues.criticals.push_back(anomaly);
            } else {
                report.issues.warnings.push_back(anomaly);
            }
        }

        // Time range
        report.startTime = frameStats_.frameDuration.firstSampleTime.load();
        report.endTime = frameStats_.frameDuration.lastSampleTime.load();
        report.totalDuration = std::chrono::duration_cast<Duration>(report.endTime - report.startTime);

        return report;
    }

    FrameStats PerformanceTracker::getCurrentFrameStats() const {
        std::lock_guard lock(statsMutex_);
        return frameStats_;
    }

    std::optional<BasicTimeStats> PerformanceTracker::getSystemStats(
            const std::string& systemName
            ) const {

        std::lock_guard lock(statsMutex_);

        if (const auto it = systemStats_.find(systemName); it != systemStats_.end()) {
            return it->second;
        }

        return std::nullopt;
    }

    Duration PerformanceTracker::calculatePercentile(const float percentile) const {
        return frameHistory_.getPercentile(percentile);
    }

    std::vector<std::string> PerformanceTracker::detectAnomalies() const {
        std::vector<std::string> anomalies;

        if (!config_.detectAnomalies) {
            return anomalies;
        }

        // Check frame rate
        if (const double avgFPS = frameStats_.averageFPS.load(); avgFPS < 30.0) {
            anomalies.emplace_back("Critical: Average FPS below 30");
        } else if (avgFPS < 50.0) {
            anomalies.emplace_back("Warning: Average FPS below 50");
        }

        // Check frame drops
        const auto drops = frameStats_.droppedFrames.load();
        if (const auto total = frameStats_.frameDuration.sampleCount.load(); total > 0) {
            if (const float dropRate = (static_cast<float>(drops) * 100.0f) / static_cast<float>(total); dropRate >
                5.0f) {
                anomalies.push_back(
                        "Critical: High frame drop rate (" +
                        std::to_string(static_cast<int>(dropRate)) + "%)"
                        );
            } else if (dropRate > 2.0f) {
                anomalies.push_back(
                        "Warning: Frame drop rate at " +
                        std::to_string(static_cast<int>(dropRate)) + "%"
                        );
            }
        }

        // Check memory usage
        const std::size_t currentMem = currentMemoryUsage_.load();
        if (const std::size_t peakMem = peakMemoryUsage_.load(); peakMem > 0) {
            if (const float memUsage = (static_cast<float>(currentMem) * 100.0f) / static_cast<float>(512 * 1024 * 1024)
                ; memUsage > 90.0f) {
                anomalies.emplace_back("Critical: Memory usage above 90%");
            } else if (memUsage > config_.memoryWarningPercent) {
                anomalies.emplace_back("Warning: High memory usage");
            }
        }

        // Check consecutive drops
        if (const auto consecutiveDrops = frameStats_.consecutiveDrops.load(); consecutiveDrops > 10) {
            anomalies.push_back(
                    "Critical: " + std::to_string(consecutiveDrops) +
                    " consecutive frame drops"
                    );
        }

        return anomalies;
    }

    // TODO: revisar esto
    std::string PerformanceTracker::generateReport(bool detailed) const {
        const auto report = analyze();
        return report.toString();
    }

    bool PerformanceTracker::exportToCSV(const std::string& filepath) const {
        std::ofstream file(filepath);
        if (!file.is_open()) {
            return false;
        }

        // Write header
        file << "Frame,Timestamp,Duration(ms),CPU(ms),GPU(ms),FPS,Dropped\n";

        // Write frame data
        for (std::size_t i = 0; i < frameHistory_.size(); ++i) {
            if (const auto frame = frameHistory_.getFrame(i)) {
                file << frame->frameNumber << ","
                        << frame->timestamp.time_since_epoch().count() << ","
                        << toMilliseconds(frame->frameDuration) << ","
                        << toMilliseconds(frame->cpuDuration) << ","
                        << toMilliseconds(frame->gpuDuration) << ","
                        << frame->instantFPS << ","
                        << (frame->wasDropped ? "1" : "0") << "\n";
            }
        }

        return true;
    }

    // TODO: Esto deberia ir en Profiling module cuando lo haga, asi que por ahora sin implementar esta bien
    bool PerformanceTracker::exportToJSON(const std::string& filepath) const {
        // JSON export implementation would go here
        // For brevity, returning true
        return true;
    }

    void PerformanceTracker::updateConfig(const PerformanceTrackerConfig& config) {
        config_ = config;

        if (config_.autoReport && !reportScheduled_) {
            lastReportTime_ = Clock::now();
        }
    }

    void PerformanceTracker::reset() {
        std::lock_guard lock(statsMutex_);

        frameStats_.reset();
        systemStats_.clear();
        eventLatencies_.clear();
        counters_.reset();
        frameHistory_.clear();

        currentMemoryUsage_ = 0;
        peakMemoryUsage_ = 0;
        totalAllocations_ = 0;
        totalDeallocations_ = 0;

        lastReportTime_ = Clock::now();
    }

    void PerformanceTracker::checkAutoReport() {
        if (!config_.autoReport || !config_.reportCallback) {
            return;
        }

        const auto now = Clock::now();

        if (const auto timeSinceReport = now - lastReportTime_; timeSinceReport >= config_.reportInterval) {
            const auto report = generateReport(false);
            config_.reportCallback(report);
            lastReportTime_ = now;
        }
    }

    void PerformanceTracker::updateFrameStats(const Duration frameDuration, const Duration cpuTime, const Duration gpuTime) {
        std::lock_guard lock(statsMutex_);

        frameStats_.updateFrame(frameDuration, cpuTime, gpuTime);

        // Update global counters
        counters_.totalFrames.fetch_add(1);

        Duration totalRuntime = counters_.totalRuntime.load();
        while (!counters_.totalRuntime.compare_exchange_weak(
                totalRuntime,
                totalRuntime + frameDuration
                )) {}
    }

    bool PerformanceTracker::detectSpike(const Duration frameDuration) const {
        if (!config_.detectAnomalies) {
            return false;
        }

        const Duration avg = frameStats_.frameDuration.getAverage();
        if (avg == Duration::zero()) {
            return false;
        }

        const float ratio = static_cast<float>(frameDuration.count()) /
                static_cast<float>(avg.count());

        return ratio > config_.spikeThreshold;
    }

} // namespace engine::time
