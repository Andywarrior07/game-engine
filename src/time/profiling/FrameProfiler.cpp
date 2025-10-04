/**
 * @file FrameProfiler.cpp
 * @brief Implementation of frame-level performance profiler
 * @details High-performance frame timing with lock-free operations for hot paths.
 *
 * @author Development Team
 * @date Created on 2024-01-20
 */

#include "FrameProfiler.h"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <numeric>

namespace engine::time {

    // =============================================================================
    // FrameSectionTracker Implementation
    // =============================================================================

    FrameSectionTracker::FrameSectionTracker(FrameProfiler* profiler, const Section section) noexcept :
        profiler_(profiler)
        , section_(section)
        , startTime_(Clock::now())
        , active_(true) {}

    FrameSectionTracker::~FrameSectionTracker() noexcept {
        if (active_ && profiler_) {
            const auto endTime = Clock::now();
            const auto duration = std::chrono::duration_cast<Duration>(endTime - startTime_);
            profiler_->endSection(section_, duration);
        }
    }

    FrameSectionTracker::FrameSectionTracker(FrameSectionTracker&& other) noexcept :
        profiler_(other.profiler_)
        , section_(other.section_)
        , startTime_(other.startTime_)
        , active_(other.active_) {
        other.active_ = false;
    }

    FrameSectionTracker& FrameSectionTracker::operator=(FrameSectionTracker&& other) noexcept {
        if (this != &other) {
            if (active_ && profiler_) {
                const auto endTime = Clock::now();
                const auto duration = std::chrono::duration_cast<Duration>(endTime - startTime_);
                profiler_->endSection(section_, duration);
            }

            profiler_ = other.profiler_;
            section_ = other.section_;
            startTime_ = other.startTime_;
            active_ = other.active_;
            other.active_ = false;
        }
        return *this;
    }

    // =============================================================================
    // FrameRingBuffer Implementation
    // =============================================================================

    void FrameProfiler::FrameRingBuffer::push(const FrameProfileData& frame) {
        const std::size_t currentHead = head.load(std::memory_order_acquire);
        const std::size_t nextHead = (currentHead + 1) % MAX_SIZE;

        // Store frame data
        data[currentHead] = frame;

        // Update head
        head.store(nextHead, std::memory_order_release);

        // Update size if not full
        if (const std::size_t currentSize = size.load(std::memory_order_acquire); currentSize < MAX_SIZE) {
            size.fetch_add(1, std::memory_order_acq_rel);
        } else {
            // Move tail if buffer is full
            const std::size_t currentTail = tail.load(std::memory_order_acquire);
            tail.store((currentTail + 1) % MAX_SIZE, std::memory_order_release);
        }
    }

    bool FrameProfiler::FrameRingBuffer::pop(FrameProfileData& frame) {
        if (const std::size_t currentSize = size.load(std::memory_order_acquire); currentSize == 0) {
            return false;
        }

        const std::size_t currentTail = tail.load(std::memory_order_acquire);
        frame = data[currentTail];

        tail.store((currentTail + 1) % MAX_SIZE, std::memory_order_release);
        size.fetch_sub(1, std::memory_order_acq_rel);

        return true;
    }

    void FrameProfiler::FrameRingBuffer::clear() {
        head.store(0, std::memory_order_release);
        tail.store(0, std::memory_order_release);
        size.store(0, std::memory_order_release);
    }

    std::vector<FrameProfileData> FrameProfiler::FrameRingBuffer::getRecent(const std::size_t count) const {
        const std::size_t currentSize = size.load(std::memory_order_acquire);
        const std::size_t numFrames = (count == 0 || count > currentSize) ? currentSize : count;

        std::vector<FrameProfileData> result;
        result.reserve(numFrames);

        const std::size_t currentHead = head.load(std::memory_order_acquire);
        const std::size_t currentTail = tail.load(std::memory_order_acquire);

        // Start from most recent and work backwards
        for (std::size_t i = 0; i < numFrames; ++i) {
            if (const std::size_t index = (currentHead + MAX_SIZE - 1 - i) % MAX_SIZE; (currentHead > currentTail && (
                    index >= currentTail && index < currentHead)) ||
                (currentHead <= currentTail && (index >= currentTail || index < currentHead))) {
                result.push_back(data[index]);
            }
        }

        return result;
    }

    // =============================================================================
    // FrameProfiler Implementation
    // =============================================================================

    FrameProfiler::FrameProfiler(
            memory::MemoryManager& memoryManager,
            const FrameProfilerConfig& config
            ) :
        config_(config)
        , memoryManager_(memoryManager) {}

    FrameProfiler::~FrameProfiler() {
        shutdown();
    }

    bool FrameProfiler::initialize() {
        if (initialized_.load(std::memory_order_acquire)) {
            return true;
        }

        // Allocate frame-local memory
        constexpr std::size_t frameMemorySize = 64 * 1024; // 64KB per frame
        frameAllocator_ = std::make_unique<memory::StackAllocator>(
                frameMemorySize,
                "FrameProfiler"
                );

        // Clear history
        frameHistory_.clear();

        // Reset stats
        frameStats_.reset();
        consecutiveDrops_.store(0, std::memory_order_release);
        totalDrops_.store(0, std::memory_order_release);
        totalVSyncMisses_.store(0, std::memory_order_release);

        // Calculate initial thresholds
        averageFrameTime_ = config_.targetFrameTime;
        spikeThreshold_ = Duration(
                static_cast<std::int64_t>(
                    config_.targetFrameTime.count() * config_.spikeThresholdMultiplier)
                );

        initialized_.store(true, std::memory_order_release);

        return true;
    }

    void FrameProfiler::shutdown() {
        if (!initialized_.load(std::memory_order_acquire)) {
            return;
        }

        // End current frame if active
        if (inFrame_.load(std::memory_order_acquire)) {
            endFrame();
        }

        // Generate final report if frames were dropped
        if (totalDrops_.load(std::memory_order_acquire) > 0) {
            std::cout << "[FrameProfiler] Shutdown Report:\n"
                    << generateReport(false) << std::endl;
        }

        frameAllocator_.reset();
        initialized_.store(false, std::memory_order_release);
    }

    void FrameProfiler::beginFrame(const FrameNumber frameNumber) {
        if (!enabled_.load(std::memory_order_acquire)) {
            return;
        }

        // Check if already in frame
        if (bool expected = false; !inFrame_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
            // Already in frame - auto-end previous frame
            endFrame();
            inFrame_.store(true, std::memory_order_release);
        }

        // Reset frame allocator
        if (frameAllocator_) {
            frameAllocator_->reset();
        }

        // Initialize new frame
        frameStartTime_ = Clock::now();
        currentFrameNumber_.store(frameNumber, std::memory_order_release);

        {
            std::lock_guard lock(currentFrameMutex_);
            currentFrame_ = FrameProfileData{};
            currentFrame_.frameNumber = frameNumber;
            currentFrame_.startTime = frameStartTime_;
        }

        // Notify performance tracker
        if (performanceTracker_) {
            performanceTracker_->beginFrame(frameNumber);
        }
    }

    void FrameProfiler::endFrame(const Duration gpuTime) {
        if (!enabled_.load(std::memory_order_acquire) ||
            !inFrame_.load(std::memory_order_acquire)) {
            return;
        }

        {
            const auto endTime = Clock::now();
            std::lock_guard lock(currentFrameMutex_);

            currentFrame_.endTime = endTime;
            currentFrame_.totalDuration = std::chrono::duration_cast<Duration>(
                    endTime - currentFrame_.startTime
                    );

            // Set GPU time if provided
            if (gpuTime != Duration::zero()) {
                currentFrame_.gpuDuration = gpuTime;
            }

            // Calculate CPU duration (total minus GPU overlap)
            currentFrame_.cpuDuration = currentFrame_.totalDuration -
                    currentFrame_.waitForGPU -
                    currentFrame_.waitForVSync;

            // Get memory snapshot if tracking
            if (config_.trackMemory) {
                currentFrame_.memoryUsed = memoryManager_.getTotalMemoryUsage();
            }
        }

        // Process completed frame
        FrameProfileData completedFrame;
        {
            std::lock_guard lock(currentFrameMutex_);
            completedFrame = currentFrame_;
        }

        processFrame(completedFrame);

        // Store in history
        frameHistory_.push(completedFrame);

        // Notify performance tracker
        if (performanceTracker_) {
            performanceTracker_->endFrame(
                    completedFrame.cpuDuration,
                    completedFrame.gpuDuration
                    );
        }

        inFrame_.store(false, std::memory_order_release);
    }

    void FrameProfiler::markFrameDropped(const std::string& reason) {
        if (!enabled_.load(std::memory_order_acquire)) {
            return;
        }

        {
            std::lock_guard lock(currentFrameMutex_);
            currentFrame_.droppedFrame = true;
        }

        consecutiveDrops_.fetch_add(1, std::memory_order_acq_rel);
        totalDrops_.fetch_add(1, std::memory_order_acq_rel);

        // Check for consecutive drops alert
        if (consecutiveDrops_.load(std::memory_order_acquire) >= config_.consecutiveDropsForAlert) {
            if (config_.onPerformanceAlert) {
                config_.onPerformanceAlert("Multiple consecutive frame drops detected: " + reason);
            }
        }

        // Notify performance tracker
        if (performanceTracker_) {
            performanceTracker_->recordFrameDrop(reason);
        }
    }

    void FrameProfiler::markVSyncMiss() {
        if (!enabled_.load(std::memory_order_acquire)) {
            return;
        }

        {
            std::lock_guard lock(currentFrameMutex_);
            currentFrame_.missedVSync = true;
        }

        totalVSyncMisses_.fetch_add(1, std::memory_order_acq_rel);

        // Notify performance tracker
        if (performanceTracker_) {
            performanceTracker_->recordVSyncMiss();
        }
    }

    FrameSectionTracker FrameProfiler::beginSection(const FrameSectionTracker::Section section) {
        return FrameSectionTracker(this, section);
    }

    void FrameProfiler::recordSection(const FrameSectionTracker::Section section, const Duration duration) {
        if (!enabled_.load(std::memory_order_acquire)) {
            return;
        }

        endSection(section, duration);
    }

    void FrameProfiler::recordGPUWait(const Duration duration) {
        if (!enabled_.load(std::memory_order_acquire)) {
            return;
        }

        std::lock_guard lock(currentFrameMutex_);
        currentFrame_.waitForGPU = duration;
    }

    void FrameProfiler::recordVSyncWait(const Duration duration) {
        if (!enabled_.load(std::memory_order_acquire)) {
            return;
        }

        std::lock_guard lock(currentFrameMutex_);
        currentFrame_.waitForVSync = duration;
    }

    void FrameProfiler::recordPresentTime(const Duration duration) {
        if (!enabled_.load(std::memory_order_acquire)) {
            return;
        }

        std::lock_guard lock(currentFrameMutex_);
        currentFrame_.presentDuration = duration;
    }

    std::optional<FrameProfileData> FrameProfiler::getCurrentFrame() const {
        if (!inFrame_.load(std::memory_order_acquire)) {
            return std::nullopt;
        }

        std::lock_guard lock(currentFrameMutex_);
        return currentFrame_;
    }

    std::vector<FrameProfileData> FrameProfiler::getFrameHistory(const std::size_t count) const {
        return frameHistory_.getRecent(count);
    }

    float FrameProfiler::calculateFPS(const std::size_t windowSize) const {
        const auto frames = frameHistory_.getRecent(windowSize);
        if (frames.size() < 2) {
            return 0.0f;
        }

        const auto totalTime = frames.front().endTime - frames.back().startTime;
        const auto totalMs = std::chrono::duration_cast<std::chrono::milliseconds>(totalTime).count();

        if (totalMs == 0) {
            return 0.0f;
        }

        return (frames.size() * 1000.0f) / totalMs;
    }

    Duration FrameProfiler::calculatePercentile(const float percentile) const {
        const auto frames = frameHistory_.getRecent(config_.historySize);
        if (frames.empty()) {
            return Duration::zero();
        }

        std::vector<Duration> frameTimes;
        frameTimes.reserve(frames.size());

        for (const auto& frame : frames) {
            frameTimes.push_back(frame.totalDuration);
        }

        std::ranges::sort(frameTimes);

        const std::size_t index = (percentile / 100.0f) * (frameTimes.size() - 1);

        return frameTimes[index];
    }

    std::vector<FrameNumber> FrameProfiler::detectSpikes() const {
        if (!config_.detectSpikes) {
            return {};
        }

        const auto frames = frameHistory_.getRecent(config_.historySize);
        std::vector<FrameNumber> spikes;

        for (const auto& frame : frames) {
            if (frame.totalDuration > spikeThreshold_) {
                spikes.push_back(frame.frameNumber);
            }
        }

        return spikes;
    }

    float FrameProfiler::predictFrameDropProbability() const {
        if (!config_.predictFrameDrops) {
            return 0.0f;
        }

        const auto frames = frameHistory_.getRecent(30); // Last 0.5s at 60fps
        if (frames.size() < 10) {
            return 0.0f;
        }

        // Simple prediction based on trend
        float avgRecent = 0.0f;
        float avgOlder = 0.0f;
        const std::size_t halfSize = frames.size() / 2;

        for (std::size_t i = 0; i < halfSize; ++i) {
            avgRecent += static_cast<float>(frames[i].totalDuration.count());
            avgOlder += static_cast<float>(frames[i + halfSize].totalDuration.count());
        }

        avgRecent /= halfSize;
        avgOlder /= halfSize;

        // Calculate trend
        const float trend = (avgRecent - avgOlder) / avgOlder;
        const float targetTime = static_cast<float>(config_.targetFrameTime.count());

        // Calculate probability based on current performance and trend
        const float currentRatio = avgRecent / targetTime;
        const float projectedRatio = currentRatio * (1.0f + trend);

        // Map to probability (sigmoid-like)
        const float x = (projectedRatio - 1.0f) * 10.0f;
        return 1.0f / (1.0f + std::exp(-x));
    }

    FrameStats FrameProfiler::getFrameStats() const {
        std::lock_guard lock(statsMutex_);
        return frameStats_;
    }

    // TODO: Revisar esto
    // PerformanceMetrics FrameProfiler::getMetrics() const {
    //     PerformanceMetrics metrics;
    //
    //     const auto frames = frameHistory_.getRecent(config_.historySize);
    //     if (frames.empty()) {
    //         return metrics;
    //     }
    //
    //     // Calculate FPS metrics
    //     metrics.currentFPS = calculateFPS(1);
    //     metrics.averageFPS = calculateFPS(60);
    //     metrics.minFPS = calculateFPS(config_.historySize);
    //
    //     // Calculate frame time percentiles
    //     metrics.frameTimeP50 = calculatePercentile(50.0f);
    //     metrics.frameTimeP95 = calculatePercentile(95.0f);
    //     metrics.frameTimeP99 = calculatePercentile(99.0f);
    //
    //     // Calculate utilization
    //     float totalCPU = 0.0f;
    //     float totalGPU = 0.0f;
    //
    //     for (const auto& frame : frames) {
    //         totalCPU += frame.getCPUUtilization();
    //         totalGPU += frame.getGPUUtilization();
    //     }
    //
    //     metrics.cpuUtilization = totalCPU / frames.size();
    //     metrics.gpuUtilization = totalGPU / frames.size();
    //
    //     // Frame drop metrics
    //     metrics.droppedFrames = totalDrops_.load(std::memory_order_acquire);
    //     metrics.vsyncMisses = totalVSyncMisses_.load(std::memory_order_acquire);
    //
    //     return metrics;
    // }

    std::string FrameProfiler::generateReport(const bool detailed) const {
        std::stringstream ss;

        ss << "=== Frame Profiler Report ===\n";

        // Current performance
        const float currentFPS = calculateFPS(60);
        const float targetFPS = 1000.0f / toMilliseconds(config_.targetFrameTime);

        ss << "Current FPS: " << std::fixed << std::setprecision(1) << currentFPS
                << " / " << targetFPS << " target\n";

        // Frame time percentiles
        ss << "Frame Times:\n";
        ss << "  P50: " << std::setprecision(2) << toMilliseconds(calculatePercentile(50)) << " ms\n";
        ss << "  P95: " << toMilliseconds(calculatePercentile(95)) << " ms\n";
        ss << "  P99: " << toMilliseconds(calculatePercentile(99)) << " ms\n";

        // Frame drops
        const auto drops = totalDrops_.load(std::memory_order_acquire);

        if (const auto vsyncMisses = totalVSyncMisses_.load(std::memory_order_acquire); drops > 0 || vsyncMisses > 0) {
            ss << "\nIssues:\n";
            if (drops > 0) {
                ss << "  Frame Drops: " << drops << "\n";
            }
            if (vsyncMisses > 0) {
                ss << "  VSync Misses: " << vsyncMisses << "\n";
            }
        }

        if (detailed) {
            // Recent spikes
            if (const auto spikes = detectSpikes(); !spikes.empty()) {
                ss << "\nRecent Spikes:\n";
                for (std::size_t i = 0; i < std::min(static_cast<size_t>(5), spikes.size()); ++i) {
                    ss << "  Frame " << spikes[i] << "\n";
                }
            }

            // Frame sections breakdown
            if (const auto frames = frameHistory_.getRecent(30); !frames.empty() && config_.trackSections) {
                Duration avgUpdate = Duration::zero();
                Duration avgPhysics = Duration::zero();
                Duration avgRender = Duration::zero();
                Duration avgAudio = Duration::zero();

                for (const auto& frame : frames) {
                    avgUpdate += frame.updateDuration;
                    avgPhysics += frame.physicsDuration;
                    avgRender += frame.renderDuration;
                    avgAudio += frame.audioDuration;
                }

                const auto count = frames.size();
                avgUpdate /= count;
                avgPhysics /= count;
                avgRender /= count;
                avgAudio /= count;

                ss << "\nSection Breakdown (avg):\n";
                ss << "  Update:  " << std::setprecision(2) << toMilliseconds(avgUpdate) << " ms\n";
                ss << "  Physics: " << toMilliseconds(avgPhysics) << " ms\n";
                ss << "  Render:  " << toMilliseconds(avgRender) << " ms\n";
                ss << "  Audio:   " << toMilliseconds(avgAudio) << " ms\n";
            }

            // Prediction
            if (const float dropProbability = predictFrameDropProbability(); dropProbability > 0.5f) {
                ss << "\n⚠ High frame drop risk: " << std::setprecision(0)
                        << (dropProbability * 100) << "%\n";
            }
        }

        return ss.str();
    }

    std::string FrameProfiler::analyzeFramePacing() const {
        std::stringstream ss;

        ss << "=== Frame Pacing Analysis ===\n";

        const auto frames = frameHistory_.getRecent(120); // 2 seconds at 60fps
        if (frames.size() < 2) {
            return "Insufficient data for pacing analysis";
        }

        // Calculate frame-to-frame deltas
        std::vector<Duration> deltas;
        deltas.reserve(frames.size() - 1);

        for (std::size_t i = 1; i < frames.size(); ++i) {
            const auto delta = frames[i - 1].startTime - frames[i].startTime;
            deltas.push_back(std::chrono::duration_cast<Duration>(delta));
        }

        // Calculate variance
        const Duration avgDelta = std::accumulate(deltas.begin(), deltas.end(), Duration::zero()) / deltas.size();

        double variance = 0.0;
        for (const auto& delta : deltas) {
            const double diff = static_cast<double>((delta - avgDelta).count());
            variance += diff * diff;
        }
        variance /= deltas.size();
        const double stdDev = std::sqrt(variance);

        ss << "Average Frame Interval: " << std::fixed << std::setprecision(2)
                << toMilliseconds(avgDelta) << " ms\n";
        ss << "Standard Deviation: " << std::setprecision(3)
                << (stdDev / 1000.0) << " ms\n";

        // Categorize pacing quality
        const double targetMs = toMilliseconds(config_.targetFrameTime);
        const double coefficientOfVariation = (stdDev / 1000.0) / targetMs;

        ss << "Pacing Quality: ";
        if (coefficientOfVariation < 0.05) {
            ss << "Excellent";
        } else if (coefficientOfVariation < 0.10) {
            ss << "Good";
        } else if (coefficientOfVariation < 0.20) {
            ss << "Fair";
        } else {
            ss << "Poor";
        }
        ss << " (CV: " << std::setprecision(1) << (coefficientOfVariation * 100) << "%)\n";

        // Identify stutters (frames significantly longer than average)
        std::uint32_t stutters = 0;
        const Duration stutterThreshold = avgDelta + Duration(static_cast<std::int64_t>(stdDev * 2));

        for (const auto& delta : deltas) {
            if (delta > stutterThreshold) {
                stutters++;
            }
        }

        if (stutters > 0) {
            ss << "Stutters Detected: " << stutters << " ("
                    << std::setprecision(1) << (stutters * 100.0 / deltas.size()) << "%)\n";
        }

        return ss.str();
    }

    void FrameProfiler::updateConfig(const FrameProfilerConfig& config) {
        config_ = config;

        // Update thresholds
        spikeThreshold_ = Duration(
                static_cast<std::int64_t>(
                    config_.targetFrameTime.count() * config_.spikeThresholdMultiplier)
                );
    }

    void FrameProfiler::reset() {
        // End current frame if active
        if (inFrame_.load(std::memory_order_acquire)) {
            endFrame();
        }

        // Clear history
        frameHistory_.clear();

        // Reset stats
        {
            std::lock_guard lock(statsMutex_);
            frameStats_.reset();
        }

        // Reset counters
        consecutiveDrops_.store(0, std::memory_order_release);
        totalDrops_.store(0, std::memory_order_release);
        totalVSyncMisses_.store(0, std::memory_order_release);
        currentFrameNumber_.store(0, std::memory_order_release);
    }

    // =============================================================================
    // Private Methods
    // =============================================================================

    void FrameProfiler::endSection(const FrameSectionTracker::Section section, const Duration duration) {
        if (!enabled_.load(std::memory_order_acquire)) {
            return;
        }

        std::lock_guard lock(currentFrameMutex_);

        switch (section) {
            case FrameSectionTracker::Section::UPDATE:
                currentFrame_.updateDuration = duration;
                break;
            case FrameSectionTracker::Section::PHYSICS:
                currentFrame_.physicsDuration = duration;
                break;
            case FrameSectionTracker::Section::RENDER:
                currentFrame_.renderDuration = duration;
                break;
            case FrameSectionTracker::Section::AUDIO:
                currentFrame_.audioDuration = duration;
                break;
            case FrameSectionTracker::Section::PRESENT:
                currentFrame_.presentDuration = duration;
                break;
            case FrameSectionTracker::Section::CUSTOM:
                // Custom sections could be accumulated separately
                break;
        }

        // Update system profiler if available
        if (systemProfiler_ && section != FrameSectionTracker::Section::CUSTOM) {
            const char* sectionName = nullptr;
            switch (section) {
                case FrameSectionTracker::Section::UPDATE:
                    sectionName = "Update";
                    break;
                case FrameSectionTracker::Section::PHYSICS:
                    sectionName = "Physics";
                    break;
                case FrameSectionTracker::Section::RENDER:
                    sectionName = "Render";
                    break;
                case FrameSectionTracker::Section::AUDIO:
                    sectionName = "Audio";
                    break;
                case FrameSectionTracker::Section::PRESENT:
                    sectionName = "Present";
                    break;
                default:
                    break;
            }

            if (sectionName) {
                systemProfiler_->recordSystemExecution(sectionName, duration, "Frame");
            }
        }
    }

    void FrameProfiler::processFrame(const FrameProfileData& frame) {
        // Update statistics
        updateStats(frame);

        // Check budget
        checkBudget(frame);

        // Trigger callbacks
        triggerCallbacks(frame);

        // Update spike threshold
        updateSpikeThreshold();

        // Reset consecutive drops if frame was good
        if (!frame.droppedFrame) {
            consecutiveDrops_.store(0, std::memory_order_release);
        }
    }

    void FrameProfiler::updateStats(const FrameProfileData& frame) {
        std::lock_guard lock(statsMutex_);

        // Update frame duration stats (esto maneja min/max/sum/count automáticamente)
        frameStats_.frameDuration.update(frame.totalDuration, frame.endTime);

        // Update CPU and GPU times
        if (frame.cpuDuration != Duration::zero()) {
            frameStats_.cpuTime.update(frame.cpuDuration, frame.endTime);
        }
        if (frame.gpuDuration != Duration::zero()) {
            frameStats_.gpuTime.update(frame.gpuDuration, frame.endTime);
        }
        if (frame.presentDuration != Duration::zero()) {
            frameStats_.presentTime.update(frame.presentDuration, frame.endTime);
        }

        // Update FPS metrics
        const auto avgFrameTime = frameStats_.frameDuration.getAverage();
        if (avgFrameTime.count() > 0) {
            frameStats_.averageFPS.store(
                    1000000.0f / static_cast<float>(avgFrameTime.count()),
                    std::memory_order_relaxed
                    );
        }

        // Calculate instant FPS
        if (frame.totalDuration.count() > 0) {
            frameStats_.instantFPS.store(
                    1000000.0f / static_cast<float>(frame.totalDuration.count()),
                    std::memory_order_relaxed
                    );
        }

        // Update variance for standard deviation calculation
        // Usar la fórmula de varianza incremental (Welford's algorithm)
        const auto sampleCount = frameStats_.frameDuration.sampleCount.load(std::memory_order_acquire);
        if (sampleCount > 1) {
            const double delta = static_cast<double>(
                (frame.totalDuration - avgFrameTime).count()
            );
            const double currentVariance = frameStats_.frameTimeVariance.load(
                    std::memory_order_relaxed
                    ).count();

            // Varianza incremental: var_n = var_(n-1) + (delta^2 - var_(n-1)) / n
            const double newVariance = currentVariance +
                    (delta * delta - currentVariance) / static_cast<double>(sampleCount);

            frameStats_.frameTimeVariance.store(
                    Duration(static_cast<std::int64_t>(newVariance)),
                    std::memory_order_relaxed
                    );

            // Calcular desviación estándar
            frameStats_.frameTimeStdDev.store(
                    Duration(static_cast<std::int64_t>(std::sqrt(newVariance))),
                    std::memory_order_relaxed
                    );

            // Calcular coeficiente de variación (CV = stdDev / mean)
            if (avgFrameTime.count() > 0) {
                const float cv = static_cast<float>(std::sqrt(newVariance)) /
                        static_cast<float>(avgFrameTime.count());
                frameStats_.frameTimeCV.store(cv, std::memory_order_relaxed);
            }
        }

        // Update drop rate
        if (const std::uint32_t totalFrames = static_cast<std::uint32_t>(sampleCount); totalFrames > 0) {
            const std::uint32_t drops = frameStats_.droppedFrames.load(std::memory_order_relaxed);
            const float dropRate = (static_cast<float>(drops) / static_cast<float>(totalFrames)) * 100.0f;
            frameStats_.dropRate.store(dropRate, std::memory_order_relaxed);
        }
    }

    void FrameProfiler::checkBudget(const FrameProfileData& frame) const {
        const float budgetUsage = static_cast<float>(frame.totalDuration.count()) /
                static_cast<float>(config_.targetFrameTime.count());

        if (budgetUsage > 1.0f) {
            const_cast<FrameProfileData&>(frame).budgetExceeded = true;

            if (budgetUsage > (config_.criticalThresholdPercent / 100.0f)) {
                if (config_.onPerformanceAlert) {
                    config_.onPerformanceAlert(
                            "Critical: Frame time exceeded budget by " +
                            std::to_string(static_cast<int>((budgetUsage - 1.0f) * 100)) + "%"
                            );
                }
            }
        }
    }

    void FrameProfiler::triggerCallbacks(const FrameProfileData& frame) const {
        if (frame.droppedFrame && config_.onFrameDrop) {
            config_.onFrameDrop(frame);
        }

        if (frame.missedVSync && config_.onVSyncMiss) {
            config_.onVSyncMiss(frame);
        }

        // Update FPS callback periodically
        if (config_.onFPSUpdate && frame.frameNumber % 60 == 0) {
            config_.onFPSUpdate(calculateFPS(60));
        }
    }

    void FrameProfiler::updateSpikeThreshold() {
        // Adaptive spike threshold based on recent performance
        const auto frames = frameHistory_.getRecent(60);
        if (frames.size() < 30) {
            return;
        }

        Duration total = Duration::zero();
        for (const auto& frame : frames) {
            total += frame.totalDuration;
        }

        averageFrameTime_ = total / frames.size();
        spikeThreshold_ = Duration(
                static_cast<std::int64_t>(
                    averageFrameTime_.count() * config_.spikeThresholdMultiplier)
                );
    }

} // namespace engine::time
