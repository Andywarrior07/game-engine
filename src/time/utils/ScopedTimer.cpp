/**
 * @file ScopedTimer.cpp
 * @brief Implementation of RAII timer utilities
 * @details Provides automatic timing and reporting functionality
 *
 * @author Development Team
 * @date Created on 2024-01-20
 */

#include "ScopedTimer.h"
#include "TimeFormatter.h"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>

namespace engine::time {

    // =============================================================================
    // Static SDL Backend Instance
    // =============================================================================

    namespace {
        /**
         * @brief Get default time backend
         * @details Thread-safe lazy initialization of SDL backend
         * @return Pointer to initialized SDL backend
         *
         * Implementation notes:
         * - Static local variable ensures single initialization
         * - std::call_once guarantees thread-safe initialization
         * - Backend is never destroyed (lives for program duration)
         */
        ITimeBackend* getDefaultBackend() noexcept {
            static SDLTimeBackend defaultBackend;
            static std::once_flag initFlag;

            std::call_once(
                    initFlag,
                    [&]() {
                        ClockConfig config;
                        config.useHighPrecisionClock = true;
                        defaultBackend.initialize(config);
                    }
                    );

            return &defaultBackend;
        }
    }

    // =============================================================================
    // TimerReport Implementation
    // =============================================================================

    std::string TimerReport::toString() const {
        std::stringstream ss;

        ss << "[Timer: " << name << "] ";
        ss << "Elapsed: " << TimeFormatter::format(elapsed, FormatStyle::HUMAN);

        // Only show statistics if we have multiple measurements
        if (callCount > 1) {
            ss << " (Calls: " << callCount;
            ss << ", Avg: " << TimeFormatter::format(getAverage(), FormatStyle::COMPACT);
            ss << ", Min: " << TimeFormatter::format(minTime, FormatStyle::COMPACT);
            ss << ", Max: " << TimeFormatter::format(maxTime, FormatStyle::COMPACT);
            ss << ", Total: " << TimeFormatter::format(totalTime, FormatStyle::COMPACT);
            ss << ")";
        }

        return ss.str();
    }

    // =============================================================================
    // ScopedTimer Implementation
    // =============================================================================

    ScopedTimer::ScopedTimer(
            std::string name,
            const ReportDestination destination,
            ScopedTimerCallback callback,
            TimerManager* manager
            ) noexcept :
        backend_(getDefaultBackend())
        , destination_(destination)
        , callback_(std::move(callback))
        , manager_(manager) {

        // Initialize report structure
        report_.name = std::move(name);
        report_.startTime = backend_->now();
        report_.threadId = std::this_thread::get_id();
        lastLapTime_ = report_.startTime;
    }

    ScopedTimer::ScopedTimer(
            std::string name,
            ITimeBackend* backend,
            const ReportDestination destination,
            ScopedTimerCallback callback,
            TimerManager* manager
            ) noexcept :
        backend_(backend ? backend : getDefaultBackend())
        , destination_(destination)
        , callback_(std::move(callback))
        , manager_(manager) {

        // Initialize report structure
        report_.name = std::move(name);
        report_.startTime = backend_->now();
        report_.threadId = std::this_thread::get_id();
        lastLapTime_ = report_.startTime;
    }

    ScopedTimer::~ScopedTimer() noexcept {
        // Only report if timer wasn't moved and is still running
        if (!moved_ && running_) {
            stop();
            report();
        }
    }

    ScopedTimer::ScopedTimer(ScopedTimer&& other) noexcept :
        backend_(other.backend_)
        , report_(std::move(other.report_))
        , destination_(other.destination_)
        , callback_(std::move(other.callback_))
        , manager_(other.manager_)
        , running_(other.running_.load())
        , lastLapTime_(other.lastLapTime_)
        , moved_(false) {

        // Mark source as moved-from
        other.moved_ = true;
        other.running_ = false;
        other.manager_ = nullptr;
    }

    ScopedTimer& ScopedTimer::operator=(ScopedTimer&& other) noexcept {
        if (this != &other) {
            // Stop current timer if running
            if (!moved_ && running_) {
                stop();
                report();
            }

            // Transfer ownership
            backend_ = other.backend_;
            report_ = std::move(other.report_);
            destination_ = other.destination_;
            callback_ = std::move(other.callback_);
            manager_ = other.manager_;
            running_ = other.running_.load();
            lastLapTime_ = other.lastLapTime_;
            moved_ = false;

            // Mark source as moved-from
            other.moved_ = true;
            other.running_ = false;
            other.manager_ = nullptr;
        }
        return *this;
    }

    Duration ScopedTimer::stop() noexcept {
        // Early exit if already stopped
        if (!running_) {
            return report_.elapsed;
        }

        // Capture end time and calculate elapsed duration
        report_.endTime = backend_->now();
        report_.elapsed = std::chrono::duration_cast<Duration>(
                report_.endTime - report_.startTime
                );

        // Initialize statistics (for single measurement)
        report_.totalTime = report_.elapsed;
        report_.minTime = report_.elapsed;
        report_.maxTime = report_.elapsed;

        running_ = false;

        return report_.elapsed;
    }

    void ScopedTimer::restart() noexcept {
        report_.startTime = backend_->now();
        lastLapTime_ = report_.startTime;
        running_ = true;
    }

    Duration ScopedTimer::getElapsed() const noexcept {
        if (!running_) {
            return report_.elapsed;
        }

        // Calculate current elapsed time without stopping
        const TimeStamp now = backend_->now();
        return std::chrono::duration_cast<Duration>(now - report_.startTime);
    }

    TimerReport ScopedTimer::getReport() const noexcept {
        TimerReport currentReport = report_;

        // Update elapsed time if still running
        if (running_) {
            currentReport.endTime = backend_->now();
            currentReport.elapsed = std::chrono::duration_cast<Duration>(
                    currentReport.endTime - currentReport.startTime
                    );
        }

        return currentReport;
    }

    Duration ScopedTimer::lap(const std::string& lapName) noexcept {
        const TimeStamp now = backend_->now();
        const auto lapTime = std::chrono::duration_cast<Duration>(
                now - lastLapTime_
                );

        // Optional: Report lap time to console
        if (!lapName.empty() && destination_ == ReportDestination::CONSOLE) {
            std::cout << "[Lap: " << report_.name;
            if (!lapName.empty()) {
                std::cout << " - " << lapName;
            }
            std::cout << "] " << TimeFormatter::format(lapTime, FormatStyle::HUMAN)
                    << std::endl;
        }

        lastLapTime_ = now;
        return lapTime;
    }

    void ScopedTimer::report() const noexcept {
        // Route report to configured destination
        switch (destination_) {
            case ReportDestination::CONSOLE:
                std::cout << report_.toString() << std::endl;
                break;

            case ReportDestination::CALLBACK:
                if (callback_) {
                    callback_(report_);
                }
                break;

            case ReportDestination::MANAGER:
                // Send to TimerManager if available
                if (manager_) {
                    manager_->registerReport(report_);
                }
                break;

            case ReportDestination::LOG:
                // Future: integrate with logging system
                // For now, fallback to console
                std::cout << report_.toString() << std::endl;
                break;

            case ReportDestination::NONE:
            default:
                break;
        }

        // Always invoke callback if provided (even if destination is different)
        if (callback_ && destination_ != ReportDestination::CALLBACK) {
            callback_(report_);
        }
    }

    // =============================================================================
    // AccumulatingTimer Implementation
    // =============================================================================

    AccumulatingTimer::AccumulatingTimer(
            std::string name,
            ITimeBackend* backend
            ) noexcept :
        backend_(backend ? backend : getDefaultBackend()) {

        report_.name = std::move(name);
        report_.callCount = 0;
    }

    void AccumulatingTimer::start() noexcept {
        std::lock_guard lock(mutex_);

        if (running_) {
            return; // Already running, ignore
        }

        currentStart_ = backend_->now();
        running_ = true;
    }

    Duration AccumulatingTimer::stop() noexcept {
        std::lock_guard lock(mutex_);

        if (!running_) {
            return Duration::zero();
        }

        // Measure elapsed time
        const TimeStamp endTime = backend_->now();
        const auto elapsed = std::chrono::duration_cast<Duration>(
                endTime - currentStart_
                );

        // Update accumulated statistics
        report_.elapsed = elapsed;
        report_.endTime = endTime;
        report_.totalTime += elapsed;
        report_.callCount++;

        // Update min/max
        if (elapsed < report_.minTime) {
            report_.minTime = elapsed;
        }
        if (elapsed > report_.maxTime) {
            report_.maxTime = elapsed;
        }

        running_ = false;
        return elapsed;
    }

    void AccumulatingTimer::reset() noexcept {
        std::lock_guard lock(mutex_);

        // Reset all statistics
        report_.callCount = 0;
        report_.minTime = Duration::max();
        report_.maxTime = Duration::min();
        report_.totalTime = Duration::zero();
        running_ = false;
    }

    TimerReport AccumulatingTimer::getReport() const noexcept {
        std::lock_guard lock(mutex_);
        return report_;
    }

    ScopedTimer AccumulatingTimer::createScoped() noexcept {
        // Create scoped timer that feeds back into this accumulator
        return ScopedTimer(
                report_.name,
                backend_,
                ReportDestination::CALLBACK,
                [this](const TimerReport& report) {
                    accumulate(report);
                },
                nullptr // No manager needed, using callback
                );
    }

    void AccumulatingTimer::accumulate(const TimerReport& measurement) noexcept {
        std::lock_guard lock(mutex_);

        // Add measurement to accumulated statistics
        report_.totalTime += measurement.elapsed;
        report_.callCount++;

        // Update min/max
        if (measurement.elapsed < report_.minTime) {
            report_.minTime = measurement.elapsed;
        }
        if (measurement.elapsed > report_.maxTime) {
            report_.maxTime = measurement.elapsed;
        }

        // Store latest measurement details
        report_.elapsed = measurement.elapsed;
        report_.endTime = measurement.endTime;
    }

    // =============================================================================
    // TimerManager Implementation
    // =============================================================================

    void TimerManager::registerReport(const TimerReport& report) noexcept {
        if (!enabled_) {
            return;
        }

        std::lock_guard lock(mutex_);
        reports_.push_back(report);
    }

    std::vector<TimerReport> TimerManager::getReports() const noexcept {
        std::lock_guard lock(mutex_);
        return reports_;
    }

    void TimerManager::clearReports() noexcept {
        std::lock_guard lock(mutex_);
        reports_.clear();
    }

    void TimerManager::printSummary() const noexcept {
        std::lock_guard lock(mutex_);

        if (reports_.empty()) {
            std::cout << "No timer reports available." << std::endl;
            return;
        }

        std::cout << "\n=== Timer Summary ===" << std::endl;
        std::cout << std::left;

        // Calculate total time across all reports
        Duration totalTime = Duration::zero();
        for (const auto& report : reports_) {
            totalTime += report.elapsed;
        }

        // Print table header
        std::cout << std::setw(30) << "Timer Name"
                << std::setw(15) << "Time"
                << std::setw(10) << "Calls"
                << std::setw(15) << "Average"
                << std::setw(10) << "Percent"
                << std::endl;
        std::cout << std::string(80, '-') << std::endl;

        // Sort reports by total time (descending)
        std::vector<TimerReport> sortedReports = reports_;
        std::ranges::sort(
                sortedReports,
                [](const TimerReport& a, const TimerReport& b) {
                    return a.totalTime > b.totalTime;
                }
                );

        // Print each timer's statistics
        for (const auto& report : sortedReports) {
            const double percent = (totalTime != Duration::zero())
                    ? (toSeconds(report.totalTime) / toSeconds(totalTime)) * 100.0
                    : 0.0;

            std::cout << std::setw(30) << report.name
                    << std::setw(15) << TimeFormatter::format(report.totalTime, FormatStyle::COMPACT)
                    << std::setw(10) << report.callCount
                    << std::setw(15) << TimeFormatter::format(report.getAverage(), FormatStyle::COMPACT)
                    << std::setw(9) << std::fixed << std::setprecision(1) << percent << "%"
                    << std::endl;
        }

        std::cout << std::string(80, '=') << std::endl;
        std::cout << "Total Time: " << TimeFormatter::format(totalTime, FormatStyle::HUMAN) << std::endl;
        std::cout << std::endl;
    }

} // namespace engine::time
