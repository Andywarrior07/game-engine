/**
 * @file ScopedTimer.h
 * @brief RAII timer for automatic profiling and benchmarking
 * @details Provides zero-overhead scoped timing with automatic reporting
 *          on destruction. Supports nested timers and custom callbacks.
 *
 * @author Development Team
 * @date Created on 2024-01-20
 */

#pragma once

#include "../core/TimeTypes.h"
#include "../core/ITimeBackend.h"
#include "../backends/SDLTimeBackend.h"

#include <string>
#include <functional>
#include <vector>
#include <mutex>
#include <atomic>

namespace engine::time {

    // Forward declaration
    class TimerManager;

    // =============================================================================
    // Timer Report Structure
    // =============================================================================

    /**
     * @brief Timing report data
     * @details Contains all timing information for a single measurement
     */
    struct TimerReport {
        std::string name; ///< Timer name/label
        TimeStamp startTime; ///< Start timestamp
        TimeStamp endTime; ///< End timestamp
        Duration elapsed; ///< Elapsed duration
        std::size_t callCount{1}; ///< Number of times measured
        Duration minTime{Duration::max()}; ///< Minimum time recorded
        Duration maxTime{Duration::min()}; ///< Maximum time recorded
        Duration totalTime{Duration::zero()}; ///< Total accumulated time
        std::thread::id threadId; ///< Thread that created timer

        /**
         * @brief Calculate average time
         * @return Average duration across all calls
         */
        [[nodiscard]] Duration getAverage() const noexcept {
            if (callCount == 0)
                return Duration::zero();
            return totalTime / callCount;
        }

        /**
         * @brief Get formatted report string
         * @return Human-readable report
         */
        [[nodiscard]] std::string toString() const;
    };

    // =============================================================================
    // Report Callbacks
    // =============================================================================

    /**
     * @brief Callback for timer completion
     * @details Invoked when timer stops, receives timing report
     */
    using ScopedTimerCallback = std::function<void(const TimerReport&)>;

    /**
     * @brief Report output destination
     * @details Determines where timing results are sent
     */
    enum class ReportDestination : std::uint8_t {
        NONE, ///< No output
        CONSOLE, ///< std::cout
        LOG, ///< Logging system (future)
        CALLBACK, ///< Custom callback
        MANAGER ///< Send to TimerManager for batch reporting
    };

    // =============================================================================
    // ScopedTimer Class
    // =============================================================================

    /**
     * @brief RAII timer for automatic profiling
     * @details Measures time from construction to destruction.
     *          Thread-safe and supports nested timing.
     *
     * Usage:
     * @code
     * {
     *     ScopedTimer timer("MyOperation");
     *     // ... code to measure ...
     * } // Reports on destruction
     * @endcode
     */
    class ScopedTimer {
    public:
        /**
         * @brief Construct timer and start measuring
         * @param name Timer label for reporting
         * @param destination Output destination
         * @param callback Optional completion callback
         * @param manager Optional TimerManager for batch reporting
         */
        explicit ScopedTimer(
                std::string name,
                ReportDestination destination = ReportDestination::CONSOLE,
                ScopedTimerCallback callback = nullptr,
                TimerManager* manager = nullptr
                ) noexcept;

        /**
         * @brief Construct with time backend
         * @param name Timer label
         * @param backend Custom time backend (non-owning pointer)
         * @param destination Output destination
         * @param callback Optional completion callback
         * @param manager Optional TimerManager for batch reporting
         */
        ScopedTimer(
                std::string name,
                ITimeBackend* backend,
                ReportDestination destination = ReportDestination::CONSOLE,
                ScopedTimerCallback callback = nullptr,
                TimerManager* manager = nullptr
                ) noexcept;

        /**
         * @brief Destructor - stops timer and reports
         * @details Automatically reports timing if still running
         */
        ~ScopedTimer() noexcept;

        // Delete copy operations (timers are unique resources)
        ScopedTimer(const ScopedTimer&) = delete;
        ScopedTimer& operator=(const ScopedTimer&) = delete;

        // Allow move operations (transfer ownership)
        ScopedTimer(ScopedTimer&& other) noexcept;
        ScopedTimer& operator=(ScopedTimer&& other) noexcept;

        /**
         * @brief Stop timer manually
         * @return Elapsed duration
         * @details Can be called multiple times, subsequent calls return cached value
         */
        Duration stop() noexcept;

        /**
         * @brief Restart the timer
         * @details Resets start time to now
         */
        void restart() noexcept;

        /**
         * @brief Get elapsed time without stopping
         * @return Current elapsed duration
         * @details Non-destructive, timer continues running
         */
        [[nodiscard]] Duration getElapsed() const noexcept;

        /**
         * @brief Check if timer is still running
         * @return True if timer is active
         */
        [[nodiscard]] bool isRunning() const noexcept {
            return running_;
        }

        /**
         * @brief Get timer name
         * @return Timer label string
         */
        [[nodiscard]] const std::string& getName() const noexcept {
            return report_.name;
        }

        /**
         * @brief Get current report
         * @return TimerReport with current timing data
         */
        [[nodiscard]] TimerReport getReport() const noexcept;

        /**
         * @brief Mark a lap/checkpoint
         * @param lapName Optional lap label
         * @return Duration since last lap
         */
        Duration lap(const std::string& lapName = "") noexcept;

    private:
        ITimeBackend* backend_; ///< Time backend (not owned)
        TimerReport report_; ///< Timer report data
        ReportDestination destination_; ///< Output destination
        ScopedTimerCallback callback_; ///< Completion callback
        TimerManager* manager_; ///< Optional manager (not owned)
        std::atomic<bool> running_{true}; ///< Is timer running
        TimeStamp lastLapTime_; ///< Last lap timestamp
        bool moved_{false}; ///< Has been moved from

        /**
         * @brief Report timer results
         * @details Sends report to configured destination
         */
        void report() const noexcept;
    };

    // =============================================================================
    // Accumulating Timer
    // =============================================================================

    /**
     * @brief Timer that accumulates multiple measurements
     * @details Useful for profiling code that runs multiple times.
     *          Thread-safe for concurrent start/stop operations.
     *
     * Usage:
     * @code
     * AccumulatingTimer timer("LoopOperation");
     * for (int i = 0; i < 1000; ++i) {
     *     timer.start();
     *     // ... operation ...
     *     timer.stop();
     * }
     * auto report = timer.getReport(); // Contains min/max/avg stats
     * @endcode
     */
    class AccumulatingTimer {
    public:
        /**
         * @brief Constructor
         * @param name Timer name
         * @param backend Optional time backend (defaults to SDL backend)
         */
        explicit AccumulatingTimer(
                std::string name,
                ITimeBackend* backend = nullptr
                ) noexcept;

        /**
         * @brief Start a new measurement
         * @details No-op if already running
         */
        void start() noexcept;

        /**
         * @brief Stop current measurement
         * @return Duration of this measurement
         * @details Updates min/max/average statistics
         */
        Duration stop() noexcept;

        /**
         * @brief Reset all accumulated data
         * @details Clears all statistics, prepares for new measurements
         */
        void reset() noexcept;

        /**
         * @brief Get accumulated report
         * @return TimerReport with aggregated statistics
         */
        [[nodiscard]] TimerReport getReport() const noexcept;

        /**
         * @brief Create scoped timer that adds to accumulator
         * @return ScopedTimer that reports to this accumulator
         * @details Each scoped timer destruction adds to this accumulator's stats
         */
        [[nodiscard]] ScopedTimer createScoped() noexcept;

    private:
        mutable std::mutex mutex_; ///< Thread safety for statistics
        ITimeBackend* backend_; ///< Time backend (not owned)
        TimerReport report_; ///< Accumulated report
        TimeStamp currentStart_; ///< Current measurement start
        std::atomic<bool> running_{false}; ///< Is currently measuring

        /**
         * @brief Accumulate measurement
         * @param measurement Timing data to add to statistics
         */
        void accumulate(const TimerReport& measurement) noexcept;

        friend class ScopedTimer;
    };

    // =============================================================================
    // Timer Manager (Non-Singleton)
    // =============================================================================

    /**
     * @brief Timer registry for batch reporting
     * @details Thread-safe collection of timing data for analysis.
     *          Should be owned by application/system that needs profiling.
     *
     * Ownership:
     * - Create one TimerManager per profiling context
     * - Pass pointer to timers that should report to it
     * - Call printSummary() to analyze collected data
     *
     * Example:
     * @code
     * TimerManager manager;
     * manager.setEnabled(true);
     * {
     *     ScopedTimer t1("Op1", ReportDestination::MANAGER, nullptr, &manager);
     *     ScopedTimer t2("Op2", ReportDestination::MANAGER, nullptr, &manager);
     * }
     * manager.printSummary(); // Shows statistics for both timers
     * @endcode
     */
    class TimerManager {
    public:
        /**
         * @brief Default constructor
         * @details Initializes disabled by default
         */
        TimerManager() noexcept = default;

        /**
         * @brief Destructor
         */
        ~TimerManager() = default;

        // Delete copy operations (manager owns collected data)
        TimerManager(const TimerManager&) = delete;
        TimerManager& operator=(const TimerManager&) = delete;

        // Allow move operations (transfer ownership of reports)
        TimerManager(TimerManager&& other) noexcept :
            reports_(std::move(other.reports_))
            , enabled_(other.enabled_.load()) {
            other.enabled_ = false;
        }

        TimerManager& operator=(TimerManager&& other) noexcept {
            if (this != &other) {
                std::lock_guard lock(mutex_);
                std::lock_guard otherLock(other.mutex_);

                reports_ = std::move(other.reports_);
                enabled_ = other.enabled_.load();
                other.enabled_ = false;
            }
            return *this;
        }

        /**
         * @brief Register timer report
         * @param report Timer report to register
         * @details Thread-safe, only registers if enabled
         */
        void registerReport(const TimerReport& report) noexcept;

        /**
         * @brief Get all reports
         * @return Vector of all timer reports (copy)
         */
        [[nodiscard]] std::vector<TimerReport> getReports() const noexcept;

        /**
         * @brief Clear all reports
         * @details Thread-safe, removes all collected timing data
         */
        void clearReports() noexcept;

        /**
         * @brief Print summary of all timers
         * @details Outputs formatted table with statistics to console
         */
        void printSummary() const noexcept;

        /**
         * @brief Enable/disable global timer collection
         * @param enabled True to collect reports
         */
        void setEnabled(const bool enabled) noexcept {
            enabled_ = enabled;
        }

        /**
         * @brief Check if collection is enabled
         * @return True if collecting reports
         */
        [[nodiscard]] bool isEnabled() const noexcept {
            return enabled_;
        }

    private:
        mutable std::mutex mutex_; ///< Thread safety
        std::vector<TimerReport> reports_; ///< Collected reports
        std::atomic<bool> enabled_{false}; ///< Collection enabled
    };

    // =============================================================================
    // Convenience Macros
    // =============================================================================

#ifdef DEBUG
    /**
     * @brief Create scoped timer (debug only)
     * @param name Timer name
     * @details Removed in release builds
     */
#define SCOPED_TIMER(name) \
        engine::time::ScopedTimer _timer_##__LINE__(name)

    /**
     * @brief Create scoped timer with custom destination
     * @param name Timer name
     * @param dest Report destination
     */
#define SCOPED_TIMER_DEST(name, dest) \
        engine::time::ScopedTimer _timer_##__LINE__(name, dest)

    /**
     * @brief Create scoped timer with manager
     * @param name Timer name
     * @param mgr Pointer to TimerManager
     */
#define SCOPED_TIMER_MGR(name, mgr) \
        engine::time::ScopedTimer _timer_##__LINE__( \
            name, engine::time::ReportDestination::MANAGER, nullptr, mgr)

    /**
     * @brief Create function timer
     * @details Uses __FUNCTION__ as timer name
     */
#define FUNCTION_TIMER() \
        engine::time::ScopedTimer _timer_##__LINE__(__FUNCTION__)

#else
    // No-op in release builds (zero overhead)
#define SCOPED_TIMER(name) ((void)0)
#define SCOPED_TIMER_DEST(name, dest) ((void)0)
#define SCOPED_TIMER_MGR(name, mgr) ((void)0)
#define FUNCTION_TIMER() ((void)0)
#endif

} // namespace engine::time
