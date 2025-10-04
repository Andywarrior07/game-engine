/**
 * @file SystemProfiler.h
 * @brief Per-system performance profiling and budget management
 * @details Provides detailed profiling for individual engine systems with
 *          budget tracking, performance analysis, and automatic optimization
 *          recommendations. Thread-safe and optimized for minimal overhead.
 *
 * @author Development Team
 * @date Created on 2024-01-20
 */

#pragma once

#include "../core/TimeTypes.h"
#include "../profiling/TimeStats.h"
#include "../profiling/ProfilingConfig.h"
#include "../profiling/PerformanceTracker.h"
#include "../utils/ScopedTimer.h"

#include <unordered_map>
#include <atomic>
#include <mutex>
#include <thread>

namespace engine::time {

    // =============================================================================
    // System Profile Data
    // =============================================================================

    /**
     * @brief Detailed profile data for a single system
     */
    struct SystemProfile {
        std::string name; ///< System identifier
        SystemBudget budget; ///< Performance budget
        BasicTimeStats stats; ///< Timing statistics

        // Performance metrics
        std::atomic<Duration> lastDuration{}; ///< Last execution time
        std::atomic<Duration> worstDuration{}; ///< Worst case time
        std::atomic<std::uint32_t> budgetExceeds{0}; ///< Budget exceed count
        std::atomic<std::uint32_t> warningCount{0}; ///< Warning count
        std::atomic<std::uint32_t> criticalCount{0}; ///< Critical count

        // Thread tracking
        std::thread::id primaryThread{}; ///< Primary execution thread
        std::atomic<std::uint32_t> threadSwitches{0}; ///< Thread switch count

        // Call graph data
        struct CallData {
            std::string caller; ///< Calling system
            std::uint32_t callCount{0}; ///< Number of calls
            Duration totalTime{}; ///< Total time in calls
        };

        std::vector<CallData> callGraph; ///< Call relationships

        // =========================================================================
        // Constructors and Assignment Operators
        // =========================================================================

        SystemProfile() = default;

        /**
         * @brief Copy constructor - manually transfer atomic values
         * @details Required because std::atomic deletes copy constructor
         */
        SystemProfile(const SystemProfile& other) noexcept :
            name(other.name)
            , budget(other.budget)
            , stats(other.stats)
            , lastDuration(other.lastDuration.load(std::memory_order_relaxed))
            , worstDuration(other.worstDuration.load(std::memory_order_relaxed))
            , budgetExceeds(other.budgetExceeds.load(std::memory_order_relaxed))
            , warningCount(other.warningCount.load(std::memory_order_relaxed))
            , criticalCount(other.criticalCount.load(std::memory_order_relaxed))
            , primaryThread(other.primaryThread)
            , threadSwitches(other.threadSwitches.load(std::memory_order_relaxed))
            , callGraph(other.callGraph) {}

        /**
         * @brief Copy assignment operator
         */
        SystemProfile& operator=(const SystemProfile& other) noexcept {
            if (this != &other) {
                name = other.name;
                budget = other.budget;
                stats = other.stats;
                lastDuration = other.lastDuration.load(std::memory_order_relaxed);
                worstDuration = other.worstDuration.load(std::memory_order_relaxed);
                budgetExceeds = other.budgetExceeds.load(std::memory_order_relaxed);
                warningCount = other.warningCount.load(std::memory_order_relaxed);
                criticalCount = other.criticalCount.load(std::memory_order_relaxed);
                primaryThread = other.primaryThread;
                threadSwitches = other.threadSwitches.load(std::memory_order_relaxed);
                callGraph = other.callGraph;
            }

            return *this;
        }

        /**
         * @brief Move constructor - delegate to copy (atomics can't truly move)
         */
        SystemProfile(SystemProfile&& other) noexcept :
            SystemProfile(other) {}

        /**
         * @brief Move assignment - delegate to copy
         */
        SystemProfile& operator=(SystemProfile&& other) noexcept {
            return *this = other;
        }

        // =========================================================================
        // Methods
        // =========================================================================

        /**
         * @brief Check if within budget
         */
        [[nodiscard]] bool isWithinBudget() const noexcept {
            return lastDuration.load() <= budget.targetTime;
        }

        /**
         * @brief Get budget usage percentage
         */
        [[nodiscard]] float getBudgetUsage() const noexcept {
            const Duration last = lastDuration.load();
            if (budget.targetTime == Duration::zero())
                return 0.0f;
            return (static_cast<float>(last.count()) /
                static_cast<float>(budget.targetTime.count())) * 100.0f;
        }
    };

    // =============================================================================
    // System Profiler Configuration
    // =============================================================================

    /**
     * @brief Configuration for system profiler
     */
    struct SystemProfilerConfig {
        ProfilingConfig profilingConfig; ///< Base profiling config

        // System settings
        std::vector<SystemBudget> systemBudgets; ///< Per-system budgets
        bool autoRegisterSystems{true}; ///< Auto-register new systems
        std::size_t maxSystems{64}; ///< Maximum tracked systems

        // Analysis settings
        bool trackCallGraph{true}; ///< Track system dependencies
        bool detectBottlenecks{true}; ///< Identify bottlenecks
        bool suggestOptimizations{true}; ///< Provide optimization hints

        // Reporting
        bool perSystemReports{true}; ///< Individual system reports
        bool comparativeAnalysis{true}; ///< Compare system performance

        // Callbacks
        std::function<void(const SystemProfile&)> onBudgetExceeded;
        std::function<void(const std::string&)> onOptimizationSuggestion;
    };

    // =============================================================================
    // Optimization Suggestion
    // =============================================================================

    /**
     * @brief Performance optimization recommendation
     */
    struct OptimizationSuggestion {
        enum class Type {
            REDUCE_COMPLEXITY, ///< Algorithm complexity too high
            CACHE_OPTIMIZATION, ///< Poor cache usage detected
            PARALLELIZE, ///< Can benefit from parallelization
            BATCH_OPERATIONS, ///< Should batch similar operations
            REDUCE_ALLOCATIONS, ///< Too many allocations
            ASYNC_PROCESSING, ///< Move to async processing
            LEVEL_OF_DETAIL, ///< Implement LOD system
            FREQUENCY_REDUCTION ///< Reduce update frequency
        };

        std::string systemName; ///< Target system
        Type type; ///< Suggestion type
        std::string description; ///< Detailed description
        float expectedImprovement; ///< Expected performance gain (%)
        std::string implementation; ///< Implementation hints
    };

    // =============================================================================
    // SystemProfiler Class
    // =============================================================================

    /**
     * @brief Comprehensive per-system performance profiler
     * @details Tracks and analyzes individual system performance with
     *          budget management and optimization recommendations.
     */
    class SystemProfiler {
    public:
        /**
         * @brief Constructor with configuration
         * @param config Profiler configuration
         */
        explicit SystemProfiler(const SystemProfilerConfig& config = {});

        /**
         * @brief Destructor
         */
        ~SystemProfiler();

        // Delete copy operations
        SystemProfiler(const SystemProfiler&) = delete;
        SystemProfiler& operator=(const SystemProfiler&) = delete;

        // =============================================================================
        // System Registration
        // =============================================================================

        /**
         * @brief Register system for profiling
         * @param name System identifier
         * @param budget Performance budget
         * @return True if registration successful
         */
        bool registerSystem(const std::string& name, const SystemBudget& budget);

        /**
         * @brief Unregister system
         * @param name System identifier
         */
        void unregisterSystem(const std::string& name);

        /**
         * @brief Check if system is registered
         * @param name System identifier
         */
        [[nodiscard]] bool isSystemRegistered(const std::string& name) const;

        // =============================================================================
        // Profiling Operations
        // =============================================================================

        /**
         * @brief Begin profiling system
         * @param systemName System identifier
         * @return Scoped timer for automatic profiling
         */
        [[nodiscard]] ScopedTimer profileSystem(const std::string& systemName);

        /**
         * @brief Record system execution manually
         * @param systemName System identifier
         * @param duration Execution duration
         * @param caller Optional calling system
         */
        void recordSystemExecution(
                const std::string& systemName,
                Duration duration,
                const std::string& caller = ""
                );

        /**
         * @brief Mark system as started
         * @param systemName System identifier
         */
        void beginSystem(const std::string& systemName);

        /**
         * @brief Mark system as completed
         * @param systemName System identifier
         */
        void endSystem(const std::string& systemName);

        // =============================================================================
        // Analysis
        // =============================================================================

        /**
         * @brief Analyze all systems
         * @return List of systems exceeding budget
         */
        [[nodiscard]] std::vector<std::string> analyzeBudgets() const;

        /**
         * @brief Get system profile
         * @param systemName System identifier
         * @return System profile or nullopt if not found
         */
        [[nodiscard]] std::optional<SystemProfile> getSystemProfile(
                const std::string& systemName
                ) const;

        /**
         * @brief Get all system profiles
         */
        [[nodiscard]] std::vector<SystemProfile> getAllProfiles() const;

        /**
         * @brief Identify performance bottlenecks
         * @return List of bottleneck systems
         */
        [[nodiscard]] std::vector<std::string> identifyBottlenecks() const;

        /**
         * @brief Generate optimization suggestions
         * @return List of optimization recommendations
         */
        [[nodiscard]] std::vector<OptimizationSuggestion> generateOptimizations() const;

        /**
         * @brief Calculate total frame budget usage
         * @return Percentage of frame budget used
         */
        [[nodiscard]] float getTotalBudgetUsage() const;

        // =============================================================================
        // Reporting
        // =============================================================================

        /**
         * @brief Generate profiling report
         * @param detailed Include detailed breakdown
         * @return Report string
         */
        [[nodiscard]] std::string generateReport(bool detailed = true) const;

        /**
         * @brief Generate system-specific report
         * @param systemName System identifier
         * @return Report string
         */
        [[nodiscard]] std::string generateSystemReport(const std::string& systemName) const;

        /**
         * @brief Generate comparative analysis
         * @return Comparative report string
         */
        [[nodiscard]] std::string generateComparativeAnalysis() const;

        // =============================================================================
        // Configuration
        // =============================================================================

        /**
         * @brief Update configuration
         * @param config New configuration
         */
        void updateConfig(const SystemProfilerConfig& config);

        /**
         * @brief Get configuration
         */
        [[nodiscard]] const SystemProfilerConfig& getConfig() const noexcept {
            return config_;
        }

        /**
         * @brief Set performance tracker
         * @param tracker Performance tracker instance
         */
        void setPerformanceTracker(PerformanceTracker* tracker) {
            performanceTracker_ = tracker;
        }

        // =============================================================================
        // Utilities
        // =============================================================================

        /**
         * @brief Reset all profiling data
         */
        void reset();

        /**
         * @brief Reset specific system data
         * @param systemName System identifier
         */
        void resetSystem(const std::string& systemName);

        /**
         * @brief Enable/disable profiling
         */
        void setEnabled(const bool enabled) {
            enabled_ = enabled;
        }

        /**
         * @brief Check if profiling is enabled
         */
        [[nodiscard]] bool isEnabled() const noexcept {
            return enabled_;
        }

    private:
        // Configuration
        SystemProfilerConfig config_;
        std::atomic<bool> enabled_{true};

        // System profiles
        mutable std::mutex profilesMutex_;
        std::unordered_map<std::string, SystemProfile> profiles_;

        // Active profiling
        struct ActiveProfile {
            TimeStamp startTime;
            std::thread::id threadId;
            std::string caller;
        };

        std::unordered_map<std::string, ActiveProfile> activeProfiles_;

        // Performance tracker integration
        PerformanceTracker* performanceTracker_{nullptr};

        // =============================================================================
        // Internal Methods
        // =============================================================================

        /**
         * @brief Create system profile if needed
         */
        void ensureSystemProfile(const std::string& name);

        /**
         * @brief Update system statistics
         */
        static void updateSystemStats(SystemProfile& profile, Duration duration);

        /**
         * @brief Check budget and trigger callbacks
         */
        void checkBudget(SystemProfile& profile) const;

        /**
         * @brief Analyze system for optimization opportunities
         */
        [[nodiscard]] static std::vector<OptimizationSuggestion> analyzeSystem(
                const SystemProfile& profile
                );

        /**
         * @brief Format duration for display
         */
        [[nodiscard]] static std::string formatDuration(Duration duration);
    };

} // namespace engine::time
