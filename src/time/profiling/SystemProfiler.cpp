/**
 * @file SystemProfiler.cpp
 * @brief Implementation of per-system performance profiler
 * @details Detailed system profiling with budget management
 *
 * @author Development Team
 * @date Created on 2024-01-20
 */

#include "SystemProfiler.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <ranges>

namespace engine::time {

    // =============================================================================
    // SystemProfiler Implementation
    // =============================================================================

    SystemProfiler::SystemProfiler(const SystemProfilerConfig& config) :
        config_(config) {

        // Pre-register systems from config
        for (const auto& budget : config.systemBudgets) {
            registerSystem(std::string(budget.name), budget);
        }
    }

    SystemProfiler::~SystemProfiler() {
        // Generate final report if any systems exceeded budget
        if (!analyzeBudgets().empty()) {
            std::cout << "[SystemProfiler] Final Report:\n" << generateReport(false) << std::endl;
        }
    }

    bool SystemProfiler::registerSystem(const std::string& name, const SystemBudget& budget) {
        std::lock_guard lock(profilesMutex_);

        if (profiles_.size() >= config_.maxSystems) {
            std::cerr << "[SystemProfiler] Maximum system limit reached!" << std::endl;
            return false;
        }

        SystemProfile profile;
        profile.name = name;
        profile.budget = budget;
        profile.primaryThread = std::this_thread::get_id();

        profiles_[name] = profile;

        std::cout << "[SystemProfiler] Registered system: " << name
                << " (budget: " << toMilliseconds(budget.targetTime) << "ms)" << std::endl;

        return true;
    }

    void SystemProfiler::unregisterSystem(const std::string& name) {
        std::lock_guard lock(profilesMutex_);
        profiles_.erase(name);
    }

    bool SystemProfiler::isSystemRegistered(const std::string& name) const {
        std::lock_guard lock(profilesMutex_);
        return profiles_.contains(name);
    }

    ScopedTimer SystemProfiler::profileSystem(const std::string& systemName) {
        if (!enabled_) {
            return ScopedTimer(systemName, ReportDestination::NONE);
        }

        beginSystem(systemName);

        return ScopedTimer(
                systemName,
                ReportDestination::CALLBACK,
                [this, systemName](const TimerReport& report) {
                    endSystem(systemName);
                    recordSystemExecution(systemName, report.elapsed);
                }
                );
    }

    void SystemProfiler::recordSystemExecution(
            const std::string& systemName,
            const Duration duration,
            const std::string& caller
            ) {

        if (!enabled_)
            return;

        std::lock_guard lock(profilesMutex_);

        ensureSystemProfile(systemName);

        auto& profile = profiles_[systemName];

        // Update statistics
        updateSystemStats(profile, duration);

        // Update call graph if caller specified
        if (!caller.empty() && config_.trackCallGraph) {
            const auto it = std::ranges::find_if(
                    profile.callGraph,
                    [&caller](const SystemProfile::CallData& data) {
                        return data.caller == caller;
                    }
                    );

            if (it != profile.callGraph.end()) {
                it->callCount++;
                it->totalTime += duration;
            } else {
                SystemProfile::CallData callData;
                callData.caller = caller;
                callData.callCount = 1;
                callData.totalTime = duration;
                profile.callGraph.push_back(callData);
            }
        }

        // Check thread switches
        if (const auto currentThread = std::this_thread::get_id(); currentThread != profile.primaryThread) {
            profile.threadSwitches.fetch_add(1);
        }

        // Check budget
        checkBudget(profile);

        // Forward to performance tracker if available
        if (performanceTracker_) {
            performanceTracker_->recordSystemTiming(systemName, duration);
        }
    }

    void SystemProfiler::beginSystem(const std::string& systemName) {
        if (!enabled_)
            return;

        std::lock_guard lock(profilesMutex_);

        ActiveProfile active;
        active.startTime = Clock::now();
        active.threadId = std::this_thread::get_id();

        activeProfiles_[systemName] = active;
    }

    void SystemProfiler::endSystem(const std::string& systemName) {
        if (!enabled_)
            return;

        std::lock_guard lock(profilesMutex_);

        const auto it = activeProfiles_.find(systemName);

        if (it == activeProfiles_.end()) {
            return;
        }

        const auto endTime = Clock::now();
        const Duration duration = std::chrono::duration_cast<Duration>(endTime - it->second.startTime);

        // Remove from active
        activeProfiles_.erase(it);

        // Record execution
        ensureSystemProfile(systemName);
        auto& profile = profiles_[systemName];
        updateSystemStats(profile, duration);
        checkBudget(profile);
    }

    std::vector<std::string> SystemProfiler::analyzeBudgets() const {
        std::lock_guard lock(profilesMutex_);

        std::vector<std::string> exceeding;

        for (const auto& [name, profile] : profiles_) {
            if (!profile.isWithinBudget()) {
                exceeding.push_back(name);
            }
        }

        // Sort by budget usage
        std::ranges::sort(
                exceeding,
                [this](const std::string& a, const std::string& b) {
                    const auto& profA = profiles_.at(a);
                    const auto& profB = profiles_.at(b);
                    return profA.getBudgetUsage() > profB.getBudgetUsage();
                }
                );

        return exceeding;
    }

    std::optional<SystemProfile> SystemProfiler::getSystemProfile(const std::string& systemName) const {
        std::lock_guard lock(profilesMutex_);

        if (const auto it = profiles_.find(systemName); it != profiles_.end()) {
            return it->second;
        }

        return std::nullopt;
    }

    std::vector<SystemProfile> SystemProfiler::getAllProfiles() const {
        std::lock_guard lock(profilesMutex_);

        std::vector<SystemProfile> result;
        result.reserve(profiles_.size());

        for (const auto& [name, profile] : profiles_) {
            result.push_back(profile);
        }

        return result;
    }

    std::vector<std::string> SystemProfiler::identifyBottlenecks() const {
        if (!config_.detectBottlenecks) {
            return {};
        }

        std::lock_guard lock(profilesMutex_);

        std::vector<std::string> bottlenecks;

        // Calculate total frame time
        Duration totalFrameTime{};
        for (const auto& profile : profiles_ | std::views::values) {
            totalFrameTime += profile.stats.getAverage();
        }

        // Identify systems taking >20% of frame time
        for (const auto& [name, profile] : profiles_) {
            const Duration avgTime = profile.stats.getAverage();
            if (totalFrameTime > Duration::zero()) {
                const float percentage = (static_cast<float>(avgTime.count()) /
                    static_cast<float>(totalFrameTime.count())) * 100.0f;
                if (percentage > 20.0f) {
                    bottlenecks.push_back(name);
                }
            }
        }

        return bottlenecks;
    }

    std::vector<OptimizationSuggestion> SystemProfiler::generateOptimizations() const {
        if (!config_.suggestOptimizations) {
            return {};
        }

        std::lock_guard lock(profilesMutex_);

        std::vector<OptimizationSuggestion> suggestions;

        for (const auto& profile : profiles_ | std::views::values) {
            const auto systemSuggestions = analyzeSystem(profile);
            suggestions.insert(
                    suggestions.end(),
                    systemSuggestions.begin(),
                    systemSuggestions.end()
                    );
        }

        return suggestions;
    }

    float SystemProfiler::getTotalBudgetUsage() const {
        std::lock_guard lock(profilesMutex_);

        Duration totalUsed{};
        Duration totalBudget{};

        for (const auto& profile : profiles_ | std::views::values) {
            totalUsed += profile.lastDuration.load();
            totalBudget += profile.budget.targetTime;
        }

        if (totalBudget == Duration::zero()) {
            return 0.0f;
        }

        return (static_cast<float>(totalUsed.count()) /
            static_cast<float>(totalBudget.count())) * 100.0f;
    }

    std::string SystemProfiler::generateReport(const bool detailed) const {
        std::lock_guard lock(profilesMutex_);

        std::stringstream ss;

        ss << "=== System Performance Report ===" << std::endl;
        ss << "Total Budget Usage: " << std::fixed << std::setprecision(1)
                << getTotalBudgetUsage() << "%" << std::endl;
        ss << std::endl;

        // Sort systems by time
        std::vector<std::pair<std::string, Duration>> systems;
        for (const auto& [name, profile] : profiles_) {
            systems.emplace_back(name, profile.stats.getAverage());
        }

        std::ranges::sort(
                systems,
                [](const auto& a, const auto& b) {
                    return a.second > b.second;
                }
                );

        // System breakdown
        ss << "System Breakdown:" << std::endl;
        ss << std::setw(25) << "System"
                << std::setw(12) << "Average"
                << std::setw(12) << "Budget"
                << std::setw(10) << "Usage %"
                << std::setw(10) << "Status" << std::endl;
        ss << std::string(70, '-') << std::endl;

        for (const auto& [name, avgTime] : systems) {
            const auto& profile = profiles_.at(name);
            const float usage = profile.getBudgetUsage();

            ss << std::setw(25) << name
                    << std::setw(12) << formatDuration(avgTime)
                    << std::setw(12) << formatDuration(profile.budget.targetTime)
                    << std::setw(9) << std::fixed << std::setprecision(1) << usage << "%";

            if (usage > 100.0f) {
                ss << std::setw(10) << "CRITICAL";
            } else if (usage > 80.0f) {
                ss << std::setw(10) << "WARNING";
            } else {
                ss << std::setw(10) << "OK";
            }

            ss << std::endl;
        }

        if (detailed) {
            ss << std::endl;

            // Bottlenecks
            if (const auto bottlenecks = identifyBottlenecks(); !bottlenecks.empty()) {
                ss << "Bottlenecks:" << std::endl;
                for (const auto& bottleneck : bottlenecks) {
                    ss << "  - " << bottleneck << std::endl;
                }
                ss << std::endl;
            }

            // Optimization suggestions
            if (const auto suggestions = generateOptimizations(); !suggestions.empty()) {
                ss << "Optimization Suggestions:" << std::endl;
                for (const auto& suggestion : suggestions) {
                    ss << "  " << suggestion.systemName << ": "
                            << suggestion.description << std::endl;
                    if (suggestion.expectedImprovement > 0) {
                        ss << "    Expected improvement: "
                                << std::fixed << std::setprecision(0)
                                << suggestion.expectedImprovement << "%" << std::endl;
                    }
                }
            }
        }

        return ss.str();
    }

    std::string SystemProfiler::generateSystemReport(const std::string& systemName) const {
        const auto profileOpt = getSystemProfile(systemName);

        if (!profileOpt) {
            return "System not found: " + systemName;
        }

        const auto& profile = *profileOpt;
        std::stringstream ss;

        ss << "=== System Report: " << systemName << " ===" << std::endl;
        ss << "Budget Usage: " << std::fixed << std::setprecision(1)
                << profile.getBudgetUsage() << "%" << std::endl;
        ss << "Average Time: " << formatDuration(profile.stats.getAverage()) << std::endl;
        ss << "Min Time: " << formatDuration(profile.stats.min.load()) << std::endl;
        ss << "Max Time: " << formatDuration(profile.stats.max.load()) << std::endl;
        ss << "Sample Count: " << profile.stats.sampleCount.load() << std::endl;
        ss << "Budget Exceeds: " << profile.budgetExceeds.load() << std::endl;
        ss << "Thread Switches: " << profile.threadSwitches.load() << std::endl;

        if (!profile.callGraph.empty()) {
            ss << std::endl << "Call Graph:" << std::endl;
            for (const auto& call : profile.callGraph) {
                ss << "  From " << call.caller << ": "
                        << call.callCount << " calls, "
                        << formatDuration(call.totalTime) << " total" << std::endl;
            }
        }

        return ss.str();
    }

    std::string SystemProfiler::generateComparativeAnalysis() const {
        if (!config_.comparativeAnalysis) {
            return "Comparative analysis disabled";
        }

        // Comparative analysis implementation
        return "Comparative analysis not yet implemented";
    }

    void SystemProfiler::updateConfig(const SystemProfilerConfig& config) {
        std::lock_guard lock(profilesMutex_);
        config_ = config;
    }

    void SystemProfiler::reset() {
        std::lock_guard lock(profilesMutex_);

        for (auto& profile : profiles_ | std::views::values) {
            profile.stats.reset();
            profile.lastDuration = Duration::zero();
            profile.worstDuration = Duration::zero();
            profile.budgetExceeds = 0;
            profile.warningCount = 0;
            profile.criticalCount = 0;
            profile.threadSwitches = 0;
            profile.callGraph.clear();
        }
    }

    void SystemProfiler::resetSystem(const std::string& systemName) {
        std::lock_guard lock(profilesMutex_);

        const auto it = profiles_.find(systemName);

        if (it == profiles_.end())
            return;

        auto& profile = it->second;
        profile.stats.reset();
        profile.lastDuration = Duration::zero();
        profile.worstDuration = Duration::zero();
        profile.budgetExceeds = 0;
        profile.warningCount = 0;
        profile.criticalCount = 0;
        profile.threadSwitches = 0;
        profile.callGraph.clear();
    }

    void SystemProfiler::ensureSystemProfile(const std::string& name) {
        if (!config_.autoRegisterSystems) {
            return;
        }

        if (!profiles_.contains(name)) {
            SystemProfile profile;
            profile.name = name;
            profile.budget.targetTime = constants::TARGET_FRAME_TIME_60FPS / 10; // Default 10% of frame
            profile.budget.warningThreshold = std::chrono::duration_cast<Duration>(profile.budget.targetTime * 1.2);
            profile.budget.criticalThreshold = std::chrono::duration_cast<Duration>(profile.budget.targetTime * 1.5);
            profile.primaryThread = std::this_thread::get_id();

            profiles_[name] = profile;
        }
    }

    void SystemProfiler::updateSystemStats(SystemProfile& profile, const Duration duration) {
        profile.stats.update(duration);
        profile.lastDuration.store(duration);

        // Update worst case
        Duration worst = profile.worstDuration.load();
        while (duration > worst &&
            !profile.worstDuration.compare_exchange_weak(worst, duration)) {}
    }

    void SystemProfiler::checkBudget(SystemProfile& profile) const {

        if (const Duration duration = profile.lastDuration.load(); duration > profile.budget.criticalThreshold) {
            profile.criticalCount.fetch_add(1);

            if (config_.onBudgetExceeded) {
                config_.onBudgetExceeded(profile);
            }
        } else if (duration > profile.budget.warningThreshold) {
            profile.warningCount.fetch_add(1);
        } else if (duration > profile.budget.targetTime) {
            profile.budgetExceeds.fetch_add(1);
        }
    }

    std::vector<OptimizationSuggestion> SystemProfiler::analyzeSystem(const SystemProfile& profile) {
        std::vector<OptimizationSuggestion> suggestions;

        // High CPU usage
        if (const float usage = profile.getBudgetUsage(); usage > 150.0f) {
            OptimizationSuggestion suggestion;
            suggestion.systemName = profile.name;
            suggestion.type = OptimizationSuggestion::Type::REDUCE_COMPLEXITY;
            suggestion.description = "System exceeds budget by " +
                    std::to_string(static_cast<int>(usage - 100)) + "%";
            suggestion.expectedImprovement = (usage - 100) / 2;
            suggestion.implementation = "Consider algorithmic optimizations or LOD";
            suggestions.push_back(suggestion);
        }

        // Thread switching
        if (profile.threadSwitches > 100) {
            OptimizationSuggestion suggestion;
            suggestion.systemName = profile.name;
            suggestion.type = OptimizationSuggestion::Type::PARALLELIZE;
            suggestion.description = "Frequent thread switching detected";
            suggestion.expectedImprovement = 10.0f;
            suggestion.implementation = "Pin to specific thread or use thread pool";
            suggestions.push_back(suggestion);
        }

        return suggestions;
    }

    std::string SystemProfiler::formatDuration(const Duration duration) {
        const double ms = toMilliseconds(duration);
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << ms << "ms";
        return ss.str();
    }

} // namespace engine::time
