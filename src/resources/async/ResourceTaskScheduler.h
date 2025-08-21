//
// Created by Andres Guerrero on 20-08-25.
//

#pragma once

#include "ResourceTask.h"
#include <queue>
#include <condition_variable>
#include <chrono>
#include <shared_mutex>

namespace engine::resources {
    /**
     * @brief Priority comparator for task queue
     */
    struct TaskPriorityComparator {
        bool operator()(const std::pair<ResourcePriority, ResourceTask::TaskID>& a,
                        const std::pair<ResourcePriority, ResourceTask::TaskID>& b) const {
            // Higher priority value means higher priority
            return static_cast<int>(a.first) < static_cast<int>(b.first);
        }
    };

    /**
     * @brief Task scheduler for managing and executing resource tasks
     *
     * This class manages a pool of worker threads that execute resource tasks
     * with support for dependencies, priorities, and progress tracking.
     */
    class ResourceTaskScheduler {
    public:
        /**
         * @brief Configuration for task scheduler
         */
        struct Config {
            std::size_t workerCount; // Number of worker threads
            std::size_t maxPendingTasks; // Maximum pending tasks
            std::size_t maxCompletedHistory; // Number of completed tasks to keep
            bool enableTaskProfiling; // Enable task timing
            std::chrono::milliseconds taskTimeout; // default 60 second timeout

            // Default constructor
            Config()
                : workerCount(4),
                  maxPendingTasks(1000),
                  maxCompletedHistory(100),
                  enableTaskProfiling(true),
                  taskTimeout(60000) {
            }
        };

        /**
         * @brief Task execution statistics
         */
        struct TaskStats {
            std::atomic<std::size_t> totalSubmitted{0};
            std::atomic<std::size_t> totalCompleted{0};
            std::atomic<std::size_t> totalFailed{0};
            std::atomic<std::size_t> totalCancelled{0};
            std::atomic<std::size_t> currentPending{0};
            std::atomic<std::size_t> currentRunning{0};
            std::atomic<std::uint64_t> totalExecutionTime{0}; // Microseconds
        };

        /**
         * @brief Constructor with configuration
         * @param config Scheduler configuration
         */
        explicit ResourceTaskScheduler(const Config& config);

        /**
         * @brief Destructor
         */
        ~ResourceTaskScheduler();

        // Delete copy, allow move
        ResourceTaskScheduler(const ResourceTaskScheduler&) = delete;
        ResourceTaskScheduler& operator=(const ResourceTaskScheduler&) = delete;
        ResourceTaskScheduler(ResourceTaskScheduler&&) = default;
        ResourceTaskScheduler& operator=(ResourceTaskScheduler&&) = default;

        /**
         * @brief Initialize the scheduler
         * @return true if successful
         */
        bool initialize();

        /**
         * @brief Shutdown the scheduler
         */
        void shutdown();

        /**
         * @brief Check if scheduler is running
         * @return true if running
         */
        bool isRunning() const { return !stopWorkers_.load(std::memory_order_acquire); }

        // ====================================================================
        // TASK SUBMISSION
        // ====================================================================

        /**
         * @brief Submit a task for execution
         * @param task Task to execute
         * @return Future that will contain the result
         */
        std::future<bool> submitTask(std::unique_ptr<ResourceTask> task);

        /**
         * @brief Submit multiple tasks
         * @param tasks Tasks to execute
         * @return Vector of futures for each task
         */
        std::vector<std::future<bool>> submitTasks(std::vector<std::unique_ptr<ResourceTask>> tasks);

        /**
         * @brief Submit a task with callback
         * @param task Task to execute
         * @param callback Callback to invoke on completion
         * @return Task ID
         */
        ResourceTask::TaskID submitTaskWithCallback(std::unique_ptr<ResourceTask> task,
                                                    std::function<void(bool)> callback);

        // ====================================================================
        // TASK MANAGEMENT
        // ====================================================================

        /**
         * @brief Cancel a task
         * @param taskId Task ID to cancel
         * @return true if task was cancelled
         */
        bool cancelTask(ResourceTask::TaskID taskId);

        /**
         * @brief Cancel all pending tasks
         * @return Number of tasks cancelled
         */
        std::size_t cancelAllTasks();

        /**
         * @brief Get task status
         * @param taskId Task ID
         * @return Task status
         */
        ResourceTaskStatus getTaskStatus(ResourceTask::TaskID taskId) const;

        /**
         * @brief Get task progress
         * @param taskId Task ID
         * @return Progress (0.0 to 1.0), or -1 if task not found
         */
        float getTaskProgress(ResourceTask::TaskID taskId) const;

        /**
         * @brief Wait for task completion
         * @param taskId Task ID
         * @param timeout Maximum wait time
         * @return true if task completed successfully
         */
        bool waitForTask(ResourceTask::TaskID taskId,
                         std::chrono::milliseconds timeout = std::chrono::milliseconds(0));

        /**
         * @brief Wait for multiple tasks
         * @param taskIds Task IDs to wait for
         * @param timeout Maximum wait time
         * @return true if all tasks completed successfully
         */
        bool waitForTasks(const std::vector<ResourceTask::TaskID>& taskIds,
                          std::chrono::milliseconds timeout = std::chrono::milliseconds(0));

        // ====================================================================
        // STATISTICS & MONITORING
        // ====================================================================

        /**
         * @brief Get scheduler statistics
         * @return Current statistics
         */
        const TaskStats& getStats() const { return stats_; }

        /**
         * @brief Get number of pending tasks
         * @return Pending task count
         */
        std::size_t getPendingTaskCount() const;

        /**
         * @brief Get number of running tasks
         * @return Running task count
         */
        std::size_t getRunningTaskCount() const;

        /**
         * @brief Get worker thread count
         * @return Number of worker threads
         */
        std::size_t getWorkerCount() const { return config_.workerCount; }

        /**
         * @brief Generate scheduler report
         * @return String report
         */
        std::string generateReport() const;

        /**
         * @brief Clear completed task history
         */
        void clearHistory();

    private:
        // Configuration
        Config config_;

        // Worker threads
        std::vector<std::thread> workers_;
        std::atomic<bool> stopWorkers_{false};

        // Task management
        std::unordered_map<ResourceTask::TaskID, std::unique_ptr<ResourceTask>> pendingTasks_;
        std::unordered_map<ResourceTask::TaskID, std::unique_ptr<ResourceTask>> runningTasks_;
        std::unordered_map<ResourceTask::TaskID, ResourceTaskStatus> completedTasks_;

        // Priority queue for ready tasks
        std::priority_queue<std::pair<ResourcePriority, ResourceTask::TaskID>,
                            std::vector<std::pair<ResourcePriority, ResourceTask::TaskID>>,
                            TaskPriorityComparator> readyQueue_;

        // Task promises for futures
        std::unordered_map<ResourceTask::TaskID, std::shared_ptr<std::promise<bool>>> taskPromises_;

        // Synchronization
        mutable std::shared_mutex taskMutex_;
        std::condition_variable_any taskCV_;

        // Statistics
        mutable TaskStats stats_;

        // Task ID generation
        std::atomic<ResourceTask::TaskID> nextTaskId_{1};

        // Worker thread function
        void workerThread(std::size_t workerId);

        // Execute a task
        void executeTask(std::unique_ptr<ResourceTask> task);

        // Check if task dependencies are met
        bool areDependenciesMet(ResourceTask::TaskID taskId) const;

        // Check tasks dependent on completed task
        void checkDependentTasks(ResourceTask::TaskID completedTaskId);

        // Clean up old completed tasks
        void cleanupCompletedTasks();

        // Generate unique task ID
        ResourceTask::TaskID generateTaskId() { return nextTaskId_.fetch_add(1); }
    };
} // namespace engine::resources
