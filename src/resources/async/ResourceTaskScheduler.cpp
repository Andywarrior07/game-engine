//
// Created by Andres Guerrero on 20-08-25.
//

#include "ResourceTaskScheduler.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>

namespace engine::resources {

    // ====================================================================
    // CONSTRUCTOR / DESTRUCTOR
    // ====================================================================

    ResourceTaskScheduler::ResourceTaskScheduler(const Config& config)
        : config_(config) {
    }

    ResourceTaskScheduler::~ResourceTaskScheduler() {
        shutdown();
    }

    // ====================================================================
    // INITIALIZATION / SHUTDOWN
    // ====================================================================

    bool ResourceTaskScheduler::initialize() {
        if (!stopWorkers_) {
            std::cerr << "TaskScheduler already initialized" << std::endl;
            return false;
        }

        stopWorkers_ = false;

        // Start worker threads
        workers_.reserve(config_.workerCount);
        for (std::size_t i = 0; i < config_.workerCount; ++i) {
            workers_.emplace_back(&ResourceTaskScheduler::workerThread, this, i);
        }

        std::cout << "TaskScheduler initialized with " << config_.workerCount << " worker threads" << std::endl;

        return true;
    }

    void ResourceTaskScheduler::shutdown() {
        if (stopWorkers_) {
            return; // Already shut down
        }

        // Signal workers to stop
        stopWorkers_ = true;
        taskCV_.notify_all();

        // Wait for all workers to finish
        for (auto& worker : workers_) {
            if (worker.joinable()) {
                worker.join();
            }
        }

        workers_.clear();

        // Cancel all pending tasks
        cancelAllTasks();

        // Clear all data
        {
            std::unique_lock lock(taskMutex_);
            pendingTasks_.clear();
            runningTasks_.clear();
            completedTasks_.clear();
            while (!readyQueue_.empty()) {
                readyQueue_.pop();
            }
            taskPromises_.clear();
        }

        std::cout << "TaskScheduler shut down" << std::endl;
    }

    // ====================================================================
    // TASK SUBMISSION
    // ====================================================================

    std::future<bool> ResourceTaskScheduler::submitTask(std::unique_ptr<ResourceTask> task) {
        if (!task) {
            std::promise<bool> emptyPromise;
            emptyPromise.set_value(false);
            return emptyPromise.get_future();
        }

        // Generate ID if not set
        if (task->getId() == 0) {
            task = std::make_unique<LoadResourceTask>(
                generateTaskId(),
                dynamic_cast<LoadResourceTask*>(task.get())->getPath(),
                dynamic_cast<LoadResourceTask*>(task.get())->getResourceType(),
                task->getPriority()
            );
        }

        ResourceTask::TaskID taskId = task->getId();
        auto promise = std::make_shared<std::promise<bool>>();
        auto future = promise->get_future();

        // Set completion callback to fulfill promise
        task->setCompletionCallback([this, promise, taskId](ResourceTask* completedTask) {
            bool success = completedTask->getStatus() == ResourceTaskStatus::COMPLETED;
            promise->set_value(success);

            // Update statistics
            if (success) {
                stats_.totalCompleted.fetch_add(1, std::memory_order_relaxed);
            } else if (completedTask->getStatus() == ResourceTaskStatus::FAILED) {
                stats_.totalFailed.fetch_add(1, std::memory_order_relaxed);
            } else if (completedTask->getStatus() == ResourceTaskStatus::CANCELLED) {
                stats_.totalCancelled.fetch_add(1, std::memory_order_relaxed);
            }
        });

        {
            std::unique_lock lock(taskMutex_);

            // Check task limit
            if (pendingTasks_.size() >= config_.maxPendingTasks) {
                std::cerr << "Task queue full, rejecting task" << std::endl;
                promise->set_value(false);
                return future;
            }

            // Store task and promise
            pendingTasks_[taskId] = std::move(task);
            taskPromises_[taskId] = promise;

            // Check if dependencies are met
            if (areDependenciesMet(taskId)) {
                readyQueue_.emplace(pendingTasks_[taskId]->getPriority(), taskId);
                taskCV_.notify_one();
            }

            // Update statistics
            stats_.totalSubmitted.fetch_add(1, std::memory_order_relaxed);
            stats_.currentPending.fetch_add(1, std::memory_order_relaxed);
        }

        return future;
    }

    std::vector<std::future<bool>> ResourceTaskScheduler::submitTasks(
        std::vector<std::unique_ptr<ResourceTask>> tasks) {

        std::vector<std::future<bool>> futures;
        futures.reserve(tasks.size());

        for (auto& task : tasks) {
            futures.push_back(submitTask(std::move(task)));
        }

        return futures;
    }

    ResourceTask::TaskID ResourceTaskScheduler::submitTaskWithCallback(
        std::unique_ptr<ResourceTask> task,
        std::function<void(bool)> callback) {

        if (!task) {
            return 0;
        }

        ResourceTask::TaskID taskId = task->getId();
        if (taskId == 0) {
            taskId = generateTaskId();
        }

        // Wrap the callback
        auto existingCallback = task->getCompletionCallback();
        task->setCompletionCallback([callback, existingCallback](ResourceTask* completedTask) {
            bool success = completedTask->getStatus() == ResourceTaskStatus::COMPLETED;
            if (callback) {
                callback(success);
            }
            if (existingCallback) {
                existingCallback(completedTask);
            }
        });

        submitTask(std::move(task));

        return taskId;
    }

    // ====================================================================
    // TASK MANAGEMENT
    // ====================================================================

    bool ResourceTaskScheduler::cancelTask(ResourceTask::TaskID taskId) {
        std::unique_lock lock(taskMutex_);

        // Check if task is pending
        auto pendingIt = pendingTasks_.find(taskId);
        if (pendingIt != pendingTasks_.end()) {
            pendingIt->second->cancel();

            // Fulfill promise with false
            auto promiseIt = taskPromises_.find(taskId);
            if (promiseIt != taskPromises_.end()) {
                promiseIt->second->set_value(false);
                taskPromises_.erase(promiseIt);
            }

            pendingTasks_.erase(pendingIt);
            stats_.currentPending.fetch_sub(1, std::memory_order_relaxed);
            stats_.totalCancelled.fetch_add(1, std::memory_order_relaxed);

            return true;
        }

        // Check if task is running (can't cancel running tasks)
        auto runningIt = runningTasks_.find(taskId);
        if (runningIt != runningTasks_.end()) {
            // Mark for cancellation but can't stop execution
            runningIt->second->cancel();
            return false;
        }

        return false;
    }

    std::size_t ResourceTaskScheduler::cancelAllTasks() {
        std::unique_lock lock(taskMutex_);

        std::size_t cancelledCount = 0;

        // Cancel all pending tasks
        for (auto& [taskId, task] : pendingTasks_) {
            task->cancel();

            // Fulfill promise with false
            auto promiseIt = taskPromises_.find(taskId);
            if (promiseIt != taskPromises_.end()) {
                promiseIt->second->set_value(false);
            }

            ++cancelledCount;
        }

        // Clear pending tasks
        pendingTasks_.clear();
        taskPromises_.clear();

        // Clear ready queue
        while (!readyQueue_.empty()) {
            readyQueue_.pop();
        }

        stats_.currentPending = 0;
        stats_.totalCancelled.fetch_add(cancelledCount, std::memory_order_relaxed);

        return cancelledCount;
    }

    ResourceTaskStatus ResourceTaskScheduler::getTaskStatus(ResourceTask::TaskID taskId) const {
        std::shared_lock lock(taskMutex_);

        // Check pending
        auto pendingIt = pendingTasks_.find(taskId);
        if (pendingIt != pendingTasks_.end()) {
            return pendingIt->second->getStatus();
        }

        // Check running
        auto runningIt = runningTasks_.find(taskId);
        if (runningIt != runningTasks_.end()) {
            return runningIt->second->getStatus();
        }

        // Check completed
        auto completedIt = completedTasks_.find(taskId);
        if (completedIt != completedTasks_.end()) {
            return completedIt->second;
        }

        return ResourceTaskStatus::FAILED;
    }

    float ResourceTaskScheduler::getTaskProgress(ResourceTask::TaskID taskId) const {
        std::shared_lock lock(taskMutex_);

        // Check running tasks (most likely to need progress)
        auto runningIt = runningTasks_.find(taskId);
        if (runningIt != runningTasks_.end()) {
            return runningIt->second->getProgress();
        }

        // Check pending
        auto pendingIt = pendingTasks_.find(taskId);
        if (pendingIt != pendingTasks_.end()) {
            return 0.0f; // Not started yet
        }

        // Check completed
        auto completedIt = completedTasks_.find(taskId);
        if (completedIt != completedTasks_.end()) {
            return completedIt->second == ResourceTaskStatus::COMPLETED ? 1.0f : -1.0f;
        }

        return -1.0f; // Task not found
    }

    bool ResourceTaskScheduler::waitForTask(ResourceTask::TaskID taskId,
                                           std::chrono::milliseconds timeout) {
        auto startTime = std::chrono::steady_clock::now();

        while (true) {
            auto status = getTaskStatus(taskId);

            if (status == ResourceTaskStatus::COMPLETED) {
                return true;
            }

            if (status == ResourceTaskStatus::FAILED ||
                status == ResourceTaskStatus::CANCELLED) {
                return false;
            }

            // Check timeout
            if (timeout.count() > 0) {
                auto elapsed = std::chrono::steady_clock::now() - startTime;
                if (elapsed >= timeout) {
                    return false;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    bool ResourceTaskScheduler::waitForTasks(const std::vector<ResourceTask::TaskID>& taskIds,
                                            std::chrono::milliseconds timeout) {
        auto startTime = std::chrono::steady_clock::now();

        for (ResourceTask::TaskID taskId : taskIds) {
            // Calculate remaining timeout
            std::chrono::milliseconds remainingTimeout = timeout;
            if (timeout.count() > 0) {
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - startTime);
                remainingTimeout = timeout - elapsed;

                if (remainingTimeout.count() <= 0) {
                    return false; // Timeout exceeded
                }
            }

            if (!waitForTask(taskId, remainingTimeout)) {
                return false;
            }
        }

        return true;
    }

    // ====================================================================
    // STATISTICS & MONITORING
    // ====================================================================

    std::size_t ResourceTaskScheduler::getPendingTaskCount() const {
        std::shared_lock lock(taskMutex_);
        return pendingTasks_.size();
    }

    std::size_t ResourceTaskScheduler::getRunningTaskCount() const {
        std::shared_lock lock(taskMutex_);
        return runningTasks_.size();
    }

    std::string ResourceTaskScheduler::generateReport() const {
        std::stringstream report;

        report << "=== Task Scheduler Report ===" << std::endl;
        report << std::endl;

        report << "Configuration:" << std::endl;
        report << "  Worker Threads: " << config_.workerCount << std::endl;
        report << "  Max Pending Tasks: " << config_.maxPendingTasks << std::endl;
        report << "  Task Timeout: " << config_.taskTimeout.count() << " ms" << std::endl;
        report << std::endl;

        report << "Statistics:" << std::endl;
        report << "  Total Submitted: " << stats_.totalSubmitted.load() << std::endl;
        report << "  Total Completed: " << stats_.totalCompleted.load() << std::endl;
        report << "  Total Failed: " << stats_.totalFailed.load() << std::endl;
        report << "  Total Cancelled: " << stats_.totalCancelled.load() << std::endl;
        report << "  Currently Pending: " << stats_.currentPending.load() << std::endl;
        report << "  Currently Running: " << stats_.currentRunning.load() << std::endl;

        if (stats_.totalCompleted > 0) {
            auto avgTime = stats_.totalExecutionTime.load() / stats_.totalCompleted.load();
            report << "  Avg Execution Time: " << (avgTime / 1000.0) << " ms" << std::endl;
        }

        report << std::endl;
        report << "Queue Status:" << std::endl;

        {
            std::shared_lock lock(taskMutex_);
            report << "  Ready Queue Size: " << readyQueue_.size() << std::endl;
            report << "  Pending Tasks: " << pendingTasks_.size() << std::endl;
            report << "  Running Tasks: " << runningTasks_.size() << std::endl;
            report << "  Completed History: " << completedTasks_.size() << std::endl;
        }

        return report.str();
    }

    void ResourceTaskScheduler::clearHistory() {
        std::unique_lock lock(taskMutex_);
        completedTasks_.clear();
    }

    // ====================================================================
    // PRIVATE METHODS
    // ====================================================================

    void ResourceTaskScheduler::workerThread(std::size_t workerId) {
        std::cout << "Worker thread " << workerId << " started" << std::endl;

        while (!stopWorkers_) {
            std::unique_ptr<ResourceTask> task;

            {
                std::unique_lock lock(taskMutex_);

                // Wait for tasks or shutdown signal
                taskCV_.wait(lock, [this] {
                    return !readyQueue_.empty() || stopWorkers_;
                });

                if (stopWorkers_) {
                    break;
                }

                if (!readyQueue_.empty()) {
                    // Get highest priority task
                    auto [priority, taskId] = readyQueue_.top();
                    readyQueue_.pop();

                    // Move task from pending to running
                    auto it = pendingTasks_.find(taskId);
                    if (it != pendingTasks_.end()) {
                        task = std::move(it->second);
                        pendingTasks_.erase(it);
                        runningTasks_[taskId] = std::move(task);
                        task = std::move(runningTasks_[taskId]);

                        stats_.currentPending.fetch_sub(1, std::memory_order_relaxed);
                        stats_.currentRunning.fetch_add(1, std::memory_order_relaxed);
                    }
                }
            }

            if (task) {
                executeTask(std::move(task));
            }
        }

        std::cout << "Worker thread " << workerId << " stopped" << std::endl;
    }

    void ResourceTaskScheduler::executeTask(std::unique_ptr<ResourceTask> task) {
        auto taskId = task->getId();
        auto startTime = std::chrono::steady_clock::now();

        // Set status to running
        task->setStatus(ResourceTaskStatus::RUNNING);

        bool success = false;
        try {
            // Execute the task
            success = task->execute();
        } catch (const std::exception& e) {
            std::cerr << "Task execution exception: " << e.what() << std::endl;
            task->setError(std::string("Exception: ") + e.what());
            success = false;
        } catch (...) {
            std::cerr << "Unknown exception during task execution" << std::endl;
            task->setError("Unknown exception");
            success = false;
        }

        // Calculate execution time
        auto endTime = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);

        // Update statistics
        stats_.totalExecutionTime.fetch_add(duration.count(), std::memory_order_relaxed);
        stats_.currentRunning.fetch_sub(1, std::memory_order_relaxed);

        // Set final status
        if (!success && task->getStatus() != ResourceTaskStatus::CANCELLED) {
            task->setStatus(ResourceTaskStatus::FAILED);
        } else if (success) {
            task->setStatus(ResourceTaskStatus::COMPLETED);
        }

        {
            std::unique_lock lock(taskMutex_);

            // Move to completed
            completedTasks_[taskId] = task->getStatus();
            runningTasks_.erase(taskId);

            // Clean up old completed tasks if needed
            if (completedTasks_.size() > config_.maxCompletedHistory) {
                cleanupCompletedTasks();
            }

            // Check for tasks that were waiting on this one
            checkDependentTasks(taskId);
        }
    }

    bool ResourceTaskScheduler::areDependenciesMet(ResourceTask::TaskID taskId) const {
        auto it = pendingTasks_.find(taskId);
        if (it == pendingTasks_.end()) {
            return false;
        }

        const auto& dependencies = it->second->getDependencies();

        for (ResourceTask::TaskID depId : dependencies) {
            // Check if dependency is completed successfully
            auto completedIt = completedTasks_.find(depId);
            if (completedIt == completedTasks_.end() ||
                completedIt->second != ResourceTaskStatus::COMPLETED) {
                return false;
            }
        }

        return true;
    }

    void ResourceTaskScheduler::checkDependentTasks(ResourceTask::TaskID completedTaskId) {
        // Check all pending tasks to see if any were waiting on this task
        for (const auto& [taskId, task] : pendingTasks_) {
            const auto& deps = task->getDependencies();

            // If this task depends on the completed task
            if (std::find(deps.begin(), deps.end(), completedTaskId) != deps.end()) {
                // Check if all dependencies are now met
                if (areDependenciesMet(taskId)) {
                    readyQueue_.emplace(task->getPriority(), taskId);
                    taskCV_.notify_one();
                }
            }
        }
    }

    void ResourceTaskScheduler::cleanupCompletedTasks() {
        // Remove oldest completed tasks
        // This is a simple implementation - could be improved with LRU

        while (completedTasks_.size() > config_.maxCompletedHistory / 2) {
            // Find and remove the first (oldest) entry
            if (!completedTasks_.empty()) {
                completedTasks_.erase(completedTasks_.begin());
            }
        }
    }

} // namespace engine::resources
