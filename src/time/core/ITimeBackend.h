/**
 * @file ITimeBackend.h
 * @brief Abstract interface for platform-specific time sources
 * @details Defines the contract for platform time backends, providing a unified
 *          interface for high-resolution time queries across different platforms.
 *          Implementations handle platform-specific optimizations and quirks.
 *
 * @author Andres Guerrero
 * @date Created on 2025-09-19
 */

#pragma once

#include "TimeTypes.h"
#include "TimeConfig.h"
#include <string>
#include <memory>
#include <optional>

namespace engine::time {
    // =============================================================================
    // Platform Capabilities
    // =============================================================================

    /**
     * @brief Platform timer capabilities and features
     * @details Describes what timing features are available on the platform.
     *          Used to optimize time system behavior per platform.
     */
    struct PlatformCapabilities {
        bool hasHighResolutionClock{true}; ///< QPC/mach_time available
        bool hasMonotonicClock{true}; ///< Monotonic time source
        bool hasProcessClock{false}; ///< Process-specific time
        bool hasThreadClock{false}; ///< Thread-specific time
        bool hasPreciseSleep{false}; ///< Precise sleep available
        bool hasWaitableTimer{false}; ///< Waitable timer objects
        bool hasClockAdjustmentDetection{false}; ///< Detect time adjustments

        Duration clockResolution{Duration(1000)}; ///< Clock resolution (ns)
        Duration sleepResolution{Duration(1000000)}; ///< Sleep resolution (μs)
        Duration contextSwitchOverhead{Duration(10000)}; ///< Context switch cost

        std::uint64_t clockFrequency{1000000}; ///< Ticks per second
        bool requiresCalibration{false}; ///< Needs calibration
        bool supportsCPUAffinity{true}; ///< Can pin to CPU

        /**
         * @brief Get human-readable capability summary
         */
        [[nodiscard]] std::string toString() const {
            std::string result = "Platform Capabilities:\n";
            result += "  High Resolution: " + std::string(hasHighResolutionClock ? "Yes" : "No") + "\n";
            result += "  Monotonic: " + std::string(hasMonotonicClock ? "Yes" : "No") + "\n";
            result += "  Clock Resolution: " + std::to_string(toMicroseconds(clockResolution)) + " μs\n";
            result += "  Clock Frequency: " + std::to_string(clockFrequency) + " Hz\n";
            return result;
        }
    };

    // =============================================================================
    // Time Backend Interface
    // =============================================================================

    /**
     * @brief Abstract interface for platform-specific time implementations
     * @details All platform backends must implement this interface.
     *          Provides unified access to platform timing features.
     */
    class ITimeBackend {
    public:
        /**
         * @brief Virtual destructor for proper cleanup
         */
        virtual ~ITimeBackend() = default;

        // =============================================================================
        // Lifecycle Management
        // =============================================================================

        /**
         * @brief Initialize the time backend
         * @param config Platform-specific configuration
         * @return True if initialization successful
         * @details Called once during time system initialization.
         *          Should set up platform-specific resources.
         */
        virtual bool initialize(const ClockConfig& config) = 0;

        /**
         * @brief Shutdown the time backend
         * @details Clean up platform-specific resources.
         *          Called during time system shutdown.
         */
        virtual void shutdown() = 0;

        /**
         * @brief Check if backend is initialized
         * @return True if backend is ready for use
         */
        [[nodiscard]] virtual bool isInitialized() const noexcept = 0;

        // =============================================================================
        // Time Queries
        // =============================================================================

        /**
         * @brief Get current high-resolution timestamp
         * @return Current time point
         * @details Primary method for time queries.
         *          Must be as fast as possible (hot path).
         */
        [[nodiscard]] virtual TimeStamp now() const noexcept = 0;

        /**
         * @brief Get monotonic timestamp
         * @return Monotonic time that never goes backwards
         * @details Used for duration measurements.
         *          Immune to system clock adjustments.
         */
        [[nodiscard]] virtual TimeStamp getMonotonicTime() const noexcept = 0;

        /**
         * @brief Get system wall clock time
         * @return Wall clock timestamp
         * @details Used for timestamps that need real-world time.
         *          May jump due to NTP or user adjustments.
         */
        [[nodiscard]] virtual TimeStamp getSystemTime() const noexcept = 0;

        /**
         * @brief Get process CPU time
         * @return CPU time used by process or nullopt if unsupported
         * @details Used for profiling CPU usage.
         *          May not be available on all platforms.
         */
        [[nodiscard]] virtual std::optional<Duration> getProcessTime() const noexcept = 0;

        /**
         * @brief Get thread CPU time
         * @return CPU time used by current thread or nullopt if unsupported
         * @details Used for per-thread profiling.
         *          May not be available on all platforms.
         */
        [[nodiscard]] virtual std::optional<Duration> getThreadTime() const noexcept = 0;

        // =============================================================================
        // Platform Features
        // =============================================================================

        /**
         * @brief Get platform capabilities
         * @return Structure describing platform features
         * @details Called during initialization to optimize behavior.
         */
        [[nodiscard]] virtual PlatformCapabilities getCapabilities() const noexcept = 0;

        /**
         * @brief Get actual clock resolution
         * @return Minimum measurable time difference
         * @details May differ from advertised resolution.
         *          Can be used to calibrate timer precision.
         */
        [[nodiscard]] virtual Duration getClockResolution() const noexcept = 0;

        /**
         * @brief Get clock frequency
         * @return Ticks per second for the clock source
         * @details Used for tick-to-time conversions.
         */
        [[nodiscard]] virtual std::uint64_t getClockFrequency() const noexcept = 0;

        /**
         * @brief Check if clock has been adjusted
         * @return True if system clock was adjusted since last check
         * @details Used to detect NTP adjustments or user changes.
         */
        [[nodiscard]] virtual bool hasClockAdjustmentOccurred() const noexcept = 0;

        // =============================================================================
        // Sleep and Wait Functions
        // =============================================================================

        /**
         * @brief High-precision sleep
         * @param duration Time to sleep
         * @return Actual time slept
         * @details Platform-optimized sleep implementation.
         *          May spin-wait for very short durations.
         */
        virtual Duration sleep(Duration duration) noexcept = 0;

        /**
         * @brief Yield current thread time slice
         * @details Allows other threads to run.
         *          More efficient than sleep(0).
         */
        virtual void yield() noexcept = 0;

        /**
         * @brief Wait until specific timestamp
         * @param targetTime Time to wait until
         * @return True if wait completed normally
         * @details Used for precise frame timing.
         *          May use platform-specific waitable timers.
         */
        virtual bool waitUntil(TimeStamp targetTime) noexcept = 0;

        // =============================================================================
        // Calibration and Optimization
        // =============================================================================

        /**
         * @brief Calibrate timing functions
         * @return True if calibration successful
         * @details Measures actual timer overhead and resolution.
         *          Should be called during initialization.
         */
        virtual bool calibrate() = 0;

        /**
         * @brief Get timing overhead
         * @return Overhead of calling now()
         * @details Used to compensate for measurement overhead.
         *          Determined during calibration.
         */
        [[nodiscard]] virtual Duration getTimingOverhead() const noexcept = 0;

        /**
         * @brief Warm up timing functions
         * @details Pre-fault pages and warm caches.
         *          Reduces first-call latency.
         */
        virtual void warmUp() noexcept = 0;

        // =============================================================================
        // Platform Information
        // =============================================================================

        /**
         * @brief Get backend name
         * @return Human-readable backend identifier
         */
        [[nodiscard]] virtual std::string getName() const noexcept = 0;

        /**
         * @brief Get backend version
         * @return Version string
         */
        [[nodiscard]] virtual std::string getVersion() const noexcept = 0;

        /**
         * @brief Get detailed backend information
         * @return Detailed platform and configuration info
         */
        [[nodiscard]] virtual std::string getDetailedInfo() const noexcept = 0;

        // =============================================================================
        // Error Handling
        // =============================================================================

        /**
         * @brief Get last error message
         * @return Error description or empty if no error
         */
        [[nodiscard]] virtual std::string getLastError() const noexcept = 0;

        /**
         * @brief Clear error state
         */
        virtual void clearError() noexcept = 0;

    protected:
        /**
         * @brief Set error message
         * @param error Error description
         */
        virtual void setError(const std::string& error) noexcept = 0;
    };

    // =============================================================================
    // Backend Factory
    // =============================================================================

    /**
     * @brief Factory for creating platform-specific backends
     * @details Centralizes backend creation and selection logic.
     */
    class TimeBackendFactory {
    public:
        /**
         * @brief Backend type enumeration
         */
        enum class BackendType {
            AUTO, ///< Auto-detect best backend
            SDL, ///< SDL backend
            NATIVE, ///< Native platform backend
            HIGH_PRECISION, ///< Highest precision available
            LOW_POWER ///< Power-efficient backend
        };

        /**
         * @brief Create time backend
         * @param type Backend type to create
         * @return Unique pointer to backend instance
         * @throws std::runtime_error if backend unavailable
         */
        [[nodiscard]] static std::unique_ptr<ITimeBackend> create(
            BackendType type = BackendType::AUTO);

        /**
         * @brief Check if backend type is available
         * @param type Backend type to check
         * @return True if backend can be created
         */
        [[nodiscard]] static bool isAvailable(BackendType type) noexcept;

        /**
         * @brief Get default backend type for platform
         * @return Recommended backend type
         */
        [[nodiscard]] static BackendType getDefaultType() noexcept;

        /**
         * @brief Get all available backend types
         * @return Vector of available backends
         */
        [[nodiscard]] static std::vector<BackendType> getAvailableTypes() noexcept;
    };

    // =============================================================================
    // Backend Base Implementation
    // =============================================================================

    /**
     * @brief Base implementation with common functionality
     * @details Provides default implementations for optional methods.
     *          Concrete backends should derive from this.
     */
    class TimeBackendBase : public ITimeBackend {
    public:
        TimeBackendBase() noexcept = default;
        ~TimeBackendBase() override = default;

        // Default implementations for optional methods

        [[nodiscard]] std::optional<Duration> getProcessTime() const noexcept override {
            return std::nullopt; // Not supported by default
        }

        [[nodiscard]] std::optional<Duration> getThreadTime() const noexcept override {
            return std::nullopt; // Not supported by default
        }

        [[nodiscard]] bool hasClockAdjustmentOccurred() const noexcept override {
            return false; // Can't detect by default
        }

        void yield() noexcept override {
            // Default to sleep(0)
            sleep(Duration::zero());
        }

        bool waitUntil(const TimeStamp targetTime) noexcept override {
            // Default implementation using sleep
            const auto now_time = now();
            if (targetTime <= now_time) {
                return true;
            }

            const auto duration = targetTime - now_time;
            sleep(std::chrono::duration_cast<Duration>(duration));
            return true;
        }

        [[nodiscard]] Duration getTimingOverhead() const noexcept override {
            return overhead_;
        }

        void warmUp() noexcept override {
            // Default warm-up: call now() several times
            for (int i = 0; i < 100; ++i) {
                const volatile auto t = now();
                (void)t;
            }
        }

        [[nodiscard]] std::string getLastError() const noexcept override {
            return lastError_;
        }

        void clearError() noexcept override {
            lastError_.clear();
        }

    protected:
        void setError(const std::string& error) noexcept override {
            lastError_ = error;
        }

        bool initialized_{false}; ///< Initialization state
        Duration overhead_{Duration::zero()}; ///< Measured overhead
        std::string lastError_; ///< Last error message
        ClockConfig config_; ///< Configuration
    };
} // namespace engine::time
