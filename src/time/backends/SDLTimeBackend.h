/**
 * @file SDLTimeBackend.h
 * @brief SDL-specific time backend implementation
 * @details Wraps SDL timing functions to provide cross-platform time source.
 *          Uses SDL_GetPerformanceCounter and SDL_GetPerformanceFrequency for
 *          high-resolution timing with platform-specific optimizations.
 *
 * @author Andres Guerrero
 * @date Created on 2025-09-19
 */

#pragma once

#include "../core/ITimeBackend.h"

#include <SDL2/SDL.h>

#include <atomic>
#include <thread>
#include <chrono>

namespace engine::time {
    /**
     * @brief SDL-based time backend implementation
     * @details Provides cross-platform timing using SDL's performance counter API.
     *          SDL handles platform differences internally, providing consistent
     *          behavior across Windows, macOS, Linux, and other platforms.
     *
     * Performance characteristics:
     * - Windows: Uses QueryPerformanceCounter (QPC)
     * - macOS/iOS: Uses mach_absolute_time
     * - Linux: Uses clock_gettime(CLOCK_MONOTONIC)
     * - Others: Platform-specific high-res timer
     */
    class SDLTimeBackend final : public TimeBackendBase {
    public:
        /**
         * @brief Default constructor
         */
        SDLTimeBackend() noexcept
            : TimeBackendBase()
              , performanceFrequency_(0)
              , initialCounter_(0)
              , lastSystemTimeSync_(TimeStamp{})
              , systemTimeOffset_(Duration::zero()) {
            // Initialize SDL error state
            SDL_ClearError();
        }

        /**
         * @brief Destructor
         */
        ~SDLTimeBackend() override;

        // Delete copy operations
        SDLTimeBackend(const SDLTimeBackend&) = delete;
        SDLTimeBackend& operator=(const SDLTimeBackend&) = delete;

        // =============================================================================
        // Lifecycle Management
        // =============================================================================

        /**
         * @brief Initialize SDL time backend
         * @param config Clock configuration
         * @return True if initialization successful
         */
        bool initialize(const ClockConfig& config) override;

        /**
         * @brief Shutdown SDL time backend
         */
        void shutdown() override;

        /**
         * @brief Check initialization status
         */
        [[nodiscard]] bool isInitialized() const noexcept override {
            return initialized_;
        }

        // =============================================================================
        // Time Queries
        // =============================================================================

        /**
         * @brief Get current high-resolution timestamp
         * @return Current time point
         * @details Primary timing method using SDL performance counter.
         *          Guaranteed monotonic and high resolution.
         */
        [[nodiscard]] TimeStamp now() const noexcept override;

        /**
         * @brief Get monotonic timestamp
         * @return Monotonic time (same as now() for SDL)
         */
        [[nodiscard]] TimeStamp getMonotonicTime() const noexcept override;

        /**
         * @brief Get system wall clock time
         * @return Wall clock timestamp
         * @details Approximates wall clock by adding offset to monotonic time.
         *          Less accurate than system clock but maintains consistency.
         */
        [[nodiscard]] TimeStamp getSystemTime() const noexcept override;

        // =============================================================================
        // Platform Features
        // =============================================================================

        /**
         * @brief Get SDL platform capabilities
         */
        [[nodiscard]] PlatformCapabilities getCapabilities() const noexcept override;

        /**
         * @brief Get clock resolution
         */
        [[nodiscard]] Duration getClockResolution() const noexcept override;

        /**
         * @brief Get clock frequency
         */
        [[nodiscard]] std::uint64_t getClockFrequency() const noexcept override {
            return performanceFrequency_;
        }

        // =============================================================================
        // Sleep and Wait Functions
        // =============================================================================

        /**
         * @brief High-precision sleep
         * @param duration Time to sleep
         * @return Actual time slept
         * @details Uses SDL_Delay for long sleeps, spin-wait for short ones.
         *          SDL_Delay has limited precision (typically 10-15ms).
         */
        // TODO: Mejorado, revisarlo
        Duration sleep(const Duration duration) noexcept override;

        /**
         * @brief Yield current thread
         */
        void yield() noexcept override {
            SDL_Delay(0); // SDL's way to yield
        }

        // =============================================================================
        // Calibration and Optimization
        // =============================================================================

        /**
         * @brief Calibrate timing functions
         * @return True if calibration successful
         * @details Measures actual timer overhead and resolution.
         *          Important for accurate profiling.
         */
        bool calibrate() override;

        // =============================================================================
        // Platform Information
        // =============================================================================

        /**
         * @brief Get backend name
         */
        [[nodiscard]] std::string getName() const noexcept override {
            return "SDL Time Backend";
        }

        /**
         * @brief Get backend version
         */
        [[nodiscard]] std::string getVersion() const noexcept override {
            SDL_version compiled;
            SDL_version linked;

            SDL_VERSION(&compiled);
            SDL_GetVersion(&linked);

            return std::string("SDL ") +
                std::to_string(linked.major) + "." +
                std::to_string(linked.minor) + "." +
                std::to_string(linked.patch) +
                " (compiled with " +
                std::to_string(compiled.major) + "." +
                std::to_string(compiled.minor) + "." +
                std::to_string(compiled.patch) + ")";
        }

        /**
         * @brief Get detailed backend information
         */
        [[nodiscard]] std::string getDetailedInfo() const noexcept override {
            std::string info = getName() + " " + getVersion() + "\n";
            info += "Performance Frequency: " + std::to_string(performanceFrequency_) + " Hz\n";
            info += "Clock Resolution: " + std::to_string(toMicroseconds(getClockResolution())) + " μs\n";
            info += "Measured Resolution: " + std::to_string(toMicroseconds(measuredResolution_)) + " μs\n";
            info += "Timing Overhead: " + std::to_string(toMicroseconds(overhead_)) + " μs\n";
            info += "Platform: " + std::string(SDL_GetPlatform()) + "\n";

            return info;
        }

    private:
        std::uint64_t performanceFrequency_; ///< SDL performance counter frequency
        std::uint64_t initialCounter_; ///< Initial counter value
        bool sdlTimerInitialized_{false}; ///< Whether we initialized SDL timer

        // System time synchronization
        TimeStamp lastSystemTimeSync_; ///< Last sync timestamp
        Duration systemTimeOffset_; ///< Offset for wall clock

        // Calibration results
        Duration measuredResolution_{Duration(1000)}; ///< Measured clock resolution

        /**
         * @brief Synchronize with system time
         * @details Updates offset for wall clock approximation
         */
        void syncWithSystemTime();
    };
} // namespace engine::time
