//
// Created by Andres Guerrero on 21-09-25.
//

#include "SDLTimeBackend.h"

namespace engine::time {
    SDLTimeBackend::~SDLTimeBackend() {
        shutdown();
    }

    bool SDLTimeBackend::initialize(const ClockConfig& config) {
        if (initialized_) {
            return true; // Already initialized
        }

        config_ = config;

        // Initialize SDL timer subsystem if not already
        if (SDL_WasInit(SDL_INIT_TIMER) == 0) {
            if (SDL_InitSubSystem(SDL_INIT_TIMER) < 0) {
                setError(std::string("Failed to initialize SDL timer: ") + SDL_GetError());
                return false;
            }
            sdlTimerInitialized_ = true;
        }

        // Get performance counter frequency
        performanceFrequency_ = SDL_GetPerformanceFrequency();
        if (performanceFrequency_ == 0) {
            setError("SDL_GetPerformanceFrequency returned 0");
            return false;
        }

        // Store initial counter for relative time calculations
        initialCounter_ = SDL_GetPerformanceCounter();

        // Sync with system time for wall clock queries
        syncWithSystemTime();

        // Calibrate timing overhead
        if (config_.useHighPrecisionClock && !calibrate()) {
            // Calibration failed but not critical
            overhead_ = Duration(100); // Assume 100ns overhead
        }

        // Warm up timing functions
        warmUp();

        initialized_ = true;
        return true;
    }

    void SDLTimeBackend::shutdown() {
        if (!initialized_) {
            return;
        }

        // Clean up SDL timer if we initialized it
        if (sdlTimerInitialized_) {
            SDL_QuitSubSystem(SDL_INIT_TIMER);
            sdlTimerInitialized_ = false;
        }

        initialized_ = false;
    }

    TimeStamp SDLTimeBackend::now() const noexcept {
        if (!initialized_) {
            return TimeStamp{};
        }

        const std::uint64_t counter = SDL_GetPerformanceCounter();
        const std::uint64_t deltaTicks = counter - initialCounter_;

        // Convert ticks to microseconds
        const std::uint64_t microseconds =
            (deltaTicks * 1000000ULL) / performanceFrequency_;

        return TimeStamp{} + Duration(microseconds);
    }

    TimeStamp SDLTimeBackend::getMonotonicTime() const noexcept {
        return now(); // SDL performance counter is always monotonic
    }

    TimeStamp SDLTimeBackend::getSystemTime() const noexcept {
        const auto monotonic = now();

        return monotonic + systemTimeOffset_;
    }

    PlatformCapabilities SDLTimeBackend::getCapabilities() const noexcept {
        PlatformCapabilities caps;

        caps.hasHighResolutionClock = true; // SDL always provides this
        caps.hasMonotonicClock = true; // Performance counter is monotonic
        caps.hasProcessClock = false; // SDL doesn't provide process time
        caps.hasThreadClock = false; // SDL doesn't provide thread time
        caps.hasPreciseSleep = false; // SDL_Delay has limited precision
        caps.hasWaitableTimer = false; // Not available through SDL
        caps.hasClockAdjustmentDetection = false; // Can't detect through SDL

        // Calculate actual resolution
        if (performanceFrequency_ > 0) {
            // Resolution in nanoseconds
            const std::uint64_t nsPerTick = 1000000000ULL / performanceFrequency_;
            caps.clockResolution = Duration(nsPerTick / 1000); // Convert to microseconds
            caps.clockFrequency = performanceFrequency_;
        }
        else {
            caps.clockResolution = Duration(1000); // 1ms fallback
            caps.clockFrequency = 1000000; // 1MHz fallback
        }

        // SDL_Delay typically has 10-15ms resolution
        caps.sleepResolution = Duration(10000); // 10ms

        // Context switch overhead varies by platform
#ifdef _WIN32
        caps.contextSwitchOverhead = Duration(5000); // ~5μs on Windows
#elif defined(__APPLE__)
        caps.contextSwitchOverhead = Duration(2000); // ~2μs on macOS
#else
        caps.contextSwitchOverhead = Duration(10000); // ~10μs on Linux
#endif

        caps.requiresCalibration = config_.useHighPrecisionClock;
        caps.supportsCPUAffinity = true; // Most platforms support this

        return caps;
    }

    Duration SDLTimeBackend::getClockResolution() const noexcept {
        if (performanceFrequency_ == 0) {
            return Duration(1000); // 1ms fallback
        }

        // Calculate microseconds per tick
        const std::uint64_t nsPerTick = 1000000000ULL / performanceFrequency_;
        return Duration(nsPerTick / 1000); // Convert to microseconds
    }

    Duration SDLTimeBackend::sleep(const Duration duration) noexcept {
        if (duration <= Duration::zero()) return Duration::zero();

        const auto start = now();

        // Estrategia escalonada de sleep
        if (duration > Duration(2000)) {  // > 2ms
            // Sleep la mayoría del tiempo
            const auto sleepTime = duration - Duration(500);  // Leave 0.5ms for spin
            SDL_Delay(static_cast<uint32_t>(toMilliseconds(sleepTime)));
        }

        // Spin-wait final con yield inteligente
        const auto target = start + duration;
        while (now() < target) {
            std::this_thread::yield();  // Más eficiente que SDL_Delay(0)
        }

        return std::chrono::duration_cast<Duration>(now() - start);
    }

    bool SDLTimeBackend::calibrate() {
        if (!initialized_) {
            return false;
        }

        constexpr int CALIBRATION_SAMPLES = 1000;
        std::uint64_t totalOverhead = 0;

        // Measure timing overhead
        for (int i = 0; i < CALIBRATION_SAMPLES; ++i) {
            const std::uint64_t start = SDL_GetPerformanceCounter();
            const volatile std::uint64_t dummy = SDL_GetPerformanceCounter();
            const std::uint64_t end = SDL_GetPerformanceCounter();

            (void)dummy; // Prevent optimization

            totalOverhead += (end - start);
        }

        // Calculate average overhead in microseconds
        const std::uint64_t avgOverheadTicks = totalOverhead / CALIBRATION_SAMPLES;
        const std::uint64_t overheadMicros =
            (avgOverheadTicks * 1000000ULL) / performanceFrequency_;

        overhead_ = Duration(overheadMicros);

        // Verify resolution by measuring minimum delta
        std::uint64_t minDelta = std::numeric_limits<std::uint64_t>::max();

        for (int i = 0; i < CALIBRATION_SAMPLES; ++i) {
            const std::uint64_t t1 = SDL_GetPerformanceCounter();

            if (const std::uint64_t t2 = SDL_GetPerformanceCounter(); t2 > t1) {
                minDelta = std::min(minDelta, t2 - t1);
            }
        }

        if (minDelta < std::numeric_limits<std::uint64_t>::max()) {
            const std::uint64_t resolutionMicros =
                (minDelta * 1000000ULL) / performanceFrequency_;
            measuredResolution_ = Duration(resolutionMicros);
        }

        return true;
    }

    void SDLTimeBackend::syncWithSystemTime() {
        const auto sdlNow = now();
        const auto systemNow = Clock::now();

        systemTimeOffset_ = std::chrono::duration_cast<Duration>(
                systemNow.time_since_epoch()) -
            std::chrono::duration_cast<Duration>(
                sdlNow.time_since_epoch());

        lastSystemTimeSync_ = sdlNow;
    }
} // namespace engine::time
