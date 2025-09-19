/**
 * @file DeviceCapabilities.h
 * @brief Device capability detection and management
 * @author Andrés Guerrero
 * @date 12-09-2025
 *
 * Defines capabilities, features, and properties that input devices can have.
 * Used for feature detection and optimal path selection.
 */

#pragma once

#include "../../core/InputTypes.h"

#include <string>
#include <array>

namespace engine::input {
    /**
     * @brief Extended device capability flags
     */
    enum class DeviceCapabilityFlags : std::uint64_t {
        NONE = 0,

        // Basic input capabilities
        DIGITAL_BUTTONS = 1ULL << 0, // Has digital buttons
        ANALOG_STICKS = 1ULL << 1, // Has analog sticks
        ANALOG_TRIGGERS = 1ULL << 2, // Has analog triggers
        DPAD = 1ULL << 3, // Has directional pad

        // Feedback capabilities
        RUMBLE_BASIC = 1ULL << 4, // Basic rumble support
        RUMBLE_ADVANCED = 1ULL << 5, // Variable frequency rumble
        HAPTIC_FEEDBACK = 1ULL << 6, // Advanced haptic feedback
        ADAPTIVE_TRIGGERS = 1ULL << 7, // PS5-style adaptive triggers
        FORCE_FEEDBACK = 1ULL << 8, // Force feedback (wheels)

        // Motion capabilities
        GYROSCOPE = 1ULL << 9, // Has gyroscope
        ACCELEROMETER = 1ULL << 10, // Has accelerometer
        MAGNETOMETER = 1ULL << 11, // Has magnetometer
        MOTION_TRACKING = 1ULL << 12, // Full motion tracking

        // Special features
        TOUCHPAD = 1ULL << 13, // Has touchpad
        LIGHT_BAR = 1ULL << 14, // Has light bar/LED
        SPEAKER = 1ULL << 15, // Has built-in speaker
        MICROPHONE = 1ULL << 16, // Has microphone
        HEADSET_JACK = 1ULL << 17, // Has audio jack

        // Connectivity
        WIRELESS = 1ULL << 18, // Wireless connection
        BLUETOOTH = 1ULL << 19, // Bluetooth support
        USB = 1ULL << 20, // USB connection
        BATTERY_STATUS = 1ULL << 21, // Can report battery

        // Advanced features
        PRESSURE_SENSITIVE = 1ULL << 22, // Pressure-sensitive buttons
        MULTI_TOUCH = 1ULL << 23, // Multi-touch support
        HIGH_PRECISION = 1ULL << 24, // High DPI/precision
        PROGRAMMABLE = 1ULL << 25, // Programmable buttons
        PROFILE_SWITCHING = 1ULL << 26, // Profile switching

        // Platform-specific
        STEAM_INPUT = 1ULL << 27, // Steam Input API support
        XINPUT = 1ULL << 28, // XInput support
        DIRECTINPUT = 1ULL << 29, // DirectInput support
        RAW_INPUT = 1ULL << 30, // Raw input support

        // Extended capabilities
        RGB_LIGHTING = 1ULL << 31, // RGB lighting control
        OLED_DISPLAY = 1ULL << 32, // Has OLED display
        BIOMETRIC = 1ULL << 33, // Biometric sensors
        AI_ENHANCED = 1ULL << 34, // AI-enhanced features
    };

    // Operators for DeviceCapabilityFlags
    inline DeviceCapabilityFlags operator|(DeviceCapabilityFlags a, DeviceCapabilityFlags b) noexcept {
        return static_cast<DeviceCapabilityFlags>(
            static_cast<std::uint64_t>(a) | static_cast<std::uint64_t>(b)
        );
    }

    inline DeviceCapabilityFlags operator&(DeviceCapabilityFlags a, DeviceCapabilityFlags b) noexcept {
        return static_cast<DeviceCapabilityFlags>(
            static_cast<std::uint64_t>(a) & static_cast<std::uint64_t>(b)
        );
    }

    inline DeviceCapabilityFlags operator~(DeviceCapabilityFlags a) noexcept {
        return static_cast<DeviceCapabilityFlags>(
            ~static_cast<std::uint64_t>(a)
        );
    }


    inline DeviceCapabilityFlags& operator|=(DeviceCapabilityFlags& a, const DeviceCapabilityFlags b) noexcept {
        a = a | b;
        return a;
    }

    inline DeviceCapabilityFlags& operator&=(DeviceCapabilityFlags& a, const DeviceCapabilityFlags b) noexcept {
        a = a & b;
        return a;
    }

    inline bool hasCapability(const DeviceCapabilityFlags caps, const DeviceCapabilityFlags cap) noexcept {
        return (caps & cap) == cap;
    }

    /**
     * @brief Device performance characteristics
     */
    struct DevicePerformanceInfo {
        std::uint32_t maxPollRate; // Maximum polling rate in Hz
        std::uint32_t currentPollRate; // Current polling rate in Hz
        std::uint32_t maxDPI; // Maximum DPI (for mice)
        std::uint32_t currentDPI; // Current DPI
        float latencyMs; // Average latency in milliseconds
        float jitter; // Input jitter (0-1)
        std::uint32_t bufferSize; // Internal buffer size
        std::uint32_t droppedPackets; // Number of dropped packets

        DevicePerformanceInfo() noexcept
            : maxPollRate(1000)
              , currentPollRate(125)
              , maxDPI(0)
              , currentDPI(0)
              , latencyMs(0.0f)
              , jitter(0.0f)
              , bufferSize(0)
              , droppedPackets(0) {
        }
    };

    /**
     * @brief Detailed device specifications
     */
    struct DeviceSpecs {
        // Basic info
        std::uint16_t vendorId;
        std::uint16_t productId;
        std::uint16_t versionNumber;
        std::string manufacturer{};
        std::string productName{};
        std::string serialNumber{};
        std::string firmwareVersion{};

        // Physical characteristics
        std::uint8_t buttonCount;
        std::uint8_t axisCount;
        std::uint8_t hatCount;
        std::uint8_t ballCount;

        // Analog specifications
        struct AnalogSpec {
            float minValue;
            float maxValue;
            float deadzone;
            std::uint16_t resolution; // Bits of precision
            bool hasDeadzone;

            AnalogSpec() noexcept
                : minValue(-1.0f)
                  , maxValue(1.0f)
                  , deadzone(0.0f)
                  , resolution(8)
                  , hasDeadzone(false) {
            }
        };

        std::array<AnalogSpec, 8> analogSpecs{}; // For each axis

        // Rumble/Haptic specifications
        struct RumbleSpec {
            float minFrequency; // Hz
            float maxFrequency; // Hz
            float minAmplitude; // 0-1
            float maxAmplitude; // 0-1
            std::uint8_t motorCount;
            bool supportsCustomWaveforms;

            RumbleSpec() noexcept
                : minFrequency(0.0f)
                  , maxFrequency(1000.0f)
                  , minAmplitude(0.0f)
                  , maxAmplitude(1.0f)
                  , motorCount(0)
                  , supportsCustomWaveforms(false) {
            }
        } rumbleSpec;

        // Motion sensor specifications
        struct MotionSpec {
            float gyroMaxDPS; // Degrees per second
            float accelMaxG; // G-forces
            float sampleRate; // Hz
            std::uint8_t axes; // Number of axes (3 or 6)

            MotionSpec() noexcept
                : gyroMaxDPS(0.0f)
                  , accelMaxG(0.0f)
                  , sampleRate(0.0f)
                  , axes(0) {
            }
        } motionSpec{};

        DeviceSpecs() noexcept
            : vendorId(0)
              , productId(0)
              , versionNumber(0)
              , buttonCount(0)
              , axisCount(0)
              , hatCount(0)
              , ballCount(0) {
        }
    };

    /**
     * @brief Device capability set with feature detection
     */
    class DeviceCapabilities {
    public:
        DeviceCapabilities() noexcept
            : flags_(DeviceCapabilityFlags::NONE) {
        }

        /**
         * @brief Set capability flag
         */
        void setCapability(const DeviceCapabilityFlags flag, const bool enabled = true) noexcept {
            if (enabled) {
                flags_ |= flag;
            }
            else {
                flags_ &= ~flag;
            }
        }

        /**
         * @brief Check if device has capability
         */
        [[nodiscard]] bool hasCapability(const DeviceCapabilityFlags flag) const noexcept {
            return input::hasCapability(flags_, flag);
        }

        /**
         * @brief Check if device has all specified capabilities
         */
        [[nodiscard]] bool hasAllCapabilities(const DeviceCapabilityFlags flags) const noexcept {
            return (flags_ & flags) == flags;
        }

        /**
         * @brief Check if device has any of the specified capabilities
         */
        [[nodiscard]] bool hasAnyCapability(const DeviceCapabilityFlags flags) const noexcept {
            return (flags_ & flags) != DeviceCapabilityFlags::NONE;
        }

        /**
         * @brief Get all capability flags
         */
        [[nodiscard]] DeviceCapabilityFlags getFlags() const noexcept {
            return flags_;
        }

        /**
         * @brief Get device specifications
         */
        [[nodiscard]] const DeviceSpecs& getSpecs() const noexcept {
            return specs_;
        }

        [[nodiscard]] DeviceSpecs& getSpecsMutable() noexcept {
            return specs_;
        }

        /**
         * @brief Get performance info
         */
        [[nodiscard]] const DevicePerformanceInfo& getPerformance() const noexcept {
            return performance_;
        }

        [[nodiscard]] DevicePerformanceInfo& getPerformanceMutable() noexcept {
            return performance_;
        }

        /**
         * @brief Check if device supports rumble
         */
        [[nodiscard]] bool supportsRumble() const noexcept {
            return hasAnyCapability(DeviceCapabilityFlags::RUMBLE_BASIC |
                DeviceCapabilityFlags::RUMBLE_ADVANCED |
                DeviceCapabilityFlags::HAPTIC_FEEDBACK);
        }

        /**
         * @brief Check if device supports motion sensing
         */
        [[nodiscard]] bool supportsMotion() const noexcept {
            return hasAnyCapability(DeviceCapabilityFlags::GYROSCOPE |
                DeviceCapabilityFlags::ACCELEROMETER |
                DeviceCapabilityFlags::MOTION_TRACKING);
        }

        /**
         * @brief Check if device is wireless
         */
        [[nodiscard]] bool isWireless() const noexcept {
            return hasAnyCapability(DeviceCapabilityFlags::WIRELESS |
                DeviceCapabilityFlags::BLUETOOTH);
        }

        /**
         * @brief Get capability string for debugging
         */
        [[nodiscard]] std::string getCapabilityString() const {
            std::string result = "Capabilities: ";

            struct CapabilityName {
                DeviceCapabilityFlags flag;
                const char* name;
            };

            static const CapabilityName names[] = {
                {DeviceCapabilityFlags::DIGITAL_BUTTONS, "DigitalButtons"},
                {DeviceCapabilityFlags::ANALOG_STICKS, "AnalogSticks"},
                {DeviceCapabilityFlags::ANALOG_TRIGGERS, "AnalogTriggers"},
                {DeviceCapabilityFlags::DPAD, "DPad"},
                {DeviceCapabilityFlags::RUMBLE_BASIC, "BasicRumble"},
                {DeviceCapabilityFlags::RUMBLE_ADVANCED, "AdvancedRumble"},
                {DeviceCapabilityFlags::HAPTIC_FEEDBACK, "HapticFeedback"},
                {DeviceCapabilityFlags::ADAPTIVE_TRIGGERS, "AdaptiveTriggers"},
                {DeviceCapabilityFlags::GYROSCOPE, "Gyroscope"},
                {DeviceCapabilityFlags::ACCELEROMETER, "Accelerometer"},
                {DeviceCapabilityFlags::TOUCHPAD, "Touchpad"},
                {DeviceCapabilityFlags::LIGHT_BAR, "LightBar"},
                {DeviceCapabilityFlags::WIRELESS, "Wireless"},
                {DeviceCapabilityFlags::BATTERY_STATUS, "BatteryStatus"}
            };

            bool first = true;
            for (const auto& cap : names) {
                if (hasCapability(cap.flag)) {
                    if (!first) result += ", ";
                    result += cap.name;
                    first = false;
                }
            }

            return result;
        }

        /**
         * @brief Auto-detect capabilities based on device type
         */
        void autoDetectCapabilities(const DeviceType type) noexcept {
            flags_ = DeviceCapabilityFlags::NONE;

            switch (type) {
            case DeviceType::KEYBOARD:
                flags_ |= DeviceCapabilityFlags::DIGITAL_BUTTONS;
                flags_ |= DeviceCapabilityFlags::USB;
                specs_.buttonCount = 255; // Standard keyboard keys
                break;

            case DeviceType::MOUSE:
                flags_ |= DeviceCapabilityFlags::DIGITAL_BUTTONS;
                flags_ |= DeviceCapabilityFlags::HIGH_PRECISION;
                flags_ |= DeviceCapabilityFlags::USB;
                specs_.buttonCount = 8;
                specs_.axisCount = 3; // X, Y, Wheel
                performance_.maxDPI = 16000; // Modern gaming mouse
                break;

            case DeviceType::GAMEPAD:
                flags_ |= DeviceCapabilityFlags::DIGITAL_BUTTONS;
                flags_ |= DeviceCapabilityFlags::ANALOG_STICKS;
                flags_ |= DeviceCapabilityFlags::ANALOG_TRIGGERS;
                flags_ |= DeviceCapabilityFlags::DPAD;
                flags_ |= DeviceCapabilityFlags::RUMBLE_BASIC;
                specs_.buttonCount = 16;
                specs_.axisCount = 6; // 2 sticks + 2 triggers
                specs_.hatCount = 1; // D-pad
                specs_.rumbleSpec.motorCount = 2;
                break;

            case DeviceType::TOUCH:
                flags_ |= DeviceCapabilityFlags::MULTI_TOUCH;
                flags_ |= DeviceCapabilityFlags::PRESSURE_SENSITIVE;
                specs_.axisCount = 2; // X, Y per touch
                break;

            default:
                break;
            }
        }

    private:
        DeviceCapabilityFlags flags_;
        DeviceSpecs specs_;
        DevicePerformanceInfo performance_;
    };

    /**
     * @brief Capability requirements for input contexts
     */
    struct CapabilityRequirements {
        DeviceCapabilityFlags required; // Must have all these
        DeviceCapabilityFlags preferred; // Nice to have
        DeviceCapabilityFlags forbidden; // Must not have

        CapabilityRequirements() noexcept
            : required(DeviceCapabilityFlags::NONE)
              , preferred(DeviceCapabilityFlags::NONE)
              , forbidden(DeviceCapabilityFlags::NONE) {
        }

        /**
         * @brief Check if device meets requirements
         */
        [[nodiscard]] bool isSatisfiedBy(const DeviceCapabilities& caps) const noexcept {
            // Check required capabilities
            if (!caps.hasAllCapabilities(required)) {
                return false;
            }

            // Check forbidden capabilities
            if (caps.hasAnyCapability(forbidden)) {
                return false;
            }

            return true;
        }

        /**
         * @brief Calculate match score (0-1)
         */
        [[nodiscard]] float getMatchScore(const DeviceCapabilities& caps) const noexcept {
            if (!isSatisfiedBy(caps)) {
                return 0.0f;
            }

            // Base score for meeting requirements
            float score = 0.5f;

            // Add bonus for preferred capabilities
            if (preferred != DeviceCapabilityFlags::NONE) {
                std::uint32_t preferredCount = 0;
                std::uint32_t hasCount = 0;

                for (std::uint32_t i = 0; i < 64; ++i) {
                    if (const auto flag = static_cast<DeviceCapabilityFlags>(1ULL << i);
                        hasCapability(preferred, flag)) {
                        preferredCount++;
                        if (caps.hasCapability(flag)) {
                            hasCount++;
                        }
                    }
                }

                if (preferredCount > 0) {
                    score += 0.5f * (static_cast<float>(hasCount) / static_cast<float>(preferredCount));
                }
            }

            return score;
        }
    };
} // namespace engine::input
