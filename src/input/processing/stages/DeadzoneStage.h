/**
 * @file DeadzoneStage.h
 * @brief Deadzone processing stage for analog inputs
 * @author Andrés Guerrero
 * @date 12-09-2025
 *
 * Applies deadzone filtering to analog inputs like sticks and triggers.
 */

#pragma once

#include "../../core/InputEvent.h"
#include "../../core/InputTypes.h"
#include "../../utils/InputUtils.h"

#include <unordered_map>

namespace engine::input::processing {
    /**
     * @brief Deadzone configuration per device type
     */
    struct DeadzoneConfig {
        // Stick deadzones
        DeadzoneSettings leftStickDeadzone{0.15f, 0.95f};
        DeadzoneSettings rightStickDeadzone{0.15f, 0.95f};

        // Trigger deadzones
        DeadzoneSettings leftTriggerDeadzone{0.1f, 1.0f};
        DeadzoneSettings rightTriggerDeadzone{0.1f, 1.0f};

        // Mouse wheel deadzone
        float mouseWheelDeadzone = 0.01f;

        // Deadzone types
        enum class DeadzoneType {
            RADIAL, // Circular deadzone (best for sticks)
            AXIAL, // Independent per axis
            HYBRID, // Combination of radial and axial
            SCALED // Scaled radial (preserves direction)
        };

        DeadzoneType stickDeadzoneType = DeadzoneType::HYBRID;
        DeadzoneType triggerDeadzoneType = DeadzoneType::RADIAL;

        // Per-device overrides
        bool usePerDeviceSettings = false;
        std::unordered_map<DeviceID, DeadzoneSettings> deviceOverrides;
    };

    /**
     * @brief Deadzone processing stage
     *
     * Applies deadzone filtering to analog inputs to eliminate drift
     * and improve control precision.
     */
    class DeadzoneStage {
    public:
        DeadzoneStage() noexcept;
        explicit DeadzoneStage(const DeadzoneConfig& config) noexcept;
        ~DeadzoneStage() = default;

        // Disable copy, enable move
        DeadzoneStage(const DeadzoneStage&) = delete;
        DeadzoneStage& operator=(const DeadzoneStage&) = delete;
        DeadzoneStage(DeadzoneStage&&) noexcept = default;
        DeadzoneStage& operator=(DeadzoneStage&&) noexcept = default;

        /**
         * @brief Process a batch of events
         * @param events Events to process (modified in place)
         * @param deltaTime Time since last update
         * @return Number of events processed
         */
        std::size_t process(std::vector<InputEvent>& events, float deltaTime);

        /**
         * @brief Process single event
         * @param event Event to process (modified in place)
         * @return True if event was processed
         */
        bool processEvent(InputEvent& event);

        /**
         * @brief Set deadzone configuration
         */
        void setConfig(const DeadzoneConfig& config) noexcept;

        /**
         * @brief Get current configuration
         */
        [[nodiscard]] const DeadzoneConfig& getConfig() const noexcept {
            return config_;
        }

        /**
         * @brief Set deadzone for specific device
         */
        void setDeviceDeadzone(DeviceID deviceId, const DeadzoneSettings& settings);

        /**
         * @brief Remove device-specific deadzone
         */
        void removeDeviceDeadzone(DeviceID deviceId);

        /**
         * @brief Set stick deadzone
         */
        void setStickDeadzone(const DeadzoneSettings& settings, bool leftStick = true) noexcept;

        /**
         * @brief Set trigger deadzone
         */
        void setTriggerDeadzone(const DeadzoneSettings& settings, bool leftTrigger = true) noexcept;

        /**
         * @brief Reset to default settings
         */
        void reset() noexcept;

        /**
         * @brief Get statistics
         */
        struct Statistics {
            std::uint64_t totalProcessed = 0;
            std::uint64_t eventsFiltered = 0; // Events zeroed out by deadzone
            std::uint64_t sticksProcessed = 0;
            std::uint64_t triggersProcessed = 0;
            std::uint64_t wheelsProcessed = 0;

            void reset() noexcept {
                totalProcessed = 0;
                eventsFiltered = 0;
                sticksProcessed = 0;
                triggersProcessed = 0;
                wheelsProcessed = 0;
            }
        };

        [[nodiscard]] const Statistics& getStatistics() const noexcept {
            return stats_;
        }

        /**
         * @brief Enable/disable deadzone processing
         */
        void setEnabled(const bool enabled) noexcept {
            enabled_ = enabled;
        }

        [[nodiscard]] bool isEnabled() const noexcept {
            return enabled_;
        }

    private:
        // Configuration
        DeadzoneConfig config_;
        bool enabled_ = true;

        // Statistics
        Statistics stats_;

        // Cached values for smooth transitions
        struct CachedValue {
            math::Vec2 lastStickValue;
            float lastTriggerValue;
            std::chrono::steady_clock::time_point lastUpdate;
            bool wasInDeadzone;
        };

        std::unordered_map<std::uint64_t, CachedValue> cachedValues_;

        // ============================================================================
        // Processing Methods
        // ============================================================================

        /**
         * @brief Process gamepad axis event
         */
        bool processGamepadAxis(GamepadAxisEventData& data, DeviceID deviceId);

        /**
         * @brief Process mouse wheel event
         */
        bool processMouseWheel(MouseWheelEventData& data);

        /**
         * @brief Apply deadzone to stick input
         */
        [[nodiscard]] static math::Vec2 applyStickDeadzone(const math::Vec2& input,
                                                           const DeadzoneSettings& settings,
                                                           DeadzoneConfig::DeadzoneType type);

        /**
         * @brief Apply deadzone to trigger input
         */
        [[nodiscard]] static float applyTriggerDeadzone(float input,
                                                        const DeadzoneSettings& settings);

        /**
         * @brief Get deadzone settings for device
         */
        [[nodiscard]] DeadzoneSettings getDeviceDeadzone(DeviceID deviceId,
                                                         GamepadAxis axis) const;

        /**
         * @brief Generate cache key for input
         */
        [[nodiscard]] static std::uint64_t generateCacheKey(DeviceID deviceId,
                                                            GamepadAxis axis) noexcept;

        /**
         * @brief Smooth transition when entering/exiting deadzone
         */
        [[nodiscard]] static float smoothDeadzoneTransition(float current, float previous,
                                                            bool wasInDeadzone, float deltaTime);
    };
} // namespace engine::input::processing
