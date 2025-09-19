/**
 * @file MouseDevice.h
 * @brief Mouse input device implementation
 * @author Andrés Guerrero
 * @date 12-09-2025
 *
 * Handles mouse input processing including movement, buttons, wheel,
 * and cursor locking functionality.
 */

#pragma once

#include "MouseState.h"
#include "MouseLock.h"

#include "../base/InputDevice.h"
#include "../../core/InputEvent.h"
#include "../../utils/InputUtils.h"

namespace engine::input {
    /**
     * @brief Mouse device implementation
     *
     * Manages mouse input with support for relative/absolute movement,
     * multiple buttons, scroll wheel, and cursor locking.
     */
    class MouseDevice final : public InputDeviceBase {
    public:
        /**
         * @brief Constructor
         */
        explicit MouseDevice() noexcept;

        /**
         * @brief Destructor
         */
        ~MouseDevice() override = default;

        // ============================================================================
        // InputDevice Interface Implementation
        // ============================================================================

        bool initialize(const DeviceInitParams& params) override;
        void shutdown() override;
        void reset() noexcept override;
        void update(float deltaTime) override;
        bool pollEvents() override;
        bool processRawInput(const void* data, std::size_t size) override;

        // Rumble support (not applicable for mouse)
        bool setRumble(const float leftMotor, const float rightMotor, const float duration) override {
            (void)leftMotor;
            (void)rightMotor;
            (void)duration;
            return false;
        }

        void stopRumble() noexcept override {
        }

        bool playHapticPattern(const std::uint32_t pattern, const float intensity) override {
            (void)pattern;
            (void)intensity;
            return false;
        }

        // ============================================================================
        // Mouse-Specific Methods
        // ============================================================================

        /**
         * @brief Handle mouse movement
         */
        void onMouseMove(float x, float y, float deltaX, float deltaY);

        /**
         * @brief Handle mouse button press
         */
        void onMouseButtonPress(MouseButton button, float x, float y);

        /**
         * @brief Handle mouse button release
         */
        void onMouseButtonRelease(MouseButton button, float x, float y);

        /**
         * @brief Handle mouse wheel
         */
        void onMouseWheel(float deltaX, float deltaY);

        /**
         * @brief Handle mouse enter window
         */
        void onMouseEnter();

        /**
         * @brief Handle mouse leave window
         */
        void onMouseLeave();

        /**
         * @brief Get mouse lock manager
         */
        [[nodiscard]] MouseLock& getMouseLock() noexcept {
            return mouseLock_;
        }

        [[nodiscard]] const MouseLock& getMouseLock() const noexcept {
            return mouseLock_;
        }

        /**
         * @brief Check if button is pressed
         */
        [[nodiscard]] bool isButtonPressed(const MouseButton button) const noexcept {
            return mouseState_.isButtonPressed(button);
        }

        /**
         * @brief Check if button was just pressed
         */
        [[nodiscard]] bool isButtonJustPressed(const MouseButton button) const noexcept {
            return mouseState_.isButtonJustPressed(button);
        }

        /**
         * @brief Check if button was just released
         */
        [[nodiscard]] bool isButtonJustReleased(const MouseButton button) const noexcept {
            return mouseState_.isButtonJustReleased(button);
        }

        /**
         * @brief Get mouse position
         */
        [[nodiscard]] const math::Vec2& getPosition() const noexcept {
            return mouseState_.position;
        }

        /**
         * @brief Get mouse delta
         */
        [[nodiscard]] const math::Vec2& getDelta() const noexcept {
            return mouseState_.delta;
        }

        /**
         * @brief Get normalized position (0-1)
         */
        [[nodiscard]] const math::Vec2& getNormalizedPosition() const noexcept {
            return mouseState_.normalizedPos;
        }

        /**
         * @brief Get wheel delta
         */
        [[nodiscard]] const math::Vec2& getWheelDelta() const noexcept {
            return mouseState_.wheelDelta;
        }

        /**
         * @brief Check if mouse is in window
         */
        [[nodiscard]] bool isInWindow() const noexcept {
            return mouseState_.isInWindow;
        }

        /**
         * @brief Get mouse state
         */
        [[nodiscard]] const MouseState& getMouseState() const noexcept {
            return mouseState_;
        }

        /**
         * @brief Update snapshot with mouse state
         */
        void updateSnapshot(InputSnapshot& snapshot) const;

        /**
         * @brief Set mouse sensitivity
         */
        void setSensitivity(float sensitivity) noexcept;

        /**
         * @brief Get mouse sensitivity
         */
        [[nodiscard]] float getSensitivity() const noexcept {
            return sensitivity_;
        }

        /**
         * @brief Set acceleration curve
         */
        void setAcceleration(bool enabled, float factor = 1.5f) noexcept;

        /**
         * @brief Set window size for normalization
         */
        void setWindowSize(int width, int height) noexcept;

        /**
         * @brief Enable/disable raw input
         */
        void setRawInputEnabled(const bool enabled) noexcept {
            rawInputEnabled_ = enabled;
        }

        /**
         * @brief Check if raw input is enabled
         */
        [[nodiscard]] bool isRawInputEnabled() const noexcept {
            return rawInputEnabled_;
        }

        /**
         * @brief Set double-click time threshold
         */
        void setDoubleClickTime(const float seconds) noexcept {
            doubleClickTime_ = std::max(0.1f, seconds);
        }

        /**
         * @brief Set double-click distance threshold
         */
        void setDoubleClickDistance(const float pixels) noexcept {
            doubleClickDistance_ = std::max(1.0f, pixels);
        }

    private:
        // ============================================================================
        // Member Variables
        // ============================================================================

        // State tracking
        MouseState mouseState_;
        MouseState previousState_;

        // Mouse lock manager
        MouseLock mouseLock_;

        // Settings
        float sensitivity_;
        bool accelerationEnabled_;
        float accelerationFactor_;
        bool rawInputEnabled_;

        // Window info
        int windowWidth_;
        int windowHeight_;

        // Double-click detection
        float doubleClickTime_; // Maximum time between clicks
        float doubleClickDistance_; // Maximum distance between clicks
        std::array<math::Vec2, static_cast<std::size_t>(MouseButton::BUTTON_COUNT)> lastClickPositions_;

        // Filtering
        utils::OneEuroFilter xFilter_;
        utils::OneEuroFilter yFilter_;
        bool filteringEnabled_;

        // Delta accumulation for sub-pixel precision
        math::Vec2 accumulatedDelta_;

        // ============================================================================
        // Private Methods
        // ============================================================================

        /**
         * @brief Clear mouse state
         */
        void clearState() noexcept;

        /**
         * @brief Process mouse movement with filtering and acceleration
         */
        math::Vec2 processMovement(float deltaX, float deltaY, float dt);

        /**
         * @brief Update click count for double/triple click detection
         */
        void updateClickCount(MouseButton button, const math::Vec2& position);

        /**
         * @brief Generate mouse event
         */
        void generateMouseEvent(InputEventType type, const MouseButtonEventData& data);
        void generateMouseEvent(InputEventType type, const MouseMotionEventData& data);
        void generateMouseEvent(InputEventType type, const MouseWheelEventData& data);

        /**
         * @brief Normalize position to window
         */
        math::Vec2 normalizePosition(const math::Vec2& position) const noexcept;
    };
} // namespace engine::input
