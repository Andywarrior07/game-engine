/**
 * @file MouseLock.h
 * @brief Mouse cursor locking and confinement utilities
 * @author Andrés Guerrero
 * @date 12-09-2025
 *
 * Handles relative mouse mode, cursor confinement, and visibility management.
 * Platform-agnostic with callback system for backend operations.
 */

#pragma once

#include "../../core/InputTypes.h"

#include "../../../math/core/MathFunctions.h"

#include <atomic>
#include <functional>
#include <mutex>

namespace engine::input {
    /**
     * @brief Mouse lock and confinement manager
     *
     * Manages mouse cursor locking for FPS-style controls,
     * cursor confinement to window, and visibility.
     * Uses callbacks to interface with platform-specific backend.
     */
    class MouseLock {
    public:
        enum class LockState {
            UNLOCKED, // Normal cursor movement
            LOCKED, // Cursor locked to center (FPS mode)
            CONFINED, // Cursor confined to area
            HIDDEN // Cursor hidden but not locked
        };

        // Platform callbacks
        using WarpMouseCallback = std::function<void(int x, int y)>;
        using SetRelativeModeCallback = std::function<bool(bool enabled)>;
        using SetCursorVisibilityCallback = std::function<void(bool visible)>;
        using SetConfinementCallback = std::function<bool(int x, int y, int width, int height)>;
        using GetWindowSizeCallback = std::function<std::pair<int, int>()>;

        MouseLock() noexcept
            : state_(LockState::UNLOCKED)
              , isVisible_(true)
              , allowEscape_(true)
              , confineBounds_{0, 0, 0, 0}
              , centerX_(0)
              , centerY_(0)
              , accumulatedDelta_(0, 0)
              , sensitivity_(1.0f)
              , smoothingEnabled_(false)
              , smoothingFactor_(0.5f) {
        }

        /**
         * @brief Set platform callbacks
         */
        void setPlatformCallbacks(WarpMouseCallback warpMouse,
                                  SetRelativeModeCallback setRelativeMode,
                                  SetCursorVisibilityCallback setCursorVisibility,
                                  SetConfinementCallback setConfinement = nullptr,
                                  GetWindowSizeCallback getWindowSize = nullptr) {
            std::lock_guard lock(callbackMutex_);
            warpMouseCallback_ = std::move(warpMouse);
            setRelativeModeCallback_ = std::move(setRelativeMode);
            setCursorVisibilityCallback_ = std::move(setCursorVisibility);
            setConfinementCallback_ = std::move(setConfinement);
            getWindowSizeCallback_ = std::move(getWindowSize);
        }

        /**
         * @brief Initialize mouse lock system
         */
        bool initialize() {
            state_ = LockState::UNLOCKED;
            isVisible_ = true;

            // Get initial window center
            if (getWindowSizeCallback_) {
                auto [width, height] = getWindowSizeCallback_();
                centerX_ = width / 2;
                centerY_ = height / 2;
            }

            return true;
        }

        /**
         * @brief Shutdown mouse lock system
         */
        void shutdown() {
            unlock();
        }

        /**
         * @brief Update mouse lock state
         */
        void update(const float deltaTime) {
            (void)deltaTime;

            // Apply smoothing to accumulated delta if enabled
            if (smoothingEnabled_ && state_ == LockState::LOCKED) {
                accumulatedDelta_ = accumulatedDelta_ * smoothingFactor_;
            }
        }

        /**
         * @brief Lock cursor to center (FPS mode)
         */
        bool lock() {
            if (state_ == LockState::LOCKED) return true;

            std::lock_guard lock(callbackMutex_);

            // Set relative mode
            if (setRelativeModeCallback_) {
                if (!setRelativeModeCallback_(true)) {
                    return false;
                }
            }

            // Hide cursor
            if (setCursorVisibilityCallback_) {
                setCursorVisibilityCallback_(false);
            }

            state_ = LockState::LOCKED;
            isVisible_ = false;
            accumulatedDelta_ = math::VEC2_ZERO;

            return true;
        }

        /**
         * @brief Unlock cursor
         */
        void unlock() {
            if (state_ == LockState::UNLOCKED) return;

            std::lock_guard lock(callbackMutex_);

            // Disable relative mode
            if (setRelativeModeCallback_) {
                // TODO: Revisar esto
                setRelativeModeCallback_(false);
            }

            // Show cursor
            if (setCursorVisibilityCallback_) {
                setCursorVisibilityCallback_(true);
            }

            state_ = LockState::UNLOCKED;
            isVisible_ = true;
            accumulatedDelta_ = math::VEC2_ZERO;
        }

        /**
         * @brief Confine cursor to rectangle
         */
        bool confine(const int x, const int y, const int width, const int height) {
            confineBounds_ = {x, y, width, height};

            std::lock_guard lock(callbackMutex_);

            if (setConfinementCallback_) {
                if (!setConfinementCallback_(x, y, width, height)) {
                    return false;
                }
            }

            state_ = LockState::CONFINED;
            return true;
        }

        /**
         * @brief Hide cursor without locking
         */
        void hide() {
            std::lock_guard lock(callbackMutex_);

            if (setCursorVisibilityCallback_) {
                setCursorVisibilityCallback_(false);
            }

            isVisible_ = false;
            state_ = LockState::HIDDEN;
        }

        /**
         * @brief Show cursor
         */
        void show() {
            std::lock_guard lock(callbackMutex_);

            if (setCursorVisibilityCallback_) {
                setCursorVisibilityCallback_(true);
            }

            isVisible_ = true;
            if (state_ == LockState::HIDDEN) {
                state_ = LockState::UNLOCKED;
            }
        }

        /**
         * @brief Check if cursor is locked
         */
        [[nodiscard]] bool isLocked() const noexcept {
            return state_ == LockState::LOCKED;
        }

        /**
         * @brief Check if cursor is confined
         */
        [[nodiscard]] bool isConfined() const noexcept {
            return state_ == LockState::CONFINED;
        }

        /**
         * @brief Check if cursor is visible
         */
        [[nodiscard]] bool isVisible() const noexcept {
            return isVisible_;
        }

        /**
         * @brief Get lock state
         */
        [[nodiscard]] LockState getState() const noexcept {
            return state_;
        }

        /**
         * @brief Set whether escape key unlocks
         */
        void setAllowEscape(const bool allow) noexcept {
            allowEscape_ = allow;
        }

        /**
         * @brief Process mouse movement in locked mode
         */
        [[nodiscard]] math::Vec2 processLockedMovement(float deltaX, float deltaY) {
            if (state_ != LockState::LOCKED) {
                return math::Vec2(deltaX, deltaY);
            }

            // Apply sensitivity
            deltaX *= sensitivity_;
            deltaY *= sensitivity_;

            // Accumulate delta
            accumulatedDelta_.x += deltaX;
            accumulatedDelta_.y += deltaY;

            // Reset cursor to center
            std::lock_guard lock(callbackMutex_);
            if (warpMouseCallback_) {
                warpMouseCallback_(centerX_, centerY_);
            }

            return accumulatedDelta_;
        }

        /**
         * @brief Clamp position to confinement bounds
         */
        [[nodiscard]] math::Vec2 clampToConfinement(const math::Vec2& position) const {
            if (state_ != LockState::CONFINED) {
                return position;
            }

            math::Vec2 clamped = position;
            clamped.x = math::clamp(clamped.x,
                                    static_cast<float>(confineBounds_.x),
                                    static_cast<float>(confineBounds_.x + confineBounds_.width));
            clamped.y = math::clamp(clamped.y,
                                    static_cast<float>(confineBounds_.y),
                                    static_cast<float>(confineBounds_.y + confineBounds_.height));
            return clamped;
        }

        /**
         * @brief Set window center for locked mode
         */
        void setWindowCenter(const int x, const int y) noexcept {
            centerX_ = x;
            centerY_ = y;
        }

        /**
         * @brief Update window size (updates center)
         */
        void updateWindowSize(const int width, const int height) noexcept {
            centerX_ = width / 2;
            centerY_ = height / 2;
        }

        /**
         * @brief Reset accumulated delta
         */
        void resetDelta() noexcept {
            accumulatedDelta_ = math::VEC2_ZERO;
        }

        /**
         * @brief Set mouse sensitivity
         */
        void setSensitivity(const float sensitivity) noexcept {
            sensitivity_ = math::clamp(sensitivity, 0.1f, 10.0f);
        }

        /**
         * @brief Get mouse sensitivity
         */
        [[nodiscard]] float getSensitivity() const noexcept {
            return sensitivity_;
        }

        /**
         * @brief Enable/disable smoothing
         */
        void setSmoothing(const bool enabled, const float factor = 0.5f) noexcept {
            smoothingEnabled_ = enabled;
            smoothingFactor_ = math::clamp(factor, 0.0f, 1.0f);
        }

        /**
         * @brief Check if escape unlock is allowed
         */
        [[nodiscard]] bool isEscapeAllowed() const noexcept {
            return allowEscape_;
        }

        /**
         * @brief Handle escape key (called by input system)
         */
        bool handleEscape() {
            if (allowEscape_ && state_ == LockState::LOCKED) {
                unlock();
                return true;
            }
            return false;
        }

    private:
        std::atomic<LockState> state_;
        std::atomic<bool> isVisible_;
        bool allowEscape_;

        struct Bounds {
            int x, y, width, height;
        } confineBounds_;

        int centerX_;
        int centerY_;
        math::Vec2 accumulatedDelta_;

        // Mouse settings
        float sensitivity_;
        bool smoothingEnabled_;
        float smoothingFactor_;

        // Platform callbacks
        mutable std::mutex callbackMutex_;
        WarpMouseCallback warpMouseCallback_;
        SetRelativeModeCallback setRelativeModeCallback_;
        SetCursorVisibilityCallback setCursorVisibilityCallback_;
        SetConfinementCallback setConfinementCallback_;
        GetWindowSizeCallback getWindowSizeCallback_;
    };
} // namespace engine::input
