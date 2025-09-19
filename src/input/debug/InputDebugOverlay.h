/**
 * @file InputDebugOverlay.h
 * @brief Visual debug overlay for input system
 * @author Andrés Guerrero
 * @date 13-09-2025
 *
 * Provides visual debugging information for input states, events, and performance.
 * Designed to be rendered on top of the game view.
 */

#pragma once

#include "../core/InputSnapshot.h"
#include "../core/InputEvent.h"

#include <string>
#include <vector>
#include <chrono>
#include <deque>

namespace engine::input::debug {
    /**
     * @brief Overlay display sections
     */
    enum class OverlaySection : std::uint32_t {
        NONE = 0,
        KEYBOARD = 1 << 0,
        MOUSE = 1 << 1,
        GAMEPAD = 1 << 2,
        TOUCH = 1 << 3,
        ACTIONS = 1 << 4,
        EVENTS = 1 << 5,
        PERFORMANCE = 1 << 6,
        CONTEXT = 1 << 7,
        DEVICES = 1 << 8,
        HISTORY = 1 << 9,
        ALL = 0xFFFFFFFF
    };

    inline OverlaySection operator|(OverlaySection a, OverlaySection b) {
        return static_cast<OverlaySection>(
            static_cast<std::uint32_t>(a) | static_cast<std::uint32_t>(b)
        );
    }

    inline OverlaySection operator&(OverlaySection a, OverlaySection b) {
        return static_cast<OverlaySection>(
            static_cast<std::uint32_t>(a) & static_cast<std::uint32_t>(b)
        );
    }

    inline OverlaySection operator^(OverlaySection a, OverlaySection b) {
        return static_cast<OverlaySection>(
            static_cast<std::uint32_t>(a) ^ static_cast<std::uint32_t>(b)
        );
    }


    /**
     * @brief Overlay configuration
     */
    struct OverlayConfig {
        // Display settings
        OverlaySection enabledSections = OverlaySection::ALL;
        bool enabled = false;
        float opacity = 0.8f;

        // Position and size
        float x = 10.0f;
        float y = 10.0f;
        float width = 400.0f;
        float maxHeight = 800.0f;

        // Colors (RGBA)
        struct {
            float background[4] = {0.0f, 0.0f, 0.0f, 0.8f};
            float text[4] = {1.0f, 1.0f, 1.0f, 1.0f};
            float header[4] = {0.2f, 0.8f, 1.0f, 1.0f};
            float pressed[4] = {0.0f, 1.0f, 0.0f, 1.0f};
            float released[4] = {1.0f, 0.0f, 0.0f, 1.0f};
            float analog[4] = {1.0f, 1.0f, 0.0f, 1.0f};
        } colors;

        // Text settings
        float fontSize = 12.0f;
        float lineHeight = 16.0f;
        std::string fontName = "monospace";

        // History settings
        std::size_t maxEventHistory = 20;
        std::size_t maxActionHistory = 10;
        float historyDuration = 2.0f; // Seconds to show history

        // Performance graph
        bool showPerformanceGraph = true;
        std::size_t graphSamples = 120; // 2 seconds at 60fps
        float graphHeight = 100.0f;

        // Update settings
        float updateInterval = 0.016f; // Update overlay at 60fps

        OverlayConfig() = default;
    };

    /**
     * @brief Debug text line
     */
    struct DebugLine {
        std::string text;
        float color[4];
        float indent;
        bool bold;

        explicit DebugLine(const std::string& t = "",
                           const float r = 1.0f, const float g = 1.0f, const float b = 1.0f, const float a = 1.0f,
                           const float ind = 0.0f, const bool bold = false)
            : text(t), indent(ind), bold(bold) {
            color[0] = r;
            color[1] = g;
            color[2] = b;
            color[3] = a;
        }
    };

    // TODO: Revisar esto ya que no tiene implementaciones
    /**
     * @brief Visual debug overlay for input system
     *
     * Generates debug visualization data that can be rendered by the game's
     * rendering system. Does not perform actual rendering.
     */
    class InputDebugOverlay {
    public:
        /**
         * @brief Constructor
         */
        InputDebugOverlay() noexcept;

        /**
         * @brief Destructor
         */
        ~InputDebugOverlay() = default;

        // Disable copy, enable move
        InputDebugOverlay(const InputDebugOverlay&) = delete;
        InputDebugOverlay& operator=(const InputDebugOverlay&) = delete;
        InputDebugOverlay(InputDebugOverlay&&) noexcept = default;
        InputDebugOverlay& operator=(InputDebugOverlay&&) noexcept = default;

        // ============================================================================
        // Configuration
        // ============================================================================

        /**
         * @brief Set overlay configuration
         * @param config New configuration
         */
        void setConfig(const OverlayConfig& config) noexcept {
            config_ = config;
        }

        /**
         * @brief Get overlay configuration
         */
        [[nodiscard]] const OverlayConfig& getConfig() const noexcept {
            return config_;
        }

        /**
         * @brief Enable/disable overlay
         * @param enabled Enable state
         */
        void setEnabled(const bool enabled) noexcept {
            config_.enabled = enabled;
        }

        /**
         * @brief Check if overlay is enabled
         */
        [[nodiscard]] bool isEnabled() const noexcept {
            return config_.enabled;
        }

        /**
         * @brief Toggle overlay visibility
         */
        void toggle() noexcept {
            config_.enabled = !config_.enabled;
        }

        /**
         * @brief Set enabled sections
         * @param sections Sections to enable
         */
        void setEnabledSections(const OverlaySection sections) noexcept {
            config_.enabledSections = sections;
        }

        /**
         * @brief Toggle section
         * @param section Section to toggle
         */
        void toggleSection(const OverlaySection section) noexcept {
            config_.enabledSections = config_.enabledSections ^ section;
        }

        // ============================================================================
        // Update
        // ============================================================================

        /**
         * @brief Update overlay with new snapshot
         * @param snapshot Current input snapshot
         * @param deltaTime Time since last update
         */
        void update(const InputSnapshot& snapshot, float deltaTime);

        /**
         * @brief Add event to history
         * @param event Event to add
         */
        void addEvent(const InputEvent& event);

        /**
         * @brief Add performance sample
         * @param frameTime Frame time in milliseconds
         */
        void addPerformanceSample(float frameTime);

        /**
         * @brief Clear all history
         */
        void clearHistory() noexcept;

        // ============================================================================
        // Rendering Data
        // ============================================================================

        /**
         * @brief Get debug lines to render
         * @return Vector of debug lines
         */
        [[nodiscard]] const std::vector<DebugLine>& getDebugLines() const noexcept {
            return debugLines_;
        }

        /**
         * @brief Get overlay bounds
         * @param x Output: X position
         * @param y Output: Y position
         * @param width Output: Width
         * @param height Output: Height
         */
        void getBounds(float& x, float& y, float& width, float& height) const noexcept {
            x = config_.x;
            y = config_.y;
            width = config_.width;
            height = currentHeight_;
        }

        /**
         * @brief Get performance graph data
         * @return Performance samples
         */
        [[nodiscard]] const std::deque<float>& getPerformanceGraph() const noexcept {
            return performanceSamples_;
        }

        // ============================================================================
        // Custom Sections
        // ============================================================================

        /**
         * @brief Add custom debug section
         * @param name Section name
         * @param lines Debug lines for section
         */
        void addCustomSection(const std::string& name, const std::vector<DebugLine>& lines);

        /**
         * @brief Remove custom section
         * @param name Section name
         */
        void removeCustomSection(const std::string& name);

        /**
         * @brief Clear custom sections
         */
        void clearCustomSections();

    private:
        // ============================================================================
        // Internal Types
        // ============================================================================

        struct EventHistoryEntry {
            InputEvent event;
            std::chrono::steady_clock::time_point timestamp;
        };

        struct ActionHistoryEntry {
            ActionID actionId;
            std::string actionName;
            ActionState state;
            std::chrono::steady_clock::time_point timestamp;
        };

        // ============================================================================
        // Member Variables
        // ============================================================================

        // Configuration
        OverlayConfig config_;

        // Current state
        InputSnapshot currentSnapshot_;
        float currentHeight_ = 0.0f;
        float timeSinceUpdate_ = 0.0f;

        // Debug lines
        std::vector<DebugLine> debugLines_;

        // Event history
        std::deque<EventHistoryEntry> eventHistory_;
        std::deque<ActionHistoryEntry> actionHistory_;

        // Performance tracking
        std::deque<float> performanceSamples_;
        float averageFrameTime_ = 0.0f;
        float peakFrameTime_ = 0.0f;

        // Custom sections
        std::unordered_map<std::string, std::vector<DebugLine>> customSections_;

        // ============================================================================
        // Internal Methods
        // ============================================================================

        /**
         * @brief Generate debug lines
         */
        void generateDebugLines();

        /**
         * @brief Generate keyboard section
         */
        void generateKeyboardSection();

        /**
         * @brief Generate mouse section
         */
        void generateMouseSection();

        /**
         * @brief Generate gamepad section
         */
        void generateGamepadSection();

        /**
         * @brief Generate touch section
         */
        void generateTouchSection();

        /**
         * @brief Generate actions section
         */
        void generateActionsSection();

        /**
         * @brief Generate events section
         */
        void generateEventsSection();

        /**
         * @brief Generate performance section
         */
        void generatePerformanceSection();

        /**
         * @brief Generate context section
         */
        void generateContextSection();

        /**
         * @brief Generate devices section
         */
        void generateDevicesSection();

        /**
         * @brief Generate history section
         */
        void generateHistorySection();

        /**
         * @brief Add section header
         * @param title Section title
         */
        void addSectionHeader(const std::string& title);

        /**
         * @brief Add debug line
         * @param text Line text
         * @param indent Indentation level
         * @param r Red component
         * @param g Green component
         * @param b Blue component
         * @param bold Bold text
         */
        void addLine(const std::string& text,
                     float indent = 0.0f,
                     float r = 1.0f, float g = 1.0f, float b = 1.0f,
                     bool bold = false);

        /**
         * @brief Add separator line
         */
        void addSeparator();

        /**
         * @brief Format key name
         * @param key Key code
         * @return Formatted name
         */
        [[nodiscard]] std::string formatKey(KeyCode key) const;

        /**
         * @brief Format button name
         * @param button Button code
         * @return Formatted name
         */
        [[nodiscard]] std::string formatButton(MouseButton button) const;
        [[nodiscard]] std::string formatButton(GamepadButton button) const;

        /**
         * @brief Format axis value
         * @param value Axis value
         * @return Formatted string
         */
        [[nodiscard]] std::string formatAxis(float value) const;

        /**
         * @brief Format vector
         * @param vec Vector value
         * @return Formatted string
         */
        [[nodiscard]] std::string formatVector(const math::Vec2& vec) const;

        /**
         * @brief Clean up old history entries
         */
        void cleanupHistory();
    };
} // namespace engine::input::debug
