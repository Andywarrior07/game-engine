/**
 * @file Viewport.h
 * @brief Viewport configuration for camera rendering
 * @author Andrés Guerrero
 * @date 23-08-2025
 */

#pragma once

#include "CameraTypes.h"

namespace engine::camera {
    /**
     * @brief Viewport configuration for camera rendering
     *
     * Defines the rectangular area of the screen where a camera renders its view.
     * Supports multiple viewports for split-screen, minimaps, and picture-in-picture effects.
     */
    class Viewport {
    public:
        /**
         * @brief Default constructor - create fullscreen viewport
         */
        Viewport();

        /**
         * @brief Constructor with specific dimensions
         * @param x Left position un pixels
         * @param y Top position in pixels
         * @param width Width in pixels
         * @param height Height in pixels
         */
        Viewport(int x, int y, int width, int height);

        /**
         * @brief Constructor from screen dimensions (fullscreen)
         * @param screenWidth Screen width in pixels
         * @param screenHeight Screen height in pixels
         */
        Viewport(int screenWidth, int screenHeight);

        // ========================================================================
        // ACCESSORS
        // ========================================================================

        /**
         * @brief Get viewport X position
         * @return Left position in pixels
         */
        [[nodiscard]] int getX() const noexcept { return x_; }

        /**
         * @brief Get viewport Y position
         * @return Top position in pixels
         */
        [[nodiscard]] int getY() const noexcept { return y_; }

        /**
         * @brief Get viewport width
         * @return Width in pixels
         */
        [[nodiscard]] int getWidth() const noexcept { return width_; }

        /**
         * @brief Get viewport height
         * @return Height in pixels
         */
        [[nodiscard]] int getHeight() const noexcept { return height_; }

        /**
         * @brief Get aspect ratio
         * @return Width/height ratio
         */
        [[nodiscard]] float getAspectRatio() const noexcept;

        /**
         * @brief Get viewport center X
         * @return Center X coordinate in pixels
         */
        [[nodiscard]] float getCenterX() const noexcept;

        /**
         * @brief Get view port center Y
         * @return Center Y coordinate in pixels
         */
        [[nodiscard]] float getCenterY() const noexcept;

        /**
         * @brief Get viewport center as 2D vector
         * @return Center position
         */
        [[nodiscard]] Vec2 getCenter() const;

        /**
         * @brief Get viewport area
         * @return Area in ssquare pixels
         */
        [[nodiscard]] int getArea() const noexcept {
            return width_ * height_;
        }

        // ========================================================================
        // MUTATORS
        // ========================================================================

        /**
         * @brief Set viewport position
         * @param x Left position in pixels
         * @param y Top position in pixels
         */
        void setPosition(const int x, const int y) {
            x_ = x;
            y_ = y;
        }

        /**
         * @brief Set viewport size
         * @param width Width in pixels
         * @param height Height in pixels
         */
        void setSize(int width, int height);

        /**
         * @brief Set all viewport parameters
         * @param x Left position in pixels
         * @param y Top position in pixels
         * @param width Width in pixels
         * @param height Height in pixels
         */
        void set(int x, int y, int width, int height);

        /**
         * @brief Scale viewport by a factor
         * @param scale Scale factor (1.0 = no change)
         */
        void scale(float scale);

        /**
         * @brief Center viewport on screen
         * @param screenWidth Screen width
         * @param screenHeight Screen height
         */
        void centerOn(int screenWidth, int screenHeight);

        // ========================================================================
        // COORDINATE CONVERSIONS
        // ========================================================================

        /**
         * @brief Concert normalized coordinates to viewport coordinates
         * @param normalized Normalized coordinates (-1 to 1)
         * @return Viewport coordinates in pixels
         */
        [[nodiscard]] Vec2 normalizedToViewport(const Vec2& normalized) const;

        /**
         * @brief Convert viewport coordinates to normalized coordinates
         * @param viewport Viewport coordinates in pixels
         * @return Normalized coordinates (-1 to 1)
         */
        [[nodiscard]] Vec2 viewportToNormalized(const Vec2& viewport) const;

        /**
         * @brief Check if point is inside viewport
         * @param x X coordinate in pixels
         * @param y Y coordinate in pixels
         * @return true if point is within viewport bounds
         */
        [[nodiscard]] bool contains(int x, int y) const;

        /**
         * @brief Check if point is inside viewport
         * @param point Point in pixels
         * @return true if point is within viewport bounds
         */
        [[nodiscard]] bool contains(const Vec2& point) const;

        /**
         * @brief Clamp coordinates to viewport bounds
         * @param x X coordinate to clamp
         * @param y Y coordinate to clamp
         * @return Clamped coordinates
         */
        [[nodiscard]] Vec2 clamp(float x, float y) const;

        // ========================================================================
        // SPLIT SCREEN UTILITIES
        // ========================================================================

        /**
         * @brief Create horizonal split viewports
         * @param count Number of viewports to create
         * @return Vector of slipt viewports
         */
        [[nodiscard]] std::vector<Viewport> splitHorizontal(int count) const;

        /**
         * @brief Create vertical split viewports
         * @param count Number of viewports to create
         * @return Vector of split viewports
         */
        [[nodiscard]] std::vector<Viewport> splitVertical(int count) const;

        /**
         * @brief Create four-way split viewports
         * @return Vector of four viewports
         */
        [[nodiscard]] std::vector<Viewport> splitQuad() const;

        // ========================================================================
        // VALIDATION
        // ========================================================================

        /**
         * @brief Validate viewport dimensions
         * @return true if viewport is valid
         */
        [[nodiscard]] bool isValid() const noexcept;

        /**
         * @brief Reset to default fullscreen viewport
         * @param screenWidth Screen width
         * @param screenHeight Screen height
         */
        void reset(int screenWidth = 1280, int screenHeight = 720);

        // ========================================================================
        // OPERATORS
        // ========================================================================

        /**
         * @brief  Equality operator
         * @param other Other viewport to compare
         * @return true if viewports are equal
        */
        bool operator==(const Viewport& other) const noexcept {
            return x_ == other.x_ && y_ == other.y_ &&
                width_ == other.width_ && height_ == other.height_;
        }

        /**
         * @brief Inequality operator
         * @param other Other viewport to compare
         * @return true if viewports are different
         */
        bool operator!=(const Viewport& other) const noexcept {
            return !(*this == other);
        }

    private:
        int x_; ///< Left position in pixels
        int y_; ///< Top position in pixels
        int width_; ///< Width in pixels
        int height_; ///< Height in pixels

        /**
         * @brief Internal validation to ensure positive dimensions
         */
        void validate();
    };
} // names-ace engine::camera
