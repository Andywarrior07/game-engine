// //
// // Created by Andres Guerrero on 23-08-25.
// //
//
// #include "Viewport.h"
//
// namespace engine::camera {
//     Viewport::Viewport() : x_(0), y_(0), width_(1280), height_(720) {
//     }
//
//     Viewport::Viewport(const int x, const int y, const int width, const int height)
//         : x_(x), y_(y), width_(width), height_(height) {
//         validate();
//     }
//
//     Viewport::Viewport(const int screenWidth, const int screenHeight)
//         : x_(0), y_(0), width_(screenWidth), height_(screenHeight) {
//         validate();
//     }
//
//     float Viewport::getAspectRatio() const noexcept {
//         return height_ > 0 ? static_cast<float>(width_) / static_cast<float>(height_) : 1.0f;
//     }
//
//     float Viewport::getCenterX() const noexcept {
//         return x_ + width_ * 0.5f;
//     }
//
//     float Viewport::getCenterY() const noexcept {
//         return y_ + height_ * 0.5f;
//     }
//
//     Vector2 Viewport::getCenter() const {
//         return Vector2(getCenterX(), getCenterY());
//     }
//
//     void Viewport::setSize(const int width, const int height) {
//         width_ = width;
//         height_ = height;
//         validate();
//     }
//
//     void Viewport::set(const int x, const int y, const int width, const int height) {
//         x_ = x;
//         y_ = y;
//         width_ = width;
//         height_ = height;
//         validate();
//     }
//
//     void Viewport::scale(const float scale) {
//         width_ = static_cast<int>(width_ * scale);
//         height_ = static_cast<int>(height_ * scale);
//         validate();
//     }
//
//     void Viewport::centerOn(const int screenWidth, const int screenHeight) {
//         x_ = (screenWidth - width_) / 2;
//         y_ = (screenHeight - height_) / 2;
//     }
//
//     Vector2 Viewport::normalizedToViewport(const Vector2& normalized) const {
//         return Vector2(
//             x_ + (normalized.x + 1.0f) * width_ * 0.5f,
//             y_ + (1.0f - normalized.y) * height_ * 0.5f
//         );
//     }
//
//     Vector2 Viewport::viewportToNormalized(const Vector2& viewport) const {
//         return Vector2(
//             (viewport.x - x_) / (width_ * 0.5f) - 1.0f,
//             1.0f - (viewport.y - y_) / (height_ * 0.5f)
//         );
//     }
//
//     bool Viewport::contains(const int x, const int y) const {
//         return x >= x_ && x < (x_ + width_) &&
//             y >= y_ && y < (y_ + height_);
//     }
//
//     bool Viewport::contains(const Vector2& point) const {
//         return contains(static_cast<int>(point.x), static_cast<int>(point.y));
//     }
//
//     Vector2 Viewport::clamp(const float x, const float y) const {
//         return Vector2(
//             math::clamp(x, static_cast<float>(x_), static_cast<float>(x_ + width_ - 1)),
//             math::clamp(y, static_cast<float>(y_), static_cast<float>(y_ + height_ - 1))
//
//         );
//     }
//
//     std::vector<Viewport> Viewport::splitHorizontal(const int count) const {
//         std::vector<Viewport> viewports;
//         if (count <= 0) return viewports;
//
//         int splitHeight = height_ / count;
//         for (int i = 0; i < count; i++) {
//             viewports.emplace_back(x_, y_ + i * splitHeight, width_, splitHeight);
//         }
//
//         return viewports;
//     }
//
//     std::vector<Viewport> Viewport::splitVertical(const int count) const {
//         std::vector<Viewport> viewports;
//         if (count <= 0) return viewports;
//
//         int splitWidth = width_ / count;
//         for (int i = 0; i < count; i++) {
//             viewports.emplace_back(x_ + i * splitWidth, y_, splitWidth, height_);
//         }
//
//         return viewports;
//     }
//
//     std::vector<Viewport> Viewport::splitQuad() const {
//         std::vector<Viewport> viewports;
//         int halfWidth = width_ / 2;
//         int halfHeight = height_ / 2;
//
//         // Top-left
//         viewports.emplace_back(x_, y_, halfWidth, halfHeight);
//         // Top-right
//         viewports.emplace_back(x_ + halfWidth, y_, halfWidth, halfHeight);
//         // Bottom-left
//         viewports.emplace_back(x_, y_ + halfHeight, halfWidth, halfHeight);
//         // Bottom-right
//         viewports.emplace_back(x_ + halfWidth, y_ + halfHeight, halfWidth, halfHeight);
//
//         return viewports;
//     }
//
//     bool Viewport::isValid() const noexcept {
//         return width_ > 0 && height_ > 0;
//     }
//
//     void Viewport::reset(const int screenWidth, const int screenHeight) {
//         x_ = 0;
//         y_ = 0;
//         width_ = screenWidth;
//         height_ = screenHeight;
//     }
//
//     void Viewport::validate() {
//         width_ = std::max(1, width_);
//         height_ = std::max(1, height_);
//     }
// } // namespace engine::camera
