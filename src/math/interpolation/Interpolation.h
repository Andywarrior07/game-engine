/**
 * @file Interpolation.h
 * @brief Advanced interpolation methods and parametric curves
 * @author Andrés Guerrero
 * @date 27-08-2025
 *
 * Provides spline interpolation, Bezier curves, Catmull-Rom splines,
 * and other advanced interpolation techniques for animation and path following.
 */

#pragma once

#include "../core/MathTypes.h"
#include "../core/MathConstants.h"
#include "../core/MathFunctions.h"

#include <vector>

namespace engine::math {

    // ============================================================================
    // Advanced Interpolation Functions
    // ============================================================================

    /**
     * @brief Cosine interpolation (smoother than linear)
     */
    template<typename T>
    [[nodiscard]] T cosineInterpolate(T a, T b, const Float t) noexcept {
        Float mu = (1.0f - std::cos(t * PI<Float>)) * 0.5f;
        return a * (1.0f - mu) + b * mu;
    }

    /**
     * @brief Cubic interpolation using 4 control points
     * p1 and p2 are the values to interpolate between
     * p0 is the value before p1, p3 is the value after p2
     */
    template<typename T>
    [[nodiscard]] T cubicInterpolate(T p0, T p1, T p2, T p3, Float t) noexcept {
        T a0 = p3 - p2 - p0 + p1;
        T a1 = p0 - p1 - a0;
        T a2 = p2 - p0;
        T a3 = p1;

        Float t2 = t * t;
        Float t3 = t2 * t;

        return a0 * t3 + a1 * t2 + a2 * t + a3;
    }

    /**
     * @brief Catmull-Rom spline interpolation
     * Passes through all control points
     */
    template<typename T>
    [[nodiscard]] T catmullRom(T p0, T p1, T p2, T p3, Float t) noexcept {
        Float t2 = t * t;
        Float t3 = t2 * t;

        return ((p1 * 2.0f) +
                (-p0 + p2) * t +
                (p0 * 2.0f - p1 * 5.0f + p2 * 4.0f - p3) * t2 +
                (-p0 + p1 * 3.0f - p2 * 3.0f + p3) * t3) * 0.5f;
    }

    /**
     * @brief Hermite spline interpolation
     * Uses positions and tangents for smooth curves
     */
    template<typename T>
    [[nodiscard]] T hermiteSpline(T p0, T p1, T t0, T t1, const Float t) noexcept {
        const Float t2 = t * t;
        const Float t3 = t2 * t;

        Float h1 = 2.0f * t3 - 3.0f * t2 + 1.0f;
        Float h2 = -2.0f * t3 + 3.0f * t2;
        Float h3 = t3 - 2.0f * t2 + t;
        Float h4 = t3 - t2;

        return h1 * p0 + h2 * p1 + h3 * t0 + h4 * t1;
    }

    // ============================================================================
    // Bezier Curves
    // ============================================================================

    /**
     * @brief Quadratic Bezier curve (3 control points)
     */
    template<typename T>
    [[nodiscard]] T bezierQuadratic(T p0, T p1, T p2, const Float t) noexcept {
        const Float oneMinusT = 1.0f - t;
        Float oneMinusT2 = oneMinusT * oneMinusT;
        Float t2 = t * t;

        return p0 * oneMinusT2 +
               p1 * (2.0f * oneMinusT * t) +
               p2 * t2;
    }

    /**
     * @brief Cubic Bezier curve (4 control points)
     */
    template<typename T>
    [[nodiscard]] T bezierCubic(T p0, T p1, T p2, T p3, const Float t) noexcept {
        const Float oneMinusT = 1.0f - t;
        const Float oneMinusT2 = oneMinusT * oneMinusT;
        Float oneMinusT3 = oneMinusT2 * oneMinusT;
        const Float t2 = t * t;
        const Float t3 = t2 * t;

        return p0 * oneMinusT3 +
               p1 * (3.0f * oneMinusT2 * t) +
               p2 * (3.0f * oneMinusT * t2) +
               p3 * t3;
    }

    /**
     * @brief Generic Bezier curve using De Casteljau's algorithm
     * Handles any number of control points
     */
    template<typename T>
    [[nodiscard]] T bezier(const std::vector<T>& controlPoints, Float t) noexcept {
        if (controlPoints.empty()) return T{};
        if (controlPoints.size() == 1) return controlPoints[0];

        std::vector<T> points = controlPoints;

        // De Casteljau's algorithm
        for (std::size_t k = 1; k < controlPoints.size(); ++k) {
            for (std::size_t i = 0; i < controlPoints.size() - k; ++i) {
                points[i] = lerp(points[i], points[i + 1], t);
            }
        }

        return points[0];
    }

    /**
     * @brief Calculate tangent on cubic Bezier curve
     */
    template<typename T>
    [[nodiscard]] T bezierCubicTangent(T p0, T p1, T p2, T p3, const Float t) noexcept {
        const Float oneMinusT = 1.0f - t;
        const Float oneMinusT2 = oneMinusT * oneMinusT;
        const Float t2 = t * t;

        return (p1 - p0) * (3.0f * oneMinusT2) +
               (p2 - p1) * (6.0f * oneMinusT * t) +
               (p3 - p2) * (3.0f * t2);
    }

    // ============================================================================
    // B-Spline Curves
    // ============================================================================

    /**
     * @brief Uniform B-spline basis function
     */
    [[nodiscard]] inline Float bsplineBasis(const int i, const int p, const Float t, const std::vector<Float>& knots) noexcept {
        if (p == 0) {
            return (t >= knots[i] && t < knots[i + 1]) ? 1.0f : 0.0f;
        }

        Float c1 = 0.0f, c2 = 0.0f;

        if (knots[i + p] != knots[i]) {
            c1 = (t - knots[i]) / (knots[i + p] - knots[i]) *
                 bsplineBasis(i, p - 1, t, knots);
        }

        if (knots[i + p + 1] != knots[i + 1]) {
            c2 = (knots[i + p + 1] - t) / (knots[i + p + 1] - knots[i + 1]) *
                 bsplineBasis(i + 1, p - 1, t, knots);
        }

        return c1 + c2;
    }

    /**
     * @brief Cubic B-spline curve
     */
    template<typename T>
    [[nodiscard]] T bsplineCubic(const std::vector<T>& controlPoints, const Float t) noexcept {
        if (controlPoints.size() < 4) return T{};

        // Generate uniform knot vector
        std::vector<Float> knots;
        for (std::size_t i = 0; i < controlPoints.size() + 4; ++i) {
            knots.push_back(static_cast<Float>(i) / (controlPoints.size() + 3));
        }

        T result{};
        for (std::size_t i = 0; i < controlPoints.size(); ++i) {
            Float basis = bsplineBasis(i, 3, t, knots);
            result = result + controlPoints[i] * basis;
        }

        return result;
    }

    // ============================================================================
    // Spline Path Class
    // ============================================================================

    /**
     * @brief Represents a parametric spline path for smooth motion
     */
    template<typename T>
    class SplinePath {
    public:
        enum class SplineType {
            LINEAR,
            CATMULL_ROM,
            CUBIC_BEZIER,
            B_SPLINE
        };

        explicit SplinePath(const SplineType type = SplineType::CATMULL_ROM)
            : type_(type), totalLength_(0), isClosed_(false) {}

        /**
         * @brief Add a control point to the path
         */
        void addPoint(const T& point) {
            points_.push_back(point);
            updateLength();
        }

        /**
         * @brief Set all control points
         */
        void setPoints(const std::vector<T>& points) {
            points_ = points;
            updateLength();
        }

        /**
         * @brief Get interpolated position at parameter t [0,1]
         */
        [[nodiscard]] T getPosition(Float t) const noexcept {
            if (points_.empty()) return T{};
            if (points_.size() == 1) return points_[0];

            t = saturate(t);

            switch (type_) {
                case SplineType::LINEAR:
                    return getLinearPosition(t);

                case SplineType::CATMULL_ROM:
                    return getCatmullRomPosition(t);

                case SplineType::CUBIC_BEZIER:
                    return getBezierPosition(t);

                case SplineType::B_SPLINE:
                    return getBSplinePosition(t);

                default:
                    return getLinearPosition(t);
            }
        }

        /**
         * @brief Get tangent at parameter t [0,1]
         */
        [[nodiscard]] T getTangent(const Float t) const noexcept {
            constexpr Float delta = 0.001f;
            T p1 = getPosition(t - delta);
            T p2 = getPosition(t + delta);

            return glm::normalize(p2 - p1);
        }

        /**
         * @brief Get position at arc length distance
         */
        [[nodiscard]] T getPositionAtDistance(const Float distance) const noexcept {
            if (totalLength_ <= 0) return points_.empty() ? T{} : points_[0];
            const Float t = distance / totalLength_;

            return getPosition(t);
        }

        /**
         * @brief Set whether the path loops back to start
         */
        void setClosed(const bool closed) {
            isClosed_ = closed;
            updateLength();
        }

        [[nodiscard]] Float getTotalLength() const noexcept { return totalLength_; }
        [[nodiscard]] std::size_t getPointCount() const noexcept { return points_.size(); }
        [[nodiscard]] bool isClosed() const noexcept { return isClosed_; }

    private:
        std::vector<T> points_;
        SplineType type_;
        Float totalLength_;
        bool isClosed_;

        /**
         * @brief Linear interpolation between points
         */
        [[nodiscard]] T getLinearPosition(const Float t) const noexcept {
            const std::size_t numSegments = isClosed_ ? points_.size() : points_.size() - 1;
            if (numSegments == 0) return points_[0];

            const Float segment = t * numSegments;
            std::size_t i = segment;
            Float localT = segment - i;

            if (i >= numSegments) {
                i = numSegments - 1;
                localT = 1.0f;
            }

            std::size_t next = (i + 1) % points_.size();

            return lerp(points_[i], points_[next], localT);
        }

        /**
         * @brief Catmull-Rom spline position
         */
        [[nodiscard]] T getCatmullRomPosition(const Float t) const noexcept {
            if (points_.size() < 2) return points_[0];

            const std::size_t numSegments = isClosed_ ? points_.size() : points_.size() - 1;
            const Float segment = t * numSegments;
            std::size_t i = segment;
            Float localT = segment - i;

            if (i >= numSegments) {
                i = numSegments - 1;
                localT = 1.0f;
            }

            // Get the 4 control points
            std::size_t i0 = (i == 0) ? (isClosed_ ? points_.size() - 1 : 0) : i - 1;
            std::size_t i1 = i;
            std::size_t i2 = (i + 1) % points_.size();
            std::size_t i3 = (i + 2) % points_.size();

            if (!isClosed_ && i == 0) i0 = 0;
            if (!isClosed_ && i >= points_.size() - 2) i3 = points_.size() - 1;

            return catmullRom(points_[i0], points_[i1], points_[i2], points_[i3], localT);
        }

        /**
         * @brief Cubic Bezier position (treats every 4 points as a curve)
         */
        [[nodiscard]] T getBezierPosition(const Float t) const noexcept {
            if (points_.size() < 4) return getLinearPosition(t);

            const std::size_t numCurves = points_.size() / 4;
            if (numCurves == 0) return points_[0];

            const Float curve = t * numCurves;
            std::size_t curveIndex = curve;
            Float localT = curve - curveIndex;

            if (curveIndex >= numCurves) {
                curveIndex = numCurves - 1;
                localT = 1.0f;
            }

            std::size_t base = curveIndex * 4;
            return bezierCubic(
                points_[base],
                points_[base + 1],
                points_[base + 2],
                points_[base + 3],
                localT
            );
        }

        /**
         * @brief B-Spline position
         */
        [[nodiscard]] T getBSplinePosition(Float t) const noexcept {
            return bsplineCubic(points_, t);
        }

        /**
         * @brief Update total arc length of the path
         */
        void updateLength() {
            totalLength_ = 0;
            if (points_.size() < 2) return;

            // Sample the spline to approximate arc length
            constexpr int samples = 100;
            T prevPos = getPosition(0);

            for (int i = 1; i <= samples; ++i) {
                Float t = static_cast<Float>(i) / samples;
                T currentPos = getPosition(t);
                totalLength_ += glm::length(currentPos - prevPos);
                prevPos = currentPos;
            }
        }
    };

    using SplinePath2D = SplinePath<Vec2>;
    using SplinePath3D = SplinePath<Vec3>;

} // namespace engine::math