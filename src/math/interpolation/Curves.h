/**
 * @file Curves.h
 * @brief Comprehensive parametric curve implementations
 *
 * Provides Bezier, Catmull-Rom, B-Spline, and Hermite curves
 * with evaluation, derivatives, and arc length parameterization.
 */

#pragma once

#include "../core/MathTypes.h"
#include "../core/MathConstants.h"
#include "../core/MathFunctions.h"

#include <vector>

namespace engine::math::curves {

    /**
     * @brief Base class for parametric curves
     */
    template<typename T>
    class ParametricCurve {
    public:
        virtual ~ParametricCurve() = default;

        /**
         * @brief Evaluate curve at parameter t [0,1]
         */
        [[nodiscard]] virtual T evaluate(Float t) const noexcept = 0;

        /**
         * @brief Get first derivative (tangent) at t
         */
        [[nodiscard]] virtual T derivative(Float t) const noexcept = 0;

        /**
         * @brief Get second derivative (acceleration) at t
         */
        [[nodiscard]] virtual T secondDerivative(Float t) const noexcept = 0;

        /**
         * @brief Approximate arc length of curve
         */
        [[nodiscard]] virtual Float arcLength(const int samples) const noexcept {
            Float length = 0;
            T prevPoint = evaluate(0);

            for (int i = 1; i <= samples; ++i) {
                Float t = static_cast<Float>(i) / static_cast<Float>(samples);
                T currentPoint = evaluate(t);
                length += glm::length(currentPoint - prevPoint);
                prevPoint = currentPoint;
            }

            return length;
        }
    };

    // ============================================================================
    // Bezier Curves
    // ============================================================================

    /**
     * @brief Quadratic Bezier curve (3 control points)
     */
    template<typename T>
    class QuadraticBezier final : public ParametricCurve<T> {
    public:
        QuadraticBezier(const T& p0, const T& p1, const T& p2)
            : p0_(p0), p1_(p1), p2_(p2) {}

        [[nodiscard]] T evaluate(const Float t) const noexcept override {
            const Float oneMinusT = 1.0f - t;
            Float oneMinusT2 = oneMinusT * oneMinusT;
            Float t2 = t * t;

            return p0_ * oneMinusT2 +
                   p1_ * (2.0f * oneMinusT * t) +
                   p2_ * t2;
        }

        [[nodiscard]] T derivative(const Float t) const noexcept override {
            const Float oneMinusT = 1.0f - t;
            return (p1_ - p0_) * (2.0f * oneMinusT) +
                   (p2_ - p1_) * (2.0f * t);
        }

        [[nodiscard]] T secondDerivative(const Float t) const noexcept override {
            // Para curvas Bézier cuadráticas, la segunda derivada es constante
            // El parámetro t no se utiliza pero se mantiene por compatibilidad con la interfaz

            (void)t; // Evita warning de parámetro no utilizado

            return (p2_ - p1_ * 2.0f + p0_) * 2.0f;
        }

        /**
         * @brief Split curve at parameter t into two curves
         */
        void split(Float t, QuadraticBezier& left, QuadraticBezier& right) const noexcept {
            T q0 = lerp(p0_, p1_, t);
            T q1 = lerp(p1_, p2_, t);
            T r = lerp(q0, q1, t);

            left = QuadraticBezier(p0_, q0, r);
            right = QuadraticBezier(r, q1, p2_);
        }

    private:
        T p0_, p1_, p2_;
    };

    /**
     * @brief Cubic Bezier curve (4 control points)
     */
    template<typename T>
    class CubicBezier final : public ParametricCurve<T> {
    public:
        CubicBezier(const T& p0, const T& p1, const T& p2, const T& p3)
            : p0_(p0), p1_(p1), p2_(p2), p3_(p3) {
            computeCoefficients();
        }

        [[nodiscard]] T evaluate(Float t) const noexcept override {
            Float t2 = t * t;
            Float t3 = t2 * t;
            return a_ * t3 + b_ * t2 + c_ * t + d_;
        }

        [[nodiscard]] T derivative(const Float t) const noexcept override {
            const Float t2 = t * t;
            return a_ * (3.0f * t2) + b_ * (2.0f * t) + c_;
        }

        [[nodiscard]] T secondDerivative(const Float t) const noexcept override {
            return a_ * (6.0f * t) + b_ * 2.0f;
        }

        /**
         * @brief Get curvature at parameter t
         */
        [[nodiscard]] Float curvature(const Float t) const noexcept {
            T d1 = derivative(t);
            T d2 = secondDerivative(t);
            const Float speed = glm::length(d1);
            if (speed < EPSILON) return 0;

            return glm::length(glm::cross(Vec3(d1), Vec3(d2))) / (speed * speed * speed);
        }

        /**
         * @brief Split curve at parameter t
         */
        void split(Float t, CubicBezier& left, CubicBezier& right) const noexcept {
            // De Casteljau's algorithm
            T q0 = lerp(p0_, p1_, t);
            T q1 = lerp(p1_, p2_, t);
            T q2 = lerp(p2_, p3_, t);

            T r0 = lerp(q0, q1, t);
            T r1 = lerp(q1, q2, t);

            T s = lerp(r0, r1, t);

            left = CubicBezier(p0_, q0, r0, s);
            right = CubicBezier(s, r1, q2, p3_);
        }

        /**
         * @brief Find closest point on curve to given point (approximate)
         */
        [[nodiscard]] Float findClosestParameter(const T& point, const int iterations) const noexcept {
            // Newton-Raphson iteration
            Float t = 0.5f;

            for (int i = 0; i < iterations; ++i) {
                T curvePoint = evaluate(t);
                T tangent = derivative(t);
                const Float f = glm::dot(curvePoint - point, tangent);
                const Float df = glm::dot(tangent, tangent) + glm::dot(curvePoint - point, secondDerivative(t));

                if (std::abs(df) < EPSILON) break;
                t = saturate(t - f / df);
            }

            return t;
        }

    private:
        T p0_, p1_, p2_, p3_;
        T a_, b_, c_, d_; // Polynomial coefficients

        void computeCoefficients() {
            // Convert from Bernstein to polynomial form for efficiency
            a_ = -p0_ + p1_ * 3.0f - p2_ * 3.0f + p3_;
            b_ = p0_ * 3.0f - p1_ * 6.0f + p2_ * 3.0f;
            c_ = -p0_ * 3.0f + p1_ * 3.0f;
            d_ = p0_;
        }
    };

    /**
     * @brief Generic Bezier curve with N control points
     */
    template<typename T>
    class BezierCurve final : public ParametricCurve<T> {
    public:
        explicit BezierCurve(const std::vector<T>& controlPoints)
            : controlPoints_(controlPoints) {}

        [[nodiscard]] T evaluate(const Float t) const noexcept override {
            return deCasteljau(controlPoints_, t);
        }

        [[nodiscard]] T derivative(const Float t) const noexcept override {
            if (controlPoints_.size() < 2) return T{};

            std::vector<T> derivativePoints;
            for (std::size_t i = 0; i < controlPoints_.size() - 1; ++i) {
                derivativePoints.push_back(
                    (controlPoints_[i + 1] - controlPoints_[i]) * static_cast<Float>(controlPoints_.size() - 1)
                );
            }

            return deCasteljau(derivativePoints, t);
        }

        [[nodiscard]] T secondDerivative(const Float t) const noexcept override {
            if (controlPoints_.size() < 3) return T{};

            std::vector<T> firstDeriv, secondDeriv;

            // First derivative control points
            for (std::size_t i = 0; i < controlPoints_.size() - 1; ++i) {
                firstDeriv.push_back(
                    (controlPoints_[i + 1] - controlPoints_[i]) * static_cast<Float>(controlPoints_.size() - 1)
                );
            }

            // Second derivative control points
            for (std::size_t i = 0; i < firstDeriv.size() - 1; ++i) {
                secondDeriv.push_back(
                    (firstDeriv[i + 1] - firstDeriv[i]) * static_cast<Float>(firstDeriv.size() - 1)
                );
            }

            return deCasteljau(secondDeriv, t);
        }

        /**
         * @brief Elevate degree by adding a control point
         */
        void elevateDegree() {
            std::vector<T> newPoints;
            newPoints.reserve(controlPoints_.size() + 1);

            newPoints.push_back(controlPoints_[0]);

            for (std::size_t i = 1; i < controlPoints_.size(); ++i) {
                Float alpha = static_cast<Float>(i) / (controlPoints_.size());
                T point = controlPoints_[i - 1] * alpha + controlPoints_[i] * (1.0f - alpha);
                newPoints.push_back(point);
            }

            newPoints.push_back(controlPoints_.back());
            controlPoints_ = std::move(newPoints);
        }

    private:
        std::vector<T> controlPoints_;

        /**
         * @brief De Casteljau's algorithm for curve evaluation
         */
        [[nodiscard]] static T deCasteljau(const std::vector<T>& points, Float t) noexcept {
            if (points.empty()) return T{};
            if (points.size() == 1) return points[0];

            std::vector<T> temp = points;

            for (std::size_t k = 1; k < points.size(); ++k) {
                for (std::size_t i = 0; i < points.size() - k; ++i) {
                    temp[i] = lerp(temp[i], temp[i + 1], t);
                }
            }

            return temp[0];
        }
    };

    // ============================================================================
    // Catmull-Rom Splines
    // ============================================================================

    /**
     * @brief Catmull-Rom spline segment (4 control points)
     * Passes through p1 and p2, uses p0 and p3 for tangent calculation
     */
    template<typename T>
    class CatmullRomSegment final : public ParametricCurve<T> {
    public:
        CatmullRomSegment(const T& p0, const T& p1, const T& p2, const T& p3, const Float tension = 0.5f)
            : p0_(p0), p1_(p1), p2_(p2), p3_(p3), tension_(tension) {
            computeCoefficients();
        }

        [[nodiscard]] T evaluate(Float t) const noexcept override {
            Float t2 = t * t;
            Float t3 = t2 * t;
            return a_ * t3 + b_ * t2 + c_ * t + d_;
        }

        [[nodiscard]] T derivative(const Float t) const noexcept override {
            const Float t2 = t * t;
            return a_ * (3.0f * t2) + b_ * (2.0f * t) + c_;
        }

        [[nodiscard]] T secondDerivative(const Float t) const noexcept override {
            return a_ * (6.0f * t) + b_ * 2.0f;
        }

    private:
        T p0_, p1_, p2_, p3_;
        Float tension_;
        T a_, b_, c_, d_;

        void computeCoefficients() {
            // Catmull-Rom to cubic coefficients
            T v0 = (p2_ - p0_) * tension_;
            T v1 = (p3_ - p1_) * tension_;

            a_ = p1_ * 2.0f - p2_ * 2.0f + v0 + v1;
            b_ = -p1_ * 3.0f + p2_ * 3.0f - v0 * 2.0f - v1;
            c_ = v0;
            d_ = p1_;
        }
    };

    /**
     * @brief Complete Catmull-Rom spline through multiple points
     */
    template<typename T>
    class CatmullRomSpline {
    public:
        explicit CatmullRomSpline(const std::vector<T>& points, const Float tension = 0.5f, const bool closed = false)
            : points_(points), tension_(tension), closed_(closed) {}

        [[nodiscard]] T evaluate(const Float t) const noexcept {
            if (points_.size() < 2) return points_.empty() ? T{} : points_[0];

            const std::size_t numSegments = closed_ ? points_.size() : points_.size() - 1;
            const Float segment = t * numSegments;
            auto index = static_cast<std::size_t>(segment);
            Float localT = segment - static_cast<Float>(index);

            if (index >= numSegments) {
                index = numSegments - 1;
                localT = 1.0f;
            }

            return evaluateSegment(index, localT);
        }

        [[nodiscard]] T derivative(const Float t) const noexcept {
            if (points_.size() < 2) return T{};

            const std::size_t numSegments = closed_ ? points_.size() : points_.size() - 1;
            const Float segment = t * numSegments;
            auto index = static_cast<std::size_t>(segment);
            Float localT = segment - static_cast<Float>(index);

            if (index >= numSegments) {
                index = numSegments - 1;
                localT = 1.0f;
            }

            auto seg = getSegment(index);
            return seg.derivative(localT) * static_cast<Float>(numSegments);
        }

    private:
        std::vector<T> points_;
        Float tension_;
        bool closed_;

        [[nodiscard]] CatmullRomSegment<T> getSegment(const std::size_t index) const noexcept {
            const std::size_t n = points_.size();

            std::size_t i0 = (index == 0) ? (closed_ ? n - 1 : 0) : index - 1;
            std::size_t i1 = index;
            std::size_t i2 = (index + 1) % n;
            std::size_t i3 = (index + 2) % n;

            if (!closed_) {
                if (index == 0) i0 = 0;
                if (index >= n - 2) i3 = n - 1;
            }

            return CatmullRomSegment<T>(points_[i0], points_[i1], points_[i2], points_[i3], tension_);
        }

        [[nodiscard]] T evaluateSegment(const std::size_t index, Float t) const noexcept {
            return getSegment(index).evaluate(t);
        }
    };

    // ============================================================================
    // B-Splines
    // ============================================================================

    /**
     * @brief Uniform B-Spline curve
     */
    template<typename T>
    class UniformBSpline final : public ParametricCurve<T> {
    public:
        explicit UniformBSpline(const std::vector<T>& controlPoints, const int degree = 3)
            : controlPoints_(controlPoints), degree_(degree) {
            generateKnots();
        }

        [[nodiscard]] T evaluate(Float t) const noexcept override {
            if (controlPoints_.empty()) return T{};

            t = saturate(t);
            const Float u = t * (knots_.size() - 2 * degree_ - 1) + degree_;

            T result{};
            for (std::size_t i = 0; i < controlPoints_.size(); ++i) {
                Float basis = basisFunction(i, degree_, u);
                result = result + controlPoints_[i] * basis;
            }

            return result;
        }

        [[nodiscard]] T derivative(Float t) const noexcept override {
            if (controlPoints_.size() < 2) return T{};

            t = saturate(t);
            const Float u = t * (knots_.size() - 2 * degree_ - 1) + degree_;

            T result{};
            for (std::size_t i = 0; i < controlPoints_.size() - 1; ++i) {
                Float basis = basisFunction(i, degree_ - 1, u);
                T diff = (controlPoints_[i + 1] - controlPoints_[i]) * static_cast<Float>(degree_);
                if (Float denom = knots_[i + degree_] - knots_[i]; denom > EPSILON) {
                    result = result + (diff / denom) * basis;
                }
            }

            return result;
        }

        [[nodiscard]] T secondDerivative(const Float t) const noexcept override {
            // Simplified - would need recursive implementation for accuracy
            (void)t;
            return T{};
        }

    private:
        std::vector<T> controlPoints_;
        std::vector<Float> knots_;
        int degree_;

        void generateKnots() {
            const std::size_t n = controlPoints_.size();
            const std::size_t numKnots = n + degree_ + 1;
            knots_.resize(numKnots);

            // Uniform knot vector
            for (std::size_t i = 0; i < numKnots; ++i) {
                knots_[i] = static_cast<Float>(i);
            }
        }

        /**
         * @brief Cox-de Boor recursion for B-spline basis functions
         */
        [[nodiscard]] Float basisFunction(const std::size_t i, const int p, const Float u) const noexcept {
            if (p == 0) {
                return (u >= knots_[i] && u < knots_[i + 1]) ? 1.0f : 0.0f;
            }

            Float c1 = 0, c2 = 0;

            if (const Float denom1 = knots_[i + p] - knots_[i]; denom1 > EPSILON) {
                c1 = (u - knots_[i]) / denom1 * basisFunction(i, p - 1, u);
            }

            if (const Float denom2 = knots_[i + p + 1] - knots_[i + 1]; denom2 > EPSILON) {
                c2 = (knots_[i + p + 1] - u) / denom2 * basisFunction(i + 1, p - 1, u);
            }

            return c1 + c2;
        }
    };

    using Vec2Curve = CubicBezier<Vec2>;
    using Vec3Curve = CubicBezier<Vec3>;
    using CatmullRom2D = CatmullRomSpline<Vec2>;
    using CatmullRom3D = CatmullRomSpline<Vec3>;

} // namespace engine::math::curves