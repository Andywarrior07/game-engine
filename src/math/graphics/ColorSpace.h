/**
 * @file ColorSpace.h
 * @brief Color space conversion utilities
 * @author Andrés Guerrero
 * @date 28-08-2025
 * Provides conversions between RGB, HSV, HSL, XYZ, LAB color spaces
 * and various color manipulation utilities for graphics programming.
 */

#pragma once

#include "../core/MathTypes.h"
#include "../core/MathConstants.h"
#include "../core/MathFunctions.h"

// TODO: Preguntar porque este namespace
namespace engine::math::color {
    // ============================================================================
    // RGB <-> HSV Conversions
    // ============================================================================

    /**
     * @brief Convert RGB to HSV color space
     * @param rgb RGB values [0,1]
     * @return HSV values - H[0,360], S[0,1], V[0,1]
     */
    [[nodiscard]] inline Vec3 rgbToHSV(const Vec3& rgb) noexcept {
        const Float r = rgb.r;
        const Float g = rgb.g;
        const Float b = rgb.b;

        const Float maxVal = max3(r, g, b);
        const Float minVal = min3(r, g, b);
        const Float delta = maxVal - minVal;

        // Value
        const Float v = maxVal;

        // Saturation
        Float s = 0;
        if (maxVal > EPSILON) {
            s = delta / maxVal;
        }

        // Hue
        Float h = 0;
        if (delta > EPSILON) {
            if (isNearlyEqual(maxVal, r)) {
                h = 60.0f * fmod((g - b) / delta, 6.0f);
            }
            else if (isNearlyEqual(maxVal, g)) {
                h = 60.0f * ((b - r) / delta + 2.0f);
            }
            else {
                h = 60.0f * ((r - g) / delta + 4.0f);
            }

            if (h < 0) h += 360.0f;
        }

        return Vec3(h, s, v);
    }

    /**
     * @brief Convert HSV to RGB color space
     * @param hsv HSV values - H[0,360], S[0,1], V[0,1]
     * @return RGB values [0,1]
     */
    [[nodiscard]] inline Vec3 hsvToRGB(const Vec3& hsv) noexcept {
        const Float h = hsv.x;
        const Float s = saturate(hsv.y);
        const Float v = saturate(hsv.z);

        const Float c = v * s;
        const Float x = c * (1.0f - std::abs(fmod(h / 60.0f, 2.0f) - 1.0f));
        const Float m = v - c;

        Vec3 rgb;
        if (h < 60) {
            rgb = Vec3(c, x, 0);
        }
        else if (h < 120) {
            rgb = Vec3(x, c, 0);
        }
        else if (h < 180) {
            rgb = Vec3(0, c, x);
        }
        else if (h < 240) {
            rgb = Vec3(0, x, c);
        }
        else if (h < 300) {
            rgb = Vec3(x, 0, c);
        }
        else {
            rgb = Vec3(c, 0, x);
        }

        return rgb + Vec3(m);
    }

    // ============================================================================
    // RGB <-> HSL Conversions
    // ============================================================================

    /**
     * @brief Convert RGB to HSL color space
     * @param rgb RGB values [0,1]
     * @return HSL values - H[0,360], S[0,1], L[0,1]
     */
    [[nodiscard]] inline Vec3 rgbToHSL(const Vec3& rgb) noexcept {
        const Float r = rgb.r;
        const Float g = rgb.g;
        const Float b = rgb.b;

        const Float maxVal = max3(r, g, b);
        const Float minVal = min3(r, g, b);
        const Float delta = maxVal - minVal;

        // Lightness
        const Float l = (maxVal + minVal) * 0.5f;

        // Saturation
        Float s = 0;
        if (delta > EPSILON) {
            if (l < 0.5f) {
                s = delta / (maxVal + minVal);
            }
            else {
                s = delta / (2.0f - maxVal - minVal);
            }
        }

        // Hue (same as HSV)
        Float h = 0;
        if (delta > EPSILON) {
            if (isNearlyEqual(maxVal, r)) {
                h = 60.0f * fmod((g - b) / delta, 6.0f);
            }
            else if (isNearlyEqual(maxVal, g)) {
                h = 60.0f * ((b - r) / delta + 2.0f);
            }
            else {
                h = 60.0f * ((r - g) / delta + 4.0f);
            }

            if (h < 0) h += 360.0f;
        }

        return Vec3(h, s, l);
    }

    /**
     * @brief Convert HSL to RGB color space
     * @param hsl HSL values - H[0,360], S[0,1], L[0,1]
     * @return RGB values [0,1]
     */
    [[nodiscard]] inline Vec3 hslToRGB(const Vec3& hsl) noexcept {
        const Float h = hsl.x;
        const Float s = saturate(hsl.y);
        const Float l = saturate(hsl.z);

        const Float c = (1.0f - std::abs(2.0f * l - 1.0f)) * s;
        const Float x = c * (1.0f - std::abs(fmod(h / 60.0f, 2.0f) - 1.0f));
        const Float m = l - c * 0.5f;

        Vec3 rgb;
        if (h < 60) {
            rgb = Vec3(c, x, 0);
        }
        else if (h < 120) {
            rgb = Vec3(x, c, 0);
        }
        else if (h < 180) {
            rgb = Vec3(0, c, x);
        }
        else if (h < 240) {
            rgb = Vec3(0, x, c);
        }
        else if (h < 300) {
            rgb = Vec3(x, 0, c);
        }
        else {
            rgb = Vec3(c, 0, x);
        }

        return rgb + Vec3(m);
    }

    // ============================================================================
    // RGB <-> XYZ Conversions (CIE 1931)
    // ============================================================================

    /**
     * @brief Convert linear RGB to XYZ color space
     * Assumes sRGB primaries and D65 illuminant
     */
    [[nodiscard]] inline Vec3 linearRGBToXYZ(const Vec3& rgb) noexcept {
        // sRGB to XYZ matrix (D65 illuminant)
        const Mat3 rgbToXYZ(
            0.4124564f, 0.3575761f, 0.1804375f,
            0.2126729f, 0.7151522f, 0.0721750f,
            0.0193339f, 0.1191920f, 0.9503041f
        );

        return rgbToXYZ * rgb;
    }

    /**
     * @brief Convert XYZ to linear RGB color space
     */
    [[nodiscard]] inline Vec3 xyzToLinearRGB(const Vec3& xyz) noexcept {
        // XYZ to sRGB matrix (D65 illuminant)
        const Mat3 xyzToRGB(
            3.2404542f, -1.5371385f, -0.4985314f,
            -0.9692660f, 1.8760108f, 0.0415560f,
            0.0556434f, -0.2040259f, 1.0572252f
        );

        return xyzToRGB * xyz;
    }

    // ============================================================================
    // RGB <-> LAB Conversions (CIELAB)
    // ============================================================================

    /**
     * @brief Convert XYZ to LAB color space
     * L*[0,100], a*[-128,127], b*[-128,127]
     * @return CIE LAB color as (L*, a*, b*)
     */
    [[nodiscard]] inline Vec3 xyzToLAB(const Vec3& xyz) noexcept {
        // D65 illuminant
        const Vec3 whitePoint(0.95047f, 1.00000f, 1.08883f);

        const Vec3 normalized = xyz / whitePoint;

        auto f = [](Float const t) -> Float {
            constexpr Float delta = 6.0f / 29.0f;
            constexpr Float delta2 = delta * delta;

            if (constexpr Float delta3 = delta2 * delta; t > delta3) {
                return std::pow(t, 1.0f / 3.0f);
            }

            return t / (3.0f * delta2) + 4.0f / 29.0f;
        };

        const Float fx = f(normalized.x);
        const Float fy = f(normalized.y);
        const Float fz = f(normalized.z);

        const Float L = 116.0f * fy - 16.0f;
        const Float a = 500.0f * (fx - fy);
        const Float b = 200.0f * (fy - fz);

        return Vec3(L, a, b);
    }

    /**
     * @brief Convert LAB to XYZ color space
     */
    [[nodiscard]] inline Vec3 labToXYZ(const Vec3& lab) noexcept {
        const Float L = lab.x;
        const Float a = lab.y;
        const Float b = lab.z;

        const Float fy = (L + 16.0f) / 116.0f;
        const Float fx = a / 500.0f + fy;
        const Float fz = fy - b / 200.0f;

        auto finv = [](const Float t) -> Float {
            constexpr Float delta = 6.0f / 29.0f;
            constexpr Float delta2 = delta * delta;

            if (t > delta) {
                return t * t * t;
            }

            return 3.0f * delta2 * (t - 4.0f / 29.0f);
        };

        const Vec3 whitePoint(0.95047f, 1.00000f, 1.08883f);

        return Vec3(
            finv(fx) * whitePoint.x,
            finv(fy) * whitePoint.y,
            finv(fz) * whitePoint.z
        );
    }

    // ============================================================================
    // Gamma Correction
    // ============================================================================

    /**
     * @brief Convert linear RGB to sRGB (apply gamma)
     */
    [[nodiscard]] inline Vec3 linearToSRGB(const Vec3& linear) noexcept {
        auto toSRGB = [](const Float channel) -> Float {
            if (channel <= 0.0031308f) {
                return 12.92f * channel;
            }
            else {
                return 1.055f * std::pow(channel, 1.0f / 2.4f) - 0.055f;
            }
        };

        return Vec3(toSRGB(linear.r), toSRGB(linear.g), toSRGB(linear.b));
    }

    /**
     * @brief Convert sRGB to linear RGB (remove gamma)
     */
    [[nodiscard]] inline Vec3 sRGBToLinear(const Vec3& srgb) noexcept {
        auto toLinear = [](const Float channel) -> Float {
            if (channel <= 0.04045f) {
                return channel / 12.92f;
            }

            return std::pow((channel + 0.055f) / 1.055f, 2.4f);
        };

        return Vec3(toLinear(srgb.r), toLinear(srgb.g), toLinear(srgb.b));
    }

    /**
     * @brief Apply custom gamma correction
     */
    [[nodiscard]] inline Vec3 applyGamma(const Vec3& linear, const Float gamma) noexcept {
        const Float invGamma = 1.0f / gamma;

        return Vec3(
            std::pow(linear.r, invGamma),
            std::pow(linear.g, invGamma),
            std::pow(linear.b, invGamma)
        );
    }

    // ============================================================================
    // Color Temperature
    // ============================================================================

    /**
     * @brief Convert color temperature (Kelvin) to RGB
     * @param kelvin Temperature in Kelvin (1000-40000)
     * @return RGB color normalized to [0,1]
     */
    [[nodiscard]] inline Vec3 kelvinToRGB(Float kelvin) noexcept {
        kelvin = clamp(kelvin, 1000.0f, 40000.0f);
        const Float temp = kelvin / 100.0f;

        Float r, g, b;

        // Red
        if (temp <= 66) {
            r = 255;
        }
        else {
            r = temp - 60;
            r = 329.698727446f * std::pow(r, -0.1332047592f);
        }

        // Green
        if (temp <= 66) {
            g = temp;
            g = 99.4708025861f * std::log(g) - 161.1195681661f;
        }
        else {
            g = temp - 60;
            g = 288.1221695283f * std::pow(g, -0.0755148492f);
        }

        // Blue
        if (temp >= 66) {
            b = 255;
        }
        else if (temp <= 19) {
            b = 0;
        }
        else {
            b = temp - 10;
            b = 138.5177312231f * std::log(b) - 305.0447927307f;
        }

        return Vec3(
            saturate(r / 255.0f),
            saturate(g / 255.0f),
            saturate(b / 255.0f)
        );
    }

    // ============================================================================
    // Color Manipulation
    // ============================================================================

    /**
     * @brief Adjust hue of RGB color
     */
    [[nodiscard]] inline Vec3 adjustHue(const Vec3& rgb, const Float hueDelta) noexcept {
        Vec3 hsv = rgbToHSV(rgb);
        hsv.x = fmod(hsv.x + hueDelta, 360.0f);
        if (hsv.x < 0) hsv.x += 360.0f;

        return hsvToRGB(hsv);
    }

    /**
     * @brief Adjust saturation of RGB color
     */
    [[nodiscard]] inline Vec3 adjustSaturation(const Vec3& rgb, const Float factor) noexcept {
        Vec3 hsv = rgbToHSV(rgb);
        hsv.y = saturate(hsv.y * factor);

        return hsvToRGB(hsv);
    }

    /**
     * @brief Adjust brightness of RGB color
     */
    [[nodiscard]] inline Vec3 adjustBrightness(const Vec3& rgb, const Float factor) noexcept {
        return rgb * factor;
    }

    /**
     * @brief Adjust contrast of RGB color
     */
    [[nodiscard]] inline Vec3 adjustContrast(const Vec3& rgb, const Float factor) noexcept {
        const Vec3 gray(0.5f);
        return gray + (rgb - gray) * factor;
    }

    /**
     * @brief Convert to grayscale using luminance weights
     */
    [[nodiscard]] inline Float toGrayscale(const Vec3& rgb) noexcept {
        // ITU-R BT.709 luminance weights
        const Vec3 weights(0.2126f, 0.7152f, 0.0722f);
        return glm::dot(rgb, weights);
    }

    /**
     * @brief Color difference using Delta E CIE 2000
     * Returns perceptual difference between two colors
     */
    [[nodiscard]] inline Float deltaE2000(const Vec3& lab1, const Vec3& lab2) noexcept {
        // CIE 2000 formula (simplified)
        const Float L1 = lab1.x, a1 = lab1.y, b1 = lab1.z;
        const Float L2 = lab2.x, a2 = lab2.y, b2 = lab2.z;

        const Float dL = L2 - L1;
        const Float da = a2 - a1;
        const Float db = b2 - b1;

        const Float C1 = std::sqrt(a1 * a1 + b1 * b1);
        const Float C2 = std::sqrt(a2 * a2 + b2 * b2);
        const Float dC = C2 - C1;

        const Float dH2 = da * da + db * db - dC * dC;
        const Float dH = dH2 > 0 ? std::sqrt(dH2) : 0;

        // Weighting factors (simplified)
        constexpr Float kL = 1.0f, kC = 1.0f, kH = 1.0f;

        constexpr Float SL = 1.0f;
        const Float SC = 1.0f + 0.045f * C1;
        const Float SH = 1.0f + 0.015f * C1;

        const Float dLp = dL / (kL * SL);
        const Float dCp = dC / (kC * SC);
        const Float dHp = dH / (kH * SH);

        return std::sqrt(dLp * dLp + dCp * dCp + dHp * dHp);
    }

    /**
     * @brief Mix two colors in LAB space for perceptually uniform blending
     */
    [[nodiscard]] inline Vec3 mixColorsLAB(const Vec3& rgb1, const Vec3& rgb2, Float t) noexcept {
        Vec3 linear1 = sRGBToLinear(rgb1);
        Vec3 linear2 = sRGBToLinear(rgb2);

        Vec3 xyz1 = linearRGBToXYZ(linear1);
        Vec3 xyz2 = linearRGBToXYZ(linear2);

        Vec3 lab1 = xyzToLAB(xyz1);
        Vec3 lab2 = xyzToLAB(xyz2);

        Vec3 labMixed = glm::mix(lab1, lab2, t);

        Vec3 xyzMixed = labToXYZ(labMixed);
        Vec3 linearMixed = xyzToLinearRGB(xyzMixed);

        return linearToSRGB(linearMixed);
    }
} // namespace engine::math::color
