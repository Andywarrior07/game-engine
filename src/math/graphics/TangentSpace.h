/**
 * @file TangentSpace.h
 * @brief Tangent space calculations for normal mapping
 * @author Andrés Guerrero
 * @date 28-08-2025
 *
 * Provides TBN (Tangent-Bitangent-Normal) matrix construction
 * and tangent space operations for advanced lighting.
 */

#pragma once

#include "../core/MathTypes.h"
#include "../core/MathConstants.h"
#include "../core/MathFunctions.h"

#include <vector>

// TODO: Porque este namespace y no solo engine::math?
namespace engine::math::graphics {
    /**
     * @brief Vertex data for tangent space calculation
     */
    struct Vertex {
        Vec3 position;
        Vec3 normal;
        Vec2 texCoord;
        Vec3 tangent;
        Vec3 bitangent;
        Float handedness; // +1 or -1 for mirrored UVs

        Vertex() : position(0), normal(0, 1, 0), texCoord(0),
                   tangent(1, 0, 0), bitangent(0, 0, 1), handedness(1.0f) {
        }
    };

    /**
     * @brief Triangle indices
     */
    struct Triangle {
        UInt indices[3];

        Triangle() : indices{0, 0, 0} {
        }

        Triangle(const UInt i0, const UInt i1, const UInt i2)
            : indices{i0, i1, i2} {
        }
    };

    /**
     * @brief Tangent space calculator for meshes
     */
    class TangentSpace {
    public:
        /**
         * @brief Calculate tangent and bitangent for a single triangle
         */
        static void calculateTriangleTangents(
            const Vec3& p0, const Vec3& p1, const Vec3& p2,
            const Vec2& uv0, const Vec2& uv1, const Vec2& uv2,
            Vec3& tangent, Vec3& bitangent) noexcept;

        /**
         * @brief Calculate tangent space for entire mesh
         */
        static void calculateMeshTangents(
            std::vector<Vertex>& vertices,
            const std::vector<Triangle>& triangles) noexcept;

        /**
         * @brief Calculate smooth tangents using MikkTSpace algorithm approach
         */
        static void calculateSmoothTangents(
            std::vector<Vertex>& vertices,
            const std::vector<Triangle>& triangles,
            Float angleThreshold = 80.0f) noexcept;

        /**
         * @brief Build TBN matrix from tangent space vectors
         */
        [[nodiscard]] static Mat3 buildTBNMatrix(
            const Vec3& tangent,
            const Vec3& bitangent,
            const Vec3& normal) noexcept {
            return Mat3(tangent, bitangent, normal);
        }

        /**
         * @brief Build TBN matrix with handedness
         */
        [[nodiscard]] static Mat3 buildTBNMatrix(const Vertex& vertex) noexcept;

        /**
         * @brief Transform vector from tangent space to world space
         */
        [[nodiscard]] static Vec3 tangentToWorld(
            const Vec3& tangentVector,
            const Mat3& tbnMatrix) noexcept {
            return tbnMatrix * tangentVector;
        }

        /**
         * @brief Transform vector from world space to tangent space
         */
        [[nodiscard]] static Vec3 worldToTangent(
            const Vec3& worldVector,
            const Mat3& tbnMatrix) noexcept {
            return glm::transpose(tbnMatrix) * worldVector;
        }

        /**
         * @brief Create TBN matrix in vertex shader (optimized)
         * Uses only tangent and normal, reconstructs bitangent
         */
        [[nodiscard]] static Mat3 buildTBNFromTN(
            const Vec3& tangent,
            const Vec3& normal,
            Float handedness) noexcept;

        /**
         * @brief Pack tangent and handedness into Vec4 for GPU
         */
        [[nodiscard]] static Vec4 packTangent(
            const Vec3& tangent,
            const Float handedness) noexcept {
            return Vec4(tangent, handedness);
        }

        /**
         * @brief Unpack tangent from Vec4
         */
        static void unpackTangent(
            const Vec4& packed,
            Vec3& tangent,
            Float& handedness) noexcept {
            tangent = Vec3(packed);
            handedness = packed.w;
        }

        /**
         * @brief Calculate tangent space for height map normal generation
         */
        [[nodiscard]] static Vec3 heightMapToNormal(
            Float heightCenter,
            Float heightLeft,
            Float heightRight,
            Float heightUp,
            Float heightDown,
            Float scale = 1.0f) noexcept;

        /**
         * @brief Blend two normal maps
         */
        [[nodiscard]] static Vec3 blendNormals(
            const Vec3& normal1,
            const Vec3& normal2,
            const Float weight = 0.5f) noexcept;

        /**
         * @brief Reoriented Normal Mapping (RNM) blend
         * Better quality than UDN blending
         */
        [[nodiscard]] static Vec3 blendNormalsRNM(
            const Vec3& normal1,
            const Vec3& normal2) noexcept;

    private:
        /**
         * @brief Orthogonalize tangent with respect to normal (Gram-Schmidt)
         */
        static void orthogonalizeTangent(Vertex& vertex) noexcept;
    };
} // namespace engine::math::graphics
