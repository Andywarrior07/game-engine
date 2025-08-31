/**
 * @file TangentSpace.cpp
 * @brief Tangent space calculations for normal mapping
 * @author Andrés Guerrero
 * @date 28-08-2025
 *
 * Provides TBN (Tangent-Bitangent-Normal) matrix construction
 * and tangent space operations for advanced lighting.
 */

#include "TangentSpace.h"

namespace engine::math::graphics {
    void TangentSpace::calculateTriangleTangents(
        const Vec3& p0, const Vec3& p1, const Vec3& p2,
        const Vec2& uv0, const Vec2& uv1, const Vec2& uv2,
        Vec3& tangent, Vec3& bitangent) noexcept {
        // Edge vectors
        const Vec3 edge1 = p1 - p0;
        const Vec3 edge2 = p2 - p0;

        // UV deltas
        const Vec2 deltaUV1 = uv1 - uv0;
        const Vec2 deltaUV2 = uv2 - uv0;

        // Calculate tangent and bitangent
        const Float denominator = deltaUV1.x * deltaUV2.y - deltaUV2.x * deltaUV1.y;

        if (std::abs(denominator) < EPSILON) {
            // Degenerate UV coordinates, use arbitrary tangent
            buildOrthonormalBasis(glm::normalize(edge1), tangent, bitangent);
            return;
        }

        const Float invDenom = 1.0f / denominator;

        tangent = (edge1 * deltaUV2.y - edge2 * deltaUV1.y) * invDenom;
        bitangent = (edge2 * deltaUV1.x - edge1 * deltaUV2.x) * invDenom;

        tangent = glm::normalize(tangent);
        bitangent = glm::normalize(bitangent);
    }

    void TangentSpace::calculateMeshTangents(
        std::vector<Vertex>& vertices,
        const std::vector<Triangle>& triangles) noexcept {
        // Initialize tangent and bitangent to zero
        for (auto& vertex : vertices) {
            vertex.tangent = Vec3(0);
            vertex.bitangent = Vec3(0);
        }

        // Accumulate tangents and bitangents for each triangle
        for (const auto& triangle : triangles) {
            const UInt i0 = triangle.indices[0];
            const UInt i1 = triangle.indices[1];
            const UInt i2 = triangle.indices[2];

            const Vertex& v0 = vertices[i0];
            const Vertex& v1 = vertices[i1];
            const Vertex& v2 = vertices[i2];

            Vec3 tangent, bitangent;
            calculateTriangleTangents(
                v0.position, v1.position, v2.position,
                v0.texCoord, v1.texCoord, v2.texCoord,
                tangent, bitangent
            );

            // Add to vertices (will be averaged)
            vertices[i0].tangent += tangent;
            vertices[i0].bitangent += bitangent;
            vertices[i1].tangent += tangent;
            vertices[i1].bitangent += bitangent;
            vertices[i2].tangent += tangent;
            vertices[i2].bitangent += bitangent;
        }

        // Orthogonalize and normalize
        for (auto& vertex : vertices) {
            orthogonalizeTangent(vertex);
        }
    }

    void TangentSpace::calculateSmoothTangents(
        std::vector<Vertex>& vertices,
        const std::vector<Triangle>& triangles,
        Float angleThreshold) noexcept {
        angleThreshold = toRadians(angleThreshold);

        // First pass: calculate per-face tangents
        struct FaceTangent {
            Vec3 tangent;
            Vec3 bitangent;
            Vec3 normal;
        };

        std::vector<FaceTangent> faceTangents;
        faceTangents.reserve(triangles.size());

        for (const auto& triangle : triangles) {
            const Vertex& v0 = vertices[triangle.indices[0]];
            const Vertex& v1 = vertices[triangle.indices[1]];
            const Vertex& v2 = vertices[triangle.indices[2]];

            FaceTangent face;
            calculateTriangleTangents(
                v0.position, v1.position, v2.position,
                v0.texCoord, v1.texCoord, v2.texCoord,
                face.tangent, face.bitangent
            );

            // Calculate face normal
            Vec3 edge1 = v1.position - v0.position;
            Vec3 edge2 = v2.position - v0.position;
            face.normal = glm::normalize(glm::cross(edge1, edge2));

            faceTangents.push_back(face);
        }

        // Second pass: average tangents for smooth groups
        for (auto& vertex : vertices) {
            vertex.tangent = Vec3(0);
            vertex.bitangent = Vec3(0);
        }

        for (std::size_t i = 0; i < triangles.size(); ++i) {
            const auto& triangle = triangles[i];
            const auto& face = faceTangents[i];

            for (int j = 0; j < 3; ++j) {
                UInt vertexIndex = triangle.indices[j];
                Vertex& vertex = vertices[vertexIndex];

                // Check angle between vertex normal and face normal

                if (const Float angle = std::acos(saturate(glm::dot(vertex.normal, face.normal))); angle <=
                    angleThreshold) {
                    // Add to smooth group
                    vertex.tangent += face.tangent;
                    vertex.bitangent += face.bitangent;
                }
            }
        }

        // Normalize and orthogonalize
        for (auto& vertex : vertices) {
            orthogonalizeTangent(vertex);
        }
    }

    Mat3 TangentSpace::buildTBNMatrix(const Vertex& vertex) noexcept {
        const Vec3 T = vertex.tangent;
        const Vec3 B = vertex.bitangent * vertex.handedness;
        const Vec3 N = vertex.normal;

        return Mat3(T, B, N);
    }

    Mat3 TangentSpace::buildTBNFromTN(
        const Vec3& tangent,
        const Vec3& normal,
        const Float handedness) noexcept {
        const Vec3 T = glm::normalize(tangent);
        const Vec3 N = glm::normalize(normal);
        const Vec3 B = glm::cross(N, T) * handedness;

        return Mat3(T, B, N);
    }

    // TODO: Ver que esto sea correcto, en caso contrario eliminar heightCenter
    Vec3 TangentSpace::heightMapToNormal(
        const Float heightCenter,
        const Float heightLeft,
        const Float heightRight,
        const Float heightUp,
        const Float heightDown,
        const Float scale) noexcept {
        // Calcular diferencias desde el centro
        const Float dx = ((heightRight - heightCenter) - (heightCenter - heightLeft)) * scale;
        const Float dy = ((heightUp - heightCenter) - (heightCenter - heightDown)) * scale;

        const Vec3 normal(-dx, -dy, 1.0f); // Normal apuntando hacia arriba
        return glm::normalize(normal);
    }

    Vec3 TangentSpace::blendNormals(
        const Vec3& normal1,
        const Vec3& normal2,
        const Float weight) noexcept {
        // Unpack from [0,1] to [-1,1] if needed
        const Vec3 n1 = normal1 * 2.0f - 1.0f;
        const Vec3 n2 = normal2 * 2.0f - 1.0f;

        // UDN blending (Unity Developer Network method)
        Vec3 blended = glm::normalize(Vec3(
            n1.x + n2.x,
            n1.y + n2.y,
            n1.z
        ));

        // Weighted blend
        blended = glm::mix(n1, blended, weight);

        // Pack back to [0,1]
        return blended * 0.5f + 0.5f;
    }

    Vec3 TangentSpace::blendNormalsRNM(
        const Vec3& normal1,
        const Vec3& normal2) noexcept {
        const Vec3 n1 = normal1 * 2.0f - 1.0f;
        const Vec3 n2 = normal2 * 2.0f - 1.0f;

        const Vec3 t = n1 + Vec3(0, 0, 1);
        const Vec3 u = n2 * Vec3(-1, -1, 1);
        const Vec3 blended = t * glm::dot(t, u) - u * t.z;

        return glm::normalize(blended) * 0.5f + 0.5f;
    }

    void TangentSpace::orthogonalizeTangent(Vertex& vertex) noexcept {
        Vec3& T = vertex.tangent;
        Vec3& B = vertex.bitangent;
        const Vec3& N = vertex.normal;

        // Normalize
        T = glm::normalize(T);
        B = glm::normalize(B);

        // Gram-Schmidt orthogonalization
        T = glm::normalize(T - N * glm::dot(N, T));

        // Calculate handedness
        vertex.handedness = (glm::dot(glm::cross(N, T), B) < 0.0f) ? -1.0f : 1.0f;

        // Recalculate bitangent to ensure orthogonality
        B = glm::cross(N, T) * vertex.handedness;
    }
} // namespace engine::math::graphics
