/**
 * @file AnimationMath.h
 * @brief Skeletal hierarchy and IK solvers for animation
 * @autor Andrés Guerrero
 * @date 29-08-2025
 *
 * Separates bone hierarchy management from IK solving algorithms.
 * Animation playback should be handled by a separate AnimationSystem.
 */

#pragma once

#include "../core/MathTypes.h"
#include "../core/MathConstants.h"
#include "../core/MathFunctions.h"
#include "../transform/Transform.h"
#include <vector>
#include <array>
#include <functional>
#include <string>
#include <unordered_map>

namespace engine::math::animation {
    // ============================================================================
    // Bone Hierarchy System
    // ============================================================================

    /**
     * @brief Bone structure for skeletal hierarchy
     * This is purely data - animation is handled by AnimationSystem
     */
    struct Bone {
        std::string name;
        int parentIndex = -1; // Index of parent bone (-1 for root)
        Transform localTransform; // Transform relative to parent
        Transform bindPose; // Initial pose transform
        Mat4 inverseBindMatrix; // Inverse of bind pose matrix
        Mat4 worldMatrix; // Current world transform (calculated)
        Vec3 boneLength = Vec3(1, 0, 0); // Bone direction/length vector

        // Constraints for IK
        struct Constraints {
            bool hasLimits = false;
            Vec3 minRotation = Vec3(-PI<Float>);
            Vec3 maxRotation = Vec3(PI<Float>);
            bool lockX = false;
            bool lockY = false;
            bool lockZ = false;
        } constraints;
    };

    /**
     * @brief Skeletal hierarchy container
     * Manages bone structure and world transform updates only
     */
    class SkeletonHierarchy {
    public:
        /**
         * @brief Add bone to skeleton
         */
        int addBone(const std::string& name, int parentIndex = -1);

        /**
         * @brief Set bind pose for bone
         */
        void setBindPose(int boneIndex, const Transform& bindTransform);

        /**
         * @brief Update world transforms for all bones
         */
        void updateWorldTransforms();

        /**
         * @brief Get bone by index
         */
        [[nodiscard]] Bone& getBone(const int index) {
            return bones_[index];
        }

        [[nodiscard]] const Bone& getBone(const int index) const {
            return bones_[index];
        }

        /**
         * @brief Find bone by name
         */
        [[nodiscard]] int findBone(const std::string& name) const {
            const auto it = boneNameMap_.find(name);
            return (it != boneNameMap_.end()) ? it->second : -1;
        }

        [[nodiscard]] std::size_t getBoneCount() const {
            return bones_.size();
        }

        /**
         * @brief Get skinning matrices for GPU
         * These are the final matrices used for vertex skinning
         */
        [[nodiscard]] std::vector<Mat4> getSkinningMatrices() const;

        /**
         * @brief Reset to bind pose
         */
        void resetToBindPose();

    private:
        std::vector<Bone> bones_;
        std::unordered_map<std::string, int> boneNameMap_;

        void updateBoneWorldTransform(int boneIndex);
    };

    // ============================================================================
    // Inverse Kinematics Solvers
    // ============================================================================

    /**
     * @brief Two-bone IK solver (analytical solution for arms/legs)
     */
    class TwoBoneIK {
    public:
        /**
         * @brief Solve IK for two-bone chain
         * @param rootPos Position of root bone
         * @param midPos Current position of middle joint (will be modified)
         * @param endPos Current position of end effector (will be modified)
         * @param targetPos Target position for end effector
         * @param poleVector Hint vector for middle joint direction
         * @param rootToMidLength Length of first bone
         * @param midToEndLength Length of second bone
         */
        static bool solve(
            const Vec3& rootPos,
            Vec3& midPos,
            Vec3& endPos,
            const Vec3& targetPos,
            const Vec3& poleVector,
            Float rootToMidLength,
            Float midToEndLength) noexcept;

        /**
         * @brief Calculate bone rotations from solved positions
         */
        static void calculateRotations(
            const Vec3& rootPos,
            const Vec3& midPos,
            const Vec3& endPos,
            const Vec3& rootForward,
            Quat& rootRotation,
            Quat& midRotation) noexcept;
    };

    /**
     * @brief FABRIK (Forward And Backward Reaching) IK solver
     */
    class FABRIK {
    public:
        struct Joint {
            Vec3 position;
            Float length; // Distance to next joint
            Vec3 constraintAxis; // Constraint axis (if any)
            Float constraintAngle; // Max angle from constraint axis
        };

        /**
         * @brief Solve IK chain using FABRIK algorithm
         */
        static bool solve(
            std::vector<Joint>& joints,
            const Vec3& target,
            Float tolerance = 0.01f,
            int maxIterations = 10) noexcept;
    };

    /**
     * @brief CCD (Cyclic Coordinate Descent) IK solver
     */
    class CCDIK {
    public:
        static bool solve(
            std::vector<Vec3>& jointPositions,
            const std::vector<Float>& boneLengths,
            const Vec3& target,
            Float tolerance = 0.01f,
            int maxIterations = 10) noexcept;
    };

    /**
     * @brief Foot IK for ground alignment
     */
    class FootIK {
    public:
        struct FootData {
            Vec3 position;
            Vec3 normal;
            Float height;
            bool isGrounded;
        };

        /**
         * @brief Adjust foot position and rotation for terrain
         */
        static FootData alignToGround(
            const Vec3& footPos,
            const Vec3& hipPos,
            Float maxReach,
            const std::function<Float(const Vec3&)>& getGroundHeight,
            const std::function<Vec3(const Vec3&)>& getGroundNormal) noexcept;
    };
} // namespace engine::math::animation
