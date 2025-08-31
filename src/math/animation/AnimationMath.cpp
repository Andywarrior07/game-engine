/**
 * @file AnimationMath.h
 * @brief Skeletal hierarchy and IK solvers for animation
 * @autor Andrés Guerrero
 * @date 29-08-2025
 *
 * Separates bone hierarchy management from IK solving algorithms.
 * Animation playback should be handled by a separate AnimationSystem.
 */

#include "AnimationMath.h"

namespace engine::math::animation {
    int SkeletonHierarchy::addBone(const std::string& name, const int parentIndex) {
        Bone bone;
        bone.name = name;
        bone.parentIndex = parentIndex;
        bones_.push_back(bone);
        boneNameMap_[name] = static_cast<int>(bones_.size() - 1);
        return static_cast<int>(bones_.size() - 1);
    }

    void SkeletonHierarchy::setBindPose(const int boneIndex, const Transform& bindTransform) {
        if (boneIndex < 0 || boneIndex >= bones_.size()) return;

        bones_[boneIndex].bindPose = bindTransform;
        bones_[boneIndex].localTransform = bindTransform;

        // Calculate inverse bind matrix
        updateBoneWorldTransform(boneIndex);
        bones_[boneIndex].inverseBindMatrix = glm::inverse(bones_[boneIndex].worldMatrix);
    }

    void SkeletonHierarchy::updateWorldTransforms() {
        for (std::size_t i = 0; i < bones_.size(); ++i) {
            updateBoneWorldTransform(static_cast<int>(i));
        }
    }

    std::vector<Mat4> SkeletonHierarchy::getSkinningMatrices() const {
        std::vector<Mat4> matrices;
        matrices.reserve(bones_.size());

        for (const auto& bone : bones_) {
            // Skinning matrix = current world transform * inverse bind pose
            matrices.push_back(bone.worldMatrix * bone.inverseBindMatrix);
        }

        return matrices;
    }

    void SkeletonHierarchy::resetToBindPose() {
        for (auto& bone : bones_) {
            bone.localTransform = bone.bindPose;
        }
        updateWorldTransforms();
    }

    void SkeletonHierarchy::updateBoneWorldTransform(const int boneIndex) {
        if (auto& bone = bones_[boneIndex]; bone.parentIndex >= 0 && bone.parentIndex < bones_.size()) {
            const auto& parent = bones_[bone.parentIndex];
            bone.worldMatrix = parent.worldMatrix * bone.localTransform.getLocalMatrix();
        }
        else {
            bone.worldMatrix = bone.localTransform.getLocalMatrix();
        }
    }

    bool TwoBoneIK::solve(
        const Vec3& rootPos,
        Vec3& midPos,
        Vec3& endPos,
        const Vec3& targetPos,
        const Vec3& poleVector,
        const Float rootToMidLength,
        const Float midToEndLength) noexcept {
        Vec3 toTarget = targetPos - rootPos;
        Float targetDistance = glm::length(toTarget);

        // Check if target is reachable
        const Float maxReach = rootToMidLength + midToEndLength;
        const Float minReach = std::abs(rootToMidLength - midToEndLength);

        if (targetDistance > maxReach) {
            // Target too far - stretch towards it
            toTarget = glm::normalize(toTarget) * maxReach;
            targetDistance = maxReach;
        }
        else if (targetDistance < minReach) {
            // Target too close
            toTarget = glm::normalize(toTarget) * minReach;
            targetDistance = minReach;
        }

        // Calculate angles using law of cosines
        const Float rootAngle = std::acos(
            saturate((rootToMidLength * rootToMidLength +
                    targetDistance * targetDistance -
                    midToEndLength * midToEndLength) /
                (2.0f * rootToMidLength * targetDistance))
        );

        // Build coordinate frame
        const Vec3 forward = glm::normalize(toTarget);
        const Vec3 up = glm::normalize(poleVector - forward * glm::dot(poleVector, forward));
        const Vec3 right = glm::cross(up, forward); // TODO: Ver pq no se usa y si deberia usarlo y como

        // Calculate joint positions
        midPos = rootPos + forward * std::cos(rootAngle) * rootToMidLength +
            up * std::sin(rootAngle) * rootToMidLength;

        endPos = targetPos; // End effector reaches target

        return targetDistance <= maxReach;
    }

    void TwoBoneIK::calculateRotations(
        const Vec3& rootPos,
        const Vec3& midPos,
        const Vec3& endPos,
        const Vec3& rootForward,
        Quat& rootRotation,
        Quat& midRotation) noexcept {
        // Calculate root bone rotation
        const Vec3 rootToMid = glm::normalize(midPos - rootPos);
        rootRotation = glm::rotation(rootForward, rootToMid);

        // Calculate mid bone rotation in root's local space
        const Mat3 rootRotMat = glm::mat3_cast(rootRotation);
        const Vec3 localMidToEnd = glm::transpose(rootRotMat) * glm::normalize(endPos - midPos);
        const Vec3 localForward = glm::transpose(rootRotMat) * rootForward;
        midRotation = glm::rotation(localForward, localMidToEnd);
    }

    bool FABRIK::solve(
        std::vector<Joint>& joints,
        const Vec3& target,
        const Float tolerance,
        const int maxIterations) noexcept {
        if (joints.size() < 2) return false;

        const Vec3 rootPos = joints[0].position;
        Float totalLength = 0;

        // Calculate total chain length
        for (std::size_t i = 0; i < joints.size() - 1; ++i) {
            totalLength += joints[i].length;
        }

        // Check if target is reachable
        if (const Float targetDistance = glm::length(target - rootPos); targetDistance > totalLength) {
            // Stretch towards target
            const Vec3 direction = glm::normalize(target - rootPos);
            for (std::size_t i = 0; i < joints.size() - 1; ++i) {
                joints[i + 1].position = joints[i].position + direction * joints[i].length;
            }
            return false;
        }

        // FABRIK iterations
        for (int iter = 0; iter < maxIterations; ++iter) {
            Vec3 endPos = joints.back().position;

            if (const Float error = glm::length(target - endPos); error < tolerance) {
                return true;
            }

            // Backward pass - from end to root
            joints.back().position = target;
            for (int i = static_cast<int>(joints.size()) - 2; i >= 0; --i) {
                Vec3 direction = glm::normalize(joints[i].position - joints[i + 1].position);
                joints[i].position = joints[i + 1].position + direction * joints[i].length;
            }

            // Forward pass - from root to end
            joints[0].position = rootPos; // Fix root
            for (std::size_t i = 1; i < joints.size(); ++i) {
                Vec3 direction = glm::normalize(joints[i].position - joints[i - 1].position);
                joints[i].position = joints[i - 1].position + direction * joints[i - 1].length;
            }
        }

        return glm::length(target - joints.back().position) < tolerance;
    }

    bool CCDIK::solve(
        std::vector<Vec3>& jointPositions,
        const std::vector<Float>& boneLengths,
        // TODO: Ver porque no se usa y como deberia usarse si es que es necesario
        const Vec3& target,
        const Float tolerance,
        const int maxIterations) noexcept {
        if (jointPositions.size() < 2) return false;

        for (int iter = 0; iter < maxIterations; ++iter) {
            if (const Float error = glm::length(target - jointPositions.back()); error < tolerance) return true;

            // Iterate from end to root
            for (int i = static_cast<int>(jointPositions.size()) - 2; i >= 0; --i) {
                Vec3& joint = jointPositions[i];
                Vec3& endEffector = jointPositions.back();

                Vec3 toEndEffector = endEffector - joint;
                Vec3 toTarget = target - joint;

                if (const Float dot = glm::dot(glm::normalize(toEndEffector), glm::normalize(toTarget)); dot < 0.999f) {
                    if (Vec3 axis = glm::cross(toEndEffector, toTarget); glm::length2(axis) > EPSILON_SQUARED) {
                        axis = glm::normalize(axis);
                        Float angle = std::acos(saturate(dot));
                        Quat rotation = glm::angleAxis(angle, axis);

                        // Rotate all joints after current
                        for (std::size_t j = i + 1; j < jointPositions.size(); ++j) {
                            Vec3 relative = jointPositions[j] - joint;
                            jointPositions[j] = joint + glm::rotate(rotation, relative);
                        }
                    }
                }
            }
        }

        return glm::length(target - jointPositions.back()) < tolerance;
    }

    FootIK::FootData FootIK::alignToGround(
        const Vec3& footPos,
        const Vec3& hipPos,
        const Float maxReach,
        const std::function<Float(const Vec3&)>& getGroundHeight,
        const std::function<Vec3(const Vec3&)>& getGroundNormal) noexcept {
        FootData result;
        result.position = footPos;

        // Get ground height at foot position
        const Float groundHeight = getGroundHeight(footPos);

        // Check if foot can reach ground
        if (const Float hipToGround = hipPos.y - groundHeight; hipToGround > maxReach) {
            result.isGrounded = false;
            result.height = groundHeight;
            return result;
        }

        // Adjust foot to ground
        result.position.y = groundHeight;
        result.normal = getGroundNormal(footPos);
        result.height = groundHeight;
        result.isGrounded = true;

        return result;
    }
} // namespace engine::math::animation
