/**
 * @file TransformHierarchy.h
 * @brief Hierarchical transform management system
 * @author Andrés Guerrero
 * @date 27-08-2025
 * Manages complex parent-child transform relationships with efficient
 * dirty propagation and batch updates for scene graphs.
 */

#pragma once

#include "Transform.h"

#include "../core/MathTypes.h"
#include "../core/MathConstants.h"

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <functional>

namespace engine::math {
    /**
     * @brief Entity identifier for transform system
     */
    using EntityID = std::uint32_t;
    constexpr EntityID INVALID_ENTITY = 0;

    /**
     * @brief Hierarchical transform management for scene graphs
     *
     * Efficiently manages parent-child relationships with:
     * - Automatic dirty propagation
     * - Batch world matrix updates
     * - Hierarchy queries and traversal
     * - Transform constraints
     */
    class TransformHierarchy {
    public:
        /**
         * @brief Transform node in hierarchy
         */
        struct TransformNode {
            Transform transform;
            EntityID entity = INVALID_ENTITY;
            EntityID parent = INVALID_ENTITY;
            std::vector<EntityID> children;

            // Cached world transform
            mutable Mat4 worldMatrix = Mat4(1.0f);
            mutable bool isDirty = true;
            mutable std::uint32_t lastUpdateFrame = 0;

            // Optional constraints
            bool lockPosition = false;
            bool lockRotation = false;
            bool lockScale = false;
            Vec3 minPosition = Vec3(-INFINITY_VALUE<Float>);
            Vec3 maxPosition = Vec3(INFINITY_VALUE<Float>);
            Vec3 minScale = Vec3(0.001f);
            Vec3 maxScale = Vec3(1000.0f);
        };

        TransformHierarchy() = default;
        ~TransformHierarchy() = default;

        // ============================================================================
        // Entity Management
        // ============================================================================

        /**
         * @brief Create new transform entity
         */
        EntityID createEntity(const Transform& transform = Transform());

        /**
         * @brief Destroy entity and optionally its children
         */
        void destroyEntity(EntityID entity, bool destroyChildren = true);

        /**
         * @brief Check if entity exists
         */
        [[nodiscard]] bool exists(const EntityID entity) const {
            return nodes_.contains(entity);
        }

        // ============================================================================
        // Hierarchy Management
        // ============================================================================

        /**
         * @brief Set parent-child relationship
         */
        void setParent(EntityID child, EntityID parent, bool maintainWorldTransform = true);

        /**
         * @brief Get parent of entity
         */
        [[nodiscard]] EntityID getParent(const EntityID entity) const {
            const auto it = nodes_.find(entity);
            return it != nodes_.end() ? it->second.parent : INVALID_ENTITY;
        }

        /**
         * @brief Get children of entity
         */
        [[nodiscard]] std::vector<EntityID> getChildren(const EntityID entity) const {
            const auto it = nodes_.find(entity);
            return it != nodes_.end() ? it->second.children : std::vector<EntityID>();
        }

        /**
         * @brief Check if ancestor is an ancestor of descendant
         */
        [[nodiscard]] bool isDescendantOf(EntityID descendant, EntityID ancestor) const;

        /**
         * @brief Get root entity (entity with no parent)
         */
        [[nodiscard]] EntityID getRoot(EntityID entity) const;

        /**
         * @brief Get all roots in hierarchy
         */
        [[nodiscard]] std::vector<EntityID> getAllRoots() const;

        // ============================================================================
        // Transform Operations
        // ============================================================================

        /**
         * @brief Get local transform
         */
        [[nodiscard]] Transform& getTransform(EntityID entity);

        [[nodiscard]] const Transform& getTransform(EntityID entity) const;

        /**
         * @brief Set local transform
         */
        void setTransform(EntityID entity, const Transform& transform);

        /**
         * @brief Get world transformation matrix
         */
        [[nodiscard]] const Mat4& getWorldMatrix(EntityID entity) const;

        /**
         * @brief Set world transformation matrix
         */
        void setWorldMatrix(EntityID entity, const Mat4& worldMatrix);

        /**
         * @brief Get world position
         */
        [[nodiscard]] Vec3 getWorldPosition(const EntityID entity) const {
            return Vec3(getWorldMatrix(entity)[3]);
        }

        /**
         * @brief Get world rotation
         */
        [[nodiscard]] Quat getWorldRotation(EntityID entity) const;

        /**
         * @brief Get world scale
         */
        [[nodiscard]] Vec3 getWorldScale(EntityID entity) const;

        // ============================================================================
        // Constraints
        // ============================================================================

        /**
         * @brief Set position constraints
         */
        void setPositionConstraints(EntityID entity, bool locked,
                                    const Vec3& min = Vec3(-INFINITY_VALUE<Float>),
                                    const Vec3& max = Vec3(INFINITY_VALUE<Float>));

        /**
         * @brief Set rotation constraints
         */
        void setRotationConstraints(const EntityID entity, const bool locked) {
            if (!exists(entity)) return;
            nodes_[entity].lockRotation = locked;
        }

        /**
         * @brief Set scale constraints
         */
        void setScaleConstraints(EntityID entity, bool locked,
                                 const Vec3& min = Vec3(0.001f),
                                 const Vec3& max = Vec3(1000.0f));

        // ============================================================================
        // Batch Updates
        // ============================================================================

        /**
         * @brief Update all dirty transforms for current frame
         */
        void updateFrame(Int frameNumber);

        /**
         * @brief Force update of specific entity
         */
        void forceUpdate(EntityID entity);

        // ============================================================================
        // Traversal
        // ============================================================================

        /**
         * @brief Traverse hierarchy depth-first
         */
        void traverseDepthFirst(EntityID root, const std::function<void(EntityID)>& callback) const;

        /**
         * @brief Traverse hierarchy breadth-first
         */
        void traverseBreadthFirst(EntityID root, const std::function<void(EntityID)>& callback) const;

    private:
        std::unordered_map<EntityID, TransformNode> nodes_;
        std::unordered_set<EntityID> dirtyNodes_;
        mutable std::uint32_t currentFrame_ = 0;
        EntityID nextEntityId_ = 1;

        /**
         * @brief Mark entity and children as dirty
         */
        void markDirty(EntityID entity);

        /**
         * @brief Update world matrix for entity
         */
        void updateWorldMatrix(EntityID entity) const;

        /**
         * @brief Apply constraints to transform
         */
        static void applyConstraints(const TransformNode& node, Transform& transform);
    };
} // namespace engine::math
