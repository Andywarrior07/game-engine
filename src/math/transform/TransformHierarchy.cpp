/**
 * @file TransformHierarchy.cpp
 * @brief Hierarchical transform management system
 * @author Andrés Guerrero
 * @date 27-08-2025
 *
 * Manages complex parent-child transform relationships with efficient
 * dirty propagation and batch updates for scene graphs.
 */

#include "TransformHierarchy.h"

#include <queue>

namespace engine::math {
    EntityID TransformHierarchy::createEntity(const Transform& transform) {
        const EntityID id = nextEntityId_++;

        nodes_[id] = TransformNode();
        nodes_[id].entity = id;
        nodes_[id].transform = transform;
        dirtyNodes_.insert(id);

        return id;
    }

    void TransformHierarchy::destroyEntity(const EntityID entity, const bool destroyChildren) {
        if (!exists(entity)) return;

        const auto& node = nodes_[entity];

        // Handle children
        if (destroyChildren) {
            // Recursively destroy children
            for (const auto childrenCopy = node.children; const EntityID child : childrenCopy) {
                destroyEntity(child, true);
            }
        }
        else {
            // Reparent children to this entity's parent
            for (const EntityID child : node.children) {
                setParent(child, node.parent);
            }
        }

        // Remove from parent's children list
        if (node.parent != INVALID_ENTITY && nodes_.contains(node.parent)) {
            auto& parentChildren = nodes_[node.parent].children;
            std::erase(parentChildren, entity);
        }

        // Remove from tracking
        dirtyNodes_.erase(entity);
        nodes_.erase(entity);
    }

    void TransformHierarchy::setParent(const EntityID child, const EntityID parent, const bool maintainWorldTransform) {
        if (!exists(child)) return;
        if (parent != INVALID_ENTITY && !exists(parent)) return;

        // Check for circular dependency
        if (parent != INVALID_ENTITY && isDescendantOf(parent, child)) {
            return; // Would create cycle
        }

        auto& childNode = nodes_[child];

        // Store world transform if maintaining
        Mat4 worldTransform;
        if (maintainWorldTransform) {
            worldTransform = getWorldMatrix(child);
        }

        // Remove from old parent
        if (childNode.parent != INVALID_ENTITY && nodes_.contains(childNode.parent)) {
            auto& oldParentChildren = nodes_[childNode.parent].children;
            std::erase(oldParentChildren, child);
        }

        // Set new parent
        childNode.parent = parent;
        if (parent != INVALID_ENTITY) {
            nodes_[parent].children.push_back(child);
        }

        // Maintain world transform if requested
        if (maintainWorldTransform) {
            setWorldMatrix(child, worldTransform);
        }
        else {
            markDirty(child);
        }
    }

    bool TransformHierarchy::isDescendantOf(const EntityID descendant, const EntityID ancestor) const {
        if (!exists(descendant) || !exists(ancestor)) return false;

        EntityID current = descendant;
        while (current != INVALID_ENTITY) {
            if (current == ancestor) return true;
            current = getParent(current);
        }
        return false;
    }

    EntityID TransformHierarchy::getRoot(const EntityID entity) const {
        if (!exists(entity)) return INVALID_ENTITY;

        EntityID current = entity;
        EntityID parent = getParent(current);
        while (parent != INVALID_ENTITY) {
            current = parent;
            parent = getParent(current);
        }
        return current;
    }

    std::vector<EntityID> TransformHierarchy::getAllRoots() const {
        std::vector<EntityID> roots;
        for (const auto& [id, node] : nodes_) {
            if (node.parent == INVALID_ENTITY) {
                roots.push_back(id);
            }
        }
        return roots;
    }

    Transform& TransformHierarchy::getTransform(const EntityID entity) {
        static Transform dummy;
        const auto it = nodes_.find(entity);

        return it != nodes_.end() ? it->second.transform : dummy;
    }

    const Transform& TransformHierarchy::getTransform(const EntityID entity) const {
        static const Transform dummy;
        const auto it = nodes_.find(entity);

        return it != nodes_.end() ? it->second.transform : dummy;
    }

    void TransformHierarchy::setTransform(const EntityID entity, const Transform& transform) {
        if (!exists(entity)) return;

        auto& node = nodes_[entity];

        // Apply constraints
        Transform constrainedTransform = transform;
        applyConstraints(node, constrainedTransform);

        node.transform = constrainedTransform;
        markDirty(entity);
    }

    const Mat4& TransformHierarchy::getWorldMatrix(const EntityID entity) const {
        static const Mat4 identity(1.0f);
        const auto it = nodes_.find(entity);

        if (it == nodes_.end()) return identity;

        const auto& node = it->second;
        if (node.isDirty || node.lastUpdateFrame != currentFrame_) {
            updateWorldMatrix(entity);
        }
        return node.worldMatrix;
    }

    void TransformHierarchy::setWorldMatrix(EntityID entity, const Mat4& worldMatrix) {
        if (!exists(entity)) return;

        if (auto& node = nodes_[entity]; node.parent == INVALID_ENTITY) {
            // No parent, world = local
            Vec3 position, scale;
            Quat rotation;
            Transform::decompose(worldMatrix, position, rotation, scale);
            Transform newTransform(position, rotation, scale);
            applyConstraints(node, newTransform);
            node.transform = newTransform;
        }
        else {
            // Has parent, need to convert to local space
            Mat4 parentWorld = getWorldMatrix(node.parent);
            Mat4 localMatrix = glm::inverse(parentWorld) * worldMatrix;

            Vec3 position, scale;
            Quat rotation;
            Transform::decompose(localMatrix, position, rotation, scale);
            Transform newTransform(position, rotation, scale);
            applyConstraints(node, newTransform);
            node.transform = newTransform;
        }

        markDirty(entity);
    }

    Quat TransformHierarchy::getWorldRotation(const EntityID entity) const {
        const Mat4 world = getWorldMatrix(entity);
        Vec3 position, scale;
        Quat rotation;
        Transform::decompose(world, position, rotation, scale);
        return rotation;
    }

    Vec3 TransformHierarchy::getWorldScale(const EntityID entity) const {
        const Mat4 world = getWorldMatrix(entity);
        Vec3 position, scale;
        Quat rotation;
        Transform::decompose(world, position, rotation, scale);
        return scale;
    }

    void TransformHierarchy::setPositionConstraints(const EntityID entity,
                                                    const bool locked,
                                                    const Vec3& min,
                                                    const Vec3& max) {
        if (!exists(entity)) return;
        auto& node = nodes_[entity];
        node.lockPosition = locked;
        node.minPosition = min;
        node.maxPosition = max;
    }

    void TransformHierarchy::setScaleConstraints(const EntityID entity,
                                                 const bool locked,
                                                 const Vec3& min,
                                                 const Vec3& max) {
        if (!exists(entity)) return;
        auto& node = nodes_[entity];
        node.lockScale = locked;
        node.minScale = min;
        node.maxScale = max;
    }

    void TransformHierarchy::updateFrame(const Int frameNumber) {
        currentFrame_ = frameNumber;

        // Process all dirty nodes
        while (!dirtyNodes_.empty()) {
            const EntityID entity = *dirtyNodes_.begin();
            dirtyNodes_.erase(dirtyNodes_.begin());
            updateWorldMatrix(entity);
        }
    }

    void TransformHierarchy::forceUpdate(const EntityID entity) {
        if (!exists(entity)) return;
        markDirty(entity);
        updateWorldMatrix(entity);
    }

    void TransformHierarchy::traverseDepthFirst(const EntityID root, const std::function<void(EntityID)>& callback) const {
        if (!exists(root)) return;

        callback(root);
        for (const EntityID child : getChildren(root)) {
            traverseDepthFirst(child, callback);
        }
    }

    void TransformHierarchy::traverseBreadthFirst(const EntityID root, const std::function<void(EntityID)>& callback) const {
        if (!exists(root)) return;

        std::queue<EntityID> queue;
        queue.push(root);

        while (!queue.empty()) {
            const EntityID entity = queue.front();
            queue.pop();

            callback(entity);

            for (EntityID child : getChildren(entity)) {
                queue.push(child);
            }
        }
    }

    void TransformHierarchy::markDirty(const EntityID entity) {
        if (!exists(entity)) return;

        // Mark this node and all descendants as dirty
        std::queue<EntityID> queue;
        queue.push(entity);

        while (!queue.empty()) {
            EntityID current = queue.front();
            queue.pop();

            nodes_[current].isDirty = true;
            dirtyNodes_.insert(current);

            for (EntityID child : nodes_[current].children) {
                queue.push(child);
            }
        }
    }

    void TransformHierarchy::updateWorldMatrix(const EntityID entity) const {
        const auto it = nodes_.find(entity);
        if (it == nodes_.end()) return;

        auto& node = it->second;

        if (node.parent == INVALID_ENTITY) {
            node.worldMatrix = node.transform.getLocalMatrix();
        }
        else {
            const Mat4& parentWorld = getWorldMatrix(node.parent);
            node.worldMatrix = parentWorld * node.transform.getLocalMatrix();
        }

        node.isDirty = false;
        node.lastUpdateFrame = currentFrame_;
    }

    void TransformHierarchy::applyConstraints(const TransformNode& node, Transform& transform) {
        if (node.lockPosition) {
            transform.setPosition(node.transform.getPosition());
        }
        else {
            Vec3 pos = transform.getPosition();
            pos = glm::clamp(pos, node.minPosition, node.maxPosition);
            transform.setPosition(pos);
        }

        if (node.lockRotation) {
            transform.setRotation(node.transform.getRotation());
        }

        if (node.lockScale) {
            transform.setScale(node.transform.getScale());
        }
        else {
            Vec3 scale = transform.getScale();
            scale = glm::clamp(scale, node.minScale, node.maxScale);
            transform.setScale(scale);
        }
    }
} // namespace engine::math
