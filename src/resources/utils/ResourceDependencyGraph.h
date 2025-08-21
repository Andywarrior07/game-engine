//
// Created by Andres Guerrero on 20-08-25.
//

#pragma once

#include "../core/ResourceTypes.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <queue>
#include <shared_mutex>

namespace engine::resources {
    /**
     * @brief Manages dependency relationships between resources
     *
     * This class maintains a directed acyclic graph (DAG) of resource dependencies,
     * allowing for proper loading order calculation, circular dependency detection,
     * and cascade operations (loading/unloading dependent resources).
     */
    class ResourceDependencyGraph {
    public:
        /**
         * @brief Constructor
         */
        ResourceDependencyGraph() = default;

        /**
         * @brief Destructor
         */
        ~ResourceDependencyGraph() = default;

        // Delete copy, allow move
        ResourceDependencyGraph(const ResourceDependencyGraph&) = delete;
        ResourceDependencyGraph& operator=(const ResourceDependencyGraph&) = delete;
        ResourceDependencyGraph(ResourceDependencyGraph&&) = default;
        ResourceDependencyGraph& operator=(ResourceDependencyGraph&&) = default;

        // ====================================================================
        // DEPENDENCY MANAGEMENT
        // ====================================================================

        /**
         * @brief Add a dependency relationship
         * @param resource The resource that depends on something
         * @param dependency The resource that is depended upon
         * @return true if successfully added, false if would create a cycle
         */
        bool addDependency(ResourceID resource, ResourceID dependency);

        /**
         * @brief Add multiple dependencies for a resource
         * @param resource The resource that depends on others
         * @param deps List of resources that are depended upon
         * @return true if all successfully added, false if any would create a cycle
         */
        bool addDependencies(ResourceID resource, const std::vector<ResourceID>& deps);

        /**
         * @brief Remove a dependency relationship
         * @param resource The resource that depends on something
         * @param dependency The resource that is depended upon
         */
        void removeDependency(ResourceID resource, ResourceID dependency);

        /**
         * @brief Remove all dependencies for a resource
         * @param resource The resource to remove from the graph
         */
        void removeResource(ResourceID resource);

        /**
         * @brief Clear all dependencies
         */
        void clear();

        // ====================================================================
        // DEPENDENCY QUERIES
        // ====================================================================

        /**
         * @brief Get all direct dependencies of a resource
         * @param resource The resource to query
         * @return List of resource IDs that this resource depends on
         */
        std::vector<ResourceID> getDependencies(ResourceID resource) const;

        /**
         * @brief Get all resources that directly depend on this resource
         * @param resource The resource to query
         * @return List of resource IDs that depend on this resource
         */
        std::vector<ResourceID> getDependents(ResourceID resource) const;

        /**
         * @brief Get all dependencies recursively (transitive closure)
         * @param resource The resource to query
         * @return List of all resource IDs that this resource depends on (directly or indirectly)
         */
        std::vector<ResourceID> getAllDependencies(ResourceID resource) const;

        /**
         * @brief Get all dependents recursively
         * @param resource The resource to query
         * @return List of all resource IDs that depend on this resource (directly or indirectly)
         */
        std::vector<ResourceID> getAllDependents(ResourceID resource) const;

        /**
         * @brief Check if one resource depends on another
         * @param resource The potential dependent
         * @param dependency The potential dependency
         * @return true if resource depends on dependency (directly or indirectly)
         */
        bool dependsOn(ResourceID resource, ResourceID dependency) const;

        /**
         * @brief Get the dependency depth of a resource
         * @param resource The resource to query
         * @return Maximum depth in the dependency tree (0 if no dependencies)
         */
        std::size_t getDependencyDepth(ResourceID resource) const;

        // ====================================================================
        // LOADING ORDER
        // ====================================================================

        /**
         * @brief Get loading order for a set of resources (topological sort)
         * @param resources List of resources to load
         * @return Ordered list of resources (dependencies first)
         */
        std::vector<ResourceID> getLoadingOrder(const std::vector<ResourceID>& resources) const;

        /**
         * @brief Get unloading order for a set of resources (reverse topological sort)
         * @param resources List of resources to unload
         * @return Ordered list of resources (dependents first)
         */
        std::vector<ResourceID> getUnloadingOrder(const std::vector<ResourceID>& resources) const;

        // ====================================================================
        // CYCLE DETECTION
        // ====================================================================

        /**
         * @brief Check if there's any circular dependency in the graph
         * @return true if a cycle exists
         */
        bool hasCycle() const;

        /**
         * @brief Check if adding a dependency would create a cycle
         * @param resource The resource that would depend on something
         * @param dependency The resource that would be depended upon
         * @return true if adding this dependency would create a cycle
         */
        bool wouldCreateCycle(ResourceID resource, ResourceID dependency) const;

        /**
         * @brief Find all cycles in the graph
         * @return List of cycles, where each cycle is a list of resource IDs
         */
        std::vector<std::vector<ResourceID>> findAllCycles() const;

        // ====================================================================
        // STATISTICS
        // ====================================================================

        /**
         * @brief Get total number of resources in the graph
         * @return Number of resources
         */
        std::size_t getResourceCount() const;

        /**
         * @brief Get total number of dependency edges
         * @return Number of edges
         */
        std::size_t getEdgeCount() const;

        /**
         * @brief Check if graph is empty
         * @return true if no resources or dependencies
         */
        bool isEmpty() const;

        /**
         * @brief Get resources with no dependencies (roots)
         * @return List of resource IDs with no dependencies
         */
        std::vector<ResourceID> getRootResources() const;

        /**
         * @brief Get resources with no dependents (leaves)
         * @return List of resource IDs with no dependents
         */
        std::vector<ResourceID> getLeafResources() const;

        // ====================================================================
        // VALIDATION
        // ====================================================================

        /**
         * @brief Validate the integrity of the graph
         * @return true if graph is valid (no inconsistencies)
         */
        bool validate() const;

        /**
         * @brief Get a string representation of the graph for debugging
         * @return String representation
         */
        std::string toString() const;

    private:
        // Graph representation
        std::unordered_map<ResourceID, std::unordered_set<ResourceID>> dependencies_; // resource -> what it depends on
        std::unordered_map<ResourceID, std::unordered_set<ResourceID>> dependents_; // resource -> what depends on it

        // Thread safety
        mutable std::shared_mutex mutex_;

        // Helper methods
        bool hasCycleDFS(ResourceID node,
                         std::unordered_set<ResourceID>& visited,
                         std::unordered_set<ResourceID>& recursionStack) const;

        void getAllDependenciesDFS(ResourceID resource,
                                   std::unordered_set<ResourceID>& visited,
                                   std::vector<ResourceID>& result) const;

        void getAllDependentsDFS(ResourceID resource,
                                 std::unordered_set<ResourceID>& visited,
                                 std::vector<ResourceID>& result) const;

        std::size_t getDependencyDepthDFS(ResourceID resource,
                                          std::unordered_map<ResourceID, std::size_t>& cache) const;
    };
} // namespace engine::resources
