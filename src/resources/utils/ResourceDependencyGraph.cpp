//
// Created by Andres Guerrero on 20-08-25.
//

#include "ResourceDependencyGraph.h"

#include <algorithm>
#include <iostream>
#include <mutex>
#include <ranges>
#include <sstream>

namespace engine::resources {
// ====================================================================
// DEPENDENCY MANAGEMENT
// ====================================================================

bool ResourceDependencyGraph::addDependency(const ResourceID resource,
                                            const ResourceID dependency) {
  // Check for self-dependency
  if (resource == dependency) {
    std::cerr << "Warning: Cannot add self-dependency for resource " << resource
              << std::endl;
    return false;
  }

  // Check if a would create cycle
  if (wouldCreateCycle(resource, dependency)) {
    std::cerr << "Warning: Adding dependency would create cycle: " << resource
              << " -> " << dependency << std::endl;
    return false;
  }

  std::unique_lock lock(mutex_);

  dependencies_[resource].insert(dependency);
  dependents_[dependency].insert(resource);

  return true;
}

bool ResourceDependencyGraph::addDependencies(
    const ResourceID resource, const std::vector<ResourceID> &deps) {
  bool allAdded = true;

  for (const ResourceID dep : deps) {
    if (!addDependency(resource, dep)) {
      allAdded = false;
    }
  }

  return allAdded;
}

void ResourceDependencyGraph::removeDependency(const ResourceID resource,
                                               const ResourceID dependency) {
  std::unique_lock lock(mutex_);

  if (auto depIt = dependencies_.find(resource); depIt != dependencies_.end()) {
    depIt->second.erase(dependency);
    if (depIt->second.empty()) {
      dependencies_.erase(depIt);
    }
  }

  if (const auto depentIt = dependents_.find(dependency);
      depentIt != dependents_.end()) {
    depentIt->second.erase(resource);
    if (depentIt->second.empty()) {
      dependents_.erase(depentIt);
    }
  }
}

void ResourceDependencyGraph::removeResource(const ResourceID resource) {
  std::unique_lock lock(mutex_);

  // Remove all dependencies of this resource
  if (const auto depIt = dependencies_.find(resource);
      depIt != dependencies_.end()) {
    for (ResourceID dep : depIt->second) {
      if (const auto depentIt = dependents_.find(dep);
          depentIt != dependents_.end()) {
        depentIt->second.erase(resource);
        if (depentIt->second.empty()) {
          dependents_.erase(depentIt);
        }
      }
    }
    dependencies_.erase(depIt);
  }

  // Remove this resource as a dependency of others
  if (const auto depentIt = dependents_.find(resource);
      depentIt != dependents_.end()) {
    for (ResourceID dependent : depentIt->second) {
      auto depIt2 = dependencies_.find(dependent);
      if (depIt2 != dependencies_.end()) {
        depIt2->second.erase(resource);
        if (depIt2->second.empty()) {
          dependencies_.erase(depIt2);
        }
      }
    }
    dependents_.erase(depentIt);
  }
}

void ResourceDependencyGraph::clear() {
  std::unique_lock lock(mutex_);
  dependencies_.clear();
  dependents_.clear();
}

// ====================================================================
// DEPENDENCY QUERIES
// ====================================================================

std::vector<ResourceID>
ResourceDependencyGraph::getDependencies(const ResourceID resource) const {
  std::shared_lock lock(mutex_);

  if (const auto it = dependencies_.find(resource); it != dependencies_.end()) {
    return {it->second.begin(), it->second.end()};
  }

  return {};
}

std::vector<ResourceID>
ResourceDependencyGraph::getDependents(const ResourceID resource) const {
  std::shared_lock lock(mutex_);

  if (const auto it = dependents_.find(resource); it != dependents_.end()) {
    return {it->second.begin(), it->second.end()};
  }

  return {};
}

std::vector<ResourceID>
ResourceDependencyGraph::getAllDependencies(const ResourceID resource) const {
  std::shared_lock lock(mutex_);

  std::unordered_set<ResourceID> visited;
  std::vector<ResourceID> result;

  getAllDependenciesDFS(resource, visited, result);

  return result;
}

std::vector<ResourceID>
ResourceDependencyGraph::getAllDependents(const ResourceID resource) const {
  std::shared_lock lock(mutex_);

  std::unordered_set<ResourceID> visited;
  std::vector<ResourceID> result;

  getAllDependentsDFS(resource, visited, result);

  return result;
}

bool ResourceDependencyGraph::dependsOn(const ResourceID resource,
                                        const ResourceID dependency) const {
  auto allDeps = getAllDependencies(resource);
  return std::ranges::find(allDeps, dependency) != allDeps.end();
}

std::size_t
ResourceDependencyGraph::getDependencyDepth(const ResourceID resource) const {
  std::shared_lock lock(mutex_);

  std::unordered_map<ResourceID, std::size_t> cache;
  return getDependencyDepthDFS(resource, cache);
}

// ====================================================================
// LOADING ORDER
// ====================================================================

std::vector<ResourceID> ResourceDependencyGraph::getLoadingOrder(
    const std::vector<ResourceID> &resources) const {
  std::shared_lock lock(mutex_);

  std::vector<ResourceID> result;
  std::unordered_map<ResourceID, int> inDegree;
  std::queue<ResourceID> queue;

  // Initialize in-degrees for requested resources
  for (ResourceID res : resources) {
    inDegree[res] = 0;
  }

  // Calculate in-degrees
  for (ResourceID res : resources) {
    if (auto it = dependencies_.find(res); it != dependencies_.end()) {
      for (ResourceID dep : it->second) {
        if (inDegree.contains(dep)) {
          ++inDegree[dep];
        }
      }
    }
  }

  // Find resources with no dependencies
  for (const auto &[res, degree] : inDegree) {
    if (degree == 0) {
      queue.push(res);
    }
  }

  // Process queue (Kahn's algorithm)
  while (!queue.empty()) {
    ResourceID current = queue.front();
    queue.pop();
    result.push_back(current);

    // Reduce in-degree of dependents
    if (auto depIt = dependents_.find(current); depIt != dependents_.end()) {
      for (ResourceID dependent : depIt->second) {
        if (inDegree.contains(dependent)) {
          if (--inDegree[dependent] == 0) {
            queue.push(dependent);
          }
        }
      }
    }
  }

  // Check for cycles
  if (result.size() != resources.size()) {
    std::cerr << "Warning: Circular dependency detected! Only " << result.size()
              << " of " << resources.size() << " resources can be loaded."
              << std::endl;
  }

  return result;
}

std::vector<ResourceID> ResourceDependencyGraph::getUnloadingOrder(
    const std::vector<ResourceID> &resources) const {
  // Unloading order is the reverse of loading order
  auto loadingOrder = getLoadingOrder(resources);
  std::ranges::reverse(loadingOrder);
  return loadingOrder;
}

// ====================================================================
// CYCLE DETECTION
// ====================================================================

bool ResourceDependencyGraph::hasCycle() const {
  std::shared_lock lock(mutex_);

  std::unordered_set<ResourceID> visited;
  std::unordered_set<ResourceID> recursionStack;

  for (const auto &resource : dependencies_ | std::views::keys) {
    if (hasCycleDFS(resource, visited, recursionStack)) {
      return true;
    }
  }

  return false;
}

bool ResourceDependencyGraph::wouldCreateCycle(
    const ResourceID resource, const ResourceID dependency) const {
  // Check if dependency already depends on resource (directly or indirectly)
  return dependsOn(dependency, resource);
}

std::vector<std::vector<ResourceID>>
ResourceDependencyGraph::findAllCycles() const {
  std::vector<std::vector<ResourceID>> cycles;

  // This is a simplified implementation
  // A full implementation would use Tarjan's or Johnson's algorithm
  // for finding all cycles in a directed graph

  if (hasCycle()) {
    // Placeholder - in production, implement proper cycle detection
    std::cerr << "Cycle detection not fully implemented" << std::endl;
  }

  return cycles;
}

// ====================================================================
// STATISTICS
// ====================================================================

std::size_t ResourceDependencyGraph::getResourceCount() const {
  std::shared_lock lock(mutex_);

  std::unordered_set<ResourceID> allResources;

  for (const auto &[res, deps] : dependencies_) {
    allResources.insert(res);
    allResources.insert(deps.begin(), deps.end());
  }

  for (const auto &[res, deps] : dependents_) {
    allResources.insert(res);
    allResources.insert(deps.begin(), deps.end());
  }

  return allResources.size();
}

std::size_t ResourceDependencyGraph::getEdgeCount() const {
  std::shared_lock lock(mutex_);

  std::size_t count = 0;
  for (const auto &deps : dependencies_ | std::views::values) {
    count += deps.size();
  }

  return count;
}

bool ResourceDependencyGraph::isEmpty() const {
  std::shared_lock lock(mutex_);
  return dependencies_.empty() && dependents_.empty();
}

std::vector<ResourceID> ResourceDependencyGraph::getRootResources() const {
  std::shared_lock lock(mutex_);

  std::vector<ResourceID> roots;
  std::unordered_set<ResourceID> allResources;

  // Collect all resources
  for (const auto &res : dependencies_ | std::views::keys) {
    allResources.insert(res);
  }
  for (const auto &res : dependents_ | std::views::keys) {
    allResources.insert(res);
  }

  // Find resources with no dependencies
  for (ResourceID res : allResources) {
    if (!dependencies_.contains(res) || dependencies_.at(res).empty()) {
      roots.push_back(res);
    }
  }

  return roots;
}

std::vector<ResourceID> ResourceDependencyGraph::getLeafResources() const {
  std::shared_lock lock(mutex_);

  std::vector<ResourceID> leaves;
  std::unordered_set<ResourceID> allResources;

  // Collect all resources
  for (const auto &res : dependencies_ | std::views::keys) {
    allResources.insert(res);
  }
  for (const auto &res : dependents_ | std::views::keys) {
    allResources.insert(res);
  }

  // Find resources with no dependents
  for (ResourceID res : allResources) {
    if (!dependents_.contains(res) || dependents_.at(res).empty()) {
      leaves.push_back(res);
    }
  }

  return leaves;
}

// ====================================================================
// VALIDATION
// ====================================================================

bool ResourceDependencyGraph::validate() const {
  std::shared_lock lock(mutex_);

  // Check that dependencies and dependents are consistent
  for (const auto &[resource, deps] : dependencies_) {
    for (ResourceID dep : deps) {
      auto it = dependents_.find(dep);
      if (it == dependents_.end() || !it->second.contains(resource)) {
        std::cerr << "Inconsistency: " << resource << " depends on " << dep
                  << " but reverse link missing" << std::endl;
        return false;
      }
    }
  }

  for (const auto &[resource, deps] : dependents_) {
    for (ResourceID dep : deps) {
      if (const auto it = dependencies_.find(dep);
          it == dependencies_.end() || !it->second.contains(resource)) {
        std::cerr << "Inconsistency: " << dep << " is dependent of " << resource
                  << " but forward link missing" << std::endl;
        return false;
      }
    }
  }

  return true;
}

std::string ResourceDependencyGraph::toString() const {
  std::shared_lock lock(mutex_);

  std::stringstream ss;
  ss << "ResourceDependencyGraph:\n";
  ss << "  Resources: " << getResourceCount() << "\n";
  ss << "  Edges: " << getEdgeCount() << "\n";
  ss << "  Has Cycles: " << (hasCycle() ? "Yes" : "No") << "\n";
  ss << "\nDependencies:\n";

  for (const auto &[resource, deps] : dependencies_) {
    ss << "  " << resource << " -> {";
    bool first = true;
    for (ResourceID dep : deps) {
      if (!first)
        ss << ", ";
      ss << dep;
      first = false;
    }
    ss << "}\n";
  }

  return ss.str();
}

// ====================================================================
// PRIVATE HELPER METHODS
// ====================================================================

bool ResourceDependencyGraph::hasCycleDFS(
    ResourceID node, std::unordered_set<ResourceID> &visited,
    std::unordered_set<ResourceID> &recursionStack) const {
  visited.insert(node);
  recursionStack.insert(node);

  if (const auto it = dependencies_.find(node); it != dependencies_.end()) {
    for (ResourceID neighbor : it->second) {
      if (!visited.contains(neighbor)) {
        if (hasCycleDFS(neighbor, visited, recursionStack)) {
          return true;
        }
      } else if (recursionStack.contains(neighbor)) {
        return true;
      }
    }
  }

  recursionStack.erase(node);
  return false;
}

void ResourceDependencyGraph::getAllDependenciesDFS(
    ResourceID resource, std::unordered_set<ResourceID> &visited,
    std::vector<ResourceID> &result) const {
  if (const auto it = dependencies_.find(resource); it != dependencies_.end()) {
    for (ResourceID dep : it->second) {
      if (!visited.contains(dep)) {
        visited.insert(dep);
        result.push_back(dep);
        getAllDependenciesDFS(dep, visited, result);
      }
    }
  }
}

void ResourceDependencyGraph::getAllDependentsDFS(
    ResourceID resource, std::unordered_set<ResourceID> &visited,
    std::vector<ResourceID> &result) const {
  if (const auto it = dependents_.find(resource); it != dependents_.end()) {
    for (ResourceID dep : it->second) {
      if (!visited.contains(dep)) {
        visited.insert(dep);
        result.push_back(dep);
        getAllDependentsDFS(dep, visited, result);
      }
    }
  }
}

std::size_t ResourceDependencyGraph::getDependencyDepthDFS(
    const ResourceID resource,
    std::unordered_map<ResourceID, std::size_t> &cache) const {
  // Check cache
  auto cacheIt = cache.find(resource);
  if (cacheIt != cache.end()) {
    return cacheIt->second;
  }

  std::size_t maxDepth = 0;

  if (const auto it = dependencies_.find(resource); it != dependencies_.end()) {
    for (const ResourceID dep : it->second) {
      const std::size_t depthOfDep = getDependencyDepthDFS(dep, cache);
      maxDepth = std::max(maxDepth, depthOfDep + 1);
    }
  }

  cache[resource] = maxDepth;
  return maxDepth;
}
} // namespace engine::resources
