//
// Created by Andres Guerrero on 20-08-25.
//

#pragma once

#include "../core/ResourceTypes.h"
#include <unordered_map>
#include <vector>
#include <string>
#include <optional>
#include <shared_mutex>
#include <functional>

namespace engine::resources {

    /**
     * @brief Global registry for resource type metadata and path mappings
     *
     * This singleton class manages the registration of resource types,
     * their associated file extensions, and path aliases for resource loading.
     * It provides a centralized location for resource type configuration.
     */
    class ResourceRegistry {
    public:
        ResourceRegistry() = default;  // Constructor público
        ~ResourceRegistry() = default;

        // Permitir move, prohibir copy (o permitir ambos si tiene sentido)
        ResourceRegistry(const ResourceRegistry&) = delete;
        ResourceRegistry& operator=(const ResourceRegistry&) = delete;
        ResourceRegistry(ResourceRegistry&&) = default;
        ResourceRegistry& operator=(ResourceRegistry&&) = default;

        /**
         * @brief Type information for a resource type
         */
        struct TypeInfo {
            std::string name;                          // Human-readable name
            std::string description;                   // Description of the resource type
            std::vector<std::string> extensions;       // Supported file extensions
            std::size_t averageSize = 0;              // Average size in bytes (for memory planning)
            bool supportsStreaming = false;            // Whether this type supports streaming
            bool supportsCompression = false;          // Whether this type supports compression
            bool supportsCaching = true;               // Whether this type should be cached
        };

        /**
         * @brief Path alias information
         */
        struct PathAlias {
            std::string path;                          // Actual path
            std::string description;                   // Description of what this alias represents
            bool isRelative = true;                    // Whether the path is relative
        };

        /**
         * @brief Loader information for a resource type
         */
        struct LoaderInfo {
            std::string name;                          // Loader name
            int priority = 0;                          // Loader priority (higher = preferred)
            std::function<bool(const std::string&)> canLoad;  // Function to check if loader can handle file
        };

    public:
        /**
         * @brief Get singleton instance
         * @return Reference to the global ResourceRegistry
         */
        static ResourceRegistry& getInstance();

        // ====================================================================
        // RESOURCE TYPE REGISTRATION
        // ====================================================================

        /**
         * @brief Register a resource type with basic metadata
         * @param type Resource type enum value
         * @param name Human-readable name
         * @param extensions Supported file extensions (with dots, e.g., ".png")
         */
        void registerResourceType(ResourceType type,
                                 const std::string& name,
                                 const std::vector<std::string>& extensions);

        /**
         * @brief Register a resource type with full metadata
         * @param type Resource type enum value
         * @param info Complete type information
         */
        void registerResourceType(ResourceType type, const TypeInfo& info);

        /**
         * @brief Unregister a resource type
         * @param type Resource type to unregister
         */
        void unregisterResourceType(ResourceType type);

        /**
         * @brief Update resource type information
         * @param type Resource type to update
         * @param info New type information
         * @return true if type exists and was updated
         */
        bool updateResourceType(ResourceType type, const TypeInfo& info);

        // ====================================================================
        // RESOURCE TYPE QUERIES
        // ====================================================================

        /**
         * @brief Get resource type from file extension
         * @param extension File extension (with or without dot)
         * @return Resource type if found
         */
        std::optional<ResourceType> getTypeFromExtension(const std::string& extension) const;

        /**
         * @brief Get resource type from file path
         * @param filepath Full file path
         * @return Resource type if extension is recognized
         */
        std::optional<ResourceType> getTypeFromPath(const std::string& filepath) const;

        /**
         * @brief Get type information
         * @param type Resource type
         * @return Type information if registered
         */
        std::optional<TypeInfo> getTypeInfo(ResourceType type) const;

        /**
         * @brief Get all registered resource types
         * @return List of registered resource types
         */
        std::vector<ResourceType> getRegisteredTypes() const;

        /**
         * @brief Check if a resource type is registered
         * @param type Resource type to check
         * @return true if registered
         */
        bool isTypeRegistered(ResourceType type) const;

        /**
         * @brief Get all extensions for a resource type
         * @param type Resource type
         * @return List of extensions (empty if not registered)
         */
        std::vector<std::string> getExtensionsForType(ResourceType type) const;

        // ====================================================================
        // PATH ALIAS MANAGEMENT
        // ====================================================================

        /**
         * @brief Register a path alias
         * @param alias Alias name (e.g., "assets", "shaders")
         * @param path Actual path
         * @param description Optional description
         */
        void registerPath(const std::string& alias,
                         const std::string& path,
                         const std::string& description = "");

        /**
         * @brief Unregister a path alias
         * @param alias Alias to remove
         */
        void unregisterPath(const std::string& alias);

        /**
         * @brief Resolve path with aliases
         * @param path Path that may contain aliases (e.g., "@assets/textures/player.png")
         * @return Resolved path with aliases replaced
         */
        std::string resolvePath(const std::string& path) const;

        /**
         * @brief Get path alias information
         * @param alias Alias name
         * @return Path alias information if found
         */
        std::optional<PathAlias> getPathAlias(const std::string& alias) const;

        /**
         * @brief Get all registered path aliases
         * @return Map of alias to PathAlias
         */
        std::unordered_map<std::string, PathAlias> getAllPathAliases() const;

        // ====================================================================
        // LOADER REGISTRATION
        // ====================================================================

        /**
         * @brief Register a loader for a resource type
         * @param type Resource type
         * @param info Loader information
         */
        void registerLoader(ResourceType type, const LoaderInfo& info);

        /**
         * @brief Get loaders for a resource type
         * @param type Resource type
         * @return List of loader information, sorted by priority
         */
        std::vector<LoaderInfo> getLoadersForType(ResourceType type) const;

        // ====================================================================
        // UTILITY METHODS
        // ====================================================================

        /**
         * @brief Normalize file extension (ensure it starts with a dot)
         * @param extension Extension to normalize
         * @return Normalized extension
         */
        static std::string normalizeExtension(const std::string& extension);

        /**
         * @brief Extract extension from file path
         * @param filepath File path
         * @return Extension (with dot) or empty string
         */
        static std::string extractExtension(const std::string& filepath);

        /**
         * @brief Clear all registrations
         */
        void clear();

        /**
         * @brief Reset to default registrations
         */
        void resetToDefaults();

        /**
         * @brief Generate a report of all registrations
         * @return String containing registration information
         */
        std::string generateReport() const;

    private:
        // Initialize default resource types
        void initializeDefaults();

        // Data storage
        std::unordered_map<ResourceType, TypeInfo> typeInfo_;
        std::unordered_map<std::string, ResourceType> extensionToType_;
        std::unordered_map<std::string, PathAlias> pathAliases_;
        std::unordered_map<ResourceType, std::vector<LoaderInfo>> loaders_;

        // Thread safety
        mutable std::shared_mutex mutex_;
    };

} // namespace engine::resources