//
// Created by Andres Guerrero on 17-08-25.
//

#pragma once

#include <functional>
#include <string>
#include <typeindex>

#include "../core/Resource.h"

namespace engine::resources {
    // Resource creation function
    using ResourceCreator = std::function<std::unique_ptr<Resource>(ResourceID, std::string&)>;

    class ResourceFactory {
    public:
        // Register a resource type
        template <typename T>
        void registerType(ResourceType type) {
            static_assert(std::is_base_of_v<Resource, T>, "T must derive from Resource");

            creators_[type] = [](ResourceID id, const std::string& name) -> std::unique_ptr<Resource> {
                return std::make_unique<T>(id, name);
            };

            typeMap_[std::type_index(typeid(T))] = type;
        }

        // Create a resource
        std::unique_ptr<Resource> create(const ResourceType type, const ResourceID id, std::string& name) {
            auto it = creators_.find(type);

            if (it != creators_.end()) {
                return it->second(id, name);
            }

            return nullptr;
        }

        // Get resource type from class type
        template<typename T>
        std::optional<ResourceType> getResourceType() const {
            auto it = typeMap_.find(std::type_index(typeid(T)));

            if (it != typeMap_.end()) {
                return it->second;
            }

            return std::nullopt;
        }

    private:
        std::unordered_map<ResourceType, ResourceCreator> creators_;
        std::unordered_map<std::type_index, ResourceType> typeMap_;
    };
}
