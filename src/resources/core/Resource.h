//
// Created by Andres Guerrero on 16-08-25.
//

#pragma once

#include <shared_mutex>
#include <string>

#include "ResourceMetadata.h"
#include "ResourceTypes.h"

namespace engine::resources {
    class ResourceManager;
    class IResourceLoader;

    // Base class for all resources
    class Resource {
    public:
        Resource(ResourceID id, const std::string& name, ResourceType type);
        virtual ~Resource() = default;

        // Delete copy, allow move
        Resource(const Resource&) = delete;
        Resource& operator=(const Resource&) = delete;
        Resource(Resource&&) = default;
        Resource& operator=(Resource&&) = default;

        // Core interface
        virtual bool load(const std::uint8_t* data, ResourceSize size) = 0;
        virtual bool unload() = 0;
        virtual void reload() { unload(); /* Manager will trigger Re-load */}
        virtual ResourceSize getMemoryUsage() const = 0;
        virtual bool validate() const { return state_ == ResourceState::READY; }

        ResourceID getId() const { return id_; }
        const std::string& getName() const { return name_; }
        ResourceType getType() const { return type_; }
        ResourceState getState() const { return state_.load(std::memory_order_acquire); }
        ResourceVersion getVersion() const { return version_; }
        std::uint32_t getReferenceCount() const { return referenceCount_.load(std::memory_order_acquire); }
        const ResourceMetadata& getMetadata() const { return metadata_; }

        void setState(const ResourceState newState) { state_.store(newState, std::memory_order_release); }
        void incrementReference() { referenceCount_.fetch_add(1, std::memory_order_acq_rel); }
        void decrementReference() { referenceCount_.fetch_sub(1, std::memory_order_acq_rel); }

        // Hot reload support
        virtual bool supportsHotReload() const { return hasFlag(metadata_.flags, ResourceFlags::HOT_RELOAD); }
        virtual void onHotReload() {}

        // Serialization support
        virtual bool serialize(std::vector<std::uint8_t>& buffer) const { return false; }
        virtual bool deserialize(const std::uint8_t* data, ResourceSize size) { return false; }

    protected:
        ResourceID id_;
        std::string name_;
        ResourceType type_;
        std::atomic<ResourceState> state_;
        std::atomic<std::uint32_t> referenceCount_;
        ResourceVersion version_;
        ResourceMetadata metadata_;

        // Thread safety
        mutable std::shared_mutex mutex_;

        // Manager reference (weak to avoid circular dependency)
        ResourceManager* manager_ = nullptr;

        friend class ResourceManager;
        void setManager(ResourceManager* requestedManager) { manager_ = requestedManager;}
    };

    // Smart pointer types for resources
    template<typename T>
    using ResourcePtr = std::shared_ptr<T>;

    template<typename T>
    using WeakResourcePtr = std::weak_ptr<T>;
}
