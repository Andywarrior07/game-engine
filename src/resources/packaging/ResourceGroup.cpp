//
// Created by Andres Guerrero on 18-08-25.
//

#include "ResourceGroup.h"

namespace engine::resources {
    void ResourceGroup::loadAll(ResourceManager* manager, ResourcePriority priority) {
        if (!manager) return;

        isLoading_ = true;
        loadedCount_ = 0;

        std::shared_lock lock(mutex_);

        for (ResourceID id: resources_) {
            // This would need to be extended to handle the actual loading
            // For now, it's a placeholder
            ++loadedCount_;
        }

        isLoading_ = false;
    }

    void ResourceGroup::unloadAll(ResourceManager* manager) {
        if (!manager) return;

        std::shared_lock lock(mutex_);

        for (ResourceID id: resources_) {
            manager->unload(id);
        }

        loadedCount_ = 0;
    }
}
