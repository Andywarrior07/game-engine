//
// Created by Andres Guerrero on 16-08-25.
//

#include "Resource.h"

namespace engine::resources {
    Resource::Resource(ResourceID id, const std::string& name, const ResourceType type)
        : id_(id)
          , name_(name)
          , type_(type)
          , state_(ResourceState::UNLOADED)
          , referenceCount_(0)
          , version_(0) {
        metadata_.id = id;
        metadata_.name = name;
        metadata_.type = type;
    }
}
