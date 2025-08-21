cmake_minimum_required(VERSION 3.31)
project(simple_game)

set(CMAKE_CXX_STANDARD 20)

#find_package(PkgConfig REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glm REQUIRED)

set(SDL2_INCLUDE_DIR /opt/homebrew/include/SDL2)
set(SDL2_LIBRARY /opt/homebrew/lib/libSDL2.dylib)
set(SDL2_IMAGE_INCLUDE_DIR /opt/homebrew/include/SDL2)
set(SDL2_IMAGE_LIBRARY /opt/homebrew/lib/libSDL2_image.dylib)

include_directories(
        ${SDL2_INCLUDE_DIR}
        ${SDL2_IMAGE_INCLUDE_DIR}
        /opt/homebrew/include
        /opt/homebrew/include/bullet
)
link_libraries(${SDL2_LIBRARY} ${SDL2_IMAGE_LIBRARY})

#pkg_check_modules(SDL2 REQUIRED sdl2)

find_package(Bullet REQUIRED)
add_executable(simple_game src/main.cpp
        src/Input/InputManager.cpp
        Game.cpp
        src/Animation/AnimationManager.cpp
        src/Camera/CameraManager.cpp
        src/Physics/PhysicsManager.cpp
        src/memory/allocators/PoolAllocator.cpp
        src/memory/allocators/StackAllocator.cpp
        src/memory/core/IAllocator.cpp
        src/memory/allocators/LinearAllocator.cpp
        src/memory/allocators/LinearAllocator.h
        src/memory/allocators/RingBufferAllocator.cpp
        src/memory/allocators/RingBufferAllocator.h
        src/memory/manager/MemoryManager.cpp
        src/memory/manager/MemoryManager.h
        src/memory/manager/MemoryConfig.h
        src/memory/utils/ScopedAllocator.cpp
        src/memory/utils/ScopedAllocator.h
        src/memory/MemorySystem.h
        src/resources/core/ResourceTypes.h
        src/resources/core/ResourceMetadata.h
        src/resources/core/Resource.cpp
        src/resources/core/Resource.h
        src/resources/core/ResourceHandle.cpp
        src/resources/core/ResourceHandle.h
        src/resources/loader/IResourceLoader.cpp
        src/resources/loader/IResourceLoader.h
        src/resources/cache/ResourceCache.cpp
        src/resources/cache/ResourceCache.h
        src/resources/manager/ResourceFactory.cpp
        src/resources/manager/ResourceFactory.h
        src/resources/manager/ResourceManager.cpp
        src/resources/manager/ResourceManager.h
        src/resources/packaging/ResourceGroup.cpp
        src/resources/packaging/ResourceGroup.h
        src/resources/packaging/ResourcePackage.cpp
        src/resources/packaging/ResourcePackage.h
        src/resources/utils/ResourcePool.h
        src/resources/streaming/StreamableResource.cpp
        src/resources/streaming/StreamableResource.h
        src/resources/utils/ResourceDependencyGraph.cpp
        src/resources/utils/ResourceDependencyGraph.h
        src/resources/manager/ResourceRegistry.cpp
        src/resources/manager/ResourceRegistry.h
        src/resources/types/textures/TextureResource.cpp
        src/resources/types/textures/TextureResource.h
        src/resources/async/ResourceTask.cpp
        src/resources/async/ResourceTask.h
        src/resources/async/ResourceTaskScheduler.cpp
        src/resources/async/ResourceTaskScheduler.h
        src/resources/types/textures/TextureFormat.h
        src/resources/types/textures/TextureUtils.h
)

target_link_libraries(${PROJECT_NAME}
        ${SDL2_LIBRARIES}
        ${SDL2_IMAGE_LIBRARY}
        ${OPENGL_LIBRARIES}
        GLEW::GLEW
        ${BULLET_LIBRARIES}
)

target_compile_options(${PROJECT_NAME} PRIVATE ${SDL2_CFLAGS_OTHER})

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0 -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")