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
        src/Physics/PhysicsManager.cpp
        src/memory/MemorySystem.h
        src/memory/allocators/LinearAllocator.cpp
        src/memory/allocators/LinearAllocator.h
        src/memory/allocators/PoolAllocator.cpp
        src/memory/allocators/RingBufferAllocator.cpp
        src/memory/allocators/RingBufferAllocator.h
        src/memory/allocators/StackAllocator.cpp
        src/memory/core/IAllocator.cpp
        src/memory/core/SystemInfo.h
        src/memory/manager/MemoryConfig.h
        src/memory/manager/MemoryManager.cpp
        src/memory/manager/MemoryManager.h
        src/memory/utils/ScopedAllocator.cpp
        src/memory/utils/ScopedAllocator.h
        src/resources/async/ResourceTask.cpp
        src/resources/async/ResourceTask.h
        src/resources/async/ResourceTaskScheduler.cpp
        src/resources/async/ResourceTaskScheduler.h
        src/resources/cache/ResourceCache.cpp
        src/resources/cache/ResourceCache.h
        src/resources/core/Resource.cpp
        src/resources/core/Resource.h
        src/resources/core/ResourceHandle.cpp
        src/resources/core/ResourceHandle.h
        src/resources/core/ResourceMetadata.h
        src/resources/core/ResourceTypes.h
        src/resources/loader/IResourceLoader.cpp
        src/resources/loader/IResourceLoader.h
        src/resources/manager/ResourceFactory.cpp
        src/resources/manager/ResourceFactory.h
        src/resources/manager/ResourceManager.cpp
        src/resources/manager/ResourceManager.h
        src/resources/manager/ResourceRegistry.cpp
        src/resources/manager/ResourceRegistry.h
        src/resources/packaging/ResourceGroup.cpp
        src/resources/packaging/ResourceGroup.h
        src/resources/packaging/ResourcePackage.cpp
        src/resources/packaging/ResourcePackage.h
        src/resources/streaming/StreamableResource.cpp
        src/resources/streaming/StreamableResource.h
        src/resources/types/texture/TextureFormat.h
        src/resources/types/texture/TextureResource.cpp
        src/resources/types/texture/TextureResource.h
        src/resources/utils/ResourceDependencyGraph.cpp
        src/resources/utils/ResourceDependencyGraph.h
        src/resources/utils/ResourcePool.h
        src/resources/types/texture/TextureFormatUtils.cpp
        src/resources/types/texture/TextureFormatUtils.h
        src/resources/types/texture/TextureDesc.cpp
        src/resources/types/texture/TextureDesc.h
        src/memory/allocators/HeapAllocator.cpp
        src/memory/allocators/HeapAllocator.h
        src/memory/allocators/SystemAllocator.cpp
        src/memory/allocators/SystemAllocator.h
        src/camera/CameraSystem.h
        src/camera/cameras/BaseCamera.cpp
        src/camera/cameras/BaseCamera.h
        src/camera/cameras/Camera2D.cpp
        src/camera/cameras/Camera2D.h
        src/camera/cameras/Camera3D.cpp
        src/camera/cameras/Camera3D.h
        src/camera/core/CameraBounds.cpp
        src/camera/core/CameraBounds.h
        src/camera/core/CameraTypes.h
        src/camera/effects/CameraShake.cpp
        src/camera/effects/CameraShake.h
        src/camera/effects/ShakeConfig.h
        src/camera/effects/ShakePattern.h
        src/camera/input/CameraInputHandler.cpp
        src/camera/input/CameraInputHandler.h
        src/camera/manager/CameraManagerConfig.h
        src/camera/transitions/TransitionConfig.h
        src/camera/transitions/TransitionEasing.cpp
        src/camera/transitions/TransitionEasing.h
        src/camera/manager/CameraManager.cpp
        src/camera/manager/CameraManager.h
        src/camera/transitions/CameraTransition.cpp
        src/camera/transitions/CameraTransition.h
        src/camera/presets/FPSCamera.cpp
        src/camera/presets/FPSCamera.h
        src/camera/presets/ThirdPersonCamera.cpp
        src/camera/presets/ThirdPersonCamera.h
        src/camera/presets/TopDownCamera.cpp
        src/camera/presets/TopDownCamera.h
        src/camera/presets/SideSCrollerCamera.cpp
        src/camera/presets/SideSCrollerCamera.h
        src/camera/presets/ISometricCamera.cpp
        src/camera/presets/ISometricCamera.h
        src/camera/utils/CoordinateTransform.cpp
        src/camera/utils/CoordinateTransform.h
        src/camera/utils/CameraDebugger.cpp
        src/camera/utils/CameraDebugger.h
        src/camera/manager/CameraRegistry.cpp
        src/camera/manager/CameraRegistry.h
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