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
        src/Resource/ResourceManager.cpp
        src/Texture/TextureResource.cpp
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