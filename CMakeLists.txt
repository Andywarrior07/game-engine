cmake_minimum_required(VERSION 3.31)
project(game_engine)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- FIX PARA HEADERS DE C++ EN MACOS ---
if (APPLE)
    execute_process(
            COMMAND xcrun --show-sdk-path
            OUTPUT_VARIABLE CMAKE_OSX_SYSROOT
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif ()

# ----------------------------------------

# Basically get all .cpp files inside src
file(GLOB_RECURSE ENGINE_SOURCES CONFIGURE_DEPENDS
    src/*.cpp
)

# Ignored files
list(REMOVE_ITEM ENGINE_SOURCES
    ${CMAKE_SOURCE_DIR}/src/camera/input/CameraInputHandler.cpp
)

add_executable(game_engine
    ${ENGINE_SOURCES}
)

# ----------------------------------------
 
# Resolve dependancies
find_package(OpenGL    REQUIRED)
find_package(GLEW      REQUIRED)
find_package(glm       REQUIRED)
find_package(SDL2      REQUIRED)
# find_package(SDL2_image REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2_IMAGE REQUIRED SDL2_image)
find_package(Bullet    REQUIRED)

target_include_directories(game_engine PRIVATE
    ${SDL2_INCLUDE_DIRS}
    ${SDL2_IMAGE_INCLUDE_DIRS}
)

include_directories(
    ${BULLET_INCLUDE_DIRS}
    ${GLM_INCLUDE_DIRS}
)

target_link_libraries(game_engine
    PRIVATE
        ${SDL2_LIBRARIES}
        ${SDL2_IMAGE_LIBRARIES}
        ${OPENGL_LIBRARIES}
        GLEW::GLEW
        ${BULLET_LIBRARIES}
)

set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -g -O0 -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")
