cmake_minimum_required(VERSION 3.25)
project(game_engine)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ========================================
# Platform Detection & Setup
# ========================================

if(APPLE)
    # Fix para headers de C++ estándar en macOS
    execute_process(
            COMMAND xcrun --show-sdk-path
            OUTPUT_VARIABLE CMAKE_OSX_SYSROOT
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
    message(STATUS "macOS SDK: ${CMAKE_OSX_SYSROOT}")
elseif(WIN32)
    # Definiciones específicas de Windows
    add_compile_definitions(SDL_MAIN_HANDLED GLEW_STATIC)
endif()

# ========================================
# Source Files
# ========================================

file(GLOB_RECURSE ENGINE_SOURCES CONFIGURE_DEPENDS src/*.cpp)

# Remover archivos que no queremos compilar
list(REMOVE_ITEM ENGINE_SOURCES
        ${CMAKE_SOURCE_DIR}/src/camera/input/CameraInputHandler.cpp
)

add_executable(game_engine ${ENGINE_SOURCES})

# ========================================
# Compiler Flags por Plataforma
# ========================================

if(MSVC)
    # Windows con MSVC
    target_compile_options(game_engine PRIVATE
            /W4           # Warning level 4
            $<$<CONFIG:Release>:/O2>
            $<$<CONFIG:Debug>:/Od /Zi>
    )
elseif(UNIX)
    # macOS y Linux (GCC/Clang)
    target_compile_options(game_engine PRIVATE
            -Wall -Wextra -Wpedantic
            $<$<CONFIG:Release>:-O3>
            $<$<CONFIG:Debug>:-g -O0>
    )
endif()

# ========================================
# Find Dependencies
# ========================================

# --- OpenGL (multiplataforma) ---
find_package(OpenGL REQUIRED)
message(STATUS "OpenGL found: ${OPENGL_LIBRARIES}")

# --- GLEW ---
if(WIN32)
    find_package(glew CONFIG REQUIRED)
else()
    find_package(GLEW REQUIRED)
endif()
message(STATUS "GLEW found")

# --- GLM (manejo diferente por plataforma) ---
if(WIN32)
    # Windows con vcpkg
    find_package(glm CONFIG REQUIRED)
    set(GLM_FOUND TRUE)
elseif(APPLE)
    # macOS: intentar pkg-config primero, luego ruta manual
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(GLM glm)
    endif()

    if(NOT GLM_FOUND)
        # Fallback a ruta de Homebrew
        if(EXISTS /opt/homebrew/include/glm)
            set(GLM_INCLUDE_DIRS /opt/homebrew/include)
            set(GLM_FOUND TRUE)
            message(STATUS "GLM found in Homebrew: ${GLM_INCLUDE_DIRS}")
        endif()
    endif()
else()
    # Linux: usar find_package estándar
    find_package(glm REQUIRED)
    set(GLM_FOUND TRUE)
endif()

if(NOT GLM_FOUND)
    message(FATAL_ERROR "GLM not found! Install with: brew install glm (macOS) or vcpkg install glm (Windows)")
endif()

# --- SDL2 ---
if(WIN32)
    # Windows con vcpkg
    find_package(SDL2 CONFIG REQUIRED)
    find_package(SDL2_image CONFIG REQUIRED)

    set(SDL2_LIBRARIES SDL2::SDL2 $<TARGET_NAME_IF_EXISTS:SDL2::SDL2main>)
    set(SDL2_IMAGE_LIBRARIES SDL2_image::SDL2_image)

elseif(APPLE)
    # macOS con Homebrew
    set(HOMEBREW_PREFIX /opt/homebrew)

    set(SDL2_INCLUDE_DIRS ${HOMEBREW_PREFIX}/include/SDL2)
    set(SDL2_LIBRARIES ${HOMEBREW_PREFIX}/lib/libSDL2.dylib)
    set(SDL2_IMAGE_INCLUDE_DIRS ${HOMEBREW_PREFIX}/include/SDL2)
    set(SDL2_IMAGE_LIBRARIES ${HOMEBREW_PREFIX}/lib/libSDL2_image.dylib)

    # Verificar que existen
    if(NOT EXISTS ${SDL2_LIBRARIES})
        message(FATAL_ERROR "SDL2 not found in Homebrew! Install with: brew install sdl2 sdl2_image")
    endif()

    message(STATUS "SDL2 found (Homebrew): ${SDL2_LIBRARIES}")
    message(STATUS "SDL2_image found (Homebrew): ${SDL2_IMAGE_LIBRARIES}")

else()
    # Linux: usar find_package y pkg-config
    find_package(SDL2 REQUIRED)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(SDL2_IMAGE REQUIRED SDL2_image)

    message(STATUS "SDL2 found (pkg-config): ${SDL2_LIBRARIES}")
    message(STATUS "SDL2_image found (pkg-config): ${SDL2_IMAGE_LIBRARIES}")
endif()

# --- Bullet Physics ---
find_package(Bullet REQUIRED)

if(WIN32)
    # En vcpkg, los targets de Bullet no tienen el prefijo "Bullet::"
    set(BULLET_LINK_LIBRARIES
            BulletDynamics
            BulletCollision
            LinearMath
    )
else()
    # En Unix (macOS/Linux), usar la variable estándar
    set(BULLET_LINK_LIBRARIES ${BULLET_LIBRARIES})
endif()

message(STATUS "Bullet found: ${BULLET_LINK_LIBRARIES}")

# ========================================
# Include Directories
# ========================================

target_include_directories(game_engine PRIVATE
        ${SDL2_INCLUDE_DIRS}
        ${SDL2_IMAGE_INCLUDE_DIRS}
        ${GLM_INCLUDE_DIRS}
        ${BULLET_INCLUDE_DIRS}
)

# Incluir directorios adicionales para macOS/Homebrew
if(APPLE)
    target_include_directories(game_engine PRIVATE
            /opt/homebrew/include
            /opt/homebrew/include/bullet
    )
endif()

# ========================================
# Link Libraries
# ========================================

target_link_libraries(game_engine PRIVATE
        ${OPENGL_LIBRARIES}
        GLEW::GLEW
        ${SDL2_LIBRARIES}
        ${SDL2_IMAGE_LIBRARIES}
        ${BULLET_LINK_LIBRARIES}
)

# GLM en Windows necesita linkeo explícito del target
if(WIN32)
    target_link_libraries(game_engine PRIVATE glm::glm)
endif()

# ========================================
# Post-Build Info
# ========================================

message(STATUS "")
message(STATUS "========================================")
message(STATUS "Game Engine Configuration Summary")
message(STATUS "========================================")
message(STATUS "Platform: ${CMAKE_SYSTEM_NAME}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "C++ Standard: C++${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "========================================")
message(STATUS "Dependencies:")
message(STATUS "  - OpenGL: ${OPENGL_LIBRARIES}")
message(STATUS "  - GLEW: Found")
message(STATUS "  - GLM: ${GLM_INCLUDE_DIRS}")
message(STATUS "  - SDL2: Found")
message(STATUS "  - SDL2_image: Found")
message(STATUS "  - Bullet: ${BULLET_LINK_LIBRARIES}")
message(STATUS "========================================")
message(STATUS "")