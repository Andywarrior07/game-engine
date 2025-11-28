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

# Recolectar todos los archivos .cpp
file(GLOB_RECURSE ENGINE_SOURCES CONFIGURE_DEPENDS
    src/*.cpp
)

# Archivos ignorados
list(REMOVE_ITEM ENGINE_SOURCES
    ${CMAKE_SOURCE_DIR}/src/camera/input/CameraInputHandler.cpp
)

add_executable(game_engine ${ENGINE_SOURCES})

# ----------------------------------------
# Dependencias multiplataforma
# ----------------------------------------

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(Bullet REQUIRED)

# GLM: manejo diferente según plataforma
if (APPLE)
    # En macOS, glm se instala sin archivos de configuración CMake
    find_package(PkgConfig QUIET)
    if (PkgConfig_FOUND)
        pkg_check_modules(GLM glm)
    endif()
    
    # Si pkg-config no lo encuentra, usar ruta manual
    if (NOT GLM_FOUND)
        set(GLM_INCLUDE_DIRS /opt/homebrew/include)
    endif()
else()
    # En Linux (NixOS), glm sí tiene archivos de configuración
    find_package(glm REQUIRED)
endif()

# SDL2: configuración específica por plataforma
if (APPLE)
    set(SDL2_INCLUDE_DIR /opt/homebrew/include/SDL2)
    set(SDL2_LIBRARY /opt/homebrew/lib/libSDL2.dylib)
    set(SDL2_IMAGE_INCLUDE_DIR /opt/homebrew/include/SDL2)
    set(SDL2_IMAGE_LIBRARY /opt/homebrew/lib/libSDL2_image.dylib)
    
    set(SDL2_INCLUDE_DIRS ${SDL2_INCLUDE_DIR})
    set(SDL2_LIBRARIES ${SDL2_LIBRARY})
    set(SDL2_IMAGE_INCLUDE_DIRS ${SDL2_IMAGE_INCLUDE_DIR})
    set(SDL2_IMAGE_LIBRARIES ${SDL2_IMAGE_LIBRARY})
else()
    # En Linux, usar find_package estándar
    find_package(SDL2 REQUIRED)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(SDL2_IMAGE REQUIRED SDL2_image)
endif()

# ----------------------------------------
# Include directories
# ----------------------------------------

target_include_directories(game_engine PRIVATE
    ${SDL2_INCLUDE_DIRS}
    ${SDL2_IMAGE_INCLUDE_DIRS}
)

include_directories(
    ${GLM_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS}
)

# Directorio adicional solo para macOS
if (APPLE)
    include_directories(
        /opt/homebrew/include
        /opt/homebrew/include/bullet
    )
endif()

# ----------------------------------------
# Linking
# ----------------------------------------

target_link_libraries(game_engine
    PRIVATE
        ${SDL2_LIBRARIES}
        ${SDL2_IMAGE_LIBRARIES}
        ${OPENGL_LIBRARIES}
        GLEW::GLEW
        ${BULLET_LIBRARIES}
)

# ----------------------------------------
# Compiler flags
# ----------------------------------------

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0 -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")
