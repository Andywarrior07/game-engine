cmake_minimum_required(VERSION 3.25)
project(game_engine)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ----------------------------------------
# 1. Fuentes
# ----------------------------------------
file(GLOB_RECURSE ENGINE_SOURCES CONFIGURE_DEPENDS src/*.cpp)
# Asegúrate que la ruta del remove sea correcta, a veces CMAKE_SOURCE_DIR ayuda
list(REMOVE_ITEM ENGINE_SOURCES ${CMAKE_SOURCE_DIR}/src/camera/input/CameraInputHandler.cpp)

add_executable(game_engine ${ENGINE_SOURCES})

# ----------------------------------------
# 2. Configuración OS
# ----------------------------------------
if(WIN32)
    target_compile_definitions(game_engine PRIVATE SDL_MAIN_HANDLED GLEW_STATIC)
    target_compile_options(game_engine PRIVATE /W4 /O2 /DNDEBUG)
elseif(APPLE)
    execute_process(COMMAND xcrun --show-sdk-path OUTPUT_VARIABLE CMAKE_OSX_SYSROOT OUTPUT_STRIP_TRAILING_WHITESPACE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
    target_include_directories(game_engine PRIVATE /opt/homebrew/include /opt/homebrew/include/SDL2 /opt/homebrew/include/bullet)
    link_directories(/opt/homebrew/lib)
else()
    target_compile_options(game_engine PRIVATE -Wall -Wextra)
endif()

# ----------------------------------------
# 3. Dependencias
# ----------------------------------------

find_package(OpenGL REQUIRED)

if(WIN32)
    # --- WINDOWS (VCPKG) ---
    find_package(SDL2 CONFIG REQUIRED)
    find_package(SDL2_image CONFIG REQUIRED)
    find_package(glew CONFIG REQUIRED)
    find_package(glm CONFIG REQUIRED)
    find_package(Bullet CONFIG REQUIRED)
else()
    # --- MAC / LINUX ---
    find_package(GLEW REQUIRED)
    find_package(SDL2 REQUIRED)
    find_package(Bullet REQUIRED)

    if(APPLE)
        find_package(PkgConfig QUIET)
        if(PkgConfig_FOUND)
            pkg_check_modules(GLM glm)
        endif()
    else()
        find_package(PkgConfig REQUIRED)
        pkg_check_modules(SDL2_IMAGE REQUIRED SDL2_image)
        find_package(glm REQUIRED)
    endif()
endif()

# ----------------------------------------
# 4. Linking
# ----------------------------------------

target_link_libraries(game_engine PRIVATE
        ${OPENGL_LIBRARIES}
        GLEW::GLEW
)

if(WIN32)
    target_link_libraries(game_engine PRIVATE
            SDL2::SDL2
            $<TARGET_NAME_IF_EXISTS:SDL2::SDL2main>
            SDL2_image::SDL2_image
            glm::glm

            # --- FIX PARA BULLET EN WINDOWS ---
            # En vcpkg, los targets suelen venir SIN el prefijo "Bullet::"
            BulletDynamics
            BulletCollision
            LinearMath
    )
elseif(APPLE)
    target_link_libraries(game_engine PRIVATE
            /opt/homebrew/lib/libSDL2.dylib
            /opt/homebrew/lib/libSDL2_image.dylib
            ${BULLET_LIBRARIES}
    )
else()
    target_link_libraries(game_engine PRIVATE
            ${SDL2_LIBRARIES}
            ${SDL2_IMAGE_LIBRARIES}
            ${BULLET_LIBRARIES}
    )
endif()