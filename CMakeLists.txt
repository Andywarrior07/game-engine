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
add_executable(game_engine src/main.cpp
        src/Animation/AnimationManager.cpp
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
        src/camera/core/Viewport.cpp
        src/camera/core/Viewport.cpp
        src/camera/effects/CameraShake.cpp
        src/camera/effects/CameraShake.h
        src/camera/effects/ShakeConfig.h
        src/camera/effects/ShakePattern.h
#        src/camera/input/CameraInputHandler.cpp
#        src/camera/input/CameraInputHandler.h
        src/camera/manager/CameraManager.cpp
        src/camera/manager/CameraManager.h
        src/camera/manager/CameraManagerConfig.h
        src/camera/manager/CameraRegistry.cpp
        src/camera/manager/CameraRegistry.h
        src/camera/presets/FPSCamera.cpp
        src/camera/presets/FPSCamera.h
        src/camera/presets/IsometricCamera.cpp
        src/camera/presets/IsometricCamera.h
        src/camera/presets/SideScrollerCamera.cpp
        src/camera/presets/SideScrollerCamera.h
        src/camera/presets/ThirdPersonCamera.cpp
        src/camera/presets/ThirdPersonCamera.h
        src/camera/presets/TopDownCamera.cpp
        src/camera/presets/TopDownCamera.h
        src/camera/transitions/CameraTransition.cpp
        src/camera/transitions/CameraTransition.h
        src/camera/transitions/TransitionConfig.h
        src/camera/transitions/TransitionEasing.cpp
        src/camera/transitions/TransitionEasing.h
        src/camera/utils/CameraDebugger.cpp
        src/camera/utils/CameraDebugger.h
        src/camera/utils/CoordinateTransform.cpp
        src/camera/utils/CoordinateTransform.h
        src/math/MathSystem.h
        src/math/animation/AnimationMath.cpp
        src/math/animation/AnimationMath.h
        src/math/animation/DualQuaternion.cpp
        src/math/animation/DualQuaternion.h
        src/math/core/FastMath.h
        src/math/core/MathConstants.h
        src/math/core/MathFunctions.h
        src/math/core/MathTypes.h
        src/math/geometry/Distance.h
        src/math/geometry/Intersections.h
        src/math/geometry/Primitives.h
        src/math/graphics/ColorSpace.h
        src/math/graphics/ProjectionMatrix.cpp
        src/math/graphics/ProjectionMatrix.h
        src/math/graphics/TangentSpace.cpp
        src/math/graphics/TangentSpace.h
        src/math/interpolation/Curves.h
        src/math/interpolation/Easing.h
        src/math/interpolation/Interpolation.h
        src/math/noise/PerlinNoise.cpp
        src/math/noise/PerlinNoise.h
        src/math/noise/SimplexNoise.cpp
        src/math/noise/SimplexNoise.h
        src/math/noise/WorleyNoise.cpp
        src/math/noise/WorleyNoise.h
        src/math/simd/SimdMath.h
        src/math/simd/SimdTypes.h
        src/math/spatial/MortonCode.cpp
        src/math/spatial/MortonCode.h
        src/math/spatial/Octree.h
        src/math/spatial/SpatialHash.h
        src/math/transform/Transform.cpp
        src/math/transform/Transform.h
        src/math/transform/TransformHierarchy.cpp
        src/math/transform/TransformHierarchy.h
        src/math/utils/MathDebug.h
        src/math/utils/Random.cpp
        src/math/utils/Random.h
        src/memory/MemorySystem.h
        src/memory/allocators/HeapAllocator.cpp
        src/memory/allocators/HeapAllocator.h
        src/memory/allocators/LinearAllocator.cpp
        src/memory/allocators/LinearAllocator.h
        src/memory/allocators/PoolAllocator.cpp
        src/memory/allocators/RingBufferAllocator.cpp
        src/memory/allocators/RingBufferAllocator.h
        src/memory/allocators/StackAllocator.cpp
        src/memory/allocators/SystemAllocator.cpp
        src/memory/allocators/SystemAllocator.h
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
        src/resources/types/texture/TextureDesc.cpp
        src/resources/types/texture/TextureDesc.h
        src/resources/types/texture/TextureFormat.h
        src/resources/types/texture/TextureFormatUtils.cpp
        src/resources/types/texture/TextureFormatUtils.h
        src/resources/types/texture/TextureResource.cpp
        src/resources/types/texture/TextureResource.h
        src/resources/utils/ResourceDependencyGraph.cpp
        src/resources/utils/ResourceDependencyGraph.h
        src/resources/utils/ResourcePool.h
        src/physics/core/PhysicsConstants.h
        src/physics/core/PhysicsTypes.h
        src/physics/core/PhysicsConfig.h
        src/physics/core/PhysicsWorld.cpp
        src/physics/core/PhysicsWorld.h
        src/physics/utils/PhysicsDebug.cpp
        src/physics/utils/PhysicsDebug.h
        src/physics/manager/PhysicsManager.cpp
        src/physics/manager/PhysicsManager.h
        src/physics/dynamics/RigidBody.cpp
        src/physics/dynamics/RigidBody.h
        src/physics/collision/CollisionShapes.cpp
        src/physics/collision/CollisionShapes.h
        src/physics/gameplay/CharacterController.cpp
        src/physics/gameplay/CharacterController.h
        src/physics/manager/PhysicsProfiler.cpp
        src/physics/manager/PhysicsProfiler.h
        src/physics/gameplay/VehiclePhysics.cpp
        src/physics/gameplay/VehiclePhysics.h
        src/physics/dynamics/ForceGenerators.cpp
        src/physics/dynamics/ForceGenerators.h
        src/physics/streaming/PhysicsLOD.cpp
        src/physics/streaming/PhysicsLOD.h
        src/physics/gameplay/TriggerSystem.cpp
        src/physics/gameplay/TriggerSystem.h
        src/physics/collision/BroadPhase.cpp
        src/physics/collision/BroadPhase.h
        src/physics/collision/NarrowPhase.cpp
        src/physics/collision/NarrowPhase.h
        src/physics/collision/CollisionDetection.cpp
        src/physics/collision/CollisionDetection.h
        src/physics/constraints/ConstraintSolver.cpp
        src/physics/constraints/ConstraintSolver.h
        src/physics/constraints/ConstraintTypes.cpp
        src/physics/constraints/ConstraintTypes.h
        src/physics/constraints/MotorConstraints.cpp
        src/physics/constraints/MotorConstraints.h
        src/physics/dynamics/Integration.cpp
        src/physics/dynamics/Integration.h
        src/physics/dynamics/SoftBody.cpp
        src/physics/dynamics/SoftBody.h
        src/physics/gameplay/DestructionSystem.cpp
        src/physics/gameplay/DestructionSystem.h
        src/physics/integration/BulletIntegration.cpp
        src/physics/integration/BulletIntegration.h
        src/physics/integration/BulletWrappers.cpp
        src/physics/integration/BulletWrappers.h
        src/physics/integration/ComponentBridge.cpp
        src/physics/integration/ComponentBridge.h
        src/physics/queries/PhysicsQueries.cpp
        src/physics/queries/PhysicsQueries.h
        src/physics/queries/RaycastSystem.cpp
        src/physics/queries/RaycastSystem.h
        src/physics/PhysicsSystem.cpp
        src/physics/PhysicsSystem.h
        src/physics/collision/CollisionFiltering.cpp
        src/physics/collision/CollisionFiltering.h
        src/physics/utils/PhysicsUtils.cpp
        src/physics/utils/PhysicsUtils.h
        src/memory/manager/ObjectPool.h
        src/physics/core/PhysicsTypes.cpp
        src/input/core/InputTypes.h
        src/input/core/InputEvent.h
        src/input/core/InputSnapshot.h
        src/input/devices/base/InputDevice.h
        src/input/platform/sdl2/SDLInputBackend.cpp
        src/input/platform/sdl2/SDLInputBackend.h
        src/input/platform/sdl2/SDLKeyMap.h
        src/input/manager/InputService.cpp
        src/input/manager/InputService.h
        src/input/mapping/ActionMap.h
        src/input/mapping/InputBinding.h
        src/input/mapping/InputContext.h
        src/input/core/InputConfig.h
        src/input/core/InputConstants.h
        src/input/core/InputEventQueue.h
        src/input/core/InputMemoryPools.h
        src/input/devices/base/DeviceState.h
        src/input/devices/base/DeviceService.h
        src/input/devices/keyboard/KeyboardDevice.cpp
        src/input/devices/keyboard/KeyboardDevice.h
        src/input/devices/keyboard/KeyboardState.h
        src/input/devices/mouse/MouseState.h
        src/input/devices/mouse/MouseDevice.cpp
        src/input/devices/mouse/MouseDevice.h
        src/input/devices/mouse/MouseLock.h
        src/input/utils/InputUtils.h
        src/input/devices/base/DeviceCapabilities.h
        src/input/devices/base/DeviceService.cpp
        src/input/devices/gamepad/GamepadDevice.cpp
        src/input/devices/gamepad/GamepadDevice.h
        src/input/devices/gamepad/GamepadState.h
        src/input/processing/stages/FilterStage.h
        src/input/processing/stages/DeadzoneStage.h
        src/input/processing/stages/SensitivityStage.h
        src/input/processing/EventProcessor.cpp
        src/input/processing/EventProcessor.h
        src/input/processing/SnapshotManager.cpp
        src/input/processing/SnapshotManager.h
        src/input/mapping/BindingResolver.h
        src/input/mapping/ContextStack.h
        src/input/core/InputMemoryPools.cpp
        src/input/platform/sdl2/SDLKeyMap.cpp
        src/input/manager/InputLifecycle.h
        src/input/utils/ConfigLoader.h
        src/input/debug/InputLogger.h
        src/input/debug/InputDebugOverlay.h
        src/input/InputSystem.cpp
        src/input/InputSystem.h
        src/input/devices/base/DeviceConnectionStat.h
        src/input/devices/mouse/MouseDevice.cpp
        src/PlayerController.cpp
        src/PlayerController.h
        src/input/processing/stages/DeadzoneStage.cpp
        src/input/processing/stages/SensitivityStage.cpp
        src/input/processing/stages/FilterStage.cpp
        src/input/manager/InputLifecycle.cpp
        src/input/debug/InputLogger.cpp
        src/input/debug/InputDebugOverlay.cpp
        src/input/mapping/ContextStack.cpp
        src/input/utils/ConfigLoader.cpp
        src/memory/allocators/CircularBufferAllocator.cpp
        src/memory/allocators/CircularBufferAllocator.h
        src/memory/allocators/TimerPoolAllocator.cpp
        src/memory/allocators/TimerPoolAllocator.h
        src/memory/allocators/TimelineDataAllocator.cpp
        src/memory/allocators/TimelineDataAllocator.h
        src/time/core/TimeTypes.h
        src/time/core/TimelineType.h
        src/time/core/TimestepMode.h
        src/time/profiling/Profilingconfig.h
        src/time/core/TimeConfig.h
        src/time/manager/TimeManagerConfig.h
        src/time/timers/Timer.cpp
        src/time/timers/Timer.h
        src/time/profiling/TimeStats.h
        src/time/timers/TimerHandle.h
        src/time/core/ITimeBackend.h
        src/time/backends/SDLTimeBackend.cpp
        src/time/backends/SDLTimeBackend.h
        src/time/timelines/RealTimeline.cpp
        src/time/timelines/RealTimeline.h
        src/time/core/Timeline.h
        src/time/timelines/GameTimeline.cpp
        src/time/timelines/GameTimeline.h
        src/time/timelines/UITimeline.cpp
        src/time/timelines/UITimeline.h
        src/time/timelines/PhysicsTimeline.cpp
        src/time/timelines/PhysicsTimeline.h
        src/time/timelines/AudioTimeline.cpp
        src/time/timelines/AudioTimeline.h
        src/time/timelines/CustomTimeline.h
        src/time/manager/TimelineManager.cpp
        src/time/manager/TimelineManager.h
        src/time/timers/TimerPool.cpp
        src/time/timers/TimerPool.h
        src/time/timers/TimerSystem.cpp
        src/time/timers/TimerSystem.h
        src/time/utils/TimeConverter.h
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
