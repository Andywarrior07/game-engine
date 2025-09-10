/**
 * @file PhysicsConstants.cpp
 * @brief Core physics constants and compile-time configurations *
 * @details Central location for all physics-related constants, thresholds,
 *          and compile-time configurations used throughout the physics system
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

#include "../../math/MathSystem.h"

namespace engine::physics {
    // ============================================================================
    // World Constants
    // ============================================================================

    namespace world {
        // Gravity presets (m/s²)
        inline constexpr math::Float GRAVITY_EARTH = -9.80665f;
        inline constexpr math::Float GRAVITY_MOON = -1.62f;
        inline constexpr math::Float GRAVITY_MARS = -3.71f;
        inline constexpr math::Float GRAVITY_JUPITER = -24.79f;
        inline constexpr math::Float GRAVITY_ZERO = 0.0f;
        inline constexpr math::Float GRAVITY_ARCADE = -20.0f; // For responsive platformers
        inline constexpr math::Float GRAVITY_FLOATY = -5.0f; // For underwater/space

        // World bounds (meters)
        inline constexpr math::Float WORLD_MIN_BOUND = -100000.0f;
        inline constexpr math::Float WORLD_MAX_BOUND = 100000.0f;
        inline constexpr math::Float WORLD_KILL_Z = -1000.0f; // Fall death threshold

        // Air resistance
        inline constexpr math::Float AIR_DENSITY = 1.225f; // kg/m³ at sea level
        inline constexpr math::Float WATER_DENSITY = 1000.0f; // kg/m³
        inline constexpr math::Float TERMINAL_VELOCITY = -53.0f; // m/s (human terminal velocity)

        // Timestep
        inline constexpr math::Float FIXED_TIMESTEP_60HZ = 1.0f / 60.0f;
        inline constexpr math::Float FIXED_TIMESTEP_120HZ = 1.0f / 120.0f;
        inline constexpr math::Float FIXED_TIMESTEP_30HZ = 1.0f / 30.0f;
    }

    // ============================================================================
    // Simulation Constants
    // ============================================================================

    namespace simulation {
        // Time steps
        inline constexpr math::Float FIXED_TIMESTEP_60HZ = 1.0f / 60.0f; // 16.67ms
        inline constexpr math::Float FIXED_TIMESTEP_120HZ = 1.0f / 120.0f; // 8.33ms
        inline constexpr math::Float FIXED_TIMESTEP_240HZ = 1.0f / 240.0f; // 4.17ms
        inline constexpr math::Float FIXED_TIMESTEP_30HZ = 1.0f / 30.0f; // 33.33ms

        inline constexpr math::Float DEFAULT_TIMESTEP = FIXED_TIMESTEP_60HZ;
        inline constexpr math::Float MIN_TIMESTEP = FIXED_TIMESTEP_240HZ;
        inline constexpr math::Float MAX_TIMESTEP = 0.25f; // Prevent spiral of death

        // Substeps
        inline constexpr math::Int MIN_SUBSTEPS = 1;
        inline constexpr math::Int DEFAULT_SUBSTEPS = 1;
        inline constexpr math::Int MAX_SUBSTEPS = 10;

        // Solver iterations
        inline constexpr math::Int MIN_SOLVER_ITERATIONS = 4;
        inline constexpr math::Int DEFAULT_SOLVER_ITERATIONS = 10;
        inline constexpr math::Int MAX_SOLVER_ITERATIONS = 50;
        inline constexpr math::Int VEHICLE_SOLVER_ITERATIONS = 15; // More for vehicles

        // Error correction
        inline constexpr math::Float DEFAULT_ERP = 0.2f; // Error reduction parameter
        inline constexpr math::Float DEFAULT_ERP2 = 0.8f; // For split impulse
        inline constexpr math::Float DEFAULT_CFM = 0.0f; // Constraint force mixing
        inline constexpr math::Float SOFT_CFM = 0.01f; // For soft constraints
    }

    // ============================================================================
    // Collision Constants
    // ============================================================================

    namespace collision {
        // Collision margins
        inline constexpr math::Float DEFAULT_MARGIN = 0.04f; // meters
        inline constexpr math::Float THIN_MARGIN = 0.01f; // For precise objects
        inline constexpr math::Float THICK_MARGIN = 0.1f; // For rough objects

        // Contact thresholds
        inline constexpr math::Float CONTACT_BREAKING_THRESHOLD = 0.02f;
        inline constexpr math::Float CONTACT_PROCESSING_THRESHOLD = 0.0f;
        inline constexpr math::Float CONTACT_DISTANCE_TOLERANCE = 0.001f;
        inline constexpr math::Float PERSISTENT_CONTACT_DIST_THRESHOLD = 0.02f;

        // Penetration
        inline constexpr math::Float MAX_PENETRATION_DEPTH = 0.2f;
        inline constexpr math::Float PENETRATION_RECOVERY_SPEED = 0.1f;
        inline constexpr math::Float SPLIT_IMPULSE_PENETRATION_THRESHOLD = -0.04f;

        // CCD (Continuous Collision Detection)
        inline constexpr math::Float CCD_MOTION_THRESHOLD = 1.0f; // meters
        inline constexpr math::Float CCD_SWEPT_SPHERE_RADIUS = 0.2f;
        inline constexpr math::Float CCD_SQUARE_MOTION_THRESHOLD = CCD_MOTION_THRESHOLD * CCD_MOTION_THRESHOLD;

        // Collision layers/groups limits
        inline constexpr std::size_t MAX_COLLISION_GROUPS = 32;
        inline constexpr std::size_t MAX_COLLISION_PAIRS = 65536;
        inline constexpr std::size_t MAX_CONTACT_POINTS = 4;
        inline constexpr std::size_t MAX_MANIFOLDS = 4096;

        // Broadphase
        inline constexpr std::size_t DEFAULT_MAX_PROXIES = 65536;
        inline constexpr std::size_t SMALL_WORLD_MAX_PROXIES = 1024;
        inline constexpr std::size_t LARGE_WORLD_MAX_PROXIES = 262144;
        inline constexpr math::Float BROADPHASE_AABB_EXPANSION = 0.1f;
    }

    // ============================================================================
    // Material Constants
    // ============================================================================

    namespace material {
        // Friction coefficients
        inline constexpr math::Float FRICTION_ZERO = 0.0f;
        inline constexpr math::Float FRICTION_ICE = 0.05f;
        inline constexpr math::Float FRICTION_SLIPPERY = 0.2f;
        inline constexpr math::Float FRICTION_DEFAULT = 0.5f;
        inline constexpr math::Float FRICTION_RUBBER = 0.9f;
        inline constexpr math::Float FRICTION_STICKY = 1.5f;
        inline constexpr math::Float FRICTION_MAX = 2.0f;

        // Restitution (bounciness)
        inline constexpr math::Float RESTITUTION_NONE = 0.0f;
        inline constexpr math::Float RESTITUTION_WOOD = 0.2f;
        inline constexpr math::Float RESTITUTION_DEFAULT = 0.2f;
        inline constexpr math::Float RESTITUTION_BALL = 0.8f;
        inline constexpr math::Float RESTITUTION_SUPERBALL = 0.95f;
        inline constexpr math::Float RESTITUTION_MAX = 1.0f;

        // Damping
        inline constexpr math::Float DAMPING_NONE = 0.0f;
        inline constexpr math::Float DAMPING_DEFAULT_LINEAR = 0.05f;
        inline constexpr math::Float DAMPING_DEFAULT_ANGULAR = 0.05f;
        inline constexpr math::Float DAMPING_HIGH_LINEAR = 0.5f;
        inline constexpr math::Float DAMPING_HIGH_ANGULAR = 0.5f;
        inline constexpr math::Float DAMPING_WATER_LINEAR = 2.0f;
        inline constexpr math::Float DAMPING_WATER_ANGULAR = 1.0f;

        // Rolling/spinning friction
        inline constexpr math::Float ROLLING_FRICTION_DEFAULT = 0.01f;
        inline constexpr math::Float SPINNING_FRICTION_DEFAULT = 0.01f;

        // Density (kg/m³)
        inline constexpr math::Float DENSITY_AIR = 1.225f;
        inline constexpr math::Float DENSITY_WOOD = 700.0f;
        inline constexpr math::Float DENSITY_WATER = 1000.0f;
        inline constexpr math::Float DENSITY_HUMAN = 1062.0f;
        inline constexpr math::Float DENSITY_CONCRETE = 2400.0f;
        inline constexpr math::Float DENSITY_GLASS = 2500.0f;
        inline constexpr math::Float DENSITY_ALUMINUM = 2700.0f;
        inline constexpr math::Float DENSITY_STEEL = 7850.0f;
        inline constexpr math::Float DENSITY_LEAD = 11340.0f;
    }

    // ============================================================================
    // Body Constants
    // ============================================================================

    namespace body {
        // Mass limits
        inline constexpr math::Float MIN_MASS = 0.001f; // 1 gram minimum
        inline constexpr math::Float MAX_MASS = 100000.0f; // 100 tons maximum
        inline constexpr math::Float INFINITE_MASS = 0.0f; // Static body

        // Inertia
        inline constexpr math::Float MIN_INERTIA = 0.0001f;
        inline constexpr math::Float MAX_INERTIA = 10000.0f;

        // Sleep thresholds
        inline constexpr math::Float SLEEP_LINEAR_THRESHOLD = 0.8f;
        inline constexpr math::Float SLEEP_ANGULAR_THRESHOLD = 1.0f;
        inline constexpr math::Float SLEEP_TIME_THRESHOLD = 0.5f; // seconds
        inline constexpr math::Float WAKE_THRESHOLD = 0.0f;

        // Velocity limits
        inline constexpr math::Float MAX_LINEAR_VELOCITY = 1000.0f; // m/s
        inline constexpr math::Float MAX_ANGULAR_VELOCITY = 100.0f; // rad/s

        // Force/torque limits
        inline constexpr math::Float MAX_FORCE = 1000000.0f; // Newtons
        inline constexpr math::Float MAX_TORQUE = 100000.0f; // N⋅m

        // Body counts
        inline constexpr std::size_t DEFAULT_BODY_POOL_SIZE = 1024;
        inline constexpr std::size_t MAX_BODIES = 65536;
    }

    // ============================================================================
    // Character Controller Constants
    // ============================================================================

    namespace character {
        // Dimensions
        inline constexpr math::Float DEFAULT_RADIUS = 0.4f;
        inline constexpr math::Float DEFAULT_HEIGHT = 1.8f;
        inline constexpr math::Float DEFAULT_CROUCH_HEIGHT = 0.9f;
        inline constexpr math::Float MIN_RADIUS = 0.1f;
        inline constexpr math::Float MAX_RADIUS = 2.0f;
        inline constexpr math::Float MIN_HEIGHT = 0.5f;
        inline constexpr math::Float MAX_HEIGHT = 10.0f;

        // Movement
        inline constexpr math::Float WALK_SPEED = 4.0f; // m/s
        inline constexpr math::Float RUN_SPEED = 8.0f;
        inline constexpr math::Float SPRINT_SPEED = 12.0f;
        inline constexpr math::Float CROUCH_SPEED = 2.0f;
        inline constexpr math::Float SWIM_SPEED = 3.0f;
        inline constexpr math::Float CLIMB_SPEED = 2.0f;
        inline constexpr math::Float MAX_GROUND_SPEED = 20.0f;

        // Stepping
        inline constexpr math::Float DEFAULT_STEP_HEIGHT = 0.35f;
        inline constexpr math::Float MAX_STEP_HEIGHT = 1.0f;

        // Slopes
        inline constexpr math::Float MAX_SLOPE_ANGLE = 45.0f; // degrees
        inline constexpr math::Float SLIDE_THRESHOLD = 35.0f; // degrees
        inline constexpr math::Float MAX_CLIMBABLE_ANGLE = 60.0f; // degrees

        // Jumping
        inline constexpr math::Float DEFAULT_JUMP_HEIGHT = 2.0f;
        inline constexpr math::Float MAX_JUMP_HEIGHT = 10.0f;
        inline constexpr math::Float COYOTE_TIME = 0.15f; // Grace period
        inline constexpr math::Float JUMP_BUFFER_TIME = 0.1f; // Input buffering
        inline constexpr math::Int MAX_AIR_JUMPS = 2; // Double jump

        // Air control
        inline constexpr math::Float AIR_CONTROL_FACTOR = 0.3f;
        inline constexpr math::Float AIR_ACCELERATION = 10.0f;
        inline constexpr math::Float GROUND_ACCELERATION = 50.0f;

        // Push force
        inline constexpr math::Float DEFAULT_PUSH_POWER = 2.0f;
        inline constexpr math::Float MAX_PUSH_FORCE = 1000.0f;
    }

    // ============================================================================
    // Vehicle Constants
    // ============================================================================

    namespace vehicle {
        // Wheels
        inline constexpr math::Int MIN_WHEELS = 2;
        inline constexpr math::Int MAX_WHEELS = 8;
        inline constexpr math::Float DEFAULT_WHEEL_RADIUS = 0.4f;
        inline constexpr math::Float DEFAULT_WHEEL_WIDTH = 0.3f;

        // Suspension
        inline constexpr math::Float DEFAULT_SUSPENSION_STIFFNESS = 20.0f;
        inline constexpr math::Float DEFAULT_SUSPENSION_DAMPING = 2.3f;
        inline constexpr math::Float DEFAULT_SUSPENSION_COMPRESSION = 4.4f;
        inline constexpr math::Float DEFAULT_SUSPENSION_REST_LENGTH = 0.6f;
        inline constexpr math::Float MAX_SUSPENSION_TRAVEL = 0.5f;
        inline constexpr math::Float MAX_SUSPENSION_FORCE = 6000.0f;

        // Friction
        inline constexpr math::Float DEFAULT_WHEEL_FRICTION = 1000.0f;
        inline constexpr math::Float SLIP_THRESHOLD = 2.0f;

        // Engine
        inline constexpr math::Float DEFAULT_ENGINE_FORCE = 2000.0f;
        inline constexpr math::Float DEFAULT_BRAKE_FORCE = 1000.0f;
        inline constexpr math::Float MAX_ENGINE_FORCE = 10000.0f;
        inline constexpr math::Float MAX_BRAKE_FORCE = 5000.0f;

        // Steering
        inline constexpr math::Float DEFAULT_STEERING_ANGLE = 0.3f; // radians
        inline constexpr math::Float MAX_STEERING_ANGLE = 0.6f;
        inline constexpr math::Float STEERING_INCREMENT = 0.04f;
        inline constexpr math::Float STEERING_CLAMP = 0.3f;

        // Speed limits
        inline constexpr math::Float MAX_VEHICLE_SPEED = 200.0f; // m/s (~720 km/h)
        inline constexpr math::Float REVERSE_SPEED_RATIO = 0.3f;
    }

    // ============================================================================
    // Constraint Constants
    // ============================================================================

    namespace constraint {
        // Limits
        inline constexpr std::size_t MAX_CONSTRAINTS = 4096;
        inline constexpr math::Float DEFAULT_BREAKING_IMPULSE = math::INFINITY_VALUE<math::Float>;
        inline constexpr math::Float SOFT_CONSTRAINT_ERP = 0.1f;
        inline constexpr math::Float SOFT_CONSTRAINT_CFM = 0.01f;

        // Motor
        inline constexpr math::Float DEFAULT_MOTOR_MAX_FORCE = 1000.0f;
        inline constexpr math::Float DEFAULT_MOTOR_TARGET_VELOCITY = 1.0f;
        inline constexpr math::Float MOTOR_ERP = 0.9f;

        // Spring
        inline constexpr math::Float DEFAULT_SPRING_STIFFNESS = 100.0f;
        inline constexpr math::Float DEFAULT_SPRING_DAMPING = 10.0f;

        // Hinge
        inline constexpr math::Float HINGE_DEFAULT_SOFTNESS = 0.9f;
        inline constexpr math::Float HINGE_DEFAULT_BIAS = 0.3f;
        inline constexpr math::Float HINGE_DEFAULT_RELAXATION = 1.0f;

        // Distance
        inline constexpr math::Float MIN_DISTANCE = 0.001f;
        inline constexpr math::Float MAX_DISTANCE = 1000.0f;
    }

    // ============================================================================
    // Query Constants
    // ============================================================================

    namespace query {
        // Ray casting
        inline constexpr math::Float DEFAULT_RAY_LENGTH = 1000.0f;
        inline constexpr math::Float MAX_RAY_LENGTH = 100000.0f;
        inline constexpr std::size_t MAX_RAY_HITS = 256;

        // Overlap queries
        inline constexpr std::size_t MAX_OVERLAP_RESULTS = 256;
        inline constexpr math::Float DEFAULT_OVERLAP_RADIUS = 1.0f;

        // Sweep tests
        inline constexpr math::Float DEFAULT_SWEEP_DISTANCE = 10.0f;
        inline constexpr math::Float SWEEP_EPSILON = 0.001f;

        // Contact tests
        inline constexpr math::Float CONTACT_TEST_DISTANCE = 0.1f;
        inline constexpr std::size_t MAX_CONTACT_RESULTS = 64;
    }

    // ============================================================================
    // Performance Constants
    // ============================================================================

    namespace performance {
        // LOD distances
        inline constexpr math::Float LOD_DISTANCE_HIGH = 10.0f;
        inline constexpr math::Float LOD_DISTANCE_MEDIUM = 50.0f;
        inline constexpr math::Float LOD_DISTANCE_LOW = 100.0f;
        inline constexpr math::Float LOD_DISTANCE_CULL = 200.0f;

        // Update frequencies
        inline constexpr math::Float HIGH_FREQUENCY_UPDATE = 60.0f;
        inline constexpr math::Float MEDIUM_FREQUENCY_UPDATE = 30.0f;
        inline constexpr math::Float LOW_FREQUENCY_UPDATE = 10.0f;

        // Island thresholds
        inline constexpr std::size_t MIN_ISLAND_SIZE = 2;
        inline constexpr std::size_t MAX_ISLAND_SIZE = 1024;
        inline constexpr math::Float ISLAND_SPLIT_IMPULSE = 0.01f;

        // Threading
        inline constexpr math::Int DEFAULT_THREAD_COUNT = 4;
        inline constexpr math::Int MAX_THREAD_COUNT = 16;
        inline constexpr std::size_t MIN_BODIES_PER_THREAD = 10;

        // Memory pools
        inline constexpr std::size_t DEFAULT_SHAPE_POOL_SIZE = 2048;
        inline constexpr std::size_t DEFAULT_CONSTRAINT_POOL_SIZE = 512;
        inline constexpr std::size_t DEFAULT_MANIFOLD_POOL_SIZE = 4096;
    }

    // ============================================================================
    // Debug Constants
    // ============================================================================

    namespace debug {
        // Debug draw
        inline constexpr math::Float DEBUG_DRAW_SIZE = 0.1f;
        inline constexpr math::Float DEBUG_ARROW_SIZE = 0.2f;
        inline constexpr math::Float DEBUG_TEXT_SIZE = 1.0f;
        inline constexpr math::Float DEBUG_CONTACT_POINT_SIZE = 0.05f;
        inline constexpr math::Float DEBUG_NORMAL_LENGTH = 0.5f;

        // Debug colors (as Vec4 for RGBA)
        inline const auto COLOR_STATIC_BODY = math::Vec4(0.5f, 0.5f, 0.5f, 1.0f);
        inline const auto COLOR_KINEMATIC_BODY = math::Vec4(0.0f, 0.0f, 1.0f, 1.0f);
        inline const auto COLOR_DYNAMIC_BODY = math::Vec4(0.0f, 1.0f, 0.0f, 1.0f);
        inline const auto COLOR_SLEEPING_BODY = math::Vec4(1.0f, 1.0f, 0.0f, 1.0f);
        inline const auto COLOR_CONTACT_POINT = math::Vec4(1.0f, 0.0f, 0.0f, 1.0f);
        inline const auto COLOR_CONSTRAINT = math::Vec4(1.0f, 0.0f, 1.0f, 1.0f);
        inline const auto COLOR_AABB = math::Vec4(0.0f, 1.0f, 1.0f, 1.0f);

        // Profiling
        inline constexpr std::size_t PROFILER_SAMPLE_COUNT = 60;
        inline constexpr math::Float PROFILER_UPDATE_INTERVAL = 0.5f;
    }
} // namespace engine::physics
