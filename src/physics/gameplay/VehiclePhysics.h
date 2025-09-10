/**
 * @file VehiclePhysics.h
 * @brief Advanced vehicle physics simulation
 * @details Implements realistic vehicle dynamics including wheels, suspension,
 *          engine, transmission, and various vehicle types (cars, boats, flying)
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

// #include "../core/PhysicsTypes.h"
// #include "../core/PhysicsConstants.h"
// #include "../core/PhysicsWorld.h"
#include "../dynamics/RigidBody.h"
// #include <BulletDynamics/Vehicle/btRaycastVehicle.h>

// TODO: In the future, move it to the module manager, as it is a preset
namespace engine::physics {
    /**
     * @brief Vehicle configuration parameters
     */
    struct VehicleParams {
        // Chassis
        Vec3 chassisSize = Vec3(2.0f, 1.0f, 4.0f);
        Float chassisMass = 1500.0f; // kg
        Vec3 centerOfMass = Vec3(0, -0.5f, 0);

        // Wheels
        struct WheelParams {
            Vec3 position; // Relative to chassis
            Float radius = vehicle::DEFAULT_WHEEL_RADIUS;
            Float width = vehicle::DEFAULT_WHEEL_WIDTH;
            bool isFrontWheel = false;
            bool isDriveWheel = false;
            bool isSteerWheel = false;

            // Suspension
            Float suspensionStiffness = vehicle::DEFAULT_SUSPENSION_STIFFNESS;
            Float suspensionDamping = vehicle::DEFAULT_SUSPENSION_DAMPING;
            Float suspensionCompression = vehicle::DEFAULT_SUSPENSION_COMPRESSION;
            Float suspensionRestLength = vehicle::DEFAULT_SUSPENSION_REST_LENGTH;
            Float maxSuspensionTravel = vehicle::MAX_SUSPENSION_TRAVEL;
            Float maxSuspensionForce = vehicle::MAX_SUSPENSION_FORCE;

            // Friction
            Float frictionSlip = vehicle::DEFAULT_WHEEL_FRICTION;
            Float rollInfluence = 0.1f;
        };

        std::vector<WheelParams> wheels;

        // Engine
        Float maxEngineForce = vehicle::DEFAULT_ENGINE_FORCE;
        Float maxBrakeForce = vehicle::DEFAULT_BRAKE_FORCE;
        Float engineTorqueCurve[8] = {0.5f, 0.7f, 0.9f, 1.0f, 1.0f, 0.9f, 0.8f, 0.6f};
        Float redlineRPM = 7000.0f;
        Float idleRPM = 800.0f;

        // Transmission
        Int numGears = 6;
        Float gearRatios[8] = {-3.5f, 0.0f, 3.5f, 2.5f, 1.8f, 1.3f, 1.0f, 0.8f}; // R,N,1,2,3,4,5,6
        Float finalDriveRatio = 3.5f;
        Float clutchStrength = 10.0f;

        // Steering
        Float maxSteeringAngle = vehicle::DEFAULT_STEERING_ANGLE;
        Float steeringSpeed = 2.0f; // rad/s
        Float steeringReturnSpeed = 4.0f;
        bool powerSteering = true;
        Float steeringAssist = 0.5f; // 0-1

        // Aerodynamics
        Float dragCoefficient = 0.3f;
        Float downforceCoefficient = 0.1f;
        Float frontalArea = 2.2f; // m²

        // Traction control
        bool absEnabled = true;
        bool tcsEnabled = true;
        bool escEnabled = true;
        Float absThreshold = 0.8f;
        Float tcsThreshold = 0.9f;

        // Sound (for game integration)
        std::string engineSoundPath;
        std::string tireSoundPath;
        std::string brakeSoundPath;

        VehicleParams() {
            // Default 4-wheel car setup
            wheels.resize(4);

            // Front left
            wheels[0].position = Vec3(-0.8f, -0.3f, 1.2f);
            wheels[0].isFrontWheel = true;
            wheels[0].isSteerWheel = true;

            // Front right
            wheels[1].position = Vec3(0.8f, -0.3f, 1.2f);
            wheels[1].isFrontWheel = true;
            wheels[1].isSteerWheel = true;

            // Rear left
            wheels[2].position = Vec3(-0.8f, -0.3f, -1.2f);
            wheels[2].isDriveWheel = true;

            // Rear right
            wheels[3].position = Vec3(0.8f, -0.3f, -1.2f);
            wheels[3].isDriveWheel = true;
        }

        // Preset configurations
        static VehicleParams SportsCar() {
            VehicleParams params;
            params.chassisMass = 1200.0f;
            params.maxEngineForce = 3000.0f;
            params.maxBrakeForce = 1500.0f;
            params.dragCoefficient = 0.28f;
            params.downforceCoefficient = 0.3f;
            return params;
        }

        static VehicleParams SUV() {
            VehicleParams params;
            params.chassisSize = Vec3(2.2f, 1.8f, 4.5f);
            params.chassisMass = 2200.0f;
            params.centerOfMass = Vec3(0, -0.3f, 0);
            params.maxEngineForce = 2500.0f;
            // All-wheel drive
            for (auto& wheel : params.wheels) {
                wheel.isDriveWheel = true;
            }
            return params;
        }

        static VehicleParams Motorcycle() {
            VehicleParams params;
            params.wheels.resize(2);
            params.chassisSize = Vec3(0.5f, 1.0f, 2.0f);
            params.chassisMass = 200.0f;

            // Front wheel
            params.wheels[0].position = Vec3(0, -0.3f, 0.8f);
            params.wheels[0].isFrontWheel = true;
            params.wheels[0].isSteerWheel = true;

            // Rear wheel
            params.wheels[1].position = Vec3(0, -0.3f, -0.8f);
            params.wheels[1].isDriveWheel = true;

            return params;
        }
    };

    /**
     * @brief Vehicle state for networking/replay
     */
    struct VehicleState {
        // TODO: Ver parametros que no se usan
        // Transform
        Vec3 position;
        Quat rotation;
        Vec3 linearVelocity;
        Vec3 angularVelocity;

        // Wheels
        struct WheelState {
            Float rotation;
            Float steering;
            Float suspensionLength;
            bool isGrounded;
            Float slipRatio;
        };

        std::vector<WheelState> wheelStates;

        // Engine
        Float engineRPM;
        Int currentGear;
        Float throttle;
        Float brake;
        Float handbrake;
        Float clutch;

        // Physics
        Float speed;
        Float lateralG;
        Float longitudinalG;

        VehicleState() : position(VEC3_ZERO), rotation(QUAT_IDENTITY), linearVelocity(VEC3_ZERO), angularVelocity(),
                         engineRPM(0), currentGear(1),
                         throttle(0),
                         brake(0),
                         handbrake(0), clutch(1), speed(0), lateralG(0), longitudinalG(0) {
        }
    };

    /**
     * @brief Advanced vehicle physics controller
     * @details Implements realistic vehicle dynamics with engine simulation,
     *          transmission, differential, and various driving assists
     */
    class VehiclePhysics {
    public:
        VehiclePhysics(const VehicleParams& params, PhysicsWorld* world);

        ~VehiclePhysics();

        // ============================================================================
        // Initialization
        // ============================================================================

        void initialize();

        // ============================================================================
        // Control Interface
        // ============================================================================

        /**
         * @brief Set throttle input (0-1)
         */
        void setThrottle(Float throttle);

        /**
         * @brief Set brake input (0-1)
         */
        void setBrake(Float brake);

        /**
         * @brief Set handbrake input (0-1)
         */
        void setHandbrake(Float handbrake);

        /**
         * @brief Set steering input (-1 to 1)
         */
        void setSteering(const Float steering) {
            steeringInput_ = clamp(steering, -1.0f, 1.0f);
        }

        /**
         * @brief Shift gear
         */
        void shiftUp() {
            if (currentGear_ < params_.numGears - 1) {
                currentGear_++;
                clutch_ = 0.0f; // Disengage clutch during shift
            }
        }

        void shiftDown() {
            if (currentGear_ > 0) {
                // 0 is reverse
                currentGear_--;
                clutch_ = 0.0f;
            }
        }

        void setGear(const Int gear) {
            currentGear_ = clamp(gear, 0, params_.numGears - 1);
            clutch_ = 0.0f;
        }

        void setChassisBody(RigidBody* body) { chassisBody_ = body; }

        /**
         * @brief Reset vehicle to position
         */
        void reset(const Vec3& position, const Quat& rotation = QUAT_IDENTITY);

        // ============================================================================
        // Update
        // ============================================================================

        /**
         * @brief Update vehicle physics
         */
        void update(Float deltaTime);

        // ============================================================================
        // Queries
        // ============================================================================

        [[nodiscard]] Vec3 getPosition() const {
            return chassisBody_ ? chassisBody_->getPosition() : VEC3_ZERO;
        }

        [[nodiscard]] Quat getRotation() const {
            return chassisBody_ ? chassisBody_->getRotation() : QUAT_IDENTITY;
        }

        [[nodiscard]] Vec3 getVelocity() const {
            return chassisBody_ ? chassisBody_->getLinearVelocity() : VEC3_ZERO;
        }

        [[nodiscard]] Float getSpeed() const {
            return vehicle_ ? vehicle_->getCurrentSpeedKmHour() / 3.6f : 0.0f; // m/s
        }

        [[nodiscard]] Float getEngineRPM() const { return engineRPM_; }
        [[nodiscard]] Int getCurrentGear() const { return currentGear_; }

        [[nodiscard]] const VehicleState& getState() const { return state_; }

        /**
         * @brief Check if vehicle is grounded
         */
        [[nodiscard]] bool isGrounded() const;

        /**
         * @brief Get wheel transformation for rendering
         */
        [[nodiscard]] Mat4 getWheelTransform(Int wheelIndex) const;

    private:
        VehicleParams params_;
        PhysicsWorld* world_;

        // Bullet objects
        btRaycastVehicle* vehicle_;
        btRaycastVehicle::btVehicleTuning tuning_;
        btDefaultVehicleRaycaster* vehicleRaycaster_;
        RigidBody* chassisBody_;


        // Control state
        Float throttle_;
        Float brake_;
        Float handbrake_;
        Float steering_;
        Float steeringInput_;
        Float clutch_;

        // Engine state
        Float engineRPM_;
        Int currentGear_;
        Float engineTorque_;

        // Vehicle state
        VehicleState state_;

        void cleanup();

        void addWheel(const VehicleParams::WheelParams& wheelParams) const;

        // ============================================================================
        // Physics Updates
        // ============================================================================

        void updateSteering(Float deltaTime);

        void updateEngine();

        void updateTransmission(Float deltaTime);

        void applyDriveForce() const;

        void applyBrakes() const;

        void applyHandbrake() const;

        void applyAerodynamics() const;

        // ============================================================================
        // Traction Control Systems
        // ============================================================================

        void updateABS();

        void updateTCS() const;

        void updateESC();

        // ============================================================================
        // Helper Functions
        // ============================================================================

        [[nodiscard]] Float getWheelRPM() const;

        [[nodiscard]] Float calculateSlipRatio(Int wheelIndex) const;

        [[nodiscard]] Float calculateLateralSlip() const;

        void updateState();

        bool autoShift_;
        Float absTimer_ = 0;
    };
} // namespace engine::physics
