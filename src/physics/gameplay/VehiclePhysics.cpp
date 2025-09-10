/**
 * @file VehiclePhysics.cpp
 * @brief Advanced vehicle physics simulation
 * @details Implements realistic vehicle dynamics including wheels, suspension,
 *          engine, transmission, and various vehicle types (cars, boats, flying)
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#include "VehiclePhysics.h"

#include "../core/PhysicsWorld.h"

namespace engine::physics {
    VehiclePhysics::VehiclePhysics(const VehicleParams& params, PhysicsWorld* world)
            : params_(params), world_(world), vehicle_(nullptr), vehicleRaycaster_(nullptr),
              chassisBody_(nullptr), throttle_(0.0f),
              brake_(0.0f), handbrake_(0.0f), steering_(0.0f), steeringInput_(0),
              clutch_(1.0f), engineRPM_(params.idleRPM), currentGear_(1), engineTorque_(0), autoShift_(false) {
    }

    VehiclePhysics::~VehiclePhysics() {
        cleanup();
    }

    void VehiclePhysics::setThrottle(const Float throttle) {
        throttle_ = saturate(throttle);
        updateEngine();
    }

    void VehiclePhysics::setBrake(const Float brake) {
        brake_ = saturate(brake);
        applyBrakes();
    }

    void VehiclePhysics::setHandbrake(const Float handbrake) {
        handbrake_ = saturate(handbrake);
        applyHandbrake();
    }

    void VehiclePhysics::reset(const Vec3& position, const Quat& rotation) {
        if (chassisBody_) {
            chassisBody_->setPosition(position);
            chassisBody_->setRotation(rotation);
            chassisBody_->setLinearVelocity(Vec3(0));
            chassisBody_->setAngularVelocity(Vec3(0));
        }

        engineRPM_ = params_.idleRPM;
        currentGear_ = 1;
        throttle_ = 0;
        brake_ = 0;
        steering_ = 0;

        if (vehicle_) {
            vehicle_->resetSuspension();
        }
    }

    void VehiclePhysics::update(const Float deltaTime) {
        if (!vehicle_) return;

        // Update steering with smoothing
        updateSteering(deltaTime);

        // Update engine and transmission
        updateEngine();
        updateTransmission(deltaTime);

        // Apply driving forces
        applyDriveForce();

        // Update traction control systems
        if (params_.absEnabled) updateABS();
        if (params_.tcsEnabled) updateTCS();
        if (params_.escEnabled) updateESC();

        // Apply aerodynamics
        applyAerodynamics();

        // Update state for networking/sound
        updateState();
    }

    bool VehiclePhysics::isGrounded() const {
        if (!vehicle_) return false;

        for (int i = 0; i < vehicle_->getNumWheels(); ++i) {
            if (vehicle_->getWheelInfo(i).m_raycastInfo.m_isInContact) {
                return true;
            }
        }
        return false;
    }

    Mat4 VehiclePhysics::getWheelTransform(const Int wheelIndex) const {
        if (!vehicle_ || wheelIndex >= vehicle_->getNumWheels()) {
            return Mat4(1.0f);
        }

        btTransform transform;
        vehicle_->updateWheelTransform(wheelIndex, true);
        transform = vehicle_->getWheelInfo(wheelIndex).m_worldTransform;

        btScalar matrix[16];
        transform.getOpenGLMatrix(matrix);

        return Mat4(
            matrix[0], matrix[1], matrix[2], matrix[3],
            matrix[4], matrix[5], matrix[6], matrix[7],
            matrix[8], matrix[9], matrix[10], matrix[11],
            matrix[12], matrix[13], matrix[14], matrix[15]
        );
    }

    void VehiclePhysics::initialize() {
        // Create chassis rigid body
        if (!chassisBody_) return;

        // Create vehicle raycaster
        vehicleRaycaster_ = new btDefaultVehicleRaycaster(
            world_->getBulletWorld()
        );

        // Create vehicle
        vehicle_ = new btRaycastVehicle(tuning_,
                                         chassisBody_->getBulletBody(),
                                         vehicleRaycaster_);

        // Configure vehicle
        vehicle_->setCoordinateSystem(0, 1, 2); // Y-up

        // Add wheels
        for (const auto& wheelParams : params_.wheels) {
            addWheel(wheelParams);
        }

        // Add to world
        world_->getBulletWorld()->addVehicle(vehicle_);

        // Initialize state
        state_.wheelStates.resize(params_.wheels.size());
    }

    void VehiclePhysics::cleanup() {
        if (world_ && world_->getBulletWorld()) {
            if (vehicle_) {
                world_->getBulletWorld()->removeVehicle(vehicle_);
                delete vehicle_;
                vehicle_ = nullptr;
            }
        }

        delete vehicleRaycaster_;
        vehicleRaycaster_ = nullptr;

        if (chassisBody_) {
            // Destroy through physics manager
            chassisBody_ = nullptr;
        }
    }

    void VehiclePhysics::addWheel(const VehicleParams::WheelParams& wheelParams) const {
        const btVector3 wheelDirection(0, -1, 0);
        const btVector3 wheelAxle(-1, 0, 0);

        btWheelInfo& wheel = vehicle_->addWheel(
            btVector3(wheelParams.position.x,
                      wheelParams.position.y,
                      wheelParams.position.z),
            wheelDirection,
            wheelAxle,
            wheelParams.suspensionRestLength,
            wheelParams.radius,
            tuning_,
            wheelParams.isFrontWheel
        );

        wheel.m_suspensionStiffness = wheelParams.suspensionStiffness;
        wheel.m_wheelsDampingRelaxation = wheelParams.suspensionDamping;
        wheel.m_wheelsDampingCompression = wheelParams.suspensionCompression;
        wheel.m_frictionSlip = wheelParams.frictionSlip;
        wheel.m_rollInfluence = wheelParams.rollInfluence;
        // TODO: ver esto pq da error
        // wheel.m_maxSuspensionTravel = wheelParams.maxSuspensionTravel;
        wheel.m_maxSuspensionTravelCm = wheelParams.maxSuspensionTravel;
        wheel.m_maxSuspensionForce = wheelParams.maxSuspensionForce;
    }

    void VehiclePhysics::updateSteering(const Float deltaTime) {
        // Smooth steering input
        const Float targetSteering = steeringInput_ * params_.maxSteeringAngle;

        const Float steeringSpeed = steeringInput_ == 0 ? params_.steeringReturnSpeed : params_.steeringSpeed;

        steering_ = lerp(steering_, targetSteering, steeringSpeed * deltaTime);

        // Apply speed-sensitive steering
        const Float speedFactor = 1.0f - saturate(getSpeed() / 50.0f) * 0.7f;
        const Float actualSteering = steering_ * speedFactor;

        // Set steering on wheels
        for (int i = 0; i < vehicle_->getNumWheels(); ++i) {
            if (params_.wheels[i].isSteerWheel) {
                vehicle_->setSteeringValue(actualSteering, i);
            }
        }
    }

    void VehiclePhysics::updateEngine() {
        // Simple engine simulation
        const Float targetRPM = params_.idleRPM + throttle_ *
            (params_.redlineRPM - params_.idleRPM);

        // Consider load from wheels
        const Float wheelRPM = getWheelRPM();
        const Float gearRatio = params_.gearRatios[currentGear_ + 1]; // +1 for reverse offset
        const Float engineRPMFromWheels = wheelRPM * gearRatio * params_.finalDriveRatio;

        // Blend based on clutch engagement
        const Float clutchFactor = clutch_;
        engineRPM_ = lerp(targetRPM, engineRPMFromWheels, clutchFactor);

        // Clamp to limits
        engineRPM_ = clamp(engineRPM_, params_.idleRPM, params_.redlineRPM);

        // Calculate torque from torque curve
        const Float rpmNormalized = (engineRPM_ - params_.idleRPM) /
            (params_.redlineRPM - params_.idleRPM);
        Int curveIndex = static_cast<Int>(rpmNormalized * 7);
        curveIndex = clamp(curveIndex, 0, 7);

        const Float torqueFactor = params_.engineTorqueCurve[curveIndex];
        engineTorque_ = params_.maxEngineForce * torqueFactor * throttle_;
    }

    void VehiclePhysics::updateTransmission(const Float deltaTime) {
        // Auto-clutch engagement
        if (clutch_ < 1.0f) {
            clutch_ = min(clutch_ + deltaTime * 2.0f, 1.0f);
        }

        // TODO: Ver esto sobre m_autoShift ya que es el unico lugar que se usa y no esta inicializado
        // Auto-shift (optional)
        if (autoShift_) {
            if (engineRPM_ > params_.redlineRPM * 0.9f &&
                currentGear_ < params_.numGears - 1) {
                shiftUp();
            }
            else if (engineRPM_ < params_.idleRPM * 1.5f &&
                currentGear_ > 1) {
                shiftDown();
            }
        }
    }

    void VehiclePhysics::applyDriveForce() const {
        // TODO: Revisar esto
        // if (!m_vehicle) return;

        const Float gearRatio = params_.gearRatios[currentGear_ + 1];
        const Float totalRatio = gearRatio * params_.finalDriveRatio;
        const Float driveForce = engineTorque_ * totalRatio * clutch_;

        // Apply to drive wheels
        for (int i = 0; i < vehicle_->getNumWheels(); ++i) {
            if (params_.wheels[i].isDriveWheel) {
                vehicle_->applyEngineForce(driveForce, i);
            }
        }
    }

    void VehiclePhysics::applyBrakes() const {
        if (!vehicle_) return;

        const Float brakeForce = brake_ * params_.maxBrakeForce;

        for (int i = 0; i < vehicle_->getNumWheels(); ++i) {
            // Front/rear brake bias (60/40 typical)
            const Float bias = params_.wheels[i].isFrontWheel ? 0.6f : 0.4f;
            vehicle_->setBrake(brakeForce * bias, i);
        }
    }

    void VehiclePhysics::applyHandbrake() const {
        if (!vehicle_) return;

        const Float brakeForce = handbrake_ * params_.maxBrakeForce * 0.5f;

        // Handbrake typically only on rear wheels
        for (int i = 0; i < vehicle_->getNumWheels(); ++i) {
            if (!params_.wheels[i].isFrontWheel) {
                vehicle_->setBrake(brakeForce, i);
            }
        }
    }

    void VehiclePhysics::applyAerodynamics() const {
        if (!chassisBody_) return;

        const Vec3 velocity = chassisBody_->getLinearVelocity();

        if (const Float speed = glm::length(velocity); speed > 0.1f) {
            const Vec3 direction = velocity / speed;

            // Drag force: F = 0.5 * ρ * v² * Cd * A
            const Float dragForce = 0.5f * world::AIR_DENSITY * speed * speed *
                params_.dragCoefficient * params_.frontalArea;

            chassisBody_->applyForce(-direction * dragForce);

            // Downforce (simplified)
            const Float downforce = 0.5f * world::AIR_DENSITY * speed * speed *
                params_.downforceCoefficient * params_.frontalArea;

            chassisBody_->applyForce(Vec3(0, -downforce, 0));
        }
    }

    void VehiclePhysics::updateABS() {
        // Anti-lock braking system
        for (int i = 0; i < vehicle_->getNumWheels(); ++i) {
            const btWheelInfo& wheel = vehicle_->getWheelInfo(i);

            // Check for wheel lock
            if (const Float slipRatio = calculateSlipRatio(i); std::abs(slipRatio) > params_.absThreshold && brake_ > 0) {
                // Pulse brakes
                const Float pulseFactor = std::sin(absTimer_ * 30.0f) * 0.5f + 0.5f;
                const Float adjustedBrake = brake_ * pulseFactor;

                const Float brakeForce = adjustedBrake * params_.maxBrakeForce;
                const Float bias = wheel.m_bIsFrontWheel ? 0.6f : 0.4f;
                vehicle_->setBrake(brakeForce * bias, i);
            }
        }

        absTimer_ += 1.0f / 60.0f; // Assume 60Hz update
    }

    void VehiclePhysics::updateTCS() const {
        // Traction control system
        for (int i = 0; i < vehicle_->getNumWheels(); ++i) {
            if (!params_.wheels[i].isDriveWheel) continue;

            if (const Float slipRatio = calculateSlipRatio(i); slipRatio > params_.tcsThreshold) {
                // Reduce engine power
                const Float reduction = (slipRatio - params_.tcsThreshold) /
                    (1.0f - params_.tcsThreshold);
                const Float adjustedForce = engineTorque_ * (1.0f - reduction);

                vehicle_->applyEngineForce(adjustedForce, i);
            }
        }
    }

    void VehiclePhysics::updateESC() {
        // Electronic stability control
        if (!chassisBody_) return;

        // TODO: Ver esto
        Vec3 velocity = chassisBody_->getLinearVelocity();
        Vec3 angularVelocity = chassisBody_->getAngularVelocity();

        // Detect oversteer/understeer

        if (const Float lateralSlip = calculateLateralSlip(); std::abs(lateralSlip) > 0.2f) {
            // Apply corrective torque
            const Float correction = -lateralSlip * 1000.0f;
            chassisBody_->applyTorque(Vec3(0, correction, 0));

            // Reduce engine power
            engineTorque_ *= 0.7f;
        }
    }

    Float VehiclePhysics::getWheelRPM() const {
        if (!vehicle_) return 0;

        Float totalRPM = 0;
        Int driveWheelCount = 0;

        for (int i = 0; i < vehicle_->getNumWheels(); ++i) {
            if (params_.wheels[i].isDriveWheel) {
                const btWheelInfo& wheel = vehicle_->getWheelInfo(i);
                const Float wheelRPM = wheel.m_deltaRotation * 60.0f /
                    (TWO_PI<Float> * (1.0f / 60.0f));
                totalRPM += wheelRPM;
                driveWheelCount++;
            }
        }

        return driveWheelCount > 0 ? totalRPM / driveWheelCount : 0;
    }

    Float VehiclePhysics::calculateSlipRatio(const Int wheelIndex) const {
        // TODO: Revisar esto
        // if (!m_vehicle || wheelIndex >= m_vehicle->getNumWheels()) return 0;

        const btWheelInfo& wheel = vehicle_->getWheelInfo(wheelIndex);

        Float wheelVelocity = wheel.m_deltaRotation * wheel.m_wheelsRadius;
        Float groundVelocity = vehicle_->getCurrentSpeedKmHour() / 3.6f;

        if (std::abs(groundVelocity) < 0.1f) {
            return wheelVelocity > 0.1f ? 1.0f : 0.0f;
        }

        return (wheelVelocity - groundVelocity) / std::abs(groundVelocity);
    }

    Float VehiclePhysics::calculateLateralSlip() const {
        // TODO: Revisar esto
        // if (!m_chassisBody) return 0;

        const Vec3 velocity = chassisBody_->getLinearVelocity();
        const Quat rotation = chassisBody_->getRotation();

        // Transform velocity to local space
        const Mat4 worldToLocal = glm::inverse(glm::mat4_cast(rotation));

        // Calculate slip angle
        if (const auto localVelocity = Vec3(worldToLocal * Vec4(velocity, 0)); std::abs(localVelocity.z) > 0.1f) {
            return std::atan2(localVelocity.x, std::abs(localVelocity.z));
        }

        return 0;
    }

    void VehiclePhysics::updateState() {
        // TODO: Revisar esto
        // if (!m_vehicle || !m_chassisBody) return;

        // Update basic state
        state_.position = chassisBody_->getPosition();
        state_.rotation = chassisBody_->getRotation();
        state_.linearVelocity = chassisBody_->getLinearVelocity();
        state_.angularVelocity = chassisBody_->getAngularVelocity();

        // Update wheel states
        for (int i = 0; i < vehicle_->getNumWheels(); ++i) {
            const btWheelInfo& wheel = vehicle_->getWheelInfo(i);

            state_.wheelStates[i].rotation = wheel.m_rotation;
            state_.wheelStates[i].steering = wheel.m_steering;
            state_.wheelStates[i].suspensionLength = wheel.m_raycastInfo.m_suspensionLength;
            state_.wheelStates[i].isGrounded = wheel.m_raycastInfo.m_isInContact;
            state_.wheelStates[i].slipRatio = calculateSlipRatio(i);
        }

        // Update engine state
        state_.engineRPM = engineRPM_;
        state_.currentGear = currentGear_;
        state_.throttle = throttle_;
        state_.brake = brake_;
        state_.handbrake = handbrake_;
        state_.clutch = clutch_;

        // Update physics state
        state_.speed = getSpeed();

        // Calculate G-forces
        static Vec3 lastVelocity = state_.linearVelocity;
        const Vec3 acceleration = (state_.linearVelocity - lastVelocity) * 60.0f; // Assume 60Hz
        lastVelocity = state_.linearVelocity;

        const Mat4 worldToLocal = glm::inverse(glm::mat4_cast(state_.rotation));
        const auto localAccel = Vec3(worldToLocal * Vec4(acceleration, 0));

        state_.lateralG = localAccel.x / world::GRAVITY_EARTH;
        state_.longitudinalG = localAccel.z / world::GRAVITY_EARTH;
    }
} // namespace engine::physics
