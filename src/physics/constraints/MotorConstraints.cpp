/**
 * @file MotorConstraints.cpp
 * @brief Advanced motor constraints for robotics and mechanical simulations
 * @details Implements various motor types including servo, stepper, and PID-controlled motors
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#include "MotorConstraints.h"

namespace engine::physics {
    Float PIDController::update(const Float error, const Float deltaTime) {
        // Proportional term
        const Float p = kp_ * error;

        // Integral term with anti-windup
        integral_ += error * deltaTime;
        integral_ = clamp(integral_, -integralMax_, integralMax_);
        const Float i = ki_ * integral_;

        // Derivative term with filtering
        Float derivative = (error - lastError_) / deltaTime;
        derivative = derivativeFilter_ * derivative + (1.0f - derivativeFilter_) * lastDerivative_;
        const Float d = kd_ * derivative;

        lastError_ = error;
        lastDerivative_ = derivative;

        // Calculate output with clamping
        const Float output = p + i + d;
        return clamp(output, -outputMax_, outputMax_);
    }

    void MotorConstraint::update(const Float deltaTime) {
        const Float currentAngle = getHingeAngle();
        const Float currentVelocity = getAngularVelocity();

        Float outputTorque = 0.0f;

        switch (motorParams_.controlMode) {
        case MotorControlMode::POSITION:
            outputTorque = updatePositionControl(currentAngle, currentVelocity, deltaTime);
            break;

        case MotorControlMode::VELOCITY:
            outputTorque = updateVelocityControl(currentVelocity, deltaTime);
            break;

        case MotorControlMode::TORQUE:
            outputTorque = targetTorque_;
            break;

        case MotorControlMode::SERVO:
            outputTorque = updateServoControl(currentAngle, currentVelocity, deltaTime);
            break;

        case MotorControlMode::STEPPER:
            outputTorque = updateStepperControl(currentAngle, deltaTime);
            break;
        }

        // Apply gear ratio
        outputTorque *= motorParams_.gearRatio;

        // Apply damping and friction
        outputTorque -= motorParams_.damping * currentVelocity;
        outputTorque -= motorParams_.friction * sign(currentVelocity);

        // Clamp to max torque
        outputTorque = clamp(outputTorque, -motorParams_.maxTorque, motorParams_.maxTorque);

        // Update electrical simulation
        updateElectricalModel(outputTorque, currentVelocity, deltaTime);

        // Apply torque through constraint
        setMotorTarget(outputTorque / motorParams_.maxTorque * motorParams_.maxVelocity,
                      motorParams_.maxTorque);

        // Update telemetry
        updateTelemetry(currentAngle, currentVelocity, outputTorque);
    }

    void MotorConstraint::setTargetPosition(const Float position) {
        targetPosition_ = position;
        if (motorParams_.controlMode == MotorControlMode::STEPPER) {
            stepperTargetStep_ = static_cast<Int>(position / stepAngle_);
        }
    }

    void MotorConstraint::setTargetVelocity(const Float velocity) {
        targetVelocity_ = clamp(velocity, -motorParams_.maxVelocity, motorParams_.maxVelocity);
    }

    void MotorConstraint::setTargetTorque(const Float torque) {
        targetTorque_ = clamp(torque, -motorParams_.maxTorque, motorParams_.maxTorque);
    }

    Float MotorConstraint::updatePositionControl(const Float currentAngle, const Float currentVelocity, const Float deltaTime) {
        Float error = targetPosition_ - currentAngle;

        // Wrap angle error to [-PI, PI]
        while (error > PI<Float>) error -= TWO_PI<Float>;
        while (error < -PI<Float>) error += TWO_PI<Float>;

        return pidController_.update(error, deltaTime);
    }

    Float MotorConstraint::updateVelocityControl(const Float currentVelocity, const Float deltaTime) const {
        const Float error = targetVelocity_ - currentVelocity;

        // Simple P controller for velocity
        constexpr Float kp = 5.0f;
        return kp * error;
    }

    Float MotorConstraint::updateServoControl(const Float currentAngle, const Float currentVelocity, const Float deltaTime) const {
        // Servo motors move to position at constant speed
        Float error = targetPosition_ - currentAngle;

        while (error > PI<Float>) error -= TWO_PI<Float>;
        while (error < -PI<Float>) error += TWO_PI<Float>;

        Float targetVel = 0.0f;
        if (std::abs(error) > 0.01f) {
            targetVel = sign(error) * servoSpeed_;

            // Slow down near target
            if (constexpr Float slowdownRange = 0.2f; std::abs(error) < slowdownRange) {
                targetVel *= std::abs(error) / slowdownRange;
            }
        }

        const Float velError = targetVel - currentVelocity;
        return velError * 10.0f; // High gain for servo response
    }

    Float MotorConstraint::updateStepperControl(const Float currentAngle, const Float deltaTime) {
        // Stepper motor moves in discrete steps
        stepTimer_ += deltaTime;

        if (constexpr Float stepPeriod = 0.001f; stepTimer_ >= stepPeriod) {
            stepTimer_ = 0.0f;

            if (currentStep_ < stepperTargetStep_) {
                currentStep_++;
            } else if (currentStep_ > stepperTargetStep_) {
                currentStep_--;
            }
        }

        const Float targetAngle = currentStep_ * stepAngle_;
        const Float error = targetAngle - currentAngle;

        // High gain to hold position
        return error * 50.0f;
    }

    void MotorConstraint::updateElectricalModel(const Float torque, const Float velocity, const Float deltaTime) {
        // Back EMF
        const Float backEMF = motorParams_.backEMFConstant * velocity;

        // Current from torque
        Float requiredCurrent = torque / motorParams_.torqueConstant;

        // Apply electrical dynamics (simplified)
        const Float voltageDrop = current_ * motorParams_.resistance;
        const Float availableVoltage = motorParams_.voltage - backEMF - voltageDrop;

        const Float currentChange = (availableVoltage / motorParams_.inductance) * deltaTime;
        current_ += currentChange;

        // Current limiting
        current_ = clamp(current_, -motorParams_.maxCurrent, motorParams_.maxCurrent);

        // Temperature model (simplified)
        Float powerLoss = current_ * current_ * motorParams_.resistance;
        Float cooling = (temperature_ - 20.0f) * 0.1f; // Ambient cooling
        temperature_ += (powerLoss - cooling) * deltaTime * 0.01f;

        // Efficiency calculation
        Float mechanicalPower = std::abs(torque * velocity);
        Float electricalPower = std::abs(current_ * motorParams_.voltage);
        efficiency_ = electricalPower > 0 ? mechanicalPower / electricalPower : 0.0f;
    }

    void MotorConstraint::updateTelemetry(const Float position, const Float velocity, const Float torque) {
        Telemetry data;
        data.position = position;
        data.velocity = velocity;
        data.torque = torque;
        data.current = current_;
        data.voltage = motorParams_.voltage;
        data.temperature = temperature_;
        data.power = getPower();
        data.efficiency = efficiency_;
        data.timestamp = std::chrono::steady_clock::now();

        telemetryHistory_.push(data);

        while (telemetryHistory_.size() > MAX_TELEMETRY_SAMPLES) {
            telemetryHistory_.pop();
        }
    }

    void LinearMotorConstraint::update(const Float deltaTime) {
        const Float currentPosition = getLinearPosition();
        const Float currentVelocity = estimateLinearVelocity(currentPosition, deltaTime);

        Float outputForce = 0.0f;

        switch (motorParams_.controlMode) {
        case MotorControlMode::POSITION:
            outputForce = updatePositionControl(currentPosition, deltaTime);
            break;

        case MotorControlMode::VELOCITY:
            outputForce = updateVelocityControl(currentVelocity, deltaTime);
            break;

        case MotorControlMode::TORQUE:
            outputForce = targetForce_;
            break;

        default:
            break;
        }

        // Apply damping and friction
        outputForce -= motorParams_.damping * currentVelocity;
        outputForce -= motorParams_.friction * sign(currentVelocity);

        // Clamp to max force
        outputForce = clamp(outputForce, -motorParams_.maxForce, motorParams_.maxForce);

        // Apply force through constraint
        setLinearMotor(true, outputForce / motorParams_.maxForce * motorParams_.maxVelocity,
                      motorParams_.maxForce);
    }

    Float LinearMotorConstraint::estimateLinearVelocity(const Float currentPosition, const Float deltaTime) {
        const Float velocity = (currentPosition - lastPosition_) / deltaTime;
        lastPosition_ = currentPosition;

        return velocity;
    }

    Float LinearMotorConstraint::updatePositionControl(const Float currentPosition, const Float deltaTime) {
        const Float error = targetPosition_ - currentPosition;

        return pidController_.update(error, deltaTime);
    }

    Float LinearMotorConstraint::updateVelocityControl(const Float currentVelocity, Float deltaTime) const {
        const Float error = targetVelocity_ - currentVelocity;

        return error * 10.0f; // Simple P controller
    }

    void RoboticJoint::calibrate() {
        // Move to home position
        setTargetPosition(jointParams_.homePosition);
        calibrated_ = true;
        encoderCount_ = 0;
    }

    bool RoboticJoint::moveToPosition(Float position, const Float speed) {
        if (!calibrated_) return false;

        position = clamp(position, jointParams_.minAngle, jointParams_.maxAngle);
        setTargetPosition(position);
        setTargetVelocity(speed);

        return true;
    }

    bool RoboticJoint::isAtTarget(const Float tolerance) const {
        const Float targetPosition = getTargetPosition();
        return std::abs(getHingeAngle() - targetPosition) < tolerance;
    }

    void RoboticJoint::updateEncoder(const Float currentAngle) {
        if (!jointParams_.hasEncoder) return;

        const Float angleDiff = currentAngle - lastAngle_;
        encoderCount_ += static_cast<Int>(angleDiff / TWO_PI<Float> * jointParams_.encoderResolution);
        lastAngle_ = currentAngle;
    }
} // namespace engine::physics
