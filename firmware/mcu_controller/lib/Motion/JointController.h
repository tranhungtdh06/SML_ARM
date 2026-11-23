/**
 * @file    JointController.h
 * @brief   Hardware-independent joint motion controller.
 * @project SML Arm Firmware
 * @version 0.1.0
 */
#ifndef SML_JOINT_CONTROLLER_H
#define SML_JOINT_CONTROLLER_H

#include "IStepperDriver.h"
#include "robot_config.h"

namespace SML
{
    class JointController
    {
    public:
        explicit JointController(IStepperDriver &driver);
        JointController(const JointController &) = delete;
        JointController &operator=(const JointController &) = delete;

        ErrorCode begin(const RobotConfig::JointConfig &config);
        bool isInitialized() const;
        ErrorCode setConfig(const RobotConfig::JointConfig &config);
        const RobotConfig::JointConfig &getConfig() const;
        ErrorCode setSpeed(AngularSpeed speed);
        ErrorCode setAcceleration(AngularAcceleration acceleration);
        ErrorCode setLinearAcceleration(Angle distance);
        ErrorCode setGearRatio(float ratio);
        ErrorCode setPulsesPerRevolution(uint32_t pulses);

        ErrorCode moveToAngle(Angle angle);
        ErrorCode moveRelativeAngle(Angle angle);
        ErrorCode setZero();
        ErrorCode getCurrentAngle(Angle &angle) const;
        ErrorCode angleToSteps(Angle angle, StepPosition &steps) const;
        ErrorCode stepsToAngle(StepPosition steps, Angle &angle) const;
        bool isPositionValid() const;
        bool isRunning() const;
        JointState getState() const;
        ErrorCode stop(StopMode mode = StopMode::SOFT);

    private:
        IStepperDriver &_driver;
        RobotConfig::JointConfig _config;
        bool _configured;
        bool _referenceValid;
        bool _stopping;
        bool _fault;

        double stepsPerDegree() const;
        ErrorCode moveToDegrees(double angle);
        static ErrorCode validateConfig(const RobotConfig::JointConfig &config, StepSpeed &speed, StepAcceleration &acceleration, uint32_t &linear);
    };
}
#endif // SML_JOINT_CONTROLLER_H
