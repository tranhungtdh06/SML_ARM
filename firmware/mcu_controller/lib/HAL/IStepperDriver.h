/**
 * @file    IStepperDriver.h
 * @brief   Abstract stepper motor driver interface.
 * @project SML Arm Firmware
 * @version 0.1.0
 */

#ifndef SML_I_STEPPER_DRIVER_H
#define SML_I_STEPPER_DRIVER_H
#include "Types.h"
#include "ErrorCode.h"

namespace SML
{
    class IStepperDriver
    {
    public:
        virtual ~IStepperDriver() = default;
        // ================= INITIALIZATION =================
        virtual ErrorCode begin() = 0;
        virtual bool isInitialized() const = 0;

        // ================= MOTION CONTROL =================
        virtual ErrorCode moveToSteps(StepPosition position) = 0;
        virtual ErrorCode moveSteps(StepPosition steps) = 0;

        // ================= MOTION CONFIGURATION =================
        virtual ErrorCode setSpeedHz(StepSpeed speed) = 0;
        virtual ErrorCode setAcceleration(StepAcceleration acceleration) = 0;
        virtual ErrorCode setLinearAcceleration(uint32_t steps) = 0;

        // ================= POSITION MANAGEMENT =================
        virtual StepPosition getCurrentPosition() const = 0;
        virtual ErrorCode setCurrentPosition(StepPosition position) = 0;
        virtual bool isPositionValid() const = 0;

        // ================= STATUS =================
        virtual bool isRunning() const = 0;
        virtual StepSpeed getSpeedHz() const = 0;
        virtual StepAcceleration getAcceleration() const = 0;
        virtual uint32_t getLinearAcceleration() const = 0;

        // ================= STOP CONTROL =================
        virtual ErrorCode stop(StopMode mode = StopMode::SOFT) = 0;
    };
}

#endif // SML_I_STEPPER_DRIVER_H
