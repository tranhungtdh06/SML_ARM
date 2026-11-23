/**
 * @file    StepperDriver.h
 * @brief   FastAccelStepper driver implementation.
 * @project SML Arm Firmware
 * @version 0.1.0
 */
#ifndef SML_STEPPER_DRIVER_H
#define SML_STEPPER_DRIVER_H
#include <FastAccelStepper.h>
#include "IStepperDriver.h"

namespace SML
{
    class StepperDriver : public IStepperDriver
    {
    public:
        StepperDriver(FastAccelStepperEngine &engine,uint8_t stepPin,uint8_t dirPin,bool dirHighCountsUp = true,uint16_t dirDelayUs = 50U);

        // ================= INITIALIZATION =================
        ErrorCode begin() override;
        bool isInitialized() const override;

        // ================= MOTION CONTROL =================
        ErrorCode moveToSteps(StepPosition position) override;
        ErrorCode moveSteps(StepPosition steps) override;

        // ================= CONFIGURATION =================
        ErrorCode setSpeedHz(StepSpeed speed) override;
        ErrorCode setAcceleration(StepAcceleration acceleration) override;
        ErrorCode setLinearAcceleration(uint32_t steps) override;

        // ================= POSITION =================
        StepPosition getCurrentPosition() const override;
        ErrorCode setCurrentPosition(StepPosition position) override;
        bool isPositionValid() const override;

        // ================= STATUS =================
        bool isRunning() const override;
        StepSpeed getSpeedHz() const override;
        StepAcceleration getAcceleration() const override;
        uint32_t getLinearAcceleration() const override;

        // ================= STOP CONTROL =================
        ErrorCode stop(StopMode mode = StopMode::SOFT) override;

    private:
        FastAccelStepperEngine &_engine;
        FastAccelStepper *_stepper;
        uint8_t _stepPin;
        uint8_t _dirPin;
        bool _dirHighCountsUp;
        uint16_t _dirDelayUs;
        bool _positionValid;
        StepSpeed _speed;
        StepAcceleration _acceleration;
        uint32_t _linearAcceleration;
    };
}

#endif // SML_STEPPER_DRIVER_H
