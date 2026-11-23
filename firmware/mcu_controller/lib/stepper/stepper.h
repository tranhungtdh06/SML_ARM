/**
 * @file    stepper.h
 * @brief   STEP/DIR motor control interface.
 * @project SML Arm Firmware
 * @version 0.2.0
 */

#ifndef SML_STEPPER_H
#define SML_STEPPER_H

#include <stdint.h>
#include <stdbool.h>

#define STEPPER_MAX_MOTORS 6U

// Stepper operation results.
typedef enum
{
    STEPPER_OK = 0,
    STEPPER_ERR_ID,
    STEPPER_ERR_NOT_INITIALIZED,
    STEPPER_ERR_ALREADY_INITIALIZED,
    STEPPER_ERR_INVALID_PARAM,
    STEPPER_ERR_BUSY,
    STEPPER_ERR_DRIVER,
    STEPPER_ERR_POSITION_UNKNOWN,
    STEPPER_ERR_OUT_OF_RANGE,
    STEPPER_ERR_NOT_CONFIGURED
} StepperResult;

// STEP/DIR hardware configuration.
typedef struct
{
    uint8_t stepPin;
    uint8_t dirPin;
    bool dirHighCountsUp;
    uint16_t dirDelayUs;
} StepperPins;

// Motion configuration in motor step units.
typedef struct
{
    uint32_t speedHz;
    uint32_t accelerationSteps;
    uint32_t linearAccelerationSteps;
} StepperMotionConfig;

// Initialization.
StepperResult Stepper_Init(void);
StepperResult Stepper_Attach(uint8_t jointId, const StepperPins *pins);

// Motion configuration.
StepperResult Stepper_SetMotionConfig(uint8_t jointId, const StepperMotionConfig *config);
StepperResult Stepper_GetMotionConfig(uint8_t jointId, StepperMotionConfig *config);
StepperResult Stepper_SetSpeedHz(uint8_t jointId, uint32_t speedHz);
StepperResult Stepper_SetAcceleration(uint8_t jointId, uint32_t stepsPerSecond2);
StepperResult Stepper_SetLinearAcceleration(uint8_t jointId, uint32_t steps);

// Position control.
StepperResult Stepper_SetZero(uint8_t jointId);
StepperResult Stepper_InvalidatePosition(uint8_t jointId);
StepperResult Stepper_MoveToSteps(uint8_t jointId, int32_t targetSteps);
StepperResult Stepper_MoveRelativeSteps(uint8_t jointId, int32_t deltaSteps);

// Motion stopping.
StepperResult Stepper_Stop(uint8_t jointId, bool hardStop);

// Motor status.
StepperResult Stepper_GetPositionSteps(uint8_t jointId, int32_t *position);
bool Stepper_IsAttached(uint8_t jointId);
bool Stepper_IsRunning(uint8_t jointId);
bool Stepper_IsPositionValid(uint8_t jointId);

#endif
