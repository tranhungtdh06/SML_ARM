
/**
 * @file    stepper.cpp
 * @brief   STEP/DIR motor driver using FastAccelStepper.
 * @project SML Arm Firmware
 * @version 0.2.0
 */

#include "stepper.h"
#include <FastAccelStepper.h>
#include <limits.h>

// Internal motor state.
static FastAccelStepperEngine s_engine;
static FastAccelStepper *s_motors[STEPPER_MAX_MOTORS] = {nullptr};
static StepperMotionConfig s_motion[STEPPER_MAX_MOTORS] = {};
static bool s_motionValid[STEPPER_MAX_MOTORS] = {};
static bool s_positionValid[STEPPER_MAX_MOTORS] = {};
static uint8_t s_stepPins[STEPPER_MAX_MOTORS] = {};
static uint8_t s_dirPins[STEPPER_MAX_MOTORS] = {};
static bool s_initialized = false;

static bool Stepper_ValidId(uint8_t jointId)
{
    return (jointId >= 1U && jointId <= STEPPER_MAX_MOTORS);
}

static StepperResult Stepper_CheckMotor(uint8_t jointId)
{
    if (!Stepper_ValidId(jointId))
        return STEPPER_ERR_ID;
    if (!s_initialized || s_motors[jointId - 1U] == nullptr)
        return STEPPER_ERR_NOT_INITIALIZED;
    return STEPPER_OK;
}

StepperResult Stepper_Init(void)
{
    if (s_initialized)
        return STEPPER_ERR_ALREADY_INITIALIZED;

    s_engine.init();
    s_initialized = true;
    return STEPPER_OK;
}

StepperResult Stepper_Attach(uint8_t jointId, const StepperPins *pins)
{
    if (!Stepper_ValidId(jointId))
        return STEPPER_ERR_ID;
    if (!s_initialized)
        return STEPPER_ERR_NOT_INITIALIZED;
    if (pins == nullptr || pins->stepPin == pins->dirPin ||
        pins->stepPin == UINT8_MAX || pins->dirPin == UINT8_MAX)
        return STEPPER_ERR_INVALID_PARAM;

    uint8_t index = jointId - 1U;
    if (s_motors[index] != nullptr)
        return STEPPER_ERR_ALREADY_INITIALIZED;

    // Reject pins already assigned to another motor.
    for (uint8_t i = 0U; i < STEPPER_MAX_MOTORS; i++)
    {
        if (s_motors[i] != nullptr &&
            (pins->stepPin == s_stepPins[i] || pins->stepPin == s_dirPins[i] ||
             pins->dirPin == s_stepPins[i] || pins->dirPin == s_dirPins[i]))
            return STEPPER_ERR_INVALID_PARAM;
    }

    FastAccelStepper *motor = s_engine.stepperConnectToPin(pins->stepPin);
    if (motor == nullptr)
        return STEPPER_ERR_DRIVER;

    motor->setDirectionPin(pins->dirPin, pins->dirHighCountsUp, pins->dirDelayUs);
    motor->setCurrentPosition(0);

    s_motors[index] = motor;
    s_stepPins[index] = pins->stepPin;
    s_dirPins[index] = pins->dirPin;
    s_motionValid[index] = false;
    s_positionValid[index] = false;
    return STEPPER_OK;
}

StepperResult Stepper_SetMotionConfig(uint8_t jointId, const StepperMotionConfig *config)
{
    StepperResult result = Stepper_CheckMotor(jointId);
    if (result != STEPPER_OK)
        return result;
    if (config == nullptr || config->speedHz == 0U ||
        config->accelerationSteps == 0U || config->accelerationSteps > INT32_MAX)
        return STEPPER_ERR_INVALID_PARAM;

    uint8_t index = jointId - 1U;
    FastAccelStepper *motor = s_motors[index];

    if (motor->isRunning())
        return STEPPER_ERR_BUSY;

    const bool hadMotion = s_motionValid[index];
    const StepperMotionConfig old = s_motion[index];
    StepperResult error = STEPPER_OK;
    if (motor->setSpeedInHz(config->speedHz) != 0)
        error = STEPPER_ERR_OUT_OF_RANGE;
    else if (motor->setAcceleration((int32_t)config->accelerationSteps) != 0)
        error = STEPPER_ERR_DRIVER;

    if (error != STEPPER_OK)
    {
        s_motionValid[index] = false;
        if (hadMotion && motor->setSpeedInHz(old.speedHz) == 0 &&
            motor->setAcceleration((int32_t)old.accelerationSteps) == 0)
        {
            motor->setLinearAcceleration(old.linearAccelerationSteps);
            motor->applySpeedAcceleration();
            s_motionValid[index] = true;
        }
        return error;
    }

    motor->setLinearAcceleration(config->linearAccelerationSteps);
    motor->applySpeedAcceleration();

    s_motion[index] = *config;
    s_motionValid[index] = true;
    return STEPPER_OK;
}

StepperResult Stepper_GetMotionConfig(uint8_t jointId, StepperMotionConfig *config)
{
    StepperResult result = Stepper_CheckMotor(jointId);
    if (result != STEPPER_OK)
        return result;
    if (config == nullptr)
        return STEPPER_ERR_INVALID_PARAM;

    uint8_t index = jointId - 1U;
    if (!s_motionValid[index])
        return STEPPER_ERR_NOT_CONFIGURED;

    *config = s_motion[index];
    return STEPPER_OK;
}

StepperResult Stepper_SetSpeedHz(uint8_t jointId, uint32_t speedHz)
{
    StepperMotionConfig config = {};
    StepperResult result = Stepper_GetMotionConfig(jointId, &config);
    if (result != STEPPER_OK)
        return result;

    config.speedHz = speedHz;
    return Stepper_SetMotionConfig(jointId, &config);
}

StepperResult Stepper_SetAcceleration(uint8_t jointId, uint32_t stepsPerSecond2)
{
    StepperMotionConfig config = {};
    StepperResult result = Stepper_GetMotionConfig(jointId, &config);
    if (result != STEPPER_OK)
        return result;

    config.accelerationSteps = stepsPerSecond2;
    return Stepper_SetMotionConfig(jointId, &config);
}

StepperResult Stepper_SetLinearAcceleration(uint8_t jointId, uint32_t steps)
{
    StepperMotionConfig config = {};
    StepperResult result = Stepper_GetMotionConfig(jointId, &config);
    if (result != STEPPER_OK)
        return result;

    config.linearAccelerationSteps = steps;
    return Stepper_SetMotionConfig(jointId, &config);
}

StepperResult Stepper_SetZero(uint8_t jointId)
{
    StepperResult result = Stepper_CheckMotor(jointId);
    if (result != STEPPER_OK)
        return result;

    FastAccelStepper *motor = s_motors[jointId - 1U];
    if (motor->isRunning())
        return STEPPER_ERR_BUSY;

    // Establish a software zero at the current motor position.
    motor->setCurrentPosition(0);
    s_positionValid[jointId - 1U] = true;
    return STEPPER_OK;
}

StepperResult Stepper_InvalidatePosition(uint8_t jointId)
{
    StepperResult result = Stepper_CheckMotor(jointId);
    if (result != STEPPER_OK)
        return result;
    if (s_motors[jointId - 1U]->isRunning())
        return STEPPER_ERR_BUSY;

    // Invalidate the coordinate reference without moving the motor.
    s_positionValid[jointId - 1U] = false;
    return STEPPER_OK;
}

StepperResult Stepper_CheckMoveToSteps(uint8_t jointId, int32_t targetSteps)
{
    StepperResult result = Stepper_CheckMotor(jointId);
    if (result != STEPPER_OK)
        return result;

    uint8_t index = jointId - 1U;
    FastAccelStepper *motor = s_motors[index];

    if (motor->isRunning())
        return STEPPER_ERR_BUSY;
    if (!s_positionValid[index])
        return STEPPER_ERR_POSITION_UNKNOWN;
    if (!s_motionValid[index])
        return STEPPER_ERR_NOT_CONFIGURED;

    int32_t current = motor->getCurrentPosition();
    int64_t distance = (int64_t)targetSteps - (int64_t)current;

    if (distance > INT32_MAX || distance < INT32_MIN)
        return STEPPER_ERR_OUT_OF_RANGE;
    return STEPPER_OK;
}

StepperResult Stepper_MoveToSteps(uint8_t jointId, int32_t targetSteps)
{
    StepperResult result = Stepper_CheckMoveToSteps(jointId, targetSteps);
    if (result != STEPPER_OK)
        return result;

    FastAccelStepper *motor = s_motors[jointId - 1U];
    if (targetSteps == motor->getCurrentPosition())
        return STEPPER_OK;

    return (motor->moveTo(targetSteps) == MOVE_OK) ? STEPPER_OK : STEPPER_ERR_DRIVER;
}

StepperResult Stepper_MoveRelativeSteps(uint8_t jointId, int32_t deltaSteps)
{
    StepperResult result = Stepper_CheckMotor(jointId);
    if (result != STEPPER_OK)
        return result;

    int64_t target = (int64_t)s_motors[jointId - 1U]->getCurrentPosition() + deltaSteps;
    if (target > INT32_MAX || target < INT32_MIN)
        return STEPPER_ERR_OUT_OF_RANGE;

    return Stepper_MoveToSteps(jointId, (int32_t)target);
}

StepperResult Stepper_Stop(uint8_t jointId, bool hardStop)
{
    StepperResult result = Stepper_CheckMotor(jointId);
    if (result != STEPPER_OK)
        return result;

    FastAccelStepper *motor = s_motors[jointId - 1U];

    if (hardStop)
    {
        motor->forceStopAndNewPosition(motor->getCurrentPosition());
        s_positionValid[jointId - 1U] = false;
    }
    else
    {
        motor->stopMove();
    }

    return STEPPER_OK;
}

StepperResult Stepper_GetPositionSteps(uint8_t jointId, int32_t *position)
{
    StepperResult result = Stepper_CheckMotor(jointId);
    if (result != STEPPER_OK)
        return result;
    if (position == nullptr)
        return STEPPER_ERR_INVALID_PARAM;
    if (!s_positionValid[jointId - 1U])
        return STEPPER_ERR_POSITION_UNKNOWN;

    *position = s_motors[jointId - 1U]->getCurrentPosition();
    return STEPPER_OK;
}

bool Stepper_IsAttached(uint8_t jointId)
{
    return Stepper_ValidId(jointId) && s_motors[jointId - 1U] != nullptr;
}

bool Stepper_IsRunning(uint8_t jointId)
{
    return Stepper_IsAttached(jointId) && s_motors[jointId - 1U]->isRunning();
}

bool Stepper_IsPositionValid(uint8_t jointId)
{
    return Stepper_IsAttached(jointId) && s_positionValid[jointId - 1U];
}
