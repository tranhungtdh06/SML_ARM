/**
 * @file    stepper.cpp
 * @brief   STEP/DIR motor driver using FastAccelStepper.
 * @project SML Arm Firmware
 * @version 0.2.0
 */

#include "stepper.h"
#include <FastAccelStepper.h>
#include <limits.h>

// Internal motor state
static FastAccelStepperEngine s_engine;
static FastAccelStepper *s_motors[STEPPER_MAX_MOTORS] = {nullptr};
static StepperMotionConfig s_motion[STEPPER_MAX_MOTORS] = {};
static bool s_motionValid[STEPPER_MAX_MOTORS] = {false};
static bool s_positionValid[STEPPER_MAX_MOTORS] = {false};
static uint8_t s_stepPins[STEPPER_MAX_MOTORS] = {};
static uint8_t s_dirPins[STEPPER_MAX_MOTORS] = {};
static bool s_initialized = false;

static bool Stepper_ValidId(uint8_t jointId)
{
    return (1U <= jointId) && (STEPPER_MAX_MOTORS >= jointId);
}

static StepperResult Stepper_CheckMotor(uint8_t jointId)
{
    if (!Stepper_ValidId(jointId))
    {
        return STEPPER_ERR_ID;
    }
    if (!s_initialized || (nullptr == s_motors[jointId - 1U]))
    {
        return STEPPER_ERR_NOT_INITIALIZED;
    }
    return STEPPER_OK;
}

StepperResult Stepper_Init(void)
{
    if (s_initialized)
    {
        return STEPPER_ERR_ALREADY_INITIALIZED;
    }
    s_engine.init();
    s_initialized = true;
    return STEPPER_OK;
}

StepperResult Stepper_Attach(uint8_t jointId, const StepperPins *pins)
{
    if (!Stepper_ValidId(jointId))
    {
        return STEPPER_ERR_ID;
    }
    if (!s_initialized)
    {
        return STEPPER_ERR_NOT_INITIALIZED;
    }
    if ((nullptr == pins) || (pins->stepPin == pins->dirPin))
    {
        return STEPPER_ERR_INVALID_PARAM;
    }
    uint8_t index = jointId - 1U;
    if (nullptr != s_motors[index])
    {
        return STEPPER_ERR_ALREADY_INITIALIZED;
    }
    // Prevent GPIO conflicts between attached motors.
    for (uint8_t i = 0U; i < STEPPER_MAX_MOTORS; i++)
    {
        if ((nullptr != s_motors[i]) &&
            ((pins->stepPin == s_stepPins[i]) || (pins->stepPin == s_dirPins[i]) ||
             (pins->dirPin == s_stepPins[i]) || (pins->dirPin == s_dirPins[i])))
        {
            return STEPPER_ERR_INVALID_PARAM;
        }
    }
    FastAccelStepper *motor = s_engine.stepperConnectToPin(pins->stepPin);
    if (nullptr == motor)
    {
        return STEPPER_ERR_DRIVER;
    }
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
    if (STEPPER_OK != result)
    {
        return result;
    }
    if ((nullptr == config) || (0U == config->speedHz) || (0U == config->accelerationSteps) ||
        (static_cast<uint32_t>(INT32_MAX) < config->accelerationSteps))
    {
        return STEPPER_ERR_INVALID_PARAM;
    }
    uint8_t index = jointId - 1U;
    FastAccelStepper *motor = s_motors[index];
    if (motor->isRunning())
    {
        return STEPPER_ERR_BUSY;
    }
    if (0 != motor->setSpeedInHz(config->speedHz))
    {
        s_motionValid[index] = false;
        return STEPPER_ERR_OUT_OF_RANGE;
    }
    if (0 != motor->setAcceleration(static_cast<int32_t>(config->accelerationSteps)))
    {
        s_motionValid[index] = false;
        return STEPPER_ERR_DRIVER;
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
    if (STEPPER_OK != result)
    {
        return result;
    }
    if (nullptr == config)
    {
        return STEPPER_ERR_INVALID_PARAM;
    }
    uint8_t index = jointId - 1U;
    if (!s_motionValid[index])
    {
        return STEPPER_ERR_NOT_CONFIGURED;
    }
    *config = s_motion[index];
    return STEPPER_OK;
}

StepperResult Stepper_SetSpeedHz(uint8_t jointId, uint32_t speedHz)
{
    StepperMotionConfig config;
    StepperResult result = Stepper_GetMotionConfig(jointId, &config);
    if (STEPPER_OK != result)
    {
        return result;
    }
    config.speedHz = speedHz;
    return Stepper_SetMotionConfig(jointId, &config);
}

StepperResult Stepper_SetAcceleration(uint8_t jointId, uint32_t stepsPerSecond2)
{
    StepperMotionConfig config;
    StepperResult result = Stepper_GetMotionConfig(jointId, &config);
    if (STEPPER_OK != result)
    {
        return result;
    }
    config.accelerationSteps = stepsPerSecond2;
    return Stepper_SetMotionConfig(jointId, &config);
}

StepperResult Stepper_SetLinearAcceleration(uint8_t jointId, uint32_t steps)
{
    StepperMotionConfig config;
    StepperResult result = Stepper_GetMotionConfig(jointId, &config);
    if (STEPPER_OK != result)
    {
        return result;
    }
    config.linearAccelerationSteps = steps;
    return Stepper_SetMotionConfig(jointId, &config);
}

StepperResult Stepper_SetZero(uint8_t jointId)
{
    StepperResult result = Stepper_CheckMotor(jointId);
    if (STEPPER_OK != result)
    {
        return result;
    }
    FastAccelStepper *motor = s_motors[jointId - 1U];
    if (motor->isRunning())
    {
        return STEPPER_ERR_BUSY;
    }
    motor->setCurrentPosition(0);
    s_positionValid[jointId - 1U] = true;
    return STEPPER_OK;
}

StepperResult Stepper_MoveToSteps(uint8_t jointId, int32_t targetSteps)
{
    StepperResult result = Stepper_CheckMotor(jointId);
    if (STEPPER_OK != result)
    {
        return result;
    }
    uint8_t index = jointId - 1U;
    FastAccelStepper *motor = s_motors[index];
    if (motor->isRunning())
    {
        return STEPPER_ERR_BUSY;
    }
    if (!s_positionValid[index])
    {
        return STEPPER_ERR_POSITION_UNKNOWN;
    }
    if (!s_motionValid[index])
    {
        return STEPPER_ERR_NOT_CONFIGURED;
    }
    int32_t current = motor->getCurrentPosition();
    int64_t distance = static_cast<int64_t>(targetSteps) - current;
    if ((INT32_MAX < distance) || (INT32_MIN > distance))
    {
        return STEPPER_ERR_OUT_OF_RANGE;
    }
    if (targetSteps == current)
    {
        return STEPPER_OK;
    }
    return (MOVE_OK == motor->moveTo(targetSteps)) ? STEPPER_OK : STEPPER_ERR_DRIVER;
}

StepperResult Stepper_MoveRelativeSteps(uint8_t jointId, int32_t deltaSteps)
{
    StepperResult result = Stepper_CheckMotor(jointId);
    if (STEPPER_OK != result)
    {
        return result;
    }
    int64_t target = static_cast<int64_t>(s_motors[jointId - 1U]->getCurrentPosition()) + deltaSteps;
    if ((INT32_MAX < target) || (INT32_MIN > target))
    {
        return STEPPER_ERR_OUT_OF_RANGE;
    }
    return Stepper_MoveToSteps(jointId, static_cast<int32_t>(target));
}

StepperResult Stepper_Stop(uint8_t jointId, bool hardStop)
{
    StepperResult result = Stepper_CheckMotor(jointId);
    if (STEPPER_OK != result)
    {
        return result;
    }
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
    if (STEPPER_OK != result)
    {
        return result;
    }
    if (nullptr == position)
    {
        return STEPPER_ERR_INVALID_PARAM;
    }
    if (!s_positionValid[jointId - 1U])
    {
        return STEPPER_ERR_POSITION_UNKNOWN;
    }
    *position = s_motors[jointId - 1U]->getCurrentPosition();
    return STEPPER_OK;
}

bool Stepper_IsAttached(uint8_t jointId)
{
    return Stepper_ValidId(jointId) && (nullptr != s_motors[jointId - 1U]);
}

bool Stepper_IsRunning(uint8_t jointId)
{
    return Stepper_IsAttached(jointId) && s_motors[jointId - 1U]->isRunning();
}

bool Stepper_IsPositionValid(uint8_t jointId)
{
    return Stepper_IsAttached(jointId) && s_positionValid[jointId - 1U];
}
