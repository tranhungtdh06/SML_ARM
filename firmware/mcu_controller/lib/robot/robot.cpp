/**
 * @file    robot.cpp
 * @brief   Robot initialization and joint coordination.
 * @project SML Arm Firmware
 * @version 0.1.0
 */

#include "robot.h"
#include "stepper.h"
#include "board_pins.h"
#include "robot_config.h"

static bool s_initialized = false;
static bool s_available[ROBOT_MAX_JOINTS] = {};

// Convert Joint errors to Robot errors.
static RobotResult Robot_FromJointResult(JointResult result)
{
    switch (result)
    {
    case JOINT_OK:
        return ROBOT_OK;
    case JOINT_ERR_ID:
        return ROBOT_ERR_ID;
    case JOINT_ERR_NOT_INITIALIZED:
        return ROBOT_ERR_NOT_INITIALIZED;
    case JOINT_ERR_NOT_CONFIGURED:
        return ROBOT_ERR_NOT_AVAILABLE;
    case JOINT_ERR_INVALID_PARAM:
        return ROBOT_ERR_INVALID_PARAM;
    case JOINT_ERR_BUSY:
        return ROBOT_ERR_BUSY;
    case JOINT_ERR_POSITION_UNKNOWN:
        return ROBOT_ERR_POSITION_UNKNOWN;
    case JOINT_ERR_OUT_OF_RANGE:
        return ROBOT_ERR_OUT_OF_RANGE;
    default:
        return ROBOT_ERR_DRIVER;
    }
}

static RobotResult Robot_CheckJoint(uint8_t jointId)
{
    if (!s_initialized)
        return ROBOT_ERR_NOT_INITIALIZED;
    if (jointId < 1U || jointId > ROBOT_MAX_JOINTS)
        return ROBOT_ERR_ID;
    if (!s_available[jointId - 1U])
        return ROBOT_ERR_NOT_AVAILABLE;
    return ROBOT_OK;
}

// Attach and configure one joint without establishing software zero.
static RobotResult Robot_AttachJoint(uint8_t jointId, const StepperPins *pins,
                                     const JointConfig *config, const JointMotionConfig *motion)
{
    if (pins == nullptr || config == nullptr || motion == nullptr)
        return ROBOT_ERR_INVALID_PARAM;
    if (jointId < 1U || jointId > ROBOT_MAX_JOINTS)
        return ROBOT_ERR_ID;
    if (Stepper_Attach(jointId, pins) != STEPPER_OK)
        return ROBOT_ERR_DRIVER;

    JointResult result = Joint_Configure(jointId, config);
    if (result != JOINT_OK)
        return Robot_FromJointResult(result);

    result = Joint_SetMotionConfig(jointId, motion);
    if (result != JOINT_OK)
        return Robot_FromJointResult(result);

    s_available[jointId - 1U] = true;
    return ROBOT_OK;
}

RobotResult Robot_Init(void)
{
    if (s_initialized)
        return ROBOT_OK;

    if (ROBOT_MAX_JOINTS != JOINT_MAX_COUNT ||
        ROBOT_MAX_JOINTS != STEPPER_MAX_MOTORS ||
        ROBOT_MAX_JOINTS != ROBOT_JOINT_COUNT ||
        ROBOT_MAX_JOINTS != BOARD_JOINT_COUNT)
        return ROBOT_ERR_INVALID_PARAM;

    StepperResult stepperResult = Stepper_Init();
    if (stepperResult != STEPPER_OK &&
        stepperResult != STEPPER_ERR_ALREADY_INITIALIZED)
        return ROBOT_ERR_DRIVER;

    if (Joint_Init() != JOINT_OK)
        return ROBOT_ERR_DRIVER;

    // J1 is the only joint with confirmed pin assignments.
    StepperPins j1Pins = {J1_STEP_PIN, J1_DIR_PIN, true, STEPPER_DIR_DELAY_US};
    JointConfig j1Config = {
        J1_STEPS_PER_REV,
        J1_GEAR_RATIO,
        J1_MIN_ANGLE_DEG,
        J1_MAX_ANGLE_DEG,
        J1_INVERTED};
    JointMotionConfig j1Motion = {
        J1_SPEED_DEG_S,
        J1_ACCEL_DEG_S2,
        J1_LINEAR_ACCEL_DEG};

    RobotResult result = Robot_AttachJoint(1U, &j1Pins, &j1Config, &j1Motion);
    if (result != ROBOT_OK)
        return result;

    s_initialized = true;
    return ROBOT_OK;
}

RobotResult Robot_GetStatus(uint8_t jointId, RobotJointStatus *status)
{
    RobotResult result = Robot_CheckJoint(jointId);
    if (result != ROBOT_OK)
        return result;
    if (status == nullptr)
        return ROBOT_ERR_INVALID_PARAM;

    status->available = true;
    status->running = Joint_IsRunning(jointId);
    status->positionValid = Joint_IsPositionValid(jointId);
    status->angleDeg = 0.0f;

    if (status->positionValid)
        return Robot_FromJointResult(Joint_GetAngle(jointId, &status->angleDeg));

    return ROBOT_OK;
}

bool Robot_IsReady(void)
{
    return s_initialized;
}

bool Robot_IsJointAvailable(uint8_t jointId)
{
    return s_initialized && jointId >= 1U &&
           jointId <= ROBOT_MAX_JOINTS && s_available[jointId - 1U];
}

RobotResult Robot_GetConfig(uint8_t jointId, JointConfig *config)
{
    RobotResult result = Robot_CheckJoint(jointId);
    if (result != ROBOT_OK)
        return result;
    return Robot_FromJointResult(Joint_GetConfig(jointId, config));
}

RobotResult Robot_UpdateConfig(uint8_t jointId, const JointConfig *config)
{
    RobotResult result = Robot_CheckJoint(jointId);
    if (result != ROBOT_OK)
        return result;
    return Robot_FromJointResult(Joint_UpdateConfig(jointId, config));
}

RobotResult Robot_GetMotionConfig(uint8_t jointId, JointMotionConfig *config)
{
    RobotResult result = Robot_CheckJoint(jointId);
    if (result != ROBOT_OK)
        return result;
    return Robot_FromJointResult(Joint_GetMotionConfig(jointId, config));
}

RobotResult Robot_SetMotionConfig(uint8_t jointId, const JointMotionConfig *config)
{
    RobotResult result = Robot_CheckJoint(jointId);
    if (result != ROBOT_OK)
        return result;
    return Robot_FromJointResult(Joint_SetMotionConfig(jointId, config));
}

RobotResult Robot_SetZero(uint8_t jointId)
{
    RobotResult result = Robot_CheckJoint(jointId);
    if (result != ROBOT_OK)
        return result;
    return Robot_FromJointResult(Joint_SetZero(jointId));
}

RobotResult Robot_MoveToAngle(uint8_t jointId, float angleDeg)
{
    RobotResult result = Robot_CheckJoint(jointId);
    if (result != ROBOT_OK)
        return result;
    return Robot_FromJointResult(Joint_MoveToAngle(jointId, angleDeg));
}

RobotResult Robot_MoveRelativeAngle(uint8_t jointId, float deltaDeg)
{
    RobotResult result = Robot_CheckJoint(jointId);
    if (result != ROBOT_OK)
        return result;
    return Robot_FromJointResult(Joint_MoveRelativeAngle(jointId, deltaDeg));
}

RobotResult Robot_GetAngle(uint8_t jointId, float *angleDeg)
{
    RobotResult result = Robot_CheckJoint(jointId);
    if (result != ROBOT_OK)
        return result;
    return Robot_FromJointResult(Joint_GetAngle(jointId, angleDeg));
}

RobotResult Robot_Stop(uint8_t jointId, bool hardStop)
{
    RobotResult result = Robot_CheckJoint(jointId);
    if (result != ROBOT_OK)
        return result;
    return Robot_FromJointResult(Joint_Stop(jointId, hardStop));
}

RobotResult Robot_StopAll(bool hardStop)
{
    if (!s_initialized)
        return ROBOT_ERR_NOT_INITIALIZED;

    RobotResult firstError = ROBOT_OK;
    for (uint8_t id = 1U; id <= ROBOT_MAX_JOINTS; id++)
    {
        if (!s_available[id - 1U])
            continue;

        RobotResult result = Robot_Stop(id, hardStop);
        if (result != ROBOT_OK && firstError == ROBOT_OK)
            firstError = result;
    }
    return firstError;
}
