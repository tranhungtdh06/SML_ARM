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
static uint8_t s_initFailedJoint = 0U;
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

    const bool enabled[ROBOT_MAX_JOINTS] = {
        J1_ENABLED, J2_ENABLED, J3_ENABLED,
        J4_ENABLED, J5_ENABLED, J6_ENABLED};
    const StepperPins pins[ROBOT_MAX_JOINTS] = {
        {J1_STEP_PIN, J1_DIR_PIN, STEPPER_DIR_HIGH_COUNTS_UP, STEPPER_DIR_DELAY_US},
        {J2_STEP_PIN, J2_DIR_PIN, STEPPER_DIR_HIGH_COUNTS_UP, STEPPER_DIR_DELAY_US},
        {J3_STEP_PIN, J3_DIR_PIN, STEPPER_DIR_HIGH_COUNTS_UP, STEPPER_DIR_DELAY_US},
        {J4_STEP_PIN, J4_DIR_PIN, STEPPER_DIR_HIGH_COUNTS_UP, STEPPER_DIR_DELAY_US},
        {J5_STEP_PIN, J5_DIR_PIN, STEPPER_DIR_HIGH_COUNTS_UP, STEPPER_DIR_DELAY_US},
        {J6_STEP_PIN, J6_DIR_PIN, STEPPER_DIR_HIGH_COUNTS_UP, STEPPER_DIR_DELAY_US}};
    const JointConfig defaults = {
        DEFAULT_STEPS_PER_REV, DEFAULT_GEAR_RATIO,
        DEFAULT_MIN_ANGLE_DEG, DEFAULT_MAX_ANGLE_DEG, DEFAULT_INVERTED};
    const JointMotionConfig motion = {
        DEFAULT_SPEED_DEG_S, DEFAULT_ACCEL_DEG_S2, DEFAULT_LINEAR_ACCEL_DEG};

    bool anyEnabled = false;
    for (uint8_t i = 0U; i < ROBOT_MAX_JOINTS; ++i)
    {
        if (!enabled[i])
            continue;
        anyEnabled = true;
        const uint8_t id = i + 1U;
        RobotResult result = Robot_AttachJoint(id, &pins[i], &defaults, &motion);
        if (result != ROBOT_OK)
        {
            // No motion is allowed on a partially initialized robot.
            s_initFailedJoint = id;
            return result;
        }
    }
    if (!anyEnabled)
        return ROBOT_ERR_NOT_AVAILABLE;

    s_initialized = true;
    return ROBOT_OK;
}

uint8_t Robot_GetInitFailedJoint(void)
{
    return s_initFailedJoint;
}

RobotResult Robot_GetStatus(uint8_t jointId, RobotJointStatus *status)
{
    if (!s_initialized)
        return ROBOT_ERR_NOT_INITIALIZED;
    if (jointId < 1U || jointId > ROBOT_MAX_JOINTS)
        return ROBOT_ERR_ID;
    if (status == nullptr)
        return ROBOT_ERR_INVALID_PARAM;

    *status = {};
    if (!s_available[jointId - 1U])
        return ROBOT_OK;

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

RobotResult Robot_MoveGroup(const RobotJointTarget *targets, uint8_t count,
                            uint8_t *failedJoint)
{
    if (failedJoint != nullptr)
        *failedJoint = 0U;
    if (!s_initialized)
        return ROBOT_ERR_NOT_INITIALIZED;
    if (targets == nullptr || count < 2U || count > ROBOT_MAX_JOINTS)
        return ROBOT_ERR_INVALID_PARAM;

    // Phase 1: validate every target before issuing any movement.
    bool selected[ROBOT_MAX_JOINTS] = {};
    for (uint8_t i = 0U; i < count; ++i)
    {
        const uint8_t id = targets[i].jointId;
        if (failedJoint != nullptr)
            *failedJoint = id;
        RobotResult result = Robot_CheckJoint(id);
        if (result != ROBOT_OK)
            return result;
        if (selected[id - 1U])
            return ROBOT_ERR_INVALID_PARAM;
        selected[id - 1U] = true;
        result = Robot_FromJointResult(Joint_CheckMoveToAngle(id, targets[i].angleDeg));
        if (result != ROBOT_OK)
            return result;
    }

    // Phase 2: launch independent moves back-to-back, without waiting.
    // This is NOT a time-synchronized trajectory or atomic hardware start.
    for (uint8_t i = 0U; i < count; ++i)
    {
        const uint8_t id = targets[i].jointId;
        RobotResult result = Robot_MoveToAngle(id, targets[i].angleDeg);
        if (result != ROBOT_OK)
        {
            // A runtime launch failure may occur after earlier axes started.
            // Hard-stop all selected axes and invalidate their coordinates.
            for (uint8_t k = 0U; k < count; ++k)
                Robot_Stop(targets[k].jointId, true);
            if (failedJoint != nullptr)
                *failedJoint = id;
            return result;
        }
    }
    if (failedJoint != nullptr)
        *failedJoint = 0U;
    return ROBOT_OK;
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
