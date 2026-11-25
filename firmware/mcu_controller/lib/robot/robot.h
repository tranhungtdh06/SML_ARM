
/**
 * @file    robot.h
 * @brief   Robot joint management interface.
 * @project SML Arm Firmware
 * @version 0.1.0
 */

#ifndef SML_ROBOT_H
#define SML_ROBOT_H

#include <stdint.h>
#include <stdbool.h>
#include "joint.h"

#define ROBOT_MAX_JOINTS 6U

typedef enum
{
    ROBOT_OK = 0,
    ROBOT_ERR_ID,
    ROBOT_ERR_NOT_INITIALIZED,
    ROBOT_ERR_NOT_AVAILABLE,
    ROBOT_ERR_INVALID_PARAM,
    ROBOT_ERR_BUSY,
    ROBOT_ERR_POSITION_UNKNOWN,
    ROBOT_ERR_OUT_OF_RANGE,
    ROBOT_ERR_DRIVER
} RobotResult;

typedef struct
{
    bool available;
    bool running;
    bool positionValid;
    float angleDeg;
} RobotJointStatus;

typedef struct
{
    uint8_t jointId;
    float angleDeg;
} RobotJointTarget;

// Robot initialization and status.
RobotResult Robot_Init(void);
uint8_t Robot_GetInitFailedJoint(void);
RobotResult Robot_GetStatus(uint8_t jointId, RobotJointStatus *status);
bool Robot_IsReady(void);
bool Robot_IsJointAvailable(uint8_t jointId);

// Joint configuration.
RobotResult Robot_GetConfig(uint8_t jointId, JointConfig *config);
RobotResult Robot_UpdateConfig(uint8_t jointId, const JointConfig *config);
RobotResult Robot_GetMotionConfig(uint8_t jointId, JointMotionConfig *config);
RobotResult Robot_SetMotionConfig(uint8_t jointId, const JointMotionConfig *config);

// Joint position and motion.
RobotResult Robot_SetZero(uint8_t jointId);
RobotResult Robot_MoveToAngle(uint8_t jointId, float angleDeg);
// Independent profiles, nonblocking group launch. failedJoint=0 for general errors.
RobotResult Robot_MoveGroup(const RobotJointTarget *targets, uint8_t count,
                            uint8_t *failedJoint);
RobotResult Robot_MoveRelativeAngle(uint8_t jointId, float deltaDeg);
RobotResult Robot_GetAngle(uint8_t jointId, float *angleDeg);
RobotResult Robot_Stop(uint8_t jointId, bool hardStop);
RobotResult Robot_StopAll(bool hardStop);

#endif
