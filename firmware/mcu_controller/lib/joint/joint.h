#ifndef JOINT_H
#define JOINT_H

#include <stdint.h>
#include <stdbool.h>

#define JOINT_MAX_COUNT 6U

typedef enum
{
    JOINT_OK = 0,
    JOINT_ERR_ID,
    JOINT_ERR_NOT_INITIALIZED,
    JOINT_ERR_NOT_CONFIGURED,
    JOINT_ERR_INVALID_PARAM,
    JOINT_ERR_BUSY,
    JOINT_ERR_POSITION_UNKNOWN,
    JOINT_ERR_OUT_OF_RANGE,
    JOINT_ERR_STEPPER
} JointResult;

// Mechanical configuration of a robot joint.
typedef struct
{
    uint32_t stepsPerRevolution;
    float gearRatio;
    float minAngleDeg;
    float maxAngleDeg;
    bool inverted;
} JointConfig;

// Motion parameters expressed in joint units.
typedef struct
{
    float speedDegS;
    float accelerationDegS2;
    float linearAccelerationDeg;
} JointMotionConfig;

// Initialization and mechanical configuration.
JointResult Joint_Init(void);
JointResult Joint_Configure(uint8_t jointId, const JointConfig *config);
JointResult Joint_GetConfig(uint8_t jointId, JointConfig *config);
JointResult Joint_UpdateConfig(uint8_t jointId, const JointConfig *config);

// Motion configuration.
JointResult Joint_SetMotionConfig(uint8_t jointId, const JointMotionConfig *config);
JointResult Joint_GetMotionConfig(uint8_t jointId, JointMotionConfig *config);
JointResult Joint_SetSpeed(uint8_t jointId, float speedDegS);
JointResult Joint_SetAcceleration(uint8_t jointId, float accelerationDegS2);
JointResult Joint_SetLinearAcceleration(uint8_t jointId, float angleDeg);

// Position and motion control.
JointResult Joint_SetZero(uint8_t jointId);
JointResult Joint_CheckMoveToAngle(uint8_t jointId, float angleDeg);
JointResult Joint_MoveToAngle(uint8_t jointId, float angleDeg);
JointResult Joint_MoveRelativeAngle(uint8_t jointId, float deltaDeg);
JointResult Joint_GetAngle(uint8_t jointId, float *angleDeg);
JointResult Joint_Stop(uint8_t jointId, bool hardStop);

// Joint status.
bool Joint_IsConfigured(uint8_t jointId);
bool Joint_IsRunning(uint8_t jointId);
bool Joint_IsPositionValid(uint8_t jointId);

#endif
