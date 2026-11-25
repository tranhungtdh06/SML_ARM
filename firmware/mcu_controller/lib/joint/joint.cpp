#include "joint.h"
#include "stepper.h"
#include <math.h>
#include <limits.h>
#include <float.h>

static bool initialized = false;
static bool configured[JOINT_MAX_COUNT] = {};
static JointConfig jointConfigs[JOINT_MAX_COUNT] = {};
static JointMotionConfig motionConfigs[JOINT_MAX_COUNT] = {};

static bool validId(uint8_t id)
{
    return (id >= 1U && id <= JOINT_MAX_COUNT);
}

static bool finiteValue(float value)
{
    return isfinite(value);
}

static double stepsPerDegree(uint8_t id)
{
    const JointConfig *cfg = &jointConfigs[id - 1U];
    return ((double)cfg->stepsPerRevolution * (double)cfg->gearRatio) / 360.0;
}

// Validate mechanical parameters and the representable step range.
static JointResult validateConfig(const JointConfig *cfg)
{
    if (cfg == nullptr)
        return JOINT_ERR_INVALID_PARAM;
    if (cfg->stepsPerRevolution == 0U || !finiteValue(cfg->gearRatio) || cfg->gearRatio <= 0.0f ||
        !finiteValue(cfg->minAngleDeg) || !finiteValue(cfg->maxAngleDeg) ||
        cfg->minAngleDeg >= cfg->maxAngleDeg ||
        cfg->minAngleDeg > 0.0f || cfg->maxAngleDeg < 0.0f)
        return JOINT_ERR_INVALID_PARAM;

    double scale = ((double)cfg->stepsPerRevolution * (double)cfg->gearRatio) / 360.0;
    double minSteps = (double)cfg->minAngleDeg * scale;
    double maxSteps = (double)cfg->maxAngleDeg * scale;

    if (!isfinite(scale) || scale <= 0.0 || !isfinite(minSteps) || !isfinite(maxSteps) ||
        minSteps < INT32_MIN || maxSteps > INT32_MAX)
        return JOINT_ERR_OUT_OF_RANGE;

    return JOINT_OK;
}

static JointResult checkJoint(uint8_t id)
{
    if (!initialized)
        return JOINT_ERR_NOT_INITIALIZED;
    if (!validId(id))
        return JOINT_ERR_ID;
    if (!configured[id - 1U])
        return JOINT_ERR_NOT_CONFIGURED;
    if (!Stepper_IsAttached(id))
        return JOINT_ERR_STEPPER;
    return JOINT_OK;
}

static JointResult angleToSteps(uint8_t id, float angle, int32_t *steps)
{
    if (steps == nullptr || !finiteValue(angle))
        return JOINT_ERR_INVALID_PARAM;

    double value = (double)angle * stepsPerDegree(id);
    if (jointConfigs[id - 1U].inverted)
        value = -value;

    if (!isfinite(value) || value > INT32_MAX || value < INT32_MIN)
        return JOINT_ERR_OUT_OF_RANGE;

    *steps = (int32_t)round(value);
    return JOINT_OK;
}

static JointResult convertMotion(uint8_t id, const JointMotionConfig *cfg, StepperMotionConfig *out)
{
    if (cfg == nullptr || out == nullptr)
        return JOINT_ERR_INVALID_PARAM;
    if (!finiteValue(cfg->speedDegS) || !finiteValue(cfg->accelerationDegS2) ||
        !finiteValue(cfg->linearAccelerationDeg))
        return JOINT_ERR_INVALID_PARAM;
    if (cfg->speedDegS <= 0.0f || cfg->accelerationDegS2 <= 0.0f ||
        cfg->linearAccelerationDeg < 0.0f)
        return JOINT_ERR_INVALID_PARAM;

    double scale = stepsPerDegree(id);
    double speed = (double)cfg->speedDegS * scale;
    double accel = (double)cfg->accelerationDegS2 * scale;
    double linear = (double)cfg->linearAccelerationDeg * scale;

    if (!isfinite(speed) || !isfinite(accel) || !isfinite(linear) ||
        speed < 1.0 || accel < 1.0 ||
        speed > (double)INT32_MAX || accel > (double)INT32_MAX ||
        linear > (double)UINT32_MAX)
        return JOINT_ERR_OUT_OF_RANGE;

    out->speedHz = (uint32_t)round(speed);
    out->accelerationSteps = (uint32_t)round(accel);
    out->linearAccelerationSteps = (uint32_t)round(linear);
    return JOINT_OK;
}

JointResult Joint_Init(void)
{
    if (initialized)
        return JOINT_OK;

    for (uint8_t i = 0U; i < JOINT_MAX_COUNT; i++)
    {
        configured[i] = false;
        jointConfigs[i] = {};
        motionConfigs[i] = {};
    }

    initialized = true;
    return JOINT_OK;
}

JointResult Joint_Configure(uint8_t id, const JointConfig *cfg)
{
    if (!initialized)
        return JOINT_ERR_NOT_INITIALIZED;
    if (!validId(id))
        return JOINT_ERR_ID;
    if (!Stepper_IsAttached(id))
        return JOINT_ERR_STEPPER;
    if (Stepper_IsRunning(id))
        return JOINT_ERR_BUSY;

    JointResult result = validateConfig(cfg);
    if (result != JOINT_OK)
        return result;

    // Initial configuration must not overwrite a known position.
    if (Stepper_IsPositionValid(id))
        return JOINT_ERR_BUSY;

    jointConfigs[id - 1U] = *cfg;
    motionConfigs[id - 1U] = {};
    configured[id - 1U] = true;
    return JOINT_OK;
}

JointResult Joint_GetConfig(uint8_t id, JointConfig *cfg)
{
    JointResult result = checkJoint(id);
    if (result != JOINT_OK)
        return result;
    if (cfg == nullptr)
        return JOINT_ERR_INVALID_PARAM;

    *cfg = jointConfigs[id - 1U];
    return JOINT_OK;
}

JointResult Joint_UpdateConfig(uint8_t id, const JointConfig *cfg)
{
    JointResult result = checkJoint(id);
    if (result != JOINT_OK)
        return result;
    if (Stepper_IsRunning(id))
        return JOINT_ERR_BUSY;

    result = validateConfig(cfg);
    if (result != JOINT_OK)
        return result;

    const JointConfig old = jointConfigs[id - 1U];
    bool conversionChanged = (old.stepsPerRevolution != cfg->stepsPerRevolution) ||
                             (old.gearRatio != cfg->gearRatio) ||
                             (old.inverted != cfg->inverted);

    // Limit-only changes must retain the current position safely.
    if (!conversionChanged && Stepper_IsPositionValid(id))
    {
        float current = 0.0f;
        result = Joint_GetAngle(id, &current);
        if (result != JOINT_OK)
            return result;
        if (current < cfg->minAngleDeg || current > cfg->maxAngleDeg)
            return JOINT_ERR_OUT_OF_RANGE;
    }

    // Recalculate the existing motion profile before committing changes.
    JointMotionConfig motion = motionConfigs[id - 1U];
    bool motionConfigured = (motion.speedDegS > 0.0f);
    StepperMotionConfig raw = {};

    if (conversionChanged)
        jointConfigs[id - 1U] = *cfg;

    if (motionConfigured)
    {
        result = convertMotion(id, &motion, &raw);
        if (result != JOINT_OK)
        {
            jointConfigs[id - 1U] = old;
            return result;
        }

        if (Stepper_SetMotionConfig(id, &raw) != STEPPER_OK)
        {
            jointConfigs[id - 1U] = old;
            return JOINT_ERR_STEPPER;
        }
    }

    if (conversionChanged)
    {
        // A new step-to-angle conversion invalidates the previous zero reference.
        if (Stepper_InvalidatePosition(id) != STEPPER_OK)
        {
            jointConfigs[id - 1U] = old;
            return JOINT_ERR_STEPPER;
        }
    }

    jointConfigs[id - 1U] = *cfg;
    return JOINT_OK;
}

JointResult Joint_SetMotionConfig(uint8_t id, const JointMotionConfig *cfg)
{
    JointResult result = checkJoint(id);
    if (result != JOINT_OK)
        return result;
    if (Stepper_IsRunning(id))
        return JOINT_ERR_BUSY;

    StepperMotionConfig raw = {};
    result = convertMotion(id, cfg, &raw);
    if (result != JOINT_OK)
        return result;

    if (Stepper_SetMotionConfig(id, &raw) != STEPPER_OK)
        return JOINT_ERR_STEPPER;

    motionConfigs[id - 1U] = *cfg;
    return JOINT_OK;
}

JointResult Joint_GetMotionConfig(uint8_t id, JointMotionConfig *cfg)
{
    JointResult result = checkJoint(id);
    if (result != JOINT_OK)
        return result;
    if (cfg == nullptr)
        return JOINT_ERR_INVALID_PARAM;

    *cfg = motionConfigs[id - 1U];
    return JOINT_OK;
}

JointResult Joint_SetSpeed(uint8_t id, float speedDegS)
{
    JointResult result = checkJoint(id);
    if (result != JOINT_OK)
        return result;

    JointMotionConfig cfg = motionConfigs[id - 1U];
    cfg.speedDegS = speedDegS;
    return Joint_SetMotionConfig(id, &cfg);
}

JointResult Joint_SetAcceleration(uint8_t id, float accelerationDegS2)
{
    JointResult result = checkJoint(id);
    if (result != JOINT_OK)
        return result;

    JointMotionConfig cfg = motionConfigs[id - 1U];
    cfg.accelerationDegS2 = accelerationDegS2;
    return Joint_SetMotionConfig(id, &cfg);
}

JointResult Joint_SetLinearAcceleration(uint8_t id, float angleDeg)
{
    JointResult result = checkJoint(id);
    if (result != JOINT_OK)
        return result;

    JointMotionConfig cfg = motionConfigs[id - 1U];
    cfg.linearAccelerationDeg = angleDeg;
    return Joint_SetMotionConfig(id, &cfg);
}

JointResult Joint_SetZero(uint8_t id)
{
    JointResult result = checkJoint(id);
    if (result != JOINT_OK)
        return result;
    if (Stepper_IsRunning(id))
        return JOINT_ERR_BUSY;

    return (Stepper_SetZero(id) == STEPPER_OK) ? JOINT_OK : JOINT_ERR_STEPPER;
}

JointResult Joint_CheckMoveToAngle(uint8_t id, float angleDeg)
{
    JointResult result = checkJoint(id);
    if (result != JOINT_OK)
        return result;
    if (!Stepper_IsPositionValid(id))
        return JOINT_ERR_POSITION_UNKNOWN;
    if (Stepper_IsRunning(id))
        return JOINT_ERR_BUSY;
    if (!finiteValue(angleDeg))
        return JOINT_ERR_INVALID_PARAM;

    const JointConfig *cfg = &jointConfigs[id - 1U];
    if (angleDeg < cfg->minAngleDeg || angleDeg > cfg->maxAngleDeg)
        return JOINT_ERR_OUT_OF_RANGE;

    int32_t target = 0;
    result = angleToSteps(id, angleDeg, &target);
    if (result != JOINT_OK)
        return result;

    switch (Stepper_CheckMoveToSteps(id, target))
    {
    case STEPPER_OK: return JOINT_OK;
    case STEPPER_ERR_BUSY: return JOINT_ERR_BUSY;
    case STEPPER_ERR_POSITION_UNKNOWN: return JOINT_ERR_POSITION_UNKNOWN;
    case STEPPER_ERR_OUT_OF_RANGE: return JOINT_ERR_OUT_OF_RANGE;
    default: return JOINT_ERR_STEPPER;
    }
}

JointResult Joint_MoveToAngle(uint8_t id, float angleDeg)
{
    JointResult result = Joint_CheckMoveToAngle(id, angleDeg);
    if (result != JOINT_OK)
        return result;
    int32_t target = 0;
    result = angleToSteps(id, angleDeg, &target);
    if (result != JOINT_OK)
        return result;
    return (Stepper_MoveToSteps(id, target) == STEPPER_OK) ? JOINT_OK : JOINT_ERR_STEPPER;
}

JointResult Joint_MoveRelativeAngle(uint8_t id, float deltaDeg)
{
    JointResult result = checkJoint(id);
    if (result != JOINT_OK)
        return result;
    if (!finiteValue(deltaDeg))
        return JOINT_ERR_INVALID_PARAM;
    if (Stepper_IsRunning(id))
        return JOINT_ERR_BUSY;

    float current = 0.0f;
    result = Joint_GetAngle(id, &current);
    if (result != JOINT_OK)
        return result;

    double target = (double)current + (double)deltaDeg;
    if (!isfinite(target) || target > FLT_MAX || target < -FLT_MAX)
        return JOINT_ERR_OUT_OF_RANGE;

    return Joint_MoveToAngle(id, (float)target);
}

JointResult Joint_GetAngle(uint8_t id, float *angleDeg)
{
    JointResult result = checkJoint(id);
    if (result != JOINT_OK)
        return result;
    if (angleDeg == nullptr)
        return JOINT_ERR_INVALID_PARAM;
    if (!Stepper_IsPositionValid(id))
        return JOINT_ERR_POSITION_UNKNOWN;

    int32_t steps = 0;
    if (Stepper_GetPositionSteps(id, &steps) != STEPPER_OK)
        return JOINT_ERR_STEPPER;

    float angle = (float)((double)steps / stepsPerDegree(id));
    if (jointConfigs[id - 1U].inverted)
        angle = -angle;

    *angleDeg = angle;
    return JOINT_OK;
}

JointResult Joint_Stop(uint8_t id, bool hardStop)
{
    JointResult result = checkJoint(id);
    if (result != JOINT_OK)
        return result;

    return (Stepper_Stop(id, hardStop) == STEPPER_OK) ? JOINT_OK : JOINT_ERR_STEPPER;
}

bool Joint_IsConfigured(uint8_t id)
{
    return initialized && validId(id) && configured[id - 1U];
}

bool Joint_IsRunning(uint8_t id)
{
    return Joint_IsConfigured(id) && Stepper_IsRunning(id);
}

bool Joint_IsPositionValid(uint8_t id)
{
    return Joint_IsConfigured(id) && Stepper_IsPositionValid(id);
}
