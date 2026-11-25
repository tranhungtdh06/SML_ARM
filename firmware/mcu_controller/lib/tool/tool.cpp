/**
 * @file    tool.cpp
 * @brief   Serial command parser and Robot Manager interface.
 * @project SML Arm Firmware
 * @version 0.1.0
 */

#include "tool.h"
#include "robot.h"
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <errno.h>
#include <limits.h>

#define TOOL_MAX_TOKENS (1U + 2U * ROBOT_MAX_JOINTS)

static char s_buffer[TOOL_BUFFER_SIZE] = {};
static uint16_t s_index = 0U;
static bool s_discardLine = false;
static bool s_initialized = false;

static const char *Tool_ResultText(RobotResult result)
{
    switch (result)
    {
    case ROBOT_OK:
        return "OK";
    case ROBOT_ERR_ID:
        return "INVALID_JOINT_ID";
    case ROBOT_ERR_NOT_INITIALIZED:
        return "NOT_INITIALIZED";
    case ROBOT_ERR_NOT_AVAILABLE:
        return "JOINT_NOT_AVAILABLE";
    case ROBOT_ERR_INVALID_PARAM:
        return "INVALID_PARAMETER";
    case ROBOT_ERR_BUSY:
        return "JOINT_BUSY";
    case ROBOT_ERR_POSITION_UNKNOWN:
        return "POSITION_UNKNOWN";
    case ROBOT_ERR_OUT_OF_RANGE:
        return "OUT_OF_RANGE";
    case ROBOT_ERR_DRIVER:
        return "DRIVER_ERROR";
    default:
        return "UNKNOWN_ERROR";
    }
}

static void Tool_PrintResult(RobotResult result, const char *command, uint8_t id)
{
    if (result == ROBOT_OK)
        Serial.printf("[OK] %s J%u\n", command, id);
    else
        Serial.printf("[ERROR] %s J%u %s\n", command, id, Tool_ResultText(result));
}

static bool Tool_ParseFloat(const char *text, float *value)
{
    if (text == nullptr || value == nullptr || *text == '\0')
        return false;

    errno = 0;
    char *end = nullptr;
    float parsed = strtof(text, &end);
    if (end == text || *end != '\0' || errno == ERANGE || !isfinite(parsed))
        return false;

    *value = parsed;
    return true;
}

static bool Tool_ParseUint32(const char *text, uint32_t *value)
{
    if (text == nullptr || value == nullptr || *text == '\0' || !isdigit((unsigned char)*text))
        return false;

    errno = 0;
    char *end = nullptr;
    unsigned long parsed = strtoul(text, &end, 10);
    if (end == text || *end != '\0' || errno == ERANGE || parsed == 0UL || parsed > UINT32_MAX)
        return false;

    *value = (uint32_t)parsed;
    return true;
}

static bool Tool_ParseJointId(const char *text, uint8_t *id)
{
    uint32_t value = 0U;
    if (id == nullptr || !Tool_ParseUint32(text, &value) || value > ROBOT_MAX_JOINTS)
        return false;

    *id = (uint8_t)value;
    return true;
}

static void Tool_PrintHelp(void)
{
    Serial.println("[OK] HELP");
    Serial.println("PING | HELP | STATUS <id> | CONFIG <id>");
    Serial.println("ZERO <id> | ANGLE <id>");
    Serial.println("GOTO <id> <deg> | JREL <id> <deg>");
    Serial.println("GROUP <id> <deg> <id> <deg> ... (2-6 joints, independent profiles)");
    Serial.println("PPR <id> <steps> | GEAR <id> <ratio>");
    Serial.println("INVERT <id> <0|1> | LIMIT <id> <min> <max>");
    Serial.println("SPEED <id> <deg/s> | ACCEL <id> <deg/s2>");
    Serial.println("LINEAR <id> <deg> | STOP <id> | STOP ALL");
    Serial.println("ESTOP <id> | ESTOP ALL");
}

static void Tool_PrintStatus(uint8_t id)
{
    RobotJointStatus status = {};
    RobotResult result = Robot_GetStatus(id, &status);
    if (result != ROBOT_OK)
    {
        Tool_PrintResult(result, "STATUS", id);
        return;
    }

    Serial.printf("[OK] STATUS J%u\n", id);
    Serial.printf("AVAILABLE=%u RUNNING=%u POSITION_VALID=%u\n",
                  status.available ? 1U : 0U,
                  status.running ? 1U : 0U,
                  status.positionValid ? 1U : 0U);

    if (status.positionValid)
        Serial.printf("ANGLE=%.4f\n", status.angleDeg);
    else
        Serial.printf("[WARN] POSITION_UNKNOWN J%u\n", id);
}

static void Tool_PrintConfig(uint8_t id)
{
    JointConfig config = {};
    JointMotionConfig motion = {};
    RobotResult result = Robot_GetConfig(id, &config);
    if (result != ROBOT_OK)
    {
        Tool_PrintResult(result, "CONFIG", id);
        return;
    }

    result = Robot_GetMotionConfig(id, &motion);
    if (result != ROBOT_OK)
    {
        Tool_PrintResult(result, "CONFIG", id);
        return;
    }

    Serial.printf("[OK] CONFIG J%u\n", id);
    Serial.printf("PPR=%lu GEAR=%.6f INVERT=%u\n",
                  (unsigned long)config.stepsPerRevolution,
                  config.gearRatio, config.inverted ? 1U : 0U);
    Serial.printf("MIN=%.4f MAX=%.4f\n", config.minAngleDeg, config.maxAngleDeg);
    Serial.printf("SPEED=%.4f ACCEL=%.4f LINEAR=%.4f\n",
                  motion.speedDegS, motion.accelerationDegS2,
                  motion.linearAccelerationDeg);
}

static void Tool_UpdateMechanical(uint8_t id, const char *command, char *args[], uint8_t count)
{
    JointConfig config = {};
    RobotResult result = Robot_GetConfig(id, &config);
    if (result != ROBOT_OK)
    {
        Tool_PrintResult(result, command, id);
        return;
    }

    float value = 0.0f;
    if (strcmp(command, "LIMIT") == 0)
    {
        float minimum = 0.0f;
        float maximum = 0.0f;
        if (count != 2U || !Tool_ParseFloat(args[0], &minimum) || !Tool_ParseFloat(args[1], &maximum))
        {
            Serial.println("[ERROR] INVALID_ARGUMENT");
            return;
        }
        config.minAngleDeg = minimum;
        config.maxAngleDeg = maximum;
    }
    else if (count != 1U)
    {
        Serial.println("[ERROR] INVALID_ARGUMENT");
        return;
    }
    else if (strcmp(command, "PPR") == 0)
    {
        uint32_t ppr = 0U;
        if (!Tool_ParseUint32(args[0], &ppr))
        {
            Serial.println("[ERROR] INVALID_PPR");
            return;
        }
        config.stepsPerRevolution = ppr;
    }
    else if (strcmp(command, "INVERT") == 0)
    {
        if (strcmp(args[0], "0") != 0 && strcmp(args[0], "1") != 0)
        {
            Serial.println("[ERROR] INVALID_INVERT");
            return;
        }
        config.inverted = (strcmp(args[0], "1") == 0);
    }
    else
    {
        if (!Tool_ParseFloat(args[0], &value))
        {
            Serial.println("[ERROR] INVALID_ARGUMENT");
            return;
        }
        config.gearRatio = value;
    }

    result = Robot_UpdateConfig(id, &config);
    Tool_PrintResult(result, command, id);

    if (result == ROBOT_OK && !Robot_IsJointAvailable(id))
        Serial.printf("[WARN] JOINT_NOT_AVAILABLE J%u\n", id);
    else if (result == ROBOT_OK)
    {
        RobotJointStatus status = {};
        if (Robot_GetStatus(id, &status) == ROBOT_OK && !status.positionValid)
            Serial.printf("[WARN] POSITION_UNKNOWN J%u\n", id);
    }
}

static void Tool_UpdateMotion(uint8_t id, const char *command, float value)
{
    JointMotionConfig motion = {};
    RobotResult result = Robot_GetMotionConfig(id, &motion);
    if (result != ROBOT_OK)
    {
        Tool_PrintResult(result, command, id);
        return;
    }

    if (strcmp(command, "SPEED") == 0)
        motion.speedDegS = value;
    else if (strcmp(command, "ACCEL") == 0)
        motion.accelerationDegS2 = value;
    else
        motion.linearAccelerationDeg = value;

    result = Robot_SetMotionConfig(id, &motion);
    Tool_PrintResult(result, command, id);
}

static void Tool_ProcessCommand(char *input)
{
    char *tokens[TOOL_MAX_TOKENS] = {};
    uint8_t count = 0U;
    char *token = strtok(input, " \t");

    while (token != nullptr)
    {
        if (count >= TOOL_MAX_TOKENS)
        {
            Serial.println("[ERROR] TOO_MANY_ARGUMENTS");
            return;
        }
        tokens[count++] = token;
        token = strtok(nullptr, " \t");
    }

    if (count == 0U)
        return;

    for (char *p = tokens[0]; *p != '\0'; p++)
        *p = (char)toupper((unsigned char)*p);

    const char *command = tokens[0];

    if (strcmp(command, "HELP") == 0 || strcmp(command, "PING") == 0)
    {
        if (count != 1U)
        {
            Serial.println("[ERROR] INVALID_ARGUMENT");
            return;
        }
        if (strcmp(command, "HELP") == 0)
            Tool_PrintHelp();
        else
            Serial.println(Robot_IsReady() ? "[OK] PONG" : "[ERROR] ROBOT_NOT_READY");
        return;
    }

    if (!Robot_IsReady())
    {
        Serial.println("[ERROR] ROBOT_NOT_READY");
        return;
    }

    if (strcmp(command, "STOP") == 0 || strcmp(command, "ESTOP") == 0)
    {
        if (count != 2U)
        {
            Serial.println("[ERROR] INVALID_ARGUMENT");
            return;
        }

        bool hardStop = (strcmp(command, "ESTOP") == 0);
        if (strcmp(tokens[1], "ALL") == 0)
        {
            RobotResult result = Robot_StopAll(hardStop);
            if (result == ROBOT_OK)
                Serial.printf("[OK] %s ALL\n", command);
            else
                Serial.printf("[ERROR] %s ALL %s\n", command, Tool_ResultText(result));
            return;
        }

        uint8_t id = 0U;
        if (!Tool_ParseJointId(tokens[1], &id))
        {
            Serial.println("[ERROR] INVALID_JOINT_ID");
            return;
        }
        Tool_PrintResult(Robot_Stop(id, hardStop), command, id);
        return;
    }

    if (strcmp(command, "GROUP") == 0)
    {
        if (count < 5U || (count % 2U) == 0U)
        {
            Serial.println("[ERROR] GROUP INVALID_ARGUMENT");
            return;
        }
        RobotJointTarget targets[ROBOT_MAX_JOINTS] = {};
        const uint8_t n = (count - 1U) / 2U;
        for (uint8_t i = 0U; i < n; ++i)
        {
            if (!Tool_ParseJointId(tokens[1U + 2U * i], &targets[i].jointId) ||
                !Tool_ParseFloat(tokens[2U + 2U * i], &targets[i].angleDeg))
            {
                Serial.println("[ERROR] GROUP INVALID_ARGUMENT");
                return;
            }
        }
        uint8_t failed = 0U;
        RobotResult result = Robot_MoveGroup(targets, n, &failed);
        if (result == ROBOT_OK)
            Serial.println("[OK] GROUP");
        else
            Serial.printf("[ERROR] GROUP J%u %s\n", failed, Tool_ResultText(result));
        return;
    }

    if (count < 2U)
    {
        Serial.println("[ERROR] MISSING_JOINT_ID");
        return;
    }

    uint8_t id = 0U;
    if (!Tool_ParseJointId(tokens[1], &id))
    {
        Serial.println("[ERROR] INVALID_JOINT_ID");
        return;
    }

    if (strcmp(command, "STATUS") == 0 || strcmp(command, "CONFIG") == 0 ||
        strcmp(command, "ZERO") == 0 || strcmp(command, "ANGLE") == 0)
    {
        if (count != 2U)
        {
            Serial.println("[ERROR] INVALID_ARGUMENT");
            return;
        }

        if (strcmp(command, "STATUS") == 0)
            Tool_PrintStatus(id);
        else if (strcmp(command, "CONFIG") == 0)
            Tool_PrintConfig(id);
        else if (strcmp(command, "ZERO") == 0)
            Tool_PrintResult(Robot_SetZero(id), "ZERO", id);
        else
        {
            float angle = 0.0f;
            RobotResult result = Robot_GetAngle(id, &angle);
            if (result == ROBOT_OK)
                Serial.printf("[OK] ANGLE J%u %.4f\n", id, angle);
            else
                Tool_PrintResult(result, "ANGLE", id);
        }
        return;
    }

    if (strcmp(command, "PPR") == 0 || strcmp(command, "GEAR") == 0 ||
        strcmp(command, "INVERT") == 0 || strcmp(command, "LIMIT") == 0)
    {
        Tool_UpdateMechanical(id, command, &tokens[2], count - 2U);
        return;
    }

    if (strcmp(command, "GOTO") != 0 && strcmp(command, "JREL") != 0 &&
        strcmp(command, "SPEED") != 0 && strcmp(command, "ACCEL") != 0 &&
        strcmp(command, "LINEAR") != 0)
    {
        Serial.println("[ERROR] UNKNOWN_COMMAND");
        return;
    }

    float value = 0.0f;
    if (count != 3U || !Tool_ParseFloat(tokens[2], &value))
    {
        Serial.println("[ERROR] INVALID_ARGUMENT");
        return;
    }

    if (strcmp(command, "GOTO") == 0)
        Tool_PrintResult(Robot_MoveToAngle(id, value), "GOTO", id);
    else if (strcmp(command, "JREL") == 0)
        Tool_PrintResult(Robot_MoveRelativeAngle(id, value), "JREL", id);
    else
        Tool_UpdateMotion(id, command, value);
}

void Tool_Init(uint32_t baudRate)
{
    if (s_initialized || baudRate == 0U)
        return;

    Serial.begin(baudRate);
    s_index = 0U;
    s_discardLine = false;
    s_initialized = true;
}

void Tool_Update(void)
{
    if (!s_initialized)
        return;

    while (Serial.available() > 0)
    {
        char c = (char)Serial.read();
        if (c == '\r')
            continue;

        if (c == '\n')
        {
            if (!s_discardLine && s_index > 0U)
            {
                s_buffer[s_index] = '\0';
                Tool_ProcessCommand(s_buffer);
            }
            s_index = 0U;
            s_discardLine = false;
        }
        else if (s_discardLine)
        {
            continue;
        }
        else if (s_index < TOOL_BUFFER_SIZE - 1U)
        {
            s_buffer[s_index++] = c;
        }
        else
        {
            s_index = 0U;
            s_discardLine = true;
            Serial.println("[ERROR] COMMAND_TOO_LONG");
        }
    }
}

bool Tool_IsReady(void)
{
    return s_initialized;
}
