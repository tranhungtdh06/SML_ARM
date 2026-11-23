/**
 * @file    SerialCommand.cpp
 * @brief   Serial interface for single joint testing.
 * @project SML Arm Firmware
 * @version 0.1.0
 */

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <errno.h>

#include "SerialCommand.h"
#include "robot_config.h"

using SML::ErrorCode;
using SML::JointState;
using SML::StopMode;

// ================= CONSTRUCTOR =================

SerialCommand::SerialCommand(SML::JointController &joint)
    : _joint(joint),
      _buffer{},
      _length(0U),
      _selectedJoint(1U),
      _overflow(false),
      _zeroRequired(true)
{
}

// ================= INITIALIZATION =================

void SerialCommand::begin(void)
{
    _length = 0U;
    _overflow = false;

    printHelp();
    printConfig();
    printStatus();
}

// ================= SERIAL UPDATE =================

void SerialCommand::update(void)
{
    uint16_t processed = 0U;

    while ((Serial.available() > 0) &&
           (processed < BUFFER_SIZE))
    {
        const char character =
            static_cast<char>(Serial.read());

        processed++;

        if (('\n' == character) || ('\r' == character))
        {
            if (_overflow)
            {
                Serial.println("ERR BUFFER_OVERFLOW");
            }
            else if (_length > 0U)
            {
                _buffer[_length] = '\0';
                execute(_buffer);
            }

            _length = 0U;
            _overflow = false;
        }
        else if (!_overflow)
        {
            if (_length < BUFFER_SIZE - 1U)
            {
                _buffer[_length] = character;
                _length++;
            }
            else
            {
                _overflow = true;
            }
        }
    }
}

// ================= COMMAND EXECUTION =================

void SerialCommand::execute(char *command)
{
    ErrorCode result = ErrorCode::INVALID_PARAMETER;

    for (char *p = command; '\0' != *p; p++)
    {
        *p = static_cast<char>(
            toupper(static_cast<unsigned char>(*p)));
    }

    char *context = nullptr;

    char *operation = strtok_r(command, " \t", &context);
    char *arg1 = strtok_r(nullptr, " \t", &context);
    char *arg2 = strtok_r(nullptr, " \t", &context);
    char *arg3 = strtok_r(nullptr, " \t", &context);

    if (nullptr == operation)
    {
        // Empty command.
    }
    else if (nullptr != arg3)
    {
        printResult(ErrorCode::INVALID_PARAMETER);
    }
    else if ((0 == strcmp(operation, "HELP")) &&
             (nullptr == arg1))
    {
        printHelp();
    }
    else if ((0 == strcmp(operation, "SELECT")) &&
             (nullptr != arg1) &&
             (nullptr == arg2))
    {
        printResult(handleSelect(arg1));
    }
    else if ((0 == strcmp(operation, "SET")) &&
             (nullptr != arg1) &&
             (nullptr != arg2))
    {
        printResult(handleSet(arg1, arg2));
    }
    else if ((0 == strcmp(operation, "MOVE")) &&
             (nullptr != arg1) &&
             (nullptr != arg2))
    {
        printResult(handleMove(arg1, arg2));
    }
    else if ((0 == strcmp(operation, "GET")) &&
             (nullptr != arg1) &&
             (nullptr == arg2))
    {
        if (0 == strcmp(arg1, "CONFIG"))
        {
            printConfig();
        }
        else if (0 == strcmp(arg1, "STATUS"))
        {
            printStatus();
        }
        else
        {
            printResult(ErrorCode::INVALID_PARAMETER);
        }
    }
    else if ((0 == strcmp(operation, "ZERO")) &&
             (nullptr == arg1))
    {
        result = _joint.setZero();

        if (ErrorCode::OK == result)
        {
            _zeroRequired = false;
        }

        printResult(result);
    }
    else if ((0 == strcmp(operation, "STOP")) &&
             (nullptr == arg1))
    {
        printResult(_joint.stop(StopMode::SOFT));
    }
    else if ((0 == strcmp(operation, "HARD_STOP")) &&
             (nullptr == arg1))
    {
        _zeroRequired = true;

        printResult(_joint.stop(StopMode::HARD));
    }
    else
    {
        printResult(ErrorCode::INVALID_PARAMETER);
    }
}

// ================= JOINT SELECTION =================

ErrorCode SerialCommand::handleSelect(const char *value)
{
    uint32_t jointId = 0U;

    ErrorCode result = ErrorCode::INVALID_PARAMETER;

    if (parseUnsigned(value, jointId))
    {
        if ((jointId >= 1U) &&
            (jointId <= RobotConfig::MAX_JOINTS))
        {
            result = _joint.setConfig(
                RobotConfig::JOINTS[jointId - 1U]);

            if (ErrorCode::OK == result)
            {
                _selectedJoint =
                    static_cast<uint8_t>(jointId);

                _zeroRequired = true;
            }
        }
        else
        {
            result = ErrorCode::OUT_OF_RANGE;
        }
    }

    return result;
}

// ================= CONFIGURATION =================

ErrorCode SerialCommand::handleSet(
    const char *parameter,
    const char *value)
{
    ErrorCode result = ErrorCode::INVALID_PARAMETER;

    float number = 0.0f;
    uint32_t integer = 0U;

    RobotConfig::JointConfig config =
        _joint.getConfig();

    if (0 == strcmp(parameter, "PPR"))
    {
        if (parseUnsigned(value, integer))
        {
            result = _joint.setPulsesPerRevolution(integer);
        }
    }
    else if (0 == strcmp(parameter, "INVERT"))
    {
        if (parseUnsigned(value, integer) &&
            (integer <= 1U))
        {
            config.invertDirection = (1U == integer);

            result = _joint.setConfig(config);
        }
    }
    else if (parseFloat(value, number))
    {
        if (0 == strcmp(parameter, "RATIO"))
        {
            result = _joint.setGearRatio(number);
        }
        else if (0 == strcmp(parameter, "SPEED"))
        {
            result = _joint.setSpeed(number);
        }
        else if (0 == strcmp(parameter, "ACCEL"))
        {
            result = _joint.setAcceleration(number);
        }
        else if (0 == strcmp(parameter, "LINEAR"))
        {
            result = _joint.setLinearAcceleration(number);
        }
        else if (0 == strcmp(parameter, "MIN"))
        {
            config.minAngle = number;

            result = _joint.setConfig(config);
        }
        else if (0 == strcmp(parameter, "MAX"))
        {
            config.maxAngle = number;

            result = _joint.setConfig(config);
        }
    }

    if (!_joint.isPositionValid())
    {
        _zeroRequired = true;
    }

    return result;
}

// ================= MOTION CONTROL =================

ErrorCode SerialCommand::handleMove(
    const char *mode,
    const char *value)
{
    float angle = 0.0f;

    ErrorCode result = ErrorCode::INVALID_PARAMETER;

    if (_zeroRequired)
    {
        result = ErrorCode::POSITION_UNKNOWN;
    }
    else if (parseFloat(value, angle))
    {
        if (0 == strcmp(mode, "ABS"))
        {
            result = _joint.moveToAngle(angle);
        }
        else if (0 == strcmp(mode, "REL"))
        {
            result = _joint.moveRelativeAngle(angle);
        }
    }

    return result;
}

// ================= NUMBER PARSING =================
bool SerialCommand::parseFloat(
    const char *text,
    float &value)
{
    char *end = nullptr;
    errno = 0;
    const float parsed = strtof(text, &end);

    const bool valid =
        (end != text) &&
        ('\0' == *end) &&
        (0 == errno) &&
        isfinite(parsed);

    if (valid)
    {
        value = parsed;
    }

    return valid;
}

bool SerialCommand::parseUnsigned(
    const char *text,
    uint32_t &value)
{
    bool valid = false;
    if (('\0' != *text) && ('-' != *text))
    {
        char *end = nullptr;
        errno = 0;
        const unsigned long parsed =
            strtoul(text, &end, 10);
        valid =
            (end != text) &&
            ('\0' == *end) &&
            (0 == errno) &&
            (parsed <= UINT32_MAX);
        if (valid)
        {
            value = static_cast<uint32_t>(parsed);
        }
    }
    return valid;
}

// ================= CONFIGURATION OUTPUT =================
void SerialCommand::printConfig(void) const
{
    const RobotConfig::JointConfig &config =_joint.getConfig();
    Serial.println("CONFIG_BEGIN");
    Serial.printf("JOINT=%u\n", _selectedJoint);
    Serial.printf("PPR=%lu\n",static_cast<unsigned long>(config.pulsesPerRevolution));
    Serial.printf("RATIO=%.4f\n", config.gearRatio);
    Serial.printf("SPEED=%.4f\n", config.defaultSpeed);
    Serial.printf("ACCEL=%.4f\n", config.defaultAcceleration);
    Serial.printf("LINEAR=%.4f\n", config.linearAccelerationDistance);
    Serial.printf("MIN=%.4f\n", config.minAngle);
    Serial.printf("MAX=%.4f\n", config.maxAngle);
    Serial.printf(
        "INVERT=%u\n",
        config.invertDirection ? 1U : 0U);
    const double stepsPerDegree =
        static_cast<double>(config.pulsesPerRevolution) *
        config.gearRatio / 360.0;
    Serial.printf(
        "STEPS_PER_DEGREE=%.6f\n",
        stepsPerDegree);
    Serial.printf(
        "STEP_FREQUENCY_HZ=%.0f\n",
        round(stepsPerDegree * config.defaultSpeed));
    Serial.println("CONFIG_END");
}

// ================= STATUS OUTPUT =================
void SerialCommand::printStatus(void) const
{
    SML::Angle angle = 0.0f;
    const ErrorCode result =
        _joint.getCurrentAngle(angle);
    const bool valid =
        !_zeroRequired &&
        (ErrorCode::OK == result);
    Serial.println("STATUS_BEGIN");
    Serial.printf("JOINT=%u\n", _selectedJoint);
    Serial.printf(
        "STATE=%s\n",
        stateToString(_joint.getState()));
    if (valid)
    {
        Serial.printf("ANGLE=%.4f\n", angle);
    }
    else
    {
        Serial.println("ANGLE=UNKNOWN");
    }
    Serial.printf(
        "RUNNING=%u\n",
        _joint.isRunning() ? 1U : 0U);
    Serial.printf(
        "POSITION_VALID=%u\n",
        valid ? 1U : 0U);
    Serial.println("STATUS_END");
}

// ================= RESULT OUTPUT =================
void SerialCommand::printResult(ErrorCode result)
{
    if (ErrorCode::OK == result)
    {
        Serial.println("OK");
    }
    else
    {
        Serial.printf(
            "ERR %u %s\n",
            static_cast<unsigned>(result),
            errorToString(result));
    }
}

// ================= ERROR DESCRIPTION =================
const char *SerialCommand::errorToString(ErrorCode result)
{
    const char *message = "UNKNOWN_ERROR";
    switch (result)
    {
    case ErrorCode::OK:
        message = "OK";
        break;
    case ErrorCode::NOT_INITIALIZED:
        message = "NOT_INITIALIZED";
        break;
    case ErrorCode::ALREADY_INITIALIZED:
        message = "ALREADY_INITIALIZED";
        break;
    case ErrorCode::INVALID_PARAMETER:
        message = "INVALID_PARAMETER";
        break;
    case ErrorCode::OUT_OF_RANGE:
        message = "OUT_OF_RANGE";
        break;
    case ErrorCode::BUSY:
        message = "BUSY";
        break;
    case ErrorCode::DRIVER_ERROR:
        message = "DRIVER_ERROR";
        break;
    case ErrorCode::POSITION_UNKNOWN:
        message = "POSITION_UNKNOWN";
        break;
    case ErrorCode::LIMIT_REACHED:
        message = "LIMIT_REACHED";
        break;
    case ErrorCode::TIMEOUT:
        message = "TIMEOUT";
        break;
    case ErrorCode::EMERGENCY_STOP:
        message = "EMERGENCY_STOP";
        break;
    case ErrorCode::NOT_SUPPORTED:
        message = "NOT_SUPPORTED";
        break;
    default:
        break;
    }
    return message;
}

// ================= STATE DESCRIPTION =================
const char *SerialCommand::stateToString(JointState state)
{
    const char *message = "UNKNOWN";
    switch (state)
    {
    case JointState::UNINITIALIZED:
        message = "UNINITIALIZED";
        break;
    case JointState::IDLE:
        message = "IDLE";
        break;
    case JointState::MOVING:
        message = "MOVING";
        break;
    case JointState::STOPPING:
        message = "STOPPING";
        break;
    case JointState::HOMING:
        message = "HOMING";
        break;
    case JointState::FAULT:
        message = "FAULT";
        break;
    default:
        break;
    }
    return message;
}

// ================= HELP =================
void SerialCommand::printHelp(void) const
{
    Serial.println();
    Serial.println("========== COMMANDS ==========");
    Serial.println("HELP");
    Serial.println("SELECT <1-6>");
    Serial.println("SET PPR <pulse/rev>");
    Serial.println("SET RATIO <ratio>");
    Serial.println("SET SPEED <deg/s>");
    Serial.println("SET ACCEL <deg/s^2>");
    Serial.println("SET LINEAR <deg>");
    Serial.println("SET MIN <deg>");
    Serial.println("SET MAX <deg>");
    Serial.println("SET INVERT <0|1>");
    Serial.println("MOVE ABS <deg>");
    Serial.println("MOVE REL <deg>");
    Serial.println("GET CONFIG");
    Serial.println("GET STATUS");
    Serial.println("ZERO");
    Serial.println("STOP");
    Serial.println("HARD_STOP");
    Serial.println("==============================");
    Serial.println();
}
