/**
 * @file    ErrorCode.h
 * @brief   Common firmware error codes.
 * @project SML Arm Firmware
 * @version 0.1.0
 */
#ifndef SML_ERROR_CODE_H
#define SML_ERROR_CODE_H
#include <stdint.h>

namespace SML
{
    enum class ErrorCode : uint8_t
    {
        OK = 0,
        NOT_INITIALIZED,
        ALREADY_INITIALIZED,
        INVALID_PARAMETER,
        OUT_OF_RANGE,
        BUSY,
        DRIVER_ERROR,
        POSITION_UNKNOWN,
        LIMIT_REACHED,
        TIMEOUT,
        EMERGENCY_STOP,
        NOT_SUPPORTED,
        UNKNOWN_ERROR
    };
}

#endif // SML_ERROR_CODE_H
