/**
 * @file    Types.h
 * @brief   Common data types and states.
 * @project SML Arm Firmware
 * @version 0.1.0
 */

#ifndef SML_TYPES_H
#define SML_TYPES_H

#include <stdint.h>

namespace SML
{
    // ================= DATA TYPES =================
    using StepPosition = int32_t;
    using StepSpeed = uint32_t;
    using StepAcceleration = uint32_t;
    using Angle = float;
    using AngularSpeed = float;
    using AngularAcceleration = float;

    // ================= JOINT STATES =================
    enum class JointState : uint8_t
    {
        UNINITIALIZED = 0,
        IDLE,
        MOVING,
        STOPPING,
        HOMING,
        FAULT
    };

    // ================= STOP MODES =================
    enum class StopMode : uint8_t
    {
        SOFT = 0,
        HARD
    };
}

#endif // SML_TYPES_H
