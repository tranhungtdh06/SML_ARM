/**
 * @file    robot_config.h
 * @brief   Robot joint and motion configuration.
 * @project SML Arm Firmware
 * @version 0.1.0
 */

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
#include <stdint.h>
namespace RobotConfig
{
    // ================= DATA TYPES =================
    struct JointConfig
    {
        uint32_t pulsesPerRevolution;
        float gearRatio;
        float defaultSpeed;
        float defaultAcceleration;
        float linearAccelerationDistance;
        float minAngle;
        float maxAngle;
        bool invertDirection;
    };

    // ================= ROBOT =================
    constexpr uint8_t MAX_JOINTS = 6U;
    constexpr uint8_t ACTIVE_JOINTS = 1U;

    // ================= JOINT 1 =================
    constexpr JointConfig JOINT_1 =
        {
            1600U,  // Motor pulses per revolution
            100.0f, // Gear ratio
            9.0f,   // Joint speed (deg/s)
            6.75f,  // Joint acceleration (deg/s^2)
            0.45f,  // Linear acceleration distance (deg)
            -90.0f, // Minimum angle (deg)
            90.0f,  // Maximum angle (deg)
            false   // Invert direction
    };

    // ================= JOINT 2 =================
    constexpr JointConfig JOINT_2 =
        {
            1600U,
            100.0f,
            9.0f,
            6.75f,
            0.45f,
            -90.0f,
            90.0f,
            false};

    // ================= JOINT 3 =================
    constexpr JointConfig JOINT_3 =
        {
            1600U,
            100.0f,
            9.0f,
            6.75f,
            0.45f,
            -90.0f,
            90.0f,
            false};

    // ================= JOINT 4 =================
    constexpr JointConfig JOINT_4 =
        {
            1600U,
            100.0f,
            9.0f,
            6.75f,
            0.45f,
            -90.0f,
            90.0f,
            false};

    // ================= JOINT 5 =================
    constexpr JointConfig JOINT_5 =
        {
            1600U,
            100.0f,
            9.0f,
            6.75f,
            0.45f,
            -90.0f,
            90.0f,
            false};

    // ================= JOINT 6 =================
    constexpr JointConfig JOINT_6 =
        {
            1600U,
            100.0f,
            9.0f,
            6.75f,
            0.45f,
            -90.0f,
            90.0f,
            false};

    // ================= JOINT CONFIG TABLE =================
    constexpr JointConfig JOINTS[MAX_JOINTS] =
        {
            JOINT_1,
            JOINT_2,
            JOINT_3,
            JOINT_4,
            JOINT_5,
            JOINT_6};
}

#endif // ROBOT_CONFIG_H
