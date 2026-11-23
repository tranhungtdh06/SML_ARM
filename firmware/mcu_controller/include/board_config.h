/**
 * @file    board_config.h
 * @brief   Hardware pin and board configuration.
 * @project SML Arm Firmware
 * @version 0.1.0
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H
#include <stdint.h>

namespace BoardConfig
{
    // ================= SYSTEM =================
    constexpr uint32_t SERIAL_BAUDRATE = 115200U;
    // ================= MOTOR PINS =================
    constexpr uint8_t JOINT_1_STEP_PIN = 5U;
    constexpr uint8_t JOINT_1_DIR_PIN = 6U;
    // ================= DRIVER TIMING =================
    constexpr uint16_t DIR_DELAY_US = 50U;
    constexpr bool DIR_HIGH_COUNTS_UP = true;
}

#endif // BOARD_CONFIG_H
