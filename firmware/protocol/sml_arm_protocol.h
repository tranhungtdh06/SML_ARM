#ifndef SML_ARM_PROTOCOL_H_
#define SML_ARM_PROTOCOL_H_

/**
 * @file sml_arm_protocol.h
 * @brief Shared host-to-MCU binary communication protocol for SML-Arm.
 *
 * Designed for deterministic serial (USB-CDC / RS-485) or framed transport
 * between a ROS 2 ros2_control SystemInterface and the embedded MCU controller.
 * All multi-byte fields use little-endian byte ordering.
 */

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SML_PROTOCOL_MAGIC_0 0x53U /* 'S' */
#define SML_PROTOCOL_MAGIC_1 0x4DU /* 'M' */
#define SML_PROTOCOL_VERSION 1U

#define SML_ARM_DOF 6U
#define SML_GRIPPER_JOINTS 2U

typedef enum {
  SML_MSG_CMD_JOINTS = 0x01U,
  SML_MSG_STATE_JOINTS = 0x02U,
  SML_MSG_HOME = 0x03U,
  SML_MSG_ESTOP = 0x04U
} SmlMsgType;

typedef enum {
  SML_STATUS_OK = 0x00U,
  SML_STATUS_HOMED = (1U << 0),
  SML_STATUS_ENABLED = (1U << 1),
  SML_STATUS_ESTOP = (1U << 2),
  SML_STATUS_DRIVER_FAULT = (1U << 3),
  SML_STATUS_LIMIT_HIT = (1U << 4),
  SML_STATUS_WATCHDOG_TRIPPED = (1U << 5)
} SmlStatusFlags;

#if defined(__GNUC__) || defined(__clang__)
#define SML_PACKED __attribute__((packed))
#else
#define SML_PACKED
#pragma pack(push, 1)
#endif

typedef struct SML_PACKED {
  uint8_t magic[2];
  uint8_t version;
  uint8_t msg_type;
  uint16_t seq;
  uint8_t enable_mask; /**< Bits 0..5: dof_joint1..6, Bit 6: gripper */
  uint8_t reserved;
  float joint_pos_rad[SML_ARM_DOF];       /**< Target dof_joint1..6 angles (rad) */
  float joint_vel_rad_s[SML_ARM_DOF];     /**< Feedforward/limit velocities (rad/s) */
  float gripper_pos_m[SML_GRIPPER_JOINTS]; /**< Target finger opening [0.0, 0.040] (m) */
  uint16_t crc16;                          /**< CRC-16-CCITT over preceding bytes */
} SmlArmCommandFrame;

typedef struct SML_PACKED {
  uint8_t magic[2];
  uint8_t version;
  uint8_t msg_type;
  uint16_t seq;
  uint16_t status_flags;                   /**< Bitmask of SmlStatusFlags */
  uint32_t timestamp_us;                   /**< MCU monotonic microsecond clock */
  float joint_pos_rad[SML_ARM_DOF];        /**< Measured dof_joint1..6 angles (rad) */
  float joint_vel_rad_s[SML_ARM_DOF];      /**< Measured dof_joint1..6 velocities (rad/s) */
  float gripper_pos_m[SML_GRIPPER_JOINTS]; /**< Measured finger openings (m) */
  uint16_t crc16;                          /**< CRC-16-CCITT over preceding bytes */
} SmlArmStateFrame;

typedef struct SML_PACKED {
  uint8_t magic[2];
  uint8_t version;
  uint8_t msg_type;
  uint16_t seq;
  uint8_t axis_mask; /**< Bits 0..5: arm axes, Bit 6: gripper, 0x7F: all */
  uint8_t action;    /**< 0: start homing / clear E-Stop, 1: assert E-Stop */
  uint16_t crc16;
} SmlArmControlReqFrame;

#if !defined(__GNUC__) && !defined(__clang__)
#pragma pack(pop)
#endif

/**
 * @brief Compute CRC-16-CCITT (poly 0x1021, init 0xFFFF) over a byte buffer.
 */
static inline uint16_t sml_crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFFU;
  for (size_t i = 0U; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
      if ((crc & 0x8000U) != 0U) {
        crc = (uint16_t)((crc << 1) ^ 0x1021U);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}

#ifdef __cplusplus
}
#endif

#endif /* SML_ARM_PROTOCOL_H_ */
