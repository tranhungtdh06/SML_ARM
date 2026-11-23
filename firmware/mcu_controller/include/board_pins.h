#ifndef BOARD_PINS_H
#define BOARD_PINS_H

#include <stdint.h>

// ESP32-S3 N16R8
#define BOARD_JOINT_COUNT 6U

// J1 - Current test wiring
#define J1_STEP_PIN 5U
#define J1_DIR_PIN 6U

// J2-J6 - Not assigned yet
#define PIN_NOT_ASSIGNED 255U

#define J2_STEP_PIN PIN_NOT_ASSIGNED
#define J2_DIR_PIN PIN_NOT_ASSIGNED

#define J3_STEP_PIN PIN_NOT_ASSIGNED
#define J3_DIR_PIN PIN_NOT_ASSIGNED

#define J4_STEP_PIN PIN_NOT_ASSIGNED
#define J4_DIR_PIN PIN_NOT_ASSIGNED

#define J5_STEP_PIN PIN_NOT_ASSIGNED
#define J5_DIR_PIN PIN_NOT_ASSIGNED

#define J6_STEP_PIN PIN_NOT_ASSIGNED
#define J6_DIR_PIN PIN_NOT_ASSIGNED

// Future hardware
// ENABLE, limit switches and encoders: TBD

#endif
