#ifndef BOARD_PINS_H
#define BOARD_PINS_H

#include <stdint.h>

// ESP32-S3 DevKitC-1 N16R8: STEP/DIR prototype wiring.
#define BOARD_JOINT_COUNT 6U
#define PIN_NOT_ASSIGNED 255U

#define J1_STEP_PIN 5U
#define J1_DIR_PIN  6U
#define J2_STEP_PIN 7U
#define J2_DIR_PIN  8U
#define J3_STEP_PIN 9U
#define J3_DIR_PIN  10U
#define J4_STEP_PIN 11U
#define J4_DIR_PIN  12U
#define J5_STEP_PIN 13U
#define J5_DIR_PIN  14U
#define J6_STEP_PIN 15U
#define J6_DIR_PIN  16U

// ENABLE, limit switches, encoders and hardware E-stop: TBD.
#endif // BOARD_PINS_H
