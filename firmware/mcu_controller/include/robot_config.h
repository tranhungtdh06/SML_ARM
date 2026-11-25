#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#define ROBOT_JOINT_COUNT 6U

// Enable the software channels; this does not detect connected motors.
#define J1_ENABLED true
#define J2_ENABLED true
#define J3_ENABLED true
#define J4_ENABLED true
#define J5_ENABLED true
#define J6_ENABLED true

// Default test configuration (copied separately to each joint).
#define DEFAULT_STEPS_PER_REV 1600U
#define DEFAULT_GEAR_RATIO 100.0f
#define DEFAULT_MIN_ANGLE_DEG (-180.0f)
#define DEFAULT_MAX_ANGLE_DEG 180.0f
#define DEFAULT_INVERTED false

#define DEFAULT_SPEED_DEG_S 9.0f
#define DEFAULT_ACCEL_DEG_S2 6.75f
#define DEFAULT_LINEAR_ACCEL_DEG 0.45f

#define STEPPER_DIR_DELAY_US 50U
#define STEPPER_DIR_HIGH_COUNTS_UP true

#endif // ROBOT_CONFIG_H
