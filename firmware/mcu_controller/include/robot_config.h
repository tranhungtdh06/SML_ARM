#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>

#define ROBOT_JOINT_COUNT 6U

// J1 - Test configuration
#define J1_STEPS_PER_REV 800U
#define J1_GEAR_RATIO 100.0f

#define J1_MIN_ANGLE_DEG -180.0f
#define J1_MAX_ANGLE_DEG 180.0f

#define J1_INVERTED false

#define J1_SPEED_DEG_S 18.0f
#define J1_ACCEL_DEG_S2 13.5f
#define J1_LINEAR_ACCEL_DEG 0.9f

// DM542 direction timing
#define STEPPER_DIR_DELAY_US 50U

// J2-J6: configure after hardware specifications are confirmed

#endif
