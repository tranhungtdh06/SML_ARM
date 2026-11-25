
/**
 * @file    main.cpp
 * @brief   Serial test application for SML Arm Firmware.
 * @project SML Arm Firmware
 * @version 0.2.0
 */

#include <Arduino.h>
#include "robot.h"
#include "tool.h"

#define SERIAL_BAUD_RATE 115200U

static bool systemReady = false;

void setup(void)
{
  // Initialize the serial command interface.
  Tool_Init(SERIAL_BAUD_RATE);
  delay(1500);

  // Initialize the robot and its configured joints.
  RobotResult result = Robot_Init();
  if (result != ROBOT_OK)
  {
    Serial.printf("[ERROR] ROBOT_INIT FAILED CODE=%u JOINT=%u\n",
                  (unsigned int)result, (unsigned int)Robot_GetInitFailedJoint());
    return;
  }

  systemReady = true;
  Serial.println("[OK] SML_ARM_READY");
  Serial.println("[INFO] TYPE HELP FOR AVAILABLE COMMANDS");
}

void loop(void)
{
  // Process commands without blocking robot motion.
  Tool_Update();
}
