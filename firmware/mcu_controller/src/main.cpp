
/**
 * @file    main.cpp
 * @brief   SML Arm Stepper V2 serial validation.
 * @project SML Arm Firmware
 * @version 0.2.0
 */

#include <Arduino.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "stepper.h"

#define TEST_JOINT_ID 1U
#define TEST_STEP_PIN 5U
#define TEST_DIR_PIN 6U
#define SERIAL_BUFFER_SIZE 64U
#define HEARTBEAT_INTERVAL_MS 5000U

static char s_buffer[SERIAL_BUFFER_SIZE] = {};
static uint8_t s_bufferIndex = 0U;
static bool s_ready = false;
static uint32_t s_lastHeartbeat = 0U;

static void PrintResult(StepperResult result)
{
  if (STEPPER_OK == result)
  {
    Serial.println("OK");
  }
  else
  {
    Serial.printf("ERROR: %d\n", static_cast<int>(result));
  }
}

static bool ParseNumber(const char *text, int32_t *value)
{
  if ((nullptr == text) || (nullptr == value))
  {
    return false;
  }
  errno = 0;
  char *end = nullptr;
  long parsed = strtol(text, &end, 10);
  if ((ERANGE == errno) || (text == end) || ('\0' != *end) || (parsed < INT32_MIN) || (parsed > INT32_MAX))
  {
    return false;
  }
  *value = static_cast<int32_t>(parsed);
  return true;
}

static void PrintStatus(void)
{
  int32_t position = 0;
  StepperMotionConfig config = {};
  Serial.println("----- STEPPER STATUS -----");
  Serial.printf("Joint: J%u\n", TEST_JOINT_ID);
  Serial.printf("Attached: %s\n", Stepper_IsAttached(TEST_JOINT_ID) ? "YES" : "NO");
  Serial.printf("Running: %s\n", Stepper_IsRunning(TEST_JOINT_ID) ? "YES" : "NO");
  Serial.printf("Position valid: %s\n", Stepper_IsPositionValid(TEST_JOINT_ID) ? "YES" : "NO");

  if (STEPPER_OK == Stepper_GetPositionSteps(TEST_JOINT_ID, &position))
  {
    Serial.printf("Position: %ld steps\n", static_cast<long>(position));
  }
  else
  {
    Serial.println("Position: UNKNOWN");
  }
  if (STEPPER_OK == Stepper_GetMotionConfig(TEST_JOINT_ID, &config))
  {
    Serial.printf("Speed: %lu Hz\n", static_cast<unsigned long>(config.speedHz));
    Serial.printf("Acceleration: %lu step/s^2\n", static_cast<unsigned long>(config.accelerationSteps));
    Serial.printf("Linear acceleration: %lu steps\n", static_cast<unsigned long>(config.linearAccelerationSteps));
  }
  Serial.println("--------------------------");
}

static void PrintHelp(void)
{
  Serial.println("Commands:");
  Serial.println("  HELP         - Show commands");
  Serial.println("  STATUS       - Show motor status");
  Serial.println("  ZERO         - Set software zero");
  Serial.println("  POS          - Read estimated position");
  Serial.println("  MOVE <steps> - Move to absolute position");
  Serial.println("  REL <steps>  - Move relative position");
  Serial.println("  SPEED <hz>   - Set speed");
  Serial.println("  ACCEL <val>  - Set acceleration");
  Serial.println("  LINEAR <val> - Set linear acceleration");
  Serial.println("  STOP         - Controlled stop");
  Serial.println("  ESTOP        - Immediate software stop");
}

static void ExecuteCommand(char *command)
{
  char *name = strtok(command, " \t");
  if (nullptr == name)
  {
    return;
  }
  if (0 == strcmp(name, "HELP"))
  {
    PrintHelp();
    return;
  }
  if (0 == strcmp(name, "STATUS"))
  {
    PrintStatus();
    return;
  }
  if (0 == strcmp(name, "ZERO"))
  {
    PrintResult(Stepper_SetZero(TEST_JOINT_ID));
    return;
  }
  if (0 == strcmp(name, "STOP"))
  {
    PrintResult(Stepper_Stop(TEST_JOINT_ID, false));
    return;
  }
  if (0 == strcmp(name, "ESTOP"))
  {
    PrintResult(Stepper_Stop(TEST_JOINT_ID, true));
    return;
  }
  if (0 == strcmp(name, "POS"))
  {
    int32_t position = 0;
    StepperResult result = Stepper_GetPositionSteps(TEST_JOINT_ID, &position);
    if (STEPPER_OK == result)
    {
      Serial.printf("Position: %ld steps\n", static_cast<long>(position));
    }
    else
    {
      PrintResult(result);
    }
    return;
  }

  char *argument = strtok(nullptr, " \t");
  if ((nullptr == argument) || (nullptr != strtok(nullptr, " \t")))
  {
    Serial.println("ERROR: Invalid arguments");
    return;
  }
  int32_t value = 0;
  if (!ParseNumber(argument, &value))
  {
    Serial.println("ERROR: Invalid number");
    return;
  }
  if (0 == strcmp(name, "MOVE"))
  {
    PrintResult(Stepper_MoveToSteps(TEST_JOINT_ID, value));
  }
  else if (0 == strcmp(name, "REL"))
  {
    PrintResult(Stepper_MoveRelativeSteps(TEST_JOINT_ID, value));
  }
  else if (0 == strcmp(name, "SPEED"))
  {
    PrintResult((0 < value) ? Stepper_SetSpeedHz(TEST_JOINT_ID, static_cast<uint32_t>(value)) : STEPPER_ERR_INVALID_PARAM);
  }
  else if (0 == strcmp(name, "ACCEL"))
  {
    PrintResult((0 < value) ? Stepper_SetAcceleration(TEST_JOINT_ID, static_cast<uint32_t>(value)) : STEPPER_ERR_INVALID_PARAM);
  }
  else if (0 == strcmp(name, "LINEAR"))
  {
    PrintResult((0 <= value) ? Stepper_SetLinearAcceleration(TEST_JOINT_ID, static_cast<uint32_t>(value)) : STEPPER_ERR_INVALID_PARAM);
  }
  else
  {
    Serial.println("ERROR: Unknown command");
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1500);
  Serial.println();
  Serial.println("================================");
  Serial.println(" SML Arm Firmware V2");
  Serial.println(" ESP32-S3 Stepper Controller");
  Serial.println("================================");

  Serial.println("[INIT] Stepper engine...");
  StepperResult result = Stepper_Init();
  PrintResult(result);
  if (STEPPER_OK != result)
  {
    return;
  }

  Serial.println("[INIT] Attach J1...");
  StepperPins pins = {TEST_STEP_PIN, TEST_DIR_PIN, true, 50U};
  result = Stepper_Attach(TEST_JOINT_ID, &pins);
  PrintResult(result);
  if (STEPPER_OK != result)
  {
    return;
  }

  Serial.println("[INIT] Configure motion...");
  StepperMotionConfig config = {4000U, 3000U, 200U};
  result = Stepper_SetMotionConfig(TEST_JOINT_ID, &config);
  PrintResult(result);
  if (STEPPER_OK != result)
  {
    return;
  }

  s_ready = true;
  s_lastHeartbeat = millis();
  Serial.println("[READY] Stepper initialized.");
  Serial.println("[INFO] GPIO5=STEP, GPIO6=DIR");
  Serial.println("[INFO] No motor required for software validation.");
  PrintHelp();
  Serial.println("Waiting for command...");
}

void loop()
{
  if ((millis() - s_lastHeartbeat) >= HEARTBEAT_INTERVAL_MS)
  {
    s_lastHeartbeat = millis();
    Serial.printf("[ALIVE] ESP32-S3 running | Stepper: %s\n", s_ready ? "READY" : "NOT READY");
  }

  while (Serial.available() > 0)
  {
    char character = static_cast<char>(Serial.read());
    if (('\n' == character) || ('\r' == character))
    {
      if (0U < s_bufferIndex)
      {
        s_buffer[s_bufferIndex] = '\0';
        if (s_ready)
        {
          ExecuteCommand(s_buffer);
        }
        else
        {
          Serial.println("ERROR: Stepper not initialized");
        }
        s_bufferIndex = 0U;
      }
    }
    else if (s_bufferIndex < (SERIAL_BUFFER_SIZE - 1U))
    {
      s_buffer[s_bufferIndex++] = character;
    }
    else
    {
      s_bufferIndex = 0U;
      Serial.println("ERROR: Command too long");
    }
  }
  delay(1);
}
