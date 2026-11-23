
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include "stepper.h"
#include "joint.h"

#define TEST_JOINT_ID 1U
#define TEST_STEP_PIN 5U
#define TEST_DIR_PIN 6U
#define SERIAL_BUFFER_SIZE 64U

static char serialBuffer[SERIAL_BUFFER_SIZE];
static uint8_t serialIndex = 0U;
static bool systemReady = false;
static bool discardLine = false;

static const char *resultText(JointResult result)
{
  switch (result)
  {
  case JOINT_OK:
    return "OK";
  case JOINT_ERR_ID:
    return "INVALID JOINT ID";
  case JOINT_ERR_NOT_INITIALIZED:
    return "NOT INITIALIZED";
  case JOINT_ERR_NOT_CONFIGURED:
    return "NOT CONFIGURED";
  case JOINT_ERR_INVALID_PARAM:
    return "INVALID PARAMETER";
  case JOINT_ERR_BUSY:
    return "JOINT BUSY";
  case JOINT_ERR_POSITION_UNKNOWN:
    return "POSITION UNKNOWN";
  case JOINT_ERR_OUT_OF_RANGE:
    return "OUT OF RANGE";
  case JOINT_ERR_STEPPER:
    return "STEPPER ERROR";
  default:
    return "UNKNOWN ERROR";
  }
}

static void printHelp(void)
{
  Serial.println("HELP | STATUS | ZERO | ANGLE | POS");
  Serial.println("GOTO <deg> | JREL <deg>");
  Serial.println("SPEED <deg/s> | ACCEL <deg/s2> | LINEAR <deg>");
  Serial.println("STOP | ESTOP");
}

static void printStatus(void)
{
  Serial.printf("J%u | Configured: %s | Running: %s | Position valid: %s\n",
                TEST_JOINT_ID,
                Joint_IsConfigured(TEST_JOINT_ID) ? "YES" : "NO",
                Joint_IsRunning(TEST_JOINT_ID) ? "YES" : "NO",
                Joint_IsPositionValid(TEST_JOINT_ID) ? "YES" : "NO");

  float angle = 0.0f;
  int32_t steps = 0;

  if (Joint_GetAngle(TEST_JOINT_ID, &angle) == JOINT_OK &&
      Stepper_GetPositionSteps(TEST_JOINT_ID, &steps) == STEPPER_OK)
  {
    Serial.printf("Position: %ld steps | Angle: %.3f deg\n", (long)steps, angle);
  }
  else
  {
    Serial.println("Position: UNKNOWN | Angle: UNKNOWN");
  }

  JointMotionConfig motion = {};
  if (Joint_GetMotionConfig(TEST_JOINT_ID, &motion) == JOINT_OK)
  {
    Serial.printf("Speed: %.2f deg/s | Accel: %.2f deg/s^2 | Linear: %.2f deg\n",
                  motion.speedDegS, motion.accelerationDegS2,
                  motion.linearAccelerationDeg);
  }
}

static bool parseFloat(const char *text, float *value)
{
  if (text == nullptr || value == nullptr)
    return false;

  while (isspace((unsigned char)*text))
    text++;
  if (*text == '\0')
    return false;

  char *end = nullptr;
  float parsed = strtof(text, &end);

  if (end == text || !isfinite(parsed))
    return false;
  while (isspace((unsigned char)*end))
    end++;
  if (*end != '\0')
    return false;

  *value = parsed;
  return true;
}

static void processCommand(char *input)
{
  char *command = strtok(input, " \t");
  if (command == nullptr)
    return;

  for (char *p = command; *p != '\0'; p++)
  {
    *p = (char)toupper((unsigned char)*p);
  }

  char *argument = strtok(nullptr, "");
  float value = 0.0f;

  if (strcmp(command, "HELP") == 0)
  {
    printHelp();
    return;
  }

  if (strcmp(command, "STATUS") == 0)
  {
    if (systemReady)
      printStatus();
    else
      Serial.println("ERROR: NOT READY");
    return;
  }

  if (!systemReady)
  {
    Serial.println("ERROR: NOT READY");
    return;
  }

  if (strcmp(command, "ZERO") == 0)
  {
    Serial.println(resultText(Joint_SetZero(TEST_JOINT_ID)));
    return;
  }

  if (strcmp(command, "ANGLE") == 0)
  {
    float angle = 0.0f;
    JointResult result = Joint_GetAngle(TEST_JOINT_ID, &angle);

    if (result == JOINT_OK)
      Serial.printf("ANGLE: %.3f deg\n", angle);
    else
      Serial.println(resultText(result));
    return;
  }

  if (strcmp(command, "POS") == 0)
  {
    if (!Joint_IsPositionValid(TEST_JOINT_ID))
    {
      Serial.println("POSITION UNKNOWN");
      return;
    }

    int32_t steps = 0;
    if (Stepper_GetPositionSteps(TEST_JOINT_ID, &steps) == STEPPER_OK)
      Serial.printf("POS: %ld steps\n", (long)steps);
    else
      Serial.println("STEPPER ERROR");
    return;
  }

  if (strcmp(command, "STOP") == 0)
  {
    Serial.println(resultText(Joint_Stop(TEST_JOINT_ID, false)));
    return;
  }

  if (strcmp(command, "ESTOP") == 0)
  {
    Serial.println(resultText(Joint_Stop(TEST_JOINT_ID, true)));
    return;
  }

  if (argument == nullptr || !parseFloat(argument, &value))
  {
    Serial.println("ERROR: INVALID ARGUMENT");
    return;
  }

  JointResult result = JOINT_ERR_INVALID_PARAM;

  if (strcmp(command, "GOTO") == 0)
  {
    result = Joint_MoveToAngle(TEST_JOINT_ID, value);
  }
  else if (strcmp(command, "JREL") == 0)
  {
    result = Joint_MoveRelativeAngle(TEST_JOINT_ID, value);
  }
  else if (strcmp(command, "SPEED") == 0)
  {
    result = Joint_SetSpeed(TEST_JOINT_ID, value);
  }
  else if (strcmp(command, "ACCEL") == 0)
  {
    result = Joint_SetAcceleration(TEST_JOINT_ID, value);
  }
  else if (strcmp(command, "LINEAR") == 0)
  {
    result = Joint_SetLinearAcceleration(TEST_JOINT_ID, value);
  }
  else
  {
    Serial.println("ERROR: UNKNOWN COMMAND");
    return;
  }

  Serial.println(resultText(result));
}

static void readSerial(void)
{
  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();

    if (c == '\r')
      continue;

    if (c == '\n')
    {
      if (!discardLine && serialIndex > 0U)
      {
        serialBuffer[serialIndex] = '\0';
        processCommand(serialBuffer);
      }

      serialIndex = 0U;
      discardLine = false;
    }
    else if (discardLine)
    {
      continue;
    }
    else if (serialIndex < SERIAL_BUFFER_SIZE - 1U)
    {
      serialBuffer[serialIndex++] = c;
    }
    else
    {
      serialIndex = 0U;
      discardLine = true;
      Serial.println("ERROR: COMMAND TOO LONG");
    }
  }
}

void setup(void)
{
  Serial.begin(115200);
  delay(1500);

  if (Stepper_Init() != STEPPER_OK)
  {
    Serial.println("[ERROR] Stepper initialization failed");
    return;
  }

  StepperPins pins = {TEST_STEP_PIN, TEST_DIR_PIN, true, 50U};

  if (Stepper_Attach(TEST_JOINT_ID, &pins) != STEPPER_OK)
  {
    Serial.println("[ERROR] Stepper attach failed");
    return;
  }

  if (Joint_Init() != JOINT_OK)
  {
    Serial.println("[ERROR] Joint initialization failed");
    return;
  }

  JointConfig jointConfig = {800U, 100.0f, -180.0f, 180.0f, false};

  if (Joint_Configure(TEST_JOINT_ID, &jointConfig) != JOINT_OK)
  {
    Serial.println("[ERROR] Joint configuration failed");
    return;
  }

  JointMotionConfig motionConfig = {18.0f, 13.5f, 0.9f};

  if (Joint_SetMotionConfig(TEST_JOINT_ID, &motionConfig) != JOINT_OK)
  {
    Serial.println("[ERROR] Motion configuration failed");
    return;
  }

  systemReady = true;
  Serial.println("[READY] SML Arm Joint V2 - J1");
}

void loop(void)
{
  readSerial();
}
