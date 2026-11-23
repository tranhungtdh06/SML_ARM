/**
 * @file    main.cpp
 * @brief   Firmware application entry point.
 * @project SML Arm Firmware
 * @version 0.1.0
 */
#include <Arduino.h>
#ifdef SML_TEST_SINGLE_JOINT
#include "SingleJointTest.h"
#endif
void setup()
{
#ifdef SML_TEST_SINGLE_JOINT
  SingleJointTest::init();
#endif
}

void loop()
{
#ifdef SML_TEST_SINGLE_JOINT
  SingleJointTest::update();
#endif
}