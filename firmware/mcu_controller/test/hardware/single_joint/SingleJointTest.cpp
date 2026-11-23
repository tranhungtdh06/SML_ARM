/**
 * @file    SingleJointTest.cpp
 * @brief   Single joint hardware validation.
 * @project SML Arm Firmware
 * @version 0.1.0
 */

#include <Arduino.h>
#include <FastAccelStepper.h>

#include "board_config.h"
#include "robot_config.h"

#include "StepperDriver.h"
#include "JointController.h"

#include "SingleJointTest.h"
#include "SerialCommand.h"

namespace SingleJointTest
{
    // ================= HARDWARE =================
    static FastAccelStepperEngine engine;
    static SML::StepperDriver stepper(
        engine,
        BoardConfig::JOINT_1_STEP_PIN,
        BoardConfig::JOINT_1_DIR_PIN,
        BoardConfig::DIR_HIGH_COUNTS_UP,
        BoardConfig::DIR_DELAY_US);
    // ================= CONTROLLER =================
    static SML::JointController joint(stepper);
    static SerialCommand serialCommand(joint);
    // ================= TEST STATE =================
    static bool initialized = false;
    static bool previousRunning = false;

    // ================= INITIALIZATION =================
    void init(void)
    {
        Serial.begin(BoardConfig::SERIAL_BAUDRATE);
        delay(1000);
        Serial.println();
        Serial.println("================================");
        Serial.println(" SML ARM - SINGLE JOINT TEST");
        Serial.println("================================");
        engine.init();
        const SML::ErrorCode result =
            joint.begin(RobotConfig::JOINT_1);
        if (SML::ErrorCode::OK == result)
        {
            initialized = true;
            Serial.println("[OK] Joint initialized");
            serialCommand.begin();
        }
        else
        {
            Serial.print("[ERROR] Initialization failed: ");
            Serial.println(static_cast<uint8_t>(result));
        }
    }

    // ================= TEST UPDATE =================
    void update(void)
    {
        if (initialized)
        {
            serialCommand.update();
            const bool running = joint.isRunning();
            if (previousRunning && !running)
            {
                Serial.println("EVENT MOTION_STOPPED");
                serialCommand.printStatus();
            }
            previousRunning = running;
        }
    }
}