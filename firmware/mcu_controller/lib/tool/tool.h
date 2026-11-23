/**
 * @file    tool.h
 * @brief   Serial command interface for robot testing.
 * @project SML Arm Firmware
 * @version 0.1.0
 */

#ifndef SML_TOOL_H
#define SML_TOOL_H

#include <stdint.h>
#include <stdbool.h>

#define TOOL_BUFFER_SIZE 128U

// Initialize the serial command interface.
void Tool_Init(uint32_t baudRate);

// Process incoming serial commands without blocking.
void Tool_Update(void);

// Return the current tool initialization state.
bool Tool_IsReady(void);

#endif
