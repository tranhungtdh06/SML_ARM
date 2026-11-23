/**
 * @file    SerialCommand.h
 * @brief   Serial interface for single joint testing.
 * @project SML Arm Firmware
 * @version 0.1.0
 */

#ifndef SML_SINGLE_JOINT_SERIAL_COMMAND_H
#define SML_SINGLE_JOINT_SERIAL_COMMAND_H
#include <stdint.h>
#include "JointController.h"

class SerialCommand
{
public:
    explicit SerialCommand(SML::JointController &joint);
    void begin(void);
    void update(void);
    void printHelp(void) const;
    void printStatus(void) const;
    void printConfig(void) const;

private:
    static constexpr uint16_t BUFFER_SIZE = 128U;
    SML::JointController &_joint;
    char _buffer[BUFFER_SIZE];
    uint16_t _length;
    uint8_t _selectedJoint;
    bool _overflow;
    bool _zeroRequired;
    void execute(char *command);
    SML::ErrorCode handleSelect(const char *value);
    SML::ErrorCode handleSet(const char *parameter,const char *value);
    SML::ErrorCode handleMove(const char *mode,const char *value);
    static bool parseFloat(const char *text,float &value);
    static bool parseUnsigned(const char *text,uint32_t &value);
    static void printResult(SML::ErrorCode result);
    static const char *errorToString(SML::ErrorCode result);
    static const char *stateToString(SML::JointState state);
};

#endif // SML_SINGLE_JOINT_SERIAL_COMMAND_H
