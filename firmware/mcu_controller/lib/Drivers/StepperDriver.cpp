/**
 * @file    StepperDriver.cpp
 * @brief   FastAccelStepper driver implementation.
 * @project SML Arm Firmware
 * @version 0.1.0
 */
#include "StepperDriver.h"

namespace SML
{
    // ================= CONSTRUCTOR =================
    StepperDriver::StepperDriver(
        FastAccelStepperEngine &engine,
        uint8_t stepPin,
        uint8_t dirPin,
        bool dirHighCountsUp,
        uint16_t dirDelayUs)
        : _engine(engine),
          _stepper(nullptr),
          _stepPin(stepPin),
          _dirPin(dirPin),
          _dirHighCountsUp(dirHighCountsUp),
          _dirDelayUs(dirDelayUs),
          _positionValid(false),
          _speed(0U),
          _acceleration(0U),
          _linearAcceleration(0U)
    {
    }

    // ================= INITIALIZATION =================
    ErrorCode StepperDriver::begin()
    {
        if (isInitialized())
        {
            return ErrorCode::ALREADY_INITIALIZED;
        }
        if (_stepPin == _dirPin)
        {
            return ErrorCode::INVALID_PARAMETER;
        }
        _stepper = _engine.stepperConnectToPin(_stepPin);
        if (_stepper == nullptr)
        {
            return ErrorCode::DRIVER_ERROR;
        }
        _stepper->setDirectionPin(_dirPin, _dirHighCountsUp, _dirDelayUs);
        _stepper->setCurrentPosition(0);
        // A reference position must be explicitly established.
        _positionValid = false;
        return ErrorCode::OK;
    }

    bool StepperDriver::isInitialized() const
    {
        return (_stepper != nullptr);
    }

    // ================= MOTION CONTROL =================
    ErrorCode StepperDriver::moveToSteps(StepPosition position)
    {
        if (!isInitialized())
        {
            return ErrorCode::NOT_INITIALIZED;
        }
        if (isRunning())
        {
            return ErrorCode::BUSY;
        }
        if (!_positionValid)
        {
            return ErrorCode::POSITION_UNKNOWN;
        }
        if ((_speed == 0U) || (_acceleration == 0U))
        {
            return ErrorCode::INVALID_PARAMETER;
        }
        int64_t distance = static_cast<int64_t>(position) - getCurrentPosition();
        if ((distance > INT32_MAX) || (distance < -static_cast<int64_t>(INT32_MAX)))
        {
            return ErrorCode::OUT_OF_RANGE;
        }
        if (position == getCurrentPosition())
        {
            return ErrorCode::OK;
        }
        if (MOVE_OK != _stepper->moveTo(position))
        {
            return ErrorCode::DRIVER_ERROR;
        }
        return ErrorCode::OK;
    }

    ErrorCode StepperDriver::moveSteps(StepPosition steps)
    {
        if (!isInitialized())
        {
            return ErrorCode::NOT_INITIALIZED;
        }
        if (isRunning())
        {
            return ErrorCode::BUSY;
        }
        if (!_positionValid)
        {
            return ErrorCode::POSITION_UNKNOWN;
        }
        int64_t target = static_cast<int64_t>(getCurrentPosition()) + static_cast<int64_t>(steps);
        if ((target > INT32_MAX) ||
            (target < INT32_MIN))
        {
            return ErrorCode::OUT_OF_RANGE;
        }
        return moveToSteps(static_cast<StepPosition>(target));
    }

    // ================= CONFIGURATION =================
    ErrorCode StepperDriver::setSpeedHz(StepSpeed speed)
    {
        if (!isInitialized())
        {
            return ErrorCode::NOT_INITIALIZED;
        }
        if (speed == 0U)
        {
            return ErrorCode::INVALID_PARAMETER;
        }
        if (0 != _stepper->setSpeedInHz(speed))
        {
            return ErrorCode::OUT_OF_RANGE;
        }
        _speed = speed;
        _stepper->applySpeedAcceleration();
        return ErrorCode::OK;
    }

    ErrorCode StepperDriver::setAcceleration(
        StepAcceleration acceleration)
    {
        if (!isInitialized())
        {
            return ErrorCode::NOT_INITIALIZED;
        }
        if ((acceleration == 0U) || (acceleration > INT32_MAX))
        {
            return ErrorCode::INVALID_PARAMETER;
        }
        if (0 != _stepper->setAcceleration(static_cast<int32_t>(acceleration)))
        {
            return ErrorCode::DRIVER_ERROR;
        }
        _acceleration = acceleration;
        _stepper->applySpeedAcceleration();
        return ErrorCode::OK;
    }

    ErrorCode StepperDriver::setLinearAcceleration(uint32_t steps)
    {
        if (!isInitialized())
        {
            return ErrorCode::NOT_INITIALIZED;
        }
        _stepper->setLinearAcceleration(steps);
        _linearAcceleration = steps;
        _stepper->applySpeedAcceleration();
        return ErrorCode::OK;
    }

    // ================= POSITION =================
    StepPosition StepperDriver::getCurrentPosition() const
    {
        if (!isInitialized())
        {
            return 0;
        }
        return _stepper->getCurrentPosition();
    }

    ErrorCode StepperDriver::setCurrentPosition(StepPosition position)
    {
        if (!isInitialized())
        {
            return ErrorCode::NOT_INITIALIZED;
        }
        if (isRunning())
        {
            return ErrorCode::BUSY;
        }
        _stepper->setCurrentPosition(position);
        _positionValid = true;
        return ErrorCode::OK;
    }

    bool StepperDriver::isPositionValid() const
    {
        return isInitialized() && _positionValid;
    }

    // ================= STATUS =================
    bool StepperDriver::isRunning() const
    {
        if (!isInitialized())
        {
            return false;
        }
        return _stepper->isRunning();
    }

    StepSpeed StepperDriver::getSpeedHz() const
    {
        return _speed;
    }

    StepAcceleration StepperDriver::getAcceleration() const
    {
        return _acceleration;
    }

    uint32_t StepperDriver::getLinearAcceleration() const
    {
        return _linearAcceleration;
    }

    // ================= STOP CONTROL =================
    ErrorCode StepperDriver::stop(StopMode mode)
    {
        if (!isInitialized())
        {
            return ErrorCode::NOT_INITIALIZED;
        }

        switch (mode)
        {
        case StopMode::SOFT:
        {
            _stepper->stopMove();
            break;
        }

        case StopMode::HARD:
        {
            _stepper->forceStopAndNewPosition(
                _stepper->getCurrentPosition());

            _positionValid = false;
            break;
        }
        default:
        {
            return ErrorCode::INVALID_PARAMETER;
        }
        }
        return ErrorCode::OK;
    }
}
