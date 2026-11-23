/**
 * @file    JointController.cpp
 * @brief   Joint configuration, conversion and motion control.
 * @project SML Arm Firmware
 * @version 0.1.0
 */
#include "JointController.h"
#include <cmath>
#include <climits>

namespace SML
{
    namespace
    {
        double scale(const RobotConfig::JointConfig &c)
        {
            return static_cast<double>(c.pulsesPerRevolution) * c.gearRatio / 360.0;
        }

        bool fits(double value)
        {
            return std::isfinite(value) && (value >= -static_cast<double>(INT32_MAX)) &&
                   (value <= static_cast<double>(INT32_MAX));
        }
    }

    JointController::JointController(IStepperDriver &driver)
        : _driver(driver), _config{}, _configured(false),
          _referenceValid(false), _stopping(false), _fault(false)
    {
    }

    ErrorCode JointController::validateConfig(const RobotConfig::JointConfig &c,
                                              StepSpeed &speed, StepAcceleration &acceleration,
                                              uint32_t &linear)
    {
        ErrorCode result = ErrorCode::INVALID_PARAMETER;
        if ((0U != c.pulsesPerRevolution) && std::isfinite(c.gearRatio) &&
            (c.gearRatio > 0.0f) && std::isfinite(c.defaultSpeed) &&
            (c.defaultSpeed > 0.0f) && std::isfinite(c.defaultAcceleration) &&
            (c.defaultAcceleration > 0.0f) && std::isfinite(c.linearAccelerationDistance) &&
            (c.linearAccelerationDistance >= 0.0f) && std::isfinite(c.minAngle) &&
            std::isfinite(c.maxAngle) && (c.minAngle <= 0.0f) &&
            (c.maxAngle >= 0.0f) && (c.minAngle < c.maxAngle))
        {
            const double k = scale(c);
            const double s = std::round(k * c.defaultSpeed);
            const double a = std::round(k * c.defaultAcceleration);
            const double l = std::round(k * c.linearAccelerationDistance);
            result = ErrorCode::OUT_OF_RANGE;
            if (fits(s) && fits(a) && fits(l) && (s >= 1.0) && (a >= 1.0) &&
                fits(k * c.minAngle) && fits(k * c.maxAngle) &&
                ((k * (static_cast<double>(c.maxAngle) - c.minAngle)) <= INT32_MAX))
            {
                speed = static_cast<StepSpeed>(s);
                acceleration = static_cast<StepAcceleration>(a);
                linear = static_cast<uint32_t>(l);
                result = ErrorCode::OK;
            }
        }
        return result;
    }

    ErrorCode JointController::begin(const RobotConfig::JointConfig &config)
    {
        StepSpeed s = 0U;
        StepAcceleration a = 0U;
        uint32_t l = 0U;
        ErrorCode result = ErrorCode::ALREADY_INITIALIZED;
        if (!_configured)
        {
            result = validateConfig(config, s, a, l);
            if ((ErrorCode::OK == result) && !_driver.isInitialized())
            {
                result = _driver.begin();
            }
            if (ErrorCode::OK == result)
            {
                result = setConfig(config);
            }
        }
        return result;
    }

    bool JointController::isInitialized() const
    {
        return _configured && _driver.isInitialized();
    }

    ErrorCode JointController::setConfig(const RobotConfig::JointConfig &config)
    {
        StepSpeed s = 0U;
        StepAcceleration a = 0U;
        uint32_t l = 0U;
        ErrorCode result = ErrorCode::NOT_INITIALIZED;
        if (_driver.isInitialized())
        {
            result = ErrorCode::BUSY;
            if (!_driver.isRunning())
            {
                result = validateConfig(config, s, a, l);
                const bool geometryChanged = !_configured ||
                                             (_config.pulsesPerRevolution != config.pulsesPerRevolution) ||
                                             (_config.gearRatio != config.gearRatio) ||
                                             (_config.invertDirection != config.invertDirection);
                if ((ErrorCode::OK == result) && !geometryChanged && isPositionValid())
                {
                    const double current = _driver.getCurrentPosition() / stepsPerDegree();
                    if ((current < config.minAngle) || (current > config.maxAngle))
                    {
                        result = ErrorCode::LIMIT_REACHED;
                    }
                }
                if (ErrorCode::OK == result)
                {
                    result = _driver.setSpeedHz(s);
                    if (ErrorCode::OK == result)
                        result = _driver.setAcceleration(a);
                    if (ErrorCode::OK == result)
                        result = _driver.setLinearAcceleration(l);
                    if (ErrorCode::OK == result)
                    {
                        if (geometryChanged || _fault)
                            _referenceValid = false;
                        _config = config;
                        _configured = true;
                        _fault = false;
                        _stopping = false;
                    }
                    else
                    {
                        // Partial driver configuration is unsafe for motion.
                        // Apply a complete valid configuration and re-zero to recover.
                        _fault = true;
                        _referenceValid = false;
                    }
                }
            }
        }
        return result;
    }

    const RobotConfig::JointConfig &JointController::getConfig() const { return _config; }

    ErrorCode JointController::setSpeed(AngularSpeed speed)
    {
        auto c = _config;
        c.defaultSpeed = speed;
        return setConfig(c);
    }
    ErrorCode JointController::setAcceleration(AngularAcceleration acceleration)
    {
        auto c = _config;
        c.defaultAcceleration = acceleration;
        return setConfig(c);
    }
    ErrorCode JointController::setLinearAcceleration(Angle distance)
    {
        auto c = _config;
        c.linearAccelerationDistance = distance;
        return setConfig(c);
    }
    ErrorCode JointController::setGearRatio(float ratio)
    {
        auto c = _config;
        c.gearRatio = ratio;
        return setConfig(c);
    }
    ErrorCode JointController::setPulsesPerRevolution(uint32_t pulses)
    {
        auto c = _config;
        c.pulsesPerRevolution = pulses;
        return setConfig(c);
    }

    double JointController::stepsPerDegree() const
    {
        return scale(_config) * (_config.invertDirection ? -1.0 : 1.0);
    }

    ErrorCode JointController::angleToSteps(Angle angle, StepPosition &steps) const
    {
        ErrorCode result = ErrorCode::NOT_INITIALIZED;
        if (isInitialized())
        {
            const double value = std::round(angle * stepsPerDegree());
            result = ErrorCode::OUT_OF_RANGE;
            if (fits(value))
            {
                steps = static_cast<StepPosition>(value);
                result = ErrorCode::OK;
            }
        }
        return result;
    }

    ErrorCode JointController::stepsToAngle(StepPosition steps, Angle &angle) const
    {
        ErrorCode result = ErrorCode::NOT_INITIALIZED;
        if (isInitialized())
        {
            const double value = steps / stepsPerDegree();
            const Angle converted = static_cast<Angle>(value);
            result = ErrorCode::OUT_OF_RANGE;
            if (std::isfinite(converted))
            {
                angle = converted;
                result = ErrorCode::OK;
            }
        }
        return result;
    }

    ErrorCode JointController::moveToDegrees(double angle)
    {
        ErrorCode result = ErrorCode::NOT_INITIALIZED;
        if (isInitialized())
        {
            if (_fault)
                result = ErrorCode::DRIVER_ERROR;
            else if (isRunning())
                result = ErrorCode::BUSY;
            else if (!isPositionValid())
                result = ErrorCode::POSITION_UNKNOWN;
            else if (!std::isfinite(angle))
                result = ErrorCode::INVALID_PARAMETER;
            else if ((angle < _config.minAngle) || (angle > _config.maxAngle))
                result = ErrorCode::LIMIT_REACHED;
            else
            {
                const double k = stepsPerDegree();
                const double low = std::ceil(std::fmin(_config.minAngle * k, _config.maxAngle * k));
                const double high = std::floor(std::fmax(_config.minAngle * k, _config.maxAngle * k));
                const double target = std::fmax(low, std::fmin(high, std::round(angle * k)));
                result = ErrorCode::OUT_OF_RANGE;
                if (fits(target))
                {
                    _stopping = false;
                    result = _driver.moveToSteps(static_cast<StepPosition>(target));
                }
            }
        }
        return result;
    }

    ErrorCode JointController::moveToAngle(Angle angle) { return moveToDegrees(angle); }

    ErrorCode JointController::moveRelativeAngle(Angle angle)
    {
        ErrorCode result = ErrorCode::NOT_INITIALIZED;
        if (isInitialized())
        {
            const double current = _driver.getCurrentPosition() / stepsPerDegree();
            result = moveToDegrees(current + angle);
        }
        return result;
    }

    ErrorCode JointController::setZero()
    {
        ErrorCode result = ErrorCode::NOT_INITIALIZED;
        if (isInitialized())
        {
            if (_fault)
                result = ErrorCode::DRIVER_ERROR;
            else if (isRunning())
                result = ErrorCode::BUSY;
            else
            {
                result = _driver.setCurrentPosition(0);
                _referenceValid = (ErrorCode::OK == result);
                _stopping = false;
            }
        }
        return result;
    }

    ErrorCode JointController::getCurrentAngle(Angle &angle) const
    {
        ErrorCode result = ErrorCode::POSITION_UNKNOWN;
        if (!isInitialized())
            result = ErrorCode::NOT_INITIALIZED;
        else if (isPositionValid())
            result = stepsToAngle(_driver.getCurrentPosition(), angle);
        return result;
    }

    bool JointController::isPositionValid() const
    {
        return isInitialized() && !_fault && _referenceValid && _driver.isPositionValid();
    }

    bool JointController::isRunning() const { return _driver.isRunning(); }

    JointState JointController::getState() const
    {
        JointState state = JointState::UNINITIALIZED;
        if (_fault)
            state = JointState::FAULT;
        else if (isInitialized())
        {
            state = isRunning() ? (_stopping ? JointState::STOPPING : JointState::MOVING)
                                : JointState::IDLE;
        }
        return state;
    }

    ErrorCode JointController::stop(StopMode mode)
    {
        ErrorCode result = _driver.stop(mode);
        if (ErrorCode::OK == result)
        {
            _stopping = isRunning();
            if (StopMode::HARD == mode)
                _referenceValid = false;
        }
        return result;
    }
}
