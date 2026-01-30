/**
 * @file    motor_if.hpp
 * @author  syhanjin
 * @date    2026-01-28
 */
#ifndef I_MOTOR_HPP
#define I_MOTOR_HPP
#include <cassert>

namespace motors
{
class IMotor;
} // namespace motors

namespace controllers
{
enum class ControlMode
{
    Default, // use motor's defaultControlMode()
    ExternalPID,
    InternalVelocity,
    InternalVelPos,
};

class IController;
} // namespace controllers

namespace motors
{
class IMotor
{
public:
    IMotor() : controller_(nullptr) {}
    virtual ~IMotor() = default;

    // default control mode for this motor. Controllers will use this unless user provides explicit
    // override in controller config.
    virtual controllers::ControlMode defaultControlMode() const
    {
        return controllers::ControlMode::ExternalPID;
    }

    // basic feedback
    virtual float getAngle() const    = 0;
    virtual float getVelocity() const = 0;
    virtual void  resetAngle()        = 0;

    // Current support API
    virtual bool supportsCurrent() const { return false; }
    virtual void setCurrent(const float current) { (void)current; }

    // motors may implement internal velocity/position commands
    virtual bool supportsInternalVelocity() const { return false; }
    virtual void setInternalVelocity(const float rpm) { (void)rpm; }

    virtual bool supportsInternalPosition() const { return false; }
    virtual void setInternalPosition(const float pos) { (void)pos; }

    // controller acquisition: ensure only one controller controls this motor
    bool tryAcquireController(controllers::IController* ctrl)
    {
        if (controller_ == nullptr)
        {
            controller_ = ctrl;
            return true;
        }
        return controller_ == ctrl; // re-acquire allowed
    }

    void releaseController(controllers::IController* ctrl)
    {
        if (controller_ == ctrl)
            controller_ = nullptr;
    }

    controllers::IController* currentController() const { return controller_; }

private:
    controllers::IController* controller_;
};
} // namespace motors

namespace controllers
{

class IController
{
public:
    virtual ~IController() = default;

    virtual void update() = 0;

    // enable attempts to acquire the motor; returns true when enabled
    virtual bool enable()
    {
        if (!enabled_)
        {
            if (motor_)
            {
                if (motor_->tryAcquireController(this))
                    enabled_ = true;
            }
            else
            {
                enabled_ = false;
            }
        }
        return enabled_;
    }

    // disable releases the motor if held
    virtual void disable()
    {
        if (enabled_)
        {
            if (motor_)
                motor_->releaseController(this);
            enabled_ = false;
        }
    }
    bool enabled() const { return enabled_; }

    // expose underlying motor for helpers that need direct access
    motors::IMotor* getMotor() const { return motor_; }

protected:
    IController(motors::IMotor* motor, const ControlMode ctrl_mode) : motor_(motor), enabled_(false)
    { // decide control mode: cfg_.ctrl_mode == Default -> motor default; otherwise use
      // cfg_.ctrl_mode
        if (motor_ == nullptr)
        {
            ctrl_mode_ = ControlMode::ExternalPID;
            return;
        }
        if (ctrl_mode == ControlMode::Default)
        {
            ctrl_mode_ = motor_->defaultControlMode();
        }
        else
        {
            switch (ctrl_mode)
            {
            case ControlMode::ExternalPID:
                assert(motor_->supportsCurrent());
                ctrl_mode_ = ControlMode::ExternalPID;
                break;
            case ControlMode::InternalVelocity:
                // 我们充分相信用户能决定好控制模式，如果用户决定的不对，就应该报错，而不是兼容
                assert(motor_->supportsInternalVelocity());
                ctrl_mode_ = ControlMode::InternalVelocity;
                break;
            case ControlMode::InternalVelPos:
                assert(motor_->supportsInternalPosition());
                ctrl_mode_ = ControlMode::InternalVelPos;
                break;
            default:;
            }
        }
    }
    motors::IMotor* motor_;
    ControlMode     ctrl_mode_;

private:
    bool enabled_;
};

} // namespace controllers

#endif // I_MOTOR_HPP
