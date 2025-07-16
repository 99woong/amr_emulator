#pragma once
#include <memory> // For std::shared_ptr
#include "accelerationModel.h" // AccelerationModel 추상 클래스 포함

class IMotorController 
{
public:
    virtual ~IMotorController() = default;
    virtual void setAccelerationModel(std::shared_ptr<AccelerationModel> model) = 0;
    virtual void setVelocity(double linear, double angular) = 0;
    virtual void update(double dt) = 0;
    virtual void getRPM(double& left_rpm, double& right_rpm) const = 0;
};