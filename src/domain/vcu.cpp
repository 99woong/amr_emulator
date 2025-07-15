#include "Vcu.h"
Vcu::Vcu(std::unique_ptr<IMotorController> motor, std::unique_ptr<INavigation> nav)
    : motor_(std::move(motor)), navigation_(std::move(nav)), target_x_(0), target_y_(0) 
{

}

void Vcu::setTargetPosition(double x, double y) 
{
    target_x_ = x; 
    target_y_ = y;
    navigation_->setTarget(x, y);
}

void Vcu::update() 
{
    double lRpm, rRpm, theta;
    motor_->getRPM(lRpm, rRpm);
    double linear, angular;
    // navigation_->update(x, y, linear, angular);
    motor_->setVelocity(linear, angular);
    motor_->update(0.05);
}

IMotorController& Vcu::getMotor() 
{ 
    return *motor_; 
}

INavigation& Vcu::getNavigation() 
{ 
    return *navigation_; 
}