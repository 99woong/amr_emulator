#pragma once
#include "ivcu.h"
#include "imotorController.h"
#include "inavigation.h"
#include <memory>
class Vcu : public IVcu 
{
public:
    Vcu(std::unique_ptr<IMotorController> motor, std::unique_ptr<INavigation> nav);
    void setTargetPosition(double x, double y) override;
    void update() override;
    IMotorController& getMotor();
    INavigation& getNavigation();
private:
    std::unique_ptr<IMotorController> motor_;
    std::unique_ptr<INavigation> navigation_;
    double target_x_, target_y_;
};