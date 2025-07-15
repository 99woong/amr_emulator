#pragma once
#include "INavigation.h"
class Navigation : public INavigation 
{
public:
    Navigation();
    void setTarget(double x, double y) override;
    void update(double current_x, double current_y, double& out_linear, double& out_angular) override;
private:
    double target_x_, target_y_;
};