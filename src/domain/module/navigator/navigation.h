#pragma once
#include "inavigation.h"
class Navigation : public INavigation 
{
public:
    Navigation();
    void setTarget(double x, double y, double theta) override;
    void update(double current_x, double current_y, double current_theta, double& out_linear, double& out_angular) override;
private:
    double target_x_, target_y_;
    double target_theta_;
};