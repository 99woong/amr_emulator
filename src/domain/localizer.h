#pragma once
#include "ilocalizer.h"

class Localizer : public ILocalizer 
{
public:
    Localizer(double wheel_base, double wheel_radius);

    void setInitialPose(double x, double y, double theta) override;
    void update(double left_rpm, double right_rpm, double dt) override;
    void getPose(double& x, double& y, double& theta) const override;

private:
    double x_, y_, theta_;
    double wheel_base_, wheel_radius_;
};