#pragma once
class INavigation {
public:
    virtual ~INavigation() = default;
    virtual void setTarget(double x, double y) = 0;
    virtual void update(double current_x, double current_y, double& out_linear, double& out_angular) = 0;
};