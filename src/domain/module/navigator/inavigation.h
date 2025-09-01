#pragma once
#include <vector>
class INavigation 
{
public:
    virtual ~INavigation() = default;
    virtual void setTarget(double x, double y, double theta) = 0;
    // virtual void update(double current_x, double current_y, double current_theta,double& out_linear, double& out_angular) = 0;
    virtual void update(double current_x, double current_y, double current_theta,
                            double& out_linear, double& out_angular,
                            const std::vector<std::pair<double, double>>& other_robot_positions) = 0;
    
};