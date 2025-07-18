#include "navigation.h"
#include <cmath>
#include <algorithm>

Navigation::Navigation() : target_x_(0), target_y_(0) {}
void Navigation::setTarget(double x, double y) 
{ 
    target_x_ = x; target_y_ = y; 
}

void Navigation::update(double current_x, double current_y, double& out_linear, double& out_angular) 
{
    double dx = target_x_ - current_x, dy = target_y_ - current_y;
    double distance = std::sqrt(dx*dx + dy*dy);
    double angle = std::atan2(dy, dx);
    out_linear = std::clamp(distance, 0.0, 1.0);
    out_angular = std::clamp(angle, -1.0, 1.0);
}