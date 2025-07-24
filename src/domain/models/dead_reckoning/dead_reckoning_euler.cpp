#include "dead_reckoning_euler.h"
#include <cmath>

constexpr double PI = 3.14159265358979323846;

deadReckoningEuler::deadReckoningEuler(double wheel_base, double wheel_radius)
    : x_(0.0), y_(0.0), theta_(0.0), wheel_base_(wheel_base), wheel_radius_(wheel_radius) 
    {

    }

void deadReckoningEuler::setInitialPose(double x, double y, double theta) 
{
    x_ = x; y_ = y; theta_ = theta;
}

void deadReckoningEuler::update(double left_rpm, double right_rpm, double dt) 
{
    double v_l = (left_rpm / 60.0) * 2.0 * PI * wheel_radius_;
    double v_r = (right_rpm / 60.0) * 2.0 * PI * wheel_radius_;

    double v = (v_r + v_l) / 2.0;
    double w = (v_r - v_l) / wheel_base_;

    x_ += v * std::cos(theta_) * dt;
    y_ += v * std::sin(theta_) * dt;
    theta_ += w * dt;
}

void deadReckoningEuler::getPose(double& x, double& y, double& theta) const 
{
    x = x_; y = y_; theta = theta_;
}