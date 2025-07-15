#include "localizer.h"
#include <cmath>

constexpr double PI = 3.14159265358979323846;

Localizer::Localizer(double wheel_base, double wheel_radius)
    : x_(0), y_(0), theta_(0), wheel_base_(wheel_base), wheel_radius_(wheel_radius)
{
    
}

void Localizer::setInitialPose(double x, double y, double theta) 
{
    x_ = x;
    y_ = y;
    theta_ = theta;
}

void Localizer::update(double left_rpm, double right_rpm, double dt) 
{
    // 1. rpm -> m/s 변환
    double v_l = (left_rpm / 60.0) * 2 * PI * wheel_radius_;
    double v_r = (right_rpm / 60.0) * 2 * PI * wheel_radius_;

    // 2. 차동구동 로봇의 선속도, 각속도 계산
    double v = (v_r + v_l) / 2.0;
    double w = (v_r - v_l) / wheel_base_;

    // 3. 위치 적분
    x_ += v * std::cos(theta_) * dt;
    y_ += v * std::sin(theta_) * dt;
    theta_ += w * dt;
}

void Localizer::getPose(double& x, double& y, double& theta) const 
{
    x = x_;
    y = y_;
    theta = theta_;
}