#include "navigation.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>  

constexpr double PI = 3.14159265358979323846;

Navigation::Navigation() : target_x_(0), target_y_(0) 
{

}

void Navigation::setTarget(double x, double y, double theta)
{
    target_x_ = x;
    target_y_ = y;
    target_theta_ = theta;
}

void Navigation::update(double current_x, double current_y, double current_theta,
                        double& out_linear, double& out_angular)
{
    constexpr double position_reach_threshold = 0.05;   // 위치 도달 임계 거리 (20cm)
    constexpr double angle_reach_threshold = 0.05;     // 회전 도달 임계 각도 (약 3도)

    double dx = target_x_ - current_x;
    double dy = target_y_ - current_y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    std::cout << std::fixed << std::setprecision(2);
    std::cout<< "[NAV] tx : " << target_x_ << " ty : " <<  target_y_ << " cx : " << current_x << " cy : " <<  current_y 
            << " ta : " << target_theta_ << " ca : " <<  current_theta << std::endl;
    // 위치 도달 시 각도 조정 모드로 전환
    if (distance < position_reach_threshold)
    {
        // 목표 방향과 현재 방향 차이 계산 및 보정
        double angle_diff = target_theta_ - current_theta;
        while (angle_diff > M_PI)  angle_diff -= 2.0 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

        // 위치에 거의 도달하면 선속도 제어는 0으로 하고, 각속도만 각도 차이에 맞춰 조절
        out_linear = 0.0;

        // 각도 차이가 작으면 멈추고, 크면 회전하도록 각속도 설정
        if (std::abs(angle_diff) < angle_reach_threshold)
        {
            out_angular = 0.0;  // 목표 각도 도달, 정지
        }
        else
        {
            // 각속도 클램프 (-1 ~ 1), 필요시 제어 상수 곱해 감도 조절
            double angular_speed = std::clamp(angle_diff, -1.0, 1.0);
            out_angular = angular_speed;
        }
    }
    else
    {
        // 위치 도달 전에는 기존 위치 추종 모드

        double target_angle = std::atan2(dy, dx);
        double angle_diff = target_angle - current_theta;
        while (angle_diff > M_PI)  angle_diff -= 2.0 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

        out_linear = std::clamp(distance, 0.0, 1.0);
        out_angular = std::clamp(angle_diff, -1.0, 1.0);
    }

    // std::cout<< std::endl;

}