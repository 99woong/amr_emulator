#include "navigation.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>  

constexpr double PI = 3.14159265358979323846;

Navigation::Navigation() : target_x_(50), target_y_(50) 
{

}

void Navigation::setTarget(double x, double y, double theta)
{
    target_x_ = x;
    target_y_ = y;
    target_theta_ = theta;
}

// void Navigation::update(double current_x, double current_y, double current_theta,
//                         double& out_linear, double& out_angular)
void Navigation::update(double current_x, double current_y, double current_theta,
                        double& out_linear, double& out_angular,
                        const std::vector<std::pair<double, double>>& other_robot_positions)
{
    constexpr double position_reach_threshold = 0.01;   // 위치 도달 임계 거리 (20cm)
    constexpr double angle_reach_threshold = 0.01;     // 회전 도달 임계 각도 (약 3도)

    constexpr double collision_radius = 5.0;            // 충돌 감지 반경 (1m)
    constexpr double slow_down_radius = 15.0;            // 감속 시작 반경 (2m)
    constexpr double max_linear_speed = 10.0;
    constexpr double max_angular_speed = 1.0;

    // // 충돌 감지: 주변 로봇과의 최소 거리 계산
    // double min_dist_to_robot = std::numeric_limits<double>::max();
    // for (const auto& pos : other_robot_positions)
    // {
    //     double dx = pos.first - current_x;
    //     double dy = pos.second - current_y;
    //     double dist = std::sqrt(dx*dx + dy*dy);
    //     if (dist < min_dist_to_robot)
    //         min_dist_to_robot = dist;
    // }    

    constexpr double fov_angle = M_PI / 3.0;            // 전방 시야각 (예: 60도 = PI/3)

    double min_dist_in_fov = std::numeric_limits<double>::max();

    // 전방 시야 내 최소 거리 계산
    for (const auto& pos : other_robot_positions)
    {
        double dx = pos.first - current_x;
        double dy = pos.second - current_y;
        double dist = std::sqrt(dx*dx + dy*dy);

        // 로봇 전방 방향 벡터: (cos(current_theta), sin(current_theta))
        // 대상 방향 벡터: (dx, dy)
        // 내적 계산하여 각도 구하기
        double direction_dot = (dx * std::cos(current_theta) + dy * std::sin(current_theta)) / dist;
        double angle_to_obj = std::acos(std::clamp(direction_dot, -1.0, 1.0)); // 대상과 전방 사이 각도

        if (angle_to_obj <= fov_angle / 2.0)
        {
            if (dist < min_dist_in_fov)
                min_dist_in_fov = dist;
        }
    }



    //충돌 및 감속 영역에 따른 속도 비율 계산
    double speed_scale = 1.0;
    // if (min_dist_to_robot < collision_radius)
    if (min_dist_in_fov < collision_radius)
    {
        speed_scale = 0.0;  // 충돌 영역 내 정지
        // std::cout << "[Navigation] Collision imminent! Robot stopped." << std::endl;
    }
    else if (min_dist_in_fov < slow_down_radius)
    {
        speed_scale = ((min_dist_in_fov - collision_radius) / (slow_down_radius - collision_radius)) * 0.1;
        // std::cout << "[Navigation] Slowing down due to nearby robot. Speed scale: " << speed_scale << std::endl;
    }
    else
    {
        speed_scale = 1.0;  // 정상 주행
    }    

    double dx = target_x_ - current_x;
    double dy = target_y_ - current_y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
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
            out_angular = angular_speed * speed_scale;
        }
    }
    else
    {
        // 위치 도달 전에는 기존 위치 추종 모드

        double target_angle = std::atan2(dy, dx);
        double angle_diff = target_angle - current_theta;
        while (angle_diff > M_PI)  angle_diff -= 2.0 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

        out_linear = std::clamp(distance, 0.0, 10.0) * speed_scale;
        out_angular = std::clamp(angle_diff, -1.0, 1.0) * speed_scale;
    }

    // std::cout<< std::endl;

}
