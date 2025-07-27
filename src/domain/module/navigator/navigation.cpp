#include "navigation.h"
#include <cmath>
#include <algorithm>

constexpr double PI = 3.14159265358979323846;

Navigation::Navigation() : target_x_(0), target_y_(0) 
{

}

void Navigation::setTarget(double x, double y) 
{ 
    target_x_ = x; target_y_ = y; 
}

void Navigation::update(double current_x, double current_y, double current_theta, double& out_linear, double& out_angular) 
{
    // 목표 지점까지의 벡터 차이
    double dx = target_x_ - current_x;
    double dy = target_y_ - current_y;

    // 거리
    double distance = std::sqrt(dx * dx + dy * dy);

    // 목표 방향 (로봇 방향과 무관하게 목표 좌표 각도)
    double target_angle = std::atan2(dy, dx);

    // 방향 오차: 목표 방향과 현재 자세 각도 차이 ([-pi, pi] 범위로 보정)
    double angle_diff = target_angle - current_theta;
    while (angle_diff > PI) angle_diff -= 2.0 * PI;
    while (angle_diff < -PI) angle_diff += 2.0 * PI;

    // 출력값 설정: 거리 0~1로 클램프, 각속도도 -1~1로 제한 (필요시 파라미터 조정 가능)
    out_linear = std::clamp(distance, 0.0, 1.0);
    out_angular = std::clamp(angle_diff, -1.0, 1.0);
}