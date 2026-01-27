#include "navigation.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>  

constexpr double PI = 3.14159265358979323846;

Navigation::Navigation() : target_x_(50), target_y_(50) 
{

}

void Navigation::setTarget(double x, double y)
{
    target_x_ = x;
    target_y_ = y;
    use_arc_ = false;
}

void Navigation::setArcTarget(double x, double y, double center_x, double center_y, double radius, double start_angle, double end_angle, bool clockwise)
{
    target_x_ = x;
    target_y_ = y;
    arc_center_x_ = center_x;
    arc_center_y_ = center_y;
    arc_radius_ = radius;
    arc_start_angle_ = start_angle;
    arc_end_angle_ = end_angle;
    arc_clockwise_ = clockwise;
    use_arc_ = true;
}

// void Navigation::update(double current_x, double current_y, double current_theta,
//                         double& out_linear, double& out_angular)
void Navigation::update(double current_x, double current_y, double current_theta,
                        double& out_linear, double& out_angular,
                        const std::vector<std::pair<double, double>>& other_robot_positions)
{
    constexpr double position_reach_threshold = 0.01;   // ìœ„ì¹˜ ë„ë‹¬ ìž„ê³„ ê±°ë¦¬ (20cm)
    constexpr double angle_reach_threshold = 0.01;     // íšŒì „ ë„ë‹¬ ìž„ê³„ ê°ë„ (ì•½ 3ë„)

    constexpr double collision_radius = 5.0;            // ì¶©ëŒ ê°ì§€ ë°˜ê²½ (1m)
    constexpr double slow_down_radius = 15.0;            // ê°ì† ì‹œìž' ë°˜ê²½ (2m)
    constexpr double max_linear_speed = 10.0;
    constexpr double max_angular_speed = 1.0;

    constexpr double fov_angle = M_PI / 3.0;            // ì „ë°© ì‹œì•¼ê° (ì˜ˆ: 60ë„ = PI/3)

    double min_dist_in_fov = std::numeric_limits<double>::max();

    // ì „ë°© ì‹œì•¼ ë‚´ ìµœì†Œ ê±°ë¦¬ ê³„ì‚°
    for (const auto& pos : other_robot_positions)
    {
        double dx = pos.first - current_x;
        double dy = pos.second - current_y;
        double dist = std::sqrt(dx*dx + dy*dy);

        // ë¡œë´‡ ì „ë°© ë°©í–¥ ë²¡í„°: (cos(current_theta), sin(current_theta))
        // ëŒ€ìƒ ë°©í–¥ ë²¡í„°: (dx, dy)
        // ë‚´ì  ê³„ì‚°í•˜ì—¬ ê°ë„ êµ¬í•˜ê¸°
        double direction_dot = (dx * std::cos(current_theta) + dy * std::sin(current_theta)) / dist;
        double angle_to_obj = std::acos(std::clamp(direction_dot, -1.0, 1.0)); // ëŒ€ìƒê³¼ ì „ë°© ì‚¬ì´ ê°ë„

        if (angle_to_obj <= fov_angle / 2.0)
        {
            if (dist < min_dist_in_fov)
            {
                min_dist_in_fov = dist;
            }
        }
    }

    //ì¶©ëŒ ë° ê°ì† ì˜ì—­ì— ë"°ë¥¸ ì†ë„ ë¹„ìœ¨ ê³„ì‚°
    double speed_scale = 1.0;
    // if (min_dist_to_robot < collision_radius)
    if (min_dist_in_fov < collision_radius)
    {
        speed_scale = 0.0;  // ì¶©ëŒ ì˜ì—­ ë‚´ ì •ì§€
        // std::cout << "[Navigation] Collision imminent! Robot stopped." << std::endl;
    }
    else if (min_dist_in_fov < slow_down_radius)
    {
        speed_scale = ((min_dist_in_fov - collision_radius) / (slow_down_radius - collision_radius)) * 0.1;
        // std::cout << "[Navigation] Slowing down due to nearby robot. Speed scale: " << speed_scale << std::endl;
    }
    else
    {
        speed_scale = 1.0;  // ì •ìƒ ì£¼í–‰
    }    

    if(!use_arc_)
    {
        double dx = target_x_ - current_x;
        double dy = target_y_ - current_y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        double target_angle = std::atan2(dy, dx);
        double angle_diff = target_angle - current_theta;
        while (angle_diff > M_PI)  angle_diff -= 2.0 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

        out_linear = std::clamp(distance, 0.0, 10.0) * speed_scale;
        out_angular = std::clamp(angle_diff, -1.0, 1.0) * speed_scale;

        // std::cout << target_x_ << " " << target_y_ << " " << current_x << " " << current_y << " " << target_angle << " " << current_theta << std::endl;

    }
    else
    {
        // std::cout << "go arc : " << std::endl;
        updateArc(current_x, current_y, current_theta, out_linear, out_angular);
    }

    // std::cout << "navigation : " << target_x_ << " " << target_y_ << " " << current_x << " " << current_y << " " << current_theta << " " << out_linear << " " << out_angular << std::endl;
}

void Navigation::computeTargetControl(double current_x, double current_y, double current_theta,
                                      double speed_scale, double& out_linear, double& out_angular)
{
    double dx = target_x_ - current_x;
    double dy = target_y_ - current_y;
    double distance = std::sqrt(dx * dx + dy * dy);

    double target_angle = std::atan2(dy, dx);
    double angle_diff = target_angle - current_theta;
    while (angle_diff > M_PI)  angle_diff -= 2.0 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

    out_linear = std::clamp(distance, 0.0, 10.0) * speed_scale;
    out_angular = std::clamp(angle_diff, -1.0, 1.0) * speed_scale;
}


void Navigation::updateArc(double current_x, double current_y, double current_theta, double& out_linear, double& out_angular)
{
    const double arc_follow_speed = 2.0;  // ì›í˜¸ ì£¼í–‰ ì„ ì†ë„ ì˜ˆì‹œ

    // ========================================
    // ✅ 추가: 목표 위치까지 거리 확인 (1차 종료 조건)
    // ========================================
    double dx_to_target = target_x_ - current_x;
    double dy_to_target = target_y_ - current_y;
    double dist_to_target = std::sqrt(dx_to_target*dx_to_target + dy_to_target*dy_to_target);
    
    const double position_threshold = 2.0;  // 2m 이내
    if (dist_to_target < position_threshold)
    {
        std::cout << "[Navigation] Arc target position reached! Distance: " 
                  << dist_to_target << "m" << std::endl;
        
        use_arc_ = false;
        out_linear = 0.0;
        out_angular = 0.0;
        return;
    }
    // ========================================

    // ì›í˜¸ ì¤'ì‹¬ê³¼ í˜„ìž¬ ìœ„ì¹˜ ê±°ë¦¬
    double dx = current_x - arc_center_x_;
    double dy = current_y - arc_center_y_;
    double distance_to_center = std::sqrt(dx*dx + dy*dy);

    // ì›í˜¸ ë°˜ê²½ê³¼ í˜„ìž¬ ê±°ë¦¬ ì˜¤ì°¨
    double radius_error = arc_radius_ - distance_to_center;

    // í˜„ìž¬ ìœ„ì¹˜ ê°ë„ w.r.t ì›í˜¸ ì¤'ì‹¬
    double pos_angle = std::atan2(dy, dx);

    // ëª©í'œ ê°ë„ ì§„í–‰ ê³„ì‚° (ì‹œê³„ë°©í–¥ or ë°˜ì‹œê³„ë°©í–¥)
    double angle_diff = arc_end_angle_ - pos_angle;
    // std::cout << "angle : " << arc_end_angle_ << " " << pos_angle << std::endl;

    if (arc_clockwise_) 
    {
        if (angle_diff > 0)
            angle_diff -= 2.0 * M_PI;
    } 
    else 
    {
        if (angle_diff < 0)
            angle_diff += 2.0 * M_PI;
    }

    // ========================================
    // ✅ 수정: 각도 기반 종료 조건 (2차 종료 조건)
    // ========================================
    const double angle_threshold = 0.15;  // 8.6도 (더 여유있게)
    if (std::abs(angle_diff) < angle_threshold) 
    {
        std::cout << "[Navigation] Arc angle completed! angle_diff: " 
                  << angle_diff << " rad (" << (angle_diff * 57.3) << "°)" << std::endl;
        
        use_arc_ = false;
        out_linear = 0.0;
        out_angular = 0.0;
        return;
    }
    // ========================================

    // ì¡°í–¥ì€ ì›í˜¸ì˜ ì¤'ì‹¬ ë°©í–¥ìœ¼ë¡œ íšŒì „: ì›í˜¸ ë°˜ê²½ê³¼ ê°ì†ë„ ê´€ê³„
    // ê°ì†ë„ w = v / r
    double linear_speed = arc_follow_speed;
    double angular_speed = linear_speed / arc_radius_;

    if (arc_clockwise_)
        angular_speed = -angular_speed;

    // ========================================
    // ✅ 활성화: 반지름 오차 보정 (핵심!)
    // ========================================
    applyRadiusErrorCorrection(radius_error, linear_speed, angular_speed);
    // ========================================

    out_linear = linear_speed;
    out_angular = angular_speed;

    // ========================================
    // ✅ 디버그 로그 활성화
    // ========================================
    std::cout << "[Navigation] Arc: pos=(" << current_x << "," << current_y << ")"
              << " center=(" << arc_center_x_ << "," << arc_center_y_ << ")"
              << " dist_center=" << distance_to_center 
              << " radius_err=" << radius_error
              << " angle_diff=" << angle_diff 
              << " dist_target=" << dist_to_target << std::endl;
    // ========================================
}

// void Navigation::Idle((double current_x, double current_y, double current_theta)
// {
//     target_x_ = current_x;
//     target_y_ = current_y;
//     target_t
// }


void Navigation::applyRadiusErrorCorrection(double radius_error, double& linear_speed, double& angular_speed)
{
    const double max_radius_error = 1.5;
    const double max_speed_reduction = 0.5;
    const double max_angular_increase = 0.5;

    double error_ratio = std::min(std::abs(radius_error) / max_radius_error, 1.0);

    // ì„ ì†ë„ ë³´ì •
    linear_speed *= (1.0 - max_speed_reduction * error_ratio);

    // ê°ì†ë„ ë³´ì •
    // double correction_factor = 1.0 + (max_angular_increase - 1.0) * error_ratio;
    double correction_factor = 1.0 + (max_angular_increase - 1.0) * error_ratio;
    angular_speed *= correction_factor;
}
