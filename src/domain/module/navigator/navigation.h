#pragma once
#include "inavigation.h"
class Navigation : public INavigation 
{
public:
    Navigation();
    void setTarget(double x, double y) override;
    void setArcTarget(double x, double y, double center_x, double center_y, double radius, double start_angle, double end_angle, bool clockwise) override;
    // void update(double current_x, double current_y, double current_theta, double& out_linear, double& out_angular) override;
    void update(double current_x, double current_y, double current_theta,
                            double& out_linear, double& out_angular,
                            const std::vector<std::pair<double, double>>& other_robot_positions) override;
private:
    double target_x_, target_y_;
    double target_theta_;

    bool use_arc_ = false; // 원호 주행 플래그
    double arc_center_x_, arc_center_y_, arc_radius_;
    double arc_start_angle_, arc_end_angle_;
    bool arc_clockwise_;

    void updateArc(double current_x, double current_y, double current_theta,
                   double& out_linear, double& out_angular);
    void updateStraight(double current_x, double current_y, double current_theta,
                        double& out_linear, double& out_angular);    
};