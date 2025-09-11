#include "vcu.h"
#include <iostream> 
#include <cmath>

Vcu::Vcu(std::unique_ptr<IMotorController> motor, std::unique_ptr<INavigation> nav, std::unique_ptr<ILocalizer> localizer)
    : motor_(std::move(motor)), navigation_(std::move(nav)), localizer_(std::move(localizer)), target_x_(0), target_y_(0) 
{

}

void Vcu::setTargetPosition(double start_x, double start_y, double target_x, double target_y, double target_theta, double center_x, double center_y, bool hasTurnCenter, double wheel_base)
{
    target_x_ = target_x;
    target_y_ = target_y;
    target_theta_ = target_theta;  // 목표 방향 추가 저장
    // std::cout << " 21 "<<std::endl;

    if(!hasTurnCenter)
    {
        navigation_->setTarget(target_x_, target_y_, target_theta_);
        // std::cout << "[VCU] Target position set to (" << target_x_ << ", " << target_y_ << "), theta=" << target_theta_ << 
        // "cx : " << center_x << "cy : " << center_y << "hasTurnCenter : " << hasTurnCenter <<std::endl;
        // std::cout << " 22 "<<std::endl;
    }
    else
    {
        // std::cout << "[VCU] Target ARC position set to (" << target_x_ << ", " << target_y_ << "), theta=" << target_theta_ << 
        // "cx : " << center_x << "cy : " << center_y << "hasTurnCenter : " << hasTurnCenter <<std::endl;
        double radius = std::hypot(target_x - center_x, target_y - center_y);
        
        // double radius = 27.1;
        double start_angle = std::atan2(start_y - center_y, start_x - center_x);
        double end_angle = std::atan2(target_y - center_y, target_x - center_x);
        bool clockwise = false;
        
        // std::cout << " radius "<< radius << " " << "start_angle : " << start_angle << " " << "end_angle : " << end_angle << std::endl;

        navigation_->setArcTarget(center_x, center_y, radius, start_angle, end_angle, clockwise);
    }
}

void Vcu::setTargetArc(double start_x, double start_y, double center_x,
    double center_y, double end_x, double end_y, double wheeBase)
{
    
    double radius = std::hypot(start_x - center_x, start_y - center_y);
    double start_angle = std::atan2(start_y - center_y, start_x - center_x);
    double end_angle = std::atan2(end_y - center_y, end_x - center_x);
    bool clockwise = false; 

    navigation_->setArcTarget(center_x, center_y, radius, start_angle, end_angle, clockwise);
}

// void Vcu::update(double dt) 
void Vcu::update(double dt, const std::vector<std::pair<double, double>>& other_robot_positions)
{
    double left_rpm, right_rpm;
    motor_->getRPM(left_rpm, right_rpm);

    localizer_->update(left_rpm, right_rpm, dt);
    
    double current_x = 0.0, current_y = 0.0, current_theta = 0.0;
    localizer_->getPose(current_x, current_y, current_theta);

    double linear_vel_cmd = 0.0, angular_vel_cmd = 0.0;
    navigation_->update(current_x, current_y, current_theta, linear_vel_cmd, angular_vel_cmd, other_robot_positions);
    motor_->setVelocity(linear_vel_cmd, angular_vel_cmd);
    motor_->update(dt);
}

void Vcu::setInitialPose(double x, double y, double theta)
{
    if (localizer_)
    {
        localizer_->setInitialPose(x, y, theta);
    }
}

void Vcu::getEstimatedPose(double& x, double& y, double& theta) const
{
    if (localizer_)
    {
        localizer_->getPose(x, y, theta);
    }
    else
    {
        x = y = theta = 0.0;
    }
}

IMotorController& Vcu::getMotor() 
{ 
    return *motor_; 
}

INavigation& Vcu::getNavigation() 
{ 
    return *navigation_; 
}

ILocalizer& Vcu::getLocalizer()
{
    return *localizer_;
}

void Vcu::updateNodes(const std::vector<NodeInfo>& nodes) 
{
    std::cout << "[VCU] updateNodes called with " << nodes.size() << " nodes." << std::endl;

    for (const auto& node : nodes) 
    {
        std::cout << " - NodeId: " << node.nodeId << ", Pos: (" << node.x << ", " << node.y << ", " << node.theta << ")" << std::endl;
    }
}

void Vcu::updateEdges(const std::vector<EdgeInfo>& edges) 
{
    std::cout << "[VCU] updateEdges called with " << edges.size() << " edges." << std::endl;
    
    for (const auto& edge : edges) 
    {
        std::cout << " - EdgeId: " << edge.edgeId 
                  << ", StartNodeId: " << edge.startNodeId 
                  << ", EndNodeId: " << edge.endNodeId << std::endl;
    }
}