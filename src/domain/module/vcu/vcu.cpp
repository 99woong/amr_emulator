#include "vcu.h"
#include <iostream> 

Vcu::Vcu(std::unique_ptr<IMotorController> motor, std::unique_ptr<INavigation> nav, std::unique_ptr<ILocalizer> localizer)
    : motor_(std::move(motor)), navigation_(std::move(nav)), localizer_(std::move(localizer)), target_x_(0), target_y_(0) 
{

}

void Vcu::setTargetPosition(double x, double y, double theta)
{
    target_x_ = x;
    target_y_ = y;
    target_theta_ = theta;  // 목표 방향 추가 저장
    navigation_->setTarget(x, y, theta);
    std::cout << "[VCU] Target position set to (" << x << ", " << y << "), theta=" << theta << std::endl;
}

void Vcu::update(double dt) 
{
    double left_rpm, right_rpm;
    motor_->getRPM(left_rpm, right_rpm);

    localizer_->update(left_rpm, right_rpm, dt);
    
    double current_x = 0.0, current_y = 0.0, current_theta = 0.0;
    localizer_->getPose(current_x, current_y, current_theta);

    double linear_vel_cmd = 0.0, angular_vel_cmd = 0.0;
    navigation_->update(current_x, current_y, current_theta, linear_vel_cmd, angular_vel_cmd);

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
    // 실제 동작에 맞게 내부 변수 업데이트, 밸리데이션 등 추가 구현 가능
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
    // 내부 연결 상태 갱신 등 실제 로직 추가 필요
}