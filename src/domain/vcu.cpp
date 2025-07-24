#include "vcu.h"
#include <iostream> 

Vcu::Vcu(std::unique_ptr<IMotorController> motor, std::unique_ptr<INavigation> nav)
    : motor_(std::move(motor)), navigation_(std::move(nav)), target_x_(0), target_y_(0) 
{

}

void Vcu::setTargetPosition(double x, double y) 
{
    target_x_ = x; 
    target_y_ = y;
    navigation_->setTarget(x, y);
}

void Vcu::update() 
{
    double lRpm, rRpm, theta;
    motor_->getRPM(lRpm, rRpm);
    double linear, angular;
    // navigation_->update(x, y, linear, angular);
    motor_->setVelocity(linear, angular);
    motor_->update(0.05);
}

IMotorController& Vcu::getMotor() 
{ 
    return *motor_; 
}

INavigation& Vcu::getNavigation() 
{ 
    return *navigation_; 
}

void Vcu::updateNodes(const std::vector<NodeInfo>& nodes) 
{
    // 예시 구현: 받은 노드 정보를 내부 상태로 갱신하거나 로그 출력
    std::cout << "[VCU] updateNodes called with " << nodes.size() << " nodes." << std::endl;
    for (const auto& node : nodes) {
        std::cout << " - NodeId: " << node.nodeId << ", Pos: (" << node.x << ", " << node.y << ", " << node.theta << ")" << std::endl;
    }
    // 실제 동작에 맞게 내부 변수 업데이트, 밸리데이션 등 추가 구현 가능
}

void Vcu::updateEdges(const std::vector<EdgeInfo>& edges) 
{
    std::cout << "[VCU] updateEdges called with " << edges.size() << " edges." << std::endl;
    for (const auto& edge : edges) {
        std::cout << " - EdgeId: " << edge.edgeId 
                  << ", StartNodeId: " << edge.startNodeId 
                  << ", EndNodeId: " << edge.endNodeId << std::endl;
    }
    // 내부 연결 상태 갱신 등 실제 로직 추가 필요
}