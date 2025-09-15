#include "amr.h"
#include <cmath>
#include <iostream>
#include <iomanip>  
#include <algorithm>  // std::find_if

Amr::Amr(int id, std::unique_ptr<Vcu> vcu, std::unique_ptr<IBatteryModel> battery_model) 
: id_(id), vcu_(std::move(vcu)), battery_model_(std::move(battery_model)), cur_idx_(0) 
{ 
}

void Amr::updateBattery(double dt, bool is_charging)
{
    double linear_vel = 0.0, angular_vel = 0.0;
    if (vcu_)
    {
        linear_vel = vcu_->getMotor().getLinearVelocity();     
        angular_vel = vcu_->getMotor().getAngularVelocity();   
    }
    battery_model_->update(dt, linear_vel, angular_vel, is_charging);
}

double Amr::getBatteryPercent() const
{
    if (battery_model_)
        return battery_model_->getCapacityPercent();
    return 0.0;
}

void Amr::setVcuTargetFromEdge(const EdgeInfo& edge, const std::vector<NodeInfo>& nodes, double wheel_base)
{
    // target_node 찾기
    std::cout << "set edge id : " << edge.edgeId << std::endl;
    const NodeInfo* target_node = nullptr;
    const NodeInfo* start_node = nullptr;
    auto it = std::find_if(nodes.begin(), nodes.end(),
        [&](const NodeInfo& n) { return n.nodeId == edge.endNodeId; });
    if (it != nodes.end())
        target_node = &(*it);

    auto sit = std::find_if(nodes.begin(), nodes.end(),
        [&](const NodeInfo& n) { return n.nodeId == edge.startNodeId; });
    if (sit != nodes.end())
        start_node = &(*sit);


    if(edge.has_turn_center) {
        vcu_->setTargetPosition(
            start_node ? start_node->x : 0.0,
            start_node ? start_node->y : 0.0,
            target_node ? target_node->x : 0.0,
            target_node ? target_node->y : 0.0,
            target_node ? target_node->theta : 0.0,
            edge.turn_center_x,
            edge.turn_center_y,
            true,
            wheel_base
        );
    } else {
        vcu_->setTargetPosition(
            start_node ? start_node->x : 0.0,
            start_node ? start_node->y : 0.0,
            target_node ? target_node->x : 0.0,
            target_node ? target_node->y : 0.0,
            target_node ? target_node->theta : 0.0,
            0.0, 0.0, false, wheel_base
        );
    }
}


void Amr::setOrder(const std::vector<NodeInfo>& nodes, const std::vector<EdgeInfo>& edges, double wheel_base)
{
    nodes_ = nodes;
    edges_ = edges;
    cur_edge_idx_ = 0;
    is_angle_adjusting_ = false;
    wheel_base_ = wheel_base;

    if (!edges_.empty() && vcu_)
    {
        const EdgeInfo& edge = edges_[cur_edge_idx_];

        setVcuTargetFromEdge(edge, nodes_, wheel_base);
    }
}


NodeInfo Amr::calculateTangentPoint(const Line& line, const NodeInfo& center, double radius, 
                                    const NodeInfo& ref, bool preferCloser) 
{
    double A = line.A, B = line.B, C = line.C;
    double cx = center.x, cy = center.y;
    double denom = A*A + B*B;
    double dist = (A*cx + B*cy + C) / sqrt(denom); // 원 중심-직선 거리

    // 중심에서 직선으로 내린 수선의 발
    double x0 = cx - A * (A*cx + B*cy + C) / denom;
    double y0 = cy - B * (A*cx + B*cy + C) / denom;

    double d2 = radius*radius - ((A*cx + B*cy + C)*(A*cx + B*cy + C))/denom; // 판별식

    NodeInfo pt;

    if(d2 < 0) {
        // 접점 없음(fallback)
        pt.x = x0; pt.y = y0;
    } else {
        double mult = std::sqrt(d2 / denom);

        // 직선의 방향벡터(B, -A)
        double dx = B / sqrt(denom);
        double dy = -A / sqrt(denom);

        NodeInfo p1; 
        p1.x = x0 + mult * dx;
        p1.y = y0 + mult * dy;

        NodeInfo p2; 
        p2.x = x0 - mult * dx;
        p2.y = y0 - mult * dy;

        // ref와 가까운 쪽, 아니면 preferCloser
        double dist1 = hypot(p1.x - ref.x, p1.y - ref.y);
        double dist2 = hypot(p2.x - ref.x, p2.y - ref.y);
        pt = (preferCloser == (dist1 < dist2)) ? p1 : p2;
    }
    pt.theta = 0.0;
    return pt;
}

const NodeInfo* Amr::findNodeById(const std::vector<NodeInfo>& nodes, const std::string& id)
{
    auto it = std::find_if(nodes.begin(), nodes.end(), [&](const NodeInfo& n) {
        return n.nodeId == id;
    });
    return (it != nodes.end()) ? &(*it) : nullptr;
}

std::string Amr::getState() const 
{
    if (nodes_.empty()) 
        return "AMR" + std::to_string(id_) + ": Idle";
    return "AMR" + std::to_string(id_) + " at node: " + nodes_[cur_idx_].nodeId;
}

IVcu* Amr::getVcu()
{
    return vcu_.get();
}



// void Amr::step(double dt)
void Amr::step(double dt, const std::vector<std::pair<double, double>>& other_robot_positions)
{
    constexpr double reach_threshold = 0.01;
    constexpr double angle_threshold = 0.1;
    constexpr double PI = 3.14159265358979323846;
    static double reach_distance_radius = 0.1;
    static double angle_area_radius = 0.1;

    if (edges_.empty() || cur_edge_idx_ >= edges_.size() || !vcu_)
        return;

    // vcu_->update(dt);
    vcu_->update(dt, other_robot_positions);

    double cur_x, cur_y, cur_theta;
    vcu_->getEstimatedPose(cur_x, cur_y, cur_theta);

    const EdgeInfo& cur_edge = edges_[cur_edge_idx_];
    const NodeInfo* target_node = nullptr;

    auto it = std::find_if(nodes_.begin(), nodes_.end(),
        [&](const NodeInfo& n) { return n.nodeId == cur_edge.endNodeId; });
    if (it != nodes_.end())
        target_node = &(*it);

    double dx = target_node->x - cur_x;
    double dy = target_node->y - cur_y;
    double dtheta = target_node->theta - cur_theta;
    while (dtheta > PI) dtheta -= 2.0 * PI;
    while (dtheta < -PI) dtheta += 2.0 * PI;

    //현재위치와 목표위치 차이 계산
    double dist = std::hypot(dx, dy);

    // maxSpeed를 MotorController에 설정
    vcu_->getMotor().setMaxSpeed(cur_edge.maxSpeed);
    
    std::cout << "dist : " << dist << " dtheta : " << dtheta << " tx : " << target_node->x << " cx : " << cur_x << " ty : " << target_node->y << " cy : " << cur_y << " tt : " << target_node->theta << " ct : " << cur_theta << std::endl;

    if(cur_edge.has_turn_center)
    {
        reach_distance_radius = 1.0;
        angle_area_radius = 1.0;
    }
    else
    {
        reach_distance_radius = 0.1;
        angle_area_radius = 0.1;
    }
    
    if (dist < reach_distance_radius)
    {
        cur_edge_idx_++;
        is_angle_adjusting_ = false;

        if (cur_edge_idx_ >= edges_.size())
        {
            // 오더 완료
            nodes_.clear();
            edges_.clear();
            cur_edge_idx_ = 0;
            return;
        }
        const EdgeInfo& next_edge = edges_[cur_edge_idx_];

        setVcuTargetFromEdge(next_edge, nodes_, wheel_base_);
    }
}