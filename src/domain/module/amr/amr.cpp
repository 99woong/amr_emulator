#include "amr.h"
#include <cmath>
#include <iostream>
#include <iomanip>  
#include <algorithm>  // std::find_if

Amr::Amr(int id, std::unique_ptr<Vcu> vcu) : id_(id), vcu_(std::move(vcu)), cur_idx_(0) 
{ 

}

void Amr::setOrder(const std::vector<NodeInfo>& nodes, const std::vector<EdgeInfo>& edges)
{
    nodes_ = nodes;
    edges_ = edges;
    cur_edge_idx_ = 0;
    is_angle_adjusting_ = false;

    if (!edges_.empty() && vcu_)
    {
        const std::string& target_node_id = edges_[cur_edge_idx_].endNodeId;
        auto it = std::find_if(nodes_.begin(), nodes_.end(),
             [&](const NodeInfo& n){ return n.nodeId == target_node_id; });
        if (it != nodes_.end())
        {
            vcu_->setTargetPosition(it->x, it->y, it->theta);
        }
    }
}

// void Amr::setOrder(const std::vector<NodeInfo>& nodes, const std::vector<EdgeInfo>& edges)
// {
//     nodes_ = nodes;
//     edges_ = edges;
//     cur_idx_ = 0;
//     if (!nodes_.empty() && vcu_)
//     {
//         // theta 값을 전달
//         std::cout << "[Amr::setOrder] " << nodes_[0].x << " " << nodes_[0].y << " " << nodes_[0].theta << std::endl;
//         vcu_->setTargetPosition(nodes_[0].x, nodes_[0].y, nodes_[0].theta);
//     }
// }

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

void Amr::step(double dt)
{
    constexpr double reach_threshold = 0.01;
    constexpr double angle_threshold = 0.01;
    constexpr double PI = 3.14159265358979323846;

    if (edges_.empty() || cur_edge_idx_ >= edges_.size() || !vcu_)
        return;

    vcu_->update(dt);

    double cur_x, cur_y, cur_theta;
    vcu_->getEstimatedPose(cur_x, cur_y, cur_theta);

    const auto& cur_edge = edges_[cur_edge_idx_];
    auto it = std::find_if(nodes_.begin(), nodes_.end(),
        [&](const NodeInfo& n){ return n.nodeId == cur_edge.endNodeId; });
    if (it == nodes_.end())
        return;

    const NodeInfo& target_node = *it;

    double dx = target_node.x - cur_x;
    double dy = target_node.y - cur_y;
    double dtheta = target_node.theta - cur_theta;
    while (dtheta > PI) dtheta -= 2.0 * PI;
    while (dtheta < -PI) dtheta += 2.0 * PI;

    double dist = std::hypot(dx, dy);

    // maxSpeed를 MotorController에 설정
    vcu_->getMotor().setMaxSpeed(cur_edge.maxSpeed);
    
    std::cout << "dist : " << dist << " dtheta : " << dtheta << std::endl;
    if (!is_angle_adjusting_)
    {
        if (dist < reach_threshold)
        {
            is_angle_adjusting_ = true;
            // set the target angle
            vcu_->setTargetPosition(cur_x, cur_y, target_node.theta);
        }
    }
    else
    {
        if (std::fabs(dtheta) < angle_threshold)
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

            const auto& next_edge = edges_[cur_edge_idx_];

            auto next_it = std::find_if(nodes_.begin(), nodes_.end(), [&](const NodeInfo& n)
                { 
                    return n.nodeId == next_edge.endNodeId; 
                });

            if (next_it != nodes_.end())
            {
                // set the next position
                vcu_->setTargetPosition(next_it->x, next_it->y, next_it->theta);
            }
        }
    }
}