#include "amr.h"
#include <cmath>
#include <iostream>

Amr::Amr(int id, std::unique_ptr<Vcu> vcu) : id_(id), vcu_(std::move(vcu)), cur_idx_(0) 
{ 

}

void Amr::setOrder(const std::vector<NodeInfo>& nodes, const std::vector<EdgeInfo>& edges)
{
    nodes_ = nodes;
    edges_ = edges;
    cur_idx_ = 0;
    if (!nodes_.empty() && vcu_)
    {
        // theta 값을 전달
        vcu_->setTargetPosition(nodes_[0].x, nodes_[0].y, nodes_[0].theta);
    }
}

std::string Amr::getState() const 
{
    // if (nodes_.empty()) return "Idle";
    // return "AMR" + std::to_string(id_) + " at node: " + nodes_[cur_idx_].id;
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
    if (nodes_.empty())
        return;

    vcu_->update(dt);

    double cur_x, cur_y, cur_theta;
    vcu_->getEstimatedPose(cur_x, cur_y, cur_theta);

    const NodeInfo& target_node = nodes_[cur_idx_];

    double dx = target_node.x - cur_x;
    double dy = target_node.y - cur_y;
    double dist = std::hypot(dx, dy);

    constexpr double reach_threshold = 0.05; // 5cm 이내 도달 시 다음 노드

    if (dist < reach_threshold)
    {
        if (cur_idx_ + 1 < nodes_.size())
        {
            ++cur_idx_;
            const NodeInfo& next_node = nodes_[cur_idx_];
            std::cout << "[AMR] Reached node " << target_node.nodeId << ", moving to next node " << next_node.nodeId << std::endl;
            vcu_->setTargetPosition(next_node.x, next_node.y, next_node.theta);
        }
        else
        {
            std::cout << "[AMR] Final node reached. Order complete." << std::endl;
            nodes_.clear();
            edges_.clear();
            cur_idx_ = 0;
        }
    }
}
