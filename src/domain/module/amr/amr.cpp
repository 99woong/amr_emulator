#include "amr.h"
#include <cmath>
#include <iostream>
#include <iomanip>  

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
        std::cout << "[Amr::setOrder] " << nodes_[0].x << " " << nodes_[0].y << " " << nodes_[0].theta << std::endl;
        vcu_->setTargetPosition(nodes_[0].x, nodes_[0].y, nodes_[0].theta);
    }
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


void Amr::step(double dt)
{
    constexpr double reach_threshold = 0.01;
    constexpr double angle_threshold = 0.01;
    constexpr double PI = 3.14159265358979323846;

    if (nodes_.empty() || cur_idx_ >= nodes_.size()) {
        // std::cerr << "[Amr::step] No nodes or invalid index.\n";
        return;
    }

    if (!vcu_) {
        std::cerr << "[Amr::step] vcu_ is nullptr.\n";
        return;
    }

    vcu_->update(dt);
    double cur_x, cur_y, cur_theta;
    vcu_->getEstimatedPose(cur_x, cur_y, cur_theta);

    const NodeInfo& target_node = nodes_[cur_idx_];

    double dx = target_node.x - cur_x;
    double dy = target_node.y - cur_y;
    double dtheta = target_node.theta - cur_theta;
    // -PI ~ +PI 보정
    while (dtheta > PI) dtheta -= 2.0 * PI;
    while (dtheta < -PI) dtheta += 2.0 * PI;

    double dist = std::hypot(dx, dy);

    if (!is_angle_adjusting_) 
    {
        // 아직 위치 도달 전이면 위치 판단
        if (dist < reach_threshold) 
        {
            // 위치는 도달, 각도 조정 시작
            is_angle_adjusting_ = true;
            vcu_->setTargetPosition(cur_x, cur_y, target_node.theta); // 같은 위치에서 theta 목표로
            std::cout << "[AMR] Position reached, start angle adjusting." << std::endl;
        }
    } 
    else 
    {
        // 위치 도달 후 각도 조정만 수행
        if (std::fabs(dtheta) < angle_threshold) 
        {
            // 각도까지 도달 완료
            if (cur_idx_ + 1 < nodes_.size()) 
            {
                ++cur_idx_;
                is_angle_adjusting_ = false;
                const NodeInfo& next_node = nodes_[cur_idx_];
                vcu_->setTargetPosition(next_node.x, next_node.y, next_node.theta);
                std::cout << "[AMR] Final angle reached, moving to next node "
                        << next_node.nodeId << std::endl;
            } 
            else 
            {
                std::cout << "[AMR] Final node reached. Order complete." << std::endl;
                nodes_.clear();
                edges_.clear();
                cur_idx_ = 0;
                is_angle_adjusting_ = false;
            }
        }
    }
}

// void Amr::step(double dt)
// {
//     if (nodes_.empty())
//         return;

//     std::cout <<"[Amr::step] node size : " << nodes_.size() << "cur_idx_ : " <<  cur_idx_ << std::endl;
//     vcu_->update(dt);

//     double cur_x, cur_y, cur_theta;
//     vcu_->getEstimatedPose(cur_x, cur_y, cur_theta);

//     const NodeInfo& target_node = nodes_[cur_idx_];

//     double dx = target_node.x - cur_x;
//     double dy = target_node.y - cur_y;
//     double dtheta = target_node.theta - cur_theta;
//     double dist = std::hypot(dx, dy);

//     constexpr double reach_threshold = 0.02; // 5cm 이내 도달 시 다음 노드

//     std::cout << std::fixed << std::setprecision(2);
//     std::cout<< "[amr] tx : " << target_node.x << " ty : " <<  target_node.y << " cx : " << cur_x << " cy : " <<  cur_y 
//                 << " ta : " << target_node.theta << " ca : " <<  cur_theta << " dt : " << dtheta << std::endl;

//     std::cout << "dist : " << dist << " dtheta : " <<fabs(dtheta) << std::endl;
//     if (dist < reach_threshold && (fmod(fabs(dtheta),3.14)) < 0.02)
//     {
//         if (cur_idx_ + 1 < nodes_.size())
//         {
//             ++cur_idx_;
//             const NodeInfo& next_node = nodes_[cur_idx_];
//             std::cout << "[AMR] Reached node " << target_node.nodeId << ", moving to next node " << next_node.nodeId << std::endl;
//             vcu_->setTargetPosition(next_node.x, next_node.y, next_node.theta);
//         }
//         else
//         {
//             std::cout << "[AMR] Final node reached. Order complete." << std::endl;
//             nodes_.clear();
//             edges_.clear();
//             cur_idx_ = 0;
//         }
//     }
// }
