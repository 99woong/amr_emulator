#include "amr.h"
#include <cmath>
#include <iostream>
#include <iomanip>  
#include <algorithm>  // std::find_if

Amr::Amr(int id, std::unique_ptr<Vcu> vcu) : id_(id), vcu_(std::move(vcu)), cur_idx_(0) 
{ 

}

// void Amr::setOrder(const std::vector<NodeInfo>& nodes, const std::vector<EdgeInfo>& edges)
// {
//     nodes_ = nodes;
//     edges_ = edges;
//     cur_edge_idx_ = 0;
//     is_angle_adjusting_ = false;

//     if (!edges_.empty() && vcu_)
//     {
//         const std::string& target_node_id = edges_[cur_edge_idx_].endNodeId;
//         auto it = std::find_if(nodes_.begin(), nodes_.end(),
//              [&](const NodeInfo& n){ return n.nodeId == target_node_id; });
//         if (it != nodes_.end())
//         {
//             vcu_->setTargetPosition(it->x, it->y, it->theta);
//         }
//     }
// }

Line Amr::getLineFromPoints(const NodeInfo& p1, const NodeInfo& p2) 
{
    double A = p2.y - p1.y;
    double B = p1.x - p2.x;
    double C = p2.x * p1.y - p1.x * p2.y;
    
    return {A, B, C};
}

// // 원호 접점 계산 함수 (직선과 원 중심, 반경으로)
// NodeInfo Amr::calculateTangentPoint(const Line& line, const NodeInfo& center, double radius, bool firstPoint) 
// {
//     double A = line.A, B = line.B, C = line.C;
//     double x0 = center.x;
//     double y0 = center.y;
//     double base = A*A + B*B;
//     double D = radius*radius*base - C*C;
//     NodeInfo pt;

//     if (D < 0) 
//     {
//         // 접선 없음. fallback 처리 필요
//         pt.x = x0; pt.y = y0;
//     } 
//     else 
//     {
//         double sqrtD = std::sqrt(D);
//         double sign = firstPoint ? 1.0 : -1.0;
        
//         pt.x = (A*(A*x0 + B*y0) - B*(C + sign*sqrtD)) / base;
//         pt.y = (B*(A*x0 + B*y0) + A*(C + sign*sqrtD)) / base;
//     }
    
//     pt.theta = 0.0; // 필요시 계산
//     return pt;
// }

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

void Amr::setOrder(const std::vector<NodeInfo>& nodes, const std::vector<EdgeInfo>& edges, double wheel_base)
{
    std::vector<NodeInfo> new_nodes;
    std::vector<EdgeInfo> new_edges;

    // 첫 번째 에지의 startNodeId로 초기 위치 설정
    const NodeInfo* start_node = findNodeById(nodes, edges.front().startNodeId);
    if (start_node)
    {
        new_nodes.push_back(*start_node);
    }
    else
    {
        new_nodes.push_back(nodes.front());
    }

    for (size_t i = 0; i < edges.size(); ++i)
    {
        const EdgeInfo& prev_edge = edges[i-1];
        const EdgeInfo& curr_edge = edges[i];

            // 원호 없는 에지, 그대로 추가
        new_edges.push_back(edges[i]);
        const NodeInfo* via_node = findNodeById(nodes, edges[i].endNodeId);
        if (via_node)
        {
            new_nodes.push_back(*via_node);
        }
    }

    // while(1);


    nodes_ = new_nodes;
    edges_ = new_edges;

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
    constexpr double angle_threshold = 0.01;
    constexpr double PI = 3.14159265358979323846;

    if (edges_.empty() || cur_edge_idx_ >= edges_.size() || !vcu_)
        return;

    // vcu_->update(dt);
    vcu_->update(dt, other_robot_positions);

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
    
    // std::cout << "dist : " << dist << " dtheta : " << dtheta << std::endl;
    
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