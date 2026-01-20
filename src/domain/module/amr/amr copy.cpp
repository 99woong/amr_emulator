//amr.cpp
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


void Amr::setVcuTargetFromEdge(const EdgeInfo& edge, const std::vector<NodeInfo>& nodes, const std::vector<NodeInfo>& all_nodes, double wheel_base)
{
    // target_node 찾기
    const NodeInfo* target_node = nullptr;
    const NodeInfo* start_node = nullptr;
    const NodeInfo* center_node = nullptr;

    auto it = std::find_if(nodes.begin(), nodes.end(),
        [&](const NodeInfo& n) { return n.nodeId == edge.endNodeId; });
    if (it != nodes.end())
        target_node = &(*it);

    auto sit = std::find_if(all_nodes.begin(), all_nodes.end(),
        [&](const NodeInfo& n) { return n.nodeId == edge.startNodeId; });        
    if (sit != all_nodes.end())
    {
        start_node = &(*sit);
    }

    if (!edge.centerNodeId.empty())
    {
        std::cout << "[AMR] Looking for centerNodeId: " << edge.centerNodeId << std::endl;
        
        auto cit = std::find_if(all_nodes.begin(), all_nodes.end(),
            [&](const NodeInfo& n) { return n.nodeId == edge.centerNodeId; });
        
        if (cit != all_nodes.end())
        {
            center_node = &(*cit);
            std::cout << "[AMR] Found centerNode: " << center_node->nodeId 
                      << " at (" << center_node->x << ", " << center_node->y << ")" << std::endl;
        }
        else
        {
            std::cerr << "[AMR] WARNING: centerNode '" << edge.centerNodeId 
                      << "' not found in all_nodes!" << std::endl;
        }
    }

    if(edge.has_turn_center && center_node) 
    {
        vcu_->setTargetPosition(
            start_node ? start_node->x : 0.0,
            start_node ? start_node->y : 0.0,
            target_node ? target_node->x : 0.0,
            target_node ? target_node->y : 0.0,
            center_node ? center_node->x : 0.0,
            center_node ? center_node->y : 0.0,
            true,
            wheel_base
        );
    } 
    else 
    {
        vcu_->setTargetPosition(
            start_node ? start_node->x : 0.0,
            start_node ? start_node->y : 0.0,
            target_node ? target_node->x : 0.0,
            target_node ? target_node->y : 0.0,
            0.0, 
            0.0, 
            false, 
            wheel_base
        );
    }
}


void Amr::setOrder(const std::vector<NodeInfo>& nodes, const std::vector<EdgeInfo>& edges, const std::vector<NodeInfo>& all_nodes, double wheel_base)
{
    nodes_ = nodes;
    edges_ = edges;
    all_nodes_ = all_nodes; 

    cur_edge_idx_ = 0;
    is_angle_adjusting_ = false;
    wheel_base_ = wheel_base;

    completed_nodes_.clear();
    completed_edges_.clear();

    if (!edges_.empty() && vcu_)
    {
        std::cout << "[AMR] Starting order with edge: " << edges_[cur_edge_idx_].edgeId << std::endl;
        std::cout << "[AMR] Nodes for driving (endNodes): " << nodes_.size() << std::endl;
        std::cout << "[AMR] All nodes available (including centerNodes): " << all_nodes_.size() << std::endl;
                
        std::cout << "start edge : " << edges_[cur_edge_idx_].edgeId << std::endl;
        const EdgeInfo& edge = edges_[cur_edge_idx_];
        setVcuTargetFromEdge(edge, nodes_, all_nodes_, wheel_base);
    }
}

// 현재 처리 중인 노드들 반환
std::vector<NodeInfo> Amr::getCurrentNodes() const
{
    std::vector<NodeInfo> current;
    
    if (edges_.empty() || cur_edge_idx_ >= edges_.size())
        return current;
    
    // 현재 엣지의 endNode를 현재 목표 노드로 간주
    const EdgeInfo& cur_edge = edges_[cur_edge_idx_];
    
    auto it = std::find_if(nodes_.begin(), nodes_.end(),
        [&](const NodeInfo& n) { return n.nodeId == cur_edge.endNodeId; });
    
    if (it != nodes_.end())
    {
        current.push_back(*it);
    }
    
    return current;
}

// 완료된 노드들 반환
std::vector<NodeInfo> Amr::getCompletedNodes() const
{
    return completed_nodes_;
}


// 현재 처리 중인 엣지들 반환
std::vector<EdgeInfo> Amr::getCurrentEdges() const
{
    std::vector<EdgeInfo> current;
    
    if (cur_edge_idx_ < edges_.size())
    {
        current.push_back(edges_[cur_edge_idx_]);
    }
    
    return current;
}

// 완료된 엣지들 반환
std::vector<EdgeInfo> Amr::getCompletedEdges() const
{
    return completed_edges_;
}


// 마지막 노드 ID 반환
std::string Amr::getLastNodeId() const
{
    if (completed_nodes_.empty())
        return "";
    
    return completed_nodes_.back().nodeId;
}

// 마지막 노드 시퀀스 ID 반환
int Amr::getLastNodeSequenceId() const
{
    if (completed_nodes_.empty())
        return 0;
    
    return completed_nodes_.back().sequenceId;
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

    if(d2 < 0) 
    {
        // 접점 없음(fallback)
        pt.x = x0; pt.y = y0;
    } 
    else 
    {
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

void Amr::step(double dt, const std::vector<std::pair<double, double>>& other_robot_positions)
{
    constexpr double reach_threshold = 0.01;
    constexpr double angle_threshold = 0.1;
    constexpr double PI = 3.14159265358979323846;
    static double reach_distance_radius = 0.1;
    static double angle_area_radius = 0.1;

    double cur_x, cur_y, cur_theta;
    vcu_->getEstimatedPose(cur_x, cur_y, cur_theta);

    if (edges_.empty() || cur_edge_idx_ >= edges_.size() || !vcu_)
    {
        vcu_->Idle(dt);
        return;
    }
    
    vcu_->update(dt, other_robot_positions);

    const EdgeInfo& cur_edge = edges_[cur_edge_idx_];
    const NodeInfo* target_node = nullptr;

    auto it = std::find_if(nodes_.begin(), nodes_.end(),
        [&](const NodeInfo& n) { return n.nodeId == cur_edge.endNodeId; });
    
    if (it != nodes_.end())
    {
        target_node = &(*it);
    }
    else
    {
        std::cerr << "[AMR" << id_ << "] ERROR: Target node '" << cur_edge.endNodeId 
                  << "' not found for edge '" << cur_edge.edgeId << "'" << std::endl;
        vcu_->Idle(dt);
        return;
    }

    if (!target_node)
    {
        std::cerr << "[AMR" << id_ << "] ERROR: target_node is null" << std::endl;
        vcu_->Idle(dt);
        return;
    }

    double dx = target_node->x - cur_x;
    double dy = target_node->y - cur_y;
    double dist = std::hypot(dx, dy);

    vcu_->getMotor().setMaxSpeed(cur_edge.maxSpeed);
    
    if (cur_edge.has_turn_center)
    {
        reach_distance_radius = 0.8;
        angle_area_radius = 0.8;
    }
    else
    {
        reach_distance_radius = 0.1;
        angle_area_radius = 0.1;
    }
    
    if (dist < reach_distance_radius)
    {
        std::cout << "[AMR" << id_ << "] Arrived at node: " << target_node->nodeId 
                  << " (current completed: " << completed_nodes_.size() << ")" << std::endl;

        // 에지를 먼저 추가
        completed_edges_.push_back(cur_edge);
        
        // 중복 체크 강화 - 디버깅 출력 추가
        bool is_duplicate = false;
        
        std::cout << "[AMR" << id_ << "] Checking for duplicates. Current completed_nodes_:" << std::endl;
        for (size_t i = 0; i < completed_nodes_.size(); ++i)
        {
            std::cout << "  [" << i << "] " << completed_nodes_[i].nodeId << std::endl;
            
            if (completed_nodes_[i].nodeId == target_node->nodeId)
            {
                is_duplicate = true;
                // std::cout << "[AMR" << id_ << "] DUPLICATE FOUND: Node '" << target_node->nodeId 
                //           << "' already at index " << i << std::endl;
                break;
            }
        }
        
        if (is_duplicate)
        {
            std::cout << "[AMR" << id_ << "] kipping duplicate node '" << target_node->nodeId 
                      << "' (circular path)" << std::endl;
        }
        else
        {
            completed_nodes_.push_back(*target_node);
            std::cout << "[AMR" << id_ << "] Added node '" << target_node->nodeId 
                      << "' (total: " << completed_nodes_.size() << ")" << std::endl;
        }
        
        needs_immediate_state_publish_ = true;

        cur_edge_idx_++;
        is_angle_adjusting_ = false;

        if (cur_edge_idx_ >= edges_.size())
        {
            std::cout << "[AMR" << id_ << "] All edges completed. Order finished." << std::endl;
            std::cout << "[AMR" << id_ << "] Final stats: " 
                      << completed_nodes_.size() << " nodes, " 
                      << completed_edges_.size() << " edges" << std::endl;
            
            nodes_.clear();
            edges_.clear();
            all_nodes_.clear();
            cur_edge_idx_ = 0;
            
            vcu_->Idle(dt);
            
            return;
        }

        const EdgeInfo& next_edge = edges_[cur_edge_idx_];
        std::cout << "[AMR" << id_ << "] Moving to next edge: " << next_edge.edgeId 
                  << " (idx: " << cur_edge_idx_ << "/" << edges_.size() << ")" << std::endl;
        
        setVcuTargetFromEdge(next_edge, nodes_, all_nodes_, wheel_base_);
    }
}

void Amr::markNodeAsCompleted(const NodeInfo& node)
{
    std::cout << "[AMR" << id_ << "] markNodeAsCompleted called for: '" << node.nodeId 
              << "' (sequenceId: " << node.sequenceId << ")" << std::endl;
    
    // 추가: 중복 체크
    for (const auto& completed_node : completed_nodes_)
    {
        if (completed_node.nodeId == node.nodeId)
        {
            std::cout << "[AMR" << id_ << "] Node '" << node.nodeId 
                      << "' already marked as completed. Skipping." << std::endl;
            return;
        }
    }
    
    completed_nodes_.push_back(node);
    
    std::cout << "[AMR" << id_ << "] Node marked as completed. Total completed nodes: " 
              << completed_nodes_.size() << std::endl;
}


void Amr::cancelOrder()
{
    std::cout << "[AMR" << id_ << "] Order cancellation requested" << std::endl;
    
    // 1. 모든 오더 관련 데이터 초기화
    nodes_.clear();
    edges_.clear();
    cur_edge_idx_ = 0;
    cur_idx_ = 0;
    is_angle_adjusting_ = false;
    
    // 2. 완료 리스트도 초기화 (새로운 오더를 위해)
    completed_nodes_.clear();
    completed_edges_.clear();
    
    // 3. VCU를 Idle 상태로 전환 (즉시 정지)
    if (vcu_)
    {
        double current_x = 0.0, current_y = 0.0, current_theta = 0.0;
        vcu_->getEstimatedPose(current_x, current_y, current_theta);
        
        // 현재 위치를 목표로 설정하여 정지
        vcu_->getNavigation().setTarget(current_x, current_y);
        
        // 모터를 0으로 설정
        vcu_->getMotor().setVelocity(0.0, 0.0);
        
        std::cout << "[AMR" << id_ << "] Motion stopped at position (" 
                  << current_x << ", " << current_y << ")" << std::endl;
    }
    
    std::cout << "[AMR" << id_ << "] Order cancelled - AMR is now IDLE" << std::endl;
}

