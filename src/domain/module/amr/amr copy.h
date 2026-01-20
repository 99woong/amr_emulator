#pragma once
#include "iamr.h"
#include "ivcu.h"
// #include "iprotocol.h"
#include "vcu.h"
#include "battery_model.h"
#include <vector>
#include <string>
#include <memory>

// 직선 일반형 방정식 계산
struct Line {
    double A, B, C;
};

class Amr : public IAmr 
{
public:
    Amr(int id, std::unique_ptr<Vcu> vcu, std::unique_ptr<IBatteryModel> battery_model);
    std::string getId() const { return "amr_" + std::to_string(id_); } 
    bool needsImmediateStatePublish() const { return needs_immediate_state_publish_; } 
    void resetImmediateStatePublishFlag() { needs_immediate_state_publish_ = false; }  
    // void setOrder(const std::vector<NodeInfo>& nodes, const std::vector<EdgeInfo>& edges) override;
    void setOrder(const std::vector<NodeInfo>& nodes, const std::vector<EdgeInfo>& edges, const std::vector<NodeInfo>& all_nodes, double wheel_base) override;
    std::string getState() const override;
    std::vector<NodeInfo> getNodes() const override { return nodes_; }
    std::size_t getCurIdx() const override { return cur_idx_; }
    // void step(double dt) override;
    void step(double dt, const std::vector<std::pair<double, double>>& other_robot_positions) override;  
    void updateBattery(double dt, bool is_charging) override;
    double getBatteryPercent() const override;

    std::vector<NodeInfo> getCurrentNodes() const override;
    std::vector<NodeInfo> getCompletedNodes() const override;
    std::vector<EdgeInfo> getCurrentEdges() const override;
    std::vector<EdgeInfo> getCompletedEdges() const override;
    std::string getLastNodeId() const override;
    int getLastNodeSequenceId() const override;    

    void markNodeAsCompleted(const NodeInfo& node) override;
    void cancelOrder() override;

    IVcu* getVcu() override;  

private:
    int id_;
    bool is_angle_adjusting_ = false;
    bool needs_immediate_state_publish_ = false;
    double wheel_base_;
    std::vector<NodeInfo> nodes_;
    std::vector<EdgeInfo> edges_;
    std::size_t cur_idx_ = 0;
    std::size_t cur_edge_idx_ = 0;
    std::unique_ptr<Vcu> vcu_;
    std::unique_ptr<IBatteryModel> battery_model_;

    std::vector<NodeInfo> completed_nodes_;
    std::vector<EdgeInfo> completed_edges_;

    std::vector<NodeInfo> all_nodes_;  // centerNode 찾기용 전체 노드 맵

    // IProtocol* protocol_;  // VDA5050 프로토콜 참조 추가

    Line getLineFromPoints(const NodeInfo& p1, const NodeInfo& p2);
    // NodeInfo calculateTangentPoint(const Line& line, const NodeInfo& center, double radius, bool firstPoint); 
    NodeInfo calculateTangentPoint(const Line& line, const NodeInfo& center, double radius, const NodeInfo& ref, bool preferCloser); 
    const NodeInfo* findNodeById(const std::vector<NodeInfo>& nodes, const std::string& id);
    void setVcuTargetFromEdge(const EdgeInfo& edge, const std::vector<NodeInfo>& nodes, const std::vector<NodeInfo>& all_nodes,  double wheel_base);

};