#pragma once
#include "iamr.h"
#include "ivcu.h"
#include "vcu.h"
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
    Amr(int id, std::unique_ptr<Vcu> vcu);
    std::string getId() const { return "amr_" + std::to_string(id_); }    
    // void setOrder(const std::vector<NodeInfo>& nodes, const std::vector<EdgeInfo>& edges) override;
    void setOrder(const std::vector<NodeInfo>& nodes, const std::vector<EdgeInfo>& edges, double wheel_base) override;
    std::string getState() const override;
    std::vector<NodeInfo> getNodes() const override { return nodes_; }
    std::size_t getCurIdx() const override { return cur_idx_; }
    // void step(double dt) override;
    void step(double dt, const std::vector<std::pair<double, double>>& other_robot_positions) override;  

    IVcu* getVcu() override;  
private:
    int id_;
    bool is_angle_adjusting_ = false;
    std::vector<NodeInfo> nodes_;
    std::vector<EdgeInfo> edges_;
    std::size_t cur_idx_ = 0;
    std::size_t cur_edge_idx_ = 0;
    std::unique_ptr<Vcu> vcu_;

    Line getLineFromPoints(const NodeInfo& p1, const NodeInfo& p2);
    // NodeInfo calculateTangentPoint(const Line& line, const NodeInfo& center, double radius, bool firstPoint); 
    NodeInfo calculateTangentPoint(const Line& line, const NodeInfo& center, double radius, const NodeInfo& ref, bool preferCloser); 
    const NodeInfo* findNodeById(const std::vector<NodeInfo>& nodes, const std::string& id);
};