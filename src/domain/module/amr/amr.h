#pragma once
#include "iamr.h"
#include "ivcu.h"
#include "vcu.h"
#include <vector>
#include <string>
#include <memory>

class Amr : public IAmr 
{
public:
    Amr(int id, std::unique_ptr<Vcu> vcu);
    std::string getId() const { return "amr_" + std::to_string(id_); }    
    void setOrder(const std::vector<NodeInfo>& nodes, const std::vector<EdgeInfo>& edges) override;
    std::string getState() const override;
    std::vector<NodeInfo> getNodes() const override { return nodes_; }
    std::size_t getCurIdx() const override { return cur_idx_; }
    void step(double dt) override;

    IVcu* getVcu() override;  
private:
    int id_;
    bool is_angle_adjusting_ = false;
    std::vector<NodeInfo> nodes_;
    std::vector<EdgeInfo> edges_;
    std::size_t cur_idx_ = 0;
    std::unique_ptr<Vcu> vcu_;
};