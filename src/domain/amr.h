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
    void setOrder(const std::vector<AmrNode>& nodes, const std::vector<AmrEdge>& edges) override;
    std::string getState() const override;
    std::vector<AmrNode> getNodes() const override { return nodes_; }
    std::size_t getCurIdx() const override { return cur_idx_; }
    void step() override;

    IVcu* getVcu() override;  
private:
    int id_;
    std::vector<AmrNode> nodes_;
    std::vector<AmrEdge> edges_;
    std::size_t cur_idx_ = 0;
    std::unique_ptr<Vcu> vcu_;
};