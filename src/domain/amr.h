#pragma once
#include "iamr.h"
#include "ivcu.h"
#include <vector>
#include <string>

class Amr : public IAmr {
public:
    Amr(int id);
    void setOrder(const std::vector<AmrNode>& nodes, const std::vector<AmrEdge>& edges) override;
    std::string getState() const override;
    std::vector<AmrNode> getNodes() const override { return nodes_; }
    std::size_t getCurIdx() const override { return cur_idx_; }
    void step() override;
private:
    int id_;
    std::vector<AmrNode> nodes_;
    std::vector<AmrEdge> edges_;
    std::size_t cur_idx_ = 0;
};