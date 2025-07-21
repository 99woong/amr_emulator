#pragma once
#include "iamr.h"
#include "ivcu.h"
#include "vcu.h"
#include <vector>
#include <string>
#include <memory>

// Define AmrNode and AmrEdge structs here or in a common header if not already defined
// (Based on the context, these were likely defined in iamr.h in the previous structure)
// Re-adding them here for clarity if they were missed.
// struct AmrNode 
// {
//     std::string id;
//     double x;
//     double y;
// };

// struct AmrEdge 
// {
//     std::string id;
//     std::string from;
//     std::string to;
// };


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
private:
    int id_;
    std::vector<AmrNode> nodes_;
    std::vector<AmrEdge> edges_;
    std::size_t cur_idx_ = 0;
    std::unique_ptr<Vcu> vcu_;
};