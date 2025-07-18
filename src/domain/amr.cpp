#include "amr.h"

Amr::Amr(int id) : id_(id), cur_idx_(0)
{

}

void Amr::setOrder(const std::vector<AmrNode>& nodes, const std::vector<AmrEdge>& edges) 
{
    nodes_ = nodes;
    edges_ = edges;
    cur_idx_ = 0;
}


std::vector<AmrNode> Amr::getNodes() const 
{ 
    return nodes_; 
}

std::string Amr::getState() const 
{
    if (nodes_.empty()) return "Idle";
    return "AMR" + std::to_string(id_) + " at node: " + nodes_[cur_idx_].id;
}

void Amr::step() 
{
    if (!nodes_.empty() && cur_idx_ + 1 < nodes_.size()) ++cur_idx_;
}