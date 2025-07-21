#include "amr.h"
#include <iostream>

Amr::Amr(int id, std::unique_ptr<Vcu> vcu) : id_(id), vcu_(std::move(vcu)), cur_idx_(0) 
{ 

}

void Amr::setOrder(const std::vector<AmrNode>& nodes, const std::vector<AmrEdge>& edges) 
{
    nodes_ = nodes;
    edges_ = edges;
    cur_idx_ = 0;

    if (!nodes_.empty()) 
    {
        std::cout << "[AMR " << getId() << "] Order set. Moving towards " << nodes_[0].id << std::endl;
        vcu_->setTargetPosition(nodes_[0].x, nodes_[0].y);
    }
}


// std::vector<AmrNode> Amr::getNodes() const 
// { 
//     return nodes_; 
// }

std::string Amr::getState() const 
{
    // if (nodes_.empty()) return "Idle";
    // return "AMR" + std::to_string(id_) + " at node: " + nodes_[cur_idx_].id;
    if (nodes_.empty()) 
        return "AMR" + std::to_string(id_) + ": Idle";
    return "AMR" + std::to_string(id_) + " at node: " + nodes_[cur_idx_].id;
}

void Amr::step() 
{
    // if (!nodes_.empty() && cur_idx_ + 1 < nodes_.size()) ++cur_idx_;
    // In a real system, this would involve VCU and Navigation updates
    // For now, if there's an order, simulate movement through nodes
    if (!nodes_.empty()) 
    {
        vcu_->update();

        // Simulate reaching next node if close enough or after some time
        // This is a very simplistic step simulation
        if (cur_idx_ + 1 < nodes_.size()) 
        {
            // Check if VCU has reached its target (simplistic check)
            // This needs to be much more sophisticated for real navigation
            // For now, let's just advance the index after a few steps
            static int step_counter = 0;
            if (step_counter % 10 == 0) 
            {
                ++cur_idx_;
                if (cur_idx_ < nodes_.size()) 
                {
                    std::cout << "[AMR " << getId() << "] Reached node " << nodes_[cur_idx_].id << std::endl;
                    vcu_->setTargetPosition(nodes_[cur_idx_].x, nodes_[cur_idx_].y);
                } 
                else 
                {
                    std::cout << "[AMR " << getId() << "] All nodes in order completed." << std::endl;
                    nodes_.clear();
                    cur_idx_ = 0;
                }
            }
            step_counter++;
        }
    }    
}