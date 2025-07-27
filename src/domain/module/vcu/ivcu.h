#pragma once
#include "node_edge_info.h"
#include <vector>

class IVcu 
{
public:
    virtual ~IVcu() = default;
    virtual void setTargetPosition(double x, double y) = 0;
    virtual void update() = 0;

    virtual void updateNodes(const std::vector<NodeInfo>& nodes) = 0;
    virtual void updateEdges(const std::vector<EdgeInfo>& edges) = 0;    
};
