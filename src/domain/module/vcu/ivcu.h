#pragma once
#include "node_edge_info.h"
#include <vector>

class IVcu 
{
public:
    virtual ~IVcu() = default;
    virtual void setTargetPosition(double x, double y) = 0;
    virtual void update(double dt) = 0;

    virtual void updateNodes(const std::vector<NodeInfo>& nodes) = 0;
    virtual void updateEdges(const std::vector<EdgeInfo>& edges) = 0;
    
    virtual void getEstimatedPose(double& x, double& y, double& theta) const = 0;
    virtual void setInitialPose(double x, double y, double theta) = 0;    
};
