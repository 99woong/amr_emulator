#pragma once
#include <string>

struct NodeInfo 
{
    std::string nodeId;
    std::string description;
    int sequenceId;
    double x;
    double y;

    // double theta;

    NodeInfo() : sequenceId(0), x(0), y(0){}
    // NodeInfo() : sequenceId(0), x(0), y(0), theta(0) {}
};

struct PositionInfo 
{
    double x;
    double y;
    PositionInfo() : x(0), y(0) {}
};


struct EdgeInfo 
{
    std::string edgeId;
    int sequenceId;
    std::string startNodeId;
    std::string endNodeId;
    std::string description;
    double maxSpeed;
    // std::string turnCenter;
    bool has_turn_center = false;
    double turn_center_x = 0.0;
    double turn_center_y = 0.0;    

     EdgeInfo()
        : sequenceId(0), maxSpeed(0.0),
          has_turn_center(false),      
          turn_center_x(0.0), turn_center_y(0.0)
    {}
};
