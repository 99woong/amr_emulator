#pragma once
#include <string>

struct NodeInfo 
{
    std::string nodeId;
    int sequenceId;
    double x;
    double y;
    double theta;

    NodeInfo() : sequenceId(0), x(0), y(0), theta(0) {}
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
    double maxSpeed;
    PositionInfo turnCenter;

    EdgeInfo() : sequenceId(0),maxSpeed(0.0) {}
};