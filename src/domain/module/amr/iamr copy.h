#pragma once
#include <vector>
#include <string>
#include "node_edge_info.h"

struct AmrNode 
{
    std::string id;
    double x;
    double y;
};

struct AmrEdge 
{
    std::string id;
    std::string from;
    std::string to;
};

class IVcu;  // 전방 선언

class IAmr {
public:
    virtual ~IAmr() = default;
    virtual void step(double dt, const std::vector<std::pair<double, double>>& other_robot_positions) = 0;
    virtual std::string getState() const = 0;
    virtual std::vector<NodeInfo> getNodes() const = 0; //??
    virtual std::size_t getCurIdx() const = 0;

    virtual IVcu* getVcu() = 0;  // VCU 접근용 인터페이스

    virtual void setOrder(const std::vector<NodeInfo>& nodes, const std::vector<EdgeInfo>& edges, const std::vector<NodeInfo>& all_nodes, double wheel_base) = 0;
    virtual void cancelOrder() = 0;

    virtual void markNodeAsCompleted(const NodeInfo& node) = 0;

    virtual std::vector<NodeInfo> getCurrentNodes() const = 0;
    virtual std::vector<EdgeInfo> getCurrentEdges() const = 0;
    virtual std::vector<NodeInfo> getCompletedNodes() const = 0;
    virtual std::vector<EdgeInfo> getCompletedEdges() const = 0;
    virtual std::string getLastNodeId() const = 0;
    virtual int getLastNodeSequenceId() const = 0;    

    virtual void updateBattery(double dt, bool is_charging) = 0;
    virtual double getBatteryPercent() const = 0; 

};