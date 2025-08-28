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
    virtual void setOrder(const std::vector<NodeInfo>& nodes, const std::vector<EdgeInfo>& edges, double wheel_base) = 0;
    virtual std::string getState() const = 0;
    virtual std::vector<NodeInfo> getNodes() const = 0;
    virtual std::size_t getCurIdx() const = 0;
    virtual void step(double dt) = 0;

    virtual IVcu* getVcu() = 0;  // VCU 접근용 인터페이스
};