#pragma once
#include <vector>
#include <string>

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

class IAmr {
public:
    virtual ~IAmr() = default;
    virtual void setOrder(const std::vector<AmrNode>& nodes, const std::vector<AmrEdge>& edges) = 0;
    virtual std::string getState() const = 0;
    virtual std::vector<AmrNode> getNodes() const = 0;
    virtual std::size_t getCurIdx() const = 0;
    virtual void step() = 0;
};