#include "vda5050Protocol.h"
#include "../../domain/interface/iamr.h"
#include <iostream>

Vda5050Protocol::Vda5050Protocol() : amr_(nullptr) 
{

}

void Vda5050Protocol::setAmr(IAmr* amr) 
{ 
    amr_ = amr; 
}


void Vda5050Protocol::start() 
{
    order_observer_.onBaseChanged([this](const auto& nodes, const auto& edges) {
        std::vector<AmrNode> nodeList;
        std::vector<AmrEdge> edgeList;
        for (const auto& node : nodes)
            nodeList.push_back({node->nodeId, node->nodePosition->x, node->nodePosition->y});
        for (const auto& edge : edges)
            edgeList.push_back({edge->edgeId, edge->startNodeId, edge->endNodeId});
        if (amr_) amr_->setOrder(nodeList, edgeList);

        std::cout << "[VDA5050] Order received with "
                  << nodeList.size() << " nodes, " << edgeList.size() << " edges\n";
        for(const auto& n : nodeList) std::cout << "  Node: " << n.id << " (" << n.x << "," << n.y << ")\n";
        for(const auto& e : edgeList) std::cout << "  Edge: " << e.id << " : " << e.from << "->" << e.to << std::endl;
    });
}

void Vda5050Protocol::handleMessage(const std::string&, IAmr*) 
{
    // VDA5050는 이벤트 기반이므로 수동 메시지 불필요
}

std::string Vda5050Protocol::makeStateMessage(IAmr* amr) 
{
    if (!amr) return "";
    auto nodes = amr->getNodes();
    std::size_t idx = amr->getCurIdx();
    if (nodes.empty()) return "Idle";
    return "At [" + nodes[idx].id + "]";
}