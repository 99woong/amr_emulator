// #include "vda5050Protocol.h"
// #include "../../domain/interface/iamr.h"
// #include <iostream>

// Vda5050Protocol::Vda5050Protocol() : amr_(nullptr) 
// {

// }

// void Vda5050Protocol::setAmr(IAmr* amr) 
// { 
//     amr_ = amr; 
// }


// void Vda5050Protocol::start() 
// {
//     order_observer_.onBaseChanged([this](const auto& nodes, const auto& edges) {
//         std::vector<AmrNode> nodeList;
//         std::vector<AmrEdge> edgeList;
//         for (const auto& node : nodes)
//             nodeList.push_back({node->nodeId, node->nodePosition->x, node->nodePosition->y});
//         for (const auto& edge : edges)
//             edgeList.push_back({edge->edgeId, edge->startNodeId, edge->endNodeId});
//         if (amr_) amr_->setOrder(nodeList, edgeList);

//         std::cout << "[VDA5050] Order received with "
//                   << nodeList.size() << " nodes, " << edgeList.size() << " edges\n";
//         for(const auto& n : nodeList) std::cout << "  Node: " << n.id << " (" << n.x << "," << n.y << ")\n";
//         for(const auto& e : edgeList) std::cout << "  Edge: " << e.id << " : " << e.from << "->" << e.to << std::endl;
//     });
// }

// void Vda5050Protocol::handleMessage(const std::string&, IAmr*) 
// {
//     // VDA5050는 이벤트 기반이므로 수동 메시지 불필요
// }

// std::string Vda5050Protocol::makeStateMessage(IAmr* amr) 
// {
//     if (!amr) return "";
//     auto nodes = amr->getNodes();
//     std::size_t idx = amr->getCurIdx();
//     if (nodes.empty()) return "Idle";
//     return "At [" + nodes[idx].id + "]";
// }

#include "vda5050Protocol.h"
#include <iostream>

Vda5050Protocol::Vda5050Protocol() {}

void Vda5050Protocol::setAgvId(const std::string& agv_id) { agv_id_ = agv_id; }

void Vda5050Protocol::useDefaultConfig() 
{

    vda_config_.refAgvDescription().agv_id = agv_id_;
    vda_config_.refAgvDescription().serial_number = agv_id_; // 각각 다르게!
    vda_config_.refAgvDescription().manufacturer = "MyCompany";

    vda_config_.refMqttSubConfig().refOptions().server = "tcp://localhost:1883";
    vda_config_.refMqttSubConfig().refOptions().interface = agv_id_; // 각 AMR
//     vda_config_.refAgvDescription().agv_id = agv_id_;
//     vda_config_.refAgvDescription().description = "AMR[" + agv_id_ + "]";
//     vda_config_.refAgvDescription().manufacturer = "MyCompany_(vda5050pp)";
//     // ... 추가 config 필요시 }...

//     // 예: MQTT 등
//     vda_config_.refMqttSubConfig().refOptions().server = "tcp://localhost:1883";
//     vda_config_.refMqttSubConfig().refOptions().client_id = agv_id_;
//     vda_config_.refMqttSubConfig().refOptions().username = "testuser";
//     vda_config_.refMqttSubConfig().refOptions().password = "testpassword";
//     vda_config_.refMqttSubConfig().refOptions().use_ssl  = false;
}

void Vda5050Protocol::setAmr(IAmr* amr) 
{ 
    amr_ = amr; 
}

void Vda5050Protocol::start() 
{
    if (!handle_) handle_ = std::make_unique<vda5050pp::Handle>();
    handle_->initialize(vda_config_);

    // 핸들러, 콜백 등 등록 예시(직접 amr 등 연결)
    // handle_->registerActionHandler(std::make_shared<MyActionHandler>());
    // handle_->registerNavigationHandler(std::make_shared<MyNavHandler>());

    std::cout << "[VDA5050][" << agv_id_ << "] Protocol started!" << std::endl;
}

void Vda5050Protocol::handleMessage(const std::string&, IAmr*) 
{ 
    /*불필요(이벤트 기반)*/ 
}

std::string Vda5050Protocol::makeStateMessage(IAmr* amr) {
    if (!amr) return "";
    // ... amr->getNodes(), getCurIdx() 등
    return {};
}