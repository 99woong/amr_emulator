// domain/protocols/vda5050Protocol.cpp

#include "vda5050_protocol.h"
#include "node_edge_info.h"
#include "iamr.h"
#include "ivcu.h"
#include <iostream>
#include <nlohmann/json.hpp>
#include <vda5050++/config.h>


Vda5050Protocol::Vda5050Protocol() : running_(false) 
{
    // 생성자에서 vda_config_와 handle_을 초기화할 수 있습니다.
    // 하지만 handle_.initialize(cfg)는 cfg가 완전히 설정된 후에 호출되어야 합니다.
}

void Vda5050Protocol::setAmr(IAmr* amr) 
{
    amr_ = amr;
}

void Vda5050Protocol::setAgvId(const std::string& agv_id) 
{
    agv_id_ = agv_id;
    vda_config_.refAgvDescription().agv_id = agv_id_;
}

void Vda5050Protocol::useDefaultConfig() 
{
    // Config 객체를 사용하여 AGV의 기본 정보 설정
    vda_config_.refAgvDescription().agv_id = agv_id_; // setAgvId에서 설정된 값 사용
    vda_config_.refAgvDescription().manufacturer = "zenix"; // 또는 실제 제조사 이름
    vda_config_.refAgvDescription().serial_number = "AMR_Emulator_001"; // 또는 적절한 시리얼 넘버
    // 필요하다면 다른 설정들도 추가합니다 (예: MQTT broker URL 등)
    
    auto& mqtt_conf = vda_config_.refMqttSubConfig();
    auto& opts = mqtt_conf.refOptions();
    opts.server = "tcp://localhost:1883"; // 호스트와 포트 포함

    // handle 초기화는 config가 완전히 설정된 후에 수행합니다.
    handle_ = std::make_unique<vda5050pp::Handle>();
    handle_->initialize(vda_config_);

    // 상태 토픽과 오더 토픽은 구성에서 가져올 수 있습니다.
    state_topic_ = "vda5050/agvs/" + agv_id_ + "/state"; // 예시, 라이브러리 내부에서 관리할 가능성이 높습니다.
    order_topic_ = "vda5050/agvs/" + agv_id_ + "/order"; // 예시
}

void Vda5050Protocol::start() 
{
    running_ = true;
    publish_thread_ = std::thread([this]() 
    {
        while (running_) 
        {
            // 1초 간격으로 AMR 상태 확인 후, (필요시) 상태 MQTT Publish 처리 로직
            std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 1초마다
            // 상태 갱신, 이벤트 발생 등 작업 가능
        }
    });
}

void Vda5050Protocol::stop() 
{
    running_ = false;
    if (publish_thread_.joinable()) 
    {
        publish_thread_.join();
    }
    
    if (handle_) 
    {
        handle_->shutdown();
    }
}

void Vda5050Protocol::handleMessage(const std::string& msg, IAmr* amr) 
{
    // ... (기존 로직 유지)
    // processVda5050Order(order_json);
    if (msg.empty() || !amr)
    {
        std::cerr << "[Vda5050Protocol] Empty message or AMR pointer null in handleMessage" << std::endl;
        return;
    }    
    
    try
    {
        auto order_json = nlohmann::json::parse(msg);

        if (!order_json.contains("nodes") || !order_json.contains("edges"))
        {
            std::cerr << "[Vda5050Protocol] Order message missing nodes or edges" << std::endl;
            return;
        }

        // 노드 수집
        std::vector<NodeInfo> nodes;
        for (const auto& node : order_json["nodes"])
        {
            NodeInfo n;
            n.nodeId = node.value("nodeId", "");
            n.sequenceId = node.value("sequenceId", 0);
            if (node.contains("nodePosition"))
            {
                n.x = node["nodePosition"].value("x", 0.0);
                n.y = node["nodePosition"].value("y", 0.0);
                n.theta = node["nodePosition"].value("theta", 0.0);
            }
            nodes.push_back(n);
        }

        // 엣지 수집
        std::vector<EdgeInfo> edges;
        for (const auto& edge : order_json["edges"])
        {
            EdgeInfo e;
            e.edgeId = edge.value("edgeId", "");
            e.sequenceId = edge.value("sequenceId", 0);
            e.startNodeId = edge.value("startNodeId", "");
            e.endNodeId = edge.value("endNodeId", "");
            edges.push_back(e);
        }

        // AMR의 VCU에 노드/엣지 상태 갱신 요청
        auto vcu = amr->getVcu();
        if (!vcu)
        {
            std::cerr << "[Vda5050Protocol] AMR's VCU pointer is null" << std::endl;
            return;
        }

        vcu->updateNodes(nodes);
        vcu->updateEdges(edges);

        // 필요 시 handle_를 통한 이벤트 전파 가능
        // 예) handle_->dispatch(...);

        std::cout << "[Vda5050Protocol] Processed order message for AGV '" << agv_id_ << "'" << std::endl;
    }
    catch (const nlohmann::json::parse_error& e)
    {
        std::cerr << "[Vda5050Protocol] JSON parse error in handleMessage: " << e.what() << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[Vda5050Protocol] Exception in handleMessage: " << e.what() << std::endl;
    }

}

std::string Vda5050Protocol::makeStateMessage(IAmr* amr) 
{
    // 이 메서드는 더 이상 JSON 문자열을 직접 만들지 않을 수 있습니다.
    // 대신, 필요한 AMR 상태 정보를 vda5050pp::Handle에 전달하여
    // 라이브러리가 내부적으로 상태 메시지를 생성하고 발행하도록 해야 합니다.

    // 예: AGV의 현재 노드, 위치, 배터리 상태 등을 라이브러리에 전달
    // 이 부분은 libvda5050pp가 어떤 인터페이스를 제공하는지에 따라 크게 달라집니다.
    // 예제 코드에서처럼 이벤트 기반으로 상태를 업데이트하는 경우,
    // vda5050pp::events::EventHandle을 통해 dispatch하는 방식이 될 수 있습니다.

    // 임시로 빈 문자열 반환 (이 메서드의 존재 목적이 변경될 가능성)
    return std::string();

    // 예시: vda5050pp::Handle에 상태 업데이트를 요청하는 가상의 코드 (실제 API에 따라 다름)
    // auto status_update_event = std::make_shared<vda5050pp::events::AGVStateUpdate>();
    // status_update_event->current_node_id = amr->getCurrentNodeId();
    // status_update_event->battery_level = amr->getBatteryLevel();
    // handle_->dispatch(status_update_event);
}

void Vda5050Protocol::processVda5050Order(const nlohmann::json& order_json) 
{
    // ... (기존 로직 유지)
    // 이 함수는 들어오는 MQTT 오더 메시지를 처리하는 데 사용됩니다.
    // 이는 vda5050pp::handler::BaseNavigationHandler를 통해 처리될 것입니다.
}
