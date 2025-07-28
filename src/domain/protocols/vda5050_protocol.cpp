// domain/protocols/vda5050Protocol.cpp

#include "vda5050_protocol.h"
#include "node_edge_info.h"
#include "iamr.h"
#include "ivcu.h"
#include <iostream>
#include <nlohmann/json.hpp>
#include <vda5050++/config.h>


void Vda5050Protocol::Vda5050MqttCallback::message_arrived(mqtt::const_message_ptr msg) 
{
    std::cout << "[MQTT] Message arrived on topic: " << msg->get_topic() << std::endl;
    if (msg->get_topic() == proto_->order_topic_) 
    {
        proto_->handleMessage(msg->to_string(), proto_->amr_);
    }
}


Vda5050Protocol::Vda5050Protocol() : running_(false) 
{
    // 생성자에서 vda_config_와 handle_을 초기화할 수 있습니다.
    // 하지만 handle_.initialize(cfg)는 cfg가 완전히 설정된 후에 호출되어야 합니다.
}

Vda5050Protocol::~Vda5050Protocol() 
{
    stop();
}


void Vda5050Protocol::setAmr(IAmr* amr) 
{
    amr_ = amr;
}

void Vda5050Protocol::setAgvId(const std::string& agv_id) 
{
    agv_id_ = agv_id;
    // vda_config_.refAgvDescription().agv_id = agv_id_;
    order_topic_ = "vda5050/agvs/" + agv_id + "/order";

}

void Vda5050Protocol::useDefaultConfig() 
{
    mqtt_server_uri_ = "tcp://localhost:1883";
    conn_opts_.set_clean_session(true);
    mqtt_client_ = std::make_unique<mqtt::async_client>(mqtt_server_uri_, agv_id_ + "_client");
    mqtt_callback_ = std::make_shared<Vda5050MqttCallback>(this);
    mqtt_client_->set_callback(*mqtt_callback_);

    state_topic_ = "vda5050/agvs/" + agv_id_ + "/state";
    order_topic_ = "vda5050/agvs/" + agv_id_ + "/order";

    // // Config 객체를 사용하여 AGV의 기본 정보 설정
    // vda_config_.refAgvDescription().agv_id = agv_id_; // setAgvId에서 설정된 값 사용
    // vda_config_.refAgvDescription().manufacturer = "zenix"; // 또는 실제 제조사 이름
    // vda_config_.refAgvDescription().serial_number = "AMR_Emulator_001"; // 또는 적절한 시리얼 넘버
    // // 필요하다면 다른 설정들도 추가합니다 (예: MQTT broker URL 등)
    
    // auto& mqtt_conf = vda_config_.refMqttSubConfig();
    // auto& opts = mqtt_conf.refOptions();
    // opts.server = "tcp://localhost:1883"; // 호스트와 포트 포함

    // // handle 초기화는 config가 완전히 설정된 후에 수행합니다.
    // handle_ = std::make_unique<vda5050pp::Handle>();
    // handle_->initialize(vda_config_);

    // // 상태 토픽과 오더 토픽은 구성에서 가져올 수 있습니다.
    // state_topic_ = "vda5050/agvs/" + agv_id_ + "/state"; // 예시, 라이브러리 내부에서 관리할 가능성이 높습니다.
    // order_topic_ = "vda5050/agvs/" + agv_id_ + "/order"; // 예시
}

void Vda5050Protocol::start() 
{
    try 
    {
        mqtt_client_->connect(conn_opts_)->wait();
        mqtt_client_->subscribe(order_topic_, 1)->wait();
        running_ = true;
        std::cout << "[Vda5050Protocol] MQTT connected & subscribed to: " << order_topic_ << std::endl;

        publish_thread_ = std::thread([this]()
        {
            while (running_)
            {
                // AMR가 있으면 상태 메시지 생성 및 발행
                if (amr_ && mqtt_client_ && mqtt_client_->is_connected())
                {
                    try
                    {
                        std::string state_msg = makeStateMessage(amr_);
                        if (!state_msg.empty())
                        {
                            auto msg = mqtt::make_message(state_topic_, state_msg);
                            msg->set_qos(1);
                            mqtt_client_->publish(msg);
                        }
                    }
                    catch (const std::exception& e)
                    {
                        std::cerr << "[Vda5050Protocol] Exception during state publish: " << e.what() << std::endl;
                    }
                }

                std::this_thread::sleep_for(std::chrono::seconds(1)); // 1초 간격 발행
            }
        });        
    } 
    catch (const mqtt::exception& e) 
    {
        std::cerr << "[Vda5050Protocol] MQTT connection failed: " << e.what() << std::endl;
    }

    // running_ = true;
    // publish_thread_ = std::thread([this]() 
    // {
    //     while (running_) 
    //     {
    //         // 1초 간격으로 AMR 상태 확인 후, (필요시) 상태 MQTT Publish 처리 로직
    //         std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 1초마다
    //         // 상태 갱신, 이벤트 발생 등 작업 가능
    //     }
    // });
}

void Vda5050Protocol::stop() 
{
    if (running_ && mqtt_client_ && mqtt_client_->is_connected()) 
    {
        try 
        {
            mqtt_client_->unsubscribe(order_topic_)->wait();
            mqtt_client_->disconnect()->wait();
        } catch (...) 
        {

        }
    }
    
    running_ = false;
    if (publish_thread_.joinable())
    {
        publish_thread_.join();
    }

    if (mqtt_client_ && mqtt_client_->is_connected()) 
    {
        try 
        {
            mqtt_client_->unsubscribe(order_topic_)->wait();
            mqtt_client_->disconnect()->wait();
        } 
        catch (...) 
        {

        }
    }    
    // if (handle_) 
    // {
    //     handle_->shutdown();
    // }
}

void Vda5050Protocol::handleMessage(const std::string& msg, IAmr* amr)
{
    std::cout << "[Vda5050Protocol] handleMessage called with message size: " << msg.size() << std::endl;

    if (msg.empty() || !amr) 
    {
        std::cerr << "[Vda5050Protocol] Empty message or null AMR pointer\n";
        return;
    }

    try 
    {
        auto order_json = nlohmann::json::parse(msg);
        std::cout << "[Vda5050Protocol] JSON parsed successfully." << std::endl;

        if (!order_json.contains("nodes"))
        {
            std::cerr << "[Vda5050Protocol] JSON missing field: nodes" << std::endl;
        }
        else
        {
            std::cout << "[Vda5050Protocol] JSON contains nodes: " << order_json["nodes"].size() << std::endl;
        }

        if (!order_json.contains("edges"))
        {
            std::cerr << "[Vda5050Protocol] JSON missing field: edges" << std::endl;
        }
        else
        {
            std::cout << "[Vda5050Protocol] JSON contains edges: " << order_json["edges"].size() << std::endl;
        }


        if (!order_json.contains("nodes") || !order_json.contains("edges")) 
        {
            std::cerr << "[Vda5050Protocol] Order missing nodes or edges\n";
            return;
        }

        
        // 1) nodes ID별 맵 생성
        std::unordered_map<std::string, NodeInfo> node_map;
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
            node_map[n.nodeId] = n;
            
            std::cout << "[Vda5050Protocol] Parsed NodeId: " << n.nodeId << ", Pos: (" << n.x << "," << n.y << "," << n.theta << ")" << std::endl;            
        }

        // 2) edges를 순서(sequenceId) 정렬 & startNodeId → endNodeId 매핑 생성
        std::vector<EdgeInfo> edges;
        for (const auto& edge : order_json["edges"])
        {
            EdgeInfo e;
            e.edgeId = edge.value("edgeId", "");
            e.sequenceId = edge.value("sequenceId", 0);
            e.startNodeId = edge.value("startNodeId", "");
            e.endNodeId = edge.value("endNodeId", "");
            edges.push_back(e);

            std::cout << "[Vda5050Protocol] Parsed EdgeId: " << e.edgeId << ", Start: " << e.startNodeId << ", End: " << e.endNodeId << std::endl;            
        }

        std::sort(edges.begin(), edges.end(), [](const EdgeInfo& a, const EdgeInfo& b)
        {
            return a.sequenceId < b.sequenceId;
        });

        // 3) edges를 따라 순차적 노드 경로 리스트 생성
        std::vector<NodeInfo> ordered_nodes;
        if (!edges.empty()) 
        {
            std::string current_node_id = edges.front().startNodeId;
            ordered_nodes.push_back(node_map[current_node_id]);

            for (const auto& edge : edges) 
            {
                // endNode를 추가 (startNode는 이미 추가됐으므로 중복 방지)
                if (node_map.find(edge.endNodeId) != node_map.end()) 
                {
                    ordered_nodes.push_back(node_map[edge.endNodeId]);
                } 
                else 
                {
                    std::cerr << "[Vda5050Protocol] Missing node info for id: " << edge.endNodeId << std::endl;
                }
            }
        } 
        else 
        {
            std::cerr << "[Vda5050Protocol] Empty edges array in order\n";
        }

        // 4) AMR에 edges 기반 순서로 된 노드 리스트와 원본 edges 전달
        amr->setOrder(ordered_nodes, edges);

        std::cout << "[Vda5050Protocol] Order processed with " << ordered_nodes.size()
                  << " nodes & " << edges.size() << " edges assigned to AMR.\n";

    } 
    catch (const std::exception& e) 
    {
        std::cerr << "[Vda5050Protocol] JSON parse error: " << e.what() << std::endl;
    }
}
std::string Vda5050Protocol::makeStateMessage(IAmr* amr)
{
    if (!amr) 
    {
        return {};
    }
    
    nlohmann::json state_json;

    // 예: 현재 위치, 방향, 배터리 잔량 등을 구조체/메서드에서 얻어와서 JSON에 채움
    double x = 0.0, y = 0.0, theta = 0.0;
    amr->getVcu()->getEstimatedPose(x, y, theta);

    // AMR 상태명 (예: Idle, Moving 등) - amr->getState() 가 있다고 가정
    std::string state_str = amr->getState();

    // 배터리 정보 등을 amr에서 가져올 수 있으면 추가

    state_json["headerId"] = "state_update";
    state_json["timestamp"] = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::seconds>(
                             std::chrono::system_clock::now().time_since_epoch()).count());
    state_json["agvId"] = agv_id_;
    state_json["state"] = state_str;

    state_json["pose"] = {
        {"x", x},
        {"y", y},
        {"theta", theta}
    };

    // 예시로 배터리 잔량 75% 표시
    state_json["battery"] = {
        {"percentage", 75}
    };

    // 필요한 경우 추가 상태 정보 삽입 가능

    return state_json.dump();
}

// std::string Vda5050Protocol::makeStateMessage(IAmr* amr) 
// {
//     // 이 메서드는 더 이상 JSON 문자열을 직접 만들지 않을 수 있습니다.
//     // 대신, 필요한 AMR 상태 정보를 vda5050pp::Handle에 전달하여
//     // 라이브러리가 내부적으로 상태 메시지를 생성하고 발행하도록 해야 합니다.

//     // 예: AGV의 현재 노드, 위치, 배터리 상태 등을 라이브러리에 전달
//     // 이 부분은 libvda5050pp가 어떤 인터페이스를 제공하는지에 따라 크게 달라집니다.
//     // 예제 코드에서처럼 이벤트 기반으로 상태를 업데이트하는 경우,
//     // vda5050pp::events::EventHandle을 통해 dispatch하는 방식이 될 수 있습니다.

//     // 임시로 빈 문자열 반환 (이 메서드의 존재 목적이 변경될 가능성)
//     return std::string();

//     // 예시: vda5050pp::Handle에 상태 업데이트를 요청하는 가상의 코드 (실제 API에 따라 다름)
//     // auto status_update_event = std::make_shared<vda5050pp::events::AGVStateUpdate>();
//     // status_update_event->current_node_id = amr->getCurrentNodeId();
//     // status_update_event->battery_level = amr->getBatteryLevel();
//     // handle_->dispatch(status_update_event);
// }

void Vda5050Protocol::processVda5050Order(const nlohmann::json& order_json) 
{
    // ... (기존 로직 유지)
    // 이 함수는 들어오는 MQTT 오더 메시지를 처리하는 데 사용됩니다.
    // 이는 vda5050pp::handler::BaseNavigationHandler를 통해 처리될 것입니다.
}
