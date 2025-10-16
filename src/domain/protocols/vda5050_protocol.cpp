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
    if (msg->get_topic() == proto_->order_topic_ || msg->get_topic() == proto_->instant_actions_topic) 
    {
        proto_->handleMessage(msg->to_string(), proto_->amr_);
    }
}


Vda5050Protocol::Vda5050Protocol(const AmrConfig& config) : running_(false), config_(config) 
{
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
    order_topic_ = "vda5050/agvs/" + agv_id + "/order";
    std::cout << "set_order : " << order_topic_ << std::endl;

}

void Vda5050Protocol::useDefaultConfig(const std::string& server_address) 
{
    // mqtt_server_uri_ = "tcp://localhost:1883";
    std::cout << "mqtt_server_address: " << server_address << std::endl;
    mqtt_server_uri_ = server_address;
    conn_opts_.set_clean_session(true);
    mqtt_client_ = std::make_unique<mqtt::async_client>(mqtt_server_uri_, agv_id_ + "_client");
    mqtt_callback_ = std::make_shared<Vda5050MqttCallback>(this);
    mqtt_client_->set_callback(*mqtt_callback_);

    state_topic_ = "vda5050/v1/ZENIXROBOTICS/" + agv_id_ + "/state";
    order_topic_ = "vda5050/v1/ZENIXROBOTICS/" + agv_id_ + "/order";
    instant_actions_topic = "vda5050/v1/ZENIXROBOTICS/" + agv_id_ + "/instantActions";
    visualization_topic_ = "vda5050/v1/ZENIXROBOTICS/" + agv_id_ + "/visualization";
    connection_topic_ = "vda5050/v1/ZENIXROBOTICS/" + agv_id_ + "/connection";
}

void Vda5050Protocol::start() 
{
    try 
    {
        mqtt_client_->connect(conn_opts_)->wait();

        // std::string connect_msg = makeConnectMessage();
        // mqtt_client_->publish(connection_topic_, connect_msg.c_str(), connect_msg.size(), 1, false);
        std::string connect_msg = makeConnectMessage();
        auto pubmsg = mqtt::make_message(connection_topic_, connect_msg);
        pubmsg->set_qos(1);       // 필요하면 QoS 설정
        pubmsg->set_retained(false);  // 필요하면 retained 설정

        mqtt_client_->publish(pubmsg);
        mqtt_client_->subscribe(order_topic_, 1)->wait();
        mqtt_client_->subscribe(instant_actions_topic, 1)->wait();
        running_ = true;
        std::cout << "[Vda5050Protocol] MQTT connected & subscribed to: " << order_topic_ << " and " << instant_actions_topic << std::endl;

        publish_thread_ = std::thread([this]()
        {
            while (running_)
            {
                std::this_thread::sleep_for(std::chrono::seconds(1)); // 1초 간격 발행
            }
        });        
    } 
    catch (const mqtt::exception& e) 
    {
        std::cerr << "[Vda5050Protocol] MQTT connection failed: " << e.what() << std::endl;
    }
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
}

// ISO8601 UTC 타임스탬프 생성 함수 정의
std::string Vda5050Protocol::getCurrentTimestampISO8601()
{
    using namespace std::chrono;
    auto now = system_clock::now();
    std::time_t now_c = system_clock::to_time_t(now);
    auto milliseconds = duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::tm utc_tm;
#ifdef _WIN32
    gmtime_s(&utc_tm, &now_c);
#else
    gmtime_r(&now_c, &utc_tm);
#endif

    std::ostringstream oss;
    oss << std::put_time(&utc_tm, "%Y-%m-%dT%H:%M:%S");
    oss << '.' << std::setfill('0') << std::setw(3) << milliseconds.count() << "Z";
    return oss.str();
}

std::string Vda5050Protocol::makeFactsheetMessage()
{
    nlohmann::json factsheet_json;

    // headerId 자동 증가
    factsheet_json["headerId"] = factsheet_header_id_++;
    factsheet_json["timestamp"] = getCurrentTimestampISO8601();
    factsheet_json["version"] = "0.0.1";
    factsheet_json["manufacturer"] = "ZenixRobotics";       // 실제 회사명으로 변경
    factsheet_json["serialNumber"] = agv_id_;                  // agv_id_ 멤버 변수 정의 필요

    nlohmann::json physicalParams;
    physicalParams["speedMin"] = config_.amr_params.min_speed;
    physicalParams["speedMax"] = config_.amr_params.max_speed;
    physicalParams["angularSpeedMin"] = config_.amr_params.angularSpeedMin;
    physicalParams["angularSpeedMax"] = config_.amr_params.angularSpeedMax;
    physicalParams["accelerationMax"] = config_.amr_params.accelerationMax;
    physicalParams["decelerationMax"] = config_.amr_params.decelerationMax;
    physicalParams["heightMin"] = config_.amr_params.heightMin;
    physicalParams["heightMax"] = config_.amr_params.heightMax;
    physicalParams["width"] = config_.amr_params.width;
    physicalParams["length"] = config_.amr_params.length;

    factsheet_json["physicalParameters"] = physicalParams;

    return factsheet_json.dump();
}

std::string Vda5050Protocol::detectConnection()
{
    // todo: ONLINE, OFFLINE, CONNECTIONBROKEN 검출로직 추가 필요
    return "ONLINE";
}

std::string Vda5050Protocol::makeConnectMessage()
{
    nlohmann::json factsheet_json;

    // headerId 자동 증가
    factsheet_json["headerId"] = factsheet_header_id_++;
    factsheet_json["timestamp"] = getCurrentTimestampISO8601();
    factsheet_json["version"] = "0.0.1";
    factsheet_json["manufacturer"] = "ZenixRobotics";       // 실제 회사명으로 변경
    factsheet_json["serialNumber"] = agv_id_;                  // agv_id_ 멤버 변수 정의 필요   
    factsheet_json["connectionState"] = detectConnection();                  

    return factsheet_json.dump();
}

void Vda5050Protocol::handleInstantAction(const nlohmann::json& instant_action_json)
{
    if (!mqtt_client_ || !mqtt_client_->is_connected())
    {
        std::cerr << "[Vda5050Protocol] MQTT client disconnected, cannot send factsheet.\n";
        return;
    }

    try
    {
        // instant_action_json 내 actionType 이 factsheet 요청인지 확인 (예시 키 이름, 값은 실제 프로토콜 참고)
        if (instant_action_json.contains("actionType") &&
            instant_action_json["actionType"] == "factsheetRequest")
        {
            std::string factsheet_msg = makeFactsheetMessage();
            std::string instant_action_topic = "vda5050/agvs/" + agv_id_ + "/instantActions";

            // std::cout << factsheet_msg << std::endl;
            auto msg = mqtt::make_message(instant_action_topic, factsheet_msg);
            msg->set_qos(1);
            mqtt_client_->publish(msg);
            // std::cout << "[Vda5050Protocol] Factsheet message published on topic: " << instant_action_topic << std::endl;
        }
        else
        {
            std::cout << "[Vda5050Protocol] Unknown or unsupported instant action type.\n";
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "[Vda5050Protocol] Exception publishing factsheet: " << e.what() << std::endl;
    }
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

        if (order_json.contains("instantActions") && order_json["instantActions"].is_array())
        {
            // std::cout <<"recv instantAction" <<std::endl;
            for (const auto& instant_action : order_json["instantActions"])
            {
                handleInstantAction(instant_action);
            }
        }        

        if (!order_json.contains("nodes") || !order_json.contains("edges")) 
        {
            std::cerr << "[Vda5050Protocol] Order missing nodes or edges\n";
            return;
        }

        //nodes ID별 맵생성
        std::unordered_map<std::string, NodeInfo> node_map;
        std::vector<NodeInfo> ordered_nodes;

        for (const auto& node : order_json["nodes"])
        {
            NodeInfo n;
            n.nodeId = node.value("nodeId", "");
            n.sequenceId = node.value("sequenceId", 0);
            if (node.contains("nodePosition"))
            {
                n.x = node["nodePosition"].value("x", 0.0);
                n.y = node["nodePosition"].value("y", 0.0);
            }
            node_map[n.nodeId] = n;

            ordered_nodes.push_back(n);
            
           std::cout << "[Vda5050Protocol] Parsed NodeId: " << n.nodeId << ", Pos: (" << n.x << "," << n.y << ")" << std::endl;            
        }

        std::vector<EdgeInfo> edges;
        for (const auto& edge : order_json["edges"])
        {
            EdgeInfo e;
            e.edgeId = edge.value("edgeId", "");
            e.sequenceId = edge.value("sequenceId", 0);
            e.startNodeId = edge.value("startNodeId", "");
            e.endNodeId = edge.value("endNodeId", "");
            e.centerNodeId = edge.value("centerNodeId", "");
            e.maxSpeed = edge.value("maxSpeed", 0.0);
            
            if (edge.contains("centerNodeId"))
            {
                e.has_turn_center = true;
            }

            std::cout << "start: " << e.sequenceId << " " << "center: " << e.centerNodeId << " " << "end: " << e.endNodeId << std::endl;
            
            edges.push_back(e);
        }

        std::sort(edges.begin(), edges.end(), [](const EdgeInfo& a, const EdgeInfo& b)
        {
            return a.sequenceId < b.sequenceId;
        });

        // 4) AMR에 edges 기반 순서로 된 노드 리스트와 원본 edges 전달
        amr->setOrder(ordered_nodes, edges, 15.0);

        std::cout << "[Vda5050Protocol] Order processed with " << ordered_nodes.size()
                  << " nodes & " << edges.size() << " edges assigned to AMR.\n";
    } 
    catch (const std::exception& e) 
    {
        std::cerr << "[Vda5050Protocol] JSON parse error: " << e.what() << std::endl;
    }
}

void Vda5050Protocol::publishStateMessage(IAmr* amr) 
{
    if (!amr_ || !mqtt_client_ || !mqtt_client_->is_connected()) 
    {
        return;
    }

    try {
        std::string state_msg = makeStateMessage(amr);
        // std::cout << "publishStateMessage: " << state_msg << std::endl;

        if (!state_msg.empty()) 
        {
            auto msg = mqtt::make_message(state_topic_, state_msg);
            msg->set_qos(1);
            mqtt_client_->publish(msg);
        }
    } catch (const std::exception& e) {
        std::cerr << "[Vda5050Protocol] State publish exception: " << e.what() << std::endl;
    }
}

void Vda5050Protocol::publishVisualizationMessage(IAmr* amr) 
{
    if(!amr || !mqtt_client_ || !mqtt_client_->is_connected())
    {
        // std::cout << "return" <<std::endl;
        return;
    }

    try
    {
        std::string viz_msg = makeVisualizationMessage(amr);
        // std::cout << "publishVisualizationMessage: " << visualization_topic_ << " " << viz_msg << std::endl;

        if(!viz_msg.empty())
        {
            auto msg = mqtt::make_message(visualization_topic_, viz_msg);
            msg->set_qos(1);
            mqtt_client_->publish(msg);
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << "[Vda5050Protocol] Visualization publish exception: " << e.what() << std::endl;
    }
}

std::string Vda5050Protocol::makeVisualizationMessage(IAmr* amr)
{
    static int visualization_header_id_ = 0;

    if(!amr)
        return {};

    nlohmann::json viz_json;

    // 1. Header 정보
    viz_json["headerId"] = visualization_header_id_++;
    viz_json["timestamp"] = getCurrentTimestampISO8601();
    viz_json["version"] = "0.0.1";
    viz_json["manufacturer"] = "ZenixRobotics";
    viz_json["serialNumber"] = agv_id_;

    // 2. AGV 위치 정보 (x, y, theta)
    double x = 0.0, y = 0.0, theta = 0.0;
    amr->getVcu()->getEstimatedPose(x, y, theta);

    viz_json["agvPosition"] = {
        {"mapId", "map"},
        {"positionInitialized", true},
        {"theta", theta},
        {"x", x},
        {"y", y}
    };

    // 3. 속도 정보 (예: vx, vy)
    // 실제 함수 호출로 속도 업데이트 필요
    viz_json["velocity"] = {
        {"vx", amr->getVcu()->getMotor().getLinearVelocity()},
        {"vy", 0.0}
    };

    return viz_json.dump();
}

std::string Vda5050Protocol::makeStateMessage(IAmr* amr)
{
    if (!amr) 
    {
        return {};
    }
    
    nlohmann::json state_json;
    
    // ==============================================
    // 1. 헤더 정보 (Header Information)
    // ==============================================
    state_json["headerId"] = state_header_id_++;
    state_json["timestamp"] = getCurrentTimestampISO8601();
    state_json["version"] = "0.0.1";
    state_json["manufacturer"] = "ZenixRobotics";
    state_json["serialNumber"] = agv_id_;
    
    // ==============================================
    // 2. AGV 식별 정보 (AGV Identification)
    // ==============================================
    state_json["agvId"] = agv_id_;
    
    // ==============================================
    // 3. 위치 정보 (AGV Position)
    // ==============================================
    double x = 0.0, y = 0.0, theta = 0.0;
    amr->getVcu()->getEstimatedPose(x, y, theta);
    
    state_json["agvPosition"] = {
        {"x", x},
        {"y", y},
        {"theta", theta},
        {"mapId", "default_map"},
        {"mapDescription", "Working area map"},
        {"positionInitialized", true},
        {"localizationScore", 0.95},  // 위치 추정 신뢰도 (0.0-1.0)
        {"deviationRange", 0.1}       // 위치 불확실성 범위 (미터)
    };
    
    // ==============================================
    // 4. 속도 정보 (Velocity)
    // ==============================================
    state_json["velocity"] = {
        {"vx", 0.0},     // amr->getVcu()->getCurrentVelocityX()
        {"vy", 0.0},     // amr->getVcu()->getCurrentVelocityY()  
        {"omega", 0.0}   // amr->getVcu()->getCurrentAngularVelocity()
    };
    
    // ==============================================
    // 5. 배터리 및 Load 정보 (Loads)
    // ==============================================
    nlohmann::json battery_properties = nlohmann::json::array();
    battery_properties.push_back({
        {"key", "batteryPercentage"},
        {"value", std::to_string(amr->getBatteryPercent())}
    });
    battery_properties.push_back({
        {"key", "batteryVoltage"},
        {"value", "24.0"}  // 실제 전압값으로 변경 필요
    });
    battery_properties.push_back({
        {"key", "batteryHealth"},
        {"value", "GOOD"}  // GOOD, FAIR, POOR, CRITICAL
    });
    
    nlohmann::json battery_load = {
        {"loadId", "battery"},
        {"loadType", "battery"},
        {"loadPosition", "internal"},
        {"boundingBoxReference", {
            {"x", 0.0},
            {"y", 0.0},
            {"z", 0.0}
        }},
        {"loadDimensions", {
            {"length", 0.0},
            {"width", 0.0},
            {"height", 0.0}
        }},
        {"weight", 0.0},
        {"loadProperties", battery_properties}
    };
    
    state_json["loads"] = nlohmann::json::array();
    state_json["loads"].push_back(battery_load);
    
    // ==============================================
    // 6. 주행 상태 (Driving)
    // ==============================================
    std::string amr_state = amr->getState();
    bool is_driving = (amr_state.find("MOVING") != std::string::npos ||
                      amr_state.find("DRIVING") != std::string::npos);
    state_json["driving"] = is_driving;
    
    // ==============================================
    // 7. 운영 모드 (Operating Mode)
    // ==============================================
    state_json["operatingMode"] = "AUTOMATIC";  // AUTOMATIC, SEMIAUTOMATIC, MANUAL, SERVICE, TEACHIN
    
    // ==============================================
    // 8. 노드 상태들 (Node States)
    // ==============================================
    nlohmann::json node_states = nlohmann::json::array();
    
    // 현재 노드 정보 추가 (실제 구현에서는 AMR의 현재 경로 정보를 가져와야 함)
    nlohmann::json current_node = {
        {"nodeId", getCurrentNodeId(amr)},  // 실제 현재 노드 ID 가져오기
        {"sequenceId", 0},
        {"nodeDescription", "Current position"},
        {"nodePosition", {
            {"x", x},
            {"y", y},
            {"theta", theta},
            {"allowedDeviationXY", 0.1},
            {"allowedDeviationTheta", 0.1}
        }},
        {"released", true}
    };
    node_states.push_back(current_node);
    
    // 다음 노드들도 추가 (실제로는 현재 주문의 노드들)
    auto upcoming_nodes = getUpcomingNodes(amr);
    for (const auto& node : upcoming_nodes) {
        nlohmann::json node_state = {
            {"nodeId", node.nodeId},
            {"sequenceId", node.sequenceId},
            {"nodeDescription", node.description},
            {"released", false}
        };
        node_states.push_back(node_state);
    }
    
    state_json["nodeStates"] = node_states;
    
    // ==============================================
    // 9. 엣지 상태들 (Edge States)
    // ==============================================
    nlohmann::json edge_states = nlohmann::json::array();
    
    // 현재 주행 중인 엣지가 있다면
    if (is_driving) {
        nlohmann::json current_edge = {
            {"edgeId", getCurrentEdgeId(amr)},  // 실제 현재 엣지 ID
            {"sequenceId", 0},
            {"edgeDescription", "Currently traversing"},
            {"released", false},  // 아직 주행 중
            {"trajectory", nlohmann::json::array()}  // 경로 정보는 필요시 추가
        };
        edge_states.push_back(current_edge);
    }
    
    // 다음 엣지들도 추가
    auto upcoming_edges = getUpcomingEdges(amr);
    for (const auto& edge : upcoming_edges) {
        nlohmann::json edge_state = {
            {"edgeId", edge.edgeId},
            {"sequenceId", edge.sequenceId},
            {"edgeDescription", edge.description},
            {"released", false}
        };
        edge_states.push_back(edge_state);
    }
    
    state_json["edgeStates"] = edge_states;
    
    // ==============================================
    // 10. 액션 상태들 (Action States)
    // ==============================================
    nlohmann::json action_states = nlohmann::json::array();
    
    // 현재 실행 중인 액션들 (예: 충전, 픽업, 드롭오프 등)
    auto current_actions = getCurrentActions(amr);
    for (const auto& action : current_actions) {
        nlohmann::json action_state = {
            {"actionId", action.actionId},
            {"actionType", action.actionType},
            {"actionDescription", action.description},
            {"actionStatus", action.status},  // WAITING, INITIALIZING, RUNNING, PAUSED, FINISHED, FAILED
            {"resultDescription", action.resultDescription}
        };
        
        // 액션 매개변수가 있다면 추가
        if (!action.actionParameters.empty()) {
            action_state["actionParameters"] = action.actionParameters;
        }
        
        action_states.push_back(action_state);
    }
    
    state_json["actionStates"] = action_states;
    
    // ==============================================
    // 11. AGV 상태 (AGV State)
    // ==============================================
    std::string agv_state;
    if (amr_state.find("ERROR") != std::string::npos) {
        agv_state = "ERROR";
    } else if (amr_state.find("MOVING") != std::string::npos ||
               amr_state.find("EXECUTING") != std::string::npos) {
        agv_state = "EXECUTING";
    } else if (amr_state.find("PAUSED") != std::string::npos) {
        agv_state = "PAUSED";
    } else {
        agv_state = "IDLE";
    }
    state_json["agvState"] = agv_state;
    
    // ==============================================
    // 12. 에러 정보 (Errors)
    // ==============================================
    nlohmann::json errors = nlohmann::json::array();
    
    // 배터리 부족 경고
    if (amr->getBatteryPercent() < 20.0) 
    {
        nlohmann::json battery_error = {
            {"errorType", "BATTERY_LOW"},
            {"errorLevel", "WARNING"},
            {"errorDescription", "Battery level is below 20%"},
            {"errorHint", "Consider charging soon"}
        };
        errors.push_back(battery_error);
    }
    
    // AMR의 기타 에러 상태 확인
    if (amr_state.find("ERROR") != std::string::npos) 
    {
        nlohmann::json system_error = {
            {"errorType", "SYSTEM_ERROR"},
            {"errorLevel", "FATAL"},
            {"errorDescription", amr_state},
            {"errorHint", "Check system logs for details"}
        };
        errors.push_back(system_error);
    }
    
    // 추가 에러 확인 (센서, 통신 등)
    auto additional_errors = getSystemErrors(amr);
    for (const auto& error : additional_errors) 
    {
        nlohmann::json error_json = {
            {"errorType", error.errorType},
            {"errorLevel", error.errorLevel},
            {"errorDescription", error.description},
            {"errorHint", error.hint}
        };
        errors.push_back(error_json);
    }
    
    state_json["errors"] = errors;
    
    // ==============================================
    // 13. 정보 메시지 (Information)
    // ==============================================
    nlohmann::json information = nlohmann::json::array();
    
    // 시스템 상태 정보
    nlohmann::json status_info = {
        {"infoType", "STATUS"},
        {"infoLevel", "INFO"},
        {"infoDescription", "System operating normally"}
    };
    information.push_back(status_info);
    
    // 배터리 상태 정보
    nlohmann::json battery_info = {
        {"infoType", "BATTERY_STATUS"},
        {"infoLevel", "INFO"},
        {"infoDescription", "Battery at " + std::to_string(static_cast<int>(amr->getBatteryPercent())) + "%"}
    };
    information.push_back(battery_info);
    
    // 네비게이션 상태 정보
    if (is_driving) {
        nlohmann::json nav_info = {
            {"infoType", "NAVIGATION"},
            {"infoLevel", "INFO"},
            {"infoDescription", "Moving to destination"}
        };
        information.push_back(nav_info);
    }
    
    state_json["information"] = information;
    
    // ==============================================
    // 14. 안전 상태 (Safety State)
    // ==============================================
    state_json["safetyState"] = {
        {"eStop", getEmergencyStopStatus(amr)},        // 비상정지 상태
        {"fieldViolation", getFieldViolationStatus(amr)} // 안전 영역 침범
    };
    
    // ==============================================
    // 15. 배터리 상태 (간소화된 형태도 유지)
    // ==============================================
    // 기존 코드와의 호환성을 위해 기본 배터리 정보도 유지
    state_json["batteryState"] = {
        {"batteryCharge", amr->getBatteryPercent()},
        {"batteryVoltage", 24.0},  // 실제 전압
        {"charging", isCharging(amr)}
    };
    
    // ==============================================
    // 16. 추가 메타데이터
    // ==============================================
    state_json["lastNodeId"] = getLastNodeId(amr);
    state_json["lastNodeSequenceId"] = getLastNodeSequenceId(amr);
    state_json["nodePosition"] = getCurrentNodePosition(amr);
    
    return state_json.dump();
}

// ==============================================
// 헬퍼 메서드들 구현 (vda5050_protocol.cpp에 추가)
// ==============================================

std::string Vda5050Protocol::getCurrentNodeId(IAmr* amr)
{
    return "node_001"; 
}

std::string Vda5050Protocol::getCurrentEdgeId(IAmr* amr)
{
    return "edge_001";  
}

std::vector<NodeInfo> Vda5050Protocol::getUpcomingNodes(IAmr* amr)
{
    std::vector<NodeInfo> nodes;
    return nodes;
}

std::vector<EdgeInfo> Vda5050Protocol::getUpcomingEdges(IAmr* amr)
{
    std::vector<EdgeInfo> edges;
    return edges;
}

std::vector<ActionInfo> Vda5050Protocol::getCurrentActions(IAmr* amr)
{
    std::vector<ActionInfo> actions;
    
    if (isCharging(amr)) 
    {
        ActionInfo charging_action;
        charging_action.actionId = "charge_001";
        charging_action.actionType = "charge";
        charging_action.description = "Battery charging";
        charging_action.status = "RUNNING";
        charging_action.resultDescription = "Charging in progress";
        actions.push_back(charging_action);
    }
    
    return actions;
}

std::vector<ErrorInfo> Vda5050Protocol::getSystemErrors(IAmr* amr)
{
    std::vector<ErrorInfo> errors;
    
    return errors;
}

bool Vda5050Protocol::getEmergencyStopStatus(IAmr* amr)
{
    return false;
}

bool Vda5050Protocol::getFieldViolationStatus(IAmr* amr)
{
    return false;
}

bool Vda5050Protocol::isCharging(IAmr* amr)
{
    std::string state = amr->getState();
    return state.find("CHARGING") != std::string::npos;
}

std::string Vda5050Protocol::getLastNodeId(IAmr* amr)
{
    return "node_000";  
}

int Vda5050Protocol::getLastNodeSequenceId(IAmr* amr)
{
    return 0;  
}

nlohmann::json Vda5050Protocol::getCurrentNodePosition(IAmr* amr)
{
    double x, y, theta;
    amr->getVcu()->getEstimatedPose(x, y, theta);
    
    return {
        {"x", x},
        {"y", y},
        {"theta", theta}
    };
}
