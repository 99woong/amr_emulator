// vda5050_protocol.cpp 2025.11.04, VDA5050 2.1 Schema validation

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
    // 초기 orderId, orderUpdateId 설정
    current_order_id_ = "";
    current_order_update_id_ = 0;
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
}

// std::string Vda5050Protocol::getProtocolType() const
// {
//     return "VDA5050";
// }

void Vda5050Protocol::useDefaultConfig(const std::string& server_address) 
{
    std::cout << "mqtt_server_address: " << server_address << std::endl;
    mqtt_server_uri_ = server_address;
    conn_opts_.set_clean_session(true);
    mqtt_client_ = std::make_unique<mqtt::async_client>(mqtt_server_uri_, agv_id_ + "_client");
    mqtt_callback_ = std::make_shared<Vda5050MqttCallback>(this);
    mqtt_client_->set_callback(*mqtt_callback_);

    state_topic_ = "agv/v2/ZENIXROBOTICS/" + agv_id_ + "/state";
    order_topic_ = "agv/v2/ZENIXROBOTICS/" + agv_id_ + "/order";
    instant_actions_topic = "agv/v2/ZENIXROBOTICS/" + agv_id_ + "/instantActions";
    visualization_topic_ = "agv/v2/ZENIXROBOTICS/" + agv_id_ + "/visualization";
    connection_topic_ = "agv/v2/ZENIXROBOTICS/" + agv_id_ + "/connection";
    factsheet_topic_ = "agv/v2/ZENIXROBOTICS/" + agv_id_ + "/factsheet";
}

void Vda5050Protocol::start() 
{
    try 
    {
        mqtt_client_->connect(conn_opts_)->wait();

        std::string connect_msg = makeConnectMessage();
        auto pubmsg = mqtt::make_message(connection_topic_, connect_msg);
        pubmsg->set_qos(1);
        pubmsg->set_retained(false);

        mqtt_client_->publish(pubmsg);
        mqtt_client_->subscribe(order_topic_, 1)->wait();
        mqtt_client_->subscribe(instant_actions_topic, 1)->wait();
        running_ = true;
        std::cout << "[Vda5050Protocol] MQTT connected & subscribed to: " << order_topic_ << " and " << instant_actions_topic << std::endl;

        publish_thread_ = std::thread([this]()
        {
            while (running_)
            {
                std::this_thread::sleep_for(std::chrono::seconds(1));
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
            // Ignore errors during cleanup
        }
    }    
}

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

// CONNECTION MESSAGE (connection.schema.json)
std::string Vda5050Protocol::makeConnectMessage()
{
    nlohmann::json conn_json;

    // Required fields
    conn_json["headerId"] = connection_header_id_++;
    conn_json["timestamp"] = getCurrentTimestampISO8601();
    conn_json["version"] = "2.1.0";
    conn_json["manufacturer"] = "ZenixRobotics";
    conn_json["serialNumber"] = agv_id_;
    conn_json["connectionState"] = detectConnection();

    std::cout << "make connect message : " << conn_json.dump() << std::endl;

    return conn_json.dump();
}

std::string Vda5050Protocol::detectConnection()
{
    // Todo 실제 연결 상태 감지 로직
    return "ONLINE";
}

// FACTSHEET MESSAGE (factsheet.schema.json)
std::string Vda5050Protocol::makeFactsheetMessage()
{
    nlohmann::json factsheet_json;

    // Required Header Fields
    factsheet_json["headerId"] = factsheet_header_id_++;
    factsheet_json["timestamp"] = getCurrentTimestampISO8601();
    factsheet_json["version"] = "2.1.0";
    factsheet_json["manufacturer"] = "ZenixRobotics";
    factsheet_json["serialNumber"] = agv_id_;

    // typeSpecification (Required)
    nlohmann::json typeSpec;
    typeSpec["seriesName"] = "ZenixAMR";
    typeSpec["agvKinematic"] = "OMNI";
    typeSpec["agvClass"] = "CARRIER";
    typeSpec["maxLoadMass"] = 100.0;
    typeSpec["localizationTypes"] = nlohmann::json::array({"RFID"});
    typeSpec["navigationTypes"] = nlohmann::json::array({"AUTONOMOUS"});
    typeSpec["seriesDescription"] = "Zenix Autonomous Mobile Robot Series";
    factsheet_json["typeSpecification"] = typeSpec;

    // physicalParameters (Required)
    nlohmann::json physicalParams;
    physicalParams["speedMin"] = config_.amr_params.min_speed;
    physicalParams["speedMax"] = config_.amr_params.max_speed;
    physicalParams["accelerationMax"] = config_.amr_params.accelerationMax;
    physicalParams["decelerationMax"] = config_.amr_params.decelerationMax;
    physicalParams["heightMin"] = config_.amr_params.heightMin;
    physicalParams["heightMax"] = config_.amr_params.heightMax;
    physicalParams["width"] = config_.amr_params.width;
    physicalParams["length"] = config_.amr_params.length;
    factsheet_json["physicalParameters"] = physicalParams;

    // protocolLimits (Required)
    nlohmann::json protocolLimits;
    nlohmann::json maxStringLens;
    maxStringLens["msgLen"] = 10000;
    maxStringLens["topicSerialLen"] = 256;
    maxStringLens["topicElemLen"] = 256;
    maxStringLens["idLen"] = 256;
    maxStringLens["idNumericalOnly"] = false;
    maxStringLens["enumLen"] = 256;
    maxStringLens["loadIdLen"] = 256;
    protocolLimits["maxStringLens"] = maxStringLens;
    protocolLimits["maxArrayLens"] = nlohmann::json::object();
    
    nlohmann::json timing;
    timing["minOrderInterval"] = 1.0;
    timing["minStateInterval"] = 1.0;
    timing["defaultStateInterval"] = 30.0;
    timing["visualizationInterval"] = 0.1;
    protocolLimits["timing"] = timing;
    factsheet_json["protocolLimits"] = protocolLimits;

    // protocolFeatures (Required)
    nlohmann::json protocolFeatures;
    nlohmann::json optionalParameters = nlohmann::json::array();
    optionalParameters.push_back({
        {"parameter", "trajectory"},
        {"support", "SUPPORTED"},
        {"description", "NURBS trajectory support"}
    });
    protocolFeatures["optionalParameters"] = optionalParameters;
    
    nlohmann::json agvActions = nlohmann::json::array();
    nlohmann::json pickAction;
    pickAction["actionType"] = "pick";
    pickAction["actionDescription"] = "Pick up a load";
    pickAction["actionScopes"] = nlohmann::json::array({"NODE"});
    
    nlohmann::json pickParams = nlohmann::json::array();
    pickParams.push_back({
        {"key", "stationType"},
        {"valueDataType", "STRING"},
        {"description", "Type of station"},
        {"isOptional", false}
    });
    pickAction["actionParameters"] = pickParams;
    pickAction["resultDescription"] = "Load picked successfully";
    agvActions.push_back(pickAction);
    protocolFeatures["agvActions"] = agvActions;
    factsheet_json["protocolFeatures"] = protocolFeatures;

    // agvGeometry (Required)
    nlohmann::json agvGeometry;
    nlohmann::json envelopes2d = nlohmann::json::array();
    nlohmann::json envelope;
    envelope["set"] = "default";
    nlohmann::json polygonPoints = nlohmann::json::array();
    polygonPoints.push_back({{"x", -0.5}, {"y", -0.4}});
    polygonPoints.push_back({{"x", 0.5}, {"y", -0.4}});
    polygonPoints.push_back({{"x", 0.5}, {"y", 0.4}});
    polygonPoints.push_back({{"x", -0.5}, {"y", 0.4}});
    envelope["polygonPoints"] = polygonPoints;
    envelope["description"] = "Base footprint";
    envelopes2d.push_back(envelope);
    agvGeometry["envelopes2d"] = envelopes2d;
    agvGeometry["envelopes3d"] = nlohmann::json::array();
    factsheet_json["agvGeometry"] = agvGeometry;

    // loadSpecification (Required)
    nlohmann::json loadSpec;
    loadSpec["loadPositions"] = nlohmann::json::array({"top"});
    
    nlohmann::json loadSets = nlohmann::json::array();
    nlohmann::json loadSet;
    loadSet["setName"] = "europalette";
    loadSet["loadType"] = "EPAL";
    loadSet["loadPositions"] = nlohmann::json::array({"top"});
    loadSet["boundingBoxReference"] = {
        {"x", 0.0},
        {"y", 0.0},
        {"z", 0.0},
        {"theta", 0}
    };
    loadSet["loadDimensions"] = {
        {"length", 1.2},
        {"width", 0.8},
        {"height", 0.15}
    };
    loadSet["maxWeight"] = 100.0;
    loadSet["minLoadhandlingHeight"] = 0.0;
    loadSet["maxLoadhandlingHeight"] = 1.5;
    loadSets.push_back(loadSet);
    loadSpec["loadSets"] = loadSets;
    factsheet_json["loadSpecification"] = loadSpec;

    // std::cout << "publish factsheet : " << factsheet_json.dump() << std::endl;

    return factsheet_json.dump();
}

// INSTANT ACTIONS
void Vda5050Protocol::handleInstantAction(const nlohmann::json& instant_action_json)
{
    try
    {
        std::string actionType = instant_action_json.at("actionType").get<std::string>();
        std::string actionId = instant_action_json.at("actionId").get<std::string>();
        std::string blockingType = instant_action_json.at("blockingType").get<std::string>();        

        if (actionType == "factsheetRequest")
        {
            // Factsheet 메시지 생성 (VDA5050 2.1 스키마 준수)
            std::string factsheet_msg = makeFactsheetMessage();
            auto msg = mqtt::make_message(factsheet_topic_, factsheet_msg);
            msg->set_qos(1);

            if (mqtt_client_ && mqtt_client_->is_connected())
            {
                mqtt_client_->publish(msg);
                std::cout << "[Vda5050Protocol] Factsheet published in response to request: " << actionId << std::endl;
            }
            else
            {
                std::cerr << "[Vda5050Protocol] Cannot publish factsheet: MQTT client disconnected\n";
            }
        }
        else if (actionType == "cancelOrder")
        {
            std::cout << "[Vda5050Protocol] Cancel order requested. ActionId: " << actionId << std::endl;
            // ... (실제 취소 로직 호출) ...
        }
        else
        {
            std::cout << "[Vda5050Protocol] Unknown instant action: " << actionType << ". ActionId: " << actionId << std::endl;
        }
    }
    catch (const nlohmann::json::exception& e)
    {
        std::cerr << "[Vda5050Protocol] Instant Action Schema/Parsing Error (Check actionType, actionId, blockingType): " << e.what() << std::endl;
        // FMS로 에러 보고 로직 추가 필요
    }
    catch (const std::exception& e)
    {
        std::cerr << "[Vda5050Protocol] Exception handling instant action: " << e.what() << std::endl;
    }    
}

void Vda5050Protocol::handleMessage(const std::string& msg, IAmr* amr)
{
    std::cout << "[Vda5050Protocol] handleMessage called" << std::endl;

    if (msg.empty() || !amr) 
    {
        std::cerr << "[Vda5050Protocol] Empty message or null AMR pointer\n";
        return;
    }

    try 
    {
        std::cout << "handleMessage : " << msg << std::endl;

        auto json_msg = nlohmann::json::parse(msg);

        // InstantActions 메시지 처리
        if (json_msg.contains("actions") && json_msg["actions"].is_array())
        {

            std::cout << "InstantActions received. Processing " << json_msg["actions"].size() << " action(s)." << std::endl;      
                
            // 모든 액션 순회 처리
            for (const auto& action : json_msg["actions"])
            {
                handleInstantAction(action);
            }
            return;
        }

        // Order 메시지 처리
        if (!json_msg.contains("nodes") || !json_msg.contains("edges")) 
        {
            std::cerr << "[Vda5050Protocol] Order missing nodes or edges\n";
            return;
        }

        if (json_msg.contains("orderId"))
        {
            current_order_id_ = json_msg["orderId"].get<std::string>();
        }
        if (json_msg.contains("orderUpdateId"))
        {
            current_order_update_id_ = json_msg["orderUpdateId"].get<int>();
        }
        if (json_msg.contains("zoneSetId") && !json_msg["zoneSetId"].is_null())
        {
            current_zone_set_id_ = json_msg["zoneSetId"].get<std::string>();
        }
        else
        {
            current_zone_set_id_ = "";
        }

        std::unordered_map<std::string, NodeInfo> node_map;
        std::vector<NodeInfo> ordered_nodes;

        for (const auto& node : json_msg["nodes"])
        {
            NodeInfo n;
            n.nodeId = node.value("nodeId", "");
            n.sequenceId = node.value("sequenceId", 0);
            n.released = node.value("released", false);
            
            if (node.contains("nodePosition") && !node["nodePosition"].is_null())
            {
                n.x = node["nodePosition"].value("x", 0.0);
                n.y = node["nodePosition"].value("y", 0.0);
            }
            
            node_map[n.nodeId] = n;
            ordered_nodes.push_back(n);
        }

        std::vector<EdgeInfo> edges;
        for (const auto& edge : json_msg["edges"])
        {
            EdgeInfo e;
            e.edgeId = edge.value("edgeId", "");
            e.sequenceId = edge.value("sequenceId", 0);
            e.startNodeId = edge.value("startNodeId", "");
            e.endNodeId = edge.value("endNodeId", "");
            e.released = edge.value("released", false);
            e.centerNodeId = edge.value("centerNodeId", "");            
            
            if (edge.contains("maxSpeed") && !edge["maxSpeed"].is_null())
            {
                e.maxSpeed = edge["maxSpeed"].get<double>();
            }
            
            if (edge.contains("centerNodeId"))
            {
                e.has_turn_center = true;
            }            
            
            edges.push_back(e);
        }

        std::sort(edges.begin(), edges.end(), [](const EdgeInfo& a, const EdgeInfo& b)
        {
            return a.sequenceId < b.sequenceId;
        });

        amr->setOrder(ordered_nodes, edges, 15.0);

        std::cout << "[Vda5050Protocol] Order processed: " << ordered_nodes.size()
                  << " nodes, " << edges.size() << " edges\n";
    } 
    catch (const nlohmann::json::exception& e)
    {
        std::cerr << "[Vda5050Protocol] JSON Parsing Error: " << e.what() << "\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << "[Vda5050Protocol] Unknown Error in handleMessage: " << e.what() << "\n";
    }
}

// VISUALIZATION MESSAGE
void Vda5050Protocol::publishVisualizationMessage(IAmr* amr) 
{
    if(!amr || !mqtt_client_ || !mqtt_client_->is_connected())
    {
        return;
    }

    try
    {
        std::string viz_msg = makeVisualizationMessage(amr);
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

    viz_json["headerId"] = visualization_header_id_++;
    viz_json["timestamp"] = getCurrentTimestampISO8601();
    viz_json["version"] = "2.1.0";
    viz_json["manufacturer"] = "ZenixRobotics";
    viz_json["serialNumber"] = agv_id_;

    double x = 0.0, y = 0.0, theta = 0.0;
    amr->getVcu()->getEstimatedPose(x, y, theta);

    viz_json["agvPosition"] = {
        {"x", x},
        {"y", y},
        {"theta", theta},
        {"mapId", "default_map"},
        {"positionInitialized", true}
    };
    viz_json["agvPosition"]["mapDescription"] = nullptr;
    viz_json["agvPosition"]["localizationScore"] = 0.95;
    viz_json["agvPosition"]["deviationRange"] = nullptr;

    viz_json["velocity"] = {
        {"vx", amr->getVcu()->getMotor().getLinearVelocity()},
        {"vy", nullptr},
        {"omega", nullptr}
    };

    return viz_json.dump();
}

// STATE MESSAGE
void Vda5050Protocol::publishStateMessage(IAmr* amr) 
{
    if (!amr_ || !mqtt_client_ || !mqtt_client_->is_connected()) 
    {
        return;
    }

    try 
    {
        std::string state_msg = makeStateMessage(amr);
        // std::cout << "msg : " << state_msg << std::endl;
        if (!state_msg.empty()) 
        {
            auto msg = mqtt::make_message(state_topic_, state_msg);
            msg->set_qos(1);
            mqtt_client_->publish(msg);
        }
    } 
    catch (const std::exception& e) 
    {
        std::cerr << "[Vda5050Protocol] State publish exception: " << e.what() << std::endl;
    }
}


// vda5050_protocol.cpp - makeStateMessage 함수 수정

std::string Vda5050Protocol::makeStateMessage(IAmr* amr)
{
    if (!amr) 
    {
        return {};
    }
    
    nlohmann::json state_json;
    
    // 1. Required Header Fields
    state_json["headerId"] = state_header_id_++;
    state_json["timestamp"] = getCurrentTimestampISO8601();
    state_json["version"] = "2.1.0";
    state_json["manufacturer"] = "ZenixRobotics";
    state_json["serialNumber"] = agv_id_;
    
    // 2. Required Order Information
    state_json["orderId"] = current_order_id_.empty() ? "" : current_order_id_;
    state_json["orderUpdateId"] = current_order_update_id_;
    
    if(!current_zone_set_id_.empty())
    {
        state_json["zoneSetId"] = current_zone_set_id_;
    }
    
    // 3. Required Position Information
    state_json["lastNodeId"] = getLastNodeId(amr);
    state_json["lastNodeSequenceId"] = getLastNodeSequenceId(amr);
    
    // 4. Required Driving State
    std::string amr_state = amr->getState();
    bool is_driving = (amr_state.find("MOVING") != std::string::npos ||
                      amr_state.find("DRIVING") != std::string::npos);
    state_json["driving"] = is_driving;
    
    state_json["paused"] = false; 
    state_json["newBaseRequest"] = false; 
    state_json["distanceSinceLastNode"] = getDistanceSinceLastNode(amr);
    
    // 5. Required Operating Mode
    state_json["operatingMode"] = "AUTOMATIC";
    
    // 6. Required Node States Array - 실시간 업데이트
    nlohmann::json node_states = nlohmann::json::array();
    
    // AMR의 현재 주행 정보 가져오기
    auto current_nodes = amr->getCurrentNodes();  // AMR에서 현재 처리 중인 노드들
    auto completed_nodes = amr->getCompletedNodes();  // 완료된 노드들
    
    // 완료된 노드들 추가
    for (const auto& node : completed_nodes)
    {
        nlohmann::json node_state = {
            {"nodeId", node.nodeId},
            {"sequenceId", node.sequenceId},
            {"released", true}
        };
        
        if (node.hasNodePosition)
        {
            node_state["nodePosition"] = {
                {"x", node.nodePosition.x},
                {"y", node.nodePosition.y},
                {"mapId", node.nodePosition.mapId},
                {"theta", node.nodePosition.theta},
                {"positionInitialized", true}
            };
        }
        
        node_states.push_back(node_state);
    }
    
    // 현재 처리 중인 노드들 추가
    for (const auto& node : current_nodes)
    {
        nlohmann::json node_state = {
            {"nodeId", node.nodeId},
            {"sequenceId", node.sequenceId},
            {"released", node.released}
        };
        
        if (node.hasNodePosition)
        {
            node_state["nodePosition"] = {
                {"x", node.nodePosition.x},
                {"y", node.nodePosition.y},
                {"mapId", node.nodePosition.mapId},
                {"theta", node.nodePosition.theta},
                {"positionInitialized", true}
            };
        }
        
        node_states.push_back(node_state);
    }
    
    state_json["nodeStates"] = node_states;
    
    // 7. Required Edge States Array - 실시간 업데이트
    nlohmann::json edge_states = nlohmann::json::array();
    
    auto current_edges = amr->getCurrentEdges();  // 현재 주행 중인 엣지들
    auto completed_edges = amr->getCompletedEdges();  // 완료된 엣지들
    
    // 완료된 엣지들 추가
    for (const auto& edge : completed_edges)
    {
        nlohmann::json edge_state = {
            {"edgeId", edge.edgeId},
            {"sequenceId", edge.sequenceId},
            {"released", true}
        };
        edge_states.push_back(edge_state);
    }
    
    // 현재 주행 중인 엣지들 추가
    for (const auto& edge : current_edges)
    {
        nlohmann::json edge_state = {
            {"edgeId", edge.edgeId},
            {"sequenceId", edge.sequenceId},
            {"released", edge.released}
        };
        
        // Trajectory 정보가 있으면 포함
        if (edge.hasTrajectory)
        {
            nlohmann::json trajectory = {
                {"degree", edge.trajectory.degree},
                {"knotVector", edge.trajectory.knotVector}
            };
            
            nlohmann::json control_points = nlohmann::json::array();
            for (const auto& cp : edge.trajectory.controlPoints)
            {
                nlohmann::json point = {
                    {"x", cp.x},
                    {"y", cp.y}
                };
                if (cp.hasWeight)
                {
                    point["weight"] = cp.weight;
                }
                control_points.push_back(point);
            }
            trajectory["controlPoints"] = control_points;
            edge_state["trajectory"] = trajectory;
        }
        
        edge_states.push_back(edge_state);
    }
    
    state_json["edgeStates"] = edge_states;
    
    // 8. AGV Position
    double x = 0.0, y = 0.0, theta = 0.0;
    amr->getVcu()->getEstimatedPose(x, y, theta);
    
    state_json["agvPosition"] = {
        {"x", x},
        {"y", y},
        {"theta", theta},
        {"mapId", "default_map"},
        {"positionInitialized", true}
    };
    state_json["agvPosition"]["localizationScore"] = 0.95;
    
    // 9. Velocity
    state_json["velocity"] = {
        {"vx", amr->getVcu()->getMotor().getLinearVelocity()},
        {"vy", 0.0},
        {"omega", amr->getVcu()->getMotor().getAngularVelocity()}
    };
    
    // 10. Loads (nullable array)
    state_json["loads"] = nlohmann::json::array();
    
    // 11. Required Action States Array
    nlohmann::json action_states = nlohmann::json::array();
    
    auto current_actions = getCurrentActions(amr);
    for (const auto& action : current_actions) 
    {
        nlohmann::json action_state = {
            {"actionId", action.actionId},
            {"actionStatus", action.status}
        };
        action_states.push_back(action_state);
    }
    
    state_json["actionStates"] = action_states;
    
    // 12. Required Battery State
    state_json["batteryState"] = {
        {"batteryCharge", amr->getBatteryPercent()},
        {"charging", isCharging(amr)}
    };
    
    state_json["batteryState"]["batteryVoltage"] = 0.0;
    state_json["batteryState"]["batteryHealth"] = 100.0;
    state_json["batteryState"]["reach"] = 0.0;
    
    // 13. Required Errors Array
    nlohmann::json errors = nlohmann::json::array();
    
    if (amr->getBatteryPercent() < 20.0) 
    {
        nlohmann::json battery_error = {
            {"errorType", "BATTERY_LOW"},
            {"errorLevel", "WARNING"}
        };
        battery_error["errorDescription"] = "Battery level is below 20%";
        errors.push_back(battery_error);
    }
    
    if (amr_state.find("ERROR") != std::string::npos) 
    {
        nlohmann::json system_error = {
            {"errorType", "SYSTEM_ERROR"},
            {"errorLevel", "FATAL"}
        };
        system_error["errorDescription"] = amr_state;
        errors.push_back(system_error);
    }
    
    state_json["errors"] = errors;
    
    // 14. Information Array
    state_json["information"] = nlohmann::json::array();
    
    // 15. Required Safety State
    std::string estop_status = getEmergencyStopStatus(amr) ? "MANUAL" : "NONE";
    state_json["safetyState"] = {
        {"eStop", estop_status},
        {"fieldViolation", getFieldViolationStatus(amr)}
    };
    
    return state_json.dump();
}

// Helper Methods
std::string Vda5050Protocol::getCurrentNodeId(IAmr* amr)
{
    auto current_nodes = amr->getCurrentNodes();
    if (!current_nodes.empty())
    {
        return current_nodes[0].nodeId;
    }
    return "";
}

std::string Vda5050Protocol::getCurrentEdgeId(IAmr* amr)
{
    auto current_edges = amr->getCurrentEdges();
    if (!current_edges.empty())
    {
        return current_edges[0].edgeId;
    }
    return "";
}

std::vector<NodeInfo> Vda5050Protocol::getUpcomingNodes(IAmr* amr)
{
    return amr->getCurrentNodes();
}

std::vector<EdgeInfo> Vda5050Protocol::getUpcomingEdges(IAmr* amr)
{
    return amr->getCurrentEdges();
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
    
    // 현재 노드/엣지의 액션들도 포함 가능
    auto current_nodes = amr->getCurrentNodes();
    for (const auto& node : current_nodes)
    {
        for (const auto& action : node.actions)
        {
            ActionInfo action_info;
            action_info.actionId = action.actionId;
            action_info.actionType = action.actionType;
            action_info.description = action.actionDescription;
            action_info.status = "RUNNING";  // 또는 실제 상태
            actions.push_back(action_info);
        }
    }
    
    auto current_edges = amr->getCurrentEdges();
    for (const auto& edge : current_edges)
    {
        for (const auto& action : edge.actions)
        {
            ActionInfo action_info;
            action_info.actionId = action.actionId;
            action_info.actionType = action.actionType;
            action_info.description = action.actionDescription;
            action_info.status = "RUNNING";  // 또는 실제 상태
            actions.push_back(action_info);
        }
    }
    
    return actions;
}

std::vector<ErrorInfo> Vda5050Protocol::getSystemErrors(IAmr* amr)
{
    std::vector<ErrorInfo> errors;
    
    // 배터리 경고
    if (amr->getBatteryPercent() < 20.0)
    {
        ErrorInfo error;
        error.errorType = "BATTERY_LOW";
        error.errorLevel = "WARNING";
        error.description = "Battery level is below 20%";
        errors.push_back(error);
    }
    
    // AMR 상태 기반 에러
    std::string state = amr->getState();
    if (state.find("ERROR") != std::string::npos)
    {
        ErrorInfo error;
        error.errorType = "SYSTEM_ERROR";
        error.errorLevel = "FATAL";
        error.description = state;
        errors.push_back(error);
    }
    
    return errors;
}

bool Vda5050Protocol::getEmergencyStopStatus(IAmr* amr)
{
    // TODO: 실제 E-Stop 상태 확인 로직 구현
    return false;
}

bool Vda5050Protocol::getFieldViolationStatus(IAmr* amr)
{
    // TODO: 안전영역 침범확인 로직 구현
    return false;
}

bool Vda5050Protocol::isCharging(IAmr* amr)
{
    std::string state = amr->getState();
    return state.find("CHARGING") != std::string::npos;
}

std::string Vda5050Protocol::getLastNodeId(IAmr* amr)
{
    return amr->getLastNodeId();
}

int Vda5050Protocol::getLastNodeSequenceId(IAmr* amr)
{
    return amr->getLastNodeSequenceId();
}


nlohmann::json Vda5050Protocol::getCurrentNodePosition(IAmr* amr)
{
    double x, y, theta;
    amr->getVcu()->getEstimatedPose(x, y, theta);
    
    return {
        {"x", x},
        {"y", y},
        {"theta", theta},
        {"mapId", "default_map"},
        {"positionInitialized", true}
    };
}

double Vda5050Protocol::getDistanceSinceLastNode(IAmr* amr)
{
    if (!amr)
        return 0.0;
    
    double current_x = 0.0, current_y = 0.0, current_theta = 0.0;
    amr->getVcu()->getEstimatedPose(current_x, current_y, current_theta);
    
    auto completed_nodes = amr->getCompletedNodes();
    if (completed_nodes.empty())
        return 0.0;
    
    const auto& last_node = completed_nodes.back();
    if (!last_node.hasNodePosition)
    {
        // nodePosition이 없으면 x, y 사용
        double dx = current_x - last_node.x;
        double dy = current_y - last_node.y;
        return std::hypot(dx, dy);
    }
    
    double dx = current_x - last_node.nodePosition.x;
    double dy = current_y - last_node.nodePosition.y;
    
    return std::hypot(dx, dy);
}

