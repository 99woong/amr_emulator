// vda5050_protocol.cpp - VDA5050 2.1 Schema validation

#include "vda5050_protocol.h"
#include "node_edge_info.h"
#include "iamr.h"
#include "ivcu.h"
#include <iostream>
#include <nlohmann/json.hpp>
#include <vda5050++/config.h>
#include <algorithm> 
#include <unordered_set>


void Vda5050Protocol::Vda5050MqttCallback::message_arrived(mqtt::const_message_ptr msg) 
{
    std::cout << "[MQTT] Message arrived on topic: " << msg->get_topic() << std::endl;
    if (msg->get_topic() == proto_->order_topic_ || msg->get_topic() == proto_->instant_actions_topic) 
    {
        proto_->handleMessage(msg->to_string(), proto_->amr_);
    }
}

Vda5050Protocol::Vda5050Protocol(const AmrConfig& config) 
    : running_(false), 
      config_(config),
      amr_(nullptr),
      state_header_id_(0),
      factsheet_header_id_(0),
      connection_header_id_(0),
      current_order_id_(""),
      current_order_update_id_(0),
      current_zone_set_id_(""),
      order_active_(false),
      has_order_rejection_error_(false),      
      order_rejection_error_type_(""),        
      order_rejection_error_description_("") 
{
    std::cout << "[Vda5050Protocol] Initialized" << std::endl;
}

Vda5050Protocol::~Vda5050Protocol() 
{
    stop();
}

void Vda5050Protocol::setAmr(IAmr* amr) 
{
    amr_ = amr;
    std::cout << "[Vda5050Protocol] AMR instance set" << std::endl;
}

void Vda5050Protocol::setAgvId(const std::string& agv_id) 
{
    agv_id_ = agv_id;
    std::cout << "[Vda5050Protocol] AGV ID set to: " << agv_id_ << std::endl;
}

void Vda5050Protocol::useDefaultConfig(const std::string& server_address) 
{
    std::cout << "[Vda5050Protocol] Configuring MQTT server: " << server_address << std::endl;
    mqtt_server_uri_ = server_address;
    conn_opts_.set_clean_session(true);
    mqtt_client_ = std::make_unique<mqtt::async_client>(mqtt_server_uri_, agv_id_ + "_client");
    mqtt_callback_ = std::make_shared<Vda5050MqttCallback>(this);
    mqtt_client_->set_callback(*mqtt_callback_);

    // VDA5050 topic structure: agv/v2/{manufacturer}/{agv_id}/{topic}
    state_topic_ = "agv/v2/ZENIXROBOTICS/" + agv_id_ + "/state";
    order_topic_ = "agv/v2/ZENIXROBOTICS/" + agv_id_ + "/order";
    instant_actions_topic = "agv/v2/ZENIXROBOTICS/" + agv_id_ + "/instantActions";
    visualization_topic_ = "agv/v2/ZENIXROBOTICS/" + agv_id_ + "/visualization";
    connection_topic_ = "agv/v2/ZENIXROBOTICS/" + agv_id_ + "/connection";
    factsheet_topic_ = "agv/v2/ZENIXROBOTICS/" + agv_id_ + "/factsheet";
    
    std::cout << "[Vda5050Protocol] Topics configured:" << std::endl;
    std::cout << "  - State: " << state_topic_ << std::endl;
    std::cout << "  - Order: " << order_topic_ << std::endl;
    std::cout << "  - Instant Actions: " << instant_actions_topic << std::endl;
}

void Vda5050Protocol::start() 
{
    try 
    {
        std::cout << "[Vda5050Protocol] Connecting to MQTT broker..." << std::endl;
        mqtt_client_->connect(conn_opts_)->wait();

        // Publish connection message
        std::string connect_msg = makeConnectMessage();
        auto pubmsg = mqtt::make_message(connection_topic_, connect_msg);
        pubmsg->set_qos(1);
        pubmsg->set_retained(false);
        mqtt_client_->publish(pubmsg);
        
        // Subscribe to order and instant actions topics
        mqtt_client_->subscribe(order_topic_, 1)->wait();
        mqtt_client_->subscribe(instant_actions_topic, 1)->wait();
        
        running_ = true;
        std::cout << "[Vda5050Protocol] MQTT connected & subscribed to: " 
                  << order_topic_ << " and " << instant_actions_topic << std::endl;

        // Start background thread (for periodic tasks if needed)
        publish_thread_ = std::thread([this]()
        {
            while (running_)
            {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                // Periodic tasks can be added here
                if (amr_)
                {
                    // 주기적으로 오더 완료 체크
                    checkOrderCompletion(amr_);
                }                
            }
        });        
    } 
    catch (const mqtt::exception& e) 
    {
        std::cerr << "[Vda5050Protocol] MQTT connection failed: " << e.what() << std::endl;
        throw;
    }
}

void Vda5050Protocol::stop() 
{
    std::cout << "[Vda5050Protocol] Stopping protocol..." << std::endl;
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
            std::cout << "[Vda5050Protocol] MQTT disconnected" << std::endl;
        } 
        catch (const std::exception& e) 
        {
            std::cerr << "[Vda5050Protocol] Error during disconnect: " << e.what() << std::endl;
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

    // Required fields per VDA5050 2.1 connection schema
    conn_json["headerId"] = connection_header_id_++;
    conn_json["timestamp"] = getCurrentTimestampISO8601();
    conn_json["version"] = "2.1.0";
    conn_json["manufacturer"] = "ZenixRobotics";
    conn_json["serialNumber"] = agv_id_;
    conn_json["connectionState"] = detectConnection();

    std::cout << "[Vda5050Protocol] Connection message: " << conn_json.dump() << std::endl;
    return conn_json.dump();
}

std::string Vda5050Protocol::detectConnection()
{
    // TODO: Implement actual connection state detection
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

    return factsheet_json.dump();
}

// INSTANT ACTIONS HANDLER
void Vda5050Protocol::handleInstantAction(const nlohmann::json& instant_action_json)
{
    try
    {
        std::string actionType = instant_action_json.at("actionType").get<std::string>();
        std::string actionId = instant_action_json.at("actionId").get<std::string>();
        std::string blockingType = instant_action_json.at("blockingType").get<std::string>();        

        std::cout << "[Vda5050Protocol] Processing instant action: " << actionType 
                  << " (ID: " << actionId << ")" << std::endl;

        if (actionType == "factsheetRequest")
        {
            // Generate and publish factsheet message
            std::string factsheet_msg = makeFactsheetMessage();
            auto msg = mqtt::make_message(factsheet_topic_, factsheet_msg);
            msg->set_qos(1);

            if (mqtt_client_ && mqtt_client_->is_connected())
            {
                mqtt_client_->publish(msg);
                std::cout << "[Vda5050Protocol] Factsheet published in response to request: " 
                          << actionId << std::endl;
            }
            else
            {
                std::cerr << "[Vda5050Protocol] Cannot publish factsheet: MQTT client disconnected" << std::endl;
            }
        }
        else if (actionType == "cancelOrder")
        {
            std::cout << "[Vda5050Protocol] Cancel order requested. ActionId: " << actionId << std::endl;
            
            // Cancel order and reset to IDLE state
            current_order_id_ = "";
            current_order_update_id_ = 0;
            current_zone_set_id_ = "";
            order_active_ = false;
            
            // Clear order data
            received_nodes_.clear();
            received_edges_.clear();
            
            // Send cancel command to AMR
            if (amr_)
            {
                amr_->cancelOrder();
            }
            
            // Publish state immediately (with empty nodeStates and edgeStates)
            publishStateMessage(amr_);
            std::cout << "[Vda5050Protocol] Order cancelled, switched to IDLE state" << std::endl;
        }
        else
        {
            std::cout << "[Vda5050Protocol] Unknown instant action: " << actionType 
                      << ". ActionId: " << actionId << std::endl;
        }
    }
    catch (const nlohmann::json::exception& e)
    {
        std::cerr << "[Vda5050Protocol] Instant Action Schema/Parsing Error: " << e.what() << std::endl;
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
        std::cerr << "[Vda5050Protocol] Empty message or null AMR pointer" << std::endl;
        return;
    }

    try 
    {
        std::cout << "[Vda5050Protocol] Received message: " << msg << std::endl;

        auto json_msg = nlohmann::json::parse(msg);

        // Check if this is an InstantActions message
        if (json_msg.contains("actions") && json_msg["actions"].is_array())
        {
            std::cout << "[Vda5050Protocol] InstantActions received. Processing " 
                      << json_msg["actions"].size() << " action(s)." << std::endl;      
                
            for (const auto& action : json_msg["actions"])
            {
                handleInstantAction(action);
            }
            return;
        }

        // Check if this is an Order message
        if (!json_msg.contains("nodes") || !json_msg.contains("edges")) 
        {
            std::cerr << "[Vda5050Protocol] Order missing required nodes or edges fields" << std::endl;
            return;
        }

        // Extract order information
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

        std::cout << "[Vda5050Protocol] Processing order: " << current_order_id_ 
                  << " (updateId: " << current_order_update_id_ << ")" << std::endl;

        // Clear storage
        received_nodes_.clear();
        received_edges_.clear();
        ordered_nodes_.clear();

        std::unordered_map<std::string, NodeInfo> node_map;

        // Parse nodes and store in map
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
                n.hasNodePosition = true;
                n.nodePosition.x = n.x;
                n.nodePosition.y = n.y;
                n.nodePosition.mapId = node["nodePosition"].value("mapId", "default_map");
                n.nodePosition.theta = node["nodePosition"].value("theta", 0.0);
                n.nodePosition.positionInitialized = true;
            }
            
            if (node.contains("actions") && node["actions"].is_array())
            {
                for (const auto& action_json : node["actions"])
                {
                    Action action;
                    action.actionId = action_json.value("actionId", "");
                    action.actionType = action_json.value("actionType", "");
                    action.actionDescription = action_json.value("actionDescription", "");
                    action.blockingType = action_json.value("blockingType", "HARD");
                    
                    if (action_json.contains("actionParameters") && action_json["actionParameters"].is_array())
                    {
                        for (const auto& param_json : action_json["actionParameters"])
                        {
                            ActionParameter param;
                            param.key = param_json.value("key", "");
                            param.value = param_json.value("value", nlohmann::json());
                            action.actionParameters.push_back(param);
                        }
                    }
                    
                    n.actions.push_back(action);
                }
            }
            
            node_map[n.nodeId] = n;
            received_nodes_.push_back(n);
        }

        // Parse edges
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
            
            if (!e.centerNodeId.empty())
            {
                e.has_turn_center = true;
            }
            
            if (edge.contains("actions") && edge["actions"].is_array())
            {
                for (const auto& action_json : edge["actions"])
                {
                    Action action;
                    action.actionId = action_json.value("actionId", "");
                    action.actionType = action_json.value("actionType", "");
                    action.actionDescription = action_json.value("actionDescription", "");
                    action.blockingType = action_json.value("blockingType", "HARD");
                    
                    if (action_json.contains("actionParameters") && action_json["actionParameters"].is_array())
                    {
                        for (const auto& param_json : action_json["actionParameters"])
                        {
                            ActionParameter param;
                            param.key = param_json.value("key", "");
                            param.value = param_json.value("value", nlohmann::json());
                            action.actionParameters.push_back(param);
                        }
                    }
                    
                    e.actions.push_back(action);
                }
            }            
            
            // Parse trajectory if present
            if (edge.contains("trajectory") && !edge["trajectory"].is_null())
            {
                e.hasTrajectory = true;
                e.trajectory.degree = edge["trajectory"].value("degree", 3);
                e.trajectory.knotVector = edge["trajectory"].value("knotVector", std::vector<double>());
                
                if (edge["trajectory"].contains("controlPoints") && edge["trajectory"]["controlPoints"].is_array())
                {
                    for (const auto& cp_json : edge["trajectory"]["controlPoints"])
                    {
                        ControlPoint cp;
                        cp.x = cp_json.value("x", 0.0);
                        cp.y = cp_json.value("y", 0.0);
                        if (cp_json.contains("weight") && !cp_json["weight"].is_null())
                        {
                            cp.weight = cp_json["weight"].get<double>();
                            cp.hasWeight = true;
                        }
                        e.trajectory.controlPoints.push_back(cp);
                    }
                }
            }
            
            edges.push_back(e);
            received_edges_.push_back(e);
        }

        // Sort edges by sequence ID
        std::sort(edges.begin(), edges.end(), [](const EdgeInfo& a, const EdgeInfo& b)
        {
            return a.sequenceId < b.sequenceId;
        });

        if (edges.empty())
        {
            std::cerr << "[Vda5050Protocol] Order has no edges" << std::endl;
            publishOrderRejectionError("ORDER_NO_EDGES", "Order contains no edges for driving");
            return;
        }
        
        // 에지 순서로 노드 리스트 생성 (순환 경로 지원)
        std::string first_node_id = edges[0].startNodeId;
        
        if (node_map.find(first_node_id) == node_map.end())
        {
            std::cerr << "[Vda5050Protocol] Start node not found: " << first_node_id << std::endl;
            publishOrderRejectionError("START_NODE_NOT_FOUND", "Start node " + first_node_id + " not found in order");
            return;
        }
        
        const NodeInfo& start_node = node_map[first_node_id];
        
        // 에지 순서대로 endNode만 추가 (startNode 제외)
        // 순환 경로의 경우, 마지막 에지의 endNode가 첫 번째 startNode와 같을 수 있음
        for (const auto& edge : edges)
        {
            if (node_map.find(edge.endNodeId) == node_map.end())
            {
                std::cerr << "[Vda5050Protocol] End node not found: " << edge.endNodeId << std::endl;
                publishOrderRejectionError("END_NODE_NOT_FOUND", "End node " + edge.endNodeId + " not found in order");
                return;
            }
            
            std::cout << "[Vda5050Protocol] Adding endNode to ordered_nodes_: " << edge.endNodeId << std::endl;
            ordered_nodes_.push_back(node_map[edge.endNodeId]);
        }

        std::cout << "[Vda5050Protocol] Total nodes in ordered_nodes_: " << ordered_nodes_.size() << std::endl;
        std::cout << "[Vda5050Protocol] Start node (will be marked as completed): " << start_node.nodeId << std::endl;

        // Start node position validation
        if (!start_node.hasNodePosition)
        {
            std::cerr << "[Vda5050Protocol] Start node has no position information" << std::endl;
            publishOrderRejectionError("START_NODE_NO_POSITION", "Start node missing position");
            return;
        }

        // Get current vehicle position
        double current_x = 0.0, current_y = 0.0, current_theta = 0.0;
        amr->getVcu()->getEstimatedPose(current_x, current_y, current_theta);

        // Calculate distance to start node
        double dx = start_node.nodePosition.x - current_x;
        double dy = start_node.nodePosition.y - current_y;
        double distance_to_start = std::hypot(dx, dy);

        std::cout << "[Vda5050Protocol] Distance to start node '" << start_node.nodeId 
                  << "': " << distance_to_start << "m (current: " << current_x << ", " << current_y 
                  << " | start: " << start_node.nodePosition.x << ", " << start_node.nodePosition.y << ")" 
                  << std::endl;

        constexpr double MAX_START_NODE_DISTANCE = 1.0;

        if (distance_to_start > MAX_START_NODE_DISTANCE)
        {
            std::cerr << "[Vda5050Protocol] Start node too far from current position: " 
                      << distance_to_start << "m (max: " << MAX_START_NODE_DISTANCE << "m)" << std::endl;
            
            std::ostringstream error_msg;
            error_msg << "Start node '" << start_node.nodeId << "' is " 
                      << distance_to_start << "m away from current position (max: " 
                      << MAX_START_NODE_DISTANCE << "m)";
            
            publishOrderRejectionError("START_NODE_TOO_FAR", error_msg.str());
            return;
        }

        std::cout << "[Vda5050Protocol] Start node within acceptable range (" 
                  << distance_to_start << "m). Marking as completed." << std::endl;

        // Set order active flag
        order_active_ = true;
        
        // Convert node_map to vector for all_nodes
        std::vector<NodeInfo> all_nodes;
        for (const auto& pair : node_map)
        {
            all_nodes.push_back(pair.second);
        }
        
        // Send order to AMR
        amr->setOrder(ordered_nodes_, edges, all_nodes, 15.0);

        // Mark start node as completed
        amr->markNodeAsCompleted(start_node);

        std::cout << "[Vda5050Protocol] Order sent to AMR: " << ordered_nodes_.size()
                  << " nodes (endNodes), " << edges.size() << " edges" << std::endl;
        
        // Publish state immediately after receiving order
        publishStateMessage(amr);
        std::cout << "[Vda5050Protocol] State published immediately after order reception" << std::endl;
    } 
    catch (const nlohmann::json::exception& e)
    {
        std::cerr << "[Vda5050Protocol] JSON Parsing Error: " << e.what() << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[Vda5050Protocol] Unknown Error in handleMessage: " << e.what() << std::endl;
    }
}


void Vda5050Protocol::publishOrderRejectionError(const std::string& error_type, const std::string& error_description)
{
    std::cout << "[Vda5050Protocol] Publishing order rejection error: " << error_type << std::endl;
    
    // 오더 활성화 안 함
    order_active_ = false;
    
    // 오더 정보 초기화
    current_order_id_ = "";
    current_order_update_id_ = 0;
    current_zone_set_id_ = "";
    received_nodes_.clear();
    received_edges_.clear();
    
    // State 메시지 생성 (에러 포함)
    if (amr_)
    {
        // 에러를 임시로 저장
        order_rejection_error_type_ = error_type;
        order_rejection_error_description_ = error_description;
        has_order_rejection_error_ = true;
        
        publishStateMessage(amr_);
        
        // 에러 플래그 리셋
        has_order_rejection_error_ = false;
        order_rejection_error_type_ = "";
        order_rejection_error_description_ = "";
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
    viz_json["agvPosition"]["mapDescription"] = "";
    viz_json["agvPosition"]["localizationScore"] = 0.95;
    viz_json["agvPosition"]["deviationRange"] = 0.0;

    viz_json["velocity"] = {
        {"vx", amr->getVcu()->getMotor().getLinearVelocity()},
        {"vy", 0.0},
        {"omega", amr->getVcu()->getMotor().getAngularVelocity()}
    };

    return viz_json.dump();
}

// STATE MESSAGE PUBLISHING
void Vda5050Protocol::publishStateMessage(IAmr* amr) 
{
    if (!amr_ || !mqtt_client_ || !mqtt_client_->is_connected()) 
    {
        std::cerr << "[Vda5050Protocol] Cannot publish state: AMR or MQTT not available" << std::endl;
        return;
    }

    try 
    {
        std::string state_msg = makeStateMessage(amr);
        if (!state_msg.empty()) 
        {
            auto msg = mqtt::make_message(state_topic_, state_msg);
            msg->set_qos(1);
            mqtt_client_->publish(msg);
            std::cout << "[Vda5050Protocol] State message published successfully" << std::endl;
        }
    } 
    catch (const std::exception& e) 
    {
        std::cerr << "[Vda5050Protocol] State publish exception: " << e.what() << std::endl;
    }
}

// STATE MESSAGE CREATION (VDA5050 2.1 compliant)
std::string Vda5050Protocol::makeStateMessage(IAmr* amr)
{
    if (!amr) 
    {
        std::cerr << "[Vda5050Protocol] Cannot create state message: AMR is null" << std::endl;
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
    
    // 6. Node States Array (Requirements 1, 2, 3, 4, 5, 6)
    nlohmann::json node_states = nlohmann::json::array();
    
    // IDLE state: No order or order completed (Requirements 1, 2)
    if (!order_active_ || (current_order_id_.empty() && received_nodes_.empty()))
    {
        // nodeStates must be empty array in IDLE state
        std::cout << "[Vda5050Protocol] IDLE state - empty nodeStates" << std::endl;
    }
    else
    {
        // Order execution in progress (Requirements 3, 4, 5, 6)
        auto completed_nodes = amr->getCompletedNodes();
        std::set<std::string> completed_node_ids;
        
        // Build set of completed node IDs
        for (const auto& node : completed_nodes)
        {
            completed_node_ids.insert(node.nodeId);
        }
        
        // Include all nodes from FMS order (Requirement 4)
        // Exclude completed nodes (Requirement 5)
        for (const auto& node : received_nodes_)
        {
            // Skip completed nodes
            if (completed_node_ids.find(node.nodeId) != completed_node_ids.end())
            {
                // std::cout <<"skip : " << node.nodeId << std::endl;
                continue;
            }
            
            nlohmann::json node_state = {
                {"nodeId", node.nodeId},
                {"sequenceId", node.sequenceId},
                {"released", node.released}  // Use original release value from order (Requirement 6)
            };
            
            if (node.hasNodePosition)
            {
                node_state["nodePosition"] = {
                    {"x", node.nodePosition.x},
                    {"y", node.nodePosition.y},
                    {"mapId", node.nodePosition.mapId},
                    {"theta", node.nodePosition.theta},
                    {"positionInitialized", node.nodePosition.positionInitialized}
                };
            }
            
            node_states.push_back(node_state);
        }
    }
    // std::cout << "pub_node : " << std::endl;
    // std::cout << node_states << std::endl;
    state_json["nodeStates"] = node_states;
    
    // 7. Edge States Array (Requirements 1, 2, 3, 4, 5, 6)
    nlohmann::json edge_states = nlohmann::json::array();
    
    // IDLE state: No order or order completed (Requirements 1, 2)
    if (!order_active_ || (current_order_id_.empty() && received_edges_.empty()))
    {
        // edgeStates must be empty array in IDLE state
        std::cout << "[Vda5050Protocol] IDLE state - empty edgeStates" << std::endl;
    }
    else
    {
        // Order execution in progress (Requirements 3, 4, 5, 6)
        auto completed_edges = amr->getCompletedEdges();
        std::set<std::string> completed_edge_ids;
        
        // Build set of completed edge IDs
        for (const auto& edge : completed_edges)
        {
            completed_edge_ids.insert(edge.edgeId);
        }
        
        // Include all edges from FMS order (Requirement 4)
        // Exclude completed edges (Requirement 5)
        for (const auto& edge : received_edges_)
        {
            // Skip completed edges
            if (completed_edge_ids.find(edge.edgeId) != completed_edge_ids.end())
            {
                continue;
            }
            
            nlohmann::json edge_state = {
                {"edgeId", edge.edgeId},
                {"sequenceId", edge.sequenceId},
                {"released", edge.released}  // Use original release value from order (Requirement 6)
            };
            
            // Include trajectory information if present
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
    }
    
    state_json["edgeStates"] = edge_states;
    
    // 8. AGV Position (Required)
    double x = 0.0, y = 0.0, theta = 0.0;
    amr->getVcu()->getEstimatedPose(x, y, theta);
    
    state_json["agvPosition"] = {
        {"x", x},
        {"y", y},
        {"theta", theta},
        {"mapId", "default_map"},
        {"positionInitialized", true}
    };
    state_json["agvPosition"]["mapDescription"] = "";
    state_json["agvPosition"]["localizationScore"] = 0.95;
    state_json["agvPosition"]["deviationRange"] = 0.0;
    
    // 9. Velocity (Required)
    state_json["velocity"] = {
        {"vx", amr->getVcu()->getMotor().getLinearVelocity()},
        {"vy", 0.0},
        {"omega", amr->getVcu()->getMotor().getAngularVelocity()}
    };
    
    // 10. Loads (Required, nullable array)
    state_json["loads"] = nlohmann::json::array();
    
    // 11. Action States Array (Required)
    nlohmann::json action_states = nlohmann::json::array();
    
    auto current_actions = getCurrentActions(amr);
    for (const auto& action : current_actions) 
    {
        nlohmann::json action_state = {
            {"actionId", action.actionId},
            {"actionStatus", action.status}
        };
        
        // Optional fields
        if (!action.actionType.empty())
        {
            action_state["actionType"] = action.actionType;
        }
        if (!action.description.empty())
        {
            action_state["actionDescription"] = action.description;
        }
        if (!action.resultDescription.empty())
        {
            action_state["resultDescription"] = action.resultDescription;
        }
        
        action_states.push_back(action_state);
    }
    
    state_json["actionStates"] = action_states;
    
    // 12. Battery State (Required)
    state_json["batteryState"] = {
        {"batteryCharge", amr->getBatteryPercent()},
        {"charging", isCharging(amr)}
    };
    
    // Optional battery fields
    state_json["batteryState"]["batteryVoltage"] = 0.0;
    state_json["batteryState"]["batteryHealth"] = 100.0;
    state_json["batteryState"]["reach"] = 0.0;
    
    // 13. Errors Array (Required)
    nlohmann::json errors = nlohmann::json::array();
    if (has_order_rejection_error_)
    {
        nlohmann::json rejection_error = {
            {"errorType", order_rejection_error_type_},
            {"errorLevel", "FATAL"},
            {"errorDescription", order_rejection_error_description_}
        };
        rejection_error["errorReferences"] = nlohmann::json::array();
        errors.push_back(rejection_error);
        
        std::cout << "[Vda5050Protocol] Order rejection error included in state: " 
                  << order_rejection_error_type_ << std::endl;
    }    
    
    // Check for low battery
    if (amr->getBatteryPercent() < 20.0) 
    {
        nlohmann::json battery_error = {
            {"errorType", "BATTERY_LOW"},
            {"errorLevel", "WARNING"},
            {"errorDescription", "Battery level is below 20%"}
        };
        battery_error["errorReferences"] = nlohmann::json::array();
        errors.push_back(battery_error);
    }
    
    // Check for system errors
    if (amr_state.find("ERROR") != std::string::npos) 
    {
        nlohmann::json system_error = {
            {"errorType", "SYSTEM_ERROR"},
            {"errorLevel", "FATAL"},
            {"errorDescription", amr_state}
        };
        system_error["errorReferences"] = nlohmann::json::array();
        errors.push_back(system_error);
    }
    
    state_json["errors"] = errors;
    
    // 14. Information Array (Required)
    state_json["information"] = nlohmann::json::array();
    
    // 15. Safety State (Required)
    std::string estop_status = getEmergencyStopStatus(amr) ? "MANUAL" : "NONE";
    state_json["safetyState"] = {
        {"eStop", estop_status},
        {"fieldViolation", getFieldViolationStatus(amr)}
    };
    
    return state_json.dump();
}

void Vda5050Protocol::checkOrderCompletion(IAmr* amr)
{
    if (!order_active_ || !amr)
        return;
    
    auto completed_nodes = amr->getCompletedNodes();
    auto completed_edges = amr->getCompletedEdges();
    
    // 디버깅: completed_nodes 내용 출력
    std::cout << "[checkOrderCompletion] Completed nodes list:" << std::endl;
    for (size_t i = 0; i < completed_nodes.size(); ++i)
    {
        std::cout << "  [" << i << "] " << completed_nodes[i].nodeId << std::endl;
    }
    
    // 순환 경로 감지
    bool is_circular = false;
    if (!completed_nodes.empty() && !ordered_nodes_.empty())
    {
        const std::string& start_node_id = completed_nodes.front().nodeId;
        const std::string& last_ordered_node_id = ordered_nodes_.back().nodeId;
        
        if (start_node_id == last_ordered_node_id)
        {
            is_circular = true;
            std::cout << "[Vda5050Protocol] Circular path detected: start='" << start_node_id 
                      << "' == end='" << last_ordered_node_id << "'" << std::endl;
        }
    }
    
    // 수정: 순환 경로일 때 고유 노드 개수 계산
    size_t unique_node_count = completed_nodes.size();
    
    if (is_circular)
    {
        // 순환 경로: 시작 노드와 끝 노드가 같으므로 중복 카운트
        // 고유 노드 개수 = completed_nodes.size() (중복이 이미 제거되어야 함)
        // 하지만 실제로는 중복이 있을 수 있으므로 다시 계산
        std::set<std::string> unique_nodes;
        for (const auto& node : completed_nodes)
        {
            unique_nodes.insert(node.nodeId);
        }
        unique_node_count = unique_nodes.size();
        
        std::cout << "[Vda5050Protocol] Unique node count (set): " << unique_node_count 
                  << " (original: " << completed_nodes.size() << ")" << std::endl;
    }
    
    bool all_nodes_completed;
    if (is_circular)
    {
        // 순환: 고유 노드 개수 == ordered_nodes_ 크기
        all_nodes_completed = (unique_node_count == ordered_nodes_.size());
    }
    else
    {
        all_nodes_completed = (completed_nodes.size() == ordered_nodes_.size() + 1);
    }
    
    bool all_edges_completed = (completed_edges.size() == received_edges_.size());

    std::cout << "[Vda5050Protocol] Order progress: Nodes(" << completed_nodes.size() 
              << "/" << (is_circular ? ordered_nodes_.size() : ordered_nodes_.size() + 1) 
              << ", unique: " << unique_node_count
              << "), Edges(" << completed_edges.size() << "/" << received_edges_.size() 
              << ") [" << (is_circular ? "CIRCULAR" : "LINEAR") << "]" << std::endl;    
    
    if (all_nodes_completed && all_edges_completed && !ordered_nodes_.empty())
    {
        std::cout << "[Vda5050Protocol] Order completed - switching to IDLE state" << std::endl;
        order_active_ = false;

        current_order_id_ = "";
        current_order_update_id_ = 0;
        current_zone_set_id_ = "";        
        
        publishStateMessage(amr);

        std::cout << "[Vda5050Protocol] IDLE state confirmed. Ready for next order." << std::endl;
    }
}

// HELPER METHODS
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
    
    if (!order_active_)
    {
        // In IDLE state, only report charging action if applicable
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
    
    // During order execution: report actions from uncompleted nodes/edges
    auto completed_nodes = amr->getCompletedNodes();
    std::set<std::string> completed_node_ids;
    for (const auto& node : completed_nodes)
    {
        completed_node_ids.insert(node.nodeId);
    }
    
    for (const auto& node : received_nodes_)
    {
        if (completed_node_ids.find(node.nodeId) == completed_node_ids.end())
        {
            for (const auto& action : node.actions)
            {
                ActionInfo action_info;
                action_info.actionId = action.actionId;
                action_info.actionType = action.actionType;
                action_info.description = action.actionDescription;  // 필드명 수정
                
                // TODO: Get actual action status from AMR
                // 현재는 간단히 WAITING으로 설정
                action_info.status = "WAITING";  
                
                // actionParameters 복사
                // action.actionParameters는 vector<ActionParameter>
                // action_info.actionParameters는 nlohmann::json
                nlohmann::json params_json = nlohmann::json::array();
                for (const auto& param : action.actionParameters)
                {
                    nlohmann::json param_json;
                    param_json["key"] = param.key;
                    param_json["value"] = param.value;
                    params_json.push_back(param_json);
                }
                action_info.actionParameters = params_json;
                
                actions.push_back(action_info);
            }
        }
    }  
    
    auto completed_edges = amr->getCompletedEdges();
    std::set<std::string> completed_edge_ids;
    for (const auto& edge : completed_edges)
    {
        completed_edge_ids.insert(edge.edgeId);
    }
    
    for (const auto& edge : received_edges_)
    {
        if (completed_edge_ids.find(edge.edgeId) == completed_edge_ids.end())
        {
            for (const auto& action : edge.actions)
            {
                ActionInfo action_info;
                action_info.actionId = action.actionId;
                action_info.actionType = action.actionType;
                action_info.description = action.actionDescription;  // 필드명 수정
                
                // TODO: Get actual action status from AMR
                action_info.status = "WAITING";
                
                // actionParameters 복사
                nlohmann::json params_json = nlohmann::json::array();
                for (const auto& param : action.actionParameters)
                {
                    nlohmann::json param_json;
                    param_json["key"] = param.key;
                    param_json["value"] = param.value;
                    params_json.push_back(param_json);
                }
                action_info.actionParameters = params_json;
                
                actions.push_back(action_info);
            }
        }
    }
    
    return actions;
}

std::vector<ErrorInfo> Vda5050Protocol::getSystemErrors(IAmr* amr)
{
    std::vector<ErrorInfo> errors;
    
    // Battery warning
    if (amr->getBatteryPercent() < 20.0)
    {
        ErrorInfo error;
        error.errorType = "BATTERY_LOW";
        error.errorLevel = "WARNING";
        error.description = "Battery level is below 20%";
        errors.push_back(error);
    }
    
    // System errors based on AMR state
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
    // TODO: Implement actual E-Stop status check
    return false;
}

bool Vda5050Protocol::getFieldViolationStatus(IAmr* amr)
{
    // TODO: Implement actual safety zone violation check
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
    double dx, dy;
    
    if (!last_node.hasNodePosition)
    {
        // Use x, y if nodePosition is not available
        dx = current_x - last_node.x;
        dy = current_y - last_node.y;
    }
    else
    {
        dx = current_x - last_node.nodePosition.x;
        dy = current_y - last_node.nodePosition.y;
    }
    
    return std::hypot(dx, dy);
}