// vda5050_protocol.cpp - handleMessage() 수정
// 시작 노드 위치 검증 및 처리 로직 추가

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

        // InstantActions 메시지 처리
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

        // Order 메시지 검증
        if (!json_msg.contains("nodes") || !json_msg.contains("edges")) 
        {
            std::cerr << "[Vda5050Protocol] Order missing required nodes or edges fields" << std::endl;
            return;
        }

        // Order 정보 추출
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

        // Clear and store received nodes and edges
        received_nodes_.clear();
        received_edges_.clear();

        std::unordered_map<std::string, NodeInfo> node_map;
        std::vector<NodeInfo> ordered_nodes;

        // Parse nodes
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
            
            // Parse actions
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
            ordered_nodes.push_back(n);
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
            
            // Parse actions
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
            
            // Parse trajectory
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

        // ========================================
        // ✅ 시작 노드 위치 검증 및 처리
        // ========================================
        if (ordered_nodes.empty())
        {
            std::cerr << "[Vda5050Protocol] Order has no nodes" << std::endl;
            publishOrderRejectionError("ORDER_NO_NODES", "Order contains no nodes");
            return;
        }

        // 현재 차량 위치 가져오기
        double current_x = 0.0, current_y = 0.0, current_theta = 0.0;
        amr->getVcu()->getEstimatedPose(current_x, current_y, current_theta);

        // 시작 노드 (sequenceId가 가장 작은 노드)
        const NodeInfo& start_node = ordered_nodes[0];
        
        if (!start_node.hasNodePosition)
        {
            std::cerr << "[Vda5050Protocol] Start node has no position information" << std::endl;
            publishOrderRejectionError("START_NODE_NO_POSITION", "Start node missing position");
            return;
        }

        // 시작 노드와 현재 위치 간 거리 계산
        double dx = start_node.nodePosition.x - current_x;
        double dy = start_node.nodePosition.y - current_y;
        double distance_to_start = std::hypot(dx, dy);

        std::cout << "[Vda5050Protocol] Distance to start node '" << start_node.nodeId 
                  << "': " << distance_to_start << "m (current: " << current_x << ", " << current_y 
                  << " | start: " << start_node.nodePosition.x << ", " << start_node.nodePosition.y << ")" 
                  << std::endl;

        constexpr double MAX_START_NODE_DISTANCE = 1.0;  // 1m

        if (distance_to_start > MAX_START_NODE_DISTANCE)
        {
            // ✅ 1m 이상 벗어남 → 오더 거절
            std::cerr << "[Vda5050Protocol] Start node too far from current position: " 
                      << distance_to_start << "m (max: " << MAX_START_NODE_DISTANCE << "m)" << std::endl;
            
            std::ostringstream error_msg;
            error_msg << "Start node '" << start_node.nodeId << "' is " 
                      << distance_to_start << "m away from current position (max: " 
                      << MAX_START_NODE_DISTANCE << "m)";
            
            publishOrderRejectionError("START_NODE_TOO_FAR", error_msg.str());
            return;
        }

        // ✅ 1m 이내 → 시작 노드를 완료된 것으로 처리
        std::cout << "[Vda5050Protocol] Start node within acceptable range (" 
                  << distance_to_start << "m). Marking as completed." << std::endl;

        // 시작 노드를 completed_nodes에 추가 (AMR 내부에서 처리)
        // received_nodes에서 시작 노드 제거 (state에서 제외)
        std::vector<NodeInfo> nodes_to_process;
        std::vector<EdgeInfo> edges_to_process;

        // 시작 노드 제외한 나머지 노드들만 처리
        for (size_t i = 1; i < ordered_nodes.size(); ++i)
        {
            nodes_to_process.push_back(ordered_nodes[i]);
        }

        // 첫 번째 에지 제외 (시작 노드와 연결된 에지)
        if (!edges.empty())
        {
            for (size_t i = 1; i < edges.size(); ++i)
            {
                edges_to_process.push_back(edges[i]);
            }
        }

        // Order active로 설정
        order_active_ = true;

        // AMR에 오더 전달 (시작 노드 제외)
        if (nodes_to_process.empty())
        {
            std::cout << "[Vda5050Protocol] Only start node in order - marking order as completed" << std::endl;
            
            // 시작 노드만 있는 경우 즉시 완료 처리
            order_active_ = false;
            publishStateMessage(amr);
            return;
        }

        amr->setOrder(nodes_to_process, edges_to_process, 15.0);

        // ✅ 시작 노드를 AMR의 completed_nodes에 추가
        // AMR에 시작 노드 완료 알림
        amr->markNodeAsCompleted(start_node);

        std::cout << "[Vda5050Protocol] Order sent to AMR: " << nodes_to_process.size()
                  << " nodes (excluding start node), " << edges_to_process.size() << " edges" << std::endl;
        
        // 오더 수신 즉시 state 발행
        publishStateMessage(amr);
        std::cout << "[Vda5050Protocol] State published immediately after order reception" << std::endl;
    } 
    catch (const nlohmann::json::exception& e)
    {
        std::cerr << "[Vda5050Protocol] JSON Parsing Error: " << e.what() << std::endl;
        publishOrderRejectionError("JSON_PARSE_ERROR", e.what());
    }
    catch (const std::exception& e)
    {
        std::cerr << "[Vda5050Protocol] Unknown Error in handleMessage: " << e.what() << std::endl;
        publishOrderRejectionError("UNKNOWN_ERROR", e.what());
    }
}

// ========================================
// ✅ 오더 거절 에러 발행 함수
// ========================================
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