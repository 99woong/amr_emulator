// 2025.11.04, VDA5050 2.1 Schema validation

#ifndef VDA5050_PROTOCOL_H
#define VDA5050_PROTOCOL_H

#include <string>
#include <memory>
#include <thread>
#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>
#include "iprotocol.h"
#include "yaml_config.h"

// Forward declarations
class IAmr;
struct AmrConfig;
struct NodeInfo;
struct EdgeInfo;

// Action info
struct ActionInfo 
{
    std::string actionId;
    std::string actionType;
    std::string description;
    std::string status;
    std::string resultDescription;
    nlohmann::json actionParameters;
};

// Error info 
struct ErrorInfo 
{
    std::string errorType;
    std::string errorLevel;
    std::string description;
    std::string hint;
};

class Vda5050Protocol : public IProtocol 
{
public:
    Vda5050Protocol(const AmrConfig& config);
    ~Vda5050Protocol();

    void setAmr(IAmr* amr);
    void setAgvId(const std::string& agv_id);
    void useDefaultConfig(const std::string& server_address);
    void start() override;
    void stop();
    void publishStateMessage(IAmr* amr) override;
    void publishVisualizationMessage(IAmr* amr) override;
    std::string getProtocolType() const override { return "vda5050"; }

private:
    class Vda5050MqttCallback : public mqtt::callback 
    {
    public:
        Vda5050MqttCallback(Vda5050Protocol* proto) : proto_(proto) {}
        void message_arrived(mqtt::const_message_ptr msg) override;
    private:
        Vda5050Protocol* proto_;
    };

    // MQTT 
    std::string mqtt_server_uri_;
    std::unique_ptr<mqtt::async_client> mqtt_client_;
    std::shared_ptr<Vda5050MqttCallback> mqtt_callback_;
    mqtt::connect_options conn_opts_;
    
    // Topic name
    std::string state_topic_;
    std::string order_topic_;
    std::string instant_actions_topic;
    std::string visualization_topic_;
    std::string connection_topic_;
    std::string factsheet_topic_;
    
    // AMR configuration
    IAmr* amr_;
    AmrConfig config_;
    std::string agv_id_;
    bool running_;
    std::thread publish_thread_;
    
    // Header ID counter
    int state_header_id_ = 0;
    int factsheet_header_id_ = 0;
    int connection_header_id_ = 0;
    
    // Order info
    std::string current_order_id_;
    int current_order_update_id_;
    std::string current_zone_set_id_;

    // message validation (VDA5050 Schema validation)
    std::string makeStateMessage(IAmr* amr);
    std::string makeVisualizationMessage(IAmr* amr);
    std::string makeFactsheetMessage();
    std::string makeConnectMessage();
    
    // message process 
    void handleMessage(const std::string& msg, IAmr* amr);
    void handleInstantAction(const nlohmann::json& instant_action_json);
    
    // utility function
    std::string getCurrentTimestampISO8601();
    std::string detectConnection();
    
    // State message helper function
    std::string getCurrentNodeId(IAmr* amr);
    std::string getCurrentEdgeId(IAmr* amr);
    std::string getLastNodeId(IAmr* amr);
    int getLastNodeSequenceId(IAmr* amr);
    nlohmann::json getCurrentNodePosition(IAmr* amr);
    
    std::vector<NodeInfo> getUpcomingNodes(IAmr* amr);
    std::vector<EdgeInfo> getUpcomingEdges(IAmr* amr);
    std::vector<ActionInfo> getCurrentActions(IAmr* amr);
    std::vector<ErrorInfo> getSystemErrors(IAmr* amr);
    
    bool getEmergencyStopStatus(IAmr* amr);
    bool getFieldViolationStatus(IAmr* amr);
    bool isCharging(IAmr* amr);

    // std::string getCurrentNodeId(IAmr* amr);
    // std::string getCurrentEdgeId(IAmr* amr);
    // std::vector<NodeInfo> getUpcomingNodes(IAmr* amr);
    // std::vector<EdgeInfo> getUpcomingEdges(IAmr* amr);
    // std::vector<ActionInfo> getCurrentActions(IAmr* amr);
    // std::vector<ErrorInfo> getSystemErrors(IAmr* amr);
    // bool getEmergencyStopStatus(IAmr* amr);
    // bool getFieldViolationStatus(IAmr* amr);
    // bool isCharging(IAmr* amr);
    // std::string getLastNodeId(IAmr* amr);
    // int getLastNodeSequenceId(IAmr* amr);
    // nlohmann::json getCurrentNodePosition(IAmr* amr);
    
    // 새로 추가되는 메서드
    double getDistanceSinceLastNode(IAmr* amr);    
};

#endif // VDA5050_PROTOCOL_H