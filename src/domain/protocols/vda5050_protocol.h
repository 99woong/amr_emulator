#pragma once
#include "iprotocol.h"
#include "iamr.h"

#include "yaml_config.h"
#include <vda5050++/handle.h>
#include <vda5050++/config.h>
#include <handler/base_navigation_handler.h> // Make sure this is included
#include <sinks/status_sink.h> 


#include <vda5050/Header_vda5050.h> 
#include <vda5050/AGVPosition.h>
#include <vda5050/ActionState.h>
#include <vda5050/BatteryState.h>
#include <vda5050/EdgeState.h>
#include <vda5050/Error.h>
#include <vda5050/Info.h>
#include <vda5050/Load.h>
#include <vda5050/Map.h>
#include <vda5050/NodeState.h>
#include <vda5050/OperatingMode.h>
#include <vda5050/SafetyState.h>
#include <vda5050/State.h> 
#include <vda5050/Velocity.h>

#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <list> // Required for list in BaseNavigationHandler
#include <mqtt/async_client.h>

// Forward declaration for MyNavigationHandler or define it directly in .cpp

class Vda5050Protocol : public IProtocol {
public:
    Vda5050Protocol();
    ~Vda5050Protocol();

    void setAmr(IAmr* amr);
    void setAgvId(const std::string& agv_id);
    void useDefaultConfig();

    void start() override;
    void stop();

    // 메시지 처리 - VDA5050 Order 수신 처리
    void handleMessage(const std::string& msg, IAmr* amr) override;
    // 현재 AMR 상태를 JSON 문자열로 변환
    std::string makeStateMessage(IAmr* amr) override;
    std::string getProtocolType() const override { return "vda5050"; }

    void publishStateMessage(IAmr* amr) override;
    void publishVisualizationMessage(IAmr* amr) override;
    void setConfig(const AmrConfig& config) { config_ = config; }

private:
    IAmr* amr_ = nullptr;
    std::string agv_id_;
    AmrConfig config_;  

    std::string mqtt_server_uri_ = "tcp://localhost:1883";

    vda5050pp::Config vda_config_;
    std::unique_ptr<vda5050pp::Handle> handle_;
    std::shared_ptr<vda5050pp::handler::BaseNavigationHandler> navigation_handler_; // Added for navigation handling

    std::unique_ptr<mqtt::async_client> mqtt_client_;
    mqtt::connect_options conn_opts_;

    void mqttSubscribe();

    class Vda5050MqttCallback : public virtual mqtt::callback
    {
    public:
        Vda5050MqttCallback(Vda5050Protocol* proto) : proto_(proto) {}
        void message_arrived(mqtt::const_message_ptr msg) override;

    private:
        Vda5050Protocol* proto_;
    };

    std::shared_ptr<Vda5050MqttCallback> mqtt_callback_;

    std::string order_topic_;
    std::string state_topic_;
    std::string visualization_topic_;
    std::string instant_actions_topic;

    std::thread publish_thread_;
    std::atomic<bool> running_;

    uint32_t factsheet_header_id_ = 0;

    void processVda5050Order(const nlohmann::json& order_json); // Kept for manual parsing if needed
    void handleInstantAction(const nlohmann::json& instant_action_json);
    std::string makeFactsheetMessage();
    std::string getCurrentTimestampISO8601();
};
