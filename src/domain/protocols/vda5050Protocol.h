#pragma once
#include "iprotocol.h"
#include "iamr.h"
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

// Forward declaration for MyNavigationHandler or define it directly in .cpp

class Vda5050Protocol : public IProtocol {
public:
    Vda5050Protocol();
    void setAmr(IAmr* amr);
    void setAgvId(const std::string& agv_id);
    void useDefaultConfig();
    void start() override;
    void stop();

    void handleMessage(const std::string& msg, IAmr* amr) override;
    std::string makeStateMessage(IAmr* amr) override;
    std::string getProtocolType() const override { return "vda5050"; }

private:
    IAmr* amr_ = nullptr;
    std::string agv_id_;
    vda5050pp::Config vda_config_;
    std::unique_ptr<vda5050pp::Handle> handle_;
    std::shared_ptr<vda5050pp::handler::BaseNavigationHandler> navigation_handler_; // Added for navigation handling

    std::string order_topic_;
    std::string state_topic_;

    std::thread publish_thread_;
    std::atomic<bool> running_;

    void processVda5050Order(const nlohmann::json& order_json); // Kept for manual parsing if needed
};

// #pragma once
// #include "iprotocol.h"
// #include "iamr.h"
// #include <vda5050++/handle.h>
// #include <vda5050++/config.h>
// // #include <vda5050++/vda5050.h>
// #include <memory>
// #include <string>
// #include <thread>
// #include <atomic>

// class Vda5050Protocol : public IProtocol {
// public:
//     Vda5050Protocol();
//     void setAmr(IAmr* amr);
//     void setAgvId(const std::string& agv_id);
//     void useDefaultConfig();
//     void start() override;
//     void stop();

//     void handleMessage(const std::string& msg, IAmr* amr) override;
//     std::string makeStateMessage(IAmr* amr) override;
//     std::string getProtocolType() const override { return "vda5050"; }

// private:
//     IAmr* amr_ = nullptr;
//     std::string agv_id_;
//     vda5050pp::Config vda_config_;
//     std::unique_ptr<vda5050pp::Handle> handle_;

//     std::string order_topic_;
//     std::string state_topic_;

//     std::thread publish_thread_;
//     std::atomic<bool> running_;

//     void processVda5050Order(const nlohmann::json& order_json);
// };