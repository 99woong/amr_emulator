#include "amrManager.h"
#include "motorController.h"
#include "navigation.h"
#include "vcu.h"
#include "localizer.h"
#include "deadReckoningModelFactory.h"
#include <iostream>
#include <unistd.h>


// Helper to check if a message is a VDA 5050 Order message
bool AmrManager::isVda5050OrderMessage(const std::string& msg) 
{
    try 
    {
        nlohmann::json j = nlohmann::json::parse(msg);
        // Basic check for VDA 5050 Order fields
        return j.contains("headerId") && j.contains("timestamp") && j.contains("orderId") && j.contains("nodes");
    } 
    catch (const nlohmann::json::parse_error& e) 
    {
        return false;
    }
}

// Helper to check if a message is a Custom TCP message
bool AmrManager::isCustomTcpProtocolMessage(const std::string& msg) 
{
    try 
    {
        nlohmann::json j = nlohmann::json::parse(msg);
        // Basic check for Custom TCP command fields
        return j.contains("command") && j.contains("agvId");
    } 
    catch (const nlohmann::json::parse_error& e) 
    {
        return false;
    }
}

AmrManager::AmrManager(const AmrConfig& config)
{
   for (int i = 0; i < config.amr_count; ++i)
   {
        auto motor = std::make_unique<MotorController>(config);
        auto nav = std::make_unique<Navigation>();

        std::shared_ptr<ideadReckoningModel> dr_model;
        try 
        {
            std::string dr_model_type = config.dead_reckoning_model;
            dr_model = DeadReckoningModelFactory::create(dr_model_type, config);
        } 
        catch (const std::exception& e) 
        {
            // std::cerr << "[AmrManager] DeadReckoningModel 생성 실패: " << e.what() << std::endl;
            // std::cerr << "기본값 differential_drive 모델로 대체합니다." << std::endl;
            dr_model = DeadReckoningModelFactory::create("differential_drive", config);
        }

        auto vcu = std::make_unique<Vcu>(std::move(motor), std::move(nav));

        amrs_.emplace_back(std::make_unique<Amr>(i, std::move(vcu)));
        std::cout << "port : " << config.base_port + i << std::endl;
        
        auto server = std::make_unique<TcpServer>(config.base_port + i);
        
        std::string agv_id = "amr_" + std::to_string(i);

        if (config.protocol_type == "vda5050")
        {
           auto vdaProto = std::make_unique<Vda5050Protocol>();
           vdaProto->setAgvId(agv_id);
           vdaProto->useDefaultConfig();
           vdaProto->setAmr(amrs_.back().get());
           protocols_.push_back(std::move(vdaProto));
           std::cout << "[AmrManager] AMR " << agv_id << " configured for VDA 5050 protocol." << std::endl;
       } 
       else if(config.protocol_type == "custom_tcp") 
       {
        //    auto customProto = std::make_unique<CustomTcpProtocol>();
        //    customProto->setAmr(amrs_.back().get());
        //    protocols_.push_back(std::move(customProto));
           std::cout << "[AmrManager] AMR " << agv_id << " configured for Custom TCP protocol." << std::endl;
       } 
       else
       {
           std::cerr << "[AmrManager] Error: Unknown protocol type '" << config.protocol_type << "' for AMR " << agv_id << ". Defaulting to Custom TCP." << std::endl;
        //    auto customProto = std::make_unique<CustomTcpProtocol>();
        //    customProto->setAmr(amrs_.back().get());
        //    protocols_.push_back(std::move(customProto));
       }

       int idx = i;
       server->setCommandHandler([this, idx](const std::string& msg) 
       {
            if (idx < protocols_.size())
            {
                IProtocol* currentProtocol = protocols_[idx].get();
                // Check message type and then route to the correct handler
                if (currentProtocol->getProtocolType() == "vda5050" && isVda5050OrderMessage(msg)) 
                {
                    // For VDA 5050, orders are typically via MQTT. If it comes via TCP, it's a warning.
                    std::cerr << "[AmrManager] Warning: VDA 5050 order message received via TCP. Expected MQTT. For AMR " << amrs_[idx]->getState() << std::endl;
                    currentProtocol->handleMessage(msg, amrs_[idx].get());
                } 
                else if (currentProtocol->getProtocolType() == "custom_tcp" && isCustomTcpProtocolMessage(msg)) \
                {
                    // currentProtocol->handleMessage(msg, amrs_[idx].get());
                } 
                else 
                {
                    std::cerr << "[AmrManager] Error: Mismatch between configured protocol and received message type for AMR " << amrs_[idx]->getState() << ". Ignoring message: " << msg.substr(0, std::min((size_t)100, msg.length())) << "..." << std::endl;
                }
            } 
            else 
            {
                std::cerr << "[AmrManager] Error: Protocol handler not found for AMR index " << idx << std::endl;
            }
       });
       servers_.push_back(std::move(server));
   }
}

void AmrManager::startAll() 
{
    for (auto& proto : protocols_) 
    {
        proto->start();
    }    
    
    for (auto& s : servers_) 
        s->start();
}

void AmrManager::stopAll() 
{
    for (auto& s : servers_) 
        s->stop();
}

size_t AmrManager::getProtocolCount() const 
{
    return protocols_.size();
}

IProtocol* AmrManager::getProtocol(size_t index) const 
{
    if (index < protocols_.size()) 
    {
        return protocols_[index].get();
    }
    return nullptr;
}