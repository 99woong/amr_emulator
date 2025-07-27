#include "amr_manager.h"
#include "motor_controller.h"
#include "navigation.h"
#include "vcu.h"
#include "localizer.h"
#include "dead_reckoning_model_factory.h"
#include <iostream>
#include <unistd.h>

// 생성자: 여러 AMR 인스턴스 및 관련 객체 생성 후 초기화
AmrManager::AmrManager(const AmrConfig& config)
    : config_(config)
{
   for (int i = 0; i < config.amr_count; ++i)
   {
        int port = config.base_port + i;
        std::string agv_id = "amr_" + std::to_string(i);

        auto amr = createSingleAmr(i, config);
        amrs_.push_back(std::move(amr));

        std::cout << "port : " << port << std::endl;

        auto protocol = createProtocol(config.protocol_type, agv_id, amrs_.back().get());
        if (protocol)
            protocols_.push_back(std::move(protocol));

        setupTcpServer(port, i);
   }
}

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

// 개별 AMR 생성
std::unique_ptr<Amr> AmrManager::createSingleAmr(int id, const AmrConfig& config)
{
    auto motor = std::make_unique<MotorController>(config);
    auto navigation = std::make_unique<Navigation>();

    std::shared_ptr<ideadReckoningModel> dr_model;
    try 
    {
        dr_model = DeadReckoningModelFactory::create(config.dead_reckoning_model, config);
    } 
    catch (const std::exception&) 
    {
        dr_model = DeadReckoningModelFactory::create("differential_drive", config);
    }

    auto localizer = std::make_unique<Localizer>(dr_model);
    auto vcu = std::make_unique<Vcu>(std::move(motor), std::move(navigation), std::move(localizer));

    return std::make_unique<Amr>(id, std::move(vcu));
}

// 프로토콜 생성 및 초기화
std::unique_ptr<IProtocol> AmrManager::createProtocol(const std::string& protocol_type, const std::string& agv_id, Amr* amr)
{
    if (protocol_type == "vda5050")
    {
        auto vdaProto = std::make_unique<Vda5050Protocol>();
        vdaProto->setAgvId(agv_id);
        vdaProto->useDefaultConfig();
        vdaProto->setAmr(amr);
        std::cout << "[AmrManager] AMR " << agv_id << " configured for VDA 5050 protocol." << std::endl;
        return vdaProto;
    }
    else if(protocol_type == "custom_tcp") 
    {
        // custom TCP 프로토콜 초기화 필요시 여기 구현
        std::cout << "[AmrManager] AMR " << agv_id << " configured for Custom TCP protocol." << std::endl;
        return nullptr;
    }
    else
    {
        std::cerr << "[AmrManager] Error: Unknown protocol type '" << protocol_type << "' for AMR " << agv_id 
                  << ". Defaulting to Custom TCP." << std::endl;
        return nullptr;
    }
}

// TCP 서버 생성 및 명령 핸들러 등록
void AmrManager::setupTcpServer(int port, int amr_idx)
{
    auto server = std::make_unique<TcpServer>(port);

    server->setCommandHandler([this, amr_idx](const std::string& msg) 
    {
        if (amr_idx >= protocols_.size()) 
        {
            std::cerr << "[AmrManager] Error: Protocol handler not found for AMR index " << amr_idx << std::endl;
            return;
        }

        IProtocol* currentProtocol = protocols_[amr_idx].get();
        if (!currentProtocol) 
        {
            std::cerr << "[AmrManager] Error: Protocol pointer is null for AMR index " << amr_idx << std::endl;
            return;
        }

        if (currentProtocol->getProtocolType() == "vda5050" && isVda5050OrderMessage(msg))
        {
            std::cerr << "[AmrManager] Warning: VDA 5050 order message received via TCP. Expected MQTT. For AMR " 
                      << amrs_[amr_idx]->getState() << std::endl;
            currentProtocol->handleMessage(msg, amrs_[amr_idx].get());
        }
        else if (currentProtocol->getProtocolType() == "custom_tcp" && isCustomTcpProtocolMessage(msg))
        {
            // currentProtocol->handleMessage(msg, amrs_[amr_idx].get());
        }
        else
        {
            std::cerr << "[AmrManager] Error: Mismatch between configured protocol and incoming message for AMR " 
                      << amrs_[amr_idx]->getState() << ". Ignoring message: " << msg.substr(0, std::min((size_t)100, msg.length())) << "..." << std::endl;
        }
    });

    servers_.push_back(std::move(server));
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