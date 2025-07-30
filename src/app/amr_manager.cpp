#include "amr_manager.h"
#include "dd_acceleration_model.h" 
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

        // 초기 위치 설정 (예: YAML config 선언 값 또는 하드코딩)
        double init_x = 0.0, init_y = 0.0, init_theta = 0.0;
        
        // if (!config.initial_poses.empty() && i < config.initial_poses.size()) 
        
        //     init_x = config.initial_poses[i].x;
        //     init_y = config.initial_poses[i].y;
        //     init_theta = config.initial_poses[i].theta;
        // }
        amr->getVcu()->setInitialPose(init_x, init_y, init_theta);

        amrs_.push_back(std::move(amr));

        std::cout << "port : " << port << std::endl;

        auto protocol = createProtocol(config.protocol_type, agv_id, amrs_.back().get());
        if (protocol)
            protocols_.push_back(std::move(protocol));

        setupTcpServer(port, i);
   }
}

bool AmrManager::isVda5050OrderMessage(const std::string& msg) 
{
    try 
    {
        nlohmann::json j = nlohmann::json::parse(msg);
        // 필수 필드 존재 + nodes 배열 확인
        return j.contains("headerId")
            && j.contains("timestamp")
            && j.contains("orderId")
            && j.contains("nodes")
            && j["nodes"].is_array()
            && !j["nodes"].empty();
    } 
    catch (const nlohmann::json::parse_error& e) 
    {
        std::cerr << "[isVda5050OrderMessage] JSON parse error: " << e.what() << std::endl;
        return false;
    }
}

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

    auto acc_model = std::make_shared<DDAccelerationModel>(
        config.amr_params.mass_vehicle,
        config.amr_params.load_weight,
        config.amr_params.max_torque,
        config.amr_params.friction_coeff,
        config.amr_params.max_speed,
        config.amr_params.max_acceleration,
        config.amr_params.max_deceleration,
        config.amr_params.wheel_radius,                // 휠 반경은 amr_params에 있으므로 따로 넘김
        config.amr_params.max_angular_acceleration,
        config.amr_params.max_angular_deceleration
    );
  
    motor->setAccelerationModel(acc_model);

    auto navigation = std::make_unique<Navigation>();

    std::shared_ptr<ideadReckoningModel> dr_model;

    try 
    {
        std::cout <<"try dr model : " << config.dead_reckoning_model << std::endl;
        dr_model = DeadReckoningModelFactory::create(config.dead_reckoning_model, config);
    } 
    catch (const std::exception&) 
    {
        std::cout <<"catch dr model : " << config.dead_reckoning_model << std::endl;
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
        std::cout << "[TCP Server] Received message on port for AMR " << amr_idx << ":\n" << msg << std::endl;

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

        std::cout << "[TCP Received] size: " << msg.size() << ", content:\n" << msg << std::endl;   

        if (currentProtocol->getProtocolType() == "vda5050" && isVda5050OrderMessage(msg))
        {
            std::cout << "[AmrManager] Forwarding VDA5050 Order message to protocol handler for AMR " << amrs_[amr_idx]->getState() << std::endl;
            
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