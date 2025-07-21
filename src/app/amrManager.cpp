#include "amrManager.h"
#include "../domain/motorController.h"
#include "../domain/navigation.h"
#include "../domain/vcu.h"
#include <iostream>
#include <unistd.h>

AmrManager::AmrManager(const AmrConfig& config)
{
   for (int i = 0; i < config.amr_count; ++i)
   {
       auto motor = std::make_unique<MotorController>(config);
       auto nav = std::make_unique<Navigation>();
       auto vcu = std::make_unique<Vcu>(std::move(motor), std::move(nav));
       amrs_.emplace_back(std::make_unique<Amr>(i, std::move(vcu)));
       std::cout << "port : " << config.base_port + i << std::endl;
       auto server = std::make_unique<TcpServer>(config.base_port + i);
        
       // 기존 custom protocol 핸들러 결합 예시:
       // auto proto = std::make_unique<CustomProtocol>();
       // proto->setAmr(amrs_.back().get());
       // ---------- 프로토콜 객체 생성 및 핸들러 등록 ----------
       // 필요시 config.protocol_type == "vda5050" 조건 분기
       auto vdaProto = std::make_unique<Vda5050Protocol>();

       std::string agv_id = "amr_" + std::to_string(i);
       vdaProto->setAgvId(agv_id);
       vdaProto->useDefaultConfig();

       vdaProto->setAmr(amrs_.back().get());
       vdaProto->start();
       protocols_.push_back(std::move(vdaProto));

       int idx = i;
       server->setCommandHandler([this, idx](const std::string& msg) 
       {
            if (idx < protocols_.size())
            {
                protocols_[idx]->handleMessage(msg, amrs_[idx].get());
            }
           // 상태 응답까지 하고 싶으면 server 내에 send함수 구현 필요
       });

       servers_.push_back(std::move(server));
   }
}

void AmrManager::startAll() 
{
    for (auto& s : servers_) 
        s->start();
}
void AmrManager::stopAll() 
{
    for (auto& s : servers_) 
        s->stop();
}
