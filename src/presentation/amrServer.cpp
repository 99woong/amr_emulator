#include "amrServer.h"
#include "../infrastructure/yamlConfig.h"
#include "../app/amrManager.h"
#include <thread>
#include <chrono>
#include <iostream>

void AmrServerApp::run(const std::string& config_path) 
{
    AmrConfig config = YamlConfig::load(config_path);
    AmrManager manager(config);
    manager.startAll();

    while (true) 
    {
        auto& amrs = manager.getAmrs();
        for (size_t i = 0; i < amrs.size(); ++i)
        {
           amrs[i]->step();
           std::cout << "[state]" << amrs[i]->getState() << std::endl;
           // 또는 프로토콜 포맷
           // std::cout << manager.protocol_layers_[i]->makeStateMessage(amr.get()) << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }    
}