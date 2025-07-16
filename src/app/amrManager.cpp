#include "AmrManager.h"
#include "../domain/MotorController.h"
#include "../domain/Navigation.h"
#include "../domain/Vcu.h"
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
        servers_.push_back(std::move(server));
    }
}
void AmrManager::startAll() 
{
    for (auto& s : servers_) s->start();
}
void AmrManager::stopAll() 
{
    for (auto& s : servers_) s->stop();
}