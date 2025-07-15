#include "amrServer.h"
#include "../infrastructure/YamlConfig.h"
void AmrServerApp::run(const std::string& config_path) 
{
    AmrConfig config = YamlConfig::load(config_path);
    AmrManager manager(config);
    manager.startAll();

    while (true) 
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }    
}