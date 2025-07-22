#include "amrServer.h"
#include "../infrastructure/yamlConfig.h"
#include "../app/amrManager.h" // AmrManager 클래스의 올바른 경로로 변경
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
           
           // 수정된 부분: protocols_ 멤버에 직접 접근하는 대신 public 메서드 사용
           if (i < manager.getProtocolCount()) // getProtocolCount() 사용
           {
                IProtocol* currentProtocol = manager.getProtocol(i); // getProtocol() 사용
                if (currentProtocol) { // nullptr 체크
                    std::cout << "[state] " << currentProtocol->makeStateMessage(amrs[i].get()) << std::endl;
                } else {
                    // 이 경우는 getProtocolCount()가 i보다 크지만 getProtocol(i)이 nullptr을 반환하는 예외적인 상황
                    std::cerr << "[state] Error: Protocol at index " << i << " is null for AMR" << amrs[i]->getState() << std::endl;
                }
           } 
           else 
           {
                std::cout << "[state] AMR" << amrs[i]->getState() << " (No protocol attached or invalid index)" << std::endl;
           }            
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }    
    // Note: In a real application, you would need a mechanism to gracefully
    // stop the `while(true)` loop and call `manager.stopAll()` before exiting.
}
