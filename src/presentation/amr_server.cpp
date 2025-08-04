#include "amr_server.h"
#include "yaml_config.h"
#include "amr_manager.h" // AmrManager 클래스의 올바른 경로로 변경
#include <thread>
#include <chrono>
#include <iostream>


void AmrServerApp::run(const std::string& config_path)
{
    AmrConfig config = YamlConfig::load(config_path);
    AmrManager manager(config);
    manager.startAll();

    const double speedup = config.speedup_ratio;
    const double dt_control = 0.01;       // 내부 제어 주기(10ms)
    const double dt_state = 1.0;           // state topic (1초)
    const double dt_vis = 0.05;            // visualization topic (50ms)
    const double dt_master = 0.001;        // 최소 단위 루프(가급적 작게)

    double sim_time = 0.0;
    double next_motor_update = 0.0;
    double next_state_pub = 0.0;
    double next_vis_pub = 0.0;

    while (true)
    {
        sim_time += dt_master * speedup;
        auto& amrs = manager.getAmrs();

        // 내부 제어: dt_internal 마다 호출
        if (sim_time >= next_motor_update) 
        {
            for (auto& amr : amrs)
            {
                amr->step(dt_control); // 내부적으로 vcu->update(dt_internal) 등 호출됨
            }
            next_motor_update += dt_control;
        }

        // state topic publish: dt_state 마다 호출
        // std::cout << "sim_time :  " << sim_time << " next_state_pub : " << next_state_pub << " next_vis_pub : " << next_vis_pub <<std::endl;
        if (sim_time >= next_state_pub) 
        {
            for (size_t i = 0; i < amrs.size(); ++i) 
            {
                if (i < manager.getProtocolCount()) 
                {
                    auto* p = manager.getProtocol(i);
                    if (p) 
                    {
                        // std::cout << "publishStateMessage" << std::endl;
                        p->publishStateMessage(amrs[i].get());
                    }
                }
            }
            next_state_pub += dt_state;
        }
        // visualization topic publish: dt_vis 마다 호출
        if (sim_time >= next_vis_pub) 
        {
            for (size_t i = 0; i < amrs.size(); ++i) 
            {
                if (i < manager.getProtocolCount()) 
                {
                    auto* protocol = manager.getProtocol(i);
                    if (protocol)
                    {
                        // std::cout << "publishVisualizationMessage" << std::endl;
                        protocol->publishVisualizationMessage(amrs[i].get());
                    }
                }
            }
            next_vis_pub += dt_vis;
        }
        
        std::this_thread::sleep_for(std::chrono::duration<double>(dt_master / speedup));
        // 실제 sleep 시간은 speedup 반영 (dt_master만큼 시뮬 타임 진행)
    }
}


// void AmrServerApp::run(const std::string& config_path) 
// {
//     AmrConfig config = YamlConfig::load(config_path);
//     AmrManager manager(config);
//     manager.startAll();

//     const double base_dt = 1.0; // 기존 1초 루프
//     const double sim_dt = base_dt / config.speedup_ratio; // 배속에 따라 루프주기 단축
    
//     while (true) 
//     {
//         auto& amrs = manager.getAmrs();
//         for (size_t i = 0; i < amrs.size(); ++i)
//         {
//            amrs[i]->step(sim_dt);
           
//            // 수정된 부분: protocols_ 멤버에 직접 접근하는 대신 public 메서드 사용
//            if (i < manager.getProtocolCount()) // getProtocolCount() 사용
//            {
//                 IProtocol* currentProtocol = manager.getProtocol(i); // getProtocol() 사용
//                 if (currentProtocol) 
//                 { // nullptr 체크
//                     // std::cout << "[state1] " << currentProtocol->makeStateMessage(amrs[i].get()) << std::endl;
//                 } 
//                 else 
//                 {
//                     // 이 경우는 getProtocolCount()가 i보다 크지만 getProtocol(i)이 nullptr을 반환하는 예외적인 상황
//                     std::cerr << "[state] Error: Protocol at index " << i << " is null for AMR" << amrs[i]->getState() << std::endl;
//                 }
//            } 
//            else 
//            {
//                 std::cout << "[state] AMR" << amrs[i]->getState() << " (No protocol attached or invalid index)" << std::endl;
//            }            
//         }
//         std::this_thread::sleep_for(std::chrono::duration<double>(sim_dt));
//     }    
// }


