cmake_minimum_required(VERSION 3.10)
project(amr_emulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include_directories(
    src 
    src/domain/interface 
    src/domain 
    src/app 
    src/infrastructure 
    rc/presentation 
    src/utils 
    third_party/yaml-cpp/include
    third_party/libvda5050pp/include/public
    third_party/libvda5050pp/include/private/core
    third_party/rapidjson/include
    )
add_subdirectory(third_party/yaml-cpp)
add_subdirectory(third_party/libvda5050pp)
file(GLOB_RECURSE SOURCES src/*.cpp)
add_executable(amr_emulator ${SOURCES})
# target_link_libraries(amr_emulator yaml-cpp ws2_32)
target_link_libraries(amr_emulator yaml-cpp vda5050++)
add_subdirectory(test)