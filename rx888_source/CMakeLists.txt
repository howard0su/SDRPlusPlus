cmake_minimum_required(VERSION 3.13)
project(rx888)

if (MSVC)
    set(CMAKE_CXX_FLAGS "-O2 /std:c++17")

    # Lib path
    target_link_directories(sdrpp_core PUBLIC "C:/Program Files/PothosSDR/lib/")

    # Misc headers
    target_include_directories(sdrpp_core PUBLIC "C:/Program Files/PothosSDR/include/")
else()
    set(CMAKE_CXX_FLAGS "-O3 -std=c++17 -fpermissive")
endif (MSVC)

file(GLOB SRC "src/*.cpp" "CyAPI/*.cpp")

add_library(rx888 SHARED ${SRC})
target_link_libraries(rx888 PRIVATE sdrpp_core)
target_include_directories(rx888 PRIVATE "CyAPI")
set_target_properties(rx888 PROPERTIES PREFIX "")