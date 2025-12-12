cmake_minimum_required(VERSION 3.13)
project(radio)

file(GLOB_RECURSE SRC "src/*.cpp")

find_package(fdk-aac CONFIG REQUIRED)

include(${SDRPP_MODULE_CMAKE})

target_include_directories(radio PRIVATE "src/")
target_link_libraries(radio PRIVATE FDK-AAC::fdk-aac)
