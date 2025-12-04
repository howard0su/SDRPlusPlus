cmake_minimum_required(VERSION 3.13)
project(sddc_source)

file(GLOB SRC "src/*.cpp")

# TODO: Download release from web
set(LIBSDDC_PATH "C:/work/rx888_internal/target/release")

include(${SDRPP_MODULE_CMAKE})

target_include_directories(sddc_source PRIVATE "${LIBSDDC_PATH}")
target_link_libraries(sddc_source PRIVATE "${LIBSDDC_PATH}/sddc.dll.lib")