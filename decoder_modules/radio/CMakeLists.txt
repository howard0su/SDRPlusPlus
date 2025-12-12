cmake_minimum_required(VERSION 3.13)
project(radio)

file(GLOB_RECURSE SRC "src/*.cpp")

find_package(fdk-aac CONFIG REQUIRED)
find_package(ZLIB REQUIRED)
    
include(${SDRPP_MODULE_CMAKE})

target_compile_definitions(radio PRIVATE HAVE_LIBFDK_AAC)
target_include_directories(radio PRIVATE "src/")
target_include_directories(radio PRIVATE "src/drm/")
target_link_libraries(radio PRIVATE FDK-AAC::fdk-aac)
target_link_libraries(radio PRIVATE ZLIB::ZLIB)