cmake_minimum_required(VERSION 3.13)
project(sddc_source)

file(GLOB SRC "src/*.cpp" "src/pffft/*.cpp")

include(${SDRPP_MODULE_CMAKE})

# TODO: Download release from web
if(MSVC)
    set(LIBSDDC_PATH "C:/work/rx888_internal/target/release")
    set(LIBSDDC_LIB "sddc.dll.lib")
else()
    set(LIBSDDC_PATH "/Users/junsu/rx888/rx888_internal/target/release")
    set(LIBSDDC_LIB "libsddc.dylib")
endif()

target_compile_definitions(sddc_source PRIVATE PFFFT_ENABLE_NEON)
target_include_directories(sddc_source PRIVATE "${LIBSDDC_PATH}")
target_link_libraries(sddc_source PRIVATE "${LIBSDDC_PATH}/${LIBSDDC_LIB}")