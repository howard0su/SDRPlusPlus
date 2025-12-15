cmake_minimum_required(VERSION 3.13)
project(radio)

file(GLOB_RECURSE SRC "src/*.cpp")
    
include(${SDRPP_MODULE_CMAKE})

target_compile_definitions(radio PRIVATE HAVE_LIBFDK_AAC)
target_include_directories(radio PRIVATE "src/")
target_include_directories(radio PRIVATE "src/drm/")

if(MSVC)
    find_package(fdk-aac CONFIG REQUIRED)
    find_package(ZLIB REQUIRED)

    target_link_libraries(radio PRIVATE FDK-AAC::fdk-aac)
    target_link_libraries(radio PRIVATE ZLIB::ZLIB)
else()
    find_package(PkgConfig)

    pkg_check_modules(FDKAAC REQUIRED fdk-aac)
    pkg_check_modules(ZLIB REQUIRED zlib)

    target_include_directories(sdrpp_core PUBLIC
        ${FDKAAC_INCLUDE_DIRS}
        ${ZLIB_INCLUDE_DIRS}
    )

    target_link_directories(sdrpp_core PUBLIC
        ${FDKAAC_LIBRARY_DIRS}
        ${ZLIB_LIBRARY_DIRS}
    )

    target_link_libraries(sdrpp_core PUBLIC
        ${FDKAAC_LIBRARIES}
        ${ZLIB_LIBRARIES}
    )
endif()


