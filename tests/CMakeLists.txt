cmake_minimum_required(VERSION 3.13)

file(GLOB UNITTESTS "./*.cpp")

enable_testing()

add_executable(unittest ${UNITTESTS})

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../core/src")

if (MSVC)
    set(CMAKE_CXX_FLAGS "-O2 /std:c++17 /EHsc /W3")

    # Lib path
    target_link_directories(unittest PUBLIC "C:/Program Files/PothosSDR/lib/")

    # Misc headers
    target_include_directories(unittest PUBLIC "C:/Program Files/PothosSDR/include/")

    # Volk
    target_link_libraries(unittest PUBLIC volk)
else()
    find_package(PkgConfig)

    pkg_check_modules(FFTW3 REQUIRED fftw3f)
    pkg_check_modules(VOLK REQUIRED volk)

    target_include_directories(unittest PUBLIC
        ${FFTW3_INCLUDE_DIRS}
        ${VOLK_INCLUDE_DIRS}
    )
    
    target_link_directories(unittest PUBLIC
        ${FFTW3_LIBRARY_DIRS}
        ${VOLK_LIBRARY_DIRS}
    )

    target_link_libraries(unittest PUBLIC
        ${FFTW3_LIBRARIES}
        ${VOLK_LIBRARIES}
    )

    set(CMAKE_CXX_FLAGS "-O0 -std=c++17 -Wall")
    target_link_libraries(unittest PUBLIC pthread ${ASANLIB})
endif (MSVC)


foreach(TESTSRC ${UNITTESTS})
    file(STRINGS ${TESTSRC} TESTS REGEX "^TEST_CASE\(.+\)")
    foreach(TEST ${TESTS})
        if (${TEST} MATCHES "TEST_CASE\\(([^,]+)[\\s,]*([^\\)]+)\\)")
            set(FIXTURE ${CMAKE_MATCH_1})
            set(TESTNAME ${CMAKE_MATCH_2})
            string(STRIP ${FIXTURE} FIXTURE)
            string(STRIP ${TESTNAME} TESTNAME)
            add_test(${FIXTURE}_${TESTNAME} unittest ${FIXTURE}::${TESTNAME})
        endif()
    endforeach()
endforeach()