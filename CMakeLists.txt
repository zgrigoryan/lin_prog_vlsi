cmake_minimum_required(VERSION 3.16)
project(floorplanner LANGUAGES CXX)

option(FP_WITH_HIGHS "Build HiGHS LP solver backend" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(floorplanner_lib
    src/DataModel.cpp
    src/SequencePair.cpp
    src/Placement.cpp
    src/Construction.cpp
    src/LPSolver.cpp
    src/LPFloorplanner.cpp
    src/Annealer.cpp
    src/IO.cpp
)
target_include_directories(floorplanner_lib PUBLIC include)
target_compile_features(floorplanner_lib PUBLIC cxx_std_17)

if(FP_WITH_HIGHS)
    find_package(HiGHS CONFIG QUIET)
    if(HiGHS_FOUND)
        target_compile_definitions(floorplanner_lib PRIVATE FP_WITH_HIGHS)
        if(TARGET highs::highs)
            target_link_libraries(floorplanner_lib PUBLIC highs::highs)
        elseif(TARGET HiGHS::HiGHS)
            target_link_libraries(floorplanner_lib PUBLIC HiGHS::HiGHS)
        endif()
        message(STATUS "HiGHS backend enabled")
    else()
        message(WARNING "HiGHS not found; LP solver backend will report unavailable. Use -DFP_WITH_HIGHS=OFF to silence this.")
    endif()
endif()

add_executable(floorplanner src/main.cpp)
target_link_libraries(floorplanner PRIVATE floorplanner_lib)

enable_testing()
add_executable(floorplanner_tests tests/test_floorplanner.cpp)
target_link_libraries(floorplanner_tests PRIVATE floorplanner_lib)
add_test(NAME floorplanner_tests COMMAND floorplanner_tests)
