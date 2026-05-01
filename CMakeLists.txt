cmake_minimum_required(VERSION 3.16)
project(floorplanner LANGUAGES CXX)

option(FP_WITH_HIGHS "Build HiGHS LP solver backend" ON)
option(FP_WITH_MOSEK "Build MOSEK Fusion LP solver backend" OFF)

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

if(FP_WITH_MOSEK)
    set(MOSEK_HOME "$ENV{MOSEK_HOME}" CACHE PATH "MOSEK SDK root directory")
    set(MOSEK_VERSION "$ENV{MOSEK_VERSION}" CACHE STRING "MOSEK SDK version")
    if(NOT MOSEK_HOME)
        set(MOSEK_HOME "$ENV{HOME}/Downloads/mosek")
    endif()
    if(NOT MOSEK_VERSION)
        set(MOSEK_VERSION "11.1")
    endif()

    file(GLOB MOSEK_PLATFORM_CANDIDATES
        "${MOSEK_HOME}/${MOSEK_VERSION}/tools/platform/*"
    )
    find_path(MOSEK_FUSION_INCLUDE_DIR
        NAMES fusion.h
        HINTS ${MOSEK_PLATFORM_CANDIDATES}
        PATH_SUFFIXES h
        NO_DEFAULT_PATH
    )
    find_library(MOSEK_FUSION_LIBRARY
        NAMES fusion64 libfusion64
        HINTS ${MOSEK_PLATFORM_CANDIDATES}
        PATH_SUFFIXES bin
        NO_DEFAULT_PATH
    )
    find_library(MOSEK_LIBRARY
        NAMES mosek64 libmosek64
        HINTS ${MOSEK_PLATFORM_CANDIDATES}
        PATH_SUFFIXES bin
        NO_DEFAULT_PATH
    )

    if(MOSEK_FUSION_INCLUDE_DIR AND MOSEK_FUSION_LIBRARY AND MOSEK_LIBRARY)
        get_filename_component(MOSEK_LIBRARY_DIR "${MOSEK_LIBRARY}" DIRECTORY)
        target_compile_definitions(floorplanner_lib PRIVATE FP_WITH_MOSEK)
        target_include_directories(floorplanner_lib PUBLIC "${MOSEK_FUSION_INCLUDE_DIR}")
        target_link_libraries(floorplanner_lib PUBLIC "${MOSEK_FUSION_LIBRARY}" "${MOSEK_LIBRARY}")
        message(STATUS "MOSEK backend enabled")
        message(STATUS "MOSEK include: ${MOSEK_FUSION_INCLUDE_DIR}")
        message(STATUS "MOSEK libs: ${MOSEK_FUSION_LIBRARY}; ${MOSEK_LIBRARY}")
    else()
        message(WARNING "MOSEK Fusion not found; MOSEK solver backend will report unavailable. Set MOSEK_HOME/MOSEK_VERSION or use -DFP_WITH_MOSEK=OFF.")
    endif()
endif()

add_executable(floorplanner src/main.cpp)
target_link_libraries(floorplanner PRIVATE floorplanner_lib)

enable_testing()
add_executable(floorplanner_tests tests/test_floorplanner.cpp)
target_link_libraries(floorplanner_tests PRIVATE floorplanner_lib)
target_compile_definitions(floorplanner_tests PRIVATE FP_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_test(NAME floorplanner_tests COMMAND floorplanner_tests)

if(FP_WITH_MOSEK AND MOSEK_LIBRARY_DIR)
    set_target_properties(floorplanner floorplanner_tests PROPERTIES
        BUILD_RPATH "${MOSEK_LIBRARY_DIR}"
    )
    if(APPLE)
        foreach(FP_TARGET floorplanner floorplanner_tests)
            add_custom_command(TARGET ${FP_TARGET} POST_BUILD
                COMMAND install_name_tool
                    -change "libfusion64.${MOSEK_VERSION}.dylib"
                            "${MOSEK_LIBRARY_DIR}/libfusion64.${MOSEK_VERSION}.dylib"
                            "$<TARGET_FILE:${FP_TARGET}>"
                COMMAND install_name_tool
                    -change "@rpath/libmosek64.${MOSEK_VERSION}.dylib"
                            "${MOSEK_LIBRARY_DIR}/libmosek64.${MOSEK_VERSION}.dylib"
                            "$<TARGET_FILE:${FP_TARGET}>"
                COMMENT "Fixing MOSEK dynamic library paths for ${FP_TARGET}"
            )
        endforeach()
    endif()
endif()

# Convenience targets borrowed from the 3D floorplanner workflow. They keep
# experiment outputs predictable without making generated results part of the
# source tree.
add_custom_target(output-dirs
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_SOURCE_DIR}/out
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_SOURCE_DIR}/results/comparisons
)

add_custom_target(clean-outputs
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_CURRENT_SOURCE_DIR}/out
)
