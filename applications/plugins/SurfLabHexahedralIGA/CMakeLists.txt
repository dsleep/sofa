cmake_minimum_required(VERSION 2.8.12)
project(SurfLabHexahedralIGA)

find_package(SofaGeneral)

set(SOURCE_FILES
        SurfLabHexahedralIGA.c
        TrilinearFEMForceField.cpp
        TricubicBezierForceField.cpp
        TricubicBezierSetTopologyContainer.cpp
        )
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES SOFA_NO_CXX11 1)
target_link_libraries(${PROJECT_NAME} SofaCore SofaSimulationCommon)

