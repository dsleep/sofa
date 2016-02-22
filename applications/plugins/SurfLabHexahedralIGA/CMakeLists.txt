cmake_minimum_required(VERSION 2.8.12)
project(SurfLabHexahedralIGA)

add_library(${PROJECT_NAME} SHARED SurfLabHexahedralIGA.c TrilinearHexahedralCorotationalFEMForceField.cpp)
target_link_libraries(${PROJECT_NAME} SofaCore SofaSimulationCommon SofaGuiCommon)

