cmake_minimum_required(VERSION 2.8.12)
project(SurfLabSplineSurface)

add_library(${PROJECT_NAME} SHARED SurfLabSplineSurface.c BiCubicSplineSurface.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES SOFA_NO_CXX11 1)
target_link_libraries(${PROJECT_NAME} SofaCore SofaSimulationCommon SofaGuiCommon)

