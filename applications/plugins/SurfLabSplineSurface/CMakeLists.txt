cmake_minimum_required(VERSION 2.8.12)
project(SurfLabSplineSurface)

find_package(SofaGeneral)
find_package(SofaGui)

add_library(${PROJECT_NAME} SHARED SurfLabSplineSurface.c BiCubicSplineSurface.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES SOFA_NO_CXX11 1)
target_link_libraries(${PROJECT_NAME} SofaCore SofaSimulationCommon SofaGuiCommon)

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

