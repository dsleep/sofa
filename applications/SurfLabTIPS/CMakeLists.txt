cmake_minimum_required(VERSION 2.8.12)
project(SurflabTIPS)

set(SOURCE_FILES
  SurfLabTIPS.cpp
  Visualizer.cpp
)

set(HEADER_FILES
  Visualizer.hpp
)

set(CMAKE_AUTOMOC ON)

find_package(SofaGeneral REQUIRED)
find_package(SofaMisc REQUIRED)

find_package(Qt5 COMPONENTS Core Gui Widgets OpenGL REQUIRED)

qt5_add_resources(RESOURCE_FILES resources.qrc)

add_executable(${PROJECT_NAME} ${HEADER_FILES} ${MOC_FILES} ${SOURCE_FILES} ${RESOURCE_FILES})
target_link_libraries(${PROJECT_NAME} SofaCore SofaSimulationCommon SofaSimulationGraph SofaComponentBase SofaComponentCommon SofaComponentGeneral SofaComponentAdvanced SofaComponentMisc Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL)

