cmake_minimum_required(VERSION 3.1)
project(Sensable)

if((${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU") OR (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang"))
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()
find_package(SofaHaptics REQUIRED)
find_package(SofaOpenglVisual REQUIRED)
find_package(SofaGeneral REQUIRED)

find_package(OpenHaptics)

if( NOT OPENHAPTICS_FOUND )
    message( SEND_ERROR "OpenHaptics library not found, GeomagicDriver cannot compile")
endif()


set(HEADER_FILES
    EnslavementForceFeedback.h
    NewOmniDriver.h
    OmniDriver.h
)

set(SOURCE_FILES
    EnslavementForceFeedback.cpp
    NewOmniDriver.cpp
    OmniDriver.cpp
    initSensable.cpp
)

set(README_FILES PluginSensable.txt)

find_package(SofaGeneral REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

include_directories(${OPENHAPTICS_INCLUDE_DIR})
include_directories(${OPENHAPTICS_UTIL_INCLUDE_DIR})
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/..")

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_SENSABLEPLUGIN")
target_link_libraries(${PROJECT_NAME} SofaComponentGeneral SofaHaptics SofaOpenglVisual ${OPENHAPTICS_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        COMPONENT Sensable_libraries
        EXPORT SensableTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY examples/ DESTINATION share/sofa/plugins/${PROJECT_NAME})
