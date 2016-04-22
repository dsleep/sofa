cmake_minimum_required(VERSION 2.8.12)
project(SurfLabConnectingTissue)

if((${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU") OR (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang"))
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

set(HEADER_FILES
	initConnectingTissue.h
    ConnectingTissue.h
	)

set(SOURCE_FILES
	initConnectingTissue.cpp
    ConnectingTissue.cpp
	)

set(README_FILES ConnectingTissue.txt)

find_package(SofaGeneral REQUIRED)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_CONNECTINGTISSUE")
target_link_libraries(${PROJECT_NAME} SofaComponentGeneral)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/..")

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

