cmake_minimum_required(VERSION 2.8.12)
project(AAOmni)

if((${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU") OR (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang"))
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

set(HEADER_FILES
    AAOmniDriver.h
    hdDefines.h
    hduVector.h
    AAOmniDefs.h
    AAOmniDevice.h
)

set(SOURCE_FILES
    AAOmniDriver.cpp
    initAAOmni.cpp
    Matrix.cpp
    AAOmniDevice.cpp
)

set(LIBUSB_INCLUDE_DIR "default value" CACHE PATH "Path to libusb")
set(LIBUSB_LIBRARY_DIR "default value" CACHE PATH "Path to libusb")

find_package(SofaGeneral REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/..")
include_directories("${LIBUSB_INCLUDE_DIR}")
link_directories("${LIBUSB_LIBRARY_DIR}")

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_AAOMNIPLUGIN")
target_link_libraries(${PROJECT_NAME} SofaComponentGeneral libusb-1.0)

install(TARGETS ${PROJECT_NAME}
        COMPONENT AAOmni_libraries
        EXPORT AAOmniTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY examples/ DESTINATION share/sofa/plugins/${PROJECT_NAME})