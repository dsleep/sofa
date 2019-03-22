cmake_minimum_required(VERSION 2.8.12)
project(SurfLabHaptic)

if((${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU") OR (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang"))
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

set(HEADER_FILES
	initSurfLabHaptic.h
    GraspingForceFeedback.h
	HapticManager.h
	)

set(SOURCE_FILES
	initSurfLabHaptic.cpp
    GraspingForceFeedback.cpp
	HapticManager.cpp	
	)

set(README_FILES SurfLabHaptic.txt)

#set(LIBUSB_INCLUDE_DIR "default value" CACHE PATH "Path to libusb")
#set(LIBUSB_LIBRARY_DIR "default value" CACHE PATH "Path to libusb")

find_package(SofaGeneral REQUIRED)
#link_directories("${LIBUSB_LIBRARY_DIR}")

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_SURFLABHAPTIC")
# target_link_libraries(${PROJECT_NAME} SofaComponentGeneral)
# include_directories("${CMAKE_CURRENT_SOURCE_DIR

#target_link_libraries(${PROJECT_NAME} SofaComponentGeneral ${OPENHAPTICS_LIBRARIES} Sensable AAOmni SofaGuiCommon SofaGuiQt libusb-1.0)
target_link_libraries(${PROJECT_NAME} SofaComponentGeneral ${OPENHAPTICS_LIBRARIES} Sensable SofaGuiCommon SofaGuiQt)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/.."
#${LIBUSB_INCLUDE_DIR}
#${CMAKE_CURRENT_SOURCE_DIR}/../AAOmni
${CMAKE_CURRENT_SOURCE_DIR}/../Sensable
${CMAKE_CURRENT_SOURCE_DIR}/../../sofa/gui
${CMAKE_CURRENT_SOURCE_DIR}/../../sofa/gui/qt
${OPENHAPTICS_INCLUDE_DIR}
${OPENHAPTICS_UTIL_INCLUDE_DIR}
)

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

