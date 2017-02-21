cmake_minimum_required(VERSION 2.8.12)
project(SurfLabSpeechRecognition)

if((${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU") OR (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang"))
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

set(HEADER_FILES
	initSurfLabspeechRecognition.h
    	SpeechToText.h
	)

set(SOURCE_FILES
	SpeechToText.cpp
	initSurfLabSpeechRecognition.cpp
)

set(README_FILES SurfLabSpeechRecognition.txt)

find_package(SofaGeneral REQUIRED)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_SURFLABSPEECHRECOGNITION")
target_link_libraries(${PROJECT_NAME} SofaComponentGeneral ${CMAKE_CURRENT_SOURCE_DIR}/sphinxlib/SphinxLib.lib)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/.." 
	${CMAKE_CURRENT_SOURCE_DIR}/sphinxlib 
	${CMAKE_CURRENT_SOURCE_DIR}/sphinxlib/sphinxbase
	)

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_directory
	${CMAKE_CURRENT_SOURCE_DIR}/model $<TARGET_FILE_DIR:${PROJECT_NAME}>/model)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy
	${CMAKE_CURRENT_SOURCE_DIR}/sphinxlib/SphinxLib.dll 
	$<TARGET_FILE_DIR:${PROJECT_NAME}>)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy
	${CMAKE_CURRENT_SOURCE_DIR}/sphinxlib/pocketsphinx.dll 
	$<TARGET_FILE_DIR:${PROJECT_NAME}>)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy
	${CMAKE_CURRENT_SOURCE_DIR}/sphinxlib/sphinxbase.dll 
	$<TARGET_FILE_DIR:${PROJECT_NAME}>)