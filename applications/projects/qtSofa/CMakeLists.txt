cmake_minimum_required(VERSION 3.1)
project(qtSofa)

set(SOURCE_FILES
	QSofaMainWindow.cpp
	QSofaScene.cpp
	QSofaViewer.cpp
	qtSofa.cpp
)

set(HEADER_FILES
	QSofaMainWindow.h
	QSofaScene.h
    QSofaViewer.h
    oneTetra.h
)

find_package(SofaSimpleGUI REQUIRED)

find_package(Qt5 COMPONENTS Core Gui Widgets OpenGL REQUIRED)



if(Qt5Core_FOUND)
  qt5_wrap_cpp(MOC_FILES ${HEADER_FILES})
  qt5_add_resources(RESOURCE_FILES "./icons.qrc")
  set(QT_DEPS Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL)
else()
  find_package(Qt4)
  qt4_wrap_cpp(MOC_FILES ${HEADER_FILES})
  qt4_add_resources(RESOURCE_FILES "./icons.qrc")
endif()

set(RESOURCE_FILES_GROUP "Generated")
set(MOC_FILES_GROUP "Generated")

add_executable(${PROJECT_NAME} ${HEADER_FILES} ${MOC_FILES} ${SOURCE_FILES} ${RESOURCE_FILES})
target_link_libraries(${PROJECT_NAME} SofaSimpleGUI SofaOpenglVisual ${GLUT_LIBRARIES})
compile_definitions("-DQTSOFA_SRC_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")
