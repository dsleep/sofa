cmake_minimum_required(VERSION 2.8.12)
project(SaLua)

if((${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU") OR (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang"))
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

find_package(SofaGeneral REQUIRED)

find_package(PkgConfig)
pkg_check_modules(Lua QUIET lua5.3)

if(${Lua_FOUND})
  message("Lua was found " ${Lua_VERSION})
  add_library(lua SHARED IMPORTED)
  set_target_properties(lua PROPERTIES
  	INTERFACE_INCLUDE_DIRECTORIES ${Lua_INCLUDE_DIRS}
  	INTERFACE_LINK_LIBRARIES ${Lua_LIBRARIES})
else()
    set(LUA_SRC_DIR "" CACHE PATH "Path to Lua 5.3 source files")

    set(LUA_INCLUDE_FILES
            ${LUA_SRC_DIR}/lua.h
            ${LUA_SRC_DIR}/lauxlib.h
            ${LUA_SRC_DIR}/lualib.h
            )
    set(LUA_SOURCE_FILES
            ${LUA_SRC_DIR}/lapi.c
            ${LUA_SRC_DIR}/lcode.c
            ${LUA_SRC_DIR}/lctype.c
            ${LUA_SRC_DIR}/ldebug.c
            ${LUA_SRC_DIR}/ldo.c
            ${LUA_SRC_DIR}/ldump.c
            ${LUA_SRC_DIR}/lfunc.c
            ${LUA_SRC_DIR}/lgc.c
            ${LUA_SRC_DIR}/llex.c
            ${LUA_SRC_DIR}/lmem.c
            ${LUA_SRC_DIR}/lobject.c
            ${LUA_SRC_DIR}/lopcodes.c
            ${LUA_SRC_DIR}/lparser.c
            ${LUA_SRC_DIR}/lstate.c
            ${LUA_SRC_DIR}/lstate.c
            ${LUA_SRC_DIR}/lstring.c
            ${LUA_SRC_DIR}/ltable.c
            ${LUA_SRC_DIR}/ltm.c
            ${LUA_SRC_DIR}/lundump.c
            ${LUA_SRC_DIR}/lvm.c
            ${LUA_SRC_DIR}/lzio.c
            ${LUA_SRC_DIR}/lauxlib.c
            ${LUA_SRC_DIR}/lbaselib.c
            ${LUA_SRC_DIR}/lbitlib.c
            ${LUA_SRC_DIR}/lcorolib.c
            ${LUA_SRC_DIR}/ldblib.c
            ${LUA_SRC_DIR}/liolib.c
            ${LUA_SRC_DIR}/lmathlib.c
            ${LUA_SRC_DIR}/loslib.c
            ${LUA_SRC_DIR}/lstrlib.c
            ${LUA_SRC_DIR}/ltablib.c
            ${LUA_SRC_DIR}/lutf8lib.c
            ${LUA_SRC_DIR}/loadlib.c
            ${LUA_SRC_DIR}/linit.c
            )
  

    include_directories(${LUA_SRC_DIR})
    add_library(lua SHARED ${LUA_HEADER_FILES} ${LUA_SOURCE_FILES})
    set_target_properties(lua PROPERTIES COMPILE_FLAGS "-DLUA_BUILD_AS_DLL")
endif()

set(HEADER_FILES salua.h LuaController.h LuaSceneLoader.h LuaBindings.h)
set(SOURCE_FILES salua.cpp LuaController.cpp LuaSceneLoader.cpp LuaBindings.cpp)
set(README_FILES salua.txt)
include_directories(${LUA_SRC_DIR})
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_SALUA")
target_link_libraries(${PROJECT_NAME} lua SofaCore SofaComponentGeneral SofaSimulationCommon)

install(TARGETS ${PROJECT_NAME} lua
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

