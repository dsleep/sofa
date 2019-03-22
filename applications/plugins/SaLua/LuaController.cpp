/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Plugins                               *
*                                                                             *
* Authors: Saleh Dindar                                                       *
*                                                                             *
* Contact information: saleh@cise.ufl.edu                                     *
******************************************************************************/
#include "salua.h"
#include "LuaController.h"
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>
#include <sofa/core/objectmodel/HapticDeviceEvent.h>
#include <sofa/helper/system/FileRepository.h>


#include "LuaBindings.h"

namespace sofa { 
	namespace component {
        namespace controller {

          
            SOFA_DECL_CLASS(LuaController);

            int registerLuaController = core::RegisterObject("Load a Lua script to use as a controller").add<LuaController>();

            LuaController::LuaController(): Controller(), _L(NULL)
                , source(initData(&source, "source", "Lua source code for handlers")) {}

            LuaController* LuaController::createFromLua(lua_State* L, int idx)
            {
                LuaController* c = new LuaController();
                c->_L = L;
                luaS_retain(L);
                /* at the idx in the Lua stack, there is the table of
                event handlers */
                if (lua_istable(L, idx))
                {
                    lua_pushvalue(L, idx);
                    lua_rawsetp(L, LUA_REGISTRYINDEX, c);
                    c->f_listening.setValue(true);
                }
                return c;
            }

            
            LuaController::~LuaController()
            {
                if (_L) luaS_release(_L);
            }

            /* Find the controller table and cache it in the registry */
            void LuaController::init()
            {
				std::cout << "in LuaController init()" << std::endl;
                Controller::init();

                /* if we are created without any Lua state, it probably means
                   that we are created from XML and we are going to maintain 
                   our own Lua state 
                   */
                if (_L == NULL) _L = luaS_newSaLuaState();
                
                std::string fileName = source.getFullPath();
                //if (sofa::helper::system::DataRepository.findFile(fileName, "", &serr))
				if (sofa::helper::system::DataRepository.findFile(fileName, "", &std::cerr))//changed from &serr to this &std::cerr, to make it work in v12.16
                {
                    /* create the table of handlers for the dofile
                       to fill out, we also keep this in the registry
                       at the proper pointer */
					std::cout << "lua reads source: " << fileName << std::endl;
                    lua_newtable(_L);
                    lua_setglobal(_L, "handlers");
                    luaS_pushsofaobject(_L, (BaseObject*)this);
                    lua_setglobal(_L, "controller");
                    
                    int error = (luaL_loadfile(_L, fileName.c_str()) || lua_pcall(_L, 0, 0, 0));
                    if (error)
                    {
						std::cerr << "Lua error: " << lua_tostring(_L, -1) << sendl;////changed from serr to this std::cerr, to make it work in v12.16
                        lua_pop(_L, 1);
                    }
                    lua_getglobal(_L, "handlers");
                    lua_rawsetp(_L, LUA_REGISTRYINDEX, this);
                }
            }

            void LuaController::bwdInit()
            {
                Controller::bwdInit();
                if (f_listening.getValue())
                    callEventHandler(NULL, "onLoad", 0);
            }

            /* Call the event handler with specified number of args in Lua

               Check if such a handler exists and call the handler, if the
               handler returns true then set the event as handled

               @param e  the event to be processed, we only use e->setHandled()
               @param handleName Lua name for the handler function
               @param nargs Number of arguments to pass to the handler function, the arguments
               should already has been pushed to the stack
               */
            void LuaController::callEventHandler(sofa::core::objectmodel::Event* e, const char* handlerName, int nargs)
            {
                /* where the arguments are located */
                int argidx = lua_gettop(_L) - nargs;

                lua_rawgetp(_L, LUA_REGISTRYINDEX, this);
                if (lua_getfield(_L, -1, handlerName) == LUA_TFUNCTION)
                {
                    /* push the arguments after the function */
                    for (int i = 1; i <= nargs; i++)
                        lua_pushvalue(_L, argidx + i);

                    if (lua_pcall(_L, nargs, 1, 0) == LUA_OK)
                    {
                        if (e != NULL && lua_toboolean(_L, -1))
                            e->setHandled();
                    }
                    else
                        sout << "Lua error: " << lua_tostring(_L, -1) << sendl;
                }
                /* we made a copy of arguments, we have to release them here
                   plus the registry table, 
                   plus the nil if function does not exists or return value of function or error value
                */
                lua_pop(_L, 2 + nargs);
            }

            void LuaController::handleEvent(sofa::core::objectmodel::Event* e)
            {
                Controller::handleEvent(e);
            }

            /* Call the lua controller defined as
            function sofa.controller.onKeyPressedEvent(key)
            */
            void LuaController::onKeyPressedEvent(sofa::core::objectmodel::KeypressedEvent* e)
            {
                char c[1] = { e->getKey() };
                lua_pushlstring(_L, c, 1);
                callEventHandler(e, "onKeyPressed", 1);
            }

            /* Call the lua controller defined as
            function sofa.controller.onKeyReleasedEvent(key)
            */
            void LuaController::onKeyReleasedEvent(sofa::core::objectmodel::KeyreleasedEvent* e)
            {
                char c[1] = { e->getKey() };
                lua_pushlstring(_L, c, 1);
                callEventHandler(e, "onKeyReleased", 1);
            }

            /* Call a lua function with prototype like:  
                function sofa.controller.onHaptic(deviceID, buttonState, devicePosition) 
            */
            void LuaController::onHapticDeviceEvent(sofa::core::objectmodel::HapticDeviceEvent* e)
            {
                lua_pushinteger(_L, e->getDeviceId());
                lua_pushinteger(_L, e->getButtonState());
                /* Put the position as a vector */
                sofa::defaulttype::Vector3 pos = e->getPosition();
                lua_newtable(_L);
                for (int i = 0; i < 3; i++)
                {
                    lua_pushnumber(_L, pos.at(i));
                    lua_rawseti(_L, -2, i + 1);
                }
                callEventHandler(e, "onHaptic", 3);
            }

            /* Call the lua controller defined as 
               function sofa.controller.onBeginAnimationStep(dt)
            */
            void LuaController::onBeginAnimationStep(const double dt)
            {
                lua_pushnumber(_L, dt);
                callEventHandler(NULL, "onBeginAnimationStep", 1);
            }

            /* Call the lua controller defined as
            function sofa.controller.onEndAnimationStep(dt)
            */
            void LuaController::onEndAnimationStep(const double dt)
            {
                lua_pushnumber(_L, dt);
                callEventHandler(NULL, "onEndAnimationStep", 1);
            }

		}
	}
} // controller :: component :: sofa
