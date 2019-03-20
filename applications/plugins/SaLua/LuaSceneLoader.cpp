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
#include "LuaSceneLoader.h"
#include <sofa/simulation/Simulation.h>
#include <iostream>
using std::cerr; using std::endl;
#include "LuaBindings.h"
#include "LuaController.h"

namespace sofa {
	namespace simulation {
		/* 'lua' file extension is supported. */
		bool LuaSceneLoader::canLoadFileExtension(const char* extension)
		{
			return strcmp(extension, "lua") == 0 || strcmp(extension, "salua") == 0;
		}
		
		std::string LuaSceneLoader::getFileTypeDesc()
		{
			return "Lua Scripts";
		}
		
		void LuaSceneLoader::getExtensionList(ExtensionList* list)
		{
			list->clear();
			list->push_back("lua");
            list->push_back("salua");
		}

		Node::SPtr LuaSceneLoader::load(const char* filename)
		{
            using sofa::component::controller::LuaController;
			Node::SPtr rootNode;
            
            lua_State *L = luaS_newSaLuaState();

            /* Load and execute the given file */
            int error = (luaL_loadfile(L, filename) || lua_pcall(L, 0, 1, 0));
			if (error)
			{
				cerr << "Lua error: " << lua_tostring(L, -1) << endl;
				lua_pop(L, 1);
			}
			else
			{
                Node* n = dynamic_cast<Node*>(luaS_tosofaobject(L, -1));
                if (n)
                    rootNode.reset(n);
                else
                {
                    lua_getglobal(L, "tostring");
                    lua_insert(L, -2);
                    lua_call(L, 1, 1);
                    cerr << "Lua script did not return a valid root node \n"
                        << "Lua returned: " << lua_tostring(L, -1) << endl;
                }
                lua_pop(L, 1);
            }
            
            /* decrement reference count, if there are no controllers present 
              this line would close the Lua state, otherwise the Lua state 
              will stay alive as long as there are controllers referencing it */
            luaS_release(L);
			return rootNode;
		}

		static SceneLoader* defaultLuaSceneLoader = SceneLoaderFactory::getInstance()->addEntry(new LuaSceneLoader());
	}
}