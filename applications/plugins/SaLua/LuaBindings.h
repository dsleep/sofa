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

extern "C" {
#include "lua.h"
#include "lauxlib.h"
#include "lualib.h"
}

#include <sofa/core/objectmodel/Base.h>

/*
  the function with luaS_ prefix are 
  my extensions to the Lua frame work. 
  Just like luaL_ is auxiliary extensions
  to lua
*/
void loadSOFAlib(lua_State*);
template<class T> void luaS_pushsofaobject(lua_State*L, T* o);
sofa::core::objectmodel::Base* luaS_tosofaobject(lua_State*L, int idx);

lua_State* luaS_newSaLuaState();
/* increment reference count for the Lua state, the 
   reference count is stored in the Lua registry */
void luaS_retain(lua_State *L);
/* decrement reference count in the Lua state, the 
  reference count is stored in the Lua registry, when
   the count reaches zero the Lua state is closed 
   */
void luaS_release(lua_State *L);


