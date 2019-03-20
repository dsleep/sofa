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
#include <SofaUserInteraction/Controller.h>
#include <sofa/core/objectmodel/Context.h>
#include <sofa/core/objectmodel/BaseObjectDescription.h>
#include <sofa/simulation/Node.h>

extern "C"
{
#include "lua.h"
#include "lauxlib.h"
#include "lualib.h"
}


namespace sofa {
	namespace component {
		namespace controller {
			
            /*!
               Allow Lua functions to be registered as event handlers.

               Supports following event handlers (as defined in Lua):
                 - onLoad : called at init
                 - onKeyPress
                 - onKeyRelease
                 - onHaptic
                 - onAnimationBeginStep
                 - onAnimationEndStep

               This controller can be instantiated in XML and the Lua
               source code can be referenced in the "source" property.
               The source code then needs to set handlers as properties
               of the globally defined "handlers" table. The controller itself
               is accessible as a global variable "controller". 

               Alternatively, the controller can be created from Lua code
               using sofa.newController function that takes a table of handlers.

               In both cases, the Lua state is reference counted and the controller
               only calls release on the Lua state. Thus, a Lua state can be shared
               by many controllers.
            */
			class SOFA_SaLua_API LuaController : public Controller
			{
			public:
				SOFA_CLASS(LuaController, Controller);

                LuaController();

                static LuaController* createFromLua(struct lua_State* L, int idx);
                virtual ~LuaController();
				virtual void init();
				virtual void bwdInit();

                /* Events */
                virtual void onKeyPressedEvent(sofa::core::objectmodel::KeypressedEvent*);
                virtual void onKeyReleasedEvent(sofa::core::objectmodel::KeyreleasedEvent*);
                virtual void onHapticDeviceEvent(sofa::core::objectmodel::HapticDeviceEvent*);
                virtual void onBeginAnimationStep(const double);
                virtual void onEndAnimationStep(const double);
                virtual void handleEvent(sofa::core::objectmodel::Event*);

            private:
                void callEventHandler(sofa::core::objectmodel::Event* e, const char* handlerName, int nargs);

                //! Lua state that created the scene
                struct lua_State* _L;
                sofa::core::objectmodel::DataFileName source;
			};


		}
	}
} // controller :: component :: sofa
