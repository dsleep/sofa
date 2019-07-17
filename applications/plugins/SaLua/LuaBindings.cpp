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
#include "LuaBindings.h"
#include "LuaController.h"
#include <sofa/simulation/Simulation.h>
#include <sofa/core/ObjectFactory.h>

using namespace sofa::simulation;
using namespace sofa::core::objectmodel;
typedef sofa::core::objectmodel::Base SofaBase;

const char* SOFAUSERDATA = "SOFAObject";

/*!
  This is how we store a SOFA object in Lua
  we keep the pointer, but we also keep 
  a hash code of its type retrieved by calling
  typeid(T).hash_code() 
  The hash code is used to look up method table
  in the registry
*/
struct SofaUserDataObject {
    SofaBase* object;
    const void* registryKey;
};


template<class T>
const void* registryKeyFor()
{
    return (const void*) typeid(T).hash_code();
}

template<class T>
void luaS_pushsofaobject(lua_State*L, T* o)
{
	SofaUserDataObject* n = (SofaUserDataObject*) lua_newuserdata(L, sizeof(SofaUserDataObject));
    luaL_getmetatable(L, SOFAUSERDATA);
	lua_setmetatable(L, -2);
    n->object = o;
    n->registryKey = registryKeyFor<T>();
	intrusive_ptr_add_ref(o);
}

template<class T>
T* toSOFAObject(lua_State*L, int idx)
{
    luaL_checkudata(L, idx, SOFAUSERDATA);
	SofaUserDataObject* n = reinterpret_cast<SofaUserDataObject*>(lua_touserdata(L, idx));
	T* t = dynamic_cast<T*>(n->object);
    if (t == NULL)
        luaL_error(L, "Argument %d is expected to be %s\n", idx, typeid(T).name());
    return t;
}

SofaUserDataObject* toSOFAUserDataObject(lua_State*L, int idx)
{
    luaL_checkudata(L, idx, SOFAUSERDATA);
    return reinterpret_cast<SofaUserDataObject*>(lua_touserdata(L, idx));
}


SofaBase* luaS_tosofaobject(lua_State*L, int idx)
{
    SofaUserDataObject* n = reinterpret_cast<SofaUserDataObject*>(luaL_testudata(L, idx, SOFAUSERDATA));
    return n ? n->object : NULL;
}


/* Create a new root node for a graph */
int Simulation_newGraph(lua_State* L)
{
    Simulation* sim = toSOFAObject<Simulation>(L, 1);
    const char* s = lua_tostring(L, 2);
    Node::SPtr n = sim->createNewGraph(s);
	luaS_pushsofaobject(L, n.get());
	return 1;
}

/* Create a new node */
int Simulation_newNode(lua_State* L)
{
    Simulation* sim = toSOFAObject<Simulation>(L, 1);
	const char* s = lua_tostring(L, 2);
	Node::SPtr n = sim->createNewNode(s);
	luaS_pushsofaobject(L, n.get());
	return 1;
}

/* Create a new node */
int Node_newChild(lua_State* L)
{
    Node* p = toSOFAObject<Node>(L, 1);
    const char* s = lua_tostring(L, 2);
    Node::SPtr n = p->createChild(s);
    luaS_pushsofaobject(L, n.get());
    return 1;
}

    
/*!
  Create a new object in the context of the specified node
  @param node the context node in which the object will be created
  @param desc a table describing all the arguments for creation of the object
  the argument at position 0 is used as the 
*/
int Node_newObject(lua_State* L)
{
	Node* ctx = toSOFAObject<Node>(L, 1);
    BaseObjectDescription desc;
    /* Fill in the desc */
    if (lua_isstring(L, 2))
    {
        /* Just the class name set it as the type */
        desc.setAttribute("type", lua_tostring(L, 2));
    }
    else if (lua_istable(L, 2))
    {
        /* A table, the element at point 1 is the type, everything else is string = string pairs */
        lua_pushnil(L);
        while (lua_next(L, -2))
        {
            /* -2 is the key, -1 is the value */
            int t = lua_type(L, -2);
            if (t == LUA_TNUMBER && lua_tonumber(L, -2) == 1.0)
            {
                desc.setAttribute("type", lua_tostring(L, -1));
            }
            else if (t == LUA_TSTRING)
            {
                desc.setAttribute(lua_tostring(L, -2), lua_tostring(L, -1));
            }
            else
                luaL_argerror(L, -2, "Specified key is not a valid attribute for SOFA object creation");
            lua_pop(L, 1);
        }
    }
    else
        luaL_argerror(L, 2, "Expected string or table");

    BaseObject::SPtr n = sofa::core::ObjectFactory::CreateObject(ctx, &desc);
   
    if (n == NULL)
    {
		luaL_error(L, "Error creating object: %s", desc.getErrors());
        return 0;
    }
    else
    {
        luaS_pushsofaobject(L, n.get());
        return 1;
    }
}

/*
    
 */
int Node_addChild(lua_State* L)
{
	Node* node = toSOFAObject<Node>(L, 1);
    Node* child = toSOFAObject<Node>(L, 2);
    node->addChild(child);
	return 0;
}
int Node_addObject(lua_State* L)
{
    Node* node = toSOFAObject<Node>(L, 1);
    BaseObject* object = toSOFAObject<BaseObject>(L, 2);
    node->addObject(object);
    return 0;
}



int Node_init(lua_State* L)
{
    Node *o = toSOFAObject<Node>(L, 1);
    o->init(sofa::core::ExecParams::defaultInstance());

	//UF - DS reset the opengl shaders for our nodes
	getSimulation()->initTextures(o);

    return 0;
}

bool totagset(lua_State* L, int idx, TagSet* tags)
{
    if (lua_istable(L, idx))
    {
        size_t N = lua_rawlen(L, idx);
        for (size_t i = 0; i < N; i++)
        {
            if (lua_rawgeti(L, idx, i + 1) == LUA_TSTRING)
                tags->insert(Tag(std::string(lua_tostring(L, -1))));
            lua_pop(L, 1);
        }
        return true;
    }
    else
    {
        luaL_argerror(L, 2, "array of string expected");
        return false;
    }
}


/* 
  arguments:
   self: a Node object in which context the search takes place
   tags: an array of strings that defines the tags we are looking for
   direction: one of 'up', 'down', 'root', ...
 */
int Node_getObjects(lua_State* L)
{
    Node *o = toSOFAObject<Node>(L, 1);
    /* convert array of tags to TagSet */
    TagSet tags;
    totagset(L, 2, &tags);
    const char* dirstr = lua_tostring(L, 3);
    /* convert dirstr to a proper dir */
    BaseContext::SearchDirection dir = BaseContext::SearchUp;
    if (dirstr == NULL || strcmp(dirstr, "up") == 0)
        dir = BaseContext::SearchDirection::SearchUp;
    else if (strcmp(dirstr, "down") == 0)
        dir = BaseContext::SearchDirection::SearchDown;
    else if (strcmp(dirstr, "parents") == 0)
        dir = BaseContext::SearchDirection::SearchParents;
    else if (strcmp(dirstr, "local") == 0)
        dir = BaseContext::SearchDirection::Local;
    else if (strcmp(dirstr, "root") == 0)
        dir = BaseContext::SearchDirection::SearchRoot;
    else
        luaL_argerror(L, 3, "one of 'down', 'up', 'local', 'parents' and 'root' is expected");

    std::vector<BaseObject*> container;
    o->get<BaseObject>(&container, tags, dir);

    lua_createtable(L, container.size(), 0);
    for (size_t i = 0; i < container.size(); i++)
    {
        luaS_pushsofaobject(L, container[i]);
        lua_rawseti(L, -2, i + 1);
    }
    return 1;
}

int Node_getParent(lua_State* L)
{
    Node *n = toSOFAObject<Node>(L, 1);
    sofa::helper::vector<BaseNode*> parents = n->getParents();
    Node *p = parents.size() == 0 ? NULL : dynamic_cast<Node*>(parents[0]);
    if (p != NULL)
        luaS_pushsofaobject(L, p);
    else
        lua_pushnil(L);
    return 1;    
}

void discoverChildNodes(lua_State* L, BaseNode* n, const TagSet& tags)
{
    sofa::helper::vector< BaseNode* > children = n->getChildren();
    for(size_t i = 0; i < children.size(); i++)
    {
        if(children[i]->getTags().includes(tags))
        {
            luaS_pushsofaobject(L, dynamic_cast<Node*>(children[i]));
            lua_rawseti(L, -2, lua_rawlen(L, -2) + 1);
        }
        discoverChildNodes(L, children[i], tags);
    }
}


/* 
  arguments:
  self: a Node object
  tags: a list of tags

  return:
    an array of nodes
*/
int Node_findChildNodes(lua_State* L)
{
    Node * n = toSOFAObject<Node>(L, 1);
    TagSet tags;
    totagset(L, 2, &tags);
    lua_newtable(L);
    discoverChildNodes(L, n, tags);
    return 1;
}

int Object_getContext(lua_State* L)
{
    BaseObject *o = toSOFAObject<BaseObject>(L, 1);
    Node *n = dynamic_cast<Node*>(o->getContext());
    if (n != NULL)
        luaS_pushsofaobject(L, n);
    else
        lua_pushnil(L);
    return 1;
}

int Object_init(lua_State* L)
{
    BaseObject *o = toSOFAObject<BaseObject>(L, 1);
    o->init();
    return 0;
}

int Node_setActive(lua_State* L)
{
    Node *o = toSOFAObject<Node>(L, 1);
    o->setActive(lua_toboolean(L, 2));
    return 0;
}


/* Set field of a sofa Object */
int SofaBase_getfield(lua_State*L)
{
	SofaUserDataObject* ud = toSOFAUserDataObject(L, 1);
    SofaBase* o = ud->object;
    const char* name = lua_tostring(L, 2);

    /* Try to find a method with the name */
    lua_rawgetp(L, LUA_REGISTRYINDEX, ud->registryKey);
    if (lua_getfield(L, -1, name) != LUA_TNIL)
        return 1;

    /* method not found, try to find a property */
    else if (BaseData *d = o->findData(name))
	{
        const sofa::defaulttype::AbstractTypeInfo* ti = d->getValueTypeInfo();
		const void* ptr = d->getValueVoidPtr();
		// For later, we can use nbRows to create arrays of Vec3 or Vec4 or whatever
		// for now we will just flatten the whole thing.
		// int nbRows = ti->size(d->getValueVoidPtr()) / ti->size();
        if (ti->Text())
        {
            std::string str = ti->getTextValue(ptr, 0);
            lua_pushlstring(L, str.data(), str.length());
        }
        else if (ti->Integer() && ti->FixedSize() && ti->size() == 1)
		{
			lua_pushinteger(L, ti->getIntegerValue(ptr, 0));
		}
		else if (ti->Scalar() && ti->FixedSize() && ti->size() == 1)
		{
			lua_pushnumber(L, ti->getScalarValue(ptr, 0));
		}
        else if (ti->Scalar())
		{
			lua_newtable(L);
			for (size_t i = 0; i < ti->size(ptr); i++)
			{
				lua_pushnumber(L, ti->getScalarValue(ptr, i));
				lua_rawseti(L, -2, i+1);
			}
		}
        else if (ti->Integer())
		{
			lua_newtable(L);
			for (size_t i = 0; i < ti->size(ptr); i++)
			{
				lua_pushinteger(L, ti->getIntegerValue(ptr, i));
				lua_rawseti(L, -2, i+1);
			}
		}
		else
			lua_pushstring(L, d->getValueString().c_str());
	}
	else
		luaL_error(L, "object does not have attribute %s ", name);
	return 1;
}

/* Get field of a sofa object */
int SofaBase_setfield(lua_State*L)
{
    SofaUserDataObject* ud = toSOFAUserDataObject(L, 1);
    SofaBase* o = ud->object;
    const char* name = lua_tostring(L, 2);
    
    if (BaseData *d = o->findData(name))
	{
		/* if we get to parse the input correctly it would stay true */
		bool parsed = true;
		const sofa::defaulttype::AbstractTypeInfo* ti = d->getValueTypeInfo();
		void* ptr = d->beginEditVoidPtr();
        if (ti->Text())
        {
            ti->setTextValue(ptr, 0, lua_tostring(L, 3));
        }
        else if (ti->Scalar() && ti->FixedSize() && ti->size() == 1)
		{
			ti->setScalarValue(ptr, 0, lua_tonumber(L, 3));
		}
        else if (ti->Integer() && ti->FixedSize() && ti->size() == 1)
        {
            ti->setIntegerValue(ptr, 0, lua_tointeger(L, 3));
        }
		else if (ti->Integer() && lua_istable(L, 3))
		{
            ti->setSize(ptr, lua_rawlen(L, 3));
            for (size_t i = 0; i < lua_rawlen(L, 3); i++)
			{
				lua_rawgeti(L, 3, i+1);
				ti->setIntegerValue(ptr, i, lua_tointeger(L, -1));
				lua_pop(L, 1);
			}
		}
		else if (ti->Scalar() && lua_istable(L, 3))
		{
            ti->setSize(ptr, lua_rawlen(L, 3));
			for (size_t i = 0; i < lua_rawlen(L, 3); i++)
			{
				lua_rawgeti(L, 3, i+1);
				ti->setScalarValue(ptr, i, lua_tonumber(L, -1));
				lua_pop(L, 1);
			}
		}
		else
			parsed = false;

		d->endEditVoidPtr();

		if (!parsed)
		{
			if (!(lua_isstring(L, 3) && d->read(lua_tostring(L, 3))))
				luaL_argerror(L, 3, "Cannot convert the value to the attribute type");
		}
	}
	else
		luaL_error(L, "object does not have attribute %s ", name);
	return 0;
}

int SofaBase_free(lua_State* L)
{
    SofaUserDataObject* ud = toSOFAUserDataObject(L, 1);
    SofaBase* o = ud->object;
    intrusive_ptr_release(o);
	return 0;
}

using sofa::component::controller::LuaController;

int Sofa_newController(lua_State* L)
{
    LuaController* c = LuaController::createFromLua(L, 1);
    luaS_pushsofaobject(L, (BaseObject*) c);
    return 1;
}

int SofaBase_tostring(lua_State* L)
{
    SofaUserDataObject* ud = toSOFAUserDataObject(L, 1);
    SofaBase*o = ud->object;
    std::string str = "<SOFA " + o->getClassName() + ": " + o->getName() + " >";
    lua_pushstring(L, str.c_str());
    return 1;
}

int SofaBase_className(lua_State* L)
{
    SofaUserDataObject* ud = toSOFAUserDataObject(L, 1);
    SofaBase*o = ud->object;
    lua_pushstring(L, o->getClassName().c_str());
    return 1;
}



static const struct luaL_Reg SofaBase_metamethods[] = {
    { "__index", SofaBase_getfield },
    { "__newindex", SofaBase_setfield },
    { "__gc", SofaBase_free },
    { "__tostring", SofaBase_tostring },
    { NULL, NULL }
};


static const struct luaL_Reg SofaBase_methods[] = {
    { "className", SofaBase_className },
    { NULL, NULL }
};


static const struct luaL_Reg BaseObject_methods[] = {
    { "init", Object_init },
    { "getContext", Object_getContext },
    { "className", SofaBase_className },
    { NULL, NULL }
};

static const struct luaL_Reg Node_methods[] = {
    { "init", Node_init },
    { "addChild", Node_addChild },
    { "addObject", Node_addObject },
    { "newObject", Node_newObject },
    { "newChild", Node_newChild },
    { "setActive", Node_setActive },
    { "getObjects", Node_getObjects },
    { "className", SofaBase_className },
    { "findChildNodes", Node_findChildNodes },
    { NULL, NULL }
};

static const struct luaL_Reg Simulation_methods[] = {
    { "newGraph", Simulation_newGraph },
    { "newNode", Simulation_newNode },
    { "className", SofaBase_className },
    { NULL, NULL }
};

static const struct luaL_Reg Sofa_functions[] = {
    { "newController", Sofa_newController },
    { NULL, NULL }
};

/* Load all the root functions into `sofa' table
and register sofa table in global state */
void loadSOFAlib(lua_State*L)
{
    /* Create the metatable and set the metamethods */
    luaL_newmetatable(L, SOFAUSERDATA);
	luaL_setfuncs(L, SofaBase_metamethods, 0);
    lua_pop(L, 1);

    /* Register the methods table in the registry */
    luaL_newlib(L, SofaBase_methods);
    lua_rawsetp(L, LUA_REGISTRYINDEX, registryKeyFor<SofaBase>());

    luaL_newlib(L, Node_methods);
    lua_rawsetp(L, LUA_REGISTRYINDEX, registryKeyFor<Node>());

    luaL_newlib(L, BaseObject_methods);
    lua_rawsetp(L, LUA_REGISTRYINDEX, registryKeyFor<BaseObject>());

    luaL_newlib(L, Simulation_methods);
    lua_rawsetp(L, LUA_REGISTRYINDEX, registryKeyFor<Simulation>());

	luaL_newlib(L, Sofa_functions);
    luaS_pushsofaobject(L, getSimulation());
    lua_setfield(L, -2, "simulation");
	lua_setglobal(L, "sofa");
}


/*

We reference count the Lua state. We will just have
a label called "reference-count".

*/
const char* referenceCountLabel = "reference-count";

void luaS_retain(lua_State *L)
{
    lua_getfield(L, LUA_REGISTRYINDEX, referenceCountLabel);
    int refcount = lua_tonumber(L, -1);
    lua_pop(L, 1);
    lua_pushnumber(L, refcount + 1);
    lua_setfield(L, LUA_REGISTRYINDEX, referenceCountLabel);
}

void luaS_release(lua_State *L)
{
    lua_getfield(L, LUA_REGISTRYINDEX, referenceCountLabel);
    int refcount = lua_tonumber(L, -1);
    lua_pop(L, 1);
    if (refcount > 0)
    {
        lua_pushnumber(L, refcount - 1);
        lua_setfield(L, LUA_REGISTRYINDEX, referenceCountLabel);
    }
    else
        lua_close(L);
}

lua_State* luaS_newSaLuaState()
{
    /* Create the Lua state with the reference count of 1 */
    lua_State *L = luaL_newstate();
    luaS_retain(L);
    luaL_openlibs(L);

    loadSOFAlib(L);
    return L;
}
