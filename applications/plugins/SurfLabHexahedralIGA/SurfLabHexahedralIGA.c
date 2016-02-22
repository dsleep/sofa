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
#include <sofa/config.h>
#include <stdbool.h>

SOFA_EXPORT_DYNAMIC_LIBRARY void initExternalModule()
{
	static bool first = true;
	if (first)
	{
		first = false;
	}
}

SOFA_EXPORT_DYNAMIC_LIBRARY const char* getModuleName()
{
  return "SurfLabHexahedralIGA";
}

SOFA_EXPORT_DYNAMIC_LIBRARY const char* getModuleVersion()
{
	return "0.1";
}

SOFA_EXPORT_DYNAMIC_LIBRARY const char* getModuleLicense()
{
	return "GPL";
}


SOFA_EXPORT_DYNAMIC_LIBRARY const char* getModuleDescription()
{
	return "IGA plug-in for hexahedral meshes by surflab";
}

SOFA_EXPORT_DYNAMIC_LIBRARY const char* getModuleComponentList()
{
  /// string containing comma seperated names of the classes provided by the plugin
  return "";
}

