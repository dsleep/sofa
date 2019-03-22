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
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_CONTROLLER_GRASPINGFORCEFEEDBACK_H
#define SOFA_COMPONENT_CONTROLLER_GRASPINGFORCEFEEDBACK_H

#include <SofaHaptics/ForceFeedback.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <SofaSimulationTree/GNode.h>
#include <sofa/simulation/Simulation.h>
#include <SofaSimulationTree/TreeSimulation.h>

namespace sofa
{

	namespace component
	{

		namespace controller
		{
			class GraspingForceFeedback : public sofa::component::controller::ForceFeedback
			{
			public:
				SOFA_CLASS(GraspingForceFeedback, sofa::component::controller::ForceFeedback);
				
				typedef defaulttype::Vec3Types DataTypes;
				typedef DataTypes::VecDeriv VecDeriv;
				typedef DataTypes::Real Real;
				GraspingForceFeedback(core::behavior::MechanicalState<DataTypes>* st, Real sc)
				{
					state = st;
					scale = sc;
				}

				~GraspingForceFeedback(){}

				virtual void init();
							
				virtual void computeForce(SReal x, SReal y, SReal z, SReal u, SReal v, SReal w, SReal q, SReal& fx, SReal& fy, SReal& fz);
				virtual void computeWrench(const defaulttype::SolidTypes<SReal>::Transform &world_H_tool, const defaulttype::SolidTypes<SReal>::SpatialVector &V_tool_world, defaulttype::SolidTypes<SReal>::SpatialVector &W_tool_world);

			protected:
				sofa::defaulttype::Vec3d contactForce;
				Real scale;
				core::behavior::MechanicalState<DataTypes>* state;

			};

		}
	}
}

#endif //SOFA_COMPONENT_CONTROLLER_ENSLAVEMENTFORCEFEEDBACK_H