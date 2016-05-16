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
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_CONSTRAINTSET_CONNECTINGTISSUE_H
#define SOFA_COMPONENT_CONSTRAINTSET_CONNECTINGTISSUE_H

#include "initConnectingTissue.h"
#include <sofa/core/behavior/BaseController.h>
#include <SofaOpenglVisual/OglModel.h>
#include <SofaUserInteraction/Controller.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/tree/GNode.h>
#include <sofa/simulation/common/Simulation.h>
#include <SofaBaseMechanics/BarycentricMapping.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/ObjectFactory.h>
#include <SofaDeformable/VectorSpringForceField.h>
#include <SofaConstraint/BilateralInteractionConstraint.h>
namespace sofa
{

	namespace component
	{

		namespace constraintset
		{

			class SOFA_CONNECTINGTISSUE_API ConnectingTissue : public sofa::component::controller::Controller, sofa::core::visual::VisualModel
			{
			public:
				SOFA_CLASS2(ConnectingTissue, sofa::component::controller::Controller, sofa::core::visual::VisualModel);

				typedef defaulttype::Vec3Types DataTypes;
				typedef defaulttype::Vec3d Vec3d;
				typedef defaulttype::Vec4d Vec4d;
				typedef defaulttype::RigidTypes RigidTypes;
				typedef DataTypes::Coord Coord;
				typedef DataTypes::VecCoord VecCoord;
				typedef DataTypes::Real Real;
				typedef DataTypes::Deriv Deriv;
				typedef core::behavior::MechanicalState<DataTypes> MechanicalModel;				
				typedef sofa::component::mapping::BarycentricMapping< DataTypes, DataTypes > MMapping;
				typedef sofa::component::mapping::TopologyBarycentricMapper<DataTypes, DataTypes> MMapper;
				typedef sofa::component::interactionforcefield::VectorSpringForceField<DataTypes> TSpringFF;
				typedef sofa::component::constraintset::BilateralInteractionConstraint<DataTypes> TConstraint;

				Data<helper::vector<unsigned int> > m_indices1;
				Data<helper::vector<unsigned int> > m_indices2;
				Data<Real> threshold;
				Data<Real> connectingStiffness;
				Data<bool> useConstraint;
				SingleLink<ConnectingTissue, simulation::Node, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> object1;
				SingleLink<ConnectingTissue, simulation::Node, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> object2;
				
			protected:
				ConnectingTissue();
				virtual ~ConnectingTissue();
			public:
				virtual void init();
				virtual void bwdInit();
				virtual void reset();
				virtual void handleEvent(sofa::core::objectmodel::Event* event);
				virtual void onHapticDeviceEvent(sofa::core::objectmodel::HapticDeviceEvent* ev);
				virtual void onEndAnimationStep(const double dt);
				void drawVisual(const core::visual::VisualParams* vparams);
				void updateVisual();
				
			private:
							
				VecCoord projPnts;
				
			};

		} // namespace collision

	} // namespace component

} // namespace sofa

#endif
