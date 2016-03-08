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
#ifndef SOFA_COMPONENT_COLLISION_HAPTICMANAGER_H
#define SOFA_COMPONENT_COLLISION_HAPTICMANAGER_H

#include "initSofaSuturing.h"
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/collision/Intersection.h>
#include <sofa/core/collision/NarrowPhaseDetection.h>
#include <sofa/core/CollisionModel.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/Event.h>
#include <sofa/defaulttype/Vec3Types.h>
#include <SofaUserInteraction/Controller.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/collision/DetectionOutput.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>
#include <sofa/core/objectmodel/MouseEvent.h>
#include <sofa/simulation/common/AnimateBeginEvent.h>
#include <sofa/simulation/common/AnimateEndEvent.h>
#include <sofa/core/objectmodel/HapticDeviceEvent.h>

#include <sofa/simulation/tree/GNode.h>
#include <sofa/simulation/common/Simulation.h>
#include <sofa/simulation/tree/TreeSimulation.h>
#include <SofaBaseCollision/BaseContactMapper.h>
#include <sofa/component/typedef/Sofa_typedef.h>
#include <SofaConstraint/StickContactConstraint.h>

#include "GraspingForceFeedback.h"
namespace sofa
{

	namespace component
	{

		namespace collision
		{

			class SOFA_SOFASUTURING_API HapticManager : public sofa::component::controller::Controller, sofa::core::visual::VisualModel
			{
			public:
				SOFA_CLASS2(HapticManager, sofa::component::controller::Controller, sofa::core::visual::VisualModel);

				typedef defaulttype::Vec3Types DataTypes;
				typedef DataTypes::Coord Coord;
				typedef DataTypes::VecCoord VecCoord;
				typedef DataTypes::Real Real;
				typedef DataTypes::Deriv Deriv;
				
				typedef core::CollisionModel ToolModel;
				typedef helper::vector<core::collision::DetectionOutput> ContactVector;
				typedef sofa::component::collision::BaseContactMapper< DataTypes > ContactMapper;

				Data < Real > grasp_stiffness;
				Data < Real > attach_stiffness;
				Data < Real > grasp_forcescale;
				Data < Real > duration;
				SingleLink<HapticManager, ToolModel, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> toolModel;
				/* we need a link to the omni driver just so we can get the proper ID */
				SingleLink<HapticManager, sofa::core::behavior::BaseController, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> omniDriver;

			protected:
				enum ToolFunction {
					TOOLFUNCTION_SUTURE, 
					TOOLFUNCTION_CARVE,
					TOOLFUNCTION_GRASP
				};
				struct Tool
				{
					ToolModel* modelTool;
					helper::set<int> modelGroup;
					sofa::component::constraintset::BilateralInteractionConstraint<DataTypes>::SPtr m_constraints;
					sofa::component::interactionforcefield::VectorSpringForceField<DataTypes>::SPtr m_forcefield;
					StiffSpringForceField3::SPtr ff;
					ContactMapper* m1;
					ContactMapper* m2;
					/* First button is for grasping, second button is for Haptic */
					unsigned char buttonState, newButtonState;
					/* What does the tool do when the button is pressed */
					ToolFunction function;
					bool first;
					std::vector<int> first_idx;
					std::vector<Vector3> first_point;
					int id;
				} toolState;

				std::vector<core::CollisionModel*> modelSurfaces;
				core::collision::Intersection* intersectionMethod;
				core::collision::NarrowPhaseDetection* detectionNP;
				//sofa::component::topology::TriangleSetTopologyContainer* mesh;
				
				HapticManager();

				virtual ~HapticManager();
			public:
				virtual void init();
				virtual void reset();
				virtual void handleEvent(sofa::core::objectmodel::Event* event);
				virtual void onHapticDeviceEvent(sofa::core::objectmodel::HapticDeviceEvent* ev);
				virtual void onEndAnimationStep(const double dt);
				void drawVisual(const core::visual::VisualParams* vparams);
				void updateVisual();
			private:
				void updateTool();
				void doGrasp();
				void doCarve();
				void startSuture();
				void stopSuture();
				void doSuture();
				void unGrasp();
				void updateBoundingBoxes();
				const ContactVector* getContacts();
				double start_time;
				double delta_time;
				
			};

		} // namespace collision

	} // namespace component

} // namespace sofa

#endif
