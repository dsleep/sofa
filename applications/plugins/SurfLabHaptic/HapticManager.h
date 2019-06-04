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
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/core/objectmodel/HapticDeviceEvent.h>

#include <SofaGeneralDeformable/VectorSpringForceField.h>

#include <SofaSimulationTree/GNode.h>
#include <sofa/simulation/Simulation.h>
#include <SofaSimulationTree/TreeSimulation.h>
#include <SofaBaseCollision/BaseContactMapper.h>
#include <SofaConstraint/StickContactConstraint.h>

#include <sofa/core/topology/TopologicalMapping.h>
#include <sofa/helper/gl/template.h>
#include <SofaUserInteraction/TopologicalChangeManager.h>
#include <SofaBaseTopology/TriangleSetTopologyAlgorithms.h>
#include <SofaBaseTopology/EdgeSetTopologyModifier.h>
#include <sofa/helper/AdvancedTimer.h>
#include <SofaLoader/MeshObjLoader.h>
#include <SofaMeshCollision/TriangleModel.h>
#include <SofaMeshCollision/LineModel.h>
#include <SofaMeshCollision/PointModel.h>

#include <SofaDeformable/StiffSpringForceField.h>
#include <SofaDeformable/SpringForceField.h>

#include "GraspingForceFeedback.h"
#include "initSurfLabHaptic.h"
#include <sofa/helper/gl/Capture.h>
//add the 2 libs below to support v12.16
#include <boost/scoped_ptr.hpp>
#include <sofa/core/topology/Topology.h>

//Changes for force feedback safety
#include <SofaHaptics/ForceFeedback.h>
#include "NewOmniDriver.h"
//#include "AAOmniDriver.h"
#include <math.h>

#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <stdio.h>
#include <windows.h>
//#include <ctime>
//#include <chrono>
#include "report.h"
//#include "report.cpp"

namespace sofa
{
    
	namespace component
	{

		namespace collision
		{
			using sofa::defaulttype::Vec3dTypes;

			//Changes for force feedback safety
			class ForceFeedback;

			class SOFA_SURFLABHAPTIC_API HapticManager : public sofa::component::controller::Controller, sofa::core::visual::VisualModel
			{
			public:
				SOFA_CLASS2(HapticManager, sofa::component::controller::Controller, sofa::core::visual::VisualModel);
                
                sofa::helper::gl::Capture capture; // used for capturing screenshots when user make an error

				typedef defaulttype::Vec3Types DataTypes;
				typedef defaulttype::Vec3f Vec3f;
				typedef defaulttype::Vec4f Vec4f;
				typedef defaulttype::RigidTypes RigidTypes;
				typedef defaulttype::Vector3 Vector3;
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
				Data < Vec3f > clampScale;
				sofa::core::objectmodel::DataFileName clampMesh;

				//UF - DS TODO where was this originally
				std::string warnings;

				SingleLink<HapticManager, ToolModel, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> toolModel;
				/* we need a link to the omni driver just so we can get the proper ID */
				SingleLink<HapticManager, sofa::core::behavior::BaseController, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> omniDriver;

                double time_init;
				sofa::gui::qt::SofaProcedureReport* scoring = new sofa::gui::qt::SofaProcedureReport(NULL);
			protected:
				enum ToolFunction {
                    //TOOLFUNCTION_ANIMATE,
					TOOLFUNCTION_SUTURE, 
					TOOLFUNCTION_CARVE,
					TOOLFUNCTION_CLAMP,
					TOOLFUNCTION_GRASP,
					TOOLFUNCTION_CONTAIN
				};
				struct Tool
				{
					ToolModel* modelTool;
					std::set<int> modelGroup;
					sofa::component::constraintset::BilateralInteractionConstraint<DataTypes>::SPtr m_constraints; //for grasp
					sofa::component::interactionforcefield::VectorSpringForceField<DataTypes>::SPtr m_forcefield; //for suture
					sofa::component::interactionforcefield::StiffSpringForceField<Vec3dTypes>::SPtr ff; //for suture
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
				//Changes for force feedback safety
				sofa::component::controller::NewOmniDriver *newOmniDriver;
				//sofa::component::controller::AAOmniDriver *aaOmniDriver;

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
				void initializeStaticDataMembers(){ 
					//std::vector<std::pair<component::topology::Hexahedron, int> > clampPairs;
					std::vector<std::pair<sofa::core::topology::Topology::Hexahedron, int> > clampPairs;				
					std::vector<core::behavior::MechanicalState<DataTypes>*> clipperStates;
					};
				static std::string programStartDate; //system time in string format, used for screenshot filepath
				static std::string programCompletionTime;
				static int numOfElementsCutonVeins;
				static int numOfElementsCutonFat;
			private:
				void updateTool();
				void doContain();
				void doGrasp();
				void doCarve();
                void doIncise();
				void startSuture();
				void stopSuture();
				void doSuture();
				void unGrasp();
				void doClamp();
				const ContactVector* getContacts();
                //double mistake_time;
				double start_time;
				double delta_time;
				// the following variables used in clamping			
				boost::scoped_ptr<sofa::helper::io::Mesh> clipperMesh;				
				static std::vector<std::pair<sofa::core::topology::Topology::Hexahedron, int> > clampPairs;
				static std::vector<core::behavior::MechanicalState<DataTypes>*> clipperStates;
				static std::vector<double> hexDimensions;
				static std::vector<bool> edge12along; // if edge 12 is along vessel
				static std::map <std::string, std::vector<int>> vein_clips_map;//maps the name of vein to a vector of indices of clips
				static std::set<int> veinCutSet;
				static std::set<std::string> namesOfVeinCutSet;
				static bool hasPutInBag;
				static bool hasCutVein;

				
				//updateShader is used for replace a string in shader file, it will replace 
				//the searchstring from the input file to be the replacestring of the output file
				int updateShader(std::string Input, std::string Output, std::string searchstring, std::string replacestring);
				static std::string base_path_share ;
				bool hasInstrumentTurnedRed = false;
				bool hasInstrumentTurnedGreen = false;
				static double last_update_time;//last time the shader has been updated
				static int last_clips_count;//not used
				static int achievementsCount;
				int hasBeenCut(std::string name);//check if a collision model has been cut or not, return 1 for yes, 0 for no.
				
			};

		} // namespace collision

	} // namespace component

} // namespace sofa

// initialize static data members
using namespace sofa::component::collision;
using namespace sofa::component::topology;
using namespace sofa::core::behavior;
using namespace sofa::simulation;
using sofa::simulation::getSimulation;

std::vector<std::pair<sofa::core::topology::Topology::Hexahedron, int> > HapticManager::clampPairs;
std::vector<MechanicalState<HapticManager::DataTypes>*> HapticManager::clipperStates;
std::vector<double> HapticManager::hexDimensions;
std::vector<bool> HapticManager::edge12along;
//std::vector<int> HapticManager::clipVector;
std::map< std::string, std::vector<int> > HapticManager::vein_clips_map;
std::set<int> HapticManager::veinCutSet;
std::set<std::string> HapticManager::namesOfVeinCutSet;
double HapticManager::last_update_time;
int HapticManager::last_clips_count = 0;
int HapticManager::achievementsCount = 0;
std::string HapticManager::programStartDate= "";
std::string HapticManager::programCompletionTime = "";
std::string HapticManager::base_path_share = "";
int HapticManager::numOfElementsCutonVeins = 0;
int HapticManager::numOfElementsCutonFat = 0;
bool HapticManager::hasPutInBag = false;
bool HapticManager::hasCutVein = false;

#endif
