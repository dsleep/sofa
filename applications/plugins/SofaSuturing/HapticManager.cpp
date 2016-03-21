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
#include "HapticManager.h"

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
#define int2string(a) std::to_string(a)
namespace sofa
{

	namespace component
	{

		namespace collision
		{

			using sofa::component::controller::Controller;
			using namespace sofa::core::objectmodel;

			SOFA_DECL_CLASS(HapticManager)

				int HapticManagerClass = core::RegisterObject("Manager handling Haptic operations between objects using haptic tools.")
				.add< HapticManager >()
				;


			HapticManager::HapticManager()
				: grasp_stiffness(initData(&grasp_stiffness, 10000.0, "graspStiffness", "how stiff surface is attached to the tool"))
				, attach_stiffness(initData(&attach_stiffness, 10000000.0, "attachStiffness", "how stiff surface is attached together"))
				, grasp_forcescale(initData(&grasp_forcescale, 0.001, "grasp_force_scale", "multiply the force with this coefficient"))
				, duration(initData(&duration, 10.0, "duration", "multiply the force with this coefficient"))
				, intersectionMethod(NULL)
				, detectionNP(NULL)
				, toolModel(initLink("toolModel", "Tool model that is used for grasping and Haptic"))
				, omniDriver(initLink("omniDriver", "NewOmniDriver tag that corresponds to this tool"))
				, upperJaw(initLink("upperJaw", "Collision model for the upper jaw used as clamping tool"))
				, lowerJaw(initLink("lowerJaw", "Collision model for the lower jaw used as clamping tool"))
				, clampScale(initData(&clampScale, Vec3f(0.05, 0.2, 0.2), "clampScale", "scale of the object created during clamping"))
			{
				this->f_listening.setValue(true);
			}

			HapticManager::~HapticManager()
			{
			}


			void HapticManager::init()
			{
				if (toolModel) {
					ToolModel *tm = toolModel.get();

					toolState.buttonState = 0;
					toolState.newButtonState = 0;
					toolState.id = 0;
					toolState.modelTool = tm;
					toolState.modelGroup = tm->getGroups();
					if (tm->hasTag(core::objectmodel::Tag("CarvingTool")))
						toolState.function = TOOLFUNCTION_CARVE;
					else if (tm->hasTag(core::objectmodel::Tag("SuturingTool")))
						toolState.function = TOOLFUNCTION_SUTURE;
					else
						toolState.function = TOOLFUNCTION_GRASP;
					sout << "tool is found " << sendl;
				} else if (upperJaw && lowerJaw) {
					sout << "clamping tool is found " <<sendl;
					toolState.function = TOOLFUNCTION_CLAMP;
				}
				else {
					warnings.append("No collision model for the tool is found ");
				}

				if (omniDriver)
				{
					sofa::core::objectmodel::BaseData *idData = omniDriver.get()->findData("deviceIndex");
					if (idData)
						toolState.id = atoi(idData->getValueString().c_str());
				}
				else
				{
					warnings.append("omniDriver is missing, the device id is set to 0 and may not be accurate");
				}

				/* looking for Haptic surfaces */
				//modelSurface = getContext()->get<TriangleSetModel>(core::objectmodel::BaseContext::SearchDown);
				std::vector<core::CollisionModel*> models;
				getContext()->get<core::CollisionModel>(&models, core::objectmodel::Tag("HapticSurface"), core::objectmodel::BaseContext::SearchRoot);
				if (models.empty())
				{
					getContext()->get<core::CollisionModel>(&models, core::objectmodel::BaseContext::SearchRoot);
					sofa::core::topology::TopologicalMapping * topoMapping;
					for (unsigned int i = 0; i < models.size(); ++i)
					{
						core::CollisionModel* m = models[i];
						m->getContext()->get(topoMapping);
						if (topoMapping == NULL) continue;
						modelSurfaces.push_back(m); // we found a good object
					}
				}
				else
				{
					modelSurfaces = models;
				}

				/* Looking for intersection and NP */
				intersectionMethod = getContext()->get<core::collision::Intersection>();
				detectionNP = getContext()->get<core::collision::NarrowPhaseDetection>();

				/* Set up warnings if these stuff are not found */
				if (!toolModel) warnings.append("toolModel is missing");
				if (modelSurfaces.empty()) warnings.append("modelSurface not found");
				if (intersectionMethod == NULL) warnings.append("intersectionMethod not found");
				if (detectionNP == NULL) warnings.append("NarrowPhaseDetection not found");

			}

			void HapticManager::reset()
			{
			}
			
			void HapticManager::doGrasp()
			{
				const ContactVector* contacts = getContacts();
				if (contacts == NULL) return;
				//sout << "Contact size: " << contacts->size() << sendl;
				for (unsigned int j = 0; j < 1; ++j)
				{
					const ContactVector::value_type& c = (*contacts)[j];
					// get the triangle index in the collision model of the surface
					int idx = (c.elem.first.getCollisionModel() == toolState.modelTool ? c.elem.second.getIndex() : c.elem.first.getIndex());
					// get the actual collision point. The point may lie inside in the triangle and its coordinates are calculated as barycentric coordinates w.r.t. the triangle. 
					Vector3 pnt = (c.elem.first.getCollisionModel() == toolState.modelTool ? c.point[1] : c.point[0]);
					if (idx >= 0)
					{
						toolState.m1 = ContactMapper::Create(c.elem.first.getCollisionModel());
						toolState.m2 = ContactMapper::Create(c.elem.second.getCollisionModel());
						
						core::behavior::MechanicalState<DataTypes>* mstateCollision1 = toolState.m1->createMapping(GenerateStirngID::generate().c_str());
						toolState.m1->resize(1);
						core::behavior::MechanicalState<DataTypes>* mstateCollision2 = toolState.m2->createMapping(GenerateStirngID::generate().c_str());
						toolState.m2->resize(1);
						
						toolState.m_constraints = sofa::core::objectmodel::New<sofa::component::constraintset::BilateralInteractionConstraint<DataTypes> >(mstateCollision1, mstateCollision2);
						toolState.m_constraints->clear(1);
												
						int index1 = c.elem.first.getIndex();
						int index2 = c.elem.second.getIndex();

						double r1 = 0.0;
						double r2 = 0.0;

						index1 = toolState.m1->addPointB(c.point[0], index1, r1);
						index2 = toolState.m2->addPointB(c.point[1], index2, r2);
						
						double distance = c.elem.first.getCollisionModel()->getProximity() + c.elem.second.getCollisionModel()->getProximity() + intersectionMethod->getContactDistance()+r1+r2;

						toolState.m_constraints->addContact(c.normal, c.point[0], c.point[1], distance, index1, index2, c.point[0], c.point[1]);
																								
						if (c.elem.first.getCollisionModel() == toolState.modelTool)
						{
							mstateCollision2->getContext()->addObject(toolState.m_constraints);
						}
						else
						{
							mstateCollision1->getContext()->addObject(toolState.m_constraints);
						}

						toolState.m1->update();
						toolState.m2->update();
						toolState.m1->updateXfree();
						toolState.m2->updateXfree();
					
					}

					if (c.elem.first.getCollisionModel() == toolState.modelTool)
						toolState.modelTool->setGroups(c.elem.second.getCollisionModel()->getGroups());
					else
						toolState.modelTool->setGroups(c.elem.first.getCollisionModel()->getGroups());

				}

			}
			
			void HapticManager::unGrasp()
			{
				if (toolState.m_constraints != NULL) {
					toolState.m_constraints->cleanup();
					toolState.m_constraints->getContext()->removeObject(toolState.m_constraints);
					toolState.m_constraints->reset();
					toolState.m1->cleanup();
					toolState.m2->cleanup();
				}
				toolState.modelTool->setGroups(toolState.modelGroup);
			}
			
			void HapticManager::updateVisual()
			{
				
			}
			
			void HapticManager::drawVisual(const core::visual::VisualParams* vparams)
			{
				if(!vparams->displayFlags().getShowVisualModels()) return;
				if (toolState.m_forcefield) {
					const VecCoord& x1 = toolState.m_forcefield->getObject1()->read(core::ConstVecCoordId::position())->getValue();
					const VecCoord& x2 = toolState.m_forcefield->getObject2()->read(core::ConstVecCoordId::position())->getValue();
					std::vector< Vector3 > points;
					points.push_back(Vector3(x1[0]));
					points.push_back(Vector3(x2[0]));
					vparams->drawTool()->drawLines(points, 3, sofa::defaulttype::Vec4f(0.8, 0.8, 0.8, 1));
				}
			}
			
			const HapticManager::ContactVector* HapticManager::getContacts()
			{
				sofa::helper::vector<std::pair<core::CollisionModel*, core::CollisionModel*> > vectCMPair;
				for (unsigned int s = 0; s < modelSurfaces.size(); s++)
					vectCMPair.push_back(std::make_pair(modelSurfaces[s]->getFirst(), toolState.modelTool->getFirst()));

				detectionNP->setInstance(this);
				detectionNP->setIntersectionMethod(intersectionMethod);
				detectionNP->beginNarrowPhase();
				detectionNP->addCollisionPairs(vectCMPair);
				detectionNP->endNarrowPhase();

				const core::collision::NarrowPhaseDetection::DetectionOutputMap& detectionOutputs = detectionNP->getDetectionOutputs();

				core::collision::NarrowPhaseDetection::DetectionOutputMap::const_iterator it = detectionOutputs.begin();
				if (it != detectionOutputs.end())
					return dynamic_cast<const ContactVector*>(it->second);
				else
					return NULL;
			}

			void HapticManager::doCarve()
			{
				const ContactVector* contacts = getContacts();
				if (contacts == NULL) return;
				int nbelems = 0;
				helper::vector<int> elemsToRemove;

				for (unsigned int j = 0; j < contacts->size(); ++j)
				{
					const ContactVector::value_type& c = (*contacts)[j];
					int triangleIdx = (c.elem.first.getCollisionModel() == toolState.modelTool ? c.elem.second.getIndex() : c.elem.first.getIndex());
					elemsToRemove.push_back(triangleIdx);
				}
				sofa::helper::AdvancedTimer::stepBegin("CarveElems");
				if (!elemsToRemove.empty())
				{
					static TopologicalChangeManager manager;
					const ContactVector::value_type& c = (*contacts)[0];
					if (c.elem.first.getCollisionModel() == toolState.modelTool)
						nbelems += manager.removeItemsFromCollisionModel(c.elem.second.getCollisionModel(), elemsToRemove);
					else
						nbelems += manager.removeItemsFromCollisionModel(c.elem.first.getCollisionModel(), elemsToRemove);
				}
			}
			
			void HapticManager::startSuture()
			{
				const ContactVector* contacts = getContacts();
				if (contacts == NULL) return;
				sout << "Contact size: " << contacts->size() << sendl;
				for (unsigned int j = 0; j < 1; ++j)
				{
					const ContactVector::value_type& c = (*contacts)[j];
					int idx = (c.elem.first.getCollisionModel() == toolState.modelTool ? c.elem.second.getIndex() : c.elem.first.getIndex());
					Vector3 pnt = (c.elem.first.getCollisionModel() == toolState.modelTool ? c.point[1] : c.point[0]);
					if (idx >= 0)
					{
						toolState.m1 = ContactMapper::Create(c.elem.first.getCollisionModel());
						toolState.m2 = ContactMapper::Create(c.elem.second.getCollisionModel());

						core::behavior::MechanicalState<DataTypes>* mstateCollision1 = toolState.m1->createMapping(GenerateStirngID::generate().c_str());
						toolState.m1->resize(1);
						core::behavior::MechanicalState<DataTypes>* mstateCollision2 = toolState.m2->createMapping(GenerateStirngID::generate().c_str());
						toolState.m2->resize(1);

						toolState.m_forcefield = sofa::core::objectmodel::New<sofa::component::interactionforcefield::VectorSpringForceField<DataTypes> >(mstateCollision1, mstateCollision2);

						toolState.m_forcefield->setName("HapticSurfaceContact");
						toolState.m_forcefield->clear(1);
						
						int index1 = c.elem.first.getIndex();
						int index2 = c.elem.second.getIndex();

						double r1 = 0.0;
						double r2 = 0.0;

						index1 = toolState.m1->addPointB(c.point[0], index1, r1);
						index2 = toolState.m2->addPointB(c.point[1], index2, r2);

						/* test if suture active */
						Real stiffness = grasp_stiffness.getValue();
						toolState.m_forcefield->addSpring(index1, index2, stiffness, 0.0, c.point[1] - c.point[0]);

						std::string suffix = int2string(toolState.id); /* TODO: or just its name */
						if (c.elem.first.getCollisionModel() == toolState.modelTool)
						{
							mstateCollision2->getContext()->addObject(toolState.m_forcefield);
							mstateCollision2->addTag(sofa::core::objectmodel::Tag("ConnectiveSpring" + suffix));
							sofa::component::controller::GraspingForceFeedback::SPtr fb = new sofa::component::controller::GraspingForceFeedback(mstateCollision2, grasp_forcescale.getValue());
							fb->addTag(sofa::core::objectmodel::Tag("GraspingForceFeedback" + suffix));
							this->getContext()->addObject(fb);
						}
						else
						{
							mstateCollision1->getContext()->addObject(toolState.m_forcefield);
							mstateCollision1->addTag(sofa::core::objectmodel::Tag("ConnectiveSpring" + suffix));
							sofa::component::controller::GraspingForceFeedback::SPtr fb = new sofa::component::controller::GraspingForceFeedback(mstateCollision1, grasp_forcescale.getValue());
							fb->addTag(sofa::core::objectmodel::Tag("GraspingForceFeedback" + suffix));
							this->getContext()->addObject(fb);
						}
						
						sofa::core::objectmodel::KeypressedEvent event(126 + toolState.id);
						if (omniDriver)
						{
							omniDriver->handleEvent(&event);
							sout << omniDriver->getName() << " handle event: " << 126 + toolState.id << sendl;
						}
						else
						{
							sout << "Omni Drive is NULL" << sendl;
						}

						toolState.m1->update();
						toolState.m2->update();
						toolState.m1->updateXfree();
						toolState.m2->updateXfree();
											
						{
							toolState.first_idx.push_back(idx);
							toolState.first_point.push_back(pnt);
						}
					}
				}

			}

			void HapticManager::stopSuture() 
			{
				sofa::core::objectmodel::KeyreleasedEvent event(126 + toolState.id);
				if (omniDriver) omniDriver->handleEvent(&event);
				
				if (toolState.m_forcefield != NULL) {
					toolState.m_forcefield->cleanup();
					toolState.m_forcefield->getContext()->removeObject(toolState.m_forcefield);
					toolState.m_forcefield->reset();
					toolState.m1->cleanup();
					toolState.m2->cleanup();
					toolState.m_forcefield = NULL;
				}
 				
				bool suture_active = (toolState.buttonState & 2);
				if (suture_active)
				{
					toolState.first_idx.clear();
					toolState.first_point.clear();
				}
			}
			
			void HapticManager::doSuture()
			{
				const ContactVector* contacts = getContacts();
				if (contacts == NULL) return;
				unsigned int ncontacts = contacts->size();
				std::vector<int> second_idx;
				std::vector<Vector3> second_point;
				for (unsigned int j = 0; j < ncontacts; ++j)
				{
					const ContactVector::value_type& c = (*contacts)[j];
					int idx = (c.elem.first.getCollisionModel() == toolState.modelTool ? c.elem.second.getIndex() : c.elem.first.getIndex());
					Vector3 pnt = (c.elem.first.getCollisionModel() == toolState.modelTool ? c.point[1] : c.point[0]);
					second_idx.push_back(idx);
					second_point.push_back(pnt);
				}
				unsigned int ss = std::min(toolState.first_idx.size(), second_idx.size());
				if (ss > 0)
				{
					core::CollisionModel* surf;
					if ((*contacts)[0].elem.first.getCollisionModel() == toolState.modelTool)
						surf = (*contacts)[0].elem.second.getCollisionModel();
					else
						surf = (*contacts)[0].elem.first.getCollisionModel();

					for (unsigned int j = 0; j < ss; j++)
					if (toolState.first_idx[j] != second_idx[j])
					{
						sofa::component::topology::TriangleSetTopologyContainer* triangleContainer;
						surf->getContext()->get(triangleContainer);
						sofa::component::topology::EdgeSetTopologyModifier* triangleModifier;
						surf->getContext()->get(triangleModifier);

						const component::topology::Triangle Triangle1 = triangleContainer->getTriangle(toolState.first_idx[j]);
						const component::topology::Triangle Triangle2 = triangleContainer->getTriangle(second_idx[j]);
						sofa::component::container::MechanicalObject <defaulttype::Vec3Types>* MechanicalObject;
						surf->getContext()->get(MechanicalObject, sofa::core::objectmodel::BaseContext::SearchRoot);
						if (!triangleContainer)
						{
							serr << "Error: can't find triangleContainer." << sendl;
							return;
						}
						else if (!triangleModifier)
						{
							serr << "Error: can't find edgeSetModifier." << sendl;
							return;
						}

						typedef sofa::core::topology::BaseMeshTopology::Edge Edge;

						sofa::helper::vector< Edge > edges;

						for (unsigned int i1 = 0; i1 < 2; ++i1)
						{
							for (unsigned int i2 = 0; i2 < 2; ++i2)
							{
								Edge e(Triangle1[i1], Triangle2[i2]);
								edges.push_back(e);
							}
						}

						triangleModifier->addEdges(edges);

						ContactMapper* m1 = ContactMapper::Create(surf);
						core::behavior::MechanicalState<DataTypes>* mstate1 = m1->createMapping("Mapper 1");
						m1->resize(1);
						ContactMapper* m2 = ContactMapper::Create(surf);
						core::behavior::MechanicalState<DataTypes>* mstate2 = m2->createMapping("Mapper 2");
						m2->resize(1);

						toolState.ff = sofa::core::objectmodel::New<StiffSpringForceField3>(mstate1, mstate2);

						toolState.ff->setName(GenerateStirngID::generate().c_str());
						toolState.ff->setArrowSize(0.1);
						toolState.ff->setDrawMode(1); 

						double r1 = 0.0;
						double r2 = 0.0;

						int index1 = m1->addPointB(toolState.first_point[j], toolState.first_idx[j], r1);
						int index2 = m2->addPointB(second_point[j], second_idx[j], r2);

						sout << "suturing for tool " << toolState.id << " actually happening " << sendl;
						toolState.ff->addSpring(index1, index2, grasp_stiffness.getValue(), 0.0, 0.0);
						
						mstate1->getContext()->addObject(toolState.ff);

						m1->update();
						m2->update();
						m1->updateXfree();
						m2->updateXfree();

						start_time = this->getContext()->getTime(); 
					}
				}
			}

			void HapticManager::doClamp(){
				ToolModel *upperJawModel = upperJaw.get();
				ToolModel *lowerJawModel = lowerJaw.get();

				std::pair<core::CollisionModel*, core::CollisionModel*> CMPair = std::make_pair(modelSurfaces[0]->getFirst(), upperJawModel->getFirst());

				detectionNP->setInstance(this);
				detectionNP->setIntersectionMethod(intersectionMethod);
				detectionNP->beginNarrowPhase();
				detectionNP->addCollisionPair(CMPair);
				detectionNP->endNarrowPhase();

				const core::collision::NarrowPhaseDetection::DetectionOutputMap& detectionOutputs = detectionNP->getDetectionOutputs();
				core::collision::NarrowPhaseDetection::DetectionOutputMap::const_iterator it1 = detectionOutputs.begin();
				if (it1 != detectionOutputs.end()) {
					const ContactVector* contacts = dynamic_cast<const ContactVector*>(it1->second);
					const ContactVector::value_type& c = (*contacts)[0];
					unsigned int idx1 = (c.elem.first.getCollisionModel() == upperJawModel ? c.elem.second.getIndex() : c.elem.first.getIndex());
					Vector3 P1 = (c.elem.first.getCollisionModel() == upperJawModel ? c.point[1] : c.point[0]);
					Vector3 normal1 = (c.elem.first.getCollisionModel() == upperJawModel ? -c.normal : c.normal);

					CMPair = std::make_pair(modelSurfaces[0]->getFirst(), lowerJawModel->getFirst());
					detectionNP->beginNarrowPhase();
					detectionNP->addCollisionPair(CMPair);
					detectionNP->endNarrowPhase();

					const core::collision::NarrowPhaseDetection::DetectionOutputMap& outputs = detectionNP->getDetectionOutputs();
					core::collision::NarrowPhaseDetection::DetectionOutputMap::const_iterator it2 = outputs.begin();
					if (it2 != outputs.end()){
						const ContactVector* cv = dynamic_cast<const ContactVector*>(it2->second);
						const ContactVector::value_type& ct = (*cv)[0];
						unsigned int idx2 = (ct.elem.first.getCollisionModel() == lowerJawModel ? ct.elem.second.getIndex() : ct.elem.first.getIndex());
						Vector3 P2 = (ct.elem.first.getCollisionModel() == lowerJawModel ? ct.point[1] : ct.point[0]);
						Vector3 normal2 = (ct.elem.first.getCollisionModel() == lowerJawModel ? -ct.normal : ct.normal);

						core::behavior::MechanicalState<RigidTypes>* mlow = lowerJawModel->getContext()->get<core::behavior::MechanicalState<RigidTypes> >();
						sofa::helper::ReadAccessor<sofa::core::objectmodel::Data<RigidTypes::VecCoord> > xlow = mlow->read(sofa::core::VecCoordId::position());
						core::behavior::MechanicalState<RigidTypes>* mup = upperJawModel->getContext()->get<core::behavior::MechanicalState<RigidTypes> >();
						sofa::helper::ReadAccessor<sofa::core::objectmodel::Data<RigidTypes::VecCoord> > xup = mup->read(sofa::core::VecCoordId::position());
						simulation::Node *root = dynamic_cast<simulation::Node*>(this->getContext()->getRootContext());
						Vector3 Pnt1 = (P1 + P2) / 2 + intersectionMethod->getContactDistance()*normal1;
						Vector3 Pnt2 = (P1 + P2) / 2 + intersectionMethod->getContactDistance()*normal2;
						createObstacle(root, "mesh/cube.obj", "mesh/cube.obj", "0.7 0.7 0.7", Pnt1, xlow[0].getOrientation().quatToRotationVector(), clampScale.getValue());
						createObstacle(root, "mesh/cube.obj", "mesh/cube.obj", "0.7 0.7 0.7", Pnt2, xup[0].getOrientation().quatToRotationVector(), clampScale.getValue());
						
						if (idx1 >= 0 && idx2 >= 0 && idx1 != idx2)
						{
							core::CollisionModel* surf = (ct.elem.first.getCollisionModel() == lowerJawModel ? ct.elem.second.getCollisionModel() : ct.elem.first.getCollisionModel());
							sofa::component::topology::TriangleSetTopologyContainer* triangleContainer;
							surf->getContext()->get(triangleContainer);
							const component::topology::Triangle Triangle1 = triangleContainer->getTriangle(idx1);
							const component::topology::Triangle Triangle2 = triangleContainer->getTriangle(idx2);

							StiffSpringForceField3::SPtr spring = sofa::core::objectmodel::New<StiffSpringForceField3>();
							surf->getContext()->addObject(spring);
							
							sofa::component::topology::HexahedronSetTopologyContainer* hexContainer;
							surf->getContext()->get(hexContainer);
							sofa::helper::vector< unsigned int > e1 = hexContainer->getHexahedraAroundVertex(Triangle1[0]);
							sofa::helper::vector< unsigned int > e2 = hexContainer->getHexahedraAroundVertex(Triangle1[1]);
							sofa::helper::vector< unsigned int > e3 = hexContainer->getHexahedraAroundVertex(Triangle1[2]);
							sofa::helper::vector< unsigned int > ie1;
							sofa::helper::vector< unsigned int > ie;
							std::sort(e1.begin(), e1.end());
							std::sort(e2.begin(), e2.end());
							std::sort(e3.begin(), e3.end());
							std::set_intersection(e1.begin(), e1.end(),	e2.begin(), e2.end(), std::back_inserter(ie1));
							std::set_intersection(ie1.begin(), ie1.end(), e3.begin(), e3.end(), std::back_inserter(ie));
							const component::topology::Hexahedron hex = hexContainer->getHexahedron(ie[0]);
							int v1 = hexContainer->getVertexIndexInHexahedron(hex, Triangle1[0]);
							int v2 = hexContainer->getVertexIndexInHexahedron(hex, Triangle1[1]);
							int v3 = hexContainer->getVertexIndexInHexahedron(hex, Triangle1[2]);
							const unsigned int vertexMap[6][4] = { { 4, 5, 6, 7 }, { 0, 3, 2, 1 }, { 2, 3, 7, 6 }, { 0, 4, 7, 3 }, { 1, 5, 4, 0 }, { 1, 2, 6, 5 } };
							for (int i = 0; i < 6; i++) {
								sofa::core::topology::Quad& q = hexContainer->getLocalQuadsInHexahedron(i);
								int j;
								for (j = 0; j < 4; j++) {
									if (q[j] == v1 || q[j] == v2 || q[j] == v3) {
										break;
									}
								}
								
								if (j == 4) { // found 
									spring->addSpring(hex[q[0]], hex[vertexMap[i][q[0]]], attach_stiffness.getValue(), 0.0, intersectionMethod->getContactDistance());
									spring->addSpring(hex[q[1]], hex[vertexMap[i][q[1]]], attach_stiffness.getValue(), 0.0, intersectionMethod->getContactDistance());
									spring->addSpring(hex[q[2]], hex[vertexMap[i][q[2]]], attach_stiffness.getValue(), 0.0, intersectionMethod->getContactDistance());
									spring->addSpring(hex[q[3]], hex[vertexMap[i][q[3]]], attach_stiffness.getValue(), 0.0, intersectionMethod->getContactDistance());
									break;
								}
							}
						}
					}
				}
			}
			void HapticManager::createObstacle(simulation::Node::SPtr  parent, const std::string &filenameCollision, const std::string filenameVisual, const std::string& color,
				const Deriv3& translation, const Deriv3 &rotation, const Deriv3 &scale)
			{
				simulation::Node::SPtr  nodeFixed = parent->createChild("Fixed");

				sofa::component::loader::MeshObjLoader::SPtr loaderFixed = sofa::core::objectmodel::New<sofa::component::loader::MeshObjLoader>();
				loaderFixed->setName("loader");
				loaderFixed->setFilename(sofa::helper::system::DataRepository.getFile(filenameCollision));
				loaderFixed->load();
				nodeFixed->addObject(loaderFixed);

				component::topology::MeshTopology::SPtr meshNodeFixed = sofa::core::objectmodel::New<component::topology::MeshTopology>();
				meshNodeFixed->setSrc("@" + loaderFixed->getName(), loaderFixed.get());
				nodeFixed->addObject(meshNodeFixed);
				meshNodeFixed->init();

				MechanicalObject3::SPtr dofFixed = sofa::core::objectmodel::New<MechanicalObject3>(); 
				dofFixed->setName("Fixed Object");
				dofFixed->setSrc("@" + loaderFixed->getName(), loaderFixed.get());
				dofFixed->setTranslation(translation[0], translation[1], translation[2]);
				dofFixed->setRotation(rotation[0], rotation[1], rotation[2]);
				dofFixed->setScale(scale[0], scale[1], scale[2]);
				nodeFixed->addObject(dofFixed);
				dofFixed->init();
				//component::collision::TriangleModel::SPtr triangleFixed = sofa::core::objectmodel::New<component::collision::TriangleModel>(); triangleFixed->setName("Collision Fixed");
				//triangleFixed->setSimulated(false); //Not simulated, fixed object
				//triangleFixed->setMoving(false);    //No extern events
				//nodeFixed->addObject(triangleFixed);
				//triangleFixed->init();
				//component::collision::LineModel::SPtr LineFixed = sofa::core::objectmodel::New<component::collision::LineModel>(); LineFixed->setName("Collision Fixed");
				//LineFixed->setSimulated(false); //Not simulated, fixed object
				//LineFixed->setMoving(false);    //No extern events
				//nodeFixed->addObject(LineFixed);
				//LineFixed->init();
				//component::collision::PointModel::SPtr PointFixed = sofa::core::objectmodel::New<component::collision::PointModel>(); PointFixed->setName("Collision Fixed");
				//PointFixed->setSimulated(false); //Not simulated, fixed object
				//PointFixed->setMoving(false);    //No extern events
				//nodeFixed->addObject(PointFixed);
				//PointFixed->init();
				simulation::Node::SPtr  nodeVisualFixed = nodeFixed->createChild("VM");
				component::visualmodel::OglModel::SPtr visualFixed = sofa::core::objectmodel::New<component::visualmodel::OglModel>();
				nodeVisualFixed->addObject(visualFixed);
				visualFixed->setName("visual");
				//visualFixed->setFilename(sofa::helper::system::DataRepository.getFile(filenameVisual));
				visualFixed->setColor(color);
				//visualFixed->setTranslation(translation[0], translation[1], translation[2]);
				//visualFixed->setRotation(rotation[0], rotation[1], rotation[2]);
				//visualFixed->setScale(scale[0], scale[1], scale[2]);
				visualFixed->init();
				visualFixed->initVisual();
				visualFixed->updateVisual();
				
				sofa::component::mapping::IdentityMapping< DataTypes, defaulttype::ExtVec3fTypes>::SPtr imap = sofa::core::objectmodel::New<sofa::component::mapping::IdentityMapping< DataTypes, defaulttype::ExtVec3fTypes> >();
				nodeVisualFixed->addObject(imap);
				imap->setModels(dofFixed.get(), visualFixed.get());
				imap->init();

				nodeFixed->addChild(nodeVisualFixed);
				
				parent->addChild(nodeFixed);
				nodeFixed->updateContext();
				
			}
			void HapticManager::updateTool()
			{
				unsigned char newButtonState = toolState.newButtonState;
				const unsigned char FIRST = 1, SECOND = 2;
				switch (toolState.function)
				{
				case TOOLFUNCTION_CARVE:
					if (newButtonState != 0) doCarve();
					break;
				case TOOLFUNCTION_CLAMP:
					if (((toolState.buttonState ^ newButtonState) & FIRST) != 0)
					{
						doClamp();
					}		

					break;
				case TOOLFUNCTION_GRASP:
					if (((toolState.buttonState ^ newButtonState) & FIRST) != 0)
					{
						/* the state of the first button is changing */
						if (newButtonState & FIRST != 0)
							doGrasp(); /* button down */
						else
							unGrasp(); /* button up */
					}
					break;
				case TOOLFUNCTION_SUTURE:
					if (((toolState.buttonState ^ newButtonState) & FIRST) != 0)
					{
						/* the state of the first button is changing */
						if (newButtonState & FIRST != 0)
							startSuture(); /* button down */
						else
							stopSuture(); /* button up */
					}

					if ((toolState.buttonState & FIRST) != 0 && (toolState.buttonState & SECOND) == 0 && (newButtonState & SECOND) != 0)
						doSuture();

					delta_time = this->getContext()->getTime() - start_time;
					
					if (delta_time < duration.getValue()) {
						if (toolState.ff != NULL) {
							double scale = delta_time / duration.getValue();
							double stiffness = grasp_stiffness.getValue()*(1 - scale) + attach_stiffness.getValue()*scale;
							toolState.ff->setStiffness(stiffness);
							toolState.ff->reinit();
						}
					}

					break;
				}
				toolState.buttonState = newButtonState;
			}

			void HapticManager::handleEvent(Event* event)
			{
				Controller::handleEvent(event);
			}

			void HapticManager::onHapticDeviceEvent(HapticDeviceEvent* ev)
			{
				if (ev->getDeviceId() == toolState.id) toolState.newButtonState = ev->getButtonState();
			}
			
			void HapticManager::onEndAnimationStep(const double dt) {
				if (intersectionMethod == NULL || detectionNP == NULL)
				{
					sout << "intersection method or detection NP is missing" << sendl;
					this->f_listening.setValue(false);
				}

				updateTool();
			}

		} // namespace collision

	} // namespace component

} // namespace sofa
