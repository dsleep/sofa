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
			{
				this->f_listening.setValue(true);
			}

			HapticManager::~HapticManager()
			{
			}


			void HapticManager::init()
			{

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
				/*if(!vparams->displayFlags().getShowVisualModels()) return;
				if (!mesh) return;
				sofa::helper::ReadAccessor<sofa::core::objectmodel::Data<sofa::defaulttype::Rigid3dTypes::VecCoord> > toolTip = toolState.modelTool->getContext()->get<sofa::component::container::MechanicalObject<sofa::defaulttype::Rigid3dTypes> >()->read(sofa::core::VecCoordId::position());
				Coord tip = toolTip[0].getCenter();
				sofa::core::topology::Triangle t = mesh->getTriangle(toolState.first_idx[0]);
				const VecCoord& x = mesh->getContext()->get<sofa::component::container::MechanicalObject <defaulttype::Vec3Types> >()->read(sofa::core::ConstVecCoordId::position())->getValue();
				Coord edge0 = tip - x[t[0]];
				Coord edge1 = tip - x[t[1]];
				Coord edge2 = tip - x[t[2]];
				vparams->drawTool()->setLightingEnabled(false);
				defaulttype::Vec4f color(1, 0, 0, 1);
				glBegin(GL_TRIANGLES);
				Coord normal = cross(edge0, edge1).normalized();
				vparams->drawTool()->drawTriangle(tip, x[t[0]], x[t[1]], normal, normal, normal, color, color, color);
				normal = cross(edge1, edge2).normalized();
				vparams->drawTool()->drawTriangle(tip, x[t[1]], x[t[2]], normal, normal, normal, color, color, color);
				normal = cross(edge2, edge0).normalized();
				vparams->drawTool()->drawTriangle(tip, x[t[2]], x[t[0]], normal, normal, normal, color, color, color);
				glEnd();*/
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

			void HapticManager::updateBoundingBoxes()
			{
				const bool continuous = intersectionMethod->useContinuous();
				const double dt = getContext()->getDt();
				const int depth = 6;

				if (continuous)
					toolState.modelTool->computeContinuousBoundingTree(dt, depth);
				else
					toolState.modelTool->computeBoundingTree(depth);

				for (unsigned int s = 0; s < modelSurfaces.size(); s++)
				if (continuous)
					modelSurfaces[s]->computeContinuousBoundingTree(dt, depth);
				else
					modelSurfaces[s]->computeBoundingTree(depth);
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
						sout << "suturing for tool " << toolState.id << " actually happening " << sendl;
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
						toolState.ff->setDrawMode(0); //Arrow mode if size > 0

						double r1 = 0.0;
						double r2 = 0.0;

						int index1 = m1->addPointB(toolState.first_point[j], toolState.first_idx[j], r1);
						int index2 = m2->addPointB(second_point[j], second_idx[j], r2);

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

			void HapticManager::updateTool()
			{
				unsigned char newButtonState = toolState.newButtonState;
				const unsigned char FIRST = 1, SECOND = 2;
				switch (toolState.function)
				{
				case TOOLFUNCTION_CARVE:
					if (newButtonState != 0) doCarve();
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

				updateBoundingBoxes(); // this line is probably unnecessary.
				updateTool();
			}

		} // namespace collision

	} // namespace component

} // namespace sofa
