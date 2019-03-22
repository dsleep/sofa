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
#include "GUIManager.h"
#include "BaseGUI.h"
#include "RealGUI.h"
using sofa::gui::GUIManager;
using sofa::gui::BaseGUI;
using sofa::gui::qt::RealGUI;
bool usingAA = false;
#define int2string(a) std::to_string(a)
using namespace std;

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

			//int HapticManager::_test = 0;

			HapticManager::HapticManager()
				: grasp_stiffness(initData(&grasp_stiffness, 10000.0, "graspStiffness", "how stiff surface is attached to the tool"))
				, attach_stiffness(initData(&attach_stiffness, 1.0, "attachStiffness", "how stiff surface is attached together"))
				, grasp_forcescale(initData(&grasp_forcescale, 0.001, "grasp_force_scale", "multiply the force with this coefficient"))
				, duration(initData(&duration, 10.0, "duration", "time to increase stiffness of suturing springs"))
				, intersectionMethod(NULL)
				, detectionNP(NULL)
				, toolModel(initLink("toolModel", "Tool model that is used for grasping and Haptic"))
				, omniDriver(initLink("omniDriver", "NewOmniDriver tag that corresponds to this tool"))
				, clampScale(initData(&clampScale, Vec3f(.2f, .2f, .2f), "clampScale", "scale of the object created during clamping"))
				, clampMesh(initData(&clampMesh, "mesh/cube.obj", "clampMesh", " Path to the clipper model"))
			{
				this->f_listening.setValue(true);

				//std::cout << "haptic manager construction" << std::endl;
			}

			HapticManager::~HapticManager()
			{
			}

			void HapticManager::init()
			{
				if (toolModel) {
					std::cout << "haptic manager init found tool model!" << std::endl;
					ToolModel *tm = toolModel.get();
					toolState.buttonState = 0;
					toolState.newButtonState = 0;
					toolState.id = 0;
					toolState.modelTool = tm;
					toolState.modelGroup = tm->getGroups();
					if (tm->hasTag(core::objectmodel::Tag("CarvingTool")) || tm->hasTag(core::objectmodel::Tag("DissectingTool")) || tm->hasTag(core::objectmodel::Tag("CuttingTool")))
					{
						toolState.function = TOOLFUNCTION_CARVE;
						sout << "Active tool: Carving" << sendl;
					}
					else if (tm->hasTag(core::objectmodel::Tag("SuturingTool")))
					{
						toolState.function = TOOLFUNCTION_SUTURE;
						sout << "Active tool: Suturing" << sendl;
					}
					else if (tm->hasTag(core::objectmodel::Tag("GraspingTool")))
					{
						toolState.function = TOOLFUNCTION_GRASP;
						sout << "Active tool: Grasping" << sendl;
					}
					else if (tm->hasTag(core::objectmodel::Tag("ContainerTool")))
					{
						toolState.function = TOOLFUNCTION_CONTAIN;
						std::cout << "found container!" << std::endl;
						sout << "Active tool: Container" << sendl;
					}
					else{
						toolState.function = TOOLFUNCTION_CLAMP;
						sout << "Active tool: Clamping" << sendl;
					}
					sout << "tool is found " << sendl;
					std::string meshFilename("mesh/cube.obj");
					if (sofa::helper::system::DataRepository.findFile(meshFilename)) {
						clipperMesh.reset(sofa::helper::io::Mesh::Create(meshFilename));
						if (clipperMesh.get() == 0)
							sout << "Clipper mesh not found !" << sendl;
						else
							sout << "Clipper mesh size:" << clipperMesh->getVertices().size() << sendl;
					}
				}
				else {
					warnings.append("No collision model for the tool is found ");
				}

				if (omniDriver)
				{
					sofa::core::objectmodel::BaseData *idData = omniDriver.get()->findData("deviceIndex");
					if (idData)
					{
						toolState.id = atoi(idData->getValueString().c_str());
						sofa::component::controller::NewOmniDriver* newOmniDriverDummy;
						if (usingAA) {
							//sofa::component::controller::AAOmniDriver* aaOmniDriverDummy;

							//if (omniDriver.get()->getClassName().compare(aaOmniDriverDummy->getName()) == 0)
							//{
							//	//cast to aa omni
							//	aaOmniDriver = dynamic_cast<sofa::component::controller::AAOmniDriver *>(omniDriver.get());
							//	newOmniDriver = NULL;
							//	std::cout << "haptic manager using AAOmni " << this << std::endl;
							//}
						}
						else
						{
							if (omniDriver.get()->getClassName().compare(newOmniDriverDummy->getName()) == 0)
							{
								//cast to New omni
								newOmniDriver = dynamic_cast<sofa::component::controller::NewOmniDriver *>(omniDriver.get());
								//aaOmniDriver = NULL;
								//std::cout << "haptic manager using NewOmni " << this << std::endl;

							}
							else
							{
								//throw a warning
								std::cout << "haptic manager found a rogue device" << std::endl;
							}
						}
					}

				}

				else
				{
					warnings.append("omniDriver is missing, the device id is set to 0 and may not be accurate");
				}

				/* looking for Haptic surfaces */

				std::vector<core::CollisionModel*> modelVeinSurfaces;
				std::vector<core::CollisionModel*> modelCurveSurfaces;
				std::vector<core::CollisionModel*> SafetySurfaces;
				getContext()->get<core::CollisionModel>(&modelSurfaces, core::objectmodel::Tag("HapticSurface"), core::objectmodel::BaseContext::SearchRoot);
				getContext()->get<core::CollisionModel>(&modelVeinSurfaces, core::objectmodel::Tag("HapticSurfaceVein"), core::objectmodel::BaseContext::SearchRoot);
				getContext()->get<core::CollisionModel>(&modelCurveSurfaces, core::objectmodel::Tag("HapticSurfaceCurve"), core::objectmodel::BaseContext::SearchRoot);
				getContext()->get<core::CollisionModel>(&SafetySurfaces, core::objectmodel::Tag("SafetySurface"), core::objectmodel::BaseContext::SearchRoot);
				for (int i = 0; i<modelVeinSurfaces.size(); i++)
					modelSurfaces.push_back(modelVeinSurfaces[i]);
				for (int i = 0; i<modelCurveSurfaces.size(); i++)
					modelSurfaces.push_back(modelCurveSurfaces[i]);
				for (int i = 0; i<SafetySurfaces.size(); i++)
					modelSurfaces.push_back(SafetySurfaces[i]);

				/* Looking for intersection and NP */
				intersectionMethod = getContext()->get<core::collision::Intersection>();
				detectionNP = getContext()->get<core::collision::NarrowPhaseDetection>();

				/* Set up warnings if these stuff are not found */
				if (!toolModel) warnings.append("toolModel is missing");
				if (modelSurfaces.empty()) warnings.append("Haptic Surface not found");
				if (intersectionMethod == NULL) warnings.append("intersectionMethod not found");
				if (detectionNP == NULL) warnings.append("NarrowPhaseDetection not found");

				std::string path = sofa::helper::system::DataRepository.getFirstPath();
				//std::cout << "get first path:" << path<<std::endl;
				base_path_share = path.substr(0, path.find("examples")).append("share");
				//std::cout << "base_path_share" << base_path_share << std::endl;			

				if (programStartDate.compare("") == 0) //only initilize this once
				{
					//obtain system date
					time_t rawtime;
					struct tm * timeinfo;
					char buffer[80];

					time(&rawtime);
					timeinfo = localtime(&rawtime);
					//std::cout << "path.substr(0, path.find(examples))" << path.substr(0, path.find("examples")) << std::endl;
					strftime(buffer, sizeof(buffer), "%d_%m_%Y-%I_%M_%S", timeinfo);
					programStartDate = std::string(buffer);
				}
				
				//std::string screenshotsFolder = std::string("explorer " + base_path_share);
				//std::string screenshotsFolder =  "explorer C:\\Users\\Ruiliang\\Academics\\SOFA\\TipsSofa1612\\src\\share\\TIPS_screenshot";
				//system(screenshotsFolder.c_str()); //system() only works for "\\" format
				//std::cout << "screenshotsFolder:" << screenshotsFolder << endl;
			}



			void HapticManager::reset()
			{
			}

			void HapticManager::doContain()
			{
				const ContactVector* contacts = getContacts();
				if (contacts == NULL) return;
				for (unsigned int j = 0; j < 1; ++j)
				{
					const ContactVector::value_type& c = (*contacts)[j];
					// get the triangle index in the collision model of the surface
					int idx = (c.elem.first.getCollisionModel() == toolState.modelTool ? c.elem.second.getIndex() : c.elem.first.getIndex());
					// get the actual collision point. The point may lie inside in the triangle and its coordinates are calculated as barycentric coordinates w.r.t. the triangle. 
					core::CollisionModel* surf = (c.elem.first.getCollisionModel() == toolState.modelTool ? c.elem.second.getCollisionModel() : c.elem.first.getCollisionModel());
					if ((surf->hasTag(core::objectmodel::Tag("TargetOrgan")) && namesOfVeinCutSet.size()>=1))
					{
						achievementsCount++;
						std::string SharePath = base_path_share;
						std::string capturePath(SharePath + "/TIPS_screenshot/Achievements/" + programStartDate + "achievement_");
						std::string achi("Pouch.png");
						std::string out = capturePath + int2string(achievementsCount);
						out = out + achi;
						capture.saveScreen(out,5);
						if (!hasInstrumentTurnedGreen && !hasInstrumentTurnedRed)
						 {
						 string search_string = "vec3 boundaryColor = vec3( 0., 0., 0. );";						 
						 string replace_string = "vec3 boundaryColor = vec3( 0., 1., 0. );";
						 hasInstrumentTurnedGreen = true;
						 updateShader("\\shaders\\TIPSShaders\\instrument.glsl", "\\shaders\\TIPSShaders\\outinstrument.glsl",
						 search_string, replace_string);
						 last_update_time = this->getContext()->getTime();
						 hasPutInBag = true;
						 }
					}

				}
				//string test = GUIManager::GetCurrentGUIName();
				//BaseGUI* thing = GUIManager::getGUI();
				std::cout << "programCompletionTime: " << last_update_time << std::endl; 
				std::cout << "numOfElementsCutonVeins: " << numOfElementsCutonVeins << std::endl;
				std::cout << "numOfElementsCutonFat: " << numOfElementsCutonFat - numOfElementsCutonVeins << std::endl;

				//UF - DS TODO FIX GUI
				//RealGUI* realGUI = dynamic_cast<RealGUI*>(GUIManager::getGUI());
				//realGUI->populateReport(programStartDate);
				//realGUI->showReport();
			}

			//const ContactVector* lastContacts;
			void HapticManager::doGrasp()
			{
				const ContactVector* contacts = getContacts();

				if (contacts == NULL) return;
				for (unsigned int j = 0; j < 1; ++j)
				{
					const ContactVector::value_type& c = (*contacts)[j];
					// get the triangle index in the collision model of the surface
					int idx = (c.elem.first.getCollisionModel() == toolState.modelTool ? c.elem.second.getIndex() : c.elem.first.getIndex());
					//std::cout << "haptic manager: idx is " << idx <<std::endl;
					// get the actual collision point. The point may lie inside in the triangle and its coordinates are calculated as barycentric coordinates w.r.t. the triangle. 
					Vector3 pnt = (c.elem.first.getCollisionModel() == toolState.modelTool ? c.point[1] : c.point[0]);
					//std::cout << "pnt of collision of first contact: " << pnt[0] << pnt[1] << pnt[2] << std::endl;
					if (idx >= 0)
					{
						toolState.m1 = ContactMapper::Create(c.elem.first.getCollisionModel());
						toolState.m2 = ContactMapper::Create(c.elem.second.getCollisionModel());

						core::behavior::MechanicalState<DataTypes>* mstateCollision1 = toolState.m1->createMapping(GenerateStringID::generate().c_str());
						toolState.m1->resize(1);
						core::behavior::MechanicalState<DataTypes>* mstateCollision2 = toolState.m2->createMapping(GenerateStringID::generate().c_str());
						toolState.m2->resize(1);

						toolState.m_constraints = sofa::core::objectmodel::New<sofa::component::constraintset::BilateralInteractionConstraint<DataTypes> >(mstateCollision1, mstateCollision2);
						toolState.m_constraints->clear(1);
						toolState.m_constraints->setName("BConstraint-HapticGrasping");

						int index1 = c.elem.first.getIndex();
						int index2 = c.elem.second.getIndex();

						double r1 = 0.0;
						double r2 = 0.0;

						index1 = toolState.m1->addPointB(c.point[0], index1, r1);
						index2 = toolState.m2->addPointB(c.point[1], index2, r2);

						double distance = c.elem.first.getCollisionModel()->getProximity() + c.elem.second.getCollisionModel()->getProximity() + intersectionMethod->getContactDistance() + r1 + r2;

						toolState.m_constraints->addContact(c.normal, c.point[0], c.point[1], distance, index1, index2, c.point[0], c.point[1], 1, c.id); //added "1, c.id" for the last 2 parameters with no changes.
						

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

				if (!vparams->displayFlags().getShowVisualModels()) return;
				//core::visual::tristate showCLS(true);
				//vparams->displayFlags().setShowCollisionModels();//How to use this...struggling
				if (toolState.m_forcefield) {
					const VecCoord& x1 = toolState.m_forcefield->getObject1()->read(core::ConstVecCoordId::position())->getValue();
					const VecCoord& x2 = toolState.m_forcefield->getObject2()->read(core::ConstVecCoordId::position())->getValue();
					std::vector< Vector3 > points;
					points.push_back(Vector3(x1[0]));
					points.push_back(Vector3(x2[0]));
					vparams->drawTool()->drawLines(points, 3, sofa::defaulttype::Vec4f(0.8f, 0.8f, 0.8f, 1));
				}

				for (int i = 0; i < clampPairs.size(); i++) { // construct clip "brick" (hex) on quad facet(s) of thick curve hex where the clip is applied
					sofa::core::topology::Topology::Hexahedron hex = clampPairs[i].first;
					int quad = clampPairs[i].second;
					const unsigned int vertexHex[6][4] = { { 0, 1, 2, 3 }, { 4, 7, 6, 5 }, { 1, 0, 4, 5 }, { 1, 5, 6, 2 }, { 2, 6, 7, 3 }, { 0, 3, 7, 4 } };
					const unsigned int vertexMap[6][4] = { { 4, 5, 6, 7 }, { 0, 3, 2, 1 }, { 2, 3, 7, 6 }, { 0, 4, 7, 3 }, { 1, 5, 4, 0 }, { 1, 2, 6, 5 } };

					const VecCoord& x = clipperStates[i]->read(core::ConstVecCoordId::position())->getValue();
					const unsigned int oppositeQuads[6] = { 1, 0, 3, 2, 5, 4 };
					int quadop = oppositeQuads[quad]; // opposite quad in the hex
					Vector3 n1 = (x[hex[vertexHex[quad][1]]] - x[hex[vertexHex[quad][0]]]).normalized(); //edge difference of quad
					Vector3 n2 = (x[hex[vertexHex[quad][2]]] - x[hex[vertexHex[quad][1]]]).normalized(); //edge difference of quad
					Vector3 n3 = n2.cross(n1).normalized(); // normal of the quad
					n2 = n3.cross(n1);  // in case of twisted quad, get second direction orthogonal to n1, n3	

					Vec3f sc = clampScale.getValue();
					Vector3 P; // hex center
					for (size_t iv = 0; iv < 8; iv++){ P += x[hex[iv]] / 8; }

					const helper::vector< Vector3 > &vertices = clipperMesh->getVertices(); // get model of clip  triangulated
					const helper::vector< Vector3 > &normals = clipperMesh->getNormals();
					const helper::vector< helper::vector < helper::vector <int> > > &facets = clipperMesh->getFacets();
					vector< Vector3 > vv(vertices.size()); // modifiable vertex array
					vector< Vector3 > nn(normals.size());

					//double relativeRatioForClip = 2; // relative between edge length of clip and hex
					double relativeRatioForClip = 6;// 3; // relative between edge length of clip and hex
					double relativeScale = relativeRatioForClip*hexDimensions[i] / 2;

					// update *sc*
					if (edge12along[i]){ sc[0] = relativeScale*sc[0]; sc[1] = relativeScale / 4 * sc[1]; }
					else{ sc[0] = relativeScale / 4 * sc[0]; sc[1] = relativeScale*0.8*sc[1]; };
					sc[2] = relativeScale*sc[2];

					for (int t = 0; t < vertices.size(); t++) {  // adjust clip model (scale, etc) along edge-directions of quad
						vv[t][0] = sc[0] * n1[0] * vertices[t][0] + sc[1] * n2[0] * vertices[t][1] + sc[2] * n3[0] * vertices[t][2] + P[0];
						vv[t][1] = sc[0] * n1[1] * vertices[t][0] + sc[1] * n2[1] * vertices[t][1] + sc[2] * n3[1] * vertices[t][2] + P[1];
						vv[t][2] = sc[0] * n1[2] * vertices[t][0] + sc[1] * n2[2] * vertices[t][1] + sc[2] * n3[2] * vertices[t][2] + P[2];

						nn[t][0] = n1[0] * normals[t][0] + n2[0] * normals[t][1] + n3[0] * normals[t][2];
						nn[t][1] = n1[1] * normals[t][0] + n2[1] * normals[t][1] + n3[1] * normals[t][2];
						nn[t][2] = n1[2] * normals[t][0] + n2[2] * normals[t][1] + n3[2] * normals[t][2];
					}

					glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
					glColor3f(0.8f, 0.8f, 0.8f);
					//glEnable(GL_COLOR_MATERIAL);
					glBegin(GL_TRIANGLES);  // display clip in OGL: 2 triangles per quad
					for (int t = 0; t < facets.size(); t++) {
						// plot wrt 3 vertices of the each facet (1 pair for 1 vertex)
						glNormal3d(nn[facets[t][1][0]][0], nn[facets[t][1][0]][1], nn[facets[t][1][0]][2]);
						glVertex3d(vv[facets[t][0][0]][0], vv[facets[t][0][0]][1], vv[facets[t][0][0]][2]);
						glNormal3d(nn[facets[t][1][1]][0], nn[facets[t][1][1]][1], nn[facets[t][1][1]][2]);
						glVertex3d(vv[facets[t][0][1]][0], vv[facets[t][0][1]][1], vv[facets[t][0][1]][2]);
						glNormal3d(nn[facets[t][1][2]][0], nn[facets[t][1][2]][1], nn[facets[t][1][2]][2]);
						glVertex3d(vv[facets[t][0][2]][0], vv[facets[t][0][2]][1], vv[facets[t][0][2]][2]);
					}
					glEnd();
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

			//global var used for recording mistakes, change these to static member later
			int mistatkeTolerance = 50;
			int mistatkeToleranceCut = 50;
			int mistatkeToleranceCutVein = 50;
			int mistatkeToleranceClamp = 50;
			int mistatkeToleranceDissect = 50;
			int mistakeToleranceForce = 0;
			
			void HapticManager::doCarve()
			{
				const ContactVector* contacts = getContacts();
				if (contacts == NULL) return;
				int nbelems = 0;
				helper::vector<int> elemsToRemove;
				//std::vector<int> indexActiveContacts;
				ToolModel *tm = toolModel.get();
				int active_contact_index = 0; // index of the contacts that has the collision model we want to carve.
				bool contact_index_fixed = false;

				for (unsigned int j = 0; j < contacts->size(); j++)
				{
					const ContactVector::value_type& c = (*contacts)[j];
					core::CollisionModel* surf = (c.elem.first.getCollisionModel() == toolState.modelTool ? c.elem.second.getCollisionModel() : c.elem.first.getCollisionModel());
					
					//find the correct carvable model index below
					if ((surf->hasTag(core::objectmodel::Tag("HapticSurfaceVein")) || surf->hasTag(core::objectmodel::Tag("HapticSurfaceVolume")) || surf->hasTag(core::objectmodel::Tag("HapticSurfaceCurve"))))
					{
						if (!contact_index_fixed)
						{
							if (tm->hasTag(core::objectmodel::Tag("CarvingTool")) || tm->hasTag(core::objectmodel::Tag("CuttingTool")))
							{
								if (surf->hasTag(core::objectmodel::Tag("HapticSurfaceVolume")))
								{
									contact_index_fixed = true;
									active_contact_index = j;
								}
							}	
						}					
					}

					////'CuttingTool'(marryland dissector) does nothing if not enough force has been applied
					//if (tm->hasTag(core::objectmodel::Tag("CuttingTool")) )
					//{
					//	double resultantForce = 0;
					//	if (newOmniDriver)
					//		resultantForce = sqrt(std::pow(newOmniDriver->data.currentForce[0], 2) + std::pow(newOmniDriver->data.currentForce[1], 2) + std::pow(newOmniDriver->data.currentForce[2], 2));
					//	if(resultantForce<0.5)
					//		continue; 
					//}	

					//find vein or thick-curve
					if ((surf->hasTag(core::objectmodel::Tag("HapticSurfaceVein")) || surf->hasTag(core::objectmodel::Tag("HapticSurfaceCurve"))) )
					{		
						if (tm->hasTag(core::objectmodel::Tag("CarvingTool")) || tm->hasTag(core::objectmodel::Tag("CuttingTool")) && mistatkeToleranceCutVein > 0)
						{
							int hasCutThisVein = hasBeenCut(surf->getName());
							double resultantForce = 0;
							if (usingAA /*&& aaOmniDriver*/){
								//resultantForce = sqrt(std::pow(aaOmniDriver->data.currentForce[0], 2) + std::pow(aaOmniDriver->data.currentForce[1], 2) + std::pow(aaOmniDriver->data.currentForce[2], 2));
							}
							else
							{
								if (newOmniDriver)//removed sqrt below
									resultantForce = std::pow(newOmniDriver->data.currentForce[0], 2) + std::pow(newOmniDriver->data.currentForce[1], 2) + std::pow(newOmniDriver->data.currentForce[2], 2);
							}
							if (!hasCutThisVein && resultantForce >= 1.4)
							{
								if (!hasInstrumentTurnedRed)
								{
									string search_string = "vec3 boundaryColor = vec3( 0., 0., 0. );";
									string replace_string = "vec3 boundaryColor = vec3( 1., 0., 0. );";
									hasInstrumentTurnedRed = true;
									updateShader("\\shaders\\TIPSShaders\\instrument.glsl", "\\shaders\\TIPSShaders\\outinstrument.glsl",
										search_string, replace_string);
									last_update_time = this->getContext()->getTime();
								}

								mistatkeToleranceCutVein--;
								mistatkeTolerance--;
								
								std::string SharePath = base_path_share;
								std::string capturePath(SharePath + "/TIPS_screenshot/Errors/" + programStartDate + "error"); //temp path for saving the screenshot
								std::string err("_dissecting_vein.png");
								std::string out = capturePath + int2string(int((50-mistatkeTolerance)/5));
								out = out + err;
								capture.saveScreen(out, 5);
								
								continue; // skip this element
							}
							continue; // skip this element
						}
						//else if (tm->hasTag(core::objectmodel::Tag("CuttingTool")) && mistatkeToleranceCutVein > 0)
						//{
						//	continue; //'CuttingTool'(marryland dissector) does nothing on veins, so we skip this element
						//}			
					}
					else if (surf->hasTag(core::objectmodel::Tag("SafetySurface")) && mistatkeTolerance > 0)
					{
						if (tm->hasTag(core::objectmodel::Tag("CuttingTool")))
						{
							continue; //'CuttingTool'(marryland dissector) does no harm, so we skip this element
						}
						else if (tm->hasTag(core::objectmodel::Tag("CarvingTool")) && mistatkeToleranceCut > 0)
						{
							mistatkeToleranceCut--;
							mistatkeTolerance--;
							if ((this->getContext()->getTime() - last_update_time) > 1.0)
							{
								std::string SharePath = base_path_share;
								std::string capturePath(SharePath + "/TIPS_screenshot/Errors/" + programStartDate + "error");
								std::string err("_dissecting_safety.png");
								std::string out = capturePath + int2string(50 - mistatkeTolerance);
								out = out + err;
								capture.saveScreen(out, 5);
							}
							if (!hasInstrumentTurnedRed)
							{
								string search_string = "vec3 boundaryColor = vec3( 0., 0., 0. );";
								string replace_string = "vec3 boundaryColor = vec3( 1., 0., 0. );";
								hasInstrumentTurnedRed = true;
								updateShader("\\shaders\\TIPSShaders\\instrument.glsl", "\\shaders\\TIPSShaders\\outinstrument.glsl",
									search_string, replace_string);
								last_update_time = this->getContext()->getTime();
							}				
							continue; // skip this element of safety surface
						}  
						else if (tm->hasTag(core::objectmodel::Tag("DissectingTool")) && mistatkeToleranceDissect > 0)
						{
							mistatkeToleranceDissect--;
							mistatkeTolerance--;
							if ((this->getContext()->getTime() - last_update_time) > 1.0)
							{
								std::string SharePath = base_path_share;
								std::string capturePath(SharePath + "/TIPS_screenshot/Errors/" + programStartDate + "error");
								std::string err("_cutting_safety.png");
								std::string out = capturePath + int2string(50 - mistatkeTolerance);
								out = out + err;
								capture.saveScreen(out, 5);
							}
							if (!hasInstrumentTurnedRed)
							{
								string search_string = "vec3 boundaryColor = vec3( 0., 0., 0. );";
								string replace_string = "vec3 boundaryColor = vec3( 1., 0., 0. );";
								hasInstrumentTurnedRed = true;
								updateShader("\\shaders\\TIPSShaders\\instrument.glsl", "\\shaders\\TIPSShaders\\outinstrument.glsl",
									search_string, replace_string);
								last_update_time = this->getContext()->getTime();
							}			
							continue; // skip this element of safety surface
						}

					}
					sofa::core::topology::TopologicalMapping * topoMapping = surf->getContext()->get<sofa::core::topology::TopologicalMapping>();
					if (topoMapping == NULL) return;
					int triangleIdx = (c.elem.first.getCollisionModel() == toolState.modelTool ? c.elem.second.getIndex() : c.elem.first.getIndex());

					if (surf->hasTag(core::objectmodel::Tag("HapticSurfaceVein")) && tm->hasTag(core::objectmodel::Tag("DissectingTool")) && this->getContext()->getTime() - last_update_time >= 0.3)
					{
						int hasCutThisVein = hasBeenCut(surf->getName());
						//check below if enough clips has been placed on this vein
						if (vein_clips_map[surf->getName()].size() < 3 && !hasCutThisVein)
						{
							//cout << "vein_clips_map[surf->getName()].size():" << vein_clips_map[surf->getName()].size() << endl;
							if (!hasInstrumentTurnedRed )
							{
								string search_string = "vec3 boundaryColor = vec3( 0., 0., 0. );";
								string replace_string = "vec3 boundaryColor = vec3( 1., 0., 0. );";
								hasInstrumentTurnedRed = true;
								updateShader("\\shaders\\TIPSShaders\\instrument.glsl", "\\shaders\\TIPSShaders\\outinstrument.glsl",
									search_string, replace_string);
								last_update_time = this->getContext()->getTime();
							}
							mistatkeTolerance--;
							std::string SharePath = base_path_share;
							std::string capturePath(SharePath + "/TIPS_screenshot/Errors/" + programStartDate + "error");
							std::string err("_cut_without_enough_clips.png");
							std::string out = capturePath + int2string(50 - mistatkeTolerance);
							out = out + err;
							capture.saveScreen(out,5);
							last_update_time = this->getContext()->getTime();
						}
						if (vein_clips_map[surf->getName()].size() >=1 )
						{	//check if the cut is proper or not
							int hasClipBeforeCut = 0, hasClipAfterCut = 0, hasClipOnCut = 0;
							//the following are for checking the idx of the hex being cut
							core::CollisionModel* surf = (c.elem.first.getCollisionModel() == toolState.modelTool ? c.elem.second.getCollisionModel() : c.elem.first.getCollisionModel());
							sofa::component::topology::TriangleSetTopologyContainer* triangleContainer;
							surf->getContext()->get(triangleContainer);
							const sofa::core::topology::Topology::Triangle TriangleElem = triangleContainer->getTriangle(triangleIdx);
							sofa::component::topology::HexahedronSetTopologyContainer* hexContainer;
							surf->getContext()->get(hexContainer);
							if (hexContainer == NULL) return;

							sofa::helper::vector< unsigned int > e1 = hexContainer->getHexahedraAroundVertex(TriangleElem[0]);
							sofa::helper::vector< unsigned int > e2 = hexContainer->getHexahedraAroundVertex(TriangleElem[1]);
							sofa::helper::vector< unsigned int > e3 = hexContainer->getHexahedraAroundVertex(TriangleElem[2]);
							sofa::helper::vector< unsigned int > ie1;
							sofa::helper::vector< unsigned int > ie;
							std::sort(e1.begin(), e1.end());
							std::sort(e2.begin(), e2.end());
							std::sort(e3.begin(), e3.end());
							std::set_intersection(e1.begin(), e1.end(), e2.begin(), e2.end(), std::back_inserter(ie1));
							std::set_intersection(ie1.begin(), ie1.end(), e3.begin(), e3.end(), std::back_inserter(ie));
							int idxHexCut = ie[0];//idx of the hex to be cut
							if ( vein_clips_map[surf->getName()].size()  >= 1)
							{
								//check if the cut is between clips
								for (auto i = vein_clips_map[surf->getName()].begin(); i != vein_clips_map[surf->getName()].end(); ++i)
								{
									if (*i == idxHexCut)
										hasClipOnCut = 1;
									else if (*i < idxHexCut)
										hasClipBeforeCut = 1;
									else if (*i > idxHexCut)
										hasClipAfterCut = 1;
								}

								if (hasClipBeforeCut + hasClipAfterCut < 2) //meaning cut position is wrong
								{
									if (!hasInstrumentTurnedRed)
									{
										string search_string = "vec3 boundaryColor = vec3( 0., 0., 0. );";
										string replace_string = "vec3 boundaryColor = vec3( 1., 0., 0. );";
										hasInstrumentTurnedRed = true;
										updateShader("\\shaders\\TIPSShaders\\instrument.glsl", "\\shaders\\TIPSShaders\\outinstrument.glsl",
											search_string, replace_string);
										last_update_time = this->getContext()->getTime();
									}
									mistatkeTolerance--;
									std::string SharePath = base_path_share;
									std::string capturePath(SharePath + "/TIPS_screenshot/Errors/" + programStartDate + "error");
									std::string err("_cut_at_wrong_position.png");
									std::string out = capturePath + int2string(50 - mistatkeTolerance);
									out = out + err;
									capture.saveScreen(out, 5);
									last_update_time = this->getContext()->getTime();
								}
								if (hasClipOnCut == 1)
								{
									if (!hasInstrumentTurnedRed)
									{
										string search_string = "vec3 boundaryColor = vec3( 0., 0., 0. );";
										string replace_string = "vec3 boundaryColor = vec3( 1., 0., 0. );";
										hasInstrumentTurnedRed = true;
										updateShader("\\shaders\\TIPSShaders\\instrument.glsl", "\\shaders\\TIPSShaders\\outinstrument.glsl",
											search_string, replace_string);
										last_update_time = this->getContext()->getTime();
									}
									mistatkeTolerance--;
									std::string SharePath = base_path_share;
									std::string capturePath(SharePath + "/TIPS_screenshot/Errors/" + programStartDate + "error");
									std::string err("_left_clips_inside_body.png");
									std::string out = capturePath + int2string(50 - mistatkeTolerance);
									out = out + err;
									capture.saveScreen(out, 5);
									last_update_time = this->getContext()->getTime();
								}
								//Visual feedbakc and screenshot - achievement
								else if(!hasCutThisVein && vein_clips_map[surf->getName()].size() >= 3)
								{
									if (!hasInstrumentTurnedGreen && !hasInstrumentTurnedRed)
									{
										string search_string = "vec3 boundaryColor = vec3( 0., 0., 0. );";
										string replace_string = "vec3 boundaryColor = vec3( 0., 1., 0. );";
										hasInstrumentTurnedGreen = true;
										updateShader("\\shaders\\TIPSShaders\\instrument.glsl", "\\shaders\\TIPSShaders\\outinstrument.glsl",
											search_string, replace_string);
										last_update_time = this->getContext()->getTime();
									}
									achievementsCount++;
									
									std::string SharePath = base_path_share;
									std::string capturePath(SharePath + "/TIPS_screenshot/Achievements/" + programStartDate + "achieve_");
									std::string err("cut_the_vein.png");
									std::string out = capturePath + int2string(achievementsCount);
									out = out + err;
									capture.saveScreen(out, 5);
									last_update_time = this->getContext()->getTime();
								}
							}
							
						}
						//std::cout << "surf been cut:" << surf->getName() << std::endl;
						namesOfVeinCutSet.insert(surf->getName());
						numOfElementsCutonVeins++;
					}
					/*sofa::core::topology::TopologicalMapping * topoMapping = surf->getContext()->get<sofa::core::topology::TopologicalMapping>();
					if (topoMapping == NULL) return;
					int triangleIdx = (c.elem.first.getCollisionModel() == toolState.modelTool ? c.elem.second.getIndex() : c.elem.first.getIndex());
					*/elemsToRemove.push_back(triangleIdx);
					//indexActiveContacts.push_back(triangleIdx);
					//active_contact_index = j;
				}
				sofa::helper::AdvancedTimer::stepBegin("CarveElems");
				if (!elemsToRemove.empty())
				{
					numOfElementsCutonFat++;
					int i = 0;
					//std::cout << "indexActiveContacts size:" << indexActiveContacts.size() << std::endl;
					static TopologicalChangeManager manager;
					/*while (i < indexActiveContacts.size())
					{
						if ( (*contacts)[i]->hasTag(core::objectmodel::Tag("HapticSurfaceVein")) || (*contacts)[i]->hasTag(core::objectmodel::Tag("HapticSurfaceCurve")) )
						i++;
					}*/
					//const ContactVector::value_type& c = (*contacts)[0];
					const ContactVector::value_type& c = (*contacts)[active_contact_index];
					//std::cout << "in doCarve, active_index = " << active_contact_index << std::endl;
					if (c.elem.first.getCollisionModel() == toolState.modelTool)
						nbelems += manager.removeItemsFromCollisionModel(c.elem.second.getCollisionModel(), elemsToRemove);
					else
						nbelems += manager.removeItemsFromCollisionModel(c.elem.first.getCollisionModel(), elemsToRemove);
				
				}			
			}

			void HapticManager::doIncise()
			{
				const ContactVector* contacts = getContacts();
				if (contacts == NULL) return;
				int nbelems = 0;
				helper::vector<int> incidentTriangles;
				helper::vector<Vector3> incidentPoints;
				ToolModel *tm = toolModel.get();

				for (unsigned int j = 0; j < contacts->size(); ++j)
				{
					const ContactVector::value_type& c = (*contacts)[j];
					core::CollisionModel* surf = (c.elem.first.getCollisionModel() == toolState.modelTool ? c.elem.second.getCollisionModel() : c.elem.first.getCollisionModel());
					if (surf->hasTag(core::objectmodel::Tag("HapticSurfaceVein")) && tm->hasTag(core::objectmodel::Tag("CarvingTool")) && mistatkeTolerance > 0)
					{
						mistatkeTolerance--;
						return;
					}
					sofa::core::topology::TopologicalMapping * topoMapping = surf->getContext()->get<sofa::core::topology::TopologicalMapping>();
					if (topoMapping == NULL) return;
					int triangleIdx = (c.elem.first.getCollisionModel() == toolState.modelTool ? c.elem.second.getIndex() : c.elem.first.getIndex());
					incidentTriangles.push_back(triangleIdx);
					incidentPoints.push_back(c.elem.first.getCollisionModel() == toolState.modelTool ? c.point[1] : c.point[0]);
				}
				sofa::helper::AdvancedTimer::stepBegin("CarveElems");
				std::size_t nCutPoints = incidentPoints.size(); // 
				if (!incidentTriangles.empty() && (incidentTriangles[nCutPoints - 2] != incidentTriangles.back()))
				{
					cout << "begin incise..." << endl;
					static TopologicalChangeManager manager;
					const ContactVector::value_type& c = (*contacts)[0];
					cout << "incident point 2: " << incidentPoints.back() << endl;
					cout << "incident point 1: " << incidentPoints[nCutPoints - 2] << endl;
					cout << "incident tri 2: " << incidentTriangles.back() << endl;
					cout << "incident tri 1: " << incidentTriangles[nCutPoints - 2] << endl;
					bool ok = 0;
					if (c.elem.first.getCollisionModel() == toolState.modelTool)
						ok = manager.incisionCollisionModel(c.elem.second.getCollisionModel(), incidentTriangles[nCutPoints - 2], incidentPoints[nCutPoints - 2], c.elem.second.getCollisionModel(), incidentTriangles.back(), incidentPoints.back());
					else
						ok = manager.incisionCollisionModel(c.elem.first.getCollisionModel(), incidentTriangles[nCutPoints - 2], incidentPoints[nCutPoints - 2], c.elem.first.getCollisionModel(), incidentTriangles.back(), incidentPoints.back());
					if (ok)
						cout << "OK, coord: " << endl;
					else
						cout << "NOT ok, coord: " << endl;
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

						core::behavior::MechanicalState<DataTypes>* mstateCollision1 = toolState.m1->createMapping(GenerateStringID::generate().c_str());
						toolState.m1->resize(1);
						core::behavior::MechanicalState<DataTypes>* mstateCollision2 = toolState.m2->createMapping(GenerateStringID::generate().c_str());
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
				size_t ncontacts = contacts->size();
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
				size_t ss = std::min(toolState.first_idx.size(), second_idx.size());
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

							const sofa::core::topology::Topology::Triangle Triangle1 = triangleContainer->getTriangle(toolState.first_idx[j]);
							const sofa::core::topology::Topology::Triangle Triangle2 = triangleContainer->getTriangle(second_idx[j]);
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

							toolState.ff = sofa::core::objectmodel::New< sofa::component::interactionforcefield::StiffSpringForceField<Vec3dTypes> >(mstate1, mstate2);

							toolState.ff->setName(GenerateStringID::generate().c_str());
							toolState.ff->setArrowSize(0.1f);
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
				if (modelSurfaces.empty()) return;
				ToolModel *toolModelPt = toolModel.get();	
				const ContactVector* contacts = getContacts();
				if (contacts == NULL) return;
				int active_contact_index = 0; // index of the contacts that has the collision model we want to clamp.
				if (contacts->size() > 0)
				{
					for (unsigned int j = 0; j < contacts->size(); j++)//look for the vein
					{
						const ContactVector::value_type& cc = (*contacts)[j];
						
						core::CollisionModel* surf = (cc.elem.first.getCollisionModel() == toolState.modelTool ? cc.elem.second.getCollisionModel() : cc.elem.first.getCollisionModel());
						
						int r = 0;
						if (surf->hasTag(core::objectmodel::Tag("HapticSurfaceVein")))
							r = 1;
					//	cout << "contact : " << j << ", value = " << cc.value << ", is vein? " << r << endl;
					//	if ((surf->hasTag(core::objectmodel::Tag("HapticSurfaceVein"))))
					//	{
					//		//std::cout << "found the vein!" << std::endl;
					//		active_contact_index = j;
					//		break;
					//	}
					}
				}
				const ContactVector::value_type& c = (*contacts)[active_contact_index];
				unsigned int idx1 = (c.elem.first.getCollisionModel() == toolModelPt ? c.elem.second.getIndex() : c.elem.first.getIndex());

				if (idx1 >= 0)
				{
					sofa::component::topology::TriangleSetTopologyContainer* triangleContainer;
					core::CollisionModel* surf = (c.elem.first.getCollisionModel() == toolModelPt ? c.elem.second.getCollisionModel() : c.elem.first.getCollisionModel());
					if (!surf->hasTag(core::objectmodel::Tag("HapticSurfaceVein")))//only the vein is clampable
					{
						return;
						//if (contacts->size() > 1)
						//{
						//	//cout << "found another contact" << endl;
						//	const ContactVector::value_type& c2 = (*contacts)[contacts->size()-1];
						//	surf = (c2.elem.first.getCollisionModel() == toolModelPt ? c2.elem.second.getCollisionModel() : c2.elem.first.getCollisionModel());
						//	if (surf->hasTag(core::objectmodel::Tag("HapticSurfaceVein")))
						//	{
						//		cout << "found vein as 2nd contact" << endl;
						//		idx1 = (c2.elem.first.getCollisionModel() == toolModelPt ? c2.elem.second.getIndex() : c2.elem.first.getIndex());
						//	}
						//	else
						//		return;
						//}
					}				
					surf->getContext()->get(triangleContainer);
					const sofa::core::topology::Topology::Triangle Triangle1 = triangleContainer->getTriangle(idx1);

					sofa::component::topology::HexahedronSetTopologyContainer* hexContainer;
					surf->getContext()->get(hexContainer);
					if (hexContainer == NULL) return;
					core::behavior::MechanicalState<DataTypes>* currentClipperState;
					hexContainer->getContext()->get(currentClipperState);

					const VecCoord& x = currentClipperState->read(core::ConstVecCoordId::position())->getValue();

					sofa::component::interactionforcefield::StiffSpringForceField<Vec3dTypes>::SPtr spring = sofa::core::objectmodel::New< sofa::component::interactionforcefield::StiffSpringForceField<Vec3dTypes> >();
					hexContainer->getContext()->addObject(spring);

					sofa::helper::vector< unsigned int > e1 = hexContainer->getHexahedraAroundVertex(Triangle1[0]);
					sofa::helper::vector< unsigned int > e2 = hexContainer->getHexahedraAroundVertex(Triangle1[1]);
					sofa::helper::vector< unsigned int > e3 = hexContainer->getHexahedraAroundVertex(Triangle1[2]);
					sofa::helper::vector< unsigned int > ie1;
					sofa::helper::vector< unsigned int > ie;
					std::sort(e1.begin(), e1.end());
					std::sort(e2.begin(), e2.end());
					std::sort(e3.begin(), e3.end());
					std::set_intersection(e1.begin(), e1.end(), e2.begin(), e2.end(), std::back_inserter(ie1));
					std::set_intersection(ie1.begin(), ie1.end(), e3.begin(), e3.end(), std::back_inserter(ie));

					const sofa::core::topology::Topology::Hexahedron hex = hexContainer->getHexahedron(ie[0]);
					int idxclip = ie[0];//index of the hex
					std::vector<int>::iterator it;
					it = find(vein_clips_map[surf->getName()].begin(), vein_clips_map[surf->getName()].end(), idxclip);
					if (it != vein_clips_map[surf->getName()].end())
						std::cout << "can not apply clips here!" << *it << '\n';
					else {
						//clipVector.push_back(idxclip);
						vein_clips_map[surf->getName()].push_back(idxclip);
					}
					//std::cout << "current clip vector: ";
					for (auto i = vein_clips_map[surf->getName()].begin(); i != vein_clips_map[surf->getName()].end(); ++i)
						std::cout << *i << ' ';
					std::cout << endl;

					// order of the 3 vertices in the hex (0 to 7)
					int v1 = hexContainer->getVertexIndexInHexahedron(hex, Triangle1[0]);
					int v2 = hexContainer->getVertexIndexInHexahedron(hex, Triangle1[1]);
					int v3 = hexContainer->getVertexIndexInHexahedron(hex, Triangle1[2]);

					const unsigned int vertexMap[6][4] = { { 4, 5, 6, 7 }, { 0, 3, 2, 1 }, { 2, 3, 7, 6 }, { 0, 4, 7, 3 }, { 1, 5, 4, 0 }, { 1, 2, 6, 5 } };
					const unsigned int vertexHex[6][4] = { { 0, 1, 2, 3 }, { 4, 7, 6, 5 }, { 1, 0, 4, 5 }, { 1, 5, 6, 2 }, { 2, 6, 7, 3 }, { 0, 3, 7, 4 } };
					for (int i = 0; i < 6; i++) {
						// call the quad corresponding to the face i
						const sofa::core::topology::Quad& q = hexContainer->getLocalQuadsInHexahedron(i);
						int j;
						// run over vertices of the quad i
						for (j = 0; j < 4; j++) {
							// check if the face contains one of the 3 vertices v1, v2, v3
							if (q[j] == v1 || q[j] == v2 || q[j] == v3) {
								break;
							}
						}
						if (j == 4) { // found the opposite quad face of the collision triangle	
							clipperStates.push_back(currentClipperState);
							clampPairs.push_back(std::make_pair(hex, i));
							double hexLength = -1.0; // max hex edge length
							for (size_t iv = 0; iv < 7; iv++)
							{
								hexLength = std::max(hexLength, (x[hex[iv]] - x[hex[iv + 1]]).norm());
							}
							//cout<<"max hexlength: "<<hexLength<<endl;
							hexDimensions.push_back(hexLength);
							//check if edge 2-1 in the opposite quad face is in the skeleton direction				
							// e1: all hexes containing vertex 1
							// e2: all hexes containing vertex 2
							sofa::helper::vector< unsigned int > e1 = hexContainer->getHexahedraAroundVertex(hex[vertexHex[i][1]]);
							sofa::helper::vector< unsigned int > e2 = hexContainer->getHexahedraAroundVertex(hex[vertexHex[i][2]]);
							sofa::helper::vector< unsigned int > ie; // intersection of e1 and e2							
							std::sort(e1.begin(), e1.end());
							std::sort(e2.begin(), e2.end());
							std::set_intersection(e1.begin(), e1.end(), e2.begin(), e2.end(), std::back_inserter(ie));
							bool isEdge12Along;
							// assume that *hex* is not an outermost thick curve hex
							// if there is only one hex containing both vertices 1 and 2, then edge 1-2 is along the curve direction
							if (ie.size() == 1)
							{
								isEdge12Along = true;
							}
							else if (ie.size() == 2)
							{
								isEdge12Along = false;
							}
							else
							{
								isEdge12Along = true;
								printf("\n HapticManager.cpp: unable to determine proper orientation to clamp, since the object is not a thick curve");
								printf("\n HapticManager.cpp: size of common hexes: %d", (int)ie.size());
							}
							edge12along.push_back(isEdge12Along);

							double thicknessFactor = 15.0;
							//double clippedHexSize = thicknessFactor*intersectionMethod->getContactDistance();
							double clippedHexSize = .7*hexLength; // size of the clip defined relatively from edge length of the original hexes (without springs)
							// if the edge connecting vertices 1 and 2 of the quad face is along the curve direction, then the edges 0-1 and 2-3 are 
							// orthogonal to the curve direction
							/*if (isEdge12Along)
							{
								spring->addSpring(hex[q[0]], hex[q[1]], attach_stiffness.getValue() / 100, 0.0, clippedHexSize);
								spring->addSpring(hex[q[2]], hex[q[3]], attach_stiffness.getValue() / 100, 0.0, clippedHexSize);
								spring->addSpring(hex[vertexMap[i][0]], hex[vertexMap[i][1]], attach_stiffness.getValue() / 100, 0.0, clippedHexSize);
								spring->addSpring(hex[vertexMap[i][2]], hex[vertexMap[i][3]], attach_stiffness.getValue() / 100, 0.0, clippedHexSize);
							}*/
							// if the edge connecting vertices 1 and 2 of the quad face is NOT along the curve direction, then the edges 0-3 and 2-1 are 
							// orthogonal to the curve direction
							/*else
							{
								spring->addSpring(hex[q[0]], hex[q[3]], attach_stiffness.getValue() / 100, 0.0, clippedHexSize);
								spring->addSpring(hex[q[2]], hex[q[1]], attach_stiffness.getValue() / 100, 0.0, clippedHexSize);
								spring->addSpring(hex[vertexMap[i][0]], hex[vertexMap[i][3]], attach_stiffness.getValue() / 100, 0.0, clippedHexSize);
								spring->addSpring(hex[vertexMap[i][2]], hex[vertexMap[i][1]], attach_stiffness.getValue() / 100, 0.0, clippedHexSize);
							}*/
							// adding springs to the remaining direciton
							/*for (size_t iv = 0; iv < 4; iv++)
							{
								spring->addSpring(hex[q[iv]], hex[vertexMap[i][iv]], attach_stiffness.getValue() / 10, 0.0, clippedHexSize);
							}*/
							break;
						}
					}
				} // endif
			}
			bool keyDown = false;
			bool doneContain = false;
			bool doneGrasp = false;
			bool doneClamp = false;
			bool secondHandGrasping = false;
			typedef sofa::helper::Quater<double> Quat;
			Quat secondHandQuatGrasping;
			double lastReleaseTime;
			void HapticManager::updateTool()
			{
				// for the non dominant hand, should always use a grasper with gestures control.
				//if (!newOmniDriver->isDominantHand.getValue())
				//{
				//	double currentTime = this->getContext()->getTime();
				//	if (!secondHandGrasping && currentTime-lastReleaseTime >= 1.0 && getContacts() != NULL && toolState.function == TOOLFUNCTION_GRASP)
				//	{
				//		doGrasp();
				//		secondHandGrasping = true;
				//		secondHandQuatGrasping = newOmniDriver->data.deviceData.quat;
				//		//secondHandQuatGrasping.print();
				//	}
				//	if (secondHandGrasping && toolState.function == TOOLFUNCTION_GRASP)
				//	{
				//		double forceHaptic = 0;
				//		if (newOmniDriver) {
				//			/*forceHaptic = std::pow(newOmniDriver->data.currentForce[0], 2) + std::pow(newOmniDriver->data.currentForce[1], 2) + std::pow(newOmniDriver->data.currentForce[2], 2);
				//			if (forceHaptic >= 10.0) {
				//				cout << "forceHaptic: " << forceHaptic << endl;
				//				secondHandGrasping = false;
				//				unGrasp();
				//				lastReleaseTime = this->getContext()->getTime();
				//			}*/
				//			Quat currentQuat = newOmniDriver->data.deviceData.quat;
				//			Quat copyCurrentQuat = currentQuat;
				//			Quat diffQuat = copyCurrentQuat.quatDiff(copyCurrentQuat, secondHandQuatGrasping);
				//			double* diffQuatArray = diffQuat.ptr();
				//			if (diffQuatArray[3] <= 0.97) {
				//				secondHandGrasping = false;
				//				unGrasp();
				//				lastReleaseTime = this->getContext()->getTime();
				//			}
				//		}
				//			
				//	}					
				//}
				//cout << "toolState.id = " << toolState.id << endl;
				unsigned char newButtonState = toolState.newButtonState;
				const unsigned char FIRST = 1, SECOND = 2;
				keyDown = false;
				if ((toolState.id == 1) && newOmniDriver->key_Q_down)
					keyDown = true;
				else if ((toolState.id == 0) && newOmniDriver->key_W_down)
					keyDown = true;
				switch (toolState.function)
				{
				case TOOLFUNCTION_CARVE:	
					//if ((newButtonState & SECOND) != 0)  doCarve();//Continue carving as long as the first button been pressed down	
					if (((toolState.buttonState ^ newButtonState) & SECOND) != 0 && (newButtonState & SECOND) != 0)//button down
						doCarve();
					else if (keyDown)
						doCarve();
					break;
				case TOOLFUNCTION_CLAMP:
					if (((toolState.buttonState ^ newButtonState) & SECOND) != 0 && (newButtonState & SECOND) != 0)
						doClamp(); /* button down */
					else if (keyDown && !doneClamp) {
						doClamp();
						doneClamp = true;
					}
					else
						doneClamp = false;
					break;
				case TOOLFUNCTION_CONTAIN:
					if (((toolState.buttonState ^ newButtonState) & SECOND) != 0) /* the state of the first button is changing */
					{		
						if ((newButtonState & SECOND) != 0) /* first button down */
						{
							doContain(); 
						}
					}
					else if (keyDown && !doneContain) {
						doContain();
						doneContain = true;
					}
					break;
				case TOOLFUNCTION_GRASP:
					if (((toolState.buttonState ^ newButtonState) & SECOND) != 0)
					{
						/* the state of the first button is changing */
						if ((newButtonState & SECOND) != 0)
						{
							doGrasp(); /* button down */
						}
						else
						{		
							unGrasp(); /* button up */
						}
					}
					else if (keyDown && !doneGrasp) {
						doGrasp();
						//cout << "start grasping..." << endl;
						doneGrasp = true;
					}
					else if (!keyDown && doneGrasp){
						//cout << "ungrasping..." << endl;
						unGrasp();
						doneGrasp = false;
					}
					break;
				case TOOLFUNCTION_SUTURE:
					if (((toolState.buttonState ^ newButtonState) & FIRST) != 0)
					{
						/* the state of the first button is changing */
						if ((newButtonState & FIRST) != 0)
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
				
				//Changes for force feedback safety
				double resultantForce = 0;
				const ContactVector* contacts = getContacts();
				if (contacts != NULL) {
					ToolModel *tm = toolModel.get();
					for (unsigned int j = 0; j < contacts->size(); ++j) {
						const ContactVector::value_type& c = (*contacts)[j];
						core::CollisionModel* surf = (c.elem.first.getCollisionModel() == toolState.modelTool ? c.elem.second.getCollisionModel() : c.elem.first.getCollisionModel());
						if (surf->hasTag(core::objectmodel::Tag("HapticSurfaceVein"))) {
							//TODO: Take the force direction(sign) into account while calculating resultant
							//double resultantForce = sqrt(std::pow(newOmniDriver->data.currentForce[0], 2) + std::pow(newOmniDriver->data.currentForce[1], 2) + std::pow(newOmniDriver->data.currentForce[2], 2));
							
							if (usingAA){
								//resultantForce = sqrt(std::pow(aaOmniDriver->data.currentForce[0], 2) + std::pow(aaOmniDriver->data.currentForce[1], 2) + std::pow(aaOmniDriver->data.currentForce[2], 2));
							}
							else
							{
								if(newOmniDriver)//removed sqrt
									resultantForce = std::pow(newOmniDriver->data.currentForce[0], 2) + std::pow(newOmniDriver->data.currentForce[1], 2) + std::pow(newOmniDriver->data.currentForce[2], 2);
							}
							double safetyForceThreshold = 1000;
							std::string keywordThreshold = "SafetyForceThreshold_";
							sofa::core::objectmodel::TagSet tagSet = surf->getTags();
							std::set<sofa::core::objectmodel::Tag>::iterator it;
							for (it = tagSet.begin(); it != tagSet.end(); ++it) 
							{
								sofa::core::objectmodel::Tag tag = *it;
								std::string tagString(tag);
								if (tagString.find(keywordThreshold) != std::string::npos)
								{
									safetyForceThreshold = atof(tagString.substr(keywordThreshold.length(), tagString.length() - keywordThreshold.length()).c_str());
								}
							}
							if (resultantForce > safetyForceThreshold*safetyForceThreshold && !hasBeenCut(surf->getName()) && this->getContext()->getTime() - last_update_time >= 0.5)
							{
								if (!hasInstrumentTurnedRed)
								{
									string search_string = "vec3 boundaryColor = vec3( 0., 0., 0. );";
									string replace_string = "vec3 boundaryColor = vec3( 1., 0., 0. );";
									hasInstrumentTurnedRed = true;
									updateShader("\\shaders\\TIPSShaders\\instrument.glsl", "\\shaders\\TIPSShaders\\outinstrument.glsl",
										search_string, replace_string);
									last_update_time = this->getContext()->getTime();
								}
								mistatkeTolerance--;
								std::string SharePath = base_path_share;
								std::string capturePath(SharePath + "/TIPS_screenshot/Errors/" + programStartDate + "error");
								mistakeToleranceForce++;
								std::string err("_too_much_force_on_vein.png");
								std::string out = capturePath + std::to_string(resultantForce);
								out = out + err;
								capture.saveScreen(out,5);
								last_update_time = this->getContext()->getTime();
							}
						}
						else if (surf->hasTag(core::objectmodel::Tag("TargetOrgan")) || surf->hasTag(core::objectmodel::Tag("SafetySurface"))) {
							if (toolState.function == TOOLFUNCTION_CARVE && newOmniDriver && this->getContext()->getTime()- last_update_time>=0.1) {
								resultantForce = std::pow(newOmniDriver->data.currentForce[0], 2) + std::pow(newOmniDriver->data.currentForce[1], 2) + std::pow(newOmniDriver->data.currentForce[2], 2);
								if (resultantForce >= 2.2) {
									last_update_time = this->getContext()->getTime();
									std::cout << "detected: safety organ injured..." << std::endl;
								}
									
							}
								
						}
					}
				}
			}

			void HapticManager::handleEvent(Event* event)
			{
				if (dynamic_cast<core::objectmodel::KeypressedEvent *>(event))
				{			
					core::objectmodel::KeypressedEvent *kpe = dynamic_cast<core::objectmodel::KeypressedEvent *>(event);
					//std::cout << "hapticManager : you pressed a key:" << kpe->getKey() << std::endl;
					if (kpe->getKey() == 'Q')
					{
						switch (toolState.function)
						{
						case TOOLFUNCTION_CARVE:
							doCarve();
							break;
						case TOOLFUNCTION_CLAMP:
							doClamp();
							break;
						case TOOLFUNCTION_GRASP:
							break;
						}	
					}
				}
				Controller::handleEvent(event);
			}

			void HapticManager::onHapticDeviceEvent(HapticDeviceEvent* ev)
			{
				//std::cout << "HM Received Haptic Event: devId, buttonstate  =  " << ev->getDeviceId()<<","<< ev->getButtonState() << std::endl;
				if (ev->getDeviceId() == toolState.id) toolState.newButtonState = ev->getButtonState();
			}

			void HapticManager::onEndAnimationStep(const double dt) {

				if (intersectionMethod == NULL || detectionNP == NULL)
				{
					sout << "intersection method or detection NP is missing" << sendl;
					this->f_listening.setValue(false);
				}

				if (hasInstrumentTurnedRed)
				{
					//Sleep(400);
					if (this->getContext()->getTime() - last_update_time >= 0.2)
					{
						hasInstrumentTurnedRed = false;
						string search_string = "vec3 boundaryColor = vec3( 0., 0., 0. );";
						string replace_string = "vec3 boundaryColor = vec3( 1., 0., 0. );";
						updateShader("\\shaders\\TIPSShaders\\instrument.glsl", "\\shaders\\TIPSShaders\\outinstrument.glsl",
							replace_string, search_string);
					}
					
				}
				if (hasInstrumentTurnedGreen && !hasInstrumentTurnedRed)
				{
					if (this->getContext()->getTime() - last_update_time >= 0.1)
					{
						string search_string = "vec3 boundaryColor = vec3( 0., 0., 0. );";
						string replace_string = "vec3 boundaryColor = vec3( 0., 1., 0. );";
						updateShader("\\shaders\\TIPSShaders\\instrument.glsl", "\\shaders\\TIPSShaders\\outinstrument.glsl",
							replace_string, search_string);
						hasInstrumentTurnedGreen = false;
						//Sleep(400);
						if (hasPutInBag){
							hasPutInBag = false;
							this->getContext()->getRootContext()->setAnimate(false);//pause the simulation after the final achievement
						}
					}	
				}
				updateTool();
			}

			int HapticManager::updateShader(string Input, string Output, string searchstring, string replacestring)
			{
				string temp_share_path = base_path_share;
				string input = temp_share_path.append(Input);
				temp_share_path = base_path_share;
				string output = temp_share_path.append(Output);
				//std::cout << "input and output path: " << input << "  " << output << std::endl;
				string search_string = searchstring;
				string replace_string = replacestring;
				string inbuf;
				fstream input_file(input, ios::in);
				ofstream output_file(output);
				while (!input_file.eof())
				{
					getline(input_file, inbuf);

					int spot = inbuf.find(search_string);
					if (spot >= 0)
					{
						string tmpstring = inbuf.substr(0, spot);
						tmpstring += replace_string;
						tmpstring += inbuf.substr(spot + search_string.length(), inbuf.length());
						inbuf = tmpstring;
					}
					output_file << inbuf << endl;
				}
				input_file.close();
				output_file.close();
				int resultRem = remove(input.c_str());
				if (resultRem != 0)
					perror("Error removing file");
				int resultRen = rename(output.c_str(), input.c_str());
				if (resultRen != 0)
					perror("Error renaming file");
				if (resultRem == 0 && resultRen == 0)
					return 1;
				else
					return 0;
			}

			int HapticManager::hasBeenCut(std::string name)
			{
				for (auto s = namesOfVeinCutSet.begin(); s != namesOfVeinCutSet.end(); s++){
					if (*s == name)
						return 1;
				}
				return 0;
			}
			/*int isCutBetweenClips()
			{
			int nbS = 0, nbL = 0, nbE = 0;
			for (auto j = veinCutSet.begin(); j != veinCutSet.end(); ++j)
			{
			for (auto i = clipVector.begin(); i != clipVector.end(); ++i)
			std::cout << *i << ' ';
			if (*i < j)
			nbS++;
			else if (*i == j);
			{
			nbE++;
			return 2;
			}
			else if (*i > j)
			nbL++;
			}
			}*/
		} // namespace collision

	} // namespace component

} // namespace sofa
