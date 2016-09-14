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
#include "ConnectingTissue.h"

#include <sofa/defaulttype/Mat.h>
namespace sofa
{
	namespace component
	{
		namespace constraintset
		{
			using sofa::component::controller::Controller;
			using namespace sofa::core::objectmodel;

			SOFA_DECL_CLASS(ConnectingTissue)

				int ConnectingTissueClass = core::RegisterObject("Tissue that connects two objects")
				.add< ConnectingTissue >()
				;


			ConnectingTissue::ConnectingTissue ()
				: m_indices1(initData(&m_indices1, "indices1", "vertices of the first model "))
				, m_indices2(initData(&m_indices2, "indices2", "corresponding vertices of the second model "))
				, threshold(initData(&threshold, 0.1, "threshold", "if indices1 is empty, we consider all pair with distance less than the threshold"))
				, object1(initLink("object1", "First object to connect to"))
				, object2(initLink("object2", "Second object to connect to"))
				, useConstraint(initData(&useConstraint,true,"useConstraint", "Second object to connect to"))
				, connectingStiffness(initData(&connectingStiffness, 3000.0, "connectingStiffness", "stiffness of springs if useConstraint is false"))
        , naturalLength(initData(&naturalLength, 0.5, "naturalLength", "natural length of springs as a percentage of the 2-node-distance"))
        
			{
				this->f_listening.setValue(true);
			}

			ConnectingTissue::~ConnectingTissue()
			{
			}
			void ConnectingTissue::init()
			{

			}

			void ConnectingTissue::bwdInit()
			{	
				if (object1 && object2) {
					simulation::Node* obj1 = object1.get();
					simulation::Node* obj2 = object2.get();
												
					sofa::component::topology::TriangleSetTopologyContainer* triangleContainer;
					obj2->getContext()->get(triangleContainer, core::objectmodel::BaseContext::SearchDown);
					if (triangleContainer == NULL) {
						std::cerr << "ERROR: No triangle container in scope of " << obj2->getName();
						return;
					}
					MechanicalModel *mstate2;
					triangleContainer->getContext()->get(mstate2);
					if (mstate2 == NULL) {
						std::cerr << "ERROR: No MechanicalState in scope of " << triangleContainer->getContext()->getName();
						return;
					}
					MechanicalModel *mstate1;
					obj1->getContext()->get(mstate1);

					sofa::simulation::Node::SPtr child = obj2->createChild("ObjectMapping");
					sofa::component::container::MechanicalObject<DataTypes>::SPtr mstate = sofa::core::objectmodel::New<sofa::component::container::MechanicalObject<DataTypes> >();
					child->addObject(mstate);					

					MMapper::SPtr mapper = sofa::core::objectmodel::New<sofa::component::mapping::BarycentricMapperMeshTopology<DataTypes, DataTypes> >(triangleContainer, (topology::PointSetTopologyContainer*)NULL);
					mapper->maskFrom = &mstate2->forceMask;
					mapper->maskTo = &mstate->forceMask;
					MMapping::SPtr mapping = sofa::core::objectmodel::New<MMapping>(mstate2, mstate.get(), mapper);
					child->addObject(mapping);

					TConstraint::SPtr constraints;
					TSpringFF::SPtr ff;
					if (useConstraint.getValue())
						constraints = sofa::core::objectmodel::New<TConstraint>(mstate1, mstate.get());
					else
						ff = sofa::core::objectmodel::New<TSpringFF>(mstate1, mstate.get());
					
					
					const VecCoord& x1 = mstate1->read(core::ConstVecCoordId::position())->getValue();
					const VecCoord& x2 = mstate2->read(core::ConstVecCoordId::position())->getValue();
															
					helper::vector<unsigned int>  idx1 = m_indices1.getValue();
					helper::vector<unsigned int>  idx2 = m_indices2.getValue();
					if (idx1.empty()) {
						idx2.clear();
						Real th = threshold.getValue();
						for (int i = 0; i < x1.size(); i++) {
							Coord P = x1[i];
							double min_dist = 1e6;
							int qidx = 0;
							for (int j = 0; j < x2.size(); j++){
								double len2 = (P - x2[j])*(P - x2[j]);
								if (len2 < min_dist) {
									min_dist = len2;
									qidx = j;
								}
							}
							if (min_dist < th) {
								idx1.push_back(i);
								idx2.push_back(qidx);
							}
						}
					}
					
					for (int i = 0; i < idx1.size(); i++) {
						int index1 = idx1[i];
						Coord P = x1[index1];
						// find the closest point on object 2 to P
						int qidx;
						if (idx2.empty()) {
							qidx = 0;
							double min_dist = 1e6;
							for (int j = 0; j < x2.size(); j++){
								double len2 = (P - x2[j])*(P - x2[j]);
								if (len2 < min_dist) {
									min_dist = len2;
									qidx = j;
								}
							}
						}
						else {
							qidx = idx2[i];
						}
						// get triangle list around the found vertex
						const sofa::helper::vector< unsigned int > tlist = triangleContainer->getTrianglesAroundVertex(qidx);

						// compute the shortest distance and barycentric coordinates
						Vec3d normal;
						Coord Q;
						int index2 = -1;
						double bary[] = { 0, 0, 0 };
						double dist;

						for (int j = 0; j < tlist.size(); j++) {
							// Find the projection
							const component::topology::Triangle t = triangleContainer->getTriangle(tlist[j]);
							Vec3d AB = x2[t[1]] - x2[t[0]];
							Vec3d AC = x2[t[2]] - x2[t[0]];
							Vec3d AP = P - x2[t[0]];
							normal = AB.cross(AC);
							/**due to weird calculation at line 1227 in BarycentricMapping.inl, we have to shift the barycentric coordinate
							1 -> 2, 0 -> 1, 2 -> 0
							*/
							bary[1] = normal*(AB.cross(AP)) / (normal*normal);
							bary[0] = normal*(AP.cross(AC)) / (normal*normal);
							bary[2] = 1 - bary[1] - bary[0];
							if (!(bary[0] < 0 || bary[1] < 0 || bary[0] + bary[1] > 1))
							{
								normal.normalize();
								dist = (AP*normal);
								Q = P - dist*normal;
								index2 = tlist[j];
								break;
							}
						}
						//sout << P << "->" << Q << sendl;
						// Add constraint
						if (index2 >= 0) {
							int mapIdx = mapper->addPointInTriangle(index2, bary);

							if (useConstraint.getValue())
								constraints->addContact(-normal, P, Q, dist, index1, mapIdx, P, Q);
							else
              {
                ff->addSpring(index1, mapIdx, connectingStiffness.getValue(), 0.0, (Q - P)*naturalLength.getValue());
              }	

							projPnts.push_back(Q);
						}
						else {
							const component::topology::Triangle t = triangleContainer->getTriangle(tlist[0]);
							int localIndex = triangleContainer->getVertexIndexInTriangle(t, qidx);
							bary[0] = 0; bary[1] = 0; bary[2] = 0;
							bary[(localIndex + 2) % 3] = 1.0;
							int mapIdx = mapper->addPointInTriangle(tlist[0], bary);
							Q = x2[qidx];
							if (useConstraint.getValue())
								constraints->addContact(-normal, P, Q, dist, index1, mapIdx, P, Q);
							else
              {
                ff->addSpring(index1, mapIdx, connectingStiffness.getValue(), 0.0, (Q - P)*naturalLength.getValue());
              }
								

							projPnts.push_back(Q);
						}
					}
					
					
					if (useConstraint.getValue())
						this->getContext()->addObject(constraints);
					else
						this->getContext()->addObject(ff);
				}

			}
			
			void ConnectingTissue::reset()
			{
			}
				
			
			void ConnectingTissue::updateVisual()
			{
				
			}
			
			void ConnectingTissue::drawVisual(const core::visual::VisualParams* vparams)
			{
				if (!vparams->displayFlags().getShowBehaviorModels()) return;
				vparams->drawTool()->drawPoints(projPnts, 3.0, Vec4d(1, 0, 0, 1));
			}				

			void ConnectingTissue::handleEvent(Event* event)
			{
				Controller::handleEvent(event);
			}

			void ConnectingTissue::onHapticDeviceEvent(HapticDeviceEvent* ev)
			{
				
			}
			
			void ConnectingTissue::onEndAnimationStep(const double dt) {
				
			}

		} // namespace constraintset

	} // namespace component

} // namespace sofa
