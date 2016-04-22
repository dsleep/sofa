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
				, object1(initLink("object1", "First object to connect to"))
				, object2(initLink("object2", "Second object to connect to"))
				, useConstraint(initData(&useConstraint,true,"useConstraint", "Second object to connect to"))
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
					MechanicalModel* obj1 = object1.get();
					MechanicalModel* obj2 = object2.get();
					
					sofa::simulation::Node* parent = dynamic_cast<sofa::simulation::Node*>(obj2->getContext());
					if (parent == NULL)
					{
						std::cerr << "ERROR: Can't find node \n";
						return;
					}
					sofa::simulation::Node::SPtr child = parent->createChild("ObjectMapping");
					sofa::component::container::MechanicalObject<DataTypes>::SPtr mstate = sofa::core::objectmodel::New<sofa::component::container::MechanicalObject<DataTypes> >();
					mstate->resize(3);
					child->addObject(mstate);					
					sofa::component::topology::TriangleSetTopologyContainer* triangleContainer;
					obj2->getContext()->get(triangleContainer, core::objectmodel::BaseContext::SearchDown);
					if (triangleContainer == NULL) {
						std::cerr << "ERROR: No triangle container in scope of " << obj2->getClassName();
						return;
					}
					MMapper::SPtr mapper = sofa::core::objectmodel::New<sofa::component::mapping::BarycentricMapperMeshTopology<DataTypes, DataTypes> >(triangleContainer, (topology::PointSetTopologyContainer*)NULL);
					mapper->maskFrom = &obj2->forceMask;
					mapper->maskTo = &mstate->forceMask;
					MMapping::SPtr mapping = sofa::core::objectmodel::New<MMapping>(obj2, mstate.get(), mapper);
					child->addObject(mapping);

					TConstraint::SPtr constraints;
					TSpringFF::SPtr ff;
					if (useConstraint.getValue())
						constraints = sofa::core::objectmodel::New<TConstraint>(obj1, mstate.get());
					else
						ff = sofa::core::objectmodel::New<TSpringFF>(obj1, mstate.get());
					
					const VecCoord& x1 = obj1->read(core::ConstVecCoordId::position())->getValue();
					const VecCoord& x2 = obj2->read(core::ConstVecCoordId::position())->getValue();
															
					helper::vector<unsigned int>  idx1 = m_indices1.getValue();
					helper::vector<unsigned int>  idx2 = m_indices2.getValue();
					for (int i = 0; i < idx1.size(); i++) {
						int index1 = idx1[i];
						Coord Q;
						Coord P = x1[index1];
						Vec3d normal;
						int index2;
						double bary[3];
						double dist;
						const sofa::helper::vector< unsigned int > tlist = triangleContainer->getTrianglesAroundVertex(idx2[i]);
						
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
							if (!(bary[0] < 0 || bary[1] < 0 || bary[0]+bary[1] > 1))
							{
								normal.normalize();
								dist = (AP*normal);
								Q = P - dist*normal;
								index2 = tlist[j];
								/*std::cout << P<<" "<<t<<" " <<bary[0] << " " << bary[1] << " " << bary[2] << " " << ":" << Q << std::endl;
								std::cout << x2[t[0]] << " " << x2[t[1]] << " " << x2[t[2]] << " " << std::endl;*/
								break;
							}							
						}
						
						// Add constraint
						int mapIdx = mapper->addPointInTriangle(index2, bary);
						
						if (useConstraint.getValue())
							constraints->addContact(-normal, P, Q, dist, index1, mapIdx, P, Q);
						else
							ff->addSpring(index1, mapIdx, 1e12, 0.0, Q - P);

						projPnts.push_back(Q);
					}
					
					if (useConstraint.getValue())
						obj2->getContext()->addObject(constraints);
					else
						obj2->getContext()->addObject(ff);
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
