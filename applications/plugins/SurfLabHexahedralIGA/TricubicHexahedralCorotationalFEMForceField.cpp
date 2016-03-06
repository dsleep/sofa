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
#include <sofa/helper/system/config.h>

#include <sofa/core/behavior/ForceField.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/helper/decompose.h>
#include <sofa/core/ObjectFactory.h>

#ifndef SOFA_NEW_HEXA
#pragma error "New hexa is required"
#endif
// SOFA uses the following layout for hexahedra
// We would rather use a hyper-cube layout.
//       SOFA Topology layout
//        7---------6
//       /|        /|
//      / |       / |
//     3---------2  |
//     |  |      |  |
//     |  4------|--5     Y
//     | /       | /      | Z
//     |/        |/       |/
//     0---------1        o----X
//
//      Our layout
//        6---------7
//       /|        /|
//      / |       / |
//     2---------3  |
//     |  |      |  |
//     |  4------|--5     Y
//     | /       | /      | Z
//     |/        |/       |/
//     0---------1        o----X
//
//  So the permutation to convert from SOFA layout to ours is
const int index_perm[8] = { 0, 1, 3, 2, 4, 5, 7, 6 };


// Instead of template, we just define DataTypes, this can later be
// converted to a template
typedef sofa::defaulttype::Vec3dTypes DataTypes;
using sofa::helper::vector;
using sofa::defaulttype::Vec;
using sofa::defaulttype::Mat;
using sofa::core::objectmodel::Data;
using sofa::core::topology::BaseMeshTopology;

struct SOFA_EXPORT_DYNAMIC_LIBRARY TricubicHexahedralCorotationalFEMForceField : public virtual sofa::core::behavior::ForceField<DataTypes> {
  SOFA_CLASS(TricubicHexahedralCorotationalFEMForceField, SOFA_TEMPLATE(sofa::core::behavior::ForceField, DataTypes));
  
  typedef DataTypes::VecCoord VecCoord;
  typedef DataTypes::Coord Coord;
  typedef DataTypes::Coord::value_type real;
  typedef Mat<3, 3, real> Transform;


  BaseMeshTopology* _mesh;
  Vec<3,real> _materialStiffness;

  // The only XML parameters in this module
  Data<real> _poissonRatio, _youngModulus;

  TricubicHexahedralCorotationalFEMForceField()
      : _poissonRatio(initData(&_poissonRatio,(real)0.45,"poissonRatio",""))
      , _youngModulus(initData(&_youngModulus,(real)5000,"youngModulus",""))
  {
    _poissonRatio.setRequired(true);
    _youngModulus.setRequired(true);
  }

  virtual ~TricubicHexahedralCorotationalFEMForceField() {}

  /* Read the topology, the rest is done in reinit */
  virtual void init(){
    sofa::core::behavior::ForceField<DataTypes>::init();
    _mesh = getContext()->getMeshTopology();
    if(_mesh == NULL || _mesh->getNbHexahedra() <= 0) {
      serr << "Object must have a hexahedral topology" << sendl;
      return;
    }

    reinit();
  }

  /* Do the actual initialization */
  virtual void reinit() {

    {
      const real E = _youngModulus.getValue(), v = _poissonRatio.getValue();
      const real U = E * (1 - v) / ((1 + v) * (1 - 2 * v));
      const real V = E * v / ((1 + v) * (1 - 2 * v));
      const real W = E / (2 * (1 + v));
      _materialStiffness = Vec<3, real>(U, V, W);
    }

    const VecCoord &restPose = mstate->read(sofa::core::ConstVecCoordId::restPosition())->getValue();
    const BaseMeshTopology::SeqHexahedra &elems = _mesh->getHexahedra();
    // TODO: calculate rest pose here
    // First we have to convert each hexahedra to a patch. By discovering its
    // neighbors and assembling all of them into a 4x4x4 patch. But that is only
    // for the regular case. In case of extra-ordinary points we have to have a way
    // to accomodate all the neighbors around the extra-ordinary vertex.
    
    // How do we traverse, though. In case of surfaces, we use the edge map. Here we
    // don't have a traversal strategy around a vertex. But even on surfaces, the traversal
    // strategy is around a line, so it is a boundary of the surface. So here for traversing 
    // around the vertex in 3D, we can go to the boundary of the 3D volume and traverse the surface
    
    // One way around it would be the quarter-edge data structure. Just like half edge, it is
    // an edge with some orientation. One orientation is which face it is on, just like half-edge
    // another is which side of the face it is on. 
    // In a hexahedral mesh, every face belongs to two volumes. and every edge on it can be thought
    // of belonging to one side or the other. So we call this quarter edge, because every edge not only
    // shared between two adjacent faces, but is also shared between half-faces that form a complete face.
    // This way, every quarter edge has two twins. Or we could call them twin and mirror. The twin edge
    // Is the same as in surfaces, it is the other quarter edge going in opposite direction belonging to the adjacent
    // face. The mirror is also a quarter edge going in oppsite direction, but it belongs to the face opposite
    // from current face, the other side of the face, or the mirror.
    
  }

  virtual void addForce(const sofa::core::MechanicalParams*, Data<VecDeriv>& f, const Data<VecCoord>& x, const Data<VecDeriv>&) {

    sofa::helper::WriteAccessor<Data<VecDeriv> > fw = f;
    sofa::helper::ReadAccessor<Data<VecCoord> > xr = x;

    // TODO: calculate force
  }

  virtual void addDForce(const sofa::core::MechanicalParams* mparams, Data<VecDeriv>& df, const Data<VecDeriv>& dx) {
    sofa::helper::WriteAccessor<Data<VecDeriv> > dfw = df;
    sofa::helper::ReadAccessor<Data<VecCoord> > dxr = dx;
    dfw.resize(dxr.size());

    real kFactor = mparams->kFactorIncludingRayleighDamping(rayleighStiffness.getValue());
    // TODO: Calculate force derivative
  }

  virtual SReal getPotentialEnergy(const sofa::core::MechanicalParams*, const Data<VecCoord>&) const {
    assert("not implemented, pot eng");
    return 0.0;
  }
  virtual SReal getPotentialEnergy(const sofa::core::MechanicalParams*)const {
    assert("not implemented, pot eng");
    return 0.0;
  }

  //// Auxiliary functions ////////////////////////////////////////////////////////////////////

  void computeRotationPolar( Transform &R, Vec<8,Coord> v) {
    Transform A(
        (v[1]-v[0]+v[3]-v[2]+v[5]-v[4]+v[7]-v[6])*0.25f, // 1-skip
        (v[2]-v[0]+v[3]-v[1]+v[6]-v[4]+v[7]-v[5])*0.25f, // 2-skip
        (v[4]-v[0]+v[5]-v[1]+v[6]-v[2]+v[7]-v[3])*0.25f  // 4-skip
    );
    sofa::helper::Decompose<real>::polarDecomposition(A, R);
  }

};

SOFA_DECL_CLASS(TricubicHexahedralCorotationalFEMForceField);

int TricubicHexahedralCorotationalFEMForceFieldClass =
    sofa::core::RegisterObject("Tricubic hexahedral corotational force field FEM")
        .add<TricubicHexahedralCorotationalFEMForceField>();
  

