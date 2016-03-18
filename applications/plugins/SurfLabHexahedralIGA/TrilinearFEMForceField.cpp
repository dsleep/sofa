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

struct SOFA_EXPORT_DYNAMIC_LIBRARY TrilinearFEMForceField : public virtual sofa::core::behavior::ForceField<DataTypes>  {
  SOFA_CLASS(TrilinearFEMForceField, SOFA_TEMPLATE(sofa::core::behavior::ForceField, DataTypes));
  
  typedef DataTypes::VecCoord VecCoord;
  typedef DataTypes::Coord Coord;
  typedef DataTypes::Coord::value_type real;
  // We only store the lower triangle part of the stiffness matrix
  // as 8x8 matrix of transformation submatrices
  typedef Mat<3, 3, real> Transform;
  typedef Vec<36, Transform> StiffnessMatrix;

  BaseMeshTopology* _mesh;
  vector<Vec<8,Coord> > _rotatedRestElements;
  Vec<3,real> _materialStiffness;
  vector<StiffnessMatrix> _elemStiffness;
  vector<Transform> _elemRotations;

  // The only XML parameters in this module
  Data<real> _poissonRatio, _youngModulus;
  
  TrilinearFEMForceField()
    : _poissonRatio(initData(&_poissonRatio,(real)0.45,"poissonRatio",""))
    , _youngModulus(initData(&_youngModulus,(real)5000,"youngModulus",""))
  {
    _poissonRatio.setRequired(true);
    _youngModulus.setRequired(true);
  }
  
  virtual ~TrilinearFEMForceField() {}
  
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

  Transform& stiffnessSubmatrixLookup(StiffnessMatrix& m, int i, int j) {
    assert(i < 8 && j < 8 && j <= i);
    return m[i * (i+1) /2 + j];
  }

  /* Do the actual initialization */
  virtual void reinit(){
    
    {
      const real E = _youngModulus.getValue(), v = _poissonRatio.getValue();
      const real U = E * (1-v) / ((1+v)*(1-2*v));
      const real V = E *    v  / ((1+v)*(1-2*v));
      const real W = E         / (2*(1+v));
      _materialStiffness = Vec<3, real>(U, V, W);
    }
    
    const VecCoord& restPose = mstate->read(sofa::core::ConstVecCoordId::restPosition())->getValue();
    const BaseMeshTopology::SeqHexahedra &elems = _mesh->getHexahedra();
    // allocate some arrays that cache information about elements
    // including rotations, initial rotations, ...
    _rotatedRestElements.resize(elems.size());
    _elemRotations.resize(elems.size());
    _elemStiffness.resize(elems.size());
    
    // Pre-calcluate information about each element
    for(size_t i = 0; i < elems.size(); i++) {
      Vec<8,Coord> v; 
      for(int j = 0; j < 8; j++) v[j] = restPose[elems[i][index_perm[j]]];
      
      computeRotationPolar(_elemRotations[i], v);
      for(int j = 0; j < 8; j++) v[j] = _elemRotations[i] * v[j];
      _rotatedRestElements[i] = v;
      
      computeElementStiffness(_elemStiffness[i], v);
      
    }
    
  }

  /// Apply sub-stiffness matrices to displacements to
  /// calculate forces
  /// Since the stiffness matrix only contains the lower triangle
  /// we have to do the multiplcation symmetrically as we go on
  /// as in we multiply K[l][m] and K[m][l] since they are the same.
  void applyStiffnessMatrix(StiffnessMatrix& K, const Coord d[8], Deriv f[8]) {
    for(int l = 0; l < 8; l++) {
      f[l].fill(0);
      for (int m = 0; m <= l; m++) {
        const Transform &k = stiffnessSubmatrixLookup(K, l, m);
        f[l] += k * d[m];
        if(l != m) f[m] += k.multTranspose(d[l]);
      }
    }
  }

  virtual void addForce(const sofa::core::MechanicalParams*, Data<VecDeriv>& f, const Data<VecCoord>& x, const Data<VecDeriv>&) {
    
    sofa::helper::WriteAccessor<Data<VecDeriv> > fw = f;
    sofa::helper::ReadAccessor<Data<VecCoord> > xr = x;
    
    const BaseMeshTopology::SeqHexahedra &elems = _mesh->getHexahedra();
    for(size_t i = 0; i < elems.size(); i++) {
      Vec<8,Coord> v; 
      for(int j = 0; j < 8; j++) v[j] = xr[elems[i][index_perm[j]]];
      
      computeRotationPolar(_elemRotations[i], v);
      for(int j = 0; j < 8; j++) v[j] = _elemRotations[i] * v[j];
      
      // should we re-compute element stiffness
      computeElementStiffness(_elemStiffness[i], v);

      Coord D[8]; Deriv F[8];
      for(int k = 0; k < 8; k++)
          D[k] = _rotatedRestElements[i][k] - v[k];

      applyStiffnessMatrix(_elemStiffness[i], D, F);

      for(int j = 0; j < 8; j++)
        fw[elems[i][index_perm[j]]] += _elemRotations[i].multTranspose(F[j]);
        
    }
    
  
  }
  virtual void addDForce(const sofa::core::MechanicalParams* mparams, Data<VecDeriv>& df, const Data<VecDeriv>& dx) {
    sofa::helper::WriteAccessor<Data<VecDeriv> > dfw = df;
    sofa::helper::ReadAccessor<Data<VecCoord> > dxr = dx;
    dfw.resize(dxr.size());
    
    real kFactor = mparams->kFactorIncludingRayleighDamping(rayleighStiffness.getValue());
    
    // apply force for all of the elements
    const BaseMeshTopology::SeqHexahedra &elems = _mesh->getHexahedra();
    for(size_t i = 0; i < elems.size(); i++) {

      Coord D[8]; Deriv F[8];
      for(int j = 0; j < 8; j++)
        D[j] = - _elemRotations[i] * dxr[elems[i][index_perm[j]]];


      applyStiffnessMatrix(_elemStiffness[i], D, F);

      for(int j = 0; j < 8; j++)
        dfw[elems[i][index_perm[j]]] += _elemRotations[i].multTranspose(F[j]) * kFactor;
    }
    
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
  

  void computeElementStiffness(StiffnessMatrix &K, const Vec<8, Coord> &v) {

    for(size_t i = 0; i < K.size(); i++)
      K[i].fill(0);

    const real U = _materialStiffness[0], V = _materialStiffness[1], W = _materialStiffness[2];
    
    // Calculate the Jacobian of the transformation using quadrature points
    // +-1/sqrt 3 scaled to [0,1] range
    Transform J, J_1, J_1t; real detJ; 
    const real isq3 = (real) (1.0/sqrt(3.0));
    for(int i1 = 0; i1 < 2; i1++) for(int i2 = 0; i2 < 2; i2++) for(int i3 = 0; i3 < 2; i3++) {
      const real x1 = (2*i1-1)*isq3;
      const real x2 = (2*i2-1)*isq3;
      const real x3 = (2*i3-1)*isq3;
      const real t1 = (x1 + 1)/2;
      const real t2 = (x2 + 1)/2;
      const real t3 = (x3 + 1)/2;
      
      J = Transform(
        ( (v[1]-v[0])*(1-t2)*(1-t3)+ (v[3]-v[2])*t2*(1-t3) + (v[5]-v[4])*(1-t2)*t3 + (v[7]-v[6])*t2*t3 ) ,
        ( (v[2]-v[0])*(1-t1)*(1-t3)+ (v[3]-v[1])*t1*(1-t3) + (v[6]-v[4])*(1-t1)*t3 + (v[7]-v[5])*t1*t3 ) ,
        ( (v[4]-v[0])*(1-t1)*(1-t2)+ (v[5]-v[1])*t1*(1-t2) + (v[6]-v[2])*(1-t1)*t2 + (v[7]-v[3])*t1*t2 )
      );
      J.transpose();
      J_1.invert(J);
      J_1t.transpose(J_1);
      detJ = sofa::defaulttype::determinant(J);

      // q_i is the derivative of the control point with respect 
      // to physical domain (dN_i/dx)
      // to do that, we first calculate dNi_du with respect to parameter
      // domain and then multiply that by J_1 to get the dNi_dx
      Vec<3, real> q[8];
      for(int i = 0; i < 8; i++) {
        int s0 = 2 * (i & 1) - 1, s1 = 2 * (i >> 1 & 1) - 1, s2 = 2 * (i >> 2 & 1) - 1;
        Vec<3, real> dNi_du(
             s0    *(1+s1*x2)*(1+s2*x3)/4 ,
          (1+s0*x1)*   s1    *(1+s2*x3)/4 ,
          (1+s0*x1)*(1+s1*x2)*   s2    /4
        );
        q[i] = J_1t * dNi_du;
      }
      
      // M = [ U V V 0 0 0 ]
      //     [ V U V 0 0 0 ]
      //     [ V V U 0 0 0 ]
      //     [ 0 0 0 W 0 0 ]
      //     [ 0 0 0 0 W 0 ]
      //     [ 0 0 0 0 0 W ]
      
      // Now use the corner points to calculate stiffness matrices
      for(int i = 0; i < 8; i++) for(int j = 0; j <= i; j++) {
        Transform k; // k = Bjt M Bi
        k[0][0] = q[i][0] * U * q[j][0] + q[i][1] * W * q[j][1] + q[i][2] * W * q[j][2];
        k[0][1] = q[i][0] * V * q[j][1] + q[i][1] * W * q[j][0]                        ;
        k[0][2] = q[i][0] * V * q[j][2] +                         q[i][2] * W * q[j][0];

        k[1][0] = q[i][1] * V * q[j][0] + q[i][0] * W * q[j][1];
        k[1][1] = q[i][1] * U * q[j][1] + q[i][0] * W * q[j][0] + q[i][2] * W * q[j][2];
        k[1][2] = q[i][1] * V * q[j][2] +                         q[i][2] * W * q[j][1];
        
        k[2][0] = q[i][2] * V * q[j][0] +                         q[i][0] * W * q[j][2];
        k[2][1] = q[i][2] * V * q[j][1] + q[i][1] * W * q[j][2]                        ;
        k[2][2] = q[i][2] * U * q[j][2] + q[i][1] * W * q[j][1] + q[i][0] * W * q[j][0];
        
        k = k * detJ / 8;

        stiffnessSubmatrixLookup(K, i, j) += k;
      }
    }    
  }
  
  
};


SOFA_DECL_CLASS(TrilinearFEMForceField)

int TrilinearHexahedralCorotationalFEMForceFieldClass = 
  sofa::core::RegisterObject("Trilinear hexahedral corotational FEM force field")
  .add<TrilinearFEMForceField>();
  

