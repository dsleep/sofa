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
#include <sofa/helper/vector.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/helper/decompose.h>
#include <sofa/defaulttype/Vec3Types.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/decompose.h>
#include <assert.h>

#ifndef SOFA_NEW_HEXA
  #pragma error "New hexa is required"
#endif

// Instead of tempalte, we just define DataTypes, this can later be
// converted to a template
typedef sofa::defaulttype::Vec3dTypes DataTypes;
using sofa::helper::vector;
using sofa::defaulttype::Vec;
using sofa::defaulttype::Mat;
using sofa::core::objectmodel::Data;

struct SOFA_EXPORT_DYNAMIC_LIBRARY TrilinearHexahedralCorotationalFEMForceField : public virtual sofa::core::behavior::ForceField<DataTypes>  {
  SOFA_CLASS(TrilinearHexahedralCorotationalFEMForceField, SOFA_TEMPLATE(sofa::core::behavior::ForceField, DataTypes));
  
  typedef DataTypes::VecCoord VecCoord;
  typedef DataTypes::Coord Coord;
  typedef DataTypes::Coord::value_type real;
  typedef Data<DataTypes::VecDeriv> DataVecDeriv;
  typedef Data<DataTypes::VecCoord> DataVecCoord;
  typedef Mat<24,24,real> StiffnessMatrix;
  typedef Mat<3, 3, real> Transform;
  typedef sofa::core::topology::BaseMeshTopology::SeqHexahedra VecElement;
  
  sofa::core::topology::BaseMeshTopology* _mesh;
  vector<Vec<8,Coord> > _rotatedRestElements;
  Vec<3,real> _materialStiffness;
  vector<StiffnessMatrix> _elemStiffness;
  vector<Transform> _elemRotations;
  
  Data<real> _poissonRatio, _youngModulus;
  
  TrilinearHexahedralCorotationalFEMForceField() 
    : _poissonRatio(initData(&_poissonRatio,(real)0.45,"poissonRatio",""))
    , _youngModulus(initData(&_youngModulus,(real)5000,"youngModulus",""))
  {
    _poissonRatio.setRequired(true);
    _youngModulus.setRequired(true);
  }
  
  virtual ~TrilinearHexahedralCorotationalFEMForceField() {}
  
  /* Read the topology, the rest is done in reinit */
  virtual void init(){
    sofa::core::behavior::ForceField<DataTypes>::init();
    _mesh = dynamic_cast<sofa::core::topology::BaseMeshTopology*>(getContext()->getMeshTopology());
    if(_mesh == NULL || _mesh->getNbHexahedra() <= 0) {
      serr << "Object must have a hexahedral topology" << sendl;
      return;
    }
    
    reinit();
  }
  
 
  /* Do the actual initialization */
  virtual void reinit(){
    const VecCoord& restPose = mstate->read(sofa::core::ConstVecCoordId::restPosition())->getValue();
    
    {
      const real E = _youngModulus.getValue(), v = _poissonRatio.getValue();
      const real U = E * (1-v) / ((1+v)*(1-2*v));
      const real V = E *    v  / ((1+v)*(1-2*v));
      const real W = E         / (2*(1+v));
      _materialStiffness = Vec<3, real>(U, V, W);
    }
    
    const VecElement &elems = _mesh->getHexahedra();
    // allocate some arrays that cache information about elements
    // including rotations, initial rotations, ...
    _rotatedRestElements.resize(elems.size());
    _elemRotations.resize(elems.size());
    _elemStiffness.resize(elems.size());
    
    // Pre-calcluate information about each element
    for(size_t i = 0; i < elems.size(); i++) {
      Vec<8,Coord> v; 
      for(int j = 0; j < 8; j++) v[j] = restPose[elems[i][j]];
      
      computeRotationPolar(_elemRotations[i], v);
      for(int j = 0; j < 8; j++) v[j] = _elemRotations[i] * v[j];
      _rotatedRestElements[i] = v;
      
      computeElementStiffness(_elemStiffness[i], v);
      
    }
    
  }
  
  virtual void addForce(const sofa::core::MechanicalParams*, DataVecDeriv& f, const DataVecCoord& x, const DataVecDeriv&) {
    
    sofa::helper::WriteAccessor<Data<VecDeriv> > fw = f;
    sofa::helper::ReadAccessor<Data<VecCoord> > xr = x;
    
    // maybe reinit() in case topology has changed
    
    const VecElement &elems = _mesh->getHexahedra();
    for(size_t i = 0; i < elems.size(); i++) {
      Vec<8,Coord> v; 
      for(int j = 0; j < 8; j++) v[j] = xr[elems[i][j]];
      
      computeRotationPolar(_elemRotations[i], v);
      for(int j = 0; j < 8; j++) v[j] = _elemRotations[i] * v[j];
      
      // should we re-compute element stiffness
      computeElementStiffness(_elemStiffness[i], v);
      
      // Apply the transformation, this is really really silly and stupid
      // who the fuck flattens tensors.
      
      Vec<24, real> D, F;
      for(int k = 0; k < 8; k++) for(int j = 0; j < 3; j++)
        D[k*3+j] = _rotatedRestElements[i][k][j] - v[k][j];
        
      F = _elemStiffness[i] * D;
      
      
      for(int j = 0; j < 8; j++)
        fw[elems[i][j]] += _elemRotations[i].multTranspose( Deriv(F[j * 3], F[j * 3 + 1], F[j * 3 + 2]) );
        
    }
    
  
  }
  virtual void addDForce(const sofa::core::MechanicalParams* mparams, DataVecDeriv& df, const DataVecDeriv& dx) {
    sofa::helper::WriteAccessor<Data<VecDeriv> > dfw = df;
    sofa::helper::ReadAccessor<Data<VecCoord> > dxr = dx;
    dfw.resize(dxr.size());
    
    real kFactor = mparams->kFactorIncludingRayleighDamping(rayleighStiffness.getValue());
    
    // apply force for all of the elements
    const VecElement &elems = _mesh->getHexahedra();
    for(size_t i = 0; i < elems.size(); i++) {
    
      // for an element, apply the pre-computed rotation frame to 
      // the displacement
      // accumulate all 8 displacement in a flat vector D
      Vec<24, real> D, F;
      for(int j = 0; j < 8; j++) {
        Coord v = _elemRotations[i] * dxr[elems[i][j]];
        D[j*3+0] = v[0];
        D[j*3+1] = v[1];
        D[j*3+2] = v[2];
      }
      
      // apply stiffness matrix to displacements to find forces
      F = _elemStiffness[i] * D;
      
      // rotate forces back into the original frame while extracting them 
      // from the flat vector
      for(int j = 0; j < 8; j++)
        dfw[elems[i][j]] += _elemRotations[i].multTranspose( Deriv(F[j * 3], F[j * 3 + 1], F[j * 3 + 2]) ) * kFactor;
    }    
    
  }
  
  virtual SReal getPotentialEnergy(const sofa::core::MechanicalParams*, const DataVecCoord&) const {
    assert("not implemented, pot eng");
    return 0.0;
  }
  virtual SReal getPotentialEnergy(const sofa::core::MechanicalParams*)const {
    assert("not implemented, pot eng");
    return 0.0;
  }
  

  //// Auxiliary functions 
  
  void computeRotationPolar( Transform &R, Vec<8,Coord> v) {
    Transform A(
      (v[1]-v[0]+v[2]-v[3]+v[5]-v[4]+v[6]-v[7])*0.25f,
      (v[3]-v[0]+v[2]-v[1]+v[7]-v[4]+v[6]-v[5])*0.25f,
      (v[4]-v[0]+v[5]-v[1]+v[7]-v[3]+v[6]-v[2])*0.25f
    );
    A.transpose();
    sofa::helper::Decompose<real>::polarDecomposition(A, R);
  }
  
  void computeElementStiffness(StiffnessMatrix &K, const Vec<8, Coord> &v) {
  
    // coefficients for where the corner points are respective to the center of
    // the element
    const int coef[8][3] = {
      { -1, -1, -1 },
      {  1, -1, -1 },
      {  1,  1, -1 },
      { -1,  1, -1 },
      { -1, -1,  1 },
      {  1, -1,  1 },
      {  1,  1,  1 },
      { -1,  1,  1 }
    };
  
    K.fill(0);
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
        ( (v[1]-v[0])*(1-t2)*(1-t3)+ (v[2]-v[3])*t2*(1-t3) + (v[5]-v[4])*(1-t2)*t3 + (v[6]-v[7])*t2*t3 ) /2 ,
        ( (v[3]-v[0])*(1-t1)*(1-t3)+ (v[2]-v[1])*t1*(1-t3) + (v[7]-v[4])*(1-t1)*t3 + (v[6]-v[5])*t1*t3 ) /2,
        ( (v[4]-v[0])*(1-t1)*(1-t2)+ (v[5]-v[1])*t1*(1-t2) + (v[6]-v[2])*(1-t1)*t2 + (v[7]-v[3])*t1*t2 ) /2       
      );
      J.transpose();
      J_1.invert(J);
      J_1t.transpose(J_1);
      detJ = sofa::defaulttype::determinant(J);
    
      // whatever qx, qy, qz represent, they are probably deformation
      // gradients, but why 8 of them?
      // qi are contributions of each corner vertex
      Vec<3, real> q[8];
      for(int i = 0; i < 8; i++) {
        // TODO: what is the point of divide by 8 here, I don't exactly
        // know the formula for dNi_dx but it looks weird.
        Vec<3, real> dNi_dx(
             coef[i][0]    *(1+coef[i][1]*x2)*(1+coef[i][2]*x3) / 8.0,
          (1+coef[i][0]*x1)*   coef[i][1]    *(1+coef[i][2]*x3) / 8.0,
          (1+coef[i][0]*x1)*(1+coef[i][1]*x2)*   coef[i][2]     / 8.0
        );
        q[i] = J_1t * dNi_dx;
      }
      
      // Now use the corner points to calculate stiffness matrices
      for(int i = 0; i < 8; i++) for(int j = i; j < 8; j++) {
        Mat<3, 3, real> k; // k = Bjt M Bi
        k[0][0] = q[j][0] * U * q[i][0] + q[j][1] * W * q[i][1] + q[j][2] * W * q[i][2];
        k[0][1] = q[j][0] * V * q[i][1] + q[j][1] * W * q[i][0]                        ;
        k[0][2] = q[j][0] * V * q[i][2] +                         q[j][2] * W * q[i][0];

        k[1][0] = q[j][1] * V * q[i][0] + q[j][0] * W * q[i][1];
        k[1][1] = q[j][1] * U * q[i][1] + q[j][0] * W * q[i][0] + q[j][2] * W * q[i][2];
        k[1][2] = q[j][1] * V * q[i][2] +                         q[j][2] * W * q[i][1];
        
        k[2][0] = q[j][2] * V * q[i][0] +                         q[j][0] * W * q[i][2];
        k[2][1] = q[j][2] * V * q[i][1] + q[j][1] * W * q[i][2]                        ;
        k[2][2] = q[j][2] * U * q[i][2] + q[j][1] * W * q[i][1] + q[j][0] * W * q[i][0];
        
        k = J_1t * k * J_1 * detJ;
        
        for(int m = 0; m < 3; m++) for(int l = 0; l < 3; l++) 
          K[i*3+m][j*3+l] += k[l][m];
      }
    }    
    // We only filled the upper triangle of the K matrix
    // copy the upper triangle to the lower triangle
    for(int i = 0; i < 24; i++) for(int j = i+1; j < 24; j++)
      K[j][i] = K[i][j];
  }
  
  
};


SOFA_DECL_CLASS(TrilinearHexahedralCorotationalFEMForceField);

int TrilinearHexahedralCorotationalFEMForceFieldClass = sofa::core::RegisterObject("Trilinear hexahedral corotational force field FEM")
  .add<TrilinearHexahedralCorotationalFEMForceField>();
  

