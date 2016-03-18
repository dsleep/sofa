/******************************************************************************
*   SurfLab TIPS                                                              *
*   (c) 2016 SurfLab, University of Florida                                   *
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

#include "TricubicBezierMeshTopology.h"

// Instead of template, we just define DataTypes, this can later be
// converted to a template
typedef sofa::defaulttype::Vec3dTypes DataTypes;
using sofa::helper::vector;
using sofa::defaulttype::Vec;
using sofa::defaulttype::Mat;
using sofa::core::objectmodel::Data;


struct SOFA_EXPORT_DYNAMIC_LIBRARY TricubicBezierForceField : public virtual sofa::core::behavior::ForceField<DataTypes> {
  SOFA_CLASS(TricubicBezierForceField, SOFA_TEMPLATE(sofa::core::behavior::ForceField, DataTypes));
  
  typedef DataTypes::VecCoord VecCoord;
  typedef DataTypes::Coord Coord;
  typedef DataTypes::Coord::value_type real;
  typedef Mat<3, 3, real> Transform;
  typedef Vec<64,Coord> TricubicCP;

  // We only store the lower triangle part of the stiffness matrix
  // as 8x8 matrix of transformation submatrices
  typedef Vec<64*(64+1)/2, Transform> StiffnessMatrix;

  
  TricubicBezierMeshTopology* _mesh;
  Vec<3,real> _materialStiffness;
  vector<TricubicCP> _rotatedRestElements;
  vector<StiffnessMatrix> _elemStiffness;
  vector<Transform> _elemRotations;

  // The only XML parameters in this module
  Data<real> _poissonRatio, _youngModulus;
  bool _validState;
  
  TricubicBezierForceField()
      : _poissonRatio(initData(&_poissonRatio,(real)0.45,"poissonRatio",""))
      , _youngModulus(initData(&_youngModulus,(real)5000,"youngModulus",""))
      , _validState(true)
  {
    _poissonRatio.setRequired(true);
    _youngModulus.setRequired(true);
  }
  
  virtual ~TricubicBezierForceField() {}
  
  /*!
   * \brief Initialize the force field by pre-computing the rest pose
   *
   * Real initialization is done in reinit. This method only retrieves
   * the topology and checks that the topology is indeed Tricubic Bezier
   * topology
   *
   */
  virtual void init(){
    if(!_validState) return;
    sofa::core::behavior::ForceField<DataTypes>::init();
    _mesh = dynamic_cast<TricubicBezierMeshTopology*>(getContext()->getMeshTopology());
    if(_mesh == NULL) {
      serr << "Object must have a cubic Bezier topology" << sendl;
      _validState = false;
      return;
    }
    
    reinit();
  }

  /*!
   * \brief The actual initialization, re-calculating the rest-pose
   *
   * The elasticity parameters are converted to (U,V,W) vector that is
   * actually used in stiffness matrix calculations.
   *
   * The rest-pose is analyzed here and the initial stiffness matrix and
   * initial rotations are calculated here.
   */
  virtual void reinit() {
    if(!_validState) return;

    {
      const real E = _youngModulus.getValue(), v = _poissonRatio.getValue();
      const real U = E * (1 - v) / ((1 + v) * (1 - 2 * v));
      const real V = E * v / ((1 + v) * (1 - 2 * v));
      const real W = E / (2 * (1 + v));
      _materialStiffness = Vec<3, real>(U, V, W);
    }
    
    const VecCoord &restPose = mstate->read(sofa::core::ConstVecCoordId::restPosition())->getValue();
    const TricubicBezierMeshTopology::SeqTricubicBezier &elems = _mesh->getTricubicBeziers();

    _rotatedRestElements.resize(elems.size());
    _elemRotations.resize(elems.size());
    _elemStiffness.resize(elems.size());

    for(size_t i = 0; i < elems.size(); i++){
      TricubicCP v;
      for(int j = 0; j < 64; j++) v[j] = restPose[elems[i][j]];


      computeRotationPolar(_elemRotations[i], v);

      for(int j = 0; j < 64; j++) v[j] = _elemRotations[i] * v[j];
      _rotatedRestElements[i] = v;

      computeElementStiffness(_elemStiffness[i], v);
    }
  }
  
  virtual void addForce(const sofa::core::MechanicalParams*, Data<VecDeriv>& f, const Data<VecCoord>& x, const Data<VecDeriv>&) {
    if(!_validState) return;
    sofa::helper::WriteAccessor<Data<VecDeriv> > fw = f;
    sofa::helper::ReadAccessor<Data<VecCoord> > xr = x;

    const TricubicBezierMeshTopology::SeqTricubicBezier& elems = _mesh->getTricubicBeziers();
    for(size_t i = 0; i < elems.size(); i++) {
      TricubicCP v;
      for(int j = 0; j < 64; j++) v[j] = xr[elems[i][j]];

      computeRotationPolar(_elemRotations[i], v);
      for(int j = 0; j < 64; j++) v[j] = _elemRotations[i] * v[j];

      computeElementStiffness(_elemStiffness[i], v);

      Coord D[64]; Deriv F[64];
      for(int k = 0; k < 64; k++)
        D[k] = _rotatedRestElements[i][k] - v[k];

      applyStiffnessMatrix(_elemStiffness[i], D, F);

      for(int j = 0; j < 64; j++)
        fw[elems[i][j]] += _elemRotations[i].multTranspose(F[j]);
    }
  }
  
  virtual void addDForce(const sofa::core::MechanicalParams* mparams, Data<VecDeriv>& df, const Data<VecDeriv>& dx) {
    if(!_validState) return;
    sofa::helper::WriteAccessor<Data<VecDeriv> > dfw = df;
    sofa::helper::ReadAccessor<Data<VecCoord> > dxr = dx;
    dfw.resize(dxr.size());
    
    real kFactor = mparams->kFactorIncludingRayleighDamping(rayleighStiffness.getValue());
    const TricubicBezierMeshTopology::SeqTricubicBezier& elems = _mesh->getTricubicBeziers();
    for(size_t i = 0; i < elems.size(); i++) {
      Coord D[64]; Deriv F[64];
      for(int k = 0; k < 64; k++)
        D[k] = - _elemRotations[i] * dxr[elems[i][k]];

      applyStiffnessMatrix(_elemStiffness[i], D, F);

      for(int j = 0; j < 64; j++)
        dfw[elems[i][j]] += _elemRotations[i].multTranspose(F[j]) * kFactor;
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
  
  static void computeRotationPolar( Transform &R, const TricubicCP& v) {
    Transform A; A.fill(0);
    for(int i = 0; i < 4; i++)
      for(int j = 0; j < 4; j++)
        for(int k = 0; k < 3; k++)
        {
          A[0] += v[i*16+j*4+k+1] - v[i*16+j*4+k];
          A[1] += v[i*16+(k+1)*4+j] - v[i*16+k*4+j];
          A[2] += v[(k+1)*16+i*4+j] - v[k*16+i*4+j];
        }
    A = A * (1.0/48.0);
    sofa::helper::Decompose<real>::polarDecomposition(A, R);
  }

  static Coord cubicDeCasteljau(Coord a, Coord b, Coord c, Coord d, real u) {
      Coord a1 = a * (1-u) + b * u;
      Coord b1 = b * (1-u) + c * u;
      Coord c1 = c * (1-u) + d * u;

      Coord a2 = a1 * (1-u) + b1 * u;
      Coord b2 = b1 * (1-u) + c1 * u;

      return a2 * (1-u) + b2 * u;
  }

  static Coord cubicDeCasteljauDerivative(Coord a, Coord b, Coord c, Coord d, real u) {
      Coord a1 = a * (1-u) + b * u;
      Coord b1 = b * (1-u) + c * u;
      Coord c1 = c * (1-u) + d * u;

      Coord a2 = a1 * (1-u) + b1 * u;
      Coord b2 = b1 * (1-u) + c1 * u;

      return 3.0*(b2 - a2);
  }

  /*!
   * \brief jacobianOfCubicBezier
   * \param cp Control points
   * \param t  the parameters
   * \return the jacobian matrix, the derivative in all 3 parameters arranged in a matrix
   *
   * The way that the control points are organized is i*16+j*4+k for i,j,k in [0,3].
   * i corresponds to t[2], j corresponds to t[1] and k corresponds to t[0].
   *
   * In an imprecise way we could say the innermost direction is X and the outermost
   * direction is Z, if the Bezier cube was mapped to unit cube at the origin.
   */
  static Transform jacobianOfCubicBezier(const TricubicCP& cp, Coord t) {
    Transform J;
    Coord dd[3][4];
    for(int i = 0; i < 4; i++)
    {
      Coord d[3][4];

      for(int j = 0; j < 4; j++)
      {
        d[0][j] = cubicDeCasteljauDerivative(cp[i*16+j*4+0],cp[i*16+j*4+1],cp[i*16+j*4+2],cp[i*16+j*4+3],t[0]);
        d[1][j] = cubicDeCasteljauDerivative(cp[i*16+0*4+j],cp[i*16+1*4+j],cp[i*16+2*4+j],cp[i*16+3*4+j],t[1]);
        d[2][j] = cubicDeCasteljauDerivative(cp[0*16+i*4+j],cp[1*16+i*4+j],cp[2*16+i*4+j],cp[3*16+i*4+j],t[2]);
      }

      dd[0][i] = cubicDeCasteljau(d[0][0],d[0][1],d[0][2],d[0][3], t[1]);
      dd[1][i] = cubicDeCasteljau(d[1][0],d[1][1],d[1][2],d[1][3], t[0]);
      dd[2][i] = cubicDeCasteljau(d[2][0],d[2][1],d[2][2],d[2][3], t[0]);
    }
    J[0] = cubicDeCasteljau(dd[0][0],dd[0][1],dd[0][2],dd[0][3], t[2]);
    J[1] = cubicDeCasteljau(dd[1][0],dd[1][1],dd[1][2],dd[1][3], t[2]);
    J[2] = cubicDeCasteljau(dd[2][0],dd[2][1],dd[2][2],dd[2][3], t[1]);
    J.transpose();
    return J;
  }

  static real compow(const real& t, const int& i)
  {
    switch(i) {
      case 0: return (1-t)*(1-t)*(1-t);
      case 1: return 3*(1-t)*(1-t)*t;
      case 2: return 3*(1-t)*t*t;
      case 3: return t*t*t;
      default: assert("index out of range"!=0); return NAN;
      }
  }

  static real dcompow(const real& t, const int& i)
  {
    switch(i) {
      case 0: return -3*(1-t)*(1-t);
      case 1: return 3*(1-t)*(1-3*t);
      case 2: return 3*(2-3*t)*t;
      case 3: return 3*t*t;
      default: assert("index out of range"!=0); return NAN;
      }
  }

  void computeElementStiffness(StiffnessMatrix& K, const TricubicCP& v) {
    for (size_t i = 0; i < K.size(); i++)
      K[i].fill(0);

    const real U = _materialStiffness[0], V = _materialStiffness[1], W = _materialStiffness[2];

    Transform J, J_1, J_1t;
    real detJ;
    /* two point quadrature
    const real isq3 = (real) (1.0 / sqrt(3.0));
    const real quadraturePoints[2] = { (1 - isq3) / 2, (1 + isq3) / 2 };
    const real quadratureWeights[2] = { 0.5, 0.5 };
    const int quadraturePointCount = 2;
    //*/
    /* five point quadrature */
    const real w0 = 128.0/255, w1 = (322+13*sqrt(70))/900, w2 = (322-13*sqrt(70))/900;
    const real q1 = sqrt(5.0 - 2.0 * sqrt(0.7))/3.0, q2 = sqrt(5.0 + 2.0 * sqrt(0.7))/3.0;
    const real quadraturePoints[5] = { (1 - q2)/2, (1 - q1) / 2, 0.5, (1 + q1) / 2, (1 + q2) /2 };
    const real quadratureWeights[5] = { w2, w1, w0, w1, w2 };
    const int quadraturePointCount = 5;
    //*/
    for (int i1 = 0; i1 < quadraturePointCount; i1++)
      for (int i2 = 0; i2 < quadraturePointCount; i2++)
        for (int i3 = 0; i3 < quadraturePointCount; i3++) {
          const Coord t(quadraturePoints[i1],quadraturePoints[i2],quadraturePoints[i3]);
          const real w = quadratureWeights[i1] * quadratureWeights[i2] * quadratureWeights[i3];

          /// The jacobian of the transformation is calculated by taking the first derivative
          /// of the spline function in parameter directions

          J = jacobianOfCubicBezier(v, t);

          /// After J is calculated, we can get the extra matrices from it
          J_1.invert(J);
          J_1t.transpose(J_1);
          detJ = sofa::defaulttype::determinant(J);


          // At this stage we calculate q_i = dN_i/dx: derivative of
          // the basis function for control point i with respect to
          // physical domain.
          // This is done by calculating dN_i/du first and then multiplying
          // it by J_1
          Coord q[64];
          for(int i = 0; i < 4; i++)
            for(int j = 0; j < 4; j++)
              for(int k = 0; k < 4; k++)
                {
                  Coord dNi_du(
                      dcompow(t[0], k) *  compow(t[1], j) *  compow(t[2], i),
                       compow(t[0], k) * dcompow(t[1], j) *  compow(t[2], i),
                       compow(t[0], k) *  compow(t[1], j) * dcompow(t[2], i)
                      );
                  q[i*16+j*4+k] = J_1t * dNi_du;
                }


          // M = [ U V V 0 0 0 ]
          //     [ V U V 0 0 0 ]
          //     [ V V U 0 0 0 ]
          //     [ 0 0 0 W 0 0 ]
          //     [ 0 0 0 0 W 0 ]
          //     [ 0 0 0 0 0 W ]

          // Now use the corner points to calculate stiffness matrices
          for (int i = 0; i < 64; i++)
            for (int j = 0; j <= i; j++) {
              Transform k; // k = Bjt M Bi
              k[0][0] = q[i][0] * U * q[j][0] + q[i][1] * W * q[j][1] + q[i][2] * W * q[j][2];
              k[0][1] = q[i][0] * V * q[j][1] + q[i][1] * W * q[j][0];
              k[0][2] = q[i][0] * V * q[j][2] + q[i][2] * W * q[j][0];

              k[1][0] = q[i][1] * V * q[j][0] + q[i][0] * W * q[j][1];
              k[1][1] = q[i][1] * U * q[j][1] + q[i][0] * W * q[j][0] + q[i][2] * W * q[j][2];
              k[1][2] = q[i][1] * V * q[j][2] + q[i][2] * W * q[j][1];

              k[2][0] = q[i][2] * V * q[j][0] + q[i][0] * W * q[j][2];
              k[2][1] = q[i][2] * V * q[j][1] + q[i][1] * W * q[j][2];
              k[2][2] = q[i][2] * U * q[j][2] + q[i][1] * W * q[j][1] + q[i][0] * W * q[j][0];

              k = k * (detJ * w);


              stiffnessSubmatrixLookup(K, i, j) += k;

            }
        }


  }

  static Transform& stiffnessSubmatrixLookup(StiffnessMatrix& m, int i, int j) {
    assert(i < 64 && j < 64 && j <= i);
    return m[i * (i+1) /2 + j];
  }

  /// Apply sub-stiffness matrices to displacements to
  /// calculate forces
  /// Since the stiffness matrix only contains the lower triangle
  /// we have to do the multiplcation symmetrically as we go on
  /// as in we multiply K[l][m] and K[m][l] since they are the same.
  static void applyStiffnessMatrix(StiffnessMatrix& K, const Coord d[64], Deriv f[64]) {
    for(int l = 0; l < 64; l++) {
      f[l].fill(0);
      for (int m = 0; m <= l; m++) {
        const Transform &k = stiffnessSubmatrixLookup(K, l, m);
        f[l] += k * d[m];
        if(l != m) f[m] += k.multTranspose(d[l]);
      }
    }
  }


};

SOFA_DECL_CLASS(TricubicBezierForceField)

int TricubicBezierForceFieldClass =
    sofa::core::RegisterObject("Tricubic hexahedral corotational force field FEM")
        .add<TricubicBezierForceField>();


