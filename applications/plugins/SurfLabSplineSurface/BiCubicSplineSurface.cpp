//
// Created by saleh on 3/1/16.
//
#include <sofa/helper/system/config.h>
#include <sofa/core/visual/VisualModel.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/helper/system/gl.h>
#include <Eigen/Sparse>
#include <sofa/core/BaseState.h>
#include <sofa/core/behavior/MechanicalState.h>

#ifndef SOFA_HAVE_GLEW
#pragma error "GLEW is required"
#endif

typedef sofa::defaulttype::ExtVec3fTypes DataTypes;
using sofa::core::visual::VisualParams;
using sofa::core::topology::BaseMeshTopology;
typedef DataTypes::Coord::value_type real;
using sofa::defaulttype::ResizableExtVector;
using sofa::core::objectmodel::Data;
using sofa::helper::WriteOnlyAccessor;
using sofa::defaulttype::Vec;

/**
 * Draw a bi-cubic B-spline surface from the quad structure of the local mesh.
 *
 * The drawing strategy is that, in initVisual we create
 * all the vertex buffers required.
 * In drawVisual, we update the vertex buffers, bind the client state and
 * just drawElements. The easiest way to do it is to assume OpenGL 3.x only
 *
 * We would also have to write a function to evaluate all the points. That is probably
 * going to go inside updateVisual()
 */
struct SOFA_EXPORT_DYNAMIC_LIBRARY BiCubicSplineSurface : public virtual sofa::core::visual::VisualModel, public virtual sofa::core::BaseState {
  SOFA_CLASS2(BiCubicSplineSurface, sofa::core::visual::VisualModel, sofa::core::BaseState);


  typedef sofa::core::topology::Topology::PointID PointID;
  typedef Eigen::Array<PointID, 4, 4> BiCubicPatch;
  typedef ResizableExtVector<DataTypes::Coord> ExtVecCoord;
  typedef DataTypes::Coord Coord;
  BaseMeshTopology* _mesh;
  std::vector<BiCubicPatch> _patches;
  Data<ExtVecCoord> _positions;

  Data<int> _tesselationFactor;
  std::vector<Coord> _evaluatedPoints;
  GLuint _vbuffers[2];
  std::vector<GLuint> _indices;
  std::vector<Coord> _evaluatedNormals;

  typedef Vec<4, float> Color;

  Data<Color> _diffuse, _specular;
  Data<float> _ambient, _shininess;
  Data<bool> _extrapolation;

  BiCubicSplineSurface()
      : _positions(initData(&_positions, "position", "vertex coordinates"))
      , _tesselationFactor(initData(&_tesselationFactor, 8, "tesselationFactor", "tesselation factor of the patches"))
      , _diffuse(initData(&_diffuse,Color(0.3f,0.2f,0.2f,1.0f), "diffuseColor",  "Diffuse Color"))
      , _specular(initData(&_specular,Color(0.3f, 0.2f, 0.1f, 1.0f),"specularColor",  "Specular Color"))
      , _ambient(initData(&_ambient, 0.2f,"ambientIntensity", "Ambient intensity"))
      , _shininess(initData(&_shininess, 10.0f,"shininess", "Shininess of the specular"))
      , _extrapolation(initData(&_extrapolation, true, "extrapolation", "Perform exrapolation on the control points"))
  {
    _positions.setGroup("Vector");
    _tesselationFactor.setRequired(true);
  }
  virtual ~BiCubicSplineSurface(){}

  /// Current size of all stored vectors
  virtual size_t getSize() const {
    return _positions.getValue().size();
  };

  /// Resize all stored vector
  virtual void resize(size_t vsize) {
    WriteOnlyAccessor<Data<ExtVecCoord> > p(_positions);
    p.resize(vsize);
  }

  /// Get a write data pointer
  virtual sofa::core::objectmodel::BaseData* baseWrite(sofa::core::VecId v) {
    if(v.getType() == sofa::core::V_COORD && v.getIndex() == 1){
      return &_positions;
    }
    else
      return NULL;
  }
  // Get a read data pointer
  virtual const sofa::core::objectmodel::BaseData* baseRead(sofa::core::ConstVecId v) const {
    if(v.getType() == sofa::core::V_COORD && v.getIndex() == 1){
      return &_positions;
    }
    else
      return NULL;
  }


  virtual bool insertInNode(sofa::core::objectmodel::BaseNode* node) { return sofa::core::visual::VisualModel::insertInNode(node) && sofa::core::BaseState::insertInNode(node); }
  virtual bool removeInNode(sofa::core::objectmodel::BaseNode* node) { return sofa::core::visual::VisualModel::removeInNode(node) && sofa::core::BaseState::removeInNode(node); }

  virtual void init() {
    _mesh = getContext()->getMeshTopology();
    if (_mesh == NULL || _mesh->getNbQuads() <= 0) {
      serr << "Object must have a quad topology" << sendl;
      return;
    }


    convertQuadsToPatches();
  }

  void convertQuadsToPatches() {
    // Now we have to extract 4x4 patches out of the quad structure
    //
    // 3  + --- + --- + --- +
    //    |  G  |  F  |  E  |
    //    |     |     |     |
    // 2  + --- + --- + --- +
    //    |  H  |  A  |  D  |
    //    |     |     |     |
    // 1  + --- + --- + --- +
    //    |  I  |  B  |  C  |
    //    |     |     |     |
    // 0  + --- + --- + --- +
    //    0     1     2     3
    //
    //  The letters signify the order of exploration. We start with one quad A.
    // Then we explore the twin edges to get to B, C, D and E. Then another twin edge
    // operation on those will give us F, G, H and I.
    // For all of this, we just need a sparse matrix to store all the edges.
    const BaseMeshTopology::SeqQuads& quads = _mesh->getQuads();
    // Find the largest vertex index in the quads array, that gives us
    // the total number of vertices
    int N = _mesh->getNbPoints();
    // Create the incidence matrix of the mesh graph, edges(i, j) represents
    // half-edge that goes from vertex i to vertex j. The value encodes the
    // face number and which edge it is in that face
    Eigen::SparseMatrix<size_t> edges(N, N);
    for(size_t i = 0; i < quads.size(); i++) for(int j = 0; j < 4; j++)
        edges.insert(quads[i][j],quads[i][(j+1)%4]) = (i+1) * 4 + j;
    edges.makeCompressed();


    _patches.resize(quads.size());
    // Convert every quad into a bi-cubic patch
    // the quad itself becomes the central control points,
    // we traverse the neighbouring quads to find the rest of the control points
    // in case a neighbour does not exists, we copy the control point from centeral quad
    // which gives it the appearance of clamped B-splines
    for(size_t i = 0; i < quads.size(); i++) {
      BiCubicPatch &p = _patches[i]; const BaseMeshTopology::Quad& q = quads[i];
      // Set the central control points
      p(1,1) = q[0], p(1,2) = q[1], p(2,2) = q[2], p(2,1) = q[3];

      // Traverse around and find all the neighbouring quads
      // QQ[?][?] are indices into quads B, C, D, E, F, G, H, I in the figure above
      size_t QQ[4][2];
      for(int j = 0; j < 4; j++) {
        size_t tt = edges.coeff(q[(j+1)%4], q[j]);
        QQ[j][0] = tt;
        QQ[j][1] = tt != 0 ? edges.coeff(quads[tt/4-1][tt%4],quads[tt/4-1][(tt%4+3)%4]) : 0;
      }
      // Process bottom edge of quad B
      if(QQ[0][0] != 0)
        p(0, 1) = quads[QQ[0][0]/4-1][(QQ[0][0]%4+2)%4], p(0, 2) = quads[QQ[0][0]/4-1][(QQ[0][0]%4+3)%4];
      else
        p(0, 1) = p(1, 1), p(0, 2) =  p(1, 2);
      // Process right edge of quad D
      if(QQ[1][0] != 0)
        p(1, 3) = quads[QQ[1][0]/4-1][(QQ[1][0]%4+2)%4], p(2, 3) = quads[QQ[1][0]/4-1][(QQ[1][0]%4+3)%4];
      else
        p(1, 3) = p(1, 2), p(2, 3) = p(2, 2);
      // Process top edge of quad F
      if(QQ[2][0] != 0)
        p(3, 2) = quads[QQ[2][0]/4-1][(QQ[2][0]%4+2)%4], p(3, 1) = quads[QQ[2][0]/4-1][(QQ[2][0]%4+3)%4];
      else
        p(3, 1) = p(2, 1), p(3, 2) = p(2, 2);
      // Process left edge of quad H
      if(QQ[3][0] != 0)
        p(2, 0) = quads[QQ[3][0]/4-1][(QQ[3][0]%4+2)%4], p(1, 0) = quads[QQ[3][0]/4-1][(QQ[3][0]%4+3)%4];
      else
        p(2, 0) = p(2, 1), p(1, 0) = p(1, 1);
      // Process the corner vertex of quad C
      if(QQ[0][1] != 0)
        p(0, 3) = quads[QQ[0][1]/4-1][(QQ[0][1]%4+2)%4];
      else
        p(0, 3) = QQ[0][0] != 0 ? p(0, 2) : p(1, 3);
      // Process the corner vertex of quad E
      if(QQ[1][1] != 0)
        p(3, 3) = quads[QQ[1][1]/4-1][(QQ[1][1]%4+2)%4];
      else
        p(3, 3) = QQ[1][0] != 0 ? p(2, 3) : p(3, 2);
      // Process the corner vertex of quad G
      if(QQ[2][1] != 0)
        p(3, 0) = quads[QQ[2][1]/4-1][(QQ[2][1]%4+2)%4];
      else
        p(3, 0) = QQ[2][0] != 0 ? p(3, 1) : p(2, 0);
      // Process the corner vertex of quad I
      if(QQ[3][1] != 0)
        p(0, 0) = quads[QQ[3][1]/4-1][(QQ[3][1]%4+2)%4];
      else
        p(0, 0) = QQ[3][0] != 0 ? p(1, 0) : p(0, 1);
    }

    /*
    for(size_t i = 0; i < _patches.size(); i++)
    {
      std::cerr << i << ":\n";
      for(int r = 0; r < 4; r++) {
        for(int c = 0; c < 4; c++) std::cerr << _patches[i](r,c) << "\t";
        std::cerr << "\n";
      }
      std::cerr << "\n" << std::endl;
    }
     */
  }


  void deBoorsMethodU(const Coord& a, const Coord& b, const Coord& c, const Coord& d, real u, Coord& ex, Coord& ed) {
    Coord b1 = (1-(u+2)/3)*a + (u+2)/3*b;
    Coord c1 = (1-(u+1)/3)*b + (u+1)/3 * c;
    Coord d1 = (1-u/3)*c + u/3 * d;

    Coord c2 = (1-(u+1)/2)*b1 + (u+1)/2 * c1;
    Coord d2 = (1-u/2)*c1 + u/2 * d1;

    ex = (1-u)*c2 + u*d2;
    ed = d2 - c2;
  }
  void deBoorsMethodUV(const BiCubicPatch& p, const Coord* x, real u, real v, Coord& ex, Coord& en) {
    Coord a, b, c, d, ad, bd, cd, dd, edv, edu, edd;
    deBoorsMethodU(x[p(0,0)], x[p(0,1)], x[p(0,2)], x[p(0,3)], u, a, ad);
    deBoorsMethodU(x[p(1,0)], x[p(1,1)], x[p(1,2)], x[p(1,3)], u, b, bd);
    deBoorsMethodU(x[p(2,0)], x[p(2,1)], x[p(2,2)], x[p(2,3)], u, c, cd);
    deBoorsMethodU(x[p(3,0)], x[p(3,1)], x[p(3,2)], x[p(3,3)], u, d, dd);
    deBoorsMethodU(ad,bd,cd,dd, v, edu, edd);
    deBoorsMethodU(a,b,c,d, v, ex, edv);
    en = - edu.cross(edv).normalized();
  }
  void evaluatePatches() {
    if(_positions.getValue().size() != (size_t)_mesh->getNbPoints()) {
      std::cerr << "Positions and topology point mismatch. actual " << _positions.getValue().size() << " <> " << _mesh->getNbPoints() << " expected " << std::endl;
      return;
    }
    int L = _tesselationFactor.getValue();
    const Coord *x = &_positions.getValue()[0];
    _evaluatedPoints.resize(_patches.size() * L * L);
    _evaluatedNormals.resize(_patches.size() * L * L);

    for(size_t i = 0; i < _patches.size(); i++) {
      const BiCubicPatch& p = _patches[i]; const size_t b = i * L * L;
      for(int ui = 0; ui < L; ui++)
        for(int uj = 0; uj < L; uj++) {
          deBoorsMethodUV(p, x, ui/((real)L-1), uj/((real)L-1), _evaluatedPoints[b + ui*L + uj], _evaluatedNormals[b + ui*L + uj]);
        }
    }
  }

  virtual void updateVisual() {
    updatePoints();
    evaluatePatches();
    buildIndices();
    uploadPoints();
  }

  void updatePoints() {
    using sofa::core::behavior::BaseMechanicalState;
    int N = _mesh->getNbPoints();
    std::vector<Coord> X;
    X.resize(N);
    if(_mesh->hasPos()) {
      for (int i = 0; i < N; i++) {
        X[i][0] = _mesh->getPX(i);
        X[i][1] = _mesh->getPY(i);
        X[i][2] = _mesh->getPZ(i);
      }
    } else if(BaseMechanicalState *mstate = getContext()->getMechanicalState()) {
      for(int i = 0; i < N; i++) {
        X[i][0] = mstate->getPX(i);
        X[i][1] = mstate->getPY(i);
        X[i][2] = mstate->getPZ(i);
      }
    }
    WriteOnlyAccessor<Data<ExtVecCoord> > positions(_positions);
    positions.resize(N);
    for(size_t i = 0; i < positions.size(); i++)
        positions[i] = X[i];

    /* now do the extrapolation if enabled */
    if(_extrapolation.getValue()){
        const real stencil[] = { -0.25, 1.5, -0.25 };
        for(size_t i = 0; i < _patches.size(); i++) {
          const BiCubicPatch& p = _patches[i];
          for(int ai = 0; ai < 2; ai++)
              for(int bi = 0; bi < 2; bi++)
              {
                  Coord x(0,0,0);
                  for(int ui = 0; ui < 3; ui++)
                      for(int vi = 0; vi < 3; vi++)
                          x += X[p(ai+vi,bi+ui)] * stencil[vi] * stencil[ui];
                  positions[p(ai+1,bi+1)] = x;
              }
        }
    }
  }

  void buildIndices(){
    const int L = _tesselationFactor.getValue();
    _indices.resize((L-1)*(L-1)*2*3);
    for(int i = 0; i < L-1; i++)
      for(int j = 0; j < L-1; j++)
      {
        const int b = (i*(L-1)+j)*6;
        const int c = i*L+j;
        _indices[b+0] = c;
        _indices[b+1] = c + 1;
        _indices[b+2] = c + L + 1;
        _indices[b+3] = c;
        _indices[b+4] = c + L + 1;
        _indices[b+5] = c + L;
      }
  }


  virtual void initVisual() {
    glGenBuffers(2, _vbuffers);
  }

  void uploadPoints() {
    glBindBuffer(GL_ARRAY_BUFFER, _vbuffers[0]);
    glBufferData(GL_ARRAY_BUFFER, _evaluatedPoints.size() * sizeof(Coord), _evaluatedPoints.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, _vbuffers[1]);
    glBufferData(GL_ARRAY_BUFFER, _evaluatedNormals.size() * sizeof(Coord), _evaluatedNormals.data(), GL_STATIC_DRAW);
  }

  virtual void drawVisual(const VisualParams* vparams) {
    if(vparams->displayFlags().getShowVisualModels())
    {

        Color ambient = _ambient.getValue() * _diffuse.getValue();
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, _diffuse.getValue().data());
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, _specular.getValue().data());
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient.data());
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, _shininess.getValue());

        glEnable(GL_LIGHTING);
        glEnableClientState(GL_VERTEX_ARRAY);
        glBindBuffer(GL_ARRAY_BUFFER, _vbuffers[0]);
        glVertexPointer(3, GL_FLOAT, 0, 0);

        glEnableClientState(GL_NORMAL_ARRAY);
        glBindBuffer(GL_ARRAY_BUFFER, _vbuffers[1]);
        glNormalPointer(GL_FLOAT, 0, 0);

        const int L = _tesselationFactor.getValue();
        for(size_t i = 0; i < _patches.size(); i++)
          glDrawElementsBaseVertex(GL_TRIANGLES, _indices.size(), GL_UNSIGNED_INT, _indices.data(), i * L * L);

        //glDrawArrays(GL_POINTS, 0, _evaluatedPoints.size());
    }
  }
};

SOFA_DECL_CLASS(BiCubicSplineSurface)

int BiCubicSplineSurfaceClass = sofa::core::RegisterObject("Bi-cubic spline surface")
  .add<BiCubicSplineSurface>();
