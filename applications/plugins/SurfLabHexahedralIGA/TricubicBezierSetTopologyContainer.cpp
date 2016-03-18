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

#include "TricubicBezierMeshTopology.h"

#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/Mat.h>

using sofa::helper::vector;
using sofa::defaulttype::Vec;
using sofa::defaulttype::Mat;
using sofa::core::objectmodel::Data;
using sofa::core::topology::BaseMeshTopology;

typedef sofa::core::topology::BaseMeshTopology::Edge Edge;
typedef sofa::core::topology::BaseMeshTopology::Quad Face;
typedef sofa::core::topology::BaseMeshTopology::PointID index_t;


void normalizeEdge(Edge& e) {
  if(e[0] > e[1])
  {
    index_t t = e[1];
    e[1] = e[0];
    e[0] = t;
  }
}

/*!
 * \brief noramlizeFace
 * \param f
 * Normalize face in a way that the first element is the smallest
 * and the second is also smaller of the two.
 *
 * That is: 0 3 1 2 is converted to 0 2 1 3, swapping 2 and 3
 */
void normalizeFace(Face& f){
  if(f[1] < f[0] && f[1] < f[2] && f[1] < f[3])
  {
    index_t f0 = f[0];
    f[0] = f[1], f[1] = f[2], f[2] = f[3], f[3] = f0;
  }
  else if(f[2] < f[0] && f[2] < f[1] && f[2] < f[3])
  {
    std::swap(f[0], f[2]);
  }
  else if(f[3] < f[0] && f[3] < f[1] && f[3] < f[2])
  {
    index_t f0 = f[0];
    f[0] = f[3], f[3] = f[2], f[2] = f[1], f[1] = f0;
  }
  if(f[1] > f[3]) std::swap(f[1], f[3]);
}

bool lessFace(const Face& a, const Face& b) {
  return a[0] != b[0] ? a[0] < b[0] :
      (a[1] != b[1] ? a[1] < b[1] :
      (a[2] != b[2] ? a[2] < b[2] : a[3] < b[3]));
}

bool equalFace(const Face& a, const Face &b){
  return a[0] == b[0] && a[1] == b[1] && a[2] == b[2] && a[3] == b[3];
}

bool lessEdge(const Edge& a, const Edge& b) {
  return a[0] != b[0] ? a[0] < b[0] : a[1] < b[1];
}
bool equalEdge(const Edge& a, const Edge& b) {
  return a[0] == b[0] && a[1] == b[1];
}

//      Our layout with a right handed coordinate system
//        2---------3
//       /|        /|
//      / |       / |
//     6---------7  |
//     |  |      |  |
//     |  0------|--1     Y
//     | /       | /      |
//     |/        |/       |
//     4---------5        o----X
//                       /
//                      Z
//
const int hexa_edges[12][2] = {
  { 0, 1},
  { 0, 2},
  { 0, 4},
  { 1, 3},
  { 1, 5},
  { 2, 3},
  { 2, 6},
  { 3, 7},
  { 4, 5},
  { 4, 6},
  { 5, 7},
  { 6, 7}
};

const int hexa_faces[6][4] = {
  { 0, 1, 3, 2 },
  { 0, 4, 5, 1 },
  { 0, 2, 6, 4 },
  { 7, 6, 2, 3 },
  { 7, 3, 1, 5 },
  { 7, 5, 4, 6 }
};

typedef float real;
typedef Vec<3, real> vec3;

struct HexahedronToTricubicConversion {
    typedef TricubicBezierMeshTopology::SeqHexahedra SeqHexahedra;
    typedef TricubicBezierMeshTopology::SeqEdges SeqEdges;
    typedef TricubicBezierMeshTopology::SeqQuads SeqQuads;
    typedef TricubicBezierMeshTopology::SeqTricubicBezier SeqTricubicBezier;
    typedef TricubicBezierMeshTopology::Hexahedron Hexahedron;
    typedef TricubicBezierMeshTopology::TricubicBezier TricubicBezier;

    // Input data structures
    const vector<vec3> &points;
    const SeqHexahedra& hexa;
    vector<vec3> &_positions;
    SeqTricubicBezier& beziers;

    // Auxiliary data structures
    SeqEdges _edges;
    SeqQuads _faces;

    /// Normalize all edges, sort them and remove duplicates
    /// so that we can use binary search on them now.
    void sortEdges(SeqEdges& edges){
      std::for_each(edges.begin(), edges.end(), normalizeEdge);
      std::sort(edges.begin(), edges.end(), lessEdge);
      SeqEdges::iterator new_end = std::unique(edges.begin(), edges.end(), equalEdge);
      edges.resize(new_end - edges.begin());
    }

    void sortFaces(SeqQuads& faces){
      std::for_each(faces.begin(), faces.end(), normalizeFace);
      std::sort(faces.begin(), faces.end(), lessFace);
      SeqQuads::iterator new_end = std::unique(faces.begin(), faces.end(), equalFace);
      faces.resize(new_end - faces.begin());
    }


    HexahedronToTricubicConversion(const vector<vec3> &points, const SeqHexahedra &hexahedra, vector<vec3>& positions, SeqTricubicBezier &beziers) : points(points),hexa(hexahedra),_positions(positions), beziers(beziers)
    {}

    void convert(){
        beziers.resize(hexa.size());

        // Enumerate all edges and faces
        index_t hexahedraVertexCount = 0;
        for(size_t i = 0; i < hexa.size(); i++) {
          const Hexahedron& h = hexa[i];
          for(int j = 0; j < 12; j++)
            _edges.push_back(Edge(h[hexa_edges[j][0]],h[hexa_edges[j][1]]));
          for(int j = 0; j < 6; j++)
            _faces.push_back(Face(h[hexa_faces[j][0]],h[hexa_faces[j][1]],h[hexa_faces[j][2]],h[hexa_faces[j][3]]));
          for(int j = 0; j < 8; j++)
            hexahedraVertexCount = std::min(hexahedraVertexCount, h[j]);
        }

        //if(hexahedraVertexCount > points.size())
        //  points.resize(hexahedraVertexCount);

        // Remove duplicates and sort them
        sortEdges(_edges);
        sortFaces(_faces);

        // go through all hexa and find indices for faces and edges
        for(size_t i = 0; i < hexa.size(); i++){
          // create the Bezier from the Hexa
          const Hexahedron &h = hexa[i]; TricubicBezier &b = beziers[i];
          b[0] = h[0], b[3] = h[1], b[12] = h[2], b[15] = h[3];
          b[48] = h[4], b[51] = h[5], b[60] = h[6], b[63] = h[7];

          // go through all 12 edges both ways and set the edge points
          for(int t = 0; t < 2; t++)
            for(int s = 0; s < 2; s++)
            {
              int u = t * 3, v = s * 3;
              // along X direction
              b[u*16+v*4+1] = getEdgePointIndex(h[t*4+s*2+0],h[t*4+s*2+1]);
              b[u*16+v*4+2] = getEdgePointIndex(h[t*4+s*2+1],h[t*4+s*2+0]);
              // along Y direction
              b[u*16+1*4+v] = getEdgePointIndex(h[t*4+0*2+s],h[t*4+1*2+s]);
              b[u*16+2*4+v] = getEdgePointIndex(h[t*4+1*2+s],h[t*4+0*2+s]);
              // along Z direction
              b[1*16+u*4+v] = getEdgePointIndex(h[0*4+t*2+s],h[1*4+t*2+s]);
              b[2*16+u*4+v] = getEdgePointIndex(h[1*4+t*2+s],h[0*4+t*2+s]);
            }

          // go through all 6 faces 4 ways and set the face points
          for(int t = 0; t < 2; t++)
          {
            int u = t * 3;
            // XY plane
            b[u*16+1*4+1] = getFacePointIndex(h[t*4+0],h[t*4+1],h[t*4+3],h[t*4+2]);
            b[u*16+1*4+2] = getFacePointIndex(h[t*4+1],h[t*4+3],h[t*4+2],h[t*4+0]);
            b[u*16+2*4+1] = getFacePointIndex(h[t*4+2],h[t*4+0],h[t*4+1],h[t*4+3]);
            b[u*16+2*4+2] = getFacePointIndex(h[t*4+3],h[t*4+2],h[t*4+0],h[t*4+1]);
            // XZ plane
            b[1*16+u*4+1] = getFacePointIndex(h[t*2+0],h[t*2+4],h[t*2+5],h[t*2+1]);
            b[1*16+u*4+2] = getFacePointIndex(h[t*2+1],h[t*2+0],h[t*2+4],h[t*2+5]);
            b[2*16+u*4+1] = getFacePointIndex(h[t*2+4],h[t*2+5],h[t*2+1],h[t*2+0]);
            b[2*16+u*4+2] = getFacePointIndex(h[t*2+5],h[t*2+1],h[t*2+0],h[t*2+4]);
            // YZ plane
            b[1*16+1*4+u] = getFacePointIndex(h[t+0],h[t+2],h[t+6],h[t+4]);
            b[1*16+2*4+u] = getFacePointIndex(h[t+2],h[t+6],h[t+4],h[t+0]);
            b[2*16+1*4+u] = getFacePointIndex(h[t+4],h[t+0],h[t+2],h[t+6]);
            b[2*16+2*4+u] = getFacePointIndex(h[t+6],h[t+4],h[t+0],h[t+2]);
          }

          for(int u = 1; u <= 2; u++)
            for(int v = 1; v <=2; v++)
              for(int w = 1; w <= 2; w++)
                  b[u*16+v*4+w] = getInternalPointIndex(i, u, v, w);
        }

        // Average out the hexahedral points to create initial positions
        // for the cubic Bezires

        _positions.resize(points.size() + _edges.size() * 2 + _faces.size() * 4 + hexa.size() * 8);

        for(size_t i = 0; i < points.size(); i++)
          _positions[i] = points[i];
        for(size_t i = 0; i < _edges.size(); i++)
        {
          const Edge& e = _edges[i];
          _positions[getEdgePointIndex(e[0],e[1])] = 2.0/3.0 * points[e[0]] + points[e[1]] / 3.0;
          _positions[getEdgePointIndex(e[1],e[0])] = 2.0/3.0 * points[e[1]] + points[e[0]] / 3.0;
        }
        for(size_t i = 0; i < _faces.size(); i++)
        {
          const Face& f = _faces[i]; index_t a = f[0], b = f[1], c = f[2], d = f[3];
          _positions[getFacePointIndex(a,b,c,d)] = 4.0/9.0 * points[a] + 2.0/9.0 * (points[b]+points[d]) + 1.0/9.0 * points[c];
          _positions[getFacePointIndex(b,c,d,a)] = 4.0/9.0 * points[b] + 2.0/9.0 * (points[c]+points[a]) + 1.0/9.0 * points[d];
          _positions[getFacePointIndex(c,d,a,b)] = 4.0/9.0 * points[c] + 2.0/9.0 * (points[d]+points[b]) + 1.0/9.0 * points[a];
          _positions[getFacePointIndex(d,a,b,c)] = 4.0/9.0 * points[d] + 2.0/9.0 * (points[a]+points[c]) + 1.0/9.0 * points[b];
        }
        for(size_t i = 0; i < hexa.size(); i++)
        {
          const Hexahedron& h = hexa[i];
          for(int u = 1; u <= 2; u++) for(int v = 1; v <=2; v++) for(int w = 1; w <= 2; w++)
          {
            vec3 x;
            for(int k = 0; k < 2; k++) for(int l = 0; l < 2; l++) for(int m = 0; m < 2; m++)
              x += points[h[k*4+l*2+m]] * (3-u + k*(2*u-3)) * (3-v + l*(2*v-3)) * (3-w + m*(2*w-3) ) / 27.0;
            _positions[getInternalPointIndex(i, u, v, w)] = x;
          }
        }
    }


    /*!
     * \brief getEdgePointIndex
     * \param a
     * \param b
     * \return vertex index of the edge point
     *
     * Since every edge has two points on it, we orient it
     * by the ordering of a and b.
     *
     *
     */
    index_t getEdgePointIndex(index_t a, index_t b){
      Edge e(a, b); normalizeEdge(e);
      int orientation = a < b ? 0 : 1;
      SeqEdges::const_iterator q = std::lower_bound(_edges.begin(), _edges.end(), e, lessEdge);
      assert(q != _edges.end() && *q == e);
      return (q - _edges.begin())*2 + orientation + points.size();
    }

    index_t getFacePointIndex(index_t a, index_t b, index_t c, index_t d) {
      Face f(a, b, c, d); normalizeFace(f);
      int orientation = 0;
      if(a < b && a < c && a < d) orientation = 0; // the smallest is first
      else if(a < c && (a < b || a < d)) orientation = 1; // second to smallest is first
      else if(c < a && c < b && c < d) orientation = 2; // third one is first, it is diagonal from the smallest so the smallest is c
      else orientation = 3; // 4th case
      SeqQuads::const_iterator q = std::lower_bound(_faces.begin(), _faces.end(), f, lessFace);
      assert(q != _faces.end() && *q == f);
      return (q - _faces.begin()) * 4 + orientation + points.size() + _edges.size() * 2;
    }

    /*!
     * \brief Get the DOF number of the internal point of hexahedron i,
     * (u,v,w) is the index vector, because it is internal each of
     * u,v and w are either 1 or 2. Since the whole range is 0..3, the
     * internal point has to be 1..2.
     * \param i
     * \param u
     * \param v
     * \param w
     * \return index of the vertex in the big positions array
     *
     *
     */
    index_t getInternalPointIndex(index_t i, int u, int v, int w) {
      return points.size() + _edges.size()*2 + _faces.size()*4 + i*8 + (u-1)*4 + (v-1)*2 + (w-1);
    }

};

struct TricubicBezierSetTopologyContainer : public TricubicBezierMeshTopology {

  Data<SeqTricubicBezier> _beziers;
  Data<vector<vec3> > _position;
  Data<SeqHexahedra> _hexahedra;
  Data<vector<vec3> > _hexahedralPoints;
  SeqEdges _edges;
  SeqQuads _faces;

  TricubicBezierSetTopologyContainer()
      :_beziers(initData(&_beziers, "beziers", "Bezier elements"))
      ,_position(initData(&_position,"position","Positions of Bezier control points"))
      ,_hexahedra(initData(&_hexahedra,"fromHexahedra","Input hexahedra to build Beziers out of"))
      ,_hexahedralPoints(initData(&_hexahedralPoints,"fromHexahedralPoints", "Input positions of the hexahedral vertices"))

  {}

  virtual ~TricubicBezierSetTopologyContainer()
  {}

  virtual const SeqTricubicBezier& getTricubicBeziers(){
    return _beziers.getValue();
  }


  virtual const SeqEdges& getEdges(){ return _edges; }
  virtual const SeqTriangles& getTriangles(){ static SeqTriangles empty; return empty; }
  virtual const SeqQuads& getQuads(){ return _faces; }
  virtual const SeqTetrahedra& getTetrahedra(){ static SeqTetrahedra empty; return empty; }
  virtual const SeqHexahedra& getHexahedra(){ return _hexahedra.getValue(); }


  virtual bool hasPos()const{
    return true;
  }
  virtual SReal getPX(int i) const{
    return _position.getValue()[i][0];
  }
  virtual SReal getPY(int i) const{
    return _position.getValue()[i][1];
  }
  virtual SReal getPZ(int i) const{
    return _position.getValue()[i][2];
  }
  virtual int getNbPoints() const{
    return static_cast<int>(_position.getValue().size());
  }

  virtual void init(){
    if(_hexahedra.getValue().size() > 0) {
        // buildBeziersFromHexahedra
        HexahedronToTricubicConversion h2b(_hexahedralPoints.getValue(), _hexahedra.getValue(), *_position.beginEdit(), *_beziers.beginEdit());
        h2b.convert();
        _position.endEdit();
        _beziers.endEdit();
    }

    // Create the control mesh
    const SeqTricubicBezier& bzs = _beziers.getValue();
    for(size_t i = 0; i < bzs.size(); i++) {
        const TricubicBezier& bz = bzs[i];
        for(int s = 0; s < 4; s += 3)
            for(int u = 0; u < 3; u++)
                for(int t = 0; t < 3; t++) {
                    // XY faces
                    PointID x0y0 = bz[s*16+u*4+t], x0y1 = bz[s*16+(u+1)*4+t], x1y1 = bz[s*16+(u+1)*4+t+1], x1y0 = bz[s*16+u*4+t+1];
                    _edges.push_back(Edge(x0y0,x1y0)); // along X
                    _edges.push_back(Edge(x0y0,x0y1)); // along Y
                    // Create the edges at the other end
                    if(u == 2) _edges.push_back(Edge(x0y1,x1y1));
                    if(t == 2) _edges.push_back(Edge(x1y0,x1y1));
                    _faces.push_back(s == 0 ? Face(x0y0,x0y1,x1y1,x1y0) : Face(x0y0,x1y0,x1y1,x0y1));

                    // XZ faces
                    PointID x0z0 = bz[u*16+s*4+t], x1z0 = bz[u*16+s*4+t+1], x0z1 = bz[(u+1)*16+s*4+t], x1z1 = bz[(u+1)*16+s*4+t+1];
                    _edges.push_back(Edge(x0z0,x1z0)); // along X
                    _edges.push_back(Edge(x0z0,x0z1)); // along Z
                    if(u == 2) _edges.push_back(Edge(x0z1,x1z1));
                    if(t == 2) _edges.push_back(Edge(x1z0,x1z1));
                    _faces.push_back(s == 0 ? Face(x0z0,x1z0,x1z1,x0z1) : Face(x0z0,x0z1,x1z1,x1z0));

                    // YZ faces
                    PointID y0z0 = bz[t*16+u*4+s], y1z0 = bz[t*16+(u+1)*4+s], y0z1 = bz[(t+1)*16+u*4+s], y1z1 = bz[(t+1)*16+(u+1)*4+s];
                    _edges.push_back(Edge(y0z0,y1z0)); // along Y
                    _edges.push_back(Edge(y0z0,y0z1)); // along Z
                    if(t == 2) _edges.push_back(Edge(y0z1,y1z1));
                    if(u == 2) _edges.push_back(Edge(y1z0,y1z1));
                    _faces.push_back(s == 0 ? Face(y0z0,y0z1,y1z1,y1z0) : Face(y0z0,y1z0,y1z1,y0z1));
                }
    }

  }


};

SOFA_DECL_CLASS(TricubicBezierSetTopologyContainer)

int TricubicBezierSetTopologyContainerClass =
    sofa::core::RegisterObject("Container for tri-cubic Bezier elements")
        .add<TricubicBezierSetTopologyContainer>();
