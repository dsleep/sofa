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

#include "TriCubicBezierMeshTopology.h"

#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/Mat.h>

using sofa::helper::vector;
using sofa::defaulttype::Vec;
using sofa::defaulttype::Mat;
using sofa::core::objectmodel::Data;
using sofa::core::topology::BaseMeshTopology;

struct TriCubicBezierMeshTopologyImpl : public TriCubicBezierMeshTopology {
  /// @TODO: maybe we should add points to this topology as well
  Data<SeqTriCubicBezier> _beziers;
  TriCubicBezierMeshTopologyImpl()
      :_beziers(initData(&_beziers, "beziers", "Bezeier elements"))
  {}

  virtual const SeqTriCubicBezier& getTriCubicBeziers(){
    return _beziers.getValue();
  }

  virtual const SeqEdges& getEdges(){ static SeqEdges empty; return empty; }
  virtual const SeqTriangles& getTriangles(){ static SeqTriangles empty; return empty; }
  virtual const SeqQuads& getQuads(){ static SeqQuads empty; return empty; }
  virtual const SeqTetrahedra& getTetrahedra(){ static SeqTetrahedra empty; return empty; }
  virtual const SeqHexahedra& getHexahedra(){ static SeqHexahedra empty; return empty; }
};

SOFA_DECL_CLASS(TriCubicBezierMeshTopologyImpl);

int TriCubicBezierMeshTopologyImplClass =
    sofa::core::RegisterObject("Tricubic Bezier topology")
        .add<TriCubicBezierMeshTopologyImpl>().addAlias("TricubicBezierMeshTopology");
