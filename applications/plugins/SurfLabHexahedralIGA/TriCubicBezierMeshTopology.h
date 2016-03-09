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

#ifndef SURFLABHEXAHEDRALIGA_TRICUBICBEZIERMESHTOPOLOGY_H
#define SURFLABHEXAHEDRALIGA_TRICUBICBEZIERMESHTOPOLOGY_H

#include <sofa/core/topology/BaseMeshTopology.h>


struct TriCubicBezierMeshTopology : public sofa::core::topology::BaseMeshTopology {
  SOFA_CLASS(TriCubicBezierMeshTopology, sofa::core::topology::BaseMeshTopology);
  typedef sofa::helper::fixed_array<PointID, 16> TriCubicBezier;
  typedef sofa::helper::vector<TriCubicBezier> SeqTriCubicBezier;
  virtual const SeqTriCubicBezier& getTriCubicBeziers() = 0;
  virtual ~TriCubicBezierMeshTopology();
};

#endif //SURFLABHEXAHEDRALIGA_TRICUBICBEZIERMESHTOPOLOGY_H
