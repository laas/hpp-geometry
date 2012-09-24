// Copyright (C) 2011, 2012 by Antonio El Khoury.
//
// This file is part of the hpp-geometry.
//
// hpp-geometry is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// hpp-geometry is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with hpp-geometry.  If not, see <http://www.gnu.org/licenses/>.


/**
 * \file src/hpp/geometry/collision/util.cc
 */

#include <geometric-tools/Wm5ContCapsule3.h>

#include "hpp/geometry/collision/util.hh"

namespace hpp
{
  namespace geometry
  {
    namespace collision
    {
      std::ostream&
      operator<< (std::ostream& os, const CkcdMat4& kcdMat4)
      {
	for (unsigned int rowId = 0; rowId < 4; ++rowId)
	  {
	    for (unsigned int colId = 0; colId < 4; ++colId)
	      os << kcdMat4 (rowId, colId) << " ";
	    os << std::endl;
	  }

	return os;
      }

      std::ostream&
      operator<< (std::ostream& os, const CkcdPoint& kcdPoint)
      {
	os << "["
	   << kcdPoint[0] << ", "
	   << kcdPoint[1] << ", "
	   << kcdPoint[2]
	   << "]";

	return os;
      }

      void computeBoundingCapsulePolyhedron (const polyhedrons_t& polyhedrons,
					     CkcdPoint& endPoint1,
					     CkcdPoint& endPoint2,
					     kcdReal& radius)
      {
	assert (polyhedron.size () !=0 && "Empty polyhedron vector.");

	using namespace Wm5;

	// Retrieve vector of points from polyhedrons.
	unsigned nbPoints = 0;
	for (unsigned i = 0; i < polyhedrons.size (); ++i)
	  nbPoints += polyhedrons[i]->countPoints ();

	point_t points[nbPoints];
	unsigned k = 0;
	for (unsigned i = 0; i < polyhedrons.size (); ++i)
	  {
	    polyhedron_t polyhedron = polyhedrons[i];
	    CkcdMat4 transform;
	    polyhedron->getAbsolutePosition (transform);
	    for (unsigned j = 0; j < polyhedron->countPoints (); ++j)
	      {
		CkcdPoint kcdPoint;
		point_t point;
		polyhedron->getPoint (j, kcdPoint);
		kcdPoint = transform * kcdPoint;
		convertKcdPointToVector3 (point, kcdPoint);
		points[k] = point;
		++k;
	      }
	  }

	// Compute bounding capsule of points.
	Capsule3<kcdReal> wm5Capsule = ContCapsule (nbPoints,
						    points);

	// Get capsule parameters.
	Vector3<kcdReal> wm5EndPoint1 = wm5Capsule.Segment.P0;
	Vector3<kcdReal> wm5EndPoint2 = wm5Capsule.Segment.P1;

	convertVector3ToKcdPoint (endPoint1, wm5EndPoint1);
	convertVector3ToKcdPoint (endPoint2, wm5EndPoint2);
	radius = wm5Capsule.Radius;
      }

    } // end of namespace collision.
  } // end of namespace geometry.
} // end of namespace hpp.
