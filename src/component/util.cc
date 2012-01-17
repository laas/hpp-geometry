// Copyright (C) 2012 by Antonio El Khoury.
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
 * \file src/hpp/geometry/component/util.cc
 *
 * \brief KPP utility functions.
 */

#include "hpp/geometry/collision/util.hh"
#include "hpp/geometry/component/util.hh";

namespace kpp
{
  namespace component
  {
    void convertCapsuleToPolyhedronData (CkcdPolyhedronDataShPtr& dst,
					 const double& height,
					 const double& radius,
					 const unsigned baseVertices,
					 const unsigned parallels)
    {
      // Retrieve points and facets vectors corresponding to a whole sphere.
      std::vector<CkcdPoint> vertices;
      std::vector<std::vector<unsigned int> > facets;

      CkcdPolyhedronFactory::convertSphereToIndexedFaceSet (radius,
							    baseVertices,
							    parallels,
							    vertices,
							    facets);

      unsigned hsPointsRange = (vertices.size () + baseVertices) / 2 - 1;
      unsigned hsLowerPtId = vertices.size () - 2;

      dst->reserveNPoints (vertices.size () + baseVertices);

      // Apply transformation on sphere lower half and add points
      // to form first cap.
      CkcdMat4 hsFirstTransform;
      hsFirstTransform.rotateY (M_PI / 2);
      hsFirstTransform(0, 3) = - height / 2;

      unsigned rank;
      for (unsigned i = 0; i < hsPointsRange; ++i)
	dst->addPoint (hsFirstTransform * vertices[i], rank);

      // Apply opposite transformation on sphere lower half and
      // add points to form second cap.
      CkcdMat4 hsSecondTransform;
      hsSecondTransform.rotateY (- M_PI / 2);
      hsSecondTransform(0, 3) = height / 2;

      for (unsigned i = 0; i < hsPointsRange; ++i)
	dst->addPoint (hsSecondTransform * vertices[i], rank);

      // Apply transformations on remanining single top and bottom
      // points of sphere.
      unsigned hsFirstPtRank;
      dst->addPoint (hsFirstTransform * vertices[hsLowerPtId],
			  hsFirstPtRank);

      unsigned hsSecondPtRank;
      dst->addPoint (hsSecondTransform * vertices[hsLowerPtId],
			  hsSecondPtRank);

      dst->reserveNTriangles (facets.size () + 2 * baseVertices);

      // Add facets to form first cap.
      for (unsigned i = 0; i < facets.size (); ++i)
	if (facets[i][0] < hsPointsRange
	    && facets[i][1] < hsPointsRange
	    && facets[i][2] < hsPointsRange)
	  dst->addTriangle (facets[i][0], facets[i][1], facets[i][2], rank);

      for (unsigned i = 0; i < baseVertices - 1; ++i)
	dst->addTriangle (i, i + 1, hsFirstPtRank, rank);
      dst->addTriangle (0, baseVertices - 1, hsFirstPtRank, rank);

      // Add facets to form cylinder and stitch the two caps together.
      const unsigned firstEquatorStartId
	= hsPointsRange - baseVertices;
      const unsigned secondEquatorFinishId
	= 2 * hsPointsRange - 1;

      for (unsigned i = 0; i < baseVertices / 2; ++i)
	{
	  dst->addTriangle (firstEquatorStartId + i,
				 firstEquatorStartId + i + 1,
				 secondEquatorFinishId - i + 1 - baseVertices / 2,
				 rank);
	  dst->addTriangle (firstEquatorStartId + i + 1,
				 secondEquatorFinishId - i + 1 - baseVertices / 2,
				 secondEquatorFinishId - i - baseVertices / 2,
				 rank);
	}

      for (unsigned i = baseVertices / 2; i < baseVertices - 1; ++i)
	{
	  dst->addTriangle (firstEquatorStartId + i,
				 firstEquatorStartId + i + 1,
				 secondEquatorFinishId - i + baseVertices / 2,
				 rank);
	  dst->addTriangle (firstEquatorStartId + i + 1,
				 secondEquatorFinishId - i + baseVertices / 2,
				 secondEquatorFinishId - i - 1 + baseVertices / 2,
				 rank);
	}

      dst->addTriangle (firstEquatorStartId + baseVertices - 1,
			     firstEquatorStartId,
			     secondEquatorFinishId - baseVertices + 1
			     + baseVertices / 2,
			     rank);
      dst->addTriangle (firstEquatorStartId + baseVertices / 2,
			     secondEquatorFinishId - baseVertices + 1,
			     secondEquatorFinishId,
			     rank);

      // Add facets to form second cap.
      for (unsigned i = 0; i < facets.size (); ++i)
	if ((facets[i][0] >= hsPointsRange
	     && facets[i][0] < 2 * hsPointsRange - baseVertices)
	    && (facets[i][1] >= hsPointsRange
		&& facets[i][1] < 2 * hsPointsRange - baseVertices)
	    && (facets[i][2] >= hsPointsRange
		&& facets[i][2] < 2 * hsPointsRange - baseVertices))
	  dst->addTriangle (facets[i][0], facets[i][1], facets[i][2], rank);

      const unsigned secondEquatorStartId
	= 2 * hsPointsRange - baseVertices;

      for (unsigned i = 2 * (hsPointsRange - baseVertices);
	   i < secondEquatorStartId - 1;
	   ++i)
	{
	  dst->addTriangle (i, i + 1, i + baseVertices + 1, rank);
	  dst->addTriangle (i, i + baseVertices, i + baseVertices + 1, rank);
	}
      dst->addTriangle (secondEquatorStartId - 1,
			     2 * (hsPointsRange - baseVertices),
			     secondEquatorStartId,
			     rank);
      dst->addTriangle (secondEquatorStartId - 1,
			     secondEquatorStartId - 1 + baseVertices,
			     secondEquatorStartId,
			     rank);

      for (unsigned i = hsPointsRange;
	   i < hsPointsRange + baseVertices - 1;
	   ++i)
	dst->addTriangle (i, i + 1, hsSecondPtRank, rank);
      dst->addTriangle (hsPointsRange,
			     hsPointsRange + baseVertices - 1,
			     hsSecondPtRank,
			     rank);
    }

    void convertCapsuleAxisToTransform (CkcdMat4& dst,
					const CkcdPoint& endPoint1,
					const CkcdPoint& endPoint2)
    {
      using namespace kcd;
      dst = CkcdMat4 ();

      // Compute rotation part.
      CkcdPoint axis = endPoint2 - endPoint1;
    
      kcdReal alpha = atan2 (axis[1], axis[0]);
      kcdReal beta = atan2 (axis[2],
			    sqrt (axis[0] * axis[0] +  axis[1] * axis[1]));

      CkcdMat4 rotAlpha;
      rotAlpha.rotateZ (alpha);
      CkcdMat4 rotBeta;
      rotBeta.rotateY (- beta);

      dst = rotAlpha * rotBeta;

      // Compute translation component.
      CkcdPoint center = (endPoint1 + endPoint2) / 2;

      dst(0, 3) = center[0];
      dst(1, 3) = center[1];
      dst(2, 3) = center[2];
    }

  } // end of namespace component.
} // end of namespace kpp.
