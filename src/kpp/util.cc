// Copyright (C) 2012 by Antonio El Khoury.
//
// This file is part of the kcd-capsule.
//
// kcd-capsule is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// kcd-capsule is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with kcd-capsule.  If not, see <http://www.gnu.org/licenses/>.


/**
 * \file src/kpp/util.cc
 *
 * \brief KPP utility functions.
 */

#include "kpp/util.hh";

namespace kpp
{
  void convertCapsuleToPolyhedronData (const double& height,
				       const double& radius,
				       const unsigned baseVertices,
				       const unsigned parallels,
				       CkcdPolyhedronDataShPtr& polyData)
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

    polyData->reserveNPoints (vertices.size () + baseVertices);

    // Apply transformation on sphere lower half and add points
    // to form first cap.
    CkcdMat4 hsFirstTransform;
    hsFirstTransform.rotateY (M_PI / 2);
    hsFirstTransform(0, 3) = - height / 2;

    unsigned rank;
    for (unsigned i = 0; i < hsPointsRange; ++i)
      polyData->addPoint (hsFirstTransform * vertices[i], rank);

    // Apply opposite transformation on sphere lower half and
    // add points to form second cap.
    CkcdMat4 hsSecondTransform;
    hsSecondTransform.rotateY (- M_PI / 2);
    hsSecondTransform(0, 3) = height / 2;

    for (unsigned i = 0; i < hsPointsRange; ++i)
      polyData->addPoint (hsSecondTransform * vertices[i], rank);

    // Apply transformations on remanining single top and bottom
    // points of sphere.
    unsigned hsFirstPtRank;
    polyData->addPoint (hsFirstTransform * vertices[hsLowerPtId],
			hsFirstPtRank);

    unsigned hsSecondPtRank;
    polyData->addPoint (hsSecondTransform * vertices[hsLowerPtId],
			hsSecondPtRank);

    polyData->reserveNTriangles (facets.size () + 2 * baseVertices);

    // Add facets to form first cap.
    for (unsigned i = 0; i < facets.size (); ++i)
      if (facets[i][0] < hsPointsRange
	  && facets[i][1] < hsPointsRange
	  && facets[i][2] < hsPointsRange)
	polyData->addTriangle (facets[i][0], facets[i][1], facets[i][2], rank);

    for (unsigned i = 0; i < baseVertices - 1; ++i)
      polyData->addTriangle (i, i + 1, hsFirstPtRank, rank);
    polyData->addTriangle (0, baseVertices - 1, hsFirstPtRank, rank);

    // Add facets to form cylinder and stitch the two caps together.
    const unsigned firstEquatorStartId
      = hsPointsRange - baseVertices;
    const unsigned secondEquatorFinishId
      = 2 * hsPointsRange - 1;

    for (unsigned i = 0; i < baseVertices / 2; ++i)
      {
	polyData->addTriangle (firstEquatorStartId + i,
			       firstEquatorStartId + i + 1,
			       secondEquatorFinishId - i + 1 - baseVertices / 2,
			       rank);
	polyData->addTriangle (firstEquatorStartId + i + 1,
			       secondEquatorFinishId - i + 1 - baseVertices / 2,
			       secondEquatorFinishId - i - baseVertices / 2,
			       rank);
      }

    for (unsigned i = baseVertices / 2; i < baseVertices - 1; ++i)
      {
	polyData->addTriangle (firstEquatorStartId + i,
			       firstEquatorStartId + i + 1,
			       secondEquatorFinishId - i + baseVertices / 2,
			       rank);
	polyData->addTriangle (firstEquatorStartId + i + 1,
			       secondEquatorFinishId - i + baseVertices / 2,
			       secondEquatorFinishId - i - 1 + baseVertices / 2,
			       rank);
      }

    polyData->addTriangle (firstEquatorStartId + baseVertices - 1,
			   firstEquatorStartId,
			   secondEquatorFinishId - baseVertices + 1
			   + baseVertices / 2,
			   rank);
    polyData->addTriangle (firstEquatorStartId + baseVertices / 2,
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
	polyData->addTriangle (facets[i][0], facets[i][1], facets[i][2], rank);

    const unsigned secondEquatorStartId
      = 2 * hsPointsRange - baseVertices;

    for (unsigned i = 2 * (hsPointsRange - baseVertices);
	 i < secondEquatorStartId - 1;
	 ++i)
      {
	polyData->addTriangle (i, i + 1, i + baseVertices + 1, rank);
	polyData->addTriangle (i, i + baseVertices, i + baseVertices + 1, rank);
      }
    polyData->addTriangle (secondEquatorStartId - 1,
			   2 * (hsPointsRange - baseVertices),
			   secondEquatorStartId,
			   rank);
    polyData->addTriangle (secondEquatorStartId - 1,
			   secondEquatorStartId - 1 + baseVertices,
			   secondEquatorStartId,
			   rank);

    for (unsigned i = hsPointsRange;
	 i < hsPointsRange + baseVertices - 1;
	 ++i)
      polyData->addTriangle (i, i + 1, hsSecondPtRank, rank);
    polyData->addTriangle (hsPointsRange,
			   hsPointsRange + baseVertices - 1,
			   hsSecondPtRank,
			   rank);
  }
} // end of namespace kpp.