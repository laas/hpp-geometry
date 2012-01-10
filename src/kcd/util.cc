// Copyright (C) 2011 by Antonio El Khoury.
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
 * \file src/kcd/util.cc
 */

#include <geometric-tools/Wm5DistSegment3Segment3.h>
#include <geometric-tools/Wm5DistSegment3Box3.h>
#include <geometric-tools/Wm5DistSegment3Triangle3.h>

#include "kcd/util.hh"

namespace kcd
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

  void computeSquareDistanceSegmentSegment (const CkcdPoint& leftEndPoint1,
					    const CkcdPoint& leftEndPoint2,
					    const CkcdPoint& rightEndPoint1,
					    const CkcdPoint& rightEndPoint2,
					    kcdReal& squareDistance,
					    CkcdPoint& leftSegmentClosest,
					    CkcdPoint& rightSegmentClosest)
  {
    using namespace Wm5;

    Vector3<kcdReal> leftP0;
    Vector3<kcdReal> leftP1;
    Vector3<kcdReal> rightP0;
    Vector3<kcdReal> rightP1;

    convertKcdPointToVector3 (leftP0, leftEndPoint1);
    convertKcdPointToVector3 (leftP1, leftEndPoint2);
    convertKcdPointToVector3 (rightP0, rightEndPoint1);
    convertKcdPointToVector3 (rightP1, rightEndPoint2);

    Segment3<kcdReal> s0 (leftP0, leftP1);
    Segment3<kcdReal> s1 (rightP0, rightP1);

    DistSegment3Segment3<kcdReal> distance (s0, s1);
    
    squareDistance = distance.GetSquared ();

    Vector3<kcdReal> wm5LeftSegmentClosest = s0.Center
      + distance.GetSegment0Parameter () * s0.Direction;
    Vector3<kcdReal> wm5RightSegmentClosest = s1.Center
      + distance.GetSegment1Parameter () * s1.Direction;

    convertVector3ToKcdPoint (leftSegmentClosest, wm5LeftSegmentClosest);
    convertVector3ToKcdPoint (rightSegmentClosest, wm5RightSegmentClosest);
  }

  void computeSquareDistanceSegmentBox (const CkcdPoint& leftEndPoint1,
					const CkcdPoint& leftEndPoint2,
					const CkcdTestTreeOBB::CkcdPolyOBBCache&
					rightPolyOBBCache,
					kcdReal& squareDistance)
  {
    using namespace Wm5;

    // Define segment from left capsule axis.
    Vector3<kcdReal> leftP0;
    Vector3<kcdReal> leftP1;
    convertKcdPointToVector3 (leftP0, leftEndPoint1);
    convertKcdPointToVector3 (leftP1, leftEndPoint2);

    Segment3<kcdReal> s0 (leftP0, leftP1);

    // Define box from right OBB.
    CkcdMat4 position = rightPolyOBBCache.m_matrix;

    CkcdPoint rightPolyOBBCacheCenter (position(0, 3),
				       position(1, 3),
				       position(2, 3));
    Vector3<kcdReal> center;
    convertKcdPointToVector3 (center, rightPolyOBBCacheCenter);

    Vector3<kcdReal> axis0 (position(0, 0), position(1, 0), position(2, 0));
    Vector3<kcdReal> axis1 (position(0, 1), position(1, 1), position(2, 1));
    Vector3<kcdReal> axis2 (position(0, 2), position(1, 2), position(2, 2));

    kcdReal extent0 = rightPolyOBBCache.m_halfLength[0];
    kcdReal extent1 = rightPolyOBBCache.m_halfLength[1];
    kcdReal extent2 = rightPolyOBBCache.m_halfLength[2];

    Box3<kcdReal> box (center, axis0, axis1, axis2, extent0, extent1, extent2);

    // Define distance and compute squared distance.
    DistSegment3Box3<kcdReal> distance (s0, box);

    squareDistance = distance.GetSquared ();
  }

  void computeSquareDistanceSegmentTriangle (const CkcdPoint& leftEndPoint1,
					     const CkcdPoint& leftEndPoint2,
					     const CkcdTestTreeOBB::
					     CkcdTriangleCache<CkcdPoint>&
					     rightTriangleCache,
					     kcdReal& squareDistance,
					     CkcdPoint& leftSegmentClosest,
					     CkcdPoint& rightTriangleClosest)
  {
    using namespace Wm5;

    // Define segment.
    Vector3<kcdReal> leftP0;
    Vector3<kcdReal> leftP1;
    convertKcdPointToVector3 (leftP0, leftEndPoint1);
    convertKcdPointToVector3 (leftP1, leftEndPoint2);

    Segment3<kcdReal> s0 (leftP0, leftP1);

    // Define triangle.
    Vector3<kcdReal> rightV0;
    Vector3<kcdReal> rightV1;
    Vector3<kcdReal> rightV2;
    convertKcdPointToVector3 (rightV0, rightTriangleCache.m_vertex[0]);
    convertKcdPointToVector3 (rightV1, rightTriangleCache.m_vertex[1]);
    convertKcdPointToVector3 (rightV2, rightTriangleCache.m_vertex[2]);
    
    Triangle3<kcdReal> triangle (rightV0, rightV1, rightV2);

    // Define distance.
    DistSegment3Triangle3<kcdReal> distance (s0, triangle);

    // Compute distance and closest points.
    squareDistance = distance.GetSquared ();

    Vector3<kcdReal> wm5LeftSegmentClosest = s0.Center
      + distance.GetSegmentParameter () * s0.Direction;
    Vector3<kcdReal> wm5RightSegmentClosest 
      = distance.GetTriangleBary (0) * rightV0
      + distance.GetTriangleBary (1) * rightV1
      + distance.GetTriangleBary (2) * rightV2;

    convertVector3ToKcdPoint (leftSegmentClosest, wm5LeftSegmentClosest);
    convertVector3ToKcdPoint (rightTriangleClosest, wm5RightSegmentClosest);
  }

} // end of namespace kcd.
