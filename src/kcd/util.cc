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

} // end of namespace kcd.
