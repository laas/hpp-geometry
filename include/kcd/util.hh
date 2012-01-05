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


#ifndef KCD_UTIL_HH_
# define KCD_UTIL_HH_

# include <iostream>

# include <geometric-tools/Wm5Vector3.h>

# include <kcd2/kcdInterface.h>

namespace kcd
{
  /// \brief Convert CkcdPoint to Geometric Tools Vector3.
  inline void convertKcdPointToVector3 (Wm5::Vector3<kcdReal>& dst,
					const CkcdPoint& src)
  {
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
  }

  /// \brief Convert Geometric Tools Vector3 to CkcdPoint.
  inline void convertVector3ToKcdPoint (CkcdPoint& dst,
					const Wm5::Vector3<kcdReal>& src)
  {
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
  }

  /// \brief Print kcdMat4 matrix.
  std::ostream& operator<< (std::ostream& os, const CkcdMat4& kcdMat4);

  /// \brief Compute square distance between two segments.
  ///
  /// \param leftEndPoint1 left segment first end point
  /// \param leftEndPoint1 left segment second end point
  /// \param leftEndPoint1 right segment first end point
  /// \param leftEndPoint1 right segment second end point
  /// \return squareDistance square distance between the two segments
  /// \return leftSegmentClosest closest point on left segment
  /// \return rightSegmentClosest closest point on right segment
  void computeSquareDistanceSegmentSegment (const CkcdPoint& leftEndPoint1,
					    const CkcdPoint& leftEndPoint2,
					    const CkcdPoint& rightEndPoint1,
					    const CkcdPoint& rightEndPoint2,
					    kcdReal& squareDistance,
					    CkcdPoint& leftSegmentClosest,
					    CkcdPoint& rightSegmentClosest);

} // end of namespace kcd.
#endif //! KCD_UTIL_HH_
