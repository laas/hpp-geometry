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

  /// \brief Print kcdPoint vector.
  std::ostream&  operator<< (std::ostream& os, const CkcdPoint& kcdPoint);

  /// \brief Compute square distance between a segment and a point.
  ///
  /// \param leftEndPoint1 left segment first end point
  /// \param leftEndPoint2 left segment second end point
  /// \param rightdPoint right point
  /// \return squareDistance square distance between segment and point
  /// \return leftSegmentClosest closest point on left segment
  void computeSquareDistanceSegmentPoint (const CkcdPoint& leftEndPoint1,
					  const CkcdPoint& leftEndPoint2,
					  const CkcdPoint& rightPoint,
					  kcdReal& squareDistance,
					  CkcdPoint& leftSegmentClosest);

  /// \brief Compute square distance between two segments.
  ///
  /// \param leftEndPoint1 left segment first end point
  /// \param leftEndPoint1 left segment second end point
  /// \param rightEndPoint1 right segment first end point
  /// \param rightEndPoint1 right segment second end point
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

  /// \brief Compute square distance between a segment and a box.
  ///
  /// \param leftEndPoint1 left segment first end point
  /// \param leftEndPoint1 left segment second end point
  /// \param rightBoundingBox right bounding box
  /// \return squareDistance square distance between segment and box
  void computeSquareDistanceSegmentBox (const CkcdPoint& leftEndPoint1,
					const CkcdPoint& leftEndPoint2,
					const CkcdTestTreeOBB::CkcdPolyOBBCache&
					rightPolyOBBCache,
					kcdReal& squareDistance);

  /// \brief Compute square distance between a segment and a box.
  ///
  /// \param leftEndPoint1 left segment first end point
  /// \param leftEndPoint1 left segment second end point
  /// \param rightTriangle right triangle, array of 3 vertices
  /// \return squareDistance square distance between segment and triangle
  /// \return leftSegmentClosest closest point on left segment
  /// \return rightTriangleClosest closest point on right triangle
  void computeSquareDistanceSegmentTriangle (const CkcdPoint& leftEndPoint1,
					     const CkcdPoint& leftEndPoint2,
					     const CkcdTestTreeOBB::
					     CkcdTriangleCache<CkcdPoint>&
					     rightTriangleCache,
					     kcdReal& squareDistance,
					     CkcdPoint& leftSegmentClosest,
					     CkcdPoint& rightTriangleClosest);

  /// \brief Compute bounding capsule of a polyhedron.
  ///
  /// Compute axis of capsule segment using least-squares fit. Radius
  /// is maximum distance from points to axis. Hemispherical caps are
  /// chosen as close together as possible.
  ///
  /// \param polyhedron polyhedron that contains the points
  /// \return endPoint1 bounding capsule segment first end point
  /// \return endPoint2 bounding capsule segment second end point
  /// \return radius bounding capsule radius
  void computeBoundingCapsulePolyhedron (const CkcdPolyhedronShPtr& polyhedron,
					 CkcdPoint& endPoint1,
					 CkcdPoint& endPoint2,
					 kcdReal& radius);
 
} // end of namespace kcd.
#endif //! KCD_UTIL_HH_
