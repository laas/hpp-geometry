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


#ifndef KCD_UTIL_HH_
# define KCD_UTIL_HH_

# include <iostream>

# include <geometric-tools/Wm5Vector3.h>
# include <geometric-tools/Wm5DistPoint3Segment3.h>
# include <geometric-tools/Wm5DistSegment3Segment3.h>
# include <geometric-tools/Wm5DistSegment3Box3.h>
# include <geometric-tools/Wm5DistSegment3Triangle3.h>

# include <kcd2/kcdInterface.h>

namespace hpp
{
  namespace geometry
  {
    namespace collision
    {
      typedef float hppReal;

      /// \brief Convert CkcdPoint to Geometric Tools Vector3.
      template<typename hppReal>
      inline void convertKcdPointToVector3 (Wm5::Vector3<hppReal>& dst,
					    const CkcdPoint& src)
      {
	dst[0] = src[0];
	dst[1] = src[1];
	dst[2] = src[2];
      }

      /// \brief Convert Geometric Tools Vector3 to CkcdPoint.
      template<typename hppReal>
      inline void convertVector3ToKcdPoint (CkcdPoint& dst,
					    const Wm5::Vector3<hppReal>& src)
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
      template<typename hppReal>
      inline void
      computeSquareDistanceSegmentPoint (const CkcdPoint& leftEndPoint1,
					 const CkcdPoint& leftEndPoint2,
					 const CkcdPoint& rightPoint,
					 hppReal& squareDistance,
					 CkcdPoint& leftSegmentClosest)
      {
	using namespace Wm5;

	Wm5::Vector3<hppReal> leftP0;
	Wm5::Vector3<hppReal> leftP1;
	Wm5::Vector3<hppReal> rightP;

	convertKcdPointToVector3 (leftP0, leftEndPoint1);
	convertKcdPointToVector3 (leftP1, leftEndPoint2);
	convertKcdPointToVector3 (rightP, rightPoint);

	Segment3<hppReal> s0 (leftP0, leftP1);

	DistPoint3Segment3<hppReal> distance (rightP, s0);

	squareDistance = distance.GetSquared ();

	Wm5::Vector3<hppReal> wm5LeftSegmentClosest = s0.Center
	  + distance.GetSegmentParameter () * s0.Direction;

	convertVector3ToKcdPoint (leftSegmentClosest, wm5LeftSegmentClosest);
      }
      
      /// \brief Compute square distance between two segments.
      ///
      /// \param leftEndPoint1 left segment first end point
      /// \param leftEndPoint1 left segment second end point
      /// \param rightEndPoint1 right segment first end point
      /// \param rightEndPoint1 right segment second end point
      /// \return squareDistance square distance between the two segments
      /// \return leftSegmentClosest closest point on left segment
      /// \return rightSegmentClosest closest point on right segment
      template<typename hppReal>
      inline void
      computeSquareDistanceSegmentSegment (const CkcdPoint& leftEndPoint1,
					   const CkcdPoint& leftEndPoint2,
					   const CkcdPoint& rightEndPoint1,
					   const CkcdPoint& rightEndPoint2,
					   hppReal& squareDistance,
					   CkcdPoint& leftSegmentClosest,
					   CkcdPoint& rightSegmentClosest)
      {
	using namespace Wm5;

	Wm5::Vector3<hppReal> leftP0;
	Wm5::Vector3<hppReal> leftP1;
	Wm5::Vector3<hppReal> rightP0;
	Wm5::Vector3<hppReal> rightP1;

	convertKcdPointToVector3 (leftP0, leftEndPoint1);
	convertKcdPointToVector3 (leftP1, leftEndPoint2);
	convertKcdPointToVector3 (rightP0, rightEndPoint1);
	convertKcdPointToVector3 (rightP1, rightEndPoint2);

	Segment3<hppReal> s0 (leftP0, leftP1);
	Segment3<hppReal> s1 (rightP0, rightP1);

	DistSegment3Segment3<hppReal> distance (s0, s1);
    
	squareDistance = distance.GetSquared ();

	Wm5::Vector3<hppReal> wm5LeftSegmentClosest = s0.Center
	  + distance.GetSegment0Parameter () * s0.Direction;
	Wm5::Vector3<hppReal> wm5RightSegmentClosest = s1.Center
	  + distance.GetSegment1Parameter () * s1.Direction;

	convertVector3ToKcdPoint (leftSegmentClosest, wm5LeftSegmentClosest);
	convertVector3ToKcdPoint (rightSegmentClosest, wm5RightSegmentClosest);
      }

      /// \brief Compute square distance between a segment and a box
      /// represented by an OBB.
      ///
      /// \param leftEndPoint1 left segment first end point
      /// \param leftEndPoint1 left segment second end point
      /// \param rightPolyOBBCache right OBB
      /// \return squareDistance square distance between segment and box
      template<typename hppReal>
      inline void
      computeSquareDistanceSegmentBox (const CkcdPoint& leftEndPoint1,
				       const CkcdPoint& leftEndPoint2,
				       const CkcdTestTreeOBB::CkcdPolyOBBCache&
				       rightPolyOBBCache,
				       hppReal& squareDistance)
      {
	using namespace Wm5;

	// Define segment from left capsule axis.
	Wm5::Vector3<hppReal> leftP0;
	Wm5::Vector3<hppReal> leftP1;
	convertKcdPointToVector3 (leftP0, leftEndPoint1);
	convertKcdPointToVector3 (leftP1, leftEndPoint2);

	Segment3<hppReal> s0 (leftP0, leftP1);

	// Define box from right OBB.
	CkcdMat4 position = rightPolyOBBCache.m_matrix;

	CkcdPoint rightPolyOBBCacheCenter (position(0, 3),
					   position(1, 3),
					   position(2, 3));
	Wm5::Vector3<hppReal> center;
	convertKcdPointToVector3 (center, rightPolyOBBCacheCenter);

	Wm5::Vector3<hppReal> axis0 (position(0, 0), position(1, 0), position(2, 0));
	Wm5::Vector3<hppReal> axis1 (position(0, 1), position(1, 1), position(2, 1));
	Wm5::Vector3<hppReal> axis2 (position(0, 2), position(1, 2), position(2, 2));

	hppReal extent0 = rightPolyOBBCache.m_halfLength[0];
	hppReal extent1 = rightPolyOBBCache.m_halfLength[1];
	hppReal extent2 = rightPolyOBBCache.m_halfLength[2];

	Box3<hppReal> box (center, axis0, axis1, axis2, extent0, extent1, extent2);

	// Define distance and compute squared distance.
	DistSegment3Box3<hppReal> distance (s0, box);

	squareDistance = distance.GetSquared ();
      }

      /// \brief Compute square distance between a segment and a box.
      ///
      /// \param leftEndPoint1 left segment first end point
      /// \param leftEndPoint1 left segment second end point
      /// \param rightBoundingBox right bounding box
      /// \return squareDistance square distance between segment and box
      template<typename hppReal>
      inline void
      computeSquareDistanceSegmentBox (const CkcdPoint& leftEndPoint1,
				       const CkcdPoint& leftEndPoint2,
				       const CkcdBoundingBoxShPtr&
				       rightBoundingBox,
				       hppReal& squareDistance)
      {
	using namespace Wm5;

	// Define segment from left capsule axis.
	Wm5::Vector3<hppReal> leftP0;
	Wm5::Vector3<hppReal> leftP1;
	convertKcdPointToVector3 (leftP0, leftEndPoint1);
	convertKcdPointToVector3 (leftP1, leftEndPoint2);

	Segment3<hppReal> s0 (leftP0, leftP1);

	// Define box from right box.
	CkcdMat4 position = rightBoundingBox->relativePosition ();

	CkcdPoint rightBoundingBoxCenter (position(0, 3),
					  position(1, 3),
					  position(2, 3));
	Wm5::Vector3<hppReal> center;
	convertKcdPointToVector3 (center, rightBoundingBoxCenter);

	Wm5::Vector3<hppReal>
	  axis0 (position(0, 0), position(1, 0), position(2, 0));
	Wm5::Vector3<hppReal>
	  axis1 (position(0, 1), position(1, 1), position(2, 1));
	Wm5::Vector3<hppReal>
	  axis2 (position(0, 2), position(1, 2), position(2, 2));

	hppReal extent0, extent1, extent2;
	rightBoundingBox->getHalfLengths (extent0, extent1, extent2);
	
	Box3<hppReal> box (center, axis0, axis1, axis2,
			   extent0, extent1, extent2);

	// Define distance and compute squared distance.
	DistSegment3Box3<hppReal> distance (s0, box);

	squareDistance = distance.GetSquared ();
      }

      /// \brief Compute square distance between a segment and a box.
      ///
      /// \param leftEndPoint1 left segment first end point
      /// \param leftEndPoint1 left segment second end point
      /// \param rightTriangle right triangle, array of 3 vertices
      /// \return squareDistance square distance between segment and triangle
      /// \return leftSegmentClosest closest point on left segment
      /// \return rightTriangleClosest closest point on right triangle
      template<typename hppReal>
      inline void
      computeSquareDistanceSegmentTriangle (const CkcdPoint& leftEndPoint1,
					    const CkcdPoint& leftEndPoint2,
					    const CkcdTestTreeOBB::
					    CkcdTriangleCache<CkcdPoint>&
					    rightTriangleCache,
					    hppReal& squareDistance,
					    CkcdPoint& leftSegmentClosest,
					    CkcdPoint& rightTriangleClosest)
      {
	using namespace Wm5;

	// Define segment.
	Wm5::Vector3<hppReal> leftP0;
	Wm5::Vector3<hppReal> leftP1;
	convertKcdPointToVector3 (leftP0, leftEndPoint1);
	convertKcdPointToVector3 (leftP1, leftEndPoint2);

	Segment3<hppReal> s0 (leftP0, leftP1);

	// Define triangle.
	Wm5::Vector3<hppReal> rightV0;
	Wm5::Vector3<hppReal> rightV1;
	Wm5::Vector3<hppReal> rightV2;
	convertKcdPointToVector3 (rightV0, rightTriangleCache.m_vertex[0]);
	convertKcdPointToVector3 (rightV1, rightTriangleCache.m_vertex[1]);
	convertKcdPointToVector3 (rightV2, rightTriangleCache.m_vertex[2]);
    
	Triangle3<hppReal> triangle (rightV0, rightV1, rightV2);

	// Define distance.
	DistSegment3Triangle3<hppReal> distance (s0, triangle);

	// Compute distance and closest points.
	squareDistance = distance.GetSquared ();

	Wm5::Vector3<hppReal> wm5LeftSegmentClosest = s0.Center
	  + distance.GetSegmentParameter () * s0.Direction;
	Wm5::Vector3<hppReal> wm5RightSegmentClosest 
	  = distance.GetTriangleBary (0) * rightV0
	  + distance.GetTriangleBary (1) * rightV1
	  + distance.GetTriangleBary (2) * rightV2;

	convertVector3ToKcdPoint (leftSegmentClosest, wm5LeftSegmentClosest);
	convertVector3ToKcdPoint (rightTriangleClosest, wm5RightSegmentClosest);
      }

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
 
    } // end of namespace collision.
  } // end of namespace geometry.
} // end of namespace hpp.
#endif //! KCD_UTIL_HH_
