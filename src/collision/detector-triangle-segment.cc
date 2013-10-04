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
 * \file src/hpp/geometry/collision/detector-triangle-segment.cc
 *
 * \brief Implementation of DetectorTriangleSegment.
 */

#include <limits>

#include "hpp/geometry/collision/detector-triangle-segment.hh"
#include "hpp/geometry/collision/test-tree-segment.hh"
#include "hpp/geometry/collision/util.hh"

namespace hpp
{
  namespace geometry
  {
    namespace collision
    {
      // this line registers the detector in the global detector dispatcher
      KCD_REGISTER_DETECTOR(DetectorTriangleSegment);

      DetectorTriangleSegmentShPtr DetectorTriangleSegment::
      create ()
      {
	DetectorTriangleSegment* ptr = new DetectorTriangleSegment ();
	DetectorTriangleSegmentShPtr shPtr (ptr);

	if (KD_OK != ptr->init (shPtr))
	  {
	    shPtr.reset();
	  }

	return shPtr;
      }

      DetectorTriangleSegmentShPtr DetectorTriangleSegment::
      createCopy (const DetectorTriangleSegmentConstShPtr& detector)
      {
	DetectorTriangleSegment* ptr = new DetectorTriangleSegment (*detector);
	DetectorTriangleSegmentShPtr shPtr (ptr);

	if (KD_OK != ptr->init (shPtr))
	  {
	    shPtr.reset ();
	  }

	return shPtr;
      }

      CkcdDetectorShPtr DetectorTriangleSegment::
      clone () const
      {
	return DetectorTriangleSegment::createCopy (weakPtr_.lock ());
      }

      CkcdDetectorTestAnswer DetectorTriangleSegment::
      analyze (const CkcdTreeIterator& left, 
	       const CkcdTreeIterator& right,
	       const CkcdDetectorElementaryTestDataShPtr& testData,
	       CkcdProximityQuery& query) const
      {
	CkcdDetectorTestAnswer testAnswer;

	// Retrieve segment and Triangle information.
	const CkcdTestTreePolyBV* leftTree
	  = static_cast<CkcdTestTreePolyBV*> (left.testTree ());
	const TestTreeSegment* rightTree
	  = static_cast<TestTreeSegment*> (right.testTree ());
	CkcdPoint rightEndPoint1, rightEndPoint2;
	kcdReal rightRadius;
	CkcdTestTreeOBB::CkcdTriangleCache<CkcdPoint> leftTriangleCache;

	leftTree->fillTriangleCache (left, leftTriangleCache);
	rightTree->getSegment (right, rightEndPoint1, rightEndPoint2, rightRadius);

	// Apply transformation to have both positions in the same frame.
	rightEndPoint1 = testData->rightToLeftTransformation () * rightEndPoint1;
	rightEndPoint2 = testData->rightToLeftTransformation () * rightEndPoint2;

	// Compute Distance between the segments axis and the triangle.
	hppReal squareDistance;
	CkcdPoint rightSegmentClosest;
	CkcdPoint leftTriangleClosest;

	computeSquareDistanceSegmentTriangle (rightEndPoint1,
					      rightEndPoint2,
					      leftTriangleCache,
					      squareDistance,
					      rightSegmentClosest,
					      leftTriangleClosest);

	// depending on the result, we will call one of the 4 report
	// functions of CkcdProximityQuery
	if (squareDistance < std::numeric_limits<hppReal>::epsilon ()
	    * std::numeric_limits<hppReal>::epsilon ())
	  {
	    // if it is a leaf, report a collision.
	    testAnswer = query.reportCollision (left, right, testData);
	  }
	else
	  {
	    // if it is a leaf, report an exact distance
	    testAnswer = query.reportExactDistance (left,
						    right,
						    testData,
						    static_cast<kcdReal>
						    (sqrt (squareDistance)),
						    rightSegmentClosest,
						    leftTriangleClosest);
	  }

	return testAnswer;
      }

      bool DetectorTriangleSegment::
      canHandle (unsigned int leftID, unsigned int rightID) const
      {
	return ((leftID == CkcdTestTreeOBB::triangleDispatchID ()) &&
		(rightID == TestTreeSegment::segmentDispatchID ()));
      }
  
      ktStatus DetectorTriangleSegment::
      init (const DetectorTriangleSegmentWkPtr& weakPtr)
      {
	ktStatus success = KD_OK;

	success = CkcdDetector::init (weakPtr);

	if (KD_OK == success)
	  {
	    weakPtr_ = weakPtr;
	  }

	return success;
      }

      DetectorTriangleSegment::
      DetectorTriangleSegment ()
	: CkcdDetector ()
      {
      }

      DetectorTriangleSegment::
      DetectorTriangleSegment (const DetectorTriangleSegment& detector)
	: CkcdDetector (detector)
      {
      }

    } // end of namespace collision.
  } // end of namespace geometry.
} // end of namespace hpp.
