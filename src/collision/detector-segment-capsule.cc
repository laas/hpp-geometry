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
 * \file src/hpp/geometry/collision/detector-segment-capsule.cc
 *
 * \brief Implementation of DetectorSegmentCapsule.
 */

#include "hpp/geometry/collision/detector-segment-capsule.hh"
#include "hpp/geometry/collision/test-tree-segment.hh"
#include "hpp/geometry/collision/test-tree-capsule.hh"
#include "hpp/geometry/collision/util.hh"

namespace hpp
{
  namespace geometry
  {
    namespace collision
    {
      // this line registers the detector in the global detector dispatcher
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-pedantic"
      KCD_REGISTER_DETECTOR(DetectorSegmentCapsule);
#pragma GCC diagnostic pop

      DetectorSegmentCapsuleShPtr DetectorSegmentCapsule::
      create ()
      {
	DetectorSegmentCapsule* ptr = new DetectorSegmentCapsule ();
	DetectorSegmentCapsuleShPtr	shPtr (ptr);

	if (KD_OK != ptr->init (shPtr))
	  {
	    shPtr.reset();
	  }

	return shPtr;
      }

      DetectorSegmentCapsuleShPtr DetectorSegmentCapsule::
      createCopy (const DetectorSegmentCapsuleConstShPtr& detector)
      {
	DetectorSegmentCapsule* ptr = new DetectorSegmentCapsule (*detector);
	DetectorSegmentCapsuleShPtr shPtr (ptr);

	if (KD_OK != ptr->init (shPtr))
	  {
	    shPtr.reset ();
	  }

	return shPtr;
      }

      CkcdDetectorShPtr DetectorSegmentCapsule::
      clone () const
      {
	return DetectorSegmentCapsule::createCopy (weakPtr_.lock ());
      }

      CkcdDetectorTestAnswer DetectorSegmentCapsule::
      analyze (const CkcdTreeIterator& left, 
	       const CkcdTreeIterator& right,
	       const CkcdDetectorElementaryTestDataShPtr& testData,
	       CkcdProximityQuery& query) const
      {
	CkcdDetectorTestAnswer testAnswer;

	// Retrieve segment and capsule information.
	const TestTreeSegment* leftTree
	  = static_cast<TestTreeSegment*> (left.testTree ());
	const TestTreeCapsule* rightTree
	  = static_cast<TestTreeCapsule*> (right.testTree ());
	CkcdPoint leftEndPoint1, leftEndPoint2, rightEndPoint1, rightEndPoint2;
	kcdReal leftRadius, rightRadius;

	leftTree->getSegment (left, leftEndPoint1, leftEndPoint2, leftRadius);
	rightTree->getCapsule (right, rightEndPoint1, rightEndPoint2, rightRadius);

	// Apply transformation to have both positions in the same frame.
	rightEndPoint1 = testData->rightToLeftTransformation () * rightEndPoint1;
	rightEndPoint2 = testData->rightToLeftTransformation () * rightEndPoint2;

	// Compute Distance between the two capsules axes.
	hppReal squareDistance;
	CkcdPoint leftSegmentClosest;
	CkcdPoint rightSegmentClosest;

	computeSquareDistanceSegmentSegment (leftEndPoint1,
					     leftEndPoint2,
					     rightEndPoint1,
					     rightEndPoint2,
					     squareDistance,
					     leftSegmentClosest,
					     rightSegmentClosest);

	// depending on the result, we will call one of the 4 report
	// functions of CkcdProximityQuery
	if (squareDistance < rightRadius * rightRadius)
	  {
	    // if it is a leaf, report a collision.
	    testAnswer = query.reportCollision (left, right, testData);
	  }
	else
	  {
	    // if it is a leaf, report an exact distance
	    CkitVect3 axis = rightSegmentClosest - leftSegmentClosest;
	    axis.normalize ();
	    CkcdPoint rightClosest = rightSegmentClosest
	      - axis * rightRadius;

	    testAnswer = query.reportExactDistance (left,
						    right,
						    testData,
						    static_cast<kcdReal>
						    (sqrt(squareDistance))
						    - rightRadius,
						    leftSegmentClosest,
						    rightClosest);
	  }

	return testAnswer;
      }

      bool DetectorSegmentCapsule::
      canHandle (unsigned int leftID, unsigned int rightID) const
      {
	return ((leftID == TestTreeSegment::segmentDispatchID ()) &&
		(rightID == TestTreeCapsule::capsuleDispatchID ()));
      }
  
      ktStatus DetectorSegmentCapsule::
      init (const DetectorSegmentCapsuleWkPtr& weakPtr)
      {
	ktStatus success = KD_OK;

	success = CkcdDetector::init (weakPtr);

	if (KD_OK == success)
	  {
	    weakPtr_ = weakPtr;
	  }

	return success;
      }

      DetectorSegmentCapsule::
      DetectorSegmentCapsule ()
	: CkcdDetector ()
      {
      }

      DetectorSegmentCapsule::
      DetectorSegmentCapsule (const DetectorSegmentCapsule& detector)
	: CkcdDetector (detector)
      {
      }

    } // end of namespace collision.
  } // end of namespace geometry.
} // end of namespace hpp.
