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
 * \file src/hpp/geometry/collision/detector-capsule-segment.cc
 *
 * \brief Implementation of DetectorCapsuleSegment.
 */

#include "hpp/geometry/collision/detector-capsule-segment.hh"
#include "hpp/geometry/collision/test-tree-capsule.hh"
#include "hpp/geometry/collision/test-tree-segment.hh"
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
      KCD_REGISTER_DETECTOR(DetectorCapsuleSegment);
#pragma GCC diagnostic pop

      DetectorCapsuleSegmentShPtr DetectorCapsuleSegment::
      create ()
      {
	DetectorCapsuleSegment* ptr = new DetectorCapsuleSegment ();
	DetectorCapsuleSegmentShPtr	shPtr (ptr);

	if (KD_OK != ptr->init (shPtr))
	  {
	    shPtr.reset();
	  }

	return shPtr;
      }

      DetectorCapsuleSegmentShPtr DetectorCapsuleSegment::
      createCopy (const DetectorCapsuleSegmentConstShPtr& detector)
      {
	DetectorCapsuleSegment* ptr = new DetectorCapsuleSegment (*detector);
	DetectorCapsuleSegmentShPtr shPtr (ptr);

	if (KD_OK != ptr->init (shPtr))
	  {
	    shPtr.reset ();
	  }

	return shPtr;
      }

      CkcdDetectorShPtr DetectorCapsuleSegment::
      clone () const
      {
	return DetectorCapsuleSegment::createCopy (weakPtr_.lock ());
      }

      CkcdDetectorTestAnswer DetectorCapsuleSegment::
      analyze (const CkcdTreeIterator& left, 
	       const CkcdTreeIterator& right,
	       const CkcdDetectorElementaryTestDataShPtr& testData,
	       CkcdProximityQuery& query) const
      {
	CkcdDetectorTestAnswer testAnswer;

	// Retrieve capsule and segment information.
	const TestTreeCapsule* leftTree
	  = static_cast<TestTreeCapsule*> (left.testTree ());
	const TestTreeSegment* rightTree
	  = static_cast<TestTreeSegment*> (right.testTree ());
	CkcdPoint leftEndPoint1, leftEndPoint2, rightEndPoint1, rightEndPoint2;
	kcdReal leftRadius, rightRadius;

	leftTree->getCapsule (left, leftEndPoint1, leftEndPoint2, leftRadius);
	rightTree->getSegment (right, rightEndPoint1, rightEndPoint2, rightRadius);

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
	if (squareDistance < leftRadius * leftRadius)
	  {
	    // if it is a leaf, report a collision.
	    testAnswer = query.reportCollision (left, right, testData);
	  }
	else
	  {
	    // if it is a leaf, report an exact distance
	    CkitVect3 axis = rightSegmentClosest - leftSegmentClosest;
	    axis.normalize ();
	    CkcdPoint leftClosest = leftSegmentClosest
	      + axis * leftRadius;

	    testAnswer = query.reportExactDistance (left,
						    right,
						    testData,
						    static_cast<kcdReal>
						    (sqrt(squareDistance))
						    - leftRadius,
						    leftClosest,
						    rightSegmentClosest);
	  }

	return testAnswer;
      }

      bool DetectorCapsuleSegment::
      canHandle (unsigned int leftID, unsigned int rightID) const
      {
	return ((leftID == TestTreeCapsule::capsuleDispatchID ()) &&
		(rightID == TestTreeSegment::segmentDispatchID ()));
      }
  
      ktStatus DetectorCapsuleSegment::
      init (const DetectorCapsuleSegmentWkPtr& weakPtr)
      {
	ktStatus success = KD_OK;

	success = CkcdDetector::init (weakPtr);

	if (KD_OK == success)
	  {
	    weakPtr_ = weakPtr;
	  }

	return success;
      }

      DetectorCapsuleSegment::
      DetectorCapsuleSegment ()
	: CkcdDetector ()
      {
      }

      DetectorCapsuleSegment::
      DetectorCapsuleSegment (const DetectorCapsuleSegment& detector)
	: CkcdDetector (detector)
      {
      }

    } // end of namespace collision.
  } // end of namespace geometry.
} // end of namespace hpp.
