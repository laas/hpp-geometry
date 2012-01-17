// Copyright (C) 2011 by Antonio El Khoury.
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
 * \file src/hpp/geometry/collision/detector-capsule-capsule.cc
 *
 * \brief Implementation of DetectorCapsuleCapsule.
 */

#include "hpp/geometry/collision/detector-capsule-capsule.hh"
#include "hpp/geometry/collision/test-tree-capsule.hh"
#include "hpp/geometry/collision/util.hh"

namespace kcd
{
  DetectorCapsuleCapsuleShPtr DetectorCapsuleCapsule::
  create ()
  {
    DetectorCapsuleCapsule* ptr = new DetectorCapsuleCapsule ();
    DetectorCapsuleCapsuleShPtr	shPtr (ptr);

    if (KD_OK != ptr->init (shPtr))
      {
	shPtr.reset();
      }

    return shPtr;
  }

  DetectorCapsuleCapsuleShPtr DetectorCapsuleCapsule::
  createCopy (const DetectorCapsuleCapsuleConstShPtr& detector)
  {
    DetectorCapsuleCapsule* ptr = new DetectorCapsuleCapsule (*detector);
    DetectorCapsuleCapsuleShPtr shPtr (ptr);

    if (KD_OK != ptr->init (shPtr))
      {
	shPtr.reset ();
      }

    return shPtr;
  }

  CkcdDetectorShPtr DetectorCapsuleCapsule::
  clone () const
  {
    return DetectorCapsuleCapsule::createCopy (weakPtr_.lock ());
  }

  CkcdDetectorTestAnswer DetectorCapsuleCapsule::
  analyze (const CkcdTreeIterator& left, 
	   const CkcdTreeIterator& right,
	   const CkcdDetectorElementaryTestDataShPtr& testData,
	   CkcdProximityQuery& query) const
  {
    CkcdDetectorTestAnswer testAnswer;

    // Retrieve capsules information.
    const TestTreeCapsule* leftTree
      = static_cast<TestTreeCapsule*> (left.testTree ());
    const TestTreeCapsule* rightTree
      = static_cast<TestTreeCapsule*> (right.testTree ());
    CkcdPoint leftEndPoint1, leftEndPoint2, rightEndPoint1, rightEndPoint2;
    kcdReal leftRadius, rightRadius;

    leftTree->getCapsule (left, leftEndPoint1, leftEndPoint2, leftRadius);
    rightTree->getCapsule (right, rightEndPoint1, rightEndPoint2, rightRadius);

    // Apply transformation to have both positions in the same frame.
    rightEndPoint1 = testData->rightToLeftTransformation () * rightEndPoint1;
    rightEndPoint2 = testData->rightToLeftTransformation () * rightEndPoint2;

    // Compute Distance between the two capsules axes.
    kcdReal squareDistance;
    CkcdPoint leftSegmentClosest;
    CkcdPoint rightSegmentClosest;

    computeSquareDistanceSegmentSegment (leftEndPoint1,
					 leftEndPoint2,
					 rightEndPoint1,
					 rightEndPoint2,
					 squareDistance,
					 leftSegmentClosest,
					 rightSegmentClosest);

    kcdReal radiusSum = leftRadius + rightRadius;

    // depending on the result, we will call one of the 4 report
    // functions of CkcdProximityQuery
    if (squareDistance < radiusSum * radiusSum)
      {
	if (left.countChildren () > 0 || right.countChildren () > 0)
	  {
	    // if it is not a leaf, report an overlap (of bounding volumes)
	    testAnswer = query.reportOverlap (left, right, testData);
	  }
	else
	  {
	    // if it is a leaf, report a collision.
	    testAnswer = query.reportCollision (left, right, testData);
	  }
      }
    else
      {
	if (left.countChildren () > 0 || right.countChildren () > 0)
	  {
	    // if it is not a leaf, report an estimated distance
	    testAnswer
	      = query.reportEstimatedDistance (left,
					       right,
					       testData,
					       sqrt (squareDistance) - radiusSum);
	  }
	else
	  {
	    // if it is a leaf, report an exact distance
	    CkitVect3 axis = rightSegmentClosest - leftSegmentClosest;
	    axis.normalize ();
	    CkcdPoint leftClosest = leftSegmentClosest
	      + axis * leftRadius;
	    CkcdPoint rightClosest = rightSegmentClosest
	      - axis * rightRadius;

	    testAnswer = query.reportExactDistance (left,
						    right,
						    testData,
						    sqrt(squareDistance)
						    - radiusSum,
						    leftClosest,
						    rightClosest);
	  }
      }

    return testAnswer;
  }

  bool DetectorCapsuleCapsule::
  canHandle (unsigned int leftID, unsigned int rightID) const
  {
    return ((leftID == TestTreeCapsule::capsuleDispatchID ()) &&
	    (rightID == TestTreeCapsule::capsuleDispatchID ()));
  }
  
  ktStatus DetectorCapsuleCapsule::
  init (const DetectorCapsuleCapsuleWkPtr& weakPtr)
  {
    ktStatus success = KD_OK;

    success = CkcdDetector::init (weakPtr);

    if (KD_OK == success)
      {
	weakPtr_ = weakPtr;
      }

    return success;
  }

  DetectorCapsuleCapsule::
  DetectorCapsuleCapsule ()
    : CkcdDetector ()
  {
  }

  DetectorCapsuleCapsule::
  DetectorCapsuleCapsule (const DetectorCapsuleCapsule& detector)
    : CkcdDetector (detector)
  {
  }

} // end of namespace kcd.
