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
 * \file src/hpp/geometry/collision/detector-triangle-capsule.cc
 *
 * \brief Implementation of DetectorTriangleCapsule.
 */

#include "hpp/geometry/collision/detector-triangle-capsule.hh"
#include "hpp/geometry/collision/test-tree-capsule.hh"
#include "hpp/geometry/collision/util.hh"

namespace kcd
{
  DetectorTriangleCapsuleShPtr DetectorTriangleCapsule::
  create ()
  {
    DetectorTriangleCapsule* ptr = new DetectorTriangleCapsule ();
    DetectorTriangleCapsuleShPtr shPtr (ptr);

    if (KD_OK != ptr->init (shPtr))
      {
	shPtr.reset();
      }

    return shPtr;
  }

  DetectorTriangleCapsuleShPtr DetectorTriangleCapsule::
  createCopy (const DetectorTriangleCapsuleConstShPtr& detector)
  {
    DetectorTriangleCapsule* ptr = new DetectorTriangleCapsule (*detector);
    DetectorTriangleCapsuleShPtr shPtr (ptr);

    if (KD_OK != ptr->init (shPtr))
      {
	shPtr.reset ();
      }

    return shPtr;
  }

  CkcdDetectorShPtr DetectorTriangleCapsule::
  clone () const
  {
    return DetectorTriangleCapsule::createCopy (weakPtr_.lock ());
  }

  CkcdDetectorTestAnswer DetectorTriangleCapsule::
  analyze (const CkcdTreeIterator& left, 
	   const CkcdTreeIterator& right,
	   const CkcdDetectorElementaryTestDataShPtr& testData,
	   CkcdProximityQuery& query) const
  {
    CkcdDetectorTestAnswer testAnswer;

    // Retrieve capsule and Triangle information.
    const CkcdTestTreePolyBV* leftTree
      = static_cast<CkcdTestTreePolyBV*> (left.testTree ());
    const TestTreeCapsule* rightTree
      = static_cast<TestTreeCapsule*> (right.testTree ());
    CkcdPoint rightEndPoint1, rightEndPoint2;
    kcdReal rightRadius;
    CkcdTestTreeOBB::CkcdTriangleCache<CkcdPoint> leftTriangleCache;

    leftTree->fillTriangleCache (left, leftTriangleCache);
    rightTree->getCapsule (right, rightEndPoint1, rightEndPoint2, rightRadius);

    // Apply transformation to have both positions in the same frame.
    rightEndPoint1 = testData->rightToLeftTransformation () * rightEndPoint1;
    rightEndPoint2 = testData->rightToLeftTransformation () * rightEndPoint2;

    // Compute Distance between the capsules axis and the triangle.
    kcdReal squareDistance;
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
    if (squareDistance < rightRadius * rightRadius)
      {
	if (right.countChildren () > 0)
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
	if (right.countChildren () > 0)
	  {
	    // if it is not a leaf, report an estimated distance
	    testAnswer
	      = query.reportEstimatedDistance (left,
					       right,
					       testData,
					       sqrt (squareDistance) - rightRadius);
	  }
	else
	  {
	    // if it is a leaf, report an exact distance
	    CkitVect3 axis = leftTriangleClosest - rightSegmentClosest;
	    axis.normalize ();
	    CkcdPoint rightCapsuleClosest = rightSegmentClosest
	      + axis * rightRadius;

	    testAnswer = query.reportExactDistance (left,
	    					    right,
	    					    testData,
	    					    sqrt (squareDistance)
	    					    - rightRadius,
	    					    rightCapsuleClosest,
	    					    leftTriangleClosest);
	  }
      }

    return testAnswer;
  }

  bool DetectorTriangleCapsule::
  canHandle (unsigned int leftID, unsigned int rightID) const
  {
    return ((leftID == CkcdTestTreeOBB::triangleDispatchID ()) &&
	    (rightID == TestTreeCapsule::capsuleDispatchID ()));
  }
  
  ktStatus DetectorTriangleCapsule::
  init (const DetectorTriangleCapsuleWkPtr& weakPtr)
  {
    ktStatus success = KD_OK;

    success = CkcdDetector::init (weakPtr);

    if (KD_OK == success)
      {
	weakPtr_ = weakPtr;
      }

    return success;
  }

  DetectorTriangleCapsule::
  DetectorTriangleCapsule ()
    : CkcdDetector ()
  {
  }

  DetectorTriangleCapsule::
  DetectorTriangleCapsule (const DetectorTriangleCapsule& detector)
    : CkcdDetector (detector)
  {
  }

} // end of namespace kcd.
