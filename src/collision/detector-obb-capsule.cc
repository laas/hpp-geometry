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
 * \file src/hpp/geometry/collision/detector-obb-capsule.cc
 *
 * \brief Implementation of DetectorOBBCapsule.
 */

#include "hpp/geometry/collision/detector-obb-capsule.hh"
#include "hpp/geometry/collision/test-tree-capsule.hh"
#include "hpp/geometry/collision/util.hh"

namespace hpp
{
  namespace geometry
  {
    namespace collision
    {
      // this line registers the detector in the global detector dispatcher
      KCD_REGISTER_DETECTOR(DetectorOBBCapsule);

  DetectorOBBCapsuleShPtr DetectorOBBCapsule::
  create ()
  {
    DetectorOBBCapsule* ptr = new DetectorOBBCapsule ();
    DetectorOBBCapsuleShPtr shPtr (ptr);

    if (KD_OK != ptr->init (shPtr))
      {
	shPtr.reset();
      }

    return shPtr;
  }

  DetectorOBBCapsuleShPtr DetectorOBBCapsule::
  createCopy (const DetectorOBBCapsuleConstShPtr& detector)
  {
    DetectorOBBCapsule* ptr = new DetectorOBBCapsule (*detector);
    DetectorOBBCapsuleShPtr shPtr (ptr);

    if (KD_OK != ptr->init (shPtr))
      {
	shPtr.reset ();
      }

    return shPtr;
  }

  CkcdDetectorShPtr DetectorOBBCapsule::
  clone () const
  {
    return DetectorOBBCapsule::createCopy (weakPtr_.lock ());
  }

  CkcdDetectorTestAnswer DetectorOBBCapsule::
  analyze (const CkcdTreeIterator& left, 
	   const CkcdTreeIterator& right,
	   const CkcdDetectorElementaryTestDataShPtr& testData,
	   CkcdProximityQuery& query) const
  {
    CkcdDetectorTestAnswer testAnswer;

    // Retrieve capsule and OBB information.
    const CkcdTestTreePolyBV* leftTree
      = static_cast<CkcdTestTreePolyBV*> (left.testTree ());
    const TestTreeCapsule* rightTree
      = static_cast<TestTreeCapsule*> (right.testTree ());
    CkcdPoint rightEndPoint1, rightEndPoint2;
    kcdReal rightRadius;
    CkcdTestTreeOBB::CkcdPolyOBBCache leftPolyOBBCache;
    
    leftTree->fillOBBCache (left, false, leftPolyOBBCache);
    rightTree->getCapsule (right, rightEndPoint1, rightEndPoint2, rightRadius);
    
    // Apply transformation to have both positions in the same frame.
    rightEndPoint1 = testData->rightToLeftTransformation () * rightEndPoint1;
    rightEndPoint2 = testData->rightToLeftTransformation () * rightEndPoint2;

    // Compute Distance between the capsules axis and the OBB.
    kcdReal squareDistance;

    computeSquareDistanceSegmentBox (rightEndPoint1,
    				     rightEndPoint2,
    				     leftPolyOBBCache,
    				     squareDistance);

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
	// if it is not a leaf, report an estimated distance
	testAnswer
	  = query.reportEstimatedDistance (left,
					   right,
					   testData,
					   sqrt (squareDistance) - rightRadius);
      }

    return testAnswer;
  }

  bool DetectorOBBCapsule::
  canHandle (unsigned int leftID, unsigned int rightID) const
  {
    return ((leftID == CkcdTestTreeOBB::polyOBBDispatchID ()) &&
	    (rightID == TestTreeCapsule::capsuleDispatchID ()));
  }
  
  ktStatus DetectorOBBCapsule::
  init (const DetectorOBBCapsuleWkPtr& weakPtr)
  {
    ktStatus success = KD_OK;

    success = CkcdDetector::init (weakPtr);

    if (KD_OK == success)
      {
	weakPtr_ = weakPtr;
      }

    return success;
  }

  DetectorOBBCapsule::
  DetectorOBBCapsule ()
    : CkcdDetector ()
  {
  }

  DetectorOBBCapsule::
  DetectorOBBCapsule (const DetectorOBBCapsule& detector)
    : CkcdDetector (detector)
  {
  }

    } // end of namespace collision.
  } // end of namespace geometry.
} // end of namespace hpp.
