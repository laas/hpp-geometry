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
 * \file src/kcd/detector-capsule-obb.cc
 *
 * \brief Implementation of DetectorCapsuleOBB.
 */

#include "kcd/detector-capsule-obb.hh"
#include "kcd/test-tree-capsule.hh"
#include "kcd/util.hh"

namespace kcd
{
  DetectorCapsuleOBBShPtr DetectorCapsuleOBB::
  create ()
  {
    DetectorCapsuleOBB* ptr = new DetectorCapsuleOBB ();
    DetectorCapsuleOBBShPtr	shPtr (ptr);

    if (KD_OK != ptr->init (shPtr))
      {
	shPtr.reset();
      }

    return shPtr;
  }

  DetectorCapsuleOBBShPtr DetectorCapsuleOBB::
  createCopy (const DetectorCapsuleOBBConstShPtr& detector)
  {
    DetectorCapsuleOBB* ptr = new DetectorCapsuleOBB (*detector);
    DetectorCapsuleOBBShPtr shPtr (ptr);

    if (KD_OK != ptr->init (shPtr))
      {
	shPtr.reset ();
      }

    return shPtr;
  }

  CkcdDetectorShPtr DetectorCapsuleOBB::
  clone () const
  {
    return DetectorCapsuleOBB::createCopy (weakPtr_.lock ());
  }

  CkcdDetectorTestAnswer DetectorCapsuleOBB::
  analyze (const CkcdTreeIterator& left, 
	   const CkcdTreeIterator& right,
	   const CkcdDetectorElementaryTestDataShPtr& testData,
	   CkcdProximityQuery& query) const
  {
    CkcdDetectorTestAnswer testAnswer;

    // Retrieve capsule and OBB information.
    const TestTreeCapsule* leftTree
      = static_cast<TestTreeCapsule*> (left.testTree ());
    const CkcdTestTreePolyBV* rightTree
      = static_cast<CkcdTestTreePolyBV*> (right.testTree ());
    CkcdPoint leftEndPoint1, leftEndPoint2;
    kcdReal leftRadius;
    CkcdTestTreeOBB::CkcdPolyOBBCache rightPolyOBBCache;

    leftTree->getCapsule (left, leftEndPoint1, leftEndPoint2, leftRadius);
    rightTree->fillOBBCache (right, true, rightPolyOBBCache);

    // Apply transformation to have both positions in the same frame.
    rightPolyOBBCache.m_matrix = testData->rightToLeftTransformation ()
      * rightPolyOBBCache.m_matrix;

    // Compute Distance between the capsules axis and the OBB.
    kcdReal squareDistance;

    computeSquareDistanceSegmentBox (leftEndPoint1,
    				     leftEndPoint2,
    				     rightPolyOBBCache,
    				     squareDistance);

    // depending on the result, we will call one of the 4 report
    // functions of CkcdProximityQuery
    if (squareDistance < leftRadius * leftRadius)
      {
    	if (left.countChildren () > 0)
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
					   sqrt (squareDistance) - leftRadius);
      }

    return testAnswer;
  }

  bool DetectorCapsuleOBB::
  canHandle (unsigned int leftID, unsigned int rightID) const
  {
    return ((leftID == TestTreeCapsule::capsuleDispatchID ()) &&
	    (rightID == CkcdTestTreeOBB::polyOBBDispatchID ()));
  }
  
  ktStatus DetectorCapsuleOBB::
  init (const DetectorCapsuleOBBWkPtr& weakPtr)
  {
    ktStatus success = KD_OK;

    success = CkcdDetector::init (weakPtr);

    if (KD_OK == success)
      {
	weakPtr_ = weakPtr;
      }

    return success;
  }

  DetectorCapsuleOBB::
  DetectorCapsuleOBB ()
    : CkcdDetector ()
  {
  }

  DetectorCapsuleOBB::
  DetectorCapsuleOBB (const DetectorCapsuleOBB& detector)
    : CkcdDetector (detector)
  {
  }

} // end of namespace kcd.
