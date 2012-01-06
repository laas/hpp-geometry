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
 * \file src/kcd/detector-segment-obb.cc
 *
 * \brief Implementation of DetectorSegmentOBB.
 */

#include <limits>

#include "kcd/detector-segment-obb.hh"
#include "kcd/test-tree-segment.hh"
#include "kcd/util.hh"

namespace kcd
{
  DetectorSegmentOBBShPtr DetectorSegmentOBB::
  create ()
  {
    DetectorSegmentOBB* ptr = new DetectorSegmentOBB ();
    DetectorSegmentOBBShPtr	shPtr (ptr);

    if (KD_OK != ptr->init (shPtr))
      {
	shPtr.reset();
      }

    return shPtr;
  }

  DetectorSegmentOBBShPtr DetectorSegmentOBB::
  createCopy (const DetectorSegmentOBBConstShPtr& detector)
  {
    DetectorSegmentOBB* ptr = new DetectorSegmentOBB (*detector);
    DetectorSegmentOBBShPtr shPtr (ptr);

    if (KD_OK != ptr->init (shPtr))
      {
	shPtr.reset ();
      }

    return shPtr;
  }

  CkcdDetectorShPtr DetectorSegmentOBB::
  clone () const
  {
    return DetectorSegmentOBB::createCopy (weakPtr_.lock ());
  }

  CkcdDetectorTestAnswer DetectorSegmentOBB::
  analyze (const CkcdTreeIterator& left, 
	   const CkcdTreeIterator& right,
	   const CkcdDetectorElementaryTestDataShPtr& testData,
	   CkcdProximityQuery& query) const
  {
    CkcdDetectorTestAnswer testAnswer;

    // Retrieve segment and OBB information.
    const TestTreeSegment* leftTree
      = static_cast<TestTreeSegment*> (left.testTree ());
    const CkcdTestTreePolyBV* rightTree
      = static_cast<CkcdTestTreePolyBV*> (right.testTree ());
    CkcdPoint leftEndPoint1, leftEndPoint2;
    CkcdTestTreeOBB::CkcdPolyOBBCache rightPolyOBBCache;

    leftTree->getSegment (left, leftEndPoint1, leftEndPoint2);
    rightTree->fillOBBCache (right, true, rightPolyOBBCache);

    // Apply transformation to have both positions in the same frame.
    rightPolyOBBCache.m_matrix = testData->rightToLeftTransformation ()
      * rightPolyOBBCache.m_matrix;

    // Compute Distance between the segments axis and the OBB.
    kcdReal squareDistance;

    computeSquareDistanceSegmentBox (leftEndPoint1,
    				     leftEndPoint2,
    				     rightPolyOBBCache,
    				     squareDistance);

    // depending on the result, we will call one of the 4 report
    // functions of CkcdProximityQuery
    if (squareDistance < std::numeric_limits<kcdReal>::epsilon ()
	* std::numeric_limits<kcdReal>::epsilon ())
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
					   sqrt (squareDistance));
      }

    return testAnswer;
  }

  bool DetectorSegmentOBB::
  canHandle (unsigned int leftID, unsigned int rightID) const
  {
    return ((leftID == TestTreeSegment::segmentDispatchID ()) &&
	    (rightID == CkcdTestTreeOBB::polyOBBDispatchID ()));
  }
  
  ktStatus DetectorSegmentOBB::
  init (const DetectorSegmentOBBWkPtr& weakPtr)
  {
    ktStatus success = KD_OK;

    success = CkcdDetector::init (weakPtr);

    if (KD_OK == success)
      {
	weakPtr_ = weakPtr;
      }

    return success;
  }

  DetectorSegmentOBB::
  DetectorSegmentOBB ()
    : CkcdDetector ()
  {
  }

  DetectorSegmentOBB::
  DetectorSegmentOBB (const DetectorSegmentOBB& detector)
    : CkcdDetector (detector)
  {
  }

} // end of namespace kcd.
