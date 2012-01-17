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
 * \file src/hpp/geometry/collision/detector-segment-segment.cc
 *
 * \brief Implementation of DetectorSegmentSegment.
 */

#include <limits>

#include "hpp/geometry/collision/detector-segment-segment.hh"
#include "hpp/geometry/collision/test-tree-segment.hh"
#include "hpp/geometry/collision/util.hh"

namespace kcd
{
  DetectorSegmentSegmentShPtr DetectorSegmentSegment::
  create ()
  {
    DetectorSegmentSegment* ptr = new DetectorSegmentSegment ();
    DetectorSegmentSegmentShPtr	shPtr (ptr);

    if (KD_OK != ptr->init (shPtr))
      {
	shPtr.reset();
      }

    return shPtr;
  }

  DetectorSegmentSegmentShPtr DetectorSegmentSegment::
  createCopy (const DetectorSegmentSegmentConstShPtr& detector)
  {
    DetectorSegmentSegment* ptr = new DetectorSegmentSegment (*detector);
    DetectorSegmentSegmentShPtr shPtr (ptr);

    if (KD_OK != ptr->init (shPtr))
      {
	shPtr.reset ();
      }

    return shPtr;
  }

  CkcdDetectorShPtr DetectorSegmentSegment::
  clone () const
  {
    return DetectorSegmentSegment::createCopy (weakPtr_.lock ());
  }

  CkcdDetectorTestAnswer DetectorSegmentSegment::
  analyze (const CkcdTreeIterator& left, 
	   const CkcdTreeIterator& right,
	   const CkcdDetectorElementaryTestDataShPtr& testData,
	   CkcdProximityQuery& query) const
  {
    CkcdDetectorTestAnswer testAnswer;

    // Retrieve segments information.
    const TestTreeSegment* leftTree
      = static_cast<TestTreeSegment*> (left.testTree ());
    const TestTreeSegment* rightTree
      = static_cast<TestTreeSegment*> (right.testTree ());
    CkcdPoint leftEndPoint1, leftEndPoint2, rightEndPoint1, rightEndPoint2;

    leftTree->getSegment (left, leftEndPoint1, leftEndPoint2);
    rightTree->getSegment (right, rightEndPoint1, rightEndPoint2);

    // Apply transformation to have both positions in the same frame.
    rightEndPoint1 = testData->rightToLeftTransformation () * rightEndPoint1;
    rightEndPoint2 = testData->rightToLeftTransformation () * rightEndPoint2;

    // Compute Distance between the two segments axes.
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

    // depending on the result, we will call one of the 4 report
    // functions of CkcdProximityQuery
    if (squareDistance < std::numeric_limits<kcdReal>::epsilon ()
	* std::numeric_limits<kcdReal>::epsilon ())
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
					       sqrt (squareDistance));
	  }
	else
	  {
	    // if it is a leaf, report an exact distance
	    testAnswer = query.reportExactDistance (left,
						    right,
						    testData,
						    sqrt(squareDistance),
						    leftSegmentClosest,
						    rightSegmentClosest);
	  }
      }

    return testAnswer;
  }

  bool DetectorSegmentSegment::
  canHandle (unsigned int leftID, unsigned int rightID) const
  {
    return ((leftID == TestTreeSegment::segmentDispatchID ()) &&
	    (rightID == TestTreeSegment::segmentDispatchID ()));
  }
  
  ktStatus DetectorSegmentSegment::
  init (const DetectorSegmentSegmentWkPtr& weakPtr)
  {
    ktStatus success = KD_OK;

    success = CkcdDetector::init (weakPtr);

    if (KD_OK == success)
      {
	weakPtr_ = weakPtr;
      }

    return success;
  }

  DetectorSegmentSegment::
  DetectorSegmentSegment ()
    : CkcdDetector ()
  {
  }

  DetectorSegmentSegment::
  DetectorSegmentSegment (const DetectorSegmentSegment& detector)
    : CkcdDetector (detector)
  {
  }

} // end of namespace kcd.
