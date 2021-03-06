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
 * \file src/hpp/geometry/collision/detector-obb-segment.cc
 *
 * \brief Implementation of DetectorOBBSegment.
 */

#include <limits>

#include "hpp/geometry/collision/detector-obb-segment.hh"
#include "hpp/geometry/collision/test-tree-segment.hh"
#include "hpp/geometry/collision/util.hh"

namespace hpp
{
  namespace geometry
  {
    namespace collision
    {
      // this line registers the detector in the global detector dispatcher
      HPP_KCD_REGISTER_DETECTOR(DetectorOBBSegment);

      DetectorOBBSegmentShPtr DetectorOBBSegment::
      create ()
      {
	DetectorOBBSegment* ptr = new DetectorOBBSegment ();
	DetectorOBBSegmentShPtr shPtr (ptr);

	if (KD_OK != ptr->init (shPtr))
	  {
	    shPtr.reset();
	  }

	return shPtr;
      }

      DetectorOBBSegmentShPtr DetectorOBBSegment::
      createCopy (const DetectorOBBSegmentConstShPtr& detector)
      {
	DetectorOBBSegment* ptr = new DetectorOBBSegment (*detector);
	DetectorOBBSegmentShPtr shPtr (ptr);

	if (KD_OK != ptr->init (shPtr))
	  {
	    shPtr.reset ();
	  }

	return shPtr;
      }

      CkcdDetectorShPtr DetectorOBBSegment::
      clone () const
      {
	return DetectorOBBSegment::createCopy (weakPtr_.lock ());
      }

      CkcdDetectorTestAnswer DetectorOBBSegment::
      analyze (const CkcdTreeIterator& left, 
	       const CkcdTreeIterator& right,
	       const CkcdDetectorElementaryTestDataShPtr& testData,
	       CkcdProximityQuery& query) const
      {
	CkcdDetectorTestAnswer testAnswer;

	// Retrieve segment and OBB information.
	const CkcdTestTreePolyBV* leftTree
	  = static_cast<CkcdTestTreePolyBV*> (left.testTree ());
	const TestTreeSegment* rightTree
	  = static_cast<TestTreeSegment*> (right.testTree ());
	CkcdPoint rightEndPoint1, rightEndPoint2;
	kcdReal rightRadius;
	CkcdTestTreeOBB::CkcdPolyOBBCache leftPolyOBBCache;
    
	leftTree->fillOBBCache (left, false, leftPolyOBBCache);
	rightTree->getSegment (right, rightEndPoint1, rightEndPoint2,
			       rightRadius);
    
	// Apply transformation to have both positions in the same frame.
	rightEndPoint1 = testData->rightToLeftTransformation () * rightEndPoint1;
	rightEndPoint2 = testData->rightToLeftTransformation () * rightEndPoint2;

	// Compute Distance between the segments axis and the OBB.
	hppReal squareDistance;

	computeSquareDistanceSegmentBox (rightEndPoint1,
					 rightEndPoint2,
					 leftPolyOBBCache,
					 squareDistance);

	// depending on the result, we will call one of the 4 report
	// functions of CkcdProximityQuery
	if (squareDistance < std::numeric_limits<hppReal>::epsilon ()
	    * std::numeric_limits<hppReal>::epsilon ())
	  {
	    // if it is a leaf, report overlap.
	    testAnswer = query.reportOverlap (left, right, testData);
	  }
	else
	  {
	    // if it is not a leaf, report an estimated distance
	    testAnswer
	      = query.reportEstimatedDistance (left,
					       right,
					       testData,
					       static_cast<kcdReal>
					       (sqrt (squareDistance)));
	  }

	return testAnswer;
      }

      bool DetectorOBBSegment::
      canHandle (unsigned int leftID, unsigned int rightID) const
      {
	return ((leftID == CkcdTestTreeOBB::polyOBBDispatchID ()) &&
		(rightID == TestTreeSegment::segmentDispatchID ()));
      }
  
      ktStatus DetectorOBBSegment::
      init (const DetectorOBBSegmentWkPtr& weakPtr)
      {
	ktStatus success = KD_OK;

	success = CkcdDetector::init (weakPtr);

	if (KD_OK == success)
	  {
	    weakPtr_ = weakPtr;
	  }

	return success;
      }

      DetectorOBBSegment::
      DetectorOBBSegment ()
	: CkcdDetector ()
      {
      }

      DetectorOBBSegment::
      DetectorOBBSegment (const DetectorOBBSegment& detector)
	: CkcdDetector (detector)
      {
      }

    } // end of namespace collision.
  } // end of namespace geometry.
} // end of namespace hpp.
