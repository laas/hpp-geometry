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
 * \file src/hpp/geometry/collision/detector-segment-obb.cc
 *
 * \brief Implementation of DetectorSegmentOBB.
 */

#include <limits>

#include "hpp/geometry/collision/detector-segment-obb.hh"
#include "hpp/geometry/collision/test-tree-segment.hh"
#include "hpp/geometry/collision/util.hh"

namespace hpp
{
  namespace geometry
  {
    namespace collision
    {
      // this line registers the detector in the global detector dispatcher
      HPP_KCD_REGISTER_DETECTOR(DetectorSegmentOBB);

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
	kcdReal leftRadius;
	CkcdTestTreeOBB::CkcdPolyOBBCache rightPolyOBBCache;

	leftTree->getSegment (left, leftEndPoint1, leftEndPoint2, leftRadius);
	rightTree->fillOBBCache (right, false, rightPolyOBBCache);

	// Apply transformation to have both positions in the same frame.
	rightPolyOBBCache.m_matrix = testData->rightToLeftTransformation ()
	  * rightPolyOBBCache.m_matrix;

	// Compute Distance between the segments axis and the OBB.
	hppReal squareDistance;

	computeSquareDistanceSegmentBox (leftEndPoint1,
					 leftEndPoint2,
					 rightPolyOBBCache,
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

    } // end of namespace collision.
  } // end of namespace geometry.
} // end of namespace hpp.
