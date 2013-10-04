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
 * \file src/hpp/geometry/collision/detector-box-capsule.cc
 *
 * \brief Implementation of DetectorBoxCapsule.
 */

#include "hpp/geometry/collision/detector-box-capsule.hh"
#include "hpp/geometry/collision/test-tree-capsule.hh"
#include "hpp/geometry/collision/util.hh"

namespace hpp
{
  namespace geometry
  {
    namespace collision
    {
      // this line registers the detector in the global detector dispatcher
      KCD_REGISTER_DETECTOR(DetectorBoxCapsule);

      DetectorBoxCapsuleShPtr DetectorBoxCapsule::
      create ()
      {
	DetectorBoxCapsule* ptr = new DetectorBoxCapsule ();
	DetectorBoxCapsuleShPtr	shPtr (ptr);

	if (KD_OK != ptr->init (shPtr))
	  {
	    shPtr.reset();
	  }

	return shPtr;
      }

      DetectorBoxCapsuleShPtr DetectorBoxCapsule::
      createCopy (const DetectorBoxCapsuleConstShPtr& detector)
      {
	DetectorBoxCapsule* ptr = new DetectorBoxCapsule (*detector);
	DetectorBoxCapsuleShPtr shPtr (ptr);

	if (KD_OK != ptr->init (shPtr))
	  {
	    shPtr.reset ();
	  }

	return shPtr;
      }

      CkcdDetectorShPtr DetectorBoxCapsule::
      clone () const
      {
	return DetectorBoxCapsule::createCopy (weakPtr_.lock ());
      }

      CkcdDetectorTestAnswer DetectorBoxCapsule::
      analyze (const CkcdTreeIterator& left, 
	       const CkcdTreeIterator& right,
	       const CkcdDetectorElementaryTestDataShPtr& testData,
	       CkcdProximityQuery& query) const
      {
	CkcdDetectorTestAnswer testAnswer;

	// Retrieve capsule and bounding box information.
	const CkcdTestTreeBoundingBox* leftTree
	  = static_cast<CkcdTestTreeBoundingBox*> (left.testTree ());
	const TestTreeCapsule* rightTree
	  = static_cast<TestTreeCapsule*> (right.testTree ());
	CkcdPoint rightEndPoint1, rightEndPoint2;
	kcdReal rightRadius;

	CkcdBoundingBoxShPtr leftBoundingBox
	  = CkcdBoundingBox::create (*leftTree->boundingBox (left));
	rightTree->getCapsule (right, rightEndPoint1, rightEndPoint2, rightRadius);

	// Apply transformation to have both positions in the same frame.
	rightEndPoint1 = testData->rightToLeftTransformation () * rightEndPoint1;
	rightEndPoint2 = testData->rightToLeftTransformation () * rightEndPoint2;

	// Compute distance between the capsules axis and the OBB.
	hppReal squareDistance;

	computeSquareDistanceSegmentBox (rightEndPoint1,
					 rightEndPoint2,
					 leftBoundingBox,
					 squareDistance);

	// depending on the result, we will call one of the 4 report
	// functions of CkcdProximityQuery
	if (squareDistance < rightRadius * rightRadius)
	  {
	    // if it is not a leaf, report an overlap (of bounding volumes)
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
					       (sqrt (squareDistance))
					       - rightRadius);
	  }

	return testAnswer;
      }

      bool DetectorBoxCapsule::
      canHandle (unsigned int leftID, unsigned int rightID) const
      {
	return ((leftID == CkcdTestTreeBoundingBox::boundingBoxDispatchID ()) &&
		(rightID == TestTreeCapsule::capsuleDispatchID ()));
      }
  
      ktStatus DetectorBoxCapsule::
      init (const DetectorBoxCapsuleWkPtr& weakPtr)
      {
	ktStatus success = KD_OK;

	success = CkcdDetector::init (weakPtr);

	if (KD_OK == success)
	  {
	    weakPtr_ = weakPtr;
	  }

	return success;
      }

      DetectorBoxCapsule::
      DetectorBoxCapsule ()
	: CkcdDetector ()
      {
      }

      DetectorBoxCapsule::
      DetectorBoxCapsule (const DetectorBoxCapsule& detector)
	: CkcdDetector (detector)
      {
      }

    } // end of namespace collision.
  } // end of namespace geometry.
} // end of namespace hpp.
