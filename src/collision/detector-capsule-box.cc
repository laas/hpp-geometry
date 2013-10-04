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
 * \file src/hpp/geometry/collision/detector-capsule-box.cc
 *
 * \brief Implementation of DetectorCapsuleBox.
 */

#include "hpp/geometry/collision/detector-capsule-box.hh"
#include "hpp/geometry/collision/test-tree-capsule.hh"
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
      KCD_REGISTER_DETECTOR(DetectorCapsuleBox);
#pragma GCC diagnostic pop

      DetectorCapsuleBoxShPtr DetectorCapsuleBox::
      create ()
      {
	DetectorCapsuleBox* ptr = new DetectorCapsuleBox ();
	DetectorCapsuleBoxShPtr	shPtr (ptr);

	if (KD_OK != ptr->init (shPtr))
	  {
	    shPtr.reset();
	  }

	return shPtr;
      }

      DetectorCapsuleBoxShPtr DetectorCapsuleBox::
      createCopy (const DetectorCapsuleBoxConstShPtr& detector)
      {
	DetectorCapsuleBox* ptr = new DetectorCapsuleBox (*detector);
	DetectorCapsuleBoxShPtr shPtr (ptr);

	if (KD_OK != ptr->init (shPtr))
	  {
	    shPtr.reset ();
	  }

	return shPtr;
      }

      CkcdDetectorShPtr DetectorCapsuleBox::
      clone () const
      {
	return DetectorCapsuleBox::createCopy (weakPtr_.lock ());
      }

      CkcdDetectorTestAnswer DetectorCapsuleBox::
      analyze (const CkcdTreeIterator& left, 
	       const CkcdTreeIterator& right,
	       const CkcdDetectorElementaryTestDataShPtr& testData,
	       CkcdProximityQuery& query) const
      {
	CkcdDetectorTestAnswer testAnswer;

	// Retrieve capsule and bounding box information.
	const TestTreeCapsule* leftTree
	  = static_cast<TestTreeCapsule*> (left.testTree ());
	const CkcdTestTreeBoundingBox* rightTree
	  = static_cast<CkcdTestTreeBoundingBox*> (right.testTree ());
	CkcdPoint leftEndPoint1, leftEndPoint2;
	kcdReal leftRadius;

	leftTree->getCapsule (left, leftEndPoint1, leftEndPoint2, leftRadius);
	CkcdBoundingBoxShPtr rightBoundingBox
	  = CkcdBoundingBox::create (*rightTree->boundingBox (right));

	// Apply transformation to have both positions in the same frame.
	rightBoundingBox->setRelativePosition
	  (testData->rightToLeftTransformation ()
	   * rightBoundingBox->relativePosition ());

	// Compute distance between the capsules axis and the OBB.
	hppReal squareDistance;

	computeSquareDistanceSegmentBox (leftEndPoint1,
					 leftEndPoint2,
					 rightBoundingBox,
					 squareDistance);

	// depending on the result, we will call one of the 4 report
	// functions of CkcdProximityQuery
	if (squareDistance < leftRadius * leftRadius)
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
					       - leftRadius);
	  }

	return testAnswer;
      }

      bool DetectorCapsuleBox::
      canHandle (unsigned int leftID, unsigned int rightID) const
      {
	return ((leftID == TestTreeCapsule::capsuleDispatchID ()) &&
		(rightID == CkcdTestTreeBoundingBox::boundingBoxDispatchID ()));
      }
  
      ktStatus DetectorCapsuleBox::
      init (const DetectorCapsuleBoxWkPtr& weakPtr)
      {
	ktStatus success = KD_OK;

	success = CkcdDetector::init (weakPtr);

	if (KD_OK == success)
	  {
	    weakPtr_ = weakPtr;
	  }

	return success;
      }

      DetectorCapsuleBox::
      DetectorCapsuleBox ()
	: CkcdDetector ()
      {
      }

      DetectorCapsuleBox::
      DetectorCapsuleBox (const DetectorCapsuleBox& detector)
	: CkcdDetector (detector)
      {
      }

    } // end of namespace collision.
  } // end of namespace geometry.
} // end of namespace hpp.
