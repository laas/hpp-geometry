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
 * \file src/hpp/geometry/collision/detector-capsule-triangle.cc
 *
 * \brief Implementation of DetectorCapsuleTriangle.
 */

#include "hpp/geometry/collision/detector-capsule-triangle.hh"
#include "hpp/geometry/collision/test-tree-capsule.hh"
#include "hpp/geometry/collision/util.hh"

namespace hpp
{
  namespace geometry
  {
    namespace collision
    {
      // this line registers the detector in the global detector dispatcher
      KCD_REGISTER_DETECTOR(DetectorCapsuleTriangle);

      DetectorCapsuleTriangleShPtr DetectorCapsuleTriangle::
      create ()
      {
	DetectorCapsuleTriangle* ptr = new DetectorCapsuleTriangle ();
	DetectorCapsuleTriangleShPtr shPtr (ptr);

	if (KD_OK != ptr->init (shPtr))
	  {
	    shPtr.reset();
	  }

	return shPtr;
      }

      DetectorCapsuleTriangleShPtr DetectorCapsuleTriangle::
      createCopy (const DetectorCapsuleTriangleConstShPtr& detector)
      {
	DetectorCapsuleTriangle* ptr = new DetectorCapsuleTriangle (*detector);
	DetectorCapsuleTriangleShPtr shPtr (ptr);

	if (KD_OK != ptr->init (shPtr))
	  {
	    shPtr.reset ();
	  }

	return shPtr;
      }

      CkcdDetectorShPtr DetectorCapsuleTriangle::
      clone () const
      {
	return DetectorCapsuleTriangle::createCopy (weakPtr_.lock ());
      }

      CkcdDetectorTestAnswer DetectorCapsuleTriangle::
      analyze (const CkcdTreeIterator& left, 
	       const CkcdTreeIterator& right,
	       const CkcdDetectorElementaryTestDataShPtr& testData,
	       CkcdProximityQuery& query) const
      {
	CkcdDetectorTestAnswer testAnswer;

	// Retrieve capsule and Triangle information.
	const TestTreeCapsule* leftTree
	  = static_cast<TestTreeCapsule*> (left.testTree ());
	const CkcdTestTreePolyBV* rightTree
	  = static_cast<CkcdTestTreePolyBV*> (right.testTree ());
	CkcdPoint leftEndPoint1, leftEndPoint2;
	kcdReal leftRadius;
	CkcdTestTreeOBB::CkcdTriangleCache<CkcdPoint> rightTriangleCache;

	leftTree->getCapsule (left, leftEndPoint1, leftEndPoint2, leftRadius);
	rightTree->fillTriangleCache (right, rightTriangleCache);

	// Apply transformation to have both positions in the same frame.
	rightTriangleCache.m_vertex[0] = testData->rightToLeftTransformation ()
	  * rightTriangleCache.m_vertex[0];
	rightTriangleCache.m_vertex[1] = testData->rightToLeftTransformation ()
	  * rightTriangleCache.m_vertex[1];
	rightTriangleCache.m_vertex[2] = testData->rightToLeftTransformation ()
	  * rightTriangleCache.m_vertex[2];

	// Compute Distance between the capsules axis and the triangle.
	hppReal squareDistance;
	CkcdPoint leftSegmentClosest;
	CkcdPoint rightTriangleClosest;

	computeSquareDistanceSegmentTriangle (leftEndPoint1,
					      leftEndPoint2,
					      rightTriangleCache,
					      squareDistance,
					      leftSegmentClosest,
					      rightTriangleClosest);

	// depending on the result, we will call one of the 4 report
	// functions of CkcdProximityQuery
	if (squareDistance < leftRadius * leftRadius)
	  {
	    // if it is a leaf, report a collision.
	    testAnswer = query.reportCollision (left, right, testData);
	  }
	else
	  {
	    // if it is a leaf, report an exact distance
	    CkitVect3 axis = rightTriangleClosest - leftSegmentClosest;
	    axis.normalize ();
	    CkcdPoint leftCapsuleClosest = leftSegmentClosest
	      + axis * leftRadius;

	    testAnswer = query.reportExactDistance (left,
						    right,
						    testData,
						    sqrt (squareDistance)
						    - leftRadius,
						    leftCapsuleClosest,
						    rightTriangleClosest);
	  }

	return testAnswer;
      }

      bool DetectorCapsuleTriangle::
      canHandle (unsigned int leftID, unsigned int rightID) const
      {
	return ((leftID == TestTreeCapsule::capsuleDispatchID ()) &&
		(rightID == CkcdTestTreeOBB::triangleDispatchID ()));
      }
  
      ktStatus DetectorCapsuleTriangle::
      init (const DetectorCapsuleTriangleWkPtr& weakPtr)
      {
	ktStatus success = KD_OK;

	success = CkcdDetector::init (weakPtr);

	if (KD_OK == success)
	  {
	    weakPtr_ = weakPtr;
	  }

	return success;
      }

      DetectorCapsuleTriangle::
      DetectorCapsuleTriangle ()
	: CkcdDetector ()
      {
      }

      DetectorCapsuleTriangle::
      DetectorCapsuleTriangle (const DetectorCapsuleTriangle& detector)
	: CkcdDetector (detector)
      {
      }

    } // end of namespace collision.
  } // end of namespace geometry.
} // end of namespace hpp.
