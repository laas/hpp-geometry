// Copyright (C) 2011 by Antonio El Khoury.
//
// This file is part of the kcd-segment.
//
// kcd-segment is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// kcd-segment is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with kcd-segment.  If not, see <http://www.gnu.org/licenses/>.


/**
 * \file src/hpp/geometry/collision/detector-segment-triangle.cc
 *
 * \brief Implementation of DetectorSegmentTriangle.
 */

#include <limits>

#include "hpp/geometry/collision/detector-segment-triangle.hh"
#include "hpp/geometry/collision/test-tree-segment.hh"
#include "hpp/geometry/collision/util.hh"

namespace hpp
{
  DetectorSegmentTriangleShPtr DetectorSegmentTriangle::
  create ()
  {
    DetectorSegmentTriangle* ptr = new DetectorSegmentTriangle ();
    DetectorSegmentTriangleShPtr shPtr (ptr);

    if (KD_OK != ptr->init (shPtr))
      {
	shPtr.reset();
      }

    return shPtr;
  }

  DetectorSegmentTriangleShPtr DetectorSegmentTriangle::
  createCopy (const DetectorSegmentTriangleConstShPtr& detector)
  {
    DetectorSegmentTriangle* ptr = new DetectorSegmentTriangle (*detector);
    DetectorSegmentTriangleShPtr shPtr (ptr);

    if (KD_OK != ptr->init (shPtr))
      {
	shPtr.reset ();
      }

    return shPtr;
  }

  CkcdDetectorShPtr DetectorSegmentTriangle::
  clone () const
  {
    return DetectorSegmentTriangle::createCopy (weakPtr_.lock ());
  }

  CkcdDetectorTestAnswer DetectorSegmentTriangle::
  analyze (const CkcdTreeIterator& left, 
	   const CkcdTreeIterator& right,
	   const CkcdDetectorElementaryTestDataShPtr& testData,
	   CkcdProximityQuery& query) const
  {
    CkcdDetectorTestAnswer testAnswer;

    // Retrieve segment and Triangle information.
    const TestTreeSegment* leftTree
      = static_cast<TestTreeSegment*> (left.testTree ());
    const CkcdTestTreePolyBV* rightTree
      = static_cast<CkcdTestTreePolyBV*> (right.testTree ());
    CkcdPoint leftEndPoint1, leftEndPoint2;
    CkcdTestTreeOBB::CkcdTriangleCache<CkcdPoint> rightTriangleCache;

    leftTree->getSegment (left, leftEndPoint1, leftEndPoint2);
    rightTree->fillTriangleCache (right, rightTriangleCache);

    // Apply transformation to have both positions in the same frame.
    rightTriangleCache.m_vertex[0] = testData->rightToLeftTransformation ()
      * rightTriangleCache.m_vertex[0];
    rightTriangleCache.m_vertex[1] = testData->rightToLeftTransformation ()
      * rightTriangleCache.m_vertex[1];
    rightTriangleCache.m_vertex[2] = testData->rightToLeftTransformation ()
      * rightTriangleCache.m_vertex[2];

    // Compute Distance between the segments axis and the triangle.
    kcdReal squareDistance;
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
	if (left.countChildren () > 0)
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
	    CkitVect3 axis = rightTriangleClosest - leftSegmentClosest;
	    axis.normalize ();

	    testAnswer = query.reportExactDistance (left,
	    					    right,
	    					    testData,
	    					    sqrt (squareDistance),
	    					    leftSegmentClosest,
	    					    rightTriangleClosest);
	  }
      }

    return testAnswer;
  }

  bool DetectorSegmentTriangle::
  canHandle (unsigned int leftID, unsigned int rightID) const
  {
    return ((leftID == TestTreeSegment::segmentDispatchID ()) &&
	    (rightID == CkcdTestTreeOBB::triangleDispatchID ()));
  }
  
  ktStatus DetectorSegmentTriangle::
  init (const DetectorSegmentTriangleWkPtr& weakPtr)
  {
    ktStatus success = KD_OK;

    success = CkcdDetector::init (weakPtr);

    if (KD_OK == success)
      {
	weakPtr_ = weakPtr;
      }

    return success;
  }

  DetectorSegmentTriangle::
  DetectorSegmentTriangle ()
    : CkcdDetector ()
  {
  }

  DetectorSegmentTriangle::
  DetectorSegmentTriangle (const DetectorSegmentTriangle& detector)
    : CkcdDetector (detector)
  {
  }

} // end of namespace hpp.
