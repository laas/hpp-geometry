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
 * \file src/hpp/geometry/collision/segment.cc
 *
 * \brief Implementation of Segment.
 */

#include "hpp/geometry/collision/segment.hh"
#include "hpp/geometry/collision/test-tree-segment.hh"

namespace kcd
{
  SegmentShPtr Segment::
  create (const TestTreeSegmentShPtr testTree,
	  unsigned int index,
	  const CkcdPoint& endPoint1,
	  const CkcdPoint& endPoint2)
  {
    Segment* ptr = new Segment (testTree);
    SegmentShPtr ptrShPtr (ptr);

    if (ptr->init (ptrShPtr, index, endPoint1, endPoint2) != KD_OK)
      {
	ptrShPtr.reset ();
      }

    return ptrShPtr;
  }

  Segment::~Segment()
  {
  }

  CkcdGeometryConstShPtr Segment::
  geometry () const
  {
    return testTreeSegment ()->getPolySegment (index_);
  }

  unsigned int Segment::
  index () const
  {
    return index_;
  }

  CkcdPoint Segment::
  endPoint1 () const
  {
    return endPoint1_;
  }

  CkcdPoint Segment::
  endPoint2 () const
  {
    return endPoint2_;
  }

  Segment::
  Segment(TestTreeSegmentShPtr testTree)
    : CkcdGeometrySubElement(testTree),
      index_ (-1)
  {
  }

  ktStatus Segment::
  init (const SegmentWkPtr& weakPtr,
	unsigned int index,
	const CkcdPoint& endPoint1,
	const CkcdPoint& endPoint2)
  {
    ktStatus success = CkcdGeometrySubElement::init (weakPtr);

    if (KD_OK == success)
      {
	index_ = index;
	endPoint1_ = endPoint1;
	endPoint2_ = endPoint2;
	weakPtr_ = weakPtr;
      }
    
    return success;
  }

  TestTreeSegmentShPtr Segment::
  testTreeSegment () const
  {
    if (CkcdGeometrySubElement::testTree ())
      {
  	return KIT_STATIC_PTR_CAST (TestTreeSegment,
				    CkcdGeometrySubElement::testTree ());
      }
    else
      {
  	return TestTreeSegmentShPtr ();
      }
  }
  
} // end of namespace kcd.
