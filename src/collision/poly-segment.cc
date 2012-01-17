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
 * \file src/hpp/geometry/collision/poly-segment.cc
 *
 * \brief Implementation of PolySegment.
 */

#include "hpp/geometry/collision/poly-segment.hh"

namespace kcd
{
  PolySegmentShPtr PolySegment::
  create ()
  {
    PolySegment* ptr = new PolySegment ();
    PolySegmentShPtr shPtr (ptr);

    if (KD_OK != ptr->init (shPtr))
      {
	shPtr.reset();
      }

    return shPtr;
  }

  PolySegment::
  ~PolySegment ()
  {
  }

  unsigned int PolySegment::
  countSubElements () const
  {
    return segmentVector_.size ();
  }

  void PolySegment::
  addSegment (const CkcdPoint& endPoint1,
	      const CkcdPoint& endPoint2)
  {
    segmentVector_.push_back(segment_t (endPoint1, endPoint2));
  }

  ktStatus PolySegment::
  setSegment (unsigned int index,
	      const CkcdPoint& endPoint1,
	      const CkcdPoint& endPoint2)
  {
    ktStatus result = KD_OK;

    if (index < segmentVector_.size())
      {
	segmentVector_[index] = segment_t (endPoint1, endPoint2);
      }
    else if (index == segmentVector_.size())
      {
	segmentVector_.push_back(segment_t (endPoint1, endPoint2));
      }
    else
      {
	result = KD_ERROR;
      }
    return result;
  }

  ktStatus PolySegment::
  getSegment (unsigned int index,
	      CkcdPoint& endPoint1,
	      CkcdPoint& endPoint2) const
  {
    using namespace boost;

    ktStatus result = KD_ERROR;
    if (index < segmentVector_.size ())
      {
	endPoint1 = moveMatrix_ * get<0> (segmentVector_[index]);
	endPoint2 = moveMatrix_ * get<1> (segmentVector_[index]);
	result = KD_OK;
      }
    return result;
  }
  
  CkcdPoint PolySegment::
  getSegmentFirstEndPoint (unsigned int index) const
  {
    using namespace boost;

    KCD_ASSERT(index < segmentVector_.size());
    return moveMatrix_ * get<0> (segmentVector_[index]);
  }
  
  CkcdPoint PolySegment::
  getSegmentSecondEndPoint (unsigned int index) const
  {
    using namespace boost;

    KCD_ASSERT(index < segmentVector_.size());
    return moveMatrix_ * get<1> (segmentVector_[index]);
  }
  
  ktStatus PolySegment::
  init (const PolySegmentWkPtr& weakPtr)
  {
    ktStatus status = CkcdGeometry::init (weakPtr);
    
    if (KD_OK == status)
      {
	weakPtr_ = weakPtr;
      }
    
    return status;
  }

  PolySegment::
  PolySegment ()
  {
    moveMatrix_.identity ();
  }

} // end of namespace kcd.
