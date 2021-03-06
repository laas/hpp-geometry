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
 * \file src/hpp/geometry/collision/poly-segment.cc
 *
 * \brief Implementation of PolySegment.
 */

#include "hpp/geometry/collision/poly-segment.hh"

namespace hpp
{
  namespace geometry
  {
    namespace collision
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
		  const CkcdPoint& endPoint2,
		  const kcdReal& radius)
      {
	segmentVector_.push_back(segment_t (endPoint1, endPoint2, radius));
      }

      ktStatus PolySegment::
      setSegment (unsigned int index,
		  const CkcdPoint& endPoint1,
		  const CkcdPoint& endPoint2,
		  const kcdReal& radius)
      {
	ktStatus result = KD_OK;

	if (index < segmentVector_.size())
	  {
	    segmentVector_[index] = segment_t (endPoint1, endPoint2, radius);
	  }
	else if (index == segmentVector_.size())
	  {
	    segmentVector_.push_back(segment_t (endPoint1, endPoint2, radius));
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
		  CkcdPoint& endPoint2,
		  kcdReal& radius) const
      {
	using namespace boost;

	ktStatus result = KD_ERROR;
	if (index < segmentVector_.size ())
	  {
	    endPoint1 = get<0> (segmentVector_[index]);
	    endPoint2 = get<1> (segmentVector_[index]);
	    radius = get<2> (segmentVector_[index]);
	    result = KD_OK;
	  }
	return result;
      }
  
      CkcdPoint PolySegment::
      getSegmentFirstEndPoint (unsigned int index) const
      {
	using namespace boost;

	KCD_ASSERT(index < segmentVector_.size());
	return get<0> (segmentVector_[index]);
      }
  
      CkcdPoint PolySegment::
      getSegmentSecondEndPoint (unsigned int index) const
      {
	using namespace boost;

	KCD_ASSERT(index < segmentVector_.size());
	return get<1> (segmentVector_[index]);
      }

      kcdReal PolySegment::
      getSegmentRadius (unsigned int index) const
      {
	using namespace boost;

	KCD_ASSERT(index < segmentVector_.size());
	return get<2> (segmentVector_[index]);
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
      }

    } // end of namespace collision.
  } // end of namespace geometry.
} // end of namespace hpp.
