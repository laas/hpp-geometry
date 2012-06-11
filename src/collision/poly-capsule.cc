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
 * \file src/hpp/geometry/collision/poly-capsule.cc
 *
 * \brief Implementation of PolyCapsule.
 */

#include "hpp/geometry/collision/poly-capsule.hh"

namespace hpp
{
  namespace geometry
  {
    namespace collision
    {
      PolyCapsuleShPtr PolyCapsule::
      create ()
      {
	PolyCapsule* ptr = new PolyCapsule ();
	PolyCapsuleShPtr shPtr (ptr);

	if (KD_OK != ptr->init (shPtr))
	  {
	    shPtr.reset();
	  }

	return shPtr;
      }

      PolyCapsule::
      ~PolyCapsule ()
      {
      }

      unsigned int PolyCapsule::
      countSubElements () const
      {
	return capsuleVector_.size ();
      }

      void PolyCapsule::
      addCapsule (const CkcdPoint& endPoint1,
		  const CkcdPoint& endPoint2,
		  kcdReal radius)
      {
	capsuleVector_.push_back(capsule_t (endPoint1, endPoint2, radius));
      }

      ktStatus PolyCapsule::
      setCapsule (unsigned int index,
		  const CkcdPoint& endPoint1,
		  const CkcdPoint& endPoint2,
		  kcdReal radius)
      {
	ktStatus result = KD_OK;

	if (index < capsuleVector_.size())
	  {
	    capsuleVector_[index] = capsule_t (endPoint1, endPoint2, radius);
	  }
	else if (index == capsuleVector_.size())
	  {
	    capsuleVector_.push_back(capsule_t (endPoint1, endPoint2, radius));
	  }
	else
	  {
	    result = KD_ERROR;
	  }
	return result;
      }

      ktStatus PolyCapsule::
      getCapsule (unsigned int index,
		  CkcdPoint& endPoint1,
		  CkcdPoint& endPoint2,
		  kcdReal& radius) const
      {
	using namespace boost;

	ktStatus result = KD_ERROR;
	if (index < capsuleVector_.size ())
	  {
	    endPoint1 = get<0> (capsuleVector_[index]);
	    endPoint2 = get<1> (capsuleVector_[index]);
	    radius = get<2> (capsuleVector_[index]);
	    result = KD_OK;
	  }
	return result;
      }
  
      CkcdPoint PolyCapsule::
      getCapsuleFirstEndPoint (unsigned int index) const
      {
	using namespace boost;

	KCD_ASSERT(index < capsuleVector_.size());
	return get<0> (capsuleVector_[index]);
      }
  
      CkcdPoint PolyCapsule::
      getCapsuleSecondEndPoint (unsigned int index) const
      {
	using namespace boost;

	KCD_ASSERT(index < capsuleVector_.size());
	return get<1> (capsuleVector_[index]);
      }
  
      kcdReal PolyCapsule::
      getCapsuleRadius (unsigned int index) const
      {
	using namespace boost;
    
	KCD_ASSERT(index < capsuleVector_.size());
	return get<2> (capsuleVector_[index]);
      }
  
      ktStatus PolyCapsule::
      init (const PolyCapsuleWkPtr& weakPtr)
      {
	ktStatus status = CkcdGeometry::init (weakPtr);
    
	if (KD_OK == status)
	  {
	    weakPtr_ = weakPtr;
	  }
    
	return status;
      }

      PolyCapsule::
      PolyCapsule ()
      {
      }

    } // end of namespace collision.
  } // end of namespace geometry.
} // end of namespace hpp.
