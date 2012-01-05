// Copyright (C) 2011 by Antonio El Khoury.
//
// This file is part of the kcd-capsule.
//
// kcd-capsule is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// kcd-capsule is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with kcd-capsule.  If not, see <http://www.gnu.org/licenses/>.


/**
 * \file src/kcd/poly-capsule.cc
 *
 * \brief Implementation of PolyCapsule.
 */

#include "kcd/poly-capsule.hh"

namespace kcd
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
	endPoint1 = moveMatrix_ * get<0> (capsuleVector_[index]);
	endPoint2 = moveMatrix_ * get<1> (capsuleVector_[index]);
	radius = radiusScale_ * get<2> (capsuleVector_[index]);
	result = KD_OK;
      }
    return result;
  }
  
  CkcdPoint PolyCapsule::
  getCapsuleFirstEndPoint (unsigned int index) const
  {
    using namespace boost;

    KCD_ASSERT(index < capsuleVector_.size());
    return moveMatrix_ * get<0> (capsuleVector_[index]);
  }
  
  CkcdPoint PolyCapsule::
  getCapsuleSecondEndPoint (unsigned int index) const
  {
    using namespace boost;

    KCD_ASSERT(index < capsuleVector_.size());
    return moveMatrix_ * get<1> (capsuleVector_[index]);
  }
  
  kcdReal PolyCapsule::
  getCapsuleRadius (unsigned int index) const
  {
    using namespace boost;
    
    KCD_ASSERT(index < capsuleVector_.size());
    return radiusScale_ * get<2> (capsuleVector_[index]);
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
    moveMatrix_.identity ();
    radiusScale_ = 1.;
  }

} // end of namespace kcd.