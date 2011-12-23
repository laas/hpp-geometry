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
 * \file src/capsule.cc
 *
 * \brief Implementation of Capsule.
 */

#include "kcd/capsule.hh"
#include "kcd/test-tree-capsule.hh"

namespace kcd
{
  CapsuleShPtr Capsule::
  create (const TestTreeCapsuleShPtr testTree,
	  unsigned int index,
	  const CkcdPoint& endPoint1,
	  const CkcdPoint& endPoint2,
	  kcdReal radius)
  {
    Capsule* ptr = new Capsule (testTree);
    CapsuleShPtr ptrShPtr (ptr);

    if (ptr->init (ptrShPtr, index, endPoint1, endPoint2, radius) != KD_OK)
      {
	ptrShPtr.reset ();
      }

    return ptrShPtr;
  }

  Capsule::~Capsule()
  {
  }

  CkcdGeometryConstShPtr Capsule::
  geometry () const
  {
    return testTreeCapsule ()->getPolyCapsule (index_);
  }

  unsigned int Capsule::
  index () const
  {
    return index_;
  }

  CkcdPoint Capsule::
  endPoint1 () const
  {
    return endPoint1_;
  }

  CkcdPoint Capsule::
  endPoint2 () const
  {
    return endPoint2_;
  }

  kcdReal Capsule::
  radius () const
  {
    return radius_;
  }

  Capsule::
  Capsule(TestTreeCapsuleShPtr testTree)
    : CkcdGeometrySubElement(testTree),
      index_ (-1)
  {
  }

  ktStatus Capsule::
  init (const CapsuleWkPtr& weakPtr,
	unsigned int index,
	const CkcdPoint& endPoint1,
	const CkcdPoint& endPoint2,
	kcdReal radius)
  {
    ktStatus success = CkcdGeometrySubElement::init (weakPtr);

    if (KD_OK == success)
      {
	index_ = index;
	endPoint1_ = endPoint1;
	endPoint2_ = endPoint2;
	radius_ = radius;
	weakPtr_ = weakPtr;
      }
    
    return success;
  }

  TestTreeCapsuleShPtr Capsule::
  testTreeCapsule () const
  {
    if (CkcdGeometrySubElement::testTree ())
      {
  	return KIT_STATIC_PTR_CAST (TestTreeCapsule,
				    CkcdGeometrySubElement::testTree ());
      }
    else
      {
  	return TestTreeCapsuleShPtr ();
      }
  }
  
} // end of namespace kcd.
