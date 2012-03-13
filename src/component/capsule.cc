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
 * \file src/hpp/geometry/component/component/capsule.cc
 *
 * \brief Implementation of Capsule.
 */

#include <kcd2/kcdInterface.h>

#include "hpp/geometry/component/util.hh"
#include "hpp/geometry/component/capsule.hh"

namespace hpp
{
  namespace geometry
  {
    namespace component
    {
      double Capsule::
      height () const
      {
	return heightProperty_->value ();
      }

      void Capsule::
      height (const double& height)
      {
	heightProperty_->value (height);
      }

      double Capsule::
      radius () const
      {
	return radiusProperty_->value ();
      }

      void Capsule::
      radius (const double& radius)
      {
	radiusProperty_->value (radius);
      }

      unsigned int Capsule::
      baseVertices () const
      {
	return baseVerticesProperty_->value ();
      }

      void Capsule::
      baseVertices (const unsigned int baseVertices)
      {
	baseVerticesProperty_->value (baseVertices);
      }

      unsigned int Capsule::
      parallels () const
      {
	return parallelsProperty_->value ();
      }

      void Capsule::
      parallels (const unsigned int parallels)
      {
	parallelsProperty_->value (parallels);
      }

      Capsule::
      ~Capsule ()
      {
      }

      CkppComponentShPtr Capsule::
      cloneComponent () const
      {
	CkppComponentShPtr shPtr = CkppComponent::create ("");
	return shPtr;
      }

      bool Capsule::
      isComponentClonable () const
      {
	return false;
      }

      CapsuleShPtr Capsule::
      create (const std::string& name,
	      const double& height,
	      const double& radius,
	      const unsigned int baseVertices,
	      const unsigned int parallels)
      {
	Capsule* ptr = new Capsule ();
	CapsuleShPtr ptrShPtr (ptr);

	if (ptr->init (ptrShPtr, name, height, radius, baseVertices, parallels)
	    != KD_OK)
	  {
	    ptrShPtr.reset ();
	  }

	return ptrShPtr;
      }

      Capsule::
      Capsule ()
	: CkppPolyhedron (),
	  collision::PolyCapsule (),
	  heightProperty_ (),
	  radiusProperty_ (),
	  baseVerticesProperty_ (),
	  parallelsProperty_ ()
      {
      }

      ktStatus Capsule::
      init (const CapsuleWkPtr weakPtr,
	    const std::string& name,
	    const double& height,
	    const double& radius,
	    const unsigned int baseVertices,
	    const unsigned int parallels)
      {
	ktStatus success1 = CkppPolyhedron::init (weakPtr, name);
	ktStatus success2 = collision::PolyCapsule::init (weakPtr);

	if (KD_OK == success1 && KD_OK == success2)
	  {
	    // Set attributes.
	    weakPtr_ = weakPtr;
	    heightProperty_ = CkppDoubleProperty::create ("height",
							  weakPtr.lock (),
							  0,
							  "height",
							  height);
	    radiusProperty_ = CkppDoubleProperty::create ("radius",
							  weakPtr.lock (),
							  0,
							  "radius",
							  radius);
	    baseVerticesProperty_ = CkppIntegerProperty::create ("base vertices",
								 weakPtr.lock (),
								 0,
								 "base vertices",
								 baseVertices);
	    parallelsProperty_ = CkppIntegerProperty::create ("parallels",
							      weakPtr.lock (),
							      0,
							      "parallels",
							      parallels);

	    CkcdPolyhedronDataShPtr polyData = CkcdPolyhedronData::create ();

	    convertCapsuleToPolyhedronData (polyData,
					    height,
					    radius,
					    baseVertices,
					    parallels);

	    this->polyData (polyData);
	  }

	if (KD_OK == success1 && KD_OK == success2)
	  return KD_OK;
	else
	  return KD_ERROR;
      }

      void Capsule::
      fillPropertyVector (std::vector<CkppPropertyShPtr>& propertyVector) const
      {
	CkppPolyhedron::fillPropertyVector (propertyVector);

	propertyVector.push_back (heightProperty_);
	propertyVector.push_back (radiusProperty_);
	propertyVector.push_back (baseVerticesProperty_);
	propertyVector.push_back (parallelsProperty_);
      }

      bool Capsule::
      modifiedProperty (const CkppPropertyShPtr& property)
      {
	if (!CkppPolyhedron::modifiedProperty (property))
	  return false;

	return true;
      }

      void Capsule::
      updateProperty (const CkppPropertyShPtr& property)
      {
	CkppPolyhedron::updateProperty (property);
      }

    } // end of namespace component.
  } // end of namespace geometry.
} // end of namespace hpp.
