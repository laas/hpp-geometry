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
 * \file src/kpp/capsule.cc
 *
 * \brief Implementation of Capsule.
 */

#include "kpp/component/capsule.hh"

namespace kpp
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
      : CkppKCDAssembly ()
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
      ktStatus success = CkppKCDAssembly::init (weakPtr, weakPtr, name);

      if (KD_OK == success)
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

	  // Define geometry components and insert them in assembly.
	  CkppComponentShPtr cylinder = CkppKCDCylinder::create ("cylinder",
								 radius,
								 radius,
								 height,
								 baseVertices,
								 false,
								 false);

	  CkppComponentShPtr topSphere = CkppKCDSphere::create ("top sphere",
								radius,
								baseVertices,
								parallels);

	  CkppComponentShPtr bottomSphere = CkppKCDSphere::create ("top sphere",
								   radius,
								   baseVertices,
								   parallels);
	  insertChildComponent (cylinder, 0);
	  insertChildComponent (topSphere, 1);
	  insertChildComponent (bottomSphere, 2);
	}
    
      return success;
    }

    void Capsule::
    fillPropertyVector (std::vector<CkppPropertyShPtr>& propertyVector) const
    {
      CkppKCDAssembly::fillPropertyVector (propertyVector);
      
      propertyVector.push_back (heightProperty_);
      propertyVector.push_back (radiusProperty_);
      propertyVector.push_back (baseVerticesProperty_);
      propertyVector.push_back (parallelsProperty_);
    }

    bool Capsule::
    modifiedProperty (const CkppPropertyShPtr& property)
    {
      if (!CkppKCDAssembly::modifiedProperty (property))
	return false;

      return true;
    }

    void Capsule::
    updateProperty (const CkppPropertyShPtr& property)
    {
      CkppKCDAssembly::updateProperty (property);
    }

  } // end of namespace component.
} // end of namespace kpp.
