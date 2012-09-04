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

      void Capsule::
      setAbsolutePosition (const CkitMat4 &i_matrix)
      {
	polyhedron_->setAbsolutePosition (i_matrix);
	collision::PolyCapsule::setAbsolutePosition (i_matrix);
      }

      void Capsule::
      getAbsolutePosition (CkitMat4 &o_matrix) const
      {
	collision::PolyCapsule::getAbsolutePosition (o_matrix);
      }

      void Capsule::
      getAbsolutePosition (CkcdMat4 &o_matrix) const
      {
	collision::PolyCapsule::getAbsolutePosition (o_matrix);
      }

      void Capsule::
      setRelativePosition (const CkitMat4 &i_matrix)
      {
	polyhedron_->setRelativePosition (i_matrix);
	collision::PolyCapsule::setRelativePosition (i_matrix);
      }

      void Capsule::
      getRelativePosition (CkitMat4 &o_matrix) const
      {
	collision::PolyCapsule::getRelativePosition (o_matrix);
      }

      ktStatus Capsule::
      getBBMatrixOrientation (CkitMat4 &o_matrix) const
      {
	// FIXME?
	std::cout << "FIXME" << std::endl;
	collision::PolyCapsule::getAbsolutePosition (o_matrix);
	return KD_OK;
      }

      ktStatus Capsule::
      getBBHalfLengths (float &x, float &y, float &z) const
      {
	// FIXME?
	std::cout << "FIXME" << std::endl;
	x = radius ();
	y = radius ();
	z = height () / 2 + radius ();

	return KD_OK;
      }

      bool Capsule::
      isCollisionLeaf () const
      {
	// FIXME?
	return true;
      }

      CkitMat4 Capsule::
      computeDefaultFrame () const
      {
	// FIXME.
	CkitMat4 mat;
	collision::PolyCapsule::getAbsolutePosition (mat);
	return mat;
      }

      ktStatus Capsule::
      addPoint (const CkitPoint3 &i_point)
      {
	polyhedron_->addPoint (i_point);
      }

      ktStatus Capsule::
      addTriangle (unsigned int i_p1,
		   unsigned int i_p2,
		   unsigned int i_p3)
      {
	polyhedron_->addTriangle (i_p1, i_p2, i_p3);
      }

      void Capsule::
      addPolygon (const std::vector< int > &i_verticesVector)
      {
	// FIXME.
	std::cout << "FIXME" << std::endl;
      }

      unsigned int Capsule::
      countPoints () const
      {
	return polyhedron_->countPoints ();
      }

      unsigned int Capsule::
      countTriangles () const
      {
	return polyhedron_->countTriangles ();
      }

      void Capsule::
      getTriangle (const unsigned int i_rank,
		   unsigned int &o_p1,
		   unsigned int &o_p2,
		   unsigned int &o_p3) const
      {
	polyhedron_->getTriangle (i_rank, o_p1, o_p2, o_p3);
      }

      void Capsule::
      getPoint (const unsigned int i_rank,
		float &o_x, float &o_y, float &o_z) const
      {
	polyhedron_->getPoint (i_rank, o_x, o_y, o_z);
      }

      void Capsule::
      getPoint (const unsigned int i_rank,
		CkitPoint3 &o_point) const
      {
	polyhedron_->getPoint (i_rank, o_point);
      }

      CkppAssemblyComponentShPtr Capsule::
      explode (const CkitProgressDelegateShPtr &i_delegate)
      {
	// FIXME.
	std::cout << "FIXME" << std::endl;
	CkppAssemblyComponentShPtr assemblyComponent;
	return assemblyComponent;
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
	else
	  {
	    CkcdPoint e1 (- height / 2, 0, 0);
	    CkcdPoint e2 (height / 2, 0, 0);
	    ptrShPtr->addCapsule (e1, e2, radius);
	  }

	return ptrShPtr;
      }

      CapsuleShPtr Capsule::
      create (const std::string& name,
	      const CkitPoint3& endPoint1,
	      const CkitPoint3& endPoint2,
	      const double& radius,
	      const unsigned int baseVertices,
	      const unsigned int parallels)
      {
	double height = endPoint1.distanceFrom (endPoint2);
	CapsuleShPtr shPtr = Capsule::create (name, height, radius,
					      baseVertices, parallels);

	CkcdMat4 transform;
	CkcdPoint kcdEndPoint1 (endPoint1);
	CkcdPoint kcdEndPoint2 (endPoint2);
	convertCapsuleAxisToTransform (transform, kcdEndPoint1, kcdEndPoint2);
	shPtr->setAbsolutePosition (transform);

	return shPtr;
      }

      CapsuleShPtr Capsule::
      create (const std::string& name,
	      const double& height,
	      const double& radius,
	      const CkitMat4& transformation,
	      const unsigned int baseVertices,
	      const unsigned int parallels)
      {
	CapsuleShPtr shPtr = Capsule::create (name, height, radius,
					      baseVertices, parallels);

	shPtr->setAbsolutePosition (transformation);

	return shPtr;
      }

      Capsule::
      Capsule ()
	: CkppPolyhedron (),
	  collision::PolyCapsule (),
	  heightProperty_ (),
	  radiusProperty_ (),
	  baseVerticesProperty_ (),
	  parallelsProperty_ (),
	  polyhedron_ ()
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

	    polyhedron_ = CkcdPolyhedron::create ();

	    polyhedron_->polyData (polyData);
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

      CkppPolyhedronShPtr Capsule::
      createPolyhedronFromPolyExpandingData
      (const CkppPolyExpandingDataShPtr& i_polyExpandingData,
       unsigned int i_offset)
      {
	// FIXME.
	std::cout << "FIXME" << std::endl;
	CapsuleShPtr polyhedron = Capsule::create ("", 1, 1);
	return polyhedron;
      }

    } // end of namespace component.
  } // end of namespace geometry.
} // end of namespace hpp.
