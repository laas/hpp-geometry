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
 * \file src/kpp/component/capsule.cc
 *
 * \brief Implementation of Capsule.
 */

#include <kcd2/kcdInterface.h>

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
      : CkppKCDPolyhedron ()
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
      ktStatus success = CkppKCDPolyhedron::init (weakPtr, name);

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

	  // Retrieve points and facets vectors corresponding to a whole sphere.
	  std::vector<CkcdPoint> vertices;
	  std::vector<std::vector<unsigned int> > facets;

	  CkcdPolyhedronFactory::convertSphereToIndexedFaceSet (radius,
								baseVertices,
								parallels,
								vertices,
								facets);

	  unsigned hsPointsRange = (vertices.size () + baseVertices) / 2 - 1;
	  unsigned hsLowerPtId = vertices.size () - 2;

	  reserveNPoints (vertices.size () + baseVertices);

	  // Apply transformation on sphere lower half and add points
	  // to form first cap.
	  CkcdMat4 hsFirstTransform;
	  hsFirstTransform.rotateY (M_PI / 2);
	  hsFirstTransform(0, 3) = - height / 2;

	  for (unsigned i = 0; i < hsPointsRange; ++i)
	    addPoint (hsFirstTransform * vertices[i]);

	  // Apply opposite transformation on sphere lower half and
	  // add points to form second cap.
	  CkcdMat4 hsSecondTransform;
	  hsSecondTransform.rotateY (- M_PI / 2);
	  hsSecondTransform(0, 3) = height / 2;

	  for (unsigned i = 0; i < hsPointsRange; ++i)
	    addPoint (hsSecondTransform * vertices[i]);

	  // Apply transformations on remanining single top and bottom
	  // points of sphere.
	  unsigned hsFirstPtRank;
	  CkcdPolyhedron::addPoint (hsFirstTransform * vertices[hsLowerPtId],
				    hsFirstPtRank);

	  unsigned hsSecondPtRank;
	  CkcdPolyhedron::addPoint (hsSecondTransform * vertices[hsLowerPtId],
	  			    hsSecondPtRank);

	  reserveNTriangles (facets.size () + 2 * baseVertices);

	  // Add facets to form first cap.
	  for (unsigned i = 0; i < facets.size (); ++i)
	    if (facets[i][0] < hsPointsRange
		&& facets[i][1] < hsPointsRange
		&& facets[i][2] < hsPointsRange)
	      addTriangle (facets[i][0], facets[i][1], facets[i][2]);

	  for (unsigned i = 0; i < baseVertices - 1; ++i)
	    addTriangle (i, i + 1, hsFirstPtRank);
	  addTriangle (0, baseVertices - 1, hsFirstPtRank);

	  // Add facets to form cylinder and stitch the two caps together.
	  const unsigned firstEquatorStartId
	    = hsPointsRange - baseVertices;
	  const unsigned secondEquatorFinishId
	    = 2 * hsPointsRange - 1;

	  for (unsigned i = 0; i < baseVertices / 2; ++i)
	    {
	      addTriangle (firstEquatorStartId + i,
			   firstEquatorStartId + i + 1,
			   secondEquatorFinishId - i + 1 - baseVertices / 2);
	      addTriangle (firstEquatorStartId + i + 1,
			   secondEquatorFinishId - i + 1 - baseVertices / 2,
			   secondEquatorFinishId - i - baseVertices / 2);
	    }

	  for (unsigned i = baseVertices / 2; i < baseVertices - 1; ++i)
	    {
	      addTriangle (firstEquatorStartId + i,
			   firstEquatorStartId + i + 1,
			   secondEquatorFinishId - i + baseVertices / 2);
	      addTriangle (firstEquatorStartId + i + 1,
			   secondEquatorFinishId - i + baseVertices / 2,
			   secondEquatorFinishId - i - 1 + baseVertices / 2);
	    }

	  addTriangle (firstEquatorStartId + baseVertices - 1,
		       firstEquatorStartId,
		       secondEquatorFinishId - baseVertices + 1
		       + baseVertices / 2);
	  addTriangle (firstEquatorStartId + baseVertices / 2,
		       secondEquatorFinishId - baseVertices + 1,
		       secondEquatorFinishId);

	  // Add facets to form second cap.
	  for (unsigned i = 0; i < facets.size (); ++i)
	    if ((facets[i][0] >= hsPointsRange
		 && facets[i][0] < 2 * hsPointsRange - baseVertices)
		&& (facets[i][1] >= hsPointsRange
		    && facets[i][1] < 2 * hsPointsRange - baseVertices)
		&& (facets[i][2] >= hsPointsRange
		    && facets[i][2] < 2 * hsPointsRange - baseVertices))
	      addTriangle (facets[i][0], facets[i][1], facets[i][2]);

	  const unsigned secondEquatorStartId
	    = 2 * hsPointsRange - baseVertices;

	  for (unsigned i = 2 * (hsPointsRange - baseVertices);
	       i < secondEquatorStartId - 1;
	       ++i)
	    {
	      addTriangle (i, i + 1, i + baseVertices + 1);
	      addTriangle (i, i + baseVertices, i + baseVertices + 1);
	    }
	  addTriangle (secondEquatorStartId - 1,
		       2 * (hsPointsRange - baseVertices),
		       secondEquatorStartId);
	  addTriangle (secondEquatorStartId - 1,
		       secondEquatorStartId - 1 + baseVertices,
		       secondEquatorStartId);

	  for (unsigned i = hsPointsRange;
	       i < hsPointsRange + baseVertices - 1;
	       ++i)
	    addTriangle (i, i + 1, hsSecondPtRank);
	  addTriangle (hsPointsRange,
		       hsPointsRange + baseVertices - 1,
		       hsSecondPtRank);
	}

      return success;
    }

    void Capsule::
    fillPropertyVector (std::vector<CkppPropertyShPtr>& propertyVector) const
    {
      CkppKCDPolyhedron::fillPropertyVector (propertyVector);

      propertyVector.push_back (heightProperty_);
      propertyVector.push_back (radiusProperty_);
      propertyVector.push_back (baseVerticesProperty_);
      propertyVector.push_back (parallelsProperty_);
    }

    bool Capsule::
    modifiedProperty (const CkppPropertyShPtr& property)
    {
      if (!CkppKCDPolyhedron::modifiedProperty (property))
	return false;

      return true;
    }

    void Capsule::
    updateProperty (const CkppPropertyShPtr& property)
    {
      CkppKCDPolyhedron::updateProperty (property);
    }

  } // end of namespace component.
} // end of namespace kpp.
