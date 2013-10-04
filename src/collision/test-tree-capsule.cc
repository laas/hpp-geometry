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
 * \file src/hpp/geometry/collision/test-tree-capsule.cc
 *
 * \brief Implementation of TestTreeCapsule.
 */

#include "hpp/geometry/component/util.hh"
#include "hpp/geometry/collision/test-tree-capsule.hh"

namespace hpp
{
  namespace geometry
  {
    namespace collision
    {
      // this line registers the test tree in the global locked test tree list
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-pedantic"
      KCD_REGISTER_TEST_TREE_LOCKED(TestTreeCapsule);
# pragma GCC diagnostic pop

      // this line gets a new unique dispatch ID from CkcdGlobal
      unsigned int TestTreeCapsule::capsuleDispatchID_ = CkcdGlobal::getNewDispatchID ();

      const char TestTreeCapsule::capsuleElementID_ = 0;

      TestTreeCapsuleShPtr TestTreeCapsule::
      create (CkcdObjectShPtr collisionEntity)
      {
	TestTreeCapsule* ptr = new TestTreeCapsule (collisionEntity);
	TestTreeCapsuleShPtr shPtr (ptr);

	if (KD_OK != ptr->init (shPtr))
	  {
	    shPtr.reset ();
	  }

	return shPtr;
      }

      TestTreeCapsule::~TestTreeCapsule ()
      {
      }
  
      unsigned int TestTreeCapsule::
      countChildren (const CkcdTreeIterator&) const
      {
	return 0;
      }
  
      CkcdTreeIterator TestTreeCapsule::
      getChildIterator (const CkcdTreeIterator&,
			unsigned int) const
      {
	// This should not happen, under the assumption there is only
	// one segment.
	std::cout << "FIXME" << std::endl;
	return CkcdTreeIterator ();
      }

      double TestTreeCapsule::
      tolerance (const CkcdTreeIterator&) const
      {
	return 0.;
      }

      kcdReal TestTreeCapsule::
      heuristicValue (const CkcdTreeIterator& it) const
      {
	unsigned int polyIndex, capsuleIndex;
	getCapsuleIndexes (it.index (), polyIndex, capsuleIndex);
	return polyCapsules_[polyIndex]->getCapsuleRadius (capsuleIndex);
      }

      CkcdObjectShPtr TestTreeCapsule::
      sceneAnchor (const CkcdTreeIterator&) const
      {
	return collisionEntity ();
      }

      CkcdGeometrySubElementShPtr TestTreeCapsule::
      geometrySubElement (const CkcdTreeIterator& it) const
      {
	CkcdPoint endPoint1;
	CkcdPoint endPoint2;
	kcdReal radius;
	getCapsule (it, endPoint1, endPoint2, radius);
	CapsuleShPtr capsule = Capsule::create (weakPtr_.lock (),
						it.index (),
						endPoint1,
						endPoint2,
						radius);
	return capsule;
      }

      CkcdTreeIterator TestTreeCapsule::
      geometrySubElementIterator (const CkcdGeometrySubElementConstShPtr& geo) const
      {
	CapsuleConstShPtr capsule = KIT_DYNAMIC_PTR_CAST (const Capsule, geo);
	if (capsule)
	  {
	    return CkcdTreeIterator ((CkcdTestTree*) this,
				     capsule->index (),
				     capsuleElementID_);
	  }
	else
	  {
	    KCD_ASSERT (false);
	    return CkcdTreeIterator ();
	  }
      }
  
      unsigned int TestTreeCapsule::
      dispatchID (const CkcdTreeIterator&) const
      {
	return capsuleDispatchID_;
      }
  
      CkcdTreeIterator TestTreeCapsule::
      rootIterator () const
      {
	return CkcdTreeIterator ((CkcdTestTree*) this, (size_t) 0, capsuleElementID_);
      }
  
      bool TestTreeCapsule::
      accepts (CkcdGeometryConstShPtr geometry) const
      {
	return KIT_DYNAMIC_PTR_CAST (PolyCapsule const, geometry);
      }
    
      ktStatus TestTreeCapsule::
      integrate (CkitProgressDelegateShPtr,
		 CkcdGeometryConstShPtr geometry)
      {
	ktStatus result = KD_ERROR;
	PolyCapsuleConstShPtr capsule
	  = KIT_DYNAMIC_PTR_CAST (PolyCapsule const, geometry);

	if (capsule)
	  {
	    polyCapsules_.push_back (capsule);
	    result = KD_OK;
	  }
	return result;
      }

      ktStatus TestTreeCapsule::
      startCollisionEntity (CkitProgressDelegateShPtr,
			    bool& canContinue)
      {
	return startCollisionEntity (canContinue);
      }

      ktStatus TestTreeCapsule::startCollisionEntity(bool& canContinue)
      {
	std::vector<std::pair<unsigned int, unsigned int> > indexVector;
	unsigned int index = 0;

	for (unsigned int iPolyCapsule = 0;
	     iPolyCapsule < polyCapsules_.size ();
	     iPolyCapsule++)
	  {
	    for (unsigned int iCapsule = 0;
		 iCapsule < polyCapsules_[iPolyCapsule]->countSubElements();
		 iCapsule++)
	      {
		indexVector.push_back(std::pair<unsigned int, unsigned int>
				      (iPolyCapsule, iCapsule));
		index++;
	      }
	  }

	canContinue = false;
	return KD_OK;
      }

      ktStatus TestTreeCapsule::
      continueCollisionEntity (bool&)
      {
	return KD_ERROR;
      }

      ktStatus TestTreeCapsule::
      finishCollisionEntity (ktStatus status)
      {
	return status;
      }

      bool TestTreeCapsule::
      integratedNothing () const
      {
	return (polyCapsules_.size () == 0);
      }

      ktStatus TestTreeCapsule::
      getCapsule (const CkcdTreeIterator& it,
		  CkcdPoint& endPoint1,
		  CkcdPoint& endPoint2,
		  kcdReal& radius) const
      {
	ktStatus result = KD_ERROR;

	unsigned int polyIndex;
	unsigned int capsuleIndex;
	if (KD_OK == getCapsuleIndexes(it.index (), polyIndex, capsuleIndex))
	  {
	    polyCapsules_[polyIndex]
	      ->getCapsule (capsuleIndex, endPoint1, endPoint2, radius);
	    result = KD_OK;
	  }

	return result;
      }

      PolyCapsuleConstShPtr TestTreeCapsule::
      getPolyCapsule (unsigned int index) const
      {
	PolyCapsuleConstShPtr capsule;

	for (unsigned int i = 0; i < polyCapsules_.size (); i++)
	  {
	    if (index < polyCapsules_[i]->countSubElements ())
	      {
		capsule = polyCapsules_[i];
		break;
	      }
	    else
	      {
		index -= polyCapsules_[i]->countSubElements ();
	      }
	  }
	return capsule;
      }

      TestTreeCapsule::
      TestTreeCapsule(CkcdObjectShPtr collisionEntity)
	: CkcdTestTreeLocked (collisionEntity)
      {
      }
    
      ktStatus TestTreeCapsule::
      init (const TestTreeCapsuleWkPtr& weakPtr)
      {
	ktStatus success = CkcdTestTreeLocked::init (weakPtr);

	if(KD_OK == success)
	  {
	    weakPtr_ = weakPtr;
	  }

	return success;
      }
    
      unsigned int TestTreeCapsule::
      getCapsuleIndex (unsigned int polyCapsuleIndex,
		       unsigned int indexInPolyCapsule) const
      {
	unsigned int index = 0;

	KCD_ASSERT (polyCapsuleIndex < polyCapsules_.size ());
	KCD_ASSERT(indexInPolyCapsule 
		   < polyCapsules_[polyCapsuleIndex]->countSubElements ());

	for (unsigned int i = 0; i < polyCapsuleIndex; i++)
	  {
	    index += polyCapsules_[i]->countSubElements ();
	  }

	return index + indexInPolyCapsule;
      }

      ktStatus TestTreeCapsule::
      getCapsuleIndexes (unsigned int index,
			 unsigned int& polyCapsuleIndex,
			 unsigned int& capsuleIndex) const
      {
	ktStatus result = KD_ERROR;
	for (unsigned int i = 0; i < polyCapsules_.size (); i++)
	  {
	    if (index < polyCapsules_[i]->countSubElements ())
	      {
		polyCapsuleIndex = i;
		capsuleIndex = index;
		result = KD_OK;
		break;
	      }
	    else
	      {
		index -= polyCapsules_[i]->countSubElements ();
	      }
	  }

	return result;
      }
    
      void TestTreeCapsule::
      switchIndexes (unsigned int index1,
		     unsigned int index2,
		     std::vector<std::pair<unsigned int, unsigned int> >&
		     indexVector)
      {
	unsigned int tmpPolyCapsuleIndex = indexVector[index1].first;
	unsigned int tmpCapsuleIndex = indexVector[index1].second;
	indexVector[index1].first = indexVector[index2].first;
	indexVector[index1].second = indexVector[index2].second;
	indexVector[index2].first = tmpPolyCapsuleIndex;
	indexVector[index2].second = tmpCapsuleIndex;
      }

      CkcdBoundingBoxShPtr TestTreeCapsule::
      computeBoundingBox () const
      {
	CkcdBoundingBoxShPtr bb = CkcdBoundingBox::create ();
	CkcdPoint endPoint1, endPoint2;
	kcdReal radius;
	getCapsule (rootIterator (), endPoint1, endPoint2, radius);
	CkcdPoint axis = endPoint2 - endPoint1;
	CkcdMat4 position;
	component::convertCapsuleAxisToTransform (position,
						  endPoint1, endPoint2);
	bb->setRelativePosition (position);
	bb->setHalfLengths (axis.norm () / 2 + radius, radius, radius);
	return bb;
      }

    } // end of namespace collision.
  } // end of namespace geometry.
} // end of namespace hpp.
