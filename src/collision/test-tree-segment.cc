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
 * \file src/hpp/geometry/collision/test-tree-segment.cc
 *
 * \brief Implementation of TestTreeSegment.
 */

#include "hpp/geometry/component/util.hh"
#include "hpp/geometry/collision/test-tree-segment.hh"

namespace hpp
{
  namespace geometry
  {
    namespace collision
    {
      // this line registers the test tree in the global locked test tree list
      KCD_REGISTER_TEST_TREE_LOCKED(TestTreeSegment);

      // this line gets a new unique dispatch ID from CkcdGlobal
      unsigned int TestTreeSegment::segmentDispatchID_ = CkcdGlobal::getNewDispatchID ();

      const char TestTreeSegment::segmentElementID_ = 0;

      TestTreeSegmentShPtr TestTreeSegment::
      create (CkcdObjectShPtr collisionEntity)
      {
	TestTreeSegment* ptr = new TestTreeSegment (collisionEntity);
	TestTreeSegmentShPtr shPtr (ptr);

	if (KD_OK != ptr->init (shPtr))
	  {
	    shPtr.reset ();
	  }

	return shPtr;
      }

      TestTreeSegment::~TestTreeSegment ()
      {
      }
  
      unsigned int TestTreeSegment::
      countChildren (const CkcdTreeIterator&) const
      {
	return 0;
      }
  
      CkcdTreeIterator TestTreeSegment::
      getChildIterator (const CkcdTreeIterator&,
			unsigned int) const
      {
	// This should not happen, under the assumption there is only
	// one segment.
	std::cout << "FIXME" << std::endl;
	return CkcdTreeIterator ();
      }

      double TestTreeSegment::
      tolerance (const CkcdTreeIterator&) const
      {
	return 0.;
      }

      kcdReal TestTreeSegment::
      heuristicValue (const CkcdTreeIterator&) const
      {
	return 0.f;
      }

      CkcdObjectShPtr TestTreeSegment::
      sceneAnchor (const CkcdTreeIterator&) const
      {
	return collisionEntity ();
      }

      CkcdGeometrySubElementShPtr TestTreeSegment::
      geometrySubElement (const CkcdTreeIterator& it) const
      {
	CkcdPoint endPoint1;
	CkcdPoint endPoint2;
	kcdReal radius;
	getSegment (it, endPoint1, endPoint2, radius);
	SegmentShPtr segment = Segment::create (weakPtr_.lock (),
						it.index (),
						endPoint1,
						endPoint2,
						radius);
	return segment;
      }

      CkcdTreeIterator TestTreeSegment::
      geometrySubElementIterator (const CkcdGeometrySubElementConstShPtr& geo) const
      {
	SegmentConstShPtr segment = KIT_DYNAMIC_PTR_CAST (const Segment, geo);
	if (segment)
	  {
	    return CkcdTreeIterator ((CkcdTestTree*) this,
				     segment->index (),
				     segmentElementID_);
	  }
	else
	  {
	    KCD_ASSERT (false);
	    return CkcdTreeIterator ();
	  }
      }
  
      unsigned int TestTreeSegment::
      dispatchID (const CkcdTreeIterator&) const
      {
	return segmentDispatchID_;
      }
  
      CkcdTreeIterator TestTreeSegment::
      rootIterator () const
      {
	return CkcdTreeIterator ((CkcdTestTree*) this, (size_t) 0, segmentElementID_);
      }
  
      bool TestTreeSegment::
      accepts (CkcdGeometryConstShPtr geometry) const
      {
	return KIT_DYNAMIC_PTR_CAST (PolySegment const, geometry);
      }
    
      ktStatus TestTreeSegment::
      integrate (CkitProgressDelegateShPtr,
		 CkcdGeometryConstShPtr geometry)
      {
	ktStatus result = KD_ERROR;
	PolySegmentConstShPtr segment
	  = KIT_DYNAMIC_PTR_CAST (PolySegment const, geometry);

	if (segment)
	  {
	    polySegments_.push_back (segment);
	    result = KD_OK;
	  }
	return result;
      }

      ktStatus TestTreeSegment::
      startCollisionEntity (bool& canContinue)
      {
	std::vector<std::pair<unsigned int, unsigned int> > indexVector;
	unsigned int index = 0;

	for (unsigned int iPolySegment = 0;
	     iPolySegment < polySegments_.size ();
	     iPolySegment++)
	  {
	    for (unsigned int iSegment = 0;
		 iSegment < polySegments_[iPolySegment]->countSubElements();
		 iSegment++)
	      {
		indexVector.push_back(std::pair<unsigned int, unsigned int>
				      (iPolySegment, iSegment));
		index++;
	      }
	  }

	canContinue = false;
	return KD_OK;
      }

      ktStatus TestTreeSegment::
      startCollisionEntity (CkitProgressDelegateShPtr,
			    bool& canContinue)
      {
	return startCollisionEntity (canContinue);
      }

      ktStatus TestTreeSegment::
      continueCollisionEntity (bool&)
      {
	return KD_ERROR;
      }

      ktStatus TestTreeSegment::
      finishCollisionEntity (ktStatus status)
      {
	return status;
      }

      bool TestTreeSegment::
      integratedNothing () const
      {
	return (polySegments_.size () == 0);
      }

      ktStatus TestTreeSegment::
      getSegment (const CkcdTreeIterator& it,
		  CkcdPoint& endPoint1,
		  CkcdPoint& endPoint2,
		  kcdReal& radius) const
      {
	ktStatus result = KD_ERROR;

	unsigned int polyIndex;
	unsigned int segmentIndex;
	if (KD_OK == getSegmentIndexes(it.index (), polyIndex, segmentIndex))
	  {
	    polySegments_[polyIndex]
	      ->getSegment (segmentIndex, endPoint1, endPoint2, radius);
	    result = KD_OK;
	  }

	return result;
      }

      PolySegmentConstShPtr TestTreeSegment::
      getPolySegment (unsigned int index) const
      {
	PolySegmentConstShPtr segment;

	for (unsigned int i = 0; i < polySegments_.size (); i++)
	  {
	    if (index < polySegments_[i]->countSubElements ())
	      {
		segment = polySegments_[i];
		break;
	      }
	    else
	      {
		index -= polySegments_[i]->countSubElements ();
	      }
	  }
	return segment;
      }

      TestTreeSegment::
      TestTreeSegment(CkcdObjectShPtr collisionEntity)
	: CkcdTestTreeLocked (collisionEntity)
      {
      }
    
      ktStatus TestTreeSegment::
      init (const TestTreeSegmentWkPtr& weakPtr)
      {
	ktStatus success = CkcdTestTreeLocked::init (weakPtr);

	if(KD_OK == success)
	  {
	    weakPtr_ = weakPtr;
	  }

	return success;
      }
    
      unsigned int TestTreeSegment::
      getSegmentIndex (unsigned int polySegmentIndex,
		       unsigned int indexInPolySegment) const
      {
	unsigned int index = 0;

	KCD_ASSERT (polySegmentIndex < polySegments_.size ());
	KCD_ASSERT(indexInPolySegment 
		   < polySegments_[polySegmentIndex]->countSubElements ());

	for (unsigned int i = 0; i < polySegmentIndex; i++)
	  {
	    index += polySegments_[i]->countSubElements ();
	  }

	return index + indexInPolySegment;
      }

      ktStatus TestTreeSegment::
      getSegmentIndexes (unsigned int index,
			 unsigned int& polySegmentIndex,
			 unsigned int& segmentIndex) const
      {
	ktStatus result = KD_ERROR;
	for (unsigned int i = 0; i < polySegments_.size (); i++)
	  {
	    if (index < polySegments_[i]->countSubElements ())
	      {
		polySegmentIndex = i;
		segmentIndex = index;
		result = KD_OK;
		break;
	      }
	    else
	      {
		index -= polySegments_[i]->countSubElements ();
	      }
	  }

	return result;
      }
    
      void TestTreeSegment::
      switchIndexes (unsigned int index1,
		     unsigned int index2,
		     std::vector<std::pair<unsigned int, unsigned int> >&
		     indexVector)
      {
	unsigned int tmpPolySegmentIndex = indexVector[index1].first;
	unsigned int tmpSegmentIndex = indexVector[index1].second;
	indexVector[index1].first = indexVector[index2].first;
	indexVector[index1].second = indexVector[index2].second;
	indexVector[index2].first = tmpPolySegmentIndex;
	indexVector[index2].second = tmpSegmentIndex;
      }

      CkcdBoundingBoxShPtr TestTreeSegment::
      computeBoundingBox () const
      {
	CkcdBoundingBoxShPtr bb = CkcdBoundingBox::create ();
	CkcdPoint endPoint1, endPoint2;
	kcdReal radius;
	getSegment (rootIterator (), endPoint1, endPoint2, radius);
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
