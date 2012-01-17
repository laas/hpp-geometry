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

#include "hpp/geometry/collision/test-tree-segment.hh"

namespace hpp
{
  namespace geometry
  {
    namespace collision
    {
      // this line gets a new unique dispatch ID from CkcdGlobal
      unsigned int TestTreeSegment::segmentDispatchID_ = CkcdGlobal::getNewDispatchID ();

      const char TestTreeSegment::segmentBoundingVolumeID_ = 0;
      const char TestTreeSegment::segmentElementID_ = 1;

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
      countChildren (const CkcdTreeIterator& it) const
      {
	if (it.type () == segmentBoundingVolumeID_)
	  {
	    return 2;
	  }
	else
	  {
	    return 0;
	  }
      }
  
      CkcdTreeIterator TestTreeSegment::
      getChildIterator (const CkcdTreeIterator& it,
			unsigned int rank) const
      {
	KCD_ASSERT(rank == 0 || rank == 1);
	KCD_ASSERT(it.type() == segmentBoundingVolumeID_);

	if (rank == 0)
	  {
	    if (segmentBoundingVolumes_[it.index ()].firstChildIsBoundingVolume_)
	      {
		return CkcdTreeIterator ((CkcdTestTree*) this,
					 (int) segmentBoundingVolumes_[it.index ()]
					 .firstChildIndex_,
					 segmentBoundingVolumeID_);
	      }
	    else
	      {
		return CkcdTreeIterator ((CkcdTestTree*) this,
					 (int) segmentBoundingVolumes_[it.index ()]
					 .firstChildIndex_,
					 segmentElementID_);
	      }
	  }
	else
	  {
	    if (segmentBoundingVolumes_[it.index ()].secondChildIsBoundingVolume_)
	      {
		return CkcdTreeIterator((CkcdTestTree*) this,
					(int) segmentBoundingVolumes_[it.index ()]
					.secondChildIndex_,
					segmentBoundingVolumeID_);
	      }
	    else
	      {
		return CkcdTreeIterator((CkcdTestTree*) this,
					(int) segmentBoundingVolumes_[it.index ()]
					.secondChildIndex_,
					segmentElementID_);
	      }
	  }
      }

      double TestTreeSegment::
      tolerance (const CkcdTreeIterator& it) const
      {
	return 0.;
      }

      kcdReal TestTreeSegment::
      heuristicValue (const CkcdTreeIterator& it) const
      {
	return 0.f;
      }

      CkcdObjectShPtr TestTreeSegment::
      sceneAnchor (const CkcdTreeIterator& it) const
      {
	return collisionEntity ();
      }

      CkcdGeometrySubElementShPtr TestTreeSegment::
      geometrySubElement (const CkcdTreeIterator& it) const
      {
	if (it.type () == segmentBoundingVolumeID_)
	  {
	    KCD_ASSERT(false);
	    return CkcdGeometrySubElementShPtr ();
	  }
	else
	  {
	    CkcdPoint endPoint1;
	    CkcdPoint endPoint2;
	    getSegment (it, endPoint1, endPoint2);
	    SegmentShPtr segment = Segment::create (weakPtr_.lock (),
						    it.index (),
						    endPoint1,
						    endPoint2);
	    return segment;
	  }
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
      dispatchID (const CkcdTreeIterator& it) const
      {
	return segmentDispatchID_;
      }
  
      CkcdTreeIterator TestTreeSegment::
      rootIterator () const
      {
	return CkcdTreeIterator ((CkcdTestTree*) this, (int) 0, segmentBoundingVolumeID_);
      }
  
      bool TestTreeSegment::
      accepts (CkcdGeometryConstShPtr geometry) const
      {
	return KIT_DYNAMIC_PTR_CAST (PolySegment const, geometry);
      }
    
      ktStatus TestTreeSegment::
      integrate (CkitProgressDelegateShPtr progress,
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
      startCollisionEntity (CkitProgressDelegateShPtr progress,
			    bool& canContinue)
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

	segmentBoundingVolumes_.clear();
	segmentBoundingVolumes_.push_back (SegmentBoudingVolume ());
	buildBoundingVolumes (0, 0, indexVector.size (), indexVector);

	canContinue = false;
	return KD_OK;
      }

      ktStatus TestTreeSegment::
      continueCollisionEntity (bool& canContinue)
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
		  CkcdPoint& endPoint2) const
      {
	ktStatus result = KD_ERROR;

	if (it.type () == segmentBoundingVolumeID_)
	  {
	    if (it.index () < (int) segmentBoundingVolumes_.size ())
	      {
		endPoint1 = segmentBoundingVolumes_[it.index ()].endPoint1_;
		endPoint2 = segmentBoundingVolumes_[it.index ()].endPoint2_;
		result = KD_OK;
	      }
	  }
	else
	  {
	    unsigned int polyIndex;
	    unsigned int segmentIndex;
	    if (KD_OK == getSegmentIndexes(it.index (), polyIndex, segmentIndex))
	      {
		polySegments_[polyIndex]
		  ->getSegment (segmentIndex, endPoint1, endPoint2);
		result = KD_OK;
	      }
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
      buildBoundingVolumes (unsigned int index,
			    unsigned int start,
			    unsigned int end,
			    std::vector<std::pair<unsigned int, unsigned int> >&
			    indexVector)
      {
	computeSegmentBV (start,
			  end,
			  indexVector,
			  segmentBoundingVolumes_[index].endPoint1_,
			  segmentBoundingVolumes_[index].endPoint2_);

	unsigned int mediumIndex
	  = sortSegmentBV (start,
			   end,
			   indexVector,
			   (segmentBoundingVolumes_[index].endPoint1_
			    + segmentBoundingVolumes_[index].endPoint2_) / 2);

	if ((mediumIndex - start) == 1)
	  {
	    segmentBoundingVolumes_[index].firstChildIsBoundingVolume_ = false;
	    segmentBoundingVolumes_[index].firstChildIndex_
	      = getSegmentIndex (indexVector[start].first, indexVector[start].second);
	  }
	else
	  {
	    segmentBoundingVolumes_[index].firstChildIsBoundingVolume_ = true;
	    segmentBoundingVolumes_[index].firstChildIndex_
	      = segmentBoundingVolumes_.size ();
	    segmentBoundingVolumes_.push_back (SegmentBoudingVolume ());
	    buildBoundingVolumes (segmentBoundingVolumes_[index].firstChildIndex_,
				  start,
				  mediumIndex,
				  indexVector);
	  }

	if ((end - mediumIndex) == 0)
	  {
	    // special case : only one segment !
	    // the bounding segment will have the same child twice
	    segmentBoundingVolumes_[index].secondChildIsBoundingVolume_ = false;
	    segmentBoundingVolumes_[index].secondChildIndex_
	      = getSegmentIndex (indexVector[start].first, indexVector[start].second);
	  }
	else if ((end - mediumIndex) == 1)
	  {
	    segmentBoundingVolumes_[index].secondChildIsBoundingVolume_ = false;
	    segmentBoundingVolumes_[index].secondChildIndex_
	      = getSegmentIndex (indexVector[mediumIndex].first,
				 indexVector[mediumIndex].second);
	  }
	else
	  {
	    segmentBoundingVolumes_[index].secondChildIsBoundingVolume_ = true;
	    segmentBoundingVolumes_[index].secondChildIndex_
	      = segmentBoundingVolumes_.size();
	    segmentBoundingVolumes_.push_back (SegmentBoudingVolume ());
	    buildBoundingVolumes (segmentBoundingVolumes_[index].secondChildIndex_,
				  mediumIndex,
				  end,
				  indexVector);
	  }
      }
    
      void TestTreeSegment::
      computeSegmentBV (unsigned int start,
			unsigned int end,
			std::vector<std::pair<unsigned int, unsigned int> >&
			indexVector,
			CkcdPoint& endPoint1,
			CkcdPoint& endPoint2)
      {
	endPoint1 = CkcdPoint (0, 0, 0);
	endPoint2 = CkcdPoint (0, 0, 0);
	for (unsigned int i = start; i < end; i++)
	  {
	    endPoint1 += polySegments_[indexVector[i].first]
	      ->getSegmentFirstEndPoint (indexVector[i].second);
	    endPoint2 += polySegments_[indexVector[i].first]
	      ->getSegmentSecondEndPoint (indexVector[i].second);
	  }
	endPoint1 = endPoint1 / (float) (end - start);
	endPoint2 = endPoint2 / (float) (end - start);
	CkcdPoint center = (endPoint1 + endPoint2) / 2;
      }

      unsigned int TestTreeSegment::
      sortSegmentBV (unsigned int start,
		     unsigned int end,
		     std::vector<std::pair<unsigned int, unsigned int> >&
		     indexVector,
		     const CkcdPoint& barycenter)
      {
	// compute main axis (AABB method)
	CkcdPoint center
	  = (polySegments_[indexVector[start].first]
	     ->getSegmentFirstEndPoint (indexVector[start].second)
	     + polySegments_[indexVector[start].first]
	     ->getSegmentSecondEndPoint (indexVector[start].second))
	  / 2;
	kcdReal xMin = center .x();
	kcdReal xMax = center.x ();
	kcdReal yMin = center.y ();
	kcdReal yMax = center.y ();
	kcdReal zMin = center.z ();
	kcdReal zMax = center.z ();
	for (unsigned int i = start + 1; i < end; i++)
	  {
	    center
	      = (polySegments_[indexVector[i].first]
		 ->getSegmentFirstEndPoint (indexVector[i].second)
		 + polySegments_[indexVector[i].first]
		 ->getSegmentSecondEndPoint (indexVector[i].second))
	      / 2;
	    xMin = std::min (xMin, center.x ());
	    xMax = std::max (xMax, center.x ());
	    yMin = std::min (yMin, center.y ());
	    yMax = std::max (yMax, center.y ());
	    zMin = std::min (zMin, center.z ());
	    zMax = std::max (zMax, center.z ());
	  }
	kcdReal xSize = xMax - xMin;
	kcdReal ySize = xMax - xMin;
	kcdReal zSize = xMax - xMin;

	int minIndex = start;
	int maxIndex = end - 1;
	if ((zSize > ySize) && (zSize > xSize))
	  {
	    // sort on z Axis
	    while (maxIndex >= minIndex)
	      {
		center
		  = (polySegments_[indexVector[minIndex].first]
		     ->getSegmentFirstEndPoint (indexVector[minIndex].second)
		     + polySegments_[indexVector[minIndex].first]
		     ->getSegmentSecondEndPoint (indexVector[minIndex].second))
		  / 2;
		if (center.z () < barycenter.z ())
		  {
		    minIndex++;
		  }
		else
		  {
		    //switch min and max
		    switchIndexes (minIndex, maxIndex, indexVector);
		    maxIndex--;
		  }
	      }
	  }
	else if (ySize > xSize)
	  {
	    // sort on y Axis
	    while (maxIndex >= minIndex)
	      {
		center 
		  = (polySegments_[indexVector[minIndex].first]
		     ->getSegmentFirstEndPoint (indexVector[minIndex].second)
		     + polySegments_[indexVector[minIndex].first]
		     ->getSegmentSecondEndPoint (indexVector[minIndex].second))
		  / 2;
		if (center.y () < barycenter.y ())
		  {
		    minIndex++;
		  }
		else
		  {
		    //switch min and max
		    switchIndexes (minIndex, maxIndex, indexVector);
		    maxIndex--;
		  }
	      }
	  }
	else
	  {
	    // sort on x Axis
	    while (maxIndex >= minIndex)
	      {
		center
		  = (polySegments_[indexVector[minIndex].first]
		     ->getSegmentFirstEndPoint (indexVector[minIndex].second)
		     + polySegments_[indexVector[minIndex].first]
		     ->getSegmentSecondEndPoint (indexVector[minIndex].second))
		  / 2;
		if (center.x () < barycenter.x ())
		  {
		    minIndex++;
		  }
		else
		  {
		    //switch min and max
		    switchIndexes (minIndex, maxIndex, indexVector);
		    maxIndex--;
		  }
	      }
	  }

	if (minIndex == 0)
	  {
	    //happens when all centers are equal
	    minIndex++;
	  }

	return minIndex;
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
	return bb;
      }

    } // end of namespace collision.
  } // end of namespace geometry.
} // end of namespace hpp.
