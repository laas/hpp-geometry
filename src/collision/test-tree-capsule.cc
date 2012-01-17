// Copyright (C) 2011 by Antonio El Khoury.
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

#include "hpp/geometry/collision/test-tree-capsule.hh"

namespace hpp
{
  // this line gets a new unique dispatch ID from CkcdGlobal
  unsigned int TestTreeCapsule::capsuleDispatchID_ = CkcdGlobal::getNewDispatchID ();

  const char TestTreeCapsule::capsuleBoundingVolumeID_ = 0;
  const char TestTreeCapsule::capsuleElementID_ = 1;

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
  countChildren (const CkcdTreeIterator& it) const
  {
    if (it.type () == capsuleBoundingVolumeID_)
      {
	return 2;
      }
    else
      {
	return 0;
      }
  }
  
  CkcdTreeIterator TestTreeCapsule::
  getChildIterator (const CkcdTreeIterator& it,
		    unsigned int rank) const
  {
    KCD_ASSERT(rank == 0 || rank == 1);
    KCD_ASSERT(it.type() == capsuleBoundingVolumeID_);

    if (rank == 0)
      {
	if (capsuleBoundingVolumes_[it.index ()].firstChildIsBoundingVolume_)
	  {
	    return CkcdTreeIterator ((CkcdTestTree*) this,
				     (int) capsuleBoundingVolumes_[it.index ()]
				     .firstChildIndex_,
				     capsuleBoundingVolumeID_);
	  }
	else
	  {
	    return CkcdTreeIterator ((CkcdTestTree*) this,
				     (int) capsuleBoundingVolumes_[it.index ()]
				     .firstChildIndex_,
				     capsuleElementID_);
	  }
      }
    else
      {
	if (capsuleBoundingVolumes_[it.index ()].secondChildIsBoundingVolume_)
	  {
	    return CkcdTreeIterator((CkcdTestTree*) this,
				    (int) capsuleBoundingVolumes_[it.index ()]
				    .secondChildIndex_,
				    capsuleBoundingVolumeID_);
	  }
	else
	  {
	    return CkcdTreeIterator((CkcdTestTree*) this,
				    (int) capsuleBoundingVolumes_[it.index ()]
				    .secondChildIndex_,
				    capsuleElementID_);
	  }
      }
  }

  double TestTreeCapsule::
  tolerance (const CkcdTreeIterator& it) const
  {
    return 0.;
  }

  kcdReal TestTreeCapsule::
  heuristicValue (const CkcdTreeIterator& it) const
  {
    if (it.type() == capsuleBoundingVolumeID_)
      {
	return capsuleBoundingVolumes_[it.index ()].radius_;
      }
    else
      {
	unsigned int polyIndex, capsuleIndex;
	getCapsuleIndexes (it.index (), polyIndex, capsuleIndex);
	return polyCapsules_[polyIndex]->getCapsuleRadius (capsuleIndex);
      }
  }

  CkcdObjectShPtr TestTreeCapsule::
  sceneAnchor (const CkcdTreeIterator& it) const
  {
    return collisionEntity ();
  }

  CkcdGeometrySubElementShPtr TestTreeCapsule::
  geometrySubElement (const CkcdTreeIterator& it) const
  {
    if (it.type () == capsuleBoundingVolumeID_)
      {
	KCD_ASSERT(false);
	return CkcdGeometrySubElementShPtr ();
      }
    else
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
  dispatchID (const CkcdTreeIterator& it) const
  {
    return capsuleDispatchID_;
  }
  
  CkcdTreeIterator TestTreeCapsule::
  rootIterator () const
  {
    return CkcdTreeIterator ((CkcdTestTree*) this, (int) 0, capsuleBoundingVolumeID_);
  }
  
  bool TestTreeCapsule::
  accepts (CkcdGeometryConstShPtr geometry) const
  {
    return KIT_DYNAMIC_PTR_CAST (PolyCapsule const, geometry);
  }
    
  ktStatus TestTreeCapsule::
  integrate (CkitProgressDelegateShPtr progress,
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
  startCollisionEntity (CkitProgressDelegateShPtr progress,
			bool& canContinue)
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

    capsuleBoundingVolumes_.clear();
    capsuleBoundingVolumes_.push_back (CapsuleBoudingVolume ());
    buildBoundingVolumes (0, 0, indexVector.size (), indexVector);

    canContinue = false;
    return KD_OK;
  }

  ktStatus TestTreeCapsule::
  continueCollisionEntity (bool& canContinue)
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

    if (it.type () == capsuleBoundingVolumeID_)
      {
	if (it.index () < (int) capsuleBoundingVolumes_.size ())
	  {
	    endPoint1 = capsuleBoundingVolumes_[it.index ()].endPoint1_;
	    endPoint2 = capsuleBoundingVolumes_[it.index ()].endPoint2_;
	    radius = capsuleBoundingVolumes_[it.index ()].radius_;
	    result = KD_OK;
	  }
      }
    else
      {
	unsigned int polyIndex;
	unsigned int capsuleIndex;
	if (KD_OK == getCapsuleIndexes(it.index (), polyIndex, capsuleIndex))
	  {
	    polyCapsules_[polyIndex]
	      ->getCapsule (capsuleIndex, endPoint1, endPoint2, radius);
	    result = KD_OK;
	  }
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
  buildBoundingVolumes (unsigned int index,
			unsigned int start,
			unsigned int end,
			std::vector<std::pair<unsigned int, unsigned int> >&
			indexVector)
  {
    computeCapsuleBV (start,
		      end,
		      indexVector,
		      capsuleBoundingVolumes_[index].endPoint1_,
		      capsuleBoundingVolumes_[index].endPoint2_,
		      capsuleBoundingVolumes_[index].radius_);

    unsigned int mediumIndex
      = sortCapsuleBV (start,
		       end,
		       indexVector,
		       (capsuleBoundingVolumes_[index].endPoint1_
			+ capsuleBoundingVolumes_[index].endPoint2_) / 2);

    if ((mediumIndex - start) == 1)
      {
	capsuleBoundingVolumes_[index].firstChildIsBoundingVolume_ = false;
	capsuleBoundingVolumes_[index].firstChildIndex_
	  = getCapsuleIndex (indexVector[start].first, indexVector[start].second);
      }
    else
      {
	capsuleBoundingVolumes_[index].firstChildIsBoundingVolume_ = true;
	capsuleBoundingVolumes_[index].firstChildIndex_
	  = capsuleBoundingVolumes_.size ();
	capsuleBoundingVolumes_.push_back (CapsuleBoudingVolume ());
	buildBoundingVolumes (capsuleBoundingVolumes_[index].firstChildIndex_,
			      start,
			      mediumIndex,
			      indexVector);
      }

    if ((end - mediumIndex) == 0)
      {
	// special case : only one capsule !
	// the bounding capsule will have the same child twice
	capsuleBoundingVolumes_[index].secondChildIsBoundingVolume_ = false;
	capsuleBoundingVolumes_[index].secondChildIndex_
	  = getCapsuleIndex (indexVector[start].first, indexVector[start].second);
      }
    else if ((end - mediumIndex) == 1)
      {
	capsuleBoundingVolumes_[index].secondChildIsBoundingVolume_ = false;
	capsuleBoundingVolumes_[index].secondChildIndex_
	  = getCapsuleIndex (indexVector[mediumIndex].first,
			     indexVector[mediumIndex].second);
      }
    else
      {
	capsuleBoundingVolumes_[index].secondChildIsBoundingVolume_ = true;
	capsuleBoundingVolumes_[index].secondChildIndex_
	  = capsuleBoundingVolumes_.size();
	capsuleBoundingVolumes_.push_back (CapsuleBoudingVolume ());
	buildBoundingVolumes (capsuleBoundingVolumes_[index].secondChildIndex_,
			      mediumIndex,
			      end,
			      indexVector);
      }
  }
    
  void TestTreeCapsule::
  computeCapsuleBV (unsigned int start,
		    unsigned int end,
		    std::vector<std::pair<unsigned int, unsigned int> >&
		    indexVector,
		    CkcdPoint& endPoint1,
		    CkcdPoint& endPoint2,
		    kcdReal& radius)
  {
    endPoint1 = CkcdPoint (0, 0, 0);
    endPoint2 = CkcdPoint (0, 0, 0);
    for (unsigned int i = start; i < end; i++)
      {
	endPoint1 += polyCapsules_[indexVector[i].first]
	  ->getCapsuleFirstEndPoint (indexVector[i].second);
	endPoint2 += polyCapsules_[indexVector[i].first]
	  ->getCapsuleSecondEndPoint (indexVector[i].second);
      }
    endPoint1 = endPoint1 / (float) (end - start);
    endPoint2 = endPoint2 / (float) (end - start);
    CkcdPoint center = (endPoint1 + endPoint2) / 2;

    kcdReal tmpRadius;
    radius
      = center.distanceFrom ((polyCapsules_[indexVector[start].first]
			      ->getCapsuleFirstEndPoint (indexVector[start].second)
			      + polyCapsules_[indexVector[start].first]
			      ->getCapsuleSecondEndPoint (indexVector[start].second))
			     / 2)
      + polyCapsules_[indexVector[start].first]
      ->getCapsuleRadius(indexVector[start].second);
    for (unsigned int i = start + 1; i < end; i++)
      {
	tmpRadius
	  = center.distanceFrom ((polyCapsules_[indexVector[i].first]
				  ->getCapsuleFirstEndPoint (indexVector[i].second)
				  + polyCapsules_[indexVector[i].first]
				  ->getCapsuleSecondEndPoint (indexVector[i].second))
				 / 2)
	  + polyCapsules_[indexVector[i].first]
	  ->getCapsuleRadius(indexVector[i].second);
	radius  = std::max (tmpRadius, radius);
      }
  }

  unsigned int TestTreeCapsule::
  sortCapsuleBV (unsigned int start,
		 unsigned int end,
		 std::vector<std::pair<unsigned int, unsigned int> >&
		 indexVector,
		 const CkcdPoint& barycenter)
  {
    // compute main axis (AABB method)
    CkcdPoint center
      = (polyCapsules_[indexVector[start].first]
	 ->getCapsuleFirstEndPoint (indexVector[start].second)
	 + polyCapsules_[indexVector[start].first]
	 ->getCapsuleSecondEndPoint (indexVector[start].second))
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
	  = (polyCapsules_[indexVector[i].first]
	     ->getCapsuleFirstEndPoint (indexVector[i].second)
	     + polyCapsules_[indexVector[i].first]
	     ->getCapsuleSecondEndPoint (indexVector[i].second))
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
	      = (polyCapsules_[indexVector[minIndex].first]
		 ->getCapsuleFirstEndPoint (indexVector[minIndex].second)
		 + polyCapsules_[indexVector[minIndex].first]
		 ->getCapsuleSecondEndPoint (indexVector[minIndex].second))
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
	      = (polyCapsules_[indexVector[minIndex].first]
		 ->getCapsuleFirstEndPoint (indexVector[minIndex].second)
		 + polyCapsules_[indexVector[minIndex].first]
		 ->getCapsuleSecondEndPoint (indexVector[minIndex].second))
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
	      = (polyCapsules_[indexVector[minIndex].first]
		 ->getCapsuleFirstEndPoint (indexVector[minIndex].second)
		 + polyCapsules_[indexVector[minIndex].first]
		 ->getCapsuleSecondEndPoint (indexVector[minIndex].second))
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
    return bb;
  }

} // end of namespace hpp.
