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
 * \brief Declaration of TestTreeCapsule.
 */

#ifndef KCD_TEST_TREE_CAPSULE_HH_
# define KCD_TEST_TREE_CAPSULE_HH_

# include <vector>

# include <kcd2/kcdInterface.h>

# include "hpp/geometry/collision/fwd.hh"
# include "hpp/geometry/collision/capsule.hh"
# include "hpp/geometry/collision/poly-capsule.hh"

namespace kcd
{
  class TestTreeCapsule : public CkcdTestTreeLocked
  {
  public:
    // Create new test tree for capsule.
    static TestTreeCapsuleShPtr create (CkcdObjectShPtr collisionEntity);

    // Destructor.
    virtual ~TestTreeCapsule ();

    /// @name CkcdTestTree inherited functions
    /// See \c CkcdTestTree for details.
    ///
    /// During a collision test, a first CkcdTreeIterator will be
    /// created first by rootIterator(). Then, several will be created
    /// by successive call to getChildIterator(). Each
    /// CkcdTreeIterator should represent an internal bouding volume.
    ///@{
    virtual unsigned int countChildren (const CkcdTreeIterator& it) const;

    virtual CkcdTreeIterator getChildIterator (const CkcdTreeIterator& it,
					       unsigned int rank) const;

    virtual double tolerance (const CkcdTreeIterator& it) const;

    virtual kcdReal heuristicValue (const CkcdTreeIterator& it) const;

    virtual CkcdObjectShPtr sceneAnchor (const CkcdTreeIterator& it) const;

    virtual CkcdGeometrySubElementShPtr
    geometrySubElement (const CkcdTreeIterator& it) const;

    virtual CkcdTreeIterator
    geometrySubElementIterator (const CkcdGeometrySubElementConstShPtr& geo) const;

    virtual unsigned int dispatchID (const CkcdTreeIterator& it) const;

    virtual CkcdTreeIterator rootIterator () const;
    ///@}

    /// @name CkcdTestTreeLocked inherited functions
    ///	See \c CkcdTestTreeLocked for details.
    ///
    ///	First, the tree will be filled by all CkcdGeometry it accepts
    ///	(in this case all CkttPolySphere). Then the collision entity
    ///	will be built. In this tutorials, we do not support deferred
    ///	building so all operations are made in startCollisionEntity.
    ///@{
    virtual bool accepts (CkcdGeometryConstShPtr geometry) const;
    
    virtual ktStatus integrate (CkitProgressDelegateShPtr progress,
				CkcdGeometryConstShPtr geometry);

    virtual ktStatus startCollisionEntity (CkitProgressDelegateShPtr progress,
					   bool& canContinue);

    virtual ktStatus continueCollisionEntity (bool& canContinue);

    virtual ktStatus finishCollisionEntity (ktStatus status);

    virtual bool integratedNothing () const;
    ///@}

    /// getSphere is used by the CkttDetectorCapsuleCapsule to retrieve
    /// information needed to compute collisions.
    ktStatus getCapsule (const CkcdTreeIterator& it,
			 CkcdPoint& endPoint1,
			 CkcdPoint& endPoint2,
			 kcdReal& radius) const;

    /// Retrieves a CkttPolyCapsule integrated in the CkttTestTreeCapsule
    PolyCapsuleConstShPtr getPolyCapsule (unsigned int index) const;

    /// Retrieve the unique external ID that represents element
    /// handled by CkttTestTreeCapsule. Used to dispatch tests
    /// between all CkcdDetector.
    static inline unsigned int capsuleDispatchID ()
    {
      return capsuleDispatchID_;
    };

  protected:
    /// Constructor
    TestTreeCapsule (CkcdObjectShPtr collisionEntity);
    
    /// Initialisation function
    ktStatus init (const TestTreeCapsuleWkPtr& weakPtr);
    
    /// Convert a polyCapsule index and a capsule index in this
    /// polyCapsule into a global capsule index.
    unsigned int getCapsuleIndex (unsigned int polyCapsuleIndex,
				  unsigned int indexInPolyCapsule) const;

    /// Converts a global capsule index into a polyCapsule index and an capsule index in this polyCapsule
    ktStatus getCapsuleIndexes (unsigned int index,
				unsigned int& polyCapsuleIndex,
				unsigned int& capsuleIndex) const;
    
    /// @name Bounding volumes building functions
    ///	Computes capsules that englobe other capsules
    ///@{
    void buildBoundingVolumes (unsigned int index,
			       unsigned int start,
			       unsigned int end,
			       std::vector<std::pair<unsigned int, unsigned int> >&
			       indexVector);
    
    void computeCapsuleBV (unsigned int start,
			   unsigned int end,
			   std::vector<std::pair<unsigned int, unsigned int> >&
			   io_indexVector,
			   CkcdPoint& endPoint1,
			   CkcdPoint& endPoint2,
			   kcdReal& radius);
    
    unsigned int sortCapsuleBV (unsigned int start,
				unsigned int end,
				std::vector<std::pair<unsigned int, unsigned int> >&
				indexVector,
				const CkcdPoint& barycenter);
    
    void switchIndexes (unsigned int index1,
			unsigned int index2,
			std::vector<std::pair<unsigned int, unsigned int> >&
			indexVector);
    //@}
    
    virtual CkcdBoundingBoxShPtr computeBoundingBox() const;

  private:
    // internal IDs
    static const char capsuleBoundingVolumeID_;
    static const char capsuleElementID_;

    // external ID
    static unsigned int capsuleDispatchID_;

    TestTreeCapsuleWkPtr weakPtr_;

    std::vector<PolyCapsuleConstShPtr> polyCapsules_;

    class CapsuleBoudingVolume
    {
    public:
      CkcdPoint endPoint1_;
      CkcdPoint endPoint2_;
      kcdReal radius_;
      bool firstChildIsBoundingVolume_;
      bool secondChildIsBoundingVolume_;
      unsigned int firstChildIndex_;
      unsigned int secondChildIndex_;
    };

    std::vector<CapsuleBoudingVolume> capsuleBoundingVolumes_;
  };
  
} // end of namespace kcd.

#endif //! KCD_TEST_TREE_CAPSULE_HH_
