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
 * \brief Declaration of TestTreeSegment.
 */

#ifndef KCD_TEST_TREE_SEGMENT_HH_
# define KCD_TEST_TREE_SEGMENT_HH_

# include <vector>

# include <kcd2/kcdInterface.h>

# include "hpp/geometry/collision/fwd.hh"
# include "hpp/geometry/collision/segment.hh"
# include "hpp/geometry/collision/poly-segment.hh"

namespace hpp
{
  class TestTreeSegment : public CkcdTestTreeLocked
  {
  public:
    // Create new test tree for segment.
    static TestTreeSegmentShPtr create (CkcdObjectShPtr collisionEntity);

    // Destructor.
    virtual ~TestTreeSegment ();

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

    /// getSphere is used by the CkttDetectorSegmentSegment to retrieve
    /// information needed to compute collisions.
    ktStatus getSegment (const CkcdTreeIterator& it,
			 CkcdPoint& endPoint1,
			 CkcdPoint& endPoint2) const;

    /// Retrieves a CkttPolySegment integrated in the CkttTestTreeSegment
    PolySegmentConstShPtr getPolySegment (unsigned int index) const;

    /// Retrieve the unique external ID that represents element
    /// handled by CkttTestTreeSegment. Used to dispatch tests
    /// between all CkcdDetector.
    static inline unsigned int segmentDispatchID ()
    {
      return segmentDispatchID_;
    };

  protected:
    /// Constructor
    TestTreeSegment (CkcdObjectShPtr collisionEntity);
    
    /// Initialisation function
    ktStatus init (const TestTreeSegmentWkPtr& weakPtr);
    
    /// Convert a polySegment index and a segment index in this
    /// polySegment into a global segment index.
    unsigned int getSegmentIndex (unsigned int polySegmentIndex,
				  unsigned int indexInPolySegment) const;

    /// Converts a global segment index into a polySegment index and an segment index in this polySegment
    ktStatus getSegmentIndexes (unsigned int index,
				unsigned int& polySegmentIndex,
				unsigned int& segmentIndex) const;
    
    /// @name Bounding volumes building functions
    ///	Computes segments that englobe other segments
    ///@{
    void buildBoundingVolumes (unsigned int index,
			       unsigned int start,
			       unsigned int end,
			       std::vector<std::pair<unsigned int, unsigned int> >&
			       indexVector);
    
    void computeSegmentBV (unsigned int start,
			   unsigned int end,
			   std::vector<std::pair<unsigned int, unsigned int> >&
			   io_indexVector,
			   CkcdPoint& endPoint1,
			   CkcdPoint& endPoint2);
    
    unsigned int sortSegmentBV (unsigned int start,
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
    static const char segmentBoundingVolumeID_;
    static const char segmentElementID_;

    // external ID
    static unsigned int segmentDispatchID_;

    TestTreeSegmentWkPtr weakPtr_;

    std::vector<PolySegmentConstShPtr> polySegments_;

    class SegmentBoudingVolume
    {
    public:
      CkcdPoint endPoint1_;
      CkcdPoint endPoint2_;
      bool firstChildIsBoundingVolume_;
      bool secondChildIsBoundingVolume_;
      unsigned int firstChildIndex_;
      unsigned int secondChildIndex_;
    };

    std::vector<SegmentBoudingVolume> segmentBoundingVolumes_;
  };
  
} // end of namespace hpp.

#endif //! KCD_TEST_TREE_SEGMENT_HH_
