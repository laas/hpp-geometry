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
 * \brief Declaration of Segment.
 */

#ifndef KCD_SEGMENT_HH_
# define KCD_SEGMENT_HH_

# include <kcd2/kcdInterface.h>

# include "hpp/geometry/collision/fwd.hh"

namespace hpp
{
  namespace geometry
  {
    namespace collision
    {
      class Segment : public CkcdGeometrySubElement
      {
      public:
	/// Create a new segment.
	///
	/// \param tree Tree to which the element belongs
	///	\param index Sphere index within the tree
	/// \param endPoint1 First end point of the segment axis
	///	\param endPoint2 Second end point of the segment axis
	///
	/// \return New segment
	static SegmentShPtr create (const TestTreeSegmentShPtr testTree,
				    unsigned int index,
				    const CkcdPoint& endPoint1,
				    const CkcdPoint& endPoint2);
    
	/// Destructor.
	virtual ~Segment ();

	/// Get the parent geometry.
	virtual CkcdGeometryConstShPtr geometry () const;

	/// Get index in test tree.
	unsigned int index () const;

	/// Get the coordinates of the segment's axis first end point
	/// relative to the root of the tree.
	///
	/// \return center coordinate
	CkcdPoint endPoint1 () const;

	/// Get the coordinates of the segment's axis second end point
	/// relative to the root of the tree.
	///
	/// \return center coordinate
	CkcdPoint endPoint2 () const;

      protected:
	/// Constructor.
	Segment (TestTreeSegmentShPtr testTree);

	/// Initialize.
	ktStatus init (const SegmentWkPtr& weakPtr,
		       unsigned int index,
		       const CkcdPoint& endPoint1,
		       const CkcdPoint& endPoint2);

	/// Get segment test tree.
	TestTreeSegmentShPtr testTreeSegment () const;

      private:
	SegmentWkPtr weakPtr_;

	unsigned int index_;
	CkcdPoint endPoint1_;
	CkcdPoint endPoint2_;
      };

    } // end of namespace collision.
  } // end of namespace geometry.
} // end of namespace hpp.

#endif //! KCD_SEGMENT_HH_
