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
 * \brief Declaration of PolySegment.
 */

#ifndef KCD_POLY_SEGMENT_HH_
# define KCD_POLY_SEGMENT_HH_

# include <boost/tuple/tuple.hpp>

# include <kcd2/kcdInterface.h>

# include "kcd/fwd.hh"

namespace kcd
{
  class PolySegment : public CkcdGeometry
  {
  public:
    typedef boost::tuple<CkcdPoint, CkcdPoint> segment_t;
    typedef std::vector<segment_t> segmentVector_t;

    // Create new segment polygon.
    static PolySegmentShPtr create ();

    // Destructor.
    virtual ~PolySegment ();

    /// @name CkcdObject inherited functions
    ///	the sub elements are the segments
    ///@{
    virtual unsigned int countSubElements () const;
    //@}

    /// @name additionnal functions
    ///	Used to fill the PolySegment and retrieve informations
    void addSegment (const CkcdPoint& endPoint1,
		     const CkcdPoint& endPoint2);

    ktStatus setSegment (unsigned int index,
			 const CkcdPoint& endPoint1,
			 const CkcdPoint& endPoint2);

    ktStatus getSegment (unsigned int index,
			 CkcdPoint& endPoint1,
			 CkcdPoint& endPoint2) const;

    CkcdPoint getSegmentFirstEndPoint (unsigned int index) const;

    CkcdPoint getSegmentSecondEndPoint (unsigned int index) const;
    //@}

  protected:
    // Initialization function
    ktStatus init (const PolySegmentWkPtr& weakPtr);

    // Constructor
    PolySegment ();

  private:
    // weak pointer to ( *this )
    PolySegmentWkPtr weakPtr_;

    segmentVector_t segmentVector_;

    CkcdMat4 moveMatrix_;
  };

} // end of namespace kcd.

#endif //! KCD_POLY_SEGMENT_HH_
