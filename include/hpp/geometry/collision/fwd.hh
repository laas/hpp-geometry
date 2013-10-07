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
 * \brief Forward declarations.
 */

#ifndef KCD_FWD_HH
# define KCD_FWD_HH

# include <hpp/util/kitelab.hh>

namespace hpp
{
  namespace geometry
  {
    namespace collision
    {
      HPP_KIT_PREDEF_CLASS (TestTreeCapsule);
      HPP_KIT_PREDEF_CLASS (PolyCapsule);
      HPP_KIT_PREDEF_CLASS (Capsule);
      HPP_KIT_PREDEF_CLASS (DetectorCapsuleCapsule);
      HPP_KIT_PREDEF_CLASS (DetectorCapsuleOBB);
      HPP_KIT_PREDEF_CLASS (DetectorOBBCapsule);
      HPP_KIT_PREDEF_CLASS (DetectorCapsuleTriangle);
      HPP_KIT_PREDEF_CLASS (DetectorTriangleCapsule);
      HPP_KIT_PREDEF_CLASS (TestTreeSegment);
      HPP_KIT_PREDEF_CLASS (PolySegment);
      HPP_KIT_PREDEF_CLASS (Segment);
      HPP_KIT_PREDEF_CLASS (DetectorSegmentSegment);
      HPP_KIT_PREDEF_CLASS (DetectorSegmentOBB);
      HPP_KIT_PREDEF_CLASS (DetectorOBBSegment);
      HPP_KIT_PREDEF_CLASS (DetectorSegmentTriangle);
      HPP_KIT_PREDEF_CLASS (DetectorTriangleSegment);
      HPP_KIT_PREDEF_CLASS (DetectorSegmentCapsule);
      HPP_KIT_PREDEF_CLASS (DetectorCapsuleSegment);
      HPP_KIT_PREDEF_CLASS (DetectorCapsuleBox);
      HPP_KIT_PREDEF_CLASS (DetectorBoxCapsule);
    } // end of namespace collision.
  } // end of namespace geometry.
} // end of namespace hpp.

#endif //! KCD_FWD_HH
