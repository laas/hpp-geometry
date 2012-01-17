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
 * \brief Forward declarations.
 */

#ifndef KCD_FWD_HH
# define KCD_FWD_HH

namespace hpp
{
  namespace geometry
  {
    namespace collision
    {
      KIT_PREDEF_CLASS (TestTreeCapsule);
      KIT_PREDEF_CLASS (PolyCapsule);
      KIT_PREDEF_CLASS (Capsule);
      KIT_PREDEF_CLASS (DetectorCapsuleCapsule);
      KIT_PREDEF_CLASS (DetectorCapsuleOBB);
      KIT_PREDEF_CLASS (DetectorOBBCapsule);
      KIT_PREDEF_CLASS (DetectorCapsuleTriangle);
      KIT_PREDEF_CLASS (DetectorTriangleCapsule);
      KIT_PREDEF_CLASS (TestTreeSegment);
      KIT_PREDEF_CLASS (PolySegment);
      KIT_PREDEF_CLASS (Segment);
      KIT_PREDEF_CLASS (DetectorSegmentSegment);
      KIT_PREDEF_CLASS (DetectorSegmentOBB);
      KIT_PREDEF_CLASS (DetectorOBBSegment);
      KIT_PREDEF_CLASS (DetectorSegmentTriangle);
      KIT_PREDEF_CLASS (DetectorTriangleSegment);
    } // end of namespace collision.
  } // end of namespace geometry.
} // end of namespace hpp.

#endif //! KCD_FWD_HH
