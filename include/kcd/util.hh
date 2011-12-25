// Copyright (C) 2011 by Antonio El Khoury.
//
// This file is part of the kcd-capsule.
//
// kcd-capsule is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// kcd-capsule is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with kcd-capsule.  If not, see <http://www.gnu.org/licenses/>.


#ifndef KCD_UTIL_HH_
# define KCD_UTIL_HH_

# include <iostream>

# include <geometric-tools/Wm5Vector3.h>

namespace kcd
{
  /// \brief Convert CkcdPoint to Geometric Tools Vector3.
  void convertKcdPointToVector3 (Wm5::Vector3<kcdReal>& dst, const CkcdPoint& src)
  {
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
  }

  /// \brief Convert Geometric Tools Vector3 to CkcdPoint.
  void convertVector3ToKcdPoint (CkcdPoint& dst, const Wm5::Vector3<kcdReal>& src)
  {
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
  }

  std::ostream&
  operator<< (std::ostream& os, const CkcdMat4& kcdMat4)
  {
    for (unsigned int rowId = 0; rowId < 4; ++rowId)
      {
	for (unsigned int colId = 0; colId < 4; ++colId)
	  os << kcdMat4 (rowId, colId) << " ";
	os << std::endl;
      }

    return os;
  }

  std::ostream&
  operator<< (std::ostream& os, const CkcdCollisionReportShPtr& kcdCollision)
  {
    unsigned int nbPairs = kcdCollision->countPairs();
  
    if (nbPairs == 0)
      os << "No Collisions reported" << std::endl;
    else
      os << "Collisions reported: " << std::endl;
    
    for(unsigned int i=0; i < nbPairs; i++)
      {
	os << "  colliding capsule pair :" << std::endl;

	CkcdGeometrySubElementConstShPtr gse1 = kcdCollision->leftSubElement (i);
	CkcdGeometrySubElementConstShPtr gse2 = kcdCollision->rightSubElement (i);

	CapsuleConstShPtr capsule1 = KIT_DYNAMIC_PTR_CAST (const Capsule, gse1);
	CapsuleConstShPtr capsule2 = KIT_DYNAMIC_PTR_CAST (const Capsule, gse2);

	// Retrieve the points (relative to their geometry's position)
	CkcdPoint leftEndPoint1 = capsule1->endPoint1 ();
	CkcdPoint leftEndPoint2 = capsule1->endPoint2 ();
	CkcdPoint rightEndPoint1 = capsule1->endPoint1 ();
	CkcdPoint rightEndPoint2 = capsule1->endPoint2 ();
	kcdReal r1 = capsule1->radius();
	kcdReal r2 = capsule2->radius();

	os << "    left end point 1 = ("
	   << leftEndPoint1[0] << ", "
	   << leftEndPoint1[1] << ", "
	   << leftEndPoint1[2] << "), "
	   << "left end point 2 = ("
	   << leftEndPoint2[0] << ", "
	   << leftEndPoint2[1] << ", "
	   << leftEndPoint2[2] << "), radius = "
	   << r1 << std::endl;

	os << "    right end point 1 = ("
	   << rightEndPoint1[0] << ", "
	   << rightEndPoint1[1] << ", "
	   << rightEndPoint1[2] << "), "
	   << "right end point 2 = ("
	   << rightEndPoint2[0] << ", "
	   << rightEndPoint2[1] << ", "
	   << rightEndPoint2[2] << "), radius = "
	   << r2 << std::endl;
      }

    return os;
  }

} // end of namespace kcd.
#endif //! KCD_UTIL_HH_
