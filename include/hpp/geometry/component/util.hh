// Copyright (C) 2012 by Antonio El Khoury.
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


#ifndef KPP_COMPONENT_UTIL_HH_
# define KPP_COMPONENT_UTIL_HH_

# include <kcd2/kcdInterface.h>

namespace hpp
{
  namespace geometry
  {
    namespace component
    {
      /// \brief Convert capsule parameters to polyhedron data.
      ///
      /// Returned polyhedron data corresponds to a capsule with its axis
      /// center in (0, 0, 0) and the main axis oriented along X.
      ///
      /// \param height capsule height
      /// \param capsule radius
      /// \param baseVertices number of vertices on cylindrical base.
      /// \param parallels number of parallels on sphere used to form caps.
      /// \return dst polyhedron data that contains a list of points and triangles.
      void convertCapsuleToPolyhedronData (CkcdPolyhedronDataShPtr& dst,
					   const double& height,
					   const double& radius,
					   const unsigned baseVertices,
					   const unsigned parallels);

      /// \brief convert capsule axis to a matrix transform.
      ///
      /// \param endPoint1 capsule segment first end point.
      /// \param endPoint2 capsule segment second end point.
      /// \return dst matrix transform of capsule.
      void convertCapsuleAxisToTransform (CkcdMat4& dst,
					  const CkcdPoint& endPoint1,
					  const CkcdPoint& endPoint2);

      /// \brief convert capsule axis to Euler angles as defined for
      /// cylinders in urdf files (urdf cylinders are aligned along
      /// the z axis while Kite cylinders are aligned along the x
      /// axis).
      ///
      /// \param endPoint1 capsule segment first end point.
      /// \param endPoint2 capsule segment second end point.
      /// \return roll roll angle.
      /// \return pitch pitch angle.
      /// \return yaw yaw angle.
      void convertCapsuleAxisToUrdfEuler (double& roll,
					  double& pitch,
					  double& yaw,
					  const CkcdPoint& endPoint1,
					  const CkcdPoint& endPoint2);

    } // end of namespace component.
  } // end of namespace geometry.
} // end of namespace hpp.

#endif //! KPP_COMPONENT_UTIL_HH_
