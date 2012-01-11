// Copyright (C) 2012 by Antonio El Khoury.
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


#ifndef KPP_UTIL_HH_
# define KPP_UTIL_HH_

# include <kcd2/kcdInterface.h>

namespace kpp
{
  void convertCapsuleToPolyhedronData (const double& height,
				       const double& radius,
				       const unsigned baseVertices,
				       const unsigned parallels,
				       CkcdPolyhedronDataShPtr& polyData);
} // end of namespace kpp.

#endif //! KPP_UTIL_HH_
