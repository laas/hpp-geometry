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

#ifndef KPP_COMPONENT_FWD_HH
# define KPP_COMPONENT_FWD_HH

# include "hpp/geometry/collision/util.hh"

namespace hpp
{
  namespace geometry
  {
    namespace component
    {
      typedef collision::hppReal hppReal;
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-pedantic"
      KIT_PREDEF_CLASS (Capsule)
      KIT_PREDEF_CLASS (Segment)
# pragma GCC diagnostic pop
    } // end of namespace component.
  } // end of namespace geometry.
} // end of namespace hpp.
  
#endif //! KPP_COMPONENT_FWD_HH
