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
 * \brief Declaration of PolyCapsule.
 */

#ifndef KCD_POLY_CAPSULE_HH_
# define KCD_POLY_CAPSULE_HH_

# include <boost/tuple/tuple.hpp>

# include <kcd2/kcdInterface.h>

# include "hpp/geometry/collision/fwd.hh"

namespace hpp
{
  class PolyCapsule : public CkcdGeometry
  {
  public:
    typedef boost::tuple<CkcdPoint, CkcdPoint, kcdReal> capsule_t;
    typedef std::vector<capsule_t> capsuleVector_t;

    // Create new capsule polygon.
    static PolyCapsuleShPtr create ();

    // Destructor.
    virtual ~PolyCapsule ();

    /// @name CkcdObject inherited functions
    ///	the sub elements are the capsules
    ///@{
    virtual unsigned int countSubElements () const;
    //@}

    /// @name additionnal functions
    ///	Used to fill the PolyCapsule and retrieve informations
    void addCapsule (const CkcdPoint& endPoint1,
		     const CkcdPoint& endPoint2,
		     kcdReal radius);

    ktStatus setCapsule (unsigned int index,
			 const CkcdPoint& endPoint1,
			 const CkcdPoint& endPoint2,
			 kcdReal radius);

    ktStatus getCapsule (unsigned int index,
			 CkcdPoint& endPoint1,
			 CkcdPoint& endPoint2,
			 kcdReal& radius) const;

    CkcdPoint getCapsuleFirstEndPoint (unsigned int index) const;

    CkcdPoint getCapsuleSecondEndPoint (unsigned int index) const;

    kcdReal getCapsuleRadius (unsigned int index) const;
    //@}

  protected:
    // Initialization function
    ktStatus init (const PolyCapsuleWkPtr& weakPtr);

    // Constructor
    PolyCapsule ();

  private:
    // weak pointer to ( *this )
    PolyCapsuleWkPtr weakPtr_;

    capsuleVector_t capsuleVector_;

    CkcdMat4 moveMatrix_;

    kcdReal radiusScale_;
  };

} // end of namespace hpp.

#endif //! KCD_POLY_CAPSULE_HH_
