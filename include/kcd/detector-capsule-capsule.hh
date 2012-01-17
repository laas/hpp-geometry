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
 * \brief Declaration of DetectorCapsuleCapsule.
 */

#ifndef KCD_DETECTOR_CAPSULE_CAPSULE_HH_
# define KCD_DETECTOR_CAPSULE_CAPSULE_HH_

# include <kcd2/kcdInterface.h>

# include "kcd/fwd.hh"

namespace kcd
{
  class DetectorCapsuleCapsule : public CkcdDetector
  {
  public:
    /// \brief Create a new detector.
    static DetectorCapsuleCapsuleShPtr create ();

    /// \brief Create a copy of the detector.
    static DetectorCapsuleCapsuleShPtr
    createCopy (const DetectorCapsuleCapsuleConstShPtr& detector);

    /// \brief Clones the detector.
    ///	All detectors must be clonable
    virtual CkcdDetectorShPtr clone () const;

    /// \brief Analyse collision between the bounding volume / geometry elements.
    ///	See CkcdDetector::analyze() for details
    virtual CkcdDetectorTestAnswer
    analyze (const CkcdTreeIterator& left, 
	     const CkcdTreeIterator& right,
	     const CkcdDetectorElementaryTestDataShPtr& testData,
	     CkcdProximityQuery& query) const;

    /// Should return true if the detector knows how to test
    ///	collisions between dispatchID.
    ///
    /// Should return true only if both equal
    ///	TestTreeCapsule::capsuleDispatchID()
    virtual bool canHandle (unsigned int leftID, unsigned int rightID) const;

  protected:
    /// \brief Initialize detector.
    ktStatus init (const DetectorCapsuleCapsuleWkPtr& weakPtr);

    /// \brief Constructor
    DetectorCapsuleCapsule ();

    /// \brief Copy constructor
    DetectorCapsuleCapsule (const DetectorCapsuleCapsule& detector);

  private:
    DetectorCapsuleCapsuleWkPtr weakPtr_;
  };

} // end of namespace kcd.

#endif //! KCD_DETECTOR_CAPSULE_CAPSULE_HH_
