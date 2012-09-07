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
 * \brief Declaration of DetectorBoxCapsule.
 */

#ifndef KCD_DETECTOR_BOX_CAPSULE_HH_
# define KCD_DETECTOR_BOX_CAPSULE_HH_

# include <kcd2/kcdInterface.h>

# include "hpp/geometry/collision/fwd.hh"

namespace hpp
{
  namespace geometry
  {
    namespace collision
    {
      class DetectorBoxCapsule : public CkcdDetector
      {
      public:
	/// \brief Create a new detector.
	static DetectorBoxCapsuleShPtr create ();

	/// \brief Create a copy of the detector.
	static DetectorBoxCapsuleShPtr
	createCopy (const DetectorBoxCapsuleConstShPtr& detector);

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
	/// Should return true only if left ID equals
	///	CkcdTestTreeBox::boundingBoxDispatchID() and right ID equals
	///	TestTreeCapsule::capsuleDispatchID().
	virtual bool canHandle (unsigned int leftID, unsigned int rightID) const;

      protected:
	/// \brief Initialize detector.
	ktStatus init (const DetectorBoxCapsuleWkPtr& weakPtr);

	/// \brief Constructor
	DetectorBoxCapsule ();

	/// \brief Copy constructor
	DetectorBoxCapsule (const DetectorBoxCapsule& detector);

      private:
	DetectorBoxCapsuleWkPtr weakPtr_;
      };

    } // end of namespace collision.
  } // end of namespace geometry.
} // end of namespace hpp.

#endif //! KCD_DETECTOR_BOX_CAPSULE_HH_
