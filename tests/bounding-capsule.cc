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
// along with roboptim-capsule.  If not, see <http://www.gnu.org/licenses/>.

#define BOOST_TEST_MODULE bounding-capsule

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <KineoModel/kppLicense.h>

#include <hpp/geometry/collision/util.hh>

using boost::test_tools::output_test_stream;

BOOST_AUTO_TEST_CASE (bounding_capsule)
{
  using namespace hpp::geometry::collision;

  // Validate Kineo license.
  if (!CkppLicense::initialize ())
    {
      std::cout << "Failed to validate Kineo license." << std::endl;
      return;
    }

  // Build cylinder polyhedron. Main axis is oriented along x.
  CkcdPolyhedronShPtr kcdPolyhedron = CkcdPolyhedron::create ();

  hppReal radius = 0.2f;
  hppReal height = 1.;
  unsigned int nbPhiStep = 100;
  unsigned int nbHeightStep = 100;
  int bitMask = 1;
  CkcdPolyhedronFactory::addCylinderToPolyhedron (radius,
						  height,
						  nbPhiStep,
						  nbHeightStep,
						  bitMask,
						  kcdPolyhedron);

  // Convert KCD polyhedron to polyhedron type.
  polyhedrons_t polyhedrons;
  polyhedrons.push_back (kcdPolyhedron);
  
  // Compute polyhedron bounding capsule
  CkcdPoint endPoint1;
  CkcdPoint endPoint2;
  kcdReal capsuleRadius;
  computeBoundingCapsulePolyhedron (polyhedrons,
				    endPoint1,
				    endPoint2,
				    capsuleRadius);

  std::cout << endPoint1 << std::endl;
  std::cout << endPoint2 << std::endl;
  std::cout << capsuleRadius << std::endl;
}
